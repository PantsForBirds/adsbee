#pragma once

#include <cstdint>

/**
 * Puts a local free-running 32-bit clock on the 48 MHz MLAT time base that Mode S packets use. Local clocks: the CC13xx
 * radio timer (RAT, 4 MHz, the value the RF core appends at sync word detection) and the ESP32-S3 system timer
 * (16 MHz).
 *
 * A sync point is one instant read on both clocks. A timestamp is the last point's MLAT count plus the local time since
 * it, at the measured rate. With one point the clocks are assumed to run at the nominal ratio (true on the m1421, where
 * the RAT and the MLAT timer share the 48 MHz crystal). On the ADSBee 1090 the CC1312 and ESP32 crystals are ~50 ppm
 * off the RP2040's, ~50 us a second, so they wait for a second point (min_sync_points 2). The rate is measured between
 * the last two points; once synced the estimate moves a quarter of the way to each new measurement. Taken alone, the
 * capture noise of both points (CC1312: ~0.11 us rms) would add up to about twice that by the end of an interval.
 *
 * A point that doesn't agree with the last one starts the fit again from it: one implying a rate beyond
 * kMaxRateErrorPPB, or, once synced, one further than kMaxPredictionErrorUs from the prediction.
 */
class MLATTimeSync {
   public:
    static constexpr uint32_t kMLATTicksPerUs = 48;
    static constexpr uint32_t kMLATTicksPerRATTick = kMLATTicksPerUs / 4;           // CC13xx radio timer: 4 MHz.
    static constexpr uint32_t kMLATTicksPerESP32SystimerTick = kMLATTicksPerUs / 16;  // ESP32-S3 system timer: 16 MHz.
    static constexpr int64_t kMaxRateErrorPPB = 200'000;  // 200 ppm.
    // Capture noise is ~0.1 us a point; sync pulses are a second apart.
    static constexpr uint32_t kMaxPredictionErrorUs = 100;
    // Timestamps further than this from the last sync point get none (0). Must stay below half the local clock period.
    static constexpr uint32_t kMaxExtrapolationUs = 60'000'000;

    /**
     * @param[in] mlat_ticks_per_local_tick Nominal ratio of the two clocks, a divisor of 48 (kMLATTicksPerRATTick,
     * kMLATTicksPerESP32SystimerTick).
     * @param[in] min_sync_points Points needed before timestamps are given: 1 when both clocks share a crystal, 2 when
     * the rate between them must be measured first.
     */
    explicit MLATTimeSync(uint32_t mlat_ticks_per_local_tick, uint16_t min_sync_points = 1)
        : mlat_ticks_per_local_tick_(mlat_ticks_per_local_tick),
          max_extrapolation_local_ticks_(kMaxExtrapolationUs * (kMLATTicksPerUs / mlat_ticks_per_local_tick)),
          min_sync_points_(min_sync_points) {}

    // Forgets the points (the local clock restarted or stepped, or the pulses stopped).
    void Restart() {
        num_points_ = 0;
        rate_error_ppb_ = 0;
    }

    /**
     * Adds the local and MLAT counts of the same instant.
     * @retval False if it didn't agree with the last point: the fit starts again from it.
     */
    bool AddSyncPoint(uint32_t local, uint64_t mlat_48mhz_counts) {
        bool agrees = true;
        if (num_points_ > 0) {
            uint32_t local_ticks = local - local_;
            int64_t mlat_ticks = static_cast<int64_t>(mlat_48mhz_counts - mlat_48mhz_counts_);
            int64_t expected_mlat_ticks = static_cast<int64_t>(local_ticks) * mlat_ticks_per_local_tick_;
            int64_t rate_error_ppb =
                expected_mlat_ticks > 0 ? (mlat_ticks - expected_mlat_ticks) * 1'000'000'000 / expected_mlat_ticks : 0;
            agrees = local_ticks > 0 && mlat_ticks > 0 && rate_error_ppb <= kMaxRateErrorPPB &&
                     rate_error_ppb >= -kMaxRateErrorPPB;
            if (agrees && IsSynced()) {
                int64_t error =
                    mlat_ticks - expected_mlat_ticks - expected_mlat_ticks * rate_error_ppb_ / 1'000'000'000;
                agrees = error <= kMaxPredictionErrorTicks && error >= -kMaxPredictionErrorTicks;
            }
            if (agrees) {
                rate_error_ppb_ =
                    IsSynced() ? rate_error_ppb_ + (rate_error_ppb - rate_error_ppb_) / 4 : rate_error_ppb;
            } else {
                Restart();
            }
        }
        local_ = local;
        mlat_48mhz_counts_ = mlat_48mhz_counts;
        if (num_points_ < UINT16_MAX) num_points_++;
        return agrees;
    }

    bool IsSynced() const { return num_points_ > 0 && num_points_ >= min_sync_points_; }
    uint16_t GetNumSyncPoints() const { return num_points_; }
    int64_t GetRateErrorPPB() const { return rate_error_ppb_; }
    uint32_t GetLocalTicksPerUs() const { return kMLATTicksPerUs / mlat_ticks_per_local_tick_; }

    /**
     * MLAT counts the fit predicts for a local clock value, also before timestamps are given (the nominal ratio with
     * one point). For pairing sync pulse edges with their counts.
     * @retval False if there is no point.
     */
    bool Predict(uint32_t local, uint64_t& mlat_48mhz_counts) const {
        if (num_points_ == 0) return false;
        int64_t mlat_ticks = static_cast<int64_t>(static_cast<int32_t>(local - local_)) * mlat_ticks_per_local_tick_;
        mlat_48mhz_counts = mlat_48mhz_counts_ + mlat_ticks + mlat_ticks * rate_error_ppb_ / 1'000'000'000;
        return true;
    }

    /**
     * @param[in] local Local clock value within kMaxExtrapolationUs of the last sync point.
     * @retval 48 MHz MLAT counts, or 0 (no timestamp) when not synced or too far from the last sync point.
     */
    uint64_t ToMLAT48MHzCounts(uint32_t local) const {
        if (!IsSynced()) return 0;
        int32_t local_ticks = static_cast<int32_t>(local - local_);
        if (local_ticks > static_cast<int32_t>(max_extrapolation_local_ticks_) ||
            local_ticks < -static_cast<int32_t>(max_extrapolation_local_ticks_)) {
            return 0;
        }
        int64_t mlat_ticks = static_cast<int64_t>(local_ticks) * mlat_ticks_per_local_tick_;
        return mlat_48mhz_counts_ + mlat_ticks + mlat_ticks * rate_error_ppb_ / 1'000'000'000;
    }

   private:
    static constexpr int64_t kMaxPredictionErrorTicks = static_cast<int64_t>(kMaxPredictionErrorUs) * kMLATTicksPerUs;

    uint32_t mlat_ticks_per_local_tick_;
    uint32_t max_extrapolation_local_ticks_;
    uint16_t min_sync_points_;
    uint16_t num_points_ = 0;  // Since the fit started.
    uint32_t local_ = 0;
    uint64_t mlat_48mhz_counts_ = 0;
    int64_t rate_error_ppb_ = 0;  // MLAT clock rate relative to the nominal ratio, in parts per billion.
};

/**
 * Pairs the MLAT count of a sync pulse (ObjectDictionary::SyncPulse) with the edge a coprocessor captured for it.
 *
 * A count normally arrives well within a second of its edge. Under output overload (RP2040 main loop passes over a
 * second) it can arrive just after the next pulse's edge, so the coprocessor keeps the last two edges. Once synced, the
 * count pairs with the one the fit predicts within MLATTimeSync::kMaxPredictionErrorUs (they are a second apart).
 * Otherwise (no fit yet, or neither edge predicts) it pairs with the latest edge if that is younger than
 * kMaxFirstEdgeAgeUs: if the count's own edge was missed, the latest is a pulse older. The fit then starts again from
 * that point unless it agrees. A count already handled is ignored (the ESP32 gets each one ~10 times).
 *
 * Not thread safe: callers hold the lock that guards the edges.
 */
class SyncPulsePairing {
   public:
    static constexpr uint32_t kMaxFirstEdgeAgeUs = 750'000;  // Below the 1 s between pulses.

    // Records a captured edge (local clock value).
    void AddEdge(uint32_t local) {
        previous_edge_ = latest_edge_;
        latest_edge_ = local;
        if (num_edges_ < 2) num_edges_++;
    }

    // Forgets the edges (the local clock restarted).
    void Clear() { num_edges_ = 0; }

    /**
     * @param[in] sync The coprocessor's fit, for the prediction.
     * @param[in] mlat_48mhz_counts The pulse's count. 0 (no pulse) and a count already handled are ignored.
     * @param[in] local_now Local clock now, for the age of the latest edge.
     * @param[out] edge_local Local clock value of the pulse's edge.
     * @retval True if an edge pairs: add (edge_local, mlat_48mhz_counts) to the fit.
     */
    bool Pair(const MLATTimeSync& sync, uint64_t mlat_48mhz_counts, uint32_t local_now, uint32_t& edge_local) {
        if (mlat_48mhz_counts == 0 || mlat_48mhz_counts == last_mlat_48mhz_counts_ || num_edges_ == 0) return false;
        last_mlat_48mhz_counts_ = mlat_48mhz_counts;
        if (sync.IsSynced()) {
            uint32_t edges[2] = {latest_edge_, previous_edge_};
            for (uint16_t i = 0; i < num_edges_; i++) {
                uint64_t predicted = 0;
                sync.Predict(edges[i], predicted);
                int64_t error = static_cast<int64_t>(predicted - mlat_48mhz_counts);
                if (error <= kMaxPredictionErrorTicks && error >= -kMaxPredictionErrorTicks) {
                    edge_local = edges[i];
                    return true;
                }
            }
        }
        if (local_now - latest_edge_ > kMaxFirstEdgeAgeUs * sync.GetLocalTicksPerUs()) return false;
        edge_local = latest_edge_;
        return true;
    }

   private:
    static constexpr int64_t kMaxPredictionErrorTicks =
        static_cast<int64_t>(MLATTimeSync::kMaxPredictionErrorUs) * MLATTimeSync::kMLATTicksPerUs;

    uint32_t latest_edge_ = 0;
    uint32_t previous_edge_ = 0;
    uint16_t num_edges_ = 0;
    uint64_t last_mlat_48mhz_counts_ = 0;
};
