#pragma once

#include <cstdint>

/**
 * Puts a local free-running 32-bit clock on the 48 MHz MLAT time base that Mode S packets use. Local clocks: the CC13xx
 * radio timer (RAT, 4 MHz, the value the RF core appends at sync word detection) and the ESP32-S3 system timer
 * (16 MHz).
 *
 * A sync point is one instant read on both clocks. With one point the clocks are assumed to run at the nominal ratio
 * (true on the m1421, where the RAT and the MLAT timer share the 48 MHz crystal). Each further point measures the rate
 * between the two clocks (ADSBee 1090: the CC1312 or ESP32 crystal against the RP2040's) over the interval since the
 * previous point. The estimate is the rate over about the last kRateMemoryUs (or the last interval, if longer): each
 * interval weighs by its length, so a longer interval (a late or missed pulse) counts for more. The rate is a property
 * of the crystals, so it survives Restart() (local clock restarted or stepped).
 *
 * Once synced, a point further from the prediction than kMaxResidualUs (plus kMaxResidualDriftPPB of the interval) is
 * skipped and the fit kept; a second one in a row restarts the fit from it.
 */
class MLATTimeSync {
   public:
    static constexpr uint32_t kMLATTicksPerUs = 48;
    static constexpr uint32_t kMLATTicksPerRATTick = kMLATTicksPerUs / 4;           // CC13xx radio timer: 4 MHz.
    static constexpr uint32_t kMLATTicksPerESP32SystimerTick = kMLATTicksPerUs / 16;  // ESP32-S3 system timer: 16 MHz.
    // A point further from the previous one than this, or implying a larger rate error, restarts the fit.
    static constexpr uint32_t kMaxSyncIntervalUs = 10'000'000;
    static constexpr int64_t kMaxRateErrorPPB = 200'000;  // 200 ppm.
    // Timestamps further than this from the last sync point get none (0). Must stay below half the local clock period.
    static constexpr uint32_t kMaxExtrapolationUs = 60'000'000;
    // The rate estimate spans about this much of the recent sync intervals.
    static constexpr uint32_t kRateMemoryUs = 8'000'000;
    // Outlier limit for a point once synced: a fixed part plus a rate change over the interval.
    static constexpr uint32_t kMaxResidualUs = 20;
    static constexpr uint32_t kMaxResidualDriftPPB = 2'000;

    enum Result : uint8_t {
        kAdded = 0,  // The point is in the fit.
        kSkipped,    // Synced, and the point is far from the prediction: skipped, the fit kept.
        kRestarted,  // The point didn't fit the previous ones: the fit restarts from it and waits for one more point.
    };

    /**
     * @param[in] mlat_ticks_per_local_tick Nominal ratio of the two clocks, a divisor of 48 (kMLATTicksPerRATTick,
     * kMLATTicksPerESP32SystimerTick).
     * @param[in] min_sync_points Points needed before timestamps are given: 1 when both clocks share a crystal, 2 when
     * the rate between them must be measured first.
     */
    explicit MLATTimeSync(uint32_t mlat_ticks_per_local_tick, uint16_t min_sync_points = 1)
        : mlat_ticks_per_local_tick_(mlat_ticks_per_local_tick),
          max_interval_local_ticks_(kMaxSyncIntervalUs * (kMLATTicksPerUs / mlat_ticks_per_local_tick)),
          max_extrapolation_local_ticks_(kMaxExtrapolationUs * (kMLATTicksPerUs / mlat_ticks_per_local_tick)),
          min_sync_points_(min_sync_points),
          points_needed_(min_sync_points) {}

    // Forgets everything, including the rate.
    void Reset() {
        num_points_ = 0;
        rate_error_ppb_ = 0;
        rate_valid_ = false;
        rate_span_us_ = 0;
        skipped_point_ = false;
        points_needed_ = min_sync_points_;
    }

    // The local clock restarted or stepped: drops the offset and keeps the rate. Timestamps resume once min_sync_points
    // agree again (with two, a wrongly paired first point never gives timestamps).
    void Restart() {
        num_points_ = 0;
        skipped_point_ = false;
        points_needed_ = min_sync_points_;
    }

    // Adds the local and MLAT counts of the same instant.
    Result AddSyncPoint(uint32_t local, uint64_t mlat_48mhz_counts) {
        Result result = kAdded;
        if (num_points_ > 0) {
            uint32_t local_ticks = local - local_;
            int64_t mlat_ticks = static_cast<int64_t>(mlat_48mhz_counts - mlat_48mhz_counts_);
            int64_t expected_mlat_ticks = static_cast<int64_t>(local_ticks) * mlat_ticks_per_local_tick_;
            int64_t rate_error_ppb =
                expected_mlat_ticks > 0 ? (mlat_ticks - expected_mlat_ticks) * 1'000'000'000 / expected_mlat_ticks : 0;
            bool fits = local_ticks > 0 && local_ticks <= max_interval_local_ticks_ && mlat_ticks > 0 &&
                        rate_error_ppb <= kMaxRateErrorPPB && rate_error_ppb >= -kMaxRateErrorPPB;
            uint32_t interval_us = local_ticks / (kMLATTicksPerUs / mlat_ticks_per_local_tick_);
            if (fits && IsSynced() && rate_valid_) {
                int64_t predicted_mlat_ticks =
                    expected_mlat_ticks + expected_mlat_ticks * rate_error_ppb_ / 1'000'000'000;
                int64_t residual_ticks = mlat_ticks - predicted_mlat_ticks;
                int64_t max_residual_ticks =
                    static_cast<int64_t>(kMaxResidualUs) * kMLATTicksPerUs +
                    static_cast<int64_t>(interval_us) * kMLATTicksPerUs * kMaxResidualDriftPPB / 1'000'000'000;
                fits = residual_ticks <= max_residual_ticks && residual_ticks >= -max_residual_ticks;
            }
            if (!fits && IsSynced() && !skipped_point_) {
                // One bad point (e.g. a mispaired edge) doesn't cost the fit: wait for the next one.
                skipped_point_ = true;
                return kSkipped;
            }
            if (!fits) {
                // Either point may be the bad one (a missed or extra edge): wait for a second point that agrees.
                result = kRestarted;
                num_points_ = 0;
                points_needed_ = min_sync_points_ > 1 || rate_valid_ ? 2 : 1;
            } else if (!rate_valid_) {
                rate_error_ppb_ = rate_error_ppb;
                rate_valid_ = true;
                rate_span_us_ = interval_us;
            } else {
                // The rate over the old span and this interval, each weighed by its length.
                uint64_t span_us = static_cast<uint64_t>(rate_span_us_) + interval_us;
                rate_error_ppb_ += (rate_error_ppb - rate_error_ppb_) * static_cast<int64_t>(interval_us) /
                                   static_cast<int64_t>(span_us > 0 ? span_us : 1);
                rate_span_us_ = span_us < kRateMemoryUs ? static_cast<uint32_t>(span_us) : kRateMemoryUs;
            }
        }
        skipped_point_ = false;
        local_ = local;
        mlat_48mhz_counts_ = mlat_48mhz_counts;
        if (num_points_ < UINT16_MAX) num_points_++;
        return result;
    }

    bool IsSynced() const { return num_points_ > 0 && num_points_ >= points_needed_; }
    uint16_t GetNumSyncPoints() const { return num_points_; }
    bool HasRate() const { return rate_valid_; }
    int64_t GetRateErrorPPB() const { return rate_error_ppb_; }
    // Local clock value of the last sync point.
    uint32_t GetLastSyncPointLocal() const { return local_; }
    uint64_t GetLastSyncPointMLAT48MHzCounts() const { return mlat_48mhz_counts_; }
    uint32_t GetLocalTicksPerUs() const { return kMLATTicksPerUs / mlat_ticks_per_local_tick_; }

    /**
     * MLAT counts the fit predicts for a local clock value, also before timestamps are given: from the last point at
     * the rate so far (the nominal ratio if none). For pairing sync pulse edges with their counts.
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
        uint64_t mlat_48mhz_counts = 0;
        Predict(local, mlat_48mhz_counts);
        return mlat_48mhz_counts;
    }

   private:
    uint32_t mlat_ticks_per_local_tick_;
    uint32_t max_interval_local_ticks_;
    uint32_t max_extrapolation_local_ticks_;
    uint16_t min_sync_points_;
    uint16_t points_needed_;
    uint16_t num_points_ = 0;
    bool rate_valid_ = false;
    uint32_t local_ = 0;
    uint64_t mlat_48mhz_counts_ = 0;
    int64_t rate_error_ppb_ = 0;  // MLAT clock rate relative to the nominal ratio, in parts per billion.
    uint32_t rate_span_us_ = 0;   // Length of the sync intervals the rate estimate stands for, up to kRateMemoryUs.
    bool skipped_point_ = false;
};

/**
 * Pairs the MLAT count of a sync pulse (ObjectDictionary::SyncPulse) with the edge a coprocessor captured for it.
 *
 * The coprocessor keeps its last kNumEdges captured edges. A count pairs with the edge whose local time the fit
 * predicts closest to it, if within kToleranceUs (plus kUnknownRatePPM of the time since the last point while the rate
 * isn't measured yet). Edges are a second apart, far more than that, so the order and lateness of counts don't matter:
 * a count that arrives after the next edge, or never, or twice, or an edge with no count, pairs right or not at all.
 * With no point there is nothing to predict with: the count pairs with the latest edge if it is younger than
 * kMaxFirstEdgeAgeUs (shorter than the time between pulses), else waits for the next count. That edge is the next
 * pulse's if the count arrived just after it; the fit's second point then doesn't agree (MLATTimeSync needs two before
 * timestamps). So while the fit isn't synced, a count that fits no edge is tried the same way, as a new first point.
 *
 * Not thread safe: callers hold the lock that guards the edges.
 */
class SyncPulsePairing {
   public:
    static constexpr uint16_t kNumEdges = 4;
    static constexpr uint32_t kToleranceUs = 1000;
    static constexpr uint32_t kUnknownRatePPM = 200;  // Crystal tolerance: MLATTimeSync::kMaxRateErrorPPB.
    static constexpr uint32_t kMaxFirstEdgeAgeUs = 250'000;

    // Records a captured edge (local clock value).
    void AddEdge(uint32_t local) {
        edges_[next_edge_] = local;
        next_edge_ = (next_edge_ + 1) % kNumEdges;
        if (num_edges_ < kNumEdges) num_edges_++;
    }

    // Forgets the edges (the local clock restarted).
    void Clear() {
        num_edges_ = 0;
        next_edge_ = 0;  // Edges fill from slot 0 until the ring is full.
    }

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
        if (sync.GetNumSyncPoints() > 0) {
            int64_t tolerance = static_cast<int64_t>(kToleranceUs) * MLATTimeSync::kMLATTicksPerUs;
            if (!sync.HasRate()) {
                int64_t interval = static_cast<int64_t>(mlat_48mhz_counts - sync.GetLastSyncPointMLAT48MHzCounts());
                tolerance += (interval < 0 ? -interval : interval) / 1'000'000 * kUnknownRatePPM;
            }
            bool paired = false;
            for (uint16_t i = 0; i < num_edges_; i++) {
                uint64_t predicted = 0;
                sync.Predict(edges_[i], predicted);
                int64_t error = static_cast<int64_t>(predicted - mlat_48mhz_counts);
                if (error < 0) error = -error;
                if (error <= tolerance) {
                    tolerance = error;  // Closest so far.
                    edge_local = edges_[i];
                    paired = true;
                }
            }
            if (paired || sync.IsSynced()) return paired;
        }
        uint32_t latest = edges_[(next_edge_ + kNumEdges - 1) % kNumEdges];
        if (local_now - latest > kMaxFirstEdgeAgeUs * sync.GetLocalTicksPerUs()) return false;
        edge_local = latest;
        return true;
    }

   private:
    uint32_t edges_[kNumEdges] = {};
    uint16_t next_edge_ = 0;
    uint16_t num_edges_ = 0;
    uint64_t last_mlat_48mhz_counts_ = 0;
};
