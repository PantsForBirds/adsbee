#pragma once

#include <cstdint>

/**
 * Puts a local free-running 32-bit clock on the 48 MHz MLAT time base that Mode S packets use. Local clocks: the CC13xx
 * radio timer (RAT, 4 MHz, the value the RF core appends at sync word detection) and the ESP32-S3 system timer
 * (16 MHz).
 *
 * A sync point is one instant read on both clocks. With one point the clocks are assumed to run at the nominal ratio
 * (true on the m1421, where the RAT and the MLAT timer share the 48 MHz crystal). Each further point measures the rate
 * between the two clocks (ADSBee 1090: the CC1312 or ESP32 crystal against the RP2040's), smoothed over a few points.
 * The rate is a property of the crystals, so it survives Restart() (local clock restarted or stepped): one new point
 * then re-anchors the offset.
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
    // Each new rate measurement moves the estimate by 1 / 2^kRateFilterShift of the difference.
    static constexpr uint16_t kRateFilterShift = 3;

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
        points_needed_ = min_sync_points_;
    }

    // The local clock restarted or stepped: drops the offset and keeps the rate, so the next point syncs again.
    void Restart() {
        num_points_ = 0;
        points_needed_ = rate_valid_ ? 1 : min_sync_points_;
    }

    /**
     * Adds the local and MLAT counts of the same instant.
     * @retval False if the point didn't fit the previous ones. The fit restarts from it and waits for one more point.
     */
    bool AddSyncPoint(uint32_t local, uint64_t mlat_48mhz_counts) {
        bool consistent = true;
        if (num_points_ > 0) {
            uint32_t local_ticks = local - local_;
            int64_t mlat_ticks = static_cast<int64_t>(mlat_48mhz_counts - mlat_48mhz_counts_);
            int64_t expected_mlat_ticks = static_cast<int64_t>(local_ticks) * mlat_ticks_per_local_tick_;
            int64_t rate_error_ppb =
                expected_mlat_ticks > 0 ? (mlat_ticks - expected_mlat_ticks) * 1'000'000'000 / expected_mlat_ticks : 0;
            if (local_ticks == 0 || local_ticks > max_interval_local_ticks_ || mlat_ticks <= 0 ||
                rate_error_ppb > kMaxRateErrorPPB || rate_error_ppb < -kMaxRateErrorPPB) {
                // Either point may be the bad one (a missed or extra edge): wait for a second point that agrees.
                consistent = false;
                num_points_ = 0;
                points_needed_ = min_sync_points_ > 1 || rate_valid_ ? 2 : 1;
            } else if (!rate_valid_) {
                rate_error_ppb_ = rate_error_ppb;
                rate_valid_ = true;
            } else {
                rate_error_ppb_ += (rate_error_ppb - rate_error_ppb_) / (1 << kRateFilterShift);
            }
        }
        local_ = local;
        mlat_48mhz_counts_ = mlat_48mhz_counts;
        if (num_points_ < UINT16_MAX) num_points_++;
        return consistent;
    }

    bool IsSynced() const { return num_points_ > 0 && num_points_ >= points_needed_; }
    uint16_t GetNumSyncPoints() const { return num_points_; }
    bool HasRate() const { return rate_valid_; }
    int64_t GetRateErrorPPB() const { return rate_error_ppb_; }
    // Local clock value of the last sync point.
    uint32_t GetLastSyncPointLocal() const { return local_; }

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
};

/**
 * Decides whether the last edge a coprocessor captured on the SYNC line belongs to the SyncPulse message the RP2040
 * sent after it (ObjectDictionary::SyncPulse): one edge per pulse since the previous message, and the last one recent.
 * Pulses are further apart than the age limit, so an older edge or a missed or extra one never pairs.
 */
class SyncPulsePairing {
   public:
    /**
     * @param[in] edge_count Edges captured so far (wraps).
     * @param[in] pulse_num The message's pulse number (counts pulses sent).
     * @param[in] edge_age_us Age of the last edge when the message arrived.
     * @param[in] max_edge_age_us Age limit.
     * @retval True if the last edge is this pulse's.
     */
    bool Pair(uint16_t edge_count, uint32_t pulse_num, uint32_t edge_age_us, uint32_t max_edge_age_us) {
        uint16_t num_edges = edge_count - edge_count_at_last_message_;
        uint32_t num_pulses = pulse_num - last_pulse_num_;
        edge_count_at_last_message_ = edge_count;
        last_pulse_num_ = pulse_num;
        return num_edges > 0 && (num_edges == 1 || num_edges == num_pulses) && edge_age_us <= max_edge_age_us;
    }

    // Edges up to edge_count no longer count (the local clock restarted).
    void SkipEdges(uint16_t edge_count) { edge_count_at_last_message_ = edge_count; }

   private:
    uint16_t edge_count_at_last_message_ = 0;
    uint32_t last_pulse_num_ = 0;
};
