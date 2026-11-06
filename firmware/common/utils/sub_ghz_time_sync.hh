#pragma once

#include <cstdint>

/**
 * Converts sub-GHz packet timestamps from the CC13xx radio timer (RAT: 32 bits, 4 MHz, the value the RF core appends at
 * sync word detection) to the 48 MHz MLAT time base Mode S packets use.
 *
 * A sync point is one instant read on both clocks. With one point the clocks are assumed to run at the same rate (true
 * on the m1421, where the RAT and the MLAT timer share the 48 MHz crystal). Each further point measures the rate
 * between the two clocks (ADSBee 1090: CC1312 and RP2040 crystals), smoothed over a few points.
 */
class SubGHzTimeSync {
   public:
    static constexpr uint32_t kRATTicksPerUs = 4;
    static constexpr uint32_t kMLATTicksPerUs = 48;
    static constexpr uint32_t kMLATTicksPerRATTick = kMLATTicksPerUs / kRATTicksPerUs;
    // A point further from the previous one than this, or implying a larger rate error, restarts the fit.
    static constexpr uint32_t kMaxSyncIntervalRATTicks = 10 * 1'000'000 * kRATTicksPerUs;  // 10 s.
    static constexpr int64_t kMaxRateErrorPPB = 200'000;                                   // 200 ppm.
    // Each new rate measurement moves the estimate by 1 / 2^kRateFilterShift of the difference.
    static constexpr uint16_t kRateFilterShift = 3;

    void Reset() {
        num_points_ = 0;
        rate_error_ppb_ = 0;
    }

    /**
     * Adds the RAT and MLAT counts of the same instant.
     */
    void AddSyncPoint(uint32_t rat, uint64_t mlat_48mhz_counts) {
        if (num_points_ > 0) {
            uint32_t rat_ticks = rat - rat_;
            int64_t mlat_ticks = static_cast<int64_t>(mlat_48mhz_counts - mlat_48mhz_counts_);
            int64_t expected_mlat_ticks = static_cast<int64_t>(rat_ticks) * kMLATTicksPerRATTick;
            int64_t rate_error_ppb =
                expected_mlat_ticks > 0 ? (mlat_ticks - expected_mlat_ticks) * 1'000'000'000 / expected_mlat_ticks : 0;
            if (rat_ticks == 0 || rat_ticks > kMaxSyncIntervalRATTicks || mlat_ticks <= 0 ||
                rate_error_ppb > kMaxRateErrorPPB || rate_error_ppb < -kMaxRateErrorPPB) {
                Reset();
            } else if (num_points_ == 1) {
                rate_error_ppb_ = rate_error_ppb;
            } else {
                rate_error_ppb_ += (rate_error_ppb - rate_error_ppb_) / (1 << kRateFilterShift);
            }
        }
        rat_ = rat;
        mlat_48mhz_counts_ = mlat_48mhz_counts;
        if (num_points_ < UINT16_MAX) num_points_++;
    }

    bool IsSynced() const { return num_points_ > 0; }
    uint16_t GetNumSyncPoints() const { return num_points_; }
    int64_t GetRateErrorPPB() const { return rate_error_ppb_; }

    /**
     * @param[in] rat Radio timer value within +/-536 s (half the RAT period) of the last sync point.
     * @retval 48 MHz MLAT counts, or 0 (no timestamp) before the first sync point.
     */
    uint64_t ToMLAT48MHzCounts(uint32_t rat) const {
        if (!IsSynced()) return 0;
        int64_t mlat_ticks = static_cast<int64_t>(static_cast<int32_t>(rat - rat_)) * kMLATTicksPerRATTick;
        return mlat_48mhz_counts_ + mlat_ticks + mlat_ticks * rate_error_ppb_ / 1'000'000'000;
    }

   private:
    uint16_t num_points_ = 0;
    uint32_t rat_ = 0;
    uint64_t mlat_48mhz_counts_ = 0;
    int64_t rate_error_ppb_ = 0;  // MLAT clock rate relative to 12 x RAT, in parts per billion.
};
