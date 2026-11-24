#pragma once

#include <cstdint>

/**
 * Absolute GNSS time (UTC) on the 48 MHz MLAT counter that Mode S, UAT and Remote ID timestamps use.
 *
 * The receiver core stamps each rising edge of the GNSS PPS (pulse per second) with the MLAT counter, with the same
 * reads as a Mode S stamp (ADSBee::OnDemodBegin()). The RMC sentence that follows an edge labels it with its UTC second
 * (the labeling and RMC date parsing follow PR #451 by Mark Paluch, which stamped the edge with time_us_64()). Each
 * labeled edge is a point. As in MLATTimeSync, the mapping is an anchor point plus the MLAT counter rate (the second
 * point gives the first rate). The PPS capture delay is one-sided (an edge is only ever stamped late), so the anchor is
 * not the raw last edge but a filtered one: each edge moves it a quarter of the way from the prediction, and the rate
 * by a sixteenth of the error (an alpha-beta filter), after late (and early) edges were dropped.
 *
 * An edge is only labeled when the RMC reports a fix (status A, with a date) on a whole second, when it is the one
 * edge of the last kMinEdgeIntervalUs and when it is younger than kMaxLabelDelayUs (the RMC is assumed to arrive
 * within that of its edge). A leap second (second 60) starts the fit again. Missing edges are fine: the rate is
 * measured over the gap. Once synced, a point further than kMaxResidualNs from the prediction (a PPS edge whose
 * interrupt waited for another one) is dropped. A point up to kMaxShiftNs early instead moves the anchor to it: as the
 * delay is one-sided, the fit was anchored on late edges (the capture delay is ~1.6 us longer when the receiver has had
 * no traffic for a while), or the rate is low, so the rate takes a sixteenth of the error too. After kMaxOutliers
 * dropped points in a row the anchor moves the same way to the earliest of them if the last is within kMaxShiftNs
 * (the delay got longer), else the fit starts again from the last (a mislabeled second).
 *
 * Seconds are POSIX: UTC seconds since 1970 without leap seconds. Not thread safe: core 0 only.
 */
class PPSClock {
   public:
    static constexpr int64_t kNominalCountsPerSecond = 48'000'000;
    static constexpr int64_t kMaxRateErrorPPB = 200'000;  // 200 ppm, as MLATTimeSync.
    // Counts within this of the last point convert to UTC; also the longest gap between the first two points of a fit.
    static constexpr int64_t kHoldoverSeconds = 4;
    // Longest gap between two points of a synced fit (over dropped edges).
    static constexpr int64_t kMaxGapSeconds = 16;
    static constexpr int32_t kMaxResidualNs = 500;
    static constexpr int32_t kMaxShiftNs = 20'000;
    static constexpr uint16_t kMaxOutliers = 8;
    static constexpr uint32_t kMaxLabelDelayUs = 900'000;
    static constexpr uint32_t kMinEdgeIntervalUs = 900'000;
    static constexpr int64_t kAnchorGainDivisor = 4;
    static constexpr int64_t kRateGainDivisor = 16;
    static constexpr int64_t kAnchorSubCounts = 1024;

    // A PPS edge as the receiver core stamped it.
    struct Edge {
        uint64_t mlat_48mhz_counts = 0;
        uint32_t time_us = 0;  // time_us_32() at the stamp, for the age of the edge.
    };

    enum Result : uint8_t {
        kLabeled = 0,  // The edge is a point of the fit.
        kRestarted,    // The edge didn't agree with the fit (or was its first point): the fit starts again from it.
        kOutlier,      // Dropped: too far from the prediction.
        kNoFix,        // The RMC had no fix, no date or not a whole second.
        kLeapSecond,   // Second 60: the fit starts again after it.
        kNoEdge,       // No new edge since the last RMC (a missing edge or a second RMC).
        kLate,         // The latest edge is older than kMaxLabelDelayUs.
        kExtraEdge,    // Two edges within kMinEdgeIntervalUs: which one is the second is unknown.
        kReanchored,   // The edge was early (the anchor moved to it), or the last of kMaxOutliers late ones.
        kNumResults
    };

    // Forgets the points (the PPS stopped, GNSS was disabled).
    void Restart() {
        num_points_ = 0;
        rate_error_ppb_ = 0;
        num_outliers_ = 0;
    }

    /**
     * Labels the latest PPS edge with the second of an RMC sentence that just arrived.
     * @param[in] rmc_valid True if the RMC reported a fix with a date, on a whole second below 60.
     * @param[in] leap_second True if the RMC second is 60.
     * @param[in] unix_seconds The RMC's UTC second (POSIX), if rmc_valid.
     * @param[in] num_edges Edges stamped so far. latest and previous are the last two of them.
     * @param[in] now_us time_us_32() now.
     */
    Result OnRMC(bool rmc_valid, bool leap_second, uint64_t unix_seconds, uint32_t num_edges, const Edge& latest,
                 const Edge& previous, uint32_t now_us) {
        Result result;
        if (leap_second) {
            Restart();
            result = kLeapSecond;
        } else if (!rmc_valid) {
            result = kNoFix;
        } else if (num_edges == 0 || num_edges == last_num_edges_) {
            result = kNoEdge;
        } else if (now_us - latest.time_us > kMaxLabelDelayUs) {
            result = kLate;
        } else if (num_edges > 1 && latest.time_us - previous.time_us < kMinEdgeIntervalUs) {
            result = kExtraEdge;
        } else {
            result = AddPoint(latest.mlat_48mhz_counts, unix_seconds);
            if (result != kOutlier) last_point_time_us_ = latest.time_us;
        }
        last_num_edges_ = num_edges;
        last_result_ = result;
        result_counts_[result]++;
        return result;
    }

    /**
     * Adds a PPS edge and its UTC second.
     * @retval kLabeled, kRestarted, kReanchored or kOutlier.
     */
    Result AddPoint(uint64_t mlat_48mhz_counts, uint64_t unix_seconds) {
        if (num_points_ > 0) {
            int64_t seconds = static_cast<int64_t>(unix_seconds - unix_seconds_);
            int64_t mlat_ticks = static_cast<int64_t>(mlat_48mhz_counts - mlat_48mhz_counts_);
            int64_t expected_mlat_ticks = seconds * kNominalCountsPerSecond;
            bool agrees = seconds > 0 && seconds <= (IsSynced() ? kMaxGapSeconds : kHoldoverSeconds);
            if (agrees && IsSynced()) {
                // In 1/kAnchorSubCounts counts: the anchor keeps the fraction of a count the filter gives it.
                int64_t expected_sub = expected_mlat_ticks * kAnchorSubCounts;
                int64_t predicted_sub = expected_sub + expected_sub * rate_error_ppb_ / 1'000'000'000;
                int64_t error_sub = mlat_ticks * kAnchorSubCounts - anchor_sub_counts_ - predicted_sub;
                int64_t error_ns = error_sub * 125 / (6 * kAnchorSubCounts);  // To ns.
                if (error_ns <= kMaxResidualNs && error_ns >= -kMaxResidualNs) {
                    last_residual_ns_ = static_cast<int32_t>(error_ns);
                    // Mean square over about the last 16 points.
                    int64_t square_ns2 = error_ns * error_ns;
                    mean_square_residual_ns2_ += (square_ns2 - mean_square_residual_ns2_) / 16;
                    // The anchor moves a quarter of the way from the prediction to the edge, the rate by a sixteenth of
                    // the error (an alpha-beta filter): single edges move the conversion by less than their jitter.
                    int64_t anchor_sub = anchor_sub_counts_ + predicted_sub + error_sub / kAnchorGainDivisor;
                    int64_t anchor_counts = anchor_sub / kAnchorSubCounts;
                    anchor_sub_counts_ = anchor_sub - anchor_counts * kAnchorSubCounts;
                    if (anchor_sub_counts_ < 0) {
                        anchor_sub_counts_ += kAnchorSubCounts;
                        anchor_counts--;
                    }
                    mlat_48mhz_counts_ += anchor_counts;
                    rate_error_ppb_ += error_sub * 1'000'000'000 / expected_sub / kRateGainDivisor;
                    unix_seconds_ = unix_seconds;
                    num_outliers_ = 0;
                    if (num_points_ < UINT16_MAX) num_points_++;
                    return kLabeled;
                }
                bool shifted = error_ns >= -kMaxShiftNs && error_ns <= kMaxShiftNs;
                if (shifted && error_ns < 0) {
                    return Reanchor(mlat_48mhz_counts, unix_seconds, error_sub, expected_sub);
                }
                if (shifted && (num_outliers_ == 0 || error_ns < earliest_outlier_.error_ns)) {
                    earliest_outlier_ = {mlat_48mhz_counts, unix_seconds, error_sub, expected_sub, error_ns};
                }
                if (++num_outliers_ < kMaxOutliers) return kOutlier;
                if (shifted) {
                    const Outlier& o = earliest_outlier_;
                    return Reanchor(o.mlat_48mhz_counts, o.unix_seconds, o.error_sub, o.expected_sub);
                }
                agrees = false;  // A mislabeled second, a second or more from the prediction.
            } else if (agrees) {
                int64_t rate_error_ppb = (mlat_ticks - expected_mlat_ticks) * 1'000'000'000 / expected_mlat_ticks;
                agrees = rate_error_ppb <= kMaxRateErrorPPB && rate_error_ppb >= -kMaxRateErrorPPB;
            }
            if (agrees) {
                // Second point: the rate between the two.
                rate_error_ppb_ = (mlat_ticks - expected_mlat_ticks) * 1'000'000'000 / expected_mlat_ticks;
                num_outliers_ = 0;
                mlat_48mhz_counts_ = mlat_48mhz_counts;
                anchor_sub_counts_ = 0;
                unix_seconds_ = unix_seconds;
                num_points_++;
                return kLabeled;
            }
            Restart();
        }
        // First point of a fit.
        num_outliers_ = 0;
        mean_square_residual_ns2_ = 0;
        mlat_48mhz_counts_ = mlat_48mhz_counts;
        anchor_sub_counts_ = 0;
        unix_seconds_ = unix_seconds;
        num_points_ = 1;
        return kRestarted;
    }

    bool IsSynced() const { return num_points_ >= 2; }

    // True if synced and the last point is younger than kHoldoverSeconds.
    bool IsFresh(uint32_t now_us) const {
        return IsSynced() && now_us - last_point_time_us_ < static_cast<uint32_t>(kHoldoverSeconds * 1'000'000);
    }

    /**
     * Converts an MLAT count to UTC.
     * @param[in] mlat_48mhz_counts Count within kHoldoverSeconds of the last point.
     * @param[out] unix_seconds UTC second (POSIX).
     * @param[out] ns Nanoseconds into that second.
     * @retval False if not synced or too far from the last point.
     */
    bool ToUTC(uint64_t mlat_48mhz_counts, uint64_t& unix_seconds, uint32_t& ns) const {
        if (!IsSynced()) return false;
        int64_t mlat_ticks = static_cast<int64_t>(mlat_48mhz_counts - mlat_48mhz_counts_);
        if (mlat_ticks > kHoldoverSeconds * kNominalCountsPerSecond ||
            mlat_ticks < -kHoldoverSeconds * kNominalCountsPerSecond) {
            return false;
        }
        int64_t elapsed_ns = (mlat_ticks * kAnchorSubCounts - anchor_sub_counts_) * 125 / (6 * kAnchorSubCounts);
        elapsed_ns -= elapsed_ns * rate_error_ppb_ / (1'000'000'000 + rate_error_ppb_);
        int64_t seconds = elapsed_ns / 1'000'000'000;
        int64_t remainder_ns = elapsed_ns % 1'000'000'000;
        if (remainder_ns < 0) {
            remainder_ns += 1'000'000'000;
            seconds--;
        }
        unix_seconds = unix_seconds_ + seconds;
        ns = static_cast<uint32_t>(remainder_ns);
        return true;
    }

    uint16_t GetNumPoints() const { return num_points_; }
    // MLAT counter rate relative to 48 MHz of GNSS time, in parts per billion.
    int64_t GetRateErrorPPB() const { return rate_error_ppb_; }
    // Distance of the last point from the prediction, and the RMS over about the last 16 points.
    int32_t GetLastResidualNs() const { return last_residual_ns_; }
    int64_t GetMeanSquareResidualNs2() const { return mean_square_residual_ns2_; }
    Result GetLastResult() const { return last_result_; }
    uint32_t GetResultCount(Result result) const { return result < kNumResults ? result_counts_[result] : 0; }

   private:
    // Moves the anchor to a point error_sub (sub-counts) from the prediction over expected_sub, keeping the fit.
    Result Reanchor(uint64_t mlat_48mhz_counts, uint64_t unix_seconds, int64_t error_sub, int64_t expected_sub) {
        mlat_48mhz_counts_ = mlat_48mhz_counts;
        anchor_sub_counts_ = 0;
        rate_error_ppb_ += error_sub * 1'000'000'000 / expected_sub / kRateGainDivisor;  // In case it's the rate.
        unix_seconds_ = unix_seconds;
        mean_square_residual_ns2_ = 0;
        num_outliers_ = 0;
        return kReanchored;
    }

    // The earliest point of the current run of outliers within kMaxShiftNs.
    struct Outlier {
        uint64_t mlat_48mhz_counts;
        uint64_t unix_seconds;
        int64_t error_sub;
        int64_t expected_sub;
        int64_t error_ns;
    };
    Outlier earliest_outlier_ = {};

    uint16_t num_points_ = 0;  // Since the fit started.
    uint16_t num_outliers_ = 0;  // In a row.
    // Anchor: MLAT count of the PPS edge of unix_seconds_, plus anchor_sub_counts_ / kAnchorSubCounts.
    uint64_t mlat_48mhz_counts_ = 0;
    int64_t anchor_sub_counts_ = 0;
    uint64_t unix_seconds_ = 0;
    int64_t rate_error_ppb_ = 0;
    int32_t last_residual_ns_ = 0;
    int64_t mean_square_residual_ns2_ = 0;
    uint32_t last_point_time_us_ = 0;
    uint32_t last_num_edges_ = 0;
    Result last_result_ = kNoEdge;
    uint32_t result_counts_[kNumResults] = {};
};
