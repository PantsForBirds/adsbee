#pragma once

// MODE_S_SMART time-slice policy: mostly MODE_S with short MODE_S_STRONG slices, and mostly STRONG while an aircraft
// needs it (nearby aircraft matter most). Header-only with no SDK dependencies, so the host tests can run it.

#include <cstdint>

class R1090SmartPolicy {
   public:
    static constexpr uint32_t kCycleMs = 100;
    // STRONG share of a cycle (percent) while no aircraft needs STRONG: short slices that find strong aircraft.
    static constexpr uint32_t kProbeStrongPct = 3;
    // STRONG share while an aircraft that looks strong in those slices is checked with fresh scores.
    static constexpr uint32_t kCheckStrongPct = 50;
    // A check decides after kCheckMinMs and this many STRONG frames from the aircraft, or gives up after kCheckMaxMs.
    static constexpr uint8_t kCheckFrames = 8;
    static constexpr uint32_t kCheckMinMs = 2000;
    static constexpr uint32_t kCheckMaxMs = 10000;
    static constexpr uint32_t kCheckQuietMs = 1000;
    // STRONG share while any aircraft needs STRONG: nearby aircraft come first, weak ones keep short MODE_S slices.
    static constexpr uint32_t kStrongPct = 90;
    // An aircraft looks strong when STRONG slices decode it this many times more often per second than MODE_S slices.
    static constexpr uint32_t kRateRatio = 3;
    // Strong aircraft keep the STRONG share until this long after their last frame.
    static constexpr uint32_t kHoldMs = 5000;
    // Per-aircraft scores are halved this often, so they follow the recent past.
    static constexpr uint32_t kDecayMs = 20000;
    // Aircraft are checked once heard for this long, so a lucky first frame doesn't start a check.
    static constexpr uint32_t kMinAgeMs = 1000;
    // A MODE_S-slice capture at least this strong (LR2021 RSSI of its one bits, dBm) may come from an aircraft MODE_S
    // decodes poorly: STRONG gets the check share for kBoostMs. With the AGC this RSSI tops out near -60 dBm from about
    // where MODE_S starts to lose frames, so aircraft both settings decode trigger it too: the wait before the next
    // boost doubles from kBoostIntervalMs up to kBoostIntervalMaxMs, and starts over once an aircraft needs STRONG.
    static constexpr int32_t kStrongRssiDbm = -61;
    static constexpr uint32_t kBoostMs = 3000;
    static constexpr uint32_t kBoostIntervalMs = 10000;
    static constexpr uint32_t kBoostIntervalMaxMs = 80000;
    // Frames captured this soon after a switch may have been received with the previous slice's settings.
    static constexpr uint32_t kSettleUs = 500;
    static constexpr uint16_t kNumAircraft = 32;

    uint32_t cycle_ms = kCycleMs;
    uint32_t probe_strong_pct = kProbeStrongPct;
    uint32_t check_strong_pct = kCheckStrongPct;
    uint32_t strong_share_pct = kStrongPct;
    uint32_t rate_ratio = kRateRatio;
    uint32_t decay_ms = kDecayMs;
    uint32_t min_age_ms = kMinAgeMs;
    uint32_t check_min_ms = kCheckMinMs;
    int32_t strong_rssi_dbm = kStrongRssiDbm;

    // Starts a MODE_S slice (call after every full receiver config).
    void Reset(uint32_t now_ms, uint32_t now_us) {
        strong_ = false;
        slice_start_ms_ = now_ms;
        last_decay_ms_ = now_ms;
        switch_us_ = now_us;
        prev_switch_us_ = now_us;
        for (Aircraft& a : aircraft_) a = {};
        boosted_ = false;
        boost_interval_ms_ = kBoostIntervalMs;
        strong_pct_ = probe_strong_pct;
        slice_ms_ = SliceLenMs(false);
    }

    bool strong() const { return strong_; }
    // True while icao has an entry in the aircraft table.
    bool Tracked(uint32_t icao) const {
        for (const Aircraft& a : aircraft_) {
            if (a.last_ms != 0 && a.icao == icao) return true;
        }
        return false;
    }
    // Current STRONG share of a cycle, in percent.
    uint32_t strong_pct() const { return strong_pct_; }

    // True when the current slice has ended. The caller switches the receiver, then calls Switched().
    bool SliceDone(uint32_t now_ms) const { return now_ms - slice_start_ms_ >= slice_ms_; }

    // Starts the next slice; now_us is the switch time. Returns the length in ms of the slice that ended.
    uint32_t Switched(uint32_t now_ms, uint32_t now_us) {
        const uint32_t len = now_ms - slice_start_ms_;
        strong_ = !strong_;
        slice_start_ms_ = now_ms;
        prev_switch_us_ = switch_us_;
        switch_us_ = now_us;
        if (now_ms - last_decay_ms_ >= decay_ms) {
            last_decay_ms_ = now_ms;
            for (Aircraft& a : aircraft_) {
                a.score[0] /= 2;
                a.score[1] /= 2;
            }
        }
        UpdateSplit(now_ms);
        slice_ms_ = SliceLenMs(strong_);
        return len;
    }

    // True while a capture's RSSI could change the split (a MODE_S slice without a strong aircraft or a boost).
    bool WantsRssi(uint32_t now_ms) const {
        return !strong_ && strong_pct_ == probe_strong_pct && (!boosted_ || now_ms - boost_ms_ >= boost_interval_ms_);
    }

    // A capture in a MODE_S slice had this RSSI. Returns true if it starts a boost.
    bool OnRssi(int32_t rssi_dbm, uint32_t now_ms) {
        if (rssi_dbm < strong_rssi_dbm || !WantsRssi(now_ms)) return false;
        if (boosted_ && boost_interval_ms_ < kBoostIntervalMaxMs) boost_interval_ms_ *= 2;
        boost_ms_ = now_ms;
        boosted_ = true;
        slice_ms_ = now_ms - slice_start_ms_;  // End this MODE_S slice now.
        return true;
    }

    // A valid frame from icao, captured at frame_us, was decoded at now_ms. Returns true if the aircraft needs STRONG.
    bool OnValid(uint32_t icao, uint32_t frame_us, uint32_t now_ms) {
        // The frame belongs to the slice it was captured in. Frames right after a switch or older than the previous
        // slice are not counted.
        bool in_strong = strong_;
        uint32_t since_us = frame_us - switch_us_;
        if (int32_t(since_us) < 0) {
            since_us = frame_us - prev_switch_us_;
            if (int32_t(since_us) < 0) return false;
            in_strong = !strong_;
        }
        if (since_us < kSettleUs) return false;
        // Each frame scores the inverse of its slice type's share of the time, so the scores compare decode rates.
        Aircraft& a = Find(icao, now_ms);
        // A check scores frames only while the split gives it the check share.
        if (a.state == kChecking && strong_pct_ != check_strong_pct) return false;
        a.score[in_strong] += Weight(in_strong);
        if (in_strong && a.state == kChecking) a.check_frames++;
        UpdateState(a, now_ms);
        return a.state == kStrong;
    }

   private:
    enum State : uint8_t { kUnknown = 0, kChecking, kStrong, kNotStrong };

    struct Aircraft {
        uint32_t icao;
        uint32_t first_ms;
        uint32_t last_ms;   // 0 marks an empty entry.
        uint32_t score[2];  // Weighted valid frames in MODE_S [0] and STRONG [1] slices.
        uint32_t check_ms;
        State state;
        uint8_t check_frames;
    };

    uint32_t Weight(bool strong) const {
        const uint32_t pct = strong ? strong_pct_ : 100 - strong_pct_;
        return 1600 / (pct ? pct : 1);
    }

    // A few frames in short STRONG slices weigh a lot, so an aircraft that looks strong there is checked with fresh
    // scores and more STRONG time: it needs STRONG if STRONG decodes it at least 2.5x as often as MODE_S, until MODE_S
    // decodes it at least 2/3 as often as STRONG.
    void UpdateState(Aircraft& a, uint32_t now_ms) {
        switch (a.state) {
            case kUnknown:
                // No checks while another aircraft holds the STRONG share: too few MODE_S frames to compare.
                if (strong_pct_ != strong_share_pct && now_ms - a.first_ms >= min_age_ms &&
                    a.score[1] > rate_ratio * uint64_t(a.score[0])) {
                    a.state = kChecking;
                    a.check_ms = now_ms;
                    a.check_frames = 0;
                    a.score[0] = a.score[1] = 0;
                }
                break;
            case kChecking:
                if (a.check_frames >= kCheckFrames && now_ms - a.check_ms >= check_min_ms) {
                    a.state = 2 * uint64_t(a.score[1]) >= 5 * uint64_t(a.score[0]) ? kStrong : kNotStrong;
                    a.score[0] = a.score[1] = 0;  // A strong aircraft starts fresh scores for its exit test.
                }
                break;
            case kStrong:
                // Two more MODE_S frames keep a single lucky one from ending it.
                if (3 * uint64_t(a.score[0]) > 2 * uint64_t(a.score[1]) + 2 * 3 * Weight(false)) a.state = kUnknown;
                break;
            case kNotStrong:
                break;
        }
    }

    void UpdateSplit(uint32_t now_ms) {
        bool any_strong = false, any_checking = boosted_ && now_ms - boost_ms_ < kBoostMs;
        for (Aircraft& a : aircraft_) {
            if (a.state == kChecking) {
                if (now_ms - a.check_ms >= kCheckMaxMs) {
                    a.state = kNotStrong;
                } else if (now_ms - a.last_ms < kCheckQuietMs) {
                    any_checking = true;  // A check pauses while its aircraft is silent.
                }
            } else if (a.state == kNotStrong && now_ms - a.check_ms >= kCheckMaxMs) {
                a.state = kUnknown;  // May be checked again.
            } else if (a.state == kStrong && a.last_ms != 0 && now_ms - a.last_ms < kHoldMs) {
                any_strong = true;
            }
        }
        if (any_strong) boost_interval_ms_ = kBoostIntervalMs;
        strong_pct_ = any_strong ? strong_share_pct : any_checking ? check_strong_pct : probe_strong_pct;
    }

    // The longer slice of each cycle varies by up to +-10 % at random, so short slices don't lock onto periodic
    // traffic.
    uint32_t SliceLenMs(bool strong) {
        uint32_t strong_ms = cycle_ms * strong_pct_ / 100;
        if (strong_ms < 1) strong_ms = 1;
        if (strong_ms > cycle_ms - 1) strong_ms = cycle_ms - 1;
        const uint32_t len = strong ? strong_ms : cycle_ms - strong_ms;
        if (2 * len < cycle_ms) return len;
        rand_ = rand_ * 1103515245u + 12345u;
        return len - cycle_ms / 10 + (rand_ >> 16) % (cycle_ms / 5 + 1);
    }

    // Returns the entry for icao, taking over an empty or the least recently heard one if needed.
    Aircraft& Find(uint32_t icao, uint32_t now_ms) {
        uint16_t oldest = 0;
        for (uint16_t i = 0; i < kNumAircraft; i++) {
            Aircraft& a = aircraft_[i];
            if (a.last_ms != 0 && a.icao == icao) {
                a.last_ms = now_ms | 1u;
                return a;
            }
            const Aircraft& o = aircraft_[oldest];
            if (o.last_ms != 0 && (a.last_ms == 0 || now_ms - a.last_ms > now_ms - o.last_ms)) oldest = i;
        }
        aircraft_[oldest] = {icao, now_ms, now_ms | 1u, {0, 0}, 0, kUnknown, 0};
        return aircraft_[oldest];
    }

    bool strong_ = false;
    uint32_t slice_start_ms_ = 0;
    uint32_t slice_ms_ = kCycleMs;
    uint32_t strong_pct_ = kProbeStrongPct;
    uint32_t last_decay_ms_ = 0;
    uint32_t switch_us_ = 0;
    uint32_t prev_switch_us_ = 0;
    uint32_t rand_ = 1;
    bool boosted_ = false;
    uint32_t boost_ms_ = 0;
    uint32_t boost_interval_ms_ = kBoostIntervalMs;
    Aircraft aircraft_[kNumAircraft] = {};
};
