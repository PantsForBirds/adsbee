#pragma once

// MODE_S_SMART time-slice policy: each cycle is split between MODE_S and MODE_S_STRONG slices by the aircraft that
// need each one. Header-only with no SDK dependencies, so the host tests can run it.

#include <cstdint>

class R1090SmartPolicy {
   public:
    static constexpr uint32_t kCycleMs = 100;
    // STRONG share of a cycle (percent) while no aircraft needs STRONG: short slices that find strong aircraft.
    static constexpr uint32_t kProbeStrongPct = 3;
    // STRONG share while an aircraft that looks strong in those slices is checked with fresh scores.
    static constexpr uint32_t kCheckStrongPct = 20;
    // A check decides after kCheckMinMs and this many STRONG frames from the aircraft, or gives up after kCheckMaxMs.
    static constexpr uint8_t kCheckFrames = 4;
    static constexpr uint32_t kCheckMinMs = 2000;
    static constexpr uint32_t kCheckMaxMs = 10000;
    // MODE_S share while only aircraft that need STRONG are heard: short slices that find weak aircraft.
    static constexpr uint32_t kProbeWeakPct = 5;
    // With both kinds of aircraft, STRONG gets the strong aircraft's share of them, within these bounds.
    static constexpr uint32_t kMixedMinPct = 20;
    static constexpr uint32_t kMixedMaxPct = 80;
    // An aircraft looks like it needs a setting when the other one decodes it this many times less often per second.
    static constexpr uint32_t kRateRatio = 3;
    // Aircraft count toward the split until this long after their last frame.
    static constexpr uint32_t kHoldMs = 5000;
    // Per-aircraft scores are halved this often, so they follow the recent past.
    static constexpr uint32_t kDecayMs = 20000;
    // Aircraft count toward the split once heard for this long, so their scores cover a few seconds.
    static constexpr uint32_t kMinAgeMs = 3000;
    // Frames captured this soon after a switch may have been received with the previous slice's settings.
    static constexpr uint32_t kSettleUs = 500;
    static constexpr uint16_t kNumAircraft = 32;

    uint32_t cycle_ms = kCycleMs;
    uint32_t probe_strong_pct = kProbeStrongPct;
    uint32_t check_strong_pct = kCheckStrongPct;
    uint32_t probe_weak_pct = kProbeWeakPct;
    uint32_t mixed_min_pct = kMixedMinPct;
    uint32_t mixed_max_pct = kMixedMaxPct;
    uint32_t rate_ratio = kRateRatio;
    uint32_t decay_ms = kDecayMs;
    uint32_t min_age_ms = kMinAgeMs;

    // Starts a MODE_S slice (call after every full receiver config).
    void Reset(uint32_t now_ms, uint32_t now_us) {
        strong_ = false;
        slice_start_ms_ = now_ms;
        last_decay_ms_ = now_ms;
        switch_us_ = now_us;
        prev_switch_us_ = now_us;
        for (Aircraft& a : aircraft_) a = {};
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
                if (a.state == kNotStrong) a.state = kUnknown;  // May be checked again.
            }
        }
        UpdateSplit(now_ms);
        slice_ms_ = SliceLenMs(strong_);
        return len;
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
        // A check scores frames only once the split gives it its STRONG time.
        if (a.state == kChecking && strong_pct_ < check_strong_pct) return false;
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

    bool NeedsWeak(const Aircraft& a) const { return uint64_t(a.score[0]) > uint64_t(rate_ratio) * a.score[1]; }

    // A few frames in short STRONG slices weigh a lot, so an aircraft that looks strong there is checked with fresh
    // scores and more STRONG time: it needs STRONG if its STRONG rate is at least twice MODE_S's, until MODE_S's is
    // the higher one.
    void UpdateState(Aircraft& a, uint32_t now_ms) {
        switch (a.state) {
            case kUnknown:
                if (now_ms - a.first_ms >= min_age_ms && a.score[1] > rate_ratio * uint64_t(a.score[0])) {
                    a.state = kChecking;
                    a.check_ms = now_ms;
                    a.check_frames = 0;
                    a.score[0] = a.score[1] = 0;
                }
                break;
            case kChecking:
                if (a.check_frames >= kCheckFrames && now_ms - a.check_ms >= kCheckMinMs) {
                    a.state = a.score[1] >= 2 * uint64_t(a.score[0]) ? kStrong : kNotStrong;
                }
                break;
            case kStrong:
                if (a.score[0] > a.score[1]) a.state = kUnknown;
                break;
            case kNotStrong:
                break;
        }
    }

    void UpdateSplit(uint32_t now_ms) {
        uint32_t n_strong = 0, n_weak = 0, n_checking = 0;
        for (Aircraft& a : aircraft_) {
            if (a.state == kChecking) {
                if (now_ms - a.check_ms < kCheckMaxMs) {
                    n_checking++;
                } else {
                    a.state = kNotStrong;
                }
                continue;
            }
            if (a.last_ms == 0 || now_ms - a.last_ms >= kHoldMs || now_ms - a.first_ms < min_age_ms) continue;
            if (a.state == kStrong) {
                n_strong++;
            } else if (NeedsWeak(a)) {
                n_weak++;
            }
        }
        if (n_strong == 0) {
            strong_pct_ = n_checking ? check_strong_pct : probe_strong_pct;
        } else if (n_weak == 0) {
            strong_pct_ = 100 - probe_weak_pct;
        } else {
            strong_pct_ = 100 * n_strong / (n_strong + n_weak);
            if (strong_pct_ < mixed_min_pct) strong_pct_ = mixed_min_pct;
            if (strong_pct_ > mixed_max_pct) strong_pct_ = mixed_max_pct;
        }
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
    Aircraft aircraft_[kNumAircraft] = {};
};
