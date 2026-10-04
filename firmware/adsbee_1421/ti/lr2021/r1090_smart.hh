#pragma once

// MODE_S_SMART time-slice policy: mostly MODE_S, with short MODE_S_STRONG slices that grow while aircraft are around
// that MODE_S_STRONG hears much better than MODE_S. Header-only with no SDK dependencies, so the host tests can run it.

#include <cstdint>

class R1090SmartPolicy {
   public:
    static constexpr uint32_t kWeakSliceMs = 100;
    static constexpr uint32_t kStrongSliceMinMs = 12;
    static constexpr uint32_t kStrongSliceMaxMs = 60;
    // Long STRONG slices continue this long after the last frame from a strong aircraft.
    static constexpr uint32_t kStrongHoldMs = 5000;
    // A strong aircraft is decoded at least this much more often (percent) per second of STRONG slices than per
    // second of MODE_S slices.
    static constexpr uint32_t kStrongRateRatioPct = 150;
    // Per-aircraft counts and slice times are halved this often, so they follow the recent past.
    static constexpr uint32_t kDecayMs = 5000;
    // Decodes this soon after a switch may have been captured in the previous slice, so they don't count.
    static constexpr uint32_t kSettleMs = 3;
    static constexpr uint16_t kNumAircraft = 32;

    uint32_t weak_slice_ms = kWeakSliceMs;
    uint32_t strong_slice_min_ms = kStrongSliceMinMs;
    uint32_t strong_slice_max_ms = kStrongSliceMaxMs;

    // Starts a MODE_S slice (call after every full receiver config).
    void Reset(uint32_t now_ms) {
        strong_ = false;
        slice_start_ms_ = now_ms;
        last_decay_ms_ = now_ms;
        have_strong_ = false;
        time_ms_[0] = time_ms_[1] = 0;
        for (Aircraft& a : aircraft_) a = {};
    }

    bool strong() const { return strong_; }

    // True while a strong aircraft keeps the STRONG slices long.
    bool StrongHold(uint32_t now_ms) const { return have_strong_ && now_ms - last_strong_ms_ < kStrongHoldMs; }

    uint32_t SliceMs(uint32_t now_ms) const {
        if (!strong_) return weak_slice_ms;
        return StrongHold(now_ms) ? strong_slice_max_ms : strong_slice_min_ms;
    }

    // True when the current slice has ended. The caller switches the receiver, then calls Switched().
    bool SliceDone(uint32_t now_ms) const { return now_ms - slice_start_ms_ >= SliceMs(now_ms); }

    // Returns the slice length in ms that just ended.
    uint32_t Switched(uint32_t now_ms) {
        uint32_t len = now_ms - slice_start_ms_;
        time_ms_[strong_] += len;
        strong_ = !strong_;
        slice_start_ms_ = now_ms;
        if (now_ms - last_decay_ms_ >= kDecayMs) {
            last_decay_ms_ = now_ms;
            time_ms_[0] /= 2;
            time_ms_[1] /= 2;
            for (Aircraft& a : aircraft_) {
                a.n[0] /= 2;
                a.n[1] /= 2;
            }
        }
        return len;
    }

    // A valid frame from icao was decoded now. Returns true if it comes from a strong aircraft.
    bool OnValid(uint32_t icao, uint32_t now_ms) {
        if (now_ms - slice_start_ms_ < kSettleMs) return false;
        Aircraft& a = Find(icao, now_ms);
        a.n[strong_]++;
        if (!strong_) return false;
        // Rates per ms of slice time, compared without division. The current slice counts toward the STRONG time, and
        // at least one long slice of it is assumed so a single early frame doesn't look like a high rate.
        uint64_t strong_ms = time_ms_[1] + (now_ms - slice_start_ms_);
        if (strong_ms < strong_slice_max_ms) strong_ms = strong_slice_max_ms;
        if (100 * uint64_t(a.n[1]) * time_ms_[0] <= uint64_t(kStrongRateRatioPct) * a.n[0] * strong_ms) return false;
        have_strong_ = true;
        last_strong_ms_ = now_ms;
        return true;
    }

   private:
    struct Aircraft {
        uint32_t icao;
        uint32_t last_ms;  // 0 marks an empty entry.
        uint32_t n[2];     // Valid frames in MODE_S [0] and STRONG [1] slices.
    };

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
        aircraft_[oldest] = {icao, now_ms | 1u, {0, 0}};
        return aircraft_[oldest];
    }

    bool strong_ = false;
    uint32_t slice_start_ms_ = 0;
    uint32_t last_decay_ms_ = 0;
    bool have_strong_ = false;
    uint32_t last_strong_ms_ = 0;
    uint32_t time_ms_[2] = {0, 0};  // Decayed time in MODE_S [0] and STRONG [1] slices.
    Aircraft aircraft_[kNumAircraft] = {};
};
