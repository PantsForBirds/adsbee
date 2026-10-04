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
    static constexpr uint32_t kStrongRateRatioPct = 300;
    // Per-aircraft scores are halved this often, so they follow the recent past.
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
        strong_ = !strong_;
        slice_start_ms_ = now_ms;
        if (now_ms - last_decay_ms_ >= kDecayMs) {
            last_decay_ms_ = now_ms;
            for (Aircraft& a : aircraft_) {
                a.score[0] /= 2;
                a.score[1] /= 2;
            }
        }
        return len;
    }

    // A valid frame from icao was decoded now. Returns true if it comes from a strong aircraft.
    bool OnValid(uint32_t icao, uint32_t now_ms) {
        if (now_ms - slice_start_ms_ < kSettleMs) return false;
        // Each frame scores the inverse of its slice type's share of the time, so an aircraft both settings hear
        // scores the same in both however the split changes.
        const uint32_t strong_len = StrongHold(now_ms) ? strong_slice_max_ms : strong_slice_min_ms;
        const uint32_t cycle_x16 = 16 * (weak_slice_ms + strong_len);
        const uint32_t weak_weight = cycle_x16 / weak_slice_ms;
        Aircraft& a = Find(icao, now_ms);
        a.score[strong_] += strong_ ? cycle_x16 / strong_len : weak_weight;
        if (!strong_) return false;
        // One extra MODE_S frame keeps the first few frames from tipping it.
        if (100 * uint64_t(a.score[1]) <= uint64_t(kStrongRateRatioPct) * (a.score[0] + weak_weight)) return false;
        have_strong_ = true;
        last_strong_ms_ = now_ms;
        return true;
    }

   private:
    struct Aircraft {
        uint32_t icao;
        uint32_t last_ms;   // 0 marks an empty entry.
        uint32_t score[2];  // Weighted valid frames in MODE_S [0] and STRONG [1] slices.
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
    Aircraft aircraft_[kNumAircraft] = {};
};
