#pragma once

// MODE_S_SMART time-slice policy: mostly MODE_S, with short MODE_S_STRONG slices that grow while aircraft are decoded
// that only MODE_S_STRONG hears. Header-only with no SDK dependencies, so the host tests can run it.

#include <cstdint>

class R1090SmartPolicy {
   public:
    static constexpr uint32_t kWeakSliceMs = 100;
    static constexpr uint32_t kStrongSliceMinMs = 12;
    static constexpr uint32_t kStrongSliceMaxMs = 60;
    // Long STRONG slices continue this long after the last aircraft heard only in STRONG slices.
    static constexpr uint32_t kStrongHoldMs = 5000;
    // An aircraft decoded in a MODE_S slice this recently isn't a strong-only aircraft.
    static constexpr uint32_t kWeakSeenMs = 5000;
    // Decodes this soon after a switch may have been captured in the previous slice, so they don't count.
    static constexpr uint32_t kSettleMs = 3;
    static constexpr uint16_t kNumWeakIcaos = 32;

    uint32_t weak_slice_ms = kWeakSliceMs;
    uint32_t strong_slice_min_ms = kStrongSliceMinMs;
    uint32_t strong_slice_max_ms = kStrongSliceMaxMs;

    // Starts a MODE_S slice (call after every full receiver config).
    void Reset(uint32_t now_ms) {
        strong_ = false;
        slice_start_ms_ = now_ms;
        have_strong_only_ = false;
    }

    bool strong() const { return strong_; }

    // True while aircraft heard only in STRONG slices keep the STRONG slices long.
    bool StrongHold(uint32_t now_ms) const {
        return have_strong_only_ && now_ms - last_strong_only_ms_ < kStrongHoldMs;
    }

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
        return len;
    }

    // A valid frame from icao was decoded now. Returns true if it marks a strong-only aircraft.
    bool OnValid(uint32_t icao, uint32_t now_ms) {
        if (now_ms - slice_start_ms_ < kSettleMs) return false;
        if (!strong_) {
            RememberWeak(icao, now_ms);
            return false;
        }
        if (WeakSeen(icao, now_ms)) return false;
        have_strong_only_ = true;
        last_strong_only_ms_ = now_ms;
        return true;
    }

   private:
    struct WeakIcao {
        uint32_t icao;
        uint32_t ms;
    };

    void RememberWeak(uint32_t icao, uint32_t now_ms) {
        uint16_t oldest = 0;
        for (uint16_t i = 0; i < kNumWeakIcaos; i++) {
            if (weak_[i].ms != 0 && weak_[i].icao == icao) {
                weak_[i].ms = now_ms | 1u;  // 0 marks an empty entry.
                return;
            }
            if (weak_[i].ms == 0 || now_ms - weak_[i].ms > now_ms - weak_[oldest].ms) oldest = i;
        }
        weak_[oldest] = {icao, now_ms | 1u};
    }

    bool WeakSeen(uint32_t icao, uint32_t now_ms) const {
        for (uint16_t i = 0; i < kNumWeakIcaos; i++) {
            if (weak_[i].ms != 0 && weak_[i].icao == icao && now_ms - weak_[i].ms < kWeakSeenMs) return true;
        }
        return false;
    }

    bool strong_ = false;
    uint32_t slice_start_ms_ = 0;
    bool have_strong_only_ = false;
    uint32_t last_strong_only_ms_ = 0;
    WeakIcao weak_[kNumWeakIcaos] = {};
};
