#pragma once

#include <cstdint>

/**
 * UTC anchored to a PPS (pulse per second) edge. The RMC sentence following an edge labels it with
 * its whole UTC second.
 */
class PPSClock {
   public:
    static constexpr uint32_t kMaxLabelDelayUs = 500000;
    static constexpr uint32_t kHoldoverUs = 3000000;

    void Label(uint64_t edge_us, uint64_t now_us, uint64_t utc_unix_ms) {
        valid_ = utc_unix_ms % 1000 == 0 && now_us - edge_us < kMaxLabelDelayUs;
        edge_us_ = edge_us;
        utc_unix_ms_ = utc_unix_ms;
    }

    void Invalidate() { valid_ = false; }

    // Leaves utc_unix_ms untouched without a recent anchor.
    bool GetUTCUnixMs(uint64_t now_us, uint64_t& utc_unix_ms) const {
        uint64_t elapsed_us = now_us - edge_us_;
        if (!valid_ || elapsed_us >= kHoldoverUs) return false;
        utc_unix_ms = utc_unix_ms_ + elapsed_us / 1000;
        return true;
    }

   private:
    bool valid_ = false;
    uint64_t edge_us_ = 0;
    uint64_t utc_unix_ms_ = 0;
};
