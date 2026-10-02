#include "autobaud.hh"

#include "console_baud.hh"

namespace Autobaud {

void CountsToCycles(const uint32_t* counts, size_t num_edges, uint32_t* cycles) {
    for (size_t n = 0; n < num_edges; n++) cycles[n] = 2u * (counts[0] - counts[n]) + (uint32_t)n;
}

Measurement Measure(const uint32_t* cycles, size_t num_edges, bool first_edge_falling, uint32_t clock_hz) {
    Measurement result;
    for (size_t start = 0; start + kMinIntervals < num_edges; start++) {
        bool falling = (start % 2 == 0) == first_edge_falling;
        if (!falling) continue;
        uint32_t period = cycles[start + 2] - cycles[start];  // First two bits.
        if (period == 0) continue;
        size_t end = start;  // Last edge of the run.
        while (end + 1 < num_edges) {
            uint64_t interval = cycles[end + 1] - cycles[end];
            if (interval * 20 < (uint64_t)period * 7 || interval * 20 > (uint64_t)period * 13) break;
            if (end >= start + 2) {
                // Same level as two intervals back: a high bit against a high bit, a low bit against a low bit.
                uint32_t previous = cycles[end - 1] - cycles[end - 2];
                uint32_t diff = interval > previous ? (uint32_t)interval - previous : previous - (uint32_t)interval;
                if (diff > previous / 8 + kJitterCycles) break;
            }
            if ((end + 1 - start) % 2 == 0) {
                uint32_t pair = cycles[end + 1] - cycles[end - 1];
                uint32_t diff = pair > period ? pair - period : period - pair;
                if (diff * 8u > period) break;
            }
            end++;
        }
        if (end - start < kMinIntervals) continue;
        uint32_t bits = (uint32_t)((end - start) & ~(size_t)1);
        uint32_t span = cycles[start + bits] - cycles[start];
        uint32_t baud = (uint32_t)(((uint64_t)clock_hz * bits + span / 2) / span);
        if (baud < kMinBaud || baud > kMaxBaud) continue;
        result.baud = baud;
        result.first_edge = (uint32_t)start;
        result.bits = bits;
        return result;
    }
    return result;
}

uint32_t SnapToConsoleRate(uint32_t measured) {
    uint32_t snapped = ConsoleBaud::CC1314ActualBaud(measured);
    return snapped != 0 ? snapped : measured;
}

}  // namespace Autobaud
