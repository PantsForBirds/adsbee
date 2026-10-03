#include "autobaud.hh"

#include "console_baud.hh"

namespace Autobaud {

// Last edge of the run of one-bit intervals that starts at the falling edge `start` (start itself if there is none).
static size_t RunEnd(const uint32_t* cycles, size_t num_edges, size_t start) {
    if (start + 2 >= num_edges) return start;
    uint32_t period = cycles[start + 2] - cycles[start];  // First two bits.
    if (period == 0) return start;
    size_t end = start;
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
            if (diff > period / 8 + kJitterCycles) break;
        }
        end++;
    }
    return end;
}

Measurement MeasureNewest(const uint32_t* cycles, size_t num_edges, bool first_edge_falling, uint32_t clock_hz,
                          size_t min_intervals) {
    Measurement result;
    if (num_edges <= min_intervals) return result;
    // Newest falling edge that can start a long enough run.
    size_t start = num_edges - 1 - min_intervals;
    if ((start % 2 == 0) != first_edge_falling) {
        if (start == 0) return result;
        start--;
    }
    while (true) {
        size_t end = RunEnd(cycles, num_edges, start);
        if (end - start >= min_intervals) {
            // Extend back to the start of the same square wave, if it began earlier.
            while (start >= 2 && RunEnd(cycles, num_edges, start - 2) >= end) start -= 2;
            end = RunEnd(cycles, num_edges, start);
            uint32_t bits = (uint32_t)((end - start) & ~(size_t)1);
            uint32_t span = cycles[start + bits] - cycles[start];
            uint32_t baud = (uint32_t)(((uint64_t)clock_hz * bits + span / 2) / span);
            if (baud >= kMinBaud && baud <= kMaxBaud) {
                result.baud = baud;
                result.first_edge = (uint32_t)start;
                result.bits = bits;
                result.span_cycles = span;
                result.last_edge = (uint32_t)end;
                return result;
            }
        }
        if (start < 2) return result;
        start -= 2;
    }
}

uint32_t SnapToConsoleRate(uint32_t measured) {
    if (measured == 0) return 0;
    if (measured < ConsoleBaud::kMin) measured = ConsoleBaud::kMin;
    if (measured > ConsoleBaud::kMax) measured = ConsoleBaud::kMax;
    uint32_t snapped = ConsoleBaud::CC1314ActualBaud(measured);
    return snapped != 0 ? snapped : measured;
}

bool MatchesRate(uint32_t measured, uint32_t span_cycles, uint32_t baud) {
    if (measured == 0 || baud == 0) return false;
    uint32_t actual = ConsoleBaud::CC1314ActualBaud(baud);
    if (actual == 0) actual = baud;
    uint64_t diff = measured > actual ? measured - actual : actual - measured;
    // 0.5% plus 4 cycles of edge-timer jitter over the run, in ppm.
    uint64_t tolerance_ppm = 5000 + (span_cycles != 0 ? 4000000ull / span_cycles : 1000000ull);
    return diff * 1000000ull <= tolerance_ppm * actual;
}

// A bit at `baud` in 1/16 clock cycles. Arithmetic stays in 32 bits (the RP2040 has no 64-bit multiply).
static uint32_t BitX16(uint32_t baud, uint32_t clock_hz) { return (uint32_t)((uint64_t)clock_hz * 16 / baud); }

// An interval in thousandths of a bit. Intervals too long for 32 bits are over 20 bits at any console rate.
static uint32_t Millibits(uint32_t interval, uint32_t bit_x16) {
    if (interval >= UINT32_MAX / 16000) return 20000;
    return interval * 16000 / bit_x16;
}

size_t CountShortIntervals(const uint32_t* cycles, size_t num_edges, uint32_t baud, uint32_t clock_hz) {
    uint32_t bit_x16 = BitX16(baud, clock_hz);
    size_t count = 0;
    for (size_t i = 1; i < num_edges; i++) {
        uint32_t interval = cycles[i] - cycles[i - 1];
        if (interval >= kGlitchCycles && interval < bit_x16 && interval * 1600 < bit_x16 * 85) count++;
    }
    return count;
}

// How far `millibits` is from a whole number of bits, in thousandths.
static uint32_t OffGrid(uint32_t millibits) {
    uint32_t whole = (millibits + 500) / 1000 * 1000;
    return millibits > whole ? millibits - whole : whole - millibits;
}

bool OtherRateHint(const uint32_t* cycles, size_t num_edges, bool first_edge_falling, uint32_t baud,
                   uint32_t clock_hz) {
    if (CountShortIntervals(cycles, num_edges, baud, clock_hz) >= 2) return true;
    uint32_t bit_x16 = BitX16(baud, clock_hz);
    size_t misfits = 0;
    for (size_t i = 1; i < num_edges; i++) {
        uint32_t interval = cycles[i] - cycles[i - 1];
        if (interval < kGlitchCycles) continue;
        // Only low intervals: they lie inside a frame, while a high one can include idle time.
        if (((i - 1) % 2 == 0) != first_edge_falling) continue;
        uint32_t millibits = Millibits(interval, bit_x16);
        if (millibits > 9700) return true;
        if (millibits < 700) continue;
        if (OffGrid(millibits) > 300 && ++misfits >= 3) return true;
    }
    Measurement m = MeasureNewest(cycles, num_edges, first_edge_falling, clock_hz, 9);
    return m.baud != 0 && !MatchesRate(m.baud, m.span_cycles, baud);
}

bool FitsRate(const uint32_t* cycles, size_t num_edges, bool first_edge_falling, uint32_t baud, uint32_t clock_hz) {
    uint32_t bit_x16 = BitX16(baud, clock_hz);
    size_t intervals = 0;
    size_t misfits = 0;
    size_t one_bit = 0;
    for (size_t i = 1; i < num_edges; i++) {
        uint32_t interval = cycles[i] - cycles[i - 1];
        if (interval < kGlitchCycles) continue;
        bool low = ((i - 1) % 2 == 0) == first_edge_falling;  // Starts at a falling edge.
        uint32_t millibits = Millibits(interval, bit_x16);
        if (millibits < 700) return false;
        if (low && millibits > 9700) return false;
        if (millibits > 10500) continue;
        intervals++;
        if (millibits <= 1300) one_bit++;
        // A high interval can include idle time between frames (see OtherRateHint()).
        if (low && OffGrid(millibits) > 300) misfits++;
    }
    if (intervals < 4) return true;
    return misfits * 10 <= intervals && (intervals < 16 || one_bit > 0);
}

}  // namespace Autobaud
