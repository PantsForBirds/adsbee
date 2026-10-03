#pragma once

#include <stddef.h>
#include <stdint.h>

// Measures the console's baud rate from RX edge times (rate_watch.hh). The module's "UU" framed 8N1 is a square wave
// of 20 one-bit intervals. Pure logic, host-tested in host_test/test_autobaud.cc.
namespace Autobaud {

// One-bit intervals in a "UU" (first start bit to last data bit). A single 'U' (9) is common in normal output.
static constexpr size_t kMinIntervalsUU = 17;
// Console rate range (ConsoleBaud::kMin..kMax) plus a margin.
static constexpr uint32_t kMinBaud = 9300;
static constexpr uint32_t kMaxBaud = 3100000;
// Edge jitter allowance (clock cycles).
static constexpr uint32_t kJitterCycles = 12;
// Shorter intervals are glitches (a bit at 3 Mbaud is 42 cycles).
static constexpr uint32_t kGlitchCycles = 12;

// Converts the PIO counter pushed at edge n to clock cycles since the capture started. The counter starts at ~0 and
// counts down every 2 cycles; each push costs one extra cycle. Differences wrap after 34 s at 125 MHz.
inline uint32_t CountToCycles(uint32_t count, uint64_t edge_index) {
    return 2u * (0xFFFFFFFFu - count) + (uint32_t)edge_index;
}

struct Measurement {
    uint32_t baud = 0;         // 0: no square wave found.
    uint32_t first_edge = 0;   // Index of its first edge (a falling edge: the start bit).
    uint32_t bits = 0;         // Number of bit times averaged.
    uint32_t span_cycles = 0;  // Clock cycles those bits took.
    uint32_t last_edge = 0;    // Index of the run's last edge.
};

// Finds the newest run of at least min_intervals one-bit intervals starting on a falling edge and returns its rate.
// Intervals must stay within 12.5% of the same-level interval before them. An even number of bits is averaged so
// duty-cycle distortion cancels.
Measurement MeasureNewest(const uint32_t* cycles, size_t num_edges, bool first_edge_falling, uint32_t clock_hz,
                          size_t min_intervals = kMinIntervalsUU);

// Closest rate the console's UART can generate (console_baud.hh), clamped to the console range. 0 stays 0.
uint32_t SnapToConsoleRate(uint32_t measured);

// True if `measured` (from a run of span_cycles) matches `baud` within edge jitter plus 0.5%.
bool MatchesRate(uint32_t measured, uint32_t span_cycles, uint32_t baud);

// Intervals shorter than 85% of a bit at `baud` (excluding glitches). Two or more mean faster data.
size_t CountShortIntervals(const uint32_t* cycles, size_t num_edges, uint32_t baud, uint32_t clock_hz);

// Quick check of the newest edges: true if they show data at a rate other than `baud`: short intervals, low intervals
// off the bit grid or longer than 9 bits, or a 'U' at another rate (catches "UU" at an exact fraction of `baud`, which
// frames without errors). Only low intervals are checked against the grid, since high ones can include idle time.
bool OtherRateHint(const uint32_t* cycles, size_t num_edges, bool first_edge_falling, uint32_t baud, uint32_t clock_hz);

// True if the edges look like data at `baud`: no short intervals or breaks, low intervals on the bit grid, and some
// one-bit intervals (data at a fraction of `baud` has none). Too few edges count as a fit.
bool FitsRate(const uint32_t* cycles, size_t num_edges, bool first_edge_falling, uint32_t baud, uint32_t clock_hz);

}  // namespace Autobaud
