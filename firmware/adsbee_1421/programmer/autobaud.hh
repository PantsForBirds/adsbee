#pragma once

#include <stddef.h>
#include <stdint.h>

// Edge-timing analysis for following the module console's baud rate (rate_watch.hh). Pure logic with no SDK
// dependencies so it can be host-tested (host_test/test_autobaud.cc).
//
// The ADSBee 1421 announces its console rate with "UU" (0x55 0x55): after every rate change, at boot and in answer to
// a break (firmware/adsbee_1421/ti/comms/console_autobaud.hh). 'U' framed 8N1 is a square wave: start bit 0, data
// bits 1 0 1 0 1 0 1 0 (LSB first), stop bit 1, so every bit ends in an edge, and "UU" back to back is 20 bits of it:
// 20 edges one bit apart. edge_capture.hh timestamps every edge on the Programmer's RX pin at the system clock, and
// Measure() finds the square wave among them.
namespace Autobaud {

// A run of one-bit intervals that covers a whole "UU" from the first start bit to the second 'U''s last data bit.
// Text and binary output rarely holds that much square wave; a single 'U' (9 intervals) is common.
static constexpr size_t kMinIntervalsUU = 17;
// Measured rates outside the console range (ConsoleBaud::kMin..kMax) plus a margin are rejected.
static constexpr uint32_t kMinBaud = 9300;
static constexpr uint32_t kMaxBaud = 3100000;
// Sampling and edge jitter allowance (clock cycles) when two bits of the same level are compared.
static constexpr uint32_t kJitterCycles = 12;
// Intervals shorter than this (about 100 ns at 125 MHz) are glitches, never data: a bit at 3 Mbaud is 42 cycles.
static constexpr uint32_t kGlitchCycles = 12;

// autobaud_edges.pio pushes its X counter at every edge. X starts at ~0 and counts down once per 2 clock cycles, and
// each push takes one cycle without a count, so edge n (counted from the start of the capture) came
// 2 * (~0 - x[n]) + n cycles after the capture started. Modulo 2^32: differences stay right for 34 s at 125 MHz.
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

// Finds the newest run of at least min_intervals alternating one-bit intervals that starts with a falling edge, and
// returns clock_hz divided by its average bit time. `cycles` are edge times; edge 0 is falling if first_edge_falling,
// and the edges alternate from there. Among overlapping runs it measures the longest, from its first falling edge.
//
// Each interval must be 35..65% of the run's first two-bit period and within 12.5% (plus kJitterCycles) of the interval
// two before it, which has the same level, and each further two-bit period must be within 12.5% (plus kJitterCycles) of
// the first. Glitches
// (short intervals), bytes other than 0x55 (intervals of two or more bits) and noise end a run. High and low bits are
// compared separately, and the average uses an even number of bits so they pair up: a duty-cycle distortion of the
// line (low bits longer than high bits, or the reverse) cancels.
Measurement MeasureNewest(const uint32_t* cycles, size_t num_edges, bool first_edge_falling, uint32_t clock_hz,
                          size_t min_intervals = kMinIntervalsUU);

// The rate the console's UART (a PL011 at 48 MHz, console_baud.hh) generates closest to `measured`, clamped to the
// console range. Near 3 Mbaud its rates are 1.5% apart (divisors 64/64, 65/64, ...); below 250 kbaud this changes the
// measurement by under 0.1%. 0 stays 0.
uint32_t SnapToConsoleRate(uint32_t measured);

// True if `measured` (from a run of span_cycles) is the console rate `baud` within the measurement's resolution: the
// edge timer's few cycles of jitter over the run, plus 0.5% for the line. Rates this close work on a UART anyway.
bool MatchesRate(uint32_t measured, uint32_t span_cycles, uint32_t baud);

// Number of intervals between consecutive edges that are shorter than 85% of a bit at `baud` (and longer than a
// glitch). Data at `baud` has none: two or more mean the console sends faster than the UART receives.
size_t CountShortIntervals(const uint32_t* cycles, size_t num_edges, uint32_t baud, uint32_t clock_hz);

// The cheap check run over the newest edges: true if they show the console at another rate than `baud`. Any of:
//   - two intervals shorter than 85% of a bit (faster data),
//   - three low intervals that are more than 0.3 bits off a whole number of bits, or one longer than a start bit and 8
//     data bits (slower data, or a break). Low intervals lie inside a frame; a high one can include idle time,
//   - a 'U' (9 alternating one-bit intervals) at a rate MatchesRate() doesn't accept, which also catches a "UU" at a
//     whole fraction of `baud`: at 921600 baud, "UU" at 115200 is a square wave of exactly 8-bit intervals that frames
//     without errors.
bool OtherRateHint(const uint32_t* cycles, size_t num_edges, bool first_edge_falling, uint32_t baud, uint32_t clock_hz);

// True if the edges look like data at `baud`: no interval shorter than 0.7 bits, no low stretch longer than a start
// bit and 8 data bits (that is a break, or slower data), at least 90% of the low intervals within 0.3 bits of a whole
// number of bits, and, among 16 or more intervals up to 10.5 bits, at least one of about one bit (data at a half or a
// third of `baud` has none). Fewer than 4 intervals up to 10.5 bits count as a fit (no evidence either way). Edge 0 is
// falling if first_edge_falling.
bool FitsRate(const uint32_t* cycles, size_t num_edges, bool first_edge_falling, uint32_t baud, uint32_t clock_hz);

}  // namespace Autobaud
