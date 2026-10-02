#pragma once

#include <stddef.h>
#include <stdint.h>

// Console baud measurement for the autobaud lock (console_lock.cc). Pure logic with no SDK dependencies so it can be
// host-tested (host_test/autobaud_test.cc).
//
// The ADSBee 1421 answers the autobaud trigger (its console RX held low through a reset or a SYNC wake) with "UU" at
// its console rate. 'U' (0x55) framed 8N1 is a square wave: start bit 0, data bits 1 0 1 0 1 0 1 0 (LSB first), stop
// bit 1, so every bit ends in an edge. One 'U' gives 10 edges one bit apart, "UU" back to back gives 20.
// autobaud_edges.pio timestamps the edges on the Programmer's RX pin at the system clock, and Measure() finds the
// square wave among them.
namespace Autobaud {

// A run of one-bit intervals must cover at least a whole 'U' (start bit to the last data bit) to count.
static constexpr size_t kMinIntervals = 9;
// Measured rates outside the console range (ConsoleBaud::kMin..kMax) plus a margin are rejected.
static constexpr uint32_t kMinBaud = 9300;
static constexpr uint32_t kMaxBaud = 3100000;
// Sampling and edge jitter allowance (clock cycles) when two bits of the same level are compared.
static constexpr uint32_t kJitterCycles = 12;

// autobaud_edges.pio pushes its X counter at every edge. X counts down once per 2 clock cycles, and each push takes one
// cycle without a count, so edge n came 2 * (x[0] - x[n]) + n cycles after edge 0. Writes those cycle offsets.
void CountsToCycles(const uint32_t* counts, size_t num_edges, uint32_t* cycles);

struct Measurement {
    uint32_t baud = 0;        // 0: no square wave found.
    uint32_t first_edge = 0;  // Index of its first edge (a falling edge: the start bit).
    uint32_t bits = 0;        // Number of bit times averaged.
};

// Finds the first run of at least kMinIntervals alternating one-bit intervals that starts with a falling edge, and
// returns clock_hz divided by its average bit time. `cycles` are edge times (CountsToCycles()); edge 0 is falling if
// first_edge_falling, and the edges alternate from there.
//
// Each interval must be 35..65% of the run's first two-bit period and within 12.5% (plus kJitterCycles) of the interval
// two before it, which has the same level, and each further two-bit period must be within 12.5% of the first. Glitches
// (short intervals), bytes other than 0x55 (intervals of two or more bits) and noise end a run. High and low bits are
// compared separately, and the average uses an even number of bits so they pair up: a duty-cycle distortion of the
// line (low bits longer than high bits, or the reverse) cancels.
Measurement Measure(const uint32_t* cycles, size_t num_edges, bool first_edge_falling, uint32_t clock_hz);

// The rate the console's UART (a PL011 at 48 MHz, console_baud.hh) generates closest to `measured`. Near 3 Mbaud its
// rates are 1.5% apart (divisors 64/64, 65/64, ...), which is coarser than the measurement's error (right after a SYNC
// wake the module still runs on its RC oscillator, about 0.3% fast), so this snaps a measured 3,008,021 back to the
// 3,000,000 the console receives at. Below 250 kbaud it changes the measurement by under 0.1%. 0 stays 0.
uint32_t SnapToConsoleRate(uint32_t measured);

}  // namespace Autobaud
