// Host tests for the console autobaud measurement (autobaud.hh). A cycle-level model of autobaud_edges.pio samples
// generated UART waveforms ("UU" from the module, at the rate its PL011 actually generates), and the pushed counter
// values go through CountsToCycles() and Measure() as on the Programmer.
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <initializer_list>
#include <vector>

#include "gtest/gtest.h"
#include "autobaud.hh"
#include "console_baud.hh"


static constexpr uint32_t kClockHz = 125000000;  // The Programmer's clk_sys.
static constexpr size_t kMaxEdges = 64;          // console_lock.cc's capture buffer.

namespace {

// A line level change at `t` (in clock cycles, fractional) to `level`.
struct Edge {
    double t;
    bool level;
};

}  // namespace

// Builds the 8N1 waveform of `bytes` sent back to back at `baud`, starting after `idle_bits` of idle (high). Every
// falling edge comes `skew` bit times early and every rising edge `skew` late (duty-cycle distortion: low bits longer).
static std::vector<Edge> Uart(const std::vector<uint8_t>& bytes, double baud, double idle_bits = 20,
                              double skew = 0) {
    std::vector<Edge> edges;
    double bit = kClockHz / baud;
    bool level = true;
    double t = idle_bits * bit;
    for (uint8_t byte : bytes) {
        bool bits[10] = {false};
        for (int i = 0; i < 8; i++) bits[1 + i] = (byte >> i) & 1;
        bits[9] = true;
        for (int i = 0; i < 10; i++) {
            if (bits[i] != level) {
                level = bits[i];
                edges.push_back({t + (level ? skew : -skew) * bit, level});
            }
            t += bit;
        }
    }
    return edges;
}

// Cycle-level model of autobaud_edges.pio: the pin goes through the 2-flop input synchronizer, `jmp pin` and `in`
// take one cycle each, X starts at ~0 and the state machine at `high` (line high). Returns the pushed X values.
static std::vector<uint32_t> RunPio(const std::vector<Edge>& edges, uint32_t max_edges = kMaxEdges) {
    enum { kRise, kHigh, kHighPin, kFall, kLow, kLowPin };
    std::vector<uint32_t> pushed;
    uint32_t x = 0xFFFFFFFFu;
    int pc = kHigh;
    size_t next = 0;
    bool level = true;
    bool sync1 = true, sync2 = true;
    double end = edges.empty() ? 1000 : edges.back().t + 2 * kClockHz / 9600.0;
    for (uint64_t cycle = 0; cycle < (uint64_t)end && pushed.size() < max_edges; cycle++) {
        while (next < edges.size() && edges[next].t <= (double)cycle) level = edges[next++].level;
        bool pin = sync2;  // What the state machine sees this cycle.
        sync2 = sync1;
        sync1 = level;
        switch (pc) {
            case kRise:
            case kFall:
                pushed.push_back(x);
                pc = pc == kRise ? kHigh : kLow;
                break;
            case kHigh:
            case kLow:
                x--;  // jmp x-- to the next instruction: X counts either way.
                pc = pc == kHigh ? kHighPin : kLowPin;
                break;
            case kHighPin:
                pc = pin ? kHigh : kFall;
                break;
            case kLowPin:
                pc = pin ? kRise : kLow;  // Not taken: .wrap back to low.
                break;
        }
    }
    return pushed;
}

static Autobaud::Measurement MeasureCounts(const std::vector<uint32_t>& counts, bool first_falling = true) {
    std::vector<uint32_t> cycles(counts.size());
    if (!counts.empty()) Autobaud::CountsToCycles(counts.data(), counts.size(), cycles.data());
    return Autobaud::Measure(cycles.data(), cycles.size(), first_falling, kClockHz);
}

static Autobaud::Measurement MeasureWave(const std::vector<Edge>& edges) { return MeasureCounts(RunPio(edges)); }

static uint32_t ErrorPpm(double actual, uint32_t measured) {
    double e = (measured - actual) / actual;
    return (uint32_t)((e < 0 ? -e : e) * 1e6);
}

static const std::vector<uint8_t> kUU = {'U', 'U'};

// PIO model: edge n at 2 * (x[0] - x[n]) + n cycles.
TEST(Autobaud, CounterModel) {
    // A square wave with edges exactly every 100 cycles: the cycle offsets come back exact to the 2-cycle sampling.
    std::vector<Edge> edges;
    for (int i = 0; i < 20; i++) edges.push_back({1000.0 + 100.0 * i, i % 2 == 1});
    std::vector<uint32_t> counts = RunPio(edges);
    EXPECT_EQ(counts.size(), 20u);
    std::vector<uint32_t> cycles(counts.size());
    Autobaud::CountsToCycles(counts.data(), counts.size(), cycles.data());
    for (size_t n = 0; n < cycles.size(); n++) {
        int32_t err = (int32_t)cycles[n] - (int32_t)(100 * n);
        EXPECT_TRUE(err >= -2 && err <= 2);
    }
}

// "UU" at 9600..3 M (the CC1314's actual rate) measured within 0.4%.
TEST(Autobaud, Rates) {
    uint32_t worst = 0, worst_baud = 0;
    std::vector<uint32_t> rates = {9600,   19200,  38400,  57600,   76800,   115200,  123457,  230400, 250000,
                                   345678, 460800, 500000, 921600,  1000000, 1500000, 2000000, 2500000, 3000000};
    for (uint32_t baud = ConsoleBaud::kMin; baud <= ConsoleBaud::kMax; baud += 7919) rates.push_back(baud);
    for (uint32_t nominal : rates) {
        double actual = ConsoleBaud::CC1314ActualBaud(nominal);
        // Several phases of the waveform against the sampling clock.
        for (double idle : {20.0, 20.37, 21.5, 33.81}) {
            Autobaud::Measurement m = MeasureWave(Uart(kUU, actual, idle));
            EXPECT_TRUE(m.baud != 0);
            EXPECT_EQ(m.first_edge, 0u);
            EXPECT_EQ(m.bits, 18u);
            uint32_t e = ErrorPpm(actual, m.baud);
            if (e > worst) {
                worst = e;
                worst_baud = nominal;
            }
            EXPECT_TRUE(e < 4000);
        }
    }
    printf("  worst error %u ppm (at %u baud)\n", (unsigned)worst, (unsigned)worst_baud);
}

// One 'U' is enough.
TEST(Autobaud, SingleU) {
    for (uint32_t nominal : {9600u, 115200u, 1000000u, 3000000u}) {
        double actual = ConsoleBaud::CC1314ActualBaud(nominal);
        Autobaud::Measurement m = MeasureWave(Uart({'U'}, actual));
        EXPECT_EQ(m.bits, 8u);
        EXPECT_TRUE(ErrorPpm(actual, m.baud) < 8000);
    }
}

// Edge jitter and duty-cycle distortion.
TEST(Autobaud, JitterAndDistortion) {
    srand(1);
    uint32_t worst = 0;
    for (uint32_t nominal : {9600u, 57600u, 115200u, 921600u, 1000000u, 2000000u, 3000000u}) {
        double actual = ConsoleBaud::CC1314ActualBaud(nominal);
        double bit = kClockHz / actual;
        for (int trial = 0; trial < 50; trial++) {
            // Up to +-3 cycles (24 ns) of random edge jitter on top of the sampling, and low bits up to 1.1 bit times
            // with high bits down to 0.9 (or the reverse).
            double skew = 0.05 * (trial % 5 - 2) / 2.0;
            std::vector<Edge> edges = Uart(kUU, actual, 20 + trial * 0.13, skew);
            for (Edge& edge : edges) edge.t += (rand() % 7 - 3);
            Autobaud::Measurement m = MeasureWave(edges);
            EXPECT_TRUE(m.baud != 0);
            uint32_t e = ErrorPpm(actual, m.baud);
            if (e > worst) worst = e;
            // 6 cycles of jitter and 4 of sampling over the bits averaged. Near 3 M, jitter can end the run early.
            EXPECT_TRUE(m.bits >= 8);
            EXPECT_TRUE(e <= (uint32_t)(1e6 * 10 / (m.bits * bit)) + 500);
        }
    }
    printf("  worst error %u ppm\n", (unsigned)worst);
}

// Glitches before or inside the "UU".
TEST(Autobaud, Glitches) {
    for (uint32_t nominal : {9600u, 115200u, 1000000u, 3000000u}) {
        double actual = ConsoleBaud::CC1314ActualBaud(nominal);
        double bit = kClockHz / actual;
        // A 100 ns low glitch on the idle line first: two extra edges ahead of the square wave.
        std::vector<Edge> edges = Uart(kUU, actual);
        edges.insert(edges.begin(), {{5.0 * bit, false}, {5.0 * bit + 12, true}});
        Autobaud::Measurement m = MeasureWave(edges);
        EXPECT_TRUE(m.first_edge == 2 && m.bits == 18);
        EXPECT_TRUE(ErrorPpm(actual, m.baud) < 4000);
        // A glitch inside the first 'U' (in a high bit): the second one still measures.
        if (bit > 60) {
            edges = Uart(kUU, actual);
            double t = (edges[1].t + edges[2].t) / 2;
            edges.insert(edges.begin() + 2, {{t, false}, {t + 12, true}});
            m = MeasureWave(edges);
            EXPECT_TRUE(m.baud != 0 && m.first_edge >= 4);
            EXPECT_TRUE(ErrorPpm(actual, m.baud) < 8000);
        }
        // Report data right after the "UU" doesn't disturb the measurement.
        edges = Uart({'U', 'U', 0xFD, 0x09, 0x00, 0x00, 0x13}, actual);
        m = MeasureWave(edges);
        EXPECT_TRUE(m.first_edge == 0 && m.bits >= 18);
        EXPECT_TRUE(ErrorPpm(actual, m.baud) < 4000);
    }
}

// Other traffic can't give a wrong rate: a run of alternating one-bit intervals in UART frames is either part of a 0x55
// or crosses a frame boundary (stop bit 1, start bit 0) at the true bit time, and runs of longer intervals can't line
// up with the frame boundaries for nine intervals. Such a run gives the right rate (the AT+BAUD_RATE? probe that
// follows a lock catches anything else), so the test allows "no lock" or "the true rate".
static bool NoneOrTrue(const std::vector<Edge>& edges, double baud) {
    Autobaud::Measurement m = MeasureWave(edges);
    return m.baud == 0 || ErrorPpm(baud, m.baud) < 4000;
}

// Other bytes, noise and out-of-range square waves never give a wrong rate.
TEST(Autobaud, WrongPatterns) {
    for (uint32_t baud : {9600u, 115200u, 1000000u}) {
        int locks = 0;
        for (std::vector<uint8_t> bytes : std::initializer_list<std::vector<uint8_t>>{
                 {'A', 'T', '\r', '\n'}, {0xAA, 0xAA, 0xAA, 0xAA}, {0x00, 0x00, 0x00}, {0xFF, 0xFF}, {0x33, 0x33, 0x33},
                 {0xCC, 0xCC, 0xCC}, {0x5A, 0xA5, 0x5A}, {0x0F, 0xF0, 0x0F}, {0x55 ^ 0x01, 0x55 ^ 0x80}}) {
            EXPECT_TRUE(NoneOrTrue(Uart(bytes, baud), baud));
            // With gaps of a fraction of a bit between the bytes too.
            std::vector<Edge> edges;
            double t0 = 0;
            for (uint8_t byte : bytes) {
                std::vector<Edge> one = Uart({byte}, baud, 0);
                for (Edge e : one) edges.push_back({e.t + t0, e.level});
                t0 += 10.37 * kClockHz / baud;
            }
            for (Edge& e : edges) e.t += 1000;
            EXPECT_TRUE(NoneOrTrue(edges, baud));
            locks += MeasureWave(Uart(bytes, baud)).baud != 0;
        }
        // At the true rate: "T\r" and 0x54 0xD5 hold nine one-bit intervals across the byte boundary.
        EXPECT_TRUE(locks <= 2);
    }
    // MAVLink-like binary at 57600, with and without gaps.
    srand(2);
    for (int trial = 0; trial < 200; trial++) {
        std::vector<uint8_t> bytes;
        for (int i = 0; i < 6; i++) bytes.push_back((uint8_t)(rand() & 0xFF));
        EXPECT_TRUE(NoneOrTrue(Uart(bytes, 57600), 57600));
    }
    // A square wave too fast or too slow for the console.
    EXPECT_EQ(MeasureWave(Uart(kUU, 4000000)).baud, 0u);
    EXPECT_EQ(MeasureWave(Uart(kUU, 5000)).baud, 0u);
    // Too few edges, or none.
    EXPECT_EQ(MeasureCounts({}).baud, 0u);
    std::vector<uint32_t> few = RunPio(Uart(kUU, 115200), 9);
    EXPECT_EQ(MeasureCounts(few).baud, 0u);
    // The wrong polarity: a run must start with a start bit.
    EXPECT_EQ(MeasureCounts(RunPio(Uart({'U'}, 115200u)), false).baud, 0u);
}

// Capture that starts on a low line.
TEST(Autobaud, LineLowAtStart) {
    // The first edge seen is a rise: a 'U' after it still measures.
    std::vector<Edge> edges = Uart(kUU, 230400);
    std::vector<uint32_t> counts = RunPio(edges);
    counts.insert(counts.begin(), counts[0] + 500);  // A rise 1000 cycles earlier.
    Autobaud::Measurement m = MeasureCounts(counts, false);
    EXPECT_EQ(m.first_edge, 1u);
    EXPECT_TRUE(ErrorPpm(230400, m.baud) < 4000);
}

// Snap to the console's rates.
TEST(Autobaud, Snap) {
    // Wake measurements at 3 M and 2 M from the bench (RC oscillator, 2-cycle sampling).
    for (uint32_t measured : {3008021u, 3016086u, 2993000u}) {
        EXPECT_EQ(Autobaud::SnapToConsoleRate(measured), 3000000u);
    }
    EXPECT_EQ(Autobaud::SnapToConsoleRate(2005348), 2000000u);
    EXPECT_EQ(Autobaud::SnapToConsoleRate(0), 0u);
    EXPECT_EQ(Autobaud::SnapToConsoleRate(3100000), 3100000u);  // No divisor that fast: unchanged.
    // Below 250 kbaud the snap moves the measurement by under 0.1%, and an actual rate maps to itself.
    for (uint32_t baud = ConsoleBaud::kMin; baud <= ConsoleBaud::kMax; baud += 1009) {
        uint32_t actual = ConsoleBaud::CC1314ActualBaud(baud);
        // Same divisor; the rates are truncated.
        EXPECT_LT(ErrorPpm(actual, Autobaud::SnapToConsoleRate(actual)), 200u);
        if (baud < 250000) {
            EXPECT_TRUE(ErrorPpm(baud, Autobaud::SnapToConsoleRate(baud)) < 1000);
        }
    }
}

