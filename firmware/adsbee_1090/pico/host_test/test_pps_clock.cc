#include <cmath>
#include <cstdint>
#include <random>

#include "gtest/gtest.h"
#include "pps_clock.hh"

namespace {

constexpr uint64_t kUnixStart = 1795000000;  // A whole UTC second (2026-11-18).
// The bench rounds to whole 48 MHz counts (21 ns), so conversions are checked to 11 ns once the fit has settled, and to
// 40 ns a few points in (the rate from the first two edges is only good to a count, 21 ppb).
constexpr double kSettledNs = 11, kEarlyNs = 40;

// GNSS time against an RP2040-like 48 MHz MLAT counter whose crystal runs rate_error_ppm fast. t is GNSS seconds since
// kUnixStart.
struct Bench {
    double rate_error_ppm = 23.456;
    uint64_t mlat_start = 0x123456789ULL;
    uint64_t Mlat(double t) const {
        return mlat_start + static_cast<uint64_t>(std::llround(t * 48e6 * (1 + rate_error_ppm * 1e-6)));
    }
    // Edge of second s, stamped late by delay_ns.
    PPSClock::Edge Edge(uint64_t s, double delay_ns = 0) const {
        return {.mlat_48mhz_counts = Mlat(s + delay_ns * 1e-9),
                .time_us = static_cast<uint32_t>(0xFFF00000u + s * 1'000'000)};  // time_us wraps on the way.
    }
};

// Labels the edge of second s with an RMC 200 ms later, as a receiver would.
PPSClock::Result Label(PPSClock& clock, const Bench& bench, uint32_t& num_edges, uint64_t s, double delay_ns = 0) {
    PPSClock::Edge latest = bench.Edge(s, delay_ns), previous = bench.Edge(s - 1);
    num_edges++;
    return clock.OnRMC(true, false, kUnixStart + s, num_edges, latest, previous, latest.time_us + 200'000);
}

// Converts the MLAT count of GNSS time t and returns the error in ns.
double ErrorNs(const PPSClock& clock, const Bench& bench, double t) {
    uint64_t unix_seconds = 0;
    uint32_t ns = 0;
    EXPECT_TRUE(clock.ToUTC(bench.Mlat(t), unix_seconds, ns));
    return (static_cast<double>(unix_seconds - kUnixStart) - std::floor(t)) * 1e9 + ns - (t - std::floor(t)) * 1e9;
}

}  // namespace

TEST(PPSClock, NeedsTwoPoints) {
    Bench bench;
    PPSClock clock;
    uint32_t num_edges = 0;
    uint64_t unix_seconds;
    uint32_t ns;
    EXPECT_FALSE(clock.ToUTC(bench.Mlat(0.5), unix_seconds, ns));
    EXPECT_EQ(Label(clock, bench, num_edges, 1), PPSClock::kRestarted);  // First point of a fit.
    EXPECT_FALSE(clock.IsSynced());
    EXPECT_FALSE(clock.ToUTC(bench.Mlat(1.5), unix_seconds, ns));
    EXPECT_EQ(Label(clock, bench, num_edges, 2), PPSClock::kLabeled);
    EXPECT_TRUE(clock.IsSynced());
    EXPECT_NEAR(clock.GetRateErrorPPB(), 23456, 25);  // A count is 21 ppb of a second.
}

TEST(PPSClock, ConvertsToTheNanosecond) {
    Bench bench;
    PPSClock clock;
    uint32_t num_edges = 0;
    // The rate from the first two edges is only good to a count (21 ppb); the filter refines it.
    for (uint64_t s = 1; s <= 60; s++) Label(clock, bench, num_edges, s);
    // Through the holdover, and before the last point (a timestamp taken before its PPS, converted after it).
    for (double t = 60 - 0.999; t < 60 + PPSClock::kHoldoverSeconds; t += 0.0371) {
        EXPECT_NEAR(ErrorNs(clock, bench, t), 0, kSettledNs) << t;
    }
    uint64_t unix_seconds;
    uint32_t ns;
    ASSERT_TRUE(clock.ToUTC(bench.Mlat(62.25), unix_seconds, ns));
    EXPECT_EQ(unix_seconds, kUnixStart + 62);
    EXPECT_NEAR(ns, 250'000'000, kSettledNs);
    // Past the holdover: no time.
    EXPECT_FALSE(clock.ToUTC(bench.Mlat(60 + PPSClock::kHoldoverSeconds + 0.01), unix_seconds, ns));
    EXPECT_FALSE(clock.ToUTC(bench.Mlat(60 - PPSClock::kHoldoverSeconds - 0.01), unix_seconds, ns));
}

TEST(PPSClock, Fresh) {
    Bench bench;
    PPSClock clock;
    uint32_t num_edges = 0;
    Label(clock, bench, num_edges, 1);
    EXPECT_FALSE(clock.IsFresh(bench.Edge(1).time_us + 1000));  // One point.
    Label(clock, bench, num_edges, 2);
    EXPECT_TRUE(clock.IsFresh(bench.Edge(2).time_us + 1000));
    EXPECT_TRUE(clock.IsFresh(bench.Edge(5).time_us + 999'000));
    EXPECT_FALSE(clock.IsFresh(bench.Edge(6).time_us));
}

TEST(PPSClock, MissingEdges) {
    Bench bench;
    PPSClock clock;
    uint32_t num_edges = 0;
    Label(clock, bench, num_edges, 1);
    Label(clock, bench, num_edges, 2);
    // The edges of seconds 3 and 4 never came: the RMCs of those seconds find no new edge.
    PPSClock::Edge latest = bench.Edge(2), previous = bench.Edge(1);
    EXPECT_EQ(clock.OnRMC(true, false, kUnixStart + 3, num_edges, latest, previous, bench.Edge(3).time_us + 200'000),
              PPSClock::kNoEdge);
    EXPECT_EQ(clock.OnRMC(true, false, kUnixStart + 4, num_edges, latest, previous, bench.Edge(4).time_us + 200'000),
              PPSClock::kNoEdge);
    // The rate is measured over the 3 s gap.
    EXPECT_EQ(Label(clock, bench, num_edges, 5), PPSClock::kLabeled);
    EXPECT_NEAR(clock.GetRateErrorPPB(), 23456, 25);  // A count is 21 ppb of a second.
    EXPECT_NEAR(ErrorNs(clock, bench, 5.5), 0, kEarlyNs);
    // Beyond the holdover counts don't convert; a gap longer than kMaxGapSeconds starts again.
    uint64_t unix_seconds;
    uint32_t ns;
    EXPECT_FALSE(clock.ToUTC(bench.Mlat(5 + PPSClock::kHoldoverSeconds + 0.5), unix_seconds, ns));
    EXPECT_EQ(Label(clock, bench, num_edges, 5 + PPSClock::kMaxGapSeconds + 1), PPSClock::kRestarted);
    EXPECT_FALSE(clock.IsSynced());
}

TEST(PPSClock, NoFixLateAndExtraEdges) {
    Bench bench;
    PPSClock clock;
    uint32_t num_edges = 0;
    Label(clock, bench, num_edges, 1);
    Label(clock, bench, num_edges, 2);
    PPSClock::Edge latest = bench.Edge(3), previous = bench.Edge(2);
    // No fix: the edge isn't labeled, the fit holds over.
    EXPECT_EQ(clock.OnRMC(false, false, 0, ++num_edges, latest, previous, latest.time_us + 200'000),
              PPSClock::kNoFix);
    EXPECT_TRUE(clock.IsSynced());
    // RMC late: its edge is more than kMaxLabelDelayUs old (the next one may be due).
    latest = bench.Edge(4), previous = bench.Edge(3);
    EXPECT_EQ(clock.OnRMC(true, false, kUnixStart + 4, ++num_edges, latest, previous,
                          latest.time_us + PPSClock::kMaxLabelDelayUs + 1),
              PPSClock::kLate);
    // An extra edge (a glitch) 300 ms after the edge of second 5: which one is the second is unknown.
    previous = bench.Edge(5);
    latest = bench.Edge(5);
    latest.mlat_48mhz_counts += 14'400'000;
    latest.time_us += 300'000;
    num_edges += 2;
    EXPECT_EQ(clock.OnRMC(true, false, kUnixStart + 5, num_edges, latest, previous, latest.time_us + 100'000),
              PPSClock::kExtraEdge);
    // The next second labels normally: the fit held over.
    EXPECT_EQ(Label(clock, bench, num_edges, 6), PPSClock::kLabeled);
    EXPECT_NEAR(ErrorNs(clock, bench, 6.5), 0, kEarlyNs);
    EXPECT_EQ(clock.GetResultCount(PPSClock::kNoFix), 1u);
    EXPECT_EQ(clock.GetResultCount(PPSClock::kLate), 1u);
    EXPECT_EQ(clock.GetResultCount(PPSClock::kExtraEdge), 1u);
}

TEST(PPSClock, LeapSecondStartsAgain) {
    Bench bench;
    PPSClock clock;
    uint32_t num_edges = 0;
    Label(clock, bench, num_edges, 1);
    Label(clock, bench, num_edges, 2);
    PPSClock::Edge latest = bench.Edge(3), previous = bench.Edge(2);
    EXPECT_EQ(clock.OnRMC(false, true, 0, ++num_edges, latest, previous, latest.time_us + 200'000),
              PPSClock::kLeapSecond);
    EXPECT_FALSE(clock.IsSynced());
    // After 23:59:60 the POSIX second repeats: the edge of GNSS second 4 is labeled 3 again.
    latest = bench.Edge(4), previous = bench.Edge(3);
    EXPECT_EQ(clock.OnRMC(true, false, kUnixStart + 3, ++num_edges, latest, previous, latest.time_us + 200'000),
              PPSClock::kRestarted);
    latest = bench.Edge(5), previous = bench.Edge(4);
    EXPECT_EQ(clock.OnRMC(true, false, kUnixStart + 4, ++num_edges, latest, previous, latest.time_us + 200'000),
              PPSClock::kLabeled);
    uint64_t unix_seconds;
    uint32_t ns;
    ASSERT_TRUE(clock.ToUTC(bench.Mlat(5.5), unix_seconds, ns));
    EXPECT_EQ(unix_seconds, kUnixStart + 4);
    EXPECT_NEAR(ns, 500'000'000, kEarlyNs);
}

TEST(PPSClock, DropsDelayedEdges) {
    Bench bench;
    PPSClock clock;
    uint32_t num_edges = 0;
    for (uint64_t s = 1; s <= 5; s++) Label(clock, bench, num_edges, s);
    // An edge whose interrupt waited 5 us for another one is dropped; the fit holds over.
    EXPECT_EQ(Label(clock, bench, num_edges, 6, 5000), PPSClock::kOutlier);
    EXPECT_NEAR(ErrorNs(clock, bench, 6.5), 0, kEarlyNs);
    EXPECT_EQ(Label(clock, bench, num_edges, 7), PPSClock::kLabeled);
    // Within kMaxResidualNs an edge is used, and its residual reported.
    EXPECT_EQ(Label(clock, bench, num_edges, 8, 300), PPSClock::kLabeled);
    EXPECT_NEAR(clock.GetLastResidualNs(), 300, kEarlyNs);
    EXPECT_EQ(clock.GetResultCount(PPSClock::kOutlier), 1u);
}

TEST(PPSClock, ReanchorsOnEarlyEdges) {
    Bench bench;
    PPSClock clock;
    uint32_t num_edges = 0;
    // The fit starts while every edge is stamped 1.6 us late (the receiver had no traffic).
    for (uint64_t s = 1; s <= 5; s++) Label(clock, bench, num_edges, s, 1600);
    EXPECT_NEAR(ErrorNs(clock, bench, 5.5), -1600, kEarlyNs);
    // The first edge on time moves the anchor to it; the fit stays synced. The rate takes a sixteenth of the shift
    // (100 ns/s), which the next edges take out again.
    EXPECT_EQ(Label(clock, bench, num_edges, 6), PPSClock::kReanchored);
    EXPECT_TRUE(clock.IsSynced());
    EXPECT_NEAR(ErrorNs(clock, bench, 6.5), 0, 60);
    for (uint64_t s = 7; s <= 40; s++) EXPECT_EQ(Label(clock, bench, num_edges, s), PPSClock::kLabeled) << s;
    EXPECT_NEAR(ErrorNs(clock, bench, 40.5), 0, kEarlyNs);
    // Late edges are dropped again.
    EXPECT_EQ(Label(clock, bench, num_edges, 41, 1600), PPSClock::kOutlier);
    EXPECT_EQ(Label(clock, bench, num_edges, 42), PPSClock::kLabeled);
    EXPECT_NEAR(ErrorNs(clock, bench, 42.5), 0, kEarlyNs);
    // Until kMaxOutliers of them in a row: the anchor moves to the last one, and the fit stays synced.
    uint64_t s = 43;
    for (; s < 43 + PPSClock::kMaxOutliers - 1; s++) {
        EXPECT_EQ(Label(clock, bench, num_edges, s, 1600), PPSClock::kOutlier);
    }
    EXPECT_EQ(Label(clock, bench, num_edges, s, 1600), PPSClock::kReanchored);
    EXPECT_TRUE(clock.IsSynced());
    EXPECT_NEAR(ErrorNs(clock, bench, s + 0.5), -1600, 60);
}

TEST(PPSClock, StartsAgainAfterOutliersInARow) {
    Bench bench;
    PPSClock clock;
    uint32_t num_edges = 0;
    for (uint64_t s = 1; s <= 5; s++) Label(clock, bench, num_edges, s);
    // The RMCs start labeling each edge one second off (e.g. they now arrive after the next edge): the fit doesn't
    // follow the first two, then starts again from the third.
    for (uint64_t s = 6; s < 6 + PPSClock::kMaxOutliers - 1; s++) {
        PPSClock::Edge latest = bench.Edge(s + 1), previous = bench.Edge(s);
        EXPECT_EQ(clock.OnRMC(true, false, kUnixStart + s, ++num_edges, latest, previous, latest.time_us + 50'000),
                  PPSClock::kOutlier);
    }
    uint64_t s = 6 + PPSClock::kMaxOutliers - 1;
    PPSClock::Edge latest = bench.Edge(s + 1), previous = bench.Edge(s);
    EXPECT_EQ(clock.OnRMC(true, false, kUnixStart + s, ++num_edges, latest, previous, latest.time_us + 50'000),
              PPSClock::kRestarted);
    EXPECT_FALSE(clock.IsSynced());
}

TEST(PPSClock, FollowsJitterAndDrift) {
    // 20 ns rms capture jitter, and a crystal whose rate moves by 0.002 ppm a second (a fast temperature change).
    std::mt19937 rng(1307);
    std::normal_distribution<double> jitter_ns(0, 20);
    PPSClock clock;
    uint32_t num_edges = 0;
    double rate_ppm = -12.0, t_mlat = 0;  // MLAT seconds at GNSS second s.
    uint64_t mlat_start = 1ULL << 40;
    double worst_ns = 0;
    for (uint64_t s = 1; s <= 600; s++) {
        rate_ppm += 0.002 * (s < 300 ? 1 : -1);  // Up for 5 minutes, then down.
        t_mlat += 1 + rate_ppm * 1e-6;
        PPSClock::Edge latest = {
            .mlat_48mhz_counts = mlat_start + static_cast<uint64_t>(std::llround((t_mlat + jitter_ns(rng) * 1e-9) * 48e6)),
            .time_us = static_cast<uint32_t>(s * 1'000'000)};
        PPSClock::Edge previous = {.time_us = static_cast<uint32_t>((s - 1) * 1'000'000)};
        PPSClock::Result result =
            clock.OnRMC(true, false, kUnixStart + s, ++num_edges, latest, previous, latest.time_us + 100'000);
        if (s > 10) {
            EXPECT_EQ(result, PPSClock::kLabeled) << s;
            // Half a second after the edge, against the true time (the next second's rate).
            uint64_t mlat_half = mlat_start + static_cast<uint64_t>(
                                                  std::llround((t_mlat + 0.5 * (1 + rate_ppm * 1e-6)) * 48e6));
            uint64_t unix_seconds;
            uint32_t ns;
            ASSERT_TRUE(clock.ToUTC(mlat_half, unix_seconds, ns));
            double error_ns = (static_cast<double>(unix_seconds - kUnixStart - s)) * 1e9 + ns - 0.5e9;
            worst_ns = std::max(worst_ns, std::fabs(error_ns));
        }
    }
    EXPECT_LT(worst_ns, 150);
    EXPECT_NEAR(std::sqrt(static_cast<double>(clock.GetMeanSquareResidualNs2())), 30, 25);
}
