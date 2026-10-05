// MODE_S_SMART slice policy (r1090_smart.hh).
#include "gtest/gtest.h"
#include "r1090_smart.hh"

using P = R1090SmartPolicy;

// Runs the policy like UpdateSmartSlices from t0 to t1 in 1 ms steps. Each aircraft sends a frame about every
// period_ms (jittered, like real squitters) that decodes with probability p_weak / p_strong (as a fraction of 1000) in
// each slice type. Returns the ms spent in STRONG slices.
struct Aircraft {
    uint32_t icao, period_ms, p_weak, p_strong;
};
static uint32_t Rand() {
    static uint32_t n = 1;
    n = n * 1103515245u + 12345u;  // Deterministic pseudo-random draw.
    return n >> 16;
}
static uint32_t RunSlices(P& p, uint32_t t0, uint32_t t1, std::initializer_list<Aircraft> aircraft = {}) {
    uint32_t strong_ms = 0;
    uint32_t next[16] = {};
    for (uint32_t t = t0; t < t1; t++) {
        if (p.SliceDone(t)) p.Switched(t, t * 1000);
        if (p.strong()) strong_ms++;
        uint16_t i = 0;
        for (const Aircraft& a : aircraft) {
            uint32_t& due = next[i++];
            if (due == 0) due = t + Rand() % a.period_ms;
            if (t < due) continue;
            due = t + a.period_ms * 3 / 4 + Rand() % (a.period_ms / 2);
            if (Rand() % 1000 < (p.strong() ? a.p_strong : a.p_weak)) p.OnValid(a.icao, t * 1000 + 500, t);
        }
    }
    return strong_ms;
}

static double Share(uint32_t strong_pct) { return strong_pct / 100.0; }

TEST(R1090Smart, StartsInAWeakSliceAndAlternates) {
    P p;
    p.Reset(1000, 1000000);
    EXPECT_FALSE(p.strong());
    const uint32_t weak_ms = P::kCycleMs * (100 - P::kProbeStrongPct) / 100;
    EXPECT_FALSE(p.SliceDone(1000 + weak_ms - P::kCycleMs / 10 - 1));
    EXPECT_TRUE(p.SliceDone(1000 + weak_ms + P::kCycleMs / 10));
    uint32_t t = 1000;
    while (!p.SliceDone(t)) t++;
    EXPECT_EQ(p.Switched(t, t * 1000), t - 1000);
    EXPECT_TRUE(p.strong());
    const uint32_t strong_ms = P::kCycleMs * P::kProbeStrongPct / 100;
    EXPECT_FALSE(p.SliceDone(t + strong_ms - 1));
    EXPECT_TRUE(p.SliceDone(t + strong_ms));
    p.Switched(t + strong_ms, (t + strong_ms) * 1000);
    EXPECT_FALSE(p.strong());
}

TEST(R1090Smart, LongSlicesVaryInLength) {
    // Short STRONG slices at a fixed period could keep missing periodic traffic.
    P p;
    p.Reset(0, 0);
    uint32_t t = 0, n = 0, sum = 0, lo = 1000, hi = 0;
    for (int i = 0; i < 400; i++) {
        while (!p.SliceDone(t)) t++;
        const uint32_t len = p.Switched(t, t * 1000);
        if (!p.strong()) continue;  // A MODE_S slice just ended.
        n++;
        sum += len;
        if (len < lo) lo = len;
        if (len > hi) hi = len;
    }
    const uint32_t weak_ms = P::kCycleMs * (100 - P::kProbeStrongPct) / 100;
    EXPECT_NEAR(double(sum) / n, weak_ms, 1.5);
    EXPECT_LT(lo, weak_ms - P::kCycleMs / 20);
    EXPECT_GT(hi, weak_ms + P::kCycleMs / 20);
}

TEST(R1090Smart, QuietSplitProbesStrong) {
    P p;
    p.Reset(0, 0);
    EXPECT_NEAR(RunSlices(p, 0, 100000) / 100000.0, Share(P::kProbeStrongPct), 0.005);
}

TEST(R1090Smart, WeakAircraftKeepTheProbeSplit) {
    P p;
    p.Reset(0, 0);
    EXPECT_NEAR(RunSlices(p, 0, 100000, {{0xADF001, 150, 950, 0}}) / 100000.0, Share(P::kProbeStrongPct), 0.005);
}

TEST(R1090Smart, AircraftHeardInBothSlicesKeepsTheProbeSplit) {
    // An aircraft both settings decode (the overlap of their level ranges) needs neither.
    P p;
    p.Reset(0, 0);
    RunSlices(p, 0, 10000, {{0xADF002, 150, 950, 950}});
    EXPECT_NEAR(RunSlices(p, 10000, 100000, {{0xADF002, 150, 950, 950}}) / 90000.0, Share(P::kProbeStrongPct), 0.02);
}

TEST(R1090Smart, OnlyStrongAircraftProbeWeak) {
    // MODE_S still decodes a few frames of a strong aircraft; it needs STRONG all the same.
    P p;
    p.Reset(0, 0);
    RunSlices(p, 0, 20000, {{0xADF003, 149, 180, 990}});
    EXPECT_EQ(p.strong_pct(), 100 - P::kProbeWeakPct);
    EXPECT_NEAR(RunSlices(p, 20000, 100000, {{0xADF003, 149, 180, 990}}) / 80000.0, Share(100 - P::kProbeWeakPct),
                0.01);
}

TEST(R1090Smart, MixedSplitFollowsTheAircraftCounts) {
    P p;
    p.Reset(0, 0);
    RunSlices(p, 0, 40000, {{0xADF004, 149, 180, 990}, {0xADF005, 141, 950, 0}});
    EXPECT_EQ(p.strong_pct(), 50u);
    p.Reset(0, 0);
    RunSlices(p, 0, 40000,
              {{0xADF004, 149, 180, 990}, {0xADF005, 141, 950, 0}, {0xADF006, 131, 950, 0}, {0xADF007, 163, 950, 0}});
    EXPECT_EQ(p.strong_pct(), 25u);
    p.Reset(0, 0);
    RunSlices(p, 0, 40000,
              {{0xADF004, 149, 180, 990},
               {0xADF005, 141, 950, 0},
               {0xADF006, 131, 950, 0},
               {0xADF007, 163, 950, 0},
               {0xADF008, 173, 950, 0},
               {0xADF009, 113, 950, 0}});
    EXPECT_EQ(p.strong_pct(), P::kMixedMinPct);
}

TEST(R1090Smart, StrongAircraftIsFoundWithinSecondsAndForgottenAfterItLeaves) {
    P p;
    p.Reset(0, 0);
    const std::initializer_list<Aircraft> traffic = {
        {0xADF020, 151, 180, 990}, {0xADF021, 173, 180, 990}, {0xADF030, 157, 980, 0}, {0xADF031, 181, 980, 0}};
    RunSlices(p, 0, 15000, traffic);
    EXPECT_EQ(p.strong_pct(), 50u);
    RunSlices(p, 15000, 30000, traffic);
    EXPECT_EQ(p.strong_pct(), 50u);
    RunSlices(p, 30000, 30000 + P::kHoldMs + 200,
              {{0xADF030, 157, 980, 0}, {0xADF031, 181, 980, 0}});  // The strong aircraft leave.
    EXPECT_EQ(p.strong_pct(), P::kProbeStrongPct);
}

TEST(R1090Smart, DelayedDecodesCountInTheSliceTheyWereCapturedIn) {
    // A weak aircraft's frames are decoded 2 ms after capture, some of them inside the next STRONG slice: they still
    // count as MODE_S frames, so no check ever adds STRONG time.
    P p;
    p.Reset(0, 0);
    uint32_t strong_ms = 0, pending_us = 0, due_ms = 0;
    for (uint32_t t = 0; t < 60000; t++) {
        if (p.SliceDone(t)) p.Switched(t, t * 1000);
        if (p.strong()) strong_ms++;
        if (due_ms != 0 && t >= due_ms) {
            p.OnValid(0xADF050, pending_us, t);
            due_ms = 0;
        }
        if (t % 7 == 0 && !p.strong() && due_ms == 0) {
            pending_us = t * 1000 + 600;
            due_ms = t + 2;
        }
    }
    EXPECT_NEAR(strong_ms / 60000.0, Share(P::kProbeStrongPct), 0.002);
}

TEST(R1090Smart, FramesRightAfterASwitchDoNotCount) {
    P p;
    p.Reset(0, 0);
    uint32_t t = 0;
    while (!p.SliceDone(t)) t++;
    p.Switched(t, t * 1000);
    ASSERT_TRUE(p.strong());
    p.OnValid(0xADF060, t * 1000 + P::kSettleUs - 100, t + 1);
    EXPECT_FALSE(p.Tracked(0xADF060));
    p.OnValid(0xADF061, t * 1000 + P::kSettleUs, t + 1);
    EXPECT_TRUE(p.Tracked(0xADF061));
}

TEST(R1090Smart, TableKeepsTheMostRecentAircraft) {
    P p;
    p.Reset(0, 0);
    for (uint32_t i = 0; i < P::kNumAircraft + 8; i++) p.OnValid(0xADF000 + i, (10 + i) * 1000, 10 + i);
    EXPECT_FALSE(p.Tracked(0xADF000));
    EXPECT_FALSE(p.Tracked(0xADF007));
    EXPECT_TRUE(p.Tracked(0xADF008));
    EXPECT_TRUE(p.Tracked(0xADF000 + P::kNumAircraft + 7));
}

TEST(R1090Smart, ResetForgetsTheAircraft) {
    P p;
    p.Reset(0, 0);
    RunSlices(p, 0, 20000, {{0xADF007, 149, 0, 990}});
    EXPECT_EQ(p.strong_pct(), 100 - P::kProbeWeakPct);
    p.Reset(20000, 20000000);
    EXPECT_FALSE(p.strong());
    EXPECT_EQ(p.strong_pct(), P::kProbeStrongPct);
}

TEST(R1090Smart, BurstyAircraftHeardInBothSlicesKeepTheProbeSplit) {
    // Eight aircraft both settings decode, in 2.5 s bursts with 10 s gaps (few frames per aircraft per decay period).
    P p;
    p.Reset(0, 0);
    uint32_t strong_ms = 0, t = 0;
    for (int burst = 0; burst < 8; burst++) {
        strong_ms += RunSlices(p, t, t + 2500,
                               {{0xADF010, 80, 980, 990},
                                {0xADF011, 83, 980, 990},
                                {0xADF012, 89, 980, 990},
                                {0xADF013, 97, 980, 990},
                                {0xADF014, 101, 980, 990},
                                {0xADF015, 107, 980, 990},
                                {0xADF016, 113, 980, 990},
                                {0xADF017, 127, 980, 990}});
        strong_ms += RunSlices(p, t + 2500, t + 12500);
        t += 12500;
    }
    // Lucky frames in short STRONG slices start checks, which add some STRONG time but never a long split.
    EXPECT_NEAR(strong_ms / double(t), Share(P::kProbeStrongPct), 0.05);
}
