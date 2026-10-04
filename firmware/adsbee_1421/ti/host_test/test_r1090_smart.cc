// MODE_S_SMART slice policy (r1090_smart.hh).
#include "gtest/gtest.h"
#include "r1090_smart.hh"

using P = R1090SmartPolicy;

// Runs the policy like UpdateSmartSlices from t0 to t1 in 1 ms steps. Each aircraft sends a frame every period_ms
// that decodes with probability p_weak / p_strong (as a fraction of 1000) in each slice type. Returns the ms spent
// in STRONG slices.
struct Aircraft {
    uint32_t icao, period_ms, p_weak, p_strong;
};
static uint32_t RunSlices(P& p, uint32_t t0, uint32_t t1, std::initializer_list<Aircraft> aircraft = {}) {
    uint32_t strong_ms = 0, n = 0;
    for (uint32_t t = t0; t < t1; t++) {
        if (p.SliceDone(t)) p.Switched(t);
        if (p.strong()) strong_ms++;
        for (const Aircraft& a : aircraft) {
            if (t % a.period_ms != 0) continue;
            n = n * 1103515245u + 12345u;  // Deterministic pseudo-random draw.
            if ((n >> 16) % 1000 < (p.strong() ? a.p_strong : a.p_weak)) p.OnValid(a.icao, t);
        }
    }
    return strong_ms;
}

static double Share(uint32_t strong_ms_per_slice) {
    return double(strong_ms_per_slice) / (P::kWeakSliceMs + strong_ms_per_slice);
}

TEST(R1090Smart, StartsInAWeakSliceAndAlternates) {
    P p;
    p.Reset(1000);
    EXPECT_FALSE(p.strong());
    EXPECT_FALSE(p.SliceDone(1000 + P::kWeakSliceMs - 1));
    EXPECT_TRUE(p.SliceDone(1000 + P::kWeakSliceMs));
    EXPECT_EQ(p.Switched(1000 + P::kWeakSliceMs), P::kWeakSliceMs);
    EXPECT_TRUE(p.strong());
    EXPECT_EQ(p.SliceMs(1100), P::kStrongSliceMinMs);
    EXPECT_TRUE(p.SliceDone(1100 + P::kStrongSliceMinMs));
    p.Switched(1100 + P::kStrongSliceMinMs);
    EXPECT_FALSE(p.strong());
}

TEST(R1090Smart, QuietSplitIsMostlyWeak) {
    P p;
    p.Reset(0);
    EXPECT_NEAR(RunSlices(p, 0, 100000) / 100000.0, Share(P::kStrongSliceMinMs), 0.01);
}

TEST(R1090Smart, WeakAircraftKeepShortStrongSlices) {
    P p;
    p.Reset(0);
    EXPECT_NEAR(RunSlices(p, 0, 100000, {{0xADF001, 150, 950, 0}}) / 100000.0, Share(P::kStrongSliceMinMs), 0.01);
}

TEST(R1090Smart, AircraftHeardInBothSlicesKeepsShortStrongSlices) {
    // An aircraft both settings decode (the overlap of their level ranges) is not a strong aircraft.
    P p;
    p.Reset(0);
    EXPECT_NEAR(RunSlices(p, 0, 100000, {{0xADF002, 150, 950, 950}}) / 100000.0, Share(P::kStrongSliceMinMs), 0.02);
}

TEST(R1090Smart, StrongAircraftLengthensStrongSlices) {
    // MODE_S still decodes a few frames of a strong aircraft; it is a strong aircraft all the same.
    P p;
    p.Reset(0);
    const uint32_t strong_ms = RunSlices(p, 0, 100000, {{0xADF003, 150, 180, 990}, {0xADF004, 150, 950, 0}});
    // Mostly long STRONG slices; the hold can lapse briefly while few frames have been counted.
    EXPECT_GT(strong_ms / 100000.0, 0.85 * Share(P::kStrongSliceMaxMs));
    EXPECT_LT(strong_ms / 100000.0, Share(P::kStrongSliceMaxMs) + 0.01);
}

TEST(R1090Smart, HoldEndsWhenTheStrongAircraftLeaves) {
    P p;
    p.Reset(0);
    RunSlices(p, 0, 20000, {{0xADF005, 150, 0, 990}});
    EXPECT_TRUE(p.StrongHold(20000));
    RunSlices(p, 20000, 20000 + P::kStrongHoldMs + 200);
    EXPECT_FALSE(p.StrongHold(20000 + P::kStrongHoldMs + 200));
    EXPECT_EQ(p.SliceMs(20000 + P::kStrongHoldMs + 200), p.strong() ? P::kStrongSliceMinMs : P::kWeakSliceMs);
}

TEST(R1090Smart, DecodesRightAfterASwitchDoNotCount) {
    P p;
    p.Reset(0);
    RunSlices(p, 0, P::kWeakSliceMs + 1);
    ASSERT_TRUE(p.strong());
    EXPECT_FALSE(p.OnValid(0xADF006, P::kWeakSliceMs + P::kSettleMs - 1));
    EXPECT_TRUE(p.OnValid(0xADF006, P::kWeakSliceMs + P::kSettleMs));  // Never heard in MODE_S slices.
}

TEST(R1090Smart, TableKeepsTheMostRecentAircraft) {
    P p;
    p.Reset(0);
    for (uint32_t i = 0; i < P::kNumAircraft + 8; i++) {
        for (uint32_t k = 0; k < 3; k++) p.OnValid(0xADF000 + i, 10 + i);  // All heard in the first MODE_S slice.
    }
    RunSlices(p, 10, P::kWeakSliceMs + 5);
    ASSERT_TRUE(p.strong());
    const uint32_t t = P::kWeakSliceMs + 5;
    EXPECT_FALSE(p.OnValid(0xADF000 + P::kNumAircraft + 7, t));  // Newest: remembered as heard in MODE_S.
    EXPECT_TRUE(p.OnValid(0xADF000, t));                         // Oldest: evicted, so it looks new.
}

TEST(R1090Smart, ResetEndsTheHold) {
    P p;
    p.Reset(0);
    RunSlices(p, 0, P::kWeakSliceMs + 5);
    EXPECT_TRUE(p.OnValid(0xADF007, P::kWeakSliceMs + 5));
    p.Reset(200);
    EXPECT_FALSE(p.strong());
    EXPECT_FALSE(p.StrongHold(201));
}

TEST(R1090Smart, BurstyAircraftHeardInBothSlicesKeepsShortStrongSlices) {
    // Eight aircraft both settings decode, in 2.5 s bursts with 10 s gaps (few frames per aircraft per decay period).
    P p;
    p.Reset(0);
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
    EXPECT_NEAR(strong_ms / double(t), Share(P::kStrongSliceMinMs), 0.03);
}

TEST(R1090Smart, StrongAircraftStartsTheHoldWithinSeconds) {
    P p;
    p.Reset(0);
    const std::initializer_list<Aircraft> traffic = {
        {0xADF020, 151, 180, 990}, {0xADF021, 173, 180, 990}, {0xADF030, 157, 980, 0}, {0xADF031, 181, 980, 0}};
    RunSlices(p, 0, 8000, traffic);
    EXPECT_TRUE(p.StrongHold(8000));
    RunSlices(p, 8000, 30000, traffic);
    EXPECT_TRUE(p.StrongHold(30000));
    RunSlices(p, 30000, 30000 + P::kStrongHoldMs + 200);  // The strong aircraft leave.
    EXPECT_FALSE(p.StrongHold(30000 + P::kStrongHoldMs + 200));
}
