// MODE_S_SMART slice policy (r1090_smart.hh).
#include "gtest/gtest.h"
#include "r1090_smart.hh"

using P = R1090SmartPolicy;

// Runs the policy like UpdateSmartSlices from t0 to t1 in 1 ms steps; returns the ms spent in STRONG slices.
static uint32_t RunSlices(P& p, uint32_t t0, uint32_t t1) {
    uint32_t strong_ms = 0;
    for (uint32_t t = t0; t < t1; t++) {
        if (p.SliceDone(t)) p.Switched(t);
        if (p.strong()) strong_ms++;
    }
    return strong_ms;
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
    const uint32_t strong_ms = RunSlices(p, 0, 100000);
    EXPECT_NEAR(strong_ms / 100000.0, double(P::kStrongSliceMinMs) / (P::kWeakSliceMs + P::kStrongSliceMinMs), 0.01);
}

TEST(R1090Smart, StrongOnlyAircraftLengthensStrongSlices) {
    P p;
    p.Reset(0);
    RunSlices(p, 0, P::kWeakSliceMs + 5);  // Into the first STRONG slice, past the settle time.
    ASSERT_TRUE(p.strong());
    EXPECT_TRUE(p.OnValid(0xADF001, P::kWeakSliceMs + 5));
    EXPECT_TRUE(p.StrongHold(P::kWeakSliceMs + 5));
    EXPECT_EQ(p.SliceMs(P::kWeakSliceMs + 5), P::kStrongSliceMaxMs);
    // Held for kStrongHoldMs, then back to short slices.
    const uint32_t t = P::kWeakSliceMs + 5;
    EXPECT_TRUE(p.StrongHold(t + P::kStrongHoldMs - 1));
    EXPECT_FALSE(p.StrongHold(t + P::kStrongHoldMs));
    const uint32_t held = RunSlices(p, t, t + P::kStrongHoldMs);
    EXPECT_NEAR(held / double(P::kStrongHoldMs),
                double(P::kStrongSliceMaxMs) / (P::kWeakSliceMs + P::kStrongSliceMaxMs), 0.05);
}

TEST(R1090Smart, AircraftHeardInWeakSlicesDoesNotCount) {
    P p;
    p.Reset(0);
    EXPECT_FALSE(p.OnValid(0xADF002, 50));  // MODE_S slice: remembered.
    RunSlices(p, 0, P::kWeakSliceMs + 5);
    ASSERT_TRUE(p.strong());
    EXPECT_FALSE(p.OnValid(0xADF002, P::kWeakSliceMs + 5));
    EXPECT_FALSE(p.StrongHold(P::kWeakSliceMs + 5));
    // Once it hasn't been heard in a MODE_S slice for kWeakSeenMs, it counts.
    EXPECT_TRUE(p.OnValid(0xADF002, 51 + P::kWeakSeenMs));
}

TEST(R1090Smart, DecodesRightAfterASwitchDoNotCount) {
    P p;
    p.Reset(0);
    RunSlices(p, 0, P::kWeakSliceMs + 1);
    ASSERT_TRUE(p.strong());
    EXPECT_FALSE(p.OnValid(0xADF003, P::kWeakSliceMs + P::kSettleMs - 1));
    EXPECT_TRUE(p.OnValid(0xADF003, P::kWeakSliceMs + P::kSettleMs));
}

TEST(R1090Smart, WeakTableKeepsTheMostRecentAircraft) {
    P p;
    p.Reset(0);
    for (uint32_t i = 0; i < P::kNumWeakIcaos + 8; i++) {
        p.OnValid(0xADF000 + i, 10 + i);  // All in the first MODE_S slice.
    }
    RunSlices(p, 10, P::kWeakSliceMs + 5);
    ASSERT_TRUE(p.strong());
    const uint32_t t = P::kWeakSliceMs + 5;
    EXPECT_FALSE(p.OnValid(0xADF000 + P::kNumWeakIcaos + 7, t));  // Newest: remembered.
    EXPECT_TRUE(p.OnValid(0xADF000, t));                          // Oldest: evicted.
}

TEST(R1090Smart, ResetEndsTheHold) {
    P p;
    p.Reset(0);
    RunSlices(p, 0, P::kWeakSliceMs + 5);
    p.OnValid(0xADF004, P::kWeakSliceMs + 5);
    p.Reset(200);
    EXPECT_FALSE(p.strong());
    EXPECT_FALSE(p.StrongHold(201));
}
