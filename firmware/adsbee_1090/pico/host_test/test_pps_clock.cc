#include "gtest/gtest.h"
#include "pps_clock.hh"

TEST(PPSClock, AnchorsToLabeledEdge) {
    PPSClock clock;
    clock.Label(1000000, 1085000, 1700000005000);
    uint64_t utc_ms = 0;
    ASSERT_TRUE(clock.GetUTCUnixMs(1250000, utc_ms));
    EXPECT_EQ(utc_ms, 1700000005250u);
}

TEST(PPSClock, RejectsLateLabel) {
    PPSClock clock;
    clock.Label(1000000, 1000000 + PPSClock::kMaxLabelDelayUs, 1700000005000);
    uint64_t utc_ms = 42;
    EXPECT_FALSE(clock.GetUTCUnixMs(1600000, utc_ms));
    EXPECT_EQ(utc_ms, 42u);
}

TEST(PPSClock, RejectsFractionalSecond) {
    PPSClock clock;
    clock.Label(1000000, 1085000, 1700000005200);
    uint64_t utc_ms;
    EXPECT_FALSE(clock.GetUTCUnixMs(1100000, utc_ms));
}

TEST(PPSClock, ExpiresAfterHoldover) {
    PPSClock clock;
    clock.Label(1000000, 1085000, 1700000005000);
    uint64_t utc_ms;
    EXPECT_TRUE(clock.GetUTCUnixMs(1000000 + PPSClock::kHoldoverUs - 1, utc_ms));
    EXPECT_FALSE(clock.GetUTCUnixMs(1000000 + PPSClock::kHoldoverUs, utc_ms));
}

TEST(PPSClock, InvalidateDropsAnchor) {
    PPSClock clock;
    clock.Label(1000000, 1085000, 1700000005000);
    clock.Invalidate();
    uint64_t utc_ms;
    EXPECT_FALSE(clock.GetUTCUnixMs(1100000, utc_ms));
}

TEST(PPSClock, HandlesTimerWrap) {
    PPSClock clock;
    constexpr uint64_t edge_us = 0xFFFF0000;
    clock.Label(edge_us, edge_us + 85000, 1700000005000);
    uint64_t utc_ms = 0;
    ASSERT_TRUE(clock.GetUTCUnixMs(edge_us + 250000, utc_ms));
    EXPECT_EQ(utc_ms, 1700000005250u);
    clock.Label(edge_us, edge_us + (1ULL << 32), 1700000005000);
    EXPECT_FALSE(clock.GetUTCUnixMs(edge_us + (1ULL << 32), utc_ms));
}
