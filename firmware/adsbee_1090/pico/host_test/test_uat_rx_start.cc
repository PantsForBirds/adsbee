#include "gtest/gtest.h"
#include "uat_rx_start.hh"

using namespace UATRxStart;

TEST(UATRxStart, PhaseClassIsThreeTicksModulo40) {
    for (uint32_t ticks = 0; ticks < 200; ticks++) {
        EXPECT_EQ(PhaseClass(1000 + ticks, 1000), (ticks * 3) % kNumPhaseClasses) << ticks;
    }
}

TEST(UATRxStart, PhaseClassRepeatsEvery40Ticks) {
    // 40 RAT ticks = 10 us = 3 modem cycles.
    for (uint32_t ticks = 0; ticks < 40; ticks++) {
        EXPECT_EQ(PhaseClass(ticks + 40, 0), PhaseClass(ticks, 0));
        EXPECT_EQ(PhaseClass(ticks + 400000, 0), PhaseClass(ticks, 0));
    }
}

TEST(UATRxStart, PhaseClassAcrossTheTimerWrap) {
    // The stamp is a few ticks past the 2^32 wrap, the start just before it.
    EXPECT_EQ(PhaseClass(5, 0xFFFFFFFBu), PhaseClass(10, 0));
    EXPECT_EQ(PhaseClass(0, 0xFFFFFFFFu), 3);
}

TEST(UATRxStart, IsPreciseReadsTheMaskBit) {
    uint64_t mask = (uint64_t{1} << 0) | (uint64_t{1} << 39);
    EXPECT_TRUE(IsPrecise(mask, 0));
    EXPECT_TRUE(IsPrecise(mask, 39));
    EXPECT_FALSE(IsPrecise(mask, 1));
    EXPECT_FALSE(IsPrecise(~uint64_t{0}, kNumPhaseClasses));  // Out of range.
}

TEST(UATRxStart, StartIsExactComparesAcrossTheWrap) {
    EXPECT_TRUE(StartIsExact(1000, 1000 - 168, 168));
    EXPECT_FALSE(StartIsExact(1000, 1000 - 167, 168));
    EXPECT_TRUE(StartIsExact(100, 0xFFFFFFFFu - 100, 168));  // Start just past the wrap.
    EXPECT_FALSE(StartIsExact(0xFFFFFFF0u, 10, 0));          // Posted after the start time.
}

TEST(UATRxStart, StampIsPreciseNeedsAnExactStartAndAPreciseClass) {
    // CC1314: class 19 is precise for ADS-B but not for uplinks; class 39 is precise for uplinks.
    Start start{1000, true};
    EXPECT_TRUE(StampIsPrecise(kCC1314, false, start, 1000 + 33));  // 99 % 40 = 19.
    EXPECT_FALSE(StampIsPrecise(kCC1314, true, start, 1000 + 33));
    EXPECT_TRUE(StampIsPrecise(kCC1314, true, start, 1000 + 13));  // Class 39.
    start.exact = false;
    EXPECT_FALSE(StampIsPrecise(kCC1314, false, start, 1000 + 33));
}

TEST(UATRxStart, ChipTablesAreConsistent) {
    for (const ChipTiming* t : {&kCC1312, &kCC1314}) {
        EXPECT_EQ(t->precise_adsb_classes >> kNumPhaseClasses, 0u);
        EXPECT_EQ(t->precise_uplink_classes >> kNumPhaseClasses, 0u);
        EXPECT_LT(t->min_margin_rat, t->lead_rat);
    }
}
