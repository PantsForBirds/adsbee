#include "cpu_utils.hh"
#include "gtest/gtest.h"

TEST(CPUMonitor, UsagePercentFromIdleDelta) {
    EXPECT_EQ(CPUMonitor::UsagePercentFromIdleDelta(0, 0), 0);  // No time elapsed.
    EXPECT_EQ(CPUMonitor::UsagePercentFromIdleDelta(1000000, 1000000), 0);
    EXPECT_EQ(CPUMonitor::UsagePercentFromIdleDelta(0, 1000000), 100);
    EXPECT_EQ(CPUMonitor::UsagePercentFromIdleDelta(250000, 1000000), 75);
    // Idle can read slightly above total when the two counters are sampled at different instants.
    EXPECT_EQ(CPUMonitor::UsagePercentFromIdleDelta(1000100, 1000000), 0);
}

TEST(CPUMonitor, UsagePercentFromIdleDeltaLargeCounters) {
    // 100 * idle overflows 32 bits once idle passes ~43 s of 1 MHz ticks. A mostly idle core over a long interval
    // must still read as mostly idle.
    EXPECT_EQ(CPUMonitor::UsagePercentFromIdleDelta(3000000000u, 3100000000u), 4);
}

TEST(CPUMonitor, UsagePercentFromIdleDeltaAcrossCounterWrap) {
    // Deltas taken with unsigned subtraction across a 32-bit wrap of the run time counter.
    uint32_t last_total = 0xFFFF0000u, last_idle = 0xFFFE0000u;
    uint32_t total = last_total + 1000000u, idle = last_idle + 900000u;  // Both wrap past zero.
    EXPECT_EQ(CPUMonitor::UsagePercentFromIdleDelta(idle - last_idle, total - last_total), 10);
}
