#include "cpu_utils.hh"
#include "gtest/gtest.h"
#include "hal_god_powers.hh"

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

#ifndef ON_ESP32
// RP2040 / CC1312 loop-tick estimator: 100e3 ticks per second is idle (0%), 100 ticks per second or fewer is 100%.
TEST(CPUMonitor, LoopTickUsage) {
    CPUMonitor monitor({.idle_ticks_per_update_interval = 100000, .full_usage_update_frequency_hz = 100,
                        .update_interval_ms = 1000});
    set_time_since_boot_ms(0);
    monitor.Update();  // Starts the first interval.

    auto run_interval = [&](uint32_t ticks) {
        for (uint32_t i = 0; i < ticks; i++) monitor.Tick();
        inc_time_since_boot_ms(1000);
        monitor.Update();
        return monitor.GetUsagePercent();
    };
    EXPECT_EQ(run_interval(100000 + 100), 0);  // Idle loop.
    EXPECT_EQ(run_interval(50000 + 100), 50);
    EXPECT_EQ(run_interval(100), 100);  // Exactly the full usage rate.
    // A loop slower than the full usage rate is saturated. It used to underflow ticks - full_usage_ticks and read 0%.
    EXPECT_EQ(run_interval(50), 100);
    EXPECT_EQ(run_interval(0), 100);
}
#endif
