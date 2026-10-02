#include "adsbee.hh"
#include "hardware_unit_tests.hh"

struct MLATCounterSnapshot {
    uint64_t timestamp_us;
    uint64_t mlat_48mhz_counts;
    uint64_t mlat_12mhz_counts;
};

// Reads the MLAT counters right after a tick of the 1MHz timer. SysTick is a per-core timer, and the MLAT counter is
// built on the SysTick of the core that ran MLATCounterInit(), which is the core that handles the demodulator ISRs.
// Run this with RunOnISRCore().
static void __not_in_flash_func(TakeMLATCounterSnapshot)(void* arg) {
    MLATCounterSnapshot* snapshot = static_cast<MLATCounterSnapshot*>(arg);
    uint32_t interrupts = save_and_disable_interrupts();
    uint32_t timestamp_us = time_us_32();
    while (time_us_32() == timestamp_us) {
        // Start on a timer tick.
    }
    snapshot->timestamp_us = time_us_64();
    snapshot->mlat_48mhz_counts = adsbee.GetMLAT48MHzCounts();
    snapshot->mlat_12mhz_counts = adsbee.GetMLAT12MHzCounts();
    restore_interrupts(interrupts);
}

// Snapshots must outlive a RunOnISRCore() timeout.
static MLATCounterSnapshot mlat_counter_snapshot_start, mlat_counter_snapshot_end;

UTEST(Clocks, Test48MHzMLATCounter) {
    static const uint32_t kTestNumRepeats = 5;
    for (uint32_t i = 0; i < kTestNumRepeats; i++) {
        ASSERT_TRUE(RunOnISRCore(TakeMLATCounterSnapshot, &mlat_counter_snapshot_start));
        sleep_ms(1000);
        ASSERT_TRUE(RunOnISRCore(TakeMLATCounterSnapshot, &mlat_counter_snapshot_end));
        // Expect the MLAT counter to advance by 48 counts per microsecond of the 1MHz timer (48000000 in 1 s).
        uint64_t elapsed_us = mlat_counter_snapshot_end.timestamp_us - mlat_counter_snapshot_start.timestamp_us;
        uint64_t mlat_counter_delta =
            mlat_counter_snapshot_end.mlat_48mhz_counts - mlat_counter_snapshot_start.mlat_48mhz_counts;
        EXPECT_NEAR(elapsed_us * 48, mlat_counter_delta, 1200);
    }
}

UTEST(Clocks, Test12MHzMLATCounter) {
    static const uint32_t kTestNumRepeats = 5;
    for (uint32_t i = 0; i < kTestNumRepeats; i++) {
        ASSERT_TRUE(RunOnISRCore(TakeMLATCounterSnapshot, &mlat_counter_snapshot_start));
        sleep_ms(1000);
        ASSERT_TRUE(RunOnISRCore(TakeMLATCounterSnapshot, &mlat_counter_snapshot_end));
        // Expect the MLAT counter to advance by 12 counts per microsecond of the 1MHz timer (12000000 in 1 s).
        uint64_t elapsed_us = mlat_counter_snapshot_end.timestamp_us - mlat_counter_snapshot_start.timestamp_us;
        uint64_t mlat_counter_delta =
            mlat_counter_snapshot_end.mlat_12mhz_counts - mlat_counter_snapshot_start.mlat_12mhz_counts;
        EXPECT_NEAR(elapsed_us * 12, mlat_counter_delta, 300);
    }
}

UTEST(Clocks, TestMLATJitterPWMSlice) {
    // Verify that the MLAT jitter PWM slice is running at 48MHz.
    static const uint32_t kTestNumRepeats = 10;
    for (uint32_t i = 0; i < kTestNumRepeats; i++) {
        uint16_t mlat_jitter_counts_start = adsbee.GetMLATJitterPWMSliceCounts();
        sleep_us(10);
        // Expect MLAT jitter PWM slice to increment by around 480 counts in 10us (48MHz).
        uint16_t mlat_jitter_counts_end = adsbee.GetMLATJitterPWMSliceCounts();
        uint16_t mlat_jitter_counts_delta = mlat_jitter_counts_end - mlat_jitter_counts_start;
        EXPECT_NEAR(480, mlat_jitter_counts_delta, 10);
    }
}