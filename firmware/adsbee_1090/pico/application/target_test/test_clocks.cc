#include "adsbee.hh"
#include "hardware_unit_tests.hh"

struct MLATCounterSnapshot {
    uint64_t timestamp_us;
    uint64_t mlat_48mhz_counts;
    uint64_t mlat_12mhz_counts;
};

// Reads the MLAT counters right after a 1MHz timer tick. Run with RunOnISRCore(): the MLAT counter uses that core's
// SysTick.
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
        // Expect 48 counts per microsecond.
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
        // Expect 12 counts per microsecond.
        uint64_t elapsed_us = mlat_counter_snapshot_end.timestamp_us - mlat_counter_snapshot_start.timestamp_us;
        uint64_t mlat_counter_delta =
            mlat_counter_snapshot_end.mlat_12mhz_counts - mlat_counter_snapshot_start.mlat_12mhz_counts;
        EXPECT_NEAR(elapsed_us * 12, mlat_counter_delta, 300);
    }
}

// Counts MLAT jitter PWM slice counts over span_us. Runs from RAM with interrupts off so nothing stretches the span.
static uint16_t __not_in_flash_func(CountMLATJitterPWMSliceSpan)(uint32_t span_us) {
    uint32_t interrupts = save_and_disable_interrupts();
    uint32_t timestamp_us = time_us_32();
    while (time_us_32() == timestamp_us) {
        // Start on a timer tick.
    }
    uint16_t counts_start = adsbee.GetMLATJitterPWMSliceCounts();
    timestamp_us = time_us_32();
    while (time_us_32() - timestamp_us < span_us) {
    }
    uint16_t counts_end = adsbee.GetMLATJitterPWMSliceCounts();
    restore_interrupts(interrupts);
    return counts_end - counts_start;  // Unsigned subtraction handles a wrap of the 16-bit counter.
}

UTEST(Clocks, TestMLATJitterPWMSlice) {
    // Verify that converted jitter counts advance at 48MHz, as OnDemodComplete() assumes.
    static const uint32_t kTestNumRepeats = 10;
    static const uint32_t kSpanUs = 500;  // Under the 16-bit counter's 524us wrap.
    for (uint32_t i = 0; i < kTestNumRepeats; i++) {
        uint32_t mlat_counts = ADSBee::MLATJitterCountsTo48MHzCounts(CountMLATJitterPWMSliceSpan(kSpanUs));
        // Expect 24000 counts in 500us at 48MHz, within 1us.
        EXPECT_NEAR(kSpanUs * 48, mlat_counts, 48);
    }
}
