#include "adsbee.hh"
#include "hardware_unit_tests.hh"

UTEST(Clocks, Test48MHzMLATCounter) {
    static const uint32_t kTestNumRepeats = 5;
    for (uint32_t i = 0; i < kTestNumRepeats; i++) {
        uint64_t mlat_counter_start = adsbee.GetMLAT48MHzCounts();
        sleep_ms(1000);
        // Expect MLAT counter to increment by at least 48000.
        uint64_t mlat_counter_end = adsbee.GetMLAT48MHzCounts();
        uint64_t mlat_counter_delta = mlat_counter_end - mlat_counter_start;
        EXPECT_NEAR(48'000'000, mlat_counter_delta, 1200);
    }
}

UTEST(Clocks, Test12MHzMLATCounter) {
    static const uint32_t kTestNumRepeats = 5;
    for (uint32_t i = 0; i < kTestNumRepeats; i++) {
        uint64_t mlat_counter_start = adsbee.GetMLAT12MHzCounts();
        sleep_ms(1000);
        // Expect MLAT counter to increment by at least 12000.
        uint64_t mlat_counter_end = adsbee.GetMLAT12MHzCounts();
        uint64_t mlat_counter_delta = mlat_counter_end - mlat_counter_start;
        EXPECT_NEAR(12'000'000, mlat_counter_delta, 300);
    }
}

// Counts MLAT jitter PWM slice counts over span_us of the 1MHz timer. Runs from RAM with interrupts disabled so that
// flash cache misses and interrupts don't stretch the span.
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
    // OnDemodComplete() subtracts MLAT jitter PWM slice counts, converted with MLATJitterCountsTo48MHzCounts(), from
    // the 48MHz MLAT timestamp. Verify that a converted span advances at 48MHz.
    static const uint32_t kTestNumRepeats = 10;
    static const uint32_t kSpanUs = 500;  // Shorter than the 524us wrap of the 16-bit counter at 125MHz.
    for (uint32_t i = 0; i < kTestNumRepeats; i++) {
        uint32_t mlat_counts = ADSBee::MLATJitterCountsTo48MHzCounts(CountMLATJitterPWMSliceSpan(kSpanUs));
        // Expect 24000 counts in 500us at 48MHz, within 1us.
        EXPECT_NEAR(kSpanUs * 48, mlat_counts, 48);
    }
}
