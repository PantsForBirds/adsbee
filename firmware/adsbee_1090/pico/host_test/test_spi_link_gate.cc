#include "gtest/gtest.h"
#include "hal_god_powers.hh"
#include "spi_link_gate.hh"

TEST(SPILinkGate, FailsFastWhileDownAndProbesPeriodically) {
    set_time_since_boot_ms(1000);
    SPILinkGate gate;
    EXPECT_FALSE(gate.IsDown());
    EXPECT_EQ(gate.AttemptsAllowed(3), 3);

    gate.Report(false);  // A transaction used up its retries.
    EXPECT_TRUE(gate.IsDown());
    EXPECT_EQ(gate.AttemptsAllowed(3), 0);
    inc_time_since_boot_ms(SPILinkGate::kProbeIntervalMs - 1);
    EXPECT_EQ(gate.AttemptsAllowed(3), 0);
    inc_time_since_boot_ms(1);
    EXPECT_EQ(gate.AttemptsAllowed(3), 1);  // Probe.
    EXPECT_EQ(gate.AttemptsAllowed(3), 0);  // Only one probe per interval.
    gate.Report(false);                     // Failed probe keeps the link down without restarting the interval.
    inc_time_since_boot_ms(SPILinkGate::kProbeIntervalMs);
    EXPECT_EQ(gate.AttemptsAllowed(3), 1);
    gate.Report(true);  // Probe succeeded.
    EXPECT_FALSE(gate.IsDown());
    EXPECT_EQ(gate.AttemptsAllowed(3), 3);
}

TEST(SPILinkGate, ResetClearsDown) {
    SPILinkGate gate;
    gate.Report(false);
    gate.Reset();
    EXPECT_FALSE(gate.IsDown());
    EXPECT_EQ(gate.AttemptsAllowed(3), 3);
}

// With the link dead, main loop passes stay short so the 5 s comms-lost check runs before the ~8.4 s watchdog.
TEST(SPILinkGate, DeadLinkLoopReachesCommsLostTimeoutBeforeWatchdog) {
    constexpr uint32_t kAttemptCostMs = 103;  // Handshake timeout plus lockout per failed attempt.
    constexpr uint32_t kTransactionsPerPass = 12;
    constexpr uint32_t kCommsLostTimeoutMs = 5000;
    constexpr uint32_t kWatchdogMs = 8388;
    set_time_since_boot_ms(10000);
    SPILinkGate gate;
    uint32_t last_success_ms = get_time_since_boot_ms();
    uint32_t longest_pass_ms = 0;
    while (get_time_since_boot_ms() - last_success_ms <= kCommsLostTimeoutMs) {
        uint32_t pass_start_ms = get_time_since_boot_ms();
        for (uint32_t i = 0; i < kTransactionsPerPass; i++) {
            uint16_t attempts = gate.AttemptsAllowed(3);
            if (attempts == 0) continue;
            inc_time_since_boot_ms(attempts * kAttemptCostMs);
            gate.Report(false);
        }
        inc_time_since_boot_ms(1);  // Rest of the loop.
        uint32_t pass_ms = get_time_since_boot_ms() - pass_start_ms;
        longest_pass_ms = pass_ms > longest_pass_ms ? pass_ms : longest_pass_ms;
    }
    EXPECT_LT(get_time_since_boot_ms() - last_success_ms, kWatchdogMs - 1000);
    EXPECT_LE(longest_pass_ms, 3 * kAttemptCostMs + kAttemptCostMs + 1);  // First failure plus at most one probe.
}
