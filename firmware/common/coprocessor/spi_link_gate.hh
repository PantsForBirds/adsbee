#pragma once

#include <cstdint>

#include "hal.hh"

/**
 * Tracks whether the SPI link to a coprocessor is up, so that a dead link fails fast instead of costing every caller
 * the full retry budget.
 *
 * Each failed SPI transaction attempt waits up to ~100 ms for a handshake, and a transaction retries 3 times. When the
 * ESP32 stops servicing SPI, the RP2040 main loop issues a dozen or more transactions per pass (raw packet flushes,
 * console writes, status reads, plus a console write for every error message those failures print), so one pass takes
 * several seconds. The RP2040 watchdog (~8.4 s) then fires before main() gets to its 5 s "cycle the ESP32 enable pin"
 * check, and the whole board reboots instead of just the ESP32.
 *
 * Once a transaction has used all of its retries the link is marked down. While it is down, transactions get no
 * attempts except for a single-attempt probe every kProbeIntervalMs, and any success marks the link up again.
 */
class SPILinkGate {
   public:
    static constexpr uint32_t kProbeIntervalMs = 250;

    /**
     * Returns how many attempts the next transaction may make.
     * @param[in] max_attempts Attempts allowed while the link is up.
     * @retval max_attempts if the link is up, 1 if the link is down and a probe is due, 0 if the transaction should
     * fail immediately.
     */
    uint16_t AttemptsAllowed(uint16_t max_attempts) {
        if (!down_) return max_attempts;
        uint32_t timestamp_ms = get_time_since_boot_ms();
        if (timestamp_ms - last_probe_timestamp_ms_ < kProbeIntervalMs) return 0;
        last_probe_timestamp_ms_ = timestamp_ms;
        return 1;
    }

    /**
     * Records the outcome of a transaction that made at least one attempt.
     * @param[in] success True if the transaction succeeded.
     */
    void Report(bool success) {
        if (success) {
            down_ = false;
        } else if (!down_) {
            down_ = true;
            last_probe_timestamp_ms_ = get_time_since_boot_ms();
        }
    }

    void Reset() { down_ = false; }
    bool IsDown() const { return down_; }

   private:
    bool down_ = false;
    uint32_t last_probe_timestamp_ms_ = 0;
};
