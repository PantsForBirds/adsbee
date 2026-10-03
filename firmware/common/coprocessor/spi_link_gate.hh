#pragma once

#include <cstdint>

#include "hal.hh"

/**
 * Makes transactions on a dead SPI link fail fast. Otherwise each one spends its full retry budget (~100 ms handshake
 * wait per attempt), a main loop pass takes seconds, and the RP2040 watchdog reboots the board before main() can reset
 * just the ESP32.
 *
 * A transaction that exhausts its retries marks the link down. While down, only a single-attempt probe runs every
 * kProbeIntervalMs; any success marks the link up.
 */
class SPILinkGate {
   public:
    static constexpr uint32_t kProbeIntervalMs = 250;

    /**
     * Returns how many attempts the next transaction may make.
     * @param[in] max_attempts Attempts allowed while the link is up.
     * @retval max_attempts if up, 1 if down and a probe is due, 0 to fail immediately.
     */
    uint16_t AttemptsAllowed(uint16_t max_attempts) {
        if (!down_) return max_attempts;
        uint32_t timestamp_ms = get_time_since_boot_ms();
        if (timestamp_ms - last_probe_timestamp_ms_ < kProbeIntervalMs) return 0;
        last_probe_timestamp_ms_ = timestamp_ms;
        return 1;
    }

    /** Records the outcome of a transaction that made at least one attempt. */
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
