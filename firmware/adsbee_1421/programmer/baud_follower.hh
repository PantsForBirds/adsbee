#pragma once

#include <stddef.h>
#include <stdint.h>

// Keeps the module console's baud rate following the host's line coding during pass-through (Option A). Pure logic
// with no SDK dependencies so it can be host-tested (host_test/baud_follower_test.cc); bridge.cc feeds it the TinyUSB
// callbacks, the host -> module bytes and the clock, and carries out the steps it asks for.
//
// The host's line coding is what a GCS or terminal expects on the wire. USB CDC baud is virtual, but the Programmer's
// UART must match the console, so a host rate the console accepts (IsRenegotiableBaud()) is applied by renegotiating
// the console: AT+BAUD_RATE=CONSOLE,<new> at the console's current rate, then retuning the Programmer's UART. Hosts
// often set several rates while opening a port (pymavlink sets 1200, then its own rate), so the follower waits for the
// line coding to settle for kSettleMs before acting on it, and only the last rate counts. Host data is held in the CDC
// FIFO from the first change until the renegotiation ends, so it reaches the console at the new rate.
//
// Rates are applied to the Programmer's UART directly, as before Option A:
//   - after a reset with SYNC high (ROM bootloader, which auto-bauds to whatever the host sends) until the next reset
//     with SYNC low, and without waiting for the line coding to settle;
//   - for rates the console doesn't accept (the 1200 baud pymavlink opens ports at, anything below 9600 or above 3 M).
//
// The bridge finds the console's rate itself after a reset with SYNC low (the autobaud lock, console_lock.hh) and
// reports it with OnReset(). The follower watches the host's commands for the ones that move the console's rate behind
// the Programmer's back: AT+BAUD_RATE=CONSOLE,<n> (n is tried first at the next renegotiation), AT+SETTINGS=RESET (the
// console drops to its factory 1 M) and AT+REBOOT (back to its saved rate, so the bridge resets and locks again).
class BaudFollower {
   public:
    // Line coding must be stable this long before a renegotiation starts; snooped commands get as long to take effect.
    static constexpr uint32_t kSettleMs = 100;
    static constexpr uint32_t kFactoryConsoleBaud = 1000000;

    // What the bridge should do next (Poll()).
    struct Step {
        enum Kind {
            kNone,
            kApplyDirect,  // Retune the Programmer's UART to `baud`; the console is not touched.
            kRenegotiate,  // Move the console to `baud` (0: only find it) and retune; then call OnRenegotiated().
            kReset,        // Reset the module into the application and lock; then call OnReset().
        };
        Kind kind = kNone;
        uint32_t baud = 0;
    };

    // Start of a pass-through session with the console at console_baud (0 = unknown) and host_baud the host's current
    // line coding (0 = never set).
    void Start(uint32_t console_baud, uint32_t host_baud, uint32_t now_ms);

    // Host line coding (only rates ClassifyHostBaud() says to apply).
    void OnHostBaud(uint32_t baud, uint32_t now_ms);

    // The bridge reset the module with SYNC high (ROM bootloader), or with SYNC low and found the console at
    // console_baud (0 = not found).
    void OnReset(bool sync_high, uint32_t console_baud);

    // Host -> module bytes, as forwarded (command snooping, see above).
    void OnHostBytes(const uint8_t* data, size_t len, uint32_t now_ms);

    // Next step at now_ms. sync_high: SYNC is driven high right now (the console may be asleep), which defers
    // renegotiation. Clears the pending work it returns; host changes that arrive while the bridge carries it out
    // queue another step.
    Step Poll(uint32_t now_ms, bool sync_high);

    // Outcome of a kRenegotiate step: the console's rate as found (0 = not found). The bridge has set its UART.
    void OnRenegotiated(uint32_t console_baud);

    // True while host -> module data should stay in the CDC FIFO (a renegotiation is coming).
    bool HoldHostData(bool sync_high) const;

    uint32_t console_baud() const { return console_baud_; }
    // Rate the host's last AT+BAUD_RATE=CONSOLE,<n> asked for, not yet confirmed (0 = none).
    uint32_t hinted_baud() const { return hinted_baud_; }
    bool in_rom_bootloader() const { return rom_bootloader_; }

   private:
    // The console rate the host wants (its line coding when the console accepts it), or 0.
    uint32_t DesiredBaud() const;
    // Something moved the console or the host's rate: work out the next step at the next Poll().
    void Pend() { pending_ = true; }
    void OnHostLine(uint32_t now_ms);

    uint32_t host_baud_ = 0;
    uint32_t console_baud_ = 0;
    uint32_t hinted_baud_ = 0;
    bool rom_bootloader_ = false;
    bool reset_wanted_ = false;  // AT+REBOOT seen.

    bool pending_ = false;
    uint32_t host_changed_ms_ = 0;  // Last host line-coding change or snooped command.

    // Current host -> module line, upper-cased; overlong lines are dropped.
    static constexpr size_t kLineMax = 40;
    char line_[kLineMax + 1] = {};
    size_t line_len_ = 0;
    bool line_overflow_ = false;
};
