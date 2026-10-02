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
// The follower also remembers the rate the console boots at (its saved rate, "boot rate"): after a reset with SYNC
// low the console comes back at it, so it is the console's rate until a renegotiation moves it. It watches the host's
// commands for the ones that move the console's rate behind the Programmer's back: AT+BAUD_RATE=CONSOLE,<n> (n is
// probed first at the next renegotiation), AT+SETTINGS=SAVE (the live rate becomes the boot rate), AT+SETTINGS=RESET
// (the console drops to its factory 1 M, live and saved) and AT+REBOOT (back to the boot rate).
class BaudFollower {
   public:
    // Line coding must be stable this long before a renegotiation starts.
    static constexpr uint32_t kSettleMs = 100;
    static constexpr uint32_t kFactoryConsoleBaud = 1000000;

    // What the bridge should do next (Poll()).
    struct Step {
        enum Kind {
            kNone,
            kApplyDirect,  // Retune the Programmer's UART to `baud`; the console is not touched.
            kRenegotiate,  // Move the console to `baud` (0: only find it) and retune; then call OnRenegotiated().
        };
        Kind kind = kNone;
        uint32_t baud = 0;
    };

    // Start of a pass-through session with the console at console_baud (0 = unknown), boot_baud the console's saved
    // rate (0 = unknown) and host_baud the host's current line coding (0 = never set).
    void Start(uint32_t console_baud, uint32_t boot_baud, uint32_t host_baud, uint32_t now_ms);

    // Host line coding (only rates ClassifyHostBaud() says to apply).
    void OnHostBaud(uint32_t baud, uint32_t now_ms);

    // The bridge pulsed RESET_N; it was released at now_ms with SYNC high (ROM bootloader) or low (application, which
    // takes boot_wait_ms to bring its console up).
    void OnReset(bool sync_high, uint32_t now_ms, uint32_t boot_wait_ms);

    // Host -> module bytes, as forwarded (command snooping, see above). boot_wait_ms as for OnReset().
    void OnHostBytes(const uint8_t* data, size_t len, uint32_t now_ms, uint32_t boot_wait_ms);

    // Next step at now_ms. sync_high: SYNC is driven high right now (the console may be asleep), which defers
    // renegotiation. Clears the pending work it returns; host changes that arrive while the bridge carries it out
    // queue another step.
    Step Poll(uint32_t now_ms, bool sync_high);

    // Outcome of a kRenegotiate step: the console's rate as found (0 = not found). The bridge has set its UART.
    void OnRenegotiated(uint32_t console_baud);

    // True while host -> module data should stay in the CDC FIFO (a renegotiation is coming).
    bool HoldHostData(bool sync_high) const;

    uint32_t console_baud() const { return console_baud_; }
    uint32_t boot_baud() const { return boot_baud_; }
    // Rate the host's last AT+BAUD_RATE=CONSOLE,<n> asked for, not yet confirmed (0 = none).
    uint32_t hinted_baud() const { return hinted_baud_; }
    bool in_rom_bootloader() const { return rom_bootloader_; }

    // Set when the boot rate changed (from a host's AT+SETTINGS=SAVE or RESET); the bridge stores it in flash and
    // calls ClearBootBaudChanged().
    bool boot_baud_changed() const { return boot_baud_changed_; }
    void ClearBootBaudChanged() { boot_baud_changed_ = false; }

   private:
    // The console rate the host wants (its line coding when the console accepts it), or 0.
    uint32_t DesiredBaud() const;
    // Something moved the console or the host's rate: work out the next step at the next Poll().
    void Pend() { pending_ = true; }
    // Wait for the console (booting, or busy with a command) until now_ms + wait_ms before renegotiating.
    void WaitForConsole(uint32_t now_ms, uint32_t wait_ms);
    void OnHostLine(uint32_t now_ms, uint32_t boot_wait_ms);

    uint32_t host_baud_ = 0;
    uint32_t console_baud_ = 0;
    uint32_t boot_baud_ = 0;
    uint32_t hinted_baud_ = 0;
    bool rom_bootloader_ = false;
    bool boot_baud_changed_ = false;

    bool pending_ = false;
    uint32_t host_changed_ms_ = 0;  // Last host line-coding change.
    bool waiting_for_boot_ = false;
    uint32_t boot_done_ms_ = 0;  // Renegotiation waits until here (waiting_for_boot_).

    // Current host -> module line, upper-cased; overlong lines are dropped.
    static constexpr size_t kLineMax = 40;
    char line_[kLineMax + 1] = {};
    size_t line_len_ = 0;
    bool line_overflow_ = false;
};

// Rates the Programmer probes when it looks for the console (AtFindConsoleBaud): the factory default, the previous
// whitelist, then other common rates. 76800 and 250000 are common MAVLink telemetry radio rates.
static constexpr uint32_t kCommonConsoleBauds[] = {1000000, 921600, 460800, 230400, 115200, 57600, 500000, 250000,
                                                   38400,   19200,  9600,   76800,  2000000, 1500000, 3000000};

// Builds the probe order: the `preferred` rates first (0s and rates the console doesn't accept are skipped), then
// kCommonConsoleBauds, without duplicates. Returns the number of rates written (at most max_out).
size_t BuildConsoleBaudCandidates(const uint32_t* preferred, size_t num_preferred, uint32_t* out, size_t max_out);
