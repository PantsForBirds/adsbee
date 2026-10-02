#pragma once

#include <stddef.h>
#include <stdint.h>

// Keeps the Programmer's UART at the module console's rate during pass-through. Pure logic with no SDK dependencies
// so it can be host-tested (host_test/test_rate_tracker.cc); bridge.cc feeds it both byte streams, the UART's framing
// error count and the clock, and carries out the steps it asks for.
//
// The USB CDC baud rate is virtual: the host opens the port at any rate and the Programmer ignores it (except the
// reboot-to-BOOTSEL magic baud, host_line_coding.hh). The UART runs at the console's actual rate, which the autobaud
// lock finds after every reset (console_lock.hh). The tracker follows the console when the host changes its rate:
//
//   1. The host sends AT+BAUD_RATE=CONSOLE,<n> (n in the console's range) or AT+SETTINGS=RESET (back to the factory
//      1 M). The module runs a command when its '\n' arrives, so OnHostBytes() lets the bytes through up to and
//      including that '\n' and holds everything after it in the CDC FIFO.
//   2. The module answers "OK" at the old rate, drains it and switches. OnConsoleBytes() forwards the console output up
//      to the end of that "OK\r\n" and Poll() returns kRetune: the bridge retunes its UART to n at once and drops what
//      came in after the OK (sent at the new rate, so garbled at the old one). Host data flows again.
//   3. "ERROR" (a refused rate) ends the hold without a retune.
//   4. No answer within the timeout: Poll() returns kWakeLock (find the console with the trigger on a SYNC wake, which
//      keeps its live settings). The host data waits for that too.
//
// AT+REBOOT holds host data and asks the bridge to reset and lock (kResetAndLock): the module comes back at its saved
// rate. Nothing here sends AT+SETTINGS=SAVE; the host decides whether a new rate persists.
//
// Safety net: the console's rate can still move without the tracker seeing it (a command it doesn't parse, another
// module plugged in). Framing errors on the UART are what that looks like, so a burst of them asks for a wake lock,
// at most once every kRelockIntervalMs.
//
// After a reset with SYNC high (the ROM bootloader, which auto-bauds to whatever rate the UART sends at) the tracker
// stands aside until the next reset with SYNC low: bytes pass both ways untouched and the UART stays at the
// bootloader rate.
class RateTracker {
   public:
    static constexpr uint32_t kFactoryConsoleBaud = 1000000;
    // AT+REBOOT: time for the module to take the command before the bridge resets it.
    static constexpr uint32_t kRebootSettleMs = 100;
    // How long the module's OK may take: it queues behind up to 8 kB of console output, which takes 8.5 s at 9600.
    static constexpr uint32_t kReplyBaseMs = 500;
    static constexpr uint32_t kReplyMaxMs = 10000;
    // kRelockErrors framing errors within kErrorWindowMs: the UART no longer matches the console.
    static constexpr uint32_t kRelockErrors = 8;
    static constexpr uint32_t kErrorWindowMs = 200;
    static constexpr uint32_t kRelockIntervalMs = 3000;

    struct Step {
        enum Kind {
            kNone,
            kRetune,        // Set the UART to `baud` now and drop received input; then call OnRetuned().
            kWakeLock,      // ConsoleLock(kSyncWake); then OnLocked(). `baud`: where to leave the UART if it fails.
            kResetAndLock,  // ConsoleLock(kReset); then OnLocked(). `baud` as for kWakeLock.
        };
        Kind kind = kNone;
        uint32_t baud = 0;
    };

    // Start of a pass-through session with the console at console_baud (0 = unknown).
    void Start(uint32_t console_baud, uint32_t now_ms);

    // A host reset: SYNC high (ROM bootloader) or, after the lock, SYNC low with the console at console_baud.
    void OnReset(bool sync_high, uint32_t console_baud, uint32_t now_ms);
    // Outcome of kWakeLock / kResetAndLock: the console's rate, 0 if not found (the UART is at the step's `baud`).
    void OnLocked(uint32_t console_baud, uint32_t now_ms);
    // The bridge carried out kRetune.
    void OnRetuned(uint32_t now_ms);

    // Host -> module bytes the bridge wants to forward. Returns how many of them may go now (the rest stay with the
    // bridge until HoldHostData() is false and are passed in again).
    size_t OnHostBytes(const uint8_t* data, size_t len, uint32_t now_ms);
    // Module -> host bytes as read from the UART. Returns how many to forward; the rest came after an acknowledged
    // switch, at the new rate, and are dropped.
    size_t OnConsoleBytes(const uint8_t* data, size_t len, uint32_t now_ms);
    // New UART framing / break errors since the last call.
    void OnRxErrors(uint32_t count, uint32_t now_ms);

    // Next step at now_ms. sync_high: SYNC is driven high right now (the module may be asleep or about to enter the
    // ROM bootloader), which defers locks.
    Step Poll(uint32_t now_ms, bool sync_high);

    // True while host -> module data must wait (a switch or a lock is under way).
    bool HoldHostData() const { return state_ != State::kIdle && state_ != State::kRomBootloader; }

    uint32_t console_baud() const { return console_baud_; }
    bool in_rom_bootloader() const { return state_ == State::kRomBootloader; }

   private:
    enum class State {
        kIdle,           // Pass-through at console_baud_.
        kAwaitReply,     // Switch command sent; waiting for OK / ERROR.
        kRetunePending,  // OK seen; the bridge retunes next.
        kRebootPending,  // AT+REBOOT sent.
        kLost,           // Needs a wake lock.
        kRomBootloader,  // Hands off until the next reset with SYNC low.
    };

    void OnHostLine(uint32_t now_ms);
    uint32_t ReplyTimeoutMs() const;
    void Idle();

    State state_ = State::kIdle;
    uint32_t console_baud_ = 0;
    uint32_t target_baud_ = 0;   // The rate the pending switch moves the console to.
    uint32_t since_ms_ = 0;      // Start of the current state.
    uint32_t last_lock_ms_ = 0;  // Last lock or retune, for the relock rate limit.

    uint32_t error_count_ = 0;  // Framing errors in the window starting at error_window_ms_.
    uint32_t error_window_ms_ = 0;

    // Current host -> module line; overlong lines are dropped.
    static constexpr size_t kLineMax = 40;
    char line_[kLineMax + 1] = {};
    size_t line_len_ = 0;
    bool line_overflow_ = false;

    // Progress through "OK\r\n" and "ERROR" in the console output while a reply is awaited.
    size_t ok_matched_ = 0;
    size_t error_matched_ = 0;
};
