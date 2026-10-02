#pragma once

#include <stdint.h>

// Console autobaud (README "Console autobaud"). The console says "UU" (0x55 0x55) at its current rate:
//   - right after every rate change (AT+BAUD_RATE=CONSOLE,<n>, AT+SETTINGS=RESET, settings applied at boot), after the
//     OK that still goes out at the old rate,
//   - at every boot, once SettingsManager::Apply() has set the saved rate,
//   - when its RX line sees a break (held low for longer than a frame at the console's rate), as soon as the main loop
//     notices. Queued console output is dropped first: a host that sends a break doesn't know the rate, so it can't
//     read that output anyway, and draining it first takes seconds at low rates.
// A 'U' framed 8N1 is a square wave with an edge at every bit (start 0, data 1 0 1 0 1 0 1 0, stop 1), and "UU" back
// to back is 20 bits of it, so a host times the edges and has the rate, the way LIN slaves use the 0x55 sync field
// after a break and many MCU UARTs auto-baud on 0x55. The ADSBee 1421 Programmer does this
// (firmware/adsbee_1421/programmer/rate_watch.hh).
//
// Pure logic with no SDK dependencies: comms.cpp uses it, and the Programmer's host tests run it in their model of
// the module.
namespace ConsoleAutobaud {

static constexpr char kAnswer[] = "UU";
static constexpr uint16_t kAnswerLen = sizeof(kAnswer) - 1;

// Keeps the NUL byte a break leaves in the console's input away from the AT parser (cppAT ends a line at a NUL, so it
// would cut a command the host is in the middle of sending). The PL011 puts one NUL with its break flag set into the
// RX FIFO per break and raises the break bit in its raw interrupt status at the same time; the UART2 driver's DMA
// copies only the data bits into its RX ring. CommsManager clears the status bit when it answers the break and calls
// OnBreak(); the NUL reaches the ring after that.
class BreakFilter {
   public:
    // Caps how many NULs are waiting to be dropped, in case a break never left one (the UART was reopened meanwhile).
    static constexpr uint8_t kMaxPendingNuls = 4;

    void OnBreak() {
        if (pending_nuls_ < kMaxPendingNuls) pending_nuls_++;
    }

    // Returns false if `c` is a break's NUL, which the caller drops.
    bool Accept(char c) {
        if (c != '\0' || pending_nuls_ == 0) return true;
        pending_nuls_--;
        return false;
    }

    uint8_t pending_nuls() const { return pending_nuls_; }

   private:
    uint8_t pending_nuls_ = 0;
};

}  // namespace ConsoleAutobaud
