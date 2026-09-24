#pragma once

#include <stdint.h>

// Transparent USB-CDC <-> UART pass-through emulating a TTL USB-UART adapter wired the way the
// existing host tools expect (see software/adsbee_1421_flasher/README.md):
//
//   host RTS bit asserted  -> SYNC pin LOW   (asserting a modem-control line drives the physical
//   host RTS bit deasserted-> SYNC pin HIGH   pin low on FTDI-style adapters; the web console and
//   host DTR bit asserted  -> RESET_N pulse   python flasher are written against that polarity)
//
// DTR is edge-triggered (a 50 ms reset pulse on assert) rather than level-held so terminals that
// keep DTR asserted for the whole session do not hold the device in reset. Host line-coding baud
// changes are applied to the UART directly.

enum class BridgeExit {
    kRecheck,          // BOOTSEL tapped: rerun the full CRC check / flash cycle.
    kEraseSettings,    // BOOTSEL held: erase the settings sectors on the way through the check cycle.
    kHostResetTarget,  // Host reset the device (DTR, SYNC low) but its line-coding baud differs
                       // from the negotiated console rate: caller should re-negotiate the
                       // console to BridgeHostBaud().
};

BridgeExit BridgeRun();
uint32_t BridgeHostBaud();  // Most recent host line-coding baud (0 if the host never set one).

// Rate the device console was last negotiated to (see NegotiateConsole); used to decide whether
// a host-driven reset needs a re-negotiation.
void BridgeSetExpectedConsoleBaud(uint32_t baud);
