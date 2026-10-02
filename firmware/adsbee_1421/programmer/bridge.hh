#pragma once

#include <stdint.h>

// Transparent USB-CDC <-> UART pass-through emulating a TTL USB-UART adapter wired the way host
// tools expect (see "Reflashing over UART" in firmware/adsbee_1421/README.md):
//
//   host RTS bit asserted  -> SYNC pin LOW   (asserting a modem-control line drives the physical
//   host RTS bit deasserted-> SYNC pin HIGH   pin low on FTDI-style adapters; the web console and
//   host DTR bit asserted  -> RESET_N pulse   host scripts are written against that polarity)
//
// DTR is edge-triggered (a 50 ms reset pulse on assert), so terminals that keep DTR asserted for
// the whole session do not hold the device in reset. RTS deasserted only drives SYNC high while
// DTR is asserted (port open) or around a DTR-edge reset, so closing the port, which drops both
// lines, leaves the device awake (see modem_lines.hh).
//
// Host line-coding baud changes move the module console to the host's rate (AT+BAUD_RATE, then the
// Programmer's UART follows), so a tool that opens the port at its own rate reads the console at that rate. In the
// ROM bootloader (after a reset with SYNC high) and for rates the console doesn't accept, the host's rate is applied
// to the UART directly. See baud_follower.hh.

enum class BridgeExit {
    kRecheck,        // BOOTSEL tapped: rerun the full CRC check / flash cycle.
    kEraseSettings,  // BOOTSEL held: erase the settings sectors on the way through the check cycle.
};

BridgeExit BridgeRun();
uint32_t BridgeHostBaud();  // Most recent host line-coding baud (0 if the host never set one).

// Rate the device console was last negotiated to (see NegotiateConsole); pass-through starts from it.
void BridgeSetExpectedConsoleBaud(uint32_t baud);
