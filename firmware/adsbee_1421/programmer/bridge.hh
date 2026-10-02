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
// The USB CDC baud rate is virtual: the host may open the port at any rate. The Programmer's UART runs at the module
// console's rate (found with the autobaud lock after every reset) and follows the console when the host changes its
// rate with AT+BAUD_RATE=CONSOLE,<n> or AT+SETTINGS=RESET, so the host keeps talking without reopening the port. In
// the ROM bootloader (after a reset with SYNC high) the UART runs at kBootloaderBaud. See rate_tracker.hh.

enum class BridgeExit {
    kRecheck,        // BOOTSEL tapped: rerun the full CRC check / flash cycle.
    kEraseSettings,  // BOOTSEL held: erase the settings sectors on the way through the check cycle.
};

BridgeExit BridgeRun();

// Rate the console was found at (NegotiateConsole() in main.cpp); pass-through starts from it.
void BridgeSetExpectedConsoleBaud(uint32_t baud);
