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
// The USB baud rate is virtual: the UART follows the console's rate (announced with "UU", rate_watch.hh), or runs at
// kBootloaderBaud in the ROM bootloader.

enum class BridgeExit {
    kRecheck,        // BOOTSEL tapped: rerun the full CRC check / flash cycle.
    kEraseSettings,  // BOOTSEL held: erase the settings sectors on the way through the check cycle.
};

// Pass-through, starting at the UART's current rate (set by ConsoleLock()).
BridgeExit BridgeRun();
