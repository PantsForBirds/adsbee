#pragma once

#include <stdint.h>

// USB serial <-> UART pass-through. Host RTS/DTR drive SYNC and RESET_N like a USB-UART adapter (modem_lines.hh).
// The host's baud rate is ignored: the UART follows the console's rate (rate_watch.hh), or runs at kBootloaderBaud in
// the ROM bootloader.

enum class BridgeExit {
    kRecheck,        // BOOTSEL tapped: rerun the full CRC check / flash cycle.
    kEraseSettings,  // BOOTSEL held: erase the settings sectors on the way through the check cycle.
};

// Pass-through, starting at the UART's current rate (set by ConsoleLock()).
BridgeExit BridgeRun();
