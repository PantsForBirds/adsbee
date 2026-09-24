#pragma once

#include "cc13x4_bootloader.hh"

enum class FlashResult { kOk, kEraseFailed, kProgramFailed, kVerifyFailed };

// All three assume the ROM bootloader is entered and baud-synced.
bool BakedImageMatches(Cc13x4Bootloader& bl);       // Bootloader CRC32 vs. baked per-segment CRCs.
FlashResult FlashBakedImage(Cc13x4Bootloader& bl);  // Erase + program + CRC verify.

// Erases ONLY the four 2 KB Settings sectors (0x000FC000..0x000FDFFF). The Device Info / OTA-key
// sectors at 0x000FE000 are never touched. On the next boot the app finds a blank region, fails its
// magic/CRC check, and rewrites defaults (see ti/settings/settings.cpp Load()).
//
// This is the only supported way back from a persisted settings blob that stops the console coming
// up, since AT+SETTINGS=RESET and AT+BOOT_UART_BOOTLOADER both need a console that already answers.
// Destructive by design, so it is never triggered automatically -- see the BOOTSEL gesture in main.cpp.
FlashResult EraseSettingsRegion(Cc13x4Bootloader& bl);
