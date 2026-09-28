#pragma once

/**
 * Write "AT+" in string literals as AT_PLUS, e.g. AT_PLUS "SETTINGS?DUMP" or "Use " AT_PLUS "ESP32_FLASH.".
 *
 * Firmware up to 0.9.1-rc4 can misread part of an OTA image as console text while receiving it (a timed-out
 * AT+OTA=WRITE leaves the rest of its payload to the AT parser), and it runs any "AT+<command>" found in a line. Help
 * text such as "AT+BOOT_USB_UF2=1DEADBEE" in the image being sent would then reboot the receiver into its USB
 * bootloader. So "AT+" must never be stored contiguously in flash; linker_scripts/ota_at_scan.py checks every .ota.
 *
 * AT_PLUS stores kATPlusMarker instead of '+'. Console text output (CommsManager::iface_vprintf on the RP2040) prints
 * it as '+'. Binary outputs (BEAST uses 0x1A as its escape byte) are not touched.
 */
#define AT_PLUS "AT\x1a"
static constexpr char kATPlusMarker = '\x1a';

/** Replaces kATPlusMarker with '+' in a NUL-terminated text buffer. */
inline void ATTextRestorePlus(char *buf) {
    for (; *buf != '\0'; buf++) {
        if (*buf == kATPlusMarker) {
            *buf = '+';
        }
    }
}
