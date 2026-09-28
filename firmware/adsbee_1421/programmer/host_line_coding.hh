#pragma once

#include <stdint.h>

// Host line coding (CDC SET_LINE_CODING) -> what the jig does with the requested baud. Pure logic with no SDK
// dependencies so it can be host-tested (host_test/host_line_coding_test.cc); bridge.cc calls it from
// tud_cdc_line_coding_cb().

// Host baud that reboots the jig's own RP2040 into its USB bootloader (RPI-RP2), so the jig can be updated without
// pressing BOOT. The same magic baud as the ADSBee 1090: PICO_STDIO_USB_RESET_MAGIC_BAUD_RATE=0XDEADBEE in
// firmware/adsbee_1090/pico/CMakeLists.txt (host_test fails if the two drift apart). Not pico-sdk's default 1200:
// pymavlink and other tools open ports at 1200 as a Linux kernel workaround. The jig owns TinyUSB directly (no
// stdio_usb), so the SDK's own handler never runs and the baud is checked here. It is far above any UART rate, and
// bridge.cc static_asserts that it is not a console or bootloader rate.
static constexpr uint32_t kRebootToBootselBaud = 0xDEADBEE;  // 233495534 baud.

enum class HostBaudAction {
    kIgnore,           // Not a baud (0): leave the target UART alone.
    kRebootToBootsel,  // kRebootToBootselBaud: reboot the jig; never forwarded or remembered as the host baud.
    kApply,            // Anything else: the host's rate for the target UART.
};

constexpr HostBaudAction ClassifyHostBaud(uint32_t baud) {
    if (baud == 0) return HostBaudAction::kIgnore;
    if (baud == kRebootToBootselBaud) return HostBaudAction::kRebootToBootsel;
    return HostBaudAction::kApply;
}
