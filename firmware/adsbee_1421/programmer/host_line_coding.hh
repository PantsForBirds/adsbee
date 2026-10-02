#pragma once

#include <stdint.h>

#include "console_baud.hh"  // firmware/adsbee_1421/ti/comms: the rates the ADSBee 1421 console accepts.

// Host line coding (CDC SET_LINE_CODING) -> what the Programmer does with the requested baud. Pure logic with no SDK
// dependencies so it can be host-tested (host_test/host_line_coding_test.cc); bridge.cc calls it from
// tud_cdc_line_coding_cb().

// Host baud that reboots the Programmer's own RP2040 into its USB bootloader (RPI-RP2), so the Programmer can be
// updated without pressing BOOT. The same magic baud as the ADSBee 1090: PICO_STDIO_USB_RESET_MAGIC_BAUD_RATE=0XDEADBEE
// in firmware/adsbee_1090/pico/CMakeLists.txt (host_test fails if the two drift apart). pico-sdk's default magic baud
// is 1200, a rate pymavlink and other tools open ports at as a Linux kernel workaround. The Programmer owns TinyUSB
// directly (no stdio_usb), so the SDK's own handler never runs and the baud is checked here. It is far above any UART
// rate, and bridge.cc static_asserts that it is not a console or bootloader rate.
static constexpr uint32_t kRebootToBootselBaud = 0xDEADBEE;  // 233495534 baud.

enum class HostBaudAction {
    kIgnore,           // Not a baud (0): leave the target UART alone.
    kRebootToBootsel,  // kRebootToBootselBaud: reboot the Programmer; never forwarded or remembered as the host baud.
    kApply,            // Anything else: the host's rate for the target UART.
};

constexpr HostBaudAction ClassifyHostBaud(uint32_t baud) {
    if (baud == 0) return HostBaudAction::kIgnore;
    if (baud == kRebootToBootselBaud) return HostBaudAction::kRebootToBootsel;
    return HostBaudAction::kApply;
}

// The Programmer's UART0 runs from clk_peri, which pico-sdk leaves at clk_sys (125 MHz on the RP2040).
static constexpr uint32_t kProgrammerUartClockHz = 125000000;

constexpr uint32_t ProgrammerActualBaud(uint32_t baud) {
    return ConsoleBaud::Pl011ActualBaud(kProgrammerUartClockHz, baud);
}

// A host rate the Programmer renegotiates the module console to (Option A in bridge.cc): one the console accepts and
// the Programmer's UART generates within ConsoleBaud::kMaxErrorPpm of the console's actual rate. Every other rate (the
// 1200 baud pymavlink opens ports at, rates below 9600 or above 3 M) is applied to the Programmer's UART directly.
constexpr bool IsRenegotiableBaud(uint32_t baud) {
    return ConsoleBaud::IsSupported(baud) &&
           ConsoleBaud::ErrorPpm(ConsoleBaud::CC1314ActualBaud(baud), ProgrammerActualBaud(baud)) <=
               ConsoleBaud::kMaxErrorPpm;
}
