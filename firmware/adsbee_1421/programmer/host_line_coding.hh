#pragma once

#include <stdint.h>

#include "console_baud.hh"  // Shared with the module firmware (ti/comms).

// What the Programmer does with the baud rate the host sets (USB CDC SET_LINE_CODING). Only the magic baud below
// does anything. Pure logic, host-tested in host_test/test_host_line_coding.cc.

// Host baud that reboots the Programmer into its USB bootloader (RPI-RP2). Same as the ADSBee 1090's
// PICO_STDIO_USB_RESET_MAGIC_BAUD_RATE; pico-sdk's default, 1200, is used by pymavlink so it can't be the magic baud.
static constexpr uint32_t kRebootToBootselBaud = 0xDEADBEE;  // 233495534 baud.

enum class HostBaudAction {
    kIgnore,           // Any rate: the host's line coding doesn't reach the UART.
    kRebootToBootsel,  // kRebootToBootselBaud: reboot the Programmer.
};

constexpr HostBaudAction ClassifyHostBaud(uint32_t baud) {
    return baud == kRebootToBootselBaud ? HostBaudAction::kRebootToBootsel : HostBaudAction::kIgnore;
}

// The Programmer's UART0 runs from clk_peri, which pico-sdk leaves at clk_sys (125 MHz on the RP2040).
static constexpr uint32_t kProgrammerUartClockHz = 125000000;

constexpr uint32_t ProgrammerActualBaud(uint32_t baud) {
    return ConsoleBaud::Pl011ActualBaud(kProgrammerUartClockHz, baud);
}
