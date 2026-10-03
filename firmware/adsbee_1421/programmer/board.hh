// Pin and timing constants for the ADSBee 1421 Programmer (Waveshare RP2040-Zero).
//
// Wiring to the ADSBee m1421 module:
//   GP28 (UART0 TX) -> SURX  (pin 20, CC1314 DIO_2, UART RX)
//   GP29 (UART0 RX) <- SUTX  (pin 21, CC1314 DIO_3, UART TX)
//   GP27            -> SYNC  (pin 28, CC1314 DIO_5, bootloader backdoor, ACTIVE HIGH;
//                             the module has a pull-down so the line idles low)
//   GP26            -> ~SRST (pin 17, CC1314 RESET_N, active low; module pull-up)
//   GND             -> GND
//
// GP28/GP29 are the only UART0-capable pins in the GP26-29 group, which fixes the UART
// assignment; SYNC/RESET_N take the remaining two.

#pragma once

#include "pico/stdlib.h"

static const uint kPinResetN = 26;  // ~SRST, active low. Pseudo open-drain (hi-Z when released).
static const uint kPinSync   = 27;  // SYNC / DIO_5 backdoor, active high, push-pull.
static const uint kPinUartTx = 28;  // UART0 TX -> module SURX (DIO_2).
static const uint kPinUartRx = 29;  // UART0 RX <- module SUTX (DIO_3).

static const uint32_t kResetPulseMs = 50;   // RESET_N low time; host tools use the same pulse.

// The ROM bootloader auto-bauds to 1 M (ROM ceiling ~1.2 M). The app console boots at its saved baud rate (factory
// default 1 M), which ConsoleLock() finds.
static constexpr uint32_t kConsoleBaud = 1000000;  // Factory default / preferred pass-through rate.
static constexpr uint32_t kBootloaderBaud = 1000000;
// The host baud that reboots the Programmer into its USB bootloader is kRebootToBootselBaud (host_line_coding.hh).
static const int kBootloaderEntryAttempts = 6;
