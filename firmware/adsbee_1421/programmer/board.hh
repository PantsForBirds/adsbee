// Pin and timing constants for the ADSBee 1421 programmer jig (Waveshare RP2040-Zero).
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

static const uint32_t kResetPulseMs = 50;   // RESET_N low time, matches the python flasher.
static const uint32_t kBootWaitMs   = 800;  // App boot time before the first AT probe.

// The ROM bootloader auto-bauds to 1 M (proven on hardware; ROM ceiling ~1.2 M). The app
// console boots at its saved baud rate (persisted via AT+SETTINGS=SAVE; factory default 1 M),
// so it may be at any rate in the firmware's whitelist — probe kConsoleBaudCandidates
// (factory-default-first) to find it.
static const uint32_t kConsoleBaud = 1000000;  // Factory default / preferred pass-through rate.
static const uint32_t kConsoleBaudCandidates[] = {1000000, 921600, 460800, 230400, 115200};
static const uint32_t kBootloaderBaud = 1000000;
static const int kBootloaderEntryAttempts = 6;
