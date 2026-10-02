#pragma once

#include <stdint.h>

// Console autobaud (README "Console autobaud"). The console says "UU" (0x55 0x55) at its current rate:
//   - right after every rate change (AT+BAUD_RATE=CONSOLE,<n>, AT+SETTINGS=RESET, settings applied at boot), after the
//     OK that still goes out at the old rate,
//   - at every boot, once SettingsManager::Apply() has set the saved rate,
//   - when its RX line sees a break (held low for longer than a frame at the console's rate), as soon as the main loop
//     notices. Queued console output is dropped first: a host that sends a break doesn't know the rate, so it can't
//     read that output anyway, and draining it first takes seconds at low rates.
// A 'U' framed 8N1 is a square wave with an edge at every bit (start 0, data 1 0 1 0 1 0 1 0, stop 1), and "UU" back
// to back is 20 bits of it, so a host times the edges and has the rate, the way LIN slaves use the 0x55 sync field
// after a break and many MCU UARTs auto-baud on 0x55. The ADSBee 1421 Programmer does this
// (firmware/adsbee_1421/programmer/rate_watch.hh).
//
// Pure logic with no SDK dependencies: comms.cpp uses it, and the Programmer's host tests run it in their model of
// the module.
namespace ConsoleAutobaud {

static constexpr char kAnswer[] = "UU";
static constexpr uint16_t kAnswerLen = sizeof(kAnswer) - 1;

// The console ignores NUL bytes. Each break leaves one in the input (the PL011 receives a break as a NUL with its
// break flag set; the UART2 driver's DMA copies only the data bits into its RX ring), and cppAT would end a line there,
// cutting a command the host is in the middle of sending. No AT command takes binary input.
//
// A break right after another, with no character in between, reaches the PL011 as a plain NUL with neither the break
// nor the framing error flag set (measured on the CC1314: the second of two breaks 200 ms apart goes unflagged), so it
// can't be told from a NUL byte and goes unanswered. Any character between them clears that: a host that may send a
// break twice sends a character (a NUL, which the console ignores) after the first one's answer, and before the next
// break. The ADSBee 1421 Programmer does both (programmer/rate_watch.hh).
inline bool IgnoredConsoleByte(char c) { return c == '\0'; }

}  // namespace ConsoleAutobaud
