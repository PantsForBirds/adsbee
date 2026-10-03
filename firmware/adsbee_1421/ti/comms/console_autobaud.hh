#pragma once

#include <stdint.h>

// The console sends "UU" at its current rate after every rate change, at boot, and when it receives a break, so a
// host can measure the rate from the bit edges. Pure logic, shared with the Programmer's host tests.
namespace ConsoleAutobaud {

static constexpr char kAnswer[] = "UU";
static constexpr uint16_t kAnswerLen = sizeof(kAnswer) - 1;

// Ignore NULs: each break arrives as one and would cut an AT command short. CC1314 quirk: a second break with no
// character since the first arrives as a plain NUL with no break flag, so hosts send a NUL before each break.
inline bool IgnoredConsoleByte(char c) { return c == '\0'; }

}  // namespace ConsoleAutobaud
