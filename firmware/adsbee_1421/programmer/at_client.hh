#pragma once

#include <stddef.h>
#include <stdint.h>

// Minimal AT-command client for the adsbee_1421 console: the version readout at startup. Finding the console's rate is
// console_lock.hh's job. The Programmer never changes the console's rate or settings itself.

// Optional abort check, polled while waiting (e.g. the host asked for a reset meanwhile). nullptr: never abort.
typedef bool (*AtAbortFn)();

// AT+DEVICE_INFO? is a silent-success query (no OK): output is collected until a quiet period,
// then scanned for "CC1314R10 Firmware Version: <ver>". Status prints only.
bool AtQueryVersion(char* version_out, size_t max_len);
