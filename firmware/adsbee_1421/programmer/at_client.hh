#pragma once

#include <stddef.h>
#include <stdint.h>

// Minimal AT-command client for the adsbee_1421 console: reads the firmware version at startup.

// Optional abort check, polled while waiting. nullptr never aborts.
typedef bool (*AtAbortFn)();

// AT+DEVICE_INFO? is a silent-success query (no OK): output is collected until a quiet period,
// then scanned for "CC1314R10 Firmware Version: <ver>". Status prints only.
bool AtQueryVersion(char* version_out, size_t max_len);
