#pragma once

#include <stddef.h>
#include <stdint.h>

// Minimal AT-command client for the adsbee_1421 console: liveness probe, optional version readout, and console baud
// negotiation. Finding the console's rate is console_lock.hh's job; the Programmer's UART is retuned with
// TargetUartSetBaud() as needed.

// Optional abort check, polled while waiting (e.g. the host asked for a reset meanwhile). nullptr: never abort.
typedef bool (*AtAbortFn)();

// AT+BAUD_RATE? at the UART's current rate. Returns the rate in the console's "BAUD_RATE=CONSOLE,<baud>" answer, or 0
// if none came within the probe timeout (a base for the console's main loop plus a share of its TX ring, which the
// answer queues behind, capped at 1 s). The answer is read to its end, so none of it reaches the host afterwards.
uint32_t AtProbeConsoleBaud();
inline bool AtProbeAlive() { return AtProbeConsoleBaud() != 0; }

// AT+DEVICE_INFO? is a silent-success query (no OK): output is collected until a quiet period,
// then scanned for "CC1314R10 Firmware Version: <ver>". Status prints only.
bool AtQueryVersion(char* version_out, size_t max_len);

// Sends AT+BAUD_RATE=CONSOLE,<baud> and waits up to 1 s for the OK (sent at the old baud, after any output already
// queued). On success the device has already switched; the caller must retune the Programmer's UART. The change is
// live-only: on any device reset the console returns to its saved rate (the Programmer never issues
// AT+SETTINGS=SAVE, so it never alters the persisted setting).
bool AtSetConsoleBaud(uint32_t baud);
