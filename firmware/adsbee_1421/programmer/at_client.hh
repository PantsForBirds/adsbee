#pragma once

#include <stddef.h>
#include <stdint.h>

// Minimal AT-command client for the adsbee_1421 console, used only after the device is
// confirmed up to date: liveness probe, optional version readout, and console baud negotiation.

bool AtProbeAlive(uint32_t timeout_ms = 500);  // AT+DEVICE_INFO? answered with a CC1314R10 line.

// Finds the app console's baud rate by probing each kConsoleBaudCandidates entry (retuning the
// jig UART per try). Returns the rate that answered, or 0 if the device never did; the jig UART
// is left at the returned rate (or the last candidate on failure). Wrong-baud garbage can never
// match the probe token, so misses fail cleanly after per_rate_timeout_ms.
uint32_t AtFindConsoleBaud(uint32_t per_rate_timeout_ms = 400);

// AT+DEVICE_INFO? is a silent-success query (no OK): output is collected until a quiet period,
// then scanned for "CC1314R10 Firmware Version: <ver>". Status prints only.
bool AtQueryVersion(char* version_out, size_t max_len);

// Sends AT+BAUD_RATE=CONSOLE,<baud> and waits for the OK (sent at the old baud). On success the
// device has already switched; the caller must retune the jig UART. The change is live-only —
// on any device reset the console returns to its saved rate (the jig never issues
// AT+SETTINGS=SAVE, so it never alters the persisted setting).
bool AtSetConsoleBaud(uint32_t baud);
