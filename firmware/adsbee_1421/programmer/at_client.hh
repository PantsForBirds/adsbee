#pragma once

#include <stddef.h>
#include <stdint.h>

// Minimal AT-command client for the adsbee_1421 console: liveness probe, optional version readout, console baud
// detection and renegotiation. The Programmer's UART is retuned with TargetUartSetBaud() as needed.

// Optional abort check, polled between probes (e.g. the host asked for a reset meanwhile). nullptr: never abort.
typedef bool (*AtAbortFn)();

// AT+BAUD_RATE? answered with its one "BAUD_RATE=CONSOLE,<baud>" line at the UART's current rate. The answer is read
// to its end, so none of it reaches the host afterwards. timeout_ms 0 picks AtProbeTimeoutMs() for the current rate.
bool AtProbeAlive(uint32_t timeout_ms = 0);
// How long a probe at `baud` waits: a base for the console's main loop plus a share of its TX ring (an answer queues
// behind report output already in it), capped so a sweep through wrong rates stays bounded.
uint32_t AtProbeTimeoutMs(uint32_t baud);

// Finds the app console's baud rate by probing each candidate in order (retuning the Programmer's UART per try).
// Returns the rate that answered, or 0 if none did or abort() returned true; the UART is left at the returned rate
// (or the last candidate tried). Wrong-baud garbage can never match the probe token, so misses fail cleanly.
uint32_t AtFindConsoleBaud(const uint32_t* candidates, size_t num_candidates, AtAbortFn abort = nullptr);
// Same, over the default probe order: the rates the console was last seen at (RAM, then the Programmer's flash, see
// baud_store.hh), then kCommonConsoleBauds.
uint32_t AtFindConsoleBaud();

// AT+DEVICE_INFO? is a silent-success query (no OK): output is collected until a quiet period,
// then scanned for "CC1314R10 Firmware Version: <ver>". Status prints only.
bool AtQueryVersion(char* version_out, size_t max_len);

// Sends AT+BAUD_RATE=CONSOLE,<baud> and waits for the OK (sent at the old baud, after any output already queued). On
// success the device has already switched; the caller must retune the Programmer's UART. The change is live-only: on
// any device reset the console returns to its saved rate (the Programmer never issues AT+SETTINGS=SAVE, so it never
// alters the persisted setting).
bool AtSetConsoleBaud(uint32_t baud);

// Moves the console to target_baud (0: only find it) and leaves the Programmer's UART at the console's rate.
// `likely` lists where the console probably is, most likely first; the full default probe order follows if none of
// them answers. Returns the console's rate afterwards: target_baud on success, another rate if the console was found
// but would not move, 0 if it was not found (or abort() returned true), in which case the UART is left at target_baud
// (if nonzero).
uint32_t AtRenegotiateConsole(uint32_t target_baud, const uint32_t* likely, size_t num_likely,
                              AtAbortFn abort = nullptr);

// AtFindConsoleBaud() probes the rate the console last answered at (any probe) and the rate it boots at first.
// AtNoteBootBaud() records the boot rate (the console's saved rate) and persists it in the Programmer's flash when it
// changed.
void AtNoteBootBaud(uint32_t baud);
uint32_t AtBootBaud();  // Last known boot rate (RAM, else flash), 0 if unknown.
