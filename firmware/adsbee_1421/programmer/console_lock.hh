#pragma once

#include <stdint.h>

#include "at_client.hh"

// Finds the ADSBee 1421 console's baud rate with the autobaud trigger (README "Console baud rate"): the Programmer
// holds the module's console RX line (its UART TX, GP28) low through a reset or a SYNC wake and releases it, and module
// firmware that supports the trigger answers with "UU" at its console rate once the console is ready. A PIO state
// machine timestamps the edges on the Programmer's RX pin (autobaud_edges.pio) and Autobaud::Measure() computes the
// rate. One AT+BAUD_RATE? probe at the measured rate confirms it and reads the exact nominal rate.
//
// Module firmware without the trigger (0.3.11-rc3 and earlier) sends nothing; the Programmer then probes the five rates
// those images accept (kLegacyConsoleBauds), once each.

enum class ConsoleTrigger {
    kReset,     // Reset into the application (SYNC low). The console comes back at its saved rate.
    kSyncWake,  // Pulse SYNC high (a sleep request) and release it. Keeps the module's live settings.
};

// Brings the module through `trigger` and finds the console. Leaves the Programmer's UART at the console's rate and
// returns it, or returns 0 if the console wasn't found or abort() returned true (abort is polled while waiting).
uint32_t ConsoleLock(ConsoleTrigger trigger, AtAbortFn abort = nullptr);

// Details of the last ConsoleLock(), for status messages.
struct ConsoleLockInfo {
    uint32_t measured_baud = 0;  // From the "UU" (0: no "UU" seen; the legacy probe pass ran).
    uint32_t console_baud = 0;   // As confirmed by the console (0: not found).
    uint32_t elapsed_ms = 0;     // From the start of the trigger.
};
const ConsoleLockInfo& LastConsoleLock();

// Moves the console to target_baud (0: only find it) and leaves the Programmer's UART at the console's rate. `likely`
// lists where the console probably is (0s are skipped); each gets one AT exchange. If none answers, the console is
// found with ConsoleLock(kSyncWake). Returns the console's rate afterwards: target_baud on success, another rate if
// the console was found but would not move, 0 if it wasn't found (or abort() returned true), in which case the UART is
// left at target_baud (if nonzero).
uint32_t ConsoleRenegotiate(uint32_t target_baud, const uint32_t* likely, size_t num_likely, AtAbortFn abort = nullptr);
