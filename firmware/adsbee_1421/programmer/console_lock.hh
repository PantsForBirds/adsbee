#pragma once

#include <stdint.h>

#include "at_client.hh"
#include "rate_watch.hh"

// Finds and follows the console's baud rate: the RateWatch the bridge polls, and the glue that applies its actions.

// Starts the edge capture. Call once after TargetUartInit().
void ConsoleWatchInit();

RateWatch& ConsoleWatch();

// Carries out a RateWatch action on the UART.
void ConsoleWatchApply(const RateWatch::Action& action);

// Resets the module into the app and locks onto its boot "UU", sending breaks if it doesn't come. Returns the console's
// rate (the UART is left there), or 0 if not found or aborted. Console output meanwhile is dropped.
uint32_t ConsoleLock(AtAbortFn abort = nullptr);
