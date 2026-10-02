#pragma once

#include <stdint.h>

#include "at_client.hh"
#include "rate_watch.hh"

// Finds and follows the ADSBee 1421 console's baud rate (rate_watch.hh): the RateWatch that bridge.cc polls during
// pass-through, the glue that carries out its actions on the UART, and blocking locks for the states around it.

// Starts the edge capture. Call once after TargetUartInit().
void ConsoleWatchInit();

RateWatch& ConsoleWatch();

// Carries out a RateWatch action on the UART.
void ConsoleWatchApply(const RateWatch::Action& action);

// Resets the module into the application and locks onto its boot "UU" (asking with a break if it doesn't come).
// Leaves the Programmer's UART at the console's rate and returns it, or returns 0 if the console wasn't found (the asks
// ran out) or abort() returned true (abort is polled while waiting). Console output received meanwhile is dropped.
uint32_t ConsoleLock(AtAbortFn abort = nullptr);
