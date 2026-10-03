#pragma once

#include <stddef.h>
#include <stdint.h>

#include "rate_watch.hh"

// Timestamps every edge on the UART RX pin (the module's console TX) with PIO1 and DMA into a ring, with no CPU time.
// Edges alternate from the starting level, so an edge's polarity follows from its index.

void EdgeCaptureInit();  // Call once, after TargetUartInit() has given the pin to the UART.

// The EdgeSource RateWatch reads.
EdgeSource& EdgeCapture();
