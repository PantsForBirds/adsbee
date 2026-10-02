#pragma once

#include <stddef.h>
#include <stdint.h>

#include "rate_watch.hh"

// Continuous edge timer on the Programmer's UART RX pin (GP29, the module's console TX). autobaud_edges.pio on PIO1
// (the WS2812 LED uses PIO0) reads the pin through the jmp pin while the UART keeps it, and pushes a timestamp at
// every edge; a DMA channel writes them into a 4096-entry ring, and a second channel re-arms the first whenever its
// transfer count runs out, so the capture never stops and never costs CPU time. RateWatch reads the ring only when
// it needs to (rate_watch.hh).
//
// The state machine starts at the pin's current level, and edges alternate from there, so an edge's polarity follows
// from its index: the ring size is even, and nothing is ever dropped (the DMA keeps up with the PIO FIFO).

void EdgeCaptureInit();  // Call once, after TargetUartInit() has given the pin to the UART.

// The EdgeSource RateWatch reads.
EdgeSource& EdgeCapture();
