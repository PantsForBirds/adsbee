#pragma once

#include <stddef.h>
#include <stdint.h>

// UART0 link to the ADSBee 1421 (console and ROM bootloader share the same pins).
//
// RX is interrupt-driven into a ring buffer sized to ride out ~60 ms of USB stall at 1 Mbaud;
// overflow drops the newest bytes and counts them. TX goes through a small software ring drained
// nonblocking so the bridge loop never stalls tud_task().

void TargetUartInit(uint32_t baud);
void TargetUartSetBaud(uint32_t baud);  // Also flushes stale RX input.

size_t TargetUartRead(uint8_t* buf, size_t max_len);  // Nonblocking, from the RX ring.
// Blocks up to timeout_ms for one byte, servicing USB (tud_task) and the status LED while
// waiting; returns -1 on timeout. The wait keeps USB alive through multi-second erase commands.
int TargetUartReadByteTimeout(uint32_t timeout_ms);

void TargetUartWriteBlocking(const uint8_t* data, size_t len);      // Protocol / AT phases.
size_t TargetUartWriteNonblocking(const uint8_t* data, size_t len); // Bridge phase; returns queued.
void TargetUartPumpTx();       // Drain the software TX ring into the hardware FIFO.
size_t TargetUartTxFree();     // Space left in the software TX ring.

void TargetUartFlushInput();
uint32_t TargetUartRxDropCount();
