#ifndef TASK_PRIORITIES_HH_
#define TASK_PRIORITIES_HH_

#include <stdint.h>

#include "freertos/task.h"

// This will cause weird crashes if it's too small to support full size SPI transfers!
static const unsigned int kSPIReceiveTaskStackSizeBytes = 1 * 4096;
// Above the W5500 Ethernet RX task (priority 15) so it can't preempt this one; the RP2040 busy-waits for responses.
static const unsigned int kSPIReceiveTaskPriority = 16;
static const unsigned int kSPIReceiveTaskCore = 1;
static const unsigned int kWiFiAPTaskPriority = tskIDLE_PRIORITY;
// Task stacks use internal SRAM, which is short without PSRAM. Sizes keep about 2 KB over peak use (build with
// HEAP_DIAGNOSTICS to see high-water marks).
static const unsigned int kWiFiAPTaskStackSizeBytes = 4096;
// static const unsigned int kWiFiAPTaskCore = 0;
static const unsigned int kIPWANTaskPriority = tskIDLE_PRIORITY;
static const unsigned int kIPWANTaskStackSizeBytes = 5120;
// static const unsigned int kIPWANTaskCore = 1;
static const unsigned int kTCPServerTaskPriority = tskIDLE_PRIORITY;
// static const unsigned int kTCPServerTaskCore = 0;
// Handles network console buffers but that happens in heap.
static const unsigned int kTCPServerTaskStackSizeBytes = 4096;
static const unsigned int kHTTPServerStackSizeBytes = 8192;

static const unsigned int kDeviceStatusUpdateTaskStackSizeBytes = 2048;
static const unsigned int kDeviceStatusUpdateTaskPriority = tskIDLE_PRIORITY;

#endif /* TASK_PRIORITIES_HH_ */