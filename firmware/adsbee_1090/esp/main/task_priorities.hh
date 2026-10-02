#ifndef TASK_PRIORITIES_HH_
#define TASK_PRIORITIES_HH_

#include <stdint.h>

#include "freertos/task.h"

// This will cause weird crashes if it's too small to support full size SPI transfers!
static const unsigned int kSPIReceiveTaskStackSizeBytes = 1 * 4096;
// Above the W5500 Ethernet RX task (w5500_tsk, IDF default priority 15, no core affinity), the only task that can
// otherwise preempt this one on core 1. The RP2040 busy-waits for each response, so a preempted SPI receive task
// costs RP2040 time and, before responses queued the next receive, dropped RP2040 packets.
static const unsigned int kSPIReceiveTaskPriority = 16;
static const unsigned int kSPIReceiveTaskCore = 1;
static const unsigned int kWiFiAPTaskPriority = tskIDLE_PRIORITY;
// Task stacks come out of internal SRAM, which WiFi AP+STA leaves short on modules without PSRAM. Sizes below leave
// about 2 KB or more of headroom over the peak use measured with the web UI open, feeds running and a web UI OTA
// update (build with HEAP_DIAGNOSTICS to see high-water marks).
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