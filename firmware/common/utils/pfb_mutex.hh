#ifndef PFB_MUTEX_HH_
#define PFB_MUTEX_HH_

// Platform abstraction for mutex locks used by PFBQueue.
// To use a different platform's mutex implementation, replace this file or
// define PFB_MUTEX_CUSTOM before including data_structures.hh and provide
// your own implementations of the macros below.

#ifndef PFB_MUTEX_CUSTOM

#if __has_include("pico/mutex.h")
// Pico SDK mutex implementation
#include "pico/mutex.h"

#define PFB_MUTEX_TYPE mutex_t
#define PFB_MUTEX_INIT(mtx) mutex_init(&(mtx))
#define PFB_MUTEX_LOCK(mtx) mutex_enter_blocking(&(mtx))
#define PFB_MUTEX_UNLOCK(mtx) mutex_exit(&(mtx))

#elif __has_include("freertos/semphr.h") && defined(ESP_PLATFORM)
// FreeRTOS spinlock implementation (ESP32 / IDF). The lock is held only briefly. A sleeping mutex held by a preempted
// low-priority task could stall the SPI receive task long enough for the RP2040 to time out.
#include "freertos/FreeRTOS.h"

#define PFB_MUTEX_TYPE        portMUX_TYPE
#define PFB_MUTEX_INIT(mtx)   portMUX_INITIALIZE(&(mtx))
#define PFB_MUTEX_LOCK(mtx)   portENTER_CRITICAL(&(mtx))
#define PFB_MUTEX_UNLOCK(mtx) portEXIT_CRITICAL(&(mtx))

#elif __has_include("freertos/semphr.h")
// FreeRTOS mutex implementation (other FreeRTOS ports).
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define PFB_MUTEX_TYPE        SemaphoreHandle_t
#define PFB_MUTEX_INIT(mtx)   ((mtx) = xSemaphoreCreateMutex())
#define PFB_MUTEX_LOCK(mtx)   xSemaphoreTake((mtx), portMAX_DELAY)
#define PFB_MUTEX_UNLOCK(mtx) xSemaphoreGive((mtx))

#else
// Stub implementation for platforms without mutex support (single-core/non-concurrent use).

struct PfbMutexStub {};
#define PFB_MUTEX_TYPE PfbMutexStub
#define PFB_MUTEX_INIT(mtx) ((void)0)
#define PFB_MUTEX_LOCK(mtx) ((void)0)
#define PFB_MUTEX_UNLOCK(mtx) ((void)0)

#endif  // platform

#endif  // PFB_MUTEX_CUSTOM

#endif  // PFB_MUTEX_HH_
