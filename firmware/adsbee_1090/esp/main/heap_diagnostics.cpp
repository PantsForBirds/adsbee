#include "heap_diagnostics.hh"

#ifdef HEAP_DIAGNOSTICS

#include <stdio.h>
#include <string.h>

#include "comms.hh"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hardware_capabilities.hh"

namespace {
constexpr uint16_t kMaxNumMarks = 16;
constexpr uint16_t kMaxNumTasks = 40;
constexpr uint16_t kLineMaxLen = 300;

struct BootMark {
    const char* label;
    uint32_t free_bytes;
    uint32_t largest_block_bytes;
};
BootMark boot_marks[kMaxNumMarks];
uint16_t num_boot_marks = 0;
}  // namespace

void HeapDiagnostics::Mark(const char* label) {
    if (num_boot_marks >= kMaxNumMarks) return;
    boot_marks[num_boot_marks++] = {.label = label,
                                    .free_bytes = HardwareCapabilities::GetInternalFreeBytes(),
                                    .largest_block_bytes = (uint32_t)heap_caps_get_largest_free_block(
                                        HardwareCapabilities::kInternalHeapCaps)};
}

void HeapDiagnostics::Report() {
    char line[kLineMaxLen];
    int len = 0;

    // Boot marks, several per line.
    for (uint16_t i = 0; i < num_boot_marks; i++) {
        len += snprintf(line + len, sizeof(line) - len, "%s=%lu/%lu ", boot_marks[i].label,
                        (unsigned long)boot_marks[i].free_bytes, (unsigned long)boot_marks[i].largest_block_bytes);
        if (len > kLineMaxLen - 60 || i == num_boot_marks - 1) {
            CONSOLE_WARNING("heapdiag", "boot %s", line);
            len = 0;
        }
    }

    multi_heap_info_t info;
    heap_caps_get_info(&info, HardwareCapabilities::kInternalHeapCaps);
    CONSOLE_WARNING("heapdiag", "internal free=%u largest=%u min_ever=%u alloc=%u blocks=%u/%u dma_free=%u",
                    info.total_free_bytes, info.largest_free_block, info.minimum_free_bytes,
                    info.total_allocated_bytes, info.allocated_blocks, info.free_blocks,
                    heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));

    // Task stack high-water marks (bytes never used).
    TaskStatus_t* tasks = (TaskStatus_t*)heap_caps_malloc(kMaxNumTasks * sizeof(TaskStatus_t), MALLOC_CAP_8BIT);
    if (tasks == nullptr) {
        CONSOLE_WARNING("heapdiag", "No heap for the task list.");
        return;
    }
    UBaseType_t num_tasks = uxTaskGetSystemState(tasks, kMaxNumTasks, nullptr);
    len = 0;
    for (UBaseType_t i = 0; i < num_tasks; i++) {
        len += snprintf(line + len, sizeof(line) - len, "%s:%lu ", tasks[i].pcTaskName,
                        (unsigned long)tasks[i].usStackHighWaterMark);
        if (len > kLineMaxLen - 40 || i == num_tasks - 1) {
            CONSOLE_WARNING("heapdiag", "stack_hwm %s", line);
            len = 0;
        }
    }
    heap_caps_free(tasks);
}

#endif  // HEAP_DIAGNOSTICS
