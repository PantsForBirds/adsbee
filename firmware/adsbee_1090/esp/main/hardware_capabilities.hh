#pragma once

#include <cstdint>

#include "esp_heap_caps.h"

/**
 * Detects whether the ESP32-S3 module has PSRAM (ESP32-S3-MINI-1U-N4R2 does, -N8 doesn't). RAM-hungry features, such as
 * Remote ID alongside WiFi, gate on HasPSRAM(). The result is sent to the RP2040 so it can reject unsupported settings.
 */
class HardwareCapabilities {
   public:
    // A 2 MB PSRAM chip shows up as slightly less than 2 MB of heap.
    static constexpr uint32_t kMinPSRAMHeapBytes = 1536 * 1024;

    // Internal SRAM only. Heap guards must use this, not MALLOC_CAP_8BIT, which also counts PSRAM and would hide an
    // internal RAM shortage (DMA buffers, BLE and task stacks need internal RAM).
    static constexpr uint32_t kInternalHeapCaps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;

    /**
     * Call once, early in app_main(), before anything calls HasPSRAM().
     */
    static void Detect();

    /**
     * True if at least kMinPSRAMHeapBytes of PSRAM heap was found.
     */
    static bool HasPSRAM() { return has_psram_; }

    /**
     * Total PSRAM heap size in bytes (0 if no PSRAM).
     */
    static size_t GetPSRAMTotalBytes() { return psram_total_bytes_; }

    /**
     * Free PSRAM heap in bytes (0 if no PSRAM).
     */
    static size_t GetPSRAMFreeBytes() { return has_psram_ ? heap_caps_get_free_size(MALLOC_CAP_SPIRAM) : 0; }

    /**
     * Free internal SRAM heap in bytes. Use this for heap guards.
     */
    static size_t GetInternalFreeBytes() { return heap_caps_get_free_size(kInternalHeapCaps); }

    /**
     * Bitfield for ObjectDictionary::ESP32DeviceStatus::hardware_capabilities.
     */
    static uint8_t GetCapabilitiesBitfield();

   private:
    static bool has_psram_;
    static size_t psram_total_bytes_;
};
