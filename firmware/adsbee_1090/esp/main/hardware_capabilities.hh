#pragma once

#include <cstdint>

#include "esp_heap_caps.h"

/**
 * Runtime detection of ESP32-S3 module hardware features that differ between ADSBee hardware revisions.
 *
 * One firmware image runs on both supported modules:
 *   - ESP32-S3-MINI-1U-N8:   8 MB flash, no PSRAM (original ADSBee 1090U).
 *   - ESP32-S3-MINI-1U-N4R2: 4 MB flash, 2 MB quad PSRAM (later hardware revisions).
 * The image is built with CONFIG_SPIRAM + CONFIG_SPIRAM_IGNORE_NOTFOUND, so the bootloader/startup code probes for PSRAM
 * and silently continues without it when the chip is absent. Detect() records what was actually found; features that
 * need more RAM than the internal SRAM can spare (e.g. Remote ID alongside WiFi AP/STA) gate on HasPSRAM().
 *
 * The result is published to the RP2040 in ObjectDictionary::ESP32DeviceStatus::hardware_capabilities so it can give
 * the user a clear error when a setting needs hardware the board doesn't have.
 */
class HardwareCapabilities {
   public:
    // Minimum PSRAM heap to count as "has PSRAM" for RAM-hungry features. A 2 MB part shows up as slightly less than
    // 2 MB of heap (the allocator reserves a little), so don't insist on the full 2 MB.
    static constexpr uint32_t kMinPSRAMHeapBytes = 1536 * 1024;

    // Heap capabilities describing internal SRAM only. Heap guards that protect the network stack must use this rather
    // than plain MALLOC_CAP_8BIT: with PSRAM present MALLOC_CAP_8BIT also counts ~2 MB of PSRAM and would hide an
    // internal-RAM shortage (DMA buffers, the BLE controller and task stacks always live in internal RAM). On a module
    // without PSRAM the two are identical.
    static constexpr uint32_t kInternalHeapCaps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;

    /**
     * Probes the hardware. Call once, early in app_main(), before anything consults HasPSRAM().
     */
    static void Detect();

    /**
     * Returns true if usable PSRAM was found and added to the heap (at least kMinPSRAMHeapBytes).
     */
    static bool HasPSRAM() { return has_psram_; }

    /**
     * Total PSRAM heap size in bytes (0 if no PSRAM).
     */
    static uint32_t GetPSRAMTotalBytes() { return psram_total_bytes_; }

    /**
     * Free PSRAM heap in bytes (0 if no PSRAM).
     */
    static uint32_t GetPSRAMFreeBytes() { return has_psram_ ? heap_caps_get_free_size(MALLOC_CAP_SPIRAM) : 0; }

    /**
     * Free internal-SRAM heap in bytes. Use this for heap guards (see kInternalHeapCaps).
     */
    static uint32_t GetInternalFreeBytes() { return heap_caps_get_free_size(kInternalHeapCaps); }

    /**
     * Bitfield for ObjectDictionary::ESP32DeviceStatus::hardware_capabilities.
     */
    static uint8_t GetCapabilitiesBitfield();

   private:
    static bool has_psram_;
    static uint32_t psram_total_bytes_;
};
