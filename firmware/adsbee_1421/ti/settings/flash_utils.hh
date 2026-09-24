#pragma once

#include "stdint.h"

// CC1314R10 (CC13X4) flash utilities for settings storage.
// Flash is memory-mapped at 0x00000000; all addresses are absolute.
class FlashUtils {
   public:
    static const uint32_t kFlashSectorSizeBytes = 0x800;           // 2 KB per sector on CC13X4
    static const uint32_t kFlashSettingsRegionSizeBytes = 0x2000;  // 8 KB (4 sectors) reserved per settings region

    /**
     * Disables interrupts before flash erase/program operations.
     */
    static void FlashSafe();

    /**
     * Restores interrupts after flash erase/program operations.
     */
    static void FlashUnsafe();

    /**
     * Erases kFlashSettingsRegionSizeBytes of flash starting at addr.
     * addr must be sector-aligned (multiple of kFlashSectorSizeBytes).
     * @retval True if every sector erased, false on the first failure (remaining sectors are skipped).
     */
    static bool EraseRegion(uint32_t addr);

    /**
     * Erases a single kFlashSectorSizeBytes flash sector at addr.
     * addr must be sector-aligned (multiple of kFlashSectorSizeBytes).
     * @retval True if the sector erased successfully.
     */
    static bool EraseSector(uint32_t addr);

    /**
     * Programs size bytes from data into flash at addr.
     * addr must be 4-byte aligned.
     * @retval True if the program operation reported success.
     */
    static bool Program(uint32_t addr, const uint8_t* data, uint32_t size);

   private:
    static uint32_t stored_key_;
};
