#include "ccfg_bootloader_flash.hh"

#include <ti/devices/cc13x4_cc26x4/driverlib/flash.h>
#include <ti/devices/cc13x4_cc26x4/driverlib/vims.h>
#include <ti/devices/cc13x4_cc26x4/inc/hw_memmap.h>

#include "flash_utils.hh"

static_assert(CcfgBootloader::kCcfgBaseAddr == CCFG_BASE, "CCFG base address mismatch.");
static_assert(CcfgBootloader::kCcfgSectorSizeBytes == FlashUtils::kFlashSectorSizeBytes,
              "CCFG sector size mismatch.");

namespace CcfgBootloader {

// RAM images of the CCFG sector: what it held at Prepare() time and what it must hold afterwards.
// FlashProgram() can't take its source from flash, and the original copy is what a failed erase
// path restores. Word aligned for FlashProgram().
static uint8_t original_sector[kCcfgSectorSizeBytes] __attribute__((aligned(4)));
static uint8_t new_sector[kCcfgSectorSizeBytes] __attribute__((aligned(4)));
static bool prepared = false;

static const uint8_t* CcfgFlash() { return reinterpret_cast<const uint8_t*>(kCcfgBaseAddr); }

uint32_t ReadBlConfig() { return ReadWord(CcfgFlash(), kBlConfigOffset); }

Plan Prepare(bool enable) {
    memcpy(original_sector, CcfgFlash(), kCcfgSectorSizeBytes);
    Plan plan = PlanUpdate(original_sector, new_sector, enable);
    prepared = plan.method != Method::kRefused;
    return plan;
}

// TI requires the VIMS cache and line buffer off while flash is erased or programmed (driverlib
// flash.h, NVSCC26XX disableFlashCache()); it also makes the read-back below come from flash.
static uint32_t DisableFlashCache() {
    uint32_t mode = VIMSModeGet(VIMS_BASE);
    VIMSLineBufDisable(VIMS_BASE);
    if (mode != VIMS_MODE_DISABLED) {
        VIMSModeSet(VIMS_BASE, VIMS_MODE_DISABLED);
        while (VIMSModeGet(VIMS_BASE) != VIMS_MODE_DISABLED) {
        }
    }
    return mode;
}

static void RestoreFlashCache(uint32_t mode) {
    if (mode != VIMS_MODE_DISABLED) {
        VIMSModeSet(VIMS_BASE, VIMS_MODE_ENABLED);
    }
    VIMSLineBufEnable(VIMS_BASE);
}

// Erases the CCFG sector and programs image[0, len). FlashSectorErase() and FlashProgram() run from
// ROM (driverlib rom.h), so they may operate on bank 0 while the application executes from it, as
// long as no interrupt handler in flash runs in the meantime (the caller masks interrupts).
static bool EraseAndProgram(uint8_t* image, uint32_t len) {
    if (FlashSectorErase(kCcfgBaseAddr) != FAPI_STATUS_SUCCESS) return false;
    if (FlashProgram(image, kCcfgBaseAddr, len) != FAPI_STATUS_SUCCESS) return false;
    return memcmp(CcfgFlash(), image, kCcfgSectorSizeBytes) == 0;
}

bool Apply(const Plan& plan, const char*& error, bool& restored) {
    error = nullptr;
    restored = false;
    if (!prepared) {
        error = "no prepared CCFG image";
        return false;
    }
    switch (plan.method) {
        case Method::kNoChange:
            return true;
        case Method::kRefused:
            error = plan.error ? plan.error : "refused";
            return false;
        default:
            break;
    }
    // CCFG ERASE_CONF_1.WEPROT_CCFG_N (latched into FLASH WEPROT_AUX_BY1 at boot) can lock the
    // sector until the next chip erase. The release CCFG leaves it unlocked.
    if (FlashProtectionGet(kCcfgBaseAddr) == FLASH_WRITE_PROTECT) {
        error = "CCFG sector is write protected";
        return false;
    }
    // The flash must still match the image Prepare() planned from.
    if (memcmp(CcfgFlash(), original_sector, kCcfgSectorSizeBytes) != 0) {
        error = "CCFG changed since it was read";
        return false;
    }

    bool success = false;
    uint32_t cache_mode = DisableFlashCache();
    FlashUtils::FlashSafe();
    if (plan.method == Method::kProgramWord) {
        // Clearing bits only: one in-place program of the word, the sector is never erased.
        uint32_t word = plan.new_bl_config;
        success = FlashProgram(reinterpret_cast<uint8_t*>(&word), kCcfgBaseAddr + kBlConfigOffset, sizeof(word)) ==
                      FAPI_STATUS_SUCCESS &&
                  memcmp(CcfgFlash(), new_sector, kCcfgSectorSizeBytes) == 0;
        if (!success) error = "BL_CONFIG program or verify failed";
    } else {
        success = EraseAndProgram(new_sector, plan.program_len_bytes);
        if (!success) {
            error = "CCFG erase, program or verify failed";
            // Never leave the sector erased or half written: put the original back. Both images
            // differ only in BL_CONFIG, so they need the same program length.
            for (int attempt = 0; attempt < kRestoreAttempts && !restored; attempt++) {
                restored = EraseAndProgram(original_sector, plan.program_len_bytes);
            }
        }
    }
    FlashUtils::FlashUnsafe();
    RestoreFlashCache(cache_mode);
    prepared = false;  // The RAM images no longer describe the flash after a write attempt.
    return success;
}

}  // namespace CcfgBootloader
