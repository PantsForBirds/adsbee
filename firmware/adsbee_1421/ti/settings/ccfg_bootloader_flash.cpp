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
// An erase-path write left the sector in an unknown state; original_sector holds what must go back and
// restore_len_bytes how much of it to program.
static bool restore_pending = false;
static uint32_t restore_len_bytes = 0;

static const uint8_t* CcfgFlash() { return reinterpret_cast<const uint8_t*>(kCcfgBaseAddr); }

uint32_t ReadBlConfig() { return ReadWord(CcfgFlash(), kBlConfigOffset); }

bool RestorePending() { return restore_pending; }

Plan Prepare(bool enable) {
    if (restore_pending) {
        // The flash no longer holds original_sector, so don't plan from it: write the original back first.
        Plan plan;
        memcpy(new_sector, original_sector, kCcfgSectorSizeBytes);
        plan.method = Method::kEraseAndProgram;
        plan.old_bl_config = ReadBlConfig();
        plan.new_bl_config = ReadWord(original_sector, kBlConfigOffset);
        plan.program_len_bytes = restore_len_bytes;
        plan.restore_retry = true;
        prepared = true;
        return plan;
    }
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

// WriteAndVerify()'s flash operations. FlashSectorErase() and FlashProgram() run from ROM (driverlib rom.h), so
// they may operate on bank 0 while the application executes from it, as long as no interrupt handler in flash runs
// in the meantime (Apply() masks interrupts).
struct RomFlash {
    bool Erase() { return FlashSectorErase(kCcfgBaseAddr) == FAPI_STATUS_SUCCESS; }
    bool Program(const uint8_t* src, uint32_t offset, uint32_t len) {
        // FlashProgram() can't take its source from flash; every caller passes a RAM buffer.
        return FlashProgram(const_cast<uint8_t*>(src), kCcfgBaseAddr + offset, len) == FAPI_STATUS_SUCCESS;
    }
    const uint8_t* Read() { return CcfgFlash(); }
};

WriteReport Apply(const Plan& plan, bool enable) {
    WriteReport report;
    report.enable = enable;
    report.method = plan.method;
    report.restore_retry = plan.restore_retry;
    report.old_bl_config = plan.old_bl_config;
    report.new_bl_config = plan.new_bl_config;
    report.original_crc = StructCrc32(CcfgFlash());
    report.expected_crc = StructCrc32(new_sector);
    auto finish = [&report](WriteResult result) {
        report.result = result;
        report.readback_bl_config = ReadBlConfig();
        report.readback_crc = StructCrc32(CcfgFlash());
        return report;
    };

    if (!prepared) {
        report.error = "no prepared CCFG image";
        return finish(WriteResult::kNotWritten);
    }
    switch (plan.method) {
        case Method::kNoChange:
            prepared = false;
            return finish(WriteResult::kOk);
        case Method::kRefused:
            report.error = plan.error ? plan.error : "refused";
            return finish(WriteResult::kNotWritten);
        default:
            break;
    }
    // CCFG ERASE_CONF_1.WEPROT_CCFG_N (latched into FLASH WEPROT_AUX_BY1 at boot) can lock the
    // sector until the next chip erase. The release CCFG leaves it unlocked.
    if (FlashProtectionGet(kCcfgBaseAddr) == FLASH_WRITE_PROTECT) {
        report.error = "CCFG sector is write protected";
        return finish(WriteResult::kNotWritten);
    }
    // The flash must still match the image Prepare() planned from. A restore retry writes over whatever the
    // failed write left.
    if (!plan.restore_retry && memcmp(CcfgFlash(), original_sector, kCcfgSectorSizeBytes) != 0) {
        report.error = "CCFG changed since it was read";
        return finish(WriteResult::kNotWritten);
    }

    uint32_t cache_mode = DisableFlashCache();
    FlashUtils::FlashSafe();
    RomFlash flash;
    WriteResult result =
        WriteAndVerify(flash, plan, original_sector, new_sector, report.restore_attempts, report.error);
    FlashUtils::FlashUnsafe();
    RestoreFlashCache(cache_mode);

    // Keep the original image in RAM while the sector is in an unknown state, so the next Prepare() can put it
    // back; otherwise the RAM images no longer describe the flash after a write attempt.
    restore_pending = result == WriteResult::kFailedNotRestored;
    if (restore_pending) restore_len_bytes = plan.program_len_bytes;
    prepared = false;
    return finish(result);
}

}  // namespace CcfgBootloader
