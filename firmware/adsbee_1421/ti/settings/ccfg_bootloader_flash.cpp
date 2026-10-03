#include "ccfg_bootloader_flash.hh"

#include <ti/devices/cc13x4_cc26x4/driverlib/flash.h>
#include <ti/devices/cc13x4_cc26x4/driverlib/vims.h>
#include <ti/devices/cc13x4_cc26x4/inc/hw_memmap.h>

#include "flash_utils.hh"

static_assert(CcfgBootloader::kCcfgBaseAddr == CCFG_BASE, "CCFG base address mismatch.");
static_assert(CcfgBootloader::kCcfgSectorSizeBytes == FlashUtils::kFlashSectorSizeBytes,
              "CCFG sector size mismatch.");

namespace CcfgBootloader {

// RAM images of the CCFG before and after the change; FlashProgram() can't read its source from flash.
static uint8_t original_sector[kCcfgSectorSizeBytes] __attribute__((aligned(4)));
static uint8_t new_sector[kCcfgSectorSizeBytes] __attribute__((aligned(4)));
static bool prepared = false;
// Set when the sector is in an unknown state and original_sector must be written back.
static bool restore_pending = false;
static uint32_t restore_len_bytes = 0;

static const uint8_t* CcfgFlash() { return reinterpret_cast<const uint8_t*>(kCcfgBaseAddr); }

uint32_t ReadBlConfig() { return ReadWord(CcfgFlash(), kBlConfigOffset); }

bool RestorePending() { return restore_pending; }

Plan Prepare(bool enable) {
    if (restore_pending) {
        // Write the original back before any new change.
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
// TI requires the flash cache (VIMS) off while erasing or programming; it also makes read-back come from flash.
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

// These ROM functions can write the flash bank we execute from, as long as interrupts are masked.
struct RomFlash {
    bool Erase() { return FlashSectorErase(kCcfgBaseAddr) == FAPI_STATUS_SUCCESS; }
    bool Program(const uint8_t* src, uint32_t offset, uint32_t len) {
        // src must be in RAM.
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
    // CCFG can lock itself until the next chip erase (ERASE_CONF_1.WEPROT_CCFG_N); release images don't.
    if (FlashProtectionGet(kCcfgBaseAddr) == FLASH_WRITE_PROTECT) {
        report.error = "CCFG sector is write protected";
        return finish(WriteResult::kNotWritten);
    }
    // Flash must still match what Prepare() read, except on a restore retry.
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

    // Keep the original in RAM while the sector is in an unknown state so the next Prepare() can restore it.
    restore_pending = result == WriteResult::kFailedNotRestored;
    if (restore_pending) restore_len_bytes = plan.program_len_bytes;
    prepared = false;
    return finish(result);
}

}  // namespace CcfgBootloader
