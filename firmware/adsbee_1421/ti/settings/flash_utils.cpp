#include "flash_utils.hh"

#include <ti/devices/cc13x4_cc26x4/driverlib/flash.h>
#include <ti/devices/cc13x4_cc26x4/driverlib/interrupt.h>

uint32_t FlashUtils::stored_key_ = 0;

void FlashUtils::FlashSafe() {
    stored_key_ = IntMasterDisable();
}

void FlashUtils::FlashUnsafe() {
    if (!stored_key_) {
        IntMasterEnable();
    }
}

// The driverlib flash calls return FAPI_STATUS_SUCCESS or an error code. Callers persist settings with
// these, so a silently dropped error is how a half-written blob ends up in flash — always propagate.

bool FlashUtils::EraseRegion(uint32_t addr) {
    for (uint32_t offset = 0; offset < kFlashSettingsRegionSizeBytes; offset += kFlashSectorSizeBytes) {
        if (!EraseSector(addr + offset)) {
            return false;  // Bail out rather than program into a region that is only partly erased.
        }
    }
    return true;
}

bool FlashUtils::EraseSector(uint32_t addr) { return FlashSectorErase(addr) == FAPI_STATUS_SUCCESS; }

bool FlashUtils::Program(uint32_t addr, const uint8_t* data, uint32_t size) {
    return FlashProgram(const_cast<uint8_t*>(data), addr, size) == FAPI_STATUS_SUCCESS;
}
