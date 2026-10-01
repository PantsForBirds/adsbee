#pragma once

#include <stdint.h>
#include <string.h>

// CCFG BL_CONFIG handling for AT+BOOTLOADER_PIN: the ROM serial bootloader backdoor on SYNC (DIO_5).
//
// On the CC13x4 the CCFG (customer configuration) is a dedicated 2 KB non-main flash sector at
// 0x50000000. The linker places only the 0x7C-byte .ccfg struct there (FLASH_CCFG in
// cc13x4_cc26x4_nortos.lds); the rest of the sector is erased (0xFF). No application code or data
// lives in it, and the CC13x4 CCFG has no CRC or checksum field. The boot ROM reads BL_CONFIG at
// every reset, so a change takes effect on the next reset.
//
// BL_CONFIG (CCFG offset 0x28, CC13x4 TRM / hw_ccfg.h):
//   [31:24] BOOTLOADER_ENABLE  0xC5 = ROM serial bootloader enabled, anything else = disabled
//   [16]    BL_LEVEL           1 = backdoor opens when the pin is high
//   [15:8]  BL_PIN_NUMBER      DIO sampled for the backdoor
//   [7:0]   BL_ENABLE          0xC5 = backdoor enabled, anything else = disabled
//
// Flash can only clear bits without an erase. Disabling the backdoor (BL_ENABLE 0xC5 -> 0x00) is
// therefore one in-place program of the BL_CONFIG word. Enabling it again has to set bits, which
// needs an erase of the whole CCFG sector and a reprogram of everything in it.
//
// Erasing the CCFG sector is safe in the sense TI designed for: the ROM FlashSectorErase()
// immediately programs a default security block at BL_CONFIG..TAP_DAP_1 (kPostEraseSecurity
// below: ROM bootloader on, backdoor off, debug ports on). IMAGE_VALID_CONF stays erased
// (0xFFFFFFFF), which makes the ROM treat the application as invalid and run the serial bootloader
// on the console UART. A write that fails after the erase therefore leaves a chip that boots into
// the ROM bootloader, where any CC13x4 bootloader client (such as the ADSBee 1421 Programmer) can
// reflash it.
//
// This header is hardware-independent so the planning logic can be unit tested on the host
// (ti/host_test). The flash access itself is in ccfg_bootloader.cpp.
namespace CcfgBootloader {

static constexpr uint32_t kCcfgBaseAddr = 0x50000000;
static constexpr uint32_t kCcfgSectorSizeBytes = 0x800;
static constexpr uint32_t kCcfgStructSizeBytes = 0x7C;  // sizeof(ccfg_t), CCFG_O_CKEY3 + 4.
static constexpr uint32_t kFlashWordSizeBytes = 16;     // CC13x4 flash word (FLASHWORDSIZE_ONE).

static constexpr uint32_t kBlConfigOffset = 0x28;         // CCFG_O_BL_CONFIG
static constexpr uint32_t kImageValidConfOffset = 0x40;   // CCFG_O_IMAGE_VALID_CONF
static constexpr uint32_t kSecurityOffset = 0x28;         // BL_CONFIG .. TAP_DAP_1
static constexpr uint32_t kSecuritySizeBytes = 0x18;

static constexpr uint32_t kBootloaderEnableMask = 0xFF000000;  // CCFG_BL_CONFIG_BOOTLOADER_ENABLE_M
static constexpr uint32_t kBootloaderEnableShift = 24;
static constexpr uint32_t kBlLevelMask = 0x00010000;  // CCFG_BL_CONFIG_BL_LEVEL_M
static constexpr uint32_t kBlLevelShift = 16;
static constexpr uint32_t kBlPinNumberMask = 0x0000FF00;  // CCFG_BL_CONFIG_BL_PIN_NUMBER_M
static constexpr uint32_t kBlPinNumberShift = 8;
static constexpr uint32_t kBlEnableMask = 0x000000FF;  // CCFG_BL_CONFIG_BL_ENABLE_M
static constexpr uint32_t kBlEnableShift = 0;

static constexpr uint8_t kEnableKey = 0xC5;          // Value that enables BOOTLOADER_ENABLE / BL_ENABLE.
static constexpr uint8_t kBlEnableDisabled = 0x00;   // Any value but 0xC5 disables; 0x00 needs no erase.
static constexpr uint8_t kSyncPinDio = 5;            // SYNC is DIO_5 (module pin 28).
static constexpr uint8_t kSyncPinActiveHigh = 1;     // Matches adsbee_1421.syscfg levelBootloaderBackdoor.

// What the ROM FlashSectorErase() programs at kSecurityOffset right after erasing the CCFG sector
// (g_pui8CcfgDefaultSec in driverlib/flash.c): BL_CONFIG 0xC5FFFFFF, ERASE_CONF 0xFFFFFFFF,
// ERASE_CONF_1 0xFFFFFFFF, CCFG_TI_OPTIONS / TAP_DAP_0 / TAP_DAP_1 0xFFC5C5C5.
static constexpr uint8_t kPostEraseSecurity[kSecuritySizeBytes] = {
    0xFF, 0xFF, 0xFF, 0xC5, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xC5, 0xC5, 0xC5, 0xFF, 0xC5, 0xC5, 0xC5, 0xFF, 0xC5, 0xC5, 0xC5, 0xFF};

inline uint32_t ReadWord(const uint8_t* buf, uint32_t offset) {
    uint32_t word;
    memcpy(&word, buf + offset, sizeof(word));  // Little-endian on both the CC1314 and the host.
    return word;
}

inline void WriteWord(uint8_t* buf, uint32_t offset, uint32_t word) { memcpy(buf + offset, &word, sizeof(word)); }

inline uint8_t BootloaderEnableField(uint32_t bl_config) {
    return (bl_config & kBootloaderEnableMask) >> kBootloaderEnableShift;
}
inline uint8_t BlEnableField(uint32_t bl_config) { return (bl_config & kBlEnableMask) >> kBlEnableShift; }
inline uint8_t BlPinNumberField(uint32_t bl_config) {
    return (bl_config & kBlPinNumberMask) >> kBlPinNumberShift;
}
inline uint8_t BlLevelField(uint32_t bl_config) { return (bl_config & kBlLevelMask) >> kBlLevelShift; }

inline bool RomBootloaderEnabled(uint32_t bl_config) { return BootloaderEnableField(bl_config) == kEnableKey; }

/**
 * True if a reset with the backdoor pin at its active level enters the ROM bootloader. The ROM
 * honors BL_ENABLE only while BOOTLOADER_ENABLE is also 0xC5.
 */
inline bool BackdoorEnabled(uint32_t bl_config) {
    return RomBootloaderEnabled(bl_config) && BlEnableField(bl_config) == kEnableKey;
}

/**
 * True if the backdoor is enabled on SYNC (DIO_5), active high: the configuration the ADSBee 1421
 * Programmer and the release images rely on.
 */
inline bool BackdoorEnabledOnSync(uint32_t bl_config) {
    return BackdoorEnabled(bl_config) && BlPinNumberField(bl_config) == kSyncPinDio &&
           BlLevelField(bl_config) == kSyncPinActiveHigh;
}

/**
 * Returns bl_config with the SYNC backdoor enabled or disabled.
 * Enable: BOOTLOADER_ENABLE = BL_ENABLE = 0xC5, BL_PIN_NUMBER = 5, BL_LEVEL = 1 (as the release
 * image's CCFG sets them). Disable: BL_ENABLE = 0x00; everything else is kept, so the ROM
 * bootloader still runs when the application image is invalid.
 */
inline uint32_t WithBackdoor(uint32_t bl_config, bool enable) {
    if (!enable) {
        return (bl_config & ~kBlEnableMask) | ((uint32_t)kBlEnableDisabled << kBlEnableShift);
    }
    bl_config &= ~(kBootloaderEnableMask | kBlLevelMask | kBlPinNumberMask | kBlEnableMask);
    return bl_config | ((uint32_t)kEnableKey << kBootloaderEnableShift) |
           ((uint32_t)kSyncPinActiveHigh << kBlLevelShift) | ((uint32_t)kSyncPinDio << kBlPinNumberShift) |
           ((uint32_t)kEnableKey << kBlEnableShift);
}

enum class Method : uint8_t {
    kNoChange = 0,      // BL_CONFIG already holds the requested value.
    kProgramWord,       // New value only clears bits: program the BL_CONFIG word in place, no erase.
    kEraseAndProgram,   // New value sets bits: erase the CCFG sector and reprogram it from RAM.
    kRefused            // Not safe to write; see Plan::error. Nothing is touched.
};

inline const char* MethodStr(Method method) {
    switch (method) {
        case Method::kNoChange:
            return "no change";
        case Method::kProgramWord:
            return "program BL_CONFIG in place (no erase)";
        case Method::kEraseAndProgram:
            return "erase and reprogram the CCFG sector";
        case Method::kRefused:
            return "refused";
    }
    return "?";
}

struct Plan {
    Method method = Method::kRefused;
    uint32_t old_bl_config = 0;
    uint32_t new_bl_config = 0;
    uint32_t program_len_bytes = 0;  // kEraseAndProgram: bytes of new_sector to program from offset 0.
    const char* error = nullptr;     // kRefused: why.
};

/**
 * Computes the new CCFG sector image for enabling or disabling the SYNC backdoor and how to write
 * it. Pure function: reads the current sector image (a RAM copy of all kCcfgSectorSizeBytes) and
 * fills new_sector (also kCcfgSectorSizeBytes) with the image the sector must hold afterwards.
 * The new image differs from the current one only in the BL_CONFIG word, which is checked here.
 */
inline Plan PlanUpdate(const uint8_t* current_sector, uint8_t* new_sector, bool enable) {
    Plan plan;
    memcpy(new_sector, current_sector, kCcfgSectorSizeBytes);
    plan.old_bl_config = ReadWord(current_sector, kBlConfigOffset);
    plan.new_bl_config = WithBackdoor(plan.old_bl_config, enable);

    // A CCFG that reads as erased can't be what this application booted from (IMAGE_VALID_CONF
    // 0xFFFFFFFF means "no valid image"). Don't build anything on top of an unexpected read.
    if (ReadWord(current_sector, kImageValidConfOffset) == 0xFFFFFFFF || plan.old_bl_config == 0xFFFFFFFF) {
        plan.error = "CCFG reads as erased";
        return plan;
    }

    WriteWord(new_sector, kBlConfigOffset, plan.new_bl_config);

    // Only the BL_CONFIG word may differ.
    if (memcmp(new_sector, current_sector, kBlConfigOffset) != 0 ||
        memcmp(new_sector + kBlConfigOffset + 4, current_sector + kBlConfigOffset + 4,
               kCcfgSectorSizeBytes - kBlConfigOffset - 4) != 0) {
        plan.error = "internal error: new image differs outside BL_CONFIG";
        return plan;
    }

    if (plan.new_bl_config == plan.old_bl_config) {
        plan.method = Method::kNoChange;
        return plan;
    }
    if ((plan.new_bl_config & plan.old_bl_config) == plan.new_bl_config) {
        plan.method = Method::kProgramWord;
        return plan;
    }

    // Erase path. After the erase the ROM has programmed kPostEraseSecurity, and programming can
    // only clear bits, so every byte of the new security block must be a bit subset of those
    // defaults. Check before erasing: a mismatch found after the erase could not be fixed.
    for (uint32_t i = 0; i < kSecuritySizeBytes; i++) {
        uint8_t b = new_sector[kSecurityOffset + i];
        if ((b & kPostEraseSecurity[i]) != b) {
            plan.error = "CCFG security fields can't be reproduced after an erase";
            return plan;
        }
    }

    // Program up to the last non-erased byte, rounded up to whole flash words.
    uint32_t used = kCcfgSectorSizeBytes;
    while (used > 0 && new_sector[used - 1] == 0xFF) used--;
    if (used < kCcfgStructSizeBytes) used = kCcfgStructSizeBytes;
    plan.program_len_bytes = (used + kFlashWordSizeBytes - 1) / kFlashWordSizeBytes * kFlashWordSizeBytes;
    plan.method = Method::kEraseAndProgram;
    return plan;
}

}  // namespace CcfgBootloader
