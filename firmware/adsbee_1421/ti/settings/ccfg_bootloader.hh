#pragma once

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <string_view>

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
// (ti/host_test). The flash access itself is in ccfg_bootloader_flash.cpp.
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
    // A previous erase-path write failed and its original CCFG could not be put back. This plan rewrites that
    // original image (kept in RAM) instead of making the requested change.
    bool restore_retry = false;
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

/**
 * CRC-32 (IEEE 802.3, the zlib/Ethernet CRC) over len bytes. Over the kCcfgStructSizeBytes CCFG struct it
 * equals the CRC that the ROM serial bootloader's COMMAND_CRC32 returns for 0x50000000 (the release image's
 * CCFG gives 0x66E9858D), so a write can be compared with what the ADSBee 1421 Programmer reads.
 */
inline uint32_t Crc32(const uint8_t* data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) crc = (crc >> 1) ^ (0xEDB88320 & (0u - (crc & 1)));
    }
    return ~crc;
}
inline uint32_t StructCrc32(const uint8_t* sector) { return Crc32(sector, kCcfgStructSizeBytes); }

// AT+BOOTLOADER_PIN set form: AT+BOOTLOADER_PIN=<enabled [1,0]>,<password>[,DRYRUN]. The fixed password makes
// a CCFG write deliberate; it is not a secret.
static constexpr char kPassword[] = "DEADBEE";

struct SetArgs {
    bool ok = false;
    bool enable = false;
    bool dry_run = false;
    const char* error = nullptr;  // !ok: why, for the AT ERROR line.
};

/**
 * Parses the arguments of the AT+BOOTLOADER_PIN set form. Every set form, DRYRUN included, needs the password,
 * and it is checked before the CCFG is read.
 */
inline SetArgs ParseSetArgs(const std::string_view* args, uint16_t num_args) {
    SetArgs result;
    if (num_args < 1 || (args[0] != "0" && args[0] != "1")) {
        result.error = "Requires arguments: AT+BOOTLOADER_PIN=<enabled [1,0]>,<password>[,DRYRUN]";
        return result;
    }
    result.enable = args[0] == "1";
    if (num_args < 2 || args[1].empty()) {
        result.error = "Requires the password (see AT+HELP or the README): AT+BOOTLOADER_PIN=<enabled [1,0]>,"
                       "<password>[,DRYRUN]. CCFG not touched";
        return result;
    }
    if (args[1] == "DRYRUN") {
        result.error = "The password goes before DRYRUN: AT+BOOTLOADER_PIN=<enabled [1,0]>,<password>,DRYRUN. "
                       "CCFG not touched";
        return result;
    }
    if (args[1] != kPassword) {
        result.error = "Wrong password. CCFG not touched";
        return result;
    }
    if (num_args >= 3 && !args[2].empty()) {
        if (args[2] != "DRYRUN") {
            result.error = "Unknown option, expected DRYRUN. CCFG not touched";
            return result;
        }
        result.dry_run = true;
    }
    if (num_args > 3) {
        result.error = "Too many arguments. CCFG not touched";
        return result;
    }
    result.ok = true;
    return result;
}

// Erase path: how many times a failed write tries to put the original CCFG back.
static constexpr int kRestoreAttempts = 3;

enum class WriteResult : uint8_t {
    kOk = 0,             // The sector holds the planned image (verified byte for byte), or nothing needed writing.
    kNotWritten,         // Refused before any flash operation; the CCFG is untouched.
    kFailedInPlace,      // kProgramWord failed: no erase happened, only BL_CONFIG may differ from the plan.
    kFailedRestored,     // Erase path failed; the original CCFG was put back and verified.
    kFailedNotRestored,  // Erase path failed and putting the original back failed too: CCFG in an unknown state.
};

/**
 * Performs a planned write (kProgramWord or kEraseAndProgram) and verifies the whole sector against new_sector.
 * The flash operations come from `flash`, so this logic runs unchanged on the device (ROM flash API, interrupts
 * masked) and in host tests (a model with fault injection):
 *   bool flash.Erase();                                              // Erase the CCFG sector.
 *   bool flash.Program(const uint8_t* src, uint32_t offset, uint32_t len);
 *   const uint8_t* flash.Read();                                     // The kCcfgSectorSizeBytes sector.
 * A failed erase-path write writes original_sector back, up to kRestoreAttempts times, each verified byte for
 * byte; a plan with restore_retry (new_sector is the original) only retries that. Only a verified sector counts
 * as success: kOk means the sector equals new_sector.
 */
template <typename Flash>
WriteResult WriteAndVerify(Flash& flash, const Plan& plan, const uint8_t* original_sector, const uint8_t* new_sector,
                           int& restore_attempts, const char*& error) {
    auto erase_and_program = [&flash, &plan](const uint8_t* image) {
        return flash.Erase() && flash.Program(image, 0, plan.program_len_bytes) &&
               memcmp(flash.Read(), image, kCcfgSectorSizeBytes) == 0;
    };
    restore_attempts = 0;
    if (plan.method == Method::kProgramWord) {
        // Clearing bits only: one in-place program of the word, the sector is never erased.
        uint32_t word = ReadWord(new_sector, kBlConfigOffset);  // Word aligned for FlashProgram().
        if (flash.Program(reinterpret_cast<const uint8_t*>(&word), kBlConfigOffset, sizeof(word)) &&
            memcmp(flash.Read(), new_sector, kCcfgSectorSizeBytes) == 0) {
            return WriteResult::kOk;
        }
        error = "BL_CONFIG program or verify failed";
        return WriteResult::kFailedInPlace;
    }
    if (plan.method != Method::kEraseAndProgram) {
        error = "nothing to write";
        return WriteResult::kNotWritten;
    }
    if (plan.restore_retry) {
        // new_sector is the original image: retrying the write is retrying the restore.
        while (restore_attempts < kRestoreAttempts) {
            restore_attempts++;
            if (erase_and_program(new_sector)) return WriteResult::kOk;
        }
        error = "CCFG erase, program or verify failed";
        return WriteResult::kFailedNotRestored;
    }
    if (erase_and_program(new_sector)) return WriteResult::kOk;
    error = "CCFG erase, program or verify failed";
    // Never leave the sector erased or half written: put the original back. Both images differ only in
    // BL_CONFIG, so they need the same program length.
    while (restore_attempts < kRestoreAttempts) {
        restore_attempts++;
        if (erase_and_program(original_sector)) return WriteResult::kFailedRestored;
    }
    return WriteResult::kFailedNotRestored;
}

// Short form for the AT ERROR line.
inline const char* WriteResultStr(WriteResult result) {
    switch (result) {
        case WriteResult::kOk:
            return "OK";
        case WriteResult::kNotWritten:
            return "CCFG not touched";
        case WriteResult::kFailedInPlace:
            return "BL_CONFIG may be partly written, rest of the CCFG unchanged";
        case WriteResult::kFailedRestored:
            return "original CCFG restored, nothing changed";
        case WriteResult::kFailedNotRestored:
            return "CCFG IN AN UNKNOWN STATE, DO NOT RESET";
    }
    return "?";
}

struct WriteReport {
    WriteResult result = WriteResult::kNotWritten;
    bool enable = false;  // Requested backdoor state.
    Method method = Method::kRefused;
    bool restore_retry = false;  // Plan::restore_retry.
    uint32_t old_bl_config = 0;
    uint32_t new_bl_config = 0;
    uint32_t readback_bl_config = 0;  // From flash after the attempt.
    uint32_t original_crc = 0;        // StructCrc32() of the CCFG before the attempt.
    uint32_t expected_crc = 0;        // StructCrc32() of the planned image.
    uint32_t readback_crc = 0;        // StructCrc32() read back from flash after the attempt.
    int restore_attempts = 0;         // Erase path: restores tried after the failed write.
    const char* error = nullptr;      // What failed.

    // A flash operation ran and didn't produce the planned image.
    bool Failed() const { return result != WriteResult::kOk && result != WriteResult::kNotWritten; }
};

static constexpr size_t kBannerLineMax = 160;

/**
 * Builds the "CCFG WRITE FAILED" banner for a failed write (report.Failed()): what was attempted, what failed,
 * the read-back, the resulting state and the recovery steps. Calls emit(const char* line) once per line (no line
 * ending). Pure so it can be host tested; the device prints each line with CONSOLE_ERROR.
 */
template <typename Emit>
void FailureBanner(const WriteReport& r, Emit emit) {
    char line[kBannerLineMax];
    const char* rule = "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
    const int n = r.enable ? 1 : 0;
    const bool backdoor_now = BackdoorEnabled(r.readback_bl_config);
    emit(rule);
    snprintf(line, sizeof(line), "!!! CCFG WRITE FAILED: AT+BOOTLOADER_PIN=%d (%s the SYNC bootloader backdoor)%s", n,
             r.enable ? "enable" : "disable", r.restore_retry ? ", retrying the restore of the original CCFG" : "");
    emit(line);
    snprintf(line, sizeof(line), "!!! Attempted: %s, BL_CONFIG 0x%08lX -> 0x%08lX", MethodStr(r.method),
             (unsigned long)r.old_bl_config, (unsigned long)r.new_bl_config);
    emit(line);
    snprintf(line, sizeof(line), "!!! Failed: %s", r.error ? r.error : "unknown error");
    emit(line);
    snprintf(line, sizeof(line),
             "!!! Read back: BL_CONFIG 0x%08lX (backdoor %s), CCFG CRC32 0x%08lX (planned 0x%08lX, original 0x%08lX)",
             (unsigned long)r.readback_bl_config, backdoor_now ? "enabled" : "disabled", (unsigned long)r.readback_crc,
             (unsigned long)r.expected_crc, (unsigned long)r.original_crc);
    emit(line);
    switch (r.result) {
        case WriteResult::kFailedInPlace:
            emit("!!! State: the CCFG sector was not erased and only BL_CONFIG can differ from before. The");
            emit("!!! application image stays valid, so the module runs and resets normally.");
            emit("!!! Recovery:");
            emit("!!!   1. Check the setting with AT+BOOTLOADER_PIN? (the read-back above is the live value).");
            snprintf(line, sizeof(line), "!!!   2. Retry AT+BOOTLOADER_PIN=%d,%s.", n, kPassword);
            emit(line);
            snprintf(line, sizeof(line),
                     "!!!   3. If it keeps failing, AT+BOOTLOADER_PIN=1,%s rewrites the whole CCFG with the backdoor",
                     kPassword);
            emit(line);
            emit("!!!      enabled; reflashing a release image over JTAG (or with the ADSBee 1421 Programmer while the");
            emit("!!!      backdoor is enabled) also restores it.");
            break;
        case WriteResult::kFailedRestored:
            snprintf(line, sizeof(line),
                     "!!! State: the original CCFG was written back and verified byte for byte (restore attempt %d of "
                     "%d).",
                     r.restore_attempts, kRestoreAttempts);
            emit(line);
            snprintf(line, sizeof(line), "!!! Nothing changed: the backdoor is still %s, and the module runs and resets normally.",
                     backdoor_now ? "enabled" : "disabled");
            emit(line);
            emit("!!! Recovery:");
            snprintf(line, sizeof(line), "!!!   1. Retry AT+BOOTLOADER_PIN=%d,%s.", n, kPassword);
            emit(line);
            if (backdoor_now) {
                emit("!!!   2. If it keeps failing, reflash a release image with the ADSBee 1421 Programmer or JTAG.");
            } else {
                emit("!!!   2. If it keeps failing, reflash a release image over JTAG (the ADSBee 1421 Programmer can't");
                emit("!!!      enter the bootloader while the backdoor is disabled).");
            }
            break;
        case WriteResult::kFailedNotRestored:
        default:
            snprintf(line, sizeof(line),
                     "!!! State: UNKNOWN. Writing the original CCFG back failed %d time(s); the sector may be erased or",
                     r.restore_attempts);
            emit(line);
            emit("!!! half written. DO NOT RESET OR POWER CYCLE THE MODULE YET.");
            emit("!!! Recovery:");
            snprintf(line, sizeof(line),
                     "!!!   1. Send AT+BOOTLOADER_PIN=%d,%s again now: it retries writing the original CCFG, which is",
                     n, kPassword);
            emit(line);
            emit("!!!      kept in RAM until the next reset. Repeat until it reports OK.");
            emit("!!!   2. If the module resets first, it most likely starts in the CC1314 ROM UART bootloader (the");
            emit("!!!      erase leaves IMAGE_VALID_CONF unset). Reflash a release image with the ADSBee 1421");
            emit("!!!      Programmer, which writes a complete CCFG.");
            emit("!!!   3. If neither works, reflash over JTAG.");
            break;
    }
    emit(rule);
}

}  // namespace CcfgBootloader
