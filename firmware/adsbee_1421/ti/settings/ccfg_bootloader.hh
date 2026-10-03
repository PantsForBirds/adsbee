#pragma once

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <string_view>

// AT+BOOTLOADER_PIN: enables or disables the ROM serial bootloader backdoor on SYNC (DIO_5) in CCFG BL_CONFIG.
//
// The CCFG (customer configuration) is a 2 KB flash sector at 0x50000000 holding only the 0x7C-byte .ccfg struct;
// the rest is erased. It has no checksum. The boot ROM reads BL_CONFIG at every reset.
//
// BL_CONFIG (CCFG offset 0x28):
//   [31:24] BOOTLOADER_ENABLE  0xC5 = ROM serial bootloader enabled
//   [16]    BL_LEVEL           1 = backdoor opens when the pin is high
//   [15:8]  BL_PIN_NUMBER      DIO sampled for the backdoor
//   [7:0]   BL_ENABLE          0xC5 = backdoor enabled
//
// Flash programming only clears bits, so disabling is one in-place word write; enabling needs a sector erase and
// reprogram. If that write fails after the erase, the chip still boots into the ROM bootloader (the erase leaves
// IMAGE_VALID_CONF unset), so it can be reflashed.
//
// Hardware-independent for host tests; flash access is in ccfg_bootloader_flash.cpp.
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

// Security block the ROM FlashSectorErase() programs right after erasing the CCFG (g_pui8CcfgDefaultSec in
// driverlib/flash.c): ROM bootloader on, backdoor off, debug ports on.
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

// The ROM honors BL_ENABLE only while BOOTLOADER_ENABLE is also 0xC5.
inline bool BackdoorEnabled(uint32_t bl_config) {
    return RomBootloaderEnabled(bl_config) && BlEnableField(bl_config) == kEnableKey;
}

// Backdoor on SYNC, active high: what the release images and the ADSBee 1421 Programmer expect.
inline bool BackdoorEnabledOnSync(uint32_t bl_config) {
    return BackdoorEnabled(bl_config) && BlPinNumberField(bl_config) == kSyncPinDio &&
           BlLevelField(bl_config) == kSyncPinActiveHigh;
}

// Enable sets the release image's values. Disable clears only BL_ENABLE, so the ROM bootloader still runs when the
// application image is invalid.
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
    // A previous erase-path write failed to restore the original CCFG; this plan rewrites that original (kept in RAM).
    bool restore_retry = false;
};

// Fills new_sector with the sector image after the change (only BL_CONFIG differs) and picks how to write it. Both
// buffers are kCcfgSectorSizeBytes.
inline Plan PlanUpdate(const uint8_t* current_sector, uint8_t* new_sector, bool enable) {
    Plan plan;
    memcpy(new_sector, current_sector, kCcfgSectorSizeBytes);
    plan.old_bl_config = ReadWord(current_sector, kBlConfigOffset);
    plan.new_bl_config = WithBackdoor(plan.old_bl_config, enable);

    // An erased-looking CCFG can't be what we booted from; don't build on a bad read.
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

    // Erase path: the new security block must be reachable from kPostEraseSecurity by clearing bits. Check before
    // erasing, since a mismatch can't be fixed afterwards.
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

// Standard CRC-32 (zlib). Over the CCFG struct it matches the ROM bootloader's COMMAND_CRC32, so results can be
// compared with what the ADSBee 1421 Programmer reads.
inline uint32_t Crc32(const uint8_t* data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) crc = (crc >> 1) ^ (0xEDB88320 & (0u - (crc & 1)));
    }
    return ~crc;
}
inline uint32_t StructCrc32(const uint8_t* sector) { return Crc32(sector, kCcfgStructSizeBytes); }

// Fixed password so a CCFG write is deliberate; not a secret.
static constexpr char kPassword[] = "DEADBEE";

struct SetArgs {
    bool ok = false;
    bool enable = false;
    bool dry_run = false;
    const char* error = nullptr;  // !ok: why, for the AT ERROR line.
};

// Parses AT+BOOTLOADER_PIN=<enabled [1,0]>,<password>[,DRYRUN]. The password is always required, DRYRUN included.
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
 * Performs a planned write and verifies the whole sector against new_sector. `flash` is the device ROM API or a
 * host test model:
 *   bool flash.Erase();
 *   bool flash.Program(const uint8_t* src, uint32_t offset, uint32_t len);
 *   const uint8_t* flash.Read();  // The whole sector.
 * A failed erase-path write restores original_sector, up to kRestoreAttempts times.
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
        // No erase.
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
    // Never leave the sector erased or half written. Both images need the same program length.
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

// Builds the "CCFG WRITE FAILED" banner with state and recovery steps. Calls emit(const char* line) per line.
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
