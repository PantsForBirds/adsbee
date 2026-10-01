// Host tests for CcfgBootloader::PlanUpdate() (AT+BOOTLOADER_PIN). Each plan is played back against
// a model of the CC13x4 CCFG flash sector (erase -> 0xFF plus the ROM's default security block,
// program -> bitwise AND) to check that the planned writes produce exactly the planned image.
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ccfg_bootloader.hh"

using namespace CcfgBootloader;

static int failures = 0;

#define EXPECT(cond)                                                 \
    do {                                                             \
        if (!(cond)) {                                               \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            failures++;                                              \
        }                                                            \
    } while (0)

// .ccfg of the adsbee_1421 release image (adsbee_1421.syscfg -> ti_devices_config.c), by word.
static const uint32_t kReleaseCcfg[kCcfgStructSizeBytes / 4] = {
    0x007CFFFD, 0xF3FBFFFF, 0xFF800010, 0xFFFFFFFF, 0xFFFFFFFF, 0x01800000, 0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFFFF, 0xFFFFFFFF, 0xC5FF05C5, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFC50000, 0xFFC5C500, 0xFF000000,
    0x00000000, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFE0400, 0xFFFC0000, 0xFFFFFFFE,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};

struct FlashSector {
    uint8_t bytes[kCcfgSectorSizeBytes];

    // A CCFG sector as the release image leaves it: the struct, then erased flash.
    FlashSector() {
        memset(bytes, 0xFF, sizeof(bytes));
        for (uint32_t i = 0; i < kCcfgStructSizeBytes / 4; i++) WriteWord(bytes, 4 * i, kReleaseCcfg[i]);
    }
    // ROM FlashSectorErase() on the CCFG sector.
    void Erase() {
        memset(bytes, 0xFF, sizeof(bytes));
        memcpy(bytes + kSecurityOffset, kPostEraseSecurity, kSecuritySizeBytes);
    }
    // ROM FlashProgram(): bits can only go from 1 to 0.
    void Program(uint32_t offset, const uint8_t* data, uint32_t len) {
        for (uint32_t i = 0; i < len; i++) bytes[offset + i] &= data[i];
    }
    // Performs a plan the way CcfgBootloader::Apply() does.
    void Apply(const Plan& plan, const uint8_t* new_sector) {
        if (plan.method == Method::kProgramWord) {
            uint32_t word = plan.new_bl_config;
            Program(kBlConfigOffset, reinterpret_cast<const uint8_t*>(&word), sizeof(word));
        } else if (plan.method == Method::kEraseAndProgram) {
            Erase();
            Program(0, new_sector, plan.program_len_bytes);
        }
    }
    uint32_t BlConfig() const { return ReadWord(bytes, kBlConfigOffset); }
};

// Plans the change on the sector, applies it and checks the sector then matches the planned image.
static Plan PlanAndApply(FlashSector& flash, bool enable) {
    uint8_t new_sector[kCcfgSectorSizeBytes];
    Plan plan = PlanUpdate(flash.bytes, new_sector, enable);
    if (plan.method != Method::kRefused) {
        flash.Apply(plan, new_sector);
        EXPECT(memcmp(flash.bytes, new_sector, sizeof(new_sector)) == 0);
    }
    return plan;
}

static void TestReleaseImageFields() {
    uint32_t bl = kReleaseCcfg[kBlConfigOffset / 4];
    EXPECT(RomBootloaderEnabled(bl));
    EXPECT(BackdoorEnabled(bl));
    EXPECT(BackdoorEnabledOnSync(bl));
    EXPECT(BlPinNumberField(bl) == 5);
    EXPECT(BlLevelField(bl) == 1);
    EXPECT(WithBackdoor(bl, true) == bl);  // Enabling reproduces the release value exactly.
    // BL_ENABLE alone doesn't open the backdoor while the ROM bootloader is disabled.
    EXPECT(!BackdoorEnabled(0x00FF05C5));
    EXPECT(!BackdoorEnabled(0xC5FF0500));
}

static void TestEnableOnReleaseIsNoChange() {
    FlashSector flash;
    Plan plan = PlanAndApply(flash, true);
    EXPECT(plan.method == Method::kNoChange);
    EXPECT(flash.BlConfig() == 0xC5FF05C5);
}

static void TestDisableIsProgramOnly() {
    FlashSector flash;
    Plan plan = PlanAndApply(flash, false);
    EXPECT(plan.method == Method::kProgramWord);
    EXPECT(plan.old_bl_config == 0xC5FF05C5);
    EXPECT(plan.new_bl_config == 0xC5FF0500);
    EXPECT(!BackdoorEnabled(flash.BlConfig()));
    EXPECT(RomBootloaderEnabled(flash.BlConfig()));  // The ROM still catches an invalid image.
    // Everything but BL_CONFIG is untouched.
    FlashSector release;
    EXPECT(memcmp(flash.bytes, release.bytes, kBlConfigOffset) == 0);
    EXPECT(memcmp(flash.bytes + kBlConfigOffset + 4, release.bytes + kBlConfigOffset + 4,
                  kCcfgSectorSizeBytes - kBlConfigOffset - 4) == 0);
    // Disabling again changes nothing.
    EXPECT(PlanAndApply(flash, false).method == Method::kNoChange);
}

static void TestReEnableRestoresReleaseSector() {
    FlashSector flash;
    PlanAndApply(flash, false);
    Plan plan = PlanAndApply(flash, true);
    EXPECT(plan.method == Method::kEraseAndProgram);
    EXPECT(plan.program_len_bytes == 0x80);  // 0x7C-byte struct, rounded up to a 16-byte flash word.
    FlashSector release;
    EXPECT(memcmp(flash.bytes, release.bytes, kCcfgSectorSizeBytes) == 0);
    EXPECT(BackdoorEnabledOnSync(flash.BlConfig()));
}

static void TestEnableFromOtherConfigs() {
    // ROM bootloader disabled, backdoor on another pin, active low: enabling sets all four fields.
    FlashSector flash;
    WriteWord(flash.bytes, kBlConfigOffset, 0x00FE0700);
    Plan plan = PlanAndApply(flash, true);
    EXPECT(plan.method == Method::kEraseAndProgram);
    EXPECT(flash.BlConfig() == 0xC5FF05C5);
    EXPECT(BackdoorEnabledOnSync(flash.BlConfig()));
}

static void TestErasedCcfgRefused() {
    FlashSector flash;
    memset(flash.bytes, 0xFF, sizeof(flash.bytes));
    uint8_t new_sector[kCcfgSectorSizeBytes];
    EXPECT(PlanUpdate(flash.bytes, new_sector, true).method == Method::kRefused);
    EXPECT(PlanUpdate(flash.bytes, new_sector, false).method == Method::kRefused);
}

static void TestIncompatibleSecurityRefusedBeforeErase() {
    // TAP_DAP_0 = 0xFFFFFFFF can't be programmed back after an erase (the ROM writes 0xFFC5C5C5).
    FlashSector flash;
    WriteWord(flash.bytes, 0x38, 0xFFFFFFFF);
    WriteWord(flash.bytes, kBlConfigOffset, 0xC5FF0500);
    uint8_t new_sector[kCcfgSectorSizeBytes];
    Plan plan = PlanUpdate(flash.bytes, new_sector, true);
    EXPECT(plan.method == Method::kRefused);
    EXPECT(plan.error != nullptr);
    // Disabling needs no erase, so it is still allowed on such a sector.
    WriteWord(flash.bytes, kBlConfigOffset, 0xC5FF05C5);
    EXPECT(PlanAndApply(flash, false).method == Method::kProgramWord);
}

static void TestDataBeyondStructIsKept() {
    FlashSector flash;
    PlanAndApply(flash, false);
    flash.bytes[0x123] = 0x5A;
    Plan plan = PlanAndApply(flash, true);
    EXPECT(plan.method == Method::kEraseAndProgram);
    EXPECT(plan.program_len_bytes == 0x130);
    EXPECT(flash.bytes[0x123] == 0x5A);
}

// The .ccfg section of a built image (objcopy -O binary -j .ccfg adsbee_1421.elf): its CCFG must
// match kReleaseCcfg, so a CCFG change in adsbee_1421.syscfg gets checked against the erase path.
static void TestBuiltImage(const char* path) {
    FlashSector built;
    memset(built.bytes, 0xFF, sizeof(built.bytes));
    FILE* f = fopen(path, "rb");
    EXPECT(f != nullptr);
    if (!f) return;
    size_t len = fread(built.bytes, 1, sizeof(built.bytes), f);
    fclose(f);
    printf("  %s: %zu bytes\n", path, len);
    EXPECT(len == kCcfgStructSizeBytes);
    FlashSector release;
    EXPECT(memcmp(built.bytes, release.bytes, sizeof(built.bytes)) == 0);
    // Disabling and re-enabling must reproduce the built CCFG exactly.
    EXPECT(PlanAndApply(built, true).method == Method::kNoChange);
    EXPECT(PlanAndApply(built, false).method == Method::kProgramWord);
    EXPECT(PlanAndApply(built, true).method == Method::kEraseAndProgram);
    EXPECT(memcmp(built.bytes, release.bytes, sizeof(built.bytes)) == 0);
}

int main(int argc, char** argv) {
    if (argc > 1) {
        TestBuiltImage(argv[1]);
    }
    TestReleaseImageFields();
    TestEnableOnReleaseIsNoChange();
    TestDisableIsProgramOnly();
    TestReEnableRestoresReleaseSector();
    TestEnableFromOtherConfigs();
    TestErasedCcfgRefused();
    TestIncompatibleSecurityRefusedBeforeErase();
    TestDataBeyondStructIsKept();
    if (failures) {
        printf("%d failure(s)\n", failures);
        return 1;
    }
    printf("all passed\n");
    return 0;
}
