// Host tests for CcfgBootloader (AT+BOOTLOADER_PIN), run against a model of the CCFG flash sector with fault
// injection. A failure must never be reported as success.
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <string_view>
#include <vector>

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

    // Operations fault_at to fault_at + fault_ops - 1 (counted from 0) get `fault`, if it matches their kind.
    enum class Fault { kNone, kEraseFails, kProgramFails, kProgramCorrupts };
    static constexpr int kAlways = 1000;
    Fault fault = Fault::kNone;
    int fault_at = -1;
    int fault_ops = 1;
    int ops = 0;

    // A CCFG sector as the release image leaves it: the struct, then erased flash.
    FlashSector() {
        memset(bytes, 0xFF, sizeof(bytes));
        for (uint32_t i = 0; i < kCcfgStructSizeBytes / 4; i++) WriteWord(bytes, 4 * i, kReleaseCcfg[i]);
    }
    // Counts one flash operation and returns the fault that hits it, if any.
    Fault NextOp() {
        int op = ops++;
        return (op >= fault_at && op < fault_at + fault_ops) ? fault : Fault::kNone;
    }
    // A failed erase leaves the sector erased without the ROM's security block.
    bool Erase() {
        Fault f = NextOp();
        memset(bytes, 0xFF, sizeof(bytes));
        if (f == Fault::kEraseFails) return false;
        memcpy(bytes + kSecurityOffset, kPostEraseSecurity, kSecuritySizeBytes);
        return true;
    }
    // Programming only clears bits. A failed program writes half; a corrupting one reports success but clears an
    // extra bit.
    bool Program(const uint8_t* data, uint32_t offset, uint32_t len) {
        Fault f = NextOp();
        for (uint32_t i = 0; i < (f == Fault::kProgramFails ? len / 2 : len); i++) bytes[offset + i] &= data[i];
        if (f == Fault::kProgramCorrupts) {
            for (uint32_t i = len; i-- > 0;) {
                if (bytes[offset + i] != 0) {
                    bytes[offset + i] &= bytes[offset + i] - 1;  // Clear the lowest set bit.
                    break;
                }
            }
        }
        return f != Fault::kProgramFails;
    }
    const uint8_t* Read() { return bytes; }
    uint32_t BlConfig() const { return ReadWord(bytes, kBlConfigOffset); }
};

// Plans the change on the sector, applies it and checks the sector then matches the planned image.
static Plan PlanAndApply(FlashSector& flash, bool enable) {
    uint8_t original[kCcfgSectorSizeBytes];
    uint8_t new_sector[kCcfgSectorSizeBytes];
    memcpy(original, flash.bytes, sizeof(original));
    Plan plan = PlanUpdate(flash.bytes, new_sector, enable);
    if (plan.method == Method::kProgramWord || plan.method == Method::kEraseAndProgram) {
        int restore_attempts = 0;
        const char* error = nullptr;
        EXPECT(WriteAndVerify(flash, plan, original, new_sector, restore_attempts, error) == WriteResult::kOk);
        EXPECT(memcmp(flash.bytes, new_sector, sizeof(new_sector)) == 0);
        EXPECT(restore_attempts == 0 && error == nullptr);
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
    // ROM bootloader off, backdoor on another pin, active low.
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
    // TAP_DAP_0 = 0xFFFFFFFF can't be restored after an erase (the ROM writes 0xFFC5C5C5).
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

// AT+BOOTLOADER_PIN=<enabled [1,0]>,<password>[,DRYRUN]
static SetArgs Parse(std::vector<std::string_view> args) {
    return ParseSetArgs(args.data(), static_cast<uint16_t>(args.size()));
}

static void TestSetArgsPassword() {
    SetArgs a = Parse({"0", "DEADBEE"});
    EXPECT(a.ok && !a.enable && !a.dry_run);
    a = Parse({"1", "DEADBEE"});
    EXPECT(a.ok && a.enable && !a.dry_run);
    a = Parse({"0", "DEADBEE", "DRYRUN"});
    EXPECT(a.ok && !a.enable && a.dry_run);
    a = Parse({"1", "DEADBEE", ""});  // Trailing empty argument (AT+BOOTLOADER_PIN=1,DEADBEE,).
    EXPECT(a.ok && a.enable && !a.dry_run);

    // Missing or wrong password: refused, for both values and for DRYRUN.
    for (const char* value : {"0", "1"}) {
        a = Parse({value});
        EXPECT(!a.ok && a.error && strstr(a.error, "password"));
        a = Parse({value, ""});
        EXPECT(!a.ok && a.error && strstr(a.error, "password"));
        a = Parse({value, "deadbee"});  // Case matters.
        EXPECT(!a.ok && a.error && strstr(a.error, "Wrong password"));
        a = Parse({value, "DEADBEEF"});
        EXPECT(!a.ok && a.error && strstr(a.error, "Wrong password"));
        a = Parse({value, "DEADBE"});
        EXPECT(!a.ok && a.error && strstr(a.error, "Wrong password"));
        a = Parse({value, " DEADBEE"});
        EXPECT(!a.ok);
        a = Parse({value, "DRYRUN"});  // The old =<0|1>,DRYRUN syntax.
        EXPECT(!a.ok && a.error && strstr(a.error, "before DRYRUN"));
        a = Parse({value, "WRONG", "DRYRUN"});
        EXPECT(!a.ok && a.error && strstr(a.error, "Wrong password"));
    }
    // The password is never echoed in an error.
    for (auto args : std::vector<std::vector<std::string_view>>{{"0"}, {"0", "x"}, {"0", "DRYRUN"}, {"2", "x"}}) {
        a = Parse(args);
        EXPECT(!a.ok && a.error && !strstr(a.error, kPassword));
    }

    // Other argument errors.
    EXPECT(!Parse({}).ok);
    EXPECT(!Parse({""}).ok);
    EXPECT(!Parse({"2", "DEADBEE"}).ok);
    EXPECT(!Parse({"01", "DEADBEE"}).ok);
    a = Parse({"0", "DEADBEE", "DRY"});
    EXPECT(!a.ok && a.error && strstr(a.error, "DRYRUN"));
    EXPECT(!Parse({"0", "DEADBEE", "DRYRUN", "X"}).ok);
    // Every refusal says the CCFG wasn't touched (except the plain usage error).
    a = Parse({"0", "nope"});
    EXPECT(a.error && strstr(a.error, "CCFG not touched"));
}

// Matches the ROM bootloader's COMMAND_CRC32 (0x66E9858D for the release CCFG) and zlib's crc32().
static void TestCrc32() {
    FlashSector release;
    EXPECT(StructCrc32(release.bytes) == 0x66E9858D);
    EXPECT(Crc32(reinterpret_cast<const uint8_t*>("123456789"), 9) == 0xCBF43926);  // CRC-32 check value.
    FlashSector disabled;
    PlanAndApply(disabled, false);
    EXPECT(StructCrc32(disabled.bytes) != 0x66E9858D);
}

// Runs one planned write with a fault and checks the result matches the sector: kOk = planned image,
// kFailedRestored = original, kFailedInPlace = only BL_CONFIG changed.
static WriteResult WriteWithFault(FlashSector& flash, bool enable, FlashSector::Fault fault, int fault_at,
                                  int fault_ops, int* restore_attempts_out = nullptr) {
    uint8_t original[kCcfgSectorSizeBytes];
    uint8_t new_sector[kCcfgSectorSizeBytes];
    memcpy(original, flash.bytes, sizeof(original));
    Plan plan = PlanUpdate(flash.bytes, new_sector, enable);
    EXPECT(plan.method == Method::kProgramWord || plan.method == Method::kEraseAndProgram);
    flash.fault = fault;
    flash.fault_at = fault_at;
    flash.fault_ops = fault_ops;
    flash.ops = 0;
    int restore_attempts = -1;
    const char* error = nullptr;
    WriteResult result = WriteAndVerify(flash, plan, original, new_sector, restore_attempts, error);
    flash.fault = FlashSector::Fault::kNone;
    switch (result) {
        case WriteResult::kOk:
            EXPECT(memcmp(flash.bytes, new_sector, sizeof(new_sector)) == 0);
            EXPECT(error == nullptr);
            break;
        case WriteResult::kFailedRestored:
            EXPECT(plan.method == Method::kEraseAndProgram);
            EXPECT(memcmp(flash.bytes, original, sizeof(original)) == 0);
            EXPECT(error != nullptr && restore_attempts >= 1 && restore_attempts <= kRestoreAttempts);
            break;
        case WriteResult::kFailedInPlace:
            EXPECT(plan.method == Method::kProgramWord);
            EXPECT(memcmp(flash.bytes, original, kBlConfigOffset) == 0);
            EXPECT(memcmp(flash.bytes + kBlConfigOffset + 4, original + kBlConfigOffset + 4,
                          kCcfgSectorSizeBytes - kBlConfigOffset - 4) == 0);
            EXPECT(error != nullptr);
            break;
        case WriteResult::kFailedNotRestored:
            EXPECT(plan.method == Method::kEraseAndProgram);
            EXPECT(restore_attempts == kRestoreAttempts && error != nullptr);
            break;
        case WriteResult::kNotWritten:
            EXPECT(false);
            break;
    }
    if (restore_attempts_out) *restore_attempts_out = restore_attempts;
    return result;
}

static void TestWriteFaultsDisable() {
    // A failed or corrupted in-place program is never reported as OK.
    for (auto fault : {FlashSector::Fault::kProgramFails, FlashSector::Fault::kProgramCorrupts}) {
        FlashSector flash;
        EXPECT(WriteWithFault(flash, false, fault, 0, 1) == WriteResult::kFailedInPlace);
        // A cut-off program only reaches BL_ENABLE (low byte), so the ROM bootloader stays enabled.
        if (fault == FlashSector::Fault::kProgramFails) EXPECT(RomBootloaderEnabled(flash.BlConfig()));
    }
}

static void TestWriteFaultsEnable() {
    using Fault = FlashSector::Fault;
    // Erase path operations: 0 erase, 1 program, then (erase, program) pairs per restore.
    {
        FlashSector flash;
        PlanAndApply(flash, false);
        int attempts = 0;
        EXPECT(WriteWithFault(flash, true, Fault::kEraseFails, 0, 1, &attempts) == WriteResult::kFailedRestored);
        EXPECT(attempts == 1 && !BackdoorEnabled(flash.BlConfig()));
    }
    {
        FlashSector flash;
        PlanAndApply(flash, false);
        EXPECT(WriteWithFault(flash, true, Fault::kProgramFails, 1, 1) == WriteResult::kFailedRestored);
    }
    {
        // Program reports success but the verify catches the wrong bit.
        FlashSector flash;
        PlanAndApply(flash, false);
        EXPECT(WriteWithFault(flash, true, Fault::kProgramCorrupts, 1, 1) == WriteResult::kFailedRestored);
    }
    {
        // Ops 1-3 fail; the second restore works.
        FlashSector flash;
        PlanAndApply(flash, false);
        int attempts = 0;
        EXPECT(WriteWithFault(flash, true, Fault::kProgramFails, 1, 3, &attempts) == WriteResult::kFailedRestored);
        EXPECT(attempts == 2);
    }
    {
        // Every operation fails: the state is unknown after exactly kRestoreAttempts restores.
        for (auto fault : {Fault::kEraseFails, Fault::kProgramFails, Fault::kProgramCorrupts}) {
            FlashSector flash;
            PlanAndApply(flash, false);
            EXPECT(WriteWithFault(flash, true, fault, 0, FlashSector::kAlways) == WriteResult::kFailedNotRestored);
            // A restore retry succeeds once the flash works again.
            uint8_t original[kCcfgSectorSizeBytes];
            FlashSector disabled;
            PlanAndApply(disabled, false);
            memcpy(original, disabled.bytes, sizeof(original));
            Plan retry;
            retry.method = Method::kEraseAndProgram;
            retry.program_len_bytes = 0x80;
            retry.restore_retry = true;
            int attempts = 0;
            const char* error = nullptr;
            EXPECT(WriteAndVerify(flash, retry, original, original, attempts, error) == WriteResult::kOk);
            EXPECT(memcmp(flash.bytes, original, sizeof(original)) == 0 && attempts == 1);
            // A restore retry on flash that still fails stays not restored.
            flash.fault = fault;
            flash.fault_at = 0;
            flash.fault_ops = FlashSector::kAlways;
            flash.ops = 0;
            error = nullptr;
            EXPECT(WriteAndVerify(flash, retry, original, original, attempts, error) ==
                   WriteResult::kFailedNotRestored);
            EXPECT(attempts == kRestoreAttempts && error != nullptr);
        }
    }
    // Any single fault ends in a verified OK (fault missed) or a verified restore. Erases are the even operations.
    for (auto fault : {Fault::kEraseFails, Fault::kProgramFails, Fault::kProgramCorrupts}) {
        for (int at = 0; at < 2 + 2 * kRestoreAttempts; at++) {
            FlashSector flash;
            PlanAndApply(flash, false);
            WriteResult result = WriteWithFault(flash, true, fault, at, 1);
            const bool hits = (fault == Fault::kEraseFails) == (at % 2 == 0);
            EXPECT(result == (hits && at < 2 ? WriteResult::kFailedRestored : WriteResult::kOk));
        }
    }
}

// The CCFG WRITE FAILED banner, with the right recovery for each failure.
static std::vector<std::string> Banner(const WriteReport& report) {
    std::vector<std::string> lines;
    FailureBanner(report, [&lines](const char* line) { lines.emplace_back(line); });
    return lines;
}

static bool AnyLineHas(const std::vector<std::string>& lines, const char* text) {
    for (const auto& line : lines) {
        if (line.find(text) != std::string::npos) return true;
    }
    return false;
}

static void TestFailureBanner() {
    WriteReport r;
    r.enable = true;
    r.method = Method::kEraseAndProgram;
    r.old_bl_config = 0xC5FF0500;
    r.new_bl_config = 0xC5FF05C5;
    r.readback_bl_config = 0xFFFFFFFF;
    r.original_crc = 0x11111111;
    r.expected_crc = 0x66E9858D;
    r.readback_crc = 0x22222222;
    r.restore_attempts = kRestoreAttempts;
    r.error = "CCFG erase, program or verify failed";

    EXPECT(!WriteReport().Failed());  // kNotWritten: no banner, the CCFG wasn't touched.
    r.result = WriteResult::kOk;
    EXPECT(!r.Failed());

    r.result = WriteResult::kFailedNotRestored;
    EXPECT(r.Failed());
    std::vector<std::string> lines = Banner(r);
    EXPECT(lines.size() >= 10);
    EXPECT(lines.front() == lines.back() && lines.front().find("!!!!!!!!") == 0);
    EXPECT(AnyLineHas(lines, "CCFG WRITE FAILED: AT+BOOTLOADER_PIN=1 (enable"));
    EXPECT(AnyLineHas(lines, "erase and reprogram the CCFG sector, BL_CONFIG 0xC5FF0500 -> 0xC5FF05C5"));
    EXPECT(AnyLineHas(lines, "Failed: CCFG erase, program or verify failed"));
    EXPECT(AnyLineHas(lines, "BL_CONFIG 0xFFFFFFFF"));
    EXPECT(AnyLineHas(lines, "CRC32 0x22222222 (planned 0x66E9858D, original 0x11111111)"));
    EXPECT(AnyLineHas(lines, "DO NOT RESET OR POWER CYCLE"));
    EXPECT(AnyLineHas(lines, "failed 3 time(s)"));
    EXPECT(AnyLineHas(lines, "AT+BOOTLOADER_PIN=1,DEADBEE again now"));
    EXPECT(AnyLineHas(lines, "ADSBee 1421"));
    EXPECT(AnyLineHas(lines, "JTAG"));
    for (const auto& line : lines) {
        EXPECT(line.size() < kBannerLineMax - 1);  // Nothing cut off by the line buffer.
        EXPECT(line.compare(0, 3, "!!!") == 0);
    }

    r.result = WriteResult::kFailedRestored;
    r.restore_attempts = 2;
    r.readback_bl_config = 0xC5FF0500;
    lines = Banner(r);
    EXPECT(AnyLineHas(lines, "verified byte for byte (restore attempt 2 of 3)"));
    EXPECT(AnyLineHas(lines, "backdoor is still disabled"));
    EXPECT(AnyLineHas(lines, "can't"));  // The ADSBee 1421 Programmer can't help while the backdoor is off.
    EXPECT(!AnyLineHas(lines, "DO NOT RESET"));

    r.enable = false;
    r.method = Method::kProgramWord;
    r.old_bl_config = 0xC5FF05C5;
    r.new_bl_config = 0xC5FF0500;
    r.readback_bl_config = 0xC5FF0581;
    r.result = WriteResult::kFailedInPlace;
    r.error = "BL_CONFIG program or verify failed";
    lines = Banner(r);
    EXPECT(AnyLineHas(lines, "AT+BOOTLOADER_PIN=0 (disable"));
    EXPECT(AnyLineHas(lines, "program BL_CONFIG in place (no erase)"));
    EXPECT(AnyLineHas(lines, "BL_CONFIG 0xC5FF0581 (backdoor disabled)"));
    EXPECT(AnyLineHas(lines, "Retry AT+BOOTLOADER_PIN=0,DEADBEE"));
    EXPECT(AnyLineHas(lines, "AT+BOOTLOADER_PIN=1,DEADBEE rewrites"));
    EXPECT(!AnyLineHas(lines, "DO NOT RESET"));
    for (const auto& line : lines) EXPECT(line.size() < kBannerLineMax - 1);

    // Short form for the AT ERROR line.
    EXPECT(strstr(WriteResultStr(WriteResult::kFailedNotRestored), "DO NOT RESET"));
}

// A built image's .ccfg must match kReleaseCcfg, so CCFG changes in adsbee_1421.syscfg get tested here.
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
    EXPECT(StructCrc32(built.bytes) == 0x66E9858D);
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
    TestSetArgsPassword();
    TestCrc32();
    TestWriteFaultsDisable();
    TestWriteFaultsEnable();
    TestFailureBanner();
    if (failures) {
        printf("%d failure(s)\n", failures);
        return 1;
    }
    printf("all passed\n");
    return 0;
}
