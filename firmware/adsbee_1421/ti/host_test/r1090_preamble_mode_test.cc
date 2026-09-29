// Host tests for the 1090 MHz receiver mode values that settings persist in flash: stored values from
// older firmware must load as the documented modes (SettingsManager::R1090PreambleModeFromStored), and
// only the current mode names are selectable by name (removed modes have an empty name).
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "settings.hh"

static int failures = 0;

#define EXPECT(cond)                                                 \
    do {                                                             \
        if (!(cond)) {                                               \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            failures++;                                              \
        }                                                            \
    } while (0)

using SM = SettingsManager;

// Stored value -> mode name, as AT+R1090_PREAMBLE? reports it after loading.
static const char* LoadedName(uint8_t stored) {
    return SM::kR1090PreambleModeStrs[SM::R1090PreambleModeFromStored(stored)];
}

// Mode selected by an AT+R1090_PREAMBLE=<name> argument, or -1 if the name is rejected (same lookup as
// CommsManager::ATR1090PreambleCallback).
static int ModeForName(const char* name) {
    for (uint16_t i = 0; i < SM::kNumR1090PreambleModes; i++) {
        if (SM::kR1090PreambleModeStrs[i][0] != '\0' && strcmp(name, SM::kR1090PreambleModeStrs[i]) == 0) {
            return i;
        }
    }
    return -1;
}

static void TestPersistedValuesAreStable() {
    // These values are in flash on deployed units and must never change.
    static_assert(SM::kR1090PreambleModeRemovedModeSPreamble == 0, "0 was MODE_S_PREAMBLE.");
    static_assert(SM::kR1090PreambleModeDF17 == 1, "1 is DF17.");
    static_assert(SM::kR1090PreambleModeModeS == 2, "2 was MODE_S_SW_CRC, now MODE_S.");
    static_assert(SM::kR1090PreambleModeModeSStrong == 3, "3 is MODE_S_STRONG.");
    static_assert(SM::kR1090PreambleModeModeSWeak == 4, "4 is MODE_S_WEAK.");
    EXPECT(SM::kNumR1090PreambleModes == 5);
}

static void TestStoredValuesLoadAsDocumented() {
    // Settings saved by 0.3.11-rc3 and earlier.
    EXPECT(SM::R1090PreambleModeFromStored(0) == SM::kR1090PreambleModeModeS);  // MODE_S_PREAMBLE
    EXPECT(SM::R1090PreambleModeFromStored(1) == SM::kR1090PreambleModeDF17);   // DF17
    EXPECT(SM::R1090PreambleModeFromStored(2) == SM::kR1090PreambleModeModeS);  // MODE_S_SW_CRC
    // Current modes load as themselves.
    EXPECT(SM::R1090PreambleModeFromStored(3) == SM::kR1090PreambleModeModeSStrong);
    EXPECT(SM::R1090PreambleModeFromStored(4) == SM::kR1090PreambleModeModeSWeak);
    // Unknown values (settings from newer firmware) load as the factory default.
    for (unsigned v = SM::kNumR1090PreambleModes; v <= 0xFF; v++) {
        EXPECT(SM::R1090PreambleModeFromStored(static_cast<uint8_t>(v)) == SM::Settings().r1090_preamble_mode);
    }
    EXPECT(strcmp(LoadedName(0), "MODE_S") == 0);
    EXPECT(strcmp(LoadedName(2), "MODE_S") == 0);
    EXPECT(strcmp(LoadedName(0xFF), "DF17") == 0);
    // No stored value loads as the placeholder of a removed mode.
    for (unsigned v = 0; v <= 0xFF; v++) {
        EXPECT(SM::R1090PreambleModeFromStored(static_cast<uint8_t>(v)) != SM::kR1090PreambleModeRemovedModeSPreamble);
        EXPECT(LoadedName(static_cast<uint8_t>(v))[0] != '\0');
    }
}

static void TestFactoryDefaultIsDF17() { EXPECT(SM::Settings().r1090_preamble_mode == SM::kR1090PreambleModeDF17); }

static void TestOnlyCurrentNamesAreSelectable() {
    EXPECT(ModeForName("MODE_S") == SM::kR1090PreambleModeModeS);
    EXPECT(ModeForName("MODE_S_STRONG") == SM::kR1090PreambleModeModeSStrong);
    EXPECT(ModeForName("MODE_S_WEAK") == SM::kR1090PreambleModeModeSWeak);
    EXPECT(ModeForName("DF17") == SM::kR1090PreambleModeDF17);
    EXPECT(ModeForName("MODE_S_SW_CRC") == -1);
    EXPECT(ModeForName("MODE_S_PREAMBLE") == -1);
    EXPECT(ModeForName("") == -1);
}

int main() {
    TestPersistedValuesAreStable();
    TestStoredValuesLoadAsDocumented();
    TestFactoryDefaultIsDF17();
    TestOnlyCurrentNamesAreSelectable();
    if (failures) {
        printf("%d failure(s)\n", failures);
        return 1;
    }
    printf("r1090_preamble_mode_test: all passed\n");
    return 0;
}
