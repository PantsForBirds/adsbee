// The 1090 MHz receiver mode values that settings persist in flash: stored values from older firmware must load
// as the documented modes (SettingsManager::R1090PreambleModeFromStored), and only the current mode names are
// selectable by name (removed modes have an empty name).
#include <cstring>

#include "gtest/gtest.h"
#include "settings.hh"

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

TEST(Settings, R1090PreambleModePersistedValuesAreStable) {
    // These values are in flash on deployed units and must never change.
    static_assert(SM::kR1090PreambleModeRemovedModeSPreamble == 0, "0 was MODE_S_PREAMBLE.");
    static_assert(SM::kR1090PreambleModeDF17 == 1, "1 is DF17.");
    static_assert(SM::kR1090PreambleModeModeS == 2, "2 was MODE_S_SW_CRC, now MODE_S.");
    static_assert(SM::kR1090PreambleModeModeSStrong == 3, "3 is MODE_S_STRONG.");
    static_assert(SM::kR1090PreambleModeRemovedModeSWeak == 4, "4 was MODE_S_WEAK.");
    EXPECT_EQ(SM::kNumR1090PreambleModes, 5);
}

TEST(Settings, R1090PreambleModeStoredValuesLoadAsDocumented) {
    // Settings saved by 0.3.11-rc3 and earlier.
    EXPECT_EQ(SM::R1090PreambleModeFromStored(0), SM::kR1090PreambleModeModeS);  // MODE_S_PREAMBLE
    EXPECT_EQ(SM::R1090PreambleModeFromStored(1), SM::kR1090PreambleModeDF17);   // DF17
    EXPECT_EQ(SM::R1090PreambleModeFromStored(2), SM::kR1090PreambleModeModeS);  // MODE_S_SW_CRC
    // Current modes load as themselves.
    EXPECT_EQ(SM::R1090PreambleModeFromStored(3), SM::kR1090PreambleModeModeSStrong);
    // Removed MODE_S_WEAK (0.3.11-rc4 development builds) loads as the factory default.
    EXPECT_EQ(SM::R1090PreambleModeFromStored(4), SM::kR1090PreambleModeDF17);
    EXPECT_STREQ(LoadedName(4), "DF17");
    // Unknown values (settings from newer firmware) load as the factory default.
    for (unsigned v = SM::kNumR1090PreambleModes; v <= 0xFF; v++) {
        EXPECT_EQ(SM::R1090PreambleModeFromStored(static_cast<uint8_t>(v)), SM::Settings().r1090_preamble_mode)
            << "stored value " << v;
    }
    EXPECT_STREQ(LoadedName(0), "MODE_S");
    EXPECT_STREQ(LoadedName(2), "MODE_S");
    EXPECT_STREQ(LoadedName(0xFF), "DF17");
    // No stored value loads as the placeholder of a removed mode.
    for (unsigned v = 0; v <= 0xFF; v++) {
        EXPECT_NE(SM::R1090PreambleModeFromStored(static_cast<uint8_t>(v)), SM::kR1090PreambleModeRemovedModeSPreamble)
            << "stored value " << v;
        EXPECT_NE(SM::R1090PreambleModeFromStored(static_cast<uint8_t>(v)), SM::kR1090PreambleModeRemovedModeSWeak)
            << "stored value " << v;
        EXPECT_STRNE(LoadedName(static_cast<uint8_t>(v)), "") << "stored value " << v;
    }
}

TEST(Settings, R1090PreambleModeFactoryDefaultIsDF17) {
    EXPECT_EQ(SM::Settings().r1090_preamble_mode, SM::kR1090PreambleModeDF17);
}

TEST(Settings, R1090PreambleModeOnlyCurrentNamesAreSelectable) {
    EXPECT_EQ(ModeForName("MODE_S"), SM::kR1090PreambleModeModeS);
    EXPECT_EQ(ModeForName("MODE_S_STRONG"), SM::kR1090PreambleModeModeSStrong);
    EXPECT_EQ(ModeForName("DF17"), SM::kR1090PreambleModeDF17);
    EXPECT_EQ(ModeForName("MODE_S_SW_CRC"), -1);
    EXPECT_EQ(ModeForName("MODE_S_PREAMBLE"), -1);
    EXPECT_EQ(ModeForName("MODE_S_WEAK"), -1);
    EXPECT_EQ(ModeForName(""), -1);
}
