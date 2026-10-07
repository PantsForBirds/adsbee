// 1090 MHz receiver mode values stored in flash, and the settings version that guards their numbering.
#include <cstring>

#include "gtest/gtest.h"
#include "settings.hh"

using SM = SettingsManager;

// Same lookup as CommsManager::ATR1090PreambleCallback; -1 if the name is rejected.
static int ModeForName(const char* name) {
    for (uint16_t i = 0; i < SM::kNumR1090PreambleModes; i++) {
        if (strcmp(name, SM::kR1090PreambleModeStrs[i]) == 0) {
            return i;
        }
    }
    return -1;
}

TEST(Settings, R1090PreambleModesAreNumberedContiguously) {
    static_assert(SM::kR1090PreambleModeModeS == 0, "0 is MODE_S.");
    static_assert(SM::kR1090PreambleModeDF17 == 1, "1 is DF17.");
    EXPECT_EQ(SM::kNumR1090PreambleModes, 2);
    for (uint16_t i = 0; i < SM::kNumR1090PreambleModes; i++) {
        EXPECT_STRNE(SM::kR1090PreambleModeStrs[i], "") << "mode " << i << " has a name";
    }
}

TEST(Settings, R1090PreambleModeFactoryDefaultIsModeS) {
    EXPECT_EQ(SM::Settings().r1090_preamble_mode, SM::kR1090PreambleModeModeS);
}

TEST(Settings, R1090PreambleModeNamesSelectTheirModes) {
    EXPECT_EQ(ModeForName("MODE_S"), SM::kR1090PreambleModeModeS);
    EXPECT_EQ(ModeForName("DF17"), SM::kR1090PreambleModeDF17);
    EXPECT_EQ(ModeForName("mode_s"), -1);
    EXPECT_EQ(ModeForName(""), -1);
}

TEST(Settings, VersionNumberingModesIsCurrent) {
    // Version 5 numbers R1090PreambleMode as above.
    EXPECT_EQ(kSettingsVersion, 5u);
    SM::Settings settings;
    settings.Stamp();
    EXPECT_TRUE(settings.IsValid());
}

TEST(Settings, BlobFromAnOlderVersionIsRejected) {
    // A valid version 4 blob (0.3.11-rc4) fails the check, so Load() resets to factory defaults.
    SM::Settings settings;
    settings.Stamp();
    settings.settings_version = 4;
    settings.crc = settings.ComputeCRC();
    EXPECT_FALSE(settings.IsValid());
}

TEST(Settings, TornOrChangedBlobIsRejected) {
    SM::Settings settings;
    settings.Stamp();
    settings.r1090_preamble_mode = SM::kR1090PreambleModeDF17;  // Changed after stamping: stale CRC.
    EXPECT_FALSE(settings.IsValid());
}
