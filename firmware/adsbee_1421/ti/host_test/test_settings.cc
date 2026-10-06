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
    static_assert(SM::kR1090PreambleModeDF17 == 0, "0 is DF17.");
    static_assert(SM::kR1090PreambleModeModeS == 1, "1 is MODE_S.");
    EXPECT_EQ(SM::kNumR1090PreambleModes, 2);
    for (uint16_t i = 0; i < SM::kNumR1090PreambleModes; i++) {
        EXPECT_STRNE(SM::kR1090PreambleModeStrs[i], "") << "mode " << i << " has a name";
    }
}

TEST(Settings, R1090PreambleModeFactoryDefaultIsModeS) {
    EXPECT_EQ(SM::Settings().r1090_preamble_mode, SM::kR1090PreambleModeModeS);
}

TEST(Settings, RemovedR1090PreambleModesLoadAsModeS) {
    EXPECT_EQ(SM::R1090PreambleModeFromStored(0), SM::kR1090PreambleModeDF17);
    EXPECT_EQ(SM::R1090PreambleModeFromStored(1), SM::kR1090PreambleModeModeS);
    EXPECT_EQ(SM::R1090PreambleModeFromStored(2), SM::kR1090PreambleModeModeS);  // MODE_S_STRONG
    EXPECT_EQ(SM::R1090PreambleModeFromStored(3), SM::kR1090PreambleModeModeS);  // MODE_S_SMART (pre-release)
    EXPECT_EQ(SM::R1090PreambleModeFromStored(4), SM::kR1090PreambleModeModeS);  // MODE_S_WIDE (pre-release)
    EXPECT_EQ(SM::R1090PreambleModeFromStored(0xFF), SM::kR1090PreambleModeModeS);
}

TEST(Settings, StoredStrongBlobIsKept) {
    // A version 4 blob saved by 0.3.11-rc4 with MODE_S_STRONG stays valid, so the other settings survive.
    SM::Settings settings;
    settings.baud_rates[0] = 115200;
    reinterpret_cast<uint8_t&>(settings.r1090_preamble_mode) = 2;
    settings.Stamp();
    EXPECT_TRUE(settings.IsValid());
    EXPECT_EQ(SM::R1090PreambleModeFromStored(settings.r1090_preamble_mode), SM::kR1090PreambleModeModeS);
    EXPECT_EQ(settings.baud_rates[0], 115200u);
}

TEST(Settings, R1090PreambleModeNamesSelectTheirModes) {
    EXPECT_EQ(ModeForName("DF17"), SM::kR1090PreambleModeDF17);
    EXPECT_EQ(ModeForName("MODE_S"), SM::kR1090PreambleModeModeS);
    EXPECT_EQ(ModeForName("MODE_S_STRONG"), -1);
    EXPECT_EQ(ModeForName("MODE_S_SMART"), -1);
    EXPECT_EQ(ModeForName("MODE_S_WIDE"), -1);
    EXPECT_EQ(ModeForName("MODE_S_SW_CRC"), -1);
    EXPECT_EQ(ModeForName("MODE_S_PREAMBLE"), -1);
    EXPECT_EQ(ModeForName("MODE_S_WEAK"), -1);
    EXPECT_EQ(ModeForName(""), -1);
}

TEST(Settings, VersionRenumberedModesIsCurrent) {
    // Version 4 renumbered R1090PreambleMode; anything stored under version 3 used the old numbers.
    EXPECT_EQ(kSettingsVersion, 4u);
    SM::Settings settings;
    settings.Stamp();
    EXPECT_TRUE(settings.IsValid());
}

TEST(Settings, BlobFromBeforeTheRenumberingIsRejected) {
    // Valid version 3 blob holding the old MODE_S_STRONG value (3).
    SM::Settings settings;
    settings.Stamp();
    settings.settings_version = 3;
    reinterpret_cast<uint8_t&>(settings.r1090_preamble_mode) = 3;
    settings.crc = settings.ComputeCRC();
    EXPECT_FALSE(settings.IsValid());
}

TEST(Settings, TornOrChangedBlobIsRejected) {
    SM::Settings settings;
    settings.Stamp();
    settings.r1090_preamble_mode = SM::kR1090PreambleModeDF17;  // Changed after stamping: stale CRC.
    EXPECT_FALSE(settings.IsValid());
}
