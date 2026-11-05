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
    // Version 5 numbers R1090PreambleMode as above; versions 6 and 7 keep that numbering.
    EXPECT_EQ(kSettingsVersion, 7u);
    SM::Settings settings;
    settings.Stamp();
    EXPECT_TRUE(settings.IsValid());
}

TEST(Settings, BlobFromAnOlderVersionIsRejected) {
    // Load() migrates or resets a blob of another version.
    SM::Settings settings;
    settings.Stamp();
    settings.settings_version = 5;
    settings.crc = settings.ComputeCRC();
    EXPECT_FALSE(settings.IsValid());
}

TEST(Settings, TornOrChangedBlobIsRejected) {
    SM::Settings settings;
    settings.Stamp();
    settings.r1090_preamble_mode = SM::kR1090PreambleModeDF17;  // Changed after stamping: stale CRC.
    EXPECT_FALSE(settings.IsValid());
}

// An older blob is today's struct up to the fields added since, then its CRC, then erased flash.
static void MakeOlderBlob(const SM::Settings& current, uint32_t version, uint16_t size, uint8_t* blob,
                          size_t blob_len) {
    memset(blob, 0xFF, blob_len);
    memcpy(blob, static_cast<const void*>(&current), size - sizeof(uint32_t));
    memcpy(blob + offsetof(SM::Settings, settings_version), &version, sizeof(version));
    uint32_t crc = crc32_ieee_802_3(blob, size - sizeof(uint32_t));
    memcpy(blob + size - sizeof(uint32_t), &crc, sizeof(crc));
}

TEST(Settings, Version5BlobMigratesWithReportFiltersOff) {
    SM::Settings current;
    current.mavlink_system_id = 42;
    current.r1090_preamble_mode = SM::kR1090PreambleModeDF17;
    current.rx_position.icao_address = 0xBEE001;
    uint8_t blob[sizeof(SM::Settings)];
    MakeOlderBlob(current, 5, SM::kVersion5SettingsSize, blob, sizeof(blob));

    SM::Settings out;
    out.filter_distance_enabled = true;
    out.filter_distance_deci_nm = 777;
    out.filter_altitude_mode = kAltitudeFilterNearRx;
    out.filter_altitude_a_ft = 300;
    out.filter_altitude_b_ft = 400;
    out.asterix_sac = 9;
    ASSERT_EQ(SM::MigrateFromOlderVersion(blob, out), 5u);
    EXPECT_TRUE(out.IsValid());
    EXPECT_EQ(out.settings_version, kSettingsVersion);
    EXPECT_EQ(out.mavlink_system_id, 42);
    EXPECT_EQ(out.r1090_preamble_mode, SM::kR1090PreambleModeDF17);
    EXPECT_EQ(out.rx_position.icao_address, 0xBEE001u);
    EXPECT_FALSE(out.filter_distance_enabled);
    EXPECT_EQ(out.filter_distance_deci_nm, SM::Settings::kDefaultFilterDistanceDeciNm);
    EXPECT_EQ(out.FilterDistanceMeters(), 0u);
    EXPECT_EQ(out.filter_altitude_mode, kAltitudeFilterOff);
    EXPECT_EQ(out.filter_altitude_a_ft, 0);
    EXPECT_EQ(out.filter_altitude_b_ft, 10000);
    EXPECT_EQ(out.asterix_sac, 0);
    EXPECT_TRUE(out.led_enabled);

    // A torn version 5 blob, or a blob of another version, is left to the reset path.
    blob[10] ^= 1;
    SM::Settings untouched;
    EXPECT_EQ(SM::MigrateFromOlderVersion(blob, untouched), 0u);
    blob[10] ^= 1;
    uint32_t version = 4;
    memcpy(blob + offsetof(SM::Settings, settings_version), &version, sizeof(version));
    EXPECT_EQ(SM::MigrateFromOlderVersion(blob, untouched), 0u);
}

TEST(Settings, Version6BlobMigratesWithLEDsEnabled) {
    // rc10 settings with non-default values everywhere they can be.
    SM::Settings current;
    current.r1090_rx_enabled = false;
    current.watchdog_timeout_sec = 0;
    current.log_level = SM::kInfo;
    current.baud_rates[0] = 921600;
    current.reporting_protocols[0] = SM::kMAVLINK2;
    current.subg_mode = SM::kSubGHzRadioModeUATRxNoUplink;
    current.filter_distance_enabled = true;
    current.filter_distance_deci_nm = 123;
    current.asterix_sic = 7;
    current.filter_altitude_mode = kAltitudeFilterBetween;
    current.filter_altitude_a_ft = 1000;
    current.filter_altitude_b_ft = 18000;
    current.led_enabled = false;  // Not in a version 6 blob: the migrated value must be the default.
    uint8_t blob[sizeof(SM::Settings)];
    MakeOlderBlob(current, 6, SM::kVersion6SettingsSize, blob, sizeof(blob));
    EXPECT_EQ(blob[SM::kVersion6SettingsSize], 0xFF);  // Erased flash where led_enabled would be.

    SM::Settings out;
    out.led_enabled = false;
    ASSERT_EQ(SM::MigrateFromOlderVersion(blob, out), 6u);
    EXPECT_TRUE(out.IsValid());
    EXPECT_EQ(out.settings_version, 7u);
    EXPECT_TRUE(out.led_enabled);
    // Every version 6 field is kept.
    SM::Settings expected = current;
    expected.led_enabled = true;
    expected.Stamp();
    EXPECT_EQ(memcmp(&out, &expected, sizeof(out)), 0);

    // A torn version 6 blob is left to the reset path.
    blob[SM::kVersion6SettingsSize - 5] ^= 1;
    EXPECT_EQ(SM::MigrateFromOlderVersion(blob, out), 0u);
}

TEST(Settings, CurrentOrUnknownBlobsAreNotMigrated) {
    SM::Settings current;
    current.Stamp();
    SM::Settings out;
    EXPECT_EQ(SM::MigrateFromOlderVersion(reinterpret_cast<const uint8_t*>(&current), out), 0u);
    uint8_t blank[sizeof(SM::Settings)];
    memset(blank, 0xFF, sizeof(blank));
    EXPECT_EQ(SM::MigrateFromOlderVersion(blank, out), 0u);
}

TEST(Settings, Version7BlobIsRejectedByVersion6Firmware) {
    // rc10 (version 6) checks magic, version 6 and the CRC over its first 74 bytes, so a version 7 blob fails its
    // check and rc10 resets to defaults on a downgrade.
    SM::Settings current;
    current.Stamp();
    uint32_t version;
    memcpy(&version, reinterpret_cast<const uint8_t*>(&current) + offsetof(SM::Settings, settings_version),
           sizeof(version));
    EXPECT_NE(version, 6u);
}

TEST(Settings, LEDsEnabledByDefault) { EXPECT_TRUE(SM::Settings().led_enabled); }

TEST(Settings, AltitudeFilterModeStrings) {
    uint8_t mode = kAltitudeFilterOff;
    EXPECT_TRUE(ParseAltitudeFilterMode("near_rx", 7, mode));
    EXPECT_EQ(mode, kAltitudeFilterNearRx);
    EXPECT_STREQ(AltitudeFilterModeStr(mode), "NEAR_RX");
    EXPECT_TRUE(ParseAltitudeFilterMode("Between", 7, mode));
    EXPECT_EQ(mode, kAltitudeFilterBetween);
    EXPECT_FALSE(ParseAltitudeFilterMode("3", 1, mode));
    EXPECT_FALSE(ParseAltitudeFilterMode("RANGE", 5, mode));
    EXPECT_EQ(mode, kAltitudeFilterBetween);
}

TEST(Settings, ASTERIXDataSourceDefaultsToZero) {
    SM::Settings settings;
    EXPECT_EQ(settings.asterix_sac, 0);
    EXPECT_EQ(settings.asterix_sic, 0);
    EXPECT_EQ(sizeof(SM::Settings), 79u);
}
