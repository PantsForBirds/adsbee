// Settings hashes printed by AT+CONFIG_HASH.
#include "gtest/gtest.h"
#include "settings_hash.hh"

using SM = SettingsManager;
using SettingsHash::kScopeFleet;
using SettingsHash::kScopeFull;

TEST(SettingsHash, SameSettingsSameHash) {
    SM::Settings a, b;
    EXPECT_EQ(SettingsHash::Compute(a, kScopeFleet), SettingsHash::Compute(b, kScopeFleet));
    EXPECT_EQ(SettingsHash::Compute(a, kScopeFull), SettingsHash::Compute(b, kScopeFull));
}

TEST(SettingsHash, DefaultSettingsHashIsPinned) {
    // Changes only with the Settings struct or its defaults.
    SM::Settings settings;
    EXPECT_EQ(SettingsHash::Compute(settings, kScopeFleet), 0x64FB12DDu);
    EXPECT_EQ(SettingsHash::Compute(settings, kScopeFull), 0x692A4AABu);
}

TEST(SettingsHash, MagicAndStoredCRCDoNotCount) {
    SM::Settings settings;
    uint32_t fleet = SettingsHash::Compute(settings, kScopeFleet);
    uint32_t full = SettingsHash::Compute(settings, kScopeFull);
    settings.Stamp();
    settings.magic = 0;
    EXPECT_EQ(SettingsHash::Compute(settings, kScopeFleet), fleet);
    EXPECT_EQ(SettingsHash::Compute(settings, kScopeFull), full);
}

template <typename Change>
static void ExpectChangesBoth(const char* name, Change change) {
    SM::Settings before, after;
    change(after);
    EXPECT_NE(SettingsHash::Compute(before, kScopeFleet), SettingsHash::Compute(after, kScopeFleet)) << name;
    EXPECT_NE(SettingsHash::Compute(before, kScopeFull), SettingsHash::Compute(after, kScopeFull)) << name;
}

TEST(SettingsHash, EverySettingChangesBothHashes) {
    ExpectChangesBoth("settings_version", [](SM::Settings& s) { s.settings_version++; });
    ExpectChangesBoth("r1090_rx_enabled", [](SM::Settings& s) { s.r1090_rx_enabled = false; });
    ExpectChangesBoth("watchdog_timeout_sec", [](SM::Settings& s) { s.watchdog_timeout_sec++; });
    ExpectChangesBoth("r1090_preamble_mode",
                      [](SM::Settings& s) { s.r1090_preamble_mode = SM::kR1090PreambleModeDF17; });
    ExpectChangesBoth("r1090_gain", [](SM::Settings& s) { s.r1090_gain = 13; });
    ExpectChangesBoth("r1090_rx_boost", [](SM::Settings& s) { s.r1090_rx_boost = 7; });
    ExpectChangesBoth("lr2021_enabled", [](SM::Settings& s) { s.lr2021_enabled = false; });
    ExpectChangesBoth("log_level", [](SM::Settings& s) { s.log_level = SM::kInfo; });
    ExpectChangesBoth("reporting_protocols", [](SM::Settings& s) { s.reporting_protocols[0] = SM::kMAVLINK2; });
    ExpectChangesBoth("baud_rates", [](SM::Settings& s) { s.baud_rates[0] = 115200; });
    ExpectChangesBoth("subg_rx_enabled", [](SM::Settings& s) { s.subg_rx_enabled = false; });
    ExpectChangesBoth("subg_mode", [](SM::Settings& s) { s.subg_mode = SM::kSubGHzRadioModeUATRxNoUplink; });
    ExpectChangesBoth("mavlink_system_id", [](SM::Settings& s) { s.mavlink_system_id++; });
    ExpectChangesBoth("mavlink_component_id", [](SM::Settings& s) { s.mavlink_component_id++; });
    ExpectChangesBoth("rx_position.source",
                      [](SM::Settings& s) { s.rx_position.source = SM::RxPosition::kPositionSourceGNSS; });
    ExpectChangesBoth("asterix_sac", [](SM::Settings& s) { s.asterix_sac = 25; });
    ExpectChangesBoth("led_enabled", [](SM::Settings& s) { s.led_enabled = false; });
}

TEST(SettingsHash, ASTERIXSICCountsOnlyInTheFullHash) {
    SM::Settings a, b;
    b.asterix_sic = 7;
    EXPECT_EQ(SettingsHash::Compute(a, kScopeFleet), SettingsHash::Compute(b, kScopeFleet));
    EXPECT_NE(SettingsHash::Compute(a, kScopeFull), SettingsHash::Compute(b, kScopeFull));
}

TEST(SettingsHash, ReceiverPositionCountsOnlyWhenItIsASetting) {
    // Followed aircraft and GNSS overwrite the position while running.
    SM::Settings a, b;
    b.rx_position.latitude_deg = 47.5f;
    b.rx_position.icao_address = 0xBEE001;
    EXPECT_EQ(SettingsHash::Compute(a, kScopeFull), SettingsHash::Compute(b, kScopeFull));

    a.rx_position.source = b.rx_position.source = SM::RxPosition::kPositionSourceFixed;
    EXPECT_EQ(SettingsHash::Compute(a, kScopeFleet), SettingsHash::Compute(b, kScopeFleet));
    EXPECT_NE(SettingsHash::Compute(a, kScopeFull), SettingsHash::Compute(b, kScopeFull));

    a.rx_position.source = b.rx_position.source = SM::RxPosition::kPositionSourceAircraftMatchingICAO;
    EXPECT_EQ(SettingsHash::Compute(a, kScopeFleet), SettingsHash::Compute(b, kScopeFleet));
    EXPECT_NE(SettingsHash::Compute(a, kScopeFull), SettingsHash::Compute(b, kScopeFull));
}
