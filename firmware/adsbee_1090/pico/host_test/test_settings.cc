#include <cstring>

#include "gtest/gtest.h"
#include "feed_ram_budget.hh"
#include "settings.hh"
#include "settings_migration.hh"
#include "settings_versions.hh"

TEST(Settings, DeviceInfoGetDefaultSSIDAndUID) {
    SettingsManager::DeviceInfo device_info;
    strncpy(device_info.part_code, (const char*)"010240002F-20240907-986541",
            SettingsManager::DeviceInfo::kPartCodeLen + 1);

    char ssid_buf[SettingsManager::Settings::kWiFiSSIDMaxLen];
    device_info.GetDefaultSSID(ssid_buf);
    EXPECT_STREQ(ssid_buf, (const char*)"ADSBee1090-0240907986541");

    uint8_t uid_buf[SettingsManager::Settings::kFeedReceiverIDNumBytes];
    device_info.GetDefaultFeedReceiverID(uid_buf);
    EXPECT_EQ(uid_buf[0], 0xBE);
    EXPECT_EQ(uid_buf[1], 0xE0);

    uint64_t uid_val = 240907986541;
    EXPECT_EQ(uid_buf[2], (uid_val >> (8 * 5)) & 0xFF);
    EXPECT_EQ(uid_buf[3], (uid_val >> (8 * 4)) & 0xFF);
    EXPECT_EQ(uid_buf[4], (uid_val >> (8 * 3)) & 0xFF);
    EXPECT_EQ(uid_buf[5], (uid_val >> (8 * 2)) & 0xFF);
    EXPECT_EQ(uid_buf[6], (uid_val >> (8 * 1)) & 0xFF);
    EXPECT_EQ(uid_buf[7], uid_val & 0xFF);
}

TEST(Settings, CoreNetworkSettings) {
    SettingsManager::Settings test_settings;
    EXPECT_FALSE(test_settings.core_network_settings.IsValid());
    test_settings.core_network_settings.UpdateCRC32();
    EXPECT_EQ(test_settings.core_network_settings.crc32, test_settings.core_network_settings.CalculateCRC32());
    EXPECT_TRUE(test_settings.core_network_settings.IsValid());

    test_settings.core_network_settings.wifi_ap_enabled = !test_settings.core_network_settings.wifi_ap_enabled;
    EXPECT_FALSE(test_settings.core_network_settings.IsValid());
    test_settings.core_network_settings.UpdateCRC32();
    EXPECT_TRUE(test_settings.core_network_settings.IsValid());
}

TEST(SettingsMigration, LayoutLockedSizes) {
    // Belt-and-suspenders alongside the static_asserts in settings_versions.hh.
    EXPECT_EQ(sizeof(settings_v12::Settings), 1088u);
    EXPECT_EQ(sizeof(settings_v12::CoreNetworkSettings),
              sizeof(SettingsManager::Settings::CoreNetworkSettings));
    EXPECT_EQ(sizeof(settings_v12::RxPosition), sizeof(SettingsManager::RxPosition));

    EXPECT_EQ(sizeof(settings_v17::Settings), 1340u);
    EXPECT_EQ(sizeof(settings_v18::Settings), 1340u);
    EXPECT_EQ(sizeof(settings_v19::Settings), 1792u);
    EXPECT_EQ(sizeof(SettingsManager::Settings), 1792u);

    EXPECT_EQ(sizeof(settings_v14::Settings), 1140u);
    EXPECT_EQ(sizeof(settings_v14::CoreNetworkSettings),
              sizeof(SettingsManager::Settings::CoreNetworkSettings));
    EXPECT_EQ(sizeof(settings_v14::RxPosition), sizeof(SettingsManager::RxPosition));

    EXPECT_EQ(sizeof(settings_v15::Settings), 1144u);
    EXPECT_EQ(sizeof(settings_v15::CoreNetworkSettings),
              sizeof(SettingsManager::Settings::CoreNetworkSettings));
    EXPECT_EQ(sizeof(settings_v15::RxPosition), sizeof(SettingsManager::RxPosition));

    EXPECT_EQ(offsetof(settings_v17::Settings, ip_mode), offsetof(SettingsManager::Settings, ip_mode));
}

// Fields new in v14 (Remote ID transmit, GNSS) and v15 (feeds_enabled) must land on their defaults when migrating from
// a version that doesn't have them.
static void ExpectV14TxDefaults(const SettingsManager::Settings& out) {
    EXPECT_TRUE(out.feeds_enabled);
    EXPECT_FALSE(out.gnss_enabled);
    EXPECT_EQ(out.gnss_receiver_type, SettingsManager::kGNSSReceiverNone);
    EXPECT_FALSE(out.gnss_notify);
    EXPECT_FALSE(out.remote_id_tx_enabled);
    EXPECT_EQ(out.remote_id_tx_transports, (uint8_t)(SettingsManager::kRemoteIDTransportBLE4 |
                                                     SettingsManager::kRemoteIDTransportBLE5Long |
                                                     SettingsManager::kRemoteIDTransportWiFiBeacon));
    EXPECT_EQ(out.remote_id_tx_uas_id_type, 1);
    EXPECT_EQ(out.remote_id_tx_ua_type, 2);
    EXPECT_STREQ(out.remote_id_tx_uas_id, "");
    EXPECT_STREQ(out.remote_id_tx_operator_id, "");
}

// Fills a v14 settings struct with distinctive, non-default values across every field (including the Remote ID
// transmit settings, new in v14), and gives it a valid-CRC CoreNetworkSettings block.
static void PopulateV14(settings_v14::Settings& v14) {
    v14 = settings_v14::Settings{};
    v14.settings_version = 14;

    SettingsManager::Settings::CoreNetworkSettings cns;  // Zero-fills strings in its constructor.
    cns.esp32_enabled = true;
    strncpy(cns.wifi_ap_ssid, "MigrateNet", sizeof(cns.wifi_ap_ssid));
    strncpy(cns.wifi_ap_password, "supersecret", sizeof(cns.wifi_ap_password));
    cns.wifi_ap_channel = 6;
    cns.ethernet_enabled = true;
    cns.UpdateCRC32();
    ASSERT_EQ(sizeof(cns), sizeof(v14.core_network_settings));
    memcpy(&v14.core_network_settings, &cns, sizeof(cns));

    v14.r1090_rx_enabled = false;
    v14.tl_offset_mv = 123;
    v14.r1090_bias_tee_enabled = true;
    v14.watchdog_timeout_sec = 42;
    v14.led_enabled = false;
    v14.gnss_enabled = true;
    v14.gnss_receiver_type = SettingsManager::kGNSSReceiverUBXMIA;
    v14.gnss_notify = true;
    v14.log_level = SettingsManager::LogLevel::kInfo;
    v14.reporting_protocols[0] = SettingsManager::ReportingProtocol::kCSBee;
    v14.reporting_protocols[1] = SettingsManager::ReportingProtocol::kGDL90;
    v14.reporting_protocols[2] = SettingsManager::ReportingProtocol::kAircraftJSON;
    v14.baud_rates[0] = 1200;
    v14.baud_rates[1] = 57600;
    v14.baud_rates[2] = 4800;
    v14.subg_enabled = SettingsManager::EnableState::kEnableStateDisabled;
    v14.subg_rx_enabled = false;
    v14.subg_bias_tee_enabled = true;
    v14.subg_mode = SettingsManager::SubGHzRadioMode::kSubGHzRadioModeUATRx;
    v14.remote_id_rx_enabled = true;
    v14.remote_id_transports = SettingsManager::kRemoteIDTransportBLE5Long;
    v14.remote_id_tx_enabled = true;
    v14.remote_id_tx_transports = SettingsManager::kRemoteIDTransportWiFiBeacon;
    v14.remote_id_tx_uas_id_type = 3;
    v14.remote_id_tx_ua_type = 15;
    strncpy(v14.remote_id_tx_uas_id, "TXID123", sizeof(v14.remote_id_tx_uas_id));
    strncpy(v14.remote_id_tx_operator_id, "OP456", sizeof(v14.remote_id_tx_operator_id));

    strncpy(v14.feed_uris[0], "myfeed.example.com", sizeof(v14.feed_uris[0]));
    strncpy(v14.feed_uris[9], "feed.adsb.fi", sizeof(v14.feed_uris[9]));
    v14.feed_ports[0] = 30005;
    v14.feed_ports[9] = 30004;
    v14.feed_is_active[0] = true;
    v14.feed_is_active[9] = true;
    v14.feed_protocols[0] = SettingsManager::ReportingProtocol::kBeast;
    v14.feed_protocols[9] = SettingsManager::ReportingProtocol::kBeast;
    v14.feed_receiver_ids[0][0] = 0xDE;
    v14.feed_receiver_ids[0][7] = 0xAD;

    v14.mavlink_system_id = 7;
    v14.mavlink_component_id = 42;

    v14.rx_position.source = 1;  // kPositionSourceFixed
    v14.rx_position.latitude_deg = 37.5f;
    v14.rx_position.longitude_deg = -122.3f;
    v14.rx_position.gnss_altitude_ft = 111;
    v14.rx_position.baro_altitude_ft = 222;
    v14.rx_position.heading_deg = 90.0f;
    v14.rx_position.speed_kts = 33;
    v14.rx_position.icao_address = 0xABCDEF;
}

TEST(SettingsMigration, V14ToV15PreservesAllFieldsAndDefaultsFeedsEnabled) {
    settings_v14::Settings v14;
    PopulateV14(v14);

    uint8_t blob[sizeof(settings_v14::Settings)];
    memcpy(blob, &v14, sizeof(v14));

    SettingsManager::Settings out;
    ASSERT_TRUE(SettingsMigrator::Migrate(blob, sizeof(blob), 14, out));

    EXPECT_EQ(out.settings_version, kSettingsVersion);

    SettingsManager::Settings::CoreNetworkSettings cns_out = out.core_network_settings;
    EXPECT_TRUE(cns_out.IsValid());
    EXPECT_STREQ(out.core_network_settings.wifi_ap_ssid, "MigrateNet");

    EXPECT_FALSE(out.r1090_rx_enabled);
    EXPECT_EQ(out.tl_offset_mv, 123);
    EXPECT_TRUE(out.r1090_bias_tee_enabled);
    EXPECT_EQ(out.watchdog_timeout_sec, 42u);
    EXPECT_FALSE(out.led_enabled);

    EXPECT_TRUE(out.feeds_enabled);  // New in v15.
    EXPECT_EQ(out.ip_mode, SettingsManager::kIPModeDual);  // New in v17 (added in the unshipped v16).
    EXPECT_TRUE(out.gnss_enabled);
    EXPECT_EQ(out.gnss_receiver_type, SettingsManager::kGNSSReceiverUBXMIA);
    EXPECT_TRUE(out.gnss_notify);

    EXPECT_EQ(out.log_level, SettingsManager::LogLevel::kInfo);
    EXPECT_EQ(out.reporting_protocols[0], SettingsManager::ReportingProtocol::kCSBee);
    EXPECT_EQ(out.baud_rates[0], 1200u);

    EXPECT_EQ(out.subg_enabled, SettingsManager::EnableState::kEnableStateDisabled);
    EXPECT_FALSE(out.subg_rx_enabled);
    EXPECT_TRUE(out.subg_bias_tee_enabled);

    EXPECT_TRUE(out.remote_id_rx_enabled);
    EXPECT_EQ(out.remote_id_transports, (uint8_t)SettingsManager::kRemoteIDTransportBLE5Long);
    EXPECT_TRUE(out.remote_id_tx_enabled);
    EXPECT_EQ(out.remote_id_tx_transports, (uint8_t)SettingsManager::kRemoteIDTransportWiFiBeacon);
    EXPECT_EQ(out.remote_id_tx_uas_id_type, 3);
    EXPECT_EQ(out.remote_id_tx_ua_type, 15);
    EXPECT_STREQ(out.remote_id_tx_uas_id, "TXID123");
    EXPECT_STREQ(out.remote_id_tx_operator_id, "OP456");

    EXPECT_STREQ(out.feed_uris[0], "myfeed.example.com");
    EXPECT_EQ(out.feed_ports[0], 30005);
    EXPECT_TRUE(out.feed_is_active[0]);
    EXPECT_EQ(out.feed_protocols[0], SettingsManager::ReportingProtocol::kBeast);
    EXPECT_EQ(out.feed_receiver_ids[0][0], 0xDE);

    EXPECT_EQ(out.mavlink_system_id, 7);
    EXPECT_EQ(out.mavlink_component_id, 42);

    EXPECT_EQ(out.rx_position.source, SettingsManager::RxPosition::kPositionSourceFixed);
    EXPECT_FLOAT_EQ(out.rx_position.latitude_deg, 37.5f);
    EXPECT_EQ(out.rx_position.icao_address, 0xABCDEFu);
}

// Fills a v15 settings struct with distinctive, non-default values across every field.
static void PopulateV15(settings_v15::Settings& v15) {
    v15 = settings_v15::Settings{};
    v15.settings_version = 15;

    SettingsManager::Settings::CoreNetworkSettings cns;  // Zero-fills strings in its constructor.
    cns.esp32_enabled = true;
    strncpy(cns.wifi_ap_ssid, "MigrateNet", sizeof(cns.wifi_ap_ssid));
    strncpy(cns.wifi_ap_password, "supersecret", sizeof(cns.wifi_ap_password));
    cns.wifi_ap_channel = 6;
    cns.ethernet_enabled = true;
    cns.UpdateCRC32();
    ASSERT_EQ(sizeof(cns), sizeof(v15.core_network_settings));
    memcpy(&v15.core_network_settings, &cns, sizeof(cns));

    v15.r1090_rx_enabled = false;
    v15.tl_offset_mv = 123;
    v15.r1090_bias_tee_enabled = true;
    v15.watchdog_timeout_sec = 42;
    v15.led_enabled = false;
    v15.feeds_enabled = false;
    v15.gnss_enabled = true;
    v15.gnss_receiver_type = SettingsManager::kGNSSReceiverUBXMIA;
    v15.gnss_notify = true;
    v15.log_level = SettingsManager::LogLevel::kInfo;
    v15.reporting_protocols[0] = SettingsManager::ReportingProtocol::kCSBee;
    v15.reporting_protocols[1] = SettingsManager::ReportingProtocol::kGDL90;
    v15.reporting_protocols[2] = SettingsManager::ReportingProtocol::kAircraftJSON;
    v15.baud_rates[0] = 1200;
    v15.baud_rates[1] = 57600;
    v15.baud_rates[2] = 4800;
    v15.subg_enabled = SettingsManager::EnableState::kEnableStateDisabled;
    v15.subg_rx_enabled = false;
    v15.subg_bias_tee_enabled = true;
    v15.subg_mode = SettingsManager::SubGHzRadioMode::kSubGHzRadioModeUATRx;
    v15.remote_id_rx_enabled = true;
    v15.remote_id_transports = SettingsManager::kRemoteIDTransportBLE5Long;
    v15.remote_id_tx_enabled = true;
    v15.remote_id_tx_transports = SettingsManager::kRemoteIDTransportWiFiBeacon;
    v15.remote_id_tx_uas_id_type = 3;
    v15.remote_id_tx_ua_type = 15;
    strncpy(v15.remote_id_tx_uas_id, "TXID123", sizeof(v15.remote_id_tx_uas_id));
    strncpy(v15.remote_id_tx_operator_id, "OP456", sizeof(v15.remote_id_tx_operator_id));

    strncpy(v15.feed_uris[0], "myfeed.example.com", sizeof(v15.feed_uris[0]));
    strncpy(v15.feed_uris[9], "feed.adsb.fi", sizeof(v15.feed_uris[9]));
    v15.feed_ports[0] = 30005;
    v15.feed_ports[9] = 30004;
    v15.feed_is_active[0] = true;
    v15.feed_is_active[9] = true;
    v15.feed_protocols[0] = SettingsManager::ReportingProtocol::kBeast;
    v15.feed_protocols[9] = SettingsManager::ReportingProtocol::kBeast;
    v15.feed_receiver_ids[0][0] = 0xDE;
    v15.feed_receiver_ids[0][7] = 0xAD;

    v15.mavlink_system_id = 7;
    v15.mavlink_component_id = 42;

    v15.rx_position.source = 1;  // kPositionSourceFixed
    v15.rx_position.latitude_deg = 37.5f;
    v15.rx_position.longitude_deg = -122.3f;
    v15.rx_position.gnss_altitude_ft = 111;
    v15.rx_position.baro_altitude_ft = 222;
    v15.rx_position.heading_deg = 90.0f;
    v15.rx_position.speed_kts = 33;
    v15.rx_position.icao_address = 0xABCDEF;
}

TEST(SettingsMigration, V15ToCurrentPreservesAllFieldsAndAddsEmptyWiFiNetworks) {
    settings_v15::Settings v15;
    PopulateV15(v15);

    uint8_t blob[sizeof(settings_v15::Settings)];
    memcpy(blob, &v15, sizeof(v15));

    SettingsManager::Settings out;
    ASSERT_TRUE(SettingsMigrator::Migrate(blob, sizeof(blob), 15, out));
    EXPECT_EQ(out.settings_version, kSettingsVersion);
    EXPECT_EQ(memcmp(&out.core_network_settings, &v15.core_network_settings, sizeof(v15.core_network_settings)), 0);

    EXPECT_FALSE(out.r1090_rx_enabled);
    EXPECT_EQ(out.tl_offset_mv, 123);
    EXPECT_EQ(out.watchdog_timeout_sec, 42u);
    EXPECT_FALSE(out.led_enabled);
    EXPECT_FALSE(out.feeds_enabled);
    EXPECT_EQ(out.gnss_receiver_type, SettingsManager::kGNSSReceiverUBXMIA);
    EXPECT_EQ(out.log_level, SettingsManager::LogLevel::kInfo);
    EXPECT_EQ(out.reporting_protocols[2], SettingsManager::ReportingProtocol::kAircraftJSON);
    EXPECT_EQ(out.baud_rates[2], 4800u);
    EXPECT_EQ(out.subg_enabled, SettingsManager::EnableState::kEnableStateDisabled);
    EXPECT_EQ(out.remote_id_tx_ua_type, 15);
    EXPECT_STREQ(out.remote_id_tx_operator_id, "OP456");
    EXPECT_STREQ(out.feed_uris[0], "myfeed.example.com");
    EXPECT_EQ(out.feed_protocols[9], SettingsManager::ReportingProtocol::kBeast);
    EXPECT_EQ(out.feed_receiver_ids[0][7], 0xAD);
    EXPECT_EQ(out.mavlink_component_id, 42);
    EXPECT_EQ(out.rx_position.icao_address, 0xABCDEFu);

    for (uint16_t i = 0; i < SettingsManager::Settings::kWiFiSTANumExtraNetworks; i++) {
        EXPECT_STREQ(out.wifi_sta_extra_networks[i].ssid, "");
        EXPECT_STREQ(out.wifi_sta_extra_networks[i].password, "");
    }
}

TEST(SettingsMigration, V17ToV18KeepsWiFiNetworksAndLeavesAutoUpdateOff) {
    settings_v15::Settings v15;
    PopulateV15(v15);
    // 0xFF where v18 put auto_update_enabled (v17 tail padding): stored padding must not turn the feature on.
    uint8_t blob[sizeof(settings_v17::Settings)];
    memset(blob, 0xFF, sizeof(blob));
    settings_v17::Settings v17;
    memset(&v17, 0, sizeof(v17));
    memcpy(&v17, &v15, sizeof(v15));
    v17.settings_version = 17;
    v17.ip_mode = SettingsManager::kIPModeIPv4;
    strcpy(v17.wifi_sta_extra_networks[1].ssid, "Hangar");
    strcpy(v17.wifi_sta_extra_networks[1].password, "wind sock");
    memcpy(blob, &v17, offsetof(settings_v17::Settings, wifi_sta_extra_networks) + sizeof(v17.wifi_sta_extra_networks));

    SettingsManager::Settings out;
    out.auto_update_enabled = true;
    ASSERT_TRUE(SettingsMigrator::Migrate(blob, sizeof(blob), 17, out));
    EXPECT_EQ(out.settings_version, kSettingsVersion);
    EXPECT_FALSE(out.auto_update_enabled);
    EXPECT_EQ(memcmp(&out.core_network_settings, &v17.core_network_settings, sizeof(v17.core_network_settings)), 0);
    EXPECT_EQ(out.ip_mode, SettingsManager::kIPModeIPv4);
    EXPECT_STREQ(out.wifi_sta_extra_networks[0].ssid, "");
    EXPECT_STREQ(out.wifi_sta_extra_networks[1].ssid, "Hangar");
    EXPECT_STREQ(out.wifi_sta_extra_networks[1].password, "wind sock");
    EXPECT_FALSE(out.feeds_enabled);
    EXPECT_STREQ(out.feed_uris[0], "myfeed.example.com");
    EXPECT_EQ(out.rx_position.icao_address, 0xABCDEFu);
}

TEST(SettingsMigration, V18ToCurrentKeepsAutoUpdateAndLeavesReportFiltersOff) {
    settings_v15::Settings v15;
    PopulateV15(v15);
    settings_v18::Settings v18;
    memset(&v18, 0, sizeof(v18));
    memcpy(&v18, &v15, sizeof(v15));
    v18.settings_version = 18;
    v18.ip_mode = SettingsManager::kIPModeIPv6;
    strcpy(v18.wifi_sta_extra_networks[0].ssid, "Tower");
    v18.auto_update_enabled = true;

    SettingsManager::Settings out;
    out.filter_distance_enabled = true;
    out.filter_distance_deci_nm = 12345;
    out.filter_altitude_mode = kAltitudeFilterBetween;
    out.filter_altitude_a_ft = 500;
    out.filter_altitude_b_ft = 600;
    out.swarm_enabled = true;
    strcpy(out.swarm_server, "http://stale");
    strcpy(out.swarm_token, "stale");
    ASSERT_TRUE(SettingsMigrator::Migrate(reinterpret_cast<const uint8_t*>(&v18), sizeof(v18), 18, out));
    EXPECT_EQ(out.settings_version, kSettingsVersion);
    EXPECT_FALSE(out.filter_distance_enabled);
    EXPECT_EQ(out.filter_distance_deci_nm, SettingsManager::Settings::kDefaultFilterDistanceDeciNm);
    EXPECT_EQ(out.FilterDistanceMeters(), 0u);
    EXPECT_EQ(out.filter_altitude_mode, kAltitudeFilterOff);
    EXPECT_EQ(out.filter_altitude_a_ft, 0);
    EXPECT_EQ(out.filter_altitude_b_ft, 10000);
    EXPECT_FALSE(out.swarm_enabled);
    EXPECT_STREQ(out.swarm_server, "https://adsbee.aero");
    EXPECT_STREQ(out.swarm_token, "");
    EXPECT_TRUE(out.auto_update_enabled);
    EXPECT_EQ(out.ip_mode, SettingsManager::kIPModeIPv6);
    EXPECT_STREQ(out.wifi_sta_extra_networks[0].ssid, "Tower");
    EXPECT_EQ(memcmp(&out.core_network_settings, &v18.core_network_settings, sizeof(v18.core_network_settings)), 0);
    EXPECT_STREQ(out.feed_uris[0], "myfeed.example.com");
    EXPECT_EQ(out.rx_position.icao_address, 0xABCDEFu);

    // A blob cut short of the v18 layout is rejected.
    EXPECT_FALSE(SettingsMigrator::Migrate(reinterpret_cast<const uint8_t*>(&v18), sizeof(v18) - 1, 18, out));
}

TEST(Settings, DistanceFilterDefaultsOffAndRoundTrips) {
    SettingsManager manager;
    EXPECT_FALSE(manager.settings.filter_distance_enabled);
    EXPECT_EQ(manager.settings.filter_distance_deci_nm, 1000u);  // 100.0 nm.
    EXPECT_EQ(manager.settings.FilterDistanceMeters(), 0u);      // Off.
    manager.settings.filter_distance_enabled = true;
    manager.settings.filter_distance_deci_nm = 355;  // 35.5 nm.
    EXPECT_EQ(manager.settings.FilterDistanceMeters(), 65746u);
    testing::internal::CaptureStdout();
    manager.PrintAT();
    std::string dump = testing::internal::GetCapturedStdout();
    EXPECT_NE(dump.find("AT+FILTER_DISTANCE_NM=1,35.5\r\n"), std::string::npos);
    EXPECT_FALSE(manager.Sanitize());

    // Out-of-range distances and garbage booleans from flash are repaired.
    manager.settings.filter_distance_deci_nm = 0;
    EXPECT_TRUE(manager.Sanitize());
    EXPECT_EQ(manager.settings.filter_distance_deci_nm, 1000u);
    manager.settings.filter_distance_deci_nm = 100001;
    EXPECT_TRUE(manager.Sanitize());
    EXPECT_EQ(manager.settings.filter_distance_deci_nm, 1000u);
    uint8_t raw = 0x5A;
    memcpy(&manager.settings.filter_distance_enabled, &raw, 1);
    EXPECT_TRUE(manager.Sanitize());
}

TEST(Settings, SwarmDefaultsOffAndDumpsItsToken) {
    SettingsManager manager;
    EXPECT_FALSE(manager.settings.swarm_enabled);
    EXPECT_STREQ(manager.settings.swarm_server, "https://adsbee.aero");
    EXPECT_STREQ(manager.settings.swarm_token, "");
    manager.settings.swarm_enabled = true;
    strcpy(manager.settings.swarm_server, "http://10.0.0.2:8000");
    strcpy(manager.settings.swarm_token, "0123456789abcdef0123456789abcdef");
    testing::internal::CaptureStdout();
    manager.PrintAT();
    std::string dump = testing::internal::GetCapturedStdout();
    // The dump restores settings, so it carries the token.
    EXPECT_NE(dump.find("AT+SWARM=1,http://10.0.0.2:8000,0123456789abcdef0123456789abcdef\r\n"), std::string::npos);
    EXPECT_FALSE(manager.Sanitize());

    uint8_t raw = 0x5A;
    memcpy(&manager.settings.swarm_enabled, &raw, 1);
    memset(manager.settings.swarm_token, 'x', sizeof(manager.settings.swarm_token));  // No terminator.
    EXPECT_TRUE(manager.Sanitize());
    EXPECT_EQ(manager.settings.swarm_token[sizeof(manager.settings.swarm_token) - 1], '\0');
}

TEST(Settings, AltitudeFilterDefaultsOffAndRoundTrips) {
    SettingsManager manager;
    EXPECT_EQ(manager.settings.filter_altitude_mode, kAltitudeFilterOff);
    EXPECT_EQ(manager.settings.filter_altitude_a_ft, 0);
    EXPECT_EQ(manager.settings.filter_altitude_b_ft, 10000);
    manager.settings.filter_altitude_mode = kAltitudeFilterBetween;
    manager.settings.filter_altitude_a_ft = -1500;
    manager.settings.filter_altitude_b_ft = 18000;
    testing::internal::CaptureStdout();
    manager.PrintAT();
    std::string dump = testing::internal::GetCapturedStdout();
    EXPECT_NE(dump.find("AT+FILTER_ALTITUDE_FT=BETWEEN,-1500,18000\r\n"), std::string::npos);
    EXPECT_FALSE(manager.Sanitize());

    // Invalid combinations from flash turn the filter off.
    struct {
        uint8_t mode;
        int32_t a_ft, b_ft;
    } invalid[] = {{5, 0, 10000},           {255, 0, 10000},     {kAltitudeFilterAbove, -2001, 0},
                   {kAltitudeFilterBelow, 100001, 0}, {kAltitudeFilterBetween, 6000, 5000},
                   {kAltitudeFilterBetween, 0, 100001}, {kAltitudeFilterNearRx, 0, 0},
                   {kAltitudeFilterNearRx, -100, 0}, {kAltitudeFilterOff, 0, INT32_MIN}};
    for (const auto& c : invalid) {
        manager.settings.filter_altitude_mode = c.mode;
        manager.settings.filter_altitude_a_ft = c.a_ft;
        manager.settings.filter_altitude_b_ft = c.b_ft;
        EXPECT_TRUE(manager.Sanitize()) << int(c.mode) << "," << c.a_ft << "," << c.b_ft;
        EXPECT_EQ(manager.settings.filter_altitude_mode, kAltitudeFilterOff);
        EXPECT_EQ(manager.settings.filter_altitude_a_ft, 0);
        EXPECT_EQ(manager.settings.filter_altitude_b_ft, 10000);
    }
}

TEST(SettingsMigration, V18ToCurrentKeepsFeedsOnTCP) {
    settings_v15::Settings v15;
    PopulateV15(v15);
    settings_v18::Settings v18;
    memset(&v18, 0, sizeof(v18));
    memcpy(&v18, &v15, sizeof(v15));
    v18.settings_version = 18;
    v18.ip_mode = SettingsManager::kIPModeIPv4;
    strcpy(v18.wifi_sta_extra_networks[0].ssid, "Hangar");
    v18.auto_update_enabled = true;
    uint8_t blob[sizeof(settings_v18::Settings)];
    memcpy(blob, &v18, sizeof(v18));

    SettingsManager::Settings out;
    out.feed_transports[0] = SettingsManager::kFeedTransportMQTT;
    strcpy(out.mqtt_broker.username, "stale");
    ASSERT_TRUE(SettingsMigrator::Migrate(blob, sizeof(blob), 18, out));
    EXPECT_EQ(out.settings_version, kSettingsVersion);
    EXPECT_TRUE(out.auto_update_enabled);
    EXPECT_EQ(out.ip_mode, SettingsManager::kIPModeIPv4);
    EXPECT_STREQ(out.wifi_sta_extra_networks[0].ssid, "Hangar");
    EXPECT_STREQ(out.feed_uris[0], "myfeed.example.com");
    EXPECT_EQ(out.feed_protocols[0], v15.feed_protocols[0]);
    EXPECT_EQ(out.feed_ports[0], v15.feed_ports[0]);
    EXPECT_EQ(out.rx_position.icao_address, 0xABCDEFu);
    for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
        EXPECT_EQ(out.feed_transports[i], SettingsManager::kFeedTransportTCP);
    }
    EXPECT_STREQ(out.mqtt_broker.host, "");
    EXPECT_EQ(out.mqtt_broker.port, SettingsManager::kDefaultMQTTPort);
    EXPECT_STREQ(out.mqtt_broker.username, "");
    EXPECT_STREQ(out.mqtt_broker.password, "");
    EXPECT_STREQ(out.mqtt_broker.topic_prefix, "");
    EXPECT_STREQ(out.mqtt_broker.websocket_path, "");
    EXPECT_FALSE(out.mqtt_broker.tls);
    EXPECT_EQ(out.mqtt_broker.qos, 0);
    EXPECT_EQ(out.mqtt_broker.transport, SettingsManager::kMQTTTransportTCP);
}

TEST(Settings, FeedTransportStrings) {
    SettingsManager::FeedTransport transport = SettingsManager::kFeedTransportTCP;
    EXPECT_TRUE(SettingsManager::FeedTransportFromStr("udp", transport));
    EXPECT_EQ(transport, SettingsManager::kFeedTransportUDP);
    EXPECT_TRUE(SettingsManager::FeedTransportFromStr("MQTT", transport));
    EXPECT_EQ(transport, SettingsManager::kFeedTransportMQTT);
    EXPECT_TRUE(SettingsManager::FeedTransportFromStr("TCP", transport));
    EXPECT_EQ(transport, SettingsManager::kFeedTransportTCP);
    EXPECT_FALSE(SettingsManager::FeedTransportFromStr("WS", transport));
    EXPECT_EQ(transport, SettingsManager::kFeedTransportTCP);
    for (uint16_t i = 0; i < SettingsManager::kNumFeedTransports; i++) {
        EXPECT_TRUE(SettingsManager::FeedTransportFromStr(SettingsManager::kFeedTransportStrs[i], transport));
        EXPECT_EQ(transport, i);
    }
}

TEST(Settings, DefaultFeedTransports) {
    typedef SettingsManager S;
    for (S::ReportingProtocol p : {S::kNoReports, S::kRaw, S::kBeast, S::kBeastNoUAT, S::kBeastNoUATUplink, S::kCSBee}) {
        EXPECT_EQ(S::DefaultFeedTransport(p), S::kFeedTransportTCP) << S::kReportingProtocolStrs[p];
    }
    for (S::ReportingProtocol p : {S::kGDL90, S::kGDL90NoUATUplink, S::kMAVLINK1, S::kMAVLINK2, S::kASTERIX}) {
        EXPECT_EQ(S::DefaultFeedTransport(p), S::kFeedTransportUDP) << S::kReportingProtocolStrs[p];
    }
    for (S::ReportingProtocol p : {S::kAircraftJSON, S::kAircraftJSONUATUplink}) {
        EXPECT_EQ(S::DefaultFeedTransport(p), S::kFeedTransportMQTT) << S::kReportingProtocolStrs[p];
    }
}

TEST(Settings, FeedTransportAndMQTTDumpReplaysOnOlderFirmware) {
    SettingsManager manager;
    for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
        EXPECT_EQ(manager.settings.feed_transports[i], SettingsManager::kFeedTransportTCP);
    }
    manager.settings.feed_transports[2] = SettingsManager::kFeedTransportUDP;
    manager.settings.feed_transports[3] = SettingsManager::kFeedTransportMQTT;
    SettingsManager::Settings::MQTTBroker& mqtt = manager.settings.mqtt_broker;
    strcpy(mqtt.host, "broker.lan");
    mqtt.port = 443;
    strcpy(mqtt.topic_prefix, "home/adsb");
    strcpy(mqtt.username, "bee");
    strcpy(mqtt.password, "pollen");
    mqtt.tls = true;
    mqtt.qos = 1;
    mqtt.transport = SettingsManager::kMQTTTransportWebSocket;

    testing::internal::CaptureStdout();
    manager.PrintAT();
    std::string dump = testing::internal::GetCapturedStdout();
    // The five-argument AT+FEED lines stay where older firmware expects them; the new commands come last, and every
    // feed's transport is replayed because a five-argument AT+FEED keeps it.
    EXPECT_NE(dump.find("AT+FEED=2,,0,0,NONE\r\n"), std::string::npos);
    size_t filter = dump.find("AT+FILTER_DISTANCE_NM=");
    size_t broker = dump.find("AT+MQTT=broker.lan,443,bee,pollen,1,1,home/adsb,WS,-\r\n");
    size_t tcp = dump.find("AT+FEED=0,,,,,TCP\r\n");
    size_t udp = dump.find("AT+FEED=2,,,,,UDP\r\n");
    size_t mqtt_transport = dump.find("AT+FEED=3,,,,,MQTT\r\n");
    ASSERT_NE(filter, std::string::npos);
    ASSERT_NE(broker, std::string::npos);
    ASSERT_NE(tcp, std::string::npos);
    ASSERT_NE(udp, std::string::npos);
    ASSERT_NE(mqtt_transport, std::string::npos);
    EXPECT_LT(filter, broker);
    EXPECT_LT(broker, tcp);

    // Out-of-range values from a corrupted blob.
    manager.settings.feed_transports[4] = static_cast<SettingsManager::FeedTransport>(7);
    mqtt.qos = 9;
    mqtt.transport = static_cast<SettingsManager::MQTTTransport>(5);
    memset(mqtt.password, 'x', sizeof(mqtt.password));
    EXPECT_TRUE(manager.Sanitize());
    EXPECT_EQ(manager.settings.feed_transports[4], SettingsManager::kFeedTransportTCP);
    EXPECT_EQ(mqtt.qos, SettingsManager::Settings::kMQTTMaxQoS);
    EXPECT_EQ(mqtt.transport, SettingsManager::kMQTTTransportTCP);
    EXPECT_EQ(strlen(mqtt.password), SettingsManager::Settings::kMQTTPasswordMaxLen);
    EXPECT_FALSE(manager.Sanitize());
}

TEST(SettingsMigration, V18ToCurrentKeepsEverythingAndDefaultsASTERIXNTPAndAltitudeFilter) {
    settings_v15::Settings v15;
    PopulateV15(v15);
    // 0xFF past the v18 struct, as Load() reads erased EEPROM there.
    uint8_t blob[sizeof(SettingsManager::Settings)];
    memset(blob, 0xFF, sizeof(blob));
    settings_v18::Settings v18;
    memset(&v18, 0, sizeof(v18));
    memcpy(&v18, &v15, sizeof(v15));
    v18.settings_version = 18;
    v18.ip_mode = SettingsManager::kIPModeIPv6;
    strcpy(v18.wifi_sta_extra_networks[0].ssid, "Tower");
    v18.auto_update_enabled = true;
    memcpy(blob, &v18, sizeof(v18));

    SettingsManager::Settings out;
    out.asterix_sac = 0x55;
    out.asterix_sic = 0x66;
    strcpy(out.ntp_server, "stale.example.com");
    out.filter_altitude_mode = kAltitudeFilterNearRx;
    out.filter_altitude_a_ft = 300;
    out.filter_altitude_b_ft = 400;
    ASSERT_TRUE(SettingsMigrator::Migrate(blob, sizeof(blob), 18, out));
    EXPECT_EQ(out.settings_version, kSettingsVersion);
    EXPECT_EQ(out.asterix_sac, 0);
    EXPECT_EQ(out.asterix_sic, 0);
    EXPECT_STREQ(out.ntp_server, "pool.ntp.org");
    EXPECT_EQ(out.filter_altitude_mode, kAltitudeFilterOff);
    EXPECT_EQ(out.filter_altitude_a_ft, 0);
    EXPECT_EQ(out.filter_altitude_b_ft, 10000);
    EXPECT_FALSE(out.swarm_enabled);
    EXPECT_STREQ(out.swarm_server, "https://adsbee.aero");
    EXPECT_STREQ(out.swarm_token, "");
    EXPECT_TRUE(out.auto_update_enabled);
    EXPECT_EQ(out.ip_mode, SettingsManager::kIPModeIPv6);
    EXPECT_STREQ(out.wifi_sta_extra_networks[0].ssid, "Tower");
    EXPECT_EQ(memcmp(&out.core_network_settings, &v18.core_network_settings, sizeof(v18.core_network_settings)), 0);
    EXPECT_STREQ(out.feed_uris[0], "myfeed.example.com");
    EXPECT_EQ(out.rx_position.icao_address, 0xABCDEFu);
    EXPECT_FALSE(SettingsMigrator::Migrate(blob, sizeof(settings_v18::Settings) - 1, 18, out));
}

TEST(FeedRAMBudget, CountsActiveFeedsAndTheSharedMQTTClientOnce) {
    SettingsManager::Settings s;
    for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
        s.feed_is_active[i] = false;
    }
    EXPECT_EQ(FeedRAMBudget::UsedBytes(s), 0u);
    s.feed_is_active[0] = true;
    EXPECT_EQ(FeedRAMBudget::UsedBytes(s), FeedRAMBudget::kTCPFeedBytes);
    s.feed_is_active[1] = true;
    s.feed_transports[1] = SettingsManager::kFeedTransportUDP;
    const uint32_t tcp_udp = FeedRAMBudget::kTCPFeedBytes + FeedRAMBudget::kUDPFeedBytes;
    EXPECT_EQ(FeedRAMBudget::UsedBytes(s), tcp_udp);
    for (uint16_t i = 2; i < 4; i++) {
        s.feed_is_active[i] = true;
        s.feed_transports[i] = SettingsManager::kFeedTransportMQTT;
    }
    EXPECT_EQ(FeedRAMBudget::UsedBytes(s), tcp_udp + FeedRAMBudget::kMQTTClientBytes);
    s.mqtt_broker.tls = true;
    s.mqtt_broker.transport = SettingsManager::kMQTTTransportWebSocket;
    EXPECT_EQ(FeedRAMBudget::UsedBytes(s), tcp_udp + FeedRAMBudget::kMQTTClientBytes + FeedRAMBudget::kMQTTTLSBytes +
                                              FeedRAMBudget::kMQTTWebSocketBytes);
    s.feeds_enabled = false;
    EXPECT_EQ(FeedRAMBudget::UsedBytes(s), 0u);
}

TEST(FeedRAMBudget, BudgetDependsOnWiFiAndPSRAM) {
    SettingsManager::Settings s;
    s.core_network_settings.wifi_ap_enabled = false;
    s.core_network_settings.wifi_sta_enabled = false;
    const uint32_t ethernet = FeedRAMBudget::BudgetBytes(s, false);
    s.core_network_settings.wifi_sta_enabled = true;
    const uint32_t wifi = FeedRAMBudget::BudgetBytes(s, false);
    EXPECT_LT(wifi, ethernet);
    EXPECT_GT(FeedRAMBudget::BudgetBytes(s, true), wifi);
}

TEST(FeedRAMBudget, DefaultFeedsFitOnWiFiWithoutPSRAM) {
    SettingsManager::Settings s;  // Four aggregator feeds over TCP.
    s.core_network_settings.wifi_ap_enabled = true;
    s.core_network_settings.wifi_sta_enabled = true;
    EXPECT_LE(FeedRAMBudget::UsedBytes(s), FeedRAMBudget::BudgetBytes(s, false));
    // Three of them and a Home Assistant MQTT feed fit too.
    s.feed_is_active[SettingsManager::Settings::kMaxNumFeeds - 1] = false;
    s.feed_is_active[0] = true;
    s.feed_transports[0] = SettingsManager::kFeedTransportMQTT;
    EXPECT_LE(FeedRAMBudget::UsedBytes(s), FeedRAMBudget::BudgetBytes(s, false));
}

TEST(Settings, AutoUpdateDefaultsOffAndDumpsAfterTheNetworkLines) {
    SettingsManager manager;
    EXPECT_FALSE(manager.settings.auto_update_enabled);
    manager.settings.auto_update_enabled = true;
    manager.settings = SettingsManager::Settings();  // What AT+SETTINGS=RESET starts from.
    EXPECT_FALSE(manager.settings.auto_update_enabled);

    // Older firmware stops replaying a dump at the first command it doesn't know, so those come last.
    manager.settings.auto_update_enabled = true;
    testing::internal::CaptureStdout();
    manager.PrintAT();
    std::string dump = testing::internal::GetCapturedStdout();
    size_t wifi = dump.find("AT+WIFI_STA=");
    size_t ip_mode = dump.find("AT+IP_MODE=");
    size_t auto_update = dump.find("AT+AUTO_UPDATE=1");
    size_t filter_distance = dump.find("AT+FILTER_DISTANCE_NM=");
    size_t filter_altitude = dump.find("AT+FILTER_ALTITUDE_FT=");
    ASSERT_NE(wifi, std::string::npos);
    ASSERT_NE(filter_altitude, std::string::npos);
    ASSERT_NE(ip_mode, std::string::npos);
    ASSERT_NE(auto_update, std::string::npos);
    ASSERT_NE(filter_distance, std::string::npos);
    EXPECT_LT(wifi, ip_mode);
    EXPECT_LT(ip_mode, auto_update);
    EXPECT_LT(auto_update, filter_distance);
    // Then the MQTT broker and the feed transports (see FeedTransportAndMQTTDumpReplaysOnOlderFirmware), then ASTERIX.
    EXPECT_EQ(dump.find("AT+", filter_distance + 1), dump.find("AT+MQTT="));
    size_t asterix = dump.find("AT+ASTERIX_SAC_SIC=0,0");
    ASSERT_NE(asterix, std::string::npos);
    EXPECT_LT(dump.rfind("AT+FEED="), asterix);
    size_t ntp = dump.find("AT+NTP_SERVER=pool.ntp.org");
    ASSERT_NE(ntp, std::string::npos);
    EXPECT_LT(asterix, ntp);
    // Then the altitude filter, swarm reports and GDL90 over Bluetooth last.
    EXPECT_LT(ntp, filter_altitude);
    size_t swarm = dump.find("AT+SWARM=0,https://adsbee.aero,-\r\n");
    ASSERT_NE(swarm, std::string::npos);
    EXPECT_EQ(dump.find("AT+", filter_altitude + 1), swarm);
    size_t ble_output = dump.find("AT+BLE_OUTPUT=OFF\r\n");
    ASSERT_NE(ble_output, std::string::npos);
    EXPECT_EQ(dump.find("AT+", swarm + 1), ble_output);
    EXPECT_EQ(dump.find("AT+", ble_output + 1), std::string::npos);

    uint8_t raw = 0x5A;
    memcpy(&manager.settings.auto_update_enabled, &raw, 1);
    EXPECT_TRUE(manager.Sanitize());
}

TEST(SettingsMigration, V19ToV20KeepsEverythingAndLeavesBLEOutputOff) {
    // A v19 blob as rc10 wrote it, 0xFF in the tail padding where v20 put ble_output_mode.
    static SettingsManager::Settings v19;
    v19 = SettingsManager::Settings();
    v19.settings_version = 19;
    strcpy(v19.core_network_settings.hostname, "hangar-bee");
    v19.core_network_settings.UpdateCRC32();
    v19.filter_distance_enabled = true;
    v19.filter_distance_deci_nm = 1234;
    v19.feed_transports[2] = SettingsManager::kFeedTransportMQTT;
    strcpy(v19.mqtt_broker.host, "broker.example.com");
    v19.asterix_sic = 7;
    v19.filter_altitude_mode = kAltitudeFilterBetween;
    v19.swarm_enabled = true;
    strcpy(v19.swarm_token, "tok-1234");
    uint8_t blob[sizeof(SettingsManager::Settings)];
    memset(blob, 0xFF, sizeof(blob));
    memcpy(blob, &v19, offsetof(settings_v19::Settings, swarm_token) + sizeof(settings_v19::Settings::swarm_token));

    static SettingsManager::Settings out;
    out.ble_output_mode = SettingsManager::kBLEOutputGDL90;
    ASSERT_TRUE(SettingsMigrator::Migrate(blob, sizeof(blob), 19, out));
    EXPECT_EQ(out.settings_version, kSettingsVersion);
    EXPECT_EQ(out.ble_output_mode, SettingsManager::kBLEOutputOff);
    EXPECT_TRUE(out.core_network_settings.IsValid());
    EXPECT_STREQ(out.core_network_settings.hostname, "hangar-bee");
    EXPECT_TRUE(out.filter_distance_enabled);
    EXPECT_EQ(out.filter_distance_deci_nm, 1234u);
    EXPECT_EQ(out.feed_transports[2], SettingsManager::kFeedTransportMQTT);
    EXPECT_STREQ(out.mqtt_broker.host, "broker.example.com");
    EXPECT_EQ(out.asterix_sic, 7);
    EXPECT_EQ(out.filter_altitude_mode, kAltitudeFilterBetween);
    EXPECT_TRUE(out.swarm_enabled);
    EXPECT_STREQ(out.swarm_token, "tok-1234");
    settings_manager.settings = out;
    EXPECT_FALSE(settings_manager.Sanitize());
}

TEST(SettingsMigration, V18ToCurrentLeavesBLEOutputOff) {
    settings_v18::Settings v18;
    memset(&v18, 0, sizeof(v18));
    v18.settings_version = 18;
    static SettingsManager::Settings out;
    out.ble_output_mode = SettingsManager::kBLEOutputGDL90;
    ASSERT_TRUE(SettingsMigrator::Migrate(reinterpret_cast<const uint8_t*>(&v18), sizeof(v18), 18, out));
    EXPECT_EQ(out.ble_output_mode, SettingsManager::kBLEOutputOff);
}

TEST(Settings, BLEOutputDefaultsOffAndRoundTripsItsModeNames) {
    SettingsManager::Settings s;
    EXPECT_EQ(s.ble_output_mode, SettingsManager::kBLEOutputOff);
    // Stored values are append-only: they must never move.
    EXPECT_EQ(SettingsManager::kBLEOutputOff, 0);
    EXPECT_EQ(SettingsManager::kBLEOutputGDL90, 1);
    for (uint8_t i = 0; i < SettingsManager::kNumBLEOutputModes; i++) {
        SettingsManager::BLEOutputMode mode;
        ASSERT_TRUE(SettingsManager::BLEOutputModeFromStr(SettingsManager::BLEOutputModeToStr(i), mode));
        EXPECT_EQ(mode, i);
    }
    SettingsManager::BLEOutputMode mode = SettingsManager::kBLEOutputGDL90;
    EXPECT_TRUE(SettingsManager::BLEOutputModeFromStr("gdl90", mode));
    EXPECT_FALSE(SettingsManager::BLEOutputModeFromStr("NMEA", mode));
    EXPECT_EQ(mode, SettingsManager::kBLEOutputGDL90);
}

TEST(SettingsManager, SanitizeTurnsAnUnknownBLEOutputModeOff) {
    SettingsManager::Settings& s = settings_manager.settings;
    s = SettingsManager::Settings{};
    s.core_network_settings.UpdateCRC32();
    s.ble_output_mode = static_cast<SettingsManager::BLEOutputMode>(SettingsManager::kNumBLEOutputModes);
    EXPECT_TRUE(settings_manager.Sanitize());
    EXPECT_EQ(s.ble_output_mode, SettingsManager::kBLEOutputOff);
}

TEST(SettingsMigration, V15ToCurrentPreservesAllFieldsAndDefaultsIPMode) {
    settings_v15::Settings v15 = settings_v15::Settings{};
    v15.settings_version = 15;

    SettingsManager::Settings::CoreNetworkSettings cns;
    strncpy(cns.hostname, "v15-host", sizeof(cns.hostname));
    strncpy(cns.wifi_sta_ssid, "v15-sta", sizeof(cns.wifi_sta_ssid));
    cns.wifi_sta_enabled = true;
    cns.ethernet_enabled = true;
    cns.UpdateCRC32();
    memcpy(&v15.core_network_settings, &cns, sizeof(cns));

    v15.r1090_rx_enabled = false;
    v15.tl_offset_mv = 321;
    v15.watchdog_timeout_sec = 17;
    v15.led_enabled = false;
    v15.feeds_enabled = false;
    v15.gnss_enabled = true;
    v15.gnss_receiver_type = SettingsManager::kGNSSReceiverGeneric;
    v15.gnss_notify = true;
    v15.log_level = SettingsManager::LogLevel::kErrors;
    v15.reporting_protocols[1] = SettingsManager::ReportingProtocol::kMAVLINK2;
    v15.baud_rates[1] = 230400;
    v15.subg_enabled = SettingsManager::EnableState::kEnableStateExternal;
    v15.remote_id_tx_enabled = true;
    strncpy(v15.remote_id_tx_operator_id, "OP15", sizeof(v15.remote_id_tx_operator_id));
    strncpy(v15.feed_uris[2], "2001:db8::5", sizeof(v15.feed_uris[2]));
    v15.feed_ports[2] = 30005;
    v15.feed_is_active[2] = true;
    v15.feed_protocols[2] = SettingsManager::ReportingProtocol::kBeastNoUAT;
    v15.feed_receiver_ids[2][3] = 0x5A;
    v15.mavlink_system_id = 9;
    v15.rx_position.source = 2;  // kPositionSourceGNSS
    v15.rx_position.icao_address = 0x123456;

    uint8_t blob[sizeof(settings_v15::Settings)];
    memcpy(blob, &v15, sizeof(v15));
    blob[1141] = 0xEE;  // v15 tail padding, now ip_mode: must not leak into the migrated value.

    SettingsManager::Settings out;
    ASSERT_TRUE(SettingsMigrator::Migrate(blob, sizeof(blob), 15, out));

    EXPECT_EQ(out.settings_version, kSettingsVersion);
    EXPECT_EQ(out.ip_mode, SettingsManager::kIPModeDual);

    SettingsManager::Settings::CoreNetworkSettings cns_out = out.core_network_settings;
    EXPECT_TRUE(cns_out.IsValid());
    EXPECT_STREQ(out.core_network_settings.hostname, "v15-host");
    EXPECT_STREQ(out.core_network_settings.wifi_sta_ssid, "v15-sta");
    EXPECT_TRUE(out.core_network_settings.wifi_sta_enabled);
    EXPECT_TRUE(out.core_network_settings.ethernet_enabled);

    EXPECT_FALSE(out.r1090_rx_enabled);
    EXPECT_EQ(out.tl_offset_mv, 321);
    EXPECT_EQ(out.watchdog_timeout_sec, 17u);
    EXPECT_FALSE(out.led_enabled);
    EXPECT_FALSE(out.feeds_enabled);
    EXPECT_TRUE(out.gnss_enabled);
    EXPECT_EQ(out.gnss_receiver_type, SettingsManager::kGNSSReceiverGeneric);
    EXPECT_TRUE(out.gnss_notify);
    EXPECT_EQ(out.log_level, SettingsManager::LogLevel::kErrors);
    EXPECT_EQ(out.reporting_protocols[1], SettingsManager::ReportingProtocol::kMAVLINK2);
    EXPECT_EQ(out.baud_rates[1], 230400u);
    EXPECT_EQ(out.subg_enabled, SettingsManager::EnableState::kEnableStateExternal);
    EXPECT_TRUE(out.remote_id_tx_enabled);
    EXPECT_STREQ(out.remote_id_tx_operator_id, "OP15");
    EXPECT_STREQ(out.feed_uris[2], "2001:db8::5");
    EXPECT_EQ(out.feed_ports[2], 30005);
    EXPECT_TRUE(out.feed_is_active[2]);
    EXPECT_EQ(out.feed_protocols[2], SettingsManager::ReportingProtocol::kBeastNoUAT);
    EXPECT_EQ(out.feed_receiver_ids[2][3], 0x5A);
    EXPECT_EQ(out.mavlink_system_id, 9);
    EXPECT_EQ(out.rx_position.source, SettingsManager::RxPosition::kPositionSourceGNSS);
    EXPECT_EQ(out.rx_position.icao_address, 0x123456u);
}

TEST(Settings, IPModeStrings) {
    SettingsManager::IPMode mode = SettingsManager::kIPModeDual;
    for (uint8_t i = 0; i < SettingsManager::kNumIPModes; i++) {
        const SettingsManager::IPMode m = static_cast<SettingsManager::IPMode>(i);
        ASSERT_TRUE(SettingsManager::IPModeFromStr(SettingsManager::IPModeToStr(m), mode));
        EXPECT_EQ(mode, m);
    }
    EXPECT_TRUE(SettingsManager::IPModeFromStr("ipv6", mode));
    EXPECT_EQ(mode, SettingsManager::kIPModeIPv6);
    EXPECT_FALSE(SettingsManager::IPModeFromStr("IPV5", mode));
    EXPECT_FALSE(SettingsManager::IPModeFromStr("", mode));
    EXPECT_EQ(mode, SettingsManager::kIPModeIPv6);  // Unchanged on failure.
    EXPECT_STREQ(SettingsManager::IPModeToStr(static_cast<SettingsManager::IPMode>(200)), "DUAL");
}

// Builds a v14 blob by raw byte offset using the layout shipped in adsbee_1090-0.9.1-rc2 (1140 B), with a distinct
// value per field, so the frozen settings_v14 snapshot can't drift from what devices hold.
TEST(SettingsMigration, V14ShippedRc2BlobMigrates) {
    uint8_t blob[1140];
    memset(blob, 0, sizeof(blob));
    auto put16 = [&](size_t off, uint16_t v) { memcpy(&blob[off], &v, sizeof(v)); };
    auto put32 = [&](size_t off, uint32_t v) { memcpy(&blob[off], &v, sizeof(v)); };
    auto putf = [&](size_t off, float v) { memcpy(&blob[off], &v, sizeof(v)); };
    auto puts = [&](size_t off, const char* str) { memcpy(&blob[off], str, strlen(str) + 1); };

    put32(0, 14);  // settings_version

    // CoreNetworkSettings at 4..243 (240 B, same layout since v12), with a valid CRC.
    SettingsManager::Settings::CoreNetworkSettings cns;
    cns.esp32_enabled = false;
    strncpy(cns.hostname, "rc2-host", sizeof(cns.hostname));
    cns.wifi_ap_enabled = false;
    cns.wifi_ap_channel = 11;
    strncpy(cns.wifi_sta_ssid, "rc2-sta", sizeof(cns.wifi_sta_ssid));
    strncpy(cns.wifi_sta_password, "rc2-pass", sizeof(cns.wifi_sta_password));
    cns.wifi_sta_enabled = true;
    cns.ethernet_enabled = true;
    cns.UpdateCRC32();
    ASSERT_EQ(sizeof(cns), 240u);
    memcpy(&blob[4], &cns, sizeof(cns));

    blob[244] = 0;        // r1090_rx_enabled (default true)
    put32(248, 0xFFFFFF85);  // tl_offset_mv = -123
    blob[252] = 1;        // r1090_bias_tee_enabled
    put32(256, 77);       // watchdog_timeout_sec
    blob[260] = 0;        // led_enabled (default true)
    blob[261] = 1;        // gnss_enabled
    blob[262] = 2;        // gnss_receiver_type = kGNSSReceiverUBXMIA
    blob[263] = 1;        // gnss_notify
    put16(264, 3);        // log_level = kInfo
    put16(266, 5);        // reporting_protocols[0] = kCSBee
    put16(268, 9);        // reporting_protocols[1] = kAircraftJSON
    put16(270, 7);        // reporting_protocols[2] = kMAVLINK2
    put32(272, 1234);     // baud_rates[0]
    put32(276, 57600);    // baud_rates[1]
    put32(280, 38400);    // baud_rates[2]
    blob[284] = 0xFF;     // subg_enabled = kEnableStateExternal (-1)
    blob[285] = 0;        // subg_rx_enabled (default true)
    blob[286] = 1;        // subg_bias_tee_enabled
    blob[287] = 0;        // subg_mode = kSubGHzRadioModeUATRx (the only mode)
    blob[288] = 1;        // remote_id_rx_enabled
    blob[289] = 0x02;     // remote_id_transports = BLE5 Long
    blob[290] = 1;        // remote_id_tx_enabled
    blob[291] = 0x04;     // remote_id_tx_transports = WiFi beacon
    blob[292] = 3;        // remote_id_tx_uas_id_type
    blob[293] = 15;       // remote_id_tx_ua_type
    puts(294, "UASID-RC2");  // remote_id_tx_uas_id[21] at 294..314
    puts(315, "OPID-RC2");   // remote_id_tx_operator_id[21] at 315..335
    for (int i = 0; i < 10; i++) {
        char uri[64];
        snprintf(uri, sizeof(uri), "feed%d.rc2.example", i);
        puts(336 + 64 * i, uri);          // feed_uris[10][64] at 336..975
        put16(976 + 2 * i, 40000 + i);    // feed_ports[10] at 976..995
        blob[996 + i] = i % 2;            // feed_is_active[10] at 996..1005
        put16(1006 + 2 * i, i % 11);      // feed_protocols[10] at 1006..1025
        for (int j = 0; j < 8; j++) {
            blob[1026 + 8 * i + j] = (uint8_t)(0xA0 + i * 8 + j);  // feed_receiver_ids[10][8] at 1026..1105
        }
    }
    blob[1106] = 9;    // mavlink_system_id
    blob[1107] = 190;  // mavlink_component_id
    // RxPosition (packed, 29 B) at 1108..1136.
    blob[1108] = 1;           // source = kPositionSourceFixed
    putf(1109, 37.5f);        // latitude_deg
    putf(1113, -122.25f);     // longitude_deg
    put32(1117, 1111);        // gnss_altitude_ft
    put32(1121, 2222);        // baro_altitude_ft
    putf(1125, 270.5f);       // heading_deg
    put32(1129, 44);          // speed_kts
    put32(1133, 0xABC123);    // icao_address
    // 1137..1139 are trailing padding.

    SettingsManager::Settings out;
    ASSERT_TRUE(SettingsMigrator::Migrate(blob, sizeof(blob), 14, out));

    EXPECT_EQ(out.settings_version, kSettingsVersion);
    EXPECT_TRUE(out.core_network_settings.IsValid());
    EXPECT_FALSE(out.core_network_settings.esp32_enabled);
    EXPECT_STREQ(out.core_network_settings.hostname, "rc2-host");
    EXPECT_FALSE(out.core_network_settings.wifi_ap_enabled);
    EXPECT_EQ(out.core_network_settings.wifi_ap_channel, 11);
    EXPECT_TRUE(out.core_network_settings.wifi_sta_enabled);
    EXPECT_STREQ(out.core_network_settings.wifi_sta_ssid, "rc2-sta");
    EXPECT_STREQ(out.core_network_settings.wifi_sta_password, "rc2-pass");
    EXPECT_TRUE(out.core_network_settings.ethernet_enabled);

    EXPECT_FALSE(out.r1090_rx_enabled);
    EXPECT_EQ(out.tl_offset_mv, -123);
    EXPECT_TRUE(out.r1090_bias_tee_enabled);
    EXPECT_EQ(out.watchdog_timeout_sec, 77u);
    EXPECT_FALSE(out.led_enabled);
    EXPECT_TRUE(out.feeds_enabled);  // New in v15: defaults to enabled.
    EXPECT_TRUE(out.gnss_enabled);
    EXPECT_EQ(out.gnss_receiver_type, SettingsManager::kGNSSReceiverUBXMIA);
    EXPECT_TRUE(out.gnss_notify);
    EXPECT_EQ(out.log_level, SettingsManager::LogLevel::kInfo);
    EXPECT_EQ(out.reporting_protocols[0], SettingsManager::ReportingProtocol::kCSBee);
    EXPECT_EQ(out.reporting_protocols[1], SettingsManager::ReportingProtocol::kAircraftJSON);
    EXPECT_EQ(out.reporting_protocols[2], SettingsManager::ReportingProtocol::kMAVLINK2);
    EXPECT_EQ(out.baud_rates[0], 1234u);
    EXPECT_EQ(out.baud_rates[1], 57600u);
    EXPECT_EQ(out.baud_rates[2], 38400u);
    EXPECT_EQ(out.subg_enabled, SettingsManager::EnableState::kEnableStateExternal);
    EXPECT_FALSE(out.subg_rx_enabled);
    EXPECT_TRUE(out.subg_bias_tee_enabled);
    EXPECT_EQ(out.subg_mode, SettingsManager::SubGHzRadioMode::kSubGHzRadioModeUATRx);
    EXPECT_TRUE(out.remote_id_rx_enabled);
    EXPECT_EQ(out.remote_id_transports, 0x02);
    EXPECT_TRUE(out.remote_id_tx_enabled);
    EXPECT_EQ(out.remote_id_tx_transports, 0x04);
    EXPECT_EQ(out.remote_id_tx_uas_id_type, 3);
    EXPECT_EQ(out.remote_id_tx_ua_type, 15);
    EXPECT_STREQ(out.remote_id_tx_uas_id, "UASID-RC2");
    EXPECT_STREQ(out.remote_id_tx_operator_id, "OPID-RC2");
    for (int i = 0; i < 10; i++) {
        char uri[64];
        snprintf(uri, sizeof(uri), "feed%d.rc2.example", i);
        EXPECT_STREQ(out.feed_uris[i], uri) << "feed " << i;
        EXPECT_EQ(out.feed_ports[i], 40000 + i) << "feed " << i;
        EXPECT_EQ(out.feed_is_active[i], i % 2 == 1) << "feed " << i;
        EXPECT_EQ(out.feed_protocols[i], i % 11) << "feed " << i;
        for (int j = 0; j < 8; j++) {
            EXPECT_EQ(out.feed_receiver_ids[i][j], (uint8_t)(0xA0 + i * 8 + j)) << "feed " << i << " byte " << j;
        }
    }
    EXPECT_EQ(out.mavlink_system_id, 9);
    EXPECT_EQ(out.mavlink_component_id, 190);
    EXPECT_EQ(out.rx_position.source, SettingsManager::RxPosition::kPositionSourceFixed);
    EXPECT_FLOAT_EQ(out.rx_position.latitude_deg, 37.5f);
    EXPECT_FLOAT_EQ(out.rx_position.longitude_deg, -122.25f);
    EXPECT_EQ(out.rx_position.gnss_altitude_ft, 1111);
    EXPECT_EQ(out.rx_position.baro_altitude_ft, 2222);
    EXPECT_FLOAT_EQ(out.rx_position.heading_deg, 270.5f);
    EXPECT_EQ(out.rx_position.speed_kts, 44);
    EXPECT_EQ(out.rx_position.icao_address, 0xABC123u);

    // A valid rc2 blob needs no sanitizing after migration.
    settings_manager.settings = out;
    EXPECT_FALSE(settings_manager.Sanitize());
}

TEST(SettingsMigration, UnmigratableVersionReturnsFalse) {
    uint8_t blob[sizeof(settings_v12::Settings)] = {0};
    SettingsManager::Settings out;
    // Pre-v12, never shipped (13, 16), unknown-future, and current version are all not migratable by this utility.
    EXPECT_FALSE(SettingsMigrator::Migrate(blob, sizeof(blob), 11, out));
    EXPECT_FALSE(SettingsMigrator::Migrate(blob, sizeof(blob), 13, out));
    EXPECT_FALSE(SettingsMigrator::Migrate(blob, sizeof(blob), 16, out));
    EXPECT_FALSE(SettingsMigrator::Migrate(blob, sizeof(blob), 999, out));
    EXPECT_FALSE(SettingsMigrator::Migrate(blob, sizeof(blob), kSettingsVersion, out));
    // A v12 version tag but a too-short blob must also be rejected.
    EXPECT_FALSE(SettingsMigrator::Migrate(blob, sizeof(settings_v12::Settings) - 1, 12, out));
    // Likewise for a v14 version tag with a too-short blob.
    uint8_t v14_blob[sizeof(settings_v14::Settings)] = {0};
    EXPECT_FALSE(SettingsMigrator::Migrate(v14_blob, sizeof(settings_v14::Settings) - 1, 14, out));
}

TEST(SettingsMigration, V12ToCurrentPreservesAllFields) {
    // Build a v12 settings blob with distinctive, non-default values across every field.
    settings_v12::Settings v12 = {};
    v12.settings_version = 12;

    // Core network settings: build a valid-CRC block using the live struct (which also proves byte compatibility),
    // then copy it into the frozen v12 layout.
    {
        SettingsManager::Settings::CoreNetworkSettings cns;  // Zero-fills strings in its constructor.
        cns.esp32_enabled = true;
        strncpy(cns.wifi_ap_ssid, "MigrateNet", sizeof(cns.wifi_ap_ssid));
        strncpy(cns.wifi_ap_password, "supersecret", sizeof(cns.wifi_ap_password));
        cns.wifi_ap_channel = 6;
        cns.ethernet_enabled = true;
        cns.UpdateCRC32();
        ASSERT_EQ(sizeof(cns), sizeof(v12.core_network_settings));
        memcpy(&v12.core_network_settings, &cns, sizeof(cns));
    }

    v12.r1090_rx_enabled = false;
    v12.tl_offset_mv = 123;
    v12.r1090_bias_tee_enabled = true;
    v12.watchdog_timeout_sec = 42;
    v12.log_level = SettingsManager::LogLevel::kInfo;  // 3
    v12.reporting_protocols[0] = SettingsManager::ReportingProtocol::kCSBee;
    v12.reporting_protocols[1] = SettingsManager::ReportingProtocol::kGDL90;
    v12.reporting_protocols[2] = SettingsManager::ReportingProtocol::kAircraftJSON;
    v12.baud_rates[0] = 1200;
    v12.baud_rates[1] = 57600;
    v12.baud_rates[2] = 4800;
    v12.subg_enabled = SettingsManager::EnableState::kEnableStateDisabled;
    v12.subg_rx_enabled = false;
    v12.subg_bias_tee_enabled = true;
    v12.subg_mode = SettingsManager::SubGHzRadioMode::kSubGHzRadioModeUATRx;

    strncpy(v12.feed_uris[0], "myfeed.example.com", sizeof(v12.feed_uris[0]));
    strncpy(v12.feed_uris[9], "feed.adsb.fi", sizeof(v12.feed_uris[9]));
    v12.feed_ports[0] = 30005;
    v12.feed_ports[9] = 30004;
    v12.feed_is_active[0] = true;
    v12.feed_is_active[9] = true;
    v12.feed_protocols[0] = SettingsManager::ReportingProtocol::kBeast;
    v12.feed_protocols[9] = SettingsManager::ReportingProtocol::kBeast;
    v12.feed_receiver_ids[0][0] = 0xDE;
    v12.feed_receiver_ids[0][7] = 0xAD;

    v12.mavlink_system_id = 7;
    v12.mavlink_component_id = 42;

    v12.rx_position.source = 1;  // kPositionSourceFixed
    v12.rx_position.latitude_deg = 37.5f;
    v12.rx_position.longitude_deg = -122.3f;
    v12.rx_position.gnss_altitude_ft = 111;
    v12.rx_position.baro_altitude_ft = 222;
    v12.rx_position.heading_deg = 90.0f;
    v12.rx_position.speed_kts = 33;
    v12.rx_position.icao_address = 0xABCDEF;

    // Serialize to a raw blob and migrate.
    uint8_t blob[sizeof(settings_v12::Settings)];
    memcpy(blob, &v12, sizeof(v12));

    SettingsManager::Settings out;
    ASSERT_TRUE(SettingsMigrator::Migrate(blob, sizeof(blob), 12, out));

    // Version updated to current.
    EXPECT_EQ(out.settings_version, kSettingsVersion);

    // Network settings preserved and still CRC-valid.
    EXPECT_TRUE(out.core_network_settings.IsValid());
    EXPECT_STREQ(out.core_network_settings.wifi_ap_ssid, "MigrateNet");
    EXPECT_STREQ(out.core_network_settings.wifi_ap_password, "supersecret");
    EXPECT_EQ(out.core_network_settings.wifi_ap_channel, 6);
    EXPECT_TRUE(out.core_network_settings.ethernet_enabled);

    // System / 1090 settings preserved.
    EXPECT_FALSE(out.r1090_rx_enabled);
    EXPECT_EQ(out.tl_offset_mv, 123);
    EXPECT_TRUE(out.r1090_bias_tee_enabled);
    EXPECT_EQ(out.watchdog_timeout_sec, 42u);

    // Comms settings preserved.
    EXPECT_EQ(out.log_level, SettingsManager::LogLevel::kInfo);
    EXPECT_EQ(out.reporting_protocols[0], SettingsManager::ReportingProtocol::kCSBee);
    EXPECT_EQ(out.reporting_protocols[1], SettingsManager::ReportingProtocol::kGDL90);
    EXPECT_EQ(out.reporting_protocols[2], SettingsManager::ReportingProtocol::kAircraftJSON);
    EXPECT_EQ(out.baud_rates[0], 1200u);
    EXPECT_EQ(out.baud_rates[1], 57600u);
    EXPECT_EQ(out.baud_rates[2], 4800u);

    // Sub-GHz settings preserved.
    EXPECT_EQ(out.subg_enabled, SettingsManager::EnableState::kEnableStateDisabled);
    EXPECT_FALSE(out.subg_rx_enabled);
    EXPECT_TRUE(out.subg_bias_tee_enabled);

    // Fields added after v12 take their defaults: Remote ID receive (from the unshipped v13)...
    EXPECT_FALSE(out.remote_id_rx_enabled);
    EXPECT_EQ(out.remote_id_transports, (uint8_t)(SettingsManager::kRemoteIDTransportBLE4 |
                                                  SettingsManager::kRemoteIDTransportBLE5Long |
                                                  SettingsManager::kRemoteIDTransportWiFiBeacon));
    // ...and Remote ID transmit (new in v14). Both arrive in the single v12 -> v14 step.
    ExpectV14TxDefaults(out);

    // Feed settings preserved.
    EXPECT_STREQ(out.feed_uris[0], "myfeed.example.com");
    EXPECT_STREQ(out.feed_uris[9], "feed.adsb.fi");
    EXPECT_EQ(out.feed_ports[0], 30005);
    EXPECT_EQ(out.feed_ports[9], 30004);
    EXPECT_TRUE(out.feed_is_active[0]);
    EXPECT_TRUE(out.feed_is_active[9]);
    EXPECT_EQ(out.feed_protocols[0], SettingsManager::ReportingProtocol::kBeast);
    EXPECT_EQ(out.feed_receiver_ids[0][0], 0xDE);
    EXPECT_EQ(out.feed_receiver_ids[0][7], 0xAD);

    // MAVLINK settings preserved.
    EXPECT_EQ(out.mavlink_system_id, 7);
    EXPECT_EQ(out.mavlink_component_id, 42);

    // Receiver position preserved.
    EXPECT_EQ(out.rx_position.source, SettingsManager::RxPosition::kPositionSourceFixed);
    EXPECT_FLOAT_EQ(out.rx_position.latitude_deg, 37.5f);
    EXPECT_FLOAT_EQ(out.rx_position.longitude_deg, -122.3f);
    EXPECT_EQ(out.rx_position.gnss_altitude_ft, 111);
    EXPECT_EQ(out.rx_position.baro_altitude_ft, 222);
    EXPECT_FLOAT_EQ(out.rx_position.heading_deg, 90.0f);
    EXPECT_EQ(out.rx_position.speed_kts, 33);
    EXPECT_EQ(out.rx_position.icao_address, 0xABCDEFu);
}

// Regression test for a real hard fault seen on hardware: a v14->v15 migration on a device whose on-flash blob had
// already drifted out from under the (un-bumped) v14 tag produced an out-of-range reporting_protocols/log_level
// value, which SettingsManager::Print() then used to index kReportingProtocolStrs[]/kConsoleLogLevelStrs[] --
// faulting the RP2040 and crash-looping the ESP32 (which receives `settings` over SPI and calls Print() at boot) on
// every subsequent boot, even after a full reflash, because Sanitize() didn't exist to catch it.
TEST(SettingsManager, SanitizeClampsOutOfRangeEnumFields) {
    SettingsManager::Settings& s = settings_manager.settings;
    s = SettingsManager::Settings{};

    s.log_level = static_cast<SettingsManager::LogLevel>(9999);
    s.reporting_protocols[0] = static_cast<SettingsManager::ReportingProtocol>(0xF1F6);
    s.feed_protocols[3] = static_cast<SettingsManager::ReportingProtocol>(0xABCD);
    s.subg_mode = static_cast<SettingsManager::SubGHzRadioMode>(200);
    s.subg_enabled = static_cast<SettingsManager::EnableState>(42);
    s.gnss_receiver_type = static_cast<SettingsManager::GNSSReceiverType>(200);
    s.rx_position.source = static_cast<SettingsManager::RxPosition::PositionSource>(200);
    s.ip_mode = static_cast<SettingsManager::IPMode>(200);

    EXPECT_TRUE(settings_manager.Sanitize());
    EXPECT_EQ(s.ip_mode, SettingsManager::kIPModeDual);

    EXPECT_LT(s.log_level, SettingsManager::LogLevel::kNumLogLevels);
    EXPECT_LT(s.reporting_protocols[0], SettingsManager::ReportingProtocol::kNumProtocols);
    EXPECT_LT(s.feed_protocols[3], SettingsManager::ReportingProtocol::kNumProtocols);
    EXPECT_LT(s.subg_mode, SettingsManager::kNumSubGHzRadioModes);
    EXPECT_EQ(s.subg_enabled, SettingsManager::EnableState::kEnableStateEnabled);
    EXPECT_EQ(s.gnss_receiver_type, SettingsManager::kGNSSReceiverNone);
    EXPECT_LT(s.rx_position.source, SettingsManager::RxPosition::kNumPositionSources);

    // A second pass over already-legal values must report no change and leave them untouched.
    SettingsManager::Settings clean = SettingsManager::Settings{};
    settings_manager.settings = clean;
    EXPECT_FALSE(settings_manager.Sanitize());
}

// Writes a raw byte into a bool's storage, the way a corrupt settings blob would (assigning an int to a bool can't).
static void PokeBoolByte(bool& b, uint8_t raw) { memcpy(&b, &raw, sizeof(raw)); }
static uint8_t PeekBoolByte(const bool& b) {
    uint8_t raw;
    memcpy(&raw, &b, sizeof(raw));
    return raw;
}

TEST(SettingsManager, SanitizeNormalizesBools) {
    SettingsManager::Settings& s = settings_manager.settings;
    s = SettingsManager::Settings{};
    s.core_network_settings.UpdateCRC32();

    PokeBoolByte(s.gnss_notify, 0x02);
    PokeBoolByte(s.feed_is_active[3], 0x75);
    PokeBoolByte(s.remote_id_tx_enabled, 0xFF);
    PokeBoolByte(s.core_network_settings.wifi_sta_enabled, 0x34);
    // Keep the CRC valid over the corrupt byte, as it would be if the corruption predates the last save.
    s.core_network_settings.UpdateCRC32();

    EXPECT_TRUE(settings_manager.Sanitize());

    EXPECT_EQ(PeekBoolByte(s.gnss_notify), 1);
    EXPECT_EQ(PeekBoolByte(s.feed_is_active[3]), 1);
    EXPECT_EQ(PeekBoolByte(s.remote_id_tx_enabled), 1);
    EXPECT_EQ(PeekBoolByte(s.core_network_settings.wifi_sta_enabled), 1);
    EXPECT_EQ(PeekBoolByte(s.feed_is_active[0]), 0);  // Untouched legal values stay as they were.
    EXPECT_TRUE(s.core_network_settings.IsValid());   // CRC recomputed after the CNS fix-up.
}

TEST(SettingsManager, SanitizeTerminatesStrings) {
    SettingsManager::Settings& s = settings_manager.settings;
    s = SettingsManager::Settings{};
    memset(s.core_network_settings.hostname, 'H', sizeof(s.core_network_settings.hostname));
    memset(s.core_network_settings.wifi_sta_password, 'P', sizeof(s.core_network_settings.wifi_sta_password));
    s.core_network_settings.UpdateCRC32();
    memset(s.feed_uris[2], 'A', sizeof(s.feed_uris[2]));
    memset(s.remote_id_tx_uas_id, 'U', sizeof(s.remote_id_tx_uas_id));
    memset(s.remote_id_tx_operator_id, 'O', sizeof(s.remote_id_tx_operator_id));
    memset(s.wifi_sta_extra_networks[1].password, 'W', sizeof(s.wifi_sta_extra_networks[1].password));

    EXPECT_TRUE(settings_manager.Sanitize());
    EXPECT_EQ(strnlen(s.wifi_sta_extra_networks[1].password, sizeof(s.wifi_sta_extra_networks[1].password)),
              sizeof(s.wifi_sta_extra_networks[1].password) - 1);

    EXPECT_EQ(strnlen(s.core_network_settings.hostname, sizeof(s.core_network_settings.hostname)),
              sizeof(s.core_network_settings.hostname) - 1);
    EXPECT_EQ(strnlen(s.core_network_settings.wifi_sta_password, sizeof(s.core_network_settings.wifi_sta_password)),
              sizeof(s.core_network_settings.wifi_sta_password) - 1);
    EXPECT_EQ(strnlen(s.feed_uris[2], sizeof(s.feed_uris[2])), sizeof(s.feed_uris[2]) - 1);
    EXPECT_EQ(strnlen(s.remote_id_tx_uas_id, sizeof(s.remote_id_tx_uas_id)), sizeof(s.remote_id_tx_uas_id) - 1);
    EXPECT_EQ(strnlen(s.remote_id_tx_operator_id, sizeof(s.remote_id_tx_operator_id)),
              sizeof(s.remote_id_tx_operator_id) - 1);
    EXPECT_STREQ(s.feed_uris[9], "feed.adsb.fi");  // Terminated strings are left alone.
    EXPECT_TRUE(s.core_network_settings.IsValid());
}

TEST(SettingsManager, SanitizeDoesNotValidateAnInvalidCoreNetworkCRC) {
    SettingsManager::Settings& s = settings_manager.settings;
    s = SettingsManager::Settings{};
    s.core_network_settings.UpdateCRC32();
    s.core_network_settings.crc32 ^= 0xFFFFFFFF;  // Stored CRC doesn't match.
    memset(s.core_network_settings.hostname, 'H', sizeof(s.core_network_settings.hostname));

    EXPECT_TRUE(settings_manager.Sanitize());
    // The fix-up must not turn an untrustworthy block into one that Load() would restore.
    EXPECT_FALSE(s.core_network_settings.IsValid());
}

TEST(SettingsManager, SanitizeResetsZeroUARTBaudRates) {
    SettingsManager::Settings& s = settings_manager.settings;
    s = SettingsManager::Settings{};
    s.baud_rates[SettingsManager::kCommsUART] = 0;
    s.baud_rates[SettingsManager::kGNSSUART] = 0;

    EXPECT_TRUE(settings_manager.Sanitize());

    EXPECT_EQ(s.baud_rates[SettingsManager::kCommsUART], SettingsManager::Settings::kDefaultCommsUARTBaudrate);
    EXPECT_EQ(s.baud_rates[SettingsManager::kGNSSUART], SettingsManager::Settings::kDefaultGNSSUARTBaudrate);
    EXPECT_EQ(s.baud_rates[SettingsManager::kConsole], 0u);  // USB CDC console: 0 is the normal value.

    // Non-default, non-zero rates are kept.
    s.baud_rates[SettingsManager::kCommsUART] = 57600;
    s.baud_rates[SettingsManager::kGNSSUART] = 38400;
    EXPECT_FALSE(settings_manager.Sanitize());
    EXPECT_EQ(s.baud_rates[SettingsManager::kCommsUART], 57600u);
    EXPECT_EQ(s.baud_rates[SettingsManager::kGNSSUART], 38400u);
}

TEST(SettingsManager, SanitizeDisablesGNSSWithoutReceiverType) {
    SettingsManager::Settings& s = settings_manager.settings;
    s = SettingsManager::Settings{};
    s.gnss_enabled = true;
    s.gnss_receiver_type = SettingsManager::kGNSSReceiverNone;
    EXPECT_TRUE(settings_manager.Sanitize());
    EXPECT_FALSE(s.gnss_enabled);

    // An out-of-range type is reset to NONE first, which then also disables GNSS.
    s.gnss_enabled = true;
    s.gnss_receiver_type = static_cast<SettingsManager::GNSSReceiverType>(77);
    EXPECT_TRUE(settings_manager.Sanitize());
    EXPECT_EQ(s.gnss_receiver_type, SettingsManager::kGNSSReceiverNone);
    EXPECT_FALSE(s.gnss_enabled);

    // A real configuration is left alone.
    s.gnss_enabled = true;
    s.gnss_receiver_type = SettingsManager::kGNSSReceiverUBXMIA;
    EXPECT_FALSE(settings_manager.Sanitize());
    EXPECT_TRUE(s.gnss_enabled);
}

TEST(Settings, ProtocolInUseCountsSerialOutputsAndActiveFeeds) {
    SettingsManager::Settings s;
    EXPECT_FALSE(SettingsManager::ProtocolInUse(s, SettingsManager::kASTERIX));

    s.reporting_protocols[SettingsManager::kCommsUART] = SettingsManager::kASTERIX;
    EXPECT_TRUE(SettingsManager::ProtocolInUse(s, SettingsManager::kASTERIX));
    s.reporting_protocols[SettingsManager::kCommsUART] = SettingsManager::kNoReports;

    s.feed_protocols[2] = SettingsManager::kASTERIX;
    s.feed_is_active[2] = false;
    EXPECT_FALSE(SettingsManager::ProtocolInUse(s, SettingsManager::kASTERIX));
    s.feed_is_active[2] = true;
    s.feeds_enabled = true;
    EXPECT_TRUE(SettingsManager::ProtocolInUse(s, SettingsManager::kASTERIX));
    s.feeds_enabled = false;
    EXPECT_FALSE(SettingsManager::ProtocolInUse(s, SettingsManager::kASTERIX));
}

TEST(Settings, NTPServerDefaultsToPoolAndIsSanitized) {
    SettingsManager manager;
    EXPECT_STREQ(manager.settings.ntp_server, "pool.ntp.org");
    memset(manager.settings.ntp_server, 'a', sizeof(manager.settings.ntp_server));
    EXPECT_TRUE(manager.Sanitize());
    EXPECT_EQ(strlen(manager.settings.ntp_server), SettingsManager::Settings::kNTPServerMaxLen);
}
