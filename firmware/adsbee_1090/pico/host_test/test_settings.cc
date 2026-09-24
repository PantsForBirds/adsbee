#include <cstring>

#include "gtest/gtest.h"
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

    EXPECT_EQ(sizeof(settings_v13::Settings), 1088u);
    EXPECT_EQ(sizeof(settings_v13::CoreNetworkSettings),
              sizeof(SettingsManager::Settings::CoreNetworkSettings));
    EXPECT_EQ(sizeof(settings_v13::RxPosition), sizeof(SettingsManager::RxPosition));

    EXPECT_EQ(sizeof(settings_v14::Settings), 1140u);
    EXPECT_EQ(sizeof(settings_v14::CoreNetworkSettings),
              sizeof(SettingsManager::Settings::CoreNetworkSettings));
    EXPECT_EQ(sizeof(settings_v14::RxPosition), sizeof(SettingsManager::RxPosition));
}

// Fills a v13 settings struct with distinctive, non-default values across every field, and gives it a valid-CRC
// CoreNetworkSettings block. Shared by the v13 and v12 migration tests.
static void PopulateV13(settings_v13::Settings& v13) {
    v13 = settings_v13::Settings{};
    v13.settings_version = 13;

    SettingsManager::Settings::CoreNetworkSettings cns;  // Zero-fills strings in its constructor.
    cns.esp32_enabled = true;
    strncpy(cns.wifi_ap_ssid, "MigrateNet", sizeof(cns.wifi_ap_ssid));
    strncpy(cns.wifi_ap_password, "supersecret", sizeof(cns.wifi_ap_password));
    cns.wifi_ap_channel = 6;
    cns.ethernet_enabled = true;
    cns.UpdateCRC32();
    ASSERT_EQ(sizeof(cns), sizeof(v13.core_network_settings));
    memcpy(&v13.core_network_settings, &cns, sizeof(cns));

    v13.r1090_rx_enabled = false;
    v13.tl_offset_mv = 123;
    v13.r1090_bias_tee_enabled = true;
    v13.watchdog_timeout_sec = 42;
    v13.log_level = SettingsManager::LogLevel::kInfo;
    v13.reporting_protocols[0] = SettingsManager::ReportingProtocol::kCSBee;
    v13.reporting_protocols[1] = SettingsManager::ReportingProtocol::kGDL90;
    v13.reporting_protocols[2] = SettingsManager::ReportingProtocol::kAircraftJSON;
    v13.baud_rates[0] = 1200;
    v13.baud_rates[1] = 57600;
    v13.baud_rates[2] = 4800;
    v13.subg_enabled = SettingsManager::EnableState::kEnableStateDisabled;
    v13.subg_rx_enabled = false;
    v13.subg_bias_tee_enabled = true;
    v13.subg_mode = SettingsManager::SubGHzRadioMode::kSubGHzRadioModeUATRx;
    v13.remote_id_rx_enabled = true;
    v13.remote_id_transports = SettingsManager::kRemoteIDTransportBLE5Long;

    strncpy(v13.feed_uris[0], "myfeed.example.com", sizeof(v13.feed_uris[0]));
    strncpy(v13.feed_uris[9], "feed.adsb.fi", sizeof(v13.feed_uris[9]));
    v13.feed_ports[0] = 30005;
    v13.feed_ports[9] = 30004;
    v13.feed_is_active[0] = true;
    v13.feed_is_active[9] = true;
    v13.feed_protocols[0] = SettingsManager::ReportingProtocol::kBeast;
    v13.feed_protocols[9] = SettingsManager::ReportingProtocol::kBeast;
    v13.feed_receiver_ids[0][0] = 0xDE;
    v13.feed_receiver_ids[0][7] = 0xAD;

    v13.mavlink_system_id = 7;
    v13.mavlink_component_id = 42;

    v13.rx_position.source = 1;  // kPositionSourceFixed
    v13.rx_position.latitude_deg = 37.5f;
    v13.rx_position.longitude_deg = -122.3f;
    v13.rx_position.gnss_altitude_ft = 111;
    v13.rx_position.baro_altitude_ft = 222;
    v13.rx_position.heading_deg = 90.0f;
    v13.rx_position.speed_kts = 33;
    v13.rx_position.icao_address = 0xABCDEF;
}

// Asserts that everything PopulateV13 set survived into the migrated live struct.
static void ExpectV13FieldsPreserved(const SettingsManager::Settings& out) {
    EXPECT_EQ(out.settings_version, kSettingsVersion);

    SettingsManager::Settings::CoreNetworkSettings cns_out = out.core_network_settings;
    EXPECT_TRUE(cns_out.IsValid());
    EXPECT_STREQ(out.core_network_settings.wifi_ap_ssid, "MigrateNet");
    EXPECT_STREQ(out.core_network_settings.wifi_ap_password, "supersecret");
    EXPECT_EQ(out.core_network_settings.wifi_ap_channel, 6);
    EXPECT_TRUE(out.core_network_settings.ethernet_enabled);

    EXPECT_FALSE(out.r1090_rx_enabled);
    EXPECT_EQ(out.tl_offset_mv, 123);
    EXPECT_TRUE(out.r1090_bias_tee_enabled);
    EXPECT_EQ(out.watchdog_timeout_sec, 42u);

    EXPECT_EQ(out.log_level, SettingsManager::LogLevel::kInfo);
    EXPECT_EQ(out.reporting_protocols[0], SettingsManager::ReportingProtocol::kCSBee);
    EXPECT_EQ(out.reporting_protocols[1], SettingsManager::ReportingProtocol::kGDL90);
    EXPECT_EQ(out.reporting_protocols[2], SettingsManager::ReportingProtocol::kAircraftJSON);
    EXPECT_EQ(out.baud_rates[0], 1200u);
    EXPECT_EQ(out.baud_rates[1], 57600u);
    EXPECT_EQ(out.baud_rates[2], 4800u);

    EXPECT_EQ(out.subg_enabled, SettingsManager::EnableState::kEnableStateDisabled);
    EXPECT_FALSE(out.subg_rx_enabled);
    EXPECT_TRUE(out.subg_bias_tee_enabled);

    EXPECT_STREQ(out.feed_uris[0], "myfeed.example.com");
    EXPECT_STREQ(out.feed_uris[9], "feed.adsb.fi");
    EXPECT_EQ(out.feed_ports[0], 30005);
    EXPECT_EQ(out.feed_ports[9], 30004);
    EXPECT_TRUE(out.feed_is_active[0]);
    EXPECT_TRUE(out.feed_is_active[9]);
    EXPECT_EQ(out.feed_protocols[0], SettingsManager::ReportingProtocol::kBeast);
    EXPECT_EQ(out.feed_receiver_ids[0][0], 0xDE);
    EXPECT_EQ(out.feed_receiver_ids[0][7], 0xAD);

    EXPECT_EQ(out.mavlink_system_id, 7);
    EXPECT_EQ(out.mavlink_component_id, 42);

    EXPECT_EQ(out.rx_position.source, SettingsManager::RxPosition::kPositionSourceFixed);
    EXPECT_FLOAT_EQ(out.rx_position.latitude_deg, 37.5f);
    EXPECT_FLOAT_EQ(out.rx_position.longitude_deg, -122.3f);
    EXPECT_EQ(out.rx_position.gnss_altitude_ft, 111);
    EXPECT_EQ(out.rx_position.baro_altitude_ft, 222);
    EXPECT_FLOAT_EQ(out.rx_position.heading_deg, 90.0f);
    EXPECT_EQ(out.rx_position.speed_kts, 33);
    EXPECT_EQ(out.rx_position.icao_address, 0xABCDEFu);
}

// The Remote ID transmit settings are new in v14, so any migration must leave them at their defaults.
static void ExpectV14TxDefaults(const SettingsManager::Settings& out) {
    EXPECT_FALSE(out.remote_id_tx_enabled);
    EXPECT_EQ(out.remote_id_tx_transports, (uint8_t)(SettingsManager::kRemoteIDTransportBLE4 |
                                                     SettingsManager::kRemoteIDTransportBLE5Long |
                                                     SettingsManager::kRemoteIDTransportWiFiBeacon));
    EXPECT_EQ(out.remote_id_tx_uas_id_type, 1);
    EXPECT_EQ(out.remote_id_tx_ua_type, 2);
    EXPECT_STREQ(out.remote_id_tx_uas_id, "");
    EXPECT_STREQ(out.remote_id_tx_operator_id, "");
}

TEST(SettingsMigration, V13ToV14PreservesAllFields) {
    settings_v13::Settings v13;
    PopulateV13(v13);

    uint8_t blob[sizeof(settings_v13::Settings)];
    memcpy(blob, &v13, sizeof(v13));

    SettingsManager::Settings out;
    ASSERT_TRUE(SettingsMigrator::Migrate(blob, sizeof(blob), 13, out));

    ExpectV13FieldsPreserved(out);
    // Remote ID receive settings carry over from v13 (unlike the transmit settings, which are new).
    EXPECT_TRUE(out.remote_id_rx_enabled);
    EXPECT_EQ(out.remote_id_transports, (uint8_t)SettingsManager::kRemoteIDTransportBLE5Long);
    ExpectV14TxDefaults(out);
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

TEST(SettingsMigration, V14ToV15PreservesAllFieldsAndDefaultsGNSS) {
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

    // gnss_enabled/gnss_receiver_type/gnss_notify are new since v14 (the fields that were silently inserted without
    // a version bump); a v14->v15 migration must land them on their live defaults, not garbage/uninitialized bytes.
    EXPECT_FALSE(out.gnss_enabled);
    EXPECT_EQ(out.gnss_receiver_type, SettingsManager::kGNSSReceiverNone);
    EXPECT_FALSE(out.gnss_notify);

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

TEST(SettingsMigration, UnmigratableVersionReturnsFalse) {
    uint8_t blob[sizeof(settings_v12::Settings)] = {0};
    SettingsManager::Settings out;
    // Pre-v12, unknown-future, and current version are all not migratable by this utility.
    EXPECT_FALSE(SettingsMigrator::Migrate(blob, sizeof(blob), 11, out));
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

    // Fields added after v12 take their defaults: Remote ID receive (new in v13)...
    EXPECT_FALSE(out.remote_id_rx_enabled);
    EXPECT_EQ(out.remote_id_transports, (uint8_t)(SettingsManager::kRemoteIDTransportBLE4 |
                                                  SettingsManager::kRemoteIDTransportBLE5Long |
                                                  SettingsManager::kRemoteIDTransportWiFiBeacon));
    // ...and Remote ID transmit (new in v14). A v12 blob migrates through the full v12 -> v13 -> v14 chain.
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

    EXPECT_TRUE(settings_manager.Sanitize());

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