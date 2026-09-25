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

// Builds a v14 blob by raw byte offset, independent of settings_v14::Settings, using the layout measured from the
// struct shipped in adsbee_1090-0.9.1-rc2 (1140 B). Every field gets a distinctive non-default value at its offset, so a
// snapshot field at the wrong offset (or of the wrong size) reads something else and fails. Guards against the frozen
// snapshot drifting from what devices actually hold.
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

    EXPECT_TRUE(settings_manager.Sanitize());

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
