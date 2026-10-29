#include <cstring>
#include <new>

#include "config_hash.hh"
#include "gtest/gtest.h"
#include "settings_hash.hh"

using Settings = SettingsManager::Settings;
using SettingsHash::kScopeFleet;
using SettingsHash::kScopeFull;

// Default settings built on top of storage filled with fill, so padding and unused string Bytes hold it.
static Settings* NewSettings(uint8_t* storage, uint8_t fill) {
    memset(storage, fill, sizeof(Settings));
    return new (storage) Settings();
}

TEST(ConfigHasher, MatchesCRC32OfTheSerializedBytes) {
    EXPECT_EQ(ConfigHasher().Value(), crc32_ieee_802_3(nullptr, 0));

    ConfigHasher hasher;
    hasher.AddBytes((const uint8_t*)"1234", 4);
    hasher.AddBytes((const uint8_t*)"56789", 5);
    EXPECT_EQ(hasher.Value(), 0xCBF43926u);  // CRC32 check value for "123456789".

    ConfigHasher typed;
    typed.AddU8(0x12);
    typed.AddBool(true);
    typed.AddU16(0xBEEF);
    typed.AddU32(0xDEADC0DE);
    typed.AddI32(-2);
    typed.AddString("hi", 4);
    const uint8_t expected[] = {0x12, 0x01, 0xEF, 0xBE, 0xDE, 0xC0, 0xAD, 0xDE,
                                0xFE, 0xFF, 0xFF, 0xFF, 'h',  'i',  0x00, 0x00};
    EXPECT_EQ(typed.Value(), crc32_ieee_802_3(expected, sizeof(expected)));
}

TEST(ConfigHasher, StringIgnoresBytesAfterTheTerminator) {
    char a[8] = {'a', 'b', '\0', 'x', 'y', 'z', 'x', 'y'};
    char b[8] = {'a', 'b', '\0', '\0', '\0', '\0', '\0', '\0'};
    ConfigHasher hasher_a, hasher_b;
    hasher_a.AddString(a, sizeof(a));
    hasher_b.AddString(b, sizeof(b));
    EXPECT_EQ(hasher_a.Value(), hasher_b.Value());
}

TEST(ConfigHasher, CombineDependsOnOrder) {
    uint32_t ab[] = {0x11111111, 0x22222222};
    uint32_t ba[] = {0x22222222, 0x11111111};
    const uint8_t ab_bytes[] = {0x11, 0x11, 0x11, 0x11, 0x22, 0x22, 0x22, 0x22};
    EXPECT_EQ(ConfigHasher::Combine(ab, 2), crc32_ieee_802_3(ab_bytes, sizeof(ab_bytes)));
    EXPECT_NE(ConfigHasher::Combine(ab, 2), ConfigHasher::Combine(ba, 2));
}

TEST(SettingsHash, SameSettingsSameHashWhateverThePadding) {
    alignas(Settings) uint8_t storage_a[sizeof(Settings)];
    alignas(Settings) uint8_t storage_b[sizeof(Settings)];
    Settings* a = NewSettings(storage_a, 0xAA);
    Settings* b = NewSettings(storage_b, 0x55);
    // Stale characters after the terminators.
    strcpy(a->core_network_settings.wifi_sta_ssid, "a much longer network name");
    strcpy(a->core_network_settings.wifi_sta_ssid, "net");
    strcpy(b->core_network_settings.wifi_sta_ssid, "net");
    ASSERT_NE(memcmp(storage_a, storage_b, sizeof(Settings)), 0);
    EXPECT_EQ(SettingsHash::Compute(*a, kScopeFleet), SettingsHash::Compute(*b, kScopeFleet));
    EXPECT_EQ(SettingsHash::Compute(*a, kScopeFull), SettingsHash::Compute(*b, kScopeFull));
}

TEST(SettingsHash, DefaultSettingsHashIsPinned) {
    // Host defaults (no device info, access point channel 1). Changes only with the Settings struct or its defaults.
    Settings settings;
    EXPECT_EQ(SettingsHash::Compute(settings, kScopeFleet), 0x6ED01E80u);
    EXPECT_EQ(SettingsHash::Compute(settings, kScopeFull), 0x1D6E4B4Bu);
}

TEST(SettingsHash, StoredCRCDoesNotCount) {
    Settings settings;
    uint32_t fleet = SettingsHash::Compute(settings, kScopeFleet);
    uint32_t full = SettingsHash::Compute(settings, kScopeFull);
    settings.core_network_settings.UpdateCRC32();
    EXPECT_EQ(SettingsHash::Compute(settings, kScopeFleet), fleet);
    EXPECT_EQ(SettingsHash::Compute(settings, kScopeFull), full);
}

// Applies change to default settings and reports which hashes moved.
template <typename Change>
static void HashesAfter(Change change, bool& fleet_changed, bool& full_changed) {
    Settings before, after;
    change(after);
    fleet_changed = SettingsHash::Compute(before, kScopeFleet) != SettingsHash::Compute(after, kScopeFleet);
    full_changed = SettingsHash::Compute(before, kScopeFull) != SettingsHash::Compute(after, kScopeFull);
}

#define EXPECT_CHANGES_BOTH(change)                                            \
    {                                                                          \
        bool fleet_changed, full_changed;                                      \
        HashesAfter([](Settings& s) { change; }, fleet_changed, full_changed); \
        EXPECT_TRUE(fleet_changed) << #change;                                 \
        EXPECT_TRUE(full_changed) << #change;                                  \
    }

#define EXPECT_CHANGES_FULL_ONLY(change)                                       \
    {                                                                          \
        bool fleet_changed, full_changed;                                      \
        HashesAfter([](Settings& s) { change; }, fleet_changed, full_changed); \
        EXPECT_FALSE(fleet_changed) << #change;                                \
        EXPECT_TRUE(full_changed) << #change;                                  \
    }

TEST(SettingsHash, EverySharedSettingChangesBothHashes) {
    EXPECT_CHANGES_BOTH(s.settings_version++);
    EXPECT_CHANGES_BOTH(s.core_network_settings.esp32_enabled = false);
    EXPECT_CHANGES_BOTH(s.core_network_settings.wifi_ap_enabled = false);
    EXPECT_CHANGES_BOTH(strcpy(s.core_network_settings.wifi_ap_password, "secret"));
    EXPECT_CHANGES_BOTH(s.core_network_settings.wifi_sta_enabled = true);
    EXPECT_CHANGES_BOTH(strcpy(s.core_network_settings.wifi_sta_ssid, "net"));
    EXPECT_CHANGES_BOTH(strcpy(s.core_network_settings.wifi_sta_password, "secret"));
    EXPECT_CHANGES_BOTH(s.core_network_settings.ethernet_enabled = true);
    EXPECT_CHANGES_BOTH(s.r1090_rx_enabled = false);
    EXPECT_CHANGES_BOTH(s.tl_offset_mv++);
    EXPECT_CHANGES_BOTH(s.r1090_bias_tee_enabled = true);
    EXPECT_CHANGES_BOTH(s.watchdog_timeout_sec++);
    EXPECT_CHANGES_BOTH(s.led_enabled = false);
    EXPECT_CHANGES_BOTH(s.feeds_enabled = false);
    EXPECT_CHANGES_BOTH(s.gnss_enabled = true);
    EXPECT_CHANGES_BOTH(s.gnss_receiver_type = SettingsManager::kGNSSReceiverGeneric);
    EXPECT_CHANGES_BOTH(s.gnss_notify = true);
    EXPECT_CHANGES_BOTH(s.log_level = SettingsManager::kInfo);
    EXPECT_CHANGES_BOTH(s.reporting_protocols[SettingsManager::kCommsUART] = SettingsManager::kMAVLINK2);
    EXPECT_CHANGES_BOTH(s.baud_rates[SettingsManager::kGNSSUART] = 115200);
    EXPECT_CHANGES_BOTH(s.subg_enabled = SettingsManager::kEnableStateDisabled);
    EXPECT_CHANGES_BOTH(s.subg_rx_enabled = false);
    EXPECT_CHANGES_BOTH(s.subg_bias_tee_enabled = true);
    EXPECT_CHANGES_BOTH(s.remote_id_rx_enabled = true);
    EXPECT_CHANGES_BOTH(s.remote_id_transports = 1);
    EXPECT_CHANGES_BOTH(s.remote_id_tx_enabled = true);
    EXPECT_CHANGES_BOTH(s.remote_id_tx_transports = 1);
    EXPECT_CHANGES_BOTH(s.remote_id_tx_uas_id_type++);
    EXPECT_CHANGES_BOTH(s.remote_id_tx_ua_type++);
    EXPECT_CHANGES_BOTH(strcpy(s.remote_id_tx_operator_id, "OP-1"));
    EXPECT_CHANGES_BOTH(strcpy(s.feed_uris[0], "feed.example.com"));
    EXPECT_CHANGES_BOTH(s.feed_ports[0] = 30005);
    EXPECT_CHANGES_BOTH(s.feed_is_active[0] = true);
    EXPECT_CHANGES_BOTH(s.feed_protocols[0] = SettingsManager::kBeast);
    EXPECT_CHANGES_BOTH(s.mavlink_system_id++);
    EXPECT_CHANGES_BOTH(s.mavlink_component_id++);
    EXPECT_CHANGES_BOTH(s.rx_position.source = SettingsManager::RxPosition::kPositionSourceGNSS);
    EXPECT_CHANGES_BOTH(s.ip_mode = SettingsManager::kIPModeIPv4);
    EXPECT_CHANGES_BOTH(strcpy(s.wifi_sta_extra_networks[1].ssid, "net"));
    EXPECT_CHANGES_BOTH(strcpy(s.wifi_sta_extra_networks[1].password, "secret"));
    EXPECT_CHANGES_BOTH(s.auto_update_enabled = true);
    EXPECT_CHANGES_BOTH(s.feed_transports[0] = SettingsManager::kFeedTransportMQTT);
    EXPECT_CHANGES_BOTH(strcpy(s.mqtt_broker.host, "broker.lan"));
    EXPECT_CHANGES_BOTH(s.mqtt_broker.port = 8883);
    EXPECT_CHANGES_BOTH(s.mqtt_broker.tls = true);
    EXPECT_CHANGES_BOTH(s.mqtt_broker.qos = 1);
    EXPECT_CHANGES_BOTH(s.mqtt_broker.transport = SettingsManager::kMQTTTransportWebSocket);
    EXPECT_CHANGES_BOTH(strcpy(s.mqtt_broker.username, "bee"));
    EXPECT_CHANGES_BOTH(strcpy(s.mqtt_broker.password, "secret"));
    EXPECT_CHANGES_BOTH(strcpy(s.mqtt_broker.topic_prefix, "home"));
    EXPECT_CHANGES_BOTH(strcpy(s.mqtt_broker.websocket_path, "/ws"));
    EXPECT_CHANGES_BOTH(s.asterix_sac = 25);
    EXPECT_CHANGES_BOTH(strcpy(s.ntp_server, "time.example.com"));
    EXPECT_CHANGES_BOTH(s.ble_output_mode = SettingsManager::kBLEOutputGDL90);
}

TEST(SettingsHash, FeedSlotsAreNotInterchangeable) {
    Settings a, b;
    strcpy(a.feed_uris[0], "feed.example.com");
    strcpy(b.feed_uris[1], "feed.example.com");
    EXPECT_NE(SettingsHash::Compute(a, kScopeFleet), SettingsHash::Compute(b, kScopeFleet));
}

TEST(SettingsHash, PerUnitSettingsChangeOnlyTheFullHash) {
    EXPECT_CHANGES_FULL_ONLY(strcpy(s.core_network_settings.hostname, "ADSBee1090-0240907986541"));
    EXPECT_CHANGES_FULL_ONLY(strcpy(s.core_network_settings.wifi_ap_ssid, "ADSBee1090-0240907986541"));
    EXPECT_CHANGES_FULL_ONLY(s.core_network_settings.wifi_ap_channel = 11);
    EXPECT_CHANGES_FULL_ONLY(s.feed_receiver_ids[9][7] = 0x42);
    EXPECT_CHANGES_FULL_ONLY(strcpy(s.remote_id_tx_uas_id, "SERIAL123"));
    EXPECT_CHANGES_FULL_ONLY(s.asterix_sic = 7);
}

TEST(SettingsHash, ReceiverPositionCountsOnlyWhenItIsASetting) {
    // Followed aircraft and GNSS overwrite the position while running.
    Settings a, b;
    b.rx_position.latitude_deg = 47.5f;
    b.rx_position.icao_address = 0xBEE001;
    EXPECT_EQ(SettingsHash::Compute(a, kScopeFull), SettingsHash::Compute(b, kScopeFull));

    a.rx_position.source = b.rx_position.source = SettingsManager::RxPosition::kPositionSourceFixed;
    EXPECT_EQ(SettingsHash::Compute(a, kScopeFleet), SettingsHash::Compute(b, kScopeFleet));
    EXPECT_NE(SettingsHash::Compute(a, kScopeFull), SettingsHash::Compute(b, kScopeFull));
    b.rx_position.latitude_deg = a.rx_position.latitude_deg;
    EXPECT_EQ(SettingsHash::Compute(a, kScopeFull), SettingsHash::Compute(b, kScopeFull));  // ICAO not used.

    a.rx_position.source = b.rx_position.source = SettingsManager::RxPosition::kPositionSourceAircraftMatchingICAO;
    EXPECT_EQ(SettingsHash::Compute(a, kScopeFleet), SettingsHash::Compute(b, kScopeFleet));
    EXPECT_NE(SettingsHash::Compute(a, kScopeFull), SettingsHash::Compute(b, kScopeFull));
}
