#include <cstdio>
#include <cstring>
#include <string>

#include "gtest/gtest.h"
#include "settings.hh"
#include "settings_migration.hh"

// Settings blobs written by each release's own Settings struct (settings_blobs/gen.cc), with a WiFi station, hostname,
// AP, Ethernet and a feed configured. Main must keep all of them across an upgrade.

namespace {

struct ReleaseBlob {
    const char* release;
    uint32_t version;
    bool migratable;  // Versions older than SettingsMigrator::kOldestMigratableVersion keep only CoreNetworkSettings.
};

const ReleaseBlob kReleaseBlobs[] = {
    {"0.9.0-rc4", 10, false}, {"0.9.0-rc9", 11, false}, {"0.9.0-rc19", 12, true},
    {"0.9.0", 12, true},      {"0.9.1-rc2", 14, true},  {"0.9.1-rc6", 15, true},
};

// Reads a blob the way Load() does: sizeof(Settings) bytes, the tail past the old struct left as erased EEPROM.
bool ReadBlob(const char* release, uint8_t* buf, size_t buf_len) {
    std::string path = std::string(SETTINGS_BLOBS_DIR) + "/adsbee_1090-" + release + ".bin";
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) return false;
    memset(buf, 0xFF, buf_len);
    size_t n = fread(buf, 1, buf_len, f);
    fclose(f);
    return n > 0;
}

void ExpectCoreNetworkSettingsKept(SettingsManager::Settings::CoreNetworkSettings& cns) {
    EXPECT_TRUE(cns.IsValid());
    EXPECT_TRUE(cns.esp32_enabled);
    EXPECT_STREQ(cns.hostname, "attic-bee");
    EXPECT_TRUE(cns.wifi_sta_enabled);
    EXPECT_STREQ(cns.wifi_sta_ssid, "Bench Net 2.4G");
    EXPECT_STREQ(cns.wifi_sta_password, "correct horse battery staple");
    EXPECT_FALSE(cns.wifi_ap_enabled);
    EXPECT_EQ(cns.wifi_ap_channel, 6);
    EXPECT_STREQ(cns.wifi_ap_ssid, "ADSBee-AP-1234");
    EXPECT_STREQ(cns.wifi_ap_password, "apsecret99");
    EXPECT_TRUE(cns.ethernet_enabled);
}

}  // namespace

TEST(SettingsReleaseBlobs, EveryReleaseKeepsItsNetworkSettings) {
    for (const ReleaseBlob& r : kReleaseBlobs) {
        SCOPED_TRACE(r.release);
        static uint8_t raw[sizeof(SettingsManager::Settings)];
        ASSERT_TRUE(ReadBlob(r.release, raw, sizeof(raw)));
        uint32_t version;
        memcpy(&version, raw, sizeof(version));
        ASSERT_EQ(version, r.version);

        SettingsManager::Settings out;
        bool migrated = SettingsMigrator::Migrate(raw, sizeof(raw), version, out);
        ASSERT_EQ(migrated, r.migratable);
        if (!migrated) {
            // Load() resets to defaults and restores this block when its CRC checks out.
            SettingsManager::Settings stored;
            memcpy(&stored, raw, sizeof(stored));
            ExpectCoreNetworkSettingsKept(stored.core_network_settings);
            continue;
        }

        EXPECT_EQ(out.settings_version, kSettingsVersion);
        ExpectCoreNetworkSettingsKept(out.core_network_settings);
        EXPECT_EQ(out.ip_mode, SettingsManager::kIPModeDual);
        for (uint16_t i = 0; i < SettingsManager::Settings::kWiFiSTANumExtraNetworks; i++) {
            EXPECT_STREQ(out.wifi_sta_extra_networks[i].ssid, "");
        }

        EXPECT_FALSE(out.r1090_rx_enabled);
        EXPECT_EQ(out.tl_offset_mv, 777);
        EXPECT_TRUE(out.r1090_bias_tee_enabled);
        EXPECT_EQ(out.watchdog_timeout_sec, 42u);
        EXPECT_EQ(static_cast<int>(out.log_level), 3);
        EXPECT_EQ(static_cast<int>(out.reporting_protocols[1]), 2);
        EXPECT_EQ(out.baud_rates[1], 921600u);
        EXPECT_TRUE(out.feeds_enabled);
        EXPECT_STREQ(out.feed_uris[1], "feed.adsb.lol");
        EXPECT_EQ(out.feed_ports[1], 30004);
        EXPECT_TRUE(out.feed_is_active[1]);
        EXPECT_EQ(static_cast<int>(out.feed_protocols[1]), 5);
        EXPECT_EQ(out.feed_receiver_ids[1][7], 0xA7);
        EXPECT_EQ(out.mavlink_system_id, 9);
        EXPECT_EQ(out.mavlink_component_id, 200);
    }
}
