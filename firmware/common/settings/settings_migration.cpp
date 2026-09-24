#include "settings_migration.hh"

#include <cstring>  // memcpy

// The frozen nested layouts must stay byte-identical to the live ones for the raw copies below to be valid. If a future
// version changes CoreNetworkSettings or RxPosition, these fire and the migration steps must switch to field-by-field
// copies for those members.
static_assert(sizeof(settings_v13::CoreNetworkSettings) == sizeof(SettingsManager::Settings::CoreNetworkSettings),
              "v13 CoreNetworkSettings layout diverged from live; migration copy is no longer valid.");
static_assert(sizeof(settings_v13::RxPosition) == sizeof(SettingsManager::RxPosition),
              "v13 RxPosition layout diverged from live; migration copy is no longer valid.");
static_assert(sizeof(settings_v12::CoreNetworkSettings) == sizeof(settings_v13::CoreNetworkSettings),
              "v12 -> v13 CoreNetworkSettings layout changed; migration copy is no longer valid.");
static_assert(sizeof(settings_v12::RxPosition) == sizeof(settings_v13::RxPosition),
              "v12 -> v13 RxPosition layout changed; migration copy is no longer valid.");

void SettingsMigrator::MigrateV12ToV13(const settings_v12::Settings& in, settings_v13::Settings& out) {
    out = settings_v13::Settings{};
    out.settings_version = 13;

    // Network settings: byte-identical block.
    memcpy(&out.core_network_settings, &in.core_network_settings, sizeof(in.core_network_settings));

    out.r1090_rx_enabled = in.r1090_rx_enabled;
    out.tl_offset_mv = in.tl_offset_mv;
    out.r1090_bias_tee_enabled = in.r1090_bias_tee_enabled;
    out.watchdog_timeout_sec = in.watchdog_timeout_sec;

    out.log_level = in.log_level;
    for (uint16_t i = 0; i < settings_v13::kNumSerialInterfaces; i++) {
        out.reporting_protocols[i] = in.reporting_protocols[i];
        out.baud_rates[i] = in.baud_rates[i];
    }

    out.subg_enabled = in.subg_enabled;
    out.subg_rx_enabled = in.subg_rx_enabled;
    out.subg_bias_tee_enabled = in.subg_bias_tee_enabled;
    out.subg_mode = in.subg_mode;

    // Remote ID receive settings are new in v13. Match the live defaults (off; all transports selected) so a v12 unit
    // upgrading straight to v14 ends up with the same values it would have gotten from a fresh v13 struct.
    out.remote_id_rx_enabled = false;
    out.remote_id_transports = SettingsManager::kRemoteIDTransportBLE4 | SettingsManager::kRemoteIDTransportBLE5Long |
                               SettingsManager::kRemoteIDTransportWiFiBeacon;

    memcpy(out.feed_uris, in.feed_uris, sizeof(in.feed_uris));
    memcpy(out.feed_ports, in.feed_ports, sizeof(in.feed_ports));
    memcpy(out.feed_is_active, in.feed_is_active, sizeof(in.feed_is_active));
    memcpy(out.feed_protocols, in.feed_protocols, sizeof(in.feed_protocols));
    memcpy(out.feed_receiver_ids, in.feed_receiver_ids, sizeof(in.feed_receiver_ids));

    out.mavlink_system_id = in.mavlink_system_id;
    out.mavlink_component_id = in.mavlink_component_id;

    memcpy(&out.rx_position, &in.rx_position, sizeof(in.rx_position));
}

void SettingsMigrator::MigrateV13ToV14(const settings_v13::Settings& in, SettingsManager::Settings& out) {
    // Start from a fresh, fully-defaulted current struct so any fields added since v13 (the Remote ID transmit settings)
    // take their current defaults and, on the RP2040, DeviceInfo-seeded defaults are applied.
    out = SettingsManager::Settings();

    out.settings_version = kSettingsVersion;

    memcpy(&out.core_network_settings, &in.core_network_settings, sizeof(in.core_network_settings));

    out.r1090_rx_enabled = in.r1090_rx_enabled;
    out.tl_offset_mv = in.tl_offset_mv;
    out.r1090_bias_tee_enabled = in.r1090_bias_tee_enabled;
    out.watchdog_timeout_sec = in.watchdog_timeout_sec;

    out.log_level = static_cast<SettingsManager::LogLevel>(in.log_level);
    for (uint16_t i = 0; i < SettingsManager::SerialInterface::kNumSerialInterfaces; i++) {
        out.reporting_protocols[i] = static_cast<SettingsManager::ReportingProtocol>(in.reporting_protocols[i]);
        out.baud_rates[i] = in.baud_rates[i];
    }

    out.subg_enabled = static_cast<SettingsManager::EnableState>(in.subg_enabled);
    out.subg_rx_enabled = in.subg_rx_enabled;
    out.subg_bias_tee_enabled = in.subg_bias_tee_enabled;
    out.subg_mode = static_cast<SettingsManager::SubGHzRadioMode>(in.subg_mode);

    // Remote ID receive settings carry over from v13.
    out.remote_id_rx_enabled = in.remote_id_rx_enabled;
    out.remote_id_transports = in.remote_id_transports;

    // Remote ID transmit settings are new in v14; left at their defaults (disabled, all transports, empty identity).

    memcpy(out.feed_uris, in.feed_uris, sizeof(in.feed_uris));
    memcpy(out.feed_ports, in.feed_ports, sizeof(in.feed_ports));
    memcpy(out.feed_is_active, in.feed_is_active, sizeof(in.feed_is_active));
    for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
        out.feed_protocols[i] = static_cast<SettingsManager::ReportingProtocol>(in.feed_protocols[i]);
    }
    memcpy(out.feed_receiver_ids, in.feed_receiver_ids, sizeof(in.feed_receiver_ids));

    out.mavlink_system_id = in.mavlink_system_id;
    out.mavlink_component_id = in.mavlink_component_id;

    memcpy(&out.rx_position, &in.rx_position, sizeof(in.rx_position));
}

bool SettingsMigrator::Migrate(const uint8_t* blob, uint16_t blob_len, uint32_t from_version,
                               SettingsManager::Settings& out) {
    // Each case enters the chain at its own version and falls through the remaining one-step upgrades to reach the
    // current version, so any stored version >= kOldestMigratableVersion lands on the live layout.
    settings_v13::Settings v13;

    switch (from_version) {
        case 12: {
            if (blob_len < sizeof(settings_v12::Settings)) {
                return false;  // Stored blob too short to be a valid v12 struct.
            }
            // Copy into a properly-aligned local rather than reinterpret_cast'ing the (possibly-unaligned) blob.
            settings_v12::Settings v12;
            memcpy(&v12, blob, sizeof(v12));
            MigrateV12ToV13(v12, v13);
            break;
        }
        case 13: {
            if (blob_len < sizeof(settings_v13::Settings)) {
                return false;
            }
            memcpy(&v13, blob, sizeof(v13));
            break;
        }
        default:
            // Older than kOldestMigratableVersion, unknown, or already current: nothing to migrate.
            return false;
    }

    MigrateV13ToV14(v13, out);  // Final step: v13 is the newest frozen version, so this lands on the live struct.
    return true;
}
