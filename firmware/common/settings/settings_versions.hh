#pragma once

#include <cstddef>  // offsetof
#include <cstdint>

/**
 * Frozen, immutable snapshots of historical SettingsManager::Settings layouts, used by the settings migration utility
 * (settings_migration.{hh,cpp}) to reinterpret an old on-flash/EEPROM settings blob and field-copy it forward into the
 * current live struct.
 *
 * RULES for these snapshots:
 *   - They are PURE PODs: no methods, no constructors, no dependencies on the live SettingsManager types or enums.
 *     Enum members are stored as their fixed-width underlying integer types, and all array-size constants are hardcoded
 *     literals. This makes each snapshot immune to any future change in the live Settings struct or its enums/constants.
 *   - Once a version is frozen here it is NEVER edited again. The byte layout is locked with static_asserts.
 *
 * RECIPE when bumping kSettingsVersion (e.g. 13 -> 14):
 *   1. Copy the CURRENT live SettingsManager::Settings layout into a new `namespace settings_v13` snapshot here
 *      (translating enum members to their underlying types, hardcoding constants), and lock it with static_asserts.
 *   2. Add a `MigrateV13ToV14()` step and extend the dispatcher in settings_migration.cpp.
 */
namespace settings_v12 {

// v12 array-size constants (hardcoded so the snapshot never moves if the live constants change).
static constexpr uint16_t kHostnameMaxLen = 32;
static constexpr uint16_t kWiFiSSIDMaxLen = 31;
static constexpr uint16_t kWiFiPasswordMaxLen = 63;
static constexpr uint16_t kNumSerialInterfaces = 3;
static constexpr uint16_t kMaxNumFeeds = 10;
static constexpr uint16_t kFeedURIMaxNumChars = 63;
static constexpr uint16_t kFeedReceiverIDNumBytes = 8;

// Byte-identical copy of v12 SettingsManager::Settings::CoreNetworkSettings (natural alignment, align 4 via crc32).
struct CoreNetworkSettings {
    bool esp32_enabled;
    char hostname[kHostnameMaxLen + 1];
    bool wifi_ap_enabled;
    uint8_t wifi_ap_channel;
    char wifi_ap_ssid[kWiFiSSIDMaxLen + 2];
    char wifi_ap_password[kWiFiPasswordMaxLen + 2];
    bool wifi_sta_enabled;
    char wifi_sta_ssid[kWiFiSSIDMaxLen + 2];
    char wifi_sta_password[kWiFiPasswordMaxLen + 2];
    bool ethernet_enabled;
    uint32_t crc32;
};

// Byte-identical copy of v12 SettingsManager::RxPosition (packed).
struct __attribute__((packed)) RxPosition {
    uint8_t source;  // PositionSource underlying type.
    float latitude_deg;
    float longitude_deg;
    int32_t gnss_altitude_ft;
    int32_t baro_altitude_ft;
    float heading_deg;
    int32_t speed_kts;
    uint32_t icao_address;
};

// Byte-identical copy of the v12 Settings layout (kSettingsVersion == 12). Enum members are stored as their v12
// underlying integer types: LogLevel/ReportingProtocol : uint16_t, EnableState : int8_t, SubGHzRadioMode : uint8_t.
struct alignas(4) Settings {
    uint32_t settings_version;
    CoreNetworkSettings core_network_settings;
    bool r1090_rx_enabled;
    int32_t tl_offset_mv;  // v12 used plain `int`.
    bool r1090_bias_tee_enabled;
    uint32_t watchdog_timeout_sec;
    uint16_t log_level;
    uint16_t reporting_protocols[kNumSerialInterfaces];
    uint32_t baud_rates[kNumSerialInterfaces];
    int8_t subg_enabled;
    bool subg_rx_enabled;
    bool subg_bias_tee_enabled;
    uint8_t subg_mode;
    // (v13 inserts `bool remote_id_rx_enabled` and `uint8_t remote_id_transports` here.)
    char feed_uris[kMaxNumFeeds][kFeedURIMaxNumChars + 1];
    uint16_t feed_ports[kMaxNumFeeds];
    bool feed_is_active[kMaxNumFeeds];
    uint16_t feed_protocols[kMaxNumFeeds];
    uint8_t feed_receiver_ids[kMaxNumFeeds][kFeedReceiverIDNumBytes];
    uint8_t mavlink_system_id;
    uint8_t mavlink_component_id;
    RxPosition rx_position;
};

// Lock the v12 byte layout. These values were measured from the real v12 struct (git main:settings.hh). If any of these
// fail, the snapshot has drifted from the true v12 layout and must be corrected (do NOT just update the numbers).
static_assert(sizeof(CoreNetworkSettings) == 240, "v12 CoreNetworkSettings must be 240 bytes.");
static_assert(sizeof(RxPosition) == 29, "v12 RxPosition must be 29 bytes.");
static_assert(sizeof(Settings) == 1088, "v12 Settings must be 1088 bytes.");
static_assert(offsetof(Settings, core_network_settings) == 4, "v12 CoreNetworkSettings offset drift.");
static_assert(offsetof(Settings, subg_mode) == 283, "v12 subg_mode offset drift.");
static_assert(offsetof(Settings, feed_uris) == 284, "v12 feed_uris offset drift.");
static_assert(offsetof(Settings, feed_receiver_ids) == 974, "v12 feed_receiver_ids offset drift.");
static_assert(offsetof(Settings, rx_position) == 1056, "v12 rx_position offset drift.");

}  // namespace settings_v12

/**
 * v13 layout: identical to v12 except two Remote ID *receive* fields inserted between subg_mode and feed_uris (which
 * shifts everything from feed_uris onward by 2 bytes). v14 appends the Remote ID *transmit* settings after them.
 */
namespace settings_v13 {

// v13 array-size constants (hardcoded; identical to v12's).
static constexpr uint16_t kHostnameMaxLen = 32;
static constexpr uint16_t kWiFiSSIDMaxLen = 31;
static constexpr uint16_t kWiFiPasswordMaxLen = 63;
static constexpr uint16_t kNumSerialInterfaces = 3;
static constexpr uint16_t kMaxNumFeeds = 10;
static constexpr uint16_t kFeedURIMaxNumChars = 63;
static constexpr uint16_t kFeedReceiverIDNumBytes = 8;

// Unchanged from v12, but re-declared so this snapshot stays self-contained and immune to edits to the v12 snapshot.
struct CoreNetworkSettings {
    bool esp32_enabled;
    char hostname[kHostnameMaxLen + 1];
    bool wifi_ap_enabled;
    uint8_t wifi_ap_channel;
    char wifi_ap_ssid[kWiFiSSIDMaxLen + 2];
    char wifi_ap_password[kWiFiPasswordMaxLen + 2];
    bool wifi_sta_enabled;
    char wifi_sta_ssid[kWiFiSSIDMaxLen + 2];
    char wifi_sta_password[kWiFiPasswordMaxLen + 2];
    bool ethernet_enabled;
    uint32_t crc32;
};

struct __attribute__((packed)) RxPosition {
    uint8_t source;
    float latitude_deg;
    float longitude_deg;
    int32_t gnss_altitude_ft;
    int32_t baro_altitude_ft;
    float heading_deg;
    int32_t speed_kts;
    uint32_t icao_address;
};

struct alignas(4) Settings {
    uint32_t settings_version;
    CoreNetworkSettings core_network_settings;
    bool r1090_rx_enabled;
    int32_t tl_offset_mv;
    bool r1090_bias_tee_enabled;
    uint32_t watchdog_timeout_sec;
    uint16_t log_level;
    uint16_t reporting_protocols[kNumSerialInterfaces];
    uint32_t baud_rates[kNumSerialInterfaces];
    int8_t subg_enabled;
    bool subg_rx_enabled;
    bool subg_bias_tee_enabled;
    uint8_t subg_mode;
    bool remote_id_rx_enabled;     // Added in v13.
    uint8_t remote_id_transports;  // Added in v13.
    // (v14 appends the Remote ID transmit settings here.)
    char feed_uris[kMaxNumFeeds][kFeedURIMaxNumChars + 1];
    uint16_t feed_ports[kMaxNumFeeds];
    bool feed_is_active[kMaxNumFeeds];
    uint16_t feed_protocols[kMaxNumFeeds];
    uint8_t feed_receiver_ids[kMaxNumFeeds][kFeedReceiverIDNumBytes];
    uint8_t mavlink_system_id;
    uint8_t mavlink_component_id;
    RxPosition rx_position;
};

// Lock the v13 byte layout (measured from the real v13 struct). Same rules as v12: never "fix" these by editing the
// numbers — a failure means the snapshot no longer matches the real historical layout.
static_assert(sizeof(CoreNetworkSettings) == 240, "v13 CoreNetworkSettings must be 240 bytes.");
static_assert(sizeof(RxPosition) == 29, "v13 RxPosition must be 29 bytes.");
static_assert(sizeof(Settings) == 1088, "v13 Settings must be 1088 bytes.");
static_assert(offsetof(Settings, core_network_settings) == 4, "v13 CoreNetworkSettings offset drift.");
static_assert(offsetof(Settings, remote_id_rx_enabled) == 284, "v13 remote_id_rx_enabled offset drift.");
static_assert(offsetof(Settings, feed_uris) == 286, "v13 feed_uris offset drift.");
static_assert(offsetof(Settings, feed_receiver_ids) == 976, "v13 feed_receiver_ids offset drift.");
static_assert(offsetof(Settings, rx_position) == 1058, "v13 rx_position offset drift.");

}  // namespace settings_v13
