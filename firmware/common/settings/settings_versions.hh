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
 *   - Only versions shipped in a tagged release or RC get a snapshot. Versions that never reached a tag (v13, v16)
 *     have none: no device holds them, and migrations step over them.
 *
 * RECIPE when bumping kSettingsVersion (only when the current version shipped in a tag, e.g. 20 -> 21):
 *   1. Copy the CURRENT live SettingsManager::Settings layout into a new `namespace settings_v20` snapshot here
 *      (translating enum members to their underlying types, hardcoding constants), and lock it with static_asserts.
 *   2. Add a `MigrateV20ToV21()` step and extend the dispatcher in settings_migration.cpp.
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
    // (v14 inserts `bool remote_id_rx_enabled` and `uint8_t remote_id_transports` here.)
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
 * v14 layout: v12 plus the Remote ID *receive* settings (`remote_id_rx_enabled`, `remote_id_transports`, between
 * subg_mode and feed_uris; added in the unshipped v13), `led_enabled`, the GNSS settings (`gnss_enabled`,
 * `gnss_receiver_type`, `gnss_notify`) and the Remote ID *transmit* settings (remote_id_tx_*). This is the layout
 * every adsbee_1090 release tagged with kSettingsVersion == 14 shipped (0.9.1-rc1/rc2); adsbee_1421 has its own
 * settings. The GNSS fields were added (e1f28fe7) a few commits after v14 was introduced (82f9813e) without a version
 * bump, but no release carried the pre-GNSS layout, so this is the only v14 layout real devices hold. The three GNSS
 * bytes landed in what had been padding, so everything from baud_rates onward sits at the same offsets either way.
 */
namespace settings_v14 {

// v14 array-size constants (hardcoded; identical to v12's, plus the Remote ID ID length).
static constexpr uint16_t kHostnameMaxLen = 32;
static constexpr uint16_t kWiFiSSIDMaxLen = 31;
static constexpr uint16_t kWiFiPasswordMaxLen = 63;
static constexpr uint16_t kNumSerialInterfaces = 3;
static constexpr uint16_t kMaxNumFeeds = 10;
static constexpr uint16_t kFeedURIMaxNumChars = 63;
static constexpr uint16_t kFeedReceiverIDNumBytes = 8;
static constexpr uint16_t kRemoteIDIDMaxLen = 20;

// Unchanged from v12, but re-declared so this snapshot stays self-contained.
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

// Byte-identical copy of the shipped v14 Settings layout. Enum members are stored as their v14 underlying integer types:
// LogLevel/ReportingProtocol : uint16_t, EnableState : int8_t, SubGHzRadioMode/GNSSReceiverType : uint8_t.
struct alignas(4) Settings {
    uint32_t settings_version;
    CoreNetworkSettings core_network_settings;
    bool r1090_rx_enabled;
    int32_t tl_offset_mv;
    bool r1090_bias_tee_enabled;
    uint32_t watchdog_timeout_sec;
    bool led_enabled;
    bool gnss_enabled;
    uint8_t gnss_receiver_type;
    bool gnss_notify;
    uint16_t log_level;
    uint16_t reporting_protocols[kNumSerialInterfaces];
    uint32_t baud_rates[kNumSerialInterfaces];
    int8_t subg_enabled;
    bool subg_rx_enabled;
    bool subg_bias_tee_enabled;
    uint8_t subg_mode;
    bool remote_id_rx_enabled;
    uint8_t remote_id_transports;
    bool remote_id_tx_enabled;         // Added in v14.
    uint8_t remote_id_tx_transports;   // Added in v14.
    uint8_t remote_id_tx_uas_id_type;  // Added in v14.
    uint8_t remote_id_tx_ua_type;      // Added in v14.
    char remote_id_tx_uas_id[kRemoteIDIDMaxLen + 1];      // Added in v14.
    char remote_id_tx_operator_id[kRemoteIDIDMaxLen + 1];  // Added in v14.
    char feed_uris[kMaxNumFeeds][kFeedURIMaxNumChars + 1];
    uint16_t feed_ports[kMaxNumFeeds];
    bool feed_is_active[kMaxNumFeeds];
    uint16_t feed_protocols[kMaxNumFeeds];
    uint8_t feed_receiver_ids[kMaxNumFeeds][kFeedReceiverIDNumBytes];
    uint8_t mavlink_system_id;
    uint8_t mavlink_component_id;
    RxPosition rx_position;
};

// Lock the v14 byte layout (measured from the live struct at tag adsbee_1090-0.9.1-rc2). Same rules as v12:
// never "fix" these by editing the numbers -- a failure means the snapshot no longer matches the true historical
// layout.
static_assert(sizeof(CoreNetworkSettings) == 240, "v14 CoreNetworkSettings must be 240 bytes.");
static_assert(sizeof(RxPosition) == 29, "v14 RxPosition must be 29 bytes.");
static_assert(sizeof(Settings) == 1140, "v14 Settings must be 1140 bytes.");
static_assert(offsetof(Settings, core_network_settings) == 4, "v14 CoreNetworkSettings offset drift.");
static_assert(offsetof(Settings, r1090_rx_enabled) == 244, "v14 r1090_rx_enabled offset drift.");
static_assert(offsetof(Settings, led_enabled) == 260, "v14 led_enabled offset drift.");
static_assert(offsetof(Settings, gnss_enabled) == 261, "v14 gnss_enabled offset drift.");
static_assert(offsetof(Settings, gnss_receiver_type) == 262, "v14 gnss_receiver_type offset drift.");
static_assert(offsetof(Settings, gnss_notify) == 263, "v14 gnss_notify offset drift.");
static_assert(offsetof(Settings, log_level) == 264, "v14 log_level offset drift.");
static_assert(offsetof(Settings, subg_mode) == 287, "v14 subg_mode offset drift.");
static_assert(offsetof(Settings, remote_id_rx_enabled) == 288, "v14 remote_id_rx_enabled offset drift.");
static_assert(offsetof(Settings, remote_id_tx_enabled) == 290, "v14 remote_id_tx_enabled offset drift.");
static_assert(offsetof(Settings, feed_uris) == 336, "v14 feed_uris offset drift.");
static_assert(offsetof(Settings, feed_receiver_ids) == 1026, "v14 feed_receiver_ids offset drift.");
static_assert(offsetof(Settings, mavlink_system_id) == 1106, "v14 mavlink_system_id offset drift.");
static_assert(offsetof(Settings, rx_position) == 1108, "v14 rx_position offset drift.");

}  // namespace settings_v14

/**
 * v15 layout: v14 plus `feeds_enabled` after `led_enabled` (shifting gnss_* and everything after them by one byte).
 * Shipped in adsbee_1090 0.9.1-rc3 through 0.9.1-rc6.
 */
namespace settings_v15 {

static constexpr uint16_t kHostnameMaxLen = 32;
static constexpr uint16_t kWiFiSSIDMaxLen = 31;
static constexpr uint16_t kWiFiPasswordMaxLen = 63;
static constexpr uint16_t kNumSerialInterfaces = 3;
static constexpr uint16_t kMaxNumFeeds = 10;
static constexpr uint16_t kFeedURIMaxNumChars = 63;
static constexpr uint16_t kFeedReceiverIDNumBytes = 8;
static constexpr uint16_t kRemoteIDIDMaxLen = 20;

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

// Byte-identical copy of the shipped v15 Settings layout. Enum members are stored as their underlying integer types.
struct alignas(4) Settings {
    uint32_t settings_version;
    CoreNetworkSettings core_network_settings;
    bool r1090_rx_enabled;
    int32_t tl_offset_mv;
    bool r1090_bias_tee_enabled;
    uint32_t watchdog_timeout_sec;
    bool led_enabled;
    bool feeds_enabled;  // Added in v15.
    bool gnss_enabled;
    uint8_t gnss_receiver_type;
    bool gnss_notify;
    uint16_t log_level;
    uint16_t reporting_protocols[kNumSerialInterfaces];
    uint32_t baud_rates[kNumSerialInterfaces];
    int8_t subg_enabled;
    bool subg_rx_enabled;
    bool subg_bias_tee_enabled;
    uint8_t subg_mode;
    bool remote_id_rx_enabled;
    uint8_t remote_id_transports;
    bool remote_id_tx_enabled;
    uint8_t remote_id_tx_transports;
    uint8_t remote_id_tx_uas_id_type;
    uint8_t remote_id_tx_ua_type;
    char remote_id_tx_uas_id[kRemoteIDIDMaxLen + 1];
    char remote_id_tx_operator_id[kRemoteIDIDMaxLen + 1];
    char feed_uris[kMaxNumFeeds][kFeedURIMaxNumChars + 1];
    uint16_t feed_ports[kMaxNumFeeds];
    bool feed_is_active[kMaxNumFeeds];
    uint16_t feed_protocols[kMaxNumFeeds];
    uint8_t feed_receiver_ids[kMaxNumFeeds][kFeedReceiverIDNumBytes];
    uint8_t mavlink_system_id;
    uint8_t mavlink_component_id;
    RxPosition rx_position;
};

// Lock the v15 byte layout (the live layout lock in settings.hh at kSettingsVersion == 15).
static_assert(sizeof(CoreNetworkSettings) == 240, "v15 CoreNetworkSettings must be 240 bytes.");
static_assert(sizeof(RxPosition) == 29, "v15 RxPosition must be 29 bytes.");
static_assert(sizeof(Settings) == 1144, "v15 Settings must be 1144 bytes.");
static_assert(offsetof(Settings, core_network_settings) == 4, "v15 CoreNetworkSettings offset drift.");
static_assert(offsetof(Settings, led_enabled) == 260, "v15 led_enabled offset drift.");
static_assert(offsetof(Settings, feeds_enabled) == 261, "v15 feeds_enabled offset drift.");
static_assert(offsetof(Settings, gnss_notify) == 264, "v15 gnss_notify offset drift.");
static_assert(offsetof(Settings, log_level) == 266, "v15 log_level offset drift.");
static_assert(offsetof(Settings, baud_rates) == 276, "v15 baud_rates offset drift.");
static_assert(offsetof(Settings, subg_mode) == 291, "v15 subg_mode offset drift.");
static_assert(offsetof(Settings, remote_id_tx_enabled) == 294, "v15 remote_id_tx_enabled offset drift.");
static_assert(offsetof(Settings, feed_uris) == 340, "v15 feed_uris offset drift.");
static_assert(offsetof(Settings, feed_protocols) == 1010, "v15 feed_protocols offset drift.");
static_assert(offsetof(Settings, feed_receiver_ids) == 1030, "v15 feed_receiver_ids offset drift.");
static_assert(offsetof(Settings, mavlink_system_id) == 1110, "v15 mavlink_system_id offset drift.");
static_assert(offsetof(Settings, rx_position) == 1112, "v15 rx_position offset drift.");

}  // namespace settings_v15

/**
 * v17 layout: v15 plus `ip_mode` in the tail padding after rx_position (adsbee_1090 #281, in the unshipped v16) and
 * `wifi_sta_extra_networks` (adsbee_1090 #280). v18 put `auto_update_enabled` in its tail padding.
 */
namespace settings_v17 {

static constexpr uint16_t kHostnameMaxLen = 32;
static constexpr uint16_t kWiFiSSIDMaxLen = 31;
static constexpr uint16_t kWiFiPasswordMaxLen = 63;
static constexpr uint16_t kNumSerialInterfaces = 3;
static constexpr uint16_t kMaxNumFeeds = 10;
static constexpr uint16_t kFeedURIMaxNumChars = 63;
static constexpr uint16_t kFeedReceiverIDNumBytes = 8;
static constexpr uint16_t kRemoteIDIDMaxLen = 20;
static constexpr uint16_t kWiFiSTANumExtraNetworks = 2;

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

struct WiFiNetwork {
    char ssid[kWiFiSSIDMaxLen + 2];
    char password[kWiFiPasswordMaxLen + 2];
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

// Byte-identical copy of the v17 Settings layout. Enum members are stored as their underlying integer types.
struct alignas(4) Settings {
    uint32_t settings_version;
    CoreNetworkSettings core_network_settings;
    bool r1090_rx_enabled;
    int32_t tl_offset_mv;
    bool r1090_bias_tee_enabled;
    uint32_t watchdog_timeout_sec;
    bool led_enabled;
    bool feeds_enabled;  // Added in v15.
    bool gnss_enabled;
    uint8_t gnss_receiver_type;
    bool gnss_notify;
    uint16_t log_level;
    uint16_t reporting_protocols[kNumSerialInterfaces];
    uint32_t baud_rates[kNumSerialInterfaces];
    int8_t subg_enabled;
    bool subg_rx_enabled;
    bool subg_bias_tee_enabled;
    uint8_t subg_mode;
    bool remote_id_rx_enabled;
    uint8_t remote_id_transports;
    bool remote_id_tx_enabled;
    uint8_t remote_id_tx_transports;
    uint8_t remote_id_tx_uas_id_type;
    uint8_t remote_id_tx_ua_type;
    char remote_id_tx_uas_id[kRemoteIDIDMaxLen + 1];
    char remote_id_tx_operator_id[kRemoteIDIDMaxLen + 1];
    char feed_uris[kMaxNumFeeds][kFeedURIMaxNumChars + 1];
    uint16_t feed_ports[kMaxNumFeeds];
    bool feed_is_active[kMaxNumFeeds];
    uint16_t feed_protocols[kMaxNumFeeds];
    uint8_t feed_receiver_ids[kMaxNumFeeds][kFeedReceiverIDNumBytes];
    uint8_t mavlink_system_id;
    uint8_t mavlink_component_id;
    RxPosition rx_position;
    uint8_t ip_mode;
    WiFiNetwork wifi_sta_extra_networks[kWiFiSTANumExtraNetworks];  // Added in v17.
};

// Lock the v17 byte layout (the live layout lock in settings.hh at kSettingsVersion == 17).
static_assert(sizeof(CoreNetworkSettings) == 240, "v17 CoreNetworkSettings must be 240 bytes.");
static_assert(sizeof(RxPosition) == 29, "v17 RxPosition must be 29 bytes.");
static_assert(sizeof(Settings) == 1340, "v17 Settings must be 1340 bytes.");
static_assert(offsetof(Settings, core_network_settings) == 4, "v17 CoreNetworkSettings offset drift.");
static_assert(offsetof(Settings, led_enabled) == 260, "v17 led_enabled offset drift.");
static_assert(offsetof(Settings, feeds_enabled) == 261, "v17 feeds_enabled offset drift.");
static_assert(offsetof(Settings, gnss_notify) == 264, "v17 gnss_notify offset drift.");
static_assert(offsetof(Settings, log_level) == 266, "v17 log_level offset drift.");
static_assert(offsetof(Settings, baud_rates) == 276, "v17 baud_rates offset drift.");
static_assert(offsetof(Settings, subg_mode) == 291, "v17 subg_mode offset drift.");
static_assert(offsetof(Settings, remote_id_tx_enabled) == 294, "v17 remote_id_tx_enabled offset drift.");
static_assert(offsetof(Settings, feed_uris) == 340, "v17 feed_uris offset drift.");
static_assert(offsetof(Settings, feed_protocols) == 1010, "v17 feed_protocols offset drift.");
static_assert(offsetof(Settings, feed_receiver_ids) == 1030, "v17 feed_receiver_ids offset drift.");
static_assert(offsetof(Settings, mavlink_system_id) == 1110, "v17 mavlink_system_id offset drift.");
static_assert(offsetof(Settings, rx_position) == 1112, "v17 rx_position offset drift.");
static_assert(offsetof(Settings, ip_mode) == 1141, "v17 ip_mode offset drift.");
static_assert(offsetof(Settings, wifi_sta_extra_networks) == 1142, "v17 wifi_sta_extra_networks offset drift.");

}  // namespace settings_v17

namespace settings_v18 {

static constexpr uint16_t kHostnameMaxLen = 32;
static constexpr uint16_t kWiFiSSIDMaxLen = 31;
static constexpr uint16_t kWiFiPasswordMaxLen = 63;
static constexpr uint16_t kNumSerialInterfaces = 3;
static constexpr uint16_t kMaxNumFeeds = 10;
static constexpr uint16_t kFeedURIMaxNumChars = 63;
static constexpr uint16_t kFeedReceiverIDNumBytes = 8;
static constexpr uint16_t kRemoteIDIDMaxLen = 20;
static constexpr uint16_t kWiFiSTANumExtraNetworks = 2;

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

struct WiFiNetwork {
    char ssid[kWiFiSSIDMaxLen + 2];
    char password[kWiFiPasswordMaxLen + 2];
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

// Byte-identical copy of the v18 Settings layout. Enum members are stored as their underlying integer types.
struct alignas(4) Settings {
    uint32_t settings_version;
    CoreNetworkSettings core_network_settings;
    bool r1090_rx_enabled;
    int32_t tl_offset_mv;
    bool r1090_bias_tee_enabled;
    uint32_t watchdog_timeout_sec;
    bool led_enabled;
    bool feeds_enabled;  // Added in v15.
    bool gnss_enabled;
    uint8_t gnss_receiver_type;
    bool gnss_notify;
    uint16_t log_level;
    uint16_t reporting_protocols[kNumSerialInterfaces];
    uint32_t baud_rates[kNumSerialInterfaces];
    int8_t subg_enabled;
    bool subg_rx_enabled;
    bool subg_bias_tee_enabled;
    uint8_t subg_mode;
    bool remote_id_rx_enabled;
    uint8_t remote_id_transports;
    bool remote_id_tx_enabled;
    uint8_t remote_id_tx_transports;
    uint8_t remote_id_tx_uas_id_type;
    uint8_t remote_id_tx_ua_type;
    char remote_id_tx_uas_id[kRemoteIDIDMaxLen + 1];
    char remote_id_tx_operator_id[kRemoteIDIDMaxLen + 1];
    char feed_uris[kMaxNumFeeds][kFeedURIMaxNumChars + 1];
    uint16_t feed_ports[kMaxNumFeeds];
    bool feed_is_active[kMaxNumFeeds];
    uint16_t feed_protocols[kMaxNumFeeds];
    uint8_t feed_receiver_ids[kMaxNumFeeds][kFeedReceiverIDNumBytes];
    uint8_t mavlink_system_id;
    uint8_t mavlink_component_id;
    RxPosition rx_position;
    uint8_t ip_mode;
    WiFiNetwork wifi_sta_extra_networks[kWiFiSTANumExtraNetworks];  // Added in v17.
    bool auto_update_enabled;                                       // Added in v18 (tail padding of v17).
};

// Lock the v18 byte layout (the live layout lock in settings.hh at kSettingsVersion == 18).
static_assert(sizeof(CoreNetworkSettings) == 240, "v18 CoreNetworkSettings must be 240 bytes.");
static_assert(sizeof(RxPosition) == 29, "v18 RxPosition must be 29 bytes.");
static_assert(sizeof(Settings) == 1340, "v18 Settings must be 1340 bytes.");
static_assert(offsetof(Settings, core_network_settings) == 4, "v18 CoreNetworkSettings offset drift.");
static_assert(offsetof(Settings, led_enabled) == 260, "v18 led_enabled offset drift.");
static_assert(offsetof(Settings, feeds_enabled) == 261, "v18 feeds_enabled offset drift.");
static_assert(offsetof(Settings, gnss_notify) == 264, "v18 gnss_notify offset drift.");
static_assert(offsetof(Settings, log_level) == 266, "v18 log_level offset drift.");
static_assert(offsetof(Settings, baud_rates) == 276, "v18 baud_rates offset drift.");
static_assert(offsetof(Settings, subg_mode) == 291, "v18 subg_mode offset drift.");
static_assert(offsetof(Settings, remote_id_tx_enabled) == 294, "v18 remote_id_tx_enabled offset drift.");
static_assert(offsetof(Settings, feed_uris) == 340, "v18 feed_uris offset drift.");
static_assert(offsetof(Settings, feed_protocols) == 1010, "v18 feed_protocols offset drift.");
static_assert(offsetof(Settings, feed_receiver_ids) == 1030, "v18 feed_receiver_ids offset drift.");
static_assert(offsetof(Settings, mavlink_system_id) == 1110, "v18 mavlink_system_id offset drift.");
static_assert(offsetof(Settings, rx_position) == 1112, "v18 rx_position offset drift.");
static_assert(offsetof(Settings, ip_mode) == 1141, "v18 ip_mode offset drift.");
static_assert(offsetof(Settings, wifi_sta_extra_networks) == 1142, "v18 wifi_sta_extra_networks offset drift.");
static_assert(offsetof(Settings, auto_update_enabled) == 1338, "v18 auto_update_enabled offset drift.");

}  // namespace settings_v18

/**
 * v19 layout: v18 plus the distance and altitude filters, feed transports, the MQTT broker, ASTERIX SAC/SIC, the NTP
 * server and swarm reporting. v20 put `ble_output_mode` in its tail padding.
 */
namespace settings_v19 {

static constexpr uint16_t kHostnameMaxLen = 32;
static constexpr uint16_t kWiFiSSIDMaxLen = 31;
static constexpr uint16_t kWiFiPasswordMaxLen = 63;
static constexpr uint16_t kNumSerialInterfaces = 3;
static constexpr uint16_t kMaxNumFeeds = 10;
static constexpr uint16_t kFeedURIMaxNumChars = 63;
static constexpr uint16_t kFeedReceiverIDNumBytes = 8;
static constexpr uint16_t kRemoteIDIDMaxLen = 20;
static constexpr uint16_t kWiFiSTANumExtraNetworks = 2;
static constexpr uint16_t kMQTTUsernameMaxLen = 31;
static constexpr uint16_t kMQTTPasswordMaxLen = 63;
static constexpr uint16_t kMQTTTopicPrefixMaxLen = 31;
static constexpr uint16_t kMQTTWebSocketPathMaxLen = 31;
static constexpr uint16_t kNTPServerMaxLen = 63;
static constexpr uint16_t kSwarmServerMaxLen = 63;
static constexpr uint16_t kSwarmTokenMaxLen = 63;

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

struct WiFiNetwork {
    char ssid[kWiFiSSIDMaxLen + 2];
    char password[kWiFiPasswordMaxLen + 2];
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

struct MQTTBroker {
    char host[kFeedURIMaxNumChars + 1];
    uint16_t port;
    bool tls;
    uint8_t qos;
    uint8_t transport;  // MQTTTransport underlying type.
    char username[kMQTTUsernameMaxLen + 1];
    char password[kMQTTPasswordMaxLen + 1];
    char topic_prefix[kMQTTTopicPrefixMaxLen + 1];
    char websocket_path[kMQTTWebSocketPathMaxLen + 1];
};

// Byte-identical copy of the v19 Settings layout. Enum members are stored as their underlying integer types.
struct alignas(4) Settings {
    uint32_t settings_version;
    CoreNetworkSettings core_network_settings;
    bool r1090_rx_enabled;
    int32_t tl_offset_mv;
    bool r1090_bias_tee_enabled;
    uint32_t watchdog_timeout_sec;
    bool led_enabled;
    bool feeds_enabled;  // Added in v15.
    bool gnss_enabled;
    uint8_t gnss_receiver_type;
    bool gnss_notify;
    uint16_t log_level;
    uint16_t reporting_protocols[kNumSerialInterfaces];
    uint32_t baud_rates[kNumSerialInterfaces];
    int8_t subg_enabled;
    bool subg_rx_enabled;
    bool subg_bias_tee_enabled;
    uint8_t subg_mode;
    bool remote_id_rx_enabled;
    uint8_t remote_id_transports;
    bool remote_id_tx_enabled;
    uint8_t remote_id_tx_transports;
    uint8_t remote_id_tx_uas_id_type;
    uint8_t remote_id_tx_ua_type;
    char remote_id_tx_uas_id[kRemoteIDIDMaxLen + 1];
    char remote_id_tx_operator_id[kRemoteIDIDMaxLen + 1];
    char feed_uris[kMaxNumFeeds][kFeedURIMaxNumChars + 1];
    uint16_t feed_ports[kMaxNumFeeds];
    bool feed_is_active[kMaxNumFeeds];
    uint16_t feed_protocols[kMaxNumFeeds];
    uint8_t feed_receiver_ids[kMaxNumFeeds][kFeedReceiverIDNumBytes];
    uint8_t mavlink_system_id;
    uint8_t mavlink_component_id;
    RxPosition rx_position;
    uint8_t ip_mode;
    WiFiNetwork wifi_sta_extra_networks[kWiFiSTANumExtraNetworks];  // Added in v17.
    bool auto_update_enabled;                                       // Added in v18 (tail padding of v17).
    bool filter_distance_enabled;
    uint32_t filter_distance_deci_nm;
    uint8_t feed_transports[kMaxNumFeeds];  // FeedTransport underlying type.
    MQTTBroker mqtt_broker;
    uint8_t asterix_sac;
    uint8_t asterix_sic;
    char ntp_server[kNTPServerMaxLen + 1];
    uint8_t filter_altitude_mode;
    int32_t filter_altitude_a_ft;
    int32_t filter_altitude_b_ft;
    bool swarm_enabled;
    char swarm_server[kSwarmServerMaxLen + 1];
    char swarm_token[kSwarmTokenMaxLen + 1];
};

// Lock the v19 byte layout (the live layout lock in settings.hh at kSettingsVersion == 18).
static_assert(sizeof(CoreNetworkSettings) == 240, "v19 CoreNetworkSettings must be 240 bytes.");
static_assert(sizeof(RxPosition) == 29, "v19 RxPosition must be 29 bytes.");
static_assert(sizeof(MQTTBroker) == 230, "v19 MQTTBroker must be 230 bytes.");
static_assert(sizeof(Settings) == 1792, "v19 Settings must be 1792 bytes.");
static_assert(offsetof(Settings, core_network_settings) == 4, "v19 CoreNetworkSettings offset drift.");
static_assert(offsetof(Settings, led_enabled) == 260, "v19 led_enabled offset drift.");
static_assert(offsetof(Settings, feeds_enabled) == 261, "v19 feeds_enabled offset drift.");
static_assert(offsetof(Settings, gnss_notify) == 264, "v19 gnss_notify offset drift.");
static_assert(offsetof(Settings, log_level) == 266, "v19 log_level offset drift.");
static_assert(offsetof(Settings, baud_rates) == 276, "v19 baud_rates offset drift.");
static_assert(offsetof(Settings, subg_mode) == 291, "v19 subg_mode offset drift.");
static_assert(offsetof(Settings, remote_id_tx_enabled) == 294, "v19 remote_id_tx_enabled offset drift.");
static_assert(offsetof(Settings, feed_uris) == 340, "v19 feed_uris offset drift.");
static_assert(offsetof(Settings, feed_protocols) == 1010, "v19 feed_protocols offset drift.");
static_assert(offsetof(Settings, feed_receiver_ids) == 1030, "v19 feed_receiver_ids offset drift.");
static_assert(offsetof(Settings, mavlink_system_id) == 1110, "v19 mavlink_system_id offset drift.");
static_assert(offsetof(Settings, rx_position) == 1112, "v19 rx_position offset drift.");
static_assert(offsetof(Settings, ip_mode) == 1141, "v19 ip_mode offset drift.");
static_assert(offsetof(Settings, wifi_sta_extra_networks) == 1142, "v19 wifi_sta_extra_networks offset drift.");
static_assert(offsetof(Settings, auto_update_enabled) == 1338, "v19 auto_update_enabled offset drift.");
static_assert(offsetof(Settings, filter_distance_enabled) == 1339, "v19 filter_distance_enabled offset drift.");
static_assert(offsetof(Settings, filter_distance_deci_nm) == 1340, "v19 filter_distance_deci_nm offset drift.");
static_assert(offsetof(Settings, feed_transports) == 1344, "v19 feed_transports offset drift.");
static_assert(offsetof(Settings, mqtt_broker) == 1354, "v19 mqtt_broker offset drift.");
static_assert(offsetof(Settings, asterix_sac) == 1584, "v19 asterix_sac offset drift.");
static_assert(offsetof(Settings, asterix_sic) == 1585, "v19 asterix_sic offset drift.");
static_assert(offsetof(Settings, ntp_server) == 1586, "v19 ntp_server offset drift.");
static_assert(offsetof(Settings, filter_altitude_mode) == 1650, "v19 filter_altitude_mode offset drift.");
static_assert(offsetof(Settings, filter_altitude_a_ft) == 1652, "v19 filter_altitude_a_ft offset drift.");
static_assert(offsetof(Settings, filter_altitude_b_ft) == 1656, "v19 filter_altitude_b_ft offset drift.");
static_assert(offsetof(Settings, swarm_enabled) == 1660, "v19 swarm_enabled offset drift.");
static_assert(offsetof(Settings, swarm_server) == 1661, "v19 swarm_server offset drift.");
static_assert(offsetof(Settings, swarm_token) == 1725, "v19 swarm_token offset drift.");

}  // namespace settings_v19
