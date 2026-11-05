#pragma once

#include "config_hash.hh"
#include "settings.hh"

// Settings hashes for AT+CONFIG_HASH. Fields are added one by one, so struct padding, Bytes after a string's
// terminator and the stored CoreNetworkSettings CRC don't count.
namespace SettingsHash {

enum Scope : uint8_t {
    kScopeFleet = 0,  // Leaves out what differs between units with the same configuration.
    kScopeFull        // Every setting.
};

// Left out of kScopeFleet: hostname, WiFi access point SSID and channel (defaults come from the part code and a random
// number), feed receiver IDs, the Remote ID UAS ID, the receiver position values, the ASTERIX SIC, the swarm token and
// static IPv4 and IPv6 addresses.
inline uint32_t Compute(const SettingsManager::Settings& settings, Scope scope) {
    static_assert(kSettingsVersion == 20 && sizeof(SettingsManager::Settings) == 1964,
                  "Settings changed: add the new fields below, then update this check.");
    typedef SettingsManager::Settings Settings;
    const bool full = scope == kScopeFull;
    ConfigHasher hasher;

    hasher.AddU32(settings.settings_version);

    const Settings::CoreNetworkSettings& net = settings.core_network_settings;
    hasher.AddBool(net.esp32_enabled);
    if (full) hasher.AddString(net.hostname, sizeof(net.hostname));
    hasher.AddBool(net.wifi_ap_enabled);
    if (full) hasher.AddU8(net.wifi_ap_channel);
    if (full) hasher.AddString(net.wifi_ap_ssid, sizeof(net.wifi_ap_ssid));
    hasher.AddString(net.wifi_ap_password, sizeof(net.wifi_ap_password));
    hasher.AddBool(net.wifi_sta_enabled);
    hasher.AddString(net.wifi_sta_ssid, sizeof(net.wifi_sta_ssid));
    hasher.AddString(net.wifi_sta_password, sizeof(net.wifi_sta_password));
    hasher.AddBool(net.ethernet_enabled);

    hasher.AddBool(settings.r1090_rx_enabled);
    hasher.AddI32(settings.tl_offset_mv);
    hasher.AddBool(settings.r1090_bias_tee_enabled);
    hasher.AddU32(settings.watchdog_timeout_sec);
    hasher.AddBool(settings.led_enabled);
    hasher.AddBool(settings.feeds_enabled);
    hasher.AddBool(settings.gnss_enabled);
    hasher.AddU8(settings.gnss_receiver_type);
    hasher.AddBool(settings.gnss_notify);

    hasher.AddU16(settings.log_level);
    for (uint16_t i = 0; i < SettingsManager::SerialInterface::kNumSerialInterfaces; i++) {
        hasher.AddU16(settings.reporting_protocols[i]);
    }
    for (uint16_t i = 0; i < SettingsManager::SerialInterface::kNumSerialInterfaces; i++) {
        hasher.AddU32(settings.baud_rates[i]);
    }

    hasher.AddU8(static_cast<uint8_t>(settings.subg_enabled));
    hasher.AddBool(settings.subg_rx_enabled);
    hasher.AddBool(settings.subg_bias_tee_enabled);
    hasher.AddU8(settings.subg_mode);

    hasher.AddBool(settings.remote_id_rx_enabled);
    hasher.AddU8(settings.remote_id_transports);
    hasher.AddBool(settings.remote_id_tx_enabled);
    hasher.AddU8(settings.remote_id_tx_transports);
    hasher.AddU8(settings.remote_id_tx_uas_id_type);
    hasher.AddU8(settings.remote_id_tx_ua_type);
    if (full) hasher.AddString(settings.remote_id_tx_uas_id, sizeof(settings.remote_id_tx_uas_id));
    hasher.AddString(settings.remote_id_tx_operator_id, sizeof(settings.remote_id_tx_operator_id));

    for (uint16_t i = 0; i < Settings::kMaxNumFeeds; i++) {
        hasher.AddString(settings.feed_uris[i], sizeof(settings.feed_uris[i]));
        hasher.AddU16(settings.feed_ports[i]);
        hasher.AddBool(settings.feed_is_active[i]);
        hasher.AddU16(settings.feed_protocols[i]);
        if (full) hasher.AddBytes(settings.feed_receiver_ids[i], Settings::kFeedReceiverIDNumBytes);
    }

    hasher.AddU8(settings.mavlink_system_id);
    hasher.AddU8(settings.mavlink_component_id);

    hasher.AddRxPosition(settings.rx_position, full);

    hasher.AddU8(settings.ip_mode);
    for (uint16_t i = 0; i < Settings::kWiFiSTANumExtraNetworks; i++) {
        hasher.AddString(settings.wifi_sta_extra_networks[i].ssid, sizeof(settings.wifi_sta_extra_networks[i].ssid));
        hasher.AddString(settings.wifi_sta_extra_networks[i].password,
                         sizeof(settings.wifi_sta_extra_networks[i].password));
    }
    hasher.AddBool(settings.auto_update_enabled);
    hasher.AddBool(settings.filter_distance_enabled);
    hasher.AddU32(settings.filter_distance_deci_nm);
    for (uint16_t i = 0; i < Settings::kMaxNumFeeds; i++) {
        hasher.AddU8(settings.feed_transports[i]);
    }
    const Settings::MQTTBroker& mqtt = settings.mqtt_broker;
    hasher.AddString(mqtt.host, sizeof(mqtt.host));
    hasher.AddU16(mqtt.port);
    hasher.AddBool(mqtt.tls);
    hasher.AddU8(mqtt.qos);
    hasher.AddU8(mqtt.transport);
    hasher.AddString(mqtt.username, sizeof(mqtt.username));
    hasher.AddString(mqtt.password, sizeof(mqtt.password));
    hasher.AddString(mqtt.topic_prefix, sizeof(mqtt.topic_prefix));
    hasher.AddString(mqtt.websocket_path, sizeof(mqtt.websocket_path));
    hasher.AddU8(settings.asterix_sac);
    if (full) hasher.AddU8(settings.asterix_sic);
    hasher.AddString(settings.ntp_server, sizeof(settings.ntp_server));
    hasher.AddU8(settings.filter_altitude_mode);
    hasher.AddI32(settings.filter_altitude_a_ft);
    hasher.AddI32(settings.filter_altitude_b_ft);
    hasher.AddBool(settings.swarm_enabled);
    hasher.AddString(settings.swarm_server, sizeof(settings.swarm_server));
    if (full) hasher.AddString(settings.swarm_token, sizeof(settings.swarm_token));
    const IPv4Config* ipv4_configs[] = {&settings.ethernet_ipv4, &settings.wifi_sta_ipv4};
    for (const IPv4Config* ipv4 : ipv4_configs) {
        hasher.AddBool(ipv4->is_static);
        if (!full) continue;
        hasher.AddBytes(ipv4->ip, sizeof(ipv4->ip));
        hasher.AddBytes(ipv4->netmask, sizeof(ipv4->netmask));
        hasher.AddBytes(ipv4->gateway, sizeof(ipv4->gateway));
        hasher.AddBytes(&ipv4->dns[0][0], sizeof(ipv4->dns));
    }
    const IPv6Config* ipv6_configs[] = {&settings.ethernet_ipv6, &settings.wifi_sta_ipv6};
    for (const IPv6Config* ipv6 : ipv6_configs) {
        hasher.AddBool(ipv6->is_static);
        if (!full) continue;
        hasher.AddU8(ipv6->prefix_len);
        hasher.AddBytes(ipv6->ip, sizeof(ipv6->ip));
        hasher.AddBytes(ipv6->gateway, sizeof(ipv6->gateway));
        hasher.AddBytes(&ipv6->dns[0][0], sizeof(ipv6->dns));
    }

    return hasher.Value();
}

}  // namespace SettingsHash
