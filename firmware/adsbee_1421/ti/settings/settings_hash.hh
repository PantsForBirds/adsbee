#pragma once

#include "config_hash.hh"
#include "settings.hh"

// Settings hashes for AT+CONFIG_HASH. Fields are added one by one; the magic word and the stored CRC don't count.
namespace SettingsHash {

enum Scope : uint8_t {
    kScopeFleet = 0,  // Leaves out what differs between units with the same configuration.
    kScopeFull        // Every setting.
};

// Left out of kScopeFleet: the receiver position values and the ASTERIX SIC.
inline uint32_t Compute(const SettingsManager::Settings& settings, Scope scope) {
    static_assert(kSettingsVersion == 7 && sizeof(SettingsManager::Settings) == 79,
                  "Settings changed: add the new fields below, then update this check.");
    ConfigHasher hasher;

    hasher.AddU32(settings.settings_version);

    hasher.AddBool(settings.r1090_rx_enabled);
    hasher.AddU32(settings.watchdog_timeout_sec);
    hasher.AddU8(settings.r1090_preamble_mode);
    hasher.AddU8(settings.r1090_gain);
    hasher.AddU8(settings.r1090_rx_boost);
    hasher.AddBool(settings.lr2021_enabled);

    hasher.AddU16(settings.log_level);
    for (uint16_t i = 0; i < SettingsManager::SerialInterface::kNumSerialInterfaces; i++) {
        hasher.AddU16(settings.reporting_protocols[i]);
    }
    for (uint16_t i = 0; i < SettingsManager::SerialInterface::kNumSerialInterfaces; i++) {
        hasher.AddU32(settings.baud_rates[i]);
    }

    hasher.AddBool(settings.subg_rx_enabled);
    hasher.AddU8(settings.subg_mode);

    hasher.AddU8(settings.mavlink_system_id);
    hasher.AddU8(settings.mavlink_component_id);

    SettingsManager::RxPosition rx_position = settings.rx_position;  // Copy: Settings is packed.
    hasher.AddRxPosition(rx_position, scope == kScopeFull);

    hasher.AddBool(settings.filter_distance_enabled);
    hasher.AddU32(settings.filter_distance_deci_nm);
    hasher.AddU8(settings.asterix_sac);
    if (scope == kScopeFull) hasher.AddU8(settings.asterix_sic);
    hasher.AddU8(settings.filter_altitude_mode);
    hasher.AddI32(settings.filter_altitude_a_ft);
    hasher.AddI32(settings.filter_altitude_b_ft);
    hasher.AddBool(settings.led_enabled);

    return hasher.Value();
}

}  // namespace SettingsHash
