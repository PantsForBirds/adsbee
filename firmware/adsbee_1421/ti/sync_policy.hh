#pragma once

#include <stdint.h>

#include "settings.hh"

// What a receiver config may do with the shared LR2021 bus.
enum class LR2021BusAction : uint8_t {
    kRelease,    // Every bus pin an input: the host owns the bus (SYNC high, also at boot) or AT+LR_ENABLE=0.
    kHoldReset,  // NSS driven high, NRESET driven low: 1090 RX disabled.
    kConfigure,  // Reset and configure the LR2021.
};

inline LR2021BusAction LR2021BusActionFor(bool sync_high, bool lr2021_enabled, bool rx_1090_enabled) {
    if (sync_high || !lr2021_enabled) {
        return LR2021BusAction::kRelease;
    }
    return rx_1090_enabled ? LR2021BusAction::kConfigure : LR2021BusAction::kHoldReset;
}

// An m1421 at rev D or later has an external SYNC pull-down (R5), so the internal one stays off.
inline bool BoardHasSyncPullDown(const SettingsManager::DeviceInfo& device_info) {
    return device_info.IsPartAtLeastRev(SettingsManager::DeviceInfo::kPNADSBeem1421, 'D');
}
