#pragma once

#include <cstdint>
#include <cstring>

#include "range_map.hh"
#include "remote_id_guard.hh"
#include "settings.hh"

// What the receivers run right now, sampled once a second for each protocol's listen time.
struct SwarmListenState {
    bool r1090_rx_enabled = false;
    SettingsManager::EnableState subg_enabled = SettingsManager::kEnableStateDisabled;
    bool subg_rx_enabled = false;
    SettingsManager::SubGHzRadioMode subg_mode = SettingsManager::kSubGHzRadioModeUATRx;
    uint16_t remote_id_status = 0;  // RemoteIDGuard::Status bits: live state, settings only say what was requested.
};

// Height bands: by aircraft altitude, or by height above the receiver (drones).
enum class SwarmBandBasis : uint8_t { kAltitude = 0, kHeightAboveRx, kNumBases };

// Frame counts that swarm protocol v1 sends as named "stats" keys. Protocols added later report "frames" instead.
struct SwarmV1Stats {
    uint32_t modes_valid = 0;
    uint32_t es_valid = 0;
    uint32_t uat_adsb = 0;
    uint32_t uat_uplink = 0;
    uint16_t aircraft_max = 0;
};

struct SwarmProtocol {
    struct V1Stat {
        const char* key;
        uint32_t SwarmV1Stats::*field;
    };

    const char* name;  // Wire key.
    uint32_t unit_m;
    SwarmBandBasis basis;
    uint8_t aircraft_type;    // Aircraft::AircraftType that feeds its map.
    bool counts_as_aircraft;  // Goes into aircraft_max and aircraft_unique.
    bool counts_frames;       // The receiver reports frame counts.
    V1Stat v1_stats[2];       // Its frame counts as v1 "stats" keys (second: extended squitter, uplink); none: "frames".
    bool (*listening)(const SwarmListenState& state);
};

inline bool SwarmListenModeS(const SwarmListenState& s) { return s.r1090_rx_enabled; }
inline bool SwarmListenUAT(const SwarmListenState& s) {
    return s.subg_enabled == SettingsManager::kEnableStateEnabled && s.subg_rx_enabled &&
           s.subg_mode == SettingsManager::kSubGHzRadioModeUATRx;
}
inline bool SwarmListenRemoteID(const SwarmListenState& s) {
    return s.remote_id_status & (RemoteIDGuard::kStatusBLEActive | RemoteIDGuard::kStatusBLECodedPHYActive |
                                 RemoteIDGuard::kStatusWiFiSnifferActive);
}

// Every swarm path loops over this table. v1 servers know the first kNumSwarmV1Protocols entries; append new ones.
inline constexpr SwarmProtocol kSwarmProtocols[] = {
    {"mode_s", RangeMap::kUnitMeters, SwarmBandBasis::kAltitude, 0, true, true,
     {{"modes_valid", &SwarmV1Stats::modes_valid}, {"es_valid", &SwarmV1Stats::es_valid}}, SwarmListenModeS},
    {"uat", RangeMap::kUnitMeters, SwarmBandBasis::kAltitude, 1, true, true,
     {{"uat_adsb", &SwarmV1Stats::uat_adsb}, {"uat_uplink", &SwarmV1Stats::uat_uplink}}, SwarmListenUAT},
    {"remote_id", RangeMap::kDroneUnitMeters, SwarmBandBasis::kHeightAboveRx, 2, false, false,
     {{nullptr, nullptr}, {nullptr, nullptr}}, SwarmListenRemoteID},
};
inline constexpr uint8_t kNumSwarmProtocols = sizeof(kSwarmProtocols) / sizeof(kSwarmProtocols[0]);
inline constexpr uint8_t kNumSwarmV1Protocols = 3;
inline constexpr uint32_t kSwarmV1ProtocolMask = (1u << kNumSwarmV1Protocols) - 1;
inline constexpr uint8_t kNoSwarmProtocol = 0xFF;
static_assert(kNumSwarmProtocols <= 32, "Protocol masks are 32 bits.");

constexpr bool SwarmStrEqual(const char* a, const char* b) {
    while (*a && *a == *b) a++, b++;
    return *a == *b;
}

// Index in kSwarmProtocols, or kNoSwarmProtocol.
constexpr uint8_t SwarmProtocolIndex(const char* name) {
    for (uint8_t p = 0; p < kNumSwarmProtocols; p++) {
        if (SwarmStrEqual(kSwarmProtocols[p].name, name)) return p;
    }
    return kNoSwarmProtocol;
}

constexpr uint8_t SwarmProtocolOfAircraftType(uint8_t aircraft_type) {
    for (uint8_t p = 0; p < kNumSwarmProtocols; p++) {
        if (kSwarmProtocols[p].aircraft_type == aircraft_type) return p;
    }
    return kNoSwarmProtocol;
}
