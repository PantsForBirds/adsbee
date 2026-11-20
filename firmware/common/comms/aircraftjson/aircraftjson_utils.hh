#pragma once
#include "aircraft_dictionary.hh"
#include "stdio.h"

const uint16_t kAircraftJSONMessageStrMaxLen = 512;

// Emergency priority status strings matching readsb nomenclature.
static const char* const kUATEmergencyStrings[] = {"none",     "general",  "lifeguard", "minfuel",
                                                    "nordo",    "unlawful", "downed",    "reserved"};
static const uint8_t kUATEmergencyStringsCount =
    sizeof(kUATEmergencyStrings) / sizeof(kUATEmergencyStrings[0]);

/**
 * Converts an EmitterCategory enum value to the readsb-style "A0"–"D7" category string.
 * Category 0 is "A0" (no category info); negative values are invalid.
 * @param[out] buf Output buffer (must hold at least 3 bytes: letter + digit + NUL).
 * @param[in] buf_len Length of output buffer.
 * @param[in] category EmitterCategory enum value.
 * @retval True if a valid category string was written, false if category is invalid.
 */
static inline bool EmitterCategoryToStr(char* buf, size_t buf_len, ADSBTypes::EmitterCategory category) {
    if (category < 0) return false;
    snprintf(buf, buf_len, "%c%d", (char)('A' + (category / 8)), category % 8);
    return true;
}

/**
 * Returns the readsb "type" string of a Mode S address type.
 */
static inline const char* ModeSAddressTypeToStr(ModeSAircraft::AddressType type) {
    switch (type) {
        case ModeSAircraft::kAddressTypeADSBICAO:
            return "adsb_icao";
        case ModeSAircraft::kAddressTypeADSBICAONonTransponder:
            return "adsb_icao_nt";
        case ModeSAircraft::kAddressTypeADSRICAO:
            return "adsr_icao";
        case ModeSAircraft::kAddressTypeTISBICAO:
            return "tisb_icao";
        case ModeSAircraft::kAddressTypeADSBOther:
            return "adsb_other";
        case ModeSAircraft::kAddressTypeADSROther:
            return "adsr_other";
        case ModeSAircraft::kAddressTypeTISBTrackFile:
            return "tisb_trackfile";
        case ModeSAircraft::kAddressTypeTISBOther:
            return "tisb_other";
        default:
            return "mode_s";
    }
}

/**
 * Appends readsb's "r_dst" (distance from the receiver, nautical miles) and "r_dir" (bearing from the receiver,
 * degrees) when the aircraft's distance is known.
 * @retval New length of buf, clamped to max.
 */
inline int16_t AppendAircraftJSONDistanceAndBearing(char buf[], int16_t n, int16_t max, const Aircraft& aircraft) {
    if (aircraft.distance_m == Aircraft::kDistanceUnknown) {
        return n;
    }
    uint32_t distance_milli_nm = static_cast<uint32_t>((static_cast<uint64_t>(aircraft.distance_m) * 1000 + 926) / 1852);
    uint32_t bearing_decideg = ((static_cast<uint32_t>(aircraft.bearing_bam16) * 3600 + 32768) >> 16) % 3600;
    n += snprintf(buf + n, max - n, ",\"r_dst\":%lu.%03lu,\"r_dir\":%lu.%lu", (unsigned long)(distance_milli_nm / 1000),
                  (unsigned long)(distance_milli_nm % 1000), (unsigned long)(bearing_decideg / 10),
                  (unsigned long)(bearing_decideg % 10));
    return n < max ? n : max;
}

/**
 * Serializes a ModeSAircraft to a single-line readsb-compatible JSON object.
 * @param[out] buf Character array of at least kAircraftJSONMessageStrMaxLen bytes.
 * @param[in] aircraft Aircraft to serialize.
 * @param[in] timestamp_ms Current time (get_time_since_boot_ms()). If nonzero, "seen_pos" is included.
 * @retval Number of characters written (excluding the NUL terminator), or negative on error.
 */
inline int16_t WriteAircraftJSONModeSAircraftStr(char buf[], const ModeSAircraft& aircraft, uint32_t timestamp_ms = 0) {
    int16_t n = 0;
    const int16_t max = kAircraftJSONMessageStrMaxLen;

    // hex (always). Non-ICAO addresses get a '~' prefix per readsb convention.
    ModeSAircraft::AddressType address_type = aircraft.GetAddressType();
    n += snprintf(buf + n, max - n,
#if defined(ON_PICO) || defined(ON_HOST)
                  "{\"hex\":\"%s%06x\"",
#else
                  "{\"hex\":\"%s%06lx\"",
#endif
                  address_type >= ModeSAircraft::kFirstNonICAOAddressType ? "~" : "", aircraft.icao_address);
    n = n < max ? n : max;

    // type (readsb address types)
    const char* type_str = ModeSAddressTypeToStr(address_type);
    n += snprintf(buf + n, max - n, ",\"type\":\"%s\",\"link\":\"1090\"", type_str);
    n = n < max ? n : max;

    // flight (only if set — default is "?")
    if (aircraft.callsign[0] != '?' && aircraft.callsign[0] != '\0') {
        n += snprintf(buf + n, max - n, ",\"flight\":\"%s\"", aircraft.callsign);
        n = n < max ? n : max;
    }

    // squawk
    if (aircraft.squawk != ADSBTypes::kSquawkCodeNotYetReceived) {
        n += snprintf(buf + n, max - n, ",\"squawk\":\"%04u\"", static_cast<unsigned>(aircraft.squawk % 10000));
        n = n < max ? n : max;
    }

    // category ("A0"–"D7")
    if (aircraft.emitter_category != ADSBTypes::kEmitterCategoryInvalid) {
        char cat_str[4];
        if (EmitterCategoryToStr(cat_str, sizeof(cat_str), aircraft.emitter_category)) {
            n += snprintf(buf + n, max - n, ",\"category\":\"%s\"", cat_str);
            n = n < max ? n : max;
        }
    }

    // lat / lon
    if (aircraft.HasBitFlag(ModeSAircraft::kBitFlagPositionValid)) {
        n += snprintf(buf + n, max - n, ",\"lat\":%.5f,\"lon\":%.5f", aircraft.latitude_deg,
                      aircraft.longitude_deg);
        n = n < max ? n : max;
    }
    n = AppendAircraftJSONDistanceAndBearing(buf, n, max, aircraft);

    // alt_baro
    if (aircraft.HasBitFlag(ModeSAircraft::kBitFlagBaroAltitudeValid)) {
        n += snprintf(buf + n, max - n,
#if defined(ON_PICO) || defined(ON_HOST)
                      ",\"alt_baro\":%d",
#else
                      ",\"alt_baro\":%ld",
#endif
                      aircraft.baro_altitude_ft);
        n = n < max ? n : max;
    }

    // alt_geom
    if (aircraft.HasBitFlag(ModeSAircraft::kBitFlagGNSSAltitudeValid)) {
        n += snprintf(buf + n, max - n,
#if defined(ON_PICO) || defined(ON_HOST)
                      ",\"alt_geom\":%d",
#else
                      ",\"alt_geom\":%ld",
#endif
                      aircraft.gnss_altitude_ft);
        n = n < max ? n : max;
    }

    // gs (ground speed)
    if (aircraft.HasBitFlag(ModeSAircraft::kBitFlagHorizontalSpeedValid)) {
        n += snprintf(buf + n, max - n,
#if defined(ON_PICO) || defined(ON_HOST)
                      ",\"gs\":%d",
#else
                      ",\"gs\":%ld",
#endif
                      aircraft.speed_kts);
        n = n < max ? n : max;
    }

    // track / mag_heading / true_heading
    if (aircraft.HasBitFlag(ModeSAircraft::kBitFlagDirectionValid)) {
        const char* dir_key;
        if (!aircraft.HasBitFlag(ModeSAircraft::kBitFlagDirectionIsHeading)) {
            dir_key = "track";
        } else if (aircraft.HasBitFlag(ModeSAircraft::kBitFlagHeadingUsesMagneticNorth)) {
            dir_key = "mag_heading";
        } else {
            dir_key = "true_heading";
        }
        n += snprintf(buf + n, max - n, ",\"%s\":%.1f", dir_key, aircraft.direction_deg);
        n = n < max ? n : max;
    }

    // baro_rate
    if (aircraft.HasBitFlag(ModeSAircraft::kBitFlagBaroVerticalRateValid)) {
        n += snprintf(buf + n, max - n,
#if defined(ON_PICO) || defined(ON_HOST)
                      ",\"baro_rate\":%d",
#else
                      ",\"baro_rate\":%ld",
#endif
                      aircraft.baro_vertical_rate_fpm);
        n = n < max ? n : max;
    }

    // geom_rate
    if (aircraft.HasBitFlag(ModeSAircraft::kBitFlagGNSSVerticalRateValid)) {
        n += snprintf(buf + n, max - n,
#if defined(ON_PICO) || defined(ON_HOST)
                      ",\"geom_rate\":%d",
#else
                      ",\"geom_rate\":%ld",
#endif
                      aircraft.gnss_vertical_rate_fpm);
        n = n < max ? n : max;
    }

    // Integrity / accuracy fields (always present)
    n += snprintf(buf + n, max - n,
                  ",\"nic\":%d,\"nic_baro\":%d,\"nac_p\":%d,\"nac_v\":%d"
                  ",\"sil\":%d,\"gva\":%d,\"sda\":%d,\"version\":%d",
                  aircraft.navigation_integrity_category, aircraft.navigation_integrity_category_baro,
                  aircraft.navigation_accuracy_category_position, aircraft.navigation_accuracy_category_velocity,
                  aircraft.surveillance_integrity_level, aircraft.geometric_vertical_accuracy,
                  aircraft.system_design_assurance, aircraft.adsb_version);
    n = n < max ? n : max;

    // rssi and message count (always present)
    // "messages" is the total since the aircraft was first seen (readsb semantics).
    n += snprintf(buf + n, max - n, ",\"rssi\":%d,\"messages\":%lu", aircraft.last_message_signal_strength_dbm,
                  (unsigned long)aircraft.num_frames_received);
    n = n < max ? n : max;

    // alert / spi (only when set)
    if (aircraft.HasBitFlag(ModeSAircraft::kBitFlagAlert)) {
        n += snprintf(buf + n, max - n, ",\"alert\":1");
        n = n < max ? n : max;
    }
    if (aircraft.HasBitFlag(ModeSAircraft::kBitFlagIdent)) {
        n += snprintf(buf + n, max - n, ",\"spi\":1");
        n = n < max ? n : max;
    }

    if (aircraft.IsOnGround()) {
        n += snprintf(buf + n, max - n, ",\"on_ground\":1");
        n = n < max ? n : max;
    }

    // seen_pos: seconds since the last position update (readsb semantics).
    if (timestamp_ms != 0 && aircraft.HasBitFlag(ModeSAircraft::kBitFlagPositionValid)) {
        uint32_t seen_pos_ds = (timestamp_ms - aircraft.last_position_update_ms) / 100;
        n += snprintf(buf + n, max - n, ",\"seen_pos\":%lu.%lu", (unsigned long)(seen_pos_ds / 10),
                      (unsigned long)(seen_pos_ds % 10));
        n = n < max ? n : max;
    }

    n += snprintf(buf + n, max - n, "}\n");

    if (n >= max) return -1;  // Buffer overrun.
    return n;
}

/**
 * Serializes a UATAircraft to a single-line readsb-compatible JSON object.
 * @param[out] buf Character array of at least kAircraftJSONMessageStrMaxLen bytes.
 * @param[in] aircraft Aircraft to serialize.
 * @param[in] timestamp_ms Current time (get_time_since_boot_ms()). If nonzero, "seen_pos" is included.
 * @retval Number of characters written (excluding the NUL terminator), or negative on error.
 */
inline int16_t WriteAircraftJSONUATAircraftStr(char buf[], const UATAircraft& aircraft, uint32_t timestamp_ms = 0) {
    int16_t n = 0;
    const int16_t max = kAircraftJSONMessageStrMaxLen;

    UATAircraft::AddressQualifier aq = aircraft.GetAddressQualifier();
    uint32_t icao_24bit = aircraft.icao_address & ~Aircraft::kAddressQualifierMask;

    // Non-ICAO addresses get a '~' prefix per readsb convention.
    bool is_non_icao = (aq == UATAircraft::kADSBTargetWithSelfAssignedTemporaryAddress ||
                        aq == UATAircraft::kTISBTargetWithTrackFileIdentifier ||
                        aq == UATAircraft::kADSRTargetWithNonICAOAddress);

    // hex (always)
    n += snprintf(buf + n, max - n,
#if defined(ON_PICO) || defined(ON_HOST)
                  "{\"hex\":\"%s%06x\"",
#else
                  "{\"hex\":\"%s%06lx\"",
#endif
                  is_non_icao ? "~" : "", icao_24bit);
    n = n < max ? n : max;

    // type (derived from address qualifier)
    const char* type_str;
    switch (aq) {
        case UATAircraft::kADSBTargetWithICAO24BitAddress:
            type_str = "adsb_icao";
            break;
        case UATAircraft::kADSBTargetWithSelfAssignedTemporaryAddress:
            type_str = "adsb_other";
            break;
        case UATAircraft::kTISBTargetWithICAO24BitAddress:
            type_str = "tisb_icao";
            break;
        case UATAircraft::kTISBTargetWithTrackFileIdentifier:
            type_str = "tisb_trackfile";
            break;
        case UATAircraft::kADSRTargetWithNonICAOAddress:
            type_str = "adsr_other";  // Was reported as a direct "adsb_icao" target.
            break;
        case UATAircraft::kSurfaceVehicle:
        case UATAircraft::kFixedADSBBeacon:
        default:
            type_str = "adsb_icao";
            break;
    }
    n += snprintf(buf + n, max - n, ",\"type\":\"%s\",\"link\":\"uat\"", type_str);
    n = n < max ? n : max;

    // flight
    if (aircraft.callsign[0] != '?' && aircraft.callsign[0] != '\0') {
        n += snprintf(buf + n, max - n, ",\"flight\":\"%s\"", aircraft.callsign);
        n = n < max ? n : max;
    }

    // squawk
    if (aircraft.squawk != ADSBTypes::kSquawkCodeNotYetReceived) {
        n += snprintf(buf + n, max - n, ",\"squawk\":\"%04u\"", static_cast<unsigned>(aircraft.squawk % 10000));
        n = n < max ? n : max;
    }

    // category
    if (aircraft.emitter_category != ADSBTypes::kEmitterCategoryInvalid) {
        char cat_str[4];
        if (EmitterCategoryToStr(cat_str, sizeof(cat_str), aircraft.emitter_category)) {
            n += snprintf(buf + n, max - n, ",\"category\":\"%s\"", cat_str);
            n = n < max ? n : max;
        }
    }

    // lat / lon
    if (aircraft.HasBitFlag(UATAircraft::kBitFlagPositionValid)) {
        n += snprintf(buf + n, max - n, ",\"lat\":%.5f,\"lon\":%.5f", aircraft.latitude_deg,
                      aircraft.longitude_deg);
        n = n < max ? n : max;
    }
    n = AppendAircraftJSONDistanceAndBearing(buf, n, max, aircraft);

    // alt_baro
    if (aircraft.HasBitFlag(UATAircraft::kBitFlagBaroAltitudeValid)) {
        n += snprintf(buf + n, max - n,
#if defined(ON_PICO) || defined(ON_HOST)
                      ",\"alt_baro\":%d",
#else
                      ",\"alt_baro\":%ld",
#endif
                      aircraft.baro_altitude_ft);
        n = n < max ? n : max;
    }

    // alt_geom
    if (aircraft.HasBitFlag(UATAircraft::kBitFlagGNSSAltitudeValid)) {
        n += snprintf(buf + n, max - n,
#if defined(ON_PICO) || defined(ON_HOST)
                      ",\"alt_geom\":%d",
#else
                      ",\"alt_geom\":%ld",
#endif
                      aircraft.gnss_altitude_ft);
        n = n < max ? n : max;
    }

    // gs
    if (aircraft.HasBitFlag(UATAircraft::kBitFlagHorizontalSpeedValid)) {
        n += snprintf(buf + n, max - n,
#if defined(ON_PICO) || defined(ON_HOST)
                      ",\"gs\":%d",
#else
                      ",\"gs\":%ld",
#endif
                      aircraft.speed_kts);
        n = n < max ? n : max;
    }

    // track / mag_heading / true_heading
    if (aircraft.HasBitFlag(UATAircraft::kBitFlagDirectionValid)) {
        const char* dir_key;
        if (!aircraft.HasBitFlag(UATAircraft::kBitFlagDirectionIsHeading)) {
            dir_key = "track";
        } else if (aircraft.HasBitFlag(UATAircraft::kBitFlagHeadingUsesMagneticNorth)) {
            dir_key = "mag_heading";
        } else {
            dir_key = "true_heading";
        }
        n += snprintf(buf + n, max - n, ",\"%s\":%.1f", dir_key, aircraft.direction_deg);
        n = n < max ? n : max;
    }

    // baro_rate
    if (aircraft.HasBitFlag(UATAircraft::kBitFlagBaroVerticalRateValid)) {
        n += snprintf(buf + n, max - n,
#if defined(ON_PICO) || defined(ON_HOST)
                      ",\"baro_rate\":%d",
#else
                      ",\"baro_rate\":%ld",
#endif
                      aircraft.baro_vertical_rate_fpm);
        n = n < max ? n : max;
    }

    // geom_rate
    if (aircraft.HasBitFlag(UATAircraft::kBitFlagGNSSVerticalRateValid)) {
        n += snprintf(buf + n, max - n,
#if defined(ON_PICO) || defined(ON_HOST)
                      ",\"geom_rate\":%d",
#else
                      ",\"geom_rate\":%ld",
#endif
                      aircraft.gnss_vertical_rate_fpm);
        n = n < max ? n : max;
    }

    // Integrity / accuracy fields (always present)
    n += snprintf(buf + n, max - n,
                  ",\"nic\":%d,\"nic_baro\":%d,\"nac_p\":%d,\"nac_v\":%d"
                  ",\"sil\":%d,\"gva\":%d,\"sda\":%d",
                  aircraft.navigation_integrity_category, aircraft.navigation_integrity_category_baro,
                  aircraft.navigation_accuracy_category_position, aircraft.navigation_accuracy_category_velocity,
                  aircraft.surveillance_integrity_level, aircraft.geometric_vertical_accuracy,
                  aircraft.system_design_assurance);
    n = n < max ? n : max;

    // version (only if set)
    if (aircraft.uat_version >= 0) {
        n += snprintf(buf + n, max - n, ",\"version\":%d", aircraft.uat_version);
        n = n < max ? n : max;
    }

    // rssi and message count (always present)
    n += snprintf(buf + n, max - n, ",\"rssi\":%d,\"messages\":%lu", aircraft.last_message_signal_strength_dbm,
                  (unsigned long)aircraft.num_frames_received);
    n = n < max ? n : max;

    // emergency (only if not none)
    if (aircraft.emergency_priority_status != UATAircraft::kEmergencyPriorityStatusNone) {
        uint8_t idx = aircraft.emergency_priority_status;
        if (idx < kUATEmergencyStringsCount) {
            n += snprintf(buf + n, max - n, ",\"emergency\":\"%s\"", kUATEmergencyStrings[idx]);
            n = n < max ? n : max;
        }
    }

    if (!aircraft.HasBitFlag(UATAircraft::kBitFlagIsAirborne) || aq == UATAircraft::kSurfaceVehicle) {
        n += snprintf(buf + n, max - n, ",\"on_ground\":1");
        n = n < max ? n : max;
    }

    // seen_pos: seconds since the last position update (readsb semantics).
    if (timestamp_ms != 0 && aircraft.HasBitFlag(UATAircraft::kBitFlagPositionValid)) {
        uint32_t seen_pos_ds = (timestamp_ms - aircraft.last_position_update_ms) / 100;
        n += snprintf(buf + n, max - n, ",\"seen_pos\":%lu.%lu", (unsigned long)(seen_pos_ds / 10),
                      (unsigned long)(seen_pos_ds % 10));
        n = n < max ? n : max;
    }

    n += snprintf(buf + n, max - n, "}\n");

    if (n >= max) return -1;  // Buffer overrun.
    return n;
}

/**
 * Serializes a RemoteIDAircraft (Broadcast Remote ID drone) to a single-line readsb-compatible JSON object. The
 * standard readsb keys (hex/lat/lon/alt_geom/gs/track) are populated from the Aircraft base class so drones render on a
 * generic map, and Remote-ID-specific fields are added under "rid_*" keys for a richer UI. The hex id is the MAC-derived
 * address with a '~' prefix (non-ICAO).
 * @param[out] buf Character array of at least kAircraftJSONMessageStrMaxLen bytes.
 * @param[in] aircraft RemoteIDAircraft to serialize.
 * @retval Number of characters written (excluding the NUL terminator), or negative on error.
 */
inline int16_t WriteAircraftJSONRemoteIDAircraftStr(char buf[], const RemoteIDAircraft& aircraft) {
    int16_t n = 0;
    const int16_t max = kAircraftJSONMessageStrMaxLen;

    // hex: MAC-derived 24-bit address, '~' prefix marks it as non-ICAO / self-assigned.
    n += snprintf(buf + n, max - n,
#if defined(ON_PICO) || defined(ON_HOST)
                  "{\"hex\":\"~%06x\"",
#else
                  "{\"hex\":\"~%06lx\"",
#endif
                  aircraft.address & 0x00FFFFFF);
    n = n < max ? n : max;

    n += snprintf(buf + n, max - n, ",\"type\":\"remote_id\"");
    n = n < max ? n : max;

    // Use the UAS serial / registration as the "flight" label so it shows in generic aircraft lists.
    if (aircraft.HasBitFlag(RemoteIDAircraft::kBitFlagBasicIDValid) && aircraft.uas_id[0] != '\0') {
        n += snprintf(buf + n, max - n, ",\"flight\":\"%s\"", aircraft.uas_id);
        n = n < max ? n : max;
    }

    // category: readsb "B7" = "Unmanned Aerial Vehicle".
    n += snprintf(buf + n, max - n, ",\"category\":\"B7\"");
    n = n < max ? n : max;

    // lat / lon
    if (aircraft.HasBitFlag(RemoteIDAircraft::kBitFlagPositionValid)) {
        n += snprintf(buf + n, max - n, ",\"lat\":%.5f,\"lon\":%.5f", aircraft.latitude_deg, aircraft.longitude_deg);
        n = n < max ? n : max;
    }
    n = AppendAircraftJSONDistanceAndBearing(buf, n, max, aircraft);

    // alt_geom (geometric / WGS84 altitude)
    if (aircraft.HasBitFlag(RemoteIDAircraft::kBitFlagGNSSAltitudeValid)) {
        n += snprintf(buf + n, max - n,
#if defined(ON_PICO) || defined(ON_HOST)
                      ",\"alt_geom\":%d",
#else
                      ",\"alt_geom\":%ld",
#endif
                      aircraft.gnss_altitude_ft);
        n = n < max ? n : max;
    }

    // gs (ground speed, kts)
    if (aircraft.HasBitFlag(RemoteIDAircraft::kBitFlagHorizontalSpeedValid)) {
        n += snprintf(buf + n, max - n,
#if defined(ON_PICO) || defined(ON_HOST)
                      ",\"gs\":%d",
#else
                      ",\"gs\":%ld",
#endif
                      aircraft.speed_kts);
        n = n < max ? n : max;
    }

    // track
    if (aircraft.HasBitFlag(RemoteIDAircraft::kBitFlagDirectionValid)) {
        n += snprintf(buf + n, max - n, ",\"track\":%.1f", aircraft.direction_deg);
        n = n < max ? n : max;
    }

    // RSSI (Remote ID advertisements always carry a signal strength).
    n += snprintf(buf + n, max - n, ",\"rssi\":%d", aircraft.last_message_signal_strength_dbm);
    n = n < max ? n : max;

    // Remote-ID-specific fields for a richer UI.
    if (aircraft.HasBitFlag(RemoteIDAircraft::kBitFlagBasicIDValid)) {
        n += snprintf(buf + n, max - n, ",\"rid_uas_id\":\"%s\",\"rid_id_type\":%u,\"rid_ua_type\":%u", aircraft.uas_id,
                      aircraft.uas_id_type, aircraft.ua_type);
        n = n < max ? n : max;
    }
    if (aircraft.HasBitFlag(RemoteIDAircraft::kBitFlagOperatorIDValid)) {
        n += snprintf(buf + n, max - n, ",\"rid_operator_id\":\"%s\"", aircraft.operator_id);
        n = n < max ? n : max;
    }
    if (aircraft.HasBitFlag(RemoteIDAircraft::kBitFlagOperatorPositionValid)) {
        n += snprintf(buf + n, max - n, ",\"rid_operator_lat\":%.5f,\"rid_operator_lon\":%.5f",
                      aircraft.operator_latitude_deg, aircraft.operator_longitude_deg);
        n = n < max ? n : max;
    }
    if (aircraft.HasBitFlag(RemoteIDAircraft::kBitFlagHeightValid)) {
        n += snprintf(buf + n, max - n,
#if defined(ON_PICO) || defined(ON_HOST)
                      ",\"rid_height\":%d",
#else
                      ",\"rid_height\":%ld",
#endif
                      aircraft.height_above_takeoff_ft);
        n = n < max ? n : max;
    }

    n += snprintf(buf + n, max - n, "}\n");

    if (n >= max) return -1;  // Buffer overrun.
    return n;
}

// ADS-L emergency status strings (spec G.1.4), readsb nomenclature where one exists.
static const char* const kADSLEmergencyStrings[] = {"undefined", "none",     "general", "lifeguard",
                                                     "nordo",     "unlawful", "downed",  "reserved"};
// ADS-L address mapping table names (spec F.2.2); manufacturer pages are "manufacturer".
static inline const char* ADSLAddressMappingToStr(uint8_t amt) {
    switch (amt) {
        case DecodedADSLPacket::kAddressMappingRandom:
            return "random";
        case DecodedADSLPacket::kAddressMappingICAO:
            return "icao";
        case DecodedADSLPacket::kAddressMappingFLARM:
            return "flarm";
        case DecodedADSLPacket::kAddressMappingOGN:
            return "ogn";
        case DecodedADSLPacket::kAddressMappingFANET:
            return "fanet";
        default:
            return amt >= DecodedADSLPacket::kAddressMappingManufacturerPage0 ? "manufacturer" : "reserved";
    }
}

/**
 * Writes an ADS-L aircraft as a readsb-style JSON object. Non-ICAO addresses get the '~' prefix. ADS-L fields:
 * "adsl_amt" (address mapping), "adsl_category" (spec G.1.3 code), "adsl_version", "adsl_relayed".
 */
inline int16_t WriteAircraftJSONADSLAircraftStr(char buf[], const ADSLAircraft& aircraft, uint32_t timestamp_ms = 0) {
    int16_t n = 0;
    const int16_t max = kAircraftJSONMessageStrMaxLen;
    bool is_icao = aircraft.address_mapping == DecodedADSLPacket::kAddressMappingICAO;

    n += snprintf(buf + n, max - n, "{\"hex\":\"%s%06lx\",\"type\":\"adsl\",\"link\":\"adsl\"", is_icao ? "" : "~",
                  (unsigned long)(aircraft.address & 0xFFFFFF));
    n = n < max ? n : max;

    char category_str[4];
    if (EmitterCategoryToStr(category_str, sizeof(category_str), aircraft.GetEmitterCategory())) {
        n += snprintf(buf + n, max - n, ",\"category\":\"%s\"", category_str);
        n = n < max ? n : max;
    }

    if (aircraft.HasBitFlag(ADSLAircraft::kBitFlagPositionValid)) {
        n += snprintf(buf + n, max - n, ",\"lat\":%.5f,\"lon\":%.5f", aircraft.latitude_deg, aircraft.longitude_deg);
        n = n < max ? n : max;
        if (timestamp_ms != 0) {
            n += snprintf(buf + n, max - n, ",\"seen_pos\":%.1f",
                          (timestamp_ms - aircraft.last_position_update_ms) / 1000.0f);
            n = n < max ? n : max;
        }
    }
    n = AppendAircraftJSONDistanceAndBearing(buf, n, max, aircraft);

    if (aircraft.flight_state == DecodedADSLPacket::kFlightStateOnGround) {
        n += snprintf(buf + n, max - n, ",\"on_ground\":true");
        n = n < max ? n : max;
    }
    if (aircraft.HasBitFlag(ADSLAircraft::kBitFlagGNSSAltitudeValid)) {
        n += snprintf(buf + n, max - n, ",\"alt_geom\":%ld", (long)aircraft.gnss_altitude_ft);
        n = n < max ? n : max;
    }
    if (aircraft.HasBitFlag(ADSLAircraft::kBitFlagHorizontalSpeedValid)) {
        n += snprintf(buf + n, max - n, ",\"gs\":%ld", (long)aircraft.speed_kts);
        n = n < max ? n : max;
    }
    if (aircraft.HasBitFlag(ADSLAircraft::kBitFlagDirectionValid)) {
        n += snprintf(buf + n, max - n, ",\"track\":%.1f", aircraft.direction_deg);
        n = n < max ? n : max;
    }
    if (aircraft.HasBitFlag(ADSLAircraft::kBitFlagGNSSVerticalRateValid)) {
        n += snprintf(buf + n, max - n, ",\"geom_rate\":%ld", (long)aircraft.gnss_vertical_rate_fpm);
        n = n < max ? n : max;
    }

    n += snprintf(buf + n, max - n, ",\"nic\":%u,\"nac_p\":%u,\"nac_v\":%u,\"sil\":%u,\"gva\":%u,\"sda\":%u",
                  aircraft.GetNIC(), aircraft.GetNACp(), aircraft.velocity_accuracy, aircraft.source_integrity_level,
                  aircraft.vertical_accuracy, aircraft.design_assurance);
    n = n < max ? n : max;

    if (aircraft.emergency > DecodedADSLPacket::kEmergencyNone && aircraft.emergency < 8) {
        n += snprintf(buf + n, max - n, ",\"emergency\":\"%s\"", kADSLEmergencyStrings[aircraft.emergency]);
        n = n < max ? n : max;
    }

    n += snprintf(buf + n, max - n,
                  ",\"rssi\":%d,\"messages\":%lu,\"adsl_amt\":\"%s\",\"adsl_category\":%u,\"adsl_version\":%u",
                  aircraft.last_message_signal_strength_dbm, (unsigned long)aircraft.num_frames_received,
                  ADSLAddressMappingToStr(aircraft.address_mapping), aircraft.category, aircraft.protocol_version);
    n = n < max ? n : max;
    if (aircraft.HasBitFlag(ADSLAircraft::kBitFlagRelayed)) {
        n += snprintf(buf + n, max - n, ",\"adsl_relayed\":true");
        n = n < max ? n : max;
    }

    n += snprintf(buf + n, max - n, "}\n");
    if (n >= max) return -1;  // Buffer overrun.
    return n;
}

/**
 * Lists the aircraft the Live Map websocket reports: the preferred entry per ICAO address (one target per aircraft),
 * minus those the report filter (AT+FILTER_DISTANCE_NM, AT+FILTER_ALTITUDE_FT) leaves out.
 * @param[in] dictionary Aircraft dictionary to list.
 * @param[in] timestamp_ms Current local time (get_time_since_boot_ms()), for preference.
 * @param[out] uids UIDs of the aircraft to report.
 * @param[in] max_uids Length of uids.
 * @param[in] filter AircraftDictionary::GetReportFilter(): aircraft to leave out. The default keeps all.
 * @retval Number of UIDs written.
 */
inline uint16_t ListAircraftJSONLiveMapUIDs(const AircraftDictionary& dictionary, uint32_t timestamp_ms,
                                            uint32_t uids[], uint16_t max_uids,
                                            const AircraftReportFilter& filter = AircraftReportFilter()) {
    uint16_t num_uids = 0;
    for (const auto& itr : dictionary.dict) {
        if (num_uids >= max_uids) {
            break;
        }
        if (dictionary.IsPreferredReportForAddress(itr.first, timestamp_ms) &&
            !std::visit([&filter](const auto& a) { return filter.Excludes(a); }, itr.second)) {
            uids[num_uids++] = itr.first;
        }
    }
    return num_uids;
}

/**
 * Serializes the next Live Map JSON array: whole aircraft objects from uids[next] on, until buf is full. UIDs no
 * longer in the dictionary are skipped, so the list can be written out over several calls while the dictionary
 * changes.
 * @param[in] dictionary Aircraft dictionary to serialize from.
 * @param[in] timestamp_ms Current local time (get_time_since_boot_ms()), for "seen_pos".
 * @param[in] uids UIDs from ListAircraftJSONLiveMapUIDs().
 * @param[in] num_uids Number of UIDs in uids.
 * @param[in,out] next Index of the next UID to write; advanced past the UIDs written or skipped.
 * @param[out] buf Buffer for the array.
 * @param[in] buf_len Length of buf. Must hold at least one aircraft object plus the brackets.
 * @retval Length of the array in buf, or 0 once no aircraft remain.
 */
inline uint16_t WriteAircraftJSONLiveMapArray(const AircraftDictionary& dictionary, uint32_t timestamp_ms,
                                              const uint32_t uids[], uint16_t num_uids, uint16_t& next, char buf[],
                                              uint16_t buf_len) {
    char json_buf[kAircraftJSONMessageStrMaxLen];
    uint16_t len_used = 0;
    for (; next < num_uids; next++) {
        auto itr = dictionary.dict.find(uids[next]);
        if (itr == dictionary.dict.end()) {
            continue;  // Removed since the list was made.
        }
        int16_t len = -1;
        if (const ModeSAircraft* ac = std::get_if<ModeSAircraft>(&itr->second); ac) {
            len = WriteAircraftJSONModeSAircraftStr(json_buf, *ac, timestamp_ms);
        } else if (const UATAircraft* ac = std::get_if<UATAircraft>(&itr->second); ac) {
            len = WriteAircraftJSONUATAircraftStr(json_buf, *ac, timestamp_ms);
        } else if (const RemoteIDAircraft* ac = std::get_if<RemoteIDAircraft>(&itr->second); ac) {
            len = WriteAircraftJSONRemoteIDAircraftStr(json_buf, *ac);
        } else if (const ADSLAircraft* ac = std::get_if<ADSLAircraft>(&itr->second); ac) {
            len = WriteAircraftJSONADSLAircraftStr(json_buf, *ac, timestamp_ms);
        }
        if (len <= 0 || len + 2 > buf_len) {
            continue;
        }
        // "[" or "," before the object, "]" after the array.
        if (len_used > 0 && len_used + 1 + len + 1 > buf_len) {
            break;  // Starts the next array.
        }
        char separator = len_used == 0 ? '[' : ',';
        buf[len_used++] = separator;
        memcpy(buf + len_used, json_buf, len);
        len_used += len;
    }
    if (len_used > 0) {
        buf[len_used++] = ']';
    }
    return len_used;
}
