#include "aircraft_dictionary.hh"

#include <cmath>

#include "awb_utils.h"
#include "comms.hh"  // For debug logging.
#include "crc.hh"
#include "decode_utils.hh"
#include "fixedmath/fixed_math.hpp"
#include "geo_utils.hh"
#include "hal.hh"
#include "macros.hh"
#include "nasa_cpr.hh"
#include "opendroneid.h"  // Vendored Open Drone ID codec (decode path only).
#include "unit_conversions.hh"

const float kDegreesPerRadian = 360.0f / (2.0f * M_PI);

// This velocity determines the maximum distance an aircraft can jump between position updates without its new position
// needing a second sample to confirm it. This catches position reports that return as valid from the CPR decoder but
// are for an obviously incorrect position.
const uint32_t kCPRPositionFilterVelocityMps =
    1000;  // This is faster than the SR-71, so we're just catching really big jumps for now.

/**
 * ModeSAircraft
 */

ModeSAircraft::ModeSAircraft(uint32_t icao_address_in) : icao_address(icao_address_in) {
    // memset(callsign, '\0', kCallSignMaxNumChars + 1);  // clear out callsign string, including extra EOS character
}

ModeSAircraft::ModeSAircraft() {
    // memset(callsign, '\0', kCallSignMaxNumChars + 1);  // clear out callsign string, including extra EOS character
}

bool ModeSAircraft::CanDecodePosition() {
    // TODO: There could be a condition here where we temporarily lose packets when the MLAT counter wraps around and
    // the packet timestamps all get seen as 0ms, but this probably is not a big deal.
    if (!(last_odd_packet_.received_timestamp_ms > 0 && last_even_packet_.received_timestamp_ms > 0)) {
        CONSOLE_INFO("ModeSAircraft::DecodePosition",
                     "Unable to decode position without receiving an odd and even packet pair for ICAO 0x%lx.",
                     icao_address);
        return false;  // need both an even and an odd packet to be able to decode position
    }
    uint32_t cpr_interval_ms = MIN(last_odd_packet_.received_timestamp_ms - last_even_packet_.received_timestamp_ms,
                                   last_even_packet_.received_timestamp_ms - last_odd_packet_.received_timestamp_ms);
    if (cpr_interval_ms > GetMaxAllowedCPRIntervalMs()) {
        // Reject CPR packet pairings that are too far apart in time.
        WriteBitFlag(BitFlag::kBitFlagPositionValid,
                     false);  // keep last known good coordinates, but mark as invalid
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
        CONSOLE_WARNING("ModeSAircraft::DecodePosition",
                        "CPR packet pair too far apart in time (%lu ms). Can't decode position.", cpr_interval_ms);
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
        return false;
    }
    return true;
}

bool ModeSAircraft::DecodePosition(bool is_airborne, uint32_t ref_lat_awb32, uint32_t ref_lon_awb32,
                                   bool filter_cpr_position) {
    // WARNING: There are two separate timebases in play here! The timebase for CPR packet timestamps is the MLAT
    // timebase, while higher level aircraft dictionary info is in system time.

    if (!CanDecodePosition()) {
        // Can't try forcing a decode and seeing if it's valid, since we'll end up using a stale even or odd packet
        // pairing.
        return false;
    }

    uint32_t most_recent_received_timestamp_ms =
        MAX(last_even_packet_.received_timestamp_ms, last_odd_packet_.received_timestamp_ms);

    NASACPRDecoder::DecodedPosition result;
    bool result_valid = false;
    if (is_airborne) {
        // Airborne CPR decoding.
        result_valid =
            NASACPRDecoder::DecodeAirborneGlobalCPR({.odd = false,
                                                     .lat_cpr = last_even_packet_.n_lat,
                                                     .lon_cpr = last_even_packet_.n_lon,
                                                     .received_timestamp_ms = last_even_packet_.received_timestamp_ms},
                                                    {.odd = true,
                                                     .lat_cpr = last_odd_packet_.n_lat,
                                                     .lon_cpr = last_odd_packet_.n_lon,
                                                     .received_timestamp_ms = last_odd_packet_.received_timestamp_ms},
                                                    result);
    } else {
        // Surface CPR decoding.
        result_valid =
            NASACPRDecoder::DecodeSurfaceGlobalCPR({.odd = false,
                                                    .lat_cpr = last_even_packet_.n_lat,
                                                    .lon_cpr = last_even_packet_.n_lon,
                                                    .received_timestamp_ms = last_even_packet_.received_timestamp_ms},
                                                   {.odd = true,
                                                    .lat_cpr = last_odd_packet_.n_lat,
                                                    .lon_cpr = last_odd_packet_.n_lon,
                                                    .received_timestamp_ms = last_odd_packet_.received_timestamp_ms},
                                                   ref_lat_awb32, ref_lon_awb32, result);
    }

    if (!result_valid) {
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
        CONSOLE_WARNING("ModeSAircraft::DecodePosition",
                        "Can't decode position from packet pair that spans different latitude zones.");
#endif                 // ADSB_VERBOSE_PACKET_WARNINGS
        return false;  // Aircraft crossed between latitude zones, can't decode from this packet pair.
    }

#ifdef FILTER_CPR_POSITIONS
    if (filter_cpr_position) {
        // Calculate how far the aircraft has jumped since its last position update, and goof check it against a maximum
        // velocity value. Could use the aircraft's known velocity, but just using a very large value for now.
        uint32_t ms_since_last_track_update = most_recent_received_timestamp_ms - last_track_update_timestamp_ms;
        uint32_t max_distance_meters =
            kCPRPositionFilterVelocityMps * ms_since_last_track_update / 1000;  // mps to meters
        uint32_t distance_meters =
            CalculateGeoidalDistanceMetersAWB(lat_awb32_, lon_awb32_, result.lat_awb, result.lon_awb);

        if (most_recent_received_timestamp_ms <= last_filter_received_timestamp_ms_) {
            // Don't allow calling DecodePosition() twice without a new packet to override the filter.
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
            CONSOLE_WARNING("ModeSAircraft::DecodePosition",
                            "Received CPR position update for ICAO 0x%lx, but timestamp %lu ms is not newer than last "
                            "filter received timestamp %lu ms.",
                            icao_address, most_recent_received_timestamp_ms, last_filter_received_timestamp_ms_);
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
            return false;
        }

        last_filter_received_timestamp_ms_ = most_recent_received_timestamp_ms;

        // Store the updated position. A location jump can be "confirmed" with a subsequent update from near the new
        // candidate position. Thus, sudden jumps in position take two subsequent position updates that are relatively
        // close together in order to be reflected in the aircraft's position.
        lat_awb32_ = result.lat_awb;
        lon_awb32_ = result.lon_awb;

        // Note: Ignore position check during first position update, to allow recording aircraft position upon receiving
        // the first packet.
        if (HasBitFlag(BitFlag::kBitFlagPositionValid) &&
            distance_meters > MAX(max_distance_meters, AircraftDictionary::kPositionFilterDeadbandMeters)) {
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
            CONSOLE_WARNING("ModeSAircraft::DecodePosition",
                            "Filtered CPR position update for ICAO 0x%lx, distance %lu m exceeds max %lu m.",
                            icao_address, distance_meters, max_distance_meters);
#endif                     // ADSB_VERBOSE_PACKET_WARNINGS
            return false;  // Filter out CPR positions that are too far from the last known position.
        }
    }
#endif

    WriteBitFlag(BitFlag::kBitFlagPositionValid, true);
    latitude_deg = WrapCPRDecodeLatitude(result.lat_deg);
    longitude_deg = WrapCPRDecodeLongitude(result.lon_deg);
    last_track_update_timestamp_ms = most_recent_received_timestamp_ms;  // Update last track update timestamp.

    return true;
}

/**
 * Returns the Wake Vortex category of the aircraft that sent a given ADS-B packet. Note that some categories have a
 * many to one mapping!
 * @param[in] packet ADS-B Packet to extract the EmitterCategory value from. Must be
 * @retval EmitterCategory that matches the combination of capability and type_code from the ADS-B packet, or
 * kEmitterCategoryInvalid if there is no matching wake vortex value.
 */
ADSBTypes::EmitterCategory ExtractCategory(const ModeSADSBPacket& packet) {
    uint8_t type_code = packet.GetNBitWordFromMessage(5, 0);
    uint8_t capability = packet.GetNBitWordFromMessage(3, 5);

    // Table 4.1 from The 1090Mhz Riddle (Junzi Sun), pg. 42.
    if (capability == 0) {
        return ADSBTypes::kEmitterCategoryNoCategoryInfo;
    }

    switch (type_code) {
        case 1:
            // EmitterCategory Set D.
            return ADSBTypes::kEmitterCategoryNoCategoryInfo;
            break;
        case 2:
            // EmitterCategory Set C.
            switch (capability) {
                case 1:
                    return ADSBTypes::kEmitterCategorySurfaceEmergencyVehicle;
                    break;
                case 2:
                    return ADSBTypes::kEmitterCategorySurfaceServiceVehicle;
                    break;
                case 3:
                    return ADSBTypes::kEmitterCategoryPointObstacle;  // Includes tethered balloons.
                    break;
                case 4:
                    return ADSBTypes::kEmitterCategoryClusterObstacle;
                    break;
                case 5:
                    return ADSBTypes::kEmitterCategoryLineObstacle;
                    break;
                case 6:
                case 7:
                    return ADSBTypes::kEmitterCategoryReserved;
                    break;
                default:
                    return ADSBTypes::kEmitterCategoryInvalid;
            }
            break;
        case 3:
            // EmitterCategory set B.
            switch (capability) {
                case 1:
                    return ADSBTypes::kEmitterCategoryGliderSailplane;
                    break;
                case 2:
                    return ADSBTypes::kEmitterCategoryLighterThanAir;
                    break;
                case 3:
                    return ADSBTypes::kEmitterCategoryParachutistSkydiver;
                    break;
                case 4:
                    return ADSBTypes::kEmitterCategoryUltralightHangGliderParaglider;
                    break;
                case 5:
                    return ADSBTypes::kEmitterCategoryReserved;
                    break;
                case 6:
                    return ADSBTypes::kEmitterCategoryUnmannedAerialVehicle;
                    break;
                case 7:
                    return ADSBTypes::kEmitterCategorySpaceTransatmosphericVehicle;
                    break;
                default:
                    return ADSBTypes::kEmitterCategoryInvalid;
            }
            break;
        case 4:
            // EmitterCategory set A.
            switch (capability) {
                case 1:
                    return ADSBTypes::kEmitterCategoryLight;
                    break;
                case 2:
                    return ADSBTypes::kEmitterCategoryMedium1;
                    break;
                case 3:
                    return ADSBTypes::kEmitterCategoryMedium2;
                    break;
                case 4:
                    return ADSBTypes::kEmitterCategoryHighVortexAircraft;
                    break;
                case 5:
                    return ADSBTypes::kEmitterCategoryHeavy;
                    break;
                case 6:
                    return ADSBTypes::kEmitterCategoryHighPerformance;
                    break;
                case 7:
                    return ADSBTypes::kEmitterCategoryRotorcraft;
                    break;
                default:
                    return ADSBTypes::kEmitterCategoryInvalid;
            }
            break;
        default:
            return ADSBTypes::kEmitterCategoryInvalid;
    }
}

bool ModeSAircraft::ApplyAircraftIDMessage(const ModeSADSBPacket& packet) {
    emitter_category = ExtractCategory(packet);
    emitter_category_raw = packet.GetNBitWordFromMessage(8, 0);
    transponder_capability = packet.ca_cf.capability;
    for (uint16_t i = 0; i < ModeSAircraft::kCallSignMaxNumChars; i++) {
        char callsign_char = LookupModeSCallsignChar(packet.GetNBitWordFromMessage(6, 8 + (6 * i)));
        callsign[i] = callsign_char;
    }

    return true;
}

bool ModeSAircraft::ApplySurfacePositionMessage(const ModeSADSBPacket& packet, uint32_t ref_lat_awb32,
                                                uint32_t ref_lon_awb32, bool filter_cpr_position,
                                                bool ref_position_valid) {
    WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne, false);

    if (NICBitIsValid(ADSBTypes::kNICBitA) && NICBitIsValid(ADSBTypes::kNICBitC)) {
        // Assign NIC based on NIC supplement bits A and C and received TypeCode.
        switch ((packet.type_code << 3) | (nic_bits & 0b101)) {
            case (5 << 3) | 0b000:
                navigation_integrity_category = ADSBTypes::kROCLessThan7p5Meters;
                break;
            case (6 << 3) | 0b000:
                navigation_integrity_category = ADSBTypes::kROCLessThan25Meters;
                break;
            case (7 << 3) | 0b100:
                navigation_integrity_category = ADSBTypes::kROCLessThan75Meters;
                break;
            case (7 << 3) | 0b000:
                navigation_integrity_category = ADSBTypes::kROCLessThan0p1NauticalMiles;
                break;
            case (8 << 3) | 0b101:
                navigation_integrity_category = ADSBTypes::kROCLessThan0p2NauticalMiles;
                break;
            case (8 << 3) | 0b100:
                // Should be <0.3NM, but NIC value is shared with <0.6NM.
                navigation_integrity_category = ADSBTypes::kROCLessThan0p6NauticalMiles;
                break;
            case (8 << 3) | 0b001:
                navigation_integrity_category = ADSBTypes::kROCLessThan0p6NauticalMiles;
                break;
            case (8 << 3) | 0b000:
                navigation_integrity_category = ADSBTypes::kROCUnknown;
                break;
            default:
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
                CONSOLE_WARNING("AircraftDictionary::ApplySurfacePositionMessage",
                                "Unable to assign NIC with type code %d and nic bits %d for ICAO 0x%lx.",
                                packet.type_code, nic_bits, icao_address);
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
                break;
        }
    }

    // ME[6-12] - Movement Subfield
    uint8_t movement = packet.GetNBitWordFromMessage(7, 5);
    speed_source = ADSBTypes::kSpeedSourceGroundSpeed;
    baro_vertical_rate_fpm = 0;
    gnss_vertical_rate_fpm = 0;
    // NOTE: We record speed as an integer value in kts, to make filtering multiplications faster on units without an
    // FPU. This also means that all of our reporting protocols use integer speed values. Surface position packets
    // include speed information with fractional values. We round these values to the nearest whole kt.
    if (movement == 0) {
        // No movement info available.
        WriteBitFlag(kBitFlagHorizontalSpeedValid, false);
    } else if (movement == 1) {
        // Aircraft stopped (ground speed = 0 kts).
        WriteBitFlag(kBitFlagHorizontalSpeedValid, true);
        speed_kts = 0;
    } else if (movement == 2) {
        // 0 kts < ground speed < 0.125 kt.
        WriteBitFlag(kBitFlagHorizontalSpeedValid, true);
        speed_kts = 0;  // Round down to 0kts since we keep speed_kts as an integer.
    } else if (movement >= 3 && movement <= 8) {
        // 0.125kts < ground speed < 1 kt.
        WriteBitFlag(kBitFlagHorizontalSpeedValid, true);
        speed_kts = 1;
    } else if (movement >= 9 && movement <= 12) {
        // 1 kt < ground speed < 2 kt.
        WriteBitFlag(kBitFlagHorizontalSpeedValid, true);
        // 0.25 kt steps.
        speed_kts = (movement - 9 + 2) / 4 + 1;  // Round up.
    } else if (movement >= 13 && movement <= 38) {
        // 2 kt < ground speed < 15 kt.
        WriteBitFlag(kBitFlagHorizontalSpeedValid, true);
        // 0.5 kt steps.
        speed_kts = (movement - 13 + 1) / 2 + 2;  // Round up.
    } else if (movement >= 39 && movement <= 93) {
        // 15 kt < ground speed < 70 kt.
        WriteBitFlag(kBitFlagHorizontalSpeedValid, true);
        // 1 kt steps.
        speed_kts = (movement - 39) + 15;
    } else if (movement >= 94 && movement <= 108) {
        // 70 kt < ground speed < 100 kt.
        WriteBitFlag(kBitFlagHorizontalSpeedValid, true);
        // 2 kt steps.
        speed_kts = (movement - 94) * 2 + 70;
    } else if (movement >= 109 && movement <= 123) {
        // 100 kt < ground speed < 175 kt.
        WriteBitFlag(kBitFlagHorizontalSpeedValid, true);
        // 5 kt steps.
        speed_kts = (movement - 109) * 5 + 100;
    } else if (movement == 124) {
        // 157 kt < ground speed.
        WriteBitFlag(kBitFlagHorizontalSpeedValid, true);
        // 5 kt steps.
        speed_kts = 175;
    } else if (movement == 125) {
        // Reserved for aircraft decelerating. Not yet defined.
        WriteBitFlag(kBitFlagHorizontalSpeedValid, false);
    } else if (movement == 126) {
        // Reserved for aircraft accelerating. Not yet defined.
        WriteBitFlag(kBitFlagHorizontalSpeedValid, false);
    } else if (movement == 127) {
        // Reserved for aircraft backing up. Not yet defined.
        WriteBitFlag(kBitFlagHorizontalSpeedValid, false);
    }

    // ME[13] - Status Bit for Heading / Ground Track
    bool ground_direction_is_valid = packet.GetNBitWordFromMessage(1, 12);
    WriteBitFlag(kBitFlagDirectionValid, ground_direction_is_valid);

    // ME[14-20] - Heading / Ground Track Subfield
    uint8_t heading_ground_track = packet.GetNBitWordFromMessage(7, 13);
    direction_deg = (heading_ground_track * 360) / 128.0f;  // Scale to degrees.

    // ME[21] - Reserved for ADS-R
    // This bit must be 0.
    bool reserved_for_adsr = packet.GetNBitWordFromMessage(1, 20);
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
    if (reserved_for_adsr) {
        CONSOLE_WARNING("AircraftDictionary::ApplySurfacePositionMessage",
                        "Received surface position message with ADS-R reserved bit set for ICAO 0x%lx.", icao_address);
    }
#else
    (void)reserved_for_adsr;  // Suppress unused variable warning in non-verbose mode.
#endif  // ADSB_VERBOSE_PACKET_WARNINGS

    // ME[22] - CPR Format
    bool odd = packet.GetNBitWordFromMessage(1, 21);

    // ME[23-39] - Encoded Latitude
    // ME[40-56] - Encoded Longitude
    SetCPRLatLon(packet.GetNBitWordFromMessage(17, 22), packet.GetNBitWordFromMessage(17, 39), odd,
                 packet.raw.GetTimestampMs());
    if (!CanDecodePosition()) {
        // Can't decode aircraft position, but this is not an error.
        return true;
    }
    if (!ref_position_valid) {
        // No valid reference position: surface CPR would resolve against a bogus reference and silently produce a
        // wrong position. Skip the position decode (the CPR frames above are cached for when a reference appears);
        // everything else in the message has been applied, so this is not an error.
        return true;
    }
    if (DecodeSurfacePosition(ref_lat_awb32, ref_lon_awb32, filter_cpr_position)) {
        // Position decode succeeded.
        WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedPosition, true);
    } else {
        // We should have been able to recover a position, but position decode failed. This happens if our position
        // filter algorithm rejects the decoded position result.
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
        CONSOLE_WARNING("AircraftDictionary::ApplyAirbornePositionMessage",
                        "Had valid packets, but aircraft position decode failed for ICAO 0x%lx.", icao_address);
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
        return false;
    }

    return true;
}

bool ModeSAircraft::ApplyAirbornePositionMessage(const ModeSADSBPacket& packet, bool filter_cpr_position) {
    WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne, true);
    uint16_t type_code = packet.type_code;

    bool decode_successful = true;
    // ME[5-6] - Surveillance Status
    uint8_t surveillance_status = packet.GetNBitWordFromMessage(2, 5);
    switch (surveillance_status) {
        case 0:  // No condition.
            WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagAlert, false);
            WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagIdent, false);
            break;
        // NOTE: It's possible to have both an alert and an IDENT at the same time, but that can't be conveyed
        // through a single Airborne Position message.
        case 1:  // Permanent alert.
        case 2:  // Temporary alert.
            // Treat permanent and temporary alerts the same.
            WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagAlert, true);
            break;
        case 3:  // SPI condition.
            WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagIdent, true);
            break;
    }

    // ME[7] - NIC B Supplement (Formerly Single Antenna Flag)
    WriteNICBit(ADSBTypes::kNICBitB, packet.GetNBitWordFromMessage(1, 7));

    if (NICBitIsValid(ADSBTypes::kNICBitA) && NICBitIsValid(ADSBTypes::kNICBitB)) {
        // Assign NIC based on NIC supplement bits A (bit 0) and B (bit 1) and received TypeCode.
        switch ((type_code << 3) | (nic_bits & 0b011)) {
            case (9 << 3) | 0b000:
                navigation_integrity_category = ADSBTypes::kROCLessThan7p5Meters;
                break;
            case (10 << 3) | 0b000:
                navigation_integrity_category = ADSBTypes::kROCLessThan25Meters;
                break;
            case (11 << 3) | 0b010:  // NIC_B=1: RC < 75m
                navigation_integrity_category = ADSBTypes::kROCLessThan75Meters;
                break;
            case (11 << 3) | 0b000:  // NIC_B=0: RC < 0.1NM
                navigation_integrity_category = ADSBTypes::kROCLessThan0p1NauticalMiles;
                break;
            case (12 << 3) | 0b000:
                navigation_integrity_category = ADSBTypes::kROCLessThan0p2NauticalMiles;
                break;
            case (13 << 3) | 0b000:  // NIC_A=0, NIC_B=0: RC < 0.3NM, but NIC value shared with <0.6NM.
            case (13 << 3) | 0b010:  // NIC_A=0, NIC_B=1: RC < 0.5NM, but NIC value shared with <0.6NM.
            case (13 << 3) | 0b001:  // NIC_A=1, NIC_B=0: RC < 0.6NM.
            case (13 << 3) | 0b011:  // NIC_A=1, NIC_B=1: RC < 0.6NM.
                navigation_integrity_category = ADSBTypes::kROCLessThan0p6NauticalMiles;
                break;
            case (14 << 3) | 0b000:
                navigation_integrity_category = ADSBTypes::kROCLessThan1NauticalMile;
                break;
            case (15 << 3) | 0b000:
                navigation_integrity_category = ADSBTypes::kROCLessThan2NauticalMiles;
                break;
            case (16 << 3) | 0b000:  // NIC_A=0: RC < 4NM
                navigation_integrity_category = ADSBTypes::kROCLessThan4NauticalMiles;
                break;
            case (16 << 3) | 0b001:  // NIC_A=1: RC < 8NM
                navigation_integrity_category = ADSBTypes::kROCLessThan8NauticalMiles;
                break;
            case (17 << 3) | 0b000:
                navigation_integrity_category = ADSBTypes::kROCLessThan20NauticalMiles;
                break;
            case (18 << 3) | 0b000:
                navigation_integrity_category = ADSBTypes::kROCUnknown;
                break;
            default:
                // Check for TypeCodes that can determine a NIC without needing to consult NIC supplement bits.
                switch (type_code) {
                    case 20:
                        navigation_integrity_category = ADSBTypes::kROCLessThan7p5Meters;
                        break;
                    case 21:
                        navigation_integrity_category = ADSBTypes::kROCLessThan25Meters;
                        break;
                    case 22:
                        navigation_integrity_category = ADSBTypes::kROCUnknown;
                        break;
                    default:
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
                        CONSOLE_WARNING("AircraftDictionary::ApplyAirbornePositionMessage",
                                        "Unable to assign NIC with type code %d and nic bits %d for ICAO 0x%lx.",
                                        type_code, nic_bits, icao_address);
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
                        break;
                }
        }
    }

    // ME[8-19] - Encoded Altitude
    // DO-260C 2.2.3.2.3.4.3 "Altitude Encoding" in ADS-B Airborne Position Messages
    switch (packet.GetTypeCodeEnum()) {
        case ModeSADSBPacket::TypeCode::kTypeCodeAirbornePositionBaroAlt: {
            uint16_t encoded_altitude_ft_with_q_bit = static_cast<uint16_t>(packet.GetNBitWordFromMessage(12, 8));
            if (encoded_altitude_ft_with_q_bit == 0) {
                altitude_source = ADSBTypes::kAltitudeSourceNotAvailable;
                WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagBaroAltitudeValid, false);
                WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagGNSSAltitudeValid, false);
                WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedBaroAltitude, true);
                WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedGNSSAltitude, true);
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
                CONSOLE_WARNING("AircraftDictionary::ApplyAirbornePositionMessage",
                                "Altitude information not available for ICAO 0x%lx.", icao_address);
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
                decode_successful = true;
            } else {
                altitude_source = ADSBTypes::kAltitudeSourceBaro;
                bool q_bit = (encoded_altitude_ft_with_q_bit & (0b000000010000)) ? true : false;

                int32_t temp_baro_altitude_ft = kAltitudeDecodeErrorInvalid;

                if (q_bit) {
                    // Q bit is set; altitude is encoded as 25N - 1000ft, where N is the value of the bits with the Q
                    // bit removed.
                    // Remove Q bit.
                    uint16_t encoded_altitude_ft = ((encoded_altitude_ft_with_q_bit & 0b111111100000) >> 1) |
                                                   (encoded_altitude_ft_with_q_bit & 0b1111);
                    // 25N - 1000ft
                    temp_baro_altitude_ft = (encoded_altitude_ft * 25) - 1000;
                } else {
                    // Q bit is cleared; altitude is Gillham-coded with the following sequence.
                    // MSG Bit  | 41 | 42 | 43 | 44 | 45 | 46 | 47 | 48 | 49 | 50 | 51 | 52
                    // ---------+----+----+----+----+----+----+----+----+----+----+----+----
                    // "ME" Bit |  9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 | 19 | 20
                    // ---------+----+----+----+----+----+----+----+----+----+----+----+----
                    // Code Bit | C1 | A1 | C2 | A2 | C4 | A4 | B1 | "Q"| B2 | D2 | B4 | D4

                    // Convert the gillham code to a Mode C reply by inserting a 0 "M" bit to the left of B1.
                    // Code Bit | C1 | A1 | C2 | A2 | C4 | A4 | "M"| B1 | "Q"| B2 | D2 | B4 | D4
                    uint16_t altitude_code = ((encoded_altitude_ft_with_q_bit & 0b111111000000) << 1) | (0b0 << 6) |
                                             (encoded_altitude_ft_with_q_bit & 0b111111);
                    temp_baro_altitude_ft = GillhamToAltitudeFt(AltitudeCodeToGillham(altitude_code));
                }

                if (temp_baro_altitude_ft > kAltitudeDecodeErrorInvalid) {
                    baro_altitude_ft = temp_baro_altitude_ft;
                    altitude_source = ADSBTypes::kAltitudeSourceBaro;
                    WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagBaroAltitudeValid, true);
                    WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedBaroAltitude, true);
                } else {
                    // We already handle the case where altitude is unavailable at the beginning of the function, so
                    // this is just Gillham decode issues.
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
                    CONSOLE_WARNING("AircraftDictionary::ApplyAirbornePositionMessage",
                                    "Gillham decode failed for ICAO 0x%lx with altitude code 0x%x.", icao_address,
                                    encoded_altitude_ft_with_q_bit);
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
                    WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagBaroAltitudeValid, false);
                }
            }
            break;
        }
        case ModeSADSBPacket::TypeCode::kTypeCodeAirbornePositionGNSSAlt: {
            altitude_source = ADSBTypes::kAltitudeSourceGNSS;
            uint16_t gnss_altitude_m = static_cast<uint16_t>(packet.GetNBitWordFromMessage(12, 8));
            gnss_altitude_ft = MetersToFeet(gnss_altitude_m);
            WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagGNSSAltitudeValid, true);
            WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedGNSSAltitude, true);
            break;
        }
        default:
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
            CONSOLE_WARNING("AircraftDictionary::ApplyAirbornePositionMessage",
                            "Received packet with unsupported type_code %d with ICAO 0x%lx.", packet.type_code,
                            icao_address);
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
            return false;
    }

    // ME[20] - Time
    // TODO: figure out if we need this

    // ME[21] - CPR Format
    bool odd = packet.GetNBitWordFromMessage(1, 21);

    // ME[23-39] - Encoded Latitude
    // ME[40-56] - Encoded Longitude
    SetCPRLatLon(packet.GetNBitWordFromMessage(17, 22), packet.GetNBitWordFromMessage(17, 39), odd,
                 packet.raw.GetTimestampMs());
    if (!CanDecodePosition()) {
        // Can't decode aircraft position, but this is not an error. Return decode_successful in case there were
        // other errors.
        return decode_successful;
    }
    if (DecodeAirbornePosition(filter_cpr_position)) {
        // Position decode succeeded.
        WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedPosition, true);
    } else {
        // We should have been able to recover a position, but position decode failed. This happens if our position
        // filter algorithm rejects the decoded position result.
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
        CONSOLE_WARNING("AircraftDictionary::ApplyAirbornePositionMessage",
                        "Had valid packets, but aircraft position decode failed for ICAO 0x%lx.", icao_address);
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
    }

    return decode_successful;
}

bool ModeSAircraft::ApplyAirborneVelocitiesMessage(const ModeSADSBPacket& packet) {
    WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne, true);
    bool decode_successful = true;

    // Decode horizontal velocity.
    ModeSADSBPacket::AirborneVelocitiesSubtype subtype =
        static_cast<ModeSADSBPacket::AirborneVelocitiesSubtype>(packet.GetNBitWordFromMessage(3, 5));
    bool is_supersonic = false;
    switch (subtype) {
        case ModeSADSBPacket::AirborneVelocitiesSubtype::kAirborneVelocitiesGroundSpeedSupersonic:
            is_supersonic = true;
            [[fallthrough]];  // Cascade into ground speed calculation.
        case ModeSADSBPacket::AirborneVelocitiesSubtype::kAirborneVelocitiesGroundSpeedSubsonic: {
            // Ground speed calculation.
            int v_ew_kts_plus_1 = static_cast<int>(packet.GetNBitWordFromMessage(10, 14));
            int v_ns_kts_plus_1 = static_cast<int>(packet.GetNBitWordFromMessage(10, 25));
            if (v_ew_kts_plus_1 == 0 || v_ns_kts_plus_1 == 0) {
                speed_source = ADSBTypes::kSpeedSourceNotAvailable;
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
                CONSOLE_WARNING("AircraftDictionary::ApplyAirborneVelocitiesMessage",
                                "Ground speed not available for ICAO 0x%lx.", icao_address);
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
            } else {
                speed_source = ADSBTypes::kSpeedSourceGroundSpeed;
                bool direction_is_east_to_west = static_cast<bool>(packet.GetNBitWordFromMessage(1, 13));
                int32_t v_x_kts = (v_ew_kts_plus_1 - 1) * (direction_is_east_to_west ? -1 : 1);
                bool direction_is_north_to_south = static_cast<bool>(packet.GetNBitWordFromMessage(1, 24));
                int32_t v_y_kts = (v_ns_kts_plus_1 - 1) * (direction_is_north_to_south ? -1 : 1);
                if (is_supersonic) {
                    v_x_kts *= 4;
                    v_y_kts *= 4;
                }
                CalculateTrackAndSpeedFromNEVelocities(v_y_kts, v_x_kts, direction_deg, speed_kts);
            }
            break;
        }
        case ModeSADSBPacket::AirborneVelocitiesSubtype::kAirborneVelocitiesAirspeedSupersonic: {
            is_supersonic = true;
            [[fallthrough]];  // Cascade into airspeed calculation.
        }
        case ModeSADSBPacket::AirborneVelocitiesSubtype::kAirborneVelocitiesAirspeedSubsonic: {
            int airspeed_kts_plus_1 = static_cast<int>(packet.GetNBitWordFromMessage(10, 25));
            if (airspeed_kts_plus_1 == 0) {
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
                CONSOLE_WARNING("AircraftDictionary::ApplyAirborneVelocitiesMessage",
                                "Airspeed not available for ICAO 0x%lx.", icao_address);
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
            } else {
                speed_kts = (airspeed_kts_plus_1 - 1) * (is_supersonic ? 4 : 1);
                bool is_true_airspeed = static_cast<bool>(packet.GetNBitWordFromMessage(1, 24));
                speed_source =
                    is_true_airspeed ? ADSBTypes::kSpeedSourceAirspeedTrue : ADSBTypes::kSpeedSourceAirspeedIndicated;
                direction_deg = static_cast<float>(fixedmath::fixed_t(packet.GetNBitWordFromMessage(10, 14) * 360) /
                                                   fixedmath::fixed_t(1024));
            }

            break;
        }
        default:
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
            CONSOLE_ERROR("AircraftDictionary::ApplyAirborneVelocitiesMessage",
                          "Encountered invalid airborne velocities message subtype %d (valid values are 1-4).",
                          subtype);
#endif                     // ADSB_VERBOSE_PACKET_WARNINGS
            return false;  // Don't attempt vertical rate decode if message type is invalid.
    }
    // Latching bit flags.
    WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagDirectionValid, true);
    WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagHorizontalSpeedValid, true);
    // Non-latching bit flags.
    WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedDirection, true);
    WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedHorizontalSpeed, true);

    // Decode vertical rate.
    int vertical_rate_magnitude_fpm = packet.GetNBitWordFromMessage(9, 37);
    if (vertical_rate_magnitude_fpm == 0) {
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
        CONSOLE_WARNING("AircraftDictionary::ApplyAirborneVelocitiesMessage",
                        "Vertical rate not available for ICAO 0x%lx.", icao_address);
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
    } else {
        ADSBTypes::VerticalRateSource vertical_rate_source =
            static_cast<ADSBTypes::VerticalRateSource>(packet.GetNBitWordFromMessage(1, 35));
        bool vertical_rate_sign_is_negative = packet.GetNBitWordFromMessage(1, 36);
        switch (vertical_rate_source) {
            case ADSBTypes::kVerticalRateSourceBaro:
                WriteBitFlag(kBitFlagBaroVerticalRateValid, true);
                WriteBitFlag(kBitFlagUpdatedBaroVerticalRate, true);
                baro_vertical_rate_fpm =
                    (vertical_rate_magnitude_fpm - 1) * 64 * (vertical_rate_sign_is_negative ? -1 : 1);
                break;
            case ADSBTypes::kVerticalRateSourceGNSS:
                WriteBitFlag(kBitFlagGNSSVerticalRateValid, true);
                WriteBitFlag(kBitFlagUpdatedGNSSVerticalRate, true);
                gnss_vertical_rate_fpm =
                    (vertical_rate_magnitude_fpm - 1) * 64 * (vertical_rate_sign_is_negative ? -1 : 1);
                break;
            default:
                // No vertical rate source specified.
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
                CONSOLE_WARNING("AircraftDictionary::ApplyAirborneVelocitiesMessage",
                                "Vertical rate source not specified for ICAO 0x%lx.", icao_address);
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
                break;
        }
    }

    // Decode altitude difference between GNSS and barometric altitude.
    bool gnss_alt_below_baro_alt = static_cast<bool>(packet.GetNBitWordFromMessage(1, 48));
    uint16_t encoded_gnss_alt_baro_alt_difference_ft = static_cast<uint16_t>(packet.GetNBitWordFromMessage(7, 49));
    if (encoded_gnss_alt_baro_alt_difference_ft == 0) {
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
        CONSOLE_WARNING("AircraftDictionary::ApplyAirborneVelocitiesMessage",
                        "Difference between GNSS and baro altitude not available for aircraft 0x%lx.", icao_address);
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
        // Don't set decode_successful to false so that we ignore missing GNSS/Baro altitude info.
    } else {
        int gnss_alt_baro_alt_difference_ft =
            (encoded_gnss_alt_baro_alt_difference_ft - 1) * 25 * (gnss_alt_below_baro_alt ? -1 : 1);
        switch (altitude_source) {
            case ADSBTypes::kAltitudeSourceBaro:
                gnss_altitude_ft = baro_altitude_ft + gnss_alt_baro_alt_difference_ft;
                WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagGNSSAltitudeValid, true);
                WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedGNSSAltitude, true);
                break;
            case ADSBTypes::kAltitudeSourceGNSS:
                baro_altitude_ft = gnss_altitude_ft - gnss_alt_baro_alt_difference_ft;
                WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagBaroAltitudeValid, true);
                WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedBaroAltitude, true);
                break;
            default:
                // Don't sweat it if the aircraft doesn't have an altitude yet.
                break;
        }
    }

    return decode_successful;
}

bool ModeSAircraft::ApplyAircraftStatusMessage(const ModeSADSBPacket& packet) {
    // TODO: Implement Airraft Status message decoding.
    return true;
}

bool ModeSAircraft::ApplyTargetStateAndStatusInfoMessage(const ModeSADSBPacket& packet) {
    // TODO: Implement Target State and Status Info message decoding.
    return true;
}

bool ModeSAircraft::ApplyAircraftOperationStatusMessage(const ModeSADSBPacket& packet) {
    // https://mode-s.org/1090mhz/content/ads-b/6-operation-status.html
    // https://mode-s.org/decode/content/ads-b/7-uncertainty.html
    //
    // Bit indexing: GetNBitWordFromMessage(n, k) reads n bits at ME offset k (0-indexed from
    // TypeCode MSB = message bit 33), so ME[k] = message bit (33+k).
    //
    // Three distinct formats exist depending on ADS-B version:
    //   v0 (DO-260): CC/OM are 4×4-bit group codes; ME[40-55] are all reserved.
    //   v1 (DO-260A): individual CC/OM flags; ME[40-55] carry NICa/NACp/BAQ/SIL/HRD.
    //   v2 (DO-260B): same as v1 + GVA replaces BAQ, SIL supplement at ME[54], NACv in airborne OM.
    //
    // Version must be read first so all subsequent reads can be version-gated.

    // ME[5-7] - Subtype Code (0 = Airborne, 1 = Surface)
    ModeSADSBPacket::OperationStatusSubtype subtype =
        static_cast<ModeSADSBPacket::OperationStatusSubtype>(packet.GetNBitWordFromMessage(3, 5));

    // ME[40-42] - ADS-B Version Number (message bits 73-75). Present in v0 as reserved (=0).
    adsb_version = packet.GetNBitWordFromMessage(3, 40);

    // ME[8-23] Capacity Class Code and ME[24-39] Operational Mode Code: individual bit fields are
    // only defined for v1+. In v0, these 32 bits carry 4×4-bit group codes with different semantics.
    if (adsb_version >= 1) {
        // ME[8-23] - Capacity Class Code (v1+)
        // ME[11] - 1090ES In
        WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagHas1090ESIn, packet.GetNBitWordFromMessage(1, 11));
        // Other CC fields handled per-subtype below.

        // ME[24-39] - Operational Mode Code (v1+)
        // ME[26] - TCAS RA Active
        WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagTCASRA, packet.GetNBitWordFromMessage(1, 26));
        // ME[27] - IDENT Switch Active
        WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagIdent, packet.GetNBitWordFromMessage(1, 27));
        // ME[29] - Single Antenna Flag
        WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagSingleAntenna, packet.GetNBitWordFromMessage(1, 29));
        // ME[30-31] - System Design Assurance
        system_design_assurance = static_cast<ADSBTypes::SystemDesignAssurance>(packet.GetNBitWordFromMessage(2, 30));
    }

    // ME[43-55]: integrity and accuracy fields — only defined for v1+. In v0, all reserved.
    if (adsb_version >= 1) {
        // ME[43] - NIC Supplement A
        WriteNICBit(ADSBTypes::kNICBitA, packet.GetNBitWordFromMessage(1, 43));

        // ME[44-47] - Navigation Accuracy Category, Position (NACp)
        navigation_accuracy_category_position =
            static_cast<ADSBTypes::NACEstimatedPositionUncertainty>(packet.GetNBitWordFromMessage(4, 44));

        // ME[50-51] - Source Integrity Level (SIL), 2-bit base value.
        // ME[54] - SIL Supplement (SILs): only defined in v2. Extends SIL to 3 bits as (SILs<<2)|SIL.
        uint8_t sil_2bit = packet.GetNBitWordFromMessage(2, 50);
        uint8_t sil_supplement = (adsb_version >= 2) ? packet.GetNBitWordFromMessage(1, 54) : 0u;
        this->surveillance_integrity_level =
            static_cast<ADSBTypes::SILProbabilityOfExceedingNICRadiusOfContainmnent>((sil_supplement << 2) | sil_2bit);

        // ME[53] - Horizontal Reference Direction (HRD)
        WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagHeadingUsesMagneticNorth, packet.GetNBitWordFromMessage(1, 53));
    }

    // Subtype-specific fields.
    switch (subtype) {
        case ModeSADSBPacket::OperationStatusSubtype::kOperationStatusSubtypeAirborne:  // ST = 0
        {
            if (adsb_version >= 1) {
                // ME[10] - TCAS Operational
                WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagTCASOperational, packet.GetNBitWordFromMessage(1, 10));

                // ME[14] - Air Referenced Velocity (ARV) Report Capability - Ignored
                // ME[15] - Target State (TS) Report Capability - Ignored
                // ME[16-17] - Trajectory Change (TC) Report Capability - Ignored
                // ME[18] - UAT In
                WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagHasUATIn, packet.GetNBitWordFromMessage(1, 18));

                // ME[52] - NIC Baro
                navigation_integrity_category_baro =
                    static_cast<ADSBTypes::NICBarometricAltitudeIntegrity>(packet.GetNBitWordFromMessage(1, 52));
            }

            // ME[48-49]: GVA (Geometric Vertical Accuracy) in v2. In v1 this field is BAQ (Barometric
            // Altitude Quality) which has a different encoding — do not store as GVA for v1.
            if (adsb_version >= 2) {
                geometric_vertical_accuracy = static_cast<ADSBTypes::GVA>(packet.GetNBitWordFromMessage(2, 48));

                // ME[32-34] - NACv in airborne OM (v2 only; reserved in v1).
                navigation_accuracy_category_velocity =
                    static_cast<ADSBTypes::NACHorizontalVelocityError>(packet.GetNBitWordFromMessage(3, 32));
            }

            break;
        }
        case ModeSADSBPacket::OperationStatusSubtype::kOperationStatusSubtypeSurface:  // ST = 1
        {
            if (adsb_version >= 1) {
                // ME[14] - B2 Low
                WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagIsClassB2GroundVehicle,
                             packet.GetNBitWordFromMessage(1, 14));
                // ME[15] - UAT In
                WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagHasUATIn, packet.GetNBitWordFromMessage(1, 15));
                // ME[16-18] - NACv
                navigation_accuracy_category_velocity =
                    static_cast<ADSBTypes::NACHorizontalVelocityError>(packet.GetNBitWordFromMessage(3, 16));
                // ME[19] - NIC Supplement C
                WriteNICBit(ADSBTypes::kNICBitC, packet.GetNBitWordFromMessage(1, 19));

                // ME[20-23] Aircraft/Vehicle Length and Width Code
                switch (packet.GetNBitWordFromMessage(4, 20)) {
                    case 0:
                        length_m = 0;
                        width_m = 0;
                        break;
                    case 1:
                        length_m = 15;
                        width_m = 23;
                        break;
                    case 2:
                        length_m = 25;
                        width_m = 29;  // Rounded up from 28.5.
                        break;
                    case 3:
                        length_m = 25;
                        width_m = 34;
                        break;
                    case 4:
                        length_m = 35;
                        width_m = 33;
                        break;
                    case 5:
                        length_m = 35;
                        width_m = 38;
                        break;
                    case 6:
                        length_m = 45;
                        width_m = 40;  // Rounded up from 39.5.
                        break;
                    case 7:
                        length_m = 45;
                        width_m = 45;
                        break;
                    case 8:
                        length_m = 55;
                        width_m = 45;
                        break;
                    case 9:
                        length_m = 55;
                        width_m = 52;
                        break;
                    case 10:
                        length_m = 65;
                        width_m = 60;  // Rounded up from 59.5.
                        break;
                    case 11:
                        length_m = 65;
                        width_m = 67;
                        break;
                    case 12:
                        length_m = 75;
                        width_m = 73;  // Rounded up from 72.5.
                        break;
                    case 13:
                        length_m = 75;
                        width_m = 80;
                        break;
                    case 14:
                        length_m = 85;
                        width_m = 80;
                        break;
                    case 15:
                        length_m = 85;
                        width_m = 90;
                        break;
                }

                // ME[32-39] - GPS Antenna Offset (surface OM only; airborne uses ME[32-34] for NACv in v2).
                switch (packet.GetNBitWordFromMessage(8, 32)) {
                    case 0b000:  // No data.
                        break;
                    case 0b001:  // 2 meters left of roll axis.
                        gnss_antenna_offset_right_of_reference_point_m = -2;
                        break;
                    case 0b010:  // 4 meters left of roll axis.
                        gnss_antenna_offset_right_of_reference_point_m = -4;
                        break;
                    case 0b011:  // 6 meters left of roll axis.
                        gnss_antenna_offset_right_of_reference_point_m = -6;
                        break;
                    case 0b100:  // Centered on roll axis.
                        gnss_antenna_offset_right_of_reference_point_m = 0;
                        break;
                    case 0b101:  // 2 meters right of roll axis.
                        gnss_antenna_offset_right_of_reference_point_m = 2;
                        break;
                    case 0b110:  // 4 meters right of roll axis.
                        gnss_antenna_offset_right_of_reference_point_m = 4;
                        break;
                    case 0b111:  // 6 meters right of roll axis.
                        gnss_antenna_offset_right_of_reference_point_m = 6;
                        break;
                }

                // ME[52] - Track Angle / Heading for Surface Position Messages
                WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagDirectionIsHeading, packet.GetNBitWordFromMessage(1, 52));
            }

            break;
        }
        default:
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
            CONSOLE_ERROR("AircraftDictionary::ApplyAircraftOperationStatusMessage",
                          "Received unsupported Operation Status (TC=31) message subtype %d. Expected 0 or 1.",
                          subtype);
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
            return false;
    }
    return true;
}

/**
 * UATAircraft
 */
UATAircraft::UATAircraft(uint32_t icao_address_in) : icao_address(icao_address_in) {};
UATAircraft::~UATAircraft() {};

bool UATAircraft::DecodePosition(const DecodedUATADSBPacket::UATStateVector& state_vector, bool filter_position) {
#ifdef FILTER_CPR_POSITIONS
    if (filter_position) {
        // Calculate how far the aircraft has jumped since its last position update, and goof check it against a maximum
        // velocity value. Could use the aircraft's known velocity, but just using a very large value for now.
        // NOTE: This calculation assumes that the aicraft's last_message_timestamp_ms has already been updated to
        // reflect the timestamp of the current packet being processed.
        uint32_t ms_since_last_track_update = last_message_timestamp_ms - last_track_update_timestamp_ms;
        uint32_t max_distance_meters =
            kCPRPositionFilterVelocityMps * ms_since_last_track_update / 1000;  // mps to meters

        uint32_t candidate_lat_awb32 = UATAWBToAWB32(state_vector.latitude_awb);
        uint32_t candidate_lon_awb32 = UATAWBToAWB32(state_vector.longitude_awb);
        uint32_t distance_meters =
            CalculateGeoidalDistanceMetersAWB(lat_awb32_, lon_awb32_, candidate_lat_awb32, candidate_lon_awb32);

        if (last_message_timestamp_ms <= last_filter_received_timestamp_ms_) {
            // Don't allow calling DecodePosition() twice without a new packet to override the filter.
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
            CONSOLE_WARNING("UATAircraft::DecodePosition",
                            "Received state vector for ICAO 0x%lx, but timestamp %lu ms is not newer than last "
                            "filter received timestamp %lu ms.",
                            icao_address, last_message_timestamp_ms, last_filter_received_timestamp_ms_);
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
            return false;
        }

        last_filter_received_timestamp_ms_ = last_message_timestamp_ms;

        // Store the updated position. A location jump can be "confirmed" with a subsequent update from near the new
        // candidate position. Thus, sudden jumps in position take two subsequent position updates that are relatively
        // close together in order to be reflected in the aircraft's position.
        lat_awb32_ = candidate_lat_awb32;
        lon_awb32_ = candidate_lon_awb32;

        // Note: Ignore position check during first position update, to allow recording aircraft position upon receiving
        // the first packet.
        if (HasBitFlag(BitFlag::kBitFlagPositionValid) &&
            distance_meters > MAX(max_distance_meters, AircraftDictionary::kPositionFilterDeadbandMeters)) {
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
            CONSOLE_WARNING("UATAircraft::DecodePosition",
                            "Filtered position update for ICAO 0x%lx, distance %lu m exceeds max %lu m.", icao_address,
                            distance_meters, max_distance_meters);
#endif                     // ADSB_VERBOSE_PACKET_WARNINGS
            return false;  // Filter out CPR positions that are too far from the last known position.
        }
    }
#endif

    WriteBitFlag(BitFlag::kBitFlagPositionValid, true);
    latitude_deg = awb2lat(lat_awb32_);
    longitude_deg = awb2lon(lon_awb32_);
    last_track_update_timestamp_ms = last_message_timestamp_ms;  // Update last track update timestamp.
    return true;
}

bool UATAircraft::ApplyUATADSBStateVector(const DecodedUATADSBPacket::UATStateVector& state_vector) {
    // Parse position.

    navigation_integrity_category = static_cast<ADSBTypes::NICRadiusOfContainment>(state_vector.nic);
    if (state_vector.latitude_awb == 0 && state_vector.longitude_awb == 0 && navigation_integrity_category == 0) {
        // Position information not available.
        WriteBitFlag(BitFlag::kBitFlagPositionValid, false);
    } else {
        if (DecodePosition(state_vector)) {
            WriteBitFlag(BitFlag::kBitFlagUpdatedPosition, true);
        }
    }

    if (latitude_deg > 90) {
        latitude_deg -= 180.0f;  // Convert to negative latitude if it exceeds 90 degrees.
    }
    if (longitude_deg > 180) {
        longitude_deg -= 360.0f;  // Convert to negative longitude if it exceeds 180 degrees.
    }

    // Parse altitue.
    if (state_vector.altitude_is_geometric_altitude) {
        int32_t temp_gnss_altitude_ft =
            DecodedUATADSBPacket::AltitudeEncodedToAltitudeFt(state_vector.altitude_encoded);
        if (temp_gnss_altitude_ft == INT32_MIN) {
            WriteBitFlag(BitFlag::kBitFlagGNSSAltitudeValid, false);
        } else {
            gnss_altitude_ft = temp_gnss_altitude_ft;
            WriteBitFlag(BitFlag::kBitFlagGNSSAltitudeValid, true);
        }
        WriteBitFlag(BitFlag::kBitFlagUpdatedGNSSAltitude, true);
    } else {
        int32_t temp_baro_altitude_ft =
            DecodedUATADSBPacket::AltitudeEncodedToAltitudeFt(state_vector.altitude_encoded);
        if (temp_baro_altitude_ft == INT32_MIN) {
            WriteBitFlag(BitFlag::kBitFlagBaroAltitudeValid, false);
        } else {
            baro_altitude_ft = temp_baro_altitude_ft;
            WriteBitFlag(BitFlag::kBitFlagBaroAltitudeValid, true);
        }
        WriteBitFlag(BitFlag::kBitFlagUpdatedBaroAltitude, true);
    }

    // AG state isn't stored directly, but is used to interpret other fields.
    ADSBTypes::AirGroundState ag_state = static_cast<ADSBTypes::AirGroundState>(state_vector.air_ground_state);
    switch (ag_state) {
        case ADSBTypes::kAirGroundStateOnGround:
            WriteBitFlag(BitFlag::kBitFlagIsAirborne, false);
            break;
        case ADSBTypes::kAirGroundStateAirborneSubsonic:
        case ADSBTypes::kAirGroundStateAirborneSupersonic:
        default:  // Put aircraft in the air by default.
            WriteBitFlag(BitFlag::kBitFlagIsAirborne, true);
            break;
    }
    ADSBTypes::DirectionType direction_type = DecodedUATADSBPacket::HorizontalVelocityToDirectionDegAndSpeedKts(
        state_vector.horizontal_velocity, state_vector.air_ground_state, direction_deg, speed_kts);
    WriteBitFlag(BitFlag::kBitFlagUpdatedDirection, true);
    WriteBitFlag(BitFlag::kBitFlagUpdatedHorizontalSpeed, true);

    // Parse direction type and use it to set flags.
    bool received_valid_hvel_data = false;
    switch (direction_type) {
        case ADSBTypes::kDirectionTypeTrueTrackAngle:
            WriteBitFlag(BitFlag::kBitFlagDirectionIsHeading, false);
            WriteBitFlag(BitFlag::kBitFlagHeadingUsesMagneticNorth, false);

            received_valid_hvel_data = true;
            break;
        case ADSBTypes::kDirectionTypeMagneticHeading:
            WriteBitFlag(BitFlag::kBitFlagDirectionIsHeading, true);
            WriteBitFlag(BitFlag::kBitFlagHeadingUsesMagneticNorth, true);

            received_valid_hvel_data = true;
            break;
        case ADSBTypes::kDirectionTypeTrueHeading:
            WriteBitFlag(BitFlag::kBitFlagDirectionIsHeading, true);
            WriteBitFlag(BitFlag::kBitFlagHeadingUsesMagneticNorth, false);

            received_valid_hvel_data = true;
            break;
        default:
            // Heading not available.
            WriteBitFlag(BitFlag::kBitFlagDirectionValid, false);
            WriteBitFlag(BitFlag::kBitFlagDirectionValid, false);

            received_valid_hvel_data = false;
    }
    WriteBitFlag(BitFlag::kBitFlagDirectionValid, received_valid_hvel_data);
    WriteBitFlag(BitFlag::kBitFlagHorizontalSpeedValid, received_valid_hvel_data);

    if (ag_state == ADSBTypes::kAirGroundStateOnGround) {
        // Parse AV dimensions.
        int16_t width_m_temp;
        int16_t length_m_temp;
        switch (DecodedUATADSBPacket::DecodeAVDimensions(state_vector.aircraft_length_width_code, width_m_temp,
                                                         length_m_temp)) {
            case ADSBTypes::kAVDimensionsTypeAVLengthWidth:
                width_m = width_m_temp;
                length_m = length_m_temp;
                break;
            case ADSBTypes::kAVDimensionsTypeGNSSSensorOffset:
                gnss_antenna_offset_forward_of_reference_point_m = length_m_temp;
                gnss_antenna_offset_right_of_reference_point_m = width_m_temp;
                break;
        }
    } else {
        // Parse vertical rate.
        int32_t vertical_rate_fpm_temp;
        ADSBTypes::VerticalRateSource vertical_rate_source = DecodedUATADSBPacket::VerticalVelocityToVerticalRateFpm(
            state_vector.vertical_velocity, state_vector.air_ground_state, vertical_rate_fpm_temp);
        bool vertical_rate_valid = (vertical_rate_fpm_temp != INT32_MIN);
        switch (vertical_rate_source) {
            case ADSBTypes::kVerticalRateSourceBaro:
                if (vertical_rate_valid) {
                    baro_vertical_rate_fpm = vertical_rate_fpm_temp;
                }
                WriteBitFlag(BitFlag::kBitFlagBaroVerticalRateValid, vertical_rate_valid);
                WriteBitFlag(BitFlag::kBitFlagUpdatedBaroVerticalRate, vertical_rate_valid);
                break;
            case ADSBTypes::kVerticalRateSourceGNSS:
                if (vertical_rate_valid) {
                    gnss_vertical_rate_fpm = vertical_rate_fpm_temp;
                }
                WriteBitFlag(BitFlag::kBitFlagGNSSVerticalRateValid, vertical_rate_valid);
                WriteBitFlag(BitFlag::kBitFlagUpdatedGNSSVerticalRate, vertical_rate_valid);
                break;
            default:
                // Vertical rate not available.
                break;
        }
    }

    return true;
}
bool UATAircraft::ApplyUATADSBModeStatus(const DecodedUATADSBPacket::UATModeStatus& mode_status) {
    // Callsign field: could be ID or squawk.
    char callsign_temp[UATAircraft::kCallSignMaxNumChars + 1] = {0};

    // Extract callsign and emitter category.
    uint16_t temp = mode_status.emitter_category_and_callsign_chars_1_2;
    emitter_category_raw = temp / 1600;
    emitter_category = static_cast<ADSBTypes::EmitterCategory>(emitter_category_raw);
    temp %= 1600;
    callsign_temp[0] = LookupUATCallsignChar(temp / 40);
    temp %= 40;
    callsign_temp[1] = LookupUATCallsignChar(temp);

    temp = mode_status.callsign_chars_3_4_5;
    callsign_temp[2] = LookupUATCallsignChar(temp / 1600);
    temp %= 1600;
    callsign_temp[3] = LookupUATCallsignChar(temp / 40);
    temp %= 40;
    callsign_temp[4] = LookupUATCallsignChar(temp);

    temp = mode_status.callsign_chars_6_7_8;
    callsign_temp[5] = LookupUATCallsignChar(temp / 1600);
    temp %= 1600;
    callsign_temp[6] = LookupUATCallsignChar(temp / 40);
    temp %= 40;
    callsign_temp[7] = LookupUATCallsignChar(temp);

    if (mode_status.csid) {
        // Callsign field encodes an ID (alphanumerica callsign).
        strncpy(callsign, callsign_temp, UATAircraft::kCallSignMaxNumChars);
        callsign[UATAircraft::kCallSignMaxNumChars] = '\0';
    } else {
        // Callsign field encodes a squawk code.
        squawk = 0;
        for (uint16_t i = 0; i < kSquawkNumDigits; i++) {
            squawk *= 10;
            squawk += (callsign_temp[i] - '0');
        }
    }

    // Emergency / Priority Status
    emergency_priority_status =
        static_cast<UATAircraft::EmergencyPriorityStatus>(mode_status.emergency_priority_status);

    // UAT MOPs version.
    uat_version = mode_status.uat_version;

    // Surveillance Integrity Level
    surveillance_integrity_level =
        static_cast<ADSBTypes::SILProbabilityOfExceedingNICRadiusOfContainmnent>(mode_status.sil);

    // Transmit Message Start Opportunity (MSO)
    transmit_mso = mode_status.transmit_mso;

    // NACp
    navigation_accuracy_category_position = static_cast<ADSBTypes::NACEstimatedPositionUncertainty>(mode_status.nac_p);

    // NACv
    navigation_accuracy_category_velocity = static_cast<ADSBTypes::NACHorizontalVelocityError>(mode_status.nac_v);

    // NICbaro
    navigation_integrity_category_baro = static_cast<ADSBTypes::NICBarometricAltitudeIntegrity>(mode_status.nic_baro);

    // Capability codes
    raw_capability_codes = mode_status.capability_codes;
    // Has CDTI traffic display capability.
    WriteBitFlag(UATAircraft::kBitFlagHasCDTI, raw_capability_codes & 0b10);
    if (raw_capability_codes & 0b01) {
        // Has TCAS/ACAS installed and operational.
        WriteBitFlag(UATAircraft::kBitFlagTCASOperational, true);
    } else {
        // Does not have TCAS/ACAS installed and operational.
        WriteBitFlag(UATAircraft::kBitFlagTCASOperational, false);
    }

    // Operational modes
    raw_operational_modes = mode_status.operational_modes;
    WriteBitFlag(UATAircraft::kBitFlagTCASRA, raw_operational_modes & 0b100);
    WriteBitFlag(UATAircraft::kBitFlagIdent, raw_operational_modes & 0b10);
    WriteBitFlag(UATAircraft::kBitFlagReceivingATCServices, raw_operational_modes & 0b1);

    // True / Magnetic Heading
    WriteBitFlag(UATAircraft::kBitFlagHeadingUsesMagneticNorth, mode_status.heading_uses_magnetic_north);

    return true;
}
bool UATAircraft::ApplyUATADSBTargetState(const DecodedUATADSBPacket::UATTargetState& target_state) { return true; }
bool UATAircraft::ApplyUATADSBTrajectoryChange(const DecodedUATADSBPacket::UATTrajectoryChange& trajectory_change) {
    return true;
}
bool UATAircraft::ApplyUATADSBAuxiliaryStateVector(
    const DecodedUATADSBPacket::UATStateVector& state_vector,
    const DecodedUATADSBPacket::UATAuxiliaryStateVector& auxiliary_state_vector) {
    // Need to refer to the state vector in order to see what type of altitude the secondary altitude is.
    if (state_vector.altitude_is_geometric_altitude) {
        // Primary altitude is gnss, so secondary altitude must be baro.
        int32_t temp_baro_altitude_ft =
            DecodedUATADSBPacket::AltitudeEncodedToAltitudeFt(auxiliary_state_vector.secondary_altitude_encoded);
        if (temp_baro_altitude_ft == INT32_MIN) {
            WriteBitFlag(BitFlag::kBitFlagBaroAltitudeValid, false);
        } else {
            baro_altitude_ft = temp_baro_altitude_ft;
            WriteBitFlag(BitFlag::kBitFlagBaroAltitudeValid, true);
        }

    } else {
        // Primary altitude is baro, so secondary altitude must be gnss.
        int32_t temp_gnss_altitude_ft =
            DecodedUATADSBPacket::AltitudeEncodedToAltitudeFt(auxiliary_state_vector.secondary_altitude_encoded);
        if (temp_gnss_altitude_ft == INT32_MIN) {
            WriteBitFlag(BitFlag::kBitFlagGNSSAltitudeValid, false);
        } else {
            gnss_altitude_ft = temp_gnss_altitude_ft;
            WriteBitFlag(BitFlag::kBitFlagGNSSAltitudeValid, true);
        }
    }

    return true;
}

/**
 * Aircraft Dictionary
 */

void AircraftDictionary::Init() {
    dict.clear();  // Remove all aircraft from the unordered map.
}

void AircraftDictionary::Update(uint32_t timestamp_ms) {
    // Check for overflow
    if (dictionary_overflowed) {
        CONSOLE_ERROR("AircraftDictionary",
                      "Aircraft dictionary overflowed. Had to remove one or more oldest aircraft to make room.");
        dictionary_overflowed = false;
    }

    // Iterate over each key-value pair in the unordered_map. Prune if stale, update metrics if still fresh.
    for (auto it = dict.begin(); it != dict.end(); /* No increment here */) {
        uint32_t last_message_timestamp_ms =
            std::visit([](Aircraft& aircraft) -> uint32_t { return aircraft.last_message_timestamp_ms; }, it->second);

        // Extract the last message timestamp of the underlying ModeSAircraft or UATAircraft.
        if (timestamp_ms - last_message_timestamp_ms > config_.aircraft_prune_interval_ms) {
            // Aircraft entry is stale, remove it.

            // Reset lowest_aircraft_entry if it points to the aircraft being removed.
            if (lowest_aircraft_entry == &it->second) {
                lowest_aircraft_entry = nullptr;
            }

            it = dict.erase(it);  // Remove stale aircraft entry. Iterator is incremented to the next element.
        } else {
            // Check the type of the aircraft and count it.
            if (std::holds_alternative<ModeSAircraft>(it->second)) {
                metrics_counter_.num_mode_s_aircraft++;
            } else if (std::holds_alternative<UATAircraft>(it->second)) {
                metrics_counter_.num_uat_aircraft++;
            } else if (std::holds_alternative<RemoteIDAircraft>(it->second)) {
                metrics_counter_.num_remote_id_aircraft++;
            } else {
                // This should never happen.
                CONSOLE_ERROR("AircraftDictionary::Update", "Encountered aircraft entry with unknown type.");
            }
            // Call UpdateMetrics on the underlying aircraft type.
            std::visit(
                [](Aircraft& aircraft) {
                    // Update the metrics for the aircraft.
                    aircraft.UpdateMetrics();
                },
                it->second);

            // Update lowest GNSS altitude if available.
            bool has_valid_gnss_altitude = false;
            int32_t current_gnss_altitude_ft = 0;
            if (auto* aircraft = std::get_if<ModeSAircraft>(&it->second)) {
                if (aircraft->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagGNSSAltitudeValid)) {
                    // Don't trust Mode S Aircraft that are already on the ground to provide a reference location, since
                    // they need a reference location to be decoded! Rely on Mode S Aircraft in the air or other types
                    // of aircraft in the air or on the ground.
                    has_valid_gnss_altitude = aircraft->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne);
                    current_gnss_altitude_ft = aircraft->gnss_altitude_ft;
                }
            } else if (auto* aircraft = std::get_if<UATAircraft>(&it->second)) {
                if (aircraft->HasBitFlag(UATAircraft::BitFlag::kBitFlagGNSSAltitudeValid)) {
                    has_valid_gnss_altitude = true;
                    current_gnss_altitude_ft = aircraft->gnss_altitude_ft;
                }
            } else if (std::holds_alternative<RemoteIDAircraft>(it->second)) {
                // Remote ID drones fly low and close to the receiver, so they must not be used as the lowest-aircraft
                // position reference (which assumes an aircraft roughly overhead). Leave has_valid_gnss_altitude false.
            } else {
                CONSOLE_ERROR("AircraftDictionary::Update", "Encountered aircraft entry with unknown variant type.");
            }
            if (has_valid_gnss_altitude) {
                int32_t lowest_gnss_altitude_ft =
                    lowest_aircraft_entry
                        ? std::visit([](auto& a) { return a.gnss_altitude_ft; }, *lowest_aircraft_entry)
                        : INT32_MAX;
                if (current_gnss_altitude_ft < lowest_gnss_altitude_ft) {
                    lowest_aircraft_entry = &it->second;
                }
            }

            it++;  // Move to the next aircraft entry.
        }
    }

    // Update aggregate statistics.
    metrics = metrics_counter_;    // Swap counter values over to publicly visible values.
    metrics_counter_ = Metrics();  // Use default constructor to clear all values.
}

bool AircraftDictionary::GetLowestAircraftPosition(float& latitude_deg, float& longitude_deg, int32_t& gnss_altitude_ft,
                                                   int32_t& baro_altitude_ft, float& heading_deg, int32_t& speed_kts) {
    if (!lowest_aircraft_entry) {
        return false;
    }

    if (auto* aircraft = std::get_if<ModeSAircraft>(lowest_aircraft_entry)) {
        if (!aircraft->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagPositionValid)) {
            return false;
        }
        latitude_deg = aircraft->latitude_deg;
        longitude_deg = aircraft->longitude_deg;
        gnss_altitude_ft = aircraft->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagGNSSAltitudeValid)
                               ? aircraft->gnss_altitude_ft
                               : INT32_MIN;
        baro_altitude_ft = aircraft->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagBaroAltitudeValid)
                               ? aircraft->baro_altitude_ft
                               : INT32_MIN;
        heading_deg =
            aircraft->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagDirectionValid) ? aircraft->direction_deg : NAN;
        speed_kts =
            aircraft->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagHorizontalSpeedValid) ? aircraft->speed_kts : NAN;
        return true;
    } else if (auto* aircraft = std::get_if<UATAircraft>(lowest_aircraft_entry)) {
        if (!aircraft->HasBitFlag(UATAircraft::BitFlag::kBitFlagPositionValid)) {
            return false;
        }
        latitude_deg = aircraft->latitude_deg;
        longitude_deg = aircraft->longitude_deg;
        gnss_altitude_ft = aircraft->HasBitFlag(UATAircraft::BitFlag::kBitFlagGNSSAltitudeValid)
                               ? aircraft->gnss_altitude_ft
                               : INT32_MIN;
        baro_altitude_ft = aircraft->HasBitFlag(UATAircraft::BitFlag::kBitFlagBaroAltitudeValid)
                               ? aircraft->baro_altitude_ft
                               : INT32_MIN;
        heading_deg =
            aircraft->HasBitFlag(UATAircraft::BitFlag::kBitFlagDirectionValid) ? aircraft->direction_deg : NAN;
        speed_kts =
            aircraft->HasBitFlag(UATAircraft::BitFlag::kBitFlagHorizontalSpeedValid) ? aircraft->speed_kts : NAN;
        return true;
    } else if (std::holds_alternative<RemoteIDAircraft>(*lowest_aircraft_entry)) {
        // Remote ID drones are never used as a position reference (see AircraftDictionary::Update), so this should not
        // normally be reached; guard defensively.
        return false;
    } else {
        CONSOLE_ERROR("AircraftDictionary::GetLowestAircraftPosition",
                      "Encountered lowest_aircraft_entry with unknown variant type.");
        return false;
    }
}

bool AircraftDictionary::ContainsAircraft(uint32_t uid) const {
    auto itr = dict.find(uid);
    if (itr != dict.end()) {
        return true;
    }
    return false;
}

bool AircraftDictionary::IngestDecodedModeSPacket(DecodedModeSPacket& packet) {
    // Check validity and record stats.
    int16_t source = packet.raw.source;

    // Validate packet against ICAO addresses in dictionary, or allow it in if it's a DF=11 all call reply
    // packet tha validated itself (e.g. it's a response to a spontaneous acquisition squitter with interrogator
    // ID=0, making the checksum useable).
    if (packet.is_address_parity
#ifndef AIRCRAFT_DICTIONARY_TRUST_FORWARDED_ADDRESS_PARITY
        && ContainsAircraft(packet.icao_address)
#endif
    ) {
        // DF=0,4,5,16,20,21 (DF=11 doesn't work with this since the interrogator ID may be overlaid with the ICAO
        // address--we expect spontaneous acquisition DF=11's to come in pre-marked as valid).
        // Packet is address parity that is incapable of validating itself, and its CRC was
        // validated against the ICAO addresses in the aircraft dictionary.
        // When AIRCRAFT_DICTIONARY_TRUST_FORWARDED_ADDRESS_PARITY is set, the upstream validator
        // (RP2040) already confirmed this ICAO, so we skip the dictionary lookup.
        packet.is_valid = true;
    }

    switch (packet.raw.buffer_len_bytes) {
        case RawModeSPacket::kSquitterPacketLenBytes:

            if (!packet.is_valid) {
                return false;
            }

            // Record a valid squitter packet.
            metrics_counter_.valid_squitter_frames++;
            if (source >= 0 && source < kMaxNumSources) {
                metrics_counter_.valid_squitter_frames_by_source[source]++;
            }
            break;
        case RawModeSPacket::kExtendedSquitterPacketLenBytes:
            if (packet.is_valid) {
                metrics_counter_.valid_extended_squitter_frames++;
                if (source >= 0 && source < kMaxNumSources) {
                    metrics_counter_.valid_extended_squitter_frames_by_source[source]++;
                }
            } else {
#ifdef ON_ESP32
                // ESP32 should only be receiving valid packets.
                CONSOLE_ERROR("AircraftDictionary::IngestDecodedModeSPacket",
                              "Extended squitter packet failed checksum.");
#endif
                return false;  // Extended squitter frame failed CRC.
            }
            break;
        default:
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
            CONSOLE_ERROR(
                "AircraftDictionary::IngestDecodedModeSPacket",
                "Received packet with unrecognized byte length %d, expected %d (Squitter) or %d (Extended Squitter).",
                packet.raw.buffer_len_bytes, RawModeSPacket::kSquitterPacketLenBytes,
                RawModeSPacket::kExtendedSquitterPacketLenBytes);
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
            return false;
    }

    // Ingest packet.
    bool ingest_ret = false;
    uint16_t downlink_format = packet.downlink_format;
    switch (downlink_format) {
        // Altitude Reply Packet.
        case DecodedModeSPacket::DownlinkFormat::kDownlinkFormatAltitudeReply:
            ingest_ret = IngestModeSAltitudeReplyPacket(ModeSAltitudeReplyPacket(packet));
            break;
        // Identity Reply Packet.
        case DecodedModeSPacket::DownlinkFormat::kDownlinkFormatIdentityReply:
            ingest_ret = IngestModeSIdentityReplyPacket(ModeSIdentityReplyPacket(packet));
            break;
        case DecodedModeSPacket::DownlinkFormat::kDownlinkFormatAllCallReply:  // DF = 11
            ingest_ret = IngestModeSAllCallReplyPacket(ModeSAllCallReplyPacket(packet));
            break;
        // ADS-B Packets.
        case DecodedModeSPacket::DownlinkFormat::kDownlinkFormatExtendedSquitter:                // DF = 17
        case DecodedModeSPacket::DownlinkFormat::kDownlinkFormatExtendedSquitterNonTransponder:  // DF = 18
        case DecodedModeSPacket::DownlinkFormat::kDownlinkFormatMilitaryExtendedSquitter:        // DF = 19
            // Handle ADS-B Packets.
            ingest_ret = IngestModeSADSBPacket(ModeSADSBPacket(packet));
            break;
        case DecodedModeSPacket::DownlinkFormat::kDownlinkFormatShortRangeAirToAirSurveillance:  // DF = 0
        case DecodedModeSPacket::DownlinkFormat::kDownlinkFormatLongRangeAirToAirSurveillance:   // DF = 16
        case DecodedModeSPacket::DownlinkFormat::kDownlinkFormatCommBAltitudeReply:              // DF = 20
        case DecodedModeSPacket::DownlinkFormat::kDownlinkFormatCommBIdentityReply:              // DF = 21
        case DecodedModeSPacket::DownlinkFormat::kDownlinkFormatCommDExtendedLengthMessage:      // DF = 24
            // Silently handle currently unsupported downlink formats.
            ingest_ret = true;
            break;

        default:
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
            CONSOLE_WARNING("AircraftDictionary::IngestDecodedModeSPacket",
                            "Encountered unexpected downlink format %d for ICAO 0x%lx.", downlink_format,
                            packet.icao_address);
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
            ingest_ret = false;
    }
    return ingest_ret;
}

bool AircraftDictionary::IngestModeSIdentityReplyPacket(const ModeSIdentityReplyPacket& packet) {
    if (!packet.is_valid) {
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
        CONSOLE_WARNING("AircraftDictionary::IngestModeSIdentityReplyPacket", "Received invalid packet.");
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
        return false;
    }
    if (packet.downlink_format != ModeSIdentityReplyPacket::kDownlinkFormatIdentityReply) {
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
        CONSOLE_WARNING("AircraftDictionary::IngestModeSIdentityReplyPacket",
                        "Received Mode S packet with invalid downlink format %d, expected %d (Identity Reply).",
                        packet.downlink_format, ModeSIdentityReplyPacket::kDownlinkFormatIdentityReply);
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
        return false;
    }

    uint32_t icao_address = packet.icao_address;
    uint32_t uid = Aircraft::ICAOToUID(icao_address, Aircraft::kAircraftTypeModeS);
    ModeSAircraft* aircraft_ptr = GetAircraftPtr<ModeSAircraft>(uid);
    if (aircraft_ptr == nullptr) {
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
        CONSOLE_WARNING("AircraftDictionary::IngestModeSIdentityReplyPacket",
                        "Unable to find or create new aircraft with ICAO address 0x%lx in dictionary.\r\n",
                        icao_address);
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
        return false;  // unable to find or create new aircraft in dictionary
    }
    if (packet.airborne_state_known) {
        aircraft_ptr->WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne, packet.is_airborne);
    }
    aircraft_ptr->WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagAlert, packet.has_alert);
    aircraft_ptr->WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagIdent, packet.has_ident);
    aircraft_ptr->squawk = packet.squawk;
    aircraft_ptr->IncrementNumFramesReceived(false);

    return true;
}

bool AircraftDictionary::IngestModeSAltitudeReplyPacket(const ModeSAltitudeReplyPacket& packet) {
    if (!packet.is_valid) {
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
        CONSOLE_WARNING("AircraftDictionary::IngestModeSAltitudeReplyPacket", "Received invalid packet.");
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
        return false;
    }
    if (packet.downlink_format != ModeSAltitudeReplyPacket::kDownlinkFormatAltitudeReply) {
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
        CONSOLE_WARNING("AircraftDictionary::IngestModeSAltitudeReplyPacket",
                        "Received Mode S packet with invalid downlink format %d, expected %d (Altitude Reply).",
                        packet.downlink_format, ModeSAltitudeReplyPacket::kDownlinkFormatAltitudeReply);
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
        return false;
    }

    uint32_t icao_address = packet.icao_address;
    uint32_t uid = Aircraft::ICAOToUID(icao_address, Aircraft::kAircraftTypeModeS);
    ModeSAircraft* aircraft_ptr = GetAircraftPtr<ModeSAircraft>(uid);
    if (aircraft_ptr == nullptr) {
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
        CONSOLE_WARNING("AircraftDictionary::IngestModeSAltitudeReplyPacket",
                        "Unable to find or create new aircraft with ICAO address 0x%lx in dictionary.\r\n",
                        icao_address);
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
        return false;  // unable to find or create new aircraft in dictionary
    }
    if (packet.airborne_state_known) {
        aircraft_ptr->WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne, packet.is_airborne);
    }
    aircraft_ptr->WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagAlert, packet.has_alert);
    aircraft_ptr->WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagIdent, packet.has_ident);
    aircraft_ptr->baro_altitude_ft = packet.altitude_ft;
    aircraft_ptr->WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagBaroAltitudeValid, true);
    aircraft_ptr->WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedBaroAltitude, true);
    aircraft_ptr->IncrementNumFramesReceived(false);

    return true;
}

bool AircraftDictionary::IngestModeSAllCallReplyPacket(const ModeSAllCallReplyPacket& packet) {
    if (!packet.is_valid) {
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
        CONSOLE_WARNING("AircraftDictionary::IngestModeSAllCallReplyPacket", "Received invalid packet.");
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
        return false;
    }
    if (packet.downlink_format != DecodedModeSPacket::kDownlinkFormatAllCallReply) {
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
        CONSOLE_WARNING("AircraftDictionary::IngestModeSAllCallReplyPacket",
                        "Received Mode S packet with invalid downlink format %d, expected %d (All Call Reply).",
                        packet.downlink_format, DecodedModeSPacket::kDownlinkFormatAllCallReply);
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
        return false;
    }

    // Populate the dictionary with the aircraft, or look it up if the ICAO doesn't yet exist.
    uint32_t icao_address = packet.icao_address;
    uint32_t uid = Aircraft::ICAOToUID(icao_address, Aircraft::kAircraftTypeModeS);
    ModeSAircraft* aircraft_ptr = GetAircraftPtr<ModeSAircraft>(uid);
    if (aircraft_ptr == nullptr) {
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
        CONSOLE_WARNING("AircraftDictionary::IngestModeSAllCallReplyPacket",
                        "Unable to find or create new aircraft with ICAO address 0x%lx in dictionary.\r\n",
                        icao_address);
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
        return false;  // unable to find or create new aircraft in dictionary
    }
    aircraft_ptr->last_message_timestamp_ms = get_time_since_boot_ms();

    // Update aircaft transponder capability.
    aircraft_ptr->transponder_capability = packet.capability;

    return true;
}

bool AircraftDictionary::IngestModeSADSBPacket(const ModeSADSBPacket& packet) {
    if (!packet.is_valid) {
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
        CONSOLE_WARNING("AircraftDictionary::IngestModeSADSBPacket", "Received invalid packet.");
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
        return false;
    }
    // We can accept DF=17 (Extended Squitter) and DF=18 (Extended Squitter Non-Transponder). Will need to check Code
    // Format field for DF=18 to make sure we can accept it.
    switch (packet.downlink_format) {
        case ModeSADSBPacket::kDownlinkFormatExtendedSquitter:  // DF = 17
            // Valid DF=17 packet.
            break;
        case ModeSADSBPacket::kDownlinkFormatExtendedSquitterNonTransponder:  // DF = 18
            // Only accept DF=18 packets with Code Format = 0, 1, 2, 5, 6.
            // DO-260C Table 2-7.
            switch (packet.ca_cf.code_format) {
                case 0:  // AA field holds the transmitting ADS-B Participant's 24-bit ICAO address.
                case 1:  // AA field holds an anonymous 24-bit address, ground vehicle address, or surface obstruction
                         // address.
                case 2:  // Fine TIS-B Message using ICAO 24-bit address.
                case 5:  // Fine TIS-B Message using non-ICAO 24-bit address.
                case 6:  // ADS-R, rebroadcast of ADSB from alternate data link, and ADS-SLR, rebroadcast of ADSB on
                         // same data link, using same type code and message format as DF17 except for bits modified in
                         // DO-260C section 2.2.18.
                    break;  // Valid Code Format for DF=18 packet.
                default:
                    // 3 Reserved.
                    // 7 Reserved.
                    // 4 Traffic Uplink Management Messages.
                    return true;  // Unsupported Code Format for DF=18 packet.
            }
            break;
        case ModeSADSBPacket::kDownlinkFormatMilitaryExtendedSquitter:  // DF = 19
            // Accept military packets but bail out after recording the ICAO.
            break;
        default:
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
            CONSOLE_WARNING("AircraftDictionary::IngestModeSADSBPacket",
                            "Received Mode S packet with invalid downlink format %d, expected %d (Extended Squitter).",
                            packet.downlink_format, ModeSADSBPacket::kDownlinkFormatExtendedSquitter);
#endif                     // ADSB_VERBOSE_PACKET_WARNINGS
            return false;  // Only allow valid DF17, DF18 packets.
    }

    uint32_t icao_address = packet.icao_address;
    uint32_t uid = Aircraft::ICAOToUID(icao_address, Aircraft::kAircraftTypeModeS);
    ModeSAircraft* aircraft_ptr = GetAircraftPtr<ModeSAircraft>(uid);
    if (aircraft_ptr == nullptr) {
        // This should never happen, since the aircraft dictionary will automatically remove the oldest aircraft to make
        // space for a new one if full.
        CONSOLE_ERROR("AircraftDictionary::IngestModeSADSBPacket",
                      "Unable to find or create new aircraft with ICAO address 0x%lx in dictionary.\r\n", icao_address);
        return false;  // unable to find or create new aircraft in dictionary
    }
    aircraft_ptr->last_message_timestamp_ms = get_time_since_boot_ms();
    aircraft_ptr->last_message_signal_strength_dbm = packet.raw.sigs_dbm;
    aircraft_ptr->last_message_signal_quality_db = packet.raw.sigq_db;

    if (packet.downlink_format == ModeSADSBPacket::kDownlinkFormatMilitaryExtendedSquitter) {
        // Don't attempt to decode military extended squitter packets, since we don't have the specs for them. Just mark
        // the aircraft as military and record the ICAO and signal strength / quality.
        aircraft_ptr->WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagIsMilitary, true);
        return true;  // Early bail out.
    } else if (packet.downlink_format == ModeSADSBPacket::kDownlinkFormatExtendedSquitterNonTransponder) {
        aircraft_ptr->WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagIsNonTransponder, true);
    }

    bool ret = false;
    uint16_t type_code = packet.type_code;
    switch (type_code) {
        case ModeSADSBPacket::kTypeCodeAircraftID:      // TC = 1 (Aircraft Identification)
        case ModeSADSBPacket::kTypeCodeAircraftID + 1:  // TC = 2 (Aircraft Identification)
        case ModeSADSBPacket::kTypeCodeAircraftID + 2:  // TC = 3 (Aircraft Identification)
        case ModeSADSBPacket::kTypeCodeAircraftID + 3:  // TC = 4 (Aircraft Identification)
            ret = aircraft_ptr->ApplyAircraftIDMessage(packet);
            break;
        case ModeSADSBPacket::kTypeCodeSurfacePosition:      // TC = 5 (Surface Position)
        case ModeSADSBPacket::kTypeCodeSurfacePosition + 1:  // TC = 6 (Surface Position)
        case ModeSADSBPacket::kTypeCodeSurfacePosition + 2:  // TC = 7 (Surface Position)
        case ModeSADSBPacket::kTypeCodeSurfacePosition + 3:  // TC = 8 (Surface Position)
            ret = aircraft_ptr->ApplySurfacePositionMessage(packet, reference_latitude_awb32_,
                                                            reference_longitude_awb32_,
                                                            config_.enable_cpr_position_filter,
                                                            reference_position_valid_);
            break;
        case ModeSADSBPacket::kTypeCodeAirbornePositionBaroAlt:      // TC = 9 (Airborne Position w/ Baro Altitude)
        case ModeSADSBPacket::kTypeCodeAirbornePositionBaroAlt + 1:  // TC = 10 (Airborne Position w/ Baro Altitude)
        case ModeSADSBPacket::kTypeCodeAirbornePositionBaroAlt + 2:  // TC = 11 (Airborne Position w/ Baro Altitude)
        case ModeSADSBPacket::kTypeCodeAirbornePositionBaroAlt + 3:  // TC = 12 (Airborne Position w/ Baro Altitude)
        case ModeSADSBPacket::kTypeCodeAirbornePositionBaroAlt + 4:  // TC = 13 (Airborne Position w/ Baro Altitude)
        case ModeSADSBPacket::kTypeCodeAirbornePositionBaroAlt + 5:  // TC = 14 (Airborne Position w/ Baro Altitude)
        case ModeSADSBPacket::kTypeCodeAirbornePositionBaroAlt + 6:  // TC = 15 (Airborne Position w/ Baro Altitude)
        case ModeSADSBPacket::kTypeCodeAirbornePositionBaroAlt + 7:  // TC = 16 (Airborne Position w/ Baro Altitude)
        case ModeSADSBPacket::kTypeCodeAirbornePositionBaroAlt + 8:  // TC = 17 (Airborne Position w/ Baro Altitude)
        case ModeSADSBPacket::kTypeCodeAirbornePositionBaroAlt + 9:  // TC = 18 (Airborne Position w/ Baro Altitude)
        case ModeSADSBPacket::kTypeCodeAirbornePositionGNSSAlt:      // TC = 20 (Airborne Position w/ GNSS Altitude)
        case ModeSADSBPacket::kTypeCodeAirbornePositionGNSSAlt + 1:  // TC = 21 (Airborne Position w/ GNSS Altitude)
        case ModeSADSBPacket::kTypeCodeAirbornePositionGNSSAlt + 2:  // TC = 22 (Airborne Position w/ GNSS Altitude)
            ret = aircraft_ptr->ApplyAirbornePositionMessage(packet, config_.enable_cpr_position_filter);
            break;
        case ModeSADSBPacket::kTypeCodeAirborneVelocities:  // TC = 19 (Airborne Velocities)
            ret = aircraft_ptr->ApplyAirborneVelocitiesMessage(packet);
            break;
        case ModeSADSBPacket::kTypeCodeReserved:      // TC = 23 (Reserved)
        case ModeSADSBPacket::kTypeCodeReserved + 1:  // TC = 24 (Reserved)
        case ModeSADSBPacket::kTypeCodeReserved + 2:  // TC = 25 (Reserved)
        case ModeSADSBPacket::kTypeCodeReserved + 3:  // TC = 26 (Reserved)
        case ModeSADSBPacket::kTypeCodeReserved + 4:  // TC = 27 (Reserved)
            ret = true;                               // Silently ignore reserved messages.
            break;
        case ModeSADSBPacket::kTypeCodeAircraftStatus:  // TC = 28 (Aircraft Status)
            ret = aircraft_ptr->ApplyAircraftStatusMessage(packet);
            break;
        case ModeSADSBPacket::kTypeCodeTargetStateAndStatusInfo:  // TC = 29 (Target state and status info)
            ret = aircraft_ptr->ApplyTargetStateAndStatusInfoMessage(packet);
            break;
        case ModeSADSBPacket::kTypeCodeAircraftOperationStatus:  // TC = 31 (Aircraft operation status)
            ret = aircraft_ptr->ApplyAircraftOperationStatusMessage(packet);
            break;
        default:
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
            CONSOLE_WARNING("AircraftDictionary::IngestModeSADSBPacket",
                            "Received ADSB message with unsupported type code %d for ICAO 0x%lx.", type_code,
                            packet.icao_address);
#endif                    // ADSB_VERBOSE_PACKET_WARNINGS
            ret = false;  // kTypeCodeInvalid, etc.
    }
    if (ret) {
        aircraft_ptr->IncrementNumFramesReceived(true);  // Count the received Mode S frame.
    } else {
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
        CONSOLE_WARNING("AircraftDictionary::IngestModeSADSBPacket",
                        "Failed to apply ADSB message with type code %d to ICAO 0x%lx.", type_code,
                        packet.icao_address);
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
    }
    return ret;
}

bool AircraftDictionary::IngestDecodedUATADSBPacket(const DecodedUATADSBPacket& packet) {
    // Check validity. We don't record stats since those are pulled from the Sub GHz radio device status directly.
    if (!packet.is_valid) {
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
        CONSOLE_WARNING("AircraftDictionary::IngestDecodedUATADSBPacket", "Received invalid packet.");
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
        return false;
    }

    UATAircraft* aircraft_ptr =
        GetAircraftPtr<UATAircraft>(Aircraft::ICAOToUID(packet.GetICAOAddress(), Aircraft::kAircraftTypeUAT));
    if (aircraft_ptr == nullptr) {
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
        CONSOLE_WARNING("AircraftDictionary::IngestDecodedUATADSBPacket",
                        "Unable to find or create new UAT aircraft with ICAO address 0x%lx in dictionary.\r\n",
                        packet.GetICAOAddress());
#endif  // ADSB_VERBOSE_PACKET_WARNINGS
        return false;  // unable to find or create new aircraft in dictionary
    }
    aircraft_ptr->last_message_timestamp_ms = get_time_since_boot_ms();
    aircraft_ptr->last_message_signal_strength_dbm = packet.raw.sigs_dbm;
    aircraft_ptr->last_message_signal_quality_bits = packet.raw.sigq_bits;

    // Apply the packet header.

    bool ingest_ret = true;
    if (packet.has_state_vector) {
        // Apply UAT state vector to aircraft.
        ingest_ret &= aircraft_ptr->ApplyUATADSBStateVector(packet.state_vector);

        switch (aircraft_ptr->GetAddressQualifier()) {
            case UATAircraft::kTISBTargetWithICAO24BitAddress:
            case UATAircraft::kTISBTargetWithTrackFileIdentifier:
                // TIS-B target. Extract TIS-B station information.
                aircraft_ptr->tis_b_site_id = (packet.state_vector.utc_coupled_or_tis_b_site_id & 0b1111);
                break;
            default:
                // ADS-B target. Has bit indicating UTC coupled condition.
                aircraft_ptr->utc_coupled = ((packet.state_vector.utc_coupled_or_tis_b_site_id >> 3) & 0b1) != 0;
        }
    }
    if (packet.has_mode_status) {
        // Apply UAT mode status to aircraft.
        ingest_ret &= aircraft_ptr->ApplyUATADSBModeStatus(packet.mode_status);
    }
    if (packet.has_auxiliary_state_vector) {
        // Apply UAT auxiliary state vector to aircraft.
        ingest_ret &=
            aircraft_ptr->ApplyUATADSBAuxiliaryStateVector(packet.state_vector, packet.auxiliary_state_vector);
    }
    if (packet.has_target_state) {
        // Apply UAT target state to aircraft.
        ingest_ret &= aircraft_ptr->ApplyUATADSBTargetState(packet.target_state);
    }
    if (packet.has_trajectory_change) {
        // Apply UAT trajectory change to aircraft.
        ingest_ret &= aircraft_ptr->ApplyUATADSBTrajectoryChange(packet.trajectory_change);
    }

    if (ingest_ret) {
        aircraft_ptr->IncrementNumFramesReceived();
    }
    return ingest_ret;
}

uint32_t RemoteIDAircraft::MACToAddress(const uint8_t mac[6]) {
    // Derive a stable 24-bit dictionary address from the transmitter MAC. The top nibble (address qualifier) is left 0.
    return crc24(mac, 6) & 0x00FFFFFFu;
}

bool AircraftDictionary::IngestRawRemoteIDPacket(const RawRemoteIDPacket& packet) {
    metrics_counter_.raw_remote_id_frames++;

    if (packet.payload_len_bytes == 0 || packet.payload_len_bytes > RawRemoteIDPacket::kMaxPayloadLenBytes) {
        return false;
    }

    // Decode the Open Drone ID payload. ODID_UAS_Data is large (~900B) but transient; it lives on the stack only for the
    // duration of this call. Both the ESP32 (16 KB main task) and RP2040 main loop have ample stack for it.
    ODID_UAS_Data uas_data;
    odid_initUasData(&uas_data);
    if (packet.GetIsMessagePack()) {
        if (decodeMessagePack(&uas_data, reinterpret_cast<const ODID_MessagePack_encoded*>(packet.payload)) !=
            ODID_SUCCESS) {
            return false;
        }
    } else {
        // A single 25-byte Open Drone ID message. decodeOpenDroneID populates the matching field + valid flag.
        if (decodeOpenDroneID(&uas_data, packet.payload) == ODID_MESSAGETYPE_INVALID) {
            return false;
        }
    }

    uint32_t address = RemoteIDAircraft::MACToAddress(packet.source_mac);
    RemoteIDAircraft* aircraft =
        GetAircraftPtr<RemoteIDAircraft>(Aircraft::ICAOToUID(address, Aircraft::kAircraftTypeRemoteID));
    if (aircraft == nullptr) {
        CONSOLE_WARNING("AircraftDictionary::IngestRawRemoteIDPacket",
                        "Unable to find or create Remote ID aircraft for address 0x%06lx.", address);
        return false;
    }

    aircraft->last_message_timestamp_ms = get_time_since_boot_ms();
    aircraft->last_message_signal_strength_dbm = packet.rssi_dbm;
    aircraft->transport = packet.transport;
    memcpy(aircraft->source_mac, packet.source_mac, sizeof(aircraft->source_mac));

    // Basic ID: UAS serial / registration + aircraft category.
    if (uas_data.BasicIDValid[0]) {
        memcpy(aircraft->uas_id, uas_data.BasicID[0].UASID, RemoteIDAircraft::kIDLenChars);
        aircraft->uas_id[RemoteIDAircraft::kIDLenChars] = '\0';
        aircraft->uas_id_type = static_cast<uint8_t>(uas_data.BasicID[0].IDType);
        aircraft->ua_type = static_cast<uint8_t>(uas_data.BasicID[0].UAType);
        aircraft->WriteBitFlag(RemoteIDAircraft::kBitFlagBasicIDValid, true);
    }

    // Location / Vector: UA position, altitude, track, speed.
    if (uas_data.LocationValid) {
        const ODID_Location_data& loc = uas_data.Location;
        aircraft->operational_status = static_cast<uint8_t>(loc.Status);
        aircraft->WriteBitFlag(RemoteIDAircraft::kBitFlagIsAirborne, loc.Status == ODID_STATUS_AIRBORNE);

        // Position: ODID uses 0 deg for "invalid / no value". A real UA at exactly (0, 0) is not a realistic case.
        if (loc.Latitude != 0.0 || loc.Longitude != 0.0) {
            aircraft->latitude_deg = static_cast<float>(loc.Latitude);
            aircraft->longitude_deg = static_cast<float>(loc.Longitude);
            aircraft->WriteBitFlag(RemoteIDAircraft::kBitFlagPositionValid, true);
        }
        if (loc.Direction != INV_DIR && loc.Direction >= 0.0f && loc.Direction < 360.0f) {
            aircraft->direction_deg = loc.Direction;
            aircraft->WriteBitFlag(RemoteIDAircraft::kBitFlagDirectionValid, true);
        }
        if (loc.SpeedHorizontal != INV_SPEED_H) {
            aircraft->speed_kts = MpsToKts(static_cast<int>(loc.SpeedHorizontal));
            aircraft->WriteBitFlag(RemoteIDAircraft::kBitFlagHorizontalSpeedValid, true);
        }
        if (loc.SpeedVertical != INV_SPEED_V) {
            aircraft->gnss_vertical_rate_fpm = MetersToFeet(static_cast<int>(loc.SpeedVertical)) * 60;
            aircraft->WriteBitFlag(RemoteIDAircraft::kBitFlagVerticalRateValid, true);
        }
        if (loc.AltitudeGeo != INV_ALT) {
            aircraft->gnss_altitude_ft = MetersToFeet(static_cast<int>(loc.AltitudeGeo));
            aircraft->WriteBitFlag(RemoteIDAircraft::kBitFlagGNSSAltitudeValid, true);
        }
        if (loc.AltitudeBaro != INV_ALT) {
            aircraft->baro_altitude_ft = MetersToFeet(static_cast<int>(loc.AltitudeBaro));
        }
        if (loc.Height != INV_ALT) {
            aircraft->height_above_takeoff_ft = MetersToFeet(static_cast<int>(loc.Height));
            aircraft->height_type = static_cast<uint8_t>(loc.HeightType);
            aircraft->WriteBitFlag(RemoteIDAircraft::kBitFlagHeightValid, true);
        }
    }

    // System: operator (pilot) location + operator geometric altitude.
    if (uas_data.SystemValid) {
        const ODID_System_data& sys = uas_data.System;
        if (sys.OperatorLatitude != 0.0 || sys.OperatorLongitude != 0.0) {
            aircraft->operator_latitude_deg = static_cast<float>(sys.OperatorLatitude);
            aircraft->operator_longitude_deg = static_cast<float>(sys.OperatorLongitude);
            aircraft->WriteBitFlag(RemoteIDAircraft::kBitFlagOperatorPositionValid, true);
        }
        if (sys.OperatorAltitudeGeo != INV_ALT) {
            aircraft->operator_altitude_geo_ft = MetersToFeet(static_cast<int>(sys.OperatorAltitudeGeo));
        }
    }

    // Operator ID: operator registration string.
    if (uas_data.OperatorIDValid) {
        memcpy(aircraft->operator_id, uas_data.OperatorID.OperatorId, RemoteIDAircraft::kIDLenChars);
        aircraft->operator_id[RemoteIDAircraft::kIDLenChars] = '\0';
        aircraft->WriteBitFlag(RemoteIDAircraft::kBitFlagOperatorIDValid, true);
    }

    metrics_counter_.valid_remote_id_frames++;
    aircraft->IncrementNumFramesReceived();
    return true;
}

bool AircraftDictionary::RemoveAircraft(uint32_t uid) {
    auto itr = dict.find(uid);
    if (itr != dict.end()) {
        dict.erase(itr);
        return true;
    }
    CONSOLE_WARNING("AircraftDictionary::RemoveAircraft",
                    "Attempted to remove aircraft with UID 0x%lx, but it was not found in the dictionary.", uid);
    return false;  // aircraft was not found in the dictionary
}

bool AircraftDictionary::RemoveOldestAircraft() {
    if (dict.empty()) {
        return false;
    }
    auto oldest_it = dict.begin();
    uint32_t oldest_timestamp = std::visit(
        [](Aircraft& aircraft) -> uint32_t { return aircraft.last_message_timestamp_ms; }, oldest_it->second);
    for (auto it = std::next(oldest_it); it != dict.end(); ++it) {
        uint32_t ts =
            std::visit([](Aircraft& aircraft) -> uint32_t { return aircraft.last_message_timestamp_ms; }, it->second);
        if (ts < oldest_timestamp) {
            oldest_timestamp = ts;
            oldest_it = it;
        }
    }
    if (lowest_aircraft_entry == &oldest_it->second) {
        lowest_aircraft_entry = nullptr;
    }
    dict.erase(oldest_it);
    return true;
}

void AircraftDictionary::SetReferencePosition(float latitude_deg, float longitude_deg) {
    reference_position_valid_ = true;
    reference_latitude_awb32_ = lat2awb(latitude_deg);
    // Remap longitudes from (-180,180) to (0, 360)
    if (longitude_deg < 0.0f) {
        longitude_deg += 360.0f;
    }
    reference_longitude_awb32_ = lon2awb(longitude_deg);
}

bool ModeSAircraft::SetCPRLatLon(uint32_t n_lat_cpr, uint32_t n_lon_cpr, bool odd, uint32_t received_timestamp_ms) {
    if (n_lat_cpr > kCPRLatLonMaxCount || n_lon_cpr > kCPRLatLonMaxCount) {
        CONSOLE_ERROR("ModeSAircraft::SetCPRLatLon",
                      "Received CPR packet with out of bounds lat/lon values (%lu, %lu).", n_lat_cpr, n_lon_cpr);
        return false;  // counts out of bounds, don't parse
    }
    CPRPacket& complementary_packet = odd ? last_even_packet_ : last_odd_packet_;
    uint32_t received_timestamp_delta_ms = received_timestamp_ms > complementary_packet.received_timestamp_ms
                                               ? received_timestamp_ms - complementary_packet.received_timestamp_ms
                                               : complementary_packet.received_timestamp_ms - received_timestamp_ms;
    if (received_timestamp_delta_ms > GetMaxAllowedCPRIntervalMs()) {
        // Clear out old packet to avoid an invalid decode from packets that are too far apart in time.
        ClearCPRPackets();
    }

    CPRPacket& packet = odd ? last_odd_packet_ : last_even_packet_;
    // NOTE: Packet received timestamps are from the MLAT timer.
    packet.received_timestamp_ms = received_timestamp_ms;
    packet.n_lat = n_lat_cpr;
    packet.n_lon = n_lon_cpr;
    return true;
}

/**
 * Private functions and associated helpers.
 */
