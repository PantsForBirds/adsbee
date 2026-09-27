#include "mavlink_utils.hh"

enum MAVLINKAltitudeType : uint8_t { kMAVLINKAltitudeTypeBaro = 0, kMAVLINKAltitudeTypeGNSS = 1 };

uint8_t AircraftCategoryToMAVLINKEmitterType(ADSBTypes::EmitterCategory emitter_category) {
    switch (emitter_category) {
        case ADSBTypes::kEmitterCategoryInvalid:
            CONSOLE_WARNING("comms_reporting.cc::AircraftCategoryToMAVLINKEmitterType",
                            "Encountered airframe type kEmitterCategoryInvalid.");
            return UINT8_MAX;
        case ADSBTypes::kEmitterCategoryNoCategoryInfo:
            return 0;  // ADSB_EMITTER_TYPE_NO_INFO
        case ADSBTypes::kEmitterCategoryLight:
            return 1;  // ADSB_EMITTER_TYPE_LIGHT
        case ADSBTypes::kEmitterCategoryMedium1:
            return 2;  // ADSB_EMITTER_TYPE_SMALL
        case ADSBTypes::kEmitterCategoryMedium2:
            return 3;  // ADSB_EMITTER_TYPE_LARGE
        case ADSBTypes::kEmitterCategoryHighVortexAircraft:
            return 4;  // ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE
        case ADSBTypes::kEmitterCategoryHeavy:
            return 5;  // ADSB_EMITTER_TYPE_HEAVY
        case ADSBTypes::kEmitterCategoryHighPerformance:
            return 6;  // ADSB_EMITTER_TYPE_HIGHLY_MANUV
        case ADSBTypes::kEmitterCategoryRotorcraft:
            return 7;  // ADSB_EMITTER_TYPE_ROTORCRAFT
        case ADSBTypes::kEmitterCategoryReserved:
            return 8;  // ADSB_EMITTER_TYPE_UNASSIGNED
        case ADSBTypes::kEmitterCategoryGliderSailplane:
            return 9;  // ADSB_EMITTER_TYPE_GLIDER
        case ADSBTypes::kEmitterCategoryLighterThanAir:
            return 10;  // ADSB_EMITTER_TYPE_LIGHTER_AIR
        case ADSBTypes::kEmitterCategoryParachutistSkydiver:
            return 11;  // ADSB_EMITTER_TYPE_PARACHUTE
        case ADSBTypes::kEmitterCategoryUltralightHangGliderParaglider:
            return 12;  // ADSB_EMITTER_TYPE_ULTRA_LIGHT
        // NOTE: no case for 13 = ADSB_EMITTER_TYPE_UNASSIGNED2
        case ADSBTypes::kEmitterCategoryUnmannedAerialVehicle:
            return 14;  // ADSB_EMITTER_TYPE_UAV
        case ADSBTypes::kEmitterCategorySpaceTransatmosphericVehicle:
            return 15;  // ADSB_EMITTER_TYPE_SPACE
        // NOTE: no case for 16 = ADSB_EMITTER_TYPE_UNASSIGNED3
        case ADSBTypes::kEmitterCategorySurfaceEmergencyVehicle:
            return 17;  // ADSB_EMITTER_TYPE_EMERGENCY_SURFACE
        case ADSBTypes::kEmitterCategorySurfaceServiceVehicle:
            return 18;  // ADSB_EMITTER_TYPE_SERVICE_SURFACE
        case ADSBTypes::kEmitterCategoryPointObstacle:
        case ADSBTypes::kEmitterCategoryClusterObstacle:
        case ADSBTypes::kEmitterCategoryLineObstacle:
            return 19;  // ADSB_EMITTER_TYPE_POINT_OBSTACLE
        default:
            CONSOLE_WARNING("comms_reporting.cc::AircraftCategoryToMAVLINKEmitterType",
                            "Encountered unknown airframe type %d.", emitter_category);
            return UINT8_MAX;
    }
    return UINT8_MAX;
}

mavlink_adsb_vehicle_t ModeSAircraftToMAVLINKADSBVehicleMessage(const ModeSAircraft &aircraft) {
    // Set MAVLINK flags.
    uint16_t flags = 0;
    if (aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagPositionValid)) {
        flags |= ADSB_FLAGS_VALID_COORDS;
    }
    if (aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagBaroAltitudeValid)) {
        // Aircraft is reporting barometric altitude.
        flags |= ADSB_FLAGS_BARO_VALID;
        flags |= ADSB_FLAGS_VALID_ALTITUDE;
    } else if (aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagGNSSAltitudeValid)) {
        // Aircraft is reporting GNSS altitude.
        flags |= ADSB_FLAGS_VALID_ALTITUDE;
    }
    if (aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagDirectionValid)) {
        flags |= ADSB_FLAGS_VALID_HEADING;
    }
    if (aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagHorizontalSpeedValid)) {
        flags |= ADSB_FLAGS_VALID_VELOCITY;
    }
    if (strnlen(aircraft.callsign, ModeSAircraft::kCallSignMaxNumChars) > ModeSAircraft::kCallSignMinNumChars) {
        flags |= ADSB_FLAGS_VALID_CALLSIGN;
    }
    if (aircraft.squawk != ADSBTypes::kSquawkCodeNotYetReceived) {
        flags |= ADSB_FLAGS_VALID_SQUAWK;
    }
    if (aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagBaroVerticalRateValid) ||
        aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagGNSSVerticalRateValid)) {
        flags |= ADSB_FLAGS_VERTICAL_VELOCITY_VALID;
    }

    // Initialize the message
    // Prefer GNSS altitude for MAVLINK applications (e.g. drone traffic avoidance).
    bool use_gnss_altitude = aircraft.HasBitFlag(ModeSAircraft::kBitFlagGNSSAltitudeValid);
    mavlink_adsb_vehicle_t adsb_vehicle_msg = {};
    adsb_vehicle_msg.ICAO_address = aircraft.icao_address;
    // Latitude [degE7]
    adsb_vehicle_msg.lat = static_cast<int32_t>(aircraft.latitude_deg * 1e7f);
    // Longitude [degE7]
    adsb_vehicle_msg.lon = static_cast<int32_t>(aircraft.longitude_deg * 1e7f);
    // Altitude [mm]: Prefer GNSS altitude but fall back to baro altitude.
    adsb_vehicle_msg.altitude =
        FeetToMeters(use_gnss_altitude ? aircraft.gnss_altitude_ft : aircraft.baro_altitude_ft) * 1000;
    // Heading [cdeg]
    adsb_vehicle_msg.heading = static_cast<uint16_t>(aircraft.direction_deg * 100);
    // Horizontal Velocity [cm/s]
    adsb_vehicle_msg.hor_velocity = static_cast<uint16_t>(KtsToMps(static_cast<int>(aircraft.speed_kts)) * 100);
    // Vertical Velocity [cm/s]: Prefer GNSS vertical velocity but fall back to baro vertical velocity.
    adsb_vehicle_msg.ver_velocity =
        static_cast<int16_t>(FpmToMps(aircraft.HasBitFlag(ModeSAircraft::kBitFlagGNSSVerticalRateValid)
                                           ? aircraft.gnss_vertical_rate_fpm
                                           : aircraft.baro_vertical_rate_fpm) *
                             100);
    adsb_vehicle_msg.flags = flags;
    adsb_vehicle_msg.squawk = aircraft.squawk != ADSBTypes::kSquawkCodeNotYetReceived ? aircraft.squawk : 0;
    adsb_vehicle_msg.altitude_type =
        static_cast<uint8_t>(use_gnss_altitude ? kMAVLINKAltitudeTypeGNSS : kMAVLINKAltitudeTypeBaro);
    // Fill out callsign later.
    adsb_vehicle_msg.emitter_type = AircraftCategoryToMAVLINKEmitterType(aircraft.emitter_category);
    // Time Since Last Contact [s]
    adsb_vehicle_msg.tslc =
        static_cast<uint8_t>((get_time_since_boot_ms() - aircraft.last_message_timestamp_ms) / 1000);
    // MAVLINK callsign field is 9 chars, so there's room to copy over the full 8 char callsign + null terminator.
    strncpy(adsb_vehicle_msg.callsign, aircraft.callsign, ModeSAircraft::kCallSignMaxNumChars + 1);

    return adsb_vehicle_msg;
}

mavlink_adsb_vehicle_t UATAircraftToMAVLINKADSBVehicleMessage(const UATAircraft &aircraft) {
    // Set MAVLINK flags.
    uint16_t flags = 0;
    if (aircraft.HasBitFlag(UATAircraft::BitFlag::kBitFlagPositionValid)) {
        flags |= ADSB_FLAGS_VALID_COORDS;
    }
    if (aircraft.HasBitFlag(UATAircraft::BitFlag::kBitFlagBaroAltitudeValid)) {
        // Aircraft is reporting barometric altitude.
        flags |= ADSB_FLAGS_BARO_VALID;
        flags |= ADSB_FLAGS_VALID_ALTITUDE;
    } else if (aircraft.HasBitFlag(UATAircraft::BitFlag::kBitFlagGNSSAltitudeValid)) {
        // Aircraft is reporting GNSS altitude.
        flags |= ADSB_FLAGS_VALID_ALTITUDE;
    }
    if (aircraft.HasBitFlag(UATAircraft::BitFlag::kBitFlagDirectionValid)) {
        flags |= ADSB_FLAGS_VALID_HEADING;
    }
    if (aircraft.HasBitFlag(UATAircraft::BitFlag::kBitFlagHorizontalSpeedValid)) {
        flags |= ADSB_FLAGS_VALID_VELOCITY;
    }
    if (strnlen(aircraft.callsign, UATAircraft::kCallSignMaxNumChars) > UATAircraft::kCallSignMinNumChars) {
        flags |= ADSB_FLAGS_VALID_CALLSIGN;
    }
    if (aircraft.squawk != ADSBTypes::kSquawkCodeNotYetReceived) {
        flags |= ADSB_FLAGS_VALID_SQUAWK;
    }
    if (aircraft.HasBitFlag(UATAircraft::BitFlag::kBitFlagBaroVerticalRateValid) ||
        aircraft.HasBitFlag(UATAircraft::BitFlag::kBitFlagGNSSVerticalRateValid)) {
        flags |= ADSB_FLAGS_VERTICAL_VELOCITY_VALID;
    }
    flags |= ADSB_FLAGS_SOURCE_UAT;

    // Initialize the message
    // Prefer GNSS altitude for MAVLINK applications (e.g. drone traffic avoidance).
    bool use_gnss_altitude = aircraft.HasBitFlag(UATAircraft::kBitFlagGNSSAltitudeValid);
    mavlink_adsb_vehicle_t adsb_vehicle_msg = {};
    adsb_vehicle_msg.ICAO_address = aircraft.icao_address;
    // Latitude [degE7]
    adsb_vehicle_msg.lat = static_cast<int32_t>(aircraft.latitude_deg * 1e7f);
    // Longitude [degE7]
    adsb_vehicle_msg.lon = static_cast<int32_t>(aircraft.longitude_deg * 1e7f);
    // Altitude [mm]: Prefer GNSS altitude but fall back to baro altitude.
    adsb_vehicle_msg.altitude =
        FeetToMeters(use_gnss_altitude ? aircraft.gnss_altitude_ft : aircraft.baro_altitude_ft) * 1000;
    // Heading [cdeg]
    adsb_vehicle_msg.heading = static_cast<uint16_t>(aircraft.direction_deg * 100);
    // Horizontal Velocity [cm/s]
    adsb_vehicle_msg.hor_velocity = static_cast<uint16_t>(KtsToMps(static_cast<int>(aircraft.speed_kts)) * 100);
    // Vertical Velocity [cm/s]: Prefer GNSS vertical velocity but fall back to baro vertical velocity.
    adsb_vehicle_msg.ver_velocity =
        static_cast<int16_t>(FpmToMps(aircraft.HasBitFlag(UATAircraft::kBitFlagGNSSVerticalRateValid)
                                           ? aircraft.gnss_vertical_rate_fpm
                                           : aircraft.baro_vertical_rate_fpm) *
                             100);
    adsb_vehicle_msg.flags = flags;
    adsb_vehicle_msg.squawk = aircraft.squawk != ADSBTypes::kSquawkCodeNotYetReceived ? aircraft.squawk : 0;
    adsb_vehicle_msg.altitude_type =
        static_cast<uint8_t>(use_gnss_altitude ? kMAVLINKAltitudeTypeGNSS : kMAVLINKAltitudeTypeBaro);
    // Fill out callsign later.
    adsb_vehicle_msg.emitter_type = AircraftCategoryToMAVLINKEmitterType(aircraft.emitter_category);
    // Time Since Last Contact [s]
    adsb_vehicle_msg.tslc =
        static_cast<uint8_t>((get_time_since_boot_ms() - aircraft.last_message_timestamp_ms) / 1000);
    // MAVLINK callsign field is 9 chars, so there's room to copy over the full 8 char callsign + null terminator.
    strncpy(adsb_vehicle_msg.callsign, aircraft.callsign, UATAircraft::kCallSignMaxNumChars + 1);

    return adsb_vehicle_msg;
}