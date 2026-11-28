#include "flarm_nmea.hh"

#include <cstdio>

#include "buffer_utils.hh"  // WriteTenths
#include "gdl90_utils.hh"
#include "macros.hh"            // Clamp, DivRound
#include "unit_conversions.hh"  // FeetToMetersRounded

namespace {
constexpr int32_t kMaxClimbTenthsMps = 327;  // PFLAA <ClimbRate> range, 32.7 m/s.
}  // namespace

uint16_t FLARMNMEAReporter::Finish(char* buf, uint16_t buf_len, int body_len) {
    // "*hh\r\n" is 5 characters.
    if (body_len <= 0 || body_len + 5 >= buf_len || body_len + 5 > kMaxSentenceLenBytes) return 0;
    uint8_t checksum = 0;
    for (int i = 1; i < body_len; i++) checksum ^= static_cast<uint8_t>(buf[i]);
    snprintf(buf + body_len, buf_len - body_len, "*%02X\r\n", checksum);
    return static_cast<uint16_t>(body_len + 5);
}

bool FLARMNMEAReporter::OwnshipFromRxPosition(Ownship& own, const SettingsManager::RxPosition& rx_position,
                                              bool rx_position_available, bool geoid_separation_valid,
                                              int32_t geoid_separation_cm) {
    own = {};
    if (!rx_position_available || !GDL90Reporter::IsValidOwnshipSource(rx_position.source)) return false;
    own.lat_awb = rx_position.latitude_awb;
    own.lon_awb = rx_position.longitude_awb;
    if (rx_position.source == SettingsManager::RxPosition::kPositionSourceGNSS) {
        own.msl_altitude_valid = true;
        own.msl_altitude_ft = rx_position.gnss_altitude_ft;
        // HAE, comparable with ADS-B and Remote ID geometric altitudes; none without the geoid separation.
        own.geometric_altitude_valid = GDL90Reporter::GNSSHeightAboveEllipsoidFt(
            rx_position.gnss_altitude_ft, geoid_separation_valid, geoid_separation_cm, own.geometric_altitude_ft);
        if (own.geometric_altitude_valid) {
            own.geoid_separation_valid = true;
            own.geoid_separation_cm = geoid_separation_cm;
        }
    }
    own.track_deg = rx_position.heading_deg;
    own.speed_kts = rx_position.speed_kts;
    own.airborne = rx_position.speed_kts > GDL90Reporter::kOwnshipAirborneSpeedKts;
    return true;
}

template <typename T>
void FLARMNMEAReporter::FillOwnshipAltitudes(Ownship& own, const T& aircraft) {
    own.baro_altitude_valid = aircraft.HasBitFlag(T::kBitFlagBaroAltitudeValid);
    own.baro_altitude_ft = aircraft.baro_altitude_ft;
    own.geometric_altitude_valid = aircraft.HasBitFlag(T::kBitFlagGNSSAltitudeValid);
    own.geometric_altitude_ft = aircraft.gnss_altitude_ft;
}

void FLARMNMEAReporter::OwnshipAltitudesFromAircraft(Ownship& own, const ModeSAircraft& aircraft) {
    FillOwnshipAltitudes(own, aircraft);
}

void FLARMNMEAReporter::OwnshipAltitudesFromAircraft(Ownship& own, const UATAircraft& aircraft) {
    FillOwnshipAltitudes(own, aircraft);
}

template <typename T>
void FLARMNMEAReporter::FillMotion(Target& target, const T& aircraft) {
    target.distance_valid = aircraft.distance_m != Aircraft::kDistanceUnknown;
    target.distance_m = aircraft.distance_m;
    target.bearing_bam16 = aircraft.bearing_bam16;
    target.baro_altitude_valid = aircraft.HasBitFlag(T::kBitFlagBaroAltitudeValid);
    target.baro_altitude_ft = aircraft.baro_altitude_ft;
    target.geometric_altitude_valid = aircraft.HasBitFlag(T::kBitFlagGNSSAltitudeValid);
    target.geometric_altitude_ft = aircraft.gnss_altitude_ft;
    target.track_valid = aircraft.HasBitFlag(T::kBitFlagDirectionValid);
    target.track_deg = aircraft.direction_deg;
    target.speed_valid = aircraft.HasBitFlag(T::kBitFlagHorizontalSpeedValid);
    target.speed_kts = aircraft.speed_kts;
    if (aircraft.HasBitFlag(T::kBitFlagBaroVerticalRateValid)) {
        target.vertical_rate_valid = true;
        target.vertical_rate_fpm = aircraft.baro_vertical_rate_fpm;
    } else if (aircraft.HasBitFlag(T::kBitFlagGNSSVerticalRateValid)) {
        target.vertical_rate_valid = true;
        target.vertical_rate_fpm = aircraft.gnss_vertical_rate_fpm;
    }
    target.type = AircraftTypeFromEmitterCategory(aircraft.emitter_category);
    target.rssi_valid = true;
    target.rssi_dbm = aircraft.last_message_signal_strength_dbm;
}

bool FLARMNMEAReporter::TargetFromAircraft(Target& target, const ModeSAircraft& aircraft) {
    target = {};
    if (!aircraft.HasBitFlag(ModeSAircraft::kBitFlagPositionValid)) return false;
    target.id = aircraft.icao_address & 0xFFFFFF;
    switch (aircraft.GetAddressType()) {
        case ModeSAircraft::kAddressTypeADSBICAO:
        case ModeSAircraft::kAddressTypeADSBICAONonTransponder:
            break;
        case ModeSAircraft::kAddressTypeADSRICAO:
            target.source = kSourceADSR;
            break;
        case ModeSAircraft::kAddressTypeTISBICAO:
            target.source = kSourceTISB;
            break;
        case ModeSAircraft::kAddressTypeADSROther:
            target.source = kSourceADSR;
            target.id_type = kIDTypeRandom;
            break;
        case ModeSAircraft::kAddressTypeTISBTrackFile:
        case ModeSAircraft::kAddressTypeTISBOther:
            target.source = kSourceTISB;
            target.id_type = kIDTypeRandom;
            break;
        default:  // Anonymous ADS-B, ground vehicles and obstacles.
            target.id_type = kIDTypeRandom;
            break;
    }
    FillMotion(target, aircraft);
    target.on_ground = aircraft.IsOnGround();
    return true;
}

bool FLARMNMEAReporter::TargetFromAircraft(Target& target, const UATAircraft& aircraft) {
    target = {};
    if (!aircraft.HasBitFlag(UATAircraft::kBitFlagPositionValid)) return false;
    target.id = aircraft.icao_address & 0xFFFFFF;
    switch (aircraft.GetAddressQualifier()) {
        case UATAircraft::kADSBTargetWithICAO24BitAddress:
            break;
        case UATAircraft::kTISBTargetWithICAO24BitAddress:
            target.source = kSourceTISB;
            break;
        case UATAircraft::kTISBTargetWithTrackFileIdentifier:
            target.source = kSourceTISB;
            target.id_type = kIDTypeRandom;
            break;
        case UATAircraft::kADSRTargetWithNonICAOAddress:
            target.source = kSourceADSR;
            target.id_type = kIDTypeRandom;
            break;
        default:  // Self-assigned, surface vehicles, beacons.
            target.id_type = kIDTypeRandom;
            break;
    }
    FillMotion(target, aircraft);
    target.on_ground = !aircraft.HasBitFlag(UATAircraft::kBitFlagIsAirborne);
    return true;
}

bool FLARMNMEAReporter::TargetFromAircraft(Target& target, const RemoteIDAircraft& aircraft) {
    target = {};
    if (!aircraft.HasBitFlag(RemoteIDAircraft::kBitFlagPositionValid)) return false;
    target.id = aircraft.address & 0xFFFFFF;
    target.id_type = kIDTypeRandom;
    target.source = kSourceNone;
    target.distance_valid = aircraft.distance_m != Aircraft::kDistanceUnknown;
    target.distance_m = aircraft.distance_m;
    target.bearing_bam16 = aircraft.bearing_bam16;
    target.geometric_altitude_valid = aircraft.HasBitFlag(RemoteIDAircraft::kBitFlagGNSSAltitudeValid);
    target.geometric_altitude_ft = aircraft.gnss_altitude_ft;
    target.track_valid = aircraft.HasBitFlag(RemoteIDAircraft::kBitFlagDirectionValid);
    target.track_deg = aircraft.direction_deg;
    target.speed_valid = aircraft.HasBitFlag(RemoteIDAircraft::kBitFlagHorizontalSpeedValid);
    target.speed_kts = aircraft.speed_kts;
    target.on_ground = !aircraft.HasBitFlag(RemoteIDAircraft::kBitFlagIsAirborne);
    target.vertical_rate_valid = aircraft.HasBitFlag(RemoteIDAircraft::kBitFlagVerticalRateValid);
    target.vertical_rate_fpm = aircraft.gnss_vertical_rate_fpm;
    target.type = kAircraftTypeUAV;
    return true;
}

bool FLARMNMEAReporter::TargetFromAircraft(Target& target, const ADSLAircraft& aircraft) {
    target = {};
    if (!aircraft.HasBitFlag(ADSLAircraft::kBitFlagPositionValid)) return false;
    target.id = aircraft.address & 0xFFFFFF;
    switch (aircraft.address_mapping) {
        case DecodedADSLPacket::kAddressMappingICAO:
            break;
        case DecodedADSLPacket::kAddressMappingFLARM:
            target.id_type = kIDTypeFLARM;
            break;
        default:
            target.id_type = kIDTypeRandom;
            break;
    }
    target.source = kSourceNone;  // FTD-012 has no ADS-L source code.
    target.distance_valid = aircraft.distance_m != Aircraft::kDistanceUnknown;
    target.distance_m = aircraft.distance_m;
    target.bearing_bam16 = aircraft.bearing_bam16;
    target.geometric_altitude_valid = aircraft.HasBitFlag(ADSLAircraft::kBitFlagGNSSAltitudeValid);
    target.geometric_altitude_ft = aircraft.gnss_altitude_ft;
    target.track_valid = aircraft.HasBitFlag(ADSLAircraft::kBitFlagDirectionValid);
    target.track_deg = aircraft.direction_deg;
    target.speed_valid = aircraft.HasBitFlag(ADSLAircraft::kBitFlagHorizontalSpeedValid);
    target.speed_kts = aircraft.speed_kts;
    target.on_ground = aircraft.IsOnGround();
    target.vertical_rate_valid = aircraft.HasBitFlag(ADSLAircraft::kBitFlagGNSSVerticalRateValid);
    target.vertical_rate_fpm = aircraft.gnss_vertical_rate_fpm;
    target.type = AircraftTypeFromEmitterCategory(aircraft.GetEmitterCategory());
    return true;
}

bool FLARMNMEAReporter::Relative(const Ownship& own, const Target& target, RelativePosition& relative) {
    if (!target.distance_valid) return false;
    int32_t delta_ft;
    if (own.baro_altitude_valid && target.baro_altitude_valid) {
        delta_ft = target.baro_altitude_ft - own.baro_altitude_ft;
    } else if (own.geometric_altitude_valid && target.geometric_altitude_valid) {
        delta_ft = target.geometric_altitude_ft - own.geometric_altitude_ft;
    } else {
        return false;
    }
    SplitNorthEastMeters(target.distance_m, target.bearing_bam16, relative.north_m, relative.east_m);
    relative.distance_m = target.distance_m;
    if (relative.north_m > kMaxRelativeHorizontalM || relative.north_m < -kMaxRelativeHorizontalM ||
        relative.east_m > kMaxRelativeHorizontalM || relative.east_m < -kMaxRelativeHorizontalM) {
        return false;
    }
    relative.vertical_m = Clamp(FeetToMetersRounded(delta_ft), -32768, 32767);
    return true;
}

FLARMNMEAReporter::AircraftType FLARMNMEAReporter::AircraftTypeFromEmitterCategory(
    ADSBTypes::EmitterCategory category) {
    switch (category) {
        case ADSBTypes::kEmitterCategoryLight:
            return kAircraftTypePiston;
        case ADSBTypes::kEmitterCategoryMedium1:
        case ADSBTypes::kEmitterCategoryMedium2:
        case ADSBTypes::kEmitterCategoryHighVortexAircraft:
        case ADSBTypes::kEmitterCategoryHeavy:
        case ADSBTypes::kEmitterCategoryHighPerformance:
            return kAircraftTypeJet;
        case ADSBTypes::kEmitterCategoryRotorcraft:
            return kAircraftTypeRotorcraft;
        case ADSBTypes::kEmitterCategoryGliderSailplane:
            return kAircraftTypeGlider;
        case ADSBTypes::kEmitterCategoryLighterThanAir:
            return kAircraftTypeBalloon;
        case ADSBTypes::kEmitterCategoryParachutistSkydiver:
            return kAircraftTypeSkydiver;
        case ADSBTypes::kEmitterCategoryUltralightHangGliderParaglider:
            return kAircraftTypeHangGlider;
        case ADSBTypes::kEmitterCategoryUnmannedAerialVehicle:
            return kAircraftTypeUAV;
        case ADSBTypes::kEmitterCategoryPointObstacle:
        case ADSBTypes::kEmitterCategoryClusterObstacle:
        case ADSBTypes::kEmitterCategoryLineObstacle:
            return kAircraftTypeStaticObstacle;
        default:
            return kAircraftTypeUnknown;
    }
}

uint16_t FLARMNMEAReporter::WritePFLAU(char* buf, uint16_t buf_len, uint16_t num_received, uint8_t gps) {
    // No alarm: bearing, vertical, distance and ID empty, as in FTD-012's "out of range" example.
    const int n = snprintf(buf, buf_len, "$PFLAU,%u,0,%u,1,0,,0,,,", num_received > 99 ? 99u : num_received,
                           gps > 2 ? 2u : static_cast<unsigned>(gps));
    return Finish(buf, buf_len, n);
}

uint16_t FLARMNMEAReporter::WritePFLAA(char* buf, uint16_t buf_len, const Ownship& own, const Target& target) {
    RelativePosition relative;
    if (!Relative(own, target, relative)) return 0;

    char track[8] = "";
    if (target.track_valid) snprintf(track, sizeof(track), "%d", RoundTrackDeg(target.track_deg));
    // FTD-012: 0 on ground, else forced above 0.
    char speed[16] = "";
    if (target.on_ground) {
        snprintf(speed, sizeof(speed), "0");
    } else if (target.speed_valid) {
        // 1 kt = 1852 / 3600 m/s.
        snprintf(speed, sizeof(speed), "%ld",
                 static_cast<long>(Clamp(DivRound(Clamp(target.speed_kts, 0, 100000) * 1852, 3600), 1, 32767)));
    }
    // XCSoar treats an empty climb rate as a stealth target.
    char climb[16] = "";
    if (target.on_ground) {
        snprintf(climb, sizeof(climb), "0.0");
    } else if (target.vertical_rate_valid) {
        // 1 fpm = 0.00508 m/s = 127 / 2500 tenths.
        const int32_t tenths = DivRound(Clamp(target.vertical_rate_fpm, -100000, 100000) * 127, 2500);
        WriteTenths(climb, sizeof(climb), Clamp(tenths, -kMaxClimbTenthsMps, kMaxClimbTenthsMps));
    }

    // <AlarmLevel>,<RelativeNorth>,<RelativeEast>,<RelativeVertical>,<IDType>,<ID>,<Track>,<TurnRate>,<GroundSpeed>,
    // <ClimbRate>,<AcftType>,<NoTrack>[,<Source>,<RSSI>]
    int n = snprintf(buf, buf_len, "$PFLAA,0,%ld,%ld,%ld,%u,%06lX,%s,,%s,%s,%X,0", static_cast<long>(relative.north_m),
                     static_cast<long>(relative.east_m), static_cast<long>(relative.vertical_m),
                     static_cast<unsigned>(target.id_type), static_cast<unsigned long>(target.id & 0xFFFFFF), track,
                     speed, climb, static_cast<unsigned>(target.type));
    if (n > 0 && n < buf_len && target.source != kSourceNone) {
        char rssi[16] = "";
        if (target.rssi_valid) snprintf(rssi, sizeof(rssi), "%d", static_cast<int>(Clamp(target.rssi_dbm, -999, 99)));
        n += snprintf(buf + n, buf_len - n, ",%u,%s", static_cast<unsigned>(target.source), rssi);
    }
    return Finish(buf, buf_len, n);
}

uint16_t FLARMNMEAReporter::WriteGPRMC(char* buf, uint16_t buf_len, const Ownship& own, const UTCTime& utc) {
    if (!utc.time_valid) return 0;
    char lat[24], lon[24];
    WriteNMEALatLon(lat, sizeof(lat), own.lat_awb, true);
    WriteNMEALatLon(lon, sizeof(lon), own.lon_awb, false);
    char date[16] = "";
    if (utc.date_valid) snprintf(date, sizeof(date), "%02u%02u%02u", utc.day, utc.month, utc.year % 100u);
    const int n = snprintf(buf, buf_len, "$GPRMC,%02u%02u%02u.00,A,%s,%s,%ld.0,%d.0,%s,,,A", utc.hour, utc.minute,
                           utc.second, lat, lon, static_cast<long>(own.speed_kts < 0 ? 0 : own.speed_kts),
                           RoundTrackDeg(own.track_deg), date);
    return Finish(buf, buf_len, n);
}

uint16_t FLARMNMEAReporter::WriteGPGGA(char* buf, uint16_t buf_len, const Ownship& own, const UTCTime& utc) {
    if (!utc.time_valid) return 0;
    char lat[24], lon[24];
    WriteNMEALatLon(lat, sizeof(lat), own.lat_awb, true);
    WriteNMEALatLon(lon, sizeof(lon), own.lon_awb, false);
    char alt[24] = "";
    if (own.msl_altitude_valid) {
        // Decimeters: 3.048 dm/ft.
        WriteTenths(alt, sizeof(alt), DivRound(Clamp(own.msl_altitude_ft, -500000, 500000) * 3048, 1000));
    }
    char sep[24] = "";
    if (own.geoid_separation_valid) {
        WriteTenths(sep, sizeof(sep), DivRound(own.geoid_separation_cm, 10));  // Decimeters.
    }
    const int n = snprintf(buf, buf_len, "$GPGGA,%02u%02u%02u.00,%s,%s,1,,,%s,M,%s,M,,", utc.hour, utc.minute,
                           utc.second, lat, lon, alt, sep);
    return Finish(buf, buf_len, n);
}

uint16_t FLARMNMEAReporter::WritePGRMZ(char* buf, uint16_t buf_len, const Ownship& own) {
    if (!own.baro_altitude_valid) return 0;
    const int n = snprintf(buf, buf_len, "$PGRMZ,%ld,F,2", static_cast<long>(own.baro_altitude_ft));
    return Finish(buf, buf_len, n);
}
