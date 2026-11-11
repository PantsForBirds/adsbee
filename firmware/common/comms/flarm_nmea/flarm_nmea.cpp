#include "flarm_nmea.hh"

#include <cmath>
#include <cstdio>

#include "gdl90_utils.hh"

namespace {
constexpr double kEarthRadiusM = 6371008.8;  // Mean radius.
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kFtToM = 0.3048;
constexpr double kKtsToMps = 0.514444;
constexpr double kFpmToMps = 0.00508;
constexpr double kMaxClimbMps = 32.7;  // PFLAA <ClimbRate> range.

int32_t Clamp(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Whole degrees, 0-359.
int TrackDeg(float track_deg) {
    int t = static_cast<int>(lroundf(track_deg)) % 360;
    return t < 0 ? t + 360 : t;
}

// "ddmm.mmmm,N" or "dddmm.mmmm,E".
void WriteLatLon(char* buf, size_t len, double deg, bool is_lat) {
    const char hemisphere = is_lat ? (deg < 0 ? 'S' : 'N') : (deg < 0 ? 'W' : 'E');
    // Ten-thousandths of a minute, rounded once so the carry reaches the degrees.
    const long units = lround(fabs(deg) * 60.0 * 10000.0);
    const long whole_deg = units / (60L * 10000L);
    const long rem = units % (60L * 10000L);
    snprintf(buf, len, is_lat ? "%02ld%02ld.%04ld,%c" : "%03ld%02ld.%04ld,%c", whole_deg, rem / 10000, rem % 10000,
             hemisphere);
}
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
                                              bool rx_position_available) {
    own = {};
    if (!rx_position_available || !GDL90Reporter::IsValidOwnshipSource(rx_position.source)) return false;
    own.latitude_deg = rx_position.latitude_deg;
    own.longitude_deg = rx_position.longitude_deg;
    if (rx_position.source == SettingsManager::RxPosition::kPositionSourceGNSS) {
        // A GNSS fix has mean sea level altitude and no baro.
        own.gnss_altitude_valid = true;
        own.gnss_altitude_ft = rx_position.gnss_altitude_ft;
    } else {
        // Our own ADS-B: compare its baro altitude with the traffic's.
        own.baro_altitude_valid = true;
        own.baro_altitude_ft = rx_position.baro_altitude_ft;
    }
    own.track_deg = rx_position.heading_deg;
    own.speed_kts = rx_position.speed_kts;
    own.airborne = rx_position.speed_kts > GDL90Reporter::kOwnshipAirborneSpeedKts;
    return true;
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
    target.latitude_deg = aircraft.latitude_deg;
    target.longitude_deg = aircraft.longitude_deg;
    target.baro_altitude_valid = aircraft.HasBitFlag(ModeSAircraft::kBitFlagBaroAltitudeValid);
    target.baro_altitude_ft = aircraft.baro_altitude_ft;
    target.gnss_altitude_valid = aircraft.HasBitFlag(ModeSAircraft::kBitFlagGNSSAltitudeValid);
    target.gnss_altitude_ft = aircraft.gnss_altitude_ft;
    target.track_valid = aircraft.HasBitFlag(ModeSAircraft::kBitFlagDirectionValid);
    target.track_deg = aircraft.direction_deg;
    target.speed_valid = aircraft.HasBitFlag(ModeSAircraft::kBitFlagHorizontalSpeedValid);
    target.speed_kts = aircraft.speed_kts;
    target.on_ground = aircraft.IsOnGround();
    if (aircraft.HasBitFlag(ModeSAircraft::kBitFlagBaroVerticalRateValid)) {
        target.vertical_rate_valid = true;
        target.vertical_rate_fpm = aircraft.baro_vertical_rate_fpm;
    } else if (aircraft.HasBitFlag(ModeSAircraft::kBitFlagGNSSVerticalRateValid)) {
        target.vertical_rate_valid = true;
        target.vertical_rate_fpm = aircraft.gnss_vertical_rate_fpm;
    }
    target.type = AircraftTypeFromEmitterCategory(aircraft.emitter_category);
    target.rssi_valid = true;
    target.rssi_dbm = aircraft.last_message_signal_strength_dbm;
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
    target.latitude_deg = aircraft.latitude_deg;
    target.longitude_deg = aircraft.longitude_deg;
    target.baro_altitude_valid = aircraft.HasBitFlag(UATAircraft::kBitFlagBaroAltitudeValid);
    target.baro_altitude_ft = aircraft.baro_altitude_ft;
    target.gnss_altitude_valid = aircraft.HasBitFlag(UATAircraft::kBitFlagGNSSAltitudeValid);
    target.gnss_altitude_ft = aircraft.gnss_altitude_ft;
    target.track_valid = aircraft.HasBitFlag(UATAircraft::kBitFlagDirectionValid);
    target.track_deg = aircraft.direction_deg;
    target.speed_valid = aircraft.HasBitFlag(UATAircraft::kBitFlagHorizontalSpeedValid);
    target.speed_kts = aircraft.speed_kts;
    target.on_ground = !aircraft.HasBitFlag(UATAircraft::kBitFlagIsAirborne);
    if (aircraft.HasBitFlag(UATAircraft::kBitFlagBaroVerticalRateValid)) {
        target.vertical_rate_valid = true;
        target.vertical_rate_fpm = aircraft.baro_vertical_rate_fpm;
    } else if (aircraft.HasBitFlag(UATAircraft::kBitFlagGNSSVerticalRateValid)) {
        target.vertical_rate_valid = true;
        target.vertical_rate_fpm = aircraft.gnss_vertical_rate_fpm;
    }
    target.type = AircraftTypeFromEmitterCategory(aircraft.emitter_category);
    target.rssi_valid = true;
    target.rssi_dbm = aircraft.last_message_signal_strength_dbm;
    return true;
}

bool FLARMNMEAReporter::TargetFromAircraft(Target& target, const RemoteIDAircraft& aircraft) {
    target = {};
    if (!aircraft.HasBitFlag(RemoteIDAircraft::kBitFlagPositionValid)) return false;
    target.id = aircraft.address & 0xFFFFFF;
    target.id_type = kIDTypeRandom;
    target.source = kSourceNone;
    target.latitude_deg = aircraft.latitude_deg;
    target.longitude_deg = aircraft.longitude_deg;
    target.gnss_altitude_valid = aircraft.HasBitFlag(RemoteIDAircraft::kBitFlagGNSSAltitudeValid);
    target.gnss_altitude_ft = aircraft.gnss_altitude_ft;
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

bool FLARMNMEAReporter::Relative(const Ownship& own, const Target& target, int32_t& north_m, int32_t& east_m,
                                 int32_t& vertical_m) {
    int32_t delta_ft;
    if (own.baro_altitude_valid && target.baro_altitude_valid) {
        delta_ft = target.baro_altitude_ft - own.baro_altitude_ft;
    } else if (own.gnss_altitude_valid && target.gnss_altitude_valid) {
        delta_ft = target.gnss_altitude_ft - own.gnss_altitude_ft;
    } else if (own.gnss_altitude_valid && target.baro_altitude_valid) {
        delta_ft = target.baro_altitude_ft - own.gnss_altitude_ft;
    } else {
        return false;
    }
    // Local tangent plane at the midpoint latitude: within a few meters at FLARM ranges.
    double dlon_deg = target.longitude_deg - own.longitude_deg;
    if (dlon_deg > 180.0) dlon_deg -= 360.0;
    if (dlon_deg < -180.0) dlon_deg += 360.0;
    const double dlat_rad = (target.latitude_deg - own.latitude_deg) * kDegToRad;
    const double lat_mid_rad = own.latitude_deg * kDegToRad + dlat_rad / 2.0;
    const double north = dlat_rad * kEarthRadiusM;
    const double east = dlon_deg * kDegToRad * kEarthRadiusM * cos(lat_mid_rad);
    if (fabs(north) > kMaxRelativeHorizontalM || fabs(east) > kMaxRelativeHorizontalM) return false;
    north_m = static_cast<int32_t>(lround(north));
    east_m = static_cast<int32_t>(lround(east));
    vertical_m = Clamp(static_cast<int32_t>(lround(delta_ft * kFtToM)), -32768, 32767);
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
    int32_t north_m, east_m, vertical_m;
    if (!Relative(own, target, north_m, east_m, vertical_m)) return 0;

    char track[8] = "";
    if (target.track_valid) snprintf(track, sizeof(track), "%d", TrackDeg(target.track_deg));
    // FTD-012: 0 on ground, else forced above 0.
    char speed[16] = "";
    if (target.on_ground) {
        snprintf(speed, sizeof(speed), "0");
    } else if (target.speed_valid) {
        snprintf(speed, sizeof(speed), "%d",
                 static_cast<int>(Clamp(static_cast<int32_t>(lround(target.speed_kts * kKtsToMps)), 1, 32767)));
    }
    // XCSoar treats an empty climb rate as a stealth target.
    char climb[16] = "";
    if (target.on_ground) {
        snprintf(climb, sizeof(climb), "0.0");
    } else if (target.vertical_rate_valid) {
        double mps = target.vertical_rate_fpm * kFpmToMps;
        mps = mps > kMaxClimbMps ? kMaxClimbMps : (mps < -kMaxClimbMps ? -kMaxClimbMps : mps);
        const long tenths = lround(mps * 10.0);
        snprintf(climb, sizeof(climb), "%s%ld.%ld", tenths < 0 ? "-" : "", labs(tenths) / 10, labs(tenths) % 10);
    }

    // <AlarmLevel>,<RelativeNorth>,<RelativeEast>,<RelativeVertical>,<IDType>,<ID>,<Track>,<TurnRate>,<GroundSpeed>,
    // <ClimbRate>,<AcftType>,<NoTrack>[,<Source>,<RSSI>]
    int n = snprintf(buf, buf_len, "$PFLAA,0,%ld,%ld,%ld,%u,%06lX,%s,,%s,%s,%X,0", static_cast<long>(north_m),
                     static_cast<long>(east_m), static_cast<long>(vertical_m), static_cast<unsigned>(target.id_type),
                     static_cast<unsigned long>(target.id & 0xFFFFFF), track, speed, climb,
                     static_cast<unsigned>(target.type));
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
    WriteLatLon(lat, sizeof(lat), own.latitude_deg, true);
    WriteLatLon(lon, sizeof(lon), own.longitude_deg, false);
    char date[16] = "";
    if (utc.date_valid) snprintf(date, sizeof(date), "%02u%02u%02u", utc.day, utc.month, utc.year % 100u);
    const int n = snprintf(buf, buf_len, "$GPRMC,%02u%02u%02u.00,A,%s,%s,%ld.0,%d.0,%s,,,A", utc.hour, utc.minute,
                           utc.second, lat, lon, static_cast<long>(own.speed_kts < 0 ? 0 : own.speed_kts),
                           TrackDeg(own.track_deg), date);
    return Finish(buf, buf_len, n);
}

uint16_t FLARMNMEAReporter::WriteGPGGA(char* buf, uint16_t buf_len, const Ownship& own, const UTCTime& utc) {
    if (!utc.time_valid) return 0;
    char lat[24], lon[24];
    WriteLatLon(lat, sizeof(lat), own.latitude_deg, true);
    WriteLatLon(lon, sizeof(lon), own.longitude_deg, false);
    char alt[24] = "";
    if (own.gnss_altitude_valid) {
        const long decimeters = lround(own.gnss_altitude_ft * kFtToM * 10.0);
        snprintf(alt, sizeof(alt), "%s%ld.%ld", decimeters < 0 ? "-" : "", labs(decimeters) / 10, labs(decimeters) % 10);
    }
    const int n = snprintf(buf, buf_len, "$GPGGA,%02u%02u%02u.00,%s,%s,1,,,%s,M,,M,,", utc.hour, utc.minute,
                           utc.second, lat, lon, alt);
    return Finish(buf, buf_len, n);
}

uint16_t FLARMNMEAReporter::WritePGRMZ(char* buf, uint16_t buf_len, const Ownship& own) {
    if (!own.baro_altitude_valid) return 0;
    const int n = snprintf(buf, buf_len, "$PGRMZ,%ld,F,2", static_cast<long>(own.baro_altitude_ft));
    return Finish(buf, buf_len, n);
}
