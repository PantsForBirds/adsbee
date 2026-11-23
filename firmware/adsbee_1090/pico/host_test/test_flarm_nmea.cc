#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "awb_test_utils.hh"
#include "flarm_nmea.hh"
#include "gtest/gtest.h"

namespace {

// Checks framing and checksum (FTD-012 section 7) and returns the sentence without "*hh\r\n".
std::string Body(const char* sentence, uint16_t len) {
    EXPECT_GT(len, 7);
    EXPECT_LE(len, FLARMNMEAReporter::kMaxSentenceLenBytes);
    EXPECT_EQ(strlen(sentence), len);
    EXPECT_EQ(sentence[0], '$');
    EXPECT_EQ(sentence[len - 5], '*');
    EXPECT_EQ(sentence[len - 2], '\r');
    EXPECT_EQ(sentence[len - 1], '\n');
    uint8_t checksum = 0;
    for (uint16_t i = 1; i < len - 5; i++) checksum ^= static_cast<uint8_t>(sentence[i]);
    char hex[3];
    snprintf(hex, sizeof(hex), "%02X", checksum);
    EXPECT_EQ(std::string(sentence + len - 4, 2), hex);
    return std::string(sentence, len - 5);
}

// A target's distance and bearing from ownship, as the dictionary stores them for an aircraft at lat, lon.
void Place(FLARMNMEAReporter::Target& t, const FLARMNMEAReporter::Ownship& own, double lat_deg, double lon_deg) {
    t.distance_valid = true;
    CalculateDistanceAndBearing(GeoReference(own.lat_awb, own.lon_awb), DegToAWB(lat_deg), DegToAWB(lon_deg),
                                t.distance_m, t.bearing_bam16);
}

// Ownship with a geometric altitude (a tracked aircraft's own ADS-B).
FLARMNMEAReporter::Ownship Own() {
    FLARMNMEAReporter::Ownship own;
    own.lat_awb = DegToAWB(37.4);
    own.lon_awb = DegToAWB(-122.1);
    own.geometric_altitude_valid = true;
    own.geometric_altitude_ft = 1000;
    own.track_deg = 271.6f;
    own.speed_kts = 95;
    own.airborne = true;
    return own;
}

FLARMNMEAReporter::Target Jet() {
    FLARMNMEAReporter::Target t;
    t.id = 0xA1B2C3;
    Place(t, Own(), 37.45, -122.0);
    t.baro_altitude_valid = true;
    t.baro_altitude_ft = 3500;
    t.geometric_altitude_valid = true;
    t.geometric_altitude_ft = 3600;
    t.track_valid = true;
    t.track_deg = 359.7f;
    t.speed_valid = true;
    t.speed_kts = 250;
    t.vertical_rate_valid = true;
    t.vertical_rate_fpm = -640;
    t.type = FLARMNMEAReporter::kAircraftTypeJet;
    t.rssi_valid = true;
    t.rssi_dbm = -71;
    return t;
}

}  // namespace

TEST(FLARMNMEA, PFLAUHeartbeatWithoutAlarm) {
    char buf[100];
    uint16_t len = FLARMNMEAReporter::WritePFLAU(buf, sizeof(buf), 2, 2);
    EXPECT_EQ(Body(buf, len), "$PFLAU,2,0,2,1,0,,0,,,");
    // No ownship: GPS 0. RX clamps to 99.
    len = FLARMNMEAReporter::WritePFLAU(buf, sizeof(buf), 150, 0);
    EXPECT_EQ(Body(buf, len), "$PFLAU,99,0,0,1,0,,0,,,");
}

TEST(FLARMNMEA, PFLAAMatchesFTD012FieldOrder) {
    char buf[100];
    const uint16_t len = FLARMNMEAReporter::WritePFLAA(buf, sizeof(buf), Own(), Jet());
    // 5554 m north, 8848 m east (GeographicLib: 5553.97, 8848.44); geometric against geometric: 2600 ft = 792 m above;
    // track 359.7 -> 0; 250 kt = 129 m/s; -640 fpm = -3.3 m/s; jet; no-track 0; source ADS-B; RSSI.
    EXPECT_EQ(Body(buf, len), "$PFLAA,0,5554,8849,792,1,A1B2C3,0,,129,-3.3,9,0,1,-71");
}

TEST(FLARMNMEA, PFLAAEmptyFieldsAndGroundTargets) {
    char buf[100];
    FLARMNMEAReporter::Target t = Jet();
    t.track_valid = false;
    t.speed_valid = false;
    t.vertical_rate_valid = false;
    t.rssi_valid = false;
    t.id_type = FLARMNMEAReporter::kIDTypeRandom;
    t.source = FLARMNMEAReporter::kSourceTISB;
    uint16_t len = FLARMNMEAReporter::WritePFLAA(buf, sizeof(buf), Own(), t);
    EXPECT_EQ(Body(buf, len), "$PFLAA,0,5554,8849,792,0,A1B2C3,,,,,9,0,4,");

    // On ground: speed and climb 0.
    t.on_ground = true;
    len = FLARMNMEAReporter::WritePFLAA(buf, sizeof(buf), Own(), t);
    EXPECT_EQ(Body(buf, len), "$PFLAA,0,5554,8849,792,0,A1B2C3,,,0,0.0,9,0,4,");

    // No FTD-012 source (Remote ID): Source and RSSI omitted.
    t.source = FLARMNMEAReporter::kSourceNone;
    len = FLARMNMEAReporter::WritePFLAA(buf, sizeof(buf), Own(), t);
    EXPECT_EQ(Body(buf, len), "$PFLAA,0,5554,8849,792,0,A1B2C3,,,0,0.0,9,0");
}

TEST(FLARMNMEA, PFLAAMovingSpeedAndClimbLimits) {
    char buf[100];
    FLARMNMEAReporter::Target t = Jet();
    t.speed_kts = 0;  // Moving but reporting 0: FTD-012 forces > 0.
    t.vertical_rate_fpm = 99999;
    uint16_t len = FLARMNMEAReporter::WritePFLAA(buf, sizeof(buf), Own(), t);
    EXPECT_EQ(Body(buf, len), "$PFLAA,0,5554,8849,792,1,A1B2C3,0,,1,32.7,9,0,1,-71");
    t.vertical_rate_fpm = -5;  // -0.03 m/s rounds to 0.0, no "-0.0".
    len = FLARMNMEAReporter::WritePFLAA(buf, sizeof(buf), Own(), t);
    EXPECT_EQ(Body(buf, len), "$PFLAA,0,5554,8849,792,1,A1B2C3,0,,1,0.0,9,0,1,-71");
}

TEST(FLARMNMEA, RelativeAltitudeReference) {
    FLARMNMEAReporter::RelativePosition r;
    FLARMNMEAReporter::Ownship own = Own();
    FLARMNMEAReporter::Target t = Jet();
    // Ownship with baro and geometric altitudes (its own ADS-B): baro against baro first.
    own.baro_altitude_valid = true;
    own.baro_altitude_ft = 1500;
    ASSERT_TRUE(FLARMNMEAReporter::Relative(own, t, r));
    EXPECT_EQ(r.vertical_m, 610);  // 2000 ft.
    // Target without baro: geometric against geometric.
    t.baro_altitude_valid = false;
    ASSERT_TRUE(FLARMNMEAReporter::Relative(own, t, r));
    EXPECT_EQ(r.vertical_m, 792);  // 2600 ft.
    // Baro is never compared with geometric: target geometric only, ownship baro only.
    own.geometric_altitude_valid = false;
    EXPECT_FALSE(FLARMNMEAReporter::Relative(own, t, r));
    // Target baro only, ownship geometric only.
    own.baro_altitude_valid = false;
    own.geometric_altitude_valid = true;
    t.baro_altitude_valid = true;
    t.geometric_altitude_valid = false;
    EXPECT_FALSE(FLARMNMEAReporter::Relative(own, t, r));
    char buf[100];
    EXPECT_EQ(FLARMNMEAReporter::WritePFLAA(buf, sizeof(buf), own, t), 0);
    // A GNSS fix's mean sea level altitude is compared with nothing (traffic geometric altitudes are above the
    // ellipsoid).
    own.geometric_altitude_valid = false;
    own.msl_altitude_valid = true;
    own.msl_altitude_ft = 1000;
    t.geometric_altitude_valid = true;
    EXPECT_FALSE(FLARMNMEAReporter::Relative(own, t, r));
    // Rounding: 1 ft = 0.3 m -> 0; 2 ft = 0.6 m -> 1; symmetric below.
    own = Own();
    t = Jet();
    t.baro_altitude_valid = false;
    for (const auto& c :
         {std::pair<int32_t, int32_t>{1, 0}, {2, 1}, {-2, -1}, {-1, 0}, {120000, 32767}, {-120000, -32768}}) {
        t.geometric_altitude_ft = own.geometric_altitude_ft + c.first;
        ASSERT_TRUE(FLARMNMEAReporter::Relative(own, t, r));
        EXPECT_EQ(r.vertical_m, c.second) << c.first;
    }
}

TEST(FLARMNMEA, GNSSOwnshipWithGeoidSeparationGetsPFLAA) {
    SettingsManager::RxPosition rx;
    rx.source = SettingsManager::RxPosition::kPositionSourceGNSS;
    rx.latitude_deg = 37.4f;
    rx.longitude_deg = -122.1f;
    rx.gnss_altitude_ft = 1000;  // MSL.
    FLARMNMEAReporter::Ownship own;
    FLARMNMEAReporter::Target t = Jet();
    t.baro_altitude_valid = false;  // ADS-B geometric (HAE) only.
    t.geometric_altitude_ft = 3893;
    FLARMNMEAReporter::RelativePosition r;
    // No geoid separation: MSL is never compared with HAE, no PFLAA.
    ASSERT_TRUE(FLARMNMEAReporter::OwnshipFromRxPosition(own, rx, true));
    EXPECT_FALSE(FLARMNMEAReporter::Relative(own, t, r));
    char buf[100];
    EXPECT_EQ(FLARMNMEAReporter::WritePFLAA(buf, sizeof(buf), own, t), 0);
    // With it: ownship HAE 893 ft, the target 3000 ft above = 914 m.
    ASSERT_TRUE(FLARMNMEAReporter::OwnshipFromRxPosition(own, rx, true, true, -3250));
    ASSERT_TRUE(FLARMNMEAReporter::Relative(own, t, r));
    EXPECT_EQ(r.vertical_m, 914);
    EXPECT_GT(FLARMNMEAReporter::WritePFLAA(buf, sizeof(buf), own, t), 0);
    // A baro-only target stays without PFLAA (a GNSS ownship has no baro altitude).
    t.baro_altitude_valid = true;
    t.geometric_altitude_valid = false;
    EXPECT_FALSE(FLARMNMEAReporter::Relative(own, t, r));
}

TEST(FLARMNMEA, RelativePositionGeometry) {
    FLARMNMEAReporter::RelativePosition r;
    FLARMNMEAReporter::Ownship own = Own();
    own.lat_awb = DegToAWB(60.0);
    own.lon_awb = DegToAWB(179.99);
    FLARMNMEAReporter::Target t = Jet();
    // 0.02 degrees east across the antimeridian at 60 N (GeographicLib: 0.17 m north, 1116.00 m east).
    Place(t, own, 60.0, -179.99);
    ASSERT_TRUE(FLARMNMEAReporter::Relative(own, t, r));
    EXPECT_EQ(r.north_m, 0);
    EXPECT_EQ(r.east_m, 1116);
    EXPECT_EQ(r.distance_m, 1116u);
    // 0.1 degrees south (GeographicLib: 11141.14 m).
    Place(t, own, 59.9, 179.99);
    ASSERT_TRUE(FLARMNMEAReporter::Relative(own, t, r));
    EXPECT_EQ(r.north_m, -11141);
    EXPECT_EQ(r.east_m, 0);
    // Antipode: beyond the PFLAA range.
    Place(t, own, -60.0, 0.0);
    EXPECT_FALSE(FLARMNMEAReporter::Relative(own, t, r));
    // No distance from the dictionary (no position, or none measured from ownship yet): no relative position.
    Place(t, own, 59.9, 179.99);
    t.distance_valid = false;
    EXPECT_FALSE(FLARMNMEAReporter::Relative(own, t, r));
}

TEST(FLARMNMEA, LongestPFLAAFits) {
    char buf[100];
    FLARMNMEAReporter::Ownship own = Own();
    own.lat_awb = DegToAWB(0);
    own.lon_awb = DegToAWB(0);
    own.geometric_altitude_ft = 200000;
    FLARMNMEAReporter::Target t = Jet();
    t.baro_altitude_valid = false;
    Place(t, own, -34.35, -136.76);  // 14130 km south west.
    t.geometric_altitude_ft = -2000;
    t.track_deg = 359;
    t.speed_kts = 99999;
    t.vertical_rate_fpm = -99999;
    t.type = FLARMNMEAReporter::kAircraftTypeStaticObstacle;
    t.source = FLARMNMEAReporter::kSourceTISB;
    t.rssi_dbm = -100;
    t.id = 0xFFFFFF;
    const uint16_t len = FLARMNMEAReporter::WritePFLAA(buf, sizeof(buf), own, t);
    EXPECT_EQ(Body(buf, len), "$PFLAA,0,-9924988,-10009080,-32768,1,FFFFFF,359,,32767,-32.7,F,0,4,-100");
    EXPECT_LE(len, 82);
}

TEST(FLARMNMEA, GPRMCAndGPGGA) {
    char buf[100];
    FLARMNMEAReporter::Ownship own = Own();
    own.msl_altitude_valid = true;
    own.msl_altitude_ft = 1000;
    FLARMNMEAReporter::UTCTime utc;
    // No UTC time: no position sentences (XCSoar drops them without a time).
    EXPECT_EQ(FLARMNMEAReporter::WriteGPRMC(buf, sizeof(buf), own, utc), 0);
    EXPECT_EQ(FLARMNMEAReporter::WriteGPGGA(buf, sizeof(buf), own, utc), 0);

    utc.time_valid = true;
    utc.hour = 12;
    utc.minute = 3;
    utc.second = 4;
    uint16_t len = FLARMNMEAReporter::WriteGPRMC(buf, sizeof(buf), own, utc);
    EXPECT_EQ(Body(buf, len), "$GPRMC,120304.00,A,3724.0000,N,12206.0000,W,95.0,272.0,,,,A");
    utc.date_valid = true;
    utc.year = 2026;
    utc.month = 10;
    utc.day = 29;
    len = FLARMNMEAReporter::WriteGPRMC(buf, sizeof(buf), own, utc);
    EXPECT_EQ(Body(buf, len), "$GPRMC,120304.00,A,3724.0000,N,12206.0000,W,95.0,272.0,291026,,,A");
    len = FLARMNMEAReporter::WriteGPGGA(buf, sizeof(buf), own, utc);
    EXPECT_EQ(Body(buf, len), "$GPGGA,120304.00,3724.0000,N,12206.0000,W,1,,,304.8,M,,M,,");
    // With the geoid separation, field 11 carries it (meters, one decimal); the altitude stays MSL.
    own.geoid_separation_valid = true;
    own.geoid_separation_cm = -3254;
    len = FLARMNMEAReporter::WriteGPGGA(buf, sizeof(buf), own, utc);
    EXPECT_EQ(Body(buf, len), "$GPGGA,120304.00,3724.0000,N,12206.0000,W,1,,,304.8,M,-32.5,M,,");
    own.geoid_separation_cm = 4695;
    len = FLARMNMEAReporter::WriteGPGGA(buf, sizeof(buf), own, utc);
    EXPECT_EQ(Body(buf, len), "$GPGGA,120304.00,3724.0000,N,12206.0000,W,1,,,304.8,M,47.0,M,,");
    own.geoid_separation_valid = false;

    // Rounding carries into the degrees; southern / eastern hemispheres.
    own.lat_awb = DegToAWB(-33.99999999);
    own.lon_awb = DegToAWB(151.2);
    own.msl_altitude_ft = -10;
    len = FLARMNMEAReporter::WriteGPGGA(buf, sizeof(buf), own, utc);
    EXPECT_EQ(Body(buf, len), "$GPGGA,120304.00,3400.0000,S,15112.0000,E,1,,,-3.0,M,,M,,");
    // No MSL altitude: empty. The 180th meridian; a float position is written as the float has it (37.4f is
    // 37.4000015 deg).
    own.msl_altitude_valid = false;
    own.lat_awb = FloatDegToAWB(37.4f);
    own.lon_awb = DegToAWB(-180.0);
    len = FLARMNMEAReporter::WriteGPGGA(buf, sizeof(buf), own, utc);
    EXPECT_EQ(Body(buf, len), "$GPGGA,120304.00,3724.0001,N,18000.0000,W,1,,,,M,,M,,");
}

TEST(FLARMNMEA, PGRMZOnlyWithBaro) {
    char buf[100];
    FLARMNMEAReporter::Ownship own = Own();
    EXPECT_EQ(FLARMNMEAReporter::WritePGRMZ(buf, sizeof(buf), own), 0);
    own.baro_altitude_valid = true;
    own.baro_altitude_ft = -120;
    const uint16_t len = FLARMNMEAReporter::WritePGRMZ(buf, sizeof(buf), own);
    EXPECT_EQ(Body(buf, len), "$PGRMZ,-120,F,2");
}

TEST(FLARMNMEA, SmallBufferWritesNothing) {
    char buf[20];
    EXPECT_EQ(FLARMNMEAReporter::WritePFLAA(buf, sizeof(buf), Own(), Jet()), 0);
}

TEST(FLARMNMEA, OwnshipFromRxPosition) {
    FLARMNMEAReporter::Ownship own;
    SettingsManager::RxPosition rx;
    rx.latitude_awb = DegToAWB(37.4f);
    rx.longitude_awb = DegToAWB(-122.1f);
    rx.gnss_altitude_ft = 1000;
    rx.baro_altitude_ft = 1200;
    rx.speed_kts = 95;
    rx.heading_deg = 270;
    // Fixed and lowest-aircraft positions aren't ownship.
    rx.source = SettingsManager::RxPosition::kPositionSourceFixed;
    EXPECT_FALSE(FLARMNMEAReporter::OwnshipFromRxPosition(own, rx, true));
    rx.source = SettingsManager::RxPosition::kPositionSourceLowestAircraft;
    EXPECT_FALSE(FLARMNMEAReporter::OwnshipFromRxPosition(own, rx, true));
    rx.source = SettingsManager::RxPosition::kPositionSourceGNSS;
    EXPECT_FALSE(FLARMNMEAReporter::OwnshipFromRxPosition(own, rx, false));
    ASSERT_TRUE(FLARMNMEAReporter::OwnshipFromRxPosition(own, rx, true));
    EXPECT_EQ(own.lat_awb, FloatDegToAWB(37.4f));
    EXPECT_EQ(own.lon_awb, FloatDegToAWB(-122.1f));
    // GNSS without a geoid separation: mean sea level for GPGGA, nothing to compare traffic with.
    EXPECT_TRUE(own.msl_altitude_valid);
    EXPECT_EQ(own.msl_altitude_ft, 1000);
    EXPECT_FALSE(own.geometric_altitude_valid);
    EXPECT_FALSE(own.geoid_separation_valid);
    EXPECT_FALSE(own.baro_altitude_valid);
    EXPECT_TRUE(own.airborne);
    // GNSS with a geoid separation: HAE = MSL + separation, as GDL90's ID 11 (-32.5 m = -106.6 ft).
    ASSERT_TRUE(FLARMNMEAReporter::OwnshipFromRxPosition(own, rx, true, true, -3250));
    EXPECT_TRUE(own.msl_altitude_valid);
    EXPECT_EQ(own.msl_altitude_ft, 1000);
    EXPECT_TRUE(own.geometric_altitude_valid);
    EXPECT_EQ(own.geometric_altitude_ft, 893);
    EXPECT_TRUE(own.geoid_separation_valid);
    EXPECT_EQ(own.geoid_separation_cm, -3250);
    EXPECT_FALSE(own.baro_altitude_valid);
    // A bad separation (beyond 1 km) gives none.
    ASSERT_TRUE(FLARMNMEAReporter::OwnshipFromRxPosition(own, rx, true, true, 200000));
    EXPECT_FALSE(own.geometric_altitude_valid);
    EXPECT_FALSE(own.geoid_separation_valid);
    // Tracked aircraft: RxPosition's altitudes have no validity flags, so none until the aircraft's own are set.
    rx.source = SettingsManager::RxPosition::kPositionSourceAircraftMatchingICAO;
    rx.speed_kts = 10;
    ASSERT_TRUE(FLARMNMEAReporter::OwnshipFromRxPosition(own, rx, true, true, -3250));  // Separation ignored.
    EXPECT_FALSE(own.geoid_separation_valid);
    EXPECT_FALSE(own.msl_altitude_valid);
    EXPECT_FALSE(own.geometric_altitude_valid);
    EXPECT_FALSE(own.baro_altitude_valid);
    EXPECT_FALSE(own.airborne);
    ModeSAircraft ac(0xBEE001);
    ac.WriteBitFlag(ModeSAircraft::kBitFlagBaroAltitudeValid, true);
    ac.baro_altitude_ft = 1200;
    ac.gnss_altitude_ft = 1325;  // Not valid yet.
    FLARMNMEAReporter::OwnshipAltitudesFromAircraft(own, ac);
    EXPECT_TRUE(own.baro_altitude_valid);
    EXPECT_EQ(own.baro_altitude_ft, 1200);
    EXPECT_FALSE(own.geometric_altitude_valid);
    ac.WriteBitFlag(ModeSAircraft::kBitFlagGNSSAltitudeValid, true);
    FLARMNMEAReporter::OwnshipAltitudesFromAircraft(own, ac);
    EXPECT_TRUE(own.geometric_altitude_valid);
    EXPECT_EQ(own.geometric_altitude_ft, 1325);
}

TEST(FLARMNMEA, TargetFromModeSAircraft) {
    FLARMNMEAReporter::Target t;
    ModeSAircraft ac(0xA1B2C3);
    EXPECT_FALSE(FLARMNMEAReporter::TargetFromAircraft(t, ac));  // No position.
    ac.WriteBitFlag(ModeSAircraft::kBitFlagPositionValid, true);
    ac.WriteBitFlag(ModeSAircraft::kBitFlagBaroAltitudeValid, true);
    ac.WriteBitFlag(ModeSAircraft::kBitFlagGNSSVerticalRateValid, true);
    ac.latitude_awb = DegToAWB(37.45f);
    ac.longitude_awb = DegToAWB(-122.0f);
    ac.baro_altitude_ft = 3500;
    ac.gnss_vertical_rate_fpm = 500;
    ac.emitter_category = ADSBTypes::kEmitterCategoryGliderSailplane;
    ac.last_message_signal_strength_dbm = -80;
    ac.position_address_type = ModeSAircraft::kAddressTypeTISBTrackFile;
    ASSERT_TRUE(FLARMNMEAReporter::TargetFromAircraft(t, ac));
    EXPECT_EQ(t.id, 0xA1B2C3u);
    EXPECT_EQ(t.id_type, FLARMNMEAReporter::kIDTypeRandom);
    EXPECT_EQ(t.source, FLARMNMEAReporter::kSourceTISB);
    EXPECT_EQ(t.type, FLARMNMEAReporter::kAircraftTypeGlider);
    EXPECT_TRUE(t.baro_altitude_valid);
    EXPECT_FALSE(t.geometric_altitude_valid);
    EXPECT_FALSE(t.distance_valid);  // No receiver position: the dictionary hasn't measured it.
    ac.distance_m = 5321;
    ac.bearing_bam16 = 0x1234;
    ASSERT_TRUE(FLARMNMEAReporter::TargetFromAircraft(t, ac));
    EXPECT_TRUE(t.distance_valid);
    EXPECT_EQ(t.distance_m, 5321u);
    EXPECT_EQ(t.bearing_bam16, 0x1234);
    EXPECT_TRUE(t.vertical_rate_valid);
    EXPECT_EQ(t.vertical_rate_fpm, 500);
    EXPECT_FALSE(t.track_valid);
    EXPECT_EQ(t.rssi_dbm, -80);
    ac.position_address_type = ModeSAircraft::kAddressTypeADSBICAO;
    ASSERT_TRUE(FLARMNMEAReporter::TargetFromAircraft(t, ac));
    EXPECT_EQ(t.id_type, FLARMNMEAReporter::kIDTypeICAO);
    EXPECT_EQ(t.source, FLARMNMEAReporter::kSourceADSB);
}

TEST(FLARMNMEA, TargetFromRemoteIDAircraft) {
    FLARMNMEAReporter::Target t;
    RemoteIDAircraft ac;
    ac.address = 0x123456;
    ac.WriteBitFlag(RemoteIDAircraft::kBitFlagPositionValid, true);
    ac.WriteBitFlag(RemoteIDAircraft::kBitFlagGNSSAltitudeValid, true);
    ac.WriteBitFlag(RemoteIDAircraft::kBitFlagIsAirborne, true);
    ASSERT_TRUE(FLARMNMEAReporter::TargetFromAircraft(t, ac));
    EXPECT_EQ(t.type, FLARMNMEAReporter::kAircraftTypeUAV);
    EXPECT_EQ(t.id_type, FLARMNMEAReporter::kIDTypeRandom);
    EXPECT_EQ(t.source, FLARMNMEAReporter::kSourceNone);
    EXPECT_FALSE(t.on_ground);
}

TEST(FLARMNMEA, AircraftTypes) {
    EXPECT_EQ(FLARMNMEAReporter::AircraftTypeFromEmitterCategory(ADSBTypes::kEmitterCategoryLight),
              FLARMNMEAReporter::kAircraftTypePiston);
    EXPECT_EQ(FLARMNMEAReporter::AircraftTypeFromEmitterCategory(ADSBTypes::kEmitterCategoryHeavy),
              FLARMNMEAReporter::kAircraftTypeJet);
    EXPECT_EQ(FLARMNMEAReporter::AircraftTypeFromEmitterCategory(ADSBTypes::kEmitterCategoryRotorcraft),
              FLARMNMEAReporter::kAircraftTypeRotorcraft);
    EXPECT_EQ(FLARMNMEAReporter::AircraftTypeFromEmitterCategory(ADSBTypes::kEmitterCategoryNoCategoryInfo),
              FLARMNMEAReporter::kAircraftTypeUnknown);
    EXPECT_EQ(FLARMNMEAReporter::AircraftTypeFromEmitterCategory(ADSBTypes::kEmitterCategorySurfaceServiceVehicle),
              FLARMNMEAReporter::kAircraftTypeUnknown);
}
