#include <cstdio>
#include <cstring>
#include <string>

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

FLARMNMEAReporter::Ownship GNSSOwnship() {
    FLARMNMEAReporter::Ownship own;
    own.latitude_deg = 37.4;
    own.longitude_deg = -122.1;
    own.gnss_altitude_valid = true;
    own.gnss_altitude_ft = 1000;
    own.track_deg = 271.6f;
    own.speed_kts = 95;
    own.airborne = true;
    return own;
}

FLARMNMEAReporter::Target Jet() {
    FLARMNMEAReporter::Target t;
    t.id = 0xA1B2C3;
    t.latitude_deg = 37.45;
    t.longitude_deg = -122.0;
    t.baro_altitude_valid = true;
    t.baro_altitude_ft = 3500;
    t.gnss_altitude_valid = true;
    t.gnss_altitude_ft = 3600;
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
    const uint16_t len = FLARMNMEAReporter::WritePFLAA(buf, sizeof(buf), GNSSOwnship(), Jet());
    // 5.56 km north, 8.83 km east; GNSS against GNSS: 2600 ft = 792 m above; track 359.7 -> 0; 250 kt = 129 m/s;
    // -640 fpm = -3.3 m/s; jet; no-track 0; source ADS-B; RSSI.
    EXPECT_EQ(Body(buf, len), "$PFLAA,0,5560,8831,792,1,A1B2C3,0,,129,-3.3,9,0,1,-71");
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
    uint16_t len = FLARMNMEAReporter::WritePFLAA(buf, sizeof(buf), GNSSOwnship(), t);
    EXPECT_EQ(Body(buf, len), "$PFLAA,0,5560,8831,792,0,A1B2C3,,,,,9,0,4,");

    // On ground: speed and climb 0.
    t.on_ground = true;
    len = FLARMNMEAReporter::WritePFLAA(buf, sizeof(buf), GNSSOwnship(), t);
    EXPECT_EQ(Body(buf, len), "$PFLAA,0,5560,8831,792,0,A1B2C3,,,0,0.0,9,0,4,");

    // No FTD-012 source (Remote ID): Source and RSSI omitted.
    t.source = FLARMNMEAReporter::kSourceNone;
    len = FLARMNMEAReporter::WritePFLAA(buf, sizeof(buf), GNSSOwnship(), t);
    EXPECT_EQ(Body(buf, len), "$PFLAA,0,5560,8831,792,0,A1B2C3,,,0,0.0,9,0");
}

TEST(FLARMNMEA, PFLAAMovingSpeedAndClimbLimits) {
    char buf[100];
    FLARMNMEAReporter::Target t = Jet();
    t.speed_kts = 0;  // Moving but reporting 0: FTD-012 forces > 0.
    t.vertical_rate_fpm = 99999;
    uint16_t len = FLARMNMEAReporter::WritePFLAA(buf, sizeof(buf), GNSSOwnship(), t);
    EXPECT_EQ(Body(buf, len), "$PFLAA,0,5560,8831,792,1,A1B2C3,0,,1,32.7,9,0,1,-71");
    t.vertical_rate_fpm = -5;  // -0.03 m/s rounds to 0.0, no "-0.0".
    len = FLARMNMEAReporter::WritePFLAA(buf, sizeof(buf), GNSSOwnship(), t);
    EXPECT_EQ(Body(buf, len), "$PFLAA,0,5560,8831,792,1,A1B2C3,0,,1,0.0,9,0,1,-71");
}

TEST(FLARMNMEA, RelativeAltitudeReference) {
    int32_t n, e, v;
    FLARMNMEAReporter::Ownship own = GNSSOwnship();
    FLARMNMEAReporter::Target t = Jet();
    // Ownship baro (its own ADS-B) and target baro: baro against baro.
    own.baro_altitude_valid = true;
    own.baro_altitude_ft = 1500;
    ASSERT_TRUE(FLARMNMEAReporter::Relative(own, t, n, e, v));
    EXPECT_EQ(v, 610);  // 2000 ft.
    // GNSS ownship, target without geometric altitude: target baro against ownship GNSS.
    own.baro_altitude_valid = false;
    t.gnss_altitude_valid = false;
    ASSERT_TRUE(FLARMNMEAReporter::Relative(own, t, n, e, v));
    EXPECT_EQ(v, 762);  // 2500 ft.
    // No common reference.
    t.baro_altitude_valid = false;
    EXPECT_FALSE(FLARMNMEAReporter::Relative(own, t, n, e, v));
    char buf[100];
    EXPECT_EQ(FLARMNMEAReporter::WritePFLAA(buf, sizeof(buf), own, t), 0);
}

TEST(FLARMNMEA, RelativePositionGeometry) {
    int32_t n, e, v;
    FLARMNMEAReporter::Ownship own = GNSSOwnship();
    own.latitude_deg = 60.0;
    own.longitude_deg = 179.99;
    FLARMNMEAReporter::Target t = Jet();
    // 0.02 degrees east across the antimeridian at 60 N: about 1112 m.
    t.latitude_deg = 60.0;
    t.longitude_deg = -179.99;
    ASSERT_TRUE(FLARMNMEAReporter::Relative(own, t, n, e, v));
    EXPECT_EQ(n, 0);
    EXPECT_NEAR(e, 1112, 2);
    // 0.1 degrees south: about 11.1 km.
    t.latitude_deg = 59.9;
    t.longitude_deg = 179.99;
    ASSERT_TRUE(FLARMNMEAReporter::Relative(own, t, n, e, v));
    EXPECT_NEAR(n, -11119, 2);
    EXPECT_EQ(e, 0);
    // Antipode: beyond the PFLAA range.
    t.latitude_deg = -60.0;
    t.longitude_deg = 0.0;
    EXPECT_FALSE(FLARMNMEAReporter::Relative(own, t, n, e, v));
}

TEST(FLARMNMEA, LongestPFLAAFits) {
    char buf[100];
    FLARMNMEAReporter::Ownship own = GNSSOwnship();
    own.latitude_deg = 0;
    own.longitude_deg = 0;
    own.gnss_altitude_ft = 200000;
    FLARMNMEAReporter::Target t = Jet();
    t.latitude_deg = -89.0;
    t.longitude_deg = -179.0;
    t.gnss_altitude_ft = -2000;
    t.track_deg = 359;
    t.speed_kts = 99999;
    t.vertical_rate_fpm = -99999;
    t.type = FLARMNMEAReporter::kAircraftTypeStaticObstacle;
    t.source = FLARMNMEAReporter::kSourceTISB;
    t.rssi_dbm = -100;
    t.id = 0xFFFFFF;
    const uint16_t len = FLARMNMEAReporter::WritePFLAA(buf, sizeof(buf), own, t);
    EXPECT_EQ(Body(buf, len), "$PFLAA,0,-9896362,-14196479,-32768,1,FFFFFF,359,,32767,-32.7,F,0,4,-100");
    EXPECT_LE(len, 82);
}

TEST(FLARMNMEA, GPRMCAndGPGGA) {
    char buf[100];
    FLARMNMEAReporter::Ownship own = GNSSOwnship();
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

    // Rounding carries into the degrees; southern / eastern hemispheres.
    own.latitude_deg = -33.99999999;
    own.longitude_deg = 151.2;
    own.gnss_altitude_ft = -10;
    len = FLARMNMEAReporter::WriteGPGGA(buf, sizeof(buf), own, utc);
    EXPECT_EQ(Body(buf, len), "$GPGGA,120304.00,3400.0000,S,15112.0000,E,1,,,-3.0,M,,M,,");
}

TEST(FLARMNMEA, PGRMZOnlyWithBaro) {
    char buf[100];
    FLARMNMEAReporter::Ownship own = GNSSOwnship();
    EXPECT_EQ(FLARMNMEAReporter::WritePGRMZ(buf, sizeof(buf), own), 0);
    own.baro_altitude_valid = true;
    own.baro_altitude_ft = -120;
    const uint16_t len = FLARMNMEAReporter::WritePGRMZ(buf, sizeof(buf), own);
    EXPECT_EQ(Body(buf, len), "$PGRMZ,-120,F,2");
}

TEST(FLARMNMEA, SmallBufferWritesNothing) {
    char buf[20];
    EXPECT_EQ(FLARMNMEAReporter::WritePFLAA(buf, sizeof(buf), GNSSOwnship(), Jet()), 0);
}

TEST(FLARMNMEA, OwnshipFromRxPosition) {
    FLARMNMEAReporter::Ownship own;
    SettingsManager::RxPosition rx;
    rx.latitude_deg = 37.4f;
    rx.longitude_deg = -122.1f;
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
    EXPECT_TRUE(own.gnss_altitude_valid);
    EXPECT_FALSE(own.baro_altitude_valid);
    EXPECT_TRUE(own.airborne);
    rx.source = SettingsManager::RxPosition::kPositionSourceAircraftMatchingICAO;
    rx.speed_kts = 10;
    ASSERT_TRUE(FLARMNMEAReporter::OwnshipFromRxPosition(own, rx, true));
    EXPECT_FALSE(own.gnss_altitude_valid);
    EXPECT_TRUE(own.baro_altitude_valid);
    EXPECT_EQ(own.baro_altitude_ft, 1200);
    EXPECT_FALSE(own.airborne);
}

TEST(FLARMNMEA, TargetFromModeSAircraft) {
    FLARMNMEAReporter::Target t;
    ModeSAircraft ac(0xA1B2C3);
    EXPECT_FALSE(FLARMNMEAReporter::TargetFromAircraft(t, ac));  // No position.
    ac.WriteBitFlag(ModeSAircraft::kBitFlagPositionValid, true);
    ac.WriteBitFlag(ModeSAircraft::kBitFlagBaroAltitudeValid, true);
    ac.WriteBitFlag(ModeSAircraft::kBitFlagGNSSVerticalRateValid, true);
    ac.latitude_deg = 37.45f;
    ac.longitude_deg = -122.0f;
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
    EXPECT_FALSE(t.gnss_altitude_valid);
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
