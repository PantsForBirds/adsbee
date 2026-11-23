#include <cmath>
#include "awb_test_utils.hh"
#include <vector>

#include "asterix_utils.hh"
#include "gtest/gtest.h"

using CAT021TargetReport = ASTERIXReporter::CAT021TargetReport;

namespace {

std::vector<uint8_t> Record(const CAT021TargetReport& report) {
    uint8_t buf[ASTERIXReporter::kCAT021RecordMaxLenBytes];
    uint16_t len = ASTERIXReporter::WriteCAT021Record(buf, sizeof(buf), report);
    return std::vector<uint8_t>(buf, buf + len);
}

// 12:34:56.789 UTC.
constexpr uint32_t kUTCMsOfDay = ((12 * 60 + 34) * 60 + 56) * 1000 + 789;
constexpr uint32_t kTimeOfDayTicks = 0x587865;  // round(45296.789 s * 128).

ASTERIXReporter::ReportContext Context(uint32_t now_ms) {
    ASTERIXReporter::ReportContext ctx;
    ctx.sac = 25;
    ctx.sic = 7;
    ctx.clock.SetUTC(kUTCMsOfDay, 1000);  // 12:34:56.789 at 1 s after boot.
    ctx.timestamp_ms = now_ms;
    return ctx;
}

ModeSAircraft AirborneModeSAircraft() {
    ModeSAircraft aircraft(0xA1B2C3);
    aircraft.WriteBitFlag(ModeSAircraft::kBitFlagIsAirborne, true);
    aircraft.WriteBitFlag(ModeSAircraft::kBitFlagPositionValid, true);
    aircraft.latitude_awb = DegToAWB(37.4f);
    aircraft.longitude_awb = DegToAWB(-122.1f);
    aircraft.last_position_update_ms = 1000;
    aircraft.WriteBitFlag(ModeSAircraft::kBitFlagBaroAltitudeValid, true);
    aircraft.baro_altitude_ft = 35025;
    aircraft.WriteBitFlag(ModeSAircraft::kBitFlagGNSSAltitudeValid, true);
    aircraft.gnss_altitude_ft = 36000;
    aircraft.speed_source = ADSBTypes::kSpeedSourceGroundSpeed;
    aircraft.WriteBitFlag(ModeSAircraft::kBitFlagHorizontalSpeedValid, true);
    aircraft.WriteBitFlag(ModeSAircraft::kBitFlagDirectionValid, true);
    aircraft.speed_kts = 450;
    aircraft.direction_deg = 270.0f;
    aircraft.WriteBitFlag(ModeSAircraft::kBitFlagBaroVerticalRateValid, true);
    aircraft.baro_vertical_rate_fpm = -1000;
    aircraft.last_velocity_update_ms = 1500;
    aircraft.squawk = 1200;
    strcpy(aircraft.callsign, "UAL123");
    aircraft.emitter_category = ADSBTypes::kEmitterCategoryMedium2;
    aircraft.last_message_signal_strength_dbm = -60;
    aircraft.navigation_integrity_category = ADSBTypes::kROCLessThan0p1NauticalMiles;  // NIC 8.
    aircraft.navigation_accuracy_category_position = ADSBTypes::kEPULessThan30Meters;  // NACp 9.
    aircraft.adsb_version = 2;
    return aircraft;
}

}  // namespace

TEST(ASTERIXUtils, TimeOfDayFromUTC) {
    ASTERIXReporter::Clock clock;
    clock.SetUTC(kUTCMsOfDay, 1000);
    EXPECT_TRUE(clock.utc_valid);
    EXPECT_EQ(clock.TimeOfDay(1000), kTimeOfDayTicks);
    EXPECT_EQ(clock.TimeOfDay(2000), kTimeOfDayTicks + 128);
    EXPECT_EQ(clock.TimeOfDay(0), kTimeOfDayTicks - 128);
}

TEST(ASTERIXUtils, TimeOfDayWrapsAtMidnight) {
    ASTERIXReporter::Clock clock;
    clock.SetUTC(ASTERIXReporter::kMsPerDay - 1000, 5000);  // 23:59:59.000.
    EXPECT_EQ(clock.TimeOfDay(5000), 86399u * 128);
    EXPECT_EQ(clock.TimeOfDay(5999), 0u);  // 23:59:59.999 rounds up to midnight.
    EXPECT_EQ(clock.TimeOfDay(6000), 0u);
    EXPECT_EQ(clock.TimeOfDay(6500), 64u);
    // A receiver timestamp before the UTC fix, across the previous midnight.
    clock.SetUTC(500, 10000);
    EXPECT_EQ(clock.TimeOfDay(9000), 11059136u);  // 23:59:59.500.
}

TEST(ASTERIXUtils, TimeOfDayWithoutUTCCountsFromBoot) {
    ASTERIXReporter::Clock clock;
    EXPECT_FALSE(clock.utc_valid);
    EXPECT_EQ(clock.TimeOfDay(0), 0u);
    EXPECT_EQ(clock.TimeOfDay(1500), 192u);
    EXPECT_EQ(clock.TimeOfDay(ASTERIXReporter::kMsPerDay + 1000), 128u);
}

TEST(ASTERIXUtils, GNSSTimeWinsOverNetworkTime) {
    ASTERIXReporter::Clock clock;
    clock.SetUTC(kUTCMsOfDay + 250, 1000, ASTERIXReporter::Clock::kSourceNetwork);
    EXPECT_TRUE(clock.utc_valid);
    EXPECT_EQ(clock.source, ASTERIXReporter::Clock::kSourceNetwork);
    EXPECT_EQ(clock.TimeOfDay(1000), kTimeOfDayTicks + 32);

    clock.SetUTC(kUTCMsOfDay, 1000, ASTERIXReporter::Clock::kSourceGNSS);
    EXPECT_EQ(clock.source, ASTERIXReporter::Clock::kSourceGNSS);
    EXPECT_EQ(clock.TimeOfDay(1000), kTimeOfDayTicks);

    // Network time set after GNSS time doesn't replace it.
    clock.SetUTC(kUTCMsOfDay + 250, 1000, ASTERIXReporter::Clock::kSourceNetwork);
    EXPECT_EQ(clock.source, ASTERIXReporter::Clock::kSourceGNSS);
    EXPECT_EQ(clock.TimeOfDay(1000), kTimeOfDayTicks);
}

TEST(ASTERIXUtils, NetworkTimeIsValidTime) {
    ModeSAircraft aircraft = AirborneModeSAircraft();
    ASTERIXReporter::ReportContext ctx;
    ctx.clock.SetUTC(kUTCMsOfDay, 3000, ASTERIXReporter::Clock::kSourceNetwork);
    ctx.timestamp_ms = 3000;
    CAT021TargetReport r;
    ASSERT_TRUE(ASTERIXReporter::BuildCAT021Report(r, aircraft, ctx));
    EXPECT_FALSE(r.no_go);

    // CAT023 ground station status: neither NOGO nor TSV (time source invalid).
    uint8_t buf[ASTERIXReporter::kCAT023RecordMaxLenBytes];
    uint16_t len = ASTERIXReporter::WriteCAT023GroundStationStatusRecord(
        buf, sizeof(buf), 0, 1, ctx.clock.TimeOfDay(3000), ctx.clock.utc_valid, false, 1);
    ASSERT_EQ(len, 9u);
    EXPECT_EQ(buf[7] & 0x88, 0);
}

TEST(ASTERIXUtils, SquawkToMode3A) {
    uint16_t code = 0;
    EXPECT_TRUE(ASTERIXReporter::SquawkToMode3A(7700, code));
    EXPECT_EQ(code, 07700);
    EXPECT_TRUE(ASTERIXReporter::SquawkToMode3A(1200, code));
    EXPECT_EQ(code, 01200);
    EXPECT_TRUE(ASTERIXReporter::SquawkToMode3A(17, code));
    EXPECT_EQ(code, 00017);
    EXPECT_FALSE(ASTERIXReporter::SquawkToMode3A(1280, code));
    EXPECT_FALSE(ASTERIXReporter::SquawkToMode3A(9999, code));
}

TEST(ASTERIXUtils, EmitterCategoryToECAT) {
    EXPECT_EQ(ASTERIXReporter::EmitterCategoryToECAT(ADSBTypes::kEmitterCategoryNoCategoryInfo), 0);
    EXPECT_EQ(ASTERIXReporter::EmitterCategoryToECAT(ADSBTypes::kEmitterCategoryLight), 1);
    EXPECT_EQ(ASTERIXReporter::EmitterCategoryToECAT(ADSBTypes::kEmitterCategoryHeavy), 5);
    EXPECT_EQ(ASTERIXReporter::EmitterCategoryToECAT(ADSBTypes::kEmitterCategoryRotorcraft), 10);
    EXPECT_EQ(ASTERIXReporter::EmitterCategoryToECAT(ADSBTypes::kEmitterCategoryGliderSailplane), 11);
    EXPECT_EQ(ASTERIXReporter::EmitterCategoryToECAT(ADSBTypes::kEmitterCategoryParachutistSkydiver), 16);
    EXPECT_EQ(ASTERIXReporter::EmitterCategoryToECAT(ADSBTypes::kEmitterCategoryUltralightHangGliderParaglider), 15);
    EXPECT_EQ(ASTERIXReporter::EmitterCategoryToECAT(ADSBTypes::kEmitterCategoryUnmannedAerialVehicle), 13);
    EXPECT_EQ(ASTERIXReporter::EmitterCategoryToECAT(ADSBTypes::kEmitterCategorySurfaceEmergencyVehicle), 20);
    EXPECT_EQ(ASTERIXReporter::EmitterCategoryToECAT(ADSBTypes::kEmitterCategoryLineObstacle), 24);
    EXPECT_EQ(ASTERIXReporter::EmitterCategoryToECAT(ADSBTypes::kEmitterCategoryReserved), 0);
}

TEST(ASTERIXUtils, NICToPIC) {
    EXPECT_EQ(ASTERIXReporter::NICToPIC(0), 0);
    EXPECT_EQ(ASTERIXReporter::NICToPIC(1), 1);
    EXPECT_EQ(ASTERIXReporter::NICToPIC(6), 7);
    EXPECT_EQ(ASTERIXReporter::NICToPIC(8), 11);
    EXPECT_EQ(ASTERIXReporter::NICToPIC(11), 14);
    EXPECT_EQ(ASTERIXReporter::NICToPIC(15), 0);
}

TEST(ASTERIXUtils, CAT021RecordMatchesHandEncoding) {
    CAT021TargetReport r;
    r.sac = 0;
    r.sic = 1;
    r.altitude_reporting_capability = ASTERIXReporter::kARC25Ft;
    r.has_track_number = true;
    r.track_number = 291;
    r.has_service_id = true;
    r.service_id = 1;
    r.has_position = true;
    r.latitude_awb = DegToAWB(37.4);
    r.longitude_awb = DegToAWB(-122.1);
    r.position_time_of_day = kTimeOfDayTicks;
    r.address = 0xA1B2C3;
    r.has_geometric_height = true;
    r.geometric_height_ft = 36000;
    r.nac_v = 1;
    r.nic = 8;
    r.nic_baro = 1;
    r.sil = 3;
    r.nac_p = 9;
    r.sda = 2;
    r.gva = 1;
    r.pic = ASTERIXReporter::NICToPIC(8);
    r.has_mops_version = true;
    r.mops_version = 2;
    r.has_squawk = true;
    r.squawk = 1200;
    r.has_flight_level = true;
    r.baro_altitude_ft = 35025;
    r.has_baro_vertical_rate = true;
    r.baro_vertical_rate_fpm = -1000;
    r.has_ground_vector = true;
    r.ground_speed_kts = 450;
    r.track_angle_deg = 270;
    r.has_callsign = true;
    strcpy(r.callsign, "UAL123");
    r.has_emitter_category = true;
    r.emitter_category = 3;
    r.has_amplitude = true;
    r.amplitude_dbm = -60;

    const std::vector<uint8_t> expected = {
        0xF3, 0x19, 0x7B, 0x29, 0xC1, 0x20,              // FSPEC: FRN 1-4, 7, 11, 12, 16-19, 21, 24, 26, 29, 30, 38.
        0x00, 0x01,                                      // I021/010 SAC 0, SIC 1.
        0x00,                                            // I021/040 ATP 0, ARC 25 ft.
        0x01, 0x23,                                      // I021/161 track 291.
        0x01,                                            // I021/015 service 1.
        0x0D, 0x4C, 0x3B, 0x2A, 0xD4, 0x96, 0x2F, 0xCA,  // I021/131 37.4, -122.1 (from AWB, rounded).
        0xA1, 0xB2, 0xC3,                                // I021/080.
        0x58, 0x78, 0x65,                                // I021/073 12:34:56.789.
        0x16, 0x80,                                      // I021/140 36000 ft.
        0x31, 0xF3, 0x13, 0xB0,                          // I021/090 NACv 1 NIC 8 | NICbaro 1 SIL 3 NACp 9 | SDA 2 GVA 1 | PIC 11.
        0x12,                                            // I021/210 version 2, 1090 ES.
        0x02, 0x80,                                      // I021/070 1200.
        0x05, 0x79,                                      // I021/145 FL 350.25.
        0x7F, 0x60,                                      // I021/155 -1000 ft/min.
        0x08, 0x00, 0xC0, 0x00,                          // I021/160 450 kt, 270 degrees.
        0x54, 0x13, 0x31, 0xCB, 0x38, 0x20,              // I021/170 "UAL123  ".
        0x03,                                            // I021/020.
        0xC4,                                            // I021/132 -60 dBm.
    };
    EXPECT_EQ(Record(r), expected);
}

TEST(ASTERIXUtils, CAT021MinimalRecordHasOnlyMandatoryItems) {
    CAT021TargetReport r;
    r.sac = 0x12;
    r.sic = 0x34;
    r.address = 0xABCDEF;
    // FSPEC FRN 1, 2, 11, 17; I021/040 ARC unknown; I021/090 all zero, no extensions.
    EXPECT_EQ(Record(r), (std::vector<uint8_t>{0xC1, 0x11, 0x20, 0x12, 0x34, 0x10, 0xAB, 0xCD, 0xEF, 0x00}));
}

TEST(ASTERIXUtils, CAT021DescriptorExtensions) {
    CAT021TargetReport r;
    r.altitude_reporting_capability = ASTERIXReporter::kARC25Ft;
    r.on_ground = true;
    std::vector<uint8_t> rec = Record(r);
    ASSERT_GE(rec.size(), 7u);
    EXPECT_EQ(rec[5], 0x01);  // I021/040 primary, FX.
    EXPECT_EQ(rec[6], 0x40);  // GBS.

    r.on_ground = false;
    r.no_go = true;
    rec = Record(r);
    EXPECT_EQ(rec[5], 0x01);
    EXPECT_EQ(rec[6], 0x01);  // Empty first extension carries FX to the second.
    EXPECT_EQ(rec[7], 0x10);  // NOGO.
}

TEST(ASTERIXUtils, CAT021ClampsAndRangeChecks) {
    CAT021TargetReport r;
    r.has_baro_vertical_rate = true;
    r.baro_vertical_rate_fpm = 200000;
    r.has_flight_level = true;
    r.baro_altitude_ft = 200000;  // Above FL1500: left out.
    r.has_ground_vector = true;
    r.ground_speed_kts = 8000;
    r.track_angle_deg = -90;
    r.has_amplitude = true;
    r.amplitude_dbm = -200;
    r.has_position = true;
    r.latitude_awb = DegToAWB(-90.0);
    r.longitude_awb = DegToAWB(180.0);
    std::vector<uint8_t> rec = Record(r);
    // FSPEC FRN 1, 2, 7 | 11, 12 | 17 | 24, 26 | none | 38.
    ASSERT_EQ(rec.size(), 6u + 2 + 1 + 8 + 3 + 3 + 1 + 2 + 4 + 1);
    EXPECT_EQ(rec[0], 0xC3);
    EXPECT_EQ(rec[1], 0x19);
    EXPECT_EQ(rec[2], 0x21);
    EXPECT_EQ(rec[3], 0x29);
    size_t i = 6 + 2 + 1;
    EXPECT_EQ(std::vector<uint8_t>(rec.begin() + i, rec.begin() + i + 8),
              (std::vector<uint8_t>{0xE0, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00}));  // -90, -180 (= 180).
    i += 8 + 3 + 3 + 1;
    EXPECT_EQ(rec[i], 0xBF);  // Range exceeded, largest climb.
    EXPECT_EQ(rec[i + 1], 0xFF);
    EXPECT_EQ(rec[i + 2], 0xFF);  // Range exceeded, largest speed.
    EXPECT_EQ(rec[i + 3], 0xFF);
    EXPECT_EQ(rec[i + 4], 0xC0);  // 270 degrees.
    EXPECT_EQ(rec[i + 5], 0x00);
    EXPECT_EQ(rec[i + 6], 0x80);  // -128 dBm.
}

TEST(ASTERIXUtils, CAT021RecordFailsWhenBufferIsShort) {
    CAT021TargetReport r;
    uint8_t buf[9];
    EXPECT_EQ(ASTERIXReporter::WriteCAT021Record(buf, sizeof(buf), r), 0);
    uint8_t big_buf[10];
    EXPECT_EQ(ASTERIXReporter::WriteCAT021Record(big_buf, sizeof(big_buf), r), 10);
}

TEST(ASTERIXUtils, BuildCAT021ReportFromModeSAircraft) {
    ModeSAircraft aircraft = AirborneModeSAircraft();
    ASTERIXReporter::ReportContext ctx = Context(3000);
    CAT021TargetReport r;
    ASSERT_TRUE(ASTERIXReporter::BuildCAT021Report(r, aircraft, ctx));
    EXPECT_EQ(r.sac, 25);
    EXPECT_EQ(r.sic, 7);
    EXPECT_FALSE(r.no_go);
    EXPECT_EQ(r.address, 0xA1B2C3u);
    EXPECT_EQ(r.address_type, ASTERIXReporter::kAddressTypeICAO);
    EXPECT_FALSE(r.on_ground);
    EXPECT_EQ(r.service_id, ASTERIXReporter::kServiceID1090ES);
    EXPECT_TRUE(r.has_position);
    EXPECT_NEAR(AWBToDeg(r.latitude_awb), 37.4, 1e-5);
    EXPECT_NEAR(AWBToDeg(r.longitude_awb), -122.1, 1e-5);
    EXPECT_EQ(r.position_time_of_day, kTimeOfDayTicks);
    EXPECT_TRUE(r.has_velocity_time);
    EXPECT_EQ(r.velocity_time_of_day, kTimeOfDayTicks + 64);
    EXPECT_TRUE(r.has_report_time);
    EXPECT_EQ(r.report_time_of_day, kTimeOfDayTicks + 256);
    EXPECT_TRUE(r.has_ground_vector);
    EXPECT_FLOAT_EQ(r.ground_speed_kts, 450);
    EXPECT_FLOAT_EQ(r.track_angle_deg, 270);
    EXPECT_TRUE(r.has_baro_vertical_rate);
    EXPECT_FALSE(r.has_geometric_vertical_rate);
    EXPECT_TRUE(r.has_geometric_height);
    EXPECT_TRUE(r.has_flight_level);
    EXPECT_EQ(r.nic, 8);
    EXPECT_EQ(r.nac_p, 9);
    EXPECT_EQ(r.pic, 11);
    EXPECT_TRUE(r.has_mops_version);
    EXPECT_EQ(r.mops_version, 2);
    EXPECT_EQ(r.link_technology, ASTERIXReporter::kLinkTechnology1090ES);
    EXPECT_TRUE(r.has_squawk);
    EXPECT_EQ(r.squawk, 1200);
    EXPECT_FALSE(r.has_target_status);
    EXPECT_TRUE(r.has_callsign);
    EXPECT_STREQ(r.callsign, "UAL123");
    EXPECT_EQ(r.emitter_category, 3);
    EXPECT_EQ(r.amplitude_dbm, -60);
}

TEST(ASTERIXUtils, BuildCAT021ReportModeSFilters) {
    ASTERIXReporter::ReportContext ctx = Context(3000);
    CAT021TargetReport r;

    ModeSAircraft no_position = AirborneModeSAircraft();
    no_position.WriteBitFlag(ModeSAircraft::kBitFlagPositionValid, false);
    EXPECT_FALSE(ASTERIXReporter::BuildCAT021Report(r, no_position, ctx));

    // Surface targets keep their position but leave the airborne velocity items out.
    ModeSAircraft surface = AirborneModeSAircraft();
    surface.SetAirGroundFromADSB(false, 3000);  // A surface position message.
    ASSERT_TRUE(ASTERIXReporter::BuildCAT021Report(r, surface, ctx));
    EXPECT_TRUE(r.on_ground);
    EXPECT_FALSE(r.has_ground_vector);
    EXPECT_FALSE(r.has_velocity_time);
}

TEST(ASTERIXUtils, BuildCAT021ReportModeSSpeedKinds) {
    ASTERIXReporter::ReportContext ctx = Context(3000);
    CAT021TargetReport r;
    ModeSAircraft aircraft = AirborneModeSAircraft();
    aircraft.speed_source = ADSBTypes::kSpeedSourceAirspeedIndicated;
    aircraft.WriteBitFlag(ModeSAircraft::kBitFlagDirectionIsHeading, true);
    aircraft.WriteBitFlag(ModeSAircraft::kBitFlagHeadingUsesMagneticNorth, true);
    ASSERT_TRUE(ASTERIXReporter::BuildCAT021Report(r, aircraft, ctx));
    EXPECT_FALSE(r.has_ground_vector);
    EXPECT_TRUE(r.has_indicated_airspeed);
    EXPECT_TRUE(r.has_magnetic_heading);
    EXPECT_FLOAT_EQ(r.magnetic_heading_deg, 270);

    aircraft.speed_source = ADSBTypes::kSpeedSourceAirspeedTrue;
    aircraft.WriteBitFlag(ModeSAircraft::kBitFlagHeadingUsesMagneticNorth, false);  // True heading: not written.
    ASSERT_TRUE(ASTERIXReporter::BuildCAT021Report(r, aircraft, ctx));
    EXPECT_TRUE(r.has_true_airspeed);
    EXPECT_EQ(r.true_airspeed_kts, 450);
    EXPECT_FALSE(r.has_magnetic_heading);
    EXPECT_TRUE(r.has_velocity_time);
}

TEST(ASTERIXUtils, BuildCAT021ReportModeSEmergencyAndNoUTC) {
    ModeSAircraft aircraft = AirborneModeSAircraft();
    aircraft.squawk = 7700;
    aircraft.WriteBitFlag(ModeSAircraft::kBitFlagAlert, true);
    ASTERIXReporter::ReportContext ctx;  // No UTC.
    ctx.timestamp_ms = 3000;
    CAT021TargetReport r;
    ASSERT_TRUE(ASTERIXReporter::BuildCAT021Report(r, aircraft, ctx));
    EXPECT_TRUE(r.no_go);
    EXPECT_EQ(r.position_time_of_day, 128u);  // Seconds since boot.
    EXPECT_TRUE(r.has_target_status);
    EXPECT_EQ(r.priority_status, 1);
    EXPECT_EQ(r.surveillance_status, 1);

    aircraft.WriteBitFlag(ModeSAircraft::kBitFlagIdent, true);
    ASSERT_TRUE(ASTERIXReporter::BuildCAT021Report(r, aircraft, ctx));
    EXPECT_EQ(r.surveillance_status, 3);
}

TEST(ASTERIXUtils, BuildCAT021ReportFromUATAircraft) {
    ASTERIXReporter::ReportContext ctx = Context(3000);
    UATAircraft aircraft(0xA00001);
    aircraft.WriteBitFlag(UATAircraft::kBitFlagIsAirborne, true);
    aircraft.WriteBitFlag(UATAircraft::kBitFlagPositionValid, true);
    aircraft.latitude_awb = DegToAWB(40.0f);
    aircraft.longitude_awb = DegToAWB(-105.0f);
    aircraft.last_position_update_ms = 2000;
    aircraft.WriteBitFlag(UATAircraft::kBitFlagHorizontalSpeedValid, true);
    aircraft.WriteBitFlag(UATAircraft::kBitFlagDirectionValid, true);
    aircraft.speed_kts = 120;
    aircraft.direction_deg = 90;
    aircraft.last_velocity_update_ms = 2000;
    aircraft.uat_version = 2;
    aircraft.emergency_priority_status = UATAircraft::kEmergencyStatusMinimumFuel;
    CAT021TargetReport r;
    ASSERT_TRUE(ASTERIXReporter::BuildCAT021Report(r, aircraft, ctx));
    EXPECT_EQ(r.address, 0xA00001u);
    EXPECT_EQ(r.address_type, ASTERIXReporter::kAddressTypeICAO);
    EXPECT_EQ(r.altitude_reporting_capability, ASTERIXReporter::kARC25Ft);
    EXPECT_EQ(r.service_id, ASTERIXReporter::kServiceIDUAT);
    EXPECT_EQ(r.link_technology, ASTERIXReporter::kLinkTechnologyUAT);
    EXPECT_EQ(r.mops_version, 2);
    EXPECT_EQ(r.position_time_of_day, kTimeOfDayTicks + 128);
    EXPECT_TRUE(r.has_ground_vector);
    EXPECT_EQ(r.priority_status, 3);
    EXPECT_FALSE(r.has_callsign);  // Still "?".
}

TEST(ASTERIXUtils, BuildCAT021ReportUATAddressQualifiers) {
    ASTERIXReporter::ReportContext ctx = Context(3000);
    CAT021TargetReport r;
    auto aircraft_with_qualifier = [](UATAircraft::AddressQualifier qualifier) {
        UATAircraft aircraft(0x123456 | (static_cast<uint32_t>(qualifier) << Aircraft::kAddressQualifierBitShift));
        aircraft.WriteBitFlag(UATAircraft::kBitFlagPositionValid, true);
        return aircraft;
    };
    ASSERT_TRUE(ASTERIXReporter::BuildCAT021Report(
        r, aircraft_with_qualifier(UATAircraft::kADSBTargetWithSelfAssignedTemporaryAddress), ctx));
    EXPECT_EQ(r.address_type, ASTERIXReporter::kAddressTypeAnonymous);
    EXPECT_EQ(r.address, 0x123456u);
    ASSERT_TRUE(ASTERIXReporter::BuildCAT021Report(r, aircraft_with_qualifier(UATAircraft::kSurfaceVehicle), ctx));
    EXPECT_EQ(r.address_type, ASTERIXReporter::kAddressTypeSurfaceVehicle);
    ASSERT_TRUE(ASTERIXReporter::BuildCAT021Report(r, aircraft_with_qualifier(UATAircraft::kFixedADSBBeacon), ctx));
    EXPECT_TRUE(r.from_field_monitor);
    EXPECT_EQ(r.service_id, ASTERIXReporter::kServiceIDUAT);

    // TIS-B and ADS-R go to the UAT rebroadcast service. Track files and non-ICAO addresses are anonymous.
    const struct {
        UATAircraft::AddressQualifier qualifier;
        ASTERIXReporter::AddressType address_type;
    } kRebroadcasts[] = {
        {UATAircraft::kTISBTargetWithICAO24BitAddress, ASTERIXReporter::kAddressTypeICAO},
        {UATAircraft::kTISBTargetWithTrackFileIdentifier, ASTERIXReporter::kAddressTypeAnonymous},
        {UATAircraft::kADSRTargetWithNonICAOAddress, ASTERIXReporter::kAddressTypeAnonymous},
    };
    for (const auto& c : kRebroadcasts) {
        ASSERT_TRUE(ASTERIXReporter::BuildCAT021Report(r, aircraft_with_qualifier(c.qualifier), ctx));
        EXPECT_EQ(r.address_type, c.address_type);
        EXPECT_EQ(r.service_id, ASTERIXReporter::kServiceIDUATRebroadcast);
        EXPECT_FALSE(r.from_field_monitor);
    }
}

TEST(ASTERIXUtils, BuildCAT021ReportModeSAddressTypes) {
    ASTERIXReporter::ReportContext ctx = Context(3000);
    CAT021TargetReport r;
    const struct {
        ModeSAircraft::AddressType type;
        ASTERIXReporter::AddressType address_type;
        uint8_t service_id;
        bool has_mops_version;
    } kCases[] = {
        {ModeSAircraft::kAddressTypeADSBICAO, ASTERIXReporter::kAddressTypeICAO, ASTERIXReporter::kServiceID1090ES,
         true},
        {ModeSAircraft::kAddressTypeADSBICAONonTransponder, ASTERIXReporter::kAddressTypeICAO,
         ASTERIXReporter::kServiceID1090ES, true},
        {ModeSAircraft::kAddressTypeADSBOther, ASTERIXReporter::kAddressTypeAnonymous,
         ASTERIXReporter::kServiceID1090ES, true},
        {ModeSAircraft::kAddressTypeADSRICAO, ASTERIXReporter::kAddressTypeICAO,
         ASTERIXReporter::kServiceID1090ESRebroadcast, true},
        {ModeSAircraft::kAddressTypeADSROther, ASTERIXReporter::kAddressTypeAnonymous,
         ASTERIXReporter::kServiceID1090ESRebroadcast, true},
        {ModeSAircraft::kAddressTypeTISBICAO, ASTERIXReporter::kAddressTypeICAO,
         ASTERIXReporter::kServiceID1090ESRebroadcast, false},
        {ModeSAircraft::kAddressTypeTISBTrackFile, ASTERIXReporter::kAddressTypeAnonymous,
         ASTERIXReporter::kServiceID1090ESRebroadcast, false},
        {ModeSAircraft::kAddressTypeTISBOther, ASTERIXReporter::kAddressTypeAnonymous,
         ASTERIXReporter::kServiceID1090ESRebroadcast, false},
    };
    for (const auto& c : kCases) {
        ModeSAircraft aircraft = AirborneModeSAircraft();
        aircraft.position_address_type = c.type;
        ASSERT_TRUE(ASTERIXReporter::BuildCAT021Report(r, aircraft, ctx));
        EXPECT_EQ(r.address_type, c.address_type) << int(c.type);
        EXPECT_EQ(r.service_id, c.service_id) << int(c.type);
        EXPECT_EQ(r.has_mops_version, c.has_mops_version) << int(c.type);
    }
}

TEST(ASTERIXUtils, TrackNumbersAreStableAndUnique) {
    static ASTERIXReporter::TrackNumbers track_numbers;
    track_numbers = ASTERIXReporter::TrackNumbers();
    track_numbers.StartRound();
    std::vector<uint16_t> numbers;
    for (uint32_t i = 0; i < kAircraftDictionaryMaxNumAircraft; i++) {
        uint16_t n;
        ASSERT_TRUE(track_numbers.Get(0x100000 + i * 37, n));
        EXPECT_LT(n, 4096);
        for (uint16_t other : numbers) ASSERT_NE(n, other);
        numbers.push_back(n);
    }
    track_numbers.StartRound();
    for (uint32_t i = 0; i < kAircraftDictionaryMaxNumAircraft; i++) {
        uint16_t n;
        ASSERT_TRUE(track_numbers.Get(0x100000 + i * 37, n));
        EXPECT_EQ(n, numbers[i]);
    }
}

TEST(ASTERIXUtils, TrackNumbersAreReusedOnlyWhenStale) {
    static ASTERIXReporter::TrackNumbers track_numbers;
    track_numbers = ASTERIXReporter::TrackNumbers();
    // Fill every slot, then one more UID finds no room.
    track_numbers.StartRound();
    uint16_t n;
    for (uint32_t i = 0; i < ASTERIXReporter::TrackNumbers::kNumSlots; i++) {
        ASSERT_TRUE(track_numbers.Get(i + 1, n));
    }
    EXPECT_FALSE(track_numbers.Get(0xFFFFFF, n));
    // Keep UID 1 alive; the rest go stale.
    uint16_t uid_1_number = 0;
    for (uint32_t round = 0; round <= ASTERIXReporter::TrackNumbers::kStaleRounds; round++) {
        track_numbers.StartRound();
        ASSERT_TRUE(track_numbers.Get(1, uid_1_number));
    }
    ASSERT_TRUE(track_numbers.Get(0xFFFFFF, n));
    EXPECT_NE(n, uid_1_number);
    uint16_t again;
    ASSERT_TRUE(track_numbers.Get(0xFFFFFF, again));
    EXPECT_EQ(again, n);
    ASSERT_TRUE(track_numbers.Get(1, again));
    EXPECT_EQ(again, uid_1_number);
}

TEST(ASTERIXUtils, CAT023GroundStationStatus) {
    uint8_t buf[ASTERIXReporter::kCAT023RecordMaxLenBytes];
    uint16_t len = ASTERIXReporter::WriteCAT023GroundStationStatusRecord(buf, sizeof(buf), 0, 1, kTimeOfDayTicks, true,
                                                                         false, 1);
    EXPECT_EQ(std::vector<uint8_t>(buf, buf + len),
              (std::vector<uint8_t>{0xD8, 0x00, 0x01, 0x01, 0x58, 0x78, 0x65, 0x01, 0x02}));

    // No UTC: time since boot, NOGO and TSV set; track numbering restarted.
    len = ASTERIXReporter::WriteCAT023GroundStationStatusRecord(buf, sizeof(buf), 0, 1, 384, false, true, 1);
    EXPECT_EQ(std::vector<uint8_t>(buf, buf + len),
              (std::vector<uint8_t>{0xD8, 0x00, 0x01, 0x01, 0x00, 0x01, 0x80, 0x8B, 0x02}));
}

TEST(ASTERIXUtils, CAT023ServiceStatus) {
    uint8_t buf[ASTERIXReporter::kCAT023RecordMaxLenBytes];
    uint16_t len = ASTERIXReporter::WriteCAT023ServiceStatusRecord(
        buf, sizeof(buf), 0, 1, ASTERIXReporter::kServiceIDUAT, ASTERIXReporter::kServiceTypeUAT, kTimeOfDayTicks, 2, 1,
        ASTERIXReporter::kServiceStatusNormal);
    // FSPEC FRN 1, 2, 3, 4, 6 | 8; type 2; SID 2 STYP 3; RP 1 s; SC 0; SSRP 1 s; STAT normal.
    EXPECT_EQ(std::vector<uint8_t>(buf, buf + len),
              (std::vector<uint8_t>{0xF5, 0x80, 0x00, 0x01, 0x02, 0x23, 0x58, 0x78, 0x65, 0x02, 0x01, 0x02, 0x08}));
}

TEST(ASTERIXUtils, DataBlockHeader) {
    uint8_t buf[300];
    uint16_t len = ASTERIXReporter::BeginDataBlock(buf, sizeof(buf), ASTERIXReporter::kCategoryADSBTargetReports);
    ASSERT_EQ(len, 3);
    len += 297;
    ASTERIXReporter::FinishDataBlock(buf, len);
    EXPECT_EQ(buf[0], 21);
    EXPECT_EQ(buf[1], 0x01);
    EXPECT_EQ(buf[2], 0x2C);
    EXPECT_EQ(ASTERIXReporter::BeginDataBlock(buf, 2, 23), 0);
}
