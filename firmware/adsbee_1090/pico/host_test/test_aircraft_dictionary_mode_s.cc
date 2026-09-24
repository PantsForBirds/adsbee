#include "aircraft_dictionary.hh"
#include "decode_utils.hh"  // for location calculation utility functions
#include "gtest/gtest.h"
#include "hal_god_powers.hh"  // for changing timestamp
#include "mode_s_packet.hh"

constexpr float kLatDegCloseEnough = 0.001f;
constexpr float kLonDegCloseEnough = 0.0001f;
constexpr float kFloatCloseEnough = 0.0001f;  // FOr generic floats.

TEST(AircraftDictionary, BasicInsertRemove) {
    AircraftDictionary dictionary = AircraftDictionary();
    EXPECT_EQ(dictionary.GetNumAircraft(), 0);
    EXPECT_FALSE(dictionary.RemoveAircraft(12345));

    ModeSAircraft test_aircraft = ModeSAircraft(12345);
    test_aircraft.emitter_category = ADSBTypes::kEmitterCategoryRotorcraft;
    dictionary.InsertAircraft(test_aircraft);
    EXPECT_EQ(dictionary.GetNumAircraft(), 1);
    EXPECT_TRUE(dictionary.ContainsAircraft(test_aircraft.icao_address));

    ModeSAircraft aircraft_out = ModeSAircraft(0);
    EXPECT_NE(aircraft_out.icao_address, test_aircraft.icao_address);
    EXPECT_NE(aircraft_out.emitter_category, test_aircraft.emitter_category);
    EXPECT_TRUE(dictionary.GetAircraft(test_aircraft.icao_address, aircraft_out));
    EXPECT_EQ(aircraft_out.icao_address, test_aircraft.icao_address);
    EXPECT_EQ(aircraft_out.emitter_category, test_aircraft.emitter_category);

    EXPECT_TRUE(dictionary.RemoveAircraft(12345));
    EXPECT_EQ(dictionary.GetNumAircraft(), 0);
}

TEST(AircraftDictionary, InsertThenRemoveTooMany) {
    AircraftDictionary dictionary = AircraftDictionary();
    EXPECT_EQ(dictionary.GetNumAircraft(), 0);

    ModeSAircraft test_aircraft = ModeSAircraft(0);
    test_aircraft.emitter_category = ADSBTypes::kEmitterCategoryGliderSailplane;

    // Insert maximum number of aircraft.
    for (uint16_t i = 0; i < AircraftDictionary::kMaxNumAircraft; i++) {
        test_aircraft.icao_address = i * 599;
        EXPECT_TRUE(dictionary.InsertAircraft(test_aircraft));
        EXPECT_EQ(dictionary.GetNumAircraft(), i + 1);
    }

    // Changing an existing aircraft should be fine.
    test_aircraft.icao_address = 3 * 599;
    EXPECT_TRUE(dictionary.InsertAircraft(test_aircraft));
    EXPECT_TRUE(dictionary.GetAircraftPtr<ModeSAircraft>(
        Aircraft::ICAOToUID(test_aircraft.icao_address, Aircraft::kAircraftTypeModeS)));

    // Remove all aircraft.
    for (uint16_t i = 0; i < AircraftDictionary::kMaxNumAircraft; i++) {
        test_aircraft.icao_address = (AircraftDictionary::kMaxNumAircraft - 1 - i) * 599;
        EXPECT_TRUE(dictionary.RemoveAircraft(test_aircraft.icao_address));
        EXPECT_EQ(dictionary.GetNumAircraft(), AircraftDictionary::kMaxNumAircraft - i - 1);
    }

    // Removing a nonexistent aircraft with a previously valid ICAO address should fail.
    EXPECT_EQ(dictionary.GetNumAircraft(), 0);
    EXPECT_FALSE(dictionary.RemoveAircraft(0));
}

TEST(AircraftDictionary, UseAircraftPtr) {
    AircraftDictionary dictionary = AircraftDictionary();
    ModeSAircraft* aircraft =
        dictionary.GetAircraftPtr<ModeSAircraft>(Aircraft::ICAOToUID(12345, Aircraft::kAircraftTypeModeS));
    EXPECT_TRUE(aircraft);  // aircraft should have been automatically inserted just fine
    aircraft->emitter_category = ADSBTypes::kEmitterCategoryLineObstacle;
    ModeSAircraft aircraft_out;
    ASSERT_TRUE(dictionary.GetAircraft(12345, aircraft_out));
    ASSERT_EQ(aircraft_out.emitter_category, ADSBTypes::kEmitterCategoryLineObstacle);
    aircraft->emitter_category = ADSBTypes::kEmitterCategoryHeavy;
    ASSERT_TRUE(dictionary.GetAircraft(12345, aircraft_out));
    ASSERT_EQ(aircraft_out.emitter_category, ADSBTypes::kEmitterCategoryHeavy);
}

TEST(AircraftDictionary, AccessFakeAircraft) {
    AircraftDictionary dictionary = AircraftDictionary();
    EXPECT_EQ(dictionary.GetNumAircraft(), 0);

    ModeSAircraft test_aircraft = ModeSAircraft(0);
    EXPECT_FALSE(dictionary.GetAircraft(0, test_aircraft));
}

TEST(AircraftDictionary, ApplyAircraftIDMessage) {
    AircraftDictionary dictionary = AircraftDictionary();
    DecodedModeSPacket tpacket = DecodedModeSPacket((char*)"8D76CE88204C9072CB48209A504D");
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(tpacket));
    EXPECT_EQ(dictionary.GetNumAircraft(), 1);
    ModeSAircraft aircraft;
    EXPECT_TRUE(dictionary.GetAircraft(0x76CE88, aircraft));
    EXPECT_EQ(aircraft.icao_address, 0x76CE88u);
    EXPECT_EQ(aircraft.transponder_capability, 5);
    EXPECT_EQ(aircraft.emitter_category, ADSBTypes::kEmitterCategoryNoCategoryInfo);
    EXPECT_STREQ(aircraft.callsign, "SIA224  ");

    tpacket = DecodedModeSPacket((char*)"8D7C7181215D01A08208204D8BF1");
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(tpacket));
    EXPECT_EQ(dictionary.GetNumAircraft(), 2);
    EXPECT_TRUE(dictionary.GetAircraft(0x7C7181, aircraft));
    EXPECT_EQ(aircraft.icao_address, 0x7C7181u);
    EXPECT_EQ(aircraft.transponder_capability, 5);
    EXPECT_EQ(aircraft.emitter_category, ADSBTypes::kEmitterCategoryLight);
    EXPECT_STREQ(aircraft.callsign, "WPF     ");

    tpacket = DecodedModeSPacket((char*)"8D7C7745226151A08208205CE9C2");
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(tpacket));
    EXPECT_EQ(dictionary.GetNumAircraft(), 3);
    EXPECT_TRUE(dictionary.GetAircraft(0x7C7745, aircraft));
    EXPECT_EQ(aircraft.icao_address, 0x7C7745u);
    EXPECT_EQ(aircraft.transponder_capability, 5);
    EXPECT_EQ(aircraft.emitter_category, ADSBTypes::kEmitterCategoryMedium1);
    EXPECT_STREQ(aircraft.callsign, "XUF     ");

    tpacket = DecodedModeSPacket((char*)"8D7C80AD2358F6B1E35C60FF1925");
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(tpacket));
    EXPECT_EQ(dictionary.GetNumAircraft(), 4);
    EXPECT_TRUE(dictionary.GetAircraft(0x7C80AD, aircraft));
    EXPECT_EQ(aircraft.icao_address, 0x7C80ADu);
    EXPECT_EQ(aircraft.transponder_capability, 5);
    EXPECT_EQ(aircraft.emitter_category, ADSBTypes::kEmitterCategoryMedium2);
    EXPECT_STREQ(aircraft.callsign, "VOZ1851 ");

    tpacket = DecodedModeSPacket((char*)"8D7C146525446074DF5820738E90");
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(tpacket));
    EXPECT_EQ(dictionary.GetNumAircraft(), 5);
    EXPECT_TRUE(dictionary.GetAircraft(0x7C1465, aircraft));
    EXPECT_EQ(aircraft.icao_address, 0x7C1465u);
    EXPECT_EQ(aircraft.transponder_capability, 5);
    EXPECT_EQ(aircraft.emitter_category, ADSBTypes::kEmitterCategoryHeavy);
    EXPECT_STREQ(aircraft.callsign, "QFA475  ");

    tpacket = DecodedModeSPacket((char*)"8D4840D6202CC371C32CE0576098");
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(tpacket));
    EXPECT_TRUE(dictionary.GetAircraft(0x4840D6, aircraft));
    EXPECT_STREQ(aircraft.callsign, "KLM1023 ");
}

TEST(AircraftDictionary, IngestInvalidAircrfaftIDMessage) {
    AircraftDictionary dictionary = AircraftDictionary();
    DecodedModeSPacket tpacket = DecodedModeSPacket((char*)"7D76CE88204C9072CB48209A504D");
    EXPECT_FALSE(dictionary.IngestDecodedModeSPacket(tpacket));
    EXPECT_EQ(dictionary.GetNumAircraft(), 0);
}

TEST(ModeSAircraft, SetCPRLatLon) {
    ModeSAircraft aircraft;
    EXPECT_FALSE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagPositionValid));

    // Send n_lat_cpr out of bounds (bigger than 2^17 bits max value).
    EXPECT_FALSE(aircraft.SetCPRLatLon(0xFFFFFF, 53663, true, get_time_since_boot_ms()));
    EXPECT_FALSE(aircraft.SetCPRLatLon(0xFFFFFF, 53663, false, get_time_since_boot_ms()));
    // Send n_lon_cpr out of bounds (bigger than 2^17 bits max value).
    EXPECT_FALSE(aircraft.SetCPRLatLon(52455, 0xFFFFFF, false, get_time_since_boot_ms()));
    EXPECT_FALSE(aircraft.SetCPRLatLon(52455, 0xFFFFFF, true, get_time_since_boot_ms()));

    // Send two even packets at startup, no odd packets.
    aircraft = ModeSAircraft();  // clear everything
    EXPECT_FALSE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedPosition));
    inc_time_since_boot_ms();
    EXPECT_TRUE(aircraft.SetCPRLatLon(578, 13425, false, get_time_since_boot_ms()));
    inc_time_since_boot_ms();
    EXPECT_TRUE(aircraft.SetCPRLatLon(578, 4651, false, get_time_since_boot_ms()));
    EXPECT_FALSE(aircraft.DecodeAirbornePosition());
    EXPECT_FALSE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagPositionValid));

    // Send two odd packets at startup, no even packets.
    aircraft = ModeSAircraft();  // clear everything
    inc_time_since_boot_ms();
    EXPECT_TRUE(aircraft.SetCPRLatLon(236, 13425, true, get_time_since_boot_ms()));
    inc_time_since_boot_ms();
    EXPECT_TRUE(aircraft.SetCPRLatLon(236, 857, true, get_time_since_boot_ms()));
    EXPECT_FALSE(aircraft.DecodeAirbornePosition());
    EXPECT_FALSE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagPositionValid));

    // Send one odd packet and one even packet at startup.
    aircraft = ModeSAircraft();  // clear everything
    inc_time_since_boot_ms();
    EXPECT_TRUE(aircraft.SetCPRLatLon(74158, 50194, true, get_time_since_boot_ms()));
    inc_time_since_boot_ms();
    EXPECT_TRUE(aircraft.SetCPRLatLon(93000, 51372, false, get_time_since_boot_ms()));
    EXPECT_TRUE(aircraft.DecodeAirbornePosition());
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagPositionValid));
    EXPECT_NEAR(aircraft.latitude_deg, 52.25720f, 1e-4);  // even latitude
    EXPECT_NEAR(aircraft.longitude_deg, 3.91937f, 1e-4);  // longitude calculated from even latitude

    // Send one even packet and one odd packet at startup.
    aircraft = ModeSAircraft();  // clear everything
    inc_time_since_boot_ms();
    EXPECT_TRUE(aircraft.SetCPRLatLon(93000, 51372, false, get_time_since_boot_ms()));
    inc_time_since_boot_ms();
    EXPECT_TRUE(aircraft.SetCPRLatLon(74158, 50194, true, get_time_since_boot_ms()));
    EXPECT_TRUE(aircraft.DecodeAirbornePosition());
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagPositionValid));
    EXPECT_NEAR(aircraft.latitude_deg, 52.26578f, 1e-4);  // odd latitude
    // don't have a test value available for the longitude calculated from odd latitude

    // Straddle two position packets between different latitude bands.
    aircraft = ModeSAircraft();  // clear everything
    EXPECT_TRUE(aircraft.SetCPRLatLon(93006, 50194, true, get_time_since_boot_ms()));
    inc_time_since_boot_ms(1000);
    EXPECT_TRUE(aircraft.SetCPRLatLon(93000, 51372, false, get_time_since_boot_ms()));
    inc_time_since_boot_ms(1000);
    EXPECT_TRUE(aircraft.DecodeAirbornePosition());
    inc_time_since_boot_ms(1000);
    // Position established, now send the curveball.
    EXPECT_TRUE(aircraft.SetCPRLatLon(93000 - 5000, 50194, true, get_time_since_boot_ms()));
    inc_time_since_boot_ms(1000);
// Re-decode the previous even CPR packet with the new zone.
#ifdef FILTER_CPR_POSITIONS
    EXPECT_FALSE(aircraft.DecodeAirbornePosition());  // This should fail due to the CPR filter.
#else
    EXPECT_TRUE(aircraft.DecodeAirbornePosition());
#endif
}

TEST(AircraftDictionary, ApplyAirbornePositionMessage) {
    AircraftDictionary dictionary = AircraftDictionary();
    DecodedModeSPacket even_tpacket = DecodedModeSPacket((char*)"8da6147f5859f18cdf4d244ac6fa");
    ASSERT_TRUE(even_tpacket.is_valid);
    DecodedModeSPacket odd_tpacket = DecodedModeSPacket((char*)"8da6147f585b05533e2ba73e43cb");
    ASSERT_TRUE(odd_tpacket.is_valid);

    ASSERT_EQ(dictionary.GetNumAircraft(), 0);

    // Set time since boot to something positive so packet ingestion time looks legit.
    set_time_since_boot_ms(1e3);
    even_tpacket.raw.mlat_48mhz_64bit_counts = get_time_since_boot_ms() * 48e3;

    // Ingest even packet.
    ASSERT_TRUE(dictionary.IngestDecodedModeSPacket(even_tpacket));
    ASSERT_EQ(dictionary.GetNumAircraft(), 1);
    auto itr = dictionary.dict.begin();
    auto& aircraft = std::get<ModeSAircraft>(itr->second);

    // Aircraft should exist but not have its location filled out.
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne));
    EXPECT_FALSE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedPosition));
    ASSERT_EQ(aircraft.icao_address, (uint32_t)0xA6147F);
    EXPECT_FLOAT_EQ(aircraft.latitude_deg, 0.0f);
    EXPECT_FLOAT_EQ(aircraft.longitude_deg, 0.0f);
    // Altitude should be filled out.
    EXPECT_EQ(aircraft.altitude_source, ADSBTypes::kAltitudeSourceBaro);
    EXPECT_EQ(aircraft.baro_altitude_ft, 16975);

    inc_time_since_boot_ms(1e3);  // Simulate time passing between odd and even packet ingestion.
    odd_tpacket.raw.mlat_48mhz_64bit_counts = get_time_since_boot_ms() * 48e3;

    // Ingest odd packet.
    ASSERT_TRUE(dictionary.IngestDecodedModeSPacket(odd_tpacket));

    // Aircraft should now have a valid location.
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedPosition));
    ASSERT_EQ(aircraft.icao_address, 0xA6147Fu);
    EXPECT_NEAR(aircraft.latitude_deg, 20.326522568524894f, kLatDegCloseEnough);
    EXPECT_NEAR(aircraft.longitude_deg, -156.5328535600142f, kLonDegCloseEnough);
    // Altitude should be filled out.
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedBaroAltitude));
    EXPECT_EQ(aircraft.altitude_source, ADSBTypes::kAltitudeSourceBaro);
    EXPECT_EQ(aircraft.baro_altitude_ft, 17000);
}

TEST(ModeSAircraft, CalculateMaxAllowedCPRInterval) {
    ModeSAircraft aircraft;
    // CPR interval enforced at reference limit when aircraft is not initialized.
    EXPECT_EQ(aircraft.GetMaxAllowedCPRIntervalMs(), ModeSAircraft::kDefaultCPRIntervalMs);

    // Setting velocity source to something other than kSpeedSourceNotAvailable or kSpeedSourceNotSet should
    // return CPR interval as a calculated function of aircraft velocity.
    aircraft.speed_source = ADSBTypes::kSpeedSourceGroundSpeed;

    // Stale track enforces default CPR interval.
    set_time_since_boot_ms(100e3);
    aircraft.last_track_update_timestamp_ms = 100e3 - ModeSAircraft::kMaxTrackUpdateIntervalMs - 1;
    EXPECT_EQ(aircraft.GetMaxAllowedCPRIntervalMs(), ModeSAircraft::kDefaultCPRIntervalMs);

    // Set track to be fresh.
    aircraft.last_track_update_timestamp_ms = 100e3;

    // Stationary aircraft = maximum allowed CPR interval.
    aircraft.speed_kts = 0;
    EXPECT_EQ(aircraft.GetMaxAllowedCPRIntervalMs(), ModeSAircraft::kMaxCPRIntervalMs);

    // Mid-speed aircraft = calculated CPR interval between max and min allowed.
    aircraft.speed_kts = 400;
    EXPECT_EQ(aircraft.GetMaxAllowedCPRIntervalMs(), ModeSAircraft::kRefCPRIntervalMs * 500 / aircraft.speed_kts);

    // Very fast aircraft = same equation, no minimum interval enforced.
    aircraft.speed_kts = 1000;
    EXPECT_EQ(aircraft.GetMaxAllowedCPRIntervalMs(), ModeSAircraft::kRefCPRIntervalMs * 500 / aircraft.speed_kts);
}

// This test case verifies that you can't ingest airborne position messages that are too far apart in time, which could
// lead to an invalid decode.
TEST(AircraftDictionary, TimeFilterAirbornePositionMessages) {
    AircraftDictionary dictionary = AircraftDictionary();
    RawModeSPacket even_tpacket = RawModeSPacket((char*)"8da6147f5859f18cdf4d244ac6fa");
    RawModeSPacket odd_tpacket = RawModeSPacket((char*)"8da6147f585b05533e2ba73e43cb");

    even_tpacket.mlat_48mhz_64bit_counts = 0;
    DecodedModeSPacket even_packet = DecodedModeSPacket(even_tpacket);

    // Ingest the even position packet. This should add the aircraft to the dictionary but not provide a valid
    // position.
    ASSERT_TRUE(dictionary.IngestDecodedModeSPacket(even_packet));
    ASSERT_EQ(dictionary.GetNumAircraft(), 1);
    auto aircraft = std::get<ModeSAircraft>(dictionary.dict.begin()->second);
    ASSERT_FALSE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagPositionValid));

    // Ensure that the system timer and aircraft track updated timestamp are in sync and won't get in the way.
    set_time_since_boot_ms(100e3);
    aircraft.last_track_update_timestamp_ms = 99e3;

    // Case 1: Aircraft has no speed data. Default packet valid interval should be used.
    ASSERT_EQ(aircraft.speed_source, ADSBTypes::kSpeedSourceNotSet);
    EXPECT_EQ(aircraft.GetMaxAllowedCPRIntervalMs(), ModeSAircraft::kDefaultCPRIntervalMs);
    // Ingest the odd position packet. This should be rejected since the timestamp is too far apart from the even
    // packet. Ingestion will succeed, and the packet will be retained, but the aircraft will still not have a valid
    // location.
    odd_tpacket.mlat_48mhz_64bit_counts =
        even_tpacket.mlat_48mhz_64bit_counts + ModeSAircraft::kDefaultCPRIntervalMs * 48e9;
    DecodedModeSPacket odd_packet = DecodedModeSPacket(odd_tpacket);
    ASSERT_TRUE(dictionary.IngestDecodedModeSPacket(odd_packet));
    EXPECT_FALSE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagPositionValid));

    // Reset by ingesting even packet again.
    ASSERT_TRUE(dictionary.IngestDecodedModeSPacket(even_packet));

    // Case 2: Aircraft has speed data and is traveling at 1000 knots.
    aircraft.speed_kts = 1000;
    aircraft.speed_source = ADSBTypes::kSpeedSourceGroundSpeed;
    EXPECT_EQ(aircraft.GetMaxAllowedCPRIntervalMs(), ModeSAircraft::kRefCPRIntervalMs * 500 / aircraft.speed_kts);

    // Case 3: Aircraft is flying slowly but has a stale track.
    aircraft.speed_kts = 0;
    aircraft.speed_source = ADSBTypes::kSpeedSourceGroundSpeed;
    // Stationary aircraft should get the max interval.
    EXPECT_EQ(aircraft.GetMaxAllowedCPRIntervalMs(), ModeSAircraft::kMaxCPRIntervalMs);
    // Set the track update timestamp to be too old. This should enforce the default CPR interval.
    aircraft.last_track_update_timestamp_ms = get_time_since_boot_ms() - ModeSAircraft::kMaxTrackUpdateIntervalMs - 1;
    EXPECT_EQ(aircraft.GetMaxAllowedCPRIntervalMs(), ModeSAircraft::kDefaultCPRIntervalMs);
}

TEST(AircraftDictionary, IngestAirbornePositionBaroAltitudeNonGillham) {
    AircraftDictionary dictionary = AircraftDictionary();
    DecodedModeSPacket tpacket = DecodedModeSPacket((char*)"8d89611348db01c6ea41c4c7b8bf");
    ASSERT_TRUE(tpacket.is_valid);
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(tpacket));
    EXPECT_EQ(dictionary.GetNumAircraft(), 1);
    auto itr = dictionary.dict.begin();
    auto& aircraft = std::get<ModeSAircraft>(itr->second);
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedBaroAltitude));
    EXPECT_EQ(aircraft.altitude_source, ADSBTypes::kAltitudeSourceBaro);
    EXPECT_EQ(aircraft.baro_altitude_ft, 42600);
}

TEST(AircraftDictionary, IngestAirbornePositionNonGillhamMax) {
    // Test with an an altitude code of 0b111'11111111, which corresponds to 0b1111'11111111 with a "Q" bit of 1, which
    // inicates baro altitude encoding as 25*N - 1000ft, where N is the value of the altitude code with the Q bit
    // removed.
    // The maximum altitude encodeable as a baro altitude in this manner is 50175 ft.
    AircraftDictionary dictionary = AircraftDictionary();
    DecodedModeSPacket tpacket = DecodedModeSPacket((char*)"8d89611348fff1c6ea41c4a16b95");
    // EXPECT_EQ(tpacket.CalculateCRC24(), 0xfff);  // Use this to get the CRC.
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(tpacket));
    EXPECT_EQ(dictionary.GetNumAircraft(), 1);
    auto itr = dictionary.dict.begin();
    auto& aircraft = std::get<ModeSAircraft>(itr->second);
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedBaroAltitude));
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagBaroAltitudeValid));
    EXPECT_EQ(aircraft.altitude_source, ADSBTypes::AltitudeSource::kAltitudeSourceBaro);
    EXPECT_EQ(aircraft.baro_altitude_ft, 50175);
}

TEST(AircraftDictionary, IngestAirbornePositionBaroAltitudeInvalidOrUnavailable) {
    AircraftDictionary dictionary = AircraftDictionary();
    DecodedModeSPacket tpacket = DecodedModeSPacket((char*)"8d896113480001c6ea41c4c7b8bf");
    // We messed with the altitude code without changing the CRC, so paper that over before ingesting.
    tpacket.is_valid = true;
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(tpacket));
    EXPECT_EQ(dictionary.GetNumAircraft(), 1);
    auto itr = dictionary.dict.begin();
    auto& aircraft = std::get<ModeSAircraft>(itr->second);
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedBaroAltitude));
    EXPECT_FALSE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagBaroAltitudeValid));
    EXPECT_EQ(aircraft.altitude_source, ADSBTypes::kAltitudeSourceNotAvailable);
}

TEST(AircraftDictionary, IngestAirbornePositionBaroAltitudeGillham) {
    // Test with a gillham code of 0b0111000110110, which corresponds to 21950 ft.
    //                                   M Q
    //                             M Q                 Q
    // Remove extra "M" bit: 0b0111000110110 -> 0b011100110110 = 0x736.
    // Note that the "Q" bit is 0, which means the altitude is gillham encoded just like the Mode C reply, but without
    // an "M" bit.
    AircraftDictionary dictionary = AircraftDictionary();
    DecodedModeSPacket tpacket = DecodedModeSPacket((char*)"8d896113487361c6ea41c40703ed");
    // EXPECT_EQ(tpacket.CalculateCRC24(), 0xfff); // Use this to get the CRC.
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(tpacket));
    EXPECT_EQ(dictionary.GetNumAircraft(), 1);
    auto itr = dictionary.dict.begin();
    auto& aircraft = std::get<ModeSAircraft>(itr->second);
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedBaroAltitude));
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagBaroAltitudeValid));
    EXPECT_EQ(aircraft.altitude_source, ADSBTypes::AltitudeSource::kAltitudeSourceBaro);
    EXPECT_EQ(aircraft.baro_altitude_ft, 21950);
}

TEST(AircraftDictionary, IngestAirbornePositionGNSSAltitude) {
    AircraftDictionary dictionary = AircraftDictionary();
    // DF = 17, TC = 20-22 indicates airborne position with GNSS altitude.
    DecodedModeSPacket tpacket = DecodedModeSPacket((char*)"8E7C6296A0A0A59C64468D6F4EDD");
    // EXPECT_EQ(tpacket.CalculateCRC24(), 0xfff);  // Use this to get the CRC.
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(tpacket));
    EXPECT_EQ(dictionary.GetNumAircraft(), 1);
    auto itr = dictionary.dict.begin();
    auto& aircraft = std::get<ModeSAircraft>(itr->second);
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedGNSSAltitude));
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagGNSSAltitudeValid));
    EXPECT_EQ(aircraft.altitude_source, ADSBTypes::AltitudeSource::kAltitudeSourceGNSS);
    EXPECT_EQ(aircraft.gnss_altitude_ft, 8431);
}

TEST(AircraftDictionary, IngestAirborneVelocityMessage) {
    AircraftDictionary dictionary = AircraftDictionary();
    DecodedModeSPacket tpacket = DecodedModeSPacket((char*)"8dae56bc99246508b8080b6c230f");
    ASSERT_TRUE(tpacket.is_valid);
    ModeSADSBPacket packet = ModeSADSBPacket(tpacket);
    ASSERT_EQ(packet.GetTypeCodeEnum(), ModeSADSBPacket::TypeCode::kTypeCodeAirborneVelocities);

    // Ingest the airborne velocities packet.
    ASSERT_TRUE(dictionary.IngestModeSADSBPacket(packet));
    ASSERT_EQ(dictionary.GetNumAircraft(), 1);
    auto itr = dictionary.dict.begin();
    auto& aircraft =
        std::get<ModeSAircraft>(itr->second);  // NOTE: Aircraft is a mutable reference until we get to Message A!

    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagDirectionValid));
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagHorizontalSpeedValid));
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagBaroVerticalRateValid));

    // Aircraft should now have velocities populated.
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedDirection));
    EXPECT_NEAR(aircraft.direction_deg, 304.2157021324374, 0.001);
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedHorizontalSpeed));
    EXPECT_EQ(aircraft.speed_kts, 121);
    EXPECT_EQ(aircraft.speed_source, ADSBTypes::kSpeedSourceGroundSpeed);
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedBaroVerticalRate));
    EXPECT_EQ(aircraft.baro_vertical_rate_fpm, -64);
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagBaroVerticalRateValid));

    // Test Message A from https://mode-s.org/decode/content/ads-b/5-airborne-velocity.html
    tpacket = DecodedModeSPacket((char*)"8D485020994409940838175B284F");
    ASSERT_TRUE(tpacket.is_valid);
    packet = ModeSADSBPacket(tpacket);
    ASSERT_EQ(packet.GetTypeCodeEnum(), ModeSADSBPacket::TypeCode::kTypeCodeAirborneVelocities);
    ASSERT_TRUE(dictionary.IngestModeSADSBPacket(packet));
    uint32_t message_a_icao = 0x485020;
    ASSERT_TRUE(dictionary.ContainsAircraft(message_a_icao));
    ASSERT_TRUE(dictionary.GetAircraft(message_a_icao, aircraft));  // NOTE: Aircraft is read-only now!

    // Check values for Message A
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedGNSSVerticalRate));
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedHorizontalSpeed));
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedDirection));
    EXPECT_EQ(aircraft.gnss_vertical_rate_fpm, -832);
    EXPECT_EQ(aircraft.speed_source, ADSBTypes::kSpeedSourceGroundSpeed);
    EXPECT_NEAR(aircraft.direction_deg, 182.88f, 0.01);
    EXPECT_EQ(aircraft.speed_kts, 159);

    // Test altitude difference between baro and GNSS altitude for Message A by re-ingesting.
    ModeSAircraft* aircraft_ptr =
        dictionary.GetAircraftPtr<ModeSAircraft>(Aircraft::ICAOToUID(0x485020, Aircraft::kAircraftTypeModeS));
    aircraft_ptr->baro_altitude_ft = 2000;
    aircraft_ptr->altitude_source = ADSBTypes::kAltitudeSourceBaro;
    // Re-ingest message A to make sure the GNSS altitude gets corrected.
    ASSERT_TRUE(dictionary.IngestModeSADSBPacket(packet));
    EXPECT_FALSE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedBaroAltitude));
    EXPECT_TRUE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedGNSSAltitude));
    ASSERT_EQ(aircraft_ptr->gnss_altitude_ft, 2000 + 550);  // GNSS altitude is 550ft above baro altitude.

    // Test Message B from https://mode-s.org/decode/content/ads-b/5-airborne-velocity.html
    tpacket = DecodedModeSPacket((char*)"8DA05F219B06B6AF189400CBC33F");
    ASSERT_TRUE(tpacket.is_valid);
    packet = ModeSADSBPacket(tpacket);
    ASSERT_EQ(packet.GetTypeCodeEnum(), ModeSADSBPacket::TypeCode::kTypeCodeAirborneVelocities);
    ASSERT_TRUE(dictionary.IngestModeSADSBPacket(packet));
    uint32_t message_b_icao = 0xA05F21;
    ASSERT_TRUE(dictionary.ContainsAircraft(message_b_icao));
    ASSERT_TRUE(dictionary.GetAircraft(message_b_icao, aircraft));

    // Check values for Message B
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedBaroVerticalRate));
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedHorizontalSpeed));
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedDirection));
    EXPECT_EQ(aircraft.baro_vertical_rate_fpm, -2304);
    EXPECT_EQ(aircraft.speed_source, ADSBTypes::kSpeedSourceAirspeedTrue);
    EXPECT_NEAR(aircraft.direction_deg, 243.98f, 0.01);
    EXPECT_NEAR(aircraft.speed_kts, 375.0f, 0.01);
}

TEST(AircraftDictionary, IngestAltitudeReply) {
    ModeSAircraft* aircraft_ptr;
    // Try ingesting a altitude reply packet that's marked as valid so that it doesn't require a cross-check with the
    // dictionary.
    AircraftDictionary dictionary = AircraftDictionary();
    DecodedModeSPacket tpacket = DecodedModeSPacket((char*)"200006A2DE8B1C");
    EXPECT_EQ(tpacket.icao_address, 0x7C1B28u);
    aircraft_ptr = dictionary.InsertAircraft<ModeSAircraft>(
        ModeSAircraft(0x7C1B28u));  // Put aircraft in the dictionary so the packet can be ingested.
    ASSERT_TRUE(aircraft_ptr);
    EXPECT_FALSE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagBaroAltitudeValid));
    EXPECT_FALSE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedBaroAltitude));
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(tpacket));

    // Check that the aircraft has the right altitude.
    EXPECT_FALSE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagIdent));
    EXPECT_EQ(aircraft_ptr->baro_altitude_ft, 10000);
    EXPECT_TRUE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagBaroAltitudeValid));
    EXPECT_TRUE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagUpdatedBaroAltitude));
    EXPECT_TRUE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne));
    EXPECT_FALSE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagIdent));
    EXPECT_FALSE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagAlert));

    // Ingest a altitude reply packet with an alert and ident.
    tpacket = DecodedModeSPacket((char*)"24000E3956BBA1");
    // Add aircraft to dictionary so packet can be ingested.
    dictionary.InsertAircraft(ModeSAircraft(tpacket.icao_address));
    aircraft_ptr =
        dictionary.GetAircraftPtr<ModeSAircraft>(Aircraft::ICAOToUID(0xD3CCBFu, Aircraft::kAircraftTypeModeS));
    EXPECT_FALSE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagBaroAltitudeValid));
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(tpacket));
    EXPECT_EQ(aircraft_ptr->baro_altitude_ft, 22025);
    EXPECT_FALSE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne));
    EXPECT_TRUE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagIdent));
    EXPECT_TRUE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagAlert));

    // Ingest a altitude reply packet with aircraft on the ground.
    tpacket = DecodedModeSPacket((char*)"210000992F8C48");
    // Add aircraft to dictionary so packet can be ingested.
    dictionary.InsertAircraft(ModeSAircraft(tpacket.icao_address));
    aircraft_ptr =
        dictionary.GetAircraftPtr<ModeSAircraft>(Aircraft::ICAOToUID(0x7C7539u, Aircraft::kAircraftTypeModeS));
    EXPECT_FALSE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagBaroAltitudeValid));
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(tpacket));
    EXPECT_EQ(aircraft_ptr->baro_altitude_ft, 25);
    EXPECT_FALSE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne));
    EXPECT_FALSE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagIdent));
    EXPECT_FALSE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagAlert));
    EXPECT_TRUE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagBaroAltitudeValid));
}

TEST(AircraftDictionary, IngestIdentityReply) {
    // Ingest a identity reply packet with an alert and ident.
    AircraftDictionary dictionary = AircraftDictionary();
    DecodedModeSPacket tpacket = DecodedModeSPacket((char*)"2C0006A2DEE500");
    // Add aircraft to dictioanry so packet can be ingested.
    dictionary.InsertAircraft(ModeSAircraft(tpacket.icao_address));
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(tpacket));
    ModeSAircraft aircraft;
    EXPECT_TRUE(dictionary.GetAircraft(0x739EE9u, aircraft));
    EXPECT_EQ(aircraft.squawk, 06520u);
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagAlert));
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagIdent));

    // Ingest a identity reply packet with an ident but no alert.
    tpacket = DecodedModeSPacket((char*)"2D0006A2DEE500");
    // Add aircraft to dictioanry so packet can be ingested.
    dictionary.InsertAircraft(ModeSAircraft(tpacket.icao_address));
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(tpacket));
    EXPECT_TRUE(dictionary.GetAircraft(0x5863BAu, aircraft));
    EXPECT_EQ(aircraft.squawk, 06520u);
    EXPECT_FALSE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagAlert));
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagIdent));

    // Ingest a identity reply packet with no ident and no alert. Aircraft is airborne.
    tpacket = DecodedModeSPacket((char*)"28000D08CEE4C5");
    // Add aircraft to dictioanry so packet can be ingested.
    dictionary.InsertAircraft(ModeSAircraft(tpacket.icao_address));
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(tpacket));
    EXPECT_TRUE(dictionary.GetAircraft(0xA8BBE7u, aircraft));
    EXPECT_EQ(aircraft.squawk, 01260);
    EXPECT_FALSE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagAlert));
    EXPECT_FALSE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagIdent));
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne));

    // Ingest a identity reply packet with no ident and no alert. Aircraft is on ground.
    tpacket = DecodedModeSPacket((char*)"29001E0D3CB4BF");
    // Add aircraft to dictioanry so packet can be ingested.
    dictionary.InsertAircraft(ModeSAircraft(tpacket.icao_address));
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(tpacket));
    EXPECT_TRUE(dictionary.GetAircraft(0x7C1471u, aircraft));
    EXPECT_EQ(aircraft.squawk, 03236);
    EXPECT_FALSE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagAlert));
    EXPECT_FALSE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagIdent));
    EXPECT_FALSE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne));
}

TEST(AircraftDictionary, AmbiguousFlightStatusDoesNotClearAirborneFlag) {
    // An aircraft already known to be airborne must stay airborne when an ambiguous FS packet
    // (FS=0b100 or 0b101) arrives. Previously these packets incorrectly reset the airborne flag.
    AircraftDictionary dictionary = AircraftDictionary();
    ModeSAircraft* aircraft_ptr;

    // --- Altitude Reply, FS=0b100 (Alert+SPI, ambiguous) ---
    // Packet "24000E3956BBA1" is DF=4, FS=0b100.
    DecodedModeSPacket tpacket = DecodedModeSPacket((char*)"24000E3956BBA1");
    dictionary.InsertAircraft(ModeSAircraft(tpacket.icao_address));
    aircraft_ptr =
        dictionary.GetAircraftPtr<ModeSAircraft>(Aircraft::ICAOToUID(0xD3CCBFu, Aircraft::kAircraftTypeModeS));
    ASSERT_TRUE(aircraft_ptr);
    // Simulate an earlier position message having set the aircraft airborne.
    aircraft_ptr->WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne, true);
    ASSERT_TRUE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne));
    // Ingest the ambiguous packet — airborne flag must not be overwritten.
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(tpacket));
    EXPECT_TRUE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne));
    EXPECT_TRUE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagIdent));
    EXPECT_TRUE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagAlert));

    // --- Identity Reply, FS=0b101 (No Alert+SPI, ambiguous) ---
    // Packet "2D0006A2DEE500" is DF=5, FS=0b101.
    tpacket = DecodedModeSPacket((char*)"2D0006A2DEE500");
    dictionary.InsertAircraft(ModeSAircraft(tpacket.icao_address));
    aircraft_ptr =
        dictionary.GetAircraftPtr<ModeSAircraft>(Aircraft::ICAOToUID(0x5863BAu, Aircraft::kAircraftTypeModeS));
    ASSERT_TRUE(aircraft_ptr);
    aircraft_ptr->WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne, true);
    ASSERT_TRUE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne));
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(tpacket));
    EXPECT_TRUE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne));
    EXPECT_TRUE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagIdent));
    EXPECT_FALSE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagAlert));
}

TEST(AircraftDictionary, NICAssignment) {
    // Verify airborne NIC lookup uses NIC_A (bit 0) and NIC_B (bit 1) with mask 0b011.
    // NIC_A comes from TC=31 status messages; we pre-set it via WriteNICBit.
    // NIC_B comes from the airborne position message (ME[7]).
    AircraftDictionary dictionary;
    ModeSAircraft* aircraft_ptr;

    // TC=9, NIC_A=0, NIC_B=0: RC < 7.5m → kROCLessThan7p5Meters (NIC=11).
    {
        const uint32_t icao = 0x896113u;
        aircraft_ptr = dictionary.GetAircraftPtr<ModeSAircraft>(
            Aircraft::ICAOToUID(icao, Aircraft::kAircraftTypeModeS));
        ASSERT_TRUE(aircraft_ptr);
        aircraft_ptr->WriteNICBit(ADSBTypes::kNICBitA, 0);
        DecodedModeSPacket packet((char*)"8d89611348db01c6ea41c4c7b8bf");
        ASSERT_TRUE(packet.is_valid);
        ASSERT_TRUE(dictionary.IngestDecodedModeSPacket(packet));
        EXPECT_EQ(aircraft_ptr->navigation_integrity_category, ADSBTypes::kROCLessThan7p5Meters);
    }

    // TC=11, NIC_A=0, NIC_B=0: RC < 0.1 NM → kROCLessThan0p1NauticalMiles (NIC=8).
    {
        const uint32_t icao = 0x40621Du;
        aircraft_ptr = dictionary.GetAircraftPtr<ModeSAircraft>(
            Aircraft::ICAOToUID(icao, Aircraft::kAircraftTypeModeS));
        ASSERT_TRUE(aircraft_ptr);
        aircraft_ptr->WriteNICBit(ADSBTypes::kNICBitA, 0);
        DecodedModeSPacket packet((char*)"8D40621D58C382D690C8AC2863A7");
        ASSERT_TRUE(packet.is_valid);
        ASSERT_TRUE(dictionary.IngestDecodedModeSPacket(packet));
        EXPECT_EQ(aircraft_ptr->navigation_integrity_category, ADSBTypes::kROCLessThan0p1NauticalMiles);
    }

    // TC=11, NIC_A=0, NIC_B=1: RC < 75m → kROCLessThan75Meters (NIC=9).
    // Bit 7 of ME[0] toggled: 0x58 → 0x59. CRC invalid; force is_valid.
    {
        const uint32_t icao = 0x40621Du;
        aircraft_ptr = dictionary.GetAircraftPtr<ModeSAircraft>(
            Aircraft::ICAOToUID(icao, Aircraft::kAircraftTypeModeS));
        ASSERT_TRUE(aircraft_ptr);
        DecodedModeSPacket packet((char*)"8D40621D59C382D690C8AC2863A7");
        packet.is_valid = true;
        ASSERT_TRUE(dictionary.IngestDecodedModeSPacket(packet));
        EXPECT_EQ(aircraft_ptr->navigation_integrity_category, ADSBTypes::kROCLessThan75Meters);
    }

    // TC=16, NIC_A=0, NIC_B=0: RC < 4 NM → kROCLessThan4NauticalMiles (NIC=3).
    // ME[0] changed to 0x80 (TypeCode=16, NIC_B=0). Force is_valid.
    {
        const uint32_t icao = 0x40621Du;
        aircraft_ptr = dictionary.GetAircraftPtr<ModeSAircraft>(
            Aircraft::ICAOToUID(icao, Aircraft::kAircraftTypeModeS));
        ASSERT_TRUE(aircraft_ptr);
        DecodedModeSPacket packet((char*)"8D40621D80C382D690C8AC2863A7");
        packet.is_valid = true;
        ASSERT_TRUE(dictionary.IngestDecodedModeSPacket(packet));
        EXPECT_EQ(aircraft_ptr->navigation_integrity_category, ADSBTypes::kROCLessThan4NauticalMiles);
    }

    // TC=16, NIC_A=1, NIC_B=0: RC < 8 NM → kROCLessThan8NauticalMiles (NIC=2).
    {
        const uint32_t icao = 0x40621Du;
        aircraft_ptr = dictionary.GetAircraftPtr<ModeSAircraft>(
            Aircraft::ICAOToUID(icao, Aircraft::kAircraftTypeModeS));
        ASSERT_TRUE(aircraft_ptr);
        aircraft_ptr->WriteNICBit(ADSBTypes::kNICBitA, 1);
        DecodedModeSPacket packet((char*)"8D40621D80C382D690C8AC2863A7");
        packet.is_valid = true;
        ASSERT_TRUE(dictionary.IngestDecodedModeSPacket(packet));
        EXPECT_EQ(aircraft_ptr->navigation_integrity_category, ADSBTypes::kROCLessThan8NauticalMiles);
    }
}

TEST(AircraftDictionary, IngestAllCallReply) {
    AircraftDictionary dictionary = AircraftDictionary();
    DecodedModeSPacket tpacket = DecodedModeSPacket((char*)"5D7C0B6DB05076");
    ASSERT_TRUE(tpacket.is_valid);
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(tpacket));
    ModeSAircraft aircraft;
    EXPECT_TRUE(dictionary.GetAircraft(0x7C0B6Du, aircraft));
    EXPECT_EQ(aircraft.transponder_capability, 5);
}

TEST(AircraftDictionary, MetricsToJSON) {
    AircraftDictionary::Metrics metrics = {.raw_squitter_frames = 10,
                                           .valid_squitter_frames = 7,
                                           .raw_extended_squitter_frames = 30,
                                           .valid_extended_squitter_frames = 16,
                                           .demods_1090 = 50,
                                           .raw_squitter_frames_by_source = {2, 3, 5, 0},
                                           .valid_squitter_frames_by_source = {1, 2, 4, 0},
                                           .raw_extended_squitter_frames_by_source = {10, 11, 9, 0},
                                           .valid_extended_squitter_frames_by_source = {3, 5, 8, 0},
                                           .demods_1090_by_source = {19, 10, 21, 0}};
    char buf[AircraftDictionary::Metrics::kMetricsJSONMaxLen] = {'\0'};
    const char* expected_result =
        "{ \"raw_squitter_frames\": 10, \
\"valid_squitter_frames\": 7, \
\"raw_extended_squitter_frames\": 30, \
\"valid_extended_squitter_frames\": 16, \
\"demods_1090\": 50, \
\"raw_squitter_frames_by_source\": [2, 3, 5, 0], \
\"valid_squitter_frames_by_source\": [1, 2, 4, 0], \
\"raw_extended_squitter_frames_by_source\": [10, 11, 9, 0], \
\"valid_extended_squitter_frames_by_source\": [3, 5, 8, 0], \
\"demods_1090_by_source\": [19, 10, 21, 0], \
\"raw_uat_adsb_frames\": 0, \"valid_uat_adsb_frames\": 0, \
\"raw_uat_uplink_frames\": 0, \"valid_uat_uplink_frames\": 0, \
\"raw_remote_id_frames\": 0, \"valid_remote_id_frames\": 0, \
\"num_mode_s_aircraft\": 0, \"num_uat_aircraft\": 0, \"num_remote_id_aircraft\": 0\
}";
    EXPECT_EQ(metrics.ToJSON(buf, AircraftDictionary::Metrics::kMetricsJSONMaxLen), strlen(expected_result));
    EXPECT_STREQ(buf, expected_result);
}

TEST(ModeSAircraft, AircraftStats) {
    ModeSAircraft aircraft;
    aircraft.IncrementNumFramesReceived();
    EXPECT_EQ(aircraft.metrics.valid_extended_squitter_frames + aircraft.metrics.valid_squitter_frames, 0);
    aircraft.UpdateMetrics();
    EXPECT_EQ(aircraft.metrics.valid_extended_squitter_frames + aircraft.metrics.valid_squitter_frames, 1);
    EXPECT_EQ(aircraft.metrics.valid_extended_squitter_frames, 0);
    EXPECT_EQ(aircraft.metrics.valid_squitter_frames, 1);
    aircraft.IncrementNumFramesReceived(false);
    aircraft.IncrementNumFramesReceived(true);
    aircraft.UpdateMetrics();
    EXPECT_EQ(aircraft.metrics.valid_extended_squitter_frames + aircraft.metrics.valid_squitter_frames, 2);
    EXPECT_EQ(aircraft.metrics.valid_extended_squitter_frames, 1);
    EXPECT_EQ(aircraft.metrics.valid_squitter_frames, 1);
}

TEST(AircraftDictionary, FilterCPRLocations) {
    AircraftDictionary dictionary;

    /**  Test Case 0 **/
    DecodedModeSPacket packet = DecodedModeSPacket((char*)"8D48922358C387D91DF354DCCCB8");  // even
    uint32_t icao = packet.icao_address;
    inc_time_since_boot_ms(1000);
    packet.raw.mlat_48mhz_64bit_counts = get_time_since_boot_ms() * 48'000;
    bool decode_result = dictionary.IngestDecodedModeSPacket(packet);
    EXPECT_TRUE(decode_result);

    // Aircraft should now exist in the dictionary.
    ModeSAircraft* aircraft =
        dictionary.GetAircraftPtr<ModeSAircraft>(Aircraft::ICAOToUID(icao, Aircraft::kAircraftTypeModeS));
    ASSERT_TRUE(aircraft);

    // Send another valid position packet and ensure a valid location decode.
    packet = DecodedModeSPacket((char*)"8D48922358C38066020D8401D571");  // odd
    inc_time_since_boot_ms(1000);
    packet.raw.mlat_48mhz_64bit_counts = get_time_since_boot_ms() * 48'000;
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(packet));
    EXPECT_TRUE(aircraft->CanDecodePosition());
    EXPECT_FALSE(aircraft->DecodeAirbornePosition());  // Double decode fails without a new packet fails.
    EXPECT_NEAR(aircraft->latitude_deg, 48.5977f, 0.0001f);
    EXPECT_NEAR(aircraft->longitude_deg, 18.70521f, 0.001f);

    // Make it look like the aircraft already has a valid location so that the CPR filter is active.
    aircraft->WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagPositionValid, true);

    // Ingest a packet pair that causes an invalid decode.
    packet = DecodedModeSPacket((char*)"8D48922358C3806C3E0C8BC657BB");  // even
    inc_time_since_boot_ms(1000);
    packet.raw.mlat_48mhz_64bit_counts = get_time_since_boot_ms() * 48'000;
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(packet));
    // Aircraft has all the ingredients to decode its location, but the decoded location is not valid.
    EXPECT_TRUE(aircraft->CanDecodePosition());
    EXPECT_FALSE(aircraft->DecodeAirbornePosition());

    // Previous position is persisted.
    EXPECT_NEAR(aircraft->latitude_deg, 48.5977f, 0.0001f);
    EXPECT_NEAR(aircraft->longitude_deg, 18.70521f, 0.001f);

    /**  Test Case 1 **/
    // Valid position pair.
    packet = DecodedModeSPacket((char*)"8D48C22D60AB00DEABC5DB78FCD6");  // odd
    icao = packet.icao_address;
    inc_time_since_boot_ms(1000);
    packet.raw.mlat_48mhz_64bit_counts = get_time_since_boot_ms() * 48'000;
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(packet));
    aircraft = dictionary.GetAircraftPtr<ModeSAircraft>(Aircraft::ICAOToUID(icao, Aircraft::kAircraftTypeModeS));
    packet = DecodedModeSPacket((char*)"8D48C22D60AB0452BFAD19A695E0");  // even
    inc_time_since_boot_ms(1000);
    packet.raw.mlat_48mhz_64bit_counts = get_time_since_boot_ms() * 48'000;
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(packet));
    EXPECT_TRUE(aircraft->CanDecodePosition());
    EXPECT_FALSE(aircraft->DecodeAirbornePosition());  // Double decode fails without new packet.
    EXPECT_NEAR(aircraft->latitude_deg, 49.30659f, 0.0001f);
    EXPECT_NEAR(aircraft->longitude_deg, 17.4134f, 0.0001f);

    // Invalid position pair.
    packet = DecodedModeSPacket((char*)"8D48C22D60AB00E705C60B37E092");  // even
    inc_time_since_boot_ms(1000);
    packet.raw.mlat_48mhz_64bit_counts = get_time_since_boot_ms() * 48'000;
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(packet));
    packet = DecodedModeSPacket((char*)"8D48C22D60B104710F94F963E8B6");  // odd
    inc_time_since_boot_ms(1000);
    packet.raw.mlat_48mhz_64bit_counts = get_time_since_boot_ms() * 48'000;
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(packet));

    // Confirm new location by re-receiving the even packet.
    inc_time_since_boot_ms(1000);
    packet = DecodedModeSPacket((char*)"8D48C22D60AB00E705C60B37E092");  // even
    packet.raw.mlat_48mhz_64bit_counts = get_time_since_boot_ms() * 48'000;
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(packet));
}

TEST(AircraftDictionary, GetLowestAircraftPositionModeS) {
    AircraftDictionary dictionary = AircraftDictionary();
    float latitude_deg, longitude_deg, heading_deg;
    int32_t gnss_altitude_ft, baro_altitude_ft, speed_kts;

    // No aircraft in dictionary, should return false.
    EXPECT_FALSE(dictionary.GetLowestAircraftPosition(latitude_deg, longitude_deg, gnss_altitude_ft, baro_altitude_ft,
                                                      heading_deg, speed_kts));
    EXPECT_EQ(dictionary.lowest_aircraft_entry, nullptr);

    // Add an aircraft without GNSS altitude - should not be selected as lowest.
    ModeSAircraft aircraft1(0x123456);
    aircraft1.latitude_deg = 37.0f;
    aircraft1.longitude_deg = -122.0f;
    aircraft1.baro_altitude_ft = 10000;
    aircraft1.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagPositionValid, true);
    aircraft1.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagBaroAltitudeValid, true);
    aircraft1.last_message_timestamp_ms = get_time_since_boot_ms();
    dictionary.InsertAircraft(aircraft1);

    // Update the dictionary to process the aircraft.
    dictionary.Update(get_time_since_boot_ms());

    // Still no valid GNSS altitude, so should return false.
    EXPECT_FALSE(dictionary.GetLowestAircraftPosition(latitude_deg, longitude_deg, gnss_altitude_ft, baro_altitude_ft,
                                                      heading_deg, speed_kts));

    // Add an aircraft with GNSS altitude at 5000ft.
    ModeSAircraft aircraft2(0x234567);
    aircraft2.latitude_deg = 38.0f;
    aircraft2.longitude_deg = -121.0f;
    aircraft2.gnss_altitude_ft = 5000;
    aircraft2.baro_altitude_ft = 4900;
    aircraft2.direction_deg = 90.0f;
    aircraft2.speed_kts = 250;
    aircraft2.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagPositionValid, true);
    aircraft2.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagGNSSAltitudeValid, true);
    aircraft2.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagBaroAltitudeValid, true);
    aircraft2.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagDirectionValid, true);
    aircraft2.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagHorizontalSpeedValid, true);
    aircraft2.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne, true);
    aircraft2.last_message_timestamp_ms = get_time_since_boot_ms();
    dictionary.InsertAircraft(aircraft2);

    // Update the dictionary.
    dictionary.Update(get_time_since_boot_ms());

    // Should now return the aircraft with GNSS altitude.
    EXPECT_TRUE(dictionary.GetLowestAircraftPosition(latitude_deg, longitude_deg, gnss_altitude_ft, baro_altitude_ft,
                                                     heading_deg, speed_kts));
    EXPECT_NEAR(latitude_deg, 38.0f, kFloatCloseEnough);
    EXPECT_NEAR(longitude_deg, -121.0f, kFloatCloseEnough);
    EXPECT_EQ(gnss_altitude_ft, 5000);
    EXPECT_EQ(baro_altitude_ft, 4900);
    EXPECT_NEAR(heading_deg, 90.0f, kFloatCloseEnough);
    EXPECT_EQ(speed_kts, 250);

    // Add another aircraft with lower GNSS altitude at 2000ft.
    ModeSAircraft aircraft3(0x345678);
    aircraft3.latitude_deg = 39.0f;
    aircraft3.longitude_deg = -120.0f;
    aircraft3.gnss_altitude_ft = 2000;
    aircraft3.baro_altitude_ft = 1900;
    aircraft3.direction_deg = 180.0f;
    aircraft3.speed_kts = 150;
    aircraft3.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagPositionValid, true);
    aircraft3.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagGNSSAltitudeValid, true);
    aircraft3.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagBaroAltitudeValid, true);
    aircraft3.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagDirectionValid, true);
    aircraft3.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagHorizontalSpeedValid, true);
    aircraft3.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne, true);
    aircraft3.last_message_timestamp_ms = get_time_since_boot_ms();
    dictionary.InsertAircraft(aircraft3);

    // Update the dictionary.
    dictionary.Update(get_time_since_boot_ms());

    // Should now return the lower aircraft.
    EXPECT_TRUE(dictionary.GetLowestAircraftPosition(latitude_deg, longitude_deg, gnss_altitude_ft, baro_altitude_ft,
                                                     heading_deg, speed_kts));
    EXPECT_NEAR(latitude_deg, 39.0f, kFloatCloseEnough);
    EXPECT_NEAR(longitude_deg, -120.0f, kFloatCloseEnough);
    EXPECT_EQ(gnss_altitude_ft, 2000);
    EXPECT_EQ(baro_altitude_ft, 1900);
    EXPECT_NEAR(heading_deg, 180.0f, kFloatCloseEnough);
    EXPECT_EQ(speed_kts, 150);

    // Add a fourth aircraft with even lower GNSS altitude at 500ft.
    ModeSAircraft aircraft4(0x456789);
    aircraft4.latitude_deg = 40.0f;
    aircraft4.longitude_deg = -119.0f;
    aircraft4.gnss_altitude_ft = 500;
    aircraft4.direction_deg = 270.0f;
    aircraft4.speed_kts = 100;
    aircraft4.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagPositionValid, true);
    aircraft4.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagGNSSAltitudeValid, true);
    // No baro altitude valid.
    aircraft4.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagDirectionValid, true);
    aircraft4.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagHorizontalSpeedValid, true);
    aircraft4.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne, true);
    aircraft4.last_message_timestamp_ms = get_time_since_boot_ms();
    dictionary.InsertAircraft(aircraft4);

    // Update the dictionary.
    dictionary.Update(get_time_since_boot_ms());

    // Should now return the lowest aircraft (500ft).
    EXPECT_TRUE(dictionary.GetLowestAircraftPosition(latitude_deg, longitude_deg, gnss_altitude_ft, baro_altitude_ft,
                                                     heading_deg, speed_kts));
    EXPECT_NEAR(latitude_deg, 40.0f, kFloatCloseEnough);
    EXPECT_NEAR(longitude_deg, -119.0f, kFloatCloseEnough);
    EXPECT_EQ(gnss_altitude_ft, 500);
    EXPECT_EQ(baro_altitude_ft, INT32_MIN);  // No baro altitude available.
    EXPECT_NEAR(heading_deg, 270.0f, kFloatCloseEnough);
    EXPECT_EQ(speed_kts, 100);
}

TEST(AircraftDictionary, LowestAircraftPtrClearedOnPrune) {
    AircraftDictionary::AircraftDictionaryConfig config;
    config.aircraft_prune_interval_ms = 1000;  // Short prune interval for testing.
    AircraftDictionary dictionary(config);

    float latitude_deg, longitude_deg, heading_deg;
    int32_t gnss_altitude_ft, baro_altitude_ft, speed_kts;

    set_time_since_boot_ms(10000);

    // Add an aircraft with valid GNSS altitude.
    ModeSAircraft aircraft(0x123456);
    aircraft.latitude_deg = 37.0f;
    aircraft.longitude_deg = -122.0f;
    aircraft.gnss_altitude_ft = 1000;
    aircraft.direction_deg = 45.0f;
    aircraft.speed_kts = 200;
    aircraft.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagPositionValid, true);
    aircraft.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagGNSSAltitudeValid, true);
    aircraft.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagDirectionValid, true);
    aircraft.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagHorizontalSpeedValid, true);
    aircraft.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne, true);
    aircraft.last_message_timestamp_ms = get_time_since_boot_ms();
    dictionary.InsertAircraft(aircraft);

    // Update the dictionary.
    dictionary.Update(get_time_since_boot_ms());

    // Should return the aircraft.
    EXPECT_TRUE(dictionary.GetLowestAircraftPosition(latitude_deg, longitude_deg, gnss_altitude_ft, baro_altitude_ft,
                                                     heading_deg, speed_kts));
    EXPECT_NE(dictionary.lowest_aircraft_entry, nullptr);
    EXPECT_NEAR(heading_deg, 45.0f, kFloatCloseEnough);
    EXPECT_NEAR(speed_kts, 200.0f, kFloatCloseEnough);

    // Advance time past prune interval.
    inc_time_since_boot_ms(2000);

    // Update the dictionary - aircraft should be pruned.
    dictionary.Update(get_time_since_boot_ms());

    // Lowest aircraft pointer should be cleared.
    EXPECT_EQ(dictionary.lowest_aircraft_entry, nullptr);
    EXPECT_EQ(dictionary.GetNumAircraft(), 0);

    // GetLowestAircraftPosition should return false.
    EXPECT_FALSE(dictionary.GetLowestAircraftPosition(latitude_deg, longitude_deg, gnss_altitude_ft, baro_altitude_ft,
                                                      heading_deg, speed_kts));
}

TEST(AircraftDictionary, IngestSurfacePositionMessagesOnGround) {
    // Ingest a pair of surface position messages and verify that the position is decoded correctly.
    // First packet: 8C4841753AAB238733C8CD4020B1
    // Second packet: 8C4841753A8A35323FAEBDAC702D
    // Reference Location:
    //  Lat: 51.990 deg
    //  Lon: 4.375 deg
    // Decoded Location:
    //  Lat: 52.320607 deg
    //  Lon: 4.734735 deg

    AircraftDictionary::AircraftDictionaryConfig config;
    AircraftDictionary dictionary(config);
    dictionary.SetReferencePosition(51.990f, 4.375f);  // Set reference latitude for surface position CPR decoding.
    DecodedModeSPacket packet_0 = DecodedModeSPacket((char*)"8C4841753AAB238733C8CD4020B1");
    DecodedModeSPacket packet_1 = DecodedModeSPacket((char*)"8C4841753A8A35323FAEBDAC702D");

    // Set packet timestamp to nonzero values close together to pass packet filter.
    packet_0.raw.mlat_48mhz_64bit_counts = 50123;
    packet_1.raw.mlat_48mhz_64bit_counts =
        100000;  // Needs to be at least 1ms * 48,000 counts / ms later to count as most recent packet.

    uint32_t uid = Aircraft::ICAOToUID(0x484175, Aircraft::AircraftType::kAircraftTypeModeS);

    // Ingest the first surface position packet.
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(packet_0));

    // After ingesting the first packet, the aircraft should exist in the dictionary, but should be marked as on ground
    // with an invalid position.
    ModeSAircraft* aircraft_ptr = dictionary.GetAircraftPtr<ModeSAircraft>(uid, false);
    ASSERT_TRUE(aircraft_ptr);
    EXPECT_FALSE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagPositionValid));
    EXPECT_FALSE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne));

    // Ingest the second surface position packet.
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(packet_1));
    EXPECT_TRUE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagPositionValid));
    EXPECT_FALSE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne));

    // Verify that the decoded position is correct.
    EXPECT_NEAR(aircraft_ptr->latitude_deg, 52.320607f, 0.001f);
    EXPECT_NEAR(aircraft_ptr->longitude_deg, 4.734735f, 0.001f);

    // Now change the reference position and observe that the decoded surface position tracks with it.
    dictionary.Init();
    dictionary.SetReferencePosition(51.990f, 4.375f + 90.0f);
    packet_0.raw.mlat_48mhz_64bit_counts += 100e3;
    packet_1.raw.mlat_48mhz_64bit_counts += 100e3;
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(packet_0));
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(packet_1));
    aircraft_ptr = dictionary.GetAircraftPtr<ModeSAircraft>(uid, false);
    EXPECT_NEAR(aircraft_ptr->latitude_deg, 52.320607f, 0.001f);
    EXPECT_NEAR(aircraft_ptr->longitude_deg, WrapCPRDecodeLongitude(4.734735f + 90.0f), 0.001f);

    dictionary.Init();
    dictionary.SetReferencePosition(51.990f, 4.375f + 180.0f);
    packet_0.raw.mlat_48mhz_64bit_counts += 100e3;
    packet_1.raw.mlat_48mhz_64bit_counts += 100e3;
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(packet_0));
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(packet_1));
    aircraft_ptr = dictionary.GetAircraftPtr<ModeSAircraft>(uid, false);
    EXPECT_NEAR(aircraft_ptr->latitude_deg, 52.320607f, 0.001f);
    EXPECT_NEAR(aircraft_ptr->longitude_deg, WrapCPRDecodeLongitude(4.734735f + 180.0f), 0.001f);

    dictionary.Init();
    dictionary.SetReferencePosition(51.990f, 4.375f - 90.0f);
    packet_0.raw.mlat_48mhz_64bit_counts += 100e3;
    packet_1.raw.mlat_48mhz_64bit_counts += 100e3;
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(packet_0));
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(packet_1));
    aircraft_ptr = dictionary.GetAircraftPtr<ModeSAircraft>(uid, false);
    EXPECT_NEAR(aircraft_ptr->latitude_deg, 52.320607f, 0.001f);
    EXPECT_NEAR(aircraft_ptr->longitude_deg, WrapCPRDecodeLongitude(4.734735f - 90.0f), 0.001f);

    dictionary.Init();
    dictionary.SetReferencePosition(51.990f, 4.375f - 180.0f);
    packet_0.raw.mlat_48mhz_64bit_counts += 100e3;
    packet_1.raw.mlat_48mhz_64bit_counts += 100e3;
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(packet_0));
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(packet_1));
    aircraft_ptr = dictionary.GetAircraftPtr<ModeSAircraft>(uid, false);
    EXPECT_NEAR(aircraft_ptr->latitude_deg, 52.320607f, 0.001f);
    EXPECT_NEAR(aircraft_ptr->longitude_deg, WrapCPRDecodeLongitude(4.734735f - 180.0f), 0.001f);
}

TEST(AircraftDictionary, SurfacePositionSkippedWithoutReference) {
    // Without a valid reference position, surface CPR would resolve its 90x90 degree ambiguity against the
    // default (0, 0) reference and silently produce a wrong position. Verify that the position decode is
    // skipped until a reference is set, while the rest of the message (air/ground state) still applies.
    AircraftDictionary::AircraftDictionaryConfig config;
    AircraftDictionary dictionary(config);
    // NOTE: No SetReferencePosition call — the dictionary starts with no valid reference.
    DecodedModeSPacket packet_0 = DecodedModeSPacket((char*)"8C4841753AAB238733C8CD4020B1");
    DecodedModeSPacket packet_1 = DecodedModeSPacket((char*)"8C4841753A8A35323FAEBDAC702D");
    packet_0.raw.mlat_48mhz_64bit_counts = 50123;
    packet_1.raw.mlat_48mhz_64bit_counts = 100000;

    uint32_t uid = Aircraft::ICAOToUID(0x484175, Aircraft::AircraftType::kAircraftTypeModeS);

    // Both packets ingest successfully (not an error), but no position is decoded.
    EXPECT_FALSE(dictionary.ReferencePositionValid());
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(packet_0));
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(packet_1));
    ModeSAircraft* aircraft_ptr = dictionary.GetAircraftPtr<ModeSAircraft>(uid, false);
    ASSERT_TRUE(aircraft_ptr);
    EXPECT_FALSE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagPositionValid));
    EXPECT_FALSE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne));  // Air/ground still applied.

    // Once a reference appears, re-ingesting decodes the position (CPR frames were cached).
    dictionary.SetReferencePosition(51.990f, 4.375f);
    EXPECT_TRUE(dictionary.ReferencePositionValid());
    packet_0.raw.mlat_48mhz_64bit_counts += 100e3;
    packet_1.raw.mlat_48mhz_64bit_counts += 100e3;
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(packet_0));
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(packet_1));
    EXPECT_TRUE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagPositionValid));
    EXPECT_NEAR(aircraft_ptr->latitude_deg, 52.320607f, 0.001f);
    EXPECT_NEAR(aircraft_ptr->longitude_deg, 4.734735f, 0.001f);

    // Clearing the reference pauses surface position decoding again (existing position stays latched).
    dictionary.ClearReferencePosition();
    EXPECT_FALSE(dictionary.ReferencePositionValid());
    packet_0.raw.mlat_48mhz_64bit_counts += 100e3;
    packet_1.raw.mlat_48mhz_64bit_counts += 100e3;
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(packet_0));
    EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(packet_1));
}

TEST(AircraftDictionary, IngestSurfacePositionMovementAndTrack) {
    AircraftDictionary::AircraftDictionaryConfig config;
    AircraftDictionary dictionary(config);

    DecodedModeSPacket packet_0 = DecodedModeSPacket((char*)"8C4841753A9A153237AEF0F275BE");
    uint32_t uid = Aircraft::ICAOToUID(0x484175, Aircraft::AircraftType::kAircraftTypeModeS);

    EXPECT_TRUE(dictionary.IngestModeSADSBPacket(packet_0));
    ModeSAircraft* aircraft_ptr = dictionary.GetAircraftPtr<ModeSAircraft>(uid, false);

    EXPECT_FALSE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagPositionValid));
    EXPECT_FALSE(aircraft_ptr->HasBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne));

    EXPECT_NEAR(aircraft_ptr->direction_deg, 92.8f, 0.1f);
    EXPECT_EQ(aircraft_ptr->speed_kts, 17);
    EXPECT_EQ(aircraft_ptr->baro_vertical_rate_fpm, 0);
    EXPECT_EQ(aircraft_ptr->gnss_vertical_rate_fpm, 0);
    EXPECT_EQ(aircraft_ptr->speed_source, ADSBTypes::SpeedSource::kSpeedSourceGroundSpeed);
}

TEST(AircraftDictionary, RemoveOldestAircraftEmpty) {
    AircraftDictionary dictionary;
    EXPECT_FALSE(dictionary.RemoveOldestAircraft());
}

TEST(AircraftDictionary, RemoveOldestAircraftRemovesOldest) {
    AircraftDictionary dictionary;

    ModeSAircraft aircraft_a(0x111111);
    aircraft_a.last_message_timestamp_ms = 1000;  // oldest
    dictionary.InsertAircraft(aircraft_a);

    ModeSAircraft aircraft_b(0x222222);
    aircraft_b.last_message_timestamp_ms = 2000;
    dictionary.InsertAircraft(aircraft_b);

    ModeSAircraft aircraft_c(0x333333);
    aircraft_c.last_message_timestamp_ms = 3000;  // newest
    dictionary.InsertAircraft(aircraft_c);

    ASSERT_EQ(dictionary.GetNumAircraft(), 3);
    EXPECT_TRUE(dictionary.RemoveOldestAircraft());
    EXPECT_EQ(dictionary.GetNumAircraft(), 2);
    EXPECT_FALSE(dictionary.ContainsAircraft(0x111111));
    EXPECT_TRUE(dictionary.ContainsAircraft(0x222222));
    EXPECT_TRUE(dictionary.ContainsAircraft(0x333333));
}

TEST(AircraftDictionary, RemoveOldestAircraftClearsLowestEntry) {
    AircraftDictionary dictionary;

    // Aircraft with the lowest GNSS altitude and the oldest timestamp.
    ModeSAircraft aircraft_old(0x111111);
    aircraft_old.gnss_altitude_ft = 500;
    aircraft_old.last_message_timestamp_ms = 100;
    aircraft_old.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagGNSSAltitudeValid, true);
    aircraft_old.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagPositionValid, true);
    aircraft_old.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne, true);
    dictionary.InsertAircraft(aircraft_old);

    ModeSAircraft aircraft_new(0x222222);
    aircraft_new.gnss_altitude_ft = 5000;
    aircraft_new.last_message_timestamp_ms = 5000;
    aircraft_new.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagGNSSAltitudeValid, true);
    aircraft_new.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagPositionValid, true);
    aircraft_new.WriteBitFlag(ModeSAircraft::BitFlag::kBitFlagIsAirborne, true);
    dictionary.InsertAircraft(aircraft_new);

    // Update so lowest_aircraft_entry is set (it will point to aircraft_old, lowest GNSS altitude).
    dictionary.Update(5000);
    EXPECT_NE(dictionary.lowest_aircraft_entry, nullptr);

    // Removing the oldest aircraft should also clear lowest_aircraft_entry.
    EXPECT_TRUE(dictionary.RemoveOldestAircraft());
    EXPECT_EQ(dictionary.lowest_aircraft_entry, nullptr);
    EXPECT_FALSE(dictionary.ContainsAircraft(0x111111));
    EXPECT_TRUE(dictionary.ContainsAircraft(0x222222));
}

TEST(AircraftDictionary, InsertAircraftEvictsOldestWhenFull) {
    AircraftDictionary dictionary;

    // Fill to capacity: aircraft i has ICAO (i+1) and timestamp (i+1), so ICAO 1 is the oldest.
    for (uint16_t i = 0; i < AircraftDictionary::kMaxNumAircraft; i++) {
        ModeSAircraft aircraft(i + 1);
        aircraft.last_message_timestamp_ms = i + 1;
        ASSERT_TRUE(dictionary.InsertAircraft(aircraft));
    }
    ASSERT_EQ(dictionary.GetNumAircraft(), AircraftDictionary::kMaxNumAircraft);

    // Inserting a new aircraft when full should evict the oldest (ICAO 1, ts=1) and succeed.
    ModeSAircraft new_aircraft(0xBEEB);
    new_aircraft.last_message_timestamp_ms = AircraftDictionary::kMaxNumAircraft + 1;
    EXPECT_TRUE(dictionary.InsertAircraft(new_aircraft));

    EXPECT_EQ(dictionary.GetNumAircraft(), AircraftDictionary::kMaxNumAircraft);
    EXPECT_TRUE(dictionary.ContainsAircraft(0xBEEB));
    EXPECT_FALSE(dictionary.ContainsAircraft(1));  // ICAO 1 was the oldest, now evicted
    EXPECT_TRUE(dictionary.ContainsAircraft(2));   // ICAO 2 was the second oldest, still present
}

// TC=31 Operation Status Message tests — one per ADS-B version (0, 1, 2).
//
// Packet layout (14 bytes = 28 hex chars):
//   Byte 0:     DF/CA = 0x8D (DF=17, CA=5)
//   Bytes 1-3:  ICAO = 0x123456
//   Byte 4:     ME[0-7]  = 0xF8 (TC=31=0b11111, ST=0=airborne)
//   Byte 5:     ME[8-15] = CC high  (ME[10]=TCAS Op, ME[11]=1090ES In, ME[14-15]=ARV/TS)
//   Byte 6:     ME[16-23]= CC low   (ME[18]=UAT In)
//   Byte 7:     ME[24-31]= OM high  (ME[26]=TCAS RA, ME[27]=IDENT, ME[29]=SingleAnt, ME[30-31]=SDA)
//   Byte 8:     ME[32-39]= OM low   (ME[32-34]=NACv in v2 airborne OM)
//   Byte 9:     ME[40-47]= ME[40-42]=version, ME[43]=NICa, ME[44-47]=NACp
//   Byte 10:    ME[48-55]= ME[48-49]=GVA/BAQ, ME[50-51]=SIL, ME[52]=NICbaro, ME[53]=HRD, ME[54]=SILs
//   Bytes 11-13: CRC (zeroed; is_valid forced to true)
//
// Bit indexing: GetNBitWordFromMessage(n,k) reads n bits at ME offset k, MSB-first.

TEST(AircraftDictionary, OperationStatusMessageVersion0) {
    // Version 0 (DO-260): ME[40-55] are all reserved; CC/OM have different semantics.
    // Verify that NICa, NACp, SIL, and GVA are NOT populated (remain at defaults).
    // Packet: all zeros except TC=31, ST=0 (airborne), version field = 000 (implied by 0x00 in byte 9).
    AircraftDictionary dictionary;
    DecodedModeSPacket tpacket = DecodedModeSPacket((char*)"8D123456F8000000000000000000");
    tpacket.is_valid = true;
    ASSERT_TRUE(dictionary.IngestDecodedModeSPacket(tpacket));
    ASSERT_EQ(dictionary.GetNumAircraft(), 1);
    auto& aircraft = std::get<ModeSAircraft>(dictionary.dict.begin()->second);

    EXPECT_EQ(aircraft.adsb_version, 0);
    // NICa should not be set — ME[43] is reserved in v0.
    EXPECT_FALSE(aircraft.NICBitIsValid(ADSBTypes::kNICBitA));
    // NACp should remain at default (unknown) — ME[44-47] reserved in v0.
    EXPECT_EQ(aircraft.navigation_accuracy_category_position,
              ADSBTypes::kEPUUnknownOrGreaterThanOrEqualTo10NauticalMiles);
    // SIL should remain at default (unknown) — ME[50-51] reserved in v0.
    EXPECT_EQ(aircraft.surveillance_integrity_level,
              ADSBTypes::kPOERCUnknownOrGreaterThan1em3PerFlightHour);
    // GVA should remain at default — ME[48-49] reserved in v0.
    EXPECT_EQ(aircraft.geometric_vertical_accuracy, ADSBTypes::kGVAUnknownOrGreaterThan150Meters);
}

TEST(AircraftDictionary, OperationStatusMessageVersion1) {
    // Version 1 (DO-260A): individual CC/OM flags; ME[48-49] = BAQ (different encoding from GVA).
    // Packet byte layout (ICAO=0x123456, airborne ST=0):
    //   Byte 5  = 0x30: ME[10]=1 (TCAS Op), ME[11]=1 (1090ES In)
    //   Byte 6  = 0x20: ME[18]=1 (UAT In)
    //   Byte 7  = 0x36: ME[26]=1 (TCAS RA), ME[27]=1 (IDENT), ME[29]=1 (SingleAnt), ME[30]=1 (SDA=2)
    //   Byte 8  = 0x00: OM lower (NACv not present in v1 airborne)
    //   Byte 9  = 0x37: ME[40-42]=001 (v1), ME[43]=1 (NICa), ME[44-47]=0111 (NACp=7)
    //   Byte 10 = 0x6A: ME[48-49]=01 (BAQ; skip), ME[50-51]=10 (SIL=2), ME[52]=1 (NICbaro), ME[54]=1 (SILs, NOT applied in v1)
    AircraftDictionary dictionary;
    DecodedModeSPacket tpacket = DecodedModeSPacket((char*)"8D123456F830203600376A000000");
    tpacket.is_valid = true;
    ASSERT_TRUE(dictionary.IngestDecodedModeSPacket(tpacket));
    ASSERT_EQ(dictionary.GetNumAircraft(), 1);
    auto& aircraft = std::get<ModeSAircraft>(dictionary.dict.begin()->second);

    EXPECT_EQ(aircraft.adsb_version, 1);
    // NICa set from ME[43].
    EXPECT_TRUE(aircraft.NICBitIsValid(ADSBTypes::kNICBitA));
    // NACp=7 from ME[44-47].
    EXPECT_EQ(aircraft.navigation_accuracy_category_position, ADSBTypes::kEPULessThan0p1NauticalMiles);
    // SIL=2 from ME[50-51]; SIL supplement NOT applied (v1 has no supplement), so result = 0b010.
    EXPECT_EQ(aircraft.surveillance_integrity_level,
              ADSBTypes::kPOERCLessThanOrEqualTo1em5PerFlightHour);
    // GVA must remain at default — ME[48-49] is BAQ in v1, not parsed as GVA.
    EXPECT_EQ(aircraft.geometric_vertical_accuracy, ADSBTypes::kGVAUnknownOrGreaterThan150Meters);
    // NACv must remain at default — not present in v1 airborne OM.
    EXPECT_EQ(aircraft.navigation_accuracy_category_velocity,
              ADSBTypes::kHVEUnknownOrGreaterThanOrEqualTo10MetersPerSecond);
    // NIC Baro set from ME[52].
    EXPECT_EQ(aircraft.navigation_integrity_category_baro,
              ADSBTypes::kBAIGillHamInputCrossCheckedOrNonGillhamSource);
    // Flags from CC/OM.
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagHas1090ESIn));
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagTCASRA));
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagTCASOperational));
    EXPECT_TRUE(aircraft.HasBitFlag(ModeSAircraft::BitFlag::kBitFlagHasUATIn));
}

TEST(AircraftDictionary, OperationStatusMessageVersion2) {
    // Version 2 (DO-260B): GVA at ME[48-49]; SIL supplement at ME[54]; NACv in airborne OM ME[32-34].
    // Packet byte layout (ICAO=0x123456, airborne ST=0):
    //   Byte 5  = 0x30: ME[10]=1 (TCAS Op), ME[11]=1 (1090ES In)
    //   Byte 6  = 0x20: ME[18]=1 (UAT In)
    //   Byte 7  = 0x36: ME[26]=1 (TCAS RA), ME[27]=1 (IDENT), ME[29]=1 (SingleAnt), ME[30]=1 (SDA=2)
    //   Byte 8  = 0xC0: ME[32-34]=110 (NACv=6 = kHVELessThan10MetersPerSecond)
    //   Byte 9  = 0x57: ME[40-42]=010 (v2), ME[43]=1 (NICa), ME[44-47]=0111 (NACp=7)
    //   Byte 10 = 0x6A: ME[48-49]=01 (GVA=1 = <=150m), ME[50-51]=10 (SIL=2), ME[52]=1 (NICbaro), ME[54]=1 (SILs=1)
    //   Combined SIL: (SILs<<2)|SIL = (1<<2)|2 = 6 = kPOERCLessThanOrEqualTo1em5PerSample
    AircraftDictionary dictionary;
    DecodedModeSPacket tpacket = DecodedModeSPacket((char*)"8D123456F8302036C0576A000000");
    tpacket.is_valid = true;
    ASSERT_TRUE(dictionary.IngestDecodedModeSPacket(tpacket));
    ASSERT_EQ(dictionary.GetNumAircraft(), 1);
    auto& aircraft = std::get<ModeSAircraft>(dictionary.dict.begin()->second);

    EXPECT_EQ(aircraft.adsb_version, 2);
    // NICa set from ME[43].
    EXPECT_TRUE(aircraft.NICBitIsValid(ADSBTypes::kNICBitA));
    // NACp=7 from ME[44-47].
    EXPECT_EQ(aircraft.navigation_accuracy_category_position, ADSBTypes::kEPULessThan0p1NauticalMiles);
    // GVA=1 from ME[48-49] (v2 only).
    EXPECT_EQ(aircraft.geometric_vertical_accuracy, ADSBTypes::GVALessThanOrEqualTo150Meters);
    // SIL 3-bit composite: (SILs<<2)|SIL = (1<<2)|2 = 6.
    EXPECT_EQ(aircraft.surveillance_integrity_level,
              ADSBTypes::kPOERCLessThanOrEqualTo1em5PerSample);
    // NACv=6 from airborne OM ME[32-34] (v2 only).
    EXPECT_EQ(aircraft.navigation_accuracy_category_velocity,
              ADSBTypes::kHVELessThan10MetersPerSecond);
    // NIC Baro set from ME[52].
    EXPECT_EQ(aircraft.navigation_integrity_category_baro,
              ADSBTypes::kBAIGillHamInputCrossCheckedOrNonGillhamSource);
}