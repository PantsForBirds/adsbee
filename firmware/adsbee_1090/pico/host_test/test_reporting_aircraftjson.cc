#include "aircraft_dictionary.hh"
#include "aircraftjson_utils.hh"
#include "gtest/gtest.h"

#include <string>
#include <string_view>

// Returns the JSON value (as a string) for the given key in a single-line JSON object, or an empty
// string if the key is not present.  Handles string values ("key":"val") and numeric values
// ("key":123).
static std::string GetJSONValue(std::string_view json, const char* key) {
    std::string search = std::string("\"") + key + "\":";
    size_t pos = json.find(search);
    if (pos == std::string_view::npos) return "";
    pos += search.size();
    if (pos >= json.size()) return "";

    if (json[pos] == '"') {
        // String value: find closing quote.
        size_t start = pos + 1;
        size_t end = json.find('"', start);
        if (end == std::string_view::npos) return "";
        return std::string(json.substr(start, end - start));
    } else {
        // Numeric value: find next comma, } or \n.
        size_t start = pos;
        size_t end = json.find_first_of(",}\n", start);
        if (end == std::string_view::npos) end = json.size();
        return std::string(json.substr(start, end - start));
    }
}

static bool HasJSONKey(std::string_view json, const char* key) {
    std::string search = std::string("\"") + key + "\":";
    return json.find(search) != std::string_view::npos;
}

// ─── Mode S ────────────────────────────────────────────────────────────────

TEST(AircraftJSON, ModeSAircraftAllFields) {
    char buf[kAircraftJSONMessageStrMaxLen];

    ModeSAircraft ac;
    ac.icao_address = 0xABCDEF;
    strcpy(ac.callsign, "UAL123");
    ac.squawk = 1234;
    ac.emitter_category = ADSBTypes::kEmitterCategoryMedium2;  // 3 → "A3"
    ac.latitude_deg = 37.12345f;
    ac.longitude_deg = -122.54321f;
    ac.baro_altitude_ft = 35000;
    ac.gnss_altitude_ft = 35200;
    ac.speed_kts = 450;
    ac.direction_deg = 270.5f;
    ac.baro_vertical_rate_fpm = -64;
    ac.gnss_vertical_rate_fpm = -128;
    ac.navigation_integrity_category = static_cast<ADSBTypes::NICRadiusOfContainment>(8);
    ac.navigation_integrity_category_baro = static_cast<ADSBTypes::NICBarometricAltitudeIntegrity>(1);
    ac.navigation_accuracy_category_position = static_cast<ADSBTypes::NACEstimatedPositionUncertainty>(9);
    ac.navigation_accuracy_category_velocity = static_cast<ADSBTypes::NACHorizontalVelocityError>(2);
    ac.surveillance_integrity_level =
        static_cast<ADSBTypes::SILProbabilityOfExceedingNICRadiusOfContainmnent>(3);
    ac.geometric_vertical_accuracy = static_cast<ADSBTypes::GVA>(2);
    ac.system_design_assurance = static_cast<ADSBTypes::SystemDesignAssurance>(2);
    ac.adsb_version = 2;
    ac.last_message_signal_strength_dbm = -75;
    ac.metrics.valid_squitter_frames = 5;
    ac.metrics.valid_extended_squitter_frames = 10;

    // Set validity flags.
    ac.WriteBitFlag(ModeSAircraft::kBitFlagPositionValid, true);
    ac.WriteBitFlag(ModeSAircraft::kBitFlagBaroAltitudeValid, true);
    ac.WriteBitFlag(ModeSAircraft::kBitFlagGNSSAltitudeValid, true);
    ac.WriteBitFlag(ModeSAircraft::kBitFlagHorizontalSpeedValid, true);
    ac.WriteBitFlag(ModeSAircraft::kBitFlagDirectionValid, true);
    ac.WriteBitFlag(ModeSAircraft::kBitFlagBaroVerticalRateValid, true);
    ac.WriteBitFlag(ModeSAircraft::kBitFlagGNSSVerticalRateValid, true);

    int16_t len = WriteAircraftJSONModeSAircraftStr(buf, ac);
    ASSERT_GT(len, 0);

    std::string_view json(buf, len);
    printf("ModeSAircraftAllFields: %.*s", len, buf);

    EXPECT_EQ(GetJSONValue(json, "hex"), "abcdef");
    EXPECT_EQ(GetJSONValue(json, "type"), "adsb_icao");
    EXPECT_EQ(GetJSONValue(json, "flight"), "UAL123");
    EXPECT_EQ(GetJSONValue(json, "squawk"), "1234");
    EXPECT_EQ(GetJSONValue(json, "category"), "A3");
    EXPECT_EQ(GetJSONValue(json, "alt_baro"), "35000");
    EXPECT_EQ(GetJSONValue(json, "alt_geom"), "35200");
    EXPECT_EQ(GetJSONValue(json, "gs"), "450");
    EXPECT_EQ(GetJSONValue(json, "track"), "270.5");
    EXPECT_EQ(GetJSONValue(json, "baro_rate"), "-64");
    EXPECT_EQ(GetJSONValue(json, "geom_rate"), "-128");
    EXPECT_EQ(GetJSONValue(json, "nic"), "8");
    EXPECT_EQ(GetJSONValue(json, "nic_baro"), "1");
    EXPECT_EQ(GetJSONValue(json, "nac_p"), "9");
    EXPECT_EQ(GetJSONValue(json, "nac_v"), "2");
    EXPECT_EQ(GetJSONValue(json, "sil"), "3");
    EXPECT_EQ(GetJSONValue(json, "gva"), "2");
    EXPECT_EQ(GetJSONValue(json, "sda"), "2");
    EXPECT_EQ(GetJSONValue(json, "version"), "2");
    EXPECT_EQ(GetJSONValue(json, "rssi"), "-75");
    EXPECT_EQ(GetJSONValue(json, "messages"), "15");  // 5 + 10

    // alert and spi absent when flags not set.
    EXPECT_FALSE(HasJSONKey(json, "alert"));
    EXPECT_FALSE(HasJSONKey(json, "spi"));

    // Ends with newline.
    EXPECT_EQ(buf[len - 1], '\n');
}

TEST(AircraftJSON, ModeSAircraftPartialFields) {
    char buf[kAircraftJSONMessageStrMaxLen];

    // Default-constructed: no flags set, default callsign "?", squawk not received.
    ModeSAircraft ac;
    ac.icao_address = 0x123456;
    ac.last_message_signal_strength_dbm = -80;

    int16_t len = WriteAircraftJSONModeSAircraftStr(buf, ac);
    ASSERT_GT(len, 0);

    std::string_view json(buf, len);
    printf("ModeSAircraftPartialFields: %.*s", len, buf);

    EXPECT_EQ(GetJSONValue(json, "hex"), "123456");
    EXPECT_EQ(GetJSONValue(json, "type"), "adsb_icao");

    // Optional fields absent when flags not set.
    EXPECT_FALSE(HasJSONKey(json, "flight"));
    EXPECT_FALSE(HasJSONKey(json, "squawk"));
    EXPECT_FALSE(HasJSONKey(json, "lat"));
    EXPECT_FALSE(HasJSONKey(json, "lon"));
    EXPECT_FALSE(HasJSONKey(json, "alt_baro"));
    EXPECT_FALSE(HasJSONKey(json, "alt_geom"));
    EXPECT_FALSE(HasJSONKey(json, "gs"));
    EXPECT_FALSE(HasJSONKey(json, "track"));
    EXPECT_FALSE(HasJSONKey(json, "baro_rate"));
    EXPECT_FALSE(HasJSONKey(json, "geom_rate"));
    EXPECT_FALSE(HasJSONKey(json, "alert"));
    EXPECT_FALSE(HasJSONKey(json, "spi"));

    // Always-present fields are still there.
    EXPECT_TRUE(HasJSONKey(json, "nic"));
    EXPECT_TRUE(HasJSONKey(json, "rssi"));
    EXPECT_TRUE(HasJSONKey(json, "messages"));
}

TEST(AircraftJSON, ModeSAircraftTypeFlags) {
    char buf[kAircraftJSONMessageStrMaxLen];

    ModeSAircraft ac;
    ac.icao_address = 0x111111;

    // Non-transponder → adsr_icao
    ac.WriteBitFlag(ModeSAircraft::kBitFlagIsNonTransponder, true);
    WriteAircraftJSONModeSAircraftStr(buf, ac);
    EXPECT_EQ(GetJSONValue(std::string_view(buf), "type"), "adsr_icao");

    // Class B2 ground vehicle → adsb_icao_nt (takes precedence check: non-transponder wins)
    ac.WriteBitFlag(ModeSAircraft::kBitFlagIsNonTransponder, false);
    ac.WriteBitFlag(ModeSAircraft::kBitFlagIsClassB2GroundVehicle, true);
    WriteAircraftJSONModeSAircraftStr(buf, ac);
    EXPECT_EQ(GetJSONValue(std::string_view(buf), "type"), "adsb_icao_nt");
}

TEST(AircraftJSON, ModeSAircraftHeadingFields) {
    char buf[kAircraftJSONMessageStrMaxLen];
    ModeSAircraft ac;
    ac.icao_address = 0x222222;
    ac.direction_deg = 180.0f;
    ac.WriteBitFlag(ModeSAircraft::kBitFlagDirectionValid, true);

    // Track (default: not a heading)
    WriteAircraftJSONModeSAircraftStr(buf, ac);
    EXPECT_TRUE(HasJSONKey(std::string_view(buf), "track"));
    EXPECT_FALSE(HasJSONKey(std::string_view(buf), "mag_heading"));
    EXPECT_FALSE(HasJSONKey(std::string_view(buf), "true_heading"));

    // Magnetic heading
    ac.WriteBitFlag(ModeSAircraft::kBitFlagDirectionIsHeading, true);
    ac.WriteBitFlag(ModeSAircraft::kBitFlagHeadingUsesMagneticNorth, true);
    WriteAircraftJSONModeSAircraftStr(buf, ac);
    EXPECT_FALSE(HasJSONKey(std::string_view(buf), "track"));
    EXPECT_TRUE(HasJSONKey(std::string_view(buf), "mag_heading"));
    EXPECT_FALSE(HasJSONKey(std::string_view(buf), "true_heading"));

    // True heading
    ac.WriteBitFlag(ModeSAircraft::kBitFlagHeadingUsesMagneticNorth, false);
    WriteAircraftJSONModeSAircraftStr(buf, ac);
    EXPECT_FALSE(HasJSONKey(std::string_view(buf), "track"));
    EXPECT_FALSE(HasJSONKey(std::string_view(buf), "mag_heading"));
    EXPECT_TRUE(HasJSONKey(std::string_view(buf), "true_heading"));
}

TEST(AircraftJSON, ModeSAircraftAlertSpi) {
    char buf[kAircraftJSONMessageStrMaxLen];
    ModeSAircraft ac;
    ac.icao_address = 0x333333;
    ac.WriteBitFlag(ModeSAircraft::kBitFlagAlert, true);
    ac.WriteBitFlag(ModeSAircraft::kBitFlagIdent, true);

    WriteAircraftJSONModeSAircraftStr(buf, ac);
    std::string_view json(buf);
    EXPECT_EQ(GetJSONValue(json, "alert"), "1");
    EXPECT_EQ(GetJSONValue(json, "spi"), "1");
}

// ─── UAT ───────────────────────────────────────────────────────────────────

TEST(AircraftJSON, UATAircraftAllFields) {
    char buf[kAircraftJSONMessageStrMaxLen];

    UATAircraft ac;
    // Use kADSBTargetWithICAO24BitAddress (AQ=0) so no '~' prefix, no upper bits set.
    ac.icao_address = 0xABCDEF;
    strcpy(ac.callsign, "AAL456");
    ac.squawk = 7654;
    ac.emitter_category = ADSBTypes::kEmitterCategoryGliderSailplane;  // 9 → "B1"
    ac.latitude_deg = 40.0f;
    ac.longitude_deg = -75.0f;
    ac.baro_altitude_ft = 10000;
    ac.gnss_altitude_ft = 10200;
    ac.speed_kts = 200;
    ac.direction_deg = 90.0f;
    ac.baro_vertical_rate_fpm = 500;
    ac.gnss_vertical_rate_fpm = 480;
    ac.navigation_integrity_category = static_cast<ADSBTypes::NICRadiusOfContainment>(7);
    ac.navigation_integrity_category_baro = static_cast<ADSBTypes::NICBarometricAltitudeIntegrity>(1);
    ac.navigation_accuracy_category_position = static_cast<ADSBTypes::NACEstimatedPositionUncertainty>(8);
    ac.navigation_accuracy_category_velocity = static_cast<ADSBTypes::NACHorizontalVelocityError>(1);
    ac.surveillance_integrity_level =
        static_cast<ADSBTypes::SILProbabilityOfExceedingNICRadiusOfContainmnent>(2);
    ac.geometric_vertical_accuracy = static_cast<ADSBTypes::GVA>(1);
    ac.system_design_assurance = static_cast<ADSBTypes::SystemDesignAssurance>(1);
    ac.uat_version = 0;
    ac.last_message_signal_strength_dbm = -60;
    ac.metrics.valid_frames = 7;
    ac.emergency_priority_status = UATAircraft::kEmergencyStatusMinimumFuel;

    ac.WriteBitFlag(UATAircraft::kBitFlagPositionValid, true);
    ac.WriteBitFlag(UATAircraft::kBitFlagBaroAltitudeValid, true);
    ac.WriteBitFlag(UATAircraft::kBitFlagGNSSAltitudeValid, true);
    ac.WriteBitFlag(UATAircraft::kBitFlagHorizontalSpeedValid, true);
    ac.WriteBitFlag(UATAircraft::kBitFlagDirectionValid, true);
    ac.WriteBitFlag(UATAircraft::kBitFlagBaroVerticalRateValid, true);
    ac.WriteBitFlag(UATAircraft::kBitFlagGNSSVerticalRateValid, true);

    int16_t len = WriteAircraftJSONUATAircraftStr(buf, ac);
    ASSERT_GT(len, 0);

    std::string_view json(buf, len);
    printf("UATAircraftAllFields: %.*s", len, buf);

    EXPECT_EQ(GetJSONValue(json, "hex"), "abcdef");
    EXPECT_EQ(GetJSONValue(json, "type"), "adsb_icao");
    EXPECT_EQ(GetJSONValue(json, "flight"), "AAL456");
    EXPECT_EQ(GetJSONValue(json, "squawk"), "7654");
    EXPECT_EQ(GetJSONValue(json, "category"), "B1");
    EXPECT_EQ(GetJSONValue(json, "alt_baro"), "10000");
    EXPECT_EQ(GetJSONValue(json, "alt_geom"), "10200");
    EXPECT_EQ(GetJSONValue(json, "gs"), "200");
    EXPECT_EQ(GetJSONValue(json, "track"), "90.0");
    EXPECT_EQ(GetJSONValue(json, "baro_rate"), "500");
    EXPECT_EQ(GetJSONValue(json, "geom_rate"), "480");
    EXPECT_EQ(GetJSONValue(json, "nic"), "7");
    EXPECT_EQ(GetJSONValue(json, "nic_baro"), "1");
    EXPECT_EQ(GetJSONValue(json, "nac_p"), "8");
    EXPECT_EQ(GetJSONValue(json, "nac_v"), "1");
    EXPECT_EQ(GetJSONValue(json, "sil"), "2");
    EXPECT_EQ(GetJSONValue(json, "gva"), "1");
    EXPECT_EQ(GetJSONValue(json, "sda"), "1");
    EXPECT_EQ(GetJSONValue(json, "version"), "0");
    EXPECT_EQ(GetJSONValue(json, "rssi"), "-60");
    EXPECT_EQ(GetJSONValue(json, "messages"), "7");
    EXPECT_EQ(GetJSONValue(json, "emergency"), "minfuel");

    EXPECT_EQ(buf[len - 1], '\n');
}

TEST(AircraftJSON, UATAircraftNonICAOHexPrefix) {
    char buf[kAircraftJSONMessageStrMaxLen];
    UATAircraft ac;

    // Self-assigned temporary address → '~' prefix.
    ac.icao_address =
        0x112233 | ((UATAircraft::kADSBTargetWithSelfAssignedTemporaryAddress << Aircraft::kAddressQualifierBitShift) &
                    Aircraft::kAddressQualifierMask);
    WriteAircraftJSONUATAircraftStr(buf, ac);
    std::string hex = GetJSONValue(std::string_view(buf), "hex");
    EXPECT_EQ(hex[0], '~');
    EXPECT_EQ(hex.substr(1), "112233");
    EXPECT_EQ(GetJSONValue(std::string_view(buf), "type"), "adsb_other");

    // TIS-B track file identifier → '~' prefix.
    ac.icao_address =
        0x445566 | ((UATAircraft::kTISBTargetWithTrackFileIdentifier << Aircraft::kAddressQualifierBitShift) &
                    Aircraft::kAddressQualifierMask);
    WriteAircraftJSONUATAircraftStr(buf, ac);
    hex = GetJSONValue(std::string_view(buf), "hex");
    EXPECT_EQ(hex[0], '~');
    EXPECT_EQ(hex.substr(1), "445566");
    EXPECT_EQ(GetJSONValue(std::string_view(buf), "type"), "tisb_trackfile");

    // TIS-B with ICAO address → no '~'.
    ac.icao_address =
        0x778899 | ((UATAircraft::kTISBTargetWithICAO24BitAddress << Aircraft::kAddressQualifierBitShift) &
                    Aircraft::kAddressQualifierMask);
    WriteAircraftJSONUATAircraftStr(buf, ac);
    hex = GetJSONValue(std::string_view(buf), "hex");
    EXPECT_NE(hex[0], '~');
    EXPECT_EQ(hex, "778899");
    EXPECT_EQ(GetJSONValue(std::string_view(buf), "type"), "tisb_icao");
}

TEST(AircraftJSON, UATAircraftNoEmergencyField) {
    char buf[kAircraftJSONMessageStrMaxLen];
    UATAircraft ac;
    ac.icao_address = 0xAAAAAA;
    // Default emergency_priority_status is kEmergencyPriorityStatusNone.
    WriteAircraftJSONUATAircraftStr(buf, ac);
    EXPECT_FALSE(HasJSONKey(std::string_view(buf), "emergency"));
}

TEST(AircraftJSON, UATAircraftVersionOmittedWhenUnset) {
    char buf[kAircraftJSONMessageStrMaxLen];
    UATAircraft ac;
    ac.icao_address = 0xBBBBBB;
    ac.uat_version = -1;  // Unset.
    WriteAircraftJSONUATAircraftStr(buf, ac);
    EXPECT_FALSE(HasJSONKey(std::string_view(buf), "version"));

    ac.uat_version = 0;
    WriteAircraftJSONUATAircraftStr(buf, ac);
    EXPECT_TRUE(HasJSONKey(std::string_view(buf), "version"));
    EXPECT_EQ(GetJSONValue(std::string_view(buf), "version"), "0");
}

// ─── EmitterCategory string conversion ─────────────────────────────────────

TEST(AircraftJSON, EmitterCategoryStrings) {
    char cat[4];

    // A0 — no category info
    EXPECT_TRUE(EmitterCategoryToStr(cat, sizeof(cat), ADSBTypes::kEmitterCategoryNoCategoryInfo));
    EXPECT_STREQ(cat, "A0");

    // A3 — medium2
    EXPECT_TRUE(EmitterCategoryToStr(cat, sizeof(cat), ADSBTypes::kEmitterCategoryMedium2));
    EXPECT_STREQ(cat, "A3");

    // A7 — rotorcraft
    EXPECT_TRUE(EmitterCategoryToStr(cat, sizeof(cat), ADSBTypes::kEmitterCategoryRotorcraft));
    EXPECT_STREQ(cat, "A7");

    // B1 — glider/sailplane (enum value 9)
    EXPECT_TRUE(EmitterCategoryToStr(cat, sizeof(cat), ADSBTypes::kEmitterCategoryGliderSailplane));
    EXPECT_STREQ(cat, "B1");

    // B6 — UAV (enum value 14)
    EXPECT_TRUE(EmitterCategoryToStr(cat, sizeof(cat), ADSBTypes::kEmitterCategoryUnmannedAerialVehicle));
    EXPECT_STREQ(cat, "B6");

    // C1 — surface emergency vehicle (enum value 17)
    EXPECT_TRUE(EmitterCategoryToStr(cat, sizeof(cat), ADSBTypes::kEmitterCategorySurfaceEmergencyVehicle));
    EXPECT_STREQ(cat, "C1");

    // C3 — point obstacle (enum value 19)
    EXPECT_TRUE(EmitterCategoryToStr(cat, sizeof(cat), ADSBTypes::kEmitterCategoryPointObstacle));
    EXPECT_STREQ(cat, "C3");

    // Invalid → returns false.
    EXPECT_FALSE(EmitterCategoryToStr(cat, sizeof(cat), ADSBTypes::kEmitterCategoryInvalid));
}
