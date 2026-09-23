#include "aircraft_dictionary.hh"
#include "csbee_utils.hh"
#include "gtest/gtest.h"

std::string_view GetNextToken(std::string_view* message_in = nullptr, char delimiter = ',') {
    static uint16_t token_start;
    static std::string_view* message;
    if (message_in != nullptr) {
        message = message_in;
        token_start = 0;
    }
    std::string_view token = message->substr(token_start, message->find(delimiter, token_start) - token_start);
    token_start += token.length() + 1;
    return token;
}

TEST(CSBeeUtils, ModeSAircraftToCSBeeString) {
    char message[kCSBeeMessageStrMaxLen];

    ModeSAircraft aircraft;
    aircraft.flags = UINT32_MAX;  // Set allll the flags.
    aircraft.last_message_timestamp_ms = 1000;
    aircraft.last_message_signal_strength_dbm = -75;
    aircraft.last_message_signal_quality_db = 2;
    aircraft.metrics.valid_squitter_frames = 1;
    aircraft.metrics.valid_extended_squitter_frames = 3;
    aircraft.transponder_capability = ModeSADSBPacket::Capability::kCALevel2PlusTransponderOnSurfaceCanSetCA7;
    aircraft.icao_address = 0x12345E;
    strcpy(aircraft.callsign, "ABCDEFG");
    aircraft.squawk = 1234;
    aircraft.emitter_category = ADSBTypes::kEmitterCategoryGliderSailplane;
    aircraft.baro_altitude_ft = 1000;
    aircraft.gnss_altitude_ft = 997;
    aircraft.altitude_source = ADSBTypes::kAltitudeSourceBaro;
    aircraft.latitude_deg = -120.654321;
    aircraft.longitude_deg = -80.123456;
    aircraft.direction_deg = 300.5678;
    aircraft.speed_kts = 123.45;
    aircraft.speed_source = ADSBTypes::kSpeedSourceAirspeedTrue;
    aircraft.baro_vertical_rate_fpm = -200;
    aircraft.gnss_vertical_rate_fpm = -205;
    aircraft.navigation_integrity_category = static_cast<ADSBTypes::NICRadiusOfContainment>(0b1011);
    aircraft.navigation_integrity_category_baro = static_cast<ADSBTypes::NICBarometricAltitudeIntegrity>(0b1);
    aircraft.navigation_accuracy_category_velocity = static_cast<ADSBTypes::NACHorizontalVelocityError>(0b101);
    aircraft.navigation_accuracy_category_position = static_cast<ADSBTypes::NACEstimatedPositionUncertainty>(0b1101);
    aircraft.geometric_vertical_accuracy = static_cast<ADSBTypes::GVA>(0b11);
    aircraft.surveillance_integrity_level = static_cast<ADSBTypes::SILProbabilityOfExceedingNICRadiusOfContainmnent>(
        ADSBTypes::kPOERCLessThanOrEqualTo1em5PerFlightHour);
    aircraft.system_design_assurance = static_cast<ADSBTypes::SystemDesignAssurance>(0b11);
    aircraft.gnss_antenna_offset_right_of_reference_point_m = -6;
    aircraft.length_m = 10;
    aircraft.width_m = 20;
    aircraft.adsb_version = 3;

    char expected_nicnac_str[100];
    sprintf(expected_nicnac_str, "%X",
            aircraft.navigation_integrity_category | (aircraft.navigation_integrity_category_baro << 4) |
                (aircraft.navigation_accuracy_category_velocity << 5) |
                (aircraft.navigation_accuracy_category_position << 8) | (aircraft.geometric_vertical_accuracy << 12) |
                (aircraft.surveillance_integrity_level << 14));

    char expected_acdims_str[100];
    sprintf(expected_acdims_str, "%X",
            aircraft.system_design_assurance | (aircraft.width_m << 2) | (aircraft.length_m << 9) |
                (aircraft.gnss_antenna_offset_right_of_reference_point_m << 16) | (0 << 24));

    WriteCSBeeModeSAircraftMessageStr(message, aircraft);
    std::string_view message_view(message);
    printf("%s\r\n", message_view.data());
    std::string_view token = GetNextToken(&message_view);
    EXPECT_EQ(token.compare("#A:12345E"), 0);                   // ICAO Address
    EXPECT_EQ(GetNextToken().compare("FFFFFFFF"), 0);           // Flags
    EXPECT_EQ(GetNextToken().compare("ABCDEFG"), 0);            // Callsign
    EXPECT_EQ(GetNextToken().compare("1234"), 0);               // Squawk
    EXPECT_EQ(GetNextToken().compare("9"), 0);                  // Emitter Category
    EXPECT_EQ(GetNextToken().compare("-120.65432"), 0);         // Latitude [deg]
    EXPECT_EQ(GetNextToken().compare("-80.12346"), 0);          // Longitude [deg]
    EXPECT_EQ(GetNextToken().compare("1000"), 0);               // Baro Altitude [ft]
    EXPECT_EQ(GetNextToken().compare("997"), 0);                // GNSS Altitude [ft]
    EXPECT_EQ(GetNextToken().compare("301"), 0);                // Direction [deg]
    EXPECT_EQ(GetNextToken().compare("123"), 0);                // Horizontal Speed [kts]
    EXPECT_EQ(GetNextToken().compare("-200"), 0);               // Baro Vertical Rate [fpm]
    EXPECT_EQ(GetNextToken().compare("-205"), 0);               // GNSS Vertical Rate [fpm]
    EXPECT_EQ(GetNextToken().compare(expected_nicnac_str), 0);  // NICNAC
    EXPECT_EQ(GetNextToken().compare(expected_acdims_str), 0);  // ACDIMS
    EXPECT_EQ(GetNextToken().compare("3"), 0);                  // ADS-B Version
    EXPECT_EQ(GetNextToken().compare("-75"), 0);                // Signal Strength [dBm]
    EXPECT_EQ(GetNextToken().compare("2"), 0);                  // Signal Qualtiy [dB]
    EXPECT_EQ(GetNextToken().compare("1"), 0);                  // Squitter Frames Per Second
    EXPECT_EQ(GetNextToken().compare("3"), 0);                  // Extended Squitter Frames Per Second

    // Check CRC
    std::string_view crc_str = GetNextToken();
    char calculated_crc_string[kCRCMaxNumChars + 1];
    sprintf(calculated_crc_string, "%X\r\n",
            CalculateCRC16((uint8_t*)message, message_view.length() - crc_str.length()));
    printf("Reported CRC=%s Calculated CRC=%s\r\n", std::string(crc_str).c_str(), calculated_crc_string);
    EXPECT_EQ(crc_str.compare(calculated_crc_string), 0);
}

TEST(CSBeeUtils, UATAircraftToCSBeeString) {
    char message[kCSBeeMessageStrMaxLen];

    UATAircraft aircraft;
    aircraft.flags = UINT32_MAX;  // Set allll the flags.
    aircraft.last_message_timestamp_ms = 1000;
    aircraft.last_message_signal_strength_dbm = -75;
    aircraft.last_message_signal_quality_bits = 2;
    aircraft.metrics.valid_frames = 1;
    aircraft.transponder_capability = ModeSADSBPacket::Capability::kCALevel2PlusTransponderOnSurfaceCanSetCA7;
    aircraft.icao_address = 0x12345E;
    strcpy(aircraft.callsign, "ABCDEFG");
    aircraft.squawk = 1234;
    aircraft.emitter_category = ADSBTypes::kEmitterCategoryGliderSailplane;
    aircraft.baro_altitude_ft = 1000;
    aircraft.gnss_altitude_ft = 997;
    aircraft.latitude_deg = -120.654321;
    aircraft.longitude_deg = -80.123456;
    aircraft.direction_deg = 300.5678;
    aircraft.speed_kts = 123.45;
    aircraft.baro_vertical_rate_fpm = -200;
    aircraft.gnss_vertical_rate_fpm = -205;
    aircraft.emergency_priority_status = static_cast<UATAircraft::EmergencyPriorityStatus>(6);
    aircraft.navigation_integrity_category = static_cast<ADSBTypes::NICRadiusOfContainment>(0b1011);
    aircraft.navigation_integrity_category_baro = static_cast<ADSBTypes::NICBarometricAltitudeIntegrity>(0b1);
    aircraft.navigation_accuracy_category_velocity = static_cast<ADSBTypes::NACHorizontalVelocityError>(0b101);
    aircraft.navigation_accuracy_category_position = static_cast<ADSBTypes::NACEstimatedPositionUncertainty>(0b1101);
    aircraft.geometric_vertical_accuracy = static_cast<ADSBTypes::GVA>(0b11);
    aircraft.surveillance_integrity_level = static_cast<ADSBTypes::SILProbabilityOfExceedingNICRadiusOfContainmnent>(
        ADSBTypes::kPOERCLessThanOrEqualTo1em5PerFlightHour);
    aircraft.system_design_assurance = static_cast<ADSBTypes::SystemDesignAssurance>(0b11);
    aircraft.gnss_antenna_offset_right_of_reference_point_m = -6;
    aircraft.gnss_antenna_offset_forward_of_reference_point_m = -127;
    aircraft.length_m = 10;
    aircraft.width_m = 20;
    aircraft.uat_version = 3;

    char expected_nicnac_str[100];
    sprintf(expected_nicnac_str, "%X",
            aircraft.navigation_integrity_category | (aircraft.navigation_integrity_category_baro << 4) |
                (aircraft.navigation_accuracy_category_velocity << 5) |
                (aircraft.navigation_accuracy_category_position << 8) | (aircraft.geometric_vertical_accuracy << 12) |
                (aircraft.surveillance_integrity_level << 14));

    char expected_acdims_str[100];
    sprintf(expected_acdims_str, "%X",
            aircraft.system_design_assurance | (aircraft.width_m << 2) | (aircraft.length_m << 9) |
                (aircraft.gnss_antenna_offset_right_of_reference_point_m << 16) |
                (aircraft.gnss_antenna_offset_forward_of_reference_point_m << 24));

    WriteCSBeeUATAircraftMessageStr(message, aircraft);
    std::string_view message_view(message);
    printf("%s\r\n", message_view.data());
    std::string_view token = GetNextToken(&message_view);
    EXPECT_EQ(token.compare("#U:12345E"), 0);                   // ICAO Address
    EXPECT_EQ(GetNextToken().compare("FFFFFFFF"), 0);           // Flags
    EXPECT_EQ(GetNextToken().compare("ABCDEFG"), 0);            // Callsign
    EXPECT_EQ(GetNextToken().compare("1234"), 0);               // Squawk
    EXPECT_EQ(GetNextToken().compare("9"), 0);                  // Emitter Category
    EXPECT_EQ(GetNextToken().compare("-120.65432"), 0);         // Latitude [deg]
    EXPECT_EQ(GetNextToken().compare("-80.12346"), 0);          // Longitude [deg]
    EXPECT_EQ(GetNextToken().compare("1000"), 0);               // Baro Altitude [ft]
    EXPECT_EQ(GetNextToken().compare("997"), 0);                // GNSS Altitude [ft]
    EXPECT_EQ(GetNextToken().compare("301"), 0);                // Direction [deg]
    EXPECT_EQ(GetNextToken().compare("123"), 0);                // Horizontal Speed [kts]
    EXPECT_EQ(GetNextToken().compare("-200"), 0);               // Baro Vertical Rate [fpm]
    EXPECT_EQ(GetNextToken().compare("-205"), 0);               // GNSS Vertical Rate [fpm]
    EXPECT_EQ(GetNextToken().compare("6"), 0);                  // Emergency Priority Status
    EXPECT_EQ(GetNextToken().compare(expected_nicnac_str), 0);  // NICNAC
    EXPECT_EQ(GetNextToken().compare(expected_acdims_str), 0);  // ACDIMS
    EXPECT_EQ(GetNextToken().compare("3"), 0);                  // ADS-B Version
    EXPECT_EQ(GetNextToken().compare("-75"), 0);                // Signal Strength [dBm]
    EXPECT_EQ(GetNextToken().compare("2"), 0);                  // Signal Qualtiy [bits]
    EXPECT_EQ(GetNextToken().compare("1"), 0);                  // UAT Frames Per Second

    // Check CRC
    std::string_view crc_str = GetNextToken();
    char calculated_crc_string[kCRCMaxNumChars + 1];
    sprintf(calculated_crc_string, "%X\r\n",
            CalculateCRC16((uint8_t*)message, message_view.length() - crc_str.length()));
    printf("Reported CRC=%s Calculated CRC=%s\r\n", std::string(crc_str).c_str(), calculated_crc_string);
    EXPECT_EQ(crc_str.compare(calculated_crc_string), 0);
}

TEST(CSBeeUtils, UATAircraftWithAddressQualifierToCSBeeString) {
    char message[kCSBeeMessageStrMaxLen];

    UATAircraft aircraft;
    aircraft.flags = 0;
    aircraft.last_message_timestamp_ms = 1000;
    aircraft.last_message_signal_strength_dbm = -80;
    aircraft.last_message_signal_quality_bits = 3;
    aircraft.metrics.valid_frames = 2;
    // Pack the address qualifier into the upper bits of icao_address.
    aircraft.icao_address =
        0x12345E | ((UATAircraft::kTISBTargetWithICAO24BitAddress << Aircraft::kAddressQualifierBitShift) &
                    Aircraft::kAddressQualifierMask);
    // Make sure we did the packing right.
    ASSERT_EQ(aircraft.GetAddressQualifier(), UATAircraft::kTISBTargetWithICAO24BitAddress);
    ASSERT_EQ(aircraft.icao_address & (~Aircraft::kAddressQualifierMask), 0x12345Eu);

    WriteCSBeeUATAircraftMessageStr(message, aircraft);
    std::string_view message_view(message);
    printf("%s\r\n", message_view.data());
    std::string_view token = GetNextToken(&message_view);
    // Address qualifier 2 (kTISBTargetWithICAO24BitAddress) is packed into bits 24-26, so ICAO prints as 2<<24 |
    // 0x12345E = 0x212345E.
    EXPECT_EQ(token.compare("#U:212345E"), 0);  // ICAO with address qualifier prepended

    // Advance past the remaining data fields to reach the CRC.
    for (int i = 0; i < 19; i++) {
        GetNextToken();
    }

    // Check CRC
    std::string_view crc_str = GetNextToken();
    char calculated_crc_string[kCRCMaxNumChars + 1];
    sprintf(calculated_crc_string, "%X\r\n",
            CalculateCRC16((uint8_t*)message, message_view.length() - crc_str.length()));
    printf("Reported CRC=%s Calculated CRC=%s\r\n", std::string(crc_str).c_str(), calculated_crc_string);
    EXPECT_EQ(crc_str.compare(calculated_crc_string), 0);
}

TEST(CSBeeUtils, CSBeeStatisticsMessage) {
    char message[kCSBeeMessageStrMaxLen];
    WriteCSBeeStatisticsMessageStr(message,
                                   10,      // sdps
                                   20,      // raw_sfps
                                   18,      // sfps
                                   10,      // raw_esfps
                                   8,       // esfps
                                   5,       // raw_uatfps
                                   4,       // uatfps
                                   10,      // num_aircraft
                                   123456,  // tscal
                                   654321   // uptime
    );

    std::string_view message_view(message);
    printf("%s\r\n", message_view.data());
    std::string_view token = GetNextToken(&message_view);

    char begin_str[10];
    sprintf(begin_str, "#S:%d", kCSBeeProtocolVersion);  // sdps

    EXPECT_EQ(token.compare(begin_str), 0);          // Start of statistics message
    EXPECT_EQ(GetNextToken().compare("10"), 0);      // sdps
    EXPECT_EQ(GetNextToken().compare("20"), 0);      // raw_sfps
    EXPECT_EQ(GetNextToken().compare("18"), 0);      // sfps
    EXPECT_EQ(GetNextToken().compare("10"), 0);      // raw_esfps
    EXPECT_EQ(GetNextToken().compare("8"), 0);       // esfps
    EXPECT_EQ(GetNextToken().compare("5"), 0);       // raw_uatfps
    EXPECT_EQ(GetNextToken().compare("4"), 0);       // uatfps
    EXPECT_EQ(GetNextToken().compare("10"), 0);      // num_aircraft
    EXPECT_EQ(GetNextToken().compare("123456"), 0);  // tscal
    EXPECT_EQ(GetNextToken().compare("654321"), 0);  // uptime

    // Check CRC
    std::string_view crc_str = GetNextToken();
    char calculated_crc_string[kCRCMaxNumChars + 1];
    sprintf(calculated_crc_string, "%X\r\n",
            CalculateCRC16((uint8_t*)message, message_view.length() - crc_str.length()));
    printf("Reported CRC=%s Calculated CRC=%s\r\n", std::string(crc_str).c_str(), calculated_crc_string);
    EXPECT_EQ(crc_str.compare(calculated_crc_string), 0);
}