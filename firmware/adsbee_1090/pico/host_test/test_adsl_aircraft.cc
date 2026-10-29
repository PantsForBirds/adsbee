#include <cstring>
#include <string>
#include <vector>

#include "adsl/adsl_vectors.hh"
#include "aircraft_dictionary.hh"
#include "aircraftjson_utils.hh"
#include "composite_array.hh"
#include "gtest/gtest.h"
#include "hal_god_powers.hh"
#include "raw_utils.hh"

namespace {
std::vector<uint8_t> Hex(const char* hex) {
    std::vector<uint8_t> out;
    for (size_t i = 0; hex[i] && hex[i + 1]; i += 2) out.push_back(std::stoi(std::string(hex + i, 2), nullptr, 16));
    return out;
}
const ADSLGoldenVector& Vector(const char* name) {
    for (const ADSLGoldenVector& v : kADSLGoldenVectors) {
        if (strcmp(v.name, name) == 0) return v;
    }
    abort();
}
RawADSLPacket Packet(const char* name, int16_t rssi = -70) {
    std::vector<uint8_t> b = Hex(Vector(name).packet_hex);
    RawADSLPacket p(b.data(), b.size(), rssi, 0, 12345);
    p.channel = RawADSLPacket::kChannel868_2MHz;
    return p;
}
}  // namespace

TEST(ADSLAircraft, TrafficCreatesAircraft) {
    set_time_since_boot_ms(1000);
    AircraftDictionary dict;
    const ADSLGoldenVector& v = Vector("traffic_v0_k0");
    ASSERT_TRUE(dict.IngestADSLPacket(Packet("traffic_v0_k0")));
    uint32_t uid = ADSLAircraft::AddressToUID(0xBEE000, DecodedADSLPacket::kAddressMappingICAO);
    EXPECT_EQ(Aircraft::UIDToAircraftType(uid), Aircraft::kAircraftTypeADSL);
    ADSLAircraft* ac = dict.GetAircraftPtr<ADSLAircraft>(uid, false);
    ASSERT_NE(ac, nullptr);
    EXPECT_EQ(ac->address, 0xBEE000u);
    EXPECT_EQ(ac->address_mapping, DecodedADSLPacket::kAddressMappingICAO);
    EXPECT_TRUE(ac->HasBitFlag(ADSLAircraft::kBitFlagPositionValid));
    EXPECT_NEAR(ac->latitude_deg, v.lat, 1e-4);
    EXPECT_NEAR(ac->longitude_deg, v.lon, 1e-4);
    EXPECT_TRUE(ac->HasBitFlag(ADSLAircraft::kBitFlagGNSSAltitudeValid));
    EXPECT_EQ(ac->gnss_altitude_ft, 3280);  // 1000 m.
    EXPECT_EQ(ac->speed_kts, 117);          // 60 m/s.
    EXPECT_NEAR(ac->direction_deg, 135.0f, 0.01f);
    EXPECT_EQ(ac->gnss_vertical_rate_fpm, 492);  // 2.5 m/s.
    EXPECT_TRUE(ac->HasBitFlag(ADSLAircraft::kBitFlagIsAirborne));
    EXPECT_EQ(ac->GetEmitterCategory(), ADSBTypes::kEmitterCategoryLight);
    EXPECT_EQ(ac->GetNACp(), 9);  // HFOM 5 (10-30 m).
    EXPECT_EQ(ac->last_message_signal_strength_dbm, -70);
    EXPECT_EQ(ac->num_frames_received, 1u);

    // Status from the same sender updates it.
    ASSERT_TRUE(dict.IngestADSLPacket(Packet("status_v1_k0")));
    EXPECT_TRUE(ac->HasBitFlag(ADSLAircraft::kBitFlagStatusValid));
    EXPECT_EQ(ac->max_protocol_version, 1);
    EXPECT_EQ(ac->num_frames_received, 2u);

    dict.Update(1000);
    EXPECT_EQ(dict.metrics.num_adsl_aircraft, 1);
}

TEST(ADSLAircraft, StatusAloneCreatesNothingAndBadCRCIsDropped) {
    AircraftDictionary dict;
    EXPECT_FALSE(dict.IngestADSLPacket(Packet("status_v1_k0")));
    EXPECT_FALSE(dict.IngestADSLPacket(Packet("traffic_bad_crc")));
    EXPECT_EQ(dict.GetNumAircraft(), 0);
}

TEST(ADSLAircraft, InvalidFieldsLeaveFlagsCleared) {
    AircraftDictionary dict;
    ASSERT_TRUE(dict.IngestADSLPacket(Packet("traffic_invalid")));
    ADSLAircraft* ac = dict.GetAircraftPtr<ADSLAircraft>(
        ADSLAircraft::AddressToUID(Vector("traffic_invalid").address, DecodedADSLPacket::kAddressMappingICAO), false);
    ASSERT_NE(ac, nullptr);
    EXPECT_FALSE(ac->HasBitFlag(ADSLAircraft::kBitFlagPositionValid));
    EXPECT_FALSE(ac->HasBitFlag(ADSLAircraft::kBitFlagGNSSAltitudeValid));
    EXPECT_FALSE(ac->HasBitFlag(ADSLAircraft::kBitFlagHorizontalSpeedValid));
    EXPECT_FALSE(ac->HasBitFlag(ADSLAircraft::kBitFlagGNSSVerticalRateValid));
    EXPECT_FALSE(ac->HasBitFlag(ADSLAircraft::kBitFlagDirectionValid));
}

TEST(ADSLAircraft, AddressMappingIsPartOfTheUID) {
    EXPECT_NE(ADSLAircraft::AddressToUID(0xBEE001, DecodedADSLPacket::kAddressMappingICAO),
              ADSLAircraft::AddressToUID(0xBEE001, DecodedADSLPacket::kAddressMappingFLARM));
    EXPECT_NE(ADSLAircraft::AddressToUID(0xBEE001, DecodedADSLPacket::kAddressMappingICAO),
              Aircraft::ICAOToUID(0xBEE001, Aircraft::kAircraftTypeModeS));
    // Manufacturer pages above 15 share the top qualifier value.
    EXPECT_EQ(ADSLAircraft::AddressToUID(0x000001, 40), ADSLAircraft::AddressToUID(0x000001, 63));
    ADSLAircraft from_uid(Aircraft::UIDToICAOAddress(ADSLAircraft::AddressToUID(0xBEE001, 8)));
    EXPECT_EQ(from_uid.address, 0xBEE001u);
    EXPECT_EQ(from_uid.address_mapping, 8);
}

TEST(ADSLAircraft, AircraftJSON) {
    set_time_since_boot_ms(5000);
    AircraftDictionary dict;
    ASSERT_TRUE(dict.IngestADSLPacket(Packet("traffic_neg")));
    const ADSLGoldenVector& v = Vector("traffic_neg");
    ADSLAircraft* ac = dict.GetAircraftPtr<ADSLAircraft>(ADSLAircraft::AddressToUID(v.address, v.amt), false);
    ASSERT_NE(ac, nullptr);
    char buf[kAircraftJSONMessageStrMaxLen];
    int16_t len = WriteAircraftJSONADSLAircraftStr(buf, *ac, 6000);
    ASSERT_GT(len, 0);
    std::string json(buf, len);
    char hex[16];
    snprintf(hex, sizeof(hex), "\"hex\":\"%06x\"", (unsigned)v.address);
    EXPECT_NE(json.find(hex), std::string::npos) << json;
    EXPECT_NE(json.find("\"type\":\"adsl\""), std::string::npos) << json;
    EXPECT_NE(json.find("\"lat\":"), std::string::npos) << json;
    EXPECT_NE(json.find("\"seen_pos\":1.0"), std::string::npos) << json;
    EXPECT_NE(json.find("\"adsl_amt\":\"icao\""), std::string::npos) << json;
    EXPECT_EQ(json.back(), '\n');
}

TEST(ADSLAircraft, RawFrame) {
    RawADSLPacket p = Packet("traffic_v0_k0", -61);
    char buf[kRawADSLFrameMaxNumChars];
    uint16_t len = BuildRawADSLFrame(p, buf);
    ASSERT_GT(len, 0);
    std::string expected = "#ADL*" + std::string(Vector("traffic_v0_k0").packet_hex) + ";(-61,0,0,0000000000003039)\r\n";
    for (char& c : expected) c = toupper(c);
    expected[0] = '#';
    std::string got(buf, len);
    for (char& c : got) c = toupper(c);
    EXPECT_EQ(got, expected);
}
