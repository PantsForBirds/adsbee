#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>

#include "aircraft_dictionary.hh"
#include "awb_test_utils.hh"
#include "awb_utils.h"
#include "decode_utils.hh"
#include "gdl90_utils.hh"
#include "geo_utils.hh"
#include "gtest/gtest.h"
#include "hal_god_powers.hh"
#include "nmea_parser.hh"
#include "opendroneid.h"

// Latitudes and longitudes as AWB: integer conversions to and from decimal degrees, and every decoder and reporter
// against the float degrees they used before.

namespace {

// Exact value of awb in units of 1/scale degree, rounded half away from zero, from host 128-bit integers.
int64_t ExactScaledDeg(uint32_t awb, int64_t scale) {
    __int128 v = static_cast<__int128>(static_cast<int32_t>(awb)) * 360 * scale;
    __int128 half = static_cast<__int128>(1) << 31;
    __int128 q = v >= 0 ? (v + half) >> 32 : -((-v + half) >> 32);
    return static_cast<int64_t>(q);
}

}  // namespace

TEST(AWBPositions, AWBToScaledDegIsExactlyRounded) {
    std::mt19937 rng(1302);
    for (int i = 0; i < 200000; i++) {
        uint32_t awb = rng();
        for (uint8_t d = 0; d <= 7; d++) {
            ASSERT_EQ(AWBToScaledDeg(awb, d), ExactScaledDeg(awb, static_cast<int64_t>(pow(10, d)))) << awb;
        }
    }
    EXPECT_EQ(AWBToDegE7(0x40000000u), 900000000);    // 90 N.
    EXPECT_EQ(AWBToDegE7(0xC0000000u), -900000000);   // 90 S.
    EXPECT_EQ(AWBToDegE7(0x80000000u), -1800000000);  // 180 deg is -180.
    EXPECT_EQ(AWBToDegE7(0x7FFFFFFFu), 1799999999);  // 179.99999992 deg.
}

TEST(AWBPositions, DegE7RoundTrips) {
    std::mt19937 rng(7);
    std::uniform_int_distribution<int32_t> deg_e7(-1800000000, 1800000000);
    for (int i = 0; i < 200000; i++) {
        int32_t v = deg_e7(rng);
        uint32_t awb = DegE7ToAWB(v);
        // Nearest AWB (1 AWB = 0.84e-7 deg, so the round trip to 1e-7 deg is exact).
        __int128 exact = static_cast<__int128>(v) << 32;
        int64_t nearest = static_cast<int64_t>((exact + (exact >= 0 ? 1800000000 : -1800000000)) / 3600000000);
        ASSERT_EQ(awb, static_cast<uint32_t>(nearest)) << v;
        ASSERT_EQ(AWBToDegE7(awb), v == 1800000000 ? -1800000000 : v);
    }
}

TEST(AWBPositions, ScaledToAWBIsExactlyRounded) {
    std::mt19937_64 rng(11);
    const uint64_t turns[] = {360, 3600000000ull, 360ull * 60 * 10000000, (1ull << 46) + 12345};
    for (uint64_t units : turns) {
        for (int i = 0; i < 100000; i++) {
            int64_t v = static_cast<int64_t>(rng() % units) - static_cast<int64_t>(units / 2);
            __int128 exact = static_cast<__int128>(v < 0 ? -v : v) << 32;
            uint32_t magnitude = static_cast<uint32_t>((exact + units / 2) / units);
            ASSERT_EQ(ScaledToAWB(v, units), v < 0 ? 0u - magnitude : magnitude) << v << " " << units;
        }
    }
}

TEST(AWBPositions, FormatAWBDegMatchesPrintfOfTheExactValue) {
    std::mt19937 rng(3);
    char text[kAWBDegStrMaxLen], reference[32];
    for (int i = 0; i < 200000; i++) {
        uint32_t awb = rng();
        if (awb % (1u << 21) == 0) continue;  // Exact ties: printf rounds them to even, FormatAWBDeg away from zero.
        for (uint8_t d : {5, 6}) {
            FormatAWBDeg(text, sizeof(text), awb, d);
            snprintf(reference, sizeof(reference), "%.*f", d, AWBToDeg(awb));  // AWBToDeg is exact in double.
            if (strcmp(reference, "-0.00000") == 0 || strcmp(reference, "-0.000000") == 0) {
                memmove(reference, reference + 1, strlen(reference));
            }
            ASSERT_STREQ(text, reference) << awb;
        }
    }
    FormatAWBDeg(text, sizeof(text), 0x80000000u, 7);
    EXPECT_STREQ(text, "-180.0000000");  // The longest text.
    FormatAWBDeg(text, sizeof(text), DegToAWB(37.5), 0);
    EXPECT_STREQ(text, "38");
    FormatAWBDeg(text, sizeof(text), DegToAWB(-0.000001), 5);
    EXPECT_STREQ(text, "0.00000");  // No negative zero.
}

TEST(AWBPositions, DecimalTextRoundTrips) {
    // AT+RX_POSITION text in, AWB, then AT+RX_POSITION? text out: the same 6 decimals.
    std::mt19937 rng(5);
    std::uniform_int_distribution<int32_t> micro_deg(-180000000, 180000000);
    char in[24], out[kAWBDegStrMaxLen];
    for (int i = 0; i < 100000; i++) {
        int32_t v = micro_deg(rng);
        snprintf(in, sizeof(in), "%s%d.%06d", v < 0 ? "-" : "", abs(v) / 1000000, abs(v) % 1000000);
        int32_t deg_e7;
        ASSERT_TRUE(ParseDegE7(in, deg_e7)) << in;
        ASSERT_EQ(deg_e7, static_cast<int64_t>(v) * 10);
        FormatAWBDeg(out, sizeof(out), DegE7ToAWB(deg_e7), 6);
        std::string expected = std::string(in) == "-0.000000" ? "0.000000" : in;
        if (v == 180000000) expected = "-180.000000";  // 180 E is 180 W.
        ASSERT_EQ(std::string(out), expected);
    }
}

TEST(AWBPositions, ParseDegE7) {
    struct Case {
        const char* text;
        bool ok;
        int32_t deg_e7;
    };
    const Case cases[] = {
        {"37.4", true, 374000000},         {"-122.1", true, -1221000000},     {"+5", true, 50000000},
        {"0", true, 0},                    {"-0", true, 0},                   {".5", true, 5000000},
        {"5.", true, 50000000},            {"1e-7", true, 1},                 {"1E2", true, 1000000000},
        {"1e+2", true, 1000000000},        {"0.00000005", true, 1},           {"-0.00000005", true, -1},
        {"0.000000049999", true, 0},       {"37.41234567", true, 374123457},  {"5e-20", true, 0},
        {"180", true, 1800000000},         {"-180.0000000", true, -1800000000}, {"214.7483647", true, INT32_MAX},
        {"0000000000000000000000037.5", true, 375000000},                     {"37.50000000000000000000001", true, 375000000},
        {"214.7483648", false, 0},         {"1000", false, 0},                {"", false, 0},
        {"-", false, 0},                   {".", false, 0},                   {"abc", false, 0},
        {"1.2.3", false, 0},               {"--1", false, 0},                 {"1e", false, 0},
        {"37.4 ", false, 0},               {"0x10", false, 0},                {"nan", false, 0},
    };
    for (const Case& c : cases) {
        int32_t v = 12345;
        EXPECT_EQ(ParseDegE7(c.text, v), c.ok) << c.text;
        EXPECT_EQ(v, c.ok ? c.deg_e7 : 12345) << c.text;
    }
}

TEST(AWBPositions, AWBIsLatitude) {
    EXPECT_TRUE(AWBIsLatitude(0));
    EXPECT_TRUE(AWBIsLatitude(0x40000000u));
    EXPECT_TRUE(AWBIsLatitude(0xC0000000u));
    EXPECT_FALSE(AWBIsLatitude(0x40000001u));
    EXPECT_FALSE(AWBIsLatitude(0xBFFFFFFFu));
    EXPECT_FALSE(AWBIsLatitude(0x80000000u));
}

TEST(AWBPositions, FloatDegToAWBMatchesNASAConversions) {
    // The settings migration converts stored float degrees with FloatDegToAWB; the old CPR path used lat2awb/lon2awb.
    std::mt19937 rng(9);
    std::uniform_real_distribution<float> lat(-90.0f, 90.0f), lon(-180.0f, 180.0f);
    for (int i = 0; i < 100000; i++) {
        float a = lat(rng), o = lon(rng);
        ASSERT_LE(std::abs(static_cast<int32_t>(FloatDegToAWB(a) - DegToAWB(a))), 1) << a;
        ASSERT_LE(std::abs(static_cast<int32_t>(FloatDegToAWB(o) - DegToAWB(o))), 1) << o;
        // lat2awb/lon2awb go through float arithmetic: within its error at 0..360 deg (2^-15 deg = 364 AWB, twice).
        ASSERT_LE(std::abs(static_cast<int32_t>(lat2awb(a) - FloatDegToAWB(a))), 800) << a;
        ASSERT_LE(std::abs(static_cast<int32_t>(lon2awb(o) - FloatDegToAWB(o))), 800) << o;
    }
}

namespace {

// Latitude and longitude fields of a GDL90 traffic report, unescaped.
void GDL90LatLon(const uint8_t* frame, uint16_t len, uint32_t& lat24, uint32_t& lon24) {
    uint8_t raw[64];
    uint16_t n = 0;
    for (uint16_t i = 1; i + 1 < len && n < sizeof(raw); i++) {
        raw[n++] = frame[i] == GDL90Reporter::kGDL90ControlEscapeChar ? frame[++i] ^ 0x20 : frame[i];
    }
    lat24 = (raw[5] << 16) | (raw[6] << 8) | raw[7];
    lon24 = (raw[8] << 16) | (raw[9] << 8) | raw[10];
}

// GDL90 encoding before AWB: the float degrees awb2lat()/awb2lon() gave the dictionary, times 2^23 / 180, truncated.
uint32_t OldGDL90(float deg) { return static_cast<int32_t>(deg * 46603.3777778f) & 0x00FFFFFF; }

}  // namespace

TEST(AWBPositions, GDL90LatLonIsAWBRoundedTo24Bits) {
    GDL90Reporter gdl90;
    std::mt19937 rng(13);
    int differ = 0, total = 0;
    for (int i = 0; i < 20000; i++) {
        uint32_t lat_awb = rng();
        if (!AWBIsLatitude(lat_awb)) lat_awb ^= 0x80000000u;  // Into -90..90.
        uint32_t lon_awb = rng();
        GDL90Reporter::GDL90TargetReportData data;
        data.latitude_awb = lat_awb;
        data.longitude_awb = lon_awb;
        uint8_t frame[GDL90Reporter::kGDL90MessageMaxLenBytes];
        uint16_t len = gdl90.WriteGDL90TargetReportMessage(frame, sizeof(frame), data, false);
        uint32_t lat24, lon24;
        GDL90LatLon(frame, len, lat24, lon24);
        // Nearest 24-bit semicircle value of the exact position (ties up), exact in double: deg 2^23 / 180 = AWB / 256.
        ASSERT_EQ(lat24, static_cast<uint32_t>(static_cast<int64_t>(
                             std::floor(AWBToDeg(lat_awb) * 8388608.0 / 180.0 + 0.5))) & 0xFFFFFF);
        ASSERT_EQ(lon24, static_cast<uint32_t>(static_cast<int64_t>(
                             std::floor(AWBToDeg(lon_awb) * 8388608.0 / 180.0 + 0.5))) & 0xFFFFFF);
        // Before: truncated float, off by at most one LSB (2.1e-5 deg).
        uint32_t old_lat = OldGDL90(WrapCPRDecodeLatitude(awb2lat(lat_awb)));
        uint32_t old_lon = OldGDL90(WrapCPRDecodeLongitude(awb2lon(lon_awb)));
        ASSERT_LE(std::abs(static_cast<int32_t>(((lat24 - old_lat + 0x800000) & 0xFFFFFF) - 0x800000)), 1);
        ASSERT_LE(std::abs(static_cast<int32_t>(((lon24 - old_lon + 0x800000) & 0xFFFFFF) - 0x800000)), 1);
        differ += (lat24 != old_lat) + (lon24 != old_lon);
        total += 2;
    }
    // UAT positions arrive as 24-bit AWB: those now go out bit-identical to the received value.
    for (uint32_t v : {0x000001u, 0x123456u, 0x7FFFFFu, 0x800000u, 0xABCDEFu, 0xFFFFFFu}) {
        GDL90Reporter::GDL90TargetReportData data;
        data.latitude_awb = v << 8;
        data.longitude_awb = v << 8;
        uint8_t frame[GDL90Reporter::kGDL90MessageMaxLenBytes];
        uint16_t len = gdl90.WriteGDL90TargetReportMessage(frame, sizeof(frame), data, false);
        uint32_t lat24, lon24;
        GDL90LatLon(frame, len, lat24, lon24);
        EXPECT_EQ(lat24, v);
        EXPECT_EQ(lon24, v);
    }
    printf("GDL90: %d of %d lat/lon fields differ from the float encoding by one LSB.\n", differ, total);
}

TEST(AWBPositions, RemoteIDLatLonFrom1e7Degrees) {
    AircraftDictionary dictionary;
    const int32_t lat_e7 = -335678901, lon_e7 = 1512345678;  // As sent, 1e-7 deg.
    const int32_t operator_lat_e7 = -335600001, operator_lon_e7 = -1799999999;

    ODID_UAS_Data uas;
    odid_initUasData(&uas);
    uas.Location.Status = ODID_STATUS_AIRBORNE;
    uas.Location.Latitude = lat_e7 / 1e7;
    uas.Location.Longitude = lon_e7 / 1e7;
    uas.LocationValid = 1;
    uas.System.OperatorLatitude = operator_lat_e7 / 1e7;
    uas.System.OperatorLongitude = operator_lon_e7 / 1e7;
    uas.SystemValid = 1;
    ODID_MessagePack_encoded pack;
    ODID_MessagePack_data pack_data;
    odid_initMessagePackData(&pack_data);
    ASSERT_EQ(encodeLocationMessage(reinterpret_cast<ODID_Location_encoded*>(&pack_data.Messages[0]), &uas.Location),
              ODID_SUCCESS);
    ASSERT_EQ(encodeSystemMessage(reinterpret_cast<ODID_System_encoded*>(&pack_data.Messages[1]), &uas.System),
              ODID_SUCCESS);
    pack_data.MsgPackSize = 2;
    ASSERT_EQ(encodeMessagePack(&pack, &pack_data), ODID_SUCCESS);
    // The encoder rounds to the same integers.
    ASSERT_EQ(pack.Messages[0].location.Latitude, lat_e7);
    ASSERT_EQ(pack.Messages[1].system.OperatorLongitude, operator_lon_e7);

    RawRemoteIDPacket packet;
    const uint8_t mac[6] = {0x02, 0x13, 0x02, 0x00, 0x00, 0x01};
    memcpy(packet.source_mac, mac, sizeof(mac));
    packet.transport = RawRemoteIDPacket::kTransportBT4Legacy;
    packet.rssi_dbm = -60;
    memcpy(packet.payload, &pack, 3 + 2 * ODID_MESSAGE_SIZE);
    packet.payload_len_bytes = 3 + 2 * ODID_MESSAGE_SIZE;
    ASSERT_TRUE(dictionary.IngestRawRemoteIDPacket(packet));

    const RemoteIDAircraft* drone = dictionary.GetAircraftPtr<RemoteIDAircraft>(
        Aircraft::ICAOToUID(RemoteIDAircraft::MACToAddress(mac), Aircraft::kAircraftTypeRemoteID), false);
    ASSERT_NE(drone, nullptr);
    ASSERT_TRUE(drone->HasBitFlag(RemoteIDAircraft::kBitFlagPositionValid));
    EXPECT_EQ(drone->latitude_awb, DegE7ToAWB(lat_e7));
    EXPECT_EQ(drone->longitude_awb, DegE7ToAWB(lon_e7));
    EXPECT_EQ(AWBToDegE7(drone->latitude_awb), lat_e7);  // The 1e-7 deg value survives exactly.
    EXPECT_EQ(AWBToDegE7(drone->longitude_awb), lon_e7);
    EXPECT_EQ(AWBToDegE7(drone->operator_latitude_awb), operator_lat_e7);
    EXPECT_EQ(AWBToDegE7(drone->operator_longitude_awb), operator_lon_e7);
    // Before: float degrees, 6e-6 deg error at 151 deg.
    EXPECT_NEAR(AWBToDeg(drone->longitude_awb), static_cast<float>(lon_e7 / 1e7), 8e-6);
}

TEST(AWBPositions, NMEALatLonToAWB) {
    NMEAParser parser;
    const char* sentence = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";
    for (const char* c = sentence; *c; c++) parser.IngestByte(*c);
    ASSERT_TRUE(parser.fix().valid);
    // 48 deg 7.038 min and 11 deg 31 min, exactly rounded.
    EXPECT_EQ(parser.fix().latitude_awb, ScaledToAWB((48 * 60) * 1000LL + 7038, 360 * 60 * 1000LL));
    EXPECT_EQ(parser.fix().longitude_awb, ScaledToAWB((11 * 60 + 31) * 1000LL, 360 * 60 * 1000LL));
    EXPECT_NEAR(AWBToDeg(parser.fix().latitude_awb), 48.0 + 7.038 / 60, 1e-7);

    // Southern and western hemispheres, 8 minute decimals (rounded on the 8th), 3-digit degrees.
    const char* gga =
        "$GNGGA,001043.00,3353.61234567,S,15112.34567891,W,1,12,0.9,10.0,M,20.0,M,,*";
    uint8_t checksum = 0;
    for (const char* c = gga + 1; *c != '*'; c++) checksum ^= static_cast<uint8_t>(*c);
    char line[128];
    snprintf(line, sizeof(line), "%s%02X\r\n", gga, checksum);
    for (const char* c = line; *c; c++) parser.IngestByte(*c);
    EXPECT_EQ(parser.fix().latitude_awb, ScaledToAWB(-((33 * 60 + 53) * 10000000LL + 6123457), 216000000000LL));
    EXPECT_EQ(parser.fix().longitude_awb, ScaledToAWB(-((151 * 60 + 12) * 10000000LL + 3456789), 216000000000LL));
    EXPECT_NEAR(AWBToDeg(parser.fix().latitude_awb), -(33 + 53.61234567 / 60), 1e-7);
    EXPECT_NEAR(AWBToDeg(parser.fix().longitude_awb), -(151 + 12.34567891 / 60), 1e-7);
}
