#include <cstring>
#include <string>

#include "gtest/gtest.h"
#include "nmea_parser.hh"

namespace {

// Feed an entire NMEA sentence (including a trailing CRLF) byte-by-byte into the parser and return
// the SentenceType reported on the terminating byte.
NMEAParser::SentenceType FeedSentence(NMEAParser& parser, const char* sentence) {
    NMEAParser::SentenceType result = NMEAParser::kSentenceNone;
    for (const char* p = sentence; *p != '\0'; p++) {
        NMEAParser::SentenceType r = parser.IngestByte(*p);
        if (r != NMEAParser::kSentenceNone) result = r;
    }
    return result;
}

}  // namespace

TEST(NMEAParser, ParsesGPGGA) {
    NMEAParser parser;
    parser.SetTimestampMs(1000);
    // Standard GPS GGA with a valid fix at ~48.1173N, 11.5167E, 545.4m altitude, 8 satellites.
    auto type = FeedSentence(
        parser, "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n");
    EXPECT_EQ(type, NMEAParser::kSentenceGGA);

    const NMEAParser::GNSSFix& fix = parser.fix();
    EXPECT_TRUE(fix.valid);
    EXPECT_EQ(fix.num_satellites, 8);
    EXPECT_EQ(fix.fix_quality, 1);
    EXPECT_NEAR(fix.latitude_deg, 48.1173f, 0.001f);
    EXPECT_NEAR(fix.longitude_deg, 11.5167f, 0.001f);
    EXPECT_NEAR(fix.altitude_ft, static_cast<int32_t>(545.4f * 3.28084f), 2);
    EXPECT_EQ(fix.last_update_timestamp_ms, 1000u);
    EXPECT_TRUE(fix.utc_time_valid);
    EXPECT_EQ(fix.utc_hour, 12);
    EXPECT_EQ(fix.utc_minute, 35);
    EXPECT_EQ(fix.utc_second, 19);
    EXPECT_EQ(fix.utc_millisecond, 0);
}

TEST(NMEAParser, ParsesGPRMC) {
    NMEAParser parser;
    auto type = FeedSentence(
        parser, "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n");
    EXPECT_EQ(type, NMEAParser::kSentenceRMC);

    const NMEAParser::GNSSFix& fix = parser.fix();
    EXPECT_TRUE(fix.valid);
    EXPECT_NEAR(fix.latitude_deg, 48.1173f, 0.001f);
    EXPECT_NEAR(fix.longitude_deg, 11.5167f, 0.001f);
    EXPECT_EQ(fix.speed_kts, 22);          // 022.4 knots truncated to int.
    EXPECT_NEAR(fix.heading_deg, 84.4f, 0.1f);
}

// Multi-constellation talker IDs (GN/GL/GA) must parse the same as GP.
TEST(NMEAParser, TalkerIdAgnostic) {
    NMEAParser parser;
    auto type = FeedSentence(
        parser, "$GNRMC,123519,A,4807.038,N,01131.000,E,010.0,090.0,230394,,,A*66\r\n");
    EXPECT_EQ(type, NMEAParser::kSentenceRMC);
    EXPECT_TRUE(parser.fix().valid);
    EXPECT_NEAR(parser.fix().latitude_deg, 48.1173f, 0.001f);
}

// CASIC AT6558 receivers identify themselves with GPTXT and then emit multi-constellation
// GN-prefixed NMEA.  The informational and unsupported navigation sentences must not prevent the
// GGA/RMC sentences in the same stream from updating the fix state.
TEST(NMEAParser, ParsesAT6558StartupStream) {
    NMEAParser parser;

    EXPECT_EQ(FeedSentence(parser, "$GPTXT,01,01,02,MA=CASIC*27\r\n"),
              NMEAParser::kSentenceUnknown);
    EXPECT_EQ(FeedSentence(parser, "$GPTXT,01,01,02,IC=AT6558F-5N-32-1C580900*06\r\n"),
              NMEAParser::kSentenceUnknown);
    EXPECT_EQ(FeedSentence(parser, "$GPTXT,01,01,02,SW=URANUS5,V5.3.0.0*1D\r\n"),
              NMEAParser::kSentenceUnknown);

    parser.SetTimestampMs(1234);
    EXPECT_EQ(FeedSentence(parser, "$GNGGA,005538.537,,,,,0,00,25.5,,,,,,*70\r\n"),
              NMEAParser::kSentenceGGA);
    EXPECT_FALSE(parser.fix().valid);
    EXPECT_EQ(parser.fix().fix_quality, 0);
    EXPECT_EQ(parser.fix().num_satellites, 0);
    EXPECT_TRUE(parser.fix().utc_time_valid);
    EXPECT_EQ(parser.fix().utc_hour, 0);
    EXPECT_EQ(parser.fix().utc_minute, 55);
    EXPECT_EQ(parser.fix().utc_second, 38);
    EXPECT_EQ(parser.fix().utc_millisecond, 537);

    EXPECT_EQ(FeedSentence(parser, "$GPGSV,3,1,09,03,40,211,,04,67,330,,07,19,272,,08,08,169,,0*68\r\n"),
              NMEAParser::kSentenceUnknown);
    EXPECT_EQ(FeedSentence(parser, "$BDGSV,1,1,03,12,19,151,,19,45,158,,36,80,343,,0*45\r\n"),
              NMEAParser::kSentenceUnknown);
    EXPECT_EQ(FeedSentence(parser, "$GNRMC,005538.537,V,,,,,,,020826,,,M,V*2E\r\n"),
              NMEAParser::kSentenceRMC);
    EXPECT_FALSE(parser.fix().valid);
    EXPECT_EQ(parser.fix().utc_millisecond, 537);
}

// A NMEA 4.11-style sentence with extra appended trailing fields must still parse the leading
// fields and ignore the extras.
TEST(NMEAParser, ToleratesExtraTrailingFields) {
    NMEAParser parser;
    // GGA with extra trailing fields after the standard layout (checksum recomputed to be ignored
    // by feeding a sentence with no '*' so the tolerant path accepts it).
    auto type = FeedSentence(
        parser, "$GNGGA,123519,4807.038,N,01131.000,E,1,12,0.8,100.0,M,46.9,M,,,EXTRA,FIELDS\r\n");
    EXPECT_EQ(type, NMEAParser::kSentenceGGA);
    EXPECT_TRUE(parser.fix().valid);
    EXPECT_EQ(parser.fix().num_satellites, 12);
}

// A sentence with an incorrect checksum must be rejected (fix unchanged).
TEST(NMEAParser, RejectsBadChecksum) {
    NMEAParser parser;
    // Establish a known-good fix first.
    FeedSentence(parser, "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n");
    ASSERT_TRUE(parser.fix().valid);
    float good_lat = parser.fix().latitude_deg;

    // Now feed a sentence with a deliberately wrong checksum (*00); it must be rejected.
    auto type = FeedSentence(
        parser, "$GPGGA,123519,0000.000,S,00000.000,W,1,08,0.9,1.0,M,46.9,M,,*00\r\n");
    EXPECT_EQ(type, NMEAParser::kSentenceChecksumFail);
    // Fix must be unchanged by the rejected sentence.
    EXPECT_EQ(parser.fix().latitude_deg, good_lat);
}

// A GGA reporting no fix (quality 0) must mark the fix invalid.
TEST(NMEAParser, NoFixMarksInvalid) {
    NMEAParser parser;
    auto type = FeedSentence(parser, "$GPGGA,123519,,,,,0,00,,,M,,M,,*6B\r\n");
    EXPECT_EQ(type, NMEAParser::kSentenceGGA);
    EXPECT_FALSE(parser.fix().valid);
}

// Unknown sentence types are reported as such without disturbing the fix.
TEST(NMEAParser, UnknownSentenceType) {
    NMEAParser parser;
    auto type = FeedSentence(parser, "$GPVTG,084.4,T,,M,022.4,N,041.5,K*6C\r\n");
    EXPECT_EQ(type, NMEAParser::kSentenceUnknown);
}

TEST(NMEAParser, UTCDateAndFraction) {
    NMEAParser parser;
    parser.SetTimestampMs(100);
    FeedSentence(parser, "$GPRMC,123519.5379,A,4807.038,N,01131.000,E,022.4,084.4,230394,,,A\r\n");
    uint64_t utc = 0;
    ASSERT_TRUE(parser.GetUTCUnixMs(123, utc));
    EXPECT_EQ(utc, 764426119560ULL);
}

TEST(NMEAParser, UTCMidnightAndFreshnessIndependentOfGGA) {
    NMEAParser parser;
    parser.SetTimestampMs(1000);
    FeedSentence(parser, "$GNRMC,235959.500,A,,,,,,,311226,,,A\r\n");
    uint64_t utc = 0;
    ASSERT_TRUE(parser.GetUTCUnixMs(2000, utc));
    EXPECT_EQ(utc, 1798761600500ULL);
    parser.SetTimestampMs(5999);
    FeedSentence(parser, "$GNGGA,000004,,,,,1,08,,1,M,,,,\r\n");
    ASSERT_TRUE(parser.GetUTCUnixMs(5999, utc));
    EXPECT_EQ(utc, 1798761604499ULL);
    EXPECT_FALSE(parser.GetUTCUnixMs(6000, utc));
    parser.SetTimestampMs(6001);
    FeedSentence(parser, "$GNRMC,000004.501,A,,,,,,,010127,,,A\r\n");
    ASSERT_TRUE(parser.GetUTCUnixMs(6001, utc));
    EXPECT_EQ(utc, 1798761604501ULL);
}

TEST(NMEAParser, UTCMonotonicClockWrap) {
    NMEAParser parser;
    parser.SetTimestampMs(0xFFFFFFF0);
    FeedSentence(parser, "$GPRMC,000000,A,,,,,,,010100,,,A\r\n");
    uint64_t utc = 0;
    ASSERT_TRUE(parser.GetUTCUnixMs(0x100000010ULL, utc));
    EXPECT_EQ(utc, 946684800032ULL);
    EXPECT_FALSE(parser.GetUTCUnixMs(0x200000010ULL, utc));
}

TEST(NMEAParser, UTCGregorianDateAndYearWindow) {
    struct Sample { const char* date; uint64_t epoch_ms; };
    for (const auto& sample : {Sample{"290200", 951782400000ULL}, Sample{"010180", 315532800000ULL},
                               Sample{"311279", 3471206400000ULL}, Sample{"290224", 1709164800000ULL}}) {
        NMEAParser parser;
        std::string rmc = std::string("$GPRMC,000000,A,,,,,,,") + sample.date + ",,,A\r\n";
        FeedSentence(parser, rmc.c_str());
        uint64_t utc = 0;
        ASSERT_TRUE(parser.GetUTCUnixMs(0, utc)) << sample.date;
        EXPECT_EQ(utc, sample.epoch_ms) << sample.date;
    }
}

TEST(NMEAParser, UTCRejectsMalformedOrUntrustedSamples) {
    const char* bad_fields[] = {
        "000000,A,,,,,,,290223", "000000,A,,,,,,,310426", "000000,A,,,,,,,001026",
        "000000,A,,,,,,,011326", "000000,A,,,,,,,01102x", "000000,A,,,,,,,",
        "240000,A,,,,,,,111026", "126000,A,,,,,,,111026", "120061,A,,,,,,,111026",
        "120000x,A,,,,,,,111026", "120000.,A,,,,,,,111026", "120000.123x,A,,,,,,,111026",
        ",A,,,,,,,111026", ",V,,,,,,,111026", "120000,V,,,,,,,", "120000,X,,,,,,,111026",
        "120000,,,,,,,,111026", "120000,AX,,,,,,,111026",
        "235960,A,,,,,,,311216", "120000,A"
    };
    for (const char* fields : bad_fields) {
        NMEAParser parser;
        FeedSentence(parser, "$GPRMC,120000,A,,,,,,,111026,,,A\r\n");
        uint64_t utc = 123;
        ASSERT_TRUE(parser.GetUTCUnixMs(0, utc));
        FeedSentence(parser, (std::string("$GPRMC,") + fields + "\r\n").c_str());
        EXPECT_FALSE(parser.GetUTCUnixMs(0, utc)) << fields;
    }
}

// u-blox fills RMC time and date fields only when it considers them valid, also while status is V (no fix).
TEST(NMEAParser, UTCFromVoidRMCWithoutPosition) {
    NMEAParser parser;
    FeedSentence(parser, "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,,,A\r\n");
    FeedSentence(parser, "$GPRMC,123520,V,5000.000,N,01000.000,E,,,230394,,,N\r\n");
    uint64_t utc = 0;
    ASSERT_TRUE(parser.GetUTCUnixMs(0, utc));
    EXPECT_EQ(utc, 764426120000ULL);
    EXPECT_NEAR(parser.fix().latitude_deg, 48.1173f, 0.001f);
    EXPECT_NEAR(parser.fix().longitude_deg, 11.5167f, 0.001f);
}

TEST(NMEAParser, UTCNeedsDatedRMCAndRejectsBadChecksum) {
    NMEAParser parser;
    uint64_t utc = 123;
    FeedSentence(parser, "$GNGGA,120000,,,,,1,08,,1,M,,,,\r\n");
    EXPECT_FALSE(parser.GetUTCUnixMs(0, utc));
    EXPECT_EQ(utc, 123u);
    FeedSentence(parser, "$GPRMC,120000,A,,,,,,,111026,,,A*00\r\n");
    EXPECT_FALSE(parser.GetUTCUnixMs(0, utc));
}

TEST(NMEAParser, UTCInvalidationRequiresNewRMC) {
    NMEAParser parser;
    FeedSentence(parser, "$GPRMC,120000,A,,,,,,,111026,,,A\r\n");
    uint64_t utc;
    ASSERT_TRUE(parser.GetUTCUnixMs(0, utc));
    parser.InvalidateUTC();
    EXPECT_FALSE(parser.GetUTCUnixMs(0, utc));
    FeedSentence(parser, "$GNGGA,120000,,,,,1,08,,1,M,,,,\r\n");
    EXPECT_FALSE(parser.GetUTCUnixMs(0, utc));
    FeedSentence(parser, "$GPRMC,120001,A,,,,,,,111026,,,A\r\n");
    EXPECT_TRUE(parser.GetUTCUnixMs(0, utc));
}
