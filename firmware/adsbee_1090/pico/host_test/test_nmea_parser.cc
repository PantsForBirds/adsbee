#include <cstdio>
#include <cstring>

#include "awb_test_utils.hh"
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
    EXPECT_NEAR(AWBToDeg(fix.latitude_awb), 48.1173f, 0.001f);
    EXPECT_NEAR(AWBToDeg(fix.longitude_awb), 11.5167f, 0.001f);
    EXPECT_NEAR(fix.altitude_ft, static_cast<int32_t>(545.4f * 3.28084f), 2);
    EXPECT_EQ(fix.last_update_timestamp_ms, 1000u);
    EXPECT_TRUE(fix.utc_time_valid);
    EXPECT_EQ(fix.utc_hour, 12);
    EXPECT_EQ(fix.utc_minute, 35);
    EXPECT_EQ(fix.utc_second, 19);
    EXPECT_EQ(fix.utc_millisecond, 0);
}

TEST(NMEAParser, ParsesGGAGeoidSeparation) {
    NMEAParser parser;
    FeedSentence(parser, "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n");
    EXPECT_TRUE(parser.fix().geoid_separation_valid);
    EXPECT_EQ(parser.fix().geoid_separation_cm, 4690);

    // Negative separation (geoid below the ellipsoid, as over the US), sentence ending after the units field.
    FeedSentence(parser, "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,10.0,M,-32.5,M*54\r\n");
    EXPECT_TRUE(parser.fix().geoid_separation_valid);
    EXPECT_EQ(parser.fix().geoid_separation_cm, -3250);

    // A receiver without a geoid model leaves the field empty.
    EXPECT_EQ(FeedSentence(parser, "$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,,M,,*4C\r\n"),
              NMEAParser::kSentenceGGA);
    EXPECT_FALSE(parser.fix().geoid_separation_valid);
    EXPECT_TRUE(parser.fix().valid);
}

// Decimal fields parse to hundredths with integer math, rounded half away from zero.
TEST(NMEAParser, ParsesGeoidSeparationFixedPoint) {
    struct {
        const char* field;
        bool valid;
        int32_t cm;
    } cases[] = {{"46.9", true, 4690},    {"-32.567", true, -3257}, {"1.234", true, 123}, {"1.235", true, 124},
                 {"-1.235", true, -124},  {"+5", true, 500},        {"-.5", true, -50},   {"0", true, 0},
                 {"12.3456789", true, 1235}, {"abc", false, 0},     {"1.2x", false, 0},   {"-", false, 0}};
    for (const auto& c : cases) {
        NMEAParser parser;
        char sentence[128];
        // No checksum: the parser accepts sentences without one.
        snprintf(sentence, sizeof(sentence), "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,%s,M,,\r\n",
                 c.field);
        EXPECT_EQ(FeedSentence(parser, sentence), NMEAParser::kSentenceGGA) << c.field;
        EXPECT_EQ(parser.fix().geoid_separation_valid, c.valid) << c.field;
        if (c.valid) {
            EXPECT_EQ(parser.fix().geoid_separation_cm, c.cm) << c.field;
        }
    }
}

TEST(NMEAParser, ParsesGSTAltitudeError) {
    NMEAParser parser;
    parser.SetTimestampMs(2000);
    EXPECT_EQ(FeedSentence(parser, "$GNGST,123519.00,7.2,4.1,2.9,79.4,3.1,2.4,5.0*74\r\n"), NMEAParser::kSentenceGST);
    EXPECT_TRUE(parser.fix().altitude_std_dev_valid);
    EXPECT_EQ(parser.fix().altitude_std_dev_cm, 500);
    EXPECT_EQ(parser.fix().altitude_std_dev_timestamp_ms, 2000u);
    EXPECT_FALSE(parser.fix().valid);  // GST carries no position.

    // Without a fix the error fields are empty.
    parser.SetTimestampMs(3000);
    EXPECT_EQ(FeedSentence(parser, "$GNGST,123519.00,,,,,,,*6A\r\n"), NMEAParser::kSentenceGST);
    EXPECT_FALSE(parser.fix().altitude_std_dev_valid);
    EXPECT_EQ(parser.fix().altitude_std_dev_timestamp_ms, 2000u);
}

TEST(NMEAParser, ParsesGPRMC) {
    NMEAParser parser;
    auto type = FeedSentence(
        parser, "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n");
    EXPECT_EQ(type, NMEAParser::kSentenceRMC);

    const NMEAParser::GNSSFix& fix = parser.fix();
    EXPECT_TRUE(fix.valid);
    EXPECT_NEAR(AWBToDeg(fix.latitude_awb), 48.1173f, 0.001f);
    EXPECT_NEAR(AWBToDeg(fix.longitude_awb), 11.5167f, 0.001f);
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
    EXPECT_NEAR(AWBToDeg(parser.fix().latitude_awb), 48.1173f, 0.001f);
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
    uint32_t good_lat = parser.fix().latitude_awb;

    // Now feed a sentence with a deliberately wrong checksum (*00); it must be rejected.
    auto type = FeedSentence(
        parser, "$GPGGA,123519,0000.000,S,00000.000,W,1,08,0.9,1.0,M,46.9,M,,*00\r\n");
    EXPECT_EQ(type, NMEAParser::kSentenceChecksumFail);
    // Fix must be unchanged by the rejected sentence.
    EXPECT_EQ(parser.fix().latitude_awb, good_lat);
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

TEST(NMEAParser, RMCUnixSeconds) {
    NMEAParser parser;
    uint64_t unix_seconds = 0;
    EXPECT_FALSE(parser.GetRMCUnixSeconds(unix_seconds));  // No RMC yet.
    EXPECT_EQ(FeedSentence(parser, "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n"),
              NMEAParser::kSentenceRMC);
    ASSERT_TRUE(parser.GetRMCUnixSeconds(unix_seconds));
    EXPECT_EQ(unix_seconds, 764426119u);  // 1994-03-23 12:35:19 UTC.

    // Year end, and the next second in the next year.
    FeedSentence(parser, "$GNRMC,235959.00,A,4807.038,N,01131.000,E,0.0,,311226,,,A*68\r\n");
    ASSERT_TRUE(parser.GetRMCUnixSeconds(unix_seconds));
    EXPECT_EQ(unix_seconds, 1798761599u);
    FeedSentence(parser, "$GNRMC,000000.00,A,4807.038,N,01131.000,E,0.0,,010127,,,A*69\r\n");
    ASSERT_TRUE(parser.GetRMCUnixSeconds(unix_seconds));
    EXPECT_EQ(unix_seconds, 1798761600u);
    EXPECT_FALSE(parser.RMCIsLeapSecond());

    // February 29 in a leap year; years 00-79 are 2000-2079.
    FeedSentence(parser, "$GNRMC,120000.00,A,4807.038,N,01131.000,E,0.0,,290228,,,A*6C\r\n");
    ASSERT_TRUE(parser.GetRMCUnixSeconds(unix_seconds));
    EXPECT_EQ(unix_seconds, 1835438400u);
    FeedSentence(parser, "$GNRMC,120000.00,A,4807.038,N,01131.000,E,0.0,,010179,,,A*61\r\n");
    ASSERT_TRUE(parser.GetRMCUnixSeconds(unix_seconds));
    EXPECT_EQ(unix_seconds, 3439800000u);
}

TEST(NMEAParser, RMCUnixSecondsNeedsFixDateAndWholeSecond) {
    NMEAParser parser;
    uint64_t unix_seconds = 0;
    // No fix (status V), even with a time and date.
    EXPECT_EQ(FeedSentence(parser, "$GNRMC,120000.00,V,,,,,,,231126,,,N*65\r\n"), NMEAParser::kSentenceRMC);
    EXPECT_FALSE(parser.GetRMCUnixSeconds(unix_seconds));
    // Not a whole second: it can't label a PPS edge.
    FeedSentence(parser, "$GNRMC,120000.50,A,4807.038,N,01131.000,E,0.0,,231126,,,A*6F\r\n");
    EXPECT_FALSE(parser.GetRMCUnixSeconds(unix_seconds));
    // February 29 in a year that has none, and no date at all.
    FeedSentence(parser, "$GNRMC,120000.00,A,4807.038,N,01131.000,E,0.0,,290226,,,A*62\r\n");
    EXPECT_FALSE(parser.GetRMCUnixSeconds(unix_seconds));
    FeedSentence(parser, "$GNRMC,120000.00,A,4807.038,N,01131.000,E,0.0,,,,,A*6F\r\n");
    EXPECT_FALSE(parser.GetRMCUnixSeconds(unix_seconds));
    // A leap second has no POSIX time.
    FeedSentence(parser, "$GNRMC,235960.00,A,4807.038,N,01131.000,E,0.0,,311226,,,A*62\r\n");
    EXPECT_FALSE(parser.GetRMCUnixSeconds(unix_seconds));
    EXPECT_TRUE(parser.RMCIsLeapSecond());
}
