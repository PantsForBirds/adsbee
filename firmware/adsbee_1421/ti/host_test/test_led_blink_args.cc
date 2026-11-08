// AT+LED_BLINK argument parsing: same names and limits as the ADSBee 1090, minus its NETWORK LED.
#include <string_view>

#include "comms/led_blink_args.hh"
#include "gtest/gtest.h"

using namespace LEDBlinkArgs;

static Result Parse(std::initializer_list<std::string_view> names, Selection& selection, uint16_t& bad_index) {
    return ParseLEDs(names.begin(), names.size(), selection, bad_index);
}

TEST(LEDBlinkArgs, DurationIs1To60000Ms) {
    EXPECT_FALSE(DurationIsValid(0));
    EXPECT_TRUE(DurationIsValid(1));
    EXPECT_TRUE(DurationIsValid(60000));
    EXPECT_FALSE(DurationIsValid(60001));
    EXPECT_FALSE(DurationIsValid(UINT32_MAX));
}

TEST(LEDBlinkArgs, SingleLEDs) {
    Selection s;
    uint16_t bad = 0;
    ASSERT_EQ(Parse({"1090"}, s, bad), kOK);
    EXPECT_TRUE(s.led_1090);
    EXPECT_FALSE(s.led_subg);
    ASSERT_EQ(Parse({"SUBG"}, s, bad), kOK);
    EXPECT_FALSE(s.led_1090);
    EXPECT_TRUE(s.led_subg);
}

TEST(LEDBlinkArgs, AllAndCombinations) {
    Selection s;
    uint16_t bad = 0;
    ASSERT_EQ(Parse({"ALL"}, s, bad), kOK);
    EXPECT_TRUE(s.led_1090 && s.led_subg);
    ASSERT_EQ(Parse({"1090", "SUBG"}, s, bad), kOK);
    EXPECT_TRUE(s.led_1090 && s.led_subg);
    ASSERT_EQ(Parse({"SUBG", "SUBG", "1090"}, s, bad), kOK);  // Repeats are fine, as on the 1090.
    EXPECT_TRUE(s.led_1090 && s.led_subg);
}

TEST(LEDBlinkArgs, NetworkIsAClearError) {
    Selection s;
    uint16_t bad = 0;
    EXPECT_EQ(Parse({"NETWORK"}, s, bad), kNoNetworkLED);
    EXPECT_EQ(bad, 0);
    EXPECT_EQ(Parse({"1090", "NETWORK"}, s, bad), kNoNetworkLED);
    EXPECT_EQ(bad, 1);
    EXPECT_EQ(Parse({"ALL", "NETWORK"}, s, bad), kNoNetworkLED);  // ALL doesn't make NETWORK valid.
}

TEST(LEDBlinkArgs, UnknownNamesAreRejectedBeforeAnythingBlinks) {
    Selection s;
    uint16_t bad = 0;
    EXPECT_EQ(Parse({"1090", "BOGUS"}, s, bad), kInvalidLED);
    EXPECT_EQ(bad, 1);
    EXPECT_EQ(Parse({"subg"}, s, bad), kInvalidLED);  // Case sensitive, like the 1090.
    EXPECT_EQ(Parse({""}, s, bad), kInvalidLED);
    EXPECT_EQ(Parse({"1090 "}, s, bad), kInvalidLED);
}
