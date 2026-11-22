#include "gtest/gtest.h"
#include "time_ping.hh"

TEST(TimePing, PreservesLeadingZerosAndEmptyEpoch) {
    char reply[TimePing::kReplySize];
    ASSERT_TRUE(TimePing{"007"}.Format(reply));
    EXPECT_STREQ("TIME_PING=007,,0\r\n", reply);
}

TEST(TimePing, FitsMaximumReply) {
    TimePing ping{"00000000000000000001", true, UINT64_MAX, UINT32_MAX};
    char reply[TimePing::kReplySize];
    ASSERT_TRUE(ping.Format(reply));
    EXPECT_STREQ("TIME_PING=00000000000000000001,18446744073709551615,4294967295\r\n", reply);
}

TEST(TimePing, RejectsInvalidEcho) {
    char reply[TimePing::kReplySize];
    for (const char* echo : {"", "-5", "1,2", "000000000000000000001"}) {
        EXPECT_FALSE(TimePing{echo}.Format(reply)) << echo;
    }
}
