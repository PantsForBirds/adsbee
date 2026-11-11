#include "comms/ble_output_status.hh"
#include "gtest/gtest.h"

TEST(BLEOutputStatus, WithPSRAMItRunsNextToWiFiAndRemoteID) {
    EXPECT_EQ(BLEOutputAllowed(true, true, true, true), kBLEOutputStatusAdvertising);
}

TEST(BLEOutputStatus, WithoutPSRAMWiFiAndRemoteIDOverBluetoothBlockIt) {
    EXPECT_EQ(BLEOutputAllowed(false, false, false, false), kBLEOutputStatusAdvertising);
    EXPECT_EQ(BLEOutputAllowed(false, true, false, false), kBLEOutputStatusBlockedByWiFi);
    EXPECT_EQ(BLEOutputAllowed(false, false, true, true), kBLEOutputStatusBlockedByWiFi);
    EXPECT_EQ(BLEOutputAllowed(false, false, false, true), kBLEOutputStatusBlockedByRemoteID);
}

TEST(BLEOutputStatus, EveryStatusHasAName) {
    for (uint8_t i = 0; i < kBLEOutputStatusNumValues; i++) {
        EXPECT_STRNE(BLEOutputStatusStr(i), "UNKNOWN");
    }
    EXPECT_STREQ(BLEOutputStatusStr(kBLEOutputStatusBlockedByRemoteID), "BLOCKED_BY_REMOTE_ID");
    EXPECT_STREQ(BLEOutputStatusStr(kBLEOutputStatusNumValues), "UNKNOWN");
}
