#include "comms/wifi_network_selector.hh"
#include "gtest/gtest.h"

using Scan = WiFiNetworkSelector::ScanResult;

TEST(WiFiNetworkSelector, PicksStrongestStoredNetwork) {
    const char* stored[] = {"home", "hotspot", ""};
    Scan scan[] = {{"neighbor", -30}, {"home", -80}, {"hotspot", -55}, {"home", -60}};
    EXPECT_EQ(WiFiNetworkSelector::Strongest(stored, 3, scan, 4), 1);
    scan[3].rssi_dbm = -40;  // Second access point of "home" is now strongest.
    EXPECT_EQ(WiFiNetworkSelector::Strongest(stored, 3, scan, 4), 0);
}

TEST(WiFiNetworkSelector, TieGoesToLowerIndex) {
    const char* stored[] = {"", "a", "b"};
    Scan scan[] = {{"b", -50}, {"a", -50}};
    EXPECT_EQ(WiFiNetworkSelector::Strongest(stored, 3, scan, 2), 1);
}

TEST(WiFiNetworkSelector, NoneInRange) {
    const char* stored[] = {"home", "", ""};
    Scan scan[] = {{"neighbor", -30}, {"", -40}};  // Hidden networks report an empty SSID.
    EXPECT_EQ(WiFiNetworkSelector::Strongest(stored, 3, scan, 2), WiFiNetworkSelector::kNone);
    EXPECT_EQ(WiFiNetworkSelector::Strongest(stored, 3, scan, 0), WiFiNetworkSelector::kNone);
}

TEST(WiFiNetworkSelector, NextSkipsEmptySlotsAndWraps) {
    const char* stored[] = {"a", "", "c"};
    EXPECT_EQ(WiFiNetworkSelector::Next(stored, 3, WiFiNetworkSelector::kNone), 0);
    EXPECT_EQ(WiFiNetworkSelector::Next(stored, 3, 0), 2);
    EXPECT_EQ(WiFiNetworkSelector::Next(stored, 3, 2), 0);
    const char* single[] = {"", "b", ""};
    EXPECT_EQ(WiFiNetworkSelector::Next(single, 3, 1), 1);
    const char* empty[] = {"", "", ""};
    EXPECT_EQ(WiFiNetworkSelector::Next(empty, 3, 0), WiFiNetworkSelector::kNone);
}

TEST(WiFiNetworkSelector, PickMovesPastAFailedStrongestNetwork) {
    // "office" is strongest but its stored password is wrong; "hotspot" is weaker, "home" is out of range.
    const char* stored[] = {"home", "office", "hotspot"};
    Scan scan[] = {{"office", -40}, {"hotspot", -70}};
    uint32_t failed = 0;
    EXPECT_EQ(WiFiNetworkSelector::Pick(stored, 3, scan, 2, failed, -1), 1);
    failed |= 1 << 1;  // Auth failed on "office".
    EXPECT_EQ(WiFiNetworkSelector::Pick(stored, 3, scan, 2, failed, 1), 2);
    failed |= 1 << 2;  // "hotspot" failed too: try "home" (maybe hidden).
    EXPECT_EQ(WiFiNetworkSelector::Pick(stored, 3, scan, 2, failed, 2), 0);
    failed |= 1 << 0;  // Everything failed: start over with the strongest.
    EXPECT_EQ(WiFiNetworkSelector::Pick(stored, 3, scan, 2, failed, 0), 1);
    EXPECT_EQ(failed, 0u);
}

TEST(WiFiNetworkSelector, PickWithNothingStored) {
    const char* stored[] = {"", "", ""};
    uint32_t failed = 0b101;
    EXPECT_EQ(WiFiNetworkSelector::Pick(stored, 3, nullptr, 0, failed, 0), WiFiNetworkSelector::kNone);
}
