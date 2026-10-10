#include "../application/r1090_rssi.hh"
#include "gtest/gtest.h"

using namespace r1090_rssi;

static constexpr uint8_t kV1 = 1;
static constexpr uint8_t kV2 = 2;
static constexpr uint8_t kV3 = 3;

TEST(R1090RSSI, LegacyConversionUnchangedOnV1AndV2) {
    // 60 dB/V around 1600 mV, minus 44 dB of LNA gain, on the raw sample.
    for (uint8_t version : {kV1, kV2}) {
        EXPECT_EQ(DetectorMilliVoltsTodBm(1600, version), -44);
        EXPECT_EQ(DetectorMilliVoltsTodBm(728, version), -96);
        EXPECT_EQ(DetectorMilliVoltsTodBm(1000, version), -80);
        EXPECT_EQ(PacketMilliVoltsTodBm(1000, 728, version), -80);
        EXPECT_EQ(PacketMilliVoltsTodBm(1000, 990, version), -80);  // The noise floor is not used.
        EXPECT_FALSE(PacketSampleAtEnd(version));
    }
}

TEST(R1090RSSI, V3StaticLevelsFollowTheCWCalibration) {
    // Bench CW readings from the 1090U rev G unit: {mV, dBm commanded}.
    const int cw[][2] = {{1021, -88}, {1076, -84}, {1137, -80}, {1204, -76}, {1271, -72}, {1341, -68}, {1412, -64},
                         {1489, -60}, {1564, -56}, {1642, -52}, {1680, -50}, {1715, -48}, {1740, -46}, {1759, -44}};
    for (const auto& point : cw) {
        EXPECT_NEAR(DetectorMilliVoltsTodBm(point[0], kV3), point[1], 1) << point[0] << " mV";
    }
    EXPECT_EQ(DetectorMilliVoltsTodBm(948, kV3), -91);  // Idle noise floor of that unit.
}

TEST(R1090RSSI, V3SaturatesAtDetectorFullScale) {
    EXPECT_EQ(DetectorMilliVoltsTodBm(1770, kV3), kV3MaxdBm);
    EXPECT_EQ(DetectorMilliVoltsTodBm(1897, kV3), kV3MaxdBm);
    EXPECT_EQ(DetectorMilliVoltsTodBm(3300, kV3), kV3MaxdBm);
    EXPECT_EQ(PacketMilliVoltsTodBm(1500, 948, kV3), kV3MaxdBm);
}

TEST(R1090RSSI, V3IsMonotonic) {
    int last_dbm = DetectorMilliVoltsTodBm(600, kV3);
    for (int mv = 601; mv <= 2000; mv++) {
        int dbm = DetectorMilliVoltsTodBm(mv, kV3);
        EXPECT_GE(dbm, last_dbm) << mv << " mV";
        EXPECT_LE(dbm - last_dbm, 1) << mv << " mV";
        last_dbm = dbm;
    }
}

TEST(R1090RSSI, V3PacketsFollowTheBenchCalibration) {
    // End-of-packet samples from the bench: {sample mV, noise floor mV, dBm commanded}. The floor above 948 mV is
    // the signal generator's carrier leak at the higher levels.
    const int packets[][3] = {{1001, 948, -82},  {1028, 948, -78}, {1054, 948, -74}, {1082, 948, -70},
                              {1111, 948, -66},  {1143, 949, -62}, {1176, 951, -58}, {1208, 951, -54},
                              {1272, 1016, -50}, {1328, 1068, -46}};
    for (const auto& packet : packets) {
        EXPECT_NEAR(PacketMilliVoltsTodBm(packet[0], packet[1], kV3), packet[2], 2) << packet[2] << " dBm";
    }
    EXPECT_TRUE(PacketSampleAtEnd(kV3));
}

TEST(R1090RSSI, V3PacketAtOrBelowTheFloorReadsTheFloor) {
    EXPECT_EQ(PacketMilliVoltsTodBm(948, 948, kV3), DetectorMilliVoltsTodBm(948, kV3));
    EXPECT_EQ(PacketMilliVoltsTodBm(900, 948, kV3), DetectorMilliVoltsTodBm(948, kV3));
}
