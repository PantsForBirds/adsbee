#include "aircraft_dictionary.hh"
#include "gtest/gtest.h"
#include "mavlink_utils.hh"

// MAVLink ADSB_VEHICLE.squawk carries the Mode A code as the decimal value of its four octal digits (squawk 7700 is
// sent as 7700 = 0x1E14), and ADSB_FLAGS_VALID_SQUAWK tells the receiver whether the field is meaningful.

TEST(MAVLinkUtils, ModeSSquawkValid) {
    ModeSAircraft aircraft;
    aircraft.squawk = 7700;
    mavlink_adsb_vehicle_t msg = ModeSAircraftToMAVLINKADSBVehicleMessage(aircraft);
    EXPECT_EQ(msg.squawk, 7700);
    EXPECT_TRUE(msg.flags & ADSB_FLAGS_VALID_SQUAWK);

    // Leading zeros are not significant in the numeric field: 0356 -> 356.
    aircraft.squawk = 356;
    msg = ModeSAircraftToMAVLINKADSBVehicleMessage(aircraft);
    EXPECT_EQ(msg.squawk, 356);
    EXPECT_TRUE(msg.flags & ADSB_FLAGS_VALID_SQUAWK);
}

TEST(MAVLinkUtils, ModeSSquawkZeroIsValid) {
    ModeSAircraft aircraft;
    aircraft.squawk = 0;  // 0000 is a real (if unusual) Mode A code.
    mavlink_adsb_vehicle_t msg = ModeSAircraftToMAVLINKADSBVehicleMessage(aircraft);
    EXPECT_EQ(msg.squawk, 0);
    EXPECT_TRUE(msg.flags & ADSB_FLAGS_VALID_SQUAWK);
}

TEST(MAVLinkUtils, ModeSSquawkNotYetReceived) {
    ModeSAircraft aircraft;
    ASSERT_EQ(aircraft.squawk, ADSBTypes::kSquawkCodeNotYetReceived);  // Default-constructed.
    mavlink_adsb_vehicle_t msg = ModeSAircraftToMAVLINKADSBVehicleMessage(aircraft);
    EXPECT_EQ(msg.squawk, 0);  // The 0xFFFF sentinel must never leak onto the wire.
    EXPECT_FALSE(msg.flags & ADSB_FLAGS_VALID_SQUAWK);
}

TEST(MAVLinkUtils, UATSquawk) {
    UATAircraft aircraft;
    aircraft.squawk = 7700;
    mavlink_adsb_vehicle_t msg = UATAircraftToMAVLINKADSBVehicleMessage(aircraft);
    EXPECT_EQ(msg.squawk, 7700);
    EXPECT_TRUE(msg.flags & ADSB_FLAGS_VALID_SQUAWK);
    EXPECT_TRUE(msg.flags & ADSB_FLAGS_SOURCE_UAT);

    aircraft.squawk = 0;
    msg = UATAircraftToMAVLINKADSBVehicleMessage(aircraft);
    EXPECT_EQ(msg.squawk, 0);
    EXPECT_TRUE(msg.flags & ADSB_FLAGS_VALID_SQUAWK);

    aircraft.squawk = ADSBTypes::kSquawkCodeNotYetReceived;
    msg = UATAircraftToMAVLINKADSBVehicleMessage(aircraft);
    EXPECT_EQ(msg.squawk, 0);
    EXPECT_FALSE(msg.flags & ADSB_FLAGS_VALID_SQUAWK);
}
