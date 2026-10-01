#include <cstring>  // For memcpy.

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

TEST(MAVLinkUtils, HeartbeatFields) {
    mavlink_heartbeat_t hb = MAVLINKHeartbeatMessage();
    EXPECT_EQ(hb.type, MAV_TYPE_ADSB);
    EXPECT_EQ(hb.autopilot, MAV_AUTOPILOT_INVALID);
    EXPECT_EQ(hb.base_mode, 0);
    EXPECT_EQ(hb.custom_mode, 0u);
    EXPECT_EQ(hb.system_status, MAV_STATE_ACTIVE);
    // Fixed at 3 by the MAVLink spec for both framings; 1 or 2 here is a malformed HEARTBEAT.
    EXPECT_EQ(hb.mavlink_version, 3);
}

// Frames the heartbeat struct the way mavlink_msg_heartbeat_send_struct does on the device (the struct bytes are the
// payload) and returns the wire bytes.
static uint16_t FrameHeartbeat(bool mavlink1, uint8_t* buf) {
    mavlink_heartbeat_t hb = MAVLINKHeartbeatMessage();
    mavlink_message_t msg = {};
    msg.msgid = MAVLINK_MSG_ID_HEARTBEAT;
    memcpy(_MAV_PAYLOAD_NON_CONST(&msg), &hb, MAVLINK_MSG_ID_HEARTBEAT_LEN);
    mavlink_status_t status = {};
    if (mavlink1) {
        status.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    }
    mavlink_finalize_message_buffer(&msg, 1, 156, &status, MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN,
                                    MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
    return mavlink_msg_to_send_buffer(buf, &msg);
}

// Expected frames are from pymavlink: MAVLink_heartbeat_message(27, 8, 0, 0, 4, 3) packed with srcSystem=1,
// srcComponent=156, seq=0.
TEST(MAVLinkUtils, HeartbeatFrameMAVLink2) {
    const uint8_t expected[] = {0xFD, 0x09, 0x00, 0x00, 0x00, 0x01, 0x9C, 0x00, 0x00, 0x00, 0x00,
                                0x00, 0x00, 0x00, 0x1B, 0x08, 0x00, 0x04, 0x03, 0x5C, 0xC5};
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    ASSERT_EQ(FrameHeartbeat(false, buf), sizeof(expected));
    EXPECT_EQ(memcmp(buf, expected, sizeof(expected)), 0);
}

TEST(MAVLinkUtils, HeartbeatFrameMAVLink1) {
    const uint8_t expected[] = {0xFE, 0x09, 0x00, 0x01, 0x9C, 0x00, 0x00, 0x00, 0x00,
                                0x00, 0x1B, 0x08, 0x00, 0x04, 0x03, 0x28, 0x05};
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    ASSERT_EQ(FrameHeartbeat(true, buf), sizeof(expected));
    EXPECT_EQ(memcmp(buf, expected, sizeof(expected)), 0);
}
