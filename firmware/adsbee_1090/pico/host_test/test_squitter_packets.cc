#include "gtest/gtest.h"
#include "mode_s_packet.hh"

TEST(ModeSAltitudeReplyPacket, JasonPlaynePackets) {
    ModeSAltitudeReplyPacket packet = ModeSAltitudeReplyPacket(DecodedModeSPacket((const char*)"200006A2DE8B1C"));
    EXPECT_FALSE(packet.is_valid);
    packet.is_valid = true;
    EXPECT_TRUE(packet.is_valid);
    EXPECT_EQ(packet.utility_message, 0);
    EXPECT_FALSE(packet.has_alert);
    EXPECT_EQ(packet.altitude_ft, 10000);
    EXPECT_TRUE(packet.is_airborne);
    EXPECT_TRUE(packet.airborne_state_known);
    EXPECT_EQ(packet.icao_address, 0x7C1B28u);

    packet = ModeSAltitudeReplyPacket(DecodedModeSPacket((const char*)"210000992F8C48"));
    EXPECT_FALSE(packet.is_valid);
    packet.is_valid = true;
    EXPECT_TRUE(packet.is_valid);
    EXPECT_EQ(packet.utility_message, 0);
    EXPECT_FALSE(packet.has_alert);
    EXPECT_EQ(packet.altitude_ft, 25);
    EXPECT_FALSE(packet.is_airborne);
    EXPECT_TRUE(packet.airborne_state_known);
    EXPECT_EQ(packet.icao_address, 0x7C7539u);
}

TEST(ModeSIdentityReplyPacket, JasonPlaynePackets) {
    ModeSIdentityReplyPacket packet = ModeSIdentityReplyPacket(DecodedModeSPacket((const char*)"29001B3AF47E76"));
    EXPECT_FALSE(packet.is_valid);
    packet.is_valid = true;
    EXPECT_TRUE(packet.is_valid);
    EXPECT_EQ(packet.utility_message, ModeSIdentityReplyPacket::UtilityMessageType::kUtilityMessageNoInformation);
    EXPECT_FALSE(packet.has_alert);
    EXPECT_EQ(packet.squawk, 03751u);
    EXPECT_FALSE(packet.is_airborne);
    EXPECT_TRUE(packet.airborne_state_known);
    EXPECT_EQ(packet.icao_address, 0x7C1474u);
    EXPECT_FALSE(packet.has_ident);

    packet = ModeSIdentityReplyPacket(DecodedModeSPacket((const char*)"2820050BD0D698"));
    EXPECT_FALSE(packet.is_valid);
    packet.is_valid = true;
    EXPECT_TRUE(packet.is_valid);
    EXPECT_EQ(packet.utility_message, ModeSIdentityReplyPacket::UtilityMessageType::kUtilityMessageNoInformation);
    EXPECT_EQ(packet.downlink_request,
              ModeSIdentityReplyPacket::DownlinkRequest::kDownlinkRequestCommBBroadcastMessage1Available);
    EXPECT_FALSE(packet.has_alert);
    EXPECT_EQ(packet.squawk, 00664u);
    EXPECT_TRUE(packet.is_airborne);
    EXPECT_TRUE(packet.airborne_state_known);
    EXPECT_EQ(packet.icao_address, 0x7C7181u);
    EXPECT_FALSE(packet.has_ident);

    // Edit the previous packet to force an ident (FS=0b101 — airborne state ambiguous).
    packet = ModeSIdentityReplyPacket(DecodedModeSPacket((const char*)"2D20050BD0D698"));
    EXPECT_EQ(packet.utility_message, ModeSIdentityReplyPacket::UtilityMessageType::kUtilityMessageNoInformation);
    EXPECT_EQ(packet.downlink_request,
              ModeSIdentityReplyPacket::DownlinkRequest::kDownlinkRequestCommBBroadcastMessage1Available);
    EXPECT_FALSE(packet.has_alert);
    EXPECT_EQ(packet.squawk, 00664u);
    EXPECT_FALSE(packet.is_airborne);
    EXPECT_FALSE(packet.airborne_state_known);  // FS=0b101: state is ambiguous, do not trust is_airborne.
    EXPECT_TRUE(packet.has_ident);

    // Edit the previous packet to force an ident and alert (FS=0b100 — airborne state ambiguous).
    packet = ModeSIdentityReplyPacket(DecodedModeSPacket((const char*)"2C20050BD0D698"));
    EXPECT_EQ(packet.utility_message, ModeSIdentityReplyPacket::UtilityMessageType::kUtilityMessageNoInformation);
    EXPECT_EQ(packet.downlink_request,
              ModeSIdentityReplyPacket::DownlinkRequest::kDownlinkRequestCommBBroadcastMessage1Available);
    EXPECT_TRUE(packet.has_alert);
    EXPECT_EQ(packet.squawk, 00664u);
    EXPECT_FALSE(packet.is_airborne);
    EXPECT_FALSE(packet.airborne_state_known);  // FS=0b100: state is ambiguous, do not trust is_airborne.
    EXPECT_TRUE(packet.has_ident);
}

TEST(ModeSAllCallReplyPacket, JasonPlaynePackets) {
    ModeSAllCallReplyPacket packet = ModeSAllCallReplyPacket(DecodedModeSPacket((const char*)"5D7C0B6DB05076"));
    EXPECT_TRUE(packet.is_valid);
    EXPECT_EQ(packet.capability, 5);
    EXPECT_EQ(packet.icao_address, 0x7C0B6Du);

    // Flip one bit and watch it fail.
    packet = ModeSAllCallReplyPacket(DecodedModeSPacket((const char*)"5D7C0B6DB05075"));
    EXPECT_FALSE(packet.is_valid);
}
TEST(ModeSAllCallReplyPacket, RejectSyndromeAboveInterrogatorIDBits) {
    // Valid DF=11 (5D7C0B6DB05076) with the parity field XORed with 0x010000, and with 0xAB0000. The syndrome has no
    // bits set in its lower 16 bits, so it must not be mistaken for an interrogator ID of 0.
    DecodedModeSPacket packet = DecodedModeSPacket((const char*)"5D7C0B6DB15076");
    EXPECT_EQ(packet.crc_syndrome, 0x010000u);
    EXPECT_FALSE(packet.is_valid);
    packet = DecodedModeSPacket((const char*)"5D7C0B6D1B5076");
    EXPECT_EQ(packet.crc_syndrome, 0xAB0000u);
    EXPECT_FALSE(packet.is_valid);
}

TEST(ModeSAllCallReplyPacket, NonzeroInterrogatorCodeNeedsAddressConfirmation) {
    // 5D7C0B6DB05076 with its parity overlaid with interrogator code 5 (CL=0, IC=5).
    DecodedModeSPacket packet = DecodedModeSPacket((const char*)"5D7C0B6DB05073");
    EXPECT_FALSE(packet.is_valid);
    EXPECT_TRUE(packet.is_address_parity);
    EXPECT_EQ(packet.icao_address, 0x7C0B6Du);
    EXPECT_EQ(packet.parity_interrogator_id, 5u);

    // Largest possible overlay: CL=4, IC=15.
    packet = DecodedModeSPacket((const char*)"5D7C0B6DB05039");
    EXPECT_FALSE(packet.is_valid);
    EXPECT_TRUE(packet.is_address_parity);

    // CL=5 is not a valid code label, so this is just a corrupted packet.
    packet = DecodedModeSPacket((const char*)"5D7C0B6DB05026");
    EXPECT_FALSE(packet.is_valid);
    EXPECT_FALSE(packet.is_address_parity);
}
