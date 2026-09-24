#include "beast_utils.hh"
#include "gtest/gtest.h"
#include "mode_s_packet.hh"

TEST(BeastUtils, BuildModeSBeastFrame) {
    // Create packet with a single 0x1a that must be escaped in data, a typical RSSI value, and an unmasked MLAT
    // counter. Note: MLAT counter is shifted left by 2 bits to simulate multiplying a 48MHz counter with a 12MHz
    // desired result.
    DecodedModeSPacket tpacket = DecodedModeSPacket((const char*)"8d495066587f469bb826d21ad767",  // string
                                                    0,                                            // source
                                                    -80,                                          // sigs
                                                    50,                                           // sigq
                                                    0xABABFF1AFFFFFF1A << 2                       // mlat
    );

    uint8_t beast_frame_buf[BeastReporter::kModeSBeastFrameMaxLenBytes];
    // 1 (frame) + 6 (mlat) + 2 (mlat escape) + 1 (rssi) + 14 (data) + 1 (data escape) = 25 Bytes.
    EXPECT_EQ(BeastReporter::BuildModeSBeastFrame(beast_frame_buf, tpacket), 26);
    EXPECT_EQ(beast_frame_buf[0], BeastReporter::kBeastEscapeChar);  // Packet begins with escape char.
    EXPECT_EQ(beast_frame_buf[1], 0x33);                             // Packet type is Mode S long frame.
    EXPECT_EQ(beast_frame_buf[2], 0xFF);                             // MLAT Counter Begin
    EXPECT_EQ(beast_frame_buf[3], 0x1A);
    EXPECT_EQ(beast_frame_buf[4], 0x1A);  // escape
    EXPECT_EQ(beast_frame_buf[5], 0xFF);
    EXPECT_EQ(beast_frame_buf[6], 0xFF);
    EXPECT_EQ(beast_frame_buf[7], 0xFF);
    EXPECT_EQ(beast_frame_buf[8], 0x1A);
    EXPECT_EQ(beast_frame_buf[9], 0x1A);   // escape
    EXPECT_EQ(beast_frame_buf[10], 8);     // RSSI is -80dBm.
    EXPECT_EQ(beast_frame_buf[11], 0x8D);  // Mode S Data Begin
    EXPECT_EQ(beast_frame_buf[12], 0x49);
    EXPECT_EQ(beast_frame_buf[13], 0x50);
    EXPECT_EQ(beast_frame_buf[14], 0x66);
    EXPECT_EQ(beast_frame_buf[15], 0x58);
    EXPECT_EQ(beast_frame_buf[16], 0x7f);
    EXPECT_EQ(beast_frame_buf[17], 0x46);
    EXPECT_EQ(beast_frame_buf[18], 0x9B);
    EXPECT_EQ(beast_frame_buf[19], 0xB8);
    EXPECT_EQ(beast_frame_buf[20], 0x26);
    EXPECT_EQ(beast_frame_buf[21], 0xD2);
    EXPECT_EQ(beast_frame_buf[22], 0x1A);
    EXPECT_EQ(beast_frame_buf[23], 0x1A);  // escape
    EXPECT_EQ(beast_frame_buf[24], 0xD7);
    EXPECT_EQ(beast_frame_buf[25], 0x67);
}

TEST(BeastUtils, BuildModeSIngestBeastFrame) {
    // Create packet with a single 0x1a that must be escaped in data, a typical RSSI value, and an unmasked MLAT
    // counter. Note: MLAT counter is shifted left by 2 bits to simulate multiplying a 48MHz counter with a 12MHz
    // desired result.
    DecodedModeSPacket tpacket = DecodedModeSPacket((const char*)"8d495066587f469bb826d21ad767",  // string
                                                    0,                                            // source
                                                    0,                                            // sigs
                                                    50,                                           // sigq
                                                    0xABABFF1AFFFFFF1A << 2);

    uint8_t beast_frame_buf[BeastReporter::kModeSBeastFrameMaxLenBytes];
    // 1 (frame) + 6 (mlat) + 2 (mlat escape) + 1 (rssi) + 14 (data) + 1 (data escape) = 25 Bytes.
    uint8_t uid[BeastReporter::kReceiverIDLenBytes] = {0xde, 0xad, 0xbe, 0x1a, 0xef, 0x1a, 0xaa, 0xbb};
    uint uid_num_chars_to_escape = 2;
    uint uid_escaped_length = BeastReporter::kReceiverIDLenBytes + uid_num_chars_to_escape;

    EXPECT_EQ(BeastReporter::BuildModeSIngestBeastFrame(beast_frame_buf, tpacket, uid), uid_escaped_length + 2 + 26);
    uint bytes_compared = 0;
    EXPECT_EQ(beast_frame_buf[bytes_compared++], BeastReporter::kBeastEscapeChar);
    EXPECT_EQ(beast_frame_buf[bytes_compared++], BeastReporter::kBeastFrameTypeIngestId);
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0xde);  // uid[0]
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0xad);  // uid[1]
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0xbe);  // uid[2]
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0x1a);  // uid[3] escape
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0x1a);  // uid[3]
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0xef);  // uid[4]
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0x1a);  // uid[5] escape
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0x1a);  // uid[5]
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0xaa);  // uid[6]
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0xbb);  // uid[7]
    EXPECT_EQ(beast_frame_buf[bytes_compared++], BeastReporter::kBeastEscapeChar);
    EXPECT_EQ(beast_frame_buf[bytes_compared++],
              BeastReporter::kBeastFrameTypeModeSLong);  // Packet type is Mode S long frame.
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0xFF);  // MLAT Counter Begin
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0x1A);
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0x1A);  // escape
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0xFF);
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0xFF);
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0xFF);
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0x1A);
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0x1A);  // escape
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 255);   // RSSI is 0dBm.
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0x8D);  // Mode S Data Begin
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0x49);
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0x50);
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0x66);
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0x58);
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0x7f);
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0x46);
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0x9B);
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0xB8);
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0x26);
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0xD2);
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0x1A);
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0x1A);  // Escape
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0xD7);
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 0x67);
}

TEST(BeastUtils, BuildFeedStartFrame) {
    uint8_t beast_frame_buf[BeastReporter::kModeSBeastFrameMaxLenBytes];

    uint8_t uid[BeastReporter::kReceiverIDLenBytes] = {0xde, 0xad, 0xbe, 0x1a, 0xef, 0x1a, 0xaa, 0xbb};
    EXPECT_EQ(BeastReporter::BuildFeedStartFrame(beast_frame_buf, uid), 38);
    uint16_t bytes_compared = 0;

    EXPECT_EQ(beast_frame_buf[bytes_compared++], BeastReporter::kBeastEscapeChar);
    EXPECT_EQ(beast_frame_buf[bytes_compared++], BeastReporter::kBeastFrameTypeFeedId);
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'd');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'e');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'a');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'd');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'b');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'e');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], '1');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'a');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], '-');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'e');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'f');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], '1');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'a');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], '-');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'a');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'a');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'b');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'b');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], '-');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'd');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'e');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'a');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'd');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], '-');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'b');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'e');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], '1');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'a');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'e');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'f');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], '1');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'a');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'a');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'a');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'b');
    EXPECT_EQ(beast_frame_buf[bytes_compared++], 'b');
}

struct UATADSBPacketBeastTest {
    RawUATADSBPacket packet;
    char beast_frame_buf[2 * BeastReporter::kUATADSBBeastFrameMaxLenBytes];
};

UATADSBPacketBeastTest uat_adsb_tests[] = {
    {.packet = RawUATADSBPacket("00a66ef135445d525a0c05191190212048006cb82bc4d53a5b2bb0a8ec6e", -60, 0, 0x123456 << 2),
     .beast_frame_buf =
         "1aec730000001234562d"  // Prefix: escape | frame type | uat message type | mlat timestamp | rssi
         "00a66ef135445d525a0c05191190212048006cb82bc4d53a5b2bb0a8ec6e"}};

TEST(BeastUtils, BuildUATADSBFrame) {
    for (const auto& test : uat_adsb_tests) {
        uint8_t beast_frame_buf[BeastReporter::kUATADSBBeastFrameMaxLenBytes];
        uint16_t bytes_written = BeastReporter::BuildUATADSBBeastFrame(beast_frame_buf, test.packet);
        EXPECT_EQ(bytes_written, strlen(test.beast_frame_buf) / 2);
        // Convert beast_frame_buf binary buffer to string buffer.
        char beast_frame_buf_str[2 * BeastReporter::kUATADSBBeastFrameMaxLenBytes + 1];
        ByteBufferToHexString(beast_frame_buf_str, beast_frame_buf, bytes_written);
        EXPECT_STREQ(beast_frame_buf_str, test.beast_frame_buf);
    }
}

struct UATUplinkPacketBeastTest {
    RawUATUplinkPacket packet;
    char beast_frame_buf[2 * BeastReporter::kUATUplinkBeastFrameMaxLenBytes];
};

UATUplinkPacketBeastTest uat_uplink_tests[] = {
    {.packet = RawUATUplinkPacket(
         "352d30000000147058000000c9c70d000000523c3c000000d61f170000005cc7f1000000b60cd7000000b01f0c0000002ec7720000000"
         "08cd3000000001f1e0000003505330000000ef6c10000001d5ffc000000687c7500000022f5c300000010d91c0000000020b400000000"
         "51cb00000000600c000000ff4cf000000000f57f00000051541c00000091227000000054037f0000007c4f1e0000004d053000000050f"
         "77c00000060de17000000cb00d90000004c337d0000007400f5000000d30015000000583549000000330e43000000d724050000005df0"
         "80000000b922d3000000c710c100000077007c000000f300d3000000c700c10000002dff810000003400540000007d5be00000000747c"
         "3000000cdc02c0000007c7c78000000f54dd5000000d950f500000020601500000051cb49000000604c430000004c7805000000f5cf80"
         "0000005408d30000002233c100000003d7b20000004f5e0000000005725a000000fccf78000000756700000000c3f6000000001cc3000"
         "00000b47c00000000c71f000000004d5d0000000035f3000000007f5f000000001d5100000000705400000000c7940000000028c30700"
         "00006495b5000000b5a39a000000d06df300000066d54d00000015e4e2000000fd19ca0000001534a70000007153d3000000ac0ea0000"
         "000c86953000000472843000000779764000000706f3b000000b006f30000009b356d0000000b0ee7000000ffc3ce00000049013f0000"
         "00964ddb000000",
         -60, 0, 0x00431a123456 << 2),
     .beast_frame_buf =
         "1aec7500431a1a1234562d"  // Prefix: escape | frame type | uat message type | mlat timestamp | rssi
         "352d30000000147058000000c9c70d000000523c3c000000d61f170000005cc7f1000000b60cd7000000b01f0c0000002ec7720000000"
         "08cd3000000001f1e0000003505330000000ef6c10000001d5ffc000000687c7500000022f5c300000010d91c0000000020b400000000"
         "51cb00000000600c000000ff4cf000000000f57f00000051541c00000091227000000054037f0000007c4f1e0000004d053000000050f"
         "77c00000060de17000000cb00d90000004c337d0000007400f5000000d30015000000583549000000330e43000000d724050000005df0"
         "80000000b922d3000000c710c100000077007c000000f300d3000000c700c10000002dff810000003400540000007d5be00000000747c"
         "3000000cdc02c0000007c7c78000000f54dd5000000d950f500000020601500000051cb49000000604c430000004c7805000000f5cf80"
         "0000005408d30000002233c100000003d7b20000004f5e0000000005725a000000fccf78000000756700000000c3f6000000001cc3000"
         "00000b47c00000000c71f000000004d5d0000000035f3000000007f5f000000001d5100000000705400000000c7940000000028c30700"
         "00006495b5000000b5a39a000000d06df300000066d54d00000015e4e2000000fd19ca0000001534a70000007153d3000000ac0ea0000"
         "000c86953000000472843000000779764000000706f3b000000b006f30000009b356d0000000b0ee7000000ffc3ce00000049013f0000"
         "00964ddb000000"}};

TEST(BeastUtils, BuildUATUplinkFrame) {
    for (const auto& test : uat_uplink_tests) {
        uint8_t beast_frame_buf[BeastReporter::kUATUplinkBeastFrameMaxLenBytes];
        uint16_t bytes_written = BeastReporter::BuildUATUplinkBeastFrame(beast_frame_buf, test.packet);
        EXPECT_EQ(bytes_written, strlen(test.beast_frame_buf) / 2);
        // Convert beast_frame_buf binary buffer to string buffer.
        char beast_frame_buf_str[2 * BeastReporter::kUATUplinkBeastFrameMaxLenBytes + 1];
        ByteBufferToHexString(beast_frame_buf_str, beast_frame_buf, bytes_written);
        EXPECT_STREQ(beast_frame_buf_str, test.beast_frame_buf);
    }
}