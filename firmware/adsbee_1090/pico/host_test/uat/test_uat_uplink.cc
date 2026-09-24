#include <cstring>  // For memcmp / memcpy.

#include "fec.hh"
#include "gtest/gtest.h"
#include "uat_packet.hh"
#include "uat_test_data.h"

TEST(UATDecoderTest, UplinkEncodeDecode) {
    uint8_t decoded_data_frame[RawUATUplinkPacket::kUplinkMessagePayloadNumBytes] = {0};
    HexStringToByteBuffer(decoded_data_frame, "1234567890", 5);
    uint8_t encoded_data_frame[RawUATUplinkPacket::kUplinkMessageNumBytes] = {0};
    uat_rs.EncodeUplinkMessage(encoded_data_frame, decoded_data_frame);
    int16_t sigs_dbm = -10;                // Dummy signal strength.
    int16_t sigq_bits = 0;                 // Dummy signal quality.
    uint64_t mlat_48mhz_64bit_counts = 0;  // Dummy timestamp.

    // Ensure encoded packet with no errors is valid.
    EXPECT_EQ(encoded_data_frame[0], 0x12);
    EXPECT_EQ(encoded_data_frame[6], 0x34);
    EXPECT_EQ(encoded_data_frame[12], 0x56);
    EXPECT_EQ(encoded_data_frame[18], 0x78);
    EXPECT_EQ(encoded_data_frame[24], 0x90);
    DecodedUATUplinkPacket packet(RawUATUplinkPacket(encoded_data_frame, RawUATUplinkPacket::kUplinkMessageNumBytes,
                                                     sigs_dbm, sigq_bits, mlat_48mhz_64bit_counts));
    EXPECT_TRUE(packet.is_valid);

    // Ensure encoded packet with too many errors is invalid.
    const uint16_t kNumBytesToCorrupt = 20;
    for (uint16_t i = 0; i < kNumBytesToCorrupt; i++) {
        // Hop skip around to bypass interleaving.
        encoded_data_frame[i * RawUATUplinkPacket::kUplinkMessageNumBlocks] ^=
            0xFF;  // Invert all bits in byte to simulate error.
    }
    DecodedUATUplinkPacket bad_packet(RawUATUplinkPacket(encoded_data_frame, RawUATUplinkPacket::kUplinkMessageNumBytes,
                                                         sigs_dbm, sigq_bits, mlat_48mhz_64bit_counts));
    EXPECT_FALSE(bad_packet.is_valid);
}

// The decoder must leave raw.encoded_message as the corrected, interleaved codeword (payload AND parity) without a
// separate re-encode pass: downstream consumers (CC1312 -> RP2040 SPI, RAW/BEAST output, GDL90 de-interleave) rely on
// it. Inject errors at interleaved positions covering both data and parity symbols of every block and check that the
// raw message is restored bit-exact.
TEST(UATDecoderTest, UplinkCorrectsRawMessageInPlace) {
    uint8_t payload[RawUATUplinkPacket::kUplinkMessagePayloadNumBytes];
    for (uint16_t i = 0; i < sizeof(payload); i++) {
        payload[i] = static_cast<uint8_t>((i * 37 + 11) & 0xFF);  // Deterministic non-trivial pattern.
    }
    uint8_t clean_encoded[RawUATUplinkPacket::kUplinkMessageNumBytes] = {0};
    ASSERT_TRUE(uat_rs.EncodeUplinkMessage(clean_encoded, payload));

    uint8_t corrupted[RawUATUplinkPacket::kUplinkMessageNumBytes];
    memcpy(corrupted, clean_encoded, sizeof(corrupted));
    // Byte indices within a de-interleaved 92-byte block: 0, 40, 71 are payload; 75, 91 are parity.
    const uint16_t kBlockByteIndices[] = {0, 40, 71, 75, 91};
    uint16_t num_errors = 0;
    for (uint16_t block = 0; block < RawUATUplinkPacket::kUplinkMessageNumBlocks; block++) {
        for (uint16_t byte_index : kBlockByteIndices) {
            corrupted[byte_index * RawUATUplinkPacket::kUplinkMessageNumBlocks + block] ^= 0x5A;
            num_errors++;
        }
    }
    ASSERT_NE(memcmp(corrupted, clean_encoded, sizeof(corrupted)), 0);

    DecodedUATUplinkPacket packet(RawUATUplinkPacket(corrupted, RawUATUplinkPacket::kUplinkMessageNumBytes, -10, 0, 0));
    ASSERT_TRUE(packet.is_valid);
    EXPECT_EQ(packet.raw.sigq_bits, num_errors);
    EXPECT_EQ(memcmp(packet.decoded_payload, payload, sizeof(payload)), 0);
    EXPECT_EQ(memcmp(packet.raw.encoded_message, clean_encoded, sizeof(clean_encoded)), 0);

    // A second decode of the corrected raw message must be a clean pass (0 corrections) and de-interleaving it directly
    // must yield the same payload -- this is what the GDL90 reporter relies on.
    uint8_t redecoded_payload[RawUATUplinkPacket::kUplinkMessagePayloadNumBytes] = {0};
    EXPECT_EQ(uat_rs.DecodeUplinkMessage(redecoded_payload, packet.raw.encoded_message), 0);
    uint8_t deinterleaved_payload[RawUATUplinkPacket::kUplinkMessagePayloadNumBytes] = {0};
    UATReedSolomon::DeInterleaveUplinkMessage(deinterleaved_payload, packet.raw.encoded_message);
    EXPECT_EQ(memcmp(deinterleaved_payload, payload, sizeof(payload)), 0);
}

TEST(UATDecoderTest, UplinkFrames) {
    int count = get_uat_uplink_test_frames_count();
    for (int i = 0; i < count; i++) {
        const uat_uplink_test_frame_t* frame = get_uat_uplink_test_frame(i);
        char message[200] = {"\0"};
        sprintf(message, "Test frame %d: %s, tisb_site_id=%08X", i, frame->test_name, frame->tisb_site_id);
        SCOPED_TRACE(message);

        // Create encoded data frame.
        uint8_t decoded_data_frame[RawUATUplinkPacket::kUplinkMessagePayloadNumBytes] = {0};
        HexStringToByteBuffer(decoded_data_frame, frame->frame_data_hex, frame->frame_length);
        uint8_t encoded_data_frame[RawUATUplinkPacket::kUplinkMessageNumBytes] = {0};
        uat_rs.EncodeUplinkMessage(encoded_data_frame, decoded_data_frame);
        PrintByteBuffer("Encoded uplink message:", encoded_data_frame, RawUATUplinkPacket::kUplinkMessageNumBytes);
        int16_t sigs_dbm = -10;                // Dummy signal strength.
        int16_t sigq_bits = 0;                 // Dummy signal quality.
        uint64_t mlat_48mhz_64bit_counts = 0;  // Dummy timestamp.
        DecodedUATUplinkPacket packet(RawUATUplinkPacket(encoded_data_frame, RawUATUplinkPacket::kUplinkMessageNumBytes,
                                                         sigs_dbm, sigq_bits, mlat_48mhz_64bit_counts));
        EXPECT_TRUE(packet.is_valid);
    }
}