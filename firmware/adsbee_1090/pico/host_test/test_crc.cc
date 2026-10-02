#include <random>

#include "buffer_utils.hh"
#include "crc.hh"
#include "gtest/gtest.h"
#include "mode_s_packet.hh"
#include "unit_conversions.hh"

TEST(CRC, CRC24ModeSMessage) {
    // Message does not include CRC24 (last 3 Bytes are checksum, full message would be 112 bits = 14 Bytes).
    uint8_t message[14] = {0x8D, 0x76, 0xCE, 0x88, 0x20, 0x4C, 0x90, 0x72, 0xCB, 0x48, 0x20, 0x9a, 0x50, 0x4D};
    EXPECT_EQ(crc24(message, 11), 0x9A504Du);
    EXPECT_EQ(crc24_syndrome(message, 14), 0x0u);
}

TEST(CRC, CRC24SingleBitError112) {
    uint8_t message[14] = {0x8D, 0x76, 0xCE, 0x88, 0x20, 0x4C, 0x90, 0x72, 0xCB, 0x48, 0x20, 0x9a, 0x50, 0x4D};

    // Flip the MSB of the last byte.
    message[10] ^= (1 << 7);
    uint32_t crc = crc24(message, 11);
    EXPECT_NE(crc, 0x9A504Du);
    uint32_t syndrome = crc24_syndrome(message, 14);
    EXPECT_NE(syndrome, 0x0u);
    EXPECT_EQ(crc24_find_single_bit_error(syndrome, 112), 80);
    // Try fixing the message.
    flip_bit(message, 80);
    EXPECT_EQ(crc24_syndrome(message, 14), 0x0u);
    // Restore the message.
    message[10] = 0x20;

    // Flip a bit in the middle of the message.
    message[5] ^= (1 << 4);
    crc = crc24(message, 11);
    EXPECT_NE(crc, 0x9A504Du);
    syndrome = crc24_syndrome(message, 14);
    EXPECT_NE(syndrome, 0x0u);
    EXPECT_EQ(crc24_find_single_bit_error(syndrome, 112), 43);
    // Try fixing the message.
    flip_bit(message, 43);
    EXPECT_EQ(crc24_syndrome(message, 14), 0x0u);
    // Restore the message.
    message[5] = 0x4C;

    // Flip a bit at the beginning of the message.
    message[0] ^= (1 << 7);
    crc = crc24(message, 11);
    EXPECT_NE(crc, 0x9A504Du);
    syndrome = crc24_syndrome(message, 14);
    EXPECT_NE(syndrome, 0x0u);
    EXPECT_EQ(crc24_find_single_bit_error(syndrome, 112), 0);
    // Try fixing the message.
    flip_bit(message, 0);
    EXPECT_EQ(crc24_syndrome(message, 14), 0x0u);
    // Restore the message.
    message[0] = 0x8D;

    // Flip a bit within the message CRC section.
    message[13] ^= (1 << 7);
    crc = crc24(message, 11);
    EXPECT_EQ(crc, 0x9A504Du);  // CRC is not affected.
    syndrome = crc24_syndrome(message, 14);
    EXPECT_NE(syndrome, 0x0u);  // Syndrome is affected since the buffer's CRC is busted.
    EXPECT_EQ(crc24_find_single_bit_error(syndrome, 112), 104);
    // Try fixing the message.
    flip_bit(message, 104);
    EXPECT_EQ(crc24_syndrome(message, 14), 0x0u);
}

TEST(CRC, CRC24SingleBitError56) {
    // Valid 56-bit squitter message. Pretend we know the ICAO address, so we can look for bit errors.
    uint8_t message[7] = {0x00, 0x05, 0x03, 0x19, 0xAB, 0x8C, 0x22};
    uint32_t icao = 0x7C7B5Au;
    EXPECT_EQ(crc24_syndrome(message, 7), icao);  // Received CRC is calculated CRC XORed with the ICAO address.

    // Flip a bit in the middle of the message. Assuming we know the ICAO address, attempt recovery.
    message[3] ^= (1 << 4);
    EXPECT_EQ(crc24_find_single_bit_error(crc24_syndrome(message, 7) ^ icao, 56), 27);
    // Attempt fixing the message.
    flip_bit(message, 27);
    EXPECT_EQ(crc24_syndrome(message, 7), icao);
}
// The bitwise word-buffer CRC that DecodedModeSPacket::CalculateCRC24 used before it switched to the table CRC in
// crc.cpp (removed from mode_s_packet.cpp), kept here as the reference. Algorithm from The 1090MHz Riddle (Junzi Sun),
// pg. 91: a conditional convolution of the frame with the 25-bit generator word.
static uint32_t WordBufferCRC24(const uint32_t buffer[RawModeSPacket::kMaxPacketLenWords32], uint16_t packet_len_bits) {
    uint32_t crc_buffer[RawModeSPacket::kMaxPacketLenWords32];
    for (uint16_t i = 0; i < RawModeSPacket::kMaxPacketLenWords32; i++) crc_buffer[i] = buffer[i];
    SetNBitsInWordBuffer(24, 0x0, packet_len_bits - 24, crc_buffer);  // Zero the parity field.
    for (uint16_t i = 0; i < packet_len_bits - 24; i++) {
        uint32_t word = GetNBitsFromWordBuffer(25, i, crc_buffer);
        if (word & (1u << 24)) {
            SetNBitsInWordBuffer(25, word ^ (1u << 24 | kCRC24Generator), i, crc_buffer);
        }
    }
    return GetNBitsFromWordBuffer(24, packet_len_bits - 24, crc_buffer);
}

// CalculateCRC24 (table CRC) is bit for bit the same as the word-buffer CRC it replaced, for random 56 and 112-bit
// frames of every downlink format.
TEST(CRC, CalculateCRC24MatchesWordBufferCRC) {
    std::mt19937 rng(24);
    for (int n = 0; n < 100000; n++) {
        uint32_t words[RawModeSPacket::kMaxPacketLenWords32];
        for (uint16_t i = 0; i < RawModeSPacket::kMaxPacketLenWords32; i++) words[i] = rng();
        const bool extended = n & 1;
        const uint16_t len_bits =
            extended ? RawModeSPacket::kExtendedSquitterPacketLenBits : RawModeSPacket::kSquitterPacketLenBits;
        const uint16_t len_words =
            extended ? RawModeSPacket::kExtendedSquitterPacketNumWords32 : RawModeSPacket::kSquitterPacketNumWords32;
        const DecodedModeSPacket packet(RawModeSPacket(words, len_words));
        ASSERT_EQ(packet.CalculateCRC24(len_bits), WordBufferCRC24(packet.raw.buffer, len_bits))
            << "frame " << n << " length " << len_bits;
    }
}
