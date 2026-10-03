// Single-bit correction: DF=17/18 only, never the DF field (DO-260B 2.2.4.3.4.7.3.a), and can be turned off.
#include <cstring>
#include <random>

#include "crc.hh"
#include "gtest/gtest.h"
#include "mode_s_packet.hh"
#include "mode_s_packet_decoder.hh"

// Random frame of format df with valid parity (CRC, or CRC XOR icao for address parity formats).
static RawModeSPacket MakeFrame(uint16_t df, uint16_t len_bits, std::mt19937& rng, uint32_t icao = 0) {
    const uint16_t len_bytes = len_bits / 8;
    uint8_t bytes[RawModeSPacket::kExtendedSquitterPacketLenBytes] = {0};
    for (uint16_t i = 0; i < len_bytes - 3; i++) bytes[i] = rng() & 0xFF;
    bytes[0] = static_cast<uint8_t>((df << 3) | (bytes[0] & 0b111));
    const uint32_t parity = crc24(bytes, len_bytes - 3) ^ icao;
    bytes[len_bytes - 3] = parity >> 16;
    bytes[len_bytes - 2] = (parity >> 8) & 0xFF;
    bytes[len_bytes - 1] = parity & 0xFF;
    uint32_t words[RawModeSPacket::kMaxPacketLenWords32] = {0};
    for (uint16_t i = 0; i < len_bytes; i++) words[i / 4] |= uint32_t(bytes[i]) << (24 - 8 * (i % 4));
    return RawModeSPacket(words, len_bits == 112 ? RawModeSPacket::kExtendedSquitterPacketNumWords32
                                                 : RawModeSPacket::kSquitterPacketNumWords32);
}

static RawModeSPacket FlipBit(const RawModeSPacket& raw, uint16_t i) {
    RawModeSPacket flipped = raw;
    flip_bit(flipped.buffer, i);
    return flipped;
}

static bool SameBuffer(const RawModeSPacket& a, const RawModeSPacket& b) {
    return a.buffer_len_bytes == b.buffer_len_bytes && memcmp(a.buffer, b.buffer, sizeof(a.buffer)) == 0;
}

// DF=17/18: every single-bit error outside the DF field is corrected back to the original. DF field errors are not.
TEST(ModeSSingleBitCorrection, CorrectsDF17AndDF18OutsideTheDFField) {
    std::mt19937 rng(17);
    for (uint16_t df : {17, 18}) {
        for (int n = 0; n < 50; n++) {
            const RawModeSPacket frame = MakeFrame(df, 112, rng);
            ASSERT_TRUE(DecodedModeSPacket(frame).is_valid);
            EXPECT_FALSE(DecodedModeSPacket(frame).IsSingleBitCorrectable());  // Valid: nothing to correct.
            for (uint16_t i = 0; i < 112; i++) {
                const RawModeSPacket corrupted = FlipBit(frame, i);
                DecodedModeSPacket packet(corrupted);
                if (i < DecodedModeSPacket::kDFNumBits) {
                    EXPECT_EQ(packet.CorrectSingleBitError(), -1) << "DF " << df << " bit " << i;
                    EXPECT_TRUE(SameBuffer(packet.raw, corrupted)) << "DF " << df << " bit " << i;
                    EXPECT_FALSE(packet.is_valid) << "DF " << df << " bit " << i;
                } else {
                    EXPECT_TRUE(packet.IsSingleBitCorrectable()) << "DF " << df << " bit " << i;
                    EXPECT_EQ(packet.CorrectSingleBitError(), i) << "DF " << df << " bit " << i;
                    EXPECT_TRUE(packet.is_valid) << "DF " << df << " bit " << i;
                    EXPECT_EQ(packet.downlink_format, df);
                    EXPECT_TRUE(SameBuffer(packet.raw, frame)) << "DF " << df << " bit " << i;
                }
            }
        }
    }
}

// Two bit errors are never "corrected" into a different valid packet.
TEST(ModeSSingleBitCorrection, LeavesTwoBitErrorsAlone) {
    std::mt19937 rng(2);
    const RawModeSPacket frame = MakeFrame(17, 112, rng);
    for (uint16_t i = 0; i < 112; i++) {
        for (uint16_t j = i + 1; j < 112; j++) {
            DecodedModeSPacket packet(FlipBit(FlipBit(frame, i), j));
            ASSERT_EQ(packet.CorrectSingleBitError(), -1) << "bits " << i << ", " << j;
        }
    }
}

// Other 112-bit formats (DF=19 military, DF=22, DF=24 Comm-D) are never corrected, even with a single bit error.
TEST(ModeSSingleBitCorrection, LeavesOtherExtendedFormatsAlone) {
    std::mt19937 rng(19);
    for (uint16_t df : {19, 22, 24}) {
        const RawModeSPacket frame = MakeFrame(df, 112, rng);
        ASSERT_TRUE(DecodedModeSPacket(frame).is_valid) << "DF " << df;
        for (uint16_t i = DecodedModeSPacket::kDFNumBits; i < 112; i++) {
            const RawModeSPacket corrupted = FlipBit(frame, i);
            DecodedModeSPacket packet(corrupted);
            EXPECT_FALSE(packet.IsSingleBitCorrectable()) << "DF " << df << " bit " << i;
            EXPECT_EQ(packet.CorrectSingleBitError(), -1) << "DF " << df << " bit " << i;
            EXPECT_TRUE(SameBuffer(packet.raw, corrupted)) << "DF " << df << " bit " << i;
            EXPECT_FALSE(packet.is_valid) << "DF " << df << " bit " << i;
        }
    }
}

// Address parity formats (DF=0/4/5/16/20/21) and DF=11 are never corrected: their parity holds the address.
TEST(ModeSSingleBitCorrection, LeavesAddressParityFormatsAlone) {
    std::mt19937 rng(20);
    const uint32_t icao = 0xA1B2C3;
    for (uint16_t df : {0, 4, 5, 11, 16, 20, 21}) {
        const uint16_t len_bits = df < 16 ? 56 : 112;
        const RawModeSPacket frame = MakeFrame(df, len_bits, rng, df == 11 ? 0 : icao);
        const DecodedModeSPacket decoded(frame);
        if (df != 11) {
            ASSERT_TRUE(decoded.is_address_parity) << "DF " << df;
            ASSERT_EQ(decoded.icao_address, icao) << "DF " << df;
        }
        for (uint16_t i = 0; i < len_bits; i++) {
            const RawModeSPacket corrupted = FlipBit(frame, i);
            DecodedModeSPacket packet(corrupted);
            const bool was_address_parity = packet.is_address_parity;
            const uint32_t icao_before = packet.icao_address;
            // A DF bit error can produce a correctable format (DF=16 -> DF=17), but DF bits are never flipped back.
            if (i >= DecodedModeSPacket::kDFNumBits) {
                EXPECT_FALSE(packet.IsSingleBitCorrectable()) << "DF " << df << " bit " << i;
            }
            EXPECT_EQ(packet.CorrectSingleBitError(), -1) << "DF " << df << " bit " << i;
            EXPECT_TRUE(SameBuffer(packet.raw, corrupted)) << "DF " << df << " bit " << i;
            EXPECT_EQ(packet.is_address_parity, was_address_parity) << "DF " << df << " bit " << i;
            EXPECT_EQ(packet.icao_address, icao_before) << "DF " << df << " bit " << i;
        }
    }
}

// Corrects only with enable_1090_error_correction; address parity packets pass unchanged either way.
TEST(ModeSSingleBitCorrection, DecoderSetting) {
    std::mt19937 rng(1090);
    const RawModeSPacket df17 = MakeFrame(17, 112, rng);
    const RawModeSPacket df20 = MakeFrame(20, 112, rng, 0x123456);
    for (bool enable : {false, true}) {
        ModeSPacketDecoder decoder(ModeSPacketDecoder::PacketDecoderConfig{.enable_1090_error_correction = enable});
        decoder.raw_mode_s_packet_in_queue.Enqueue(FlipBit(df17, 40));
        decoder.raw_mode_s_packet_in_queue.Enqueue(FlipBit(df20, 40));
        decoder.UpdateDecoderLoop();

        DecodedModeSPacket out;
        if (enable) {
            ASSERT_TRUE(decoder.decoded_mode_s_packet_out_queue.Dequeue(out));
            EXPECT_TRUE(out.is_valid);
            EXPECT_TRUE(SameBuffer(out.raw, df17));
            uint16_t bit_flip_index = 0;
            ASSERT_TRUE(decoder.decoded_mode_s_packet_bit_flip_locations_out_queue.Dequeue(bit_flip_index));
            EXPECT_EQ(bit_flip_index, 40);
        }
        ASSERT_TRUE(decoder.decoded_mode_s_packet_out_queue.Dequeue(out)) << "enable " << enable;
        EXPECT_TRUE(out.is_address_parity) << "enable " << enable;
        EXPECT_FALSE(out.is_valid) << "enable " << enable;
        EXPECT_TRUE(SameBuffer(out.raw, FlipBit(df20, 40))) << "enable " << enable;
        EXPECT_FALSE(decoder.decoded_mode_s_packet_out_queue.Dequeue(out)) << "enable " << enable;
        EXPECT_EQ(decoder.decoded_mode_s_packet_bit_flip_locations_out_queue.Length(), 0) << "enable " << enable;
    }
}
