// The ADSBee 1421 PacketDecoder's handling of failed and address parity frames: single-bit correction through the
// shared DecodedModeSPacket::CorrectSingleBitError (only DF=17/18, never a bit in the DF field, per DO-260B
// 2.2.4.3.4.7.3.a) and address parity frames forwarded unchanged. The rule itself is tested with the ADSBee 1090 host
// tests (test_mode_s_single_bit_correction.cc); these check the 1421 decoder uses it.
#include <cstring>
#include <memory>
#include <random>

#include "crc.hh"
#include "gtest/gtest.h"
#include "packet_decoder.hh"
#include "settings.hh"

SettingsManager settings_manager;  // PacketDecoder::DecodeOne reads the log level.

// A 112-bit frame with downlink format df, random contents and a correct parity field (the CRC XOR icao; icao = 0 for
// formats checked by their CRC alone). The LR2021 always captures 112 bits, so 56-bit formats are sent the same way,
// with their parity field in bits 32-55 and noise after it.
static RawModeSPacket MakeCapture(uint16_t df, std::mt19937& rng, uint32_t icao = 0) {
    const uint16_t len_bytes = df < 16 ? RawModeSPacket::kSquitterPacketLenBytes
                                       : RawModeSPacket::kExtendedSquitterPacketLenBytes;
    uint8_t bytes[RawModeSPacket::kExtendedSquitterPacketLenBytes];
    for (uint8_t& b : bytes) b = rng() & 0xFF;
    bytes[0] = static_cast<uint8_t>((df << 3) | (bytes[0] & 0b111));
    const uint32_t parity = crc24(bytes, len_bytes - 3) ^ icao;
    bytes[len_bytes - 3] = parity >> 16;
    bytes[len_bytes - 2] = (parity >> 8) & 0xFF;
    bytes[len_bytes - 1] = parity & 0xFF;
    uint32_t words[RawModeSPacket::kMaxPacketLenWords32] = {0};
    for (uint16_t i = 0; i < RawModeSPacket::kExtendedSquitterPacketLenBytes; i++) {
        words[i / 4] |= uint32_t(bytes[i]) << (24 - 8 * (i % 4));
    }
    return RawModeSPacket(words, RawModeSPacket::kExtendedSquitterPacketNumWords32);
}

static RawModeSPacket FlipBit(const RawModeSPacket& raw, uint16_t i) {
    RawModeSPacket flipped = raw;
    flip_bit(flipped.buffer, i);
    return flipped;
}

static bool SameBuffer(const RawModeSPacket& a, const RawModeSPacket& b) {
    return a.buffer_len_bytes == b.buffer_len_bytes && memcmp(a.buffer, b.buffer, sizeof(a.buffer)) == 0;
}

class PacketDecoderTest : public ::testing::Test {
   protected:
    // Decodes one capture and returns how many packets came out (into out, the last one).
    int Decode(const RawModeSPacket& capture, bool df17_mode, DecodedModeSPacket& out) {
        RawModeSPacket raw = capture;
        decoder->DecodeOne(raw, df17_mode);
        int num_out = 0;
        while (decoder->decoded_mode_s_packet_out_queue.Dequeue(out)) num_out++;
        return num_out;
    }

    std::unique_ptr<PacketDecoder> decoder = std::make_unique<PacketDecoder>();
};

// DF=17 (both receiver mode families) and DF=18 (preamble modes): every single-bit error outside the DF field is
// corrected back to the original frame. Errors in the DF field are never corrected; the frame is dropped, or forwarded
// unchanged if the error made it an address parity format.
TEST_F(PacketDecoderTest, CorrectsDF17AndDF18OutsideTheDFField) {
    std::mt19937 rng(1421);
    for (uint16_t df : {17, 18}) {
        for (bool df17_mode : {false, true}) {
            if (df == 18 && df17_mode) continue;  // DF17 modes only receive DF=17.
            const RawModeSPacket frame = MakeCapture(df, rng);
            for (uint16_t i = 0; i < RawModeSPacket::kExtendedSquitterPacketLenBits; i++) {
                const uint32_t fixed_before = decoder->bitflips_fixed_count;
                DecodedModeSPacket out;
                const int num_out = Decode(FlipBit(frame, i), df17_mode, out);
                if (i < DecodedModeSPacket::kDFNumBits) {
                    EXPECT_EQ(decoder->bitflips_fixed_count, fixed_before) << "DF " << df << " bit " << i;
                    if (num_out > 0) {
                        EXPECT_FALSE(out.is_valid) << "DF " << df << " bit " << i;
                        EXPECT_TRUE(out.is_address_parity) << "DF " << df << " bit " << i;
                        EXPECT_TRUE(SameBuffer(out.raw, FlipBit(frame, i))) << "DF " << df << " bit " << i;
                    }
                } else {
                    ASSERT_EQ(num_out, 1) << "DF " << df << " bit " << i << " df17_mode " << df17_mode;
                    EXPECT_EQ(decoder->bitflips_fixed_count, fixed_before + 1) << "DF " << df << " bit " << i;
                    EXPECT_TRUE(out.is_valid) << "DF " << df << " bit " << i;
                    EXPECT_TRUE(SameBuffer(out.raw, frame)) << "DF " << df << " bit " << i;
                }
            }
        }
    }
}

// Other 112-bit formats with a single-bit error (DF=19 military, DF=22, DF=24 Comm-D) are dropped, never corrected
// into ADS-B packets.
TEST_F(PacketDecoderTest, DropsOtherExtendedFormatsWithBitErrors) {
    std::mt19937 rng(19);
    for (uint16_t df : {19, 22, 24}) {
        const RawModeSPacket frame = MakeCapture(df, rng);
        for (uint16_t i = DecodedModeSPacket::kDFNumBits; i < RawModeSPacket::kExtendedSquitterPacketLenBits; i++) {
            DecodedModeSPacket out;
            EXPECT_EQ(Decode(FlipBit(frame, i), false, out), 0) << "DF " << df << " bit " << i;
        }
    }
    EXPECT_EQ(decoder->bitflips_fixed_count, 0u);
}

// Address parity formats are forwarded as received, with or without a bit error (which only changes the recovered
// ICAO address), for the aircraft dictionary to confirm. 56-bit formats are trimmed from the 112-bit capture first.
TEST_F(PacketDecoderTest, ForwardsAddressParityFramesUnchanged) {
    std::mt19937 rng(20);
    const uint32_t icao = 0xA1B2C3;
    for (uint16_t df : {0, 4, 5, 16, 20, 21}) {
        const RawModeSPacket capture = MakeCapture(df, rng, icao);
        for (int i = -1; i < 112; i++) {
            if (df < 16 && i >= 56) break;
            // i = -1: no bit error. The DF field is left alone here: errors there make other formats.
            if (i >= 0 && i < DecodedModeSPacket::kDFNumBits) continue;
            const RawModeSPacket received = i < 0 ? capture : FlipBit(capture, i);
            DecodedModeSPacket out;
            ASSERT_EQ(Decode(received, false, out), 1) << "DF " << df << " bit " << i;
            EXPECT_TRUE(out.is_address_parity) << "DF " << df << " bit " << i;
            EXPECT_FALSE(out.is_valid) << "DF " << df << " bit " << i;
            EXPECT_EQ(out.downlink_format, df);
            if (i < 0) {
                EXPECT_EQ(out.icao_address, icao) << "DF " << df;
            }
            RawModeSPacket expected = received;
            if (df < 16) {
                expected = RawModeSPacket(expected.buffer, RawModeSPacket::kSquitterPacketNumWords32);
            }
            EXPECT_TRUE(SameBuffer(out.raw, expected)) << "DF " << df << " bit " << i;
        }
    }
    EXPECT_EQ(decoder->bitflips_fixed_count, 0u);
}
