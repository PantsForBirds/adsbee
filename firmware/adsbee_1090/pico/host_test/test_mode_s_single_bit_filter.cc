// The decoder's single-bit prefilter (mode_s_single_bit_filter.hh), checked against the firmware's syndrome table and
// crc24_find_single_bit_error.
#include <cstring>
#include <random>

#include "crc.hh"
#include "crc_tables.hh"
#include "gtest/gtest.h"
#include "mode_s_packet_decoder.hh"
#include "mode_s_single_bit_filter.hh"

using namespace ModeSSingleBitFilter;

// Mode S CRC-24 (generator 0xFFF409), bitwise, independent of the firmware's table implementation.
static uint32_t Crc24(const uint8_t* buf, uint16_t len) {
    uint32_t r = 0;
    for (uint16_t i = 0; i < len; i++) {
        r ^= static_cast<uint32_t>(buf[i]) << 16;
        for (int b = 0; b < 8; b++) r = (r & 0x800000) ? ((r << 1) ^ kCRC24Generator) : (r << 1);
    }
    return r & 0xFFFFFF;
}

static void SetBit(uint8_t* buf, uint16_t i) { buf[i / 8] = static_cast<uint8_t>(buf[i / 8] | (0x80u >> (i % 8))); }

// The search the decoder runs: the prefilter, then crc24_find_single_bit_error only for syndromes that pass it.
static int16_t FilteredFindSingleBitError(uint32_t syndrome) {
    return kFilter.MayMatch(syndrome) ? crc24_find_single_bit_error(syndrome, kFrameLenBits) : -1;
}

// The prefilter must never hide a match of crc24_find_single_bit_error: every one of the firmware's 112 single-bit
// syndromes passes it, and each is the CRC of its single bit.
TEST(ModeSSingleBitFilter, PassesEverySingleBitSyndrome) {
    for (uint16_t i = 0; i < kFrameLenBits; i++) {
        EXPECT_EQ(kBitSyndromes.v[i], crc24_single_bit_syndrome_112[i]) << "bit " << i;
        EXPECT_TRUE(kFilter.MayMatch(crc24_single_bit_syndrome_112[i])) << "bit " << i;
        uint8_t f[14] = {0};
        SetBit(f, i);
        const uint32_t parity = (uint32_t(f[11]) << 16) | (uint32_t(f[12]) << 8) | f[13];
        EXPECT_EQ(Crc24(f, 11) ^ parity, kBitSyndromes.v[i]) << "bit " << i;
    }
}

TEST(ModeSSingleBitFilter, RejectsMostRandomSyndromes) {
    int passes = 0;
    std::mt19937 rng(4096);
    for (int n = 0; n < 100000; n++) passes += kFilter.MayMatch(rng() & 0xFFFFFF);
    // The filter has 98 of its 4096 bits set, so about 2.4 % of random syndromes get through.
    EXPECT_LT(passes, 4000) << "prefilter passed " << passes << " of 100000 random syndromes";
}

// The filtered search returns exactly what the plain search returns, for random syndromes (almost all of which are
// not single-bit errors) and for every single-bit syndrome.
TEST(ModeSSingleBitFilter, FilteredSearchMatchesPlainSearch) {
    std::mt19937 rng(112);
    int num_found = 0;
    for (int n = 0; n < 200000; n++) {
        const uint32_t syndrome = rng() & 0xFFFFFF;
        const int16_t plain = crc24_find_single_bit_error(syndrome, kFrameLenBits);
        ASSERT_EQ(FilteredFindSingleBitError(syndrome), plain) << "syndrome 0x" << std::hex << syndrome;
        num_found += plain >= 0;
    }
    // A random syndrome is a single-bit syndrome with a probability of 112 / 2^24, so expect none or a few.
    EXPECT_LT(num_found, 10);

    for (uint16_t i = 0; i < kFrameLenBits; i++) {
        const uint32_t syndrome = crc24_single_bit_syndrome_112[i];
        EXPECT_EQ(crc24_find_single_bit_error(syndrome, kFrameLenBits), i);
        EXPECT_EQ(FilteredFindSingleBitError(syndrome), i);
    }
}

// Same comparison on real frames: a valid DF=17 with one or two bits flipped, syndromes from crc24_syndrome.
TEST(ModeSSingleBitFilter, FilteredSearchMatchesPlainSearchOnCorruptedFrames) {
    const uint8_t frame[14] = {0x8D, 0x40, 0x62, 0x1D, 0x58, 0xC3, 0x82, 0xD6, 0x90, 0xC8, 0xAC, 0x28, 0x63, 0xA7};
    ASSERT_EQ(crc24_syndrome(frame, sizeof(frame)), 0u);
    for (uint16_t i = 0; i < kFrameLenBits; i++) {
        uint8_t corrupted[14];
        memcpy(corrupted, frame, sizeof(frame));
        flip_bit(corrupted, i);
        const uint32_t syndrome = crc24_syndrome(corrupted, sizeof(corrupted));
        EXPECT_EQ(crc24_find_single_bit_error(syndrome, kFrameLenBits), i);
        EXPECT_EQ(FilteredFindSingleBitError(syndrome), i);
        for (uint16_t j = i + 1; j < kFrameLenBits; j++) {
            uint8_t corrupted2[14];
            memcpy(corrupted2, corrupted, sizeof(corrupted));
            flip_bit(corrupted2, j);
            const uint32_t syndrome2 = crc24_syndrome(corrupted2, sizeof(corrupted2));
            const int16_t plain = crc24_find_single_bit_error(syndrome2, kFrameLenBits);
            ASSERT_EQ(plain, -1) << "bits " << i << ", " << j;  // CRC-24 detects every 2-bit error in 112 bits.
            ASSERT_EQ(FilteredFindSingleBitError(syndrome2), plain) << "bits " << i << ", " << j;
        }
    }
}

// The decoder still corrects every single-bit error outside the DF field, at the right bit.
TEST(ModeSSingleBitFilter, DecoderCorrectsEverySingleBitError) {
    const RawModeSPacket valid_packet((const char*)"8D40621D58C382D690C8AC2863A7");
    for (uint16_t i = DecodedModeSPacket::kDFNumBits; i < kFrameLenBits; i++) {
        ModeSPacketDecoder decoder(ModeSPacketDecoder::PacketDecoderConfig{.enable_1090_error_correction = true});
        RawModeSPacket raw_packet = valid_packet;
        flip_bit(raw_packet.buffer, i);
        decoder.raw_mode_s_packet_in_queue.Enqueue(raw_packet);
        decoder.UpdateDecoderLoop();

        DecodedModeSPacket decoded_packet;
        ASSERT_TRUE(decoder.decoded_mode_s_packet_out_queue.Dequeue(decoded_packet)) << "bit " << i;
        EXPECT_TRUE(decoded_packet.is_valid) << "bit " << i;
        EXPECT_EQ(decoded_packet.icao_address, 0x40621Du) << "bit " << i;
        uint16_t bit_flip_index = 0;
        EXPECT_TRUE(decoder.decoded_mode_s_packet_bit_flip_locations_out_queue.Dequeue(bit_flip_index));
        EXPECT_EQ(bit_flip_index, i);
    }
}
