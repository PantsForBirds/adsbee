// The packet decoder's single-bit prefilter (mode_s_single_bit_filter.hh), checked against the firmware's syndrome
// table.
#include <random>

#include "crc_tables.hh"
#include "gtest/gtest.h"
#include "mode_s_single_bit_filter.hh"

using namespace ModeSSingleBitFilter;

// Mode S CRC-24 (generator 0xFFF409), bitwise, independent of the firmware's table implementation.
static uint32_t Crc24(const uint8_t* buf, uint16_t len) {
    uint32_t r = 0;
    for (uint16_t i = 0; i < len; i++) {
        r ^= static_cast<uint32_t>(buf[i]) << 16;
        for (int b = 0; b < 8; b++) r = (r & 0x800000) ? ((r << 1) ^ 0xFFF409) : (r << 1);
    }
    return r & 0xFFFFFF;
}

static void SetBit(uint8_t* buf, uint16_t i) { buf[i / 8] = static_cast<uint8_t>(buf[i / 8] | (0x80u >> (i % 8))); }

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
    // About 112 / 4096 of random syndromes get through.
    EXPECT_LT(passes, 4000) << "prefilter passed " << passes << " of 100000 random syndromes";
}
