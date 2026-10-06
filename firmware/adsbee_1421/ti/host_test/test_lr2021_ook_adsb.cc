// LR2021 Mode S detector patterns and settings (lr2021_ook_adsb.hh), checked against a chip-level model of a Mode S
// packet.
#include <random>
#include <vector>

#include "gtest/gtest.h"
#include "lr2021_ook_adsb.hh"

using namespace LR2021OokAdsb;

// Chips of a Mode S transmission: 16 preamble chips (pulses at 0, 1, 3.5 and 4.5 us), then 2 chips per
// message bit (1 -> 10, 0 -> 01), MSB of msg[0] first.
static std::vector<uint8_t> ModeSChips(const std::vector<uint8_t>& msg) {
    std::vector<uint8_t> chips(kPreambleNumChips, 0);
    chips[0] = chips[2] = chips[7] = chips[9] = 1;
    for (uint8_t byte : msg) {
        for (int b = 7; b >= 0; b--) {
            bool bit = (byte >> b) & 1;
            chips.push_back(bit ? 1 : 0);
            chips.push_back(bit ? 0 : 1);
        }
    }
    return chips;
}

// Chips first..first+len-1 of the stream packed LSB-first, the SetOokDetector pattern layout.
static uint16_t Window(const std::vector<uint8_t>& chips, size_t first, size_t len) {
    uint16_t w = 0;
    for (size_t i = 0; i < len; i++) {
        w |= static_cast<uint16_t>(chips[first + i] & 1) << i;
    }
    return w;
}

static int ChipErrors(uint16_t a, uint16_t b, size_t len) {
    uint16_t x = (a ^ b) & static_cast<uint16_t>((1u << len) - 1u);
    return __builtin_popcount(x);
}

TEST(LR2021OokAdsb, PatternsMatchThePreamble) {
    const std::vector<uint8_t> chips = ModeSChips({0x8D, 0x48, 0x40, 0xD6, 0x20, 0x2C, 0xC3, 0x71, 0xC3, 0x2C, 0xE0,
                                                   0x57, 0x60, 0x98});
    EXPECT_EQ(Window(chips, 0, kPreambleNumChips), kPreambleChips);
    EXPECT_EQ(Window(chips, kModeSPatternFirstChip, kModeSPatternLenChips), kModeSPattern);
    // The pattern ends on the last preamble chip, so the capture starts at message bit 0.
    EXPECT_EQ(kModeSPatternFirstChip + kModeSPatternLenChips, kPreambleNumChips);
    // SetOokDetector maximum.
    EXPECT_LE(kModeSPatternLenChips, 16);
}

TEST(LR2021OokAdsb, PatternValidity) {
    // SetOokDetector: the first two chips of a pattern must be 01 or 10.
    EXPECT_TRUE(PreambleChipsPatternIsValid(0, 15));
    EXPECT_TRUE(PreambleChipsPatternIsValid(6, 15));
    EXPECT_TRUE(PreambleChipsPatternIsValid(7, 15));
    EXPECT_FALSE(PreambleChipsPatternIsValid(3, 15));  // Starts with 00.
    EXPECT_FALSE(PreambleChipsPatternIsValid(10, 15));
    EXPECT_FALSE(PreambleChipsPatternIsValid(15, 15));
    EXPECT_FALSE(PreambleChipsPatternIsValid(0, 16));
    EXPECT_EQ(PreambleChipsPattern(7, 15), 0b000000101);
    EXPECT_EQ(PreambleChipsPattern(0, 9), 0b1010000101);
}

// AGC blanking zeroes the first 9 chips of a strong packet and the standard pattern then misses by 3 chips, which is
// why MODE_S_STRONG runs with the AGC off.
TEST(LR2021OokAdsb, AgcBlanking) {
    std::vector<uint8_t> chips = ModeSChips({0x5D, 0xAB, 0xCD, 0xEF, 0x00, 0x00, 0x00});
    for (size_t i = 0; i < 9; i++) {
        chips[i] = 0;
    }
    EXPECT_EQ(ChipErrors(Window(chips, kModeSPatternFirstChip, kModeSPatternLenChips), kModeSPattern,
                         kModeSPatternLenChips),
              3);
}

TEST(LR2021OokAdsb, StrongGainStep) {
    EXPECT_EQ(StrongGainStep(true, 0), kStrongGainStep);  // AGC off at the fixed STRONG step.
    EXPECT_EQ(StrongGainStep(false, 0), 0);               // MODE_S keeps the AGC.
    EXPECT_EQ(StrongGainStep(true, 9), 9);                // A manual gain wins in both.
    EXPECT_EQ(StrongGainStep(false, 9), 9);
    EXPECT_GE(kStrongGainStep, 1);
    EXPECT_LE(kStrongGainStep, 13);
}

TEST(LR2021OokAdsb, WideGainStep) {
    EXPECT_EQ(WideGainStep(0), kWideGainStep);  // Fixed gain unless a manual gain is set.
    EXPECT_EQ(WideGainStep(9), 9);
    EXPECT_GE(kWideGainStep, 1);
    EXPECT_LE(kWideGainStep, kMaxGainStep);
    EXPECT_EQ(kFrontEndModeNoSaturationBlanking & ~kFrontEndModeMask, 0u);  // Only bits 7:0 are written.
}

TEST(LR2021OokAdsb, AgcTriggerRegValue) {
    EXPECT_EQ(AgcTriggerRegValue(kAgcTriggerDefault), 0x00100000u);
    EXPECT_EQ(AgcTriggerRegValue(kAgcTriggerStandardPreamble), 0x00400000u);
    EXPECT_EQ(AgcTriggerRegValue(0xFF) & ~kAgcTriggerMask, 0u);
    EXPECT_GT(kAgcTriggerStandardPreamble, kAgcTriggerDefault);
}

TEST(LR2021OokAdsb, OokDetectThresholdRegValue) {
    EXPECT_EQ(OokDetectThresholdRegValue(kOokDetectThresholdStrong), 0x00000000u);
    EXPECT_EQ(OokDetectThresholdRegValue(0x04), 0x00400000u);
    EXPECT_EQ(OokDetectThresholdRegValue(0x61), 0x06100000u);  // Chip value read back at 3076 kHz.
    // Never touches the pattern length in bits 3:0 of the same register.
    EXPECT_EQ(OokDetectThresholdRegValue(0x7F) & 0xF, 0u);
    EXPECT_EQ(OokDetectThresholdRegValue(0xFF) & ~kOokDetectThresholdMask, 0u);
    EXPECT_EQ(OokDetectThresholdFromReg(0xA610000F), 0x61);  // MODE_S register read back.
    EXPECT_EQ(OokDetectThresholdFromReg(0xA000000F), kOokDetectThresholdStrong);
}

// The DF17 pattern matches only at the end of a DF 16/17 preamble: six quiet chips never occur inside data.
TEST(LR2021OokAdsb, DF17Pattern) {
    const std::vector<uint8_t> chips = ModeSChips({0x8D, 0x48, 0x40, 0xD6, 0x20, 0x2C, 0xC3, 0x71, 0xC3, 0x2C, 0xE0,
                                                   0x57, 0x60, 0x98});
    EXPECT_EQ(Window(chips, kDF17PatternFirstChip, kDF17PatternLenChips), kDF17Pattern);
    EXPECT_EQ(kDF17PatternLenChips, 16);
    // The capture starts right after the pattern: message bit 4.
    EXPECT_EQ(kDF17PatternFirstChip + kDF17PatternLenChips, kPreambleNumChips + 2 * kDF17HeaderLenBits);
    // DF 16 and 17 share the pattern's DF bits; every other DF differs in at least one bit, i.e. two chips.
    for (uint8_t df = 0; df < 32; df++) {
        const int errors = ChipErrors(MessageBitsPattern(df >> 1, kDF17HeaderLenBits) << 8, kDF17Pattern & 0xFF00u,
                                      kDF17PatternLenChips);
        if ((df >> 1) == kDF17HeaderBits) {
            EXPECT_EQ(errors, 0) << "DF " << int(df);
        } else {
            EXPECT_GE(errors, 2) << "DF " << int(df);
        }
    }
    // No window of random frames' data, at any chip phase, is within one chip of the pattern.
    std::mt19937 rng(8);
    for (int n = 0; n < 200; n++) {
        std::vector<uint8_t> msg(14);
        for (auto& b : msg) b = static_cast<uint8_t>(rng());
        const std::vector<uint8_t> c = ModeSChips(msg);
        for (size_t first = kPreambleNumChips; first + kDF17PatternLenChips <= c.size(); first++) {
            EXPECT_GE(ChipErrors(Window(c, first, kDF17PatternLenChips), kDF17Pattern, kDF17PatternLenChips), 2)
                << "random frame " << n << ", window at chip " << first;
        }
    }
    // And nothing earlier in the preamble (after silence) is within one chip of it either.
    std::vector<uint8_t> c(12, 0);
    c.insert(c.end(), chips.begin(), chips.end());
    for (size_t first = 0; first < 12 + kDF17PatternFirstChip; first++) {
        EXPECT_GE(ChipErrors(Window(c, first, kDF17PatternLenChips), kDF17Pattern, kDF17PatternLenChips), 2)
            << "window at chip " << first << " (12 chips of silence, then the preamble)";
    }
}

// ReconstructDF17Frame output: the consumed DF bits, the 112 capture bits, then zeros.
TEST(LR2021OokAdsb, DF17Reconstruction) {
    std::mt19937 rng(1421);
    for (int n = 0; n < 100000; n++) {
        uint8_t cap[14];
        for (auto& b : cap) b = static_cast<uint8_t>(rng());
        uint32_t words[4];
        ReconstructDF17Frame(cap, sizeof(cap), words, 4);
        for (uint16_t i = 0; i < 128; i++) {
            bool expect;
            if (i < kDF17HeaderLenBits) {
                expect = (kDF17HeaderBits >> (kDF17HeaderLenBits - 1 - i)) & 1u;
            } else if (i < kDF17HeaderLenBits + 8 * sizeof(cap)) {
                const uint16_t c = i - kDF17HeaderLenBits;
                expect = (cap[c / 8] >> (7 - c % 8)) & 1u;
            } else {
                expect = false;
            }
            ASSERT_EQ(((words[i / 32] >> (31 - i % 32)) & 1u) != 0, expect) << "random capture " << n << ", bit " << i;
        }
    }
}
