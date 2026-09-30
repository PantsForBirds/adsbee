// Host tests for the LR2021 Mode S detector patterns (lr2021_ook_adsb.hh), checked against a chip-level
// model of a Mode S transmission: the preamble, the data chips, and what the demodulator puts out while
// the AGC settles on a strong packet (the first 9 chips blanked). Also the decoder's single-bit prefilter
// (utils/mode_s_single_bit_filter.hh) against the firmware's syndrome table.
#include <stdint.h>
#include <stdio.h>

#include <algorithm>
#include <iterator>
#include <random>
#include <vector>

#include "crc_tables.hh"
#include "lr2021_ook_adsb.hh"
#include "mode_s_single_bit_filter.hh"

using namespace LR2021OokAdsb;

static int failures = 0;

#define EXPECT(cond)                                                 \
    do {                                                             \
        if (!(cond)) {                                               \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            failures++;                                              \
        }                                                            \
    } while (0)

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

static void TestPatternsMatchThePreamble() {
    const std::vector<uint8_t> chips = ModeSChips({0x8D, 0x48, 0x40, 0xD6, 0x20, 0x2C, 0xC3, 0x71, 0xC3, 0x2C, 0xE0,
                                                   0x57, 0x60, 0x98});
    EXPECT(Window(chips, 0, kPreambleNumChips) == kPreambleChips);
    EXPECT(Window(chips, kModeSPatternFirstChip, kModeSPatternLenChips) == kModeSPattern);
    EXPECT(Window(chips, kStrongPatternFirstChip, kStrongPatternLenChips) == kStrongPattern);
    // Both patterns end on the last preamble chip, so the capture starts at message bit 0.
    EXPECT(kModeSPatternFirstChip + kModeSPatternLenChips == kPreambleNumChips);
    EXPECT(kStrongPatternFirstChip + kStrongPatternLenChips == kPreambleNumChips);
    EXPECT(kModeSPatternLenChips <= 16 && kStrongPatternLenChips <= 16);  // SetOokDetector maximum.
}

static void TestPatternValidity() {
    // SetOokDetector: the first two chips of a pattern must be 01 or 10.
    EXPECT(PreambleChipsPatternIsValid(0, 15));
    EXPECT(PreambleChipsPatternIsValid(6, 15));
    EXPECT(PreambleChipsPatternIsValid(7, 15));
    EXPECT(!PreambleChipsPatternIsValid(3, 15));  // Starts with 00.
    EXPECT(!PreambleChipsPatternIsValid(10, 15));
    EXPECT(!PreambleChipsPatternIsValid(15, 15));
    EXPECT(!PreambleChipsPatternIsValid(0, 16));
    EXPECT(PreambleChipsPattern(7, 15) == 0b000000101);
    EXPECT(PreambleChipsPattern(0, 9) == 0b1010000101);
}

// A strong packet makes the AGC cut the gain during the preamble, and the demodulator puts out 0 for
// the first 9 chips. The standard pattern then differs from what the detector sees in 3 chips, the
// strong pattern in 1.
static void TestAgcBlanking() {
    std::vector<uint8_t> chips = ModeSChips({0x5D, 0xAB, 0xCD, 0xEF, 0x00, 0x00, 0x00});
    for (size_t i = 0; i < 9; i++) {
        chips[i] = 0;
    }
    EXPECT(ChipErrors(Window(chips, kModeSPatternFirstChip, kModeSPatternLenChips), kModeSPattern,
                      kModeSPatternLenChips) == 3);
    EXPECT(ChipErrors(Window(chips, kStrongPatternFirstChip, kStrongPatternLenChips), kStrongPattern,
                      kStrongPatternLenChips) == 1);
}

// Without blanking, the strong pattern also lies one chip away from the first pulse pair, 7 chips
// early (the preamble is two copies of 1010000). The detector latches onto that near-match and then
// syncs late in the data, which is why MODE_S_STRONG is only for strong signals.
static void TestStrongPatternAliasOnCleanPreamble() {
    std::vector<uint8_t> chips(7, 0);  // Silence before the packet.
    std::vector<uint8_t> packet = ModeSChips({0x8D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
    chips.insert(chips.end(), packet.begin(), packet.end());
    const size_t true_first = 7 + kStrongPatternFirstChip;
    EXPECT(ChipErrors(Window(chips, true_first, kStrongPatternLenChips), kStrongPattern, kStrongPatternLenChips) ==
           0);
    EXPECT(ChipErrors(Window(chips, true_first - 7, kStrongPatternLenChips), kStrongPattern,
                      kStrongPatternLenChips) == 1);
}

static void TestAgcTriggerRegValue() {
    EXPECT(AgcTriggerRegValue(kAgcTriggerDefault) == 0x00100000);
    EXPECT(AgcTriggerRegValue(kAgcTriggerStandardPreamble) == 0x00400000);
    EXPECT((AgcTriggerRegValue(0xFF) & ~kAgcTriggerMask) == 0);
    EXPECT(kAgcTriggerStandardPreamble > kAgcTriggerDefault);
}

static void TestOokDetectThresholdRegValue() {
    EXPECT(OokDetectThresholdRegValue(kOokDetectThresholdStrong) == 0x00400000);
    EXPECT(OokDetectThresholdRegValue(0x61) == 0x06100000);  // Chip value read back at 3076 kHz.
    // Never touches the pattern length in bits 3:0 of the same register.
    EXPECT((OokDetectThresholdRegValue(0x7F) & 0xF) == 0);
    EXPECT((OokDetectThresholdRegValue(0xFF) & ~kOokDetectThresholdMask) == 0);
}

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

// The DF17 pattern is preamble chips 8-15 plus the chips of the first four DF bits, so it matches at the end of
// the preamble of a DF 16 or 17 frame, and only there: six quiet chips never occur inside Manchester data.
static void TestDF17Pattern() {
    const std::vector<uint8_t> chips = ModeSChips({0x8D, 0x48, 0x40, 0xD6, 0x20, 0x2C, 0xC3, 0x71, 0xC3, 0x2C, 0xE0,
                                                   0x57, 0x60, 0x98});
    EXPECT(Window(chips, kDF17PatternFirstChip, kDF17PatternLenChips) == kDF17Pattern);
    EXPECT(kDF17PatternLenChips == 16);
    // The capture starts right after the pattern: message bit 4.
    EXPECT(kDF17PatternFirstChip + kDF17PatternLenChips == kPreambleNumChips + 2 * kDF17HeaderLenBits);
    // DF 16 and 17 share the pattern's DF bits; every other DF differs in at least one bit, i.e. two chips.
    for (uint8_t df = 0; df < 32; df++) {
        const int errors = ChipErrors(MessageBitsPattern(df >> 1, kDF17HeaderLenBits) << 8,
                                      kDF17Pattern & 0xFF00u, kDF17PatternLenChips);
        EXPECT((df >> 1) == kDF17HeaderBits ? errors == 0 : errors >= 2);
    }
    // No window of random frames' data, at any chip phase, is within one chip of the pattern.
    std::mt19937 rng(8);
    for (int n = 0; n < 200; n++) {
        std::vector<uint8_t> msg(14);
        for (auto& b : msg) b = static_cast<uint8_t>(rng());
        const std::vector<uint8_t> c = ModeSChips(msg);
        for (size_t first = kPreambleNumChips; first + kDF17PatternLenChips <= c.size(); first++) {
            EXPECT(ChipErrors(Window(c, first, kDF17PatternLenChips), kDF17Pattern, kDF17PatternLenChips) >= 2);
        }
    }
    // And nothing earlier in the preamble (after silence) is within one chip of it either.
    std::vector<uint8_t> c(12, 0);
    c.insert(c.end(), chips.begin(), chips.end());
    for (size_t first = 0; first < 12 + kDF17PatternFirstChip; first++) {
        EXPECT(ChipErrors(Window(c, first, kDF17PatternLenChips), kDF17Pattern, kDF17PatternLenChips) >= 2);
    }
}

// The decoder's single-bit prefilter must never hide a match of crc24_find_single_bit_error: every one of the
// firmware's 112 single-bit syndromes passes it, and each is the CRC of its single bit.
static void TestSingleBitFilter() {
    using namespace ModeSSingleBitFilter;
    for (uint16_t i = 0; i < kFrameLenBits; i++) {
        EXPECT(kBitSyndromes.v[i] == crc24_single_bit_syndrome_112[i]);
        EXPECT(kFilter.MayMatch(crc24_single_bit_syndrome_112[i]));
        uint8_t f[14] = {0};
        SetBit(f, i);
        const uint32_t parity = (uint32_t(f[11]) << 16) | (uint32_t(f[12]) << 8) | f[13];
        EXPECT((Crc24(f, 11) ^ parity) == kBitSyndromes.v[i]);
    }
    int passes = 0;
    std::mt19937 rng(4096);
    for (int n = 0; n < 100000; n++) passes += kFilter.MayMatch(rng() & 0xFFFFFF);
    EXPECT(passes < 4000);  // About 112 / 4096 of random syndromes get through.
    printf("  single-bit prefilter passes %d of 100000 random syndromes\n", passes);
}

int main() {
    TestPatternsMatchThePreamble();
    TestPatternValidity();
    TestAgcBlanking();
    TestStrongPatternAliasOnCleanPreamble();
    TestAgcTriggerRegValue();
    TestOokDetectThresholdRegValue();
    TestDF17Pattern();
    TestSingleBitFilter();
    if (failures) {
        printf("%d failure(s)\n", failures);
        return 1;
    }
    printf("lr2021_ook_adsb_test: all passed\n");
    return 0;
}
