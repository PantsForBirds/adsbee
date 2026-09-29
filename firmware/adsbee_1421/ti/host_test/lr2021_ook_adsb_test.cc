// Host tests for the LR2021 Mode S detector patterns (lr2021_ook_adsb.hh), checked against a chip-level
// model of a Mode S transmission: the preamble, the data chips, and what the demodulator puts out while
// the AGC settles on a strong packet (the first 9 chips blanked).
#include <stdint.h>
#include <stdio.h>

#include <vector>

#include "lr2021_ook_adsb.hh"

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

int main() {
    TestPatternsMatchThePreamble();
    TestPatternValidity();
    TestAgcBlanking();
    TestStrongPatternAliasOnCleanPreamble();
    TestAgcTriggerRegValue();
    TestOokDetectThresholdRegValue();
    if (failures) {
        printf("%d failure(s)\n", failures);
        return 1;
    }
    printf("lr2021_ook_adsb_test: all passed\n");
    return 0;
}
