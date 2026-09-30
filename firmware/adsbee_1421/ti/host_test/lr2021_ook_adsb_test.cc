// Host tests for the LR2021 Mode S detector patterns (lr2021_ook_adsb.hh), checked against a chip-level
// model of a Mode S transmission: the preamble, the data chips, and what the demodulator puts out while
// the AGC settles on a strong packet (the first 9 chips blanked).
#include <stdint.h>
#include <stdio.h>

#include <algorithm>
#include <iterator>
#include <random>
#include <vector>

#include "crc_tables.hh"
#include "df17_recover_reference.hh"
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

// Mode S CRC-24 (generator 0xFFF409), bitwise, independent of the firmware's table implementation.
static uint32_t Crc24(const uint8_t* buf, uint16_t len) {
    uint32_t r = 0;
    for (uint16_t i = 0; i < len; i++) {
        r ^= static_cast<uint32_t>(buf[i]) << 16;
        for (int b = 0; b < 8; b++) r = (r & 0x800000) ? ((r << 1) ^ 0xFFF409) : (r << 1);
    }
    return r & 0xFFFFFF;
}

// A valid DF17 frame: the first 11 bytes given, the parity computed.
static std::vector<uint8_t> DF17Frame(std::vector<uint8_t> data11) {
    const uint32_t p = Crc24(data11.data(), 11);
    data11.push_back(p >> 16);
    data11.push_back((p >> 8) & 0xFF);
    data11.push_back(p & 0xFF);
    return data11;
}

// What the detector's capture holds when it starts at message bit `shift`: 112 bits, bits before the frame
// (shift < 0) and after it set to `fill`.
static std::vector<uint8_t> Capture(const std::vector<uint8_t>& frame, int shift, bool fill = true) {
    std::vector<uint8_t> cap(kModeSFrameLenBytes, 0);
    for (int j = 0; j < kModeSFrameLenBits; j++) {
        const int i = j + shift;
        SetMsgBit(cap.data(), j, (i < 0 || i >= kModeSFrameLenBits) ? fill : GetMsgBit(frame.data(), i));
    }
    return cap;
}

static void TestDF17Pattern() {
    // The pattern is the chips of the five DF bits and ends on a bit boundary (message bit 5).
    const std::vector<uint8_t> chips = ModeSChips({0x8D, 0x48, 0x40, 0xD6, 0x20, 0x2C, 0xC3, 0x71, 0xC3, 0x2C, 0xE0,
                                                   0x57, 0x60, 0x98});
    EXPECT(Window(chips, kPreambleNumChips, kDF17PatternLenChips) == kDF17Pattern);
    EXPECT(kDF17PatternLenChips == 10 && kDF17PatternLenChips <= 16);
    // Every other DF differs from DF17 in at least one bit, i.e. two chips.
    for (uint8_t df = 0; df < 32; df++) {
        if (df == 17) continue;
        EXPECT(ChipErrors(MessageBitsPattern(df, 5), kDF17Pattern, kDF17PatternLenChips) >= 2);
    }
    // Nothing in the preamble (clean, or with the first 9 chips blanked by the AGC) is within one chip of it.
    // The closest window is preamble chips 2-11 (two chips off, clean preamble only): a detector firing there
    // captures from chip 12, message bit -2, one of the shifts the realignment handles.
    for (int blanked = 0; blanked <= 1; blanked++) {
        std::vector<uint8_t> c(12, 0);  // Silence before the packet.
        c.insert(c.end(), chips.begin(), chips.end());
        if (blanked) {
            for (size_t i = 12; i < 12 + 9; i++) c[i] = 0;
        }
        for (size_t first = 0; first + kDF17PatternLenChips <= 12 + kPreambleNumChips; first++) {
            const int errors = ChipErrors(Window(c, first, kDF17PatternLenChips), kDF17Pattern, kDF17PatternLenChips);
            EXPECT(errors > 1);
            EXPECT(errors > 2 || (!blanked && first == 12 + 2));
        }
    }
    EXPECT(ChipErrors(Window(chips, 2, kDF17PatternLenChips), kDF17Pattern, kDF17PatternLenChips) == 2);
    const int alias_capture_bit = (2 + kDF17PatternLenChips - static_cast<int>(kPreambleNumChips)) / 2;
    EXPECT(alias_capture_bit == -2 &&
           std::find(std::begin(kDF17Shifts), std::end(kDF17Shifts), alias_capture_bit) != std::end(kDF17Shifts));
}

static void TestDF17RecoveryShifts() {
    const std::vector<std::vector<uint8_t>> frames = {
        DF17Frame({0x8D, 0x48, 0x40, 0xD6, 0x20, 0x2C, 0xC3, 0x71, 0xC3, 0x2C, 0xE0}),  // CA 5
        DF17Frame({0x8C, 0xAB, 0xCD, 0xEF, 0x99, 0x10, 0x00, 0x00, 0x00, 0x00, 0x01}),  // CA 4
        DF17Frame({0x8E, 0x00, 0x00, 0x01, 0x58, 0xC3, 0x82, 0xD6, 0x90, 0xC8, 0xAC}),  // CA 6
        DF17Frame({0x88, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}),  // CA 0
    };
    for (const auto& f : frames) {
        EXPECT(Crc24(f.data(), 11) == ((uint32_t(f[11]) << 16) | (uint32_t(f[12]) << 8) | f[13]));
        for (int8_t shift : kDF17Shifts) {
            for (int fill = 0; fill <= 1; fill++) {
                const std::vector<uint8_t> cap = Capture(f, shift, fill);
                uint8_t out[kModeSFrameLenBytes];
                EXPECT(RecoverDF17Frame(cap.data(), out, Crc24) == shift);
                EXPECT(std::vector<uint8_t>(out, out + kModeSFrameLenBytes) == f);
            }
        }
    }
}

// The AGC settling leaves errors in message bits 0-7 of a strong frame: bits 0-4 are known, the CA bits 5-7
// are tried flipped.
static void TestDF17RecoveryHeaderErrors() {
    const std::vector<uint8_t> f = DF17Frame({0x8D, 0x48, 0x40, 0xD6, 0x20, 0x2C, 0xC3, 0x71, 0xC3, 0x2C, 0xE0});
    for (int8_t shift : kDF17Shifts) {
        for (uint16_t errs = 0; errs < 256; errs++) {  // Any error pattern in message bits 0-7.
            std::vector<uint8_t> g = f;
            for (int b = 0; b < 8; b++) {
                if ((errs >> b) & 1) SetMsgBit(g.data(), b, !GetMsgBit(g.data(), b));
            }
            const std::vector<uint8_t> cap = Capture(g, shift);
            uint8_t out[kModeSFrameLenBytes];
            EXPECT(RecoverDF17Frame(cap.data(), out, Crc24) == shift);
            EXPECT(std::vector<uint8_t>(out, out + kModeSFrameLenBytes) == f);
        }
    }
    // An error after bit 7 is left to the decoder (single-bit correction): no candidate matches, and the output
    // is the nominal reconstruction.
    std::vector<uint8_t> g = f;
    SetMsgBit(g.data(), 40, !GetMsgBit(g.data(), 40));
    const std::vector<uint8_t> cap = Capture(g, kDF17HeaderLenBits);
    uint8_t out[kModeSFrameLenBytes];
    EXPECT(RecoverDF17Frame(cap.data(), out, Crc24) == INT8_MIN);
    EXPECT(std::vector<uint8_t>(out, out + kModeSFrameLenBytes - 1) ==
           std::vector<uint8_t>(g.begin(), g.end() - 1));  // The last byte ends in capture slop.
}

// Shifts real captures don't show are not tried: such a capture gets the nominal reconstruction (and the
// decoder's single-bit correction), never a wrong frame.
static void TestDF17RecoveryOnlyListedShifts() {
    const std::vector<uint8_t> f = DF17Frame({0x8D, 0x48, 0x40, 0xD6, 0x20, 0x2C, 0xC3, 0x71, 0xC3, 0x2C, 0xE0});
    for (int shift : {-1, 0, 1, 2, 7}) {
        EXPECT(std::find(std::begin(kDF17Shifts), std::end(kDF17Shifts), shift) == std::end(kDF17Shifts));
        const std::vector<uint8_t> cap = Capture(f, shift);
        uint8_t out[kModeSFrameLenBytes];
        EXPECT(RecoverDF17Frame(cap.data(), out, Crc24) == INT8_MIN);
    }
}

// Random captures (noise triggers) must almost never turn into a valid frame: 64 candidates each, so about
// 64 / 2^24 per capture.
static void TestDF17RecoveryRejectsNoise() {
    std::mt19937 rng(1090);
    int accepted = 0;
    for (int n = 0; n < 20000; n++) {
        uint8_t cap[kModeSFrameLenBytes], out[kModeSFrameLenBytes];
        for (auto& b : cap) b = static_cast<uint8_t>(rng());
        if (RecoverDF17Frame(cap, out, Crc24) != INT8_MIN) accepted++;
    }
    EXPECT(accepted <= 1);
}

// RecoverDF17Frame (syndrome version) against the first version (df17_recover_reference.hh): the same shift and
// the same 14 output bytes, for every input.
static int equivalence_checked = 0;
static void ExpectSameAsReference(const uint8_t* cap) {
    uint8_t a[kModeSFrameLenBytes], b[kModeSFrameLenBytes];
    const int8_t sa = LR2021OokAdsbReference::RecoverDF17Frame(cap, a, Crc24);
    const int8_t sb = RecoverDF17Frame(cap, b, Crc24);
    EXPECT(sa == sb);
    EXPECT(std::equal(a, a + kModeSFrameLenBytes, b));
    if (sa != sb || !std::equal(a, a + kModeSFrameLenBytes, b)) {
        printf("    capture:");
        for (uint8_t k = 0; k < kModeSFrameLenBytes; k++) printf(" %02X", cap[k]);
        printf("  (reference shift %d, new %d)\n", sa, sb);
    }
    equivalence_checked++;
}

static void TestDF17RecoveryMatchesReference() {
    EXPECT(kDF17BitSyndromes.v[0] == 0x3935EA);
    // The firmware's own single-bit syndrome table (the decoder's crc24_find_single_bit_error) is the same, so
    // the decoder's prefilter never hides one of its matches.
    for (uint16_t i = 0; i < kModeSFrameLenBits; i++) {
        EXPECT(kDF17BitSyndromes.v[i] == crc24_single_bit_syndrome_112[i]);
        EXPECT(kModeSSingleBitFilter.MayMatch(crc24_single_bit_syndrome_112[i]));
    }
    int filter_passes = 0;
    std::mt19937 filter_rng(4096);
    for (int n = 0; n < 100000; n++) filter_passes += kModeSSingleBitFilter.MayMatch(filter_rng() & 0xFFFFFF);
    EXPECT(filter_passes < 4000);  // About 112 / 4096 of random syndromes get through.
    printf("  single-bit prefilter passes %d of 100000 random syndromes\n", filter_passes);
    // Every bit syndrome is the CRC of that single bit, by the independent bitwise CRC.
    for (uint16_t i = 0; i < kModeSFrameLenBits; i++) {
        uint8_t f[kModeSFrameLenBytes] = {0};
        SetMsgBit(f, i, true);
        const uint32_t parity = (uint32_t(f[11]) << 16) | (uint32_t(f[12]) << 8) | f[13];
        EXPECT((Crc24(f, 11) ^ parity) == kDF17BitSyndromes.v[i]);
    }
    std::mt19937 rng(17);
    auto random_frame = [&rng](uint8_t first_byte) {
        std::vector<uint8_t> d(11);
        for (auto& x : d) x = static_cast<uint8_t>(rng());
        d[0] = first_byte;
        return DF17Frame(d);
    };
    // Every shift the detector can produce and more (-4..9), every error pattern in message bits 0-7 (the CA
    // flips, and the DF bits that are overwritten anyway), both fills, every CA value, 8 frames each.
    for (uint8_t ca = 0; ca < 8; ca++) {
        for (int n = 0; n < 8; n++) {
            const std::vector<uint8_t> f = random_frame(static_cast<uint8_t>((17 << 3) | ca));
            for (int shift = -4; shift <= 9; shift++) {
                for (uint16_t errs = 0; errs < 256; errs++) {
                    std::vector<uint8_t> g = f;
                    for (int b = 0; b < 8; b++) {
                        if ((errs >> b) & 1) SetMsgBit(g.data(), b, !GetMsgBit(g.data(), b));
                    }
                    for (int fill = 0; fill <= 1; fill++) ExpectSameAsReference(Capture(g, shift, fill).data());
                }
            }
        }
    }
    // Valid frames with 1-3 errors anywhere (inside the data, the parity, the unknown bits), at every listed shift.
    for (int n = 0; n < 200000; n++) {
        std::vector<uint8_t> g = random_frame(static_cast<uint8_t>((17 << 3) | (rng() & 7)));
        const int num_errors = 1 + static_cast<int>(rng() % 3);
        for (int e = 0; e < num_errors; e++) {
            const uint16_t b = static_cast<uint16_t>(rng() % kModeSFrameLenBits);
            SetMsgBit(g.data(), b, !GetMsgBit(g.data(), b));
        }
        const int8_t shift = kDF17Shifts[rng() % kDF17NumShifts];
        ExpectSameAsReference(Capture(g, shift, rng() & 1).data());
    }
    // Other DFs and pure noise (false triggers): almost never a match, and the same nominal output.
    for (int n = 0; n < 300000; n++) {
        uint8_t cap[kModeSFrameLenBytes];
        for (auto& x : cap) x = static_cast<uint8_t>(rng());
        ExpectSameAsReference(cap);
    }
    // Every capture where a candidate matches at two shifts must pick the same (the first) one: captures built
    // to be valid at one shift and then shifted into another listed shift's window.
    for (int n = 0; n < 20000; n++) {
        const std::vector<uint8_t> f = random_frame(static_cast<uint8_t>((17 << 3) | (rng() & 7)));
        for (int8_t shift : kDF17Shifts) {
            std::vector<uint8_t> cap = Capture(f, shift, rng() & 1);
            cap[rng() % kModeSFrameLenBytes] ^= static_cast<uint8_t>(1u << (rng() % 8));
            ExpectSameAsReference(cap.data());
        }
    }
    printf("  RecoverDF17Frame == reference on %d captures\n", equivalence_checked);
}

// FIFO framing slip: with k stray bytes in the FIFO stream, every 14-byte read window holds the last k bytes of
// one capture and the first 14 - k of the next. FindDF17ByteSlip finds the frame split across two windows.
static void TestDF17ByteSlip() {
    std::mt19937 rng(1421);
    auto random_frame = [&rng]() {
        std::vector<uint8_t> d(11);
        for (auto& x : d) x = static_cast<uint8_t>(rng());
        d[0] = static_cast<uint8_t>((17 << 3) | 5);
        return DF17Frame(d);
    };
    for (uint8_t k = 1; k < kModeSFrameLenBytes; k++) {
        std::vector<std::vector<uint8_t>> frames;
        std::vector<uint8_t> stream(k);  // k stray bytes, then the captures of real frames.
        for (auto& x : stream) x = static_cast<uint8_t>(rng());
        for (int n = 0; n < 6; n++) {
            frames.push_back(random_frame());
            const std::vector<uint8_t> cap = Capture(frames.back(), kDF17HeaderLenBits, rng() & 1);
            stream.insert(stream.end(), cap.begin(), cap.end());
        }
        // Window i (from 0) holds the tail of capture i-1 and the head of capture i; windows i and i+1 hold
        // capture i whole.
        for (size_t i = 1; i + 1 < frames.size(); i++) {
            const uint8_t* w0 = stream.data() + i * kModeSFrameLenBytes;
            const uint8_t* w1 = w0 + kModeSFrameLenBytes;
            uint8_t out[kModeSFrameLenBytes], dummy[kModeSFrameLenBytes];
            EXPECT(RecoverDF17Frame(w0, dummy, Crc24) == INT8_MIN);  // The slipped window alone fails.
            EXPECT(FindDF17ByteSlip(w0, w1, out, Crc24) == k);
            EXPECT(std::vector<uint8_t>(out, out + kModeSFrameLenBytes) == frames[i]);
        }
    }
    // An aligned stream (consecutive whole captures) shows no slip, and random pairs almost never do: about
    // 13 x 64 / 2^24 per pair.
    int false_slips = 0;
    for (int n = 0; n < 20000; n++) {
        uint8_t a[kModeSFrameLenBytes], b[kModeSFrameLenBytes], out[kModeSFrameLenBytes];
        for (auto& x : a) x = static_cast<uint8_t>(rng());
        for (auto& x : b) x = static_cast<uint8_t>(rng());
        false_slips += FindDF17ByteSlip(a, b, out, Crc24) != 0;
    }
    EXPECT(false_slips <= 2);
    const std::vector<uint8_t> c1 = Capture(random_frame(), kDF17HeaderLenBits);
    const std::vector<uint8_t> c2 = Capture(random_frame(), kDF17HeaderLenBits);
    uint8_t out[kModeSFrameLenBytes];
    EXPECT(FindDF17ByteSlip(c1.data(), c2.data(), out, Crc24) == 0);
    printf("  byte slips 1-13 found; %d false slips in 20000 random pairs\n", false_slips);
}

int main() {
    TestPatternsMatchThePreamble();
    TestPatternValidity();
    TestAgcBlanking();
    TestStrongPatternAliasOnCleanPreamble();
    TestAgcTriggerRegValue();
    TestOokDetectThresholdRegValue();
    TestDF17Pattern();
    TestDF17RecoveryShifts();
    TestDF17RecoveryHeaderErrors();
    TestDF17RecoveryRejectsNoise();
    TestDF17RecoveryOnlyListedShifts();
    TestDF17RecoveryMatchesReference();
    TestDF17ByteSlip();
    if (failures) {
        printf("%d failure(s)\n", failures);
        return 1;
    }
    printf("lr2021_ook_adsb_test: all passed\n");
    return 0;
}
