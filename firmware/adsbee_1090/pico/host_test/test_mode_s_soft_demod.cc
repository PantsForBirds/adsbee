#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "../application/mode_s_soft_demod.hh"
#include "gtest/gtest.h"

using namespace mode_s_soft_demod;

// Comparator captures from a 1090U on the bench (pulse sampler output, first sample in the MSB of the first word).
// Bench capture: -86 dBm, trigger offset 300 mV. Sent 8DADF01D9A9571EA7F468007AAB8.
static const uint32_t kWeakTwoFixes[kCaptureWords + 1] = {
    0x07C600C0, 0x1E01FCC0, 0x5C0017F0, 0x07C001CF, 0xC001F300, 0x3C0019B8, 0x07001007, 0x00EC001E, 0x03F00C01,
    0xF03F81F8, 0x0E1E0700, 0x24001FF0, 0x07E001F8, 0x0FF80660, 0x01FDC001, 0x3FE001D0, 0x1FF80001, 0x7C001CFC,
    0x001FF003, 0xC07C001E, 0x01E03EE8, 0x05401807, 0xE001FFC0, 0x011BC000, 0xF800FE07, 0x80080760, 0x7C03C07C,
    0x001AF800, 0x1E03601C, 0xF005C001, 0xEFC003E0, 0x1F81E00C, 0x03C01F00, 0xC01F03E0, 0x1F80303F, 0xFC060030,
    0x0060011E, 0x4001DFC0, 0x01FDC001, 0xE3C0017C, 0x003801C0, 0x01E01E01, 0xE0000000,
};
static const char kWeakTwoFixesSent[] = "8DADF01D9A9571EA7F468007AAB8";

// Bench capture: -86 dBm, trigger offset 300 mV. Sent 8DADF01758955AB5A7A1319F3F2B.
static const uint32_t kWeakOneFix[kCaptureWords + 1] = {
    0xC7F000F8, 0x0F003FC0, 0x1E0002FC, 0x03F000BF, 0xC000CEE0, 0x1E000DFC, 0x00F00000, 0xE01E000F, 0x80FC0780,
    0xE00F007C, 0x0278000F, 0xFF010016, 0x0009FE00, 0x0C7E0230, 0x00F80F00, 0xCFE00178, 0x079E000F, 0xE7000CEE,
    0x000FB000, 0x07F603C0, 0x004FF000, 0xE26000FF, 0xE01E0005, 0x3E000F8E, 0x03C000CE, 0xE000F803, 0xF803F03C,
    0x01E001CF, 0xC000F807, 0x80B803CE, 0x000E007F, 0x800C0007, 0x80380DFC, 0x01E000FC, 0x0BFE03F0, 0x1803603F,
    0x001FC0D3, 0x203E01E0, 0x1E03801E, 0x000F80CB, 0xC0007870, 0x00EFE018, 0x00000000,
};
static const char kWeakOneFixSent[] = "8DADF01758955AB5A7A1319F3F2B";

// Bench capture: -6 dBm, trigger offset 600 mV. Sent 8DADF01567B44832B1FD554E664A.
static const uint32_t kStrong[kCaptureWords + 1] = {
    0x1FF803F8, 0x3F83FFE0, 0xFE003FFE, 0x0FE003FF, 0xE003FFE0, 0xFE003FFE, 0x0FE0FE0F, 0xE0FE003F, 0x83F83F83,
    0xF83F83F8, 0x3FFE003F, 0xFE003FFE, 0x003FFE0F, 0xE003F83F, 0xFE0FE0FE, 0x0FE003FF, 0xE0FE003F, 0xFE003F83,
    0xF83FFE00, 0x3F83FFE0, 0x03F83F83, 0xF83F83FF, 0xC0FE003F, 0x83FFE003, 0xFFE003FF, 0xE0FE003F, 0x83F83FFE,
    0x0FE0FE0F, 0xE0FE0FE0, 0xFE003FFE, 0x003FFE00, 0x3FFE003F, 0xFE003FFE, 0x003FFE00, 0x3F83FFE0, 0xFE0FE003,
    0xF83FFE0F, 0xE003F83F, 0xFE0FE003, 0xF83FFE00, 0x3F83FFE0, 0x03FFE003, 0xF8000000,
};
static const char kStrongSent[] = "8DADF01567B44832B1FD554E664A";

// Bench capture: -60 dBm, trigger offset 600 mV. Sent 5DADF01CC41466.
static const uint32_t kShort[kCaptureWords + 1] = {
    0x003FFF00, 0x1FFF07E0, 0x7E001FFE, 0x07E001FF, 0xE001FFE0, 0x7E001FFE, 0x07E07E07, 0xE07E001F, 0x81F81F81,
    0xF81F81F8, 0x1FFE07E0, 0x7E001F81, 0xFFE07E00, 0x1F81F81F, 0xFE001F81, 0xF81F81F8, 0x1FFE001F, 0xFE001F81,
    0xF81FFE07, 0xE001F81F, 0xFE07E001, 0xF8000000, 0x00000000, 0x00000400, 0x00000000, 0x00020000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000E00,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
};
static const char kShortSent[] = "5DADF01CC41466";

static const char kDF17[] = "8D4840D6202CC371C32CE0576098";

// DF11 all-call reply for address 0x4840D6 whose parity carries the given interrogator code.
static std::string MakeDF11(uint32_t interrogator_code) {
    uint32_t buffer[2] = {0x5D4840D6, 0};
    uint32_t parity = CRC24Remainder(buffer, 56) ^ interrogator_code;
    char hex[15];
    snprintf(hex, sizeof(hex), "5D4840D6%06X", static_cast<unsigned>(parity));
    return hex;
}

static std::vector<uint8_t> HexToBits(const char* hex) {
    std::vector<uint8_t> bits;
    for (const char* c = hex; *c; c++) {
        int nibble = std::stoi(std::string(1, *c), nullptr, 16);
        for (int b = 3; b >= 0; b--) {
            bits.push_back((nibble >> b) & 1);
        }
    }
    return bits;
}

static std::string BufferToHex(const Result& result) {
    static const char kDigits[] = "0123456789ABCDEF";
    std::string hex;
    for (uint16_t i = 0; i < result.len_bits / 4; i++) {
        hex += kDigits[(result.buffer[i / 8] >> (28 - 4 * (i % 8))) & 0xF];
    }
    return hex;
}

// Ideal sampler output for a message: start_offset quiet samples, then pulse_samples HI samples at the start of the
// pulse chip of each bit.
struct Capture {
    uint32_t words[kCaptureWords + 1] = {0};

    void Set(uint32_t sample, bool value) {
        if (sample >= (kCaptureWords + 1) * 32) {
            return;
        }
        uint32_t mask = 1u << (31 - (sample & 31));
        words[sample >> 5] = value ? (words[sample >> 5] | mask) : (words[sample >> 5] & ~mask);
    }

    // early_samples moves the rising edge of every pulse earlier (later if negative) without moving its falling
    // edge.
    Capture(const char* hex, uint16_t start_offset, uint16_t pulse_samples = kSamplesPerChip,
            int16_t early_samples = 0) {
        std::vector<uint8_t> bits = HexToBits(hex);
        pulse_samples += early_samples;
        for (size_t i = 0; i < bits.size(); i++) {
            uint32_t pulse_start = start_offset + i * kSamplesPerBit + (bits[i] ? 0 : kSamplesPerChip) - early_samples;
            for (uint16_t k = 0; k < pulse_samples; k++) {
                Set(pulse_start + k, true);
            }
        }
    }

    // Removes the pulse of one bit.
    void BlankBit(uint16_t start_offset, uint16_t bit_index) {
        for (uint16_t k = 0; k < kSamplesPerBit + 2; k++) {
            Set(start_offset + bit_index * kSamplesPerBit + k, false);
        }
    }
};

TEST(ModeSSoftDemod, CRC24RemainderOfIntactMessagesIsZero) {
    Capture capture(kDF17, kNominalStartOffsetSamples);
    Result result;
    ASSERT_TRUE(Demodulate(capture.words, kCaptureWords, result));
    EXPECT_EQ(CRC24Remainder(result.buffer, 112), 0u);
    result.buffer[1] ^= 0x00010000;  // Bit 47.
    EXPECT_EQ(CRC24Remainder(result.buffer, 112), kBitSyndromes.syndrome[112 - 1 - 47]);
}

TEST(ModeSSoftDemod, CRC24RemainderMatchesBitwiseDivision) {
    uint32_t state = 0x1090;
    for (uint16_t round = 0; round < 200; round++) {
        uint32_t buffer[4];
        for (uint32_t& word : buffer) {
            state = state * 1664525 + 1013904223;  // Linear congruential generator: any bit pattern will do.
            word = state;
        }
        for (uint16_t len_bits : {56, 112}) {
            uint32_t expected = 0;
            for (uint16_t i = 0; i < len_bits; i++) {
                expected = (expected << 1) | ((buffer[i >> 5] >> (31 - (i & 31))) & 1);
                if (expected & 0x1000000) {
                    expected ^= kCRC24Generator;
                }
            }
            EXPECT_EQ(CRC24Remainder(buffer, len_bits), expected & 0xFFFFFF);
        }
    }
}

TEST(ModeSSoftDemod, DecodesAtEveryStartOffset) {
    for (uint16_t offset = kMinStartOffsetSamples; offset <= kMaxStartOffsetSamples; offset++) {
        Capture capture(kDF17, offset);
        Result result;
        ASSERT_TRUE(Demodulate(capture.words, kCaptureWords, result)) << "offset " << offset;
        EXPECT_EQ(result.len_bits, 112);
        EXPECT_EQ(BufferToHex(result), kDF17) << "offset " << offset;
        EXPECT_EQ(result.start_offset_samples, offset);
        EXPECT_EQ(result.num_flipped_bits, 0);
    }
}

TEST(ModeSSoftDemod, ShortMessageFromFullAndShortCapture) {
    std::string df11 = MakeDF11(0);
    Capture capture(df11.c_str(), kNominalStartOffsetSamples);
    Result result;
    ASSERT_TRUE(Demodulate(capture.words, kCaptureWords, result));
    EXPECT_EQ(result.len_bits, 56);
    EXPECT_EQ(BufferToHex(result), df11);
    EXPECT_EQ(CRC24Remainder(result.buffer, 56), 0u);
    EXPECT_EQ(result.buffer[1] & 0xFF, 0u);
    EXPECT_EQ(result.buffer[2], 0u);
    EXPECT_EQ(result.buffer[3], 0u);

    // Cut short by the short capture alarm: the words past the short capture are not valid.
    for (uint16_t i = kShortCaptureWords; i <= kCaptureWords; i++) {
        capture.words[i] = 0xFFFFFFFF;
    }
    ASSERT_TRUE(Demodulate(capture.words, kShortCaptureWords, result));
    EXPECT_EQ(BufferToHex(result), df11);
}

TEST(ModeSSoftDemod, LongMessageInShortCaptureIsRejected) {
    Capture capture(kDF17, kNominalStartOffsetSamples);
    Result result;
    EXPECT_FALSE(Demodulate(capture.words, kShortCaptureWords, result));
    EXPECT_EQ(result.len_bits, 0);
}

TEST(ModeSSoftDemod, NarrowAndWidePulses) {
    // Weak signals leave 3 of 6 samples HI; strong ones spill 2 samples into the next chip.
    for (uint16_t pulse_samples : {3, 4, 8}) {
        Capture capture(kDF17, kNominalStartOffsetSamples, pulse_samples);
        Result result;
        ASSERT_TRUE(Demodulate(capture.words, kCaptureWords, result)) << pulse_samples;
        EXPECT_EQ(BufferToHex(result), kDF17) << pulse_samples;
    }
}

TEST(ModeSSoftDemod, FixesUpToTwoWeakBits) {
    // A missing pulse leaves a bit without pulse position: it reads 0 with no confidence.
    Capture capture(kDF17, kNominalStartOffsetSamples);
    capture.BlankBit(kNominalStartOffsetSamples, 34);  // A 1 bit.
    Result result;
    ASSERT_TRUE(Demodulate(capture.words, kCaptureWords, result));
    EXPECT_EQ(BufferToHex(result), kDF17);
    EXPECT_EQ(result.num_flipped_bits, 1);

    capture.BlankBit(kNominalStartOffsetSamples, 64);  // Another 1 bit.
    ASSERT_TRUE(Demodulate(capture.words, kCaptureWords, result));
    EXPECT_EQ(BufferToHex(result), kDF17);
    EXPECT_EQ(result.num_flipped_bits, 2);
}

TEST(ModeSSoftDemod, DoesNotFlipConfidentBits) {
    // A wrong bit with a clean pulse is not a candidate: the message is handed on unchanged, CRC failing.
    std::string corrupted = kDF17;
    corrupted[10] = corrupted[10] == '2' ? '3' : '2';
    Capture capture(corrupted.c_str(), kNominalStartOffsetSamples);
    Result result;
    ASSERT_TRUE(Demodulate(capture.words, kCaptureWords, result));
    EXPECT_EQ(BufferToHex(result), corrupted);
    EXPECT_EQ(result.num_flipped_bits, 0);
    EXPECT_NE(CRC24Remainder(result.buffer, 112), 0u);
}

TEST(ModeSSoftDemod, ThreeWeakWrongBitsAreLeftAlone) {
    Capture capture(kDF17, kNominalStartOffsetSamples);
    for (uint16_t bit : {5, 34, 64}) {  // All 1 bits.
        capture.BlankBit(kNominalStartOffsetSamples, bit);
    }
    Result result;
    ASSERT_TRUE(Demodulate(capture.words, kCaptureWords, result));
    EXPECT_EQ(result.num_flipped_bits, 0);
    EXPECT_NE(BufferToHex(result), kDF17);
}

TEST(ModeSSoftDemod, AllCallReplies) {
    // Squitter (interrogator code 0) with a missing pulse: fixed.
    std::string squitter = MakeDF11(0);
    Capture capture(squitter.c_str(), kNominalStartOffsetSamples);
    capture.BlankBit(kNominalStartOffsetSamples, 9);  // A 1 bit of the address.
    Result result;
    ASSERT_TRUE(Demodulate(capture.words, kCaptureWords, result));
    EXPECT_EQ(BufferToHex(result), squitter);
    EXPECT_EQ(result.num_flipped_bits, 1);

    // Reply with interrogator code 5 in the parity: the remainder is the code, nothing to fix.
    std::string reply = MakeDF11(5);
    Capture reply_capture(reply.c_str(), kNominalStartOffsetSamples);
    ASSERT_TRUE(Demodulate(reply_capture.words, kCaptureWords, result));
    EXPECT_EQ(BufferToHex(result), reply);
    EXPECT_EQ(result.num_flipped_bits, 0);
    EXPECT_EQ(CRC24Remainder(result.buffer, 56), 5u);
}

TEST(ModeSSoftDemod, RejectsNoise) {
    Result result;
    uint32_t quiet[kCaptureWords + 1] = {0};
    EXPECT_FALSE(Demodulate(quiet, kCaptureWords, result));

    // Sparse comparator chatter: single HI samples every 23 samples.
    uint32_t chatter[kCaptureWords + 1] = {0};
    for (uint32_t sample = 5; sample < kCaptureWords * 32; sample += 23) {
        chatter[sample >> 5] |= 1u << (31 - (sample & 31));
    }
    EXPECT_FALSE(Demodulate(chatter, kCaptureWords, result));
}

TEST(ModeSSoftDemod, BenchCaptures) {
    Result result;
    ASSERT_TRUE(Demodulate(kStrong, kCaptureWords, result));
    EXPECT_EQ(BufferToHex(result), kStrongSent);
    EXPECT_EQ(result.num_flipped_bits, 0);

    ASSERT_TRUE(Demodulate(kShort, kCaptureWords, result));
    EXPECT_EQ(BufferToHex(result), kShortSent);
    ASSERT_TRUE(Demodulate(kShort, kShortCaptureWords, result));
    EXPECT_EQ(BufferToHex(result), kShortSent);

    ASSERT_TRUE(Demodulate(kWeakOneFix, kCaptureWords, result));
    EXPECT_EQ(BufferToHex(result), kWeakOneFixSent);
    EXPECT_EQ(result.num_flipped_bits, 1);

    ASSERT_TRUE(Demodulate(kWeakTwoFixes, kCaptureWords, result));
    EXPECT_EQ(BufferToHex(result), kWeakTwoFixesSent);
    EXPECT_EQ(result.num_flipped_bits, 2);
}

TEST(ModeSSoftDemod, DataStartOfIdealPulses) {
    // Pulses of nominal width: the data starts at the start offset, 4 counts per sample.
    for (uint16_t offset = kMinStartOffsetSamples; offset <= kMaxStartOffsetSamples; offset++) {
        Capture capture(kDF17, offset);
        Result result;
        ASSERT_TRUE(Demodulate(capture.words, kCaptureWords, result)) << "offset " << offset;
        EXPECT_EQ(result.data_start_48mhz_counts, offset * kCountsPerSample) << "offset " << offset;
    }
}

TEST(ModeSSoftDemod, DataStartFollowsPulseCenters) {
    const uint16_t kOffset = 4;
    struct Shape {
        uint16_t pulse_samples;  // Before early_samples is applied.
        int16_t early_samples;
        int16_t center_shift_counts;  // Half the change of the falling edge minus that of the rising edge.
    };
    // Strong signals widen a pulse on both sides, weak ones narrow it on both sides: the timing must not move. A
    // pulse that only grows or shrinks on one side moves by half of that.
    const Shape kShapes[] = {{6, 0, 0}, {7, 1, 0},  {8, 1, 2},  {5, -1, 0}, {8, 0, 4},
                             {7, 0, 2}, {5, 0, -2}, {4, 0, -4}, {6, 1, -2}, {6, 2, -4}};
    for (const Shape& shape : kShapes) {
        Capture capture(kDF17, kOffset, shape.pulse_samples, shape.early_samples);
        Result result;
        ASSERT_TRUE(Demodulate(capture.words, kCaptureWords, result));
        EXPECT_EQ(BufferToHex(result), kDF17);
        EXPECT_EQ(result.data_start_48mhz_counts, kOffset * kCountsPerSample + shape.center_shift_counts)
            << "pulse " << shape.pulse_samples << " samples, " << shape.early_samples << " early";
    }
}

TEST(ModeSSoftDemod, DataStartIsFinerThanOneSample) {
    // Half of the pulses one sample late, as when the true position lies between two samples and noise decides.
    Capture capture(kDF17, 4);
    std::vector<uint8_t> bits = HexToBits(kDF17);
    for (uint16_t i = 0; i < kTimingEdgeBits; i += 2) {
        uint32_t pulse_start = 4 + i * kSamplesPerBit + (bits[i] ? 0 : kSamplesPerChip);
        capture.Set(pulse_start, false);
        capture.Set(pulse_start + kSamplesPerChip, true);
    }
    Result result;
    ASSERT_TRUE(Demodulate(capture.words, kCaptureWords, result));
    EXPECT_EQ(result.start_offset_samples, 4);
    EXPECT_EQ(result.data_start_48mhz_counts, 4 * kCountsPerSample + kCountsPerSample / 2);
}

TEST(ModeSSoftDemod, DataStartWithOneKindOfEdge) {
    // DF0 with zeros in its first bytes: no falling mid-bit edges to average with the rising ones.
    Capture capture("00000000A5A5A5", 5);
    Result result;
    ASSERT_TRUE(Demodulate(capture.words, kCaptureWords, result));
    EXPECT_EQ(result.len_bits, 56);
    EXPECT_EQ(result.data_start_48mhz_counts, 5 * kCountsPerSample + kDataStartFallbackCounts);
}

// Bench captures of one transmitter at five levels (attenuator steps), with the data start that an independent
// implementation of the estimate (Python, on the same samples) gives for each.
struct TimedCapture {
    const char* sent;
    int16_t data_start_48mhz_counts;
    uint32_t words[kCaptureWords + 1];
};
static const TimedCapture kTimedCaptures[] = {
    // -1 dBm.
    {"8DADF0015872210C3A83DABD0156",
     22,
     {0x07F001FC, 0x1FC1FFF0, 0x7F001FFF, 0x07F001FF, 0xF001FFF0, 0x7F001FFF, 0x07F03F07, 0xF03F001F, 0xC1FC1FC1,
      0xFC1FC0FC, 0x0FC0FC0F, 0xC0FC0FFF, 0x001FFF00, 0x0FFF03F0, 0x00FE0FE0, 0xFE0FFF83, 0xF83F800F, 0xE0FFF800,
      0xFE0FE0FF, 0xF800FE0F, 0xE0FE0FFF, 0x800FE0FE, 0x0FE0FFF8, 0x3F800FE0, 0xFE0FE0FF, 0xF83F83F8, 0x00FFF800,
      0xFFF800FE, 0x0FE0FE0F, 0xE0FFF83F, 0x83F83F80, 0x0FFF83F8, 0x00FFF800, 0xFFF800FF, 0xF83F83F8, 0x3F800FFF,
      0x800FE0FE, 0x0FE0FE0F, 0xE0FE0FFF, 0x800FFF80, 0x0FFF800F, 0xFF81F800, 0x7F000000}},
    // -16 dBm.
    {"8DADF0009816B4B5A5DE558E5C77",
     24,
     {0x07FC01FF, 0x1FE0FFF0, 0x3F000FFF, 0x03F000FF, 0xF000FFF0, 0x3F000FFF, 0x03F03F03, 0xF03F000F, 0xE0FC0FC0,
      0xFC0FC0FC, 0x0FC0FC0F, 0xC0FC0FC0, 0xFFF000FE, 0x0FFF03F0, 0x00FE0FC0, 0xFC0FC0FC, 0x0FFF000F, 0xFF03F000,
      0xFFF000FF, 0xF03F000F, 0xFF000FF0, 0xFFF000FF, 0xF03F000F, 0xFF000FFF, 0x83F800FF, 0xF800FF0F, 0xFF000FFF,
      0x81F81F80, 0x0FFF81F8, 0x1F81F800, 0xFF07FF80, 0x0FFF800F, 0xFF8007FF, 0x81F8007F, 0x07E07FF8, 0x1F81F800,
      0x7F07FF80, 0x07FF81F8, 0x1F8007F0, 0x7E07FF81, 0xF81F8007, 0xFF81F81F, 0x80000000}},
    // -31 dBm.
    {"8DADF001636BDABB894936125C72",
     25,
     {0x07F801FE, 0x1FE1FFF8, 0x3F801FFF, 0x83F800FF, 0xF800FFF8, 0x3F800FFF, 0x83F83F83, 0xF83F800F, 0xE0FE0FE0,
      0xFE0FE0FE, 0x0FE0FE0F, 0xE0FE0FFF, 0x800FFF83, 0xF800FE0F, 0xE0FFF83F, 0x800FFF83, 0xF800FFF8, 0x00FFF83F,
      0x83F83F80, 0x0FFF83F8, 0x00FFF800, 0xFFF800FF, 0xF83F83F8, 0x00FFF83F, 0xC3F800FE, 0x0FF0FFF8, 0x00FF0FFF,
      0xC00FFFC0, 0x0FF0FFF8, 0x00FF0FFF, 0xC00FF0FF, 0xF81FC00F, 0xFFC1FC00, 0xFF07F07F, 0x07FFC007, 0xF07FFC00,
      0x7F07FFC0, 0x07FFC1FC, 0x1FC007F0, 0x7F07FFC1, 0xFC1FC007, 0xF07FFC00, 0x7F000000}},
    // -46 dBm.
    {"8DADF0059B89CB4B0FDC9B17F688",
     22,
     {0x07F001FC, 0x1FC1FFF0, 0x7F001FFF, 0x07F001FF, 0xF001FFF0, 0x7F001FFF, 0x03F07F03, 0xF07F001F, 0xC1FC1FC0,
      0xFC0FC0FC, 0x0FC0FC0F, 0xFF000FFF, 0x03F000FE, 0x0FFF03F0, 0x00FFF03F, 0x03F000FE, 0x0FE0FFF0, 0x00FE0FFF,
      0x03F03F00, 0x0FE0FFF0, 0x00FFF03F, 0x800FFF80, 0x0FE0FFF8, 0x00FFF83F, 0x800FE0FE, 0x0FE0FFF8, 0x3F83F83F,
      0x83F83F80, 0x0FFF83F8, 0x3F800FE0, 0xFFF800FE, 0x0FFF83F8, 0x00FFF83F, 0x800FE0FE, 0x0FFF800F, 0xFF83F83F,
      0x81F83F81, 0xF83F800F, 0xFF81F800, 0xFFF800FE, 0x07E07FF8, 0x00FE07E0, 0x7E000000}},
    // -61 dBm.
    {"8DADF0009D618D3A8251DF51C20D",
     24,
     {0x07F001FE, 0x0FC0FFF0, 0x3F000FFF, 0x03F000FF, 0xF000FFF0, 0x3F000FFF, 0x03F03F03, 0xF03F000F, 0xC0FE0FE0,
      0xFC0FC0FC, 0x0FE0FE0F, 0xC0FC0FE0, 0xFFF000FE, 0x0FFF83F8, 0x3F800FFF, 0x800FFF83, 0xF800FE0F, 0xE0FE0FFF,
      0x81F800FE, 0x0FE0FFF8, 0x1F800FFF, 0x800FE0FF, 0xF81F81F8, 0x00FFF800, 0x7FF800FE, 0x0FE07E07, 0xE07FF800,
      0x7E07FF80, 0x07FF8007, 0xE07E07FF, 0x81F81F80, 0x07FF81F8, 0x1F81F81F, 0x8007FF80, 0x07FF8007, 0xE07E07FF,
      0x81F81F80, 0x07E07E07, 0xE07FF800, 0x7F07E07E, 0x07E07FF8, 0x1F8007FF, 0xC0000000}},
};

TEST(ModeSSoftDemod, DataStartOfBenchCaptures) {
    for (const TimedCapture& capture : kTimedCaptures) {
        Result result;
        ASSERT_TRUE(Demodulate(capture.words, kCaptureWords, result)) << capture.sent;
        EXPECT_EQ(BufferToHex(result), capture.sent);
        EXPECT_EQ(result.num_flipped_bits, 0);
        EXPECT_EQ(result.data_start_48mhz_counts, capture.data_start_48mhz_counts) << capture.sent;
    }
}

// Live sky captures (attic-bee, t-0870) whose first pick of start offset misreads the message.

// Clean DF17 one sample after the first pick, which reads its last bit wrong with confidence.
static const uint32_t kRetryOneSampleLate[kCaptureWords + 1] = {
    0x0F8003E0, 0xAE03E600, 0xD8003708, 0x038003FF, 0x0003601E, 0x02A01FE0, 0x001603D7, 0x0001FB00, 0x03990098,
    0x003FF013, 0x801A0037, 0x58078001, 0xB6400100, 0x37E40180, 0x02F01FB8, 0x00078380, 0x3EF0003F, 0xF8003A05,
    0xCF80F803, 0x0003F01F, 0x01FF8078, 0x010001B0, 0x1900F3C0, 0x01F01E01, 0x77C001F7, 0x00380F80, 0x01E03F03,
    0xB03F3400, 0x1FF00031, 0x03F00703, 0xF00F0030, 0x1701F600, 0x00D01FB0, 0x000DF000, 0x0601F01F, 0xF8000D01,
    0xF3807E07, 0xC0FC0008, 0x81F01F01, 0xC78000F7, 0x80019F80, 0x01EDC00E, 0x00000600, 0x00000000,
};

// DF17 one sample after the first pick, with one weak bit fixed there.
static const uint32_t kRetryWithFix[kCaptureWords + 1] = {
    0x0F8113F8, 0x3F03FC40, 0x7E0DC07E, 0x070003DB, 0x00048740, 0x7C002E00, 0xFFE003B0, 0x2D03D840, 0x017000EC,
    0x001D00FC, 0xC001F6C0, 0xFC07C001, 0xE180FC04, 0xC001FC1D, 0x80381FC0, 0x011F0070, 0x03E807C0, 0x01EF407C,
    0x001F81B8, 0x0FBC07C0, 0x0603C000, 0x601B0070, 0x0D00A01F, 0x00F80C00, 0xF01B81B8, 0x1081F00F, 0x80400780,
    0xA81F80D8, 0x0D01F807, 0x81F80FC0, 0xF81F80F8, 0x0F80F818, 0x80180D80, 0x181F80FF, 0x4030001E, 0x40D80FE8,
    0x02A001F7, 0xC02203F0, 0x00F80FED, 0x07F000F0, 0x0F80F80F, 0xF6000FC6, 0x00000000, 0x00000000,
};

// DF4 from tracked aircraft 0xA1CE16. The first pick, a chip early on the last preamble pulse, reads a DF9.
// Cut short after 56 bits.
static const uint32_t kRetryOneChipLate[kCaptureWords + 1] = {
    0x6007F03F, 0xFC003F03, 0xF03F03F0, 0x3F03F03F, 0x03F03F03, 0xF03F03F0, 0x3F03F03F, 0x03FFC003, 0xFFC0FC0F,
    0xC0FC003F, 0xFC0FC003, 0xF03F03F0, 0x3FFC0FC0, 0xFC003FFC, 0x0FC003FF, 0xC003FFC0, 0xFC0FC003, 0xFFC0FC0F,
    0xC003FFC0, 0xFC003F83, 0xF03F01FF, 0xC0000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
};

// Reads as a DF17 that fails its CRC at every offset.
static const uint32_t kJunkExtendedSquitter[kCaptureWords + 1] = {
    0x8F8203F8, 0x3B81FCC0, 0xFC001FFC, 0x07C001FF, 0xC001FFC0, 0x01F01E01, 0xF00F01F0, 0x1F01B01F, 0x01FFC07C,
    0x181F81EE, 0xC01DFBE0, 0x01F00FD8, 0x0064FC00, 0xF83C03CC, 0x00F84081, 0x801FA203, 0xE07E07F0, 0x0803E07E,
    0x1EC001EB, 0xE03C00F7, 0x01F81F81, 0xFFE019FE, 0x007E03E0, 0x00F8173E, 0x07003E0F, 0xC011FE00, 0x1E000F80,
    0xF8BF80FD, 0xC040F81F, 0xB0009F89, 0xE40000C0, 0x1C0183A0, 0xC1841F8E, 0xF80F80FF, 0xE03C03E0, 0x01FB407E,
    0x000F80FD, 0x6000F81F, 0x80FFC000, 0x7FE07F00, 0x1F80F80F, 0x80FFC03E, 0x00000000, 0x00000000,
};

// A second capture inside another message: a DF0 naming no tracked aircraft at any offset.
static const uint32_t kJunkOverlap[kCaptureWords + 1] = {
    0x005F81FC, 0x1F81781F, 0x81FFF001, 0xFF8001FF, 0xE001780F, 0x80F81FFE, 0x07E001F8, 0x0F81F81F, 0xFE001F81,
    0xFFC001FF, 0xC03E000F, 0x81FFE03E, 0x000FFE07, 0xE000F81F, 0xFC001FFE, 0x03E000FC, 0x0FFE000F, 0xC0F80FFE,
    0x03C01E00, 0x0FFE000F, 0x80FFE01F, 0x00000000, 0x00000100, 0x06030008, 0x10000012, 0x00010800, 0x48184084,
    0x00200080, 0x06200046, 0x218C2000, 0x41028480, 0x00025800, 0x00A20802, 0x03010001, 0x07804600, 0x00200000,
    0x00028063, 0x01C00301, 0x00000010, 0x00002000, 0xE000C005, 0x100443E0, 0x08060009, 0x00000000,
};

static bool KnownA1CE16(uint32_t address) { return address == 0xA1CE16; }
static bool KnownAllButA1CE16(uint32_t address) { return address != 0xA1CE16; }

TEST(ModeSSoftDemod, RetryReadsTheNextOffset) {
    Result result;
    ASSERT_TRUE(Demodulate(kRetryOneSampleLate, kCaptureWords, result));
    EXPECT_EQ(BufferToHex(result), "8DA12B769914E388B8500944F0AA");
    EXPECT_FALSE(result.retry_validated);

    ASSERT_TRUE(Demodulate(kRetryOneSampleLate, kCaptureWords, result, true));
    EXPECT_EQ(BufferToHex(result), "8DA12B769914E388B8500944F0AB");
    EXPECT_EQ(result.start_offset_samples, 4);
    EXPECT_EQ(result.num_retries, 1);
    EXPECT_TRUE(result.retry_validated);
    EXPECT_EQ(result.num_flipped_bits, 0);

    ASSERT_TRUE(Demodulate(kRetryWithFix, kCaptureWords, result, true));
    EXPECT_EQ(BufferToHex(result), "8FAC8B2EE11B1E0000000031B985");
    EXPECT_EQ(result.start_offset_samples, 5);
    EXPECT_EQ(result.num_flipped_bits, 1);
    EXPECT_TRUE(result.retry_validated);
}

TEST(ModeSSoftDemod, RetryTakesAddressParityOnlyForATrackedAircraft) {
    Result result;
    ASSERT_TRUE(Demodulate(kRetryOneChipLate, kShortCaptureWords, result, true, KnownA1CE16));
    EXPECT_EQ(BufferToHex(result), "200017B0ED7761");
    EXPECT_EQ(result.start_offset_samples, 8);
    EXPECT_TRUE(result.retry_validated);

    // Unknown address, or no address check: the first pick stays.
    for (KnownAddressFn known : {static_cast<KnownAddressFn>(nullptr), +[](uint32_t) { return false; }}) {
        ASSERT_TRUE(Demodulate(kRetryOneChipLate, kShortCaptureWords, result, true, known));
        EXPECT_EQ(BufferToHex(result), "4FFFE00700000E");
        EXPECT_EQ(result.start_offset_samples, 2);
        EXPECT_FALSE(result.retry_validated);
        EXPECT_EQ(result.num_retries, kMaxRetries);
    }
}

TEST(ModeSSoftDemod, RetryLeavesJunkAlone) {
    Result plain, retried;
    for (const uint32_t* capture : {kJunkExtendedSquitter, kJunkOverlap}) {
        ASSERT_TRUE(Demodulate(capture, kCaptureWords, plain));
        ASSERT_TRUE(Demodulate(capture, kCaptureWords, retried, true, KnownA1CE16));
        EXPECT_EQ(BufferToHex(retried), BufferToHex(plain));
        EXPECT_FALSE(retried.retry_validated);
        EXPECT_EQ(retried.num_retries, kMaxRetries);
    }

    uint32_t quiet[kCaptureWords + 1] = {0};
    EXPECT_FALSE(Demodulate(quiet, kCaptureWords, retried, true, KnownAllButA1CE16));
}

static uint32_t untracked_address = 0;
static bool AllButUntracked(uint32_t address) { return address != untracked_address; }

TEST(ModeSSoftDemod, RetryNeverReadsAValidFormatAChipLater) {
    // A chip later, this DF5 from an untracked aircraft ties only 3 of its first 12 bits and reads as a DF16 naming
    // another address. With every other address tracked it must still not be taken.
    const char kDF5[] = "2FF919ACDB834D";
    for (uint16_t offset : {kMinStartOffsetSamples, static_cast<uint16_t>(kMinStartOffsetSamples + 1)}) {
        Capture capture(kDF5, offset);
        Result result;
        ASSERT_TRUE(Demodulate(capture.words, kCaptureWords, result));
        untracked_address = CRC24Remainder(result.buffer, kShortMessageBits);
        ASSERT_TRUE(Demodulate(capture.words, kCaptureWords, result, true, AllButUntracked));
        EXPECT_EQ(BufferToHex(result), kDF5);
        EXPECT_FALSE(result.retry_validated);
    }
}

TEST(ModeSSoftDemod, RetryKeepsValidMessages) {
    Result result;
    ASSERT_TRUE(Demodulate(kStrong, kCaptureWords, result, true, KnownA1CE16));
    EXPECT_EQ(BufferToHex(result), kStrongSent);
    EXPECT_EQ(result.num_retries, 0);
    ASSERT_TRUE(Demodulate(kWeakTwoFixes, kCaptureWords, result, true, KnownA1CE16));
    EXPECT_EQ(BufferToHex(result), kWeakTwoFixesSent);
    EXPECT_EQ(result.num_retries, 0);
}

TEST(ModeSSoftDemod, KnownAddressFilter) {
    KnownAddressFilter filter;
    uint32_t address = 0x123456;
    for (uint16_t i = 0; i < 400; i++) {
        address = (address * 1103515245u + 12345u) & 0xFFFFFF;
        filter.Add(address);
        EXPECT_TRUE(filter.Contains(address));
    }
    uint16_t false_positives = 0;
    for (uint32_t i = 0; i < 10000; i++) {
        address = (address * 1103515245u + 12345u) & 0xFFFFFF;
        false_positives += filter.Contains(address);
    }
    EXPECT_LT(false_positives, 500);  // About 3 % expected.
    filter.Clear();
    EXPECT_FALSE(filter.Contains(address));
}
