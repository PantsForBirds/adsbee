#pragma once

#include <stdint.h>

/**
 * Software Mode S demodulator for the 1090MHz comparator sampler (pulse_sampler in soft_capture.pio).
 *
 * The sampler delivers the comparator output at 12 samples per bit, starting close to the first data bit. Each bit is
 * decided by pulse position: more HI samples in its first half-bit chip than in its second is a 1. The difference is
 * the bit's confidence. The start of the data is found by trying a few sample offsets and keeping the one with the
 * most confident first bits. Frames with a CRC over the whole message (DF11, DF17, DF18) get up to two of their
 * least confident bits flipped if that makes the CRC pass.
 */
namespace mode_s_soft_demod {

static constexpr uint16_t kSamplesPerBit = 12;
static constexpr uint16_t kSamplesPerChip = kSamplesPerBit / 2;
static constexpr uint16_t kMaxMessageBits = 112;
static constexpr uint16_t kShortMessageBits = 56;
// Offsets (in samples) tried for the first data bit. The sampler starts half a microsecond (6 samples) before the
// data when the first preamble edge is clean, minus about 1 sample of start-up. Weak first edges cross the
// comparator threshold late and wrong preamble samples delay the matcher, which both lower the offset. The range
// must stay narrower than one chip (6 samples): a pulse position pattern also looks consistent one chip off.
static constexpr uint16_t kMinStartOffsetSamples = 2;
static constexpr uint16_t kMaxStartOffsetSamples = 7;
static constexpr uint16_t kNominalStartOffsetSamples = 5;
static constexpr uint16_t kTimingSearchBits = 12;  // Bits used to pick the start offset.
static constexpr uint16_t kCaptureSamples = kMaxMessageBits * kSamplesPerBit + kMaxStartOffsetSamples;
static constexpr uint16_t kCaptureWords = (kCaptureSamples + 31) / 32;  // 43
static constexpr uint16_t kShortCaptureWords =
    (kShortMessageBits * kSamplesPerBit + kMaxStartOffsetSamples + 31) / 32;  // 22

static constexpr uint16_t kMaxWeakBits = 6;          // Least confident bits that may be flipped.
static constexpr uint16_t kMaxFlippedBits = 2;       // Bits flipped at most to make the CRC pass.
static constexpr uint8_t kWeakBitMaxConfidence = 2;  // A bit this confident or less (of 6) may be flipped.
// A message has one pulse per bit, so bits without any HI sample in either chip (or the same count in both) are
// rare. More of them than this in the timing search bits means there is no message here.
static constexpr uint16_t kMaxTiesInSearchBits = 3;

static constexpr uint32_t kCRC24Generator = 0x1FFF409;

struct Result {
    uint32_t buffer[4] = {0};          // Message bits, left aligned, first bit in the MSB of buffer[0].
    uint16_t len_bits = 0;             // 56 or 112; 0 if no message was found.
    uint8_t start_offset_samples = 0;  // Chosen data start, relative to the first sample.
    uint8_t num_flipped_bits = 0;      // Bits changed by the CRC guided fix.
    uint8_t num_ties = 0;              // Bits with no pulse position information.
    uint16_t confidence_sum = 0;       // Sum of the bit confidences (0 to 6 each).
};

/**
 * Number of HI samples (0-6) in each 6 bit value.
 */
static constexpr uint8_t kPopCount6[64] = {0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4, 1, 2, 2, 3, 2, 3,
                                           3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4,
                                           3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6};

/**
 * Returns the 12 samples of one bit, first sample in bit 11.
 * @param[in] samples Capture, first sample in the MSB of samples[0]. One word past the last sample must be readable.
 * @param[in] first_sample Index of the bit's first sample.
 */
static inline uint32_t BitSamples(const uint32_t* samples, uint32_t first_sample) {
    uint32_t word = first_sample >> 5;
    uint32_t bit = first_sample & 31;
    uint32_t window = samples[word] << bit;
    if (bit > 32 - kSamplesPerBit) {
        window |= samples[word + 1] >> (32 - bit);
    }
    return window >> (32 - kSamplesPerBit);
}

/**
 * Remainder of the Mode S CRC over the first len_bits of a left aligned message (0 for an intact DF17).
 */
static inline uint32_t CRC24Remainder(const uint32_t* buffer, uint16_t len_bits) {
    uint32_t remainder = 0;
    for (uint16_t i = 0; i < len_bits; i++) {
        remainder = (remainder << 1) | ((buffer[i >> 5] >> (31 - (i & 31))) & 1);
        if (remainder & 0x1000000) {
            remainder ^= kCRC24Generator;
        }
    }
    return remainder & 0xFFFFFF;
}

/**
 * CRC remainder caused by a single wrong bit, by its distance from the last message bit.
 */
struct BitSyndromeTable {
    uint32_t syndrome[kMaxMessageBits];
    constexpr BitSyndromeTable() : syndrome() {
        uint32_t s = 1;
        for (uint16_t i = 0; i < kMaxMessageBits; i++) {
            syndrome[i] = s;
            s <<= 1;
            if (s & 0x1000000) {
                s ^= kCRC24Generator;
            }
        }
    }
};
static constexpr BitSyndromeTable kBitSyndromes = BitSyndromeTable();

/**
 * Demodulates one capture.
 * @param[in] samples kCaptureWords words from the sampler (one more readable word is not required).
 * @param[in] num_words Number of valid words in samples (kCaptureWords, or kShortCaptureWords for a capture that
 * was cut short after a 56 bit message).
 * @param[out] result Demodulated message. len_bits is 0 if the capture does not look like a message.
 * @retval True if a message was written to result.
 */
bool Demodulate(const uint32_t* samples, uint16_t num_words, Result& result);

}  // namespace mode_s_soft_demod
