#pragma once

#include <stdint.h>

#ifdef ON_PICO
#include "pico.h"
// Tables read for every bit live in RAM: a flash cache miss costs more than the lookup.
#define MODE_S_SOFT_DEMOD_TABLE __not_in_flash("mode_s_soft_demod_tables")
#else
#define MODE_S_SOFT_DEMOD_TABLE
#endif

/**
 * Software Mode S demodulator for the 1090MHz comparator sampler (pulse_sampler in soft_capture.pio).
 *
 * The sampler delivers the comparator output at 12 samples per bit, starting close to the first data bit. Each bit is
 * decided by pulse position: more HI samples in its first half-bit chip than in its second is a 1. The difference is
 * the bit's confidence. The start of the data is found by trying a few sample offsets and keeping the one with the
 * most confident first bits. Frames with a CRC over the whole message (DF11, DF17, DF18) get up to two of their
 * least confident bits flipped if that makes the CRC pass. A frame that still does not validate is read again at the
 * next most confident offsets.
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
// Retries: a frame that fails its CRC (or, for an address/parity format, names no tracked aircraft) is read again at
// up to kMaxRetries other offsets, most confident first, and kept only if it validates there. A wrong pick is usually
// a sample or two off, so retries stay within kMaxRetryShiftSamples of it. It can also be a whole chip early, on the
// tail of the last preamble pulse: then it reads a format without parity, and one chip later is tried too. Offsets
// 3 to 5 samples off read each bit across both of its chips and are never tried. A retry must also have no more than
// kMaxTiesInSearchBits ties: a message read a chip off ties about half its bits.
static constexpr uint16_t kMaxRetryShiftSamples = 2;
static constexpr uint16_t kMaxRetryOffsetSamples = kMaxStartOffsetSamples + kMaxRetryShiftSamples;  // 9
static constexpr uint16_t kMaxRetries = 3;
// Message timing for multilateration. Every data bit has an edge in its middle: falling for a 1, rising for a 0. A
// stronger signal widens the comparator pulses, so rising edges come earlier and falling edges later, by about the
// same amount. The mean position of both kinds stays put, and averaging over several bits resolves it finer than
// one sample.
static constexpr uint16_t kTimingEdgeBits = 24;  // Bits whose mid-bit edges are averaged (a multiple of 8).
static constexpr uint16_t kCountsPerSample = 4;  // 48MHz counts per comparator sample.
// Data start after the start offset when the first bits are all 0 or all 1 (typical for a pulse of nominal width).
static constexpr int16_t kDataStartFallbackCounts = 3;
static constexpr uint16_t kCaptureSamples = kMaxMessageBits * kSamplesPerBit + kMaxStartOffsetSamples;
static constexpr uint16_t kCaptureWords = (kCaptureSamples + 31) / 32;  // 43
static constexpr uint16_t kShortCaptureWords =
    (kShortMessageBits * kSamplesPerBit + kMaxStartOffsetSamples + 31) / 32;  // 22
static_assert(kMaxMessageBits * kSamplesPerBit + kMaxRetryOffsetSamples <= kCaptureWords * 32 &&
                  kShortMessageBits * kSamplesPerBit + kMaxRetryOffsetSamples <= kShortCaptureWords * 32,
              "Retry offsets must fit in the capture.");

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
    // Start of the first data bit, in 48MHz counts (1/48 us) after the first sample. Follows the pulse centers, so
    // it does not depend on the signal level the way start_offset_samples and the sampler start do.
    int16_t data_start_48mhz_counts = 0;
    uint8_t num_flipped_bits = 0;  // Bits changed by the CRC guided fix.
    uint8_t num_retries = 0;       // Offsets read after the first one.
    bool retry_validated = false;  // The message comes from a retry.
};

/**
 * Tells whether an address belongs to a tracked aircraft. Address/parity formats (DF0, 4, 5, 16, 20, 21) have no CRC
 * of their own: their CRC remainder is the sender's address. Must be callable from the demodulator's core.
 */
typedef bool (*KnownAddressFn)(uint32_t address);

/**
 * Addresses of tracked aircraft for the retry, which runs where the aircraft dictionary can't be read. A Bloom
 * filter: it may say yes for an untracked address (3 % of them with 400 aircraft), which only costs a missed
 * retry. The dictionary still checks every address/parity message it forwards.
 */
class KnownAddressFilter {
   public:
    static constexpr uint16_t kNumBits = 4096;

    void Clear() {
        for (uint32_t& word : bits_) {
            word = 0;
        }
    }
    void Add(uint32_t address) {
        uint32_t a, b;
        Hash(address, a, b);
        bits_[a >> 5] |= 1u << (a & 31);
        bits_[b >> 5] |= 1u << (b & 31);
    }
    bool Contains(uint32_t address) const {
        uint32_t a, b;
        Hash(address, a, b);
        return (bits_[a >> 5] >> (a & 31) & 1) && (bits_[b >> 5] >> (b & 31) & 1);
    }

   private:
    static void Hash(uint32_t address, uint32_t& a, uint32_t& b) {
        address &= 0xFFFFFF;
        a = (address * 0x9E3779B1u) >> 20;  // 12 bit indexes from two multiplicative hashes.
        b = (address * 0x85EBCA77u) >> 20;
    }
    static_assert(kNumBits == 1 << 12, "Hash() gives 12 bit indexes.");
    uint32_t bits_[kNumBits / 32] = {0};
};

/**
 * Number of HI samples (0-6) in each 6 bit value.
 */
MODE_S_SOFT_DEMOD_TABLE static constexpr uint8_t kPopCount6[64] = {
    0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6};

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
 * CRC remainders of the values 0-15 shifted up by 24 bits, for taking the CRC 4 bits at a time.
 */
struct CRC24NibbleTable {
    uint32_t remainder[16];
    constexpr CRC24NibbleTable() : remainder() {
        for (uint32_t value = 0; value < 16; value++) {
            uint32_t r = value;
            for (uint16_t i = 0; i < 24; i++) {
                r <<= 1;
                if (r & 0x1000000) {
                    r ^= kCRC24Generator;
                }
            }
            remainder[value] = r;
        }
    }
};
MODE_S_SOFT_DEMOD_TABLE static constexpr CRC24NibbleTable kCRC24NibbleTable = CRC24NibbleTable();

/**
 * Remainder of the Mode S CRC over the first len_bits of a left aligned message (0 for an intact DF17).
 * @param[in] buffer Message, first bit in the MSB of buffer[0].
 * @param[in] len_bits Message length, a multiple of 4.
 */
static inline uint32_t CRC24Remainder(const uint32_t* buffer, uint16_t len_bits) {
    uint32_t remainder = 0;
    for (uint32_t i = 0; i < len_bits; i += 4) {
        remainder = (remainder << 4) | ((buffer[i >> 5] >> (28 - (i & 31))) & 0xF);
        remainder = (remainder & 0xFFFFFF) ^ kCRC24NibbleTable.remainder[remainder >> 24];
    }
    return remainder;
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
 * @param[in] samples kCaptureWords words from the sampler. Only the first num_words are read.
 * @param[in] num_words Number of valid words in samples (kCaptureWords, or kShortCaptureWords for a capture that
 * was cut short after a 56 bit message).
 * @param[out] result Demodulated message. len_bits is 0 if the capture does not look like a message.
 * @param[in] retry Read a frame that does not validate again at other offsets. Up to kMaxRetries + 1 times the work.
 * @param[in] known_address Validates address/parity frames for the retry. Without it they are passed on unchecked and
 * never taken from a retry.
 * @retval True if a message was written to result.
 */
bool Demodulate(const uint32_t* samples, uint16_t num_words, Result& result, bool retry = false,
                KnownAddressFn known_address = nullptr);

}  // namespace mode_s_soft_demod
