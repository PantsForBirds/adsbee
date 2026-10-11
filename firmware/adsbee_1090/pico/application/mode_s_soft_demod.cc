#include "mode_s_soft_demod.hh"

#ifdef ON_PICO
#include "pico.h"
#define SOFT_DEMOD_FUNC(name) __time_critical_func(name)
#else
#define SOFT_DEMOD_FUNC(name) name
#endif

namespace mode_s_soft_demod {

// One bit decision: returns confidence (0-6), sets bit.
static inline uint8_t DecideBit(const uint32_t* samples, uint32_t first_sample, uint32_t& bit) {
    uint32_t window = BitSamples(samples, first_sample);
    int first_chip = kPopCount6[window >> kSamplesPerChip];
    int second_chip = kPopCount6[window & ((1u << kSamplesPerChip) - 1)];
    bit = first_chip > second_chip;
    return static_cast<uint8_t>(first_chip > second_chip ? first_chip - second_chip : second_chip - first_chip);
}

bool SOFT_DEMOD_FUNC(Demodulate)(const uint32_t* samples, uint16_t num_words, Result& result) {
    result = Result();

    // Timing: keep the start offset whose first bits are the most confident.
    uint16_t best_offset = kNominalStartOffsetSamples;
    uint16_t best_confidence = 0;
    uint16_t best_ties = kTimingSearchBits;
    for (uint16_t offset = kMinStartOffsetSamples; offset <= kMaxStartOffsetSamples; offset++) {
        uint16_t confidence = 0;
        uint16_t ties = 0;
        for (uint16_t i = 0; i < kTimingSearchBits; i++) {
            uint32_t bit;
            uint8_t bit_confidence = DecideBit(samples, offset + i * kSamplesPerBit, bit);
            confidence += bit_confidence;
            ties += bit_confidence == 0;
        }
        if (confidence > best_confidence) {
            best_confidence = confidence;
            best_offset = offset;
            best_ties = ties;
        }
    }
    if (best_ties > kMaxTiesInSearchBits) {
        return false;  // Noise or the tail of another message.
    }

    // Bit decisions. The first bit of the downlink format tells the length: DF16 and up are 112 bits.
    static uint8_t confidences[kMaxMessageBits];  // Static: called from one interrupt context, small stack.
    uint16_t len_bits = kMaxMessageBits;
    uint16_t available_bits = (num_words * 32 - best_offset) / kSamplesPerBit;
    for (uint16_t i = 0; i < len_bits; i++) {
        uint32_t bit;
        confidences[i] = DecideBit(samples, best_offset + i * kSamplesPerBit, bit);
        result.buffer[i >> 5] |= bit << (31 - (i & 31));
        result.confidence_sum += confidences[i];
        result.num_ties += confidences[i] == 0;
        if (i == 0) {
            len_bits = bit ? kMaxMessageBits : kShortMessageBits;
            if (len_bits > available_bits) {
                return false;  // Capture was cut short but the message is long.
            }
        }
    }
    result.len_bits = len_bits;
    result.start_offset_samples = static_cast<uint8_t>(best_offset);

    // CRC guided fix for formats whose parity covers the message alone.
    uint32_t downlink_format = result.buffer[0] >> 27;
    if (downlink_format != 11 && downlink_format != 17 && downlink_format != 18) {
        return true;
    }
    uint32_t remainder = CRC24Remainder(result.buffer, len_bits);
    if (remainder == 0 || (downlink_format == 11 && remainder < 0x80)) {
        return true;  // Intact (DF11 replies carry an interrogator code in the low 7 bits).
    }
    // Collect the least confident bits.
    uint16_t weak_bits[kMaxWeakBits];
    uint16_t num_weak_bits = 0;
    for (uint8_t level = 0; level <= kWeakBitMaxConfidence && num_weak_bits < kMaxWeakBits; level++) {
        for (uint16_t i = 0; i < len_bits && num_weak_bits < kMaxWeakBits; i++) {
            if (confidences[i] == level) {
                weak_bits[num_weak_bits++] = i;
            }
        }
    }
    auto syndrome = [len_bits](uint16_t bit_index) { return kBitSyndromes.syndrome[len_bits - 1 - bit_index]; };
    auto flip = [&result](uint16_t bit_index) { result.buffer[bit_index >> 5] ^= 1u << (31 - (bit_index & 31)); };
    for (uint16_t a = 0; a < num_weak_bits; a++) {
        if (syndrome(weak_bits[a]) == remainder) {
            flip(weak_bits[a]);
            result.num_flipped_bits = 1;
            return true;
        }
    }
    if (kMaxFlippedBits >= 2) {
        for (uint16_t a = 0; a < num_weak_bits; a++) {
            for (uint16_t b = a + 1; b < num_weak_bits; b++) {
                if ((syndrome(weak_bits[a]) ^ syndrome(weak_bits[b])) == remainder) {
                    flip(weak_bits[a]);
                    flip(weak_bits[b]);
                    result.num_flipped_bits = 2;
                    return true;
                }
            }
        }
    }
    return true;  // Not fixable here: the decoder decides what to do with it.
}

}  // namespace mode_s_soft_demod
