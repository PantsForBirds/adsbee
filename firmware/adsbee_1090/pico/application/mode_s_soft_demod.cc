#include "mode_s_soft_demod.hh"

#ifdef ON_PICO
#include "pico.h"
#define SOFT_DEMOD_FUNC(name) __time_critical_func(name)
#else
#define SOFT_DEMOD_FUNC(name) name
#endif

namespace mode_s_soft_demod {

static constexpr uint32_t kBitsPerGroup = 8;  // 8 bits are 96 samples: exactly 3 capture words.
static constexpr uint32_t kSamplesPerGroup = kBitsPerGroup * kSamplesPerBit;

// The 96 samples of 8 consecutive bits, lined up with the word boundaries so every sample sits at a fixed position.
// Reads 3 words from the word of first_sample on, and a fourth one unless first_sample is on a word boundary.
static inline void AlignGroup(const uint32_t* samples, uint32_t first_sample, uint32_t& a, uint32_t& b, uint32_t& c) {
    const uint32_t* words = samples + (first_sample >> 5);
    uint32_t shift = first_sample & 31;
    a = words[0];
    b = words[1];
    c = words[2];
    if (shift != 0) {
        a = (a << shift) | (b >> (32 - shift));
        b = (b << shift) | (c >> (32 - shift));
        c = (c << shift) | (words[3] >> (32 - shift));
    }
}

// Bit margins of up to 8 consecutive bits: HI samples in the first chip of a bit minus those in its second chip.
// Positive for a 1; the size is the confidence. Only the first 4 margins are written if num_bits is 4 or less.
static inline void BitMargins(const uint32_t* samples, uint32_t first_sample, int margins[kBitsPerGroup],
                              uint32_t num_bits = kBitsPerGroup) {
    uint32_t a, b, c;
    AlignGroup(samples, first_sample, a, b, c);
    auto count = [](uint32_t chip) { return static_cast<int>(kPopCount6[chip & 0x3F]); };
    margins[0] = count(a >> 26) - count(a >> 20);
    margins[1] = count(a >> 14) - count(a >> 8);
    margins[2] = count(a >> 2) - count((a << 4) | (b >> 28));
    margins[3] = count(b >> 22) - count(b >> 16);
    if (num_bits > 4) {
        margins[4] = count(b >> 10) - count(b >> 4);
        margins[5] = count((b << 2) | (c >> 30)) - count(c >> 24);
        margins[6] = count(c >> 18) - count(c >> 12);
        margins[7] = count(c >> 6) - count(c);
    }
}

// Weighted bit margins of 8 consecutive bits (kChipTables): positive for a 1, never 0; the size is the confidence.
static inline void WeightedMargins(const uint32_t* samples, uint32_t first_sample, int margins[kBitsPerGroup]) {
    uint32_t a, b, c;
    AlignGroup(samples, first_sample, a, b, c);
    auto margin = [](uint32_t chip_1, uint32_t chip_2) {
        return static_cast<int>(kChipTables.value[chip_1 & 0x3F]) - kChipTables.value[64 + (chip_2 & 0x3F)];
    };
    margins[0] = margin(a >> 26, a >> 20);
    margins[1] = margin(a >> 14, a >> 8);
    margins[2] = margin(a >> 2, (a << 4) | (b >> 28));
    margins[3] = margin(b >> 22, b >> 16);
    margins[4] = margin(b >> 10, b >> 4);
    margins[5] = margin((b << 2) | (c >> 30), c >> 24);
    margins[6] = margin(c >> 18, c >> 12);
    margins[7] = margin(c >> 6, c);
}

// HI samples among the middle 6 samples (3 to 8) of each of 8 consecutive bits. The mid-bit edge of a bit lies in
// this window: a falling edge (1 bit) is at sample 3 + count, a rising edge (0 bit) at sample 9 - count.
static inline void MidBitHiCounts(const uint32_t* samples, uint32_t first_sample, uint8_t counts[kBitsPerGroup]) {
    uint32_t a, b, c;
    AlignGroup(samples, first_sample, a, b, c);
    auto count = [](uint32_t window) { return kPopCount6[window & 0x3F]; };
    counts[0] = count(a >> 23);
    counts[1] = count(a >> 11);
    counts[2] = count((a << 1) | (b >> 31));
    counts[3] = count(b >> 19);
    counts[4] = count(b >> 7);
    counts[5] = count((b << 5) | (c >> 27));
    counts[6] = count(c >> 15);
    counts[7] = count(c >> 3);
}

// Start of the first data bit in 48MHz counts after the first sample, from the mid-bit edges of the first
// kTimingEdgeBits bits of a demodulated message. The middle of a bit is at 6 + (mean falling edge count - mean
// rising edge count) / 2 samples after the start offset.
static int16_t SOFT_DEMOD_FUNC(DataStart48MHzCounts)(const uint32_t* samples, const Result& result) {
    int32_t fall_sum = 0, fall_bits = 0, rise_sum = 0, rise_bits = 0;
    uint8_t counts[kBitsPerGroup];
    for (uint32_t bits_done = 0; bits_done < kTimingEdgeBits; bits_done += kBitsPerGroup) {
        MidBitHiCounts(samples, result.start_offset_samples + bits_done * kSamplesPerBit, counts);
        uint32_t byte = result.buffer[0] >> (24 - bits_done);
        for (uint32_t i = 0; i < kBitsPerGroup; i++) {
            if (byte & (0x80 >> i)) {
                fall_sum += counts[i];
                fall_bits++;
            } else {
                rise_sum += counts[i];
                rise_bits++;
            }
        }
    }
    int32_t offset_counts = result.start_offset_samples * kCountsPerSample;
    if (fall_bits == 0 || rise_bits == 0) {
        return static_cast<int16_t>(offset_counts + kDataStartFallbackCounts);
    }
    // (fall mean - rise mean) / 2 samples, in counts, rounded to nearest.
    int32_t numerator = (kCountsPerSample / 2) * (fall_sum * rise_bits - rise_sum * fall_bits);
    int32_t denominator = fall_bits * rise_bits;
    numerator += numerator >= 0 ? denominator / 2 : -(denominator / 2);
    return static_cast<int16_t>(offset_counts + numerator / denominator);
}

// Confidence and ties of the first kTimingSearchBits bits read from each start offset.
struct OffsetScores {
    uint32_t confidence[kMaxRetryOffsetSamples + 1] = {0};
    uint32_t ties[kMaxRetryOffsetSamples + 1] = {0};
};

static void SOFT_DEMOD_FUNC(ScoreOffsets)(const uint32_t* samples, uint32_t first_offset, uint32_t last_offset,
                                          OffsetScores& scores) {
    int margins[kBitsPerGroup];
    for (uint32_t offset = first_offset; offset <= last_offset; offset++) {
        for (uint32_t bits_done = 0; bits_done < kTimingSearchBits; bits_done += kBitsPerGroup) {
            uint32_t bits_in_group =
                kTimingSearchBits - bits_done < kBitsPerGroup ? kTimingSearchBits - bits_done : kBitsPerGroup;
            BitMargins(samples, offset + bits_done * kSamplesPerBit, margins, bits_in_group);
            for (uint32_t i = 0; i < bits_in_group; i++) {
                scores.confidence[offset] += static_cast<uint32_t>(margins[i] < 0 ? -margins[i] : margins[i]);
                scores.ties[offset] += margins[i] == 0;
            }
        }
    }
}

enum Check : uint8_t { kNoMessage, kCRCPassed, kCRCFailed, kNoCRC };

// Reads the message whose first bit starts at offset, with the CRC guided fix.
static Check SOFT_DEMOD_FUNC(DecodeAt)(const uint32_t* samples, uint16_t num_words, uint32_t offset, Result& result) {
    result = Result();
    int margins[kBitsPerGroup];

    // Bit decisions, one message byte at a time. The first bit of the downlink format tells the length: DF16 and up
    // are 112 bits.
    uint32_t len_bits = kMaxMessageBits;
    uint32_t available_samples = num_words * 32 - offset;
    uint32_t first_sample = offset;
    for (uint32_t bits_done = 0; bits_done < len_bits; bits_done += kBitsPerGroup, first_sample += kSamplesPerGroup) {
        WeightedMargins(samples, first_sample, margins);
        uint32_t byte = 0;
        for (uint32_t i = 0; i < kBitsPerGroup; i++) {
            byte = (byte << 1) | (margins[i] > 0);
        }
        result.buffer[bits_done >> 5] |= byte << (24 - (bits_done & 31));
        if (bits_done == 0) {
            if (!(byte & 0x80)) {
                len_bits = kShortMessageBits;
            }
            if (len_bits * kSamplesPerBit > available_samples) {
                result.buffer[0] = 0;
                return kNoMessage;  // Capture was cut short but the message is long.
            }
        }
    }
    result.len_bits = static_cast<uint16_t>(len_bits);
    result.start_offset_samples = static_cast<uint8_t>(offset);

    // CRC guided fix for formats whose parity covers the message alone.
    uint32_t downlink_format = result.buffer[0] >> 27;
    if (downlink_format != 11 && downlink_format != 17 && downlink_format != 18) {
        return kNoCRC;
    }
    uint32_t remainder = CRC24Remainder(result.buffer, len_bits);
    if (remainder == 0 || (downlink_format == 11 && remainder < 0x80)) {
        return kCRCPassed;  // Intact (DF11 replies carry an interrogator code in the low 7 bits).
    }
    // The CRC failed: go over the message again and collect the least confident bits, by confidence and then position.
    // Margins are odd, so a weak bit's confidence is one of kNumWeakLevels levels: each level keeps its first bits.
    static constexpr uint32_t kNumWeakLevels = (kWeakBitMaxConfidence + 1) / 2;
    uint8_t level_bits[kNumWeakLevels][kMaxWeakBits];
    uint8_t level_counts[kNumWeakLevels] = {0};
    first_sample = offset;
    for (uint32_t bits_done = 0; bits_done < len_bits; bits_done += kBitsPerGroup, first_sample += kSamplesPerGroup) {
        WeightedMargins(samples, first_sample, margins);
        for (uint32_t i = 0; i < kBitsPerGroup; i++) {
            uint32_t confidence = static_cast<uint32_t>(margins[i] < 0 ? -margins[i] : margins[i]);
            if (confidence <= kWeakBitMaxConfidence) {
                uint32_t level = confidence >> 1;
                if (level_counts[level] < kMaxWeakBits) {
                    level_bits[level][level_counts[level]++] = static_cast<uint8_t>(bits_done + i);
                }
            }
        }
    }
    uint16_t weak_bits[kMaxWeakBits];
    uint16_t num_weak_bits = 0;
    for (uint32_t level = 0; level < kNumWeakLevels && num_weak_bits < kMaxWeakBits; level++) {
        for (uint32_t k = 0; k < level_counts[level] && num_weak_bits < kMaxWeakBits; k++) {
            weak_bits[num_weak_bits++] = level_bits[level][k];
        }
    }
    auto syndrome = [len_bits](uint16_t bit_index) { return kBitSyndromes.syndrome[len_bits - 1 - bit_index]; };
    auto flip = [&result](uint16_t bit_index) { result.buffer[bit_index >> 5] ^= 1u << (31 - (bit_index & 31)); };
    // A DF11 may be left with an interrogator code in its parity; other formats must come out exact.
    uint32_t max_left = downlink_format == 11 ? kMaxInterrogatorCode : 0;
    for (uint16_t a = 0; a < num_weak_bits; a++) {
        if ((syndrome(weak_bits[a]) ^ remainder) <= max_left) {
            flip(weak_bits[a]);
            result.num_flipped_bits = 1;
            return kCRCPassed;
        }
    }
    if (kMaxFlippedBits >= 2) {
        for (uint16_t a = 0; a < num_weak_bits; a++) {
            for (uint16_t b = a + 1; b < num_weak_bits; b++) {
                if ((syndrome(weak_bits[a]) ^ syndrome(weak_bits[b]) ^ remainder) <= max_left) {
                    flip(weak_bits[a]);
                    flip(weak_bits[b]);
                    result.num_flipped_bits = 2;
                    return kCRCPassed;
                }
            }
        }
    }
    return kCRCFailed;  // Not fixable here: the decoder decides what to do with it.
}

static inline bool IsAddressParityFormat(uint32_t downlink_format) {
    return downlink_format == 0 || downlink_format == 4 || downlink_format == 5 || downlink_format == 16 ||
           downlink_format == 20 || downlink_format == 21;
}

// Whether a message checks out. Without known_address, an address/parity message counts only if unchecked_ok.
static bool SOFT_DEMOD_FUNC(Validated)(const Result& result, Check check, KnownAddressFn known_address,
                                       bool unchecked_ok) {
    if (check != kNoCRC) {
        return check == kCRCPassed;
    }
    if (!IsAddressParityFormat(result.buffer[0] >> 27)) {
        return false;  // A format without parity: likely misread.
    }
    if (known_address == nullptr) {
        return unchecked_ok;
    }
    return known_address(CRC24Remainder(result.buffer, result.len_bits));
}

static bool SOFT_DEMOD_FUNC(DemodulateBits)(const uint32_t* samples, uint16_t num_words, Result& result, bool retry,
                                            KnownAddressFn known_address) {
    OffsetScores scores;
    ScoreOffsets(samples, kMinStartOffsetSamples, kMaxStartOffsetSamples, scores);

    // Timing: keep the start offset whose first bits are the most confident.
    uint32_t best_offset = kNominalStartOffsetSamples;
    uint32_t best_confidence = 0;
    uint32_t best_ties = kTimingSearchBits;
    for (uint32_t offset = kMinStartOffsetSamples; offset <= kMaxStartOffsetSamples; offset++) {
        if (scores.confidence[offset] > best_confidence) {
            best_confidence = scores.confidence[offset];
            best_offset = offset;
            best_ties = scores.ties[offset];
        }
    }
    if (best_ties > kMaxTiesInSearchBits) {
        result = Result();
        return false;  // Noise or the tail of another message.
    }

    Check check = DecodeAt(samples, num_words, best_offset, result);
    if (!retry || (check != kNoMessage && Validated(result, check, known_address, true))) {
        return check != kNoMessage;
    }

    // Retry at the next most confident offsets near the first pick. A first pick that reads a format without parity,
    // or fails its CRC, may sit on the last preamble pulse: then one chip later too.
    bool chip_later = check == kNoMessage || check == kCRCFailed ||
                      (check == kNoCRC && !IsAddressParityFormat(result.buffer[0] >> 27));
    ScoreOffsets(samples, kMaxStartOffsetSamples + 1, kMaxRetryOffsetSamples, scores);
    bool tried[kMaxRetryOffsetSamples + 1] = {false};
    tried[best_offset] = true;
    uint8_t num_retries = 0;
    Result trial;
    while (num_retries < kMaxRetries) {
        uint32_t offset = 0;
        uint32_t confidence = 0;
        for (uint32_t o = kMinStartOffsetSamples; o <= kMaxRetryOffsetSamples; o++) {
            uint32_t shift = o > best_offset ? o - best_offset : best_offset - o;
            if (shift > kMaxRetryShiftSamples && !(chip_later && o == best_offset + kSamplesPerChip)) {
                continue;
            }
            if (!tried[o] && scores.ties[o] <= kMaxTiesInSearchBits && scores.confidence[o] > confidence) {
                offset = o;
                confidence = scores.confidence[o];
            }
        }
        if (offset == 0) {
            break;
        }
        tried[offset] = true;
        num_retries++;
        Check trial_check = DecodeAt(samples, num_words, offset, trial);
        if (trial_check != kNoMessage && Validated(trial, trial_check, known_address, false)) {
            result = trial;
            result.num_retries = num_retries;
            result.retry_validated = true;
            return true;
        }
    }
    result.num_retries = num_retries;
    return check != kNoMessage;
}

bool SOFT_DEMOD_FUNC(Demodulate)(const uint32_t* samples, uint16_t num_words, Result& result, bool retry,
                                 KnownAddressFn known_address) {
    if (!DemodulateBits(samples, num_words, result, retry, known_address)) {
        return false;
    }
    result.data_start_48mhz_counts = DataStart48MHzCounts(samples, result);
    return true;
}

}  // namespace mode_s_soft_demod
