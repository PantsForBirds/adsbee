#pragma once

// LR2021 on-off keying (OOK) detector patterns and automatic gain control (AGC) settings for Mode S. Header-only
// with no SDK dependencies, so the host tests can check the pattern math.
//
// Mode S: 16 preamble chips of 0.5 us (pulses at chips 0, 2, 7 and 9), then data bits of two chips each
// (1 -> "10", 0 -> "01").
//
// Detector patterns are LSB-first (bit 0 is the first chip received) and their first two chips must differ
// (datasheet §16.3.8). A pattern ending at chip 15 makes the capture start at the first data bit.

#include <cstdint>

namespace LR2021OokAdsb {

static constexpr uint16_t kPreambleNumChips = 16;
// Preamble chips 0-15, LSB-first.
static constexpr uint16_t kPreambleChips = 0b0000001010000101;

// Detector pattern made of preamble chips first_chip..last_chip (LSB-first).
constexpr uint16_t PreambleChipsPattern(uint16_t first_chip, uint16_t last_chip) {
    return static_cast<uint16_t>((kPreambleChips >> first_chip) & ((1u << (last_chip - first_chip + 1)) - 1u));
}

// True if the pattern of preamble chips first_chip..last_chip starts with a transition (01 or 10), as
// SetOokDetector requires.
constexpr bool PreambleChipsPatternIsValid(uint16_t first_chip, uint16_t last_chip) {
    return last_chip < kPreambleNumChips && first_chip < last_chip &&
           (((kPreambleChips >> first_chip) & 1u) != ((kPreambleChips >> (first_chip + 1)) & 1u));
}

// Standard detector: the whole preamble (chips 0-15).
static constexpr uint16_t kModeSPatternFirstChip = 0;
static constexpr uint16_t kModeSPatternLastChip = 15;
static constexpr uint16_t kModeSPattern = PreambleChipsPattern(kModeSPatternFirstChip, kModeSPatternLastChip);
static constexpr uint8_t kModeSPatternLenChips = kModeSPatternLastChip - kModeSPatternFirstChip + 1;

static_assert(kModeSPattern == 0x0285, "Standard pattern must match the Semtech ADS-B pattern 0x285.");
static_assert(PreambleChipsPatternIsValid(kModeSPatternFirstChip, kModeSPatternLastChip),
              "Standard pattern must start with a transition.");

// DF17 detector: preamble chips 8-15, then the first four downlink format (DF) bits "1000" (16 chips, the maximum).
// The capture starts at message bit 4; the RX path puts the four consumed bits back in front.
//
// Six quiet chips in a row never occur inside data, so this pattern only fires at the end of a preamble. That keeps
// extra captures from overflowing the 256-byte FIFO in dense traffic. About a quarter of frames still latch later
// inside the frame, which software can't recover.
static constexpr uint16_t kDF17PatternFirstChip = 8;
static constexpr uint8_t kDF17HeaderBits = 0b1000;  // The first four DF bits (shared by DF 16 and 17), MSB first.
static constexpr uint8_t kDF17HeaderLenBits = 4;
// Chips of the first num_bits message bits of value `bits` (MSB first), LSB-first pattern layout.
constexpr uint16_t MessageBitsPattern(uint32_t bits, uint8_t num_bits) {
    uint16_t pattern = 0;
    for (uint8_t i = 0; i < num_bits; i++) {
        const bool bit = (bits >> (num_bits - 1 - i)) & 1u;
        pattern |= static_cast<uint16_t>((bit ? 0b01u : 0b10u) << (2 * i));  // 1 -> chips 1,0; 0 -> chips 0,1.
    }
    return pattern;
}
static constexpr uint8_t kDF17PatternLenChips = (kPreambleNumChips - kDF17PatternFirstChip) + 2 * kDF17HeaderLenBits;
static constexpr uint16_t kDF17Pattern = static_cast<uint16_t>(
    PreambleChipsPattern(kDF17PatternFirstChip, kPreambleNumChips - 1) |
    (MessageBitsPattern(kDF17HeaderBits, kDF17HeaderLenBits) << (kPreambleNumChips - kDF17PatternFirstChip)));
static_assert(kDF17Pattern == 0xA902 && kDF17PatternLenChips == 16,
              "DF17 pattern: preamble chips 8-15 + DF bits 1000 (the 0.3.11-rc3 pattern).");
static_assert(PreambleChipsPatternIsValid(kDF17PatternFirstChip, kPreambleNumChips - 1),
              "DF17 pattern must start with a transition.");
// Rebuilds a DF17-mode capture into the 112-bit frame as 32-bit words, MSB first (RawModeSPacket layout): the
// consumed DF bits, then the capture. Bits past the end are 0.
inline void ReconstructDF17Frame(const uint8_t* capture, uint16_t capture_len_bytes, uint32_t* words,
                                 uint16_t num_words) {
    uint32_t prev = static_cast<uint32_t>(kDF17HeaderBits) << (32 - kDF17HeaderLenBits);
    for (uint16_t k = 0; k < num_words; k++) {
        uint32_t w = 0;  // Capture bytes 4k .. 4k+3, MSB first; 0 past the end of the capture.
        for (uint16_t b = 0; b < 4; b++) {
            const uint16_t i = static_cast<uint16_t>(4 * k + b);
            if (i < capture_len_bytes) w |= static_cast<uint32_t>(capture[i]) << (24 - 8 * b);
        }
        words[k] = prev | (w >> kDF17HeaderLenBits);
        prev = w << (32 - kDF17HeaderLenBits);
    }
}

// The LR2021 rejects odd-length detector patterns (CMD_PERR), which leaves the receiver unconfigured.
static_assert(kModeSPatternLenChips % 2 == 0 && kDF17PatternLenChips % 2 == 0,
              "Detector patterns must have an even number of chips.");

// MODE_S runs at a fixed gain step (AGC off); 13 is the highest.
static constexpr uint8_t kModeSGainStep = 13;

// SetAgcGainManual step for MODE_S: kModeSGainStep unless a manual gain is set.
constexpr uint8_t ModeSGainStep(uint8_t agc_gain) { return agc_gain == 0 ? kModeSGainStep : agc_gain; }

// Undocumented front-end register (found by bench testing). Bits 7:0 select how the receiver reacts to a strong
// input: the chip default 0x75 runs the AGC and blanks the demodulator while the ADC saturates; 0x18 turns both off,
// so a fixed high gain still decodes pulses that saturate the ADC.
static constexpr uint32_t kFrontEndRegAddr = 0xF30150;
static constexpr uint32_t kFrontEndModeMask = 0x000000FF;
static constexpr uint8_t kFrontEndModeNoSaturationBlanking = 0x18;

}  // namespace LR2021OokAdsb
