#pragma once

// LR2021 OOK detector patterns and AGC settings for Mode S reception. Header-only with no SDK
// dependencies, so the host tests (ti/host_test) can check the pattern math.
//
// Mode S air format: 16 preamble chips of 0.5 us (pulses at chips 0, 2, 7 and 9), then the data bits,
// each one a pair of chips (1 -> "10", 0 -> "01"; the LR2021 calls this inverted Manchester).
//
// SetOokDetector patterns are LSB-first: bit 0 is the first chip received. Their first two chips must
// differ (datasheet §16.3.8). When the pattern ends at chip 15 the packet engine starts capturing at the
// first data bit, whatever chip the pattern starts at.

#include <cstdint>

namespace LR2021OokAdsb {

static constexpr uint16_t kPreambleNumChips = 16;
// Preamble chips 0..15, LSB-first: pulses at chips 0, 2, 7 and 9.
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

// Strong-signal detector: preamble chips 6-15 (the second pulse pair and the quiet tail). A packet at
// -52 dBm or more makes the LR2021 AGC cut the gain during the preamble, and the demodulator puts out
// nothing for about 4.5 us (9 chips) while it settles. The first three pulses (chips 0, 2 and 7) never
// reach the detector, so the standard pattern can't match. This pattern still does (chip 7 is its only
// miss), and the capture starts at the first data bit. With a weak packet (no AGC action) this pattern
// syncs late inside the data instead, so it only suits strong signals.
static constexpr uint16_t kStrongPatternFirstChip = 6;
static constexpr uint16_t kStrongPatternLastChip = 15;
static constexpr uint16_t kStrongPattern = PreambleChipsPattern(kStrongPatternFirstChip, kStrongPatternLastChip);
static constexpr uint8_t kStrongPatternLenChips = kStrongPatternLastChip - kStrongPatternFirstChip + 1;

static_assert(kModeSPattern == 0x0285, "Standard pattern must match the Semtech ADS-B pattern 0x285.");
static_assert(PreambleChipsPatternIsValid(kModeSPatternFirstChip, kModeSPatternLastChip),
              "Standard pattern must start with a transition.");
static_assert(kStrongPattern == 0x000A, "Strong pattern: chips 6-15 = 0101000000 (LSB first).");
static_assert(PreambleChipsPatternIsValid(kStrongPatternFirstChip, kStrongPatternLastChip),
              "Strong pattern must start with a transition.");

// DF17 detector: the second half of the preamble (chips 8-15, "01000000": the pulse at chip 9 and the quiet tail)
// followed by the first four DF bits "1000" (8 chips "10 01 01 01"). 16 chips, the detector's maximum. The capture
// starts at message bit 4, and the RX path puts the four consumed bits back in front of it.
//
// Six quiet chips in a row never occur inside Manchester data (every bit is a 01 or 10 pair), so this pattern
// can only fire at the end of a preamble. That keeps the capture rate close to the real frame rate in dense
// traffic. On the Pluto bench (t-0094, 256-packet bursts of mixed DF11/4/5/20/21/17 at 128 us spacing, 1740
// DF17 frames per point), it decoded 525-559 DF17 frames at 5.0 captures per valid frame. A pattern of the
// five DF bits alone decoded 99-150 at 31-41 captures per valid frame: it fires inside every frame's data, and
// the extra captures overflow the 256-byte FIFO.
//
// Its known cost: about a quarter of the frames are not captured at bit 4. The LR2021 then latches later
// inside the same frame, at message bits 24, 41, 70 or 87 (t-0080, t-0094: of 1387 raw captures at -90..-20
// dBm, 917 started at bit 4, 460 at bit 8 or later, 10 near bit 4 with bit errors and none a whole bit early
// or late), so software realignment has nothing to recover.
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
// Rebuilds a DF17-mode capture into the 112-bit frame, as 32-bit words MSB first (the RawModeSPacket layout):
// the kDF17HeaderLenBits DF bits the detector consumed, then the capture (capture_len_bytes bytes). Bits past the
// header plus the capture are 0. Word shifts: about a tenth of the cost of moving the capture a few bits at a
// time.
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

// The LR2021 rejects a detector pattern of odd length (CMD_PERR; 11 and 15 chips checked on a 1421, while 10, 12
// and 16 are accepted), and a rejected config leaves the receiver unconfigured.
static_assert(kModeSPatternLenChips % 2 == 0 && kStrongPatternLenChips % 2 == 0 && kDF17PatternLenChips % 2 == 0,
              "Detector patterns must have an even number of chips.");

// LR2021 AGC configuration register. Undocumented; the fields below were characterized on the bench
// (ADSBee 1421, 1090 MHz, OOK 2 Mchip/s, 3076 kHz RX bandwidth). Bits 23:16 set the input level at which
// the AGC starts cutting the gain while a packet arrives. Bits 15:8 hold enables (values with bit 4 set
// stop reception) and bits 7:0 and 31:24 had no measurable effect, so only bits 23:16 are ever written.
static constexpr uint32_t kAgcConfigRegAddr = 0xF3014C;
static constexpr uint32_t kAgcTriggerMask = 0x00FF0000;
static constexpr uint32_t kAgcTriggerShift = 16;
// Chip default: the AGC acts from about -53 dBm. The gain change blanks the start of the preamble, so
// the standard detector misses every packet from -50 dBm up.
static constexpr uint8_t kAgcTriggerDefault = 0x10;
// Raised trigger for MODE_S (standard preamble detector): packets up to -45 dBm arrive with the gain
// unchanged and keep their whole preamble. From about -43 dBm up the receiver compresses at full gain
// and decoding falls off. Triggers 0x60, 0x7F, 0xA0 and 0xFF measured the same as 0x40 from -90 to
// 0 dBm (the same ceiling and sensitivity); 0x20 dropped to 16 % at -45 dBm. MODE_S_STRONG needs the chip
// default: with 0x40 or more it decodes at most 11 % of the packets, because it relies on the blanking.
static constexpr uint8_t kAgcTriggerStandardPreamble = 0x40;

constexpr uint32_t AgcTriggerRegValue(uint8_t trigger) {
    return (static_cast<uint32_t>(trigger) << kAgcTriggerShift) & kAgcTriggerMask;
}

// OOK detection threshold, bits 26:20 of the OOK detector register (Semtech LR20xx driver,
// lr20xx_workarounds_ook_set_detection_threshold_level: field = level_dB + 74, 7 bits). The chip
// computes it from the RX bandwidth (0x61, about -105 dB at 3076 kHz). Bits 3:0 of the same register
// hold the pattern length, so only the threshold field is ever written.
static constexpr uint32_t kOokDetectRegAddr = 0xF30E14;
static constexpr uint32_t kOokDetectThresholdMask = 0x07F00000;
static constexpr uint32_t kOokDetectThresholdShift = 20;
// MODE_S_STRONG threshold (about -70 dB by the driver's formula). The short strong-signal pattern
// matches noise at the chip threshold (about 40 triggers/s with no signal, enough to trip the validity
// watchdog), and right after a -20 dBm packet the reduced AGC gain let noise trigger about three more
// captures. With this threshold both go to zero while packets from -50 dBm up decode as before.
static constexpr uint8_t kOokDetectThresholdStrong = 0x04;

constexpr uint32_t OokDetectThresholdRegValue(uint8_t threshold) {
    return (static_cast<uint32_t>(threshold) << kOokDetectThresholdShift) & kOokDetectThresholdMask;
}

}  // namespace LR2021OokAdsb
