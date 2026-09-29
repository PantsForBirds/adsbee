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
// Raised trigger for the standard preamble detector: packets up to -45 dBm arrive with the gain
// unchanged and keep their whole preamble. From about -43 dBm up the receiver compresses at full gain
// and decoding falls off; raising the trigger further (0x48, 0x50) measured the same.
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
