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

// DF17 detector: the five DF bits "10001" of the message (10 chips "10 01 01 01 10"), nothing from the
// preamble. The capture starts at message bit 5.
//
// Why no preamble chips: at a weak level (no AGC action) the LR2021 accepts a window about one chip away
// from the pattern, and any pattern made of chips 6-15 near-matches the first pulse pair (the preamble is two
// copies of 1010000), so it fires early and the real frame is lost. From about -53 dBm up the AGC blanks
// chips 0-8. Patterns that end in the preamble or just after it (the old chips 8-15 + DF bits "1000") lose
// about 20 % of the frames at every level: the detector then latches later inside the data (capture from
// message bit 24, 41 or 70). The DF bits sit after both effects.
//
// What the capture looks like (Pluto bench, 50 frames per level, t-0080): up to -60 dBm it starts at bit 5
// (98-100 % from -84 dBm). From about -60 dBm up the start moves by whole bits: bit -2, 3, 4, 6 or 7 (the
// detector fires early or late while the AGC settles), and bits 0-7 can carry errors. RecoverDF17Frame() realigns such captures against the known DF bits and the CRC.
static constexpr uint8_t kDF17HeaderBits = 0b10001;  // DF = 17, MSB first.
static constexpr uint8_t kDF17HeaderLenBits = 5;
// Chips of the first num_bits message bits of value `bits` (MSB first), LSB-first pattern layout.
constexpr uint16_t MessageBitsPattern(uint32_t bits, uint8_t num_bits) {
    uint16_t pattern = 0;
    for (uint8_t i = 0; i < num_bits; i++) {
        const bool bit = (bits >> (num_bits - 1 - i)) & 1u;
        pattern |= static_cast<uint16_t>((bit ? 0b01u : 0b10u) << (2 * i));  // 1 -> chips 1,0; 0 -> chips 0,1.
    }
    return pattern;
}
static constexpr uint16_t kDF17Pattern = MessageBitsPattern(kDF17HeaderBits, kDF17HeaderLenBits);
static constexpr uint8_t kDF17PatternLenChips = 2 * kDF17HeaderLenBits;
static_assert(kDF17Pattern == 0x01A9, "DF17 pattern: chips 1001010110 (LSB first).");
static_assert((kDF17Pattern & 1u) != ((kDF17Pattern >> 1) & 1u), "DF17 pattern must start with a transition.");

// Realignment of a DF17-mode capture. The capture holds message bits s .. s+111 for an unknown shift s;
// the nominal shift is kDF17HeaderLenBits. Shifts are tried in this order (the nominal one first, then the
// ones seen on the bench, most frequent first):
static constexpr int8_t kDF17Shifts[] = {5, 4, 3, 2, 1, 0, -1, -2, 6, 7};
static constexpr uint16_t kModeSFrameLenBits = 112;
static constexpr uint16_t kModeSFrameLenBytes = kModeSFrameLenBits / 8;
// Bits after the DF field that are also tried flipped (the CA field): the AGC settling leaves errors in
// message bits 0-7, and bits 0-4 are known.
static constexpr uint8_t kDF17FlipFirstBit = kDF17HeaderLenBits;
static constexpr uint8_t kDF17FlipNumBits = 3;

inline bool GetMsgBit(const uint8_t* buf, uint16_t i) { return (buf[i / 8] >> (7 - i % 8)) & 1u; }
inline void SetMsgBit(uint8_t* buf, uint16_t i, bool v) {
    buf[i / 8] = static_cast<uint8_t>(v ? (buf[i / 8] | (0x80u >> (i % 8))) : (buf[i / 8] & ~(0x80u >> (i % 8))));
}

// Rebuilds a 112-bit DF17 frame from a DF17-mode capture (kModeSFrameLenBytes bytes, MSB first). For each
// shift: message bits 0-4 are set to DF=17, bits the capture doesn't hold (CA bits before a late capture, the
// last bits after an early one) are tried both ways, and so are the three CA bits. The first candidate with
// a matching parity wins. crc24(buf, len) must return the Mode S CRC of len bytes. Returns the shift used, or
// INT8_MIN (frame_out = the nominal reconstruction) if no candidate matched; the decoder then gets the frame
// the old way. Worst case (no match): 112 CRCs of 11 bytes, as many candidates as the decoder's single-bit
// correction tries.
template <typename Crc24Fn>
int8_t RecoverDF17Frame(const uint8_t* capture, uint8_t* frame_out, Crc24Fn crc24) {
    for (int8_t shift : kDF17Shifts) {
        uint8_t base[kModeSFrameLenBytes] = {0};
        uint8_t unknown[4];  // Message bits the capture doesn't hold: at most 2 late + 2 early.
        uint8_t num_unknown = 0;
        for (uint16_t i = 0; i < kModeSFrameLenBits; i++) {
            const int16_t c = static_cast<int16_t>(i) - shift;  // Capture bit holding message bit i.
            if (i < kDF17HeaderLenBits) {
                SetMsgBit(base, i, (kDF17HeaderBits >> (kDF17HeaderLenBits - 1 - i)) & 1u);
            } else if (c < 0 || c >= static_cast<int16_t>(kModeSFrameLenBits)) {
                unknown[num_unknown++] = static_cast<uint8_t>(i);
            } else {
                SetMsgBit(base, i, GetMsgBit(capture, static_cast<uint16_t>(c)));
            }
        }
        // Flip bits: the CA bits, unless they are already unknown (then trying both covers them).
        uint8_t flips[kDF17FlipNumBits];
        uint8_t num_flips = 0;
        for (uint8_t b = kDF17FlipFirstBit; b < kDF17FlipFirstBit + kDF17FlipNumBits; b++) {
            bool is_unknown = false;
            for (uint8_t u = 0; u < num_unknown; u++) is_unknown |= unknown[u] == b;
            if (!is_unknown) flips[num_flips++] = b;
        }
        const uint16_t num_candidates = static_cast<uint16_t>(1u << (num_unknown + num_flips));
        for (uint16_t v = 0; v < num_candidates; v++) {
            uint8_t f[kModeSFrameLenBytes];
            for (uint16_t k = 0; k < kModeSFrameLenBytes; k++) f[k] = base[k];
            for (uint8_t u = 0; u < num_unknown; u++) SetMsgBit(f, unknown[u], (v >> u) & 1u);
            for (uint8_t j = 0; j < num_flips; j++) {
                if ((v >> (num_unknown + j)) & 1u) SetMsgBit(f, flips[j], !GetMsgBit(f, flips[j]));
            }
            const uint32_t parity = (static_cast<uint32_t>(f[11]) << 16) | (static_cast<uint32_t>(f[12]) << 8) | f[13];
            if (crc24(f, kModeSFrameLenBytes - 3) == parity) {
                for (uint16_t k = 0; k < kModeSFrameLenBytes; k++) frame_out[k] = f[k];
                return shift;
            }
        }
    }
    // No match: the nominal reconstruction (DF bits + capture).
    uint8_t f[kModeSFrameLenBytes] = {0};
    for (uint16_t i = 0; i < kModeSFrameLenBits; i++) {
        SetMsgBit(f, i, i < kDF17HeaderLenBits ? ((kDF17HeaderBits >> (kDF17HeaderLenBits - 1 - i)) & 1u)
                                               : GetMsgBit(capture, i - kDF17HeaderLenBits));
    }
    for (uint16_t k = 0; k < kModeSFrameLenBytes; k++) frame_out[k] = f[k];
    return INT8_MIN;
}

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
