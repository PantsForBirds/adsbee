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
// The LR2021 rejects a detector pattern of odd length (CMD_PERR; 11 and 15 chips checked on a 1421, while 10, 12
// and 16 are accepted), and a rejected config leaves the receiver unconfigured.
static_assert(kModeSPatternLenChips % 2 == 0 && kStrongPatternLenChips % 2 == 0 && kDF17PatternLenChips % 2 == 0,
              "Detector patterns must have an even number of chips.");
static_assert((kDF17Pattern & 1u) != ((kDF17Pattern >> 1) & 1u), "DF17 pattern must start with a transition.");

// Realignment of a DF17-mode capture. The capture holds message bits s .. s+111 for an unknown shift s;
// the nominal shift is kDF17HeaderLenBits. Only the shifts real captures show are tried, nominal first, then
// the others most frequent first. Across the t-0080 bench captures (Pluto, -92..0 dBm, every experiment with
// this pattern) the capture start was bit 5 (3261), -2 (1532, the preamble alias from about -50 dBm up), 4 (737),
// 3 (539) and 6 (491); bits 1, 8 and -1 turned up 74 times or less and mostly with errors, 0, 2 and 7 hardly at
// all. Each shift left out saves CPU on every capture that matches nothing and removes candidates that a
// garbage capture could falsely match: 64 candidates in all (8 per shift, 32 at shift -2 with its two unknown
// bits), so about 64 / 2^24 false frames per garbage capture.
static constexpr int8_t kDF17Shifts[] = {5, -2, 4, 3, 6};
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

// Mode S parity (CRC-24): the syndrome of a 112-bit frame, crc24(bits 0-87) XOR parity(bits 88-111), is the
// frame read as a polynomial modulo the generator x^24 + 0xFFF409. It is linear: flipping message bit i changes
// the syndrome by kDF17BitSyndromes.v[i] (= x^(111-i) mod G, the decoder's crc24_single_bit_syndrome_112 table).
static constexpr uint32_t kModeSCrcGenerator = 0x1FFF409;
constexpr uint32_t CrcMulX(uint32_t r) {
    r <<= 1;
    return (r & 0x1000000u) ? r ^ kModeSCrcGenerator : r;
}
// Division by x modulo the generator. Exact because the generator's constant term is 1.
constexpr uint32_t CrcDivX(uint32_t r) { return ((r & 1u) ? r ^ kModeSCrcGenerator : r) >> 1; }

struct ModeSBitSyndromes {
    uint32_t v[kModeSFrameLenBits];
};
constexpr ModeSBitSyndromes MakeModeSBitSyndromes() {
    ModeSBitSyndromes s{};
    uint32_t r = 1;  // Bit 111 is x^0.
    for (int i = kModeSFrameLenBits - 1; i >= 0; i--) {
        s.v[i] = r;
        r = CrcMulX(r);
    }
    return s;
}
static constexpr ModeSBitSyndromes kDF17BitSyndromes = MakeModeSBitSyndromes();
static_assert(kDF17BitSyndromes.v[0] == 0x3935EA && kDF17BitSyndromes.v[87] == 0xFFF409 &&
                  kDF17BitSyndromes.v[88] == 0x800000,
              "Bit syndromes must match crc24_single_bit_syndrome_112.");

// Prefilter for the decoder's single-bit correction (crc24_find_single_bit_error, a linear search of the 112 bit
// syndromes): bit b of the 4096 is set when some single-bit syndrome has low 12 bits b. A syndrome whose bit is
// clear can't be a single-bit error, which is the answer for about 97 % of failed frames (noise, garbled or
// overlapping captures), without the search.
struct ModeSSingleBitFilter {
    uint32_t bits[4096 / 32];
    bool MayMatch(uint32_t syndrome) const { return (bits[(syndrome >> 5) & 127u] >> (syndrome & 31u)) & 1u; }
};
constexpr ModeSSingleBitFilter MakeModeSSingleBitFilter() {
    ModeSSingleBitFilter f{};
    for (uint16_t i = 0; i < kModeSFrameLenBits; i++) {
        const uint32_t r = kDF17BitSyndromes.v[i];
        f.bits[(r >> 5) & 127u] |= 1u << (r & 31u);
    }
    return f;
}
static constexpr ModeSSingleBitFilter kModeSSingleBitFilter = MakeModeSSingleBitFilter();

constexpr uint32_t DF17HeaderSyndrome() {
    uint32_t r = 0;
    for (uint8_t i = 0; i < kDF17HeaderLenBits; i++) {
        if ((kDF17HeaderBits >> (kDF17HeaderLenBits - 1 - i)) & 1u) r ^= kDF17BitSyndromes.v[i];
    }
    return r;
}
static constexpr uint32_t kDF17HeaderSyndrome = DF17HeaderSyndrome();

// What RecoverDF17Frame tries at one shift: the message bits the capture doesn't hold (tried both ways), the CA
// bits (tried flipped, unless already unknown), and the syndrome of every combination, in the order the
// candidates are tried (candidate v: bit u of v sets unknown[u], bit num_unknown + j flips flips[j]).
//
// Plus what turns the capture's syndrome into this shift's syndrome in a few table lookups (DF17ShiftSyndrome):
// the multiplication by x^-shift, and the contribution of the capture's first 7 bits (the ones that land in the
// DF field, or fall off the start, depending on the shift), and a 256-bit filter on the low syndrome byte that
// rejects almost every non-matching shift before the candidate list is looked at.
struct DF17ShiftPlan {
    int8_t shift = 0;
    uint8_t num_unknown = 0;
    uint8_t unknown[2] = {0, 0};
    uint8_t num_flips = 0;
    uint8_t flips[kDF17FlipNumBits] = {0, 0, 0};
    uint8_t num_candidates = 0;
    uint32_t candidate_syndrome[32] = {};
    uint32_t candidate_filter[8] = {};  // Bit b of the 256: some candidate syndrome has low byte b.
    uint32_t mul_low[128] = {};         // shift > 0: x^-shift times the low `shift` bits. shift < 0: x^-shift
                                        // times the top -shift bits (24 - (-shift) .. 23).
    uint32_t head_hi[16] = {};          // Correction for capture bits 0-3 (capture[0] >> 4) at this shift.
    uint32_t head_lo[8] = {};           // Correction for capture bits 4-6 ((capture[0] >> 1) & 7).
};
static constexpr uint8_t kDF17NumShifts = sizeof(kDF17Shifts) / sizeof(kDF17Shifts[0]);
struct DF17ShiftPlans {
    DF17ShiftPlan plan[kDF17NumShifts];
};
constexpr uint32_t CrcMulXPow(uint32_t r, int8_t n) {  // r * x^n mod G, for n of either sign.
    for (int8_t k = 0; k < n; k++) r = CrcMulX(r);
    for (int8_t k = 0; k > n; k--) r = CrcDivX(r);
    return r;
}
// What capture bit c contributes to the syndrome of the frame at `shift` (before the DF bits are added): nothing
// if it lands at message bit 5 or later (it is part of the shifted capture), the removal of its value if it lands
// in the DF field (message bit c + shift < 5), and the removal of its value from the shifted capture if it falls
// off the start (c + shift < 0: its x^(111 - c) term, now x^(111 - c + (-shift)), is not in the frame).
constexpr uint32_t DF17CaptureBitCorrection(int8_t shift, uint8_t c) {
    const int16_t i = static_cast<int16_t>(c) + shift;  // Message bit that capture bit c lands on.
    if (i >= static_cast<int16_t>(kDF17HeaderLenBits)) return 0;
    // Its term in the shifted capture: x^(111 - c) * x^-shift. Removing it covers both cases (a DF bit is then
    // set from kDF17HeaderSyndrome).
    return CrcMulXPow(kDF17BitSyndromes.v[c], static_cast<int8_t>(-shift));
}
constexpr DF17ShiftPlans MakeDF17ShiftPlans() {
    DF17ShiftPlans p{};
    for (uint8_t k = 0; k < kDF17NumShifts; k++) {
        DF17ShiftPlan& plan = p.plan[k];
        plan.shift = kDF17Shifts[k];
        for (uint16_t i = kDF17HeaderLenBits; i < kModeSFrameLenBits; i++) {
            const int16_t c = static_cast<int16_t>(i) - plan.shift;  // Capture bit holding message bit i.
            if (c < 0 || c >= static_cast<int16_t>(kModeSFrameLenBits)) {
                plan.unknown[plan.num_unknown++] = static_cast<uint8_t>(i);
            }
        }
        for (uint8_t b = kDF17FlipFirstBit; b < kDF17FlipFirstBit + kDF17FlipNumBits; b++) {
            bool is_unknown = false;
            for (uint8_t u = 0; u < plan.num_unknown; u++) is_unknown |= plan.unknown[u] == b;
            if (!is_unknown) plan.flips[plan.num_flips++] = b;
        }
        plan.num_candidates = static_cast<uint8_t>(1u << (plan.num_unknown + plan.num_flips));
        for (uint8_t v = 0; v < plan.num_candidates; v++) {
            uint32_t r = 0;
            for (uint8_t u = 0; u < plan.num_unknown; u++) {
                if ((v >> u) & 1u) r ^= kDF17BitSyndromes.v[plan.unknown[u]];
            }
            for (uint8_t j = 0; j < plan.num_flips; j++) {
                if ((v >> (plan.num_unknown + j)) & 1u) r ^= kDF17BitSyndromes.v[plan.flips[j]];
            }
            plan.candidate_syndrome[v] = r;
            plan.candidate_filter[(r >> 5) & 7u] |= 1u << (r & 31u);
        }
        // x^-shift of the part of a 24-bit value that doesn't shift out cleanly: the low `shift` bits (shift > 0)
        // or the top -shift bits (shift < 0). The rest is a plain shift.
        const uint8_t n = static_cast<uint8_t>(plan.shift >= 0 ? plan.shift : -plan.shift);
        for (uint32_t low = 0; low < (1u << n); low++) {
            plan.mul_low[low] = plan.shift >= 0 ? CrcMulXPow(low, static_cast<int8_t>(-plan.shift))
                                                : CrcMulXPow(low << (24 - n), static_cast<int8_t>(-plan.shift));
        }
        for (uint8_t v = 0; v < 16; v++) {
            for (uint8_t b = 0; b < 4; b++) {
                if ((v >> (3 - b)) & 1u) plan.head_hi[v] ^= DF17CaptureBitCorrection(plan.shift, b);
            }
        }
        for (uint8_t v = 0; v < 8; v++) {
            for (uint8_t b = 0; b < 3; b++) {
                if ((v >> (2 - b)) & 1u) plan.head_lo[v] ^= DF17CaptureBitCorrection(plan.shift, 4 + b);
            }
        }
    }
    return p;
}
static constexpr DF17ShiftPlans kDF17ShiftPlans = MakeDF17ShiftPlans();
static_assert(kDF17Shifts[0] == kDF17HeaderLenBits, "The nominal shift is tried first (and is the fallback).");
static_assert(kDF17ShiftPlans.plan[0].num_unknown == 0, "The nominal shift leaves no message bit unknown.");
// DF17CaptureBitCorrection only looks at capture bits 0-6: every shift must put bit 7 past the DF field and
// keep bits 0-6 within reach of the head tables.
static_assert(kDF17HeaderLenBits - (-2) <= 7, "Capture bits past 6 would need a head correction.");

// Syndrome of the frame the capture gives at `shift` (DF bits set, unknown bits 0), from the capture's own
// syndrome: a shift is a multiplication by x^-shift, after taking out the capture bits that fall off the end
// (shift > 0), then the capture bits that land in the DF field or fall off the start are taken out and the DF
// bits put in. About a dozen instructions per shift.
inline uint32_t DF17ShiftSyndrome(const uint8_t* capture, uint32_t capture_syndrome, const DF17ShiftPlan& plan) {
    uint32_t r;
    if (plan.shift >= 0) {
        // Capture bits 112 - shift .. 111 (the low bits of the last byte) are past the frame: drop them, then
        // divide by x^shift. The dropped bits make the value an exact multiple of x^shift below bit `shift`, so
        // the division is (r >> shift) plus a table for the low bits of the value as it was.
        const uint32_t mask = (1u << plan.shift) - 1u;
        const uint32_t v = capture_syndrome ^ (capture[kModeSFrameLenBytes - 1] & mask);
        r = (v >> plan.shift) ^ plan.mul_low[v & mask];
    } else {
        const uint8_t n = static_cast<uint8_t>(-plan.shift);
        r = ((capture_syndrome << n) & 0xFFFFFFu) ^ plan.mul_low[capture_syndrome >> (24 - n)];
    }
    return r ^ plan.head_hi[capture[0] >> 4] ^ plan.head_lo[(capture[0] >> 1) & 7u] ^ kDF17HeaderSyndrome;
}

// Writes candidate v of `plan`: the capture moved by plan.shift bits (word operations), DF = 17, the unknown
// bits set and the CA bits flipped as v says.
inline void BuildDF17Frame(const uint8_t* capture, const DF17ShiftPlan& plan, uint8_t v, uint8_t* frame_out) {
    uint32_t w[4] = {0, 0, 0, 0};  // Capture bits 0-111, MSB first; bits 112-127 are 0.
    for (uint8_t k = 0; k < kModeSFrameLenBytes; k++) {
        w[k / 4] |= static_cast<uint32_t>(capture[k]) << (24 - 8 * (k % 4));
    }
    uint32_t f[4];
    if (plan.shift > 0) {  // Message bit i = capture bit i - shift: move towards the end.
        const uint8_t n = static_cast<uint8_t>(plan.shift);
        f[0] = w[0] >> n;
        for (uint8_t k = 1; k < 4; k++) f[k] = (w[k] >> n) | (w[k - 1] << (32 - n));
    } else if (plan.shift < 0) {  // Move towards the start.
        const uint8_t n = static_cast<uint8_t>(-plan.shift);
        for (uint8_t k = 0; k < 3; k++) f[k] = (w[k] << n) | (w[k + 1] >> (32 - n));
        f[3] = w[3] << n;
    } else {
        for (uint8_t k = 0; k < 4; k++) f[k] = w[k];
    }
    f[0] = (f[0] & (0xFFFFFFFFu >> kDF17HeaderLenBits)) |
           (static_cast<uint32_t>(kDF17HeaderBits) << (32 - kDF17HeaderLenBits));
    for (uint8_t u = 0; u < plan.num_unknown; u++) {
        const uint8_t i = plan.unknown[u];
        if ((v >> u) & 1u) f[i / 32] |= 0x80000000u >> (i % 32);  // Unknown bits start out 0.
    }
    for (uint8_t j = 0; j < plan.num_flips; j++) {
        const uint8_t i = plan.flips[j];
        if ((v >> (plan.num_unknown + j)) & 1u) f[i / 32] ^= 0x80000000u >> (i % 32);
    }
    for (uint8_t k = 0; k < kModeSFrameLenBytes; k++) {
        frame_out[k] = static_cast<uint8_t>(f[k / 4] >> (24 - 8 * (k % 4)));
    }
}

// Rebuilds a 112-bit DF17 frame from a DF17-mode capture (kModeSFrameLenBytes bytes, MSB first). For each
// shift: message bits 0-4 are set to DF=17, bits the capture doesn't hold (CA bits before a late capture, the
// last bits after an early one) are tried both ways, and so are the three CA bits. The first candidate with
// a matching parity wins. crc24(buf, len) must return the Mode S CRC of len bytes. Returns the shift used, or
// INT8_MIN (frame_out = the nominal reconstruction) if no candidate matched; the decoder then gets the frame
// the old way.
//
// Cost: the candidates are the same as in the first version (target_test/df17_recover_reference.hh, which this
// matches bit for bit over the same kDF17Shifts; host_test checks it), but no candidate is built or CRC'd. The CRC is linear, so one
// CRC of the capture gives every shift's syndrome by a few table lookups, and a candidate matches when its
// precomputed syndrome (kDF17ShiftPlans) equals that. The frame is built once, for the winner.
template <typename Crc24Fn>
int8_t RecoverDF17Frame(const uint8_t* capture, uint8_t* frame_out, Crc24Fn crc24) {
    const uint32_t parity = (static_cast<uint32_t>(capture[kModeSFrameLenBytes - 3]) << 16) |
                            (static_cast<uint32_t>(capture[kModeSFrameLenBytes - 2]) << 8) |
                            capture[kModeSFrameLenBytes - 1];
    const uint32_t capture_syndrome = crc24(capture, kModeSFrameLenBytes - 3) ^ parity;
    for (const DF17ShiftPlan& plan : kDF17ShiftPlans.plan) {
        const uint32_t syndrome = DF17ShiftSyndrome(capture, capture_syndrome, plan);
        if (!((plan.candidate_filter[(syndrome >> 5) & 7u] >> (syndrome & 31u)) & 1u)) continue;
        for (uint8_t v = 0; v < plan.num_candidates; v++) {
            if (plan.candidate_syndrome[v] == syndrome) {
                BuildDF17Frame(capture, plan, v, frame_out);
                return plan.shift;
            }
        }
    }
    // No match: the nominal reconstruction (DF bits + capture).
    BuildDF17Frame(capture, kDF17ShiftPlans.plan[0], 0, frame_out);
    return INT8_MIN;
}

// The nominal reconstruction alone (what RecoverDF17Frame returns when nothing matches), for captures the CPU
// budget leaves unrealigned.
inline void NominalDF17Frame(const uint8_t* capture, uint8_t* frame_out) {
    BuildDF17Frame(capture, kDF17ShiftPlans.plan[0], 0, frame_out);
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
