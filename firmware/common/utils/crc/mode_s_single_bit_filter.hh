#pragma once

// Prefilter for the Mode S decoder's single-bit error correction. Header-only with no SDK dependencies, so it can be
// shared by every product and the host tests check it against the firmware's own syndrome table (crc_tables.hh).
//
// Mode S parity (CRC-24): the syndrome of a 112-bit frame, crc24(bits 0-87) XOR parity(bits 88-111), is the
// frame read as a polynomial modulo the generator x^24 + 0xFFF409. Flipping message bit i changes the syndrome
// by kBitSyndromes.v[i] = x^(111-i) mod G (the values of crc24_single_bit_syndrome_112).
//
// crc24_find_single_bit_error searches those 112 values linearly for every frame that fails its CRC. The filter
// holds one bit per low-12-bit syndrome value: a syndrome whose bit is clear can't be a single-bit error. The 112
// syndromes set 98 of the 4096 bits, so a random syndrome gets through with a probability of 2.4 % and most failed
// frames (noise, garbled and overlapping captures) skip the search. The result is the same either way.

#include <cstdint>

namespace ModeSSingleBitFilter {

static constexpr uint16_t kFrameLenBits = 112;
static constexpr uint32_t kCrcGenerator = 0x1FFF409;  // x^24 + 0xFFF409.

struct BitSyndromes {
    uint32_t v[kFrameLenBits];
};
constexpr BitSyndromes MakeBitSyndromes() {
    BitSyndromes s{};
    uint32_t r = 1;  // Bit 111 is x^0.
    for (int i = kFrameLenBits - 1; i >= 0; i--) {
        s.v[i] = r;
        r <<= 1;
        if (r & 0x1000000u) r ^= kCrcGenerator;
    }
    return s;
}
static constexpr BitSyndromes kBitSyndromes = MakeBitSyndromes();
static_assert(kBitSyndromes.v[0] == 0x3935EA && kBitSyndromes.v[87] == 0xFFF409 && kBitSyndromes.v[88] == 0x800000,
              "Bit syndromes must match crc24_single_bit_syndrome_112.");

struct Filter {
    uint32_t bits[4096 / 32];
    bool MayMatch(uint32_t syndrome) const { return (bits[(syndrome >> 5) & 127u] >> (syndrome & 31u)) & 1u; }
};
constexpr Filter MakeFilter() {
    Filter f{};
    for (uint16_t i = 0; i < kFrameLenBits; i++) {
        const uint32_t r = kBitSyndromes.v[i];
        f.bits[(r >> 5) & 127u] |= 1u << (r & 31u);
    }
    return f;
}
static constexpr Filter kFilter = MakeFilter();

}  // namespace ModeSSingleBitFilter
