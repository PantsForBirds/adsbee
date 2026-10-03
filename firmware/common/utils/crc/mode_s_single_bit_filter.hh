#pragma once

// Prefilter for Mode S single-bit error correction, so most failed frames skip the linear search in
// crc24_find_single_bit_error. Flipping bit i of a 112-bit frame changes the CRC-24 syndrome by
// kBitSyndromes.v[i] = x^(111-i) mod G. The filter has one bit per value of the syndrome's low 12 bits; a clear bit
// means it can't be a single-bit error (about 2.4% of random syndromes pass). Header-only so host tests can check it.

#include <cstdint>

#include "crc.hh"  // kCRC24Generator.

namespace ModeSSingleBitFilter {

static constexpr uint16_t kFrameLenBits = 112;

struct BitSyndromes {
    uint32_t v[kFrameLenBits];
};
constexpr BitSyndromes MakeBitSyndromes() {
    BitSyndromes s{};
    uint32_t r = 1;  // Bit 111 is x^0.
    for (int i = kFrameLenBits - 1; i >= 0; i--) {
        s.v[i] = r;
        r <<= 1;
        if (r & 0x1000000u) r ^= 0x1000000u | kCRC24Generator;
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
