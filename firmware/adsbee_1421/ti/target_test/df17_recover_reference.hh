#pragma once

// The first DF17 realignment (2e47e61f), kept verbatim as the reference that
// LR2021OokAdsb::RecoverDF17Frame() must match bit for bit. It reads the shift list from kDF17Shifts, so it is
// the first version restricted to the shifts the product tries now. Used by the host test (old vs new over every shift
// and flip case and random captures) and by the on-target cycle benchmark (target_test/test_df17_cpu.cpp).
// Nothing in the product calls it.

#include <cstdint>

#include "lr2021_ook_adsb.hh"

namespace LR2021OokAdsbReference {

using LR2021OokAdsb::GetMsgBit;
using LR2021OokAdsb::kDF17FlipFirstBit;
using LR2021OokAdsb::kDF17FlipNumBits;
using LR2021OokAdsb::kDF17HeaderBits;
using LR2021OokAdsb::kDF17HeaderLenBits;
using LR2021OokAdsb::kDF17Shifts;
using LR2021OokAdsb::kModeSFrameLenBits;
using LR2021OokAdsb::kModeSFrameLenBytes;
using LR2021OokAdsb::SetMsgBit;

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

}  // namespace LR2021OokAdsbReference
