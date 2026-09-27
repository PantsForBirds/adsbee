#include <cstring>

#include "fec.hh"
#include "gtest/gtest.h"
#include "uat_packet.hh"

namespace {

// Deterministic xorshift64 so that failures are reproducible.
struct XorShift64 {
    uint64_t state;
    explicit XorShift64(uint64_t seed) : state(seed) {}
    uint8_t NextByte() {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        return static_cast<uint8_t>(state >> 32);
    }
    uint32_t NextBelow(uint32_t n) {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        return static_cast<uint32_t>((state >> 32) % n);
    }
};

// Returns true if buf[0:n] is a valid codeword of the shortened code (payload re-encodes to the same parity).
bool IsCodeword(void* rs, const uint8_t* buf, uint16_t payload_len, uint16_t parity_len) {
    uint8_t parity[32];
    encode_rs_char(rs, const_cast<uint8_t*>(buf), parity);
    return memcmp(parity, buf + payload_len, parity_len) == 0;
}

struct CodeParams {
    const char* name;
    void* rs;
    uint16_t payload_len;
    uint16_t parity_len;
};

}  // namespace

/**
 * A reported successful decode must always produce a valid codeword. Karn's decoder used to report success when some
 * error locations landed in the virtual zero-padding of the shortened code (impossible positions), leaving an output
 * that isn't a codeword at all. That made ~1 in 800 random noise frames pass as valid basic UAT ADS-B messages.
 */
TEST(UATReedSolomon, RandomFramesNeverDecodeToNonCodewords) {
    const CodeParams codes[] = {
        {"basic ADS-B RS(30,18)", uat_rs.rs_adsb_short, RawUATADSBPacket::kShortADSBMessagePayloadNumBytes,
         RawUATADSBPacket::kShortADSBMessageFECParityNumBytes},
        {"long ADS-B RS(48,34)", uat_rs.rs_adsb_long, RawUATADSBPacket::kLongADSBMessagePayloadNumBytes,
         RawUATADSBPacket::kLongADSBMessageFECParityNumBytes},
    };
    const int kNumTrials = 40000;
    for (const auto& code : codes) {
        SCOPED_TRACE(code.name);
        XorShift64 rng(0x5eed0000u + code.payload_len);
        int num_accepted = 0;
        int num_bogus = 0;
        for (int trial = 0; trial < kNumTrials; trial++) {
            uint8_t buf[RawUATADSBPacket::kADSBMessageMaxSizeBytes];
            uint16_t len = code.payload_len + code.parity_len;
            for (uint16_t i = 0; i < len; i++) buf[i] = rng.NextByte();
            uint8_t original[RawUATADSBPacket::kADSBMessageMaxSizeBytes];
            memcpy(original, buf, len);

            int ret = decode_rs_char(code.rs, buf, nullptr, 0);
            if (ret >= 0) {
                num_accepted++;
                if (!IsCodeword(code.rs, buf, code.payload_len, code.parity_len)) num_bogus++;
            } else {
                // A failed decode must leave the buffer untouched.
                EXPECT_EQ(memcmp(buf, original, len), 0);
            }
        }
        EXPECT_EQ(num_bogus, 0) << num_accepted << " random frames accepted";
        // Genuine decodes of random data are astronomically unlikely (~1e-9 for RS(30,18) at radius 6).
        EXPECT_EQ(num_accepted, 0);
    }
}

/**
 * Error correction capability must be unaffected: up to t = nroots/2 byte errors at any in-frame position (payload or
 * parity) are corrected and the reported count is exact.
 */
TEST(UATReedSolomon, CorrectsUpToTErrorsAnywhereInFrame) {
    const CodeParams codes[] = {
        {"basic ADS-B RS(30,18)", uat_rs.rs_adsb_short, RawUATADSBPacket::kShortADSBMessagePayloadNumBytes,
         RawUATADSBPacket::kShortADSBMessageFECParityNumBytes},
        {"long ADS-B RS(48,34)", uat_rs.rs_adsb_long, RawUATADSBPacket::kLongADSBMessagePayloadNumBytes,
         RawUATADSBPacket::kLongADSBMessageFECParityNumBytes},
        {"uplink block RS(92,72)", uat_rs.rs_uplink, RawUATUplinkPacket::kUplinkMessageBlockPayloadNumBytes,
         RawUATUplinkPacket::kUplinkMessageBlockFECParityNumBytes},
    };
    for (const auto& code : codes) {
        SCOPED_TRACE(code.name);
        XorShift64 rng(0xc0de0000u + code.payload_len);
        const uint16_t len = code.payload_len + code.parity_len;
        const int t = code.parity_len / 2;
        for (int trial = 0; trial < 300; trial++) {
            uint8_t codeword[RawUATUplinkPacket::kUplinkMessageBlockNumBytes];
            for (uint16_t i = 0; i < code.payload_len; i++) codeword[i] = rng.NextByte();
            encode_rs_char(code.rs, codeword, codeword + code.payload_len);

            int num_errors = trial % (t + 1);
            uint8_t received[RawUATUplinkPacket::kUplinkMessageBlockNumBytes];
            memcpy(received, codeword, len);
            bool corrupted[RawUATUplinkPacket::kUplinkMessageBlockNumBytes] = {false};
            // Always hit the first and last byte of the frame on some trials: those are the positions adjacent to
            // the padding boundary and the end of the block.
            for (int e = 0; e < num_errors; e++) {
                uint16_t pos;
                if (e == 0 && trial % 3 == 0) {
                    pos = 0;
                } else if (e == 1 && trial % 3 == 0) {
                    pos = len - 1;
                } else {
                    do {
                        pos = rng.NextBelow(len);
                    } while (corrupted[pos]);
                }
                if (corrupted[pos]) continue;
                corrupted[pos] = true;
                uint8_t flip;
                do {
                    flip = rng.NextByte();
                } while (flip == 0);
                received[pos] ^= flip;
            }
            int expected_corrections = 0;
            for (uint16_t i = 0; i < len; i++) expected_corrections += corrupted[i] ? 1 : 0;

            EXPECT_EQ(decode_rs_char(code.rs, received, nullptr, 0), expected_corrections);
            EXPECT_EQ(memcmp(received, codeword, len), 0);
        }
    }
}
