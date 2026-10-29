#include <cmath>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "adsl/adsl_vectors.hh"
#include "adsl_packet.hh"
#include "crc.hh"
#include "gtest/gtest.h"

namespace {

std::vector<uint8_t> HexToBytes(const char *hex) {
    std::vector<uint8_t> out;
    for (size_t i = 0; hex[i] && hex[i + 1]; i += 2) out.push_back(std::stoi(std::string(hex + i, 2), nullptr, 16));
    return out;
}

std::vector<uint8_t> HexToBits(const char *hex, uint16_t num_bits) {
    std::vector<uint8_t> bits;
    for (size_t i = 0; hex[i] && bits.size() < num_bits; i++) {
        int nibble = std::stoi(std::string(1, hex[i]), nullptr, 16);
        for (int b = 3; b >= 0 && bits.size() < num_bits; b--) bits.push_back((nibble >> b) & 1);
    }
    return bits;
}

// Finds the 32-chip sync word in a chip stream like the radio does and returns the chip bytes after it.
std::vector<uint8_t> ChipsAfterSync(const ADSLGoldenVector &v) {
    std::vector<uint8_t> bits = HexToBits(v.chips_hex, v.num_chips);
    uint32_t shift = 0;
    for (size_t i = 0; i < bits.size(); i++) {
        shift = (shift << 1) | bits[i];
        if (i >= 31 && shift == RawADSLPacket::kSyncWordChips) {
            std::vector<uint8_t> chips((bits.size() - i - 1 + 7) / 8, 0);
            for (size_t j = i + 1; j < bits.size(); j++) {
                chips[(j - i - 1) / 8] |= bits[j] << (7 - (j - i - 1) % 8);
            }
            return chips;
        }
    }
    return {};
}

// Spec E.3.1 sample code.
uint32_t SpecCRC24(const uint8_t *data, size_t len) {
    auto polypass = [](uint32_t crc, uint8_t input) {
        crc |= input;
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x80000000) crc ^= 0xFFFA0480;
            crc <<= 1;
        }
        return crc;
    };
    uint32_t crc = 0;
    for (size_t i = 0; i < len; i++) crc = polypass(crc, data[i]);
    for (int i = 0; i < 3; i++) crc = polypass(crc, 0);
    return crc >> 8;
}

void ExpectNear(float expected, bool valid, float actual, float tol, const char *what, const char *name) {
    if (std::isnan(expected)) {
        EXPECT_FALSE(valid) << name << " " << what;
    } else {
        EXPECT_TRUE(valid) << name << " " << what;
        EXPECT_NEAR(actual, expected, tol) << name << " " << what;
    }
}

}  // namespace

TEST(ADSLPacket, SpecBitOrderExample) {
    // Spec B.3 example 2: A = 0x12AA (14 bits), B = 0x355 (10 bits). The spec text says byte 2 = 0x55, but its bit
    // sequence 11010101 is 0xD5.
    const uint8_t buf[] = {0xAA, 0x52, 0xD5};
    EXPECT_EQ(DecodedADSLPacket::GetBits(buf, 0, 14), 0x12AAu);
    EXPECT_EQ(DecodedADSLPacket::GetBits(buf, 14, 10), 0x355u);
}

TEST(ADSLPacket, SpecExponentialExamples) {
    // Altitude (G.1.7): value + 320 m offset.
    EXPECT_EQ((int)DecodedADSLPacket::ExpDecodeUnsigned(0x0000, 12) - 320, -320);
    EXPECT_EQ((int)DecodedADSLPacket::ExpDecodeUnsigned(0x0140, 12) - 320, 0);
    EXPECT_EQ((int)DecodedADSLPacket::ExpDecodeUnsigned(0x0528, 12) - 320, 1000);
    EXPECT_EQ((int)DecodedADSLPacket::ExpDecodeUnsigned(0x3FFE, 12) - 320, 61104);
    // Ground speed (G.1.8), 0.25 m/s.
    EXPECT_EQ(DecodedADSLPacket::ExpDecodeUnsigned(0x01, 6), 1u);
    EXPECT_EQ(DecodedADSLPacket::ExpDecodeUnsigned(0x03, 6), 3u);
    EXPECT_EQ(DecodedADSLPacket::ExpDecodeUnsigned(0xC4, 6), 480u);
    EXPECT_EQ(DecodedADSLPacket::ExpDecodeUnsigned(0xFE, 6), 944u);
    // Vertical rate (G.1.9), 0.125 m/s.
    EXPECT_EQ(DecodedADSLPacket::ExpDecodeSigned(0x000, 6), 0);
    EXPECT_EQ(DecodedADSLPacket::ExpDecodeSigned(0x001, 6), 1);
    EXPECT_EQ(DecodedADSLPacket::ExpDecodeSigned(0x101, 6), -1);
    EXPECT_EQ(DecodedADSLPacket::ExpDecodeSigned(0x048, 6), 80);
    EXPECT_EQ(DecodedADSLPacket::ExpDecodeSigned(0x148, 6), -80);
    EXPECT_EQ(DecodedADSLPacket::ExpDecodeSigned(0x0FF, 6), 952);
    EXPECT_EQ(DecodedADSLPacket::ExpDecodeSigned(0x1FE, 6), -944);
}

TEST(ADSLPacket, ModeSCRCMatchesSpecSampleCode) {
    std::mt19937 rng(7);
    for (int trial = 0; trial < 200; trial++) {
        uint8_t buf[32];
        size_t len = 1 + rng() % 31;
        for (size_t i = 0; i < len; i++) buf[i] = rng();
        EXPECT_EQ(crc24(buf, len), SpecCRC24(buf, len));
    }
}

TEST(ADSLPacket, XXTEARoundTrip) {
    const uint32_t key[4] = {0, 0, 0, 0};
    std::mt19937 rng(3);
    for (uint16_t n = 2; n <= 7; n++) {
        uint32_t words[7], orig[7];
        for (uint16_t i = 0; i < n; i++) words[i] = orig[i] = rng();
        DecodedADSLPacket::XXTEAEncrypt(words, n, key);
        EXPECT_NE(memcmp(words, orig, n * 4), 0);
        DecodedADSLPacket::XXTEADecrypt(words, n, key);
        EXPECT_EQ(memcmp(words, orig, n * 4), 0);
    }
}

TEST(ADSLPacket, GoldenVectorsFromPacketBytes) {
    for (const ADSLGoldenVector &v : kADSLGoldenVectors) {
        std::vector<uint8_t> bytes = HexToBytes(v.packet_hex);
        RawADSLPacket raw(bytes.data(), bytes.size());
        EXPECT_EQ(raw.CRCIsValid(), v.crc_ok) << v.name;
        DecodedADSLPacket decoded(raw);
        EXPECT_EQ(decoded.is_valid, v.crc_ok) << v.name;
        if (!v.crc_ok) continue;

        std::vector<uint8_t> clear = HexToBytes(v.adsl_data_clear_hex);
        ASSERT_EQ(decoded.data_len_bytes, clear.size()) << v.name;
        EXPECT_EQ(memcmp(decoded.data, clear.data(), clear.size()), 0) << v.name;
        EXPECT_EQ(decoded.protocol_version, v.version) << v.name;
        EXPECT_EQ(decoded.key_index, v.key_index) << v.name;
        EXPECT_FALSE(decoded.has_signature) << v.name;
        EXPECT_FALSE(decoded.uses_fec) << v.name;
        EXPECT_EQ(decoded.payload_type, v.payload_type) << v.name;
        EXPECT_EQ(decoded.address_mapping, v.amt) << v.name;
        EXPECT_EQ(decoded.address, v.address) << v.name;
        EXPECT_FALSE(decoded.relayed) << v.name;

        if (v.payload_type == DecodedADSLPacket::kPayloadTypeTraffic) {
            const DecodedADSLPacket::Traffic &t = decoded.traffic;
            ExpectNear(v.lat, t.position_valid, t.latitude_deg, 1e-4f, "lat", v.name);
            ExpectNear(v.lon, t.position_valid, t.longitude_deg, 1e-4f, "lon", v.name);
            ExpectNear(v.alt_m, t.altitude_valid, t.altitude_wgs84_m, 0.0f, "alt", v.name);
            ExpectNear(v.gs_mps, t.ground_speed_valid, t.ground_speed_mps, 0.0f, "gs", v.name);
            ExpectNear(v.vrate_mps, t.vertical_rate_valid, t.vertical_rate_mps, 0.0f, "vrate", v.name);
            if (t.track_valid) EXPECT_NEAR(t.track_deg, v.track_deg, 1e-3f) << v.name;
            EXPECT_EQ(t.track_valid, !std::isnan(v.gs_mps) && v.gs_mps != 0.0f) << v.name;
            EXPECT_EQ(t.timestamp_quarter_s, v.ts_qs) << v.name;
            EXPECT_EQ(t.flight_state, v.flight_state) << v.name;
            EXPECT_EQ(t.category, v.category) << v.name;
            EXPECT_EQ(t.emergency, v.emergency) << v.name;
            EXPECT_EQ(t.source_integrity_level, v.sil) << v.name;
            EXPECT_EQ(t.design_assurance, v.sda) << v.name;
            EXPECT_EQ(t.navigation_integrity, v.nic) << v.name;
            EXPECT_EQ(t.horizontal_accuracy, v.hfom) << v.name;
            EXPECT_EQ(t.vertical_accuracy, v.vfom) << v.name;
            EXPECT_EQ(t.velocity_accuracy, v.vacc) << v.name;
        } else if (v.payload_type == DecodedADSLPacket::kPayloadTypeStatus) {
            const DecodedADSLPacket::Status &s = decoded.status;
            EXPECT_EQ(s.max_protocol_version, v.max_version) << v.name;
            EXPECT_EQ(s.m_band_rx, v.rx_m) << v.name;
            EXPECT_EQ(s.o_band_ldr_rx, v.rx_o_ldr) << v.name;
            EXPECT_EQ(s.o_band_hdr_rx, v.rx_o_hdr) << v.name;
            EXPECT_EQ(s.ads_l_mobile, v.mobile) << v.name;
            EXPECT_EQ(s.e_conspicuity_rx, v.econ_rx) << v.name;
            EXPECT_EQ(s.e_conspicuity_tx, v.econ_tx) << v.name;
            EXPECT_EQ(s.xpdr, v.xpdr) << v.name;
            EXPECT_EQ(s.traffic_uplink_client, v.uplink_client != 0) << v.name;
            EXPECT_EQ(s.fisb_uplink_client, v.fisb_client != 0) << v.name;
        }
    }
}

TEST(ADSLPacket, GoldenVectorsFromChips) {
    for (const ADSLGoldenVector &v : kADSLGoldenVectors) {
        std::vector<uint8_t> chips = ChipsAfterSync(v);
        ASSERT_GE(chips.size(), 2u) << v.name << ": no sync word";
        std::vector<uint8_t> bytes = HexToBytes(v.packet_hex);
        EXPECT_EQ(RawADSLPacket::ChipBytesFromLengthChips(chips[0], chips[1]), bytes.size() * 2) << v.name;

        RawADSLPacket raw;
        EXPECT_EQ(raw.FromChips(chips.data(), chips.size()), v.crc_ok) << v.name;
        if (!v.crc_ok) continue;
        EXPECT_EQ(raw.sigq_bits, 0) << v.name;
        ASSERT_EQ(raw.buffer_len_bytes, bytes.size()) << v.name;
        EXPECT_EQ(memcmp(raw.buffer, bytes.data(), bytes.size()), 0) << v.name;
    }
}

TEST(ADSLPacket, RepairsErasures) {
    const ADSLGoldenVector &v = kADSLGoldenVectors[0];
    std::vector<uint8_t> clean = ChipsAfterSync(v);
    std::vector<uint8_t> bytes = HexToBytes(v.packet_hex);
    uint16_t num_bits = bytes.size() * 8;
    std::mt19937 rng(11);
    for (uint16_t num_erasures = 1; num_erasures <= RawADSLPacket::kMaxErasures + 1; num_erasures++) {
        for (int trial = 0; trial < 50; trial++) {
            std::vector<uint8_t> chips = clean;
            std::vector<uint16_t> picked;
            while (picked.size() < num_erasures) {
                uint16_t bit = 8 + rng() % (num_bits - 8);  // Keep the length byte clean.
                if (std::find(picked.begin(), picked.end(), bit) != picked.end()) continue;
                picked.push_back(bit);
                // Bit k is chips 2k and 2k+1, MSB first. Force the pair to 00 or 11.
                uint16_t chip = 2 * bit;
                uint8_t value = rng() & 1;
                for (uint16_t c = chip; c < chip + 2; c++) {
                    uint8_t mask = 1 << (7 - c % 8);
                    chips[c / 8] = value ? (chips[c / 8] | mask) : (chips[c / 8] & ~mask);
                }
            }
            RawADSLPacket raw;
            bool ok = raw.FromChips(chips.data(), chips.size());
            if (num_erasures <= RawADSLPacket::kMaxErasures) {
                ASSERT_TRUE(ok) << num_erasures << " erasures";
                EXPECT_EQ(raw.sigq_bits, num_erasures);
                EXPECT_EQ(memcmp(raw.buffer, bytes.data(), bytes.size()), 0);
            } else {
                EXPECT_FALSE(ok);
            }
        }
    }
}

TEST(ADSLPacket, RejectsFlippedBit) {
    // Both chips of a pair flipped reads as a valid opposite bit, which only the CRC catches.
    const ADSLGoldenVector &v = kADSLGoldenVectors[0];
    std::vector<uint8_t> clean = ChipsAfterSync(v);
    for (uint16_t bit = 8; bit < HexToBytes(v.packet_hex).size() * 8; bit++) {
        std::vector<uint8_t> chips = clean;
        chips[(2 * bit) / 8] ^= 0b11 << (6 - (2 * bit) % 8);
        RawADSLPacket raw;
        EXPECT_FALSE(raw.FromChips(chips.data(), chips.size())) << bit;
    }
}

TEST(ADSLPacket, RejectsUnsupportedHeaders) {
    const ADSLGoldenVector &v = kADSLGoldenVectors[2];  // Clear (key index 3) traffic.
    ASSERT_EQ(v.key_index, 3);
    std::vector<uint8_t> bytes = HexToBytes(v.packet_hex);
    auto with_header = [&](uint8_t header) {
        std::vector<uint8_t> b = bytes;
        b[1] = header;
        uint32_t crc = crc24(b.data() + 1, b[0] - 3);
        b[b[0] - 2] = crc >> 16;
        b[b[0] - 1] = crc >> 8;
        b[b[0]] = crc;
        return DecodedADSLPacket(RawADSLPacket(b.data(), b.size()));
    };
    EXPECT_TRUE(with_header(0x60).is_valid);        // Version 0, key 3.
    EXPECT_TRUE(with_header(0x61).is_valid);        // Version 1, key 3.
    EXPECT_FALSE(with_header(0x62).is_valid);       // Version 2: unknown layout.
    EXPECT_FALSE(with_header(0x60 | 0x80).is_valid);  // FEC flag on a CRC packet.
    EXPECT_FALSE(with_header(0x20).is_valid);       // Key index 1 is reserved.
    EXPECT_TRUE(with_header(0x70).is_valid);        // Signature flag: signature is ignored.

    for (uint8_t len : {0, 11, 13, 22, 32, 255}) {
        uint8_t chips_msb = 0, chips_lsb = 0;
        for (int bit = 7; bit >= 0; bit--) {
            uint16_t pair = (len >> bit) & 1 ? 0b01 : 0b10;
            if (bit >= 4) {
                chips_msb |= pair << (2 * (bit - 4));
            } else {
                chips_lsb |= pair << (2 * bit);
            }
        }
        EXPECT_EQ(RawADSLPacket::ChipBytesFromLengthChips(chips_msb, chips_lsb), 0) << (int)len;
    }
}

TEST(ADSLPacket, NoiseRarelyPasses) {
    // Random chips after a false sync: the length byte, erasure limit and CRC must reject them.
    std::mt19937 rng(5);
    int passed = 0;
    for (int trial = 0; trial < 200000; trial++) {
        uint8_t chips[RawADSLPacket::kMaxChipBytes];
        for (uint8_t &c : chips) c = rng();
        RawADSLPacket raw;
        passed += raw.FromChips(chips, sizeof(chips));
    }
    EXPECT_EQ(passed, 0);
}
