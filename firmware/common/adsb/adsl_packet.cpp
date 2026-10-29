#include "adsl_packet.hh"

#include <cstring>

#include "crc.hh"

namespace {

// Decodes one Manchester chip byte pair into a data byte. Erasure (00 or 11) bit positions, MSB = 7, go into
// erasure_mask; those bits read as 0.
inline uint8_t ManchesterDecodeByte(uint8_t chips_msb, uint8_t chips_lsb, uint8_t &erasure_mask) {
    uint16_t chips = static_cast<uint16_t>(chips_msb << 8) | chips_lsb;
    uint8_t byte = 0;
    erasure_mask = 0;
    for (int16_t bit = 7; bit >= 0; bit--) {
        uint8_t pair = (chips >> (2 * bit)) & 0b11;
        if (pair == 0b01) {
            byte |= 1 << bit;
        } else if (pair != 0b10) {
            erasure_mask |= 1 << bit;
        }
    }
    return byte;
}

inline bool LenFieldIsSupported(uint8_t len_field) {
    // ADS-L data is a whole number of 32-bit words: network header + 4N bytes + CRC.
    return len_field >= RawADSLPacket::kMinLenField && len_field <= RawADSLPacket::kMaxLenField &&
           (len_field - RawADSLPacket::kNetworkHeaderNumBytes - RawADSLPacket::kCRCNumBytes) % 4 == 0;
}

constexpr uint32_t kXXTEADelta = 0x9E3779B9;
constexpr uint16_t kXXTEARounds = 6;
constexpr uint32_t kXXTEAKey0[4] = {0, 0, 0, 0};

inline uint32_t XXTEAMix(uint32_t z, uint32_t y, uint32_t sum, uint32_t p, uint32_t e, const uint32_t key[4]) {
    return (((z >> 5) ^ (y << 2)) + ((y >> 3) ^ (z << 4))) ^ ((sum ^ y) + (key[(p & 3) ^ e] ^ z));
}

}  // namespace

RawADSLPacket::RawADSLPacket(const uint8_t *buf, uint16_t buf_len_bytes, int16_t sigs_dbm_in, int16_t sigq_bits_in,
                             uint64_t mlat_48mhz_64bit_counts_in)
    : sigs_dbm(sigs_dbm_in), sigq_bits(sigq_bits_in), mlat_48mhz_64bit_counts(mlat_48mhz_64bit_counts_in) {
    buffer_len_bytes = buf_len_bytes > kMaxPacketLenBytes ? kMaxPacketLenBytes : buf_len_bytes;
    memcpy(buffer, buf, buffer_len_bytes);
}

uint16_t RawADSLPacket::ChipBytesFromLengthChips(uint8_t chips_msb, uint8_t chips_lsb) {
    uint8_t erasures;
    uint8_t len_field = ManchesterDecodeByte(chips_msb, chips_lsb, erasures);
    if (erasures != 0 || !LenFieldIsSupported(len_field)) {
        return 0;
    }
    return (1 + len_field) * kChipBytesPerByte;
}

bool RawADSLPacket::FromChips(const uint8_t *chips, uint16_t num_chip_bytes) {
    buffer_len_bytes = 0;
    sigq_bits = INT16_MIN;
    if (num_chip_bytes < kChipBytesPerByte) return false;
    uint8_t erasures;
    uint8_t len_field = ManchesterDecodeByte(chips[0], chips[1], erasures);
    if (erasures != 0 || !LenFieldIsSupported(len_field)) return false;
    uint16_t len_bytes = 1 + len_field;
    if (num_chip_bytes < len_bytes * kChipBytesPerByte) return false;

    // Decode, noting erasure bit positions (byte index * 8 + bit).
    uint16_t erasure_bits[kMaxErasures];
    uint16_t num_erasures = 0;
    for (uint16_t i = 0; i < len_bytes; i++) {
        buffer[i] = ManchesterDecodeByte(chips[2 * i], chips[2 * i + 1], erasures);
        for (int16_t bit = 7; erasures != 0 && bit >= 0; bit--) {
            if (!(erasures & (1 << bit))) continue;
            if (num_erasures == kMaxErasures) return false;
            erasure_bits[num_erasures++] = i * 8 + bit;
        }
    }
    buffer_len_bytes = len_bytes;

    // Try every value of the erased bits (all read as 0 so far), in Gray code order: one bit flip per try.
    for (uint32_t trial = 0; trial < (1u << num_erasures); trial++) {
        if (trial != 0) {
            uint16_t flip = __builtin_ctz(trial);
            buffer[erasure_bits[flip] / 8] ^= 1 << (erasure_bits[flip] % 8);
        }
        if (CRCIsValid()) {
            sigq_bits = num_erasures;
            return true;
        }
    }
    buffer_len_bytes = 0;
    return false;
}

bool RawADSLPacket::CRCIsValid() const {
    if (buffer_len_bytes < 1 || !LenFieldIsSupported(buffer[0]) || buffer_len_bytes < 1 + buffer[0]) return false;
    // CRC covers the network header and ADS-L data, not the length byte. It is sent MSB first.
    const uint8_t *crc_bytes = buffer + 1 + buffer[0] - kCRCNumBytes;
    uint32_t received_crc = (crc_bytes[0] << 16) | (crc_bytes[1] << 8) | crc_bytes[2];
    return crc24(buffer + 1, buffer[0] - kCRCNumBytes) == received_crc;
}

uint32_t DecodedADSLPacket::GetBits(const uint8_t *buf, uint16_t offset_bits, uint16_t width_bits) {
    uint32_t value = 0;
    for (uint16_t i = 0; i < width_bits; i++) {
        uint16_t bit = offset_bits + i;
        value |= static_cast<uint32_t>((buf[bit / 8] >> (bit % 8)) & 1) << i;
    }
    return value;
}

void DecodedADSLPacket::XXTEAEncrypt(uint32_t *data, uint16_t num_words, const uint32_t key[4]) {
    if (num_words < 2) return;
    uint32_t z = data[num_words - 1], y, sum = 0;
    for (uint16_t round = 0; round < kXXTEARounds; round++) {
        sum += kXXTEADelta;
        uint32_t e = (sum >> 2) & 3;
        for (uint32_t p = 0; p < num_words; p++) {
            y = data[(p + 1) % num_words];
            data[p] += XXTEAMix(z, y, sum, p, e, key);
            z = data[p];
        }
    }
}

void DecodedADSLPacket::XXTEADecrypt(uint32_t *data, uint16_t num_words, const uint32_t key[4]) {
    if (num_words < 2) return;
    uint32_t sum = kXXTEADelta * kXXTEARounds, z, y = data[0];
    for (uint16_t round = 0; round < kXXTEARounds; round++) {
        uint32_t e = (sum >> 2) & 3;
        for (int32_t p = num_words - 1; p >= 0; p--) {
            z = data[(p + num_words - 1) % num_words];
            data[p] -= XXTEAMix(z, y, sum, p, e, key);
            y = data[p];
        }
        sum -= kXXTEADelta;
    }
}

DecodedADSLPacket::DecodedADSLPacket(const RawADSLPacket &packet_in) : raw(packet_in) {
    if (!raw.CRCIsValid()) return;
    uint8_t network_header = raw.buffer[1];
    protocol_version = network_header & 0x0F;
    has_signature = (network_header >> 4) & 1;
    key_index = (network_header >> 5) & 0b11;
    uses_fec = (network_header >> 7) & 1;
    // A newer version may use another layout; a set FEC flag on a CRC packet is a contradiction.
    if (protocol_version > kMaxSupportedProtocolVersion || uses_fec) return;
    if (key_index != kKeyIndexScrambled && key_index != kKeyIndexClear) return;  // Reserved keys.

    data_len_bytes = raw.buffer[0] - RawADSLPacket::kNetworkHeaderNumBytes - RawADSLPacket::kCRCNumBytes;
    memcpy(data, raw.buffer + 2, data_len_bytes);
    if (key_index == kKeyIndexScrambled) {
        uint32_t words[kMaxDataNumBytes / 4];
        uint16_t num_words = data_len_bytes / 4;
        for (uint16_t i = 0; i < num_words; i++) {
            words[i] = data[4 * i] | (data[4 * i + 1] << 8) | (data[4 * i + 2] << 16) |
                       (static_cast<uint32_t>(data[4 * i + 3]) << 24);
        }
        XXTEADecrypt(words, num_words, kXXTEAKey0);
        for (uint16_t i = 0; i < num_words; i++) {
            for (uint16_t b = 0; b < 4; b++) data[4 * i + b] = (words[i] >> (8 * b)) & 0xFF;
        }
    }

    payload_type = data[0];
    address_mapping = GetBits(data, 8, 6);
    address = GetBits(data, 14, 24);
    relayed = GetBits(data, 39, 1);

    uint16_t payload_len_bytes = data_len_bytes - kADSLHeaderNumBytes;
    switch (payload_type) {
        case kPayloadTypeTraffic:
            if (payload_len_bytes < kTrafficPayloadNumBytes) return;
            ParseTraffic();
            break;
        case kPayloadTypeStatus:
            if (payload_len_bytes < kStatusPayloadNumBytes) return;
            ParseStatus();
            break;
        default:
            break;  // Headers only.
    }
    is_valid = true;
}

void DecodedADSLPacket::ParseTraffic() {
    const uint8_t *p = data + kADSLHeaderNumBytes;
    traffic.timestamp_quarter_s = GetBits(p, 0, 6);
    traffic.flight_state = static_cast<FlightState>(GetBits(p, 6, 2));
    traffic.category = static_cast<AircraftCategory>(GetBits(p, 8, 5));
    traffic.emergency = static_cast<EmergencyStatus>(GetBits(p, 13, 3));

    uint32_t lat_raw = GetBits(p, 16, 24);
    uint32_t lon_raw = GetBits(p, 40, 24);
    traffic.position_valid = lat_raw != kLatLonInvalid && lon_raw != kLatLonInvalid;
    if (traffic.position_valid) {
        // Sign extend the 24-bit fields.
        int32_t lat = static_cast<int32_t>(lat_raw << 8) >> 8;
        int32_t lon = static_cast<int32_t>(lon_raw << 8) >> 8;
        traffic.latitude_deg = static_cast<float>(lat) / kLatitudeLSBPerDeg;
        traffic.longitude_deg = static_cast<float>(lon) / kLongitudeLSBPerDeg;
        traffic.position_valid = traffic.latitude_deg >= -90.0f && traffic.latitude_deg <= 90.0f;
    }

    uint32_t gs_raw = GetBits(p, 64, 8);
    traffic.ground_speed_valid = gs_raw != kGroundSpeedInvalid;
    if (traffic.ground_speed_valid) traffic.ground_speed_mps = ExpDecodeUnsigned(gs_raw, 6) * 0.25f;

    uint32_t alt_raw = GetBits(p, 72, 14);
    traffic.altitude_valid = alt_raw != kAltitudeInvalid;
    if (traffic.altitude_valid) {
        traffic.altitude_wgs84_m = static_cast<int32_t>(ExpDecodeUnsigned(alt_raw, 12)) - kAltitudeOffsetM;
    }

    uint32_t vr_raw = GetBits(p, 86, 9);
    traffic.vertical_rate_valid = vr_raw != kVerticalRateInvalid;
    if (traffic.vertical_rate_valid) traffic.vertical_rate_mps = ExpDecodeSigned(vr_raw, 6) * 0.125f;

    traffic.track_deg = GetBits(p, 95, 9) * (360.0f / 512.0f);
    traffic.track_valid = traffic.ground_speed_valid && gs_raw != 0;

    traffic.source_integrity_level = GetBits(p, 104, 2);
    traffic.design_assurance = GetBits(p, 106, 2);
    traffic.navigation_integrity = GetBits(p, 108, 4);
    traffic.horizontal_accuracy = GetBits(p, 112, 3);
    traffic.vertical_accuracy = GetBits(p, 115, 2);
    traffic.velocity_accuracy = GetBits(p, 117, 2);
}

void DecodedADSLPacket::ParseStatus() {
    const uint8_t *p = data + kADSLHeaderNumBytes;
    status.max_protocol_version = GetBits(p, 0, 4);
    status.m_band_rx = GetBits(p, 4, 2);
    status.o_band_ldr_rx = GetBits(p, 6, 2);
    status.o_band_hdr_rx = GetBits(p, 8, 2);
    status.ads_l_mobile = GetBits(p, 10, 2);
    status.e_conspicuity_rx = GetBits(p, 12, 8);
    status.e_conspicuity_tx = GetBits(p, 20, 8);
    status.xpdr = GetBits(p, 28, 2);
    status.traffic_uplink_client = GetBits(p, 30, 1);
    status.fisb_uplink_client = GetBits(p, 31, 1);
}
