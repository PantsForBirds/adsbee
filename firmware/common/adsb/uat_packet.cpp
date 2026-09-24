#include "uat_packet.hh"

#include <cstring>  // for strlen
#include <numbers>

#include "aircraft_dictionary.hh"  // for Aircraft::kAddressQualifierBitShift
#include "comms.hh"                // for CONSOLE_INFO, CONSOLE_ERROR
#include "fec.hh"
#include "fixedmath/fixed_math.hpp"
#include "geo_utils.hh"
#include "utils/buffer_utils.hh"  // for CHAR_TO_HEX

const fixedmath::fixed_t kDegPerTrackAngleHeadingTick =
    fixedmath::fixed_t{360.0f / 512};  // Direction ticks for Track Angle / Heading field (UAT Tech Manual Table 3-24).
const fixedmath::fixed_t kDegPerRadian =
    fixedmath::fixed_t{180.0f / std::numbers::pi};  // Conversion factor from radians to degrees.
const int32_t kFPMPerEncodedVerticalRateTick = 64;

RawUATADSBPacket::RawUATADSBPacket(const char* rx_string, int16_t sigs_dbm_in, int16_t sigq_bits_in,
                                   uint64_t mlat_48mhz_64bit_counts_in)
    : sigs_dbm(sigs_dbm_in), sigq_bits(sigq_bits_in), mlat_48mhz_64bit_counts(mlat_48mhz_64bit_counts_in) {
    buffer_len_bytes = HexStringToByteBuffer(buffer, rx_string, RawUATADSBPacket::kADSBMessageMaxSizeBytes);
}

RawUATADSBPacket::RawUATADSBPacket(uint8_t rx_buffer[kADSBMessageMaxSizeBytes], uint16_t rx_buffer_len_bytes,
                                   int16_t sigs_dbm_in, int16_t sigq_bits_in, uint64_t mlat_48mhz_64bit_counts_in)
    : sigs_dbm(sigs_dbm_in), sigq_bits(sigq_bits_in), mlat_48mhz_64bit_counts(mlat_48mhz_64bit_counts_in) {
    memcpy(buffer, rx_buffer, rx_buffer_len_bytes);
    buffer_len_bytes = rx_buffer_len_bytes;
}

DecodedUATADSBPacket::DecodedUATADSBPacket(const char* rx_string, int32_t sigs_dbm, int32_t sigq_bits,
                                           uint64_t mlat_48mhz_64bit_counts)
    : raw(rx_string, sigs_dbm, sigq_bits, mlat_48mhz_64bit_counts) {
    // Validate the packet.
    ConstructUATADSBPacket();
}

DecodedUATADSBPacket::DecodedUATADSBPacket(const RawUATADSBPacket& packet) : raw(packet) {
    // Validate the packet.
    ConstructUATADSBPacket();
}

ADSBTypes::DirectionType DecodedUATADSBPacket::HorizontalVelocityToDirectionDegAndSpeedKts(
    uint32_t horizontal_velocity, ADSBTypes::AirGroundState air_ground_state, float& direction_deg_ref,
    int32_t& speed_kts_ref) {
    switch (air_ground_state) {
        case ADSBTypes::kAirGroundStateAirborneSubsonic:
        case ADSBTypes::kAirGroundStateAirborneSupersonic: {
            // When airborne, the horizontal velocity field includes north and east velocities which must be used to
            // calculate a track angle and speed.
            int32_t north_velocity_kts = HorizontalVelocityToNorthVelocityKts(horizontal_velocity, air_ground_state);
            int32_t east_velocity_kts = HorizontalVelocityToEastVelocityKts(horizontal_velocity, air_ground_state);
            if (north_velocity_kts == INT32_MIN || east_velocity_kts == INT32_MIN) {
                direction_deg_ref = 0.0f;
                speed_kts_ref = INT32_MIN;
                return ADSBTypes::kDirectionTypeNotAvailable;
            }
            CalculateTrackAndSpeedFromNEVelocities(north_velocity_kts, east_velocity_kts, direction_deg_ref,
                                                   speed_kts_ref);
            return ADSBTypes::kDirectionTypeTrueTrackAngle;
        } break;
        case ADSBTypes::kAirGroundStateOnGround: {
            // When on the ground, the horizontal velocity field includes a direction and ground speed directly.
            uint16_t speed_encoded = (horizontal_velocity >> 11) & 0b1111111111;
            if (speed_encoded == 0) {
                speed_kts_ref = INT32_MIN;
            } else {
                speed_kts_ref = speed_encoded - 1;
            }
            direction_deg_ref = static_cast<float>(fixedmath::fixed_t(horizontal_velocity & 0b111111111) *
                                                   kDegPerTrackAngleHeadingTick);  // Convert to degrees.
            return static_cast<ADSBTypes::DirectionType>((horizontal_velocity >> 9) & 0b11);
        } break;
        default:
#ifdef ADSB_VERBOSE_PACKET_WARNINGS
            CONSOLE_ERROR("DecodedUATADSBPacket::HorizontalVelocityToDirectionDegAndSpeedKts",
                          "Unrecognized air ground state %d", air_ground_state);
#endif              // ADSB_VERBOSE_PACKET_WARNINGS
            break;  // Added so the default case is happy if the error print is disabled.
    }
    return ADSBTypes::kDirectionTypeNotAvailable;
}

ADSBTypes::VerticalRateSource DecodedUATADSBPacket::VerticalVelocityToVerticalRateFpm(
    uint32_t vertical_velocity, ADSBTypes::AirGroundState air_ground_state, int32_t& vertical_rate_fpm_ref) {
    if (air_ground_state != ADSBTypes::kAirGroundStateAirborneSubsonic &&
        air_ground_state != ADSBTypes::kAirGroundStateAirborneSupersonic) {
        // Vertical velocity field is not available (it's showing aircraft dimensions on ground instead).
        vertical_rate_fpm_ref = INT32_MIN;
        return ADSBTypes::kVerticalRateSourceNotAvailable;
    }

    bool vertical_rate_is_geometric = ((vertical_velocity >> 10) & 0b1) == 0;
    bool vertical_rate_is_positive = ((vertical_velocity >> 9) & 0b1) == 0;

    uint16_t vertical_rate_encoded = vertical_velocity & 0b111111111;
    if (vertical_rate_encoded <= 0) {
        // No vertical rate information available.
        vertical_rate_fpm_ref = INT32_MIN;
    } else {
        vertical_rate_fpm_ref =
            (vertical_rate_encoded - 1) * kFPMPerEncodedVerticalRateTick * (vertical_rate_is_positive ? 1 : -1);
    }
    return vertical_rate_is_geometric ? ADSBTypes::kVerticalRateSourceGNSS : ADSBTypes::kVerticalRateSourceBaro;
}

ADSBTypes::AVDimensionsType DecodedUATADSBPacket::DecodeAVDimensions(uint32_t av_dimensions_encoded,
                                                                     int16_t& width_m_ref, int16_t& length_m_ref) {
    bool position_offset_applied = (av_dimensions_encoded >> 6) & 0b1;
    uint16_t av_length_width_encoded = (av_dimensions_encoded >> 7) & 0b1111;

    switch (av_length_width_encoded) {
        case 0:
            width_m_ref = 12;  // rounded up from 11.5
            length_m_ref = 15;
            break;
        case 1:
            width_m_ref = 23;
            length_m_ref = 15;
            break;
        case 2:
            width_m_ref = 29;  // rounded up from 28.5
            length_m_ref = 25;
            break;
        case 3:
            width_m_ref = 34;
            length_m_ref = 25;
            break;
        case 4:
            width_m_ref = 33;
            length_m_ref = 35;
            break;
        case 5:
            width_m_ref = 38;
            length_m_ref = 35;
            break;
        case 6:
            width_m_ref = 40;  // rounded up from 39.5
            length_m_ref = 45;
            break;
        case 7:
            width_m_ref = 45;
            length_m_ref = 45;
            break;
        case 8:
            width_m_ref = 45;
            length_m_ref = 55;
            break;
        case 9:
            width_m_ref = 52;
            length_m_ref = 55;
            break;
        case 10:
            width_m_ref = 60;  // rounded up from 59.5
            length_m_ref = 65;
            break;
        case 11:
            width_m_ref = 67;
            length_m_ref = 65;
            break;
        case 12:
            width_m_ref = 73;  // rounded up from 72.5
            length_m_ref = 75;
            break;
        case 13:
            width_m_ref = 80;
            length_m_ref = 75;
            break;
        case 14:
            width_m_ref = 80;
            length_m_ref = 200;
            break;
        case 15:
            width_m_ref = 81;  // W > 80, so using 81 as reference
            length_m_ref = 200;
            break;
    }

    return position_offset_applied ? ADSBTypes::kAVDimensionsTypeGNSSSensorOffset
                                   : ADSBTypes::kAVDimensionsTypeAVLengthWidth;
}

uint32_t DecodedUATADSBPacket::GetICAOAddress() const {
    if (!IsValid()) {
        return 0;
    }
    return header.icao_address | (header.address_qualifier << Aircraft::kAddressQualifierBitShift);
}

void DecodedUATADSBPacket::ConstructUATADSBPacket(bool run_fec) {
    // Copy over the message payload. Only attempt FEC correction on the CC1312 or in host-based unit tests, everyone
    // else gets sent pre-corrected raw packets.
    if (run_fec) {
        // Check if the packet is valid by validating the Reed-Solomon code.
        // We don't actually know the number of bits with high certainty. First interpret as a long ADS-B message,
        // and if that doesn't work, try it as a short ADS-B message. Decoding only available on CC1312 and in
        // non-embedded unit tests.
#if defined(ON_TI) || defined(ON_HOST)
        // Correct in place. If correction fails, the buffer will not be
        // modified.
        raw.sigq_bits = uat_rs.DecodeLongADSBMessage(raw.buffer);
        if (raw.sigq_bits >= 0) {
            // CONSOLE_INFO("DecodedUATADSBPacket", "Decoded Long ADS-B message with %d bytes corrected.",
            // raw.sigq_bits);
            message_format = kUATADSBMessageFormatLong;
        } else {
            raw.sigq_bits = uat_rs.DecodeShortADSBMessage(raw.buffer);
            if (raw.sigq_bits >= 0) {
                // CONSOLE_INFO("DecodedUATADSBPacket", "Decoded Short ADS-B message with %d bytes corrected.",
                //  raw.sigq_bits);
                message_format = kUATADSBMessageFormatShort;
            } else {
                // CONSOLE_ERROR("DecodedUATADSBPacket", "Failed to decode UAT ADS-B message, invalid packet.");
                is_valid = false;  // Invalid packet.
                return;
            }
        }
#endif /* ON_TI || ON_HOST */
    }

    // Extract information from the decoded payload.
    DecodeHeader(raw.buffer, header);

    // UAT Tech Manual Table 2-2: Composition of UAT ADS-B Message Data Block
    // Interpret the message contents based on the MDB type code.
    switch (header.mdb_type_code) {
        case 0:  // Basic UAT ADS-B message. Just header and state vector, nothing else.
            // HDR | SV | Reserved
            has_state_vector = true;
            DecodeStateVector(raw.buffer + sizeof(UATHeader), state_vector);
            break;
        case 1:
            // HDR | SV | MS | AUX SV
            has_state_vector = true;
            has_mode_status = true;
            has_auxiliary_state_vector = true;
            DecodeStateVector(raw.buffer + kStateVectorOffsetBytes, state_vector);
            DecodeModeStatus(raw.buffer + kModeStatusOffsetBytes, mode_status);
            DecodeAuxiliaryStateVector(raw.buffer + kAuxiliaryStateVectorOffsetBytes, auxiliary_state_vector);
            break;
        case 2:
            // HDR | SV | Reserved | AUX SV
            has_state_vector = true;
            has_auxiliary_state_vector = true;
            DecodeStateVector(raw.buffer + kStateVectorOffsetBytes, state_vector);
            DecodeAuxiliaryStateVector(raw.buffer + kAuxiliaryStateVectorOffsetBytes, auxiliary_state_vector);
            break;
        case 3:
            // HDR | SV | MS | TS | Reserved Byte
            has_state_vector = true;
            has_mode_status = true;
            has_target_state = true;
            DecodeStateVector(raw.buffer + kStateVectorOffsetBytes, state_vector);
            DecodeModeStatus(raw.buffer + kModeStatusOffsetBytes, mode_status);
            // Target state in this message is at the same offset as auxiliary state vector in other messages.
            DecodeTargetState(raw.buffer + kAuxiliaryStateVectorOffsetBytes, target_state);
            break;
        case 4:
            // HDR | SV | Reserved for TC+0 | TS | Reserved Byte
            has_state_vector = true;
            has_target_state = true;
            DecodeStateVector(raw.buffer + kStateVectorOffsetBytes, state_vector);
            // Target state in this message is at the same offset as auxiliary state vector in other messages.
            DecodeTargetState(raw.buffer + kAuxiliaryStateVectorOffsetBytes, target_state);
            break;
        case 5:
            // HDR | SV | Reserved for TC+1 | AUX SV
            has_state_vector = true;
            has_auxiliary_state_vector = true;
            DecodeStateVector(raw.buffer + kStateVectorOffsetBytes, state_vector);
            DecodeAuxiliaryStateVector(raw.buffer + kAuxiliaryStateVectorOffsetBytes, auxiliary_state_vector);
            break;
        case 6:
            // HDR | SV | Reserved | TS | Reserved Byte | AUX SV
            has_state_vector = true;
            has_target_state = true;
            has_auxiliary_state_vector = true;
            DecodeStateVector(raw.buffer + kStateVectorOffsetBytes, state_vector);
            DecodeTargetState(raw.buffer + 24, target_state);
            DecodeAuxiliaryStateVector(raw.buffer + kAuxiliaryStateVectorOffsetBytes, auxiliary_state_vector);
            break;
        case 7:
        case 8:
        case 9:
        case 10:
            // HDR | SV | Reserved
            has_state_vector = true;
            DecodeStateVector(raw.buffer + kStateVectorOffsetBytes, state_vector);
            break;
        default:
            // All other MDB type codes are reserved for future revisions of the UAT ADS-B protocol or reserved for
            // developmental use.
            break;
    }

    is_valid = true;
}

void DecodedUATADSBPacket::DecodeHeader(uint8_t* data, UATHeader& header_ref) {
    header_ref.mdb_type_code = GetNBitsFromByteBuffer(5, 0, data);
    header_ref.address_qualifier = GetNBitsFromByteBuffer(3, 5, data);
    header_ref.icao_address = GetNBitsFromByteBuffer(24, 8, data);
};
void DecodedUATADSBPacket::DecodeStateVector(uint8_t* data, UATStateVector& state_vector_ref) {
    state_vector_ref.latitude_awb = GetNBitsFromByteBuffer(23, 0, data);
    state_vector_ref.longitude_awb = GetNBitsFromByteBuffer(24, 23, data);
    state_vector_ref.altitude_is_geometric_altitude = GetNBitsFromByteBuffer(1, 47, data);
    state_vector_ref.altitude_encoded = GetNBitsFromByteBuffer(12, 48, data);
    state_vector_ref.nic = GetNBitsFromByteBuffer(4, 60, data);
    state_vector_ref.air_ground_state = static_cast<ADSBTypes::AirGroundState>(GetNBitsFromByteBuffer(2, 64, data));
    state_vector_ref.reserved = GetNBitsFromByteBuffer(1, 66, data);
    state_vector_ref.horizontal_velocity = GetNBitsFromByteBuffer(22, 67, data);
    state_vector_ref.vertical_velocity = GetNBitsFromByteBuffer(11, 89, data);
    state_vector_ref.utc_coupled_or_tis_b_site_id = GetNBitsFromByteBuffer(4, 100, data);
};
void DecodedUATADSBPacket::DecodeModeStatus(uint8_t* data, UATModeStatus& mode_status_ref) {
    // Byte 18-19
    mode_status_ref.emitter_category_and_callsign_chars_1_2 = GetNBitsFromByteBuffer(16, 0, data);
    // Byte 20-21
    mode_status_ref.callsign_chars_3_4_5 = GetNBitsFromByteBuffer(16, 16, data);
    // Byte 22-23
    mode_status_ref.callsign_chars_6_7_8 = GetNBitsFromByteBuffer(16, 32, data);
    // Byte 24
    mode_status_ref.emergency_priority_status = GetNBitsFromByteBuffer(3, 48, data);
    mode_status_ref.uat_version = GetNBitsFromByteBuffer(3, 51, data);
    mode_status_ref.sil = GetNBitsFromByteBuffer(2, 54, data);
    // Byte 25
    mode_status_ref.transmit_mso = GetNBitsFromByteBuffer(6, 56, data);
    // 2 bits reserved.
    // Byte 26
    mode_status_ref.nac_p = GetNBitsFromByteBuffer(4, 64, data);
    mode_status_ref.nac_v = GetNBitsFromByteBuffer(3, 68, data);
    mode_status_ref.nic_baro = GetNBitsFromByteBuffer(1, 71, data);
    // Byte 27
    mode_status_ref.capability_codes = GetNBitsFromByteBuffer(2, 72, data);
    mode_status_ref.operational_modes = GetNBitsFromByteBuffer(3, 74, data);
    mode_status_ref.heading_uses_magnetic_north = GetNBitsFromByteBuffer(1, 77, data);
    mode_status_ref.csid = GetNBitsFromByteBuffer(1, 78, data);
};

void DecodedUATADSBPacket::DecodeAuxiliaryStateVector(uint8_t* data, UATAuxiliaryStateVector& aux_state_vector_ref) {
    aux_state_vector_ref.secondary_altitude_encoded = GetNBitsFromByteBuffer(12, 0, data);
};
void DecodedUATADSBPacket::DecodeTargetState(uint8_t* data, UATTargetState& target_state_ref) {};

RawUATUplinkPacket::RawUATUplinkPacket(const char* rx_string, int16_t sigs_dbm_in, int16_t sigq_bits_in,
                                       uint64_t mlat_48mhz_64bit_counts_in)
    : sigs_dbm(sigs_dbm_in), sigq_bits(sigq_bits_in), mlat_48mhz_64bit_counts(mlat_48mhz_64bit_counts_in) {
    uint16_t rx_num_bytes =
        strnlen(rx_string, kUplinkMessageNumBytes * kBytesPerWord * kNibblesPerByte) / kNibblesPerByte;
    for (uint16_t i = 0; i < rx_num_bytes && i < kUplinkMessageNumBytes * kBytesPerWord; i++) {
        uint8_t byte = (CHAR_TO_HEX(rx_string[i * kNibblesPerByte]) << kBitsPerNibble) |
                       CHAR_TO_HEX(rx_string[i * kNibblesPerByte + 1]);
        encoded_message[i] = byte;
        encoded_message_len_bytes++;
    }
}

RawUATUplinkPacket::RawUATUplinkPacket(uint8_t rx_buffer[kUplinkMessageNumBytes], uint16_t rx_buffer_len_bytes,
                                       int16_t sigs_dbm_in, int16_t sigq_bits_in, uint64_t mlat_48mhz_64bit_counts_in)
    : sigs_dbm(sigs_dbm_in), sigq_bits(sigq_bits_in), mlat_48mhz_64bit_counts(mlat_48mhz_64bit_counts_in) {
    memcpy(encoded_message, rx_buffer, rx_buffer_len_bytes);
    encoded_message_len_bytes = rx_buffer_len_bytes;
}

DecodedUATUplinkPacket::DecodedUATUplinkPacket(const char* rx_string, int32_t sigs_dbm, int32_t sigq_bits,
                                               uint64_t mlat_48mhz_64bit_counts)
    : raw(rx_string, sigs_dbm, sigq_bits, mlat_48mhz_64bit_counts) {
    // Validate the packet.
    ConstructUATUplinkPacket();
}

DecodedUATUplinkPacket::DecodedUATUplinkPacket(const RawUATUplinkPacket& packet) : raw(packet) {
    // Validate the packet.
    ConstructUATUplinkPacket();
}

void DecodedUATUplinkPacket::ConstructUATUplinkPacket(bool run_fec) {
    if (run_fec) {
        // Check if the packet is valid by validating the Reed-Solomon code. Decoding only available on the TI RF MCUs
        // (CC1312 coprocessor / CC1314 m1421) and in non-embedded unit tests; everyone else receives pre-corrected raw
        // packets.
#if defined(ON_TI) || defined(ON_HOST)
        // DecodeUplinkMessage fills decoded_payload AND corrects raw.encoded_message in place, so the raw packet we pass
        // around downstream (SPI to the RP2040, RAW/BEAST/GDL90 reporting) is already a corrected, interleaved codeword.
        // No re-encode pass is needed.
        raw.sigq_bits = uat_rs.DecodeUplinkMessage(decoded_payload, raw.encoded_message);
        if (raw.sigq_bits < 0) {
            is_valid = false;
            return;
        }
#else
        // No FEC correction required, just de-interleave the payload and use it as is (it's already corrected).
        UATReedSolomon::DeInterleaveUplinkMessage(decoded_payload, raw.encoded_message);
#endif
    }

    // TODO: extract info.

    is_valid = true;
};
