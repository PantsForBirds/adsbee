#include "mode_s_packet.hh"

#include <cstdint>
#include <cstdio>   // for snprintf
#include <cstring>  // for strlen

#include "comms.hh"  // For debug prints.
#include "decode_utils.hh"

#define BYTES_PER_WORD_32 4
#define BITS_PER_WORD_32  32
#define BYTES_PER_WORD_24 3
#define BITS_PER_WORD_24  24
#define BITS_PER_WORD_25  25
#define BITS_PER_BYTE     8
#define NIBBLES_PER_BYTE  2
#define BITS_PER_NIBBLE   4

#define MASK_MSBIT_WORD24 (0b1 << (BITS_PER_WORD_24 - 1))
#define MASK_MSBIT_WORD25 (0b1 << BITS_PER_WORD_24)
#define MASK_WORD24       0xFFFFFF

const uint32_t kExtendedSquitterLastWordIngestionMask = 0xFFFF0000;
const uint32_t kExtendedSquitterLastWordPopCount = 16;
const uint32_t kSquitterLastWordIngestionMask = 0xFFFFFF00;
const uint32_t kSquitterLastWordPopCount = 24;

#define CRC24_USE_TABLE
#ifndef CRC24_USE_TABLE
const uint32_t kCRC24Generator = 0x1FFF409;
const uint16_t kCRC24GeneratorNumBits = 25;
#else
#include "crc.hh"
#endif

/** DecodedModeSPacket **/

RawModeSPacket::RawModeSPacket(uint32_t rx_buffer[kMaxPacketLenWords32], uint16_t rx_buffer_len_words32,
                               int16_t source_in, int16_t sigs_dbm_in, int16_t sigq_db_in,
                               uint64_t mlat_48mhz_64bit_counts_in)
    : source(source_in),
      sigs_dbm(sigs_dbm_in),
      sigq_db(sigq_db_in),
      mlat_48mhz_64bit_counts(mlat_48mhz_64bit_counts_in) {
    // Set the last word indgestion behavior based on packet length.
    uint32_t last_word_ingestion_mask, last_word_popcount;
    if (rx_buffer_len_words32 > 2) {
        // 112 bit packet (Extended Squitter)
        last_word_ingestion_mask = kExtendedSquitterLastWordIngestionMask;
        last_word_popcount = kExtendedSquitterLastWordPopCount;
    } else {
        // 56 bit packet (Squitter)
        last_word_ingestion_mask = kSquitterLastWordIngestionMask;
        last_word_popcount = kSquitterLastWordPopCount;
    }

    // Pack the packet buffer.
    for (uint16_t i = 0; i < rx_buffer_len_words32 && i < kMaxPacketLenWords32; i++) {
        if (i == rx_buffer_len_words32 - 1) {
            buffer[i] = rx_buffer[i] & last_word_ingestion_mask;  // Trim any crap off of last word.
            buffer_len_bytes += CeilBitsToBytes(last_word_popcount);
        } else {
            buffer[i] = rx_buffer[i];
            buffer_len_bytes += BYTES_PER_WORD_32;
        }
    }
}

RawModeSPacket::RawModeSPacket(const char *rx_string, int16_t source_in, int16_t sigs_dbm_in, int16_t sigq_db_in,
                               uint64_t mlat_48mhz_64bit_counts_in)
    : source(source_in),
      sigs_dbm(sigs_dbm_in),
      sigq_db(sigq_db_in),
      mlat_48mhz_64bit_counts(mlat_48mhz_64bit_counts_in) {
    uint16_t rx_num_bytes =
        strnlen(rx_string, kMaxPacketLenWords32 * BYTES_PER_WORD_32 * kNibblesPerByte) / NIBBLES_PER_BYTE;
    for (uint16_t i = 0; i < rx_num_bytes && i < kMaxPacketLenWords32 * BYTES_PER_WORD_32; i++) {
        uint8_t byte = (CHAR_TO_HEX(rx_string[i * NIBBLES_PER_BYTE]) << BITS_PER_NIBBLE) |
                       CHAR_TO_HEX(rx_string[i * NIBBLES_PER_BYTE + 1]);
        uint16_t byte_offset = i % BYTES_PER_WORD_32;  // number of Bytes to shift right from MSB of current word
        if (byte_offset == 0) {
            buffer[i / BYTES_PER_WORD_32] = byte << (3 * BITS_PER_BYTE);  // need to clear out the word
        } else {
            buffer[i / BYTES_PER_WORD_32] |= byte << ((3 - byte_offset) * BITS_PER_BYTE);
        }
        buffer_len_bytes++;
    }
}

uint16_t RawModeSPacket::PrintBuffer(char *buf, uint16_t buf_len_bytes) const {
    uint16_t len = 0;
    switch (buffer_len_bytes) {
        case kSquitterPacketLenBytes:
            len = snprintf(buf, buf_len_bytes, "%08lX%06lX", buffer[0], buffer[1] >> (2 * kBitsPerNibble));
            break;
        case kExtendedSquitterPacketLenBytes:
            len = snprintf(buf, buf_len_bytes, "%08lX%08lX%08lX%04lX", buffer[0], buffer[1], buffer[2],
                           buffer[3] >> (4 * kBitsPerNibble));
            break;
            // Don't print anything if the buffer is an invalid length.
    }
    return len;
}

DecodedModeSPacket::DecodedModeSPacket(uint32_t rx_buffer[kMaxPacketLenWords32], uint16_t rx_buffer_len_words32,
                                       int16_t source, int32_t sigs_dbm, int32_t sigq_db,
                                       uint64_t mlat_48mhz_64bit_counts)
    : raw(rx_buffer, rx_buffer_len_words32, source, sigs_dbm, sigq_db, mlat_48mhz_64bit_counts) {
    ConstructModeSPacket();
}

DecodedModeSPacket::DecodedModeSPacket(const char *rx_string, int16_t source, int32_t sigs_dbm, int32_t sigq_db,
                                       uint64_t mlat_48mhz_64bit_counts)
    : raw(rx_string, source, sigs_dbm, sigq_db, mlat_48mhz_64bit_counts) {
    ConstructModeSPacket();
}

DecodedModeSPacket::DecodedModeSPacket(const RawModeSPacket &packet_in) : raw(packet_in) { ConstructModeSPacket(); }

DecodedModeSPacket::DownlinkFormat DecodedModeSPacket::GetDownlinkFormatEnum() {
    switch (downlink_format) {
        // DF 0-11 = short messages (56 bits)
        case DownlinkFormat::kDownlinkFormatShortRangeAirToAirSurveillance:
        case DownlinkFormat::kDownlinkFormatAltitudeReply:
        case DownlinkFormat::kDownlinkFormatIdentityReply:
        case DownlinkFormat::kDownlinkFormatAllCallReply:
        // DF 16-24 = long messages (112 bits)
        case DownlinkFormat::kDownlinkFormatLongRangeAirToAirSurveillance:
        case DownlinkFormat::kDownlinkFormatExtendedSquitter:
        case DownlinkFormat::kDownlinkFormatExtendedSquitterNonTransponder:
        case DownlinkFormat::kDownlinkFormatMilitaryExtendedSquitter:
        case DownlinkFormat::kDownlinkFormatCommBAltitudeReply:
        case DownlinkFormat::kDownlinkFormatCommBIdentityReply:
        case DownlinkFormat::kDownlinkFormatCommDExtendedLengthMessage:
            // DF 1-3, 6-10, 11-15, 22-23 not used
            return static_cast<DownlinkFormat>(downlink_format);
            break;
        default:
            return DownlinkFormat::kDownlinkFormatInvalid;
    }
    // Bits 1-5: Downlink Format (DF)
    enum DownlinkFormat {
        kDownlinkFormatInvalid = -1,
        // DF 0-11 = short messages (56 bits)
        kDownlinkFormatShortRangeAirToAirSurveillance = 0,
        kDownlinkFormatAltitudeReply = 4,
        kDownlinkFormatIdentityReply = 5,
        DF_ALL_CALL_REPLY = 11,
        // DF 16-24 = long messages (112 bits)
        DF_LONG_RANGE_AIR_SURVEILLANCE = 16,
        DF_EXTENDED_SQUITTER = 17,
        DF_EXTENDED_SQUITTER_NON_TRANSPONDER = 18,
        DF_MILITARY_EXTENDED_SQUITTER = 19,
        DF_COMM_B_ALTITUDE_REPLY = 20,
        DF_COMM_B_IDENTITY_REPLY = 21,
        DF_COMM_D_EXTENDED_LENGTH_MESSAGE = 24
        // DF 1-3, 6-10, 11-15, 22-23 not used
    };
}

uint16_t DecodedModeSPacket::DumpPacketBuffer(uint32_t to_buffer[kMaxPacketLenWords32]) const {
    uint16_t bytes_written = raw.buffer_len_bytes;
    for (uint16_t i = 0; i < kMaxPacketLenWords32; i++) {
        to_buffer[i] = raw.buffer[i];
    }
    return bytes_written;
}

uint16_t DecodedModeSPacket::DumpPacketBuffer(uint8_t to_buffer[kMaxPacketLenWords32 * kBytesPerWord]) const {
    uint16_t bytes_written = raw.buffer_len_bytes;
    for (uint16_t i = 0; i < kMaxPacketLenWords32; i++) {
        // First received bit is MSb.
        to_buffer[i * kBytesPerWord] = raw.buffer[i] >> 24;
        to_buffer[i * kBytesPerWord + 1] = (raw.buffer[i] >> 16) & 0xFF;
        to_buffer[i * kBytesPerWord + 2] = (raw.buffer[i] >> 8) & 0xFF;
        to_buffer[i * kBytesPerWord + 3] = raw.buffer[i] & 0xFF;
    }
    return bytes_written;
}

uint32_t DecodedModeSPacket::CalculateCRC24(uint16_t packet_len_bits) const {
#ifndef CRC24_USE_TABLE
    // CRC calculation algorithm from https://mode-s.org/decode/book-the_1090mhz_riddle-junzi_sun.pdf pg. 91.
    // Must be called on buffer that does not have extra bit ingested at end and has all words left-aligned.
    uint32_t crc_buffer[kMaxPacketLenWords32];
    for (uint16_t i = 0; i < kMaxPacketLenWords32; i++) {
        crc_buffer[i] = raw.buffer[i];
    }

    // Overwrite 24-bit parity word with zeros.
    SetNBitsInWordBuffer(BITS_PER_WORD_24, 0x0, packet_len_bits - BITS_PER_WORD_24, crc_buffer);

    // CRC is a conditional convolve operation using the 25-bit generator word.
    for (uint16_t i = 0; i < packet_len_bits - BITS_PER_WORD_24; i++) {
        uint32_t word = GetNBitsFromWordBuffer(BITS_PER_WORD_25, i, crc_buffer);
        if (word & MASK_MSBIT_WORD25) {
            // Most significant bit is a 1. XOR with generator!
            SetNBitsInWordBuffer(BITS_PER_WORD_25, word ^ kCRC24Generator, i, crc_buffer);
        }
    }

    return GetNBitsFromWordBuffer(BITS_PER_WORD_24, packet_len_bits - BITS_PER_WORD_24, crc_buffer);
#else
    // Digest the 32-bit word packet buffer into a byte buffer.
    uint16_t packet_len_bytes = packet_len_bits / kBitsPerByte;
    uint8_t raw_buffer[packet_len_bytes];
    WordBufferToByteBuffer(raw.buffer, raw_buffer, packet_len_bytes);
    // Feed the byte buffer to the table-based CRC calculator.
    return crc24(raw_buffer, packet_len_bytes - 3);  // Don't include the CRC itself.
#endif
}

void DecodedModeSPacket::ConstructModeSPacket() {
    if (raw.buffer_len_bytes != RawModeSPacket::kExtendedSquitterPacketLenBytes &&
        raw.buffer_len_bytes != RawModeSPacket::kSquitterPacketLenBytes) {
        return;  // Invalid length; leave is_valid as false.
    }

    downlink_format = raw.buffer[0] >> 27;
    uint16_t buffer_len_bits = raw.buffer_len_bytes * kBitsPerByte;
    uint32_t calculated_checksum = CalculateCRC24(buffer_len_bits);
    uint32_t parity_value = Get24BitsFromWordBuffer(buffer_len_bits - BITS_PER_WORD_24, raw.buffer);
    crc_syndrome = calculated_checksum ^ parity_value;

    switch (static_cast<DownlinkFormat>(downlink_format)) {
        case kDownlinkFormatShortRangeAirToAirSurveillance:  // DF = 0
        case kDownlinkFormatAltitudeReply:                   // DF = 4
        case kDownlinkFormatIdentityReply:                   // DF = 5
        case kDownlinkFormatLongRangeAirToAirSurveillance:   // DF = 16
        case kDownlinkFormatCommBAltitudeReply:              // DF = 20
        case kDownlinkFormatCommBIdentityReply:              // DF = 21
        {
            is_valid = false;  // Calculated checksum is XORed with the ICAO address. See ADS-B Decoding Guide pg. 22.
            // ICAO address is a best guess, needs to be confirmed from the aircraft dictionary.
            is_address_parity = true;
            icao_address = parity_value ^ calculated_checksum;
            break;
        }
        case kDownlinkFormatAllCallReply:  // DF = 11
        {
            icao_address = Get24BitsFromWordBuffer(8, raw.buffer);
            uint16_t interrogator_id = parity_value ^ calculated_checksum;
            if (interrogator_id == 0) {
                // Reply to a spontaneous acquisition squitter.
                is_valid = true;
            } else {
                // Don't know the interrogator ID, so can't tell if it's valid.
                is_valid = false;
            }
            break;
        }

        default:  // All other DFs. Note: DF=17-19 for ADS-B.
        {
            // Process a 112-bit message.
            icao_address = raw.buffer[0] & 0xFFFFFF;
            if (calculated_checksum == parity_value) {
                is_valid = true;  // mark packet as valid if CRC matches the parity bits
            }
            // Otherwise is_valid stays false and crc_syndrome can be used for error correction.
        }
    }
}

/** ModeSADSBPacket **/

/**
 * Helper function used by constructors.
 */
void ModeSADSBPacket::ConstructADSBPacket() {
    ca_cf.capability = static_cast<ModeSADSBPacket::Capability>((raw.buffer[0] >> 24) & 0b111);
    type_code = static_cast<ModeSADSBPacket::TypeCode>(raw.buffer[1] >> 27);
    parity_interrogator_id = raw.buffer[1] & 0xFFFFFF;
}

ModeSADSBPacket::TypeCode ModeSADSBPacket::GetTypeCodeEnum() const {
    // Table 3.3 from The 1090Mhz Riddle (Junzi Sun), pg. 37.
    switch (static_cast<uint16_t>(type_code)) {
        case 1:
        case 2:
        case 3:
        case 4:
            return kTypeCodeAircraftID;
            break;
        case 5:
        case 6:
        case 7:
        case 8:
            return kTypeCodeSurfacePosition;
            break;
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
        case 14:
        case 15:
        case 16:
        case 17:
        case 18:
            return kTypeCodeAirbornePositionBaroAlt;
            break;
        case 19:
            return kTypeCodeAirborneVelocities;
            break;
        case 20:
        case 21:
        case 22:
            return kTypeCodeAirbornePositionGNSSAlt;
            break;
        case 23:
        case 24:
        case 25:
        case 26:
        case 27:
            return kTypeCodeReserved;
            break;
        case 28:
            return kTypeCodeAircraftStatus;
            break;
        case 29:
            return kTypeCodeTargetStateAndStatusInfo;
            break;
        case 31:
            return kTypeCodeAircraftOperationStatus;
            break;
        default:
            return kTypeCodeInvalid;
    }
}

ModeSAltitudeReplyPacket::ModeSAltitudeReplyPacket(const DecodedModeSPacket &decoded_packet)
    : DecodedModeSPacket(decoded_packet) {
    uint8_t flight_status = GetNBitsFromWordBuffer(3, 5, raw.buffer);  // FS = Bits 5-7.
    switch (flight_status) {
        case 0b000:  // No alert, no SPI, aircraft is airborne.
            has_alert = false;
            has_ident = false;
            is_airborne = true;
            airborne_state_known = true;
            break;
        case 0b001:  // No alert, no SPI, aircraft is on ground.
            has_alert = false;
            has_ident = false;
            is_airborne = false;
            airborne_state_known = true;
            break;
        case 0b010:  // Alert, no SPI, aircraft is airborne.
            has_alert = true;
            has_ident = false;
            is_airborne = true;
            airborne_state_known = true;
            break;
        case 0b011:  // Alert, no SPI, aircraft is on ground.
            has_alert = true;
            has_ident = false;
            is_airborne = false;
            airborne_state_known = true;
            break;
        case 0b100:  // Alert, SPI, aircraft is airborne or on ground — state unknown.
            has_alert = true;
            has_ident = true;
            break;
        case 0b101:  // No alert, SPI, aircraft is airborne or on ground — state unknown.
            has_ident = true;
            has_alert = false;
            break;
        case 0b110:  // Reserved.
            break;
        case 0b111:  // Not assigned.
            break;
    }

    downlink_request = static_cast<DownlinkRequest>(GetNBitsFromWordBuffer(5, 8, raw.buffer));
    utility_message = GetNBitsFromWordBuffer(4, 13, raw.buffer);
    utility_message_type = static_cast<UtilityMessageType>(GetNBitsFromWordBuffer(2, 17, raw.buffer));
    altitude_ft = AltitudeCodeToAltitudeFt(GetNBitsFromWordBuffer(13, 19, raw.buffer));
};

ModeSIdentityReplyPacket::ModeSIdentityReplyPacket(const DecodedModeSPacket &decoded_packet)
    : DecodedModeSPacket(decoded_packet) {
    uint8_t flight_status = GetNBitsFromWordBuffer(3, 5, raw.buffer);  // FS = Bits 5-7.
    switch (flight_status) {
        case 0b000:  // No alert, no SPI, aircraft is airborne.
            has_alert = false;
            has_ident = false;
            is_airborne = true;
            airborne_state_known = true;
            break;
        case 0b001:  // No alert, no SPI, aircraft is on ground.
            has_alert = false;
            has_ident = false;
            is_airborne = false;
            airborne_state_known = true;
            break;
        case 0b010:  // Alert, no SPI, aircraft is airborne.
            has_alert = true;
            has_ident = false;
            is_airborne = true;
            airborne_state_known = true;
            break;
        case 0b011:  // Alert, no SPI, aircraft is on ground.
            has_alert = true;
            has_ident = false;
            is_airborne = false;
            airborne_state_known = true;
            break;
        case 0b100:  // Alert, SPI, aircraft is airborne or on ground — state unknown.
            has_alert = true;
            has_ident = true;
            break;
        case 0b101:  // No alert, SPI, aircraft is airborne or on ground — state unknown.
            has_ident = true;
            has_alert = false;
            break;
        case 0b110:  // Reserved.
            break;
        case 0b111:  // Not assigned.
            break;
    }

    downlink_request = static_cast<DownlinkRequest>(GetNBitsFromWordBuffer(5, 8, raw.buffer));
    utility_message = GetNBitsFromWordBuffer(4, 13, raw.buffer);
    utility_message_type = static_cast<UtilityMessageType>(GetNBitsFromWordBuffer(2, 17, raw.buffer));
    squawk = IdentityCodeToSquawk(GetNBitsFromWordBuffer(13, 19, raw.buffer));
};
