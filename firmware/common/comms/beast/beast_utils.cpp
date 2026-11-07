#include "beast_utils.hh"

#include "beast_tables.hh"
#include "comms.hh"
#include "macros.hh"
#include "math.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"

// Mode S Beast Protocol Spec: https://github.com/firestuff/adsb-tools/blob/master/protocols/beast.md

BeastReporter::BeastFrameType BeastReporter::GetBeastFrameType(RawModeSPacket packet) {
    DecodedModeSPacket::DownlinkFormat downlink_format =
        static_cast<DecodedModeSPacket::DownlinkFormat>(packet.buffer[0] >> 27);
    switch (downlink_format) {
        case DecodedModeSPacket::DownlinkFormat::kDownlinkFormatShortRangeAirToAirSurveillance:
        case DecodedModeSPacket::DownlinkFormat::kDownlinkFormatAltitudeReply:
        case DecodedModeSPacket::DownlinkFormat::kDownlinkFormatIdentityReply:
        case DecodedModeSPacket::DownlinkFormat::kDownlinkFormatAllCallReply:
            return kBeastFrameTypeModeSShort;
        case DecodedModeSPacket::DownlinkFormat::kDownlinkFormatLongRangeAirToAirSurveillance:
        case DecodedModeSPacket::DownlinkFormat::kDownlinkFormatExtendedSquitter:
        case DecodedModeSPacket::DownlinkFormat::kDownlinkFormatExtendedSquitterNonTransponder:
        case DecodedModeSPacket::DownlinkFormat::kDownlinkFormatMilitaryExtendedSquitter:
        case DecodedModeSPacket::DownlinkFormat::kDownlinkFormatCommBAltitudeReply:
        case DecodedModeSPacket::DownlinkFormat::kDownlinkFormatCommBIdentityReply:
        case DecodedModeSPacket::DownlinkFormat::kDownlinkFormatCommDExtendedLengthMessage:
            return kBeastFrameTypeModeSLong;
        default:
            return kBeastFrameTypeInvalid;
            break;
    }
}

uint16_t BeastReporter::WriteBufferWithBeastEscapes(uint8_t to_buf[], const uint8_t from_buf[],
                                                    uint16_t from_buf_num_bytes) {
    uint16_t to_buf_num_bytes = 0;
    for (uint16_t i = 0; i < from_buf_num_bytes; i++) {
        to_buf[to_buf_num_bytes++] = from_buf[i];
        // Escape any occurrence of 0x1a.
        if (from_buf[i] == kBeastEscapeChar) {
            to_buf[to_buf_num_bytes++] = kBeastEscapeChar;
        }
    }
    return to_buf_num_bytes;
}

uint16_t BeastReporter::BuildFeedStartFrame(uint8_t *beast_frame_buf, uint8_t *receiver_id) {
    uint16_t bytes_written = 0;
    beast_frame_buf[bytes_written++] = kBeastEscapeChar;
    beast_frame_buf[bytes_written++] = BeastFrameType::kBeastFrameTypeFeedId;

    // Send UUID as ASCII (will not contain 0x1A, since it's just hex characters and dashes).
    // UUID must imitate the form that's output by `cat /proc/sys/kernel/random/uuid` on Linux (e.g.
    // 38366a5c-c54f-4256-bd0a-1557961f5ad0).
    // ADSBee UUID Format: MMMMMMMM-MMMM-MMMM-NNNN-NNNNNNNNNNNN, where M's and N's represent printed hex digits of the
    // internally stored 64-bit UUID (e.g. 0bee00038172d18c) Example ADSBee UUID: 0bee0003-8172-d18c-0bee-00038172d18c
    char uuid[kUuidNumChars + 1] = "eeeeeeee-eeee-eeee-ffff-ffffffffffff";  // Leave space for null terminator.
    sprintf(uuid, "%02x%02x%02x%02x-%02x%02x-%02x%02x-", receiver_id[0], receiver_id[1], receiver_id[2], receiver_id[3],
            receiver_id[4], receiver_id[5], receiver_id[6], receiver_id[7]);
    sprintf(uuid + (kUuidNumChars - 2 * kReceiverIDLenBytes - 1), "%02x%02x-%02x%02x%02x%02x%02x%02x", receiver_id[0],
            receiver_id[1], receiver_id[2], receiver_id[3], receiver_id[4], receiver_id[5], receiver_id[6],
            receiver_id[7]);

    // Write receiver ID string to buffer.
    strncpy((char *)(beast_frame_buf + bytes_written), uuid, kUuidNumChars);
    bytes_written += kUuidNumChars;
    return bytes_written;
}

/**
 * Write 12MHz MLAT timestamp to buffer.
 * @param[in] buf Pointer to output buffer.
 * @param[in] mlat_12mhz_counter MLAT timestamp value.
 * @retval Number of bytes written to the output buffer, including escapes.
 */
uint16_t Write12MHzMLATTimestamp(uint8_t *buf, uint64_t mlat_12mhz_counter) {
    uint8_t mlat_12mhz_counter_buf[6];
    for (uint16_t i = 0; i < BeastReporter::kBeastMLATTimestampNumBytes; i++) {
        mlat_12mhz_counter_buf[i] =
            (mlat_12mhz_counter >> (BeastReporter::kBeastMLATTimestampNumBytes - i - 1) * kBitsPerByte) & 0xFF;
    }
    return BeastReporter::WriteBufferWithBeastEscapes(buf, mlat_12mhz_counter_buf,
                                                      BeastReporter::kBeastMLATTimestampNumBytes);
}

uint16_t BeastReporter::WriteRSSIdBmAsBeastPowerLevel(uint8_t *buf, int16_t rssi_dbm) {
    uint8_t rssi_byte = kBeastNoLevelPowerByte;
    if (rssi_dbm != INT16_MIN) {
        uint16_t rssi_lut_index =
            MIN(MAX(rssi_dbm - kMinRSSIdBm, 0), static_cast<int>(sizeof(kRSSIdBmToRSSIdBFS) - 1));
        rssi_byte = static_cast<uint8_t>(kRSSIdBmToRSSIdBFS[rssi_lut_index]);
    }
    return WriteBufferWithBeastEscapes(buf, &rssi_byte, 1);
}

uint16_t BeastReporter::WriteRSSIdBmAsInt8(uint8_t *buf, int16_t rssi_dbm) {
    uint8_t dbm_byte = kBeastNoLevelDbmByte;
    if (rssi_dbm != INT16_MIN) {
        dbm_byte = static_cast<uint8_t>(static_cast<int8_t>(MIN(MAX(rssi_dbm, -127), 127)));
    }
    return WriteBufferWithBeastEscapes(buf, &dbm_byte, 1);
}

uint16_t BeastReporter::BuildModeSBeastFrame(uint8_t *beast_frame_buf, const DecodedModeSPacket &packet,
                                             uint16_t caps) {
    uint8_t packet_buf[DecodedModeSPacket::kMaxPacketLenWords32 * kBytesPerWord];
    uint16_t data_num_bytes = packet.DumpPacketBuffer(packet_buf);

    beast_frame_buf[0] = kBeastEscapeChar;
    // Determine and write frame type Byte.
    switch (data_num_bytes * kBitsPerByte) {
        case 56:
            // 56-bit data (squitter): Mode S short frame.
            beast_frame_buf[1] = kBeastFrameTypeModeSShort;
            break;
        case 112:
            // 112-bit data (extended squitter): Mode S long frame.
            beast_frame_buf[1] = kBeastFrameTypeModeSLong;
            break;
        default:
            return 0;
    }
    uint16_t bytes_written = 2;

    bytes_written += Write12MHzMLATTimestamp(beast_frame_buf + bytes_written, packet.raw.GetMLAT12MHzCounter());
    bytes_written += WriteSignalByte(beast_frame_buf + bytes_written, packet.raw.sigs_dbm, caps);

    // Write packet buffer with escape characters.
    bytes_written += WriteBufferWithBeastEscapes(beast_frame_buf + bytes_written, packet_buf, data_num_bytes);
    return bytes_written;
}

uint16_t BeastReporter::BuildModeSIngestBeastFrame(uint8_t *beast_frame_buf, const DecodedModeSPacket &packet,
                                                   const uint8_t *receiver_id) {
    uint16_t bytes_written = 0;
    beast_frame_buf[bytes_written++] = kBeastEscapeChar;
    beast_frame_buf[bytes_written++] = BeastFrameType::kBeastFrameTypeIngestId;
    bytes_written += WriteBufferWithBeastEscapes(beast_frame_buf + bytes_written, receiver_id, kReceiverIDLenBytes);
    bytes_written += BuildModeSBeastFrame(beast_frame_buf + bytes_written, packet);
    return bytes_written;
}

/**
 * @brief Writes a buffer as hexadecimal ASCII characters. Does not require beast escapes since no ASCII characters
 * overlap with the 0x1a beast escape character. Note that to_buf will need to be twice as long as from_buf, since each
 * byte takes two chars to be represented as hex ascii.
 *
 * This function is not currently used.
 *
 * @param[out] to_buf Pointer to output buffer.
 * @param[in] from_buf Pointer to input buffer.
 * @param[in] from_buf_num_bytes Number of bytes in the input buffer.
 * @retval Number of bytes written to the output buffer.
 */
uint16_t WriteBufferAsHexASCII(uint8_t *to_buf, const uint8_t *from_buf, uint16_t from_buf_num_bytes) {
    const char hex_chars[] = "0123456789abcdef";
    for (uint16_t i = 0; i < from_buf_num_bytes; i++) {
        to_buf[i * 2] = hex_chars[(from_buf[i] >> 4) & 0x0F];
        to_buf[i * 2 + 1] = hex_chars[from_buf[i] & 0x0F];
    }
    return from_buf_num_bytes * 2;
}

uint16_t BeastReporter::BuildUATADSBBeastFrame(uint8_t *beast_frame_buf, const RawUATADSBPacket &packet,
                                               uint16_t caps) {
    char uat_message_type_char = ' ';
    switch (packet.buffer_len_bytes) {
        case RawUATADSBPacket::kShortADSBMessageNumBytes:
            // Short UAT ADSB message.
            uat_message_type_char = (caps & kBeastCapUATModeSClock) ? kUATMessageTypeADSBShortModeSClock
                                                                     : kUATMessageTypeADSBShort;
            break;
        case RawUATADSBPacket::kLongADSBMessageNumBytes:
            // Long UAT ADSB message.
            uat_message_type_char = (caps & kBeastCapUATModeSClock) ? kUATMessageTypeADSBLongModeSClock
                                                                     : kUATMessageTypeADSBLong;
            break;
        default:
            CONSOLE_ERROR("BeastReporter::BuildUATADSBBeastFrame", "Invalid UAT ADSB message length %d bytes.",
                          packet.buffer_len_bytes);
            return 0;  // Invalid length.
    }

    uint16_t bytes_written = 0;
    beast_frame_buf[bytes_written++] = kBeastEscapeChar;
    beast_frame_buf[bytes_written++] = BeastFrameType::kBeastFrameTypeUAT;
    beast_frame_buf[bytes_written++] = uat_message_type_char;

    bytes_written += Write12MHzMLATTimestamp(beast_frame_buf + bytes_written, packet.GetMLAT12MHzCounter());
    bytes_written += WriteSignalByte(beast_frame_buf + bytes_written, packet.sigs_dbm, caps);

    bytes_written += WriteBufferWithBeastEscapes(beast_frame_buf + bytes_written, packet.buffer,
                                                 packet.buffer_len_bytes);

    return bytes_written;
}

uint16_t BeastReporter::BuildUATUplinkBeastFrame(uint8_t *beast_frame_buf, const RawUATUplinkPacket &packet,
                                                 uint16_t caps) {
    if (packet.encoded_message_len_bytes != RawUATUplinkPacket::kUplinkMessageNumBytes) {
        CONSOLE_ERROR("BeastReporter::BuildUATUplinkBeastFrame", "Invalid UAT Uplink message length %d bytes.",
                      packet.encoded_message_len_bytes);
        return 0;  // Invalid length.
    }

    uint16_t bytes_written = 0;
    beast_frame_buf[bytes_written++] = kBeastEscapeChar;
    beast_frame_buf[bytes_written++] = BeastFrameType::kBeastFrameTypeUAT;
    beast_frame_buf[bytes_written++] =
        (caps & kBeastCapUATModeSClock) ? kUATMessageTypeUplinkModeSClock : kUATMessageTypeUplink;

    bytes_written += Write12MHzMLATTimestamp(beast_frame_buf + bytes_written, packet.GetMLAT12MHzCounter());
    bytes_written += WriteSignalByte(beast_frame_buf + bytes_written, packet.sigs_dbm, caps);

    bytes_written += WriteBufferWithBeastEscapes(beast_frame_buf + bytes_written, packet.encoded_message,
                                                 packet.encoded_message_len_bytes);

    return bytes_written;
}

uint16_t BeastReporter::RemoteIDODIDLenBytes(const uint8_t *odid, uint16_t len) {
    if (len < 1) return 0;
    uint16_t odid_len = RawRemoteIDPacket::kSingleMessageLenBytes;
    if ((odid[0] >> 4) == RawRemoteIDPacket::kMessageTypePacked) {
        // Pack header: message type, message size, message count.
        if (len < 3 || odid[1] != RawRemoteIDPacket::kSingleMessageLenBytes || odid[2] < 1 ||
            odid[2] > kRemoteIDMaxMessagesInPack) {
            return 0;
        }
        odid_len = 3 + odid[2] * RawRemoteIDPacket::kSingleMessageLenBytes;
    }
    return odid_len <= len ? odid_len : 0;
}

uint16_t BeastReporter::WriteRemoteIDBeastFrame(uint8_t *beast_frame_buf, char transport, uint64_t mlat_12mhz_counter,
                                                int16_t rssi_dbm, const uint8_t mac[6], const uint8_t *odid,
                                                uint16_t odid_len_bytes, uint16_t caps) {
    uint16_t odid_len = RemoteIDODIDLenBytes(odid, odid_len_bytes);
    if (odid_len == 0) return 0;
    uint16_t bytes_written = 0;
    beast_frame_buf[bytes_written++] = kBeastEscapeChar;
    beast_frame_buf[bytes_written++] = kBeastFrameTypeRemoteID;
    beast_frame_buf[bytes_written++] = transport;
    bytes_written += Write12MHzMLATTimestamp(beast_frame_buf + bytes_written, mlat_12mhz_counter);
    bytes_written += WriteSignalByte(beast_frame_buf + bytes_written, rssi_dbm, caps);
    bytes_written += WriteBufferWithBeastEscapes(beast_frame_buf + bytes_written, mac, 6);
    bytes_written += WriteBufferWithBeastEscapes(beast_frame_buf + bytes_written, odid, odid_len);
    return bytes_written;
}

uint16_t BeastReporter::BuildRemoteIDBeastFrame(uint8_t *beast_frame_buf, const RawRemoteIDPacket &packet,
                                                uint16_t caps) {
    RemoteIDTransportChar transport;
    uint8_t mac[6];
    switch (packet.transport) {
        case RawRemoteIDPacket::kTransportBT4Legacy:
        case RawRemoteIDPacket::kTransportBT5LongRange:
            transport = packet.transport == RawRemoteIDPacket::kTransportBT4Legacy ? kRemoteIDTransportBT4Legacy
                                                                                   : kRemoteIDTransportBT5Coded;
            // BLE addresses are held least significant byte first.
            for (uint16_t i = 0; i < 6; i++) mac[i] = packet.source_mac[5 - i];
            break;
        case RawRemoteIDPacket::kTransportWiFiBeacon:
            transport = kRemoteIDTransportWiFiBeacon;
            memcpy(mac, packet.source_mac, 6);  // As in the frame header.
            break;
        default:
            return 0;
    }
    // The receivers store 0 for an unknown level.
    int16_t rssi_dbm = packet.rssi_dbm == 0 ? INT16_MIN : packet.rssi_dbm;
    // No Mode S clock on the ESP32 yet: timestamp 0 and a lower-case letter, so the server stamps arrival time.
    // TODO(#428): use (packet.mlat_48mhz_64bit_counts >> 2) & 0xFFFFFFFFFFFF (0 = none) and the upper-case letter.
    const uint64_t mlat_12mhz_counter = 0;
    return WriteRemoteIDBeastFrame(beast_frame_buf, transport, mlat_12mhz_counter, rssi_dbm, mac, packet.payload,
                                   packet.payload_len_bytes, caps);
}

uint16_t BeastReporter::WriteADSLBeastFrame(uint8_t *beast_frame_buf, ADSLChannelChar channel,
                                            uint64_t mlat_12mhz_counter, int16_t rssi_dbm, uint8_t repaired,
                                            const uint8_t *onair, uint16_t onair_len_bytes, uint16_t caps) {
    // The length byte bounds the frame for a decoder, so it must match the bytes sent.
    if (onair_len_bytes < 2 || onair[0] < 1 || onair[0] > kADSLMaxOnAirLenBytes || onair_len_bytes != 1 + onair[0]) {
        return 0;
    }
    uint16_t bytes_written = 0;
    beast_frame_buf[bytes_written++] = kBeastEscapeChar;
    beast_frame_buf[bytes_written++] = kBeastFrameTypeADSL;
    beast_frame_buf[bytes_written++] = channel;
    bytes_written += Write12MHzMLATTimestamp(beast_frame_buf + bytes_written, mlat_12mhz_counter);
    bytes_written += WriteSignalByte(beast_frame_buf + bytes_written, rssi_dbm, caps);
    bytes_written += WriteBufferWithBeastEscapes(beast_frame_buf + bytes_written, &repaired, 1);
    bytes_written += WriteBufferWithBeastEscapes(beast_frame_buf + bytes_written, onair, onair_len_bytes);
    return bytes_written;
}

uint16_t BeastReporter::WriteCapsFrame(uint8_t *buf, BeastCapsLetter letter, uint16_t bits) {
    buf[0] = kBeastEscapeChar;
    buf[1] = kBeastFrameTypeCapabilities;
    buf[2] = letter;
    buf[3] = 0x80 | kBeastCapsVersion;
    buf[4] = 0x80 | (bits & 0x7f);
    buf[5] = 0x80 | ((bits >> 7) & 0x7f);
    return kBeastCapsFrameLenBytes;
}

bool BeastReporter::CapsAnswerScanner::Scan(const uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len && !has_answer_; i++) {
        uint8_t x = data[i];
        switch (state_) {
            case kFindEscape:
                if (x == kBeastEscapeChar) state_ = kAfterEscape;
                break;
            case kAfterEscape:
                // 0x1a 0x1a is an escaped byte inside another frame; any other type: skip to the next 0x1a.
                if (x == kBeastFrameTypeCapabilities) {
                    state_ = kCollecting;
                    num_got_ = 0;
                } else {
                    state_ = kFindEscape;
                }
                break;
            case kCollecting:
                if (x == kBeastEscapeChar) {
                    state_ = kAfterEscape;  // Caps bytes are never 0x1a: a new frame starts here.
                    break;
                }
                got_[num_got_++] = x;
                if (num_got_ < 4) break;
                state_ = kFindEscape;
                // Only an answer counts; a query (an echo) or a byte without the high bit is skipped.
                if (got_[0] == kBeastCapsAnswer && (got_[1] & 0x80) && (got_[2] & 0x80) && (got_[3] & 0x80) &&
                    (got_[1] & 0x7f) != 0) {
                    has_answer_ = true;
                    version_ = got_[1] & 0x7f;
                    bits_ = (got_[2] & 0x7f) | ((got_[3] & 0x7f) << 7);
                    return true;
                }
                break;
        }
    }
    return false;
}

uint16_t BeastReporter::CapsSession::Start(uint8_t *buf, uint16_t offered, uint32_t timestamp_ms) {
    Reset();
    offered_ = offered & kBeastCapsKnown;
    start_timestamp_ms_ = timestamp_ms;
    state_ = kWaiting;
    return WriteCapsFrame(buf, kBeastCapsQuery, offered_);
}

bool BeastReporter::CapsSession::Receive(const uint8_t *data, uint16_t len, uint32_t timestamp_ms) {
    if (state_ != kWaiting) return false;  // After the answer or the timeout, bytes are only drained.
    Update(timestamp_ms);
    if (state_ != kWaiting || !scanner_.Scan(data, len)) return false;
    pending_bits_ = offered_ & scanner_.bits() & kBeastCapsKnown;
    state_ = kAnswered;
    return true;
}

void BeastReporter::CapsSession::Update(uint32_t timestamp_ms) {
    if (state_ == kWaiting && timestamp_ms - start_timestamp_ms_ >= kBeastCapsAnswerTimeoutMs) {
        state_ = kPlain;
    }
}

void BeastReporter::CapsSession::Confirm() {
    if (state_ == kAnswered) state_ = kAgreed;
}

void BeastReporter::CapsSession::Abandon() {
    state_ = kPlain;
    pending_bits_ = 0;
}
