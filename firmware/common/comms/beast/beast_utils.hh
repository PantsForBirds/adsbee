#pragma once

#include "mode_s_packet.hh"
#include "uat_packet.hh"

namespace BeastReporter {

const uint8_t kBeastEscapeChar = 0x1a;
const uint16_t kBeastMLATTimestampNumBytes = 6;

// Beast Frame Structure
// <preceded by 0x1a escape character>
// 1 Byte Frame type (does not need escape).
// 1-2 Byte RSSI (may need escape Byte).
// 6-12 Byte MLAT Timestamp (may need 6x escape Bytes).
// Mode-S data (2 bytes + escapes for Mode A/C, 7 bytes + escapes for squitter, 14 bytes + escapes for extended
// squitter)

/**
 * Mode S Packet Format
 *
 * 0x1a | frame type byte (no escapes) | timestamp | rssi | payload
 */

/**
 * UAT Packet Format
 *
 * 0x1a | 0xec (frame type UAT) | uat message type byte (no escapes) | timestamp | rssi | payload
 */

constexpr uint16_t kModeSBeastFrameMaxLenBytes =
    1 /* Frame Type */ + 2 * 6 /* MLAT timestamp + escapes */ + 2 /* RSSI + escape */ +
    2 * RawModeSPacket::kExtendedSquitterPacketLenBytes /* Longest Mode S data + escapes */;  // [Bytes]
constexpr uint16_t kUATADSBBeastFrameMaxLenBytes = 1 /* Frame Type */ + 1 /* UAT Message Type */ +
                                                   2 * 6 /* MLAT timestamp + escapes */ + 2 /* RSSI + escape */ +
                                                   2 * RawUATADSBPacket::kADSBMessageMaxSizeBytes;  // [Bytes]
constexpr uint16_t kUATUplinkBeastFrameMaxLenBytes = 1 /* Frame Type */ + 1 /* UAT Message Type */ +
                                                     2 * 6 /* MLAT timestamp + escapes */ + 2 /* RSSI + escape */ +
                                                     2 * RawUATUplinkPacket::kUplinkMessageNumBytes;  // [Bytes]

constexpr uint16_t kUuidNumChars = 36;
constexpr uint16_t kReceiverIDLenBytes = 8;

enum BeastFrameType {
    kBeastFrameTypeInvalid = 0x0,
    kBeastFrameTypeIngestId = 0xe3,  // Used by readsb for forwarding messages internally.
    kBeastFrameTypeFeedId = 0xe4,    // Used by readsb for establishing a feed with a UUID.
    kBeastFrameTypeUAT =
        0xec,  // Used for encapsulated UAT ADSB short messages, UAT ADSB long messages, and UAT uplink messages.
    kBeastFrameTypeModeAC = 0x31,  // Note: This is not used, since I'm assuming it does NOT refer to DF 4,5.
    kBeastFrameTypeModeSShort = 0x32,
    kBeastFrameTypeModeSLong = 0x33
};

enum UATMessageTypeChar : char {
    kUATMessageTypeADSBShort = 's',
    kUATMessageTypeADSBLong = 'l',
    kUATMessageTypeUplink = 'u'
};

/**
 * Returns the Beast frame type corresponding to a RawModeSPacket.
 * NOTE: kBeastFrameTypeModeAC is not used, since ADSBee only decodes Mode S.
 * @param[in] packet RawModeSPacket to get the downlink format from.
 * @retval BeastFrameType corresponding the packet, or kBeastFrameTypeInvalid if it wasn't recognized.
 */
BeastFrameType GetBeastFrameType(RawModeSPacket packet);

/**
 * Writes the specified number of bytes from from_buf to to_buf, adding 0x1a escape characters as necessary.
 * @param[out] to_buf Buffer to write to.
 * @param[in] from_buf Buffer to read from.
 * @param[in] from_buf_num_bytes Number of Bytes to write, not including escape characters that will be added.
 * @retval Number of bytes (including escapes) that were written to to_buf.
 */
uint16_t WriteBufferWithBeastEscapes(uint8_t to_buf[], const uint8_t from_buf[], uint16_t from_buf_num_bytes);

uint16_t BuildFeedStartFrame(uint8_t *beast_frame_buf, uint8_t *receiver_id);

/**
 * Converts a DecodedModeSPacket payload to a data buffer in Mode S Beast output format.
 * @param[out] beast_frame_buf Pointer to byte buffer to fill with payload.
 * @param[in] packet Reference to DecodedModeSPacket to convert.
 * @retval Number of bytes written to beast_frame_buf.
 */
uint16_t BuildModeSBeastFrame(uint8_t *beast_frame_buf, const DecodedModeSPacket &packet);

/**
 * Sends an Ingest Beast frame (0xe3) with a 16-Byte receiver ID prepended. This type of frame is used by readsb when
 * forwarding messages internally. For feeding, see BuildModeSBeastFrame.
 * @param[out] beast_frame_buf Pointer to byte buffer to fill with payload.
 * @param[in] packet Reference to DecodedModeSPacket to convert.
 * @param[in] receiver_id Pointer to 16-Byte receiver ID.
 * @retval Number of bytes written to beast_frame_buf.
 */
uint16_t BuildModeSIngestBeastFrame(uint8_t *beast_frame_buf, const DecodedModeSPacket &packet,
                                    const uint8_t *receiver_id);

/**
 * Write a UAT ADSB frame.
 * @param[out] beast_frame_buf Pointer to byte buffer to fill with payload.
 * @param[in] packet Reference to RawUATADSBPacket to write to the buffer. Takes the raw packet (not the decoded one) so
 * that callers holding raw packets don't trigger an implicit DecodedUATADSBPacket construction (a full RS decode on
 * FEC-capable platforms) just to build a frame.
 * @retval Number of bytes written to beast_frame_buf.
 */
uint16_t BuildUATADSBBeastFrame(uint8_t *beast_frame_buf, const RawUATADSBPacket &packet);

/**
 * Write a UAT uplink frame.
 * @param[out] beast_frame_buf Pointer to byte buffer to fill with payload.
 * @param[in] packet Reference to RawUATUplinkPacket to write to the buffer. Takes the raw packet (not the decoded one)
 * so that callers holding raw packets don't trigger an implicit DecodedUATUplinkPacket construction (6 RS block
 * decodes on FEC-capable platforms) just to build a frame.
 * @retval Number of bytes written to beast_frame_buf.
 */
uint16_t BuildUATUplinkBeastFrame(uint8_t *beast_frame_buf, const RawUATUplinkPacket &packet);

}  // namespace BeastReporter
