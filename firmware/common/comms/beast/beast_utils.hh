#pragma once

#include "mode_s_packet.hh"
#include "remote_id_packet.hh"
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

/**
 * Remote ID Packet Format (readsbee feed spec v2, READSBEE feeds only)
 *
 * 0x1a | 0xef | transport letter (no escapes) | timestamp | signed dBm | source MAC | ODID message or pack
 * The ODID bytes bound the frame: one 25-byte message, or a pack of 3 + 25 * n bytes (n = 1-9).
 */
constexpr uint16_t kRemoteIDBeastFrameMaxLenBytes =
    1 /* 0x1a */ + 1 /* Frame Type */ + 1 /* Transport */ +
    2 * (6 /* Timestamp */ + 1 /* dBm */ + 6 /* MAC */ + RawRemoteIDPacket::kMaxPayloadLenBytes);  // [Bytes]
constexpr uint8_t kRemoteIDMaxMessagesInPack = 9;
constexpr int8_t kBeastNoLevelDbm = -128;  // Signal byte of a frame that carries dBm: no level.

constexpr uint16_t kUuidNumChars = 36;
constexpr uint16_t kReceiverIDLenBytes = 8;

enum BeastFrameType {
    kBeastFrameTypeInvalid = 0x0,
    kBeastFrameTypeIngestId = 0xe3,  // Used by readsb for forwarding messages internally.
    kBeastFrameTypeFeedId = 0xe4,    // Used by readsb for establishing a feed with a UUID.
    kBeastFrameTypeUAT =
        0xec,  // Used for encapsulated UAT ADSB short messages, UAT ADSB long messages, and UAT uplink messages.
    kBeastFrameTypeRemoteID = 0xef,  // readsbee: Broadcast Remote ID (ASTM F3411) message or message pack.
    kBeastFrameTypeModeAC = 0x31,  // Note: This is not used, since I'm assuming it does NOT refer to DF 4,5.
    kBeastFrameTypeModeSShort = 0x32,
    kBeastFrameTypeModeSLong = 0x33
};

enum UATMessageTypeChar : char {
    kUATMessageTypeADSBShort = 's',
    kUATMessageTypeADSBLong = 'l',
    kUATMessageTypeUplink = 'u'
};

// Remote ID link, lower case: the timestamp is not on the Mode S clock.
enum RemoteIDTransportChar : char {
    kRemoteIDTransportBT4Legacy = 'b',
    kRemoteIDTransportBT5Coded = 'c',
    kRemoteIDTransportBT5Extended1M = 'e',
    kRemoteIDTransportWiFiBeacon = 'w',
    kRemoteIDTransportWiFiNaN = 'n'
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

/**
 * Length of an ODID message or pack from its first bytes, or 0 if it is out of bounds or longer than len.
 */
uint16_t RemoteIDODIDLenBytes(const uint8_t *odid, uint16_t len);

/**
 * Write a Remote ID frame (0xef) with a zero timestamp.
 * @param[out] beast_frame_buf Buffer of at least kRemoteIDBeastFrameMaxLenBytes.
 * @param[in] transport Transport letter (RemoteIDTransportChar).
 * @param[in] rssi_dbm Signal level in dBm, kBeastNoLevelDbm if unknown.
 * @param[in] mac Source address in display order (most significant byte first).
 * @param[in] odid ODID message or pack; only the bytes its header bounds are sent.
 * @param[in] odid_len_bytes Bytes available at odid.
 * @retval Number of bytes written, or 0 if the ODID bytes are out of bounds.
 */
uint16_t WriteRemoteIDBeastFrame(uint8_t *beast_frame_buf, RemoteIDTransportChar transport, int8_t rssi_dbm,
                                 const uint8_t mac[6], const uint8_t *odid, uint16_t odid_len_bytes);

/**
 * Write a Remote ID frame (0xef) from a received packet.
 * @retval Number of bytes written, or 0 if the transport is unknown or the ODID bytes are out of bounds.
 */
uint16_t BuildRemoteIDBeastFrame(uint8_t *beast_frame_buf, const RawRemoteIDPacket &packet);

}  // namespace BeastReporter
