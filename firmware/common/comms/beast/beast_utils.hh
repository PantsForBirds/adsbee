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
 * Remote ID Packet Format (ADSBee Beast extensions (2026-11-06), feeds that confirmed kBeastCapRemoteID)
 *
 * 0x1a | 0xef | transport letter (no escapes) | timestamp | signal | source MAC | ODID message or pack
 * The ODID bytes bound the frame: one 25-byte message, or a pack of 3 + 25 * n bytes (n = 1-9).
 */
constexpr uint16_t kRemoteIDBeastFrameMaxLenBytes =
    1 /* 0x1a */ + 1 /* Frame Type */ + 1 /* Transport */ +
    2 * (6 /* Timestamp */ + 1 /* Signal */ + 6 /* MAC */ + RawRemoteIDPacket::kMaxPayloadLenBytes);  // [Bytes]
constexpr uint8_t kRemoteIDMaxMessagesInPack = 9;

/**
 * ADS-L Packet Format (ADSBee Beast extensions (2026-11-06), feeds that confirmed kBeastCapADSL)
 *
 * 0x1a | 0xed | channel letter (no escapes) | timestamp | signal | repaired erasures | L | L on-air bytes
 * The on-air bytes follow the sync word: length byte L (1-31), then header, data and CRC.
 */
constexpr uint8_t kADSLMaxOnAirLenBytes = 31;
constexpr uint16_t kADSLBeastFrameMaxLenBytes = 1 /* 0x1a */ + 1 /* Frame Type */ + 1 /* Channel */ +
                                                2 * (6 /* Timestamp */ + 1 /* Signal */ + 1 /* Repaired */ +
                                                     1 /* L */ + kADSLMaxOnAirLenBytes);  // [Bytes]
constexpr uint8_t kADSLRepairedUnknown = 0xff;

// Signal byte with no level: power byte mode, and int8 dBm mode (kBeastCapSignalDbm).
constexpr uint8_t kBeastNoLevelPowerByte = 0x00;
constexpr uint8_t kBeastNoLevelDbmByte = 0x80;

/**
 * Capabilities check (ADSBee Beast extensions (2026-11-06), 0xe5). Every TCP Beast feed sends a query right after 0xe4. A server
 * that answers within kBeastCapsAnswerTimeoutMs gets a confirm, then the agreed frames; any other server gets plain
 * Beast.
 *
 * 0x1a | 0xe5 | 'Q' / 'A' / 'C' | 0x80 | version | 0x80 | bits 0-6 | 0x80 | bits 7-13. Never contains 0x1a.
 */
constexpr uint16_t kBeastCapSignalDbm = 1 << 0;      // Signal byte of every frame is int8 dBm.
constexpr uint16_t kBeastCapADSL = 1 << 1;           // 0xed ADS-L frames.
constexpr uint16_t kBeastCapRemoteID = 1 << 2;       // 0xef Remote ID frames.
constexpr uint16_t kBeastCapUATModeSClock = 1 << 3;  // 0xec with upper-case S / L / U: timestamp on the Mode S clock.
constexpr uint16_t kBeastCapsKnown = kBeastCapSignalDbm | kBeastCapADSL | kBeastCapRemoteID | kBeastCapUATModeSClock;
// Bits this firmware may put in a query. kBeastCapADSL waits for ADS-L receive (#409), kBeastCapUATModeSClock for UAT
// timestamps on the Mode S clock (#428); their writers are ready.
constexpr uint16_t kBeastCapsSendable = kBeastCapSignalDbm | kBeastCapRemoteID;
// Bits that change the bytes of frames every Beast feed gets. Feeds are grouped by these, one batch per group.
constexpr uint16_t kBeastCapsFrameFormat = kBeastCapSignalDbm | kBeastCapUATModeSClock;
// The version byte of the 0xe5 frame: a field of the wire format, not a document version.
constexpr uint8_t kBeastCapsVersion = 1;
constexpr uint16_t kBeastCapsFrameLenBytes = 6;
constexpr uint32_t kBeastCapsAnswerTimeoutMs = 5000;

constexpr uint16_t kUuidNumChars = 36;
constexpr uint16_t kReceiverIDLenBytes = 8;

enum BeastFrameType {
    kBeastFrameTypeInvalid = 0x0,
    kBeastFrameTypeIngestId = 0xe3,  // Used by readsb for forwarding messages internally.
    kBeastFrameTypeFeedId = 0xe4,    // Used by readsb for establishing a feed with a UUID.
    kBeastFrameTypeUAT =
        0xec,  // Used for encapsulated UAT ADSB short messages, UAT ADSB long messages, and UAT uplink messages.
    kBeastFrameTypeCapabilities = 0xe5,  // readsbee: capabilities query, answer and confirm.
    kBeastFrameTypeADSL = 0xed,          // readsbee: ADS-L packet.
    kBeastFrameTypeRemoteID = 0xef,      // readsbee: Broadcast Remote ID (ASTM F3411) message or message pack.
    kBeastFrameTypeModeAC = 0x31,  // Note: This is not used, since I'm assuming it does NOT refer to DF 4,5.
    kBeastFrameTypeModeSShort = 0x32,
    kBeastFrameTypeModeSLong = 0x33
};

enum UATMessageTypeChar : char {
    kUATMessageTypeADSBShort = 's',
    kUATMessageTypeADSBLong = 'l',
    kUATMessageTypeUplink = 'u',
    // Upper case (kBeastCapUATModeSClock): the timestamp is on the Mode S clock, 0 = not synced yet.
    kUATMessageTypeADSBShortModeSClock = 'S',
    kUATMessageTypeADSBLongModeSClock = 'L',
    kUATMessageTypeUplinkModeSClock = 'U'
};

// ADS-L channel. Always upper case: the timestamp is on the Mode S clock, 0 = not synced yet.
enum ADSLChannelChar : char { kADSLChannel868_2 = 'A', kADSLChannel868_4 = 'B', kADSLChannelUnknown = 'X' };

enum BeastCapsLetter : char { kBeastCapsQuery = 'Q', kBeastCapsAnswer = 'A', kBeastCapsConfirm = 'C' };

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
 * Writes the Beast power byte for a level in dBm: 61-entry table over -100 to -40 dBm, INT16_MIN (no level) as 0.
 * @retval Number of bytes written, including the escape.
 */
uint16_t WriteRSSIdBmAsBeastPowerLevel(uint8_t *buf, int16_t rssi_dbm);

/**
 * Writes a level as int8 dBm (kBeastCapSignalDbm): clamped to -127 to +127, INT16_MIN (no level) as 0x80.
 * @retval Number of bytes written, including the escape.
 */
uint16_t WriteRSSIdBmAsInt8(uint8_t *buf, int16_t rssi_dbm);

/**
 * Writes the signal byte of any frame type: int8 dBm if caps has kBeastCapSignalDbm, else the power byte.
 * @retval Number of bytes written, including the escape.
 */
inline uint16_t WriteSignalByte(uint8_t *buf, int16_t rssi_dbm, uint16_t caps) {
    return (caps & kBeastCapSignalDbm) ? WriteRSSIdBmAsInt8(buf, rssi_dbm) : WriteRSSIdBmAsBeastPowerLevel(buf, rssi_dbm);
}

/**
 * Power byte for an int8 dBm signal byte: the byte the builders write without kBeastCapSignalDbm for the same level.
 * Exact, since the dBm byte holds every level the 61-entry table tells apart.
 */
uint8_t PowerByteFromDbmByte(uint8_t dbm_byte);

/**
 * Rewrites the signal byte of every frame in a run of Beast frames (0x32, 0x33, 0xec, 0xed, 0xef) in place, so that
 * frames built once for one signal format can be sent in the other. When a new byte takes a different number of bytes
 * escaped, the rest of the run is moved to the end of buf and copied back behind it, so buf_size - len bytes of slack
 * must cover any growth.
 * @param[in,out] buf Frames.
 * @param[in] len Bytes of frames in buf.
 * @param[in] buf_size Size of buf.
 * @param[in] new_signal_byte Maps a signal byte to the new one.
 * @retval New length, or 0 if buf holds anything else or the growth doesn't fit: then build the frames again.
 */
uint16_t RewriteSignalBytes(uint8_t *buf, uint16_t len, uint16_t buf_size, uint8_t (*new_signal_byte)(uint8_t));

// Turns frames built with kBeastCapSignalDbm into the frames the builders write without it. See RewriteSignalBytes.
inline uint16_t ConvertSignalDbmToPowerByte(uint8_t *buf, uint16_t len, uint16_t buf_size) {
    return RewriteSignalBytes(buf, len, buf_size, PowerByteFromDbmByte);
}

/**
 * Converts a DecodedModeSPacket payload to a data buffer in Mode S Beast output format.
 * @param[out] beast_frame_buf Pointer to byte buffer to fill with payload.
 * @param[in] packet Reference to DecodedModeSPacket to convert.
 * @param[in] caps Confirmed capability bits of the feed (kBeastCapSignalDbm); 0 for plain Beast.
 * @retval Number of bytes written to beast_frame_buf.
 */
uint16_t BuildModeSBeastFrame(uint8_t *beast_frame_buf, const DecodedModeSPacket &packet, uint16_t caps = 0);

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
 * @param[in] caps Confirmed capability bits of the feed (kBeastCapSignalDbm, kBeastCapUATModeSClock); 0 for plain
 * Beast. kBeastCapUATModeSClock only changes the letter: set it only when the packet's timestamp is on the Mode S clock.
 * @retval Number of bytes written to beast_frame_buf.
 */
uint16_t BuildUATADSBBeastFrame(uint8_t *beast_frame_buf, const RawUATADSBPacket &packet, uint16_t caps = 0);

/**
 * Write a UAT uplink frame.
 * @param[out] beast_frame_buf Pointer to byte buffer to fill with payload.
 * @param[in] packet Reference to RawUATUplinkPacket to write to the buffer. Takes the raw packet (not the decoded one)
 * so that callers holding raw packets don't trigger an implicit DecodedUATUplinkPacket construction (6 RS block
 * decodes on FEC-capable platforms) just to build a frame.
 * @param[in] caps As for BuildUATADSBBeastFrame.
 * @retval Number of bytes written to beast_frame_buf.
 */
uint16_t BuildUATUplinkBeastFrame(uint8_t *beast_frame_buf, const RawUATUplinkPacket &packet, uint16_t caps = 0);

/**
 * Write an ADS-L frame (0xed).
 * @param[out] beast_frame_buf Buffer of at least kADSLBeastFrameMaxLenBytes.
 * @param[in] channel Channel letter (ADSLChannelChar).
 * @param[in] mlat_12mhz_counter Timestamp on the Mode S clock, 0 if not synced yet.
 * @param[in] rssi_dbm Signal level in dBm, INT16_MIN if unknown.
 * @param[in] repaired Manchester erasures repaired against the CRC, kADSLRepairedUnknown if unknown.
 * @param[in] onair Bytes after the sync word: length byte L (1-31), then L bytes through the CRC.
 * @param[in] onair_len_bytes Bytes at onair; must be 1 + L.
 * @param[in] caps Confirmed capability bits of the feed (kBeastCapSignalDbm).
 * @retval Number of bytes written, or 0 if the on-air bytes are out of bounds.
 */
uint16_t WriteADSLBeastFrame(uint8_t *beast_frame_buf, ADSLChannelChar channel, uint64_t mlat_12mhz_counter,
                             int16_t rssi_dbm, uint8_t repaired, const uint8_t *onair, uint16_t onair_len_bytes,
                             uint16_t caps);

/**
 * Length of an ODID message or pack from its first bytes, or 0 if it is out of bounds or longer than len.
 */
uint16_t RemoteIDODIDLenBytes(const uint8_t *odid, uint16_t len);

/**
 * Write a Remote ID frame (0xef).
 * @param[out] beast_frame_buf Buffer of at least kRemoteIDBeastFrameMaxLenBytes.
 * @param[in] transport Transport letter (RemoteIDTransportChar; upper case only with a Mode S clock timestamp).
 * @param[in] mlat_12mhz_counter Timestamp; 0 with a lower-case letter (not on the Mode S clock).
 * @param[in] rssi_dbm Signal level in dBm, INT16_MIN if unknown.
 * @param[in] mac Source address in display order (most significant byte first).
 * @param[in] odid ODID message or pack; only the bytes its header bounds are sent.
 * @param[in] odid_len_bytes Bytes available at odid.
 * @param[in] caps Confirmed capability bits of the feed (kBeastCapSignalDbm).
 * @retval Number of bytes written, or 0 if the ODID bytes are out of bounds.
 */
uint16_t WriteRemoteIDBeastFrame(uint8_t *beast_frame_buf, char transport, uint64_t mlat_12mhz_counter,
                                 int16_t rssi_dbm, const uint8_t mac[6], const uint8_t *odid, uint16_t odid_len_bytes,
                                 uint16_t caps);

/**
 * Write a Remote ID frame (0xef) from a received packet.
 * @param[in] caps Confirmed capability bits of the feed (kBeastCapSignalDbm).
 * @retval Number of bytes written, or 0 if the transport is unknown or the ODID bytes are out of bounds.
 */
uint16_t BuildRemoteIDBeastFrame(uint8_t *beast_frame_buf, const RawRemoteIDPacket &packet, uint16_t caps = 0);

/**
 * Write a capabilities frame (0xe5). Never escaped: every byte after the letter has the high bit set.
 * @param[out] buf Buffer of at least kBeastCapsFrameLenBytes.
 * @param[in] letter Query, answer or confirm.
 * @param[in] bits Feature bits 0-13.
 * @retval kBeastCapsFrameLenBytes.
 */
uint16_t WriteCapsFrame(uint8_t *buf, BeastCapsLetter letter, uint16_t bits);

/**
 * Finds the server's answer (0x1a 0xe5 'A' ...) in what it sends back on a feed socket. Skips everything else: readsb
 * pings, Beast heartbeats, an echo of the query, unknown frames, garbage. Holds 5 bytes of state.
 */
class CapsAnswerScanner {
   public:
    /**
     * Scans received bytes. Bytes after the answer are ignored.
     * @retval True once, for the bytes that complete the answer.
     */
    bool Scan(const uint8_t *data, uint16_t len);
    bool HasAnswer() const { return has_answer_; }
    uint8_t version() const { return version_; }
    uint16_t bits() const { return bits_; }  // As sent, unknown bits included.

   private:
    enum State : uint8_t { kFindEscape, kAfterEscape, kCollecting };
    State state_ = kFindEscape;
    uint8_t num_got_ = 0;
    uint8_t got_[4] = {0};
    bool has_answer_ = false;
    uint8_t version_ = 0;
    uint16_t bits_ = 0;
};

/**
 * Capabilities check of one feed connection. Start() on connect, Receive() with what the server sends, Update() every
 * pass. After Receive() returns true, the caller queues WriteConfirm() before the next frame and calls Confirm(), or
 * Abandon() if it couldn't. Nothing is kept across connections.
 */
class CapsSession {
   public:
    enum State : uint8_t {
        kPlain,     // Not checked, no answer in time, or the confirm couldn't be sent: plain Beast.
        kWaiting,   // Query sent, plain Beast until the answer.
        kAnswered,  // Answer in time: the confirm must go out before the next frame.
        kAgreed     // Confirm sent: frames use bits().
    };

    /**
     * Starts a connection's check.
     * @param[out] buf Buffer for the query, kBeastCapsFrameLenBytes.
     * @param[in] offered Bits this feed can send.
     * @retval Number of bytes written.
     */
    uint16_t Start(uint8_t *buf, uint16_t offered, uint32_t timestamp_ms);

    /**
     * Scans bytes from the server. An answer after kBeastCapsAnswerTimeoutMs is ignored.
     * @retval True once, when an answer arrived in time.
     */
    bool Receive(const uint8_t *data, uint16_t len, uint32_t timestamp_ms);

    // Ends the wait after kBeastCapsAnswerTimeoutMs without an answer.
    void Update(uint32_t timestamp_ms);

    // Writes the confirm (kAnswered): the query bits AND the answer bits.
    uint16_t WriteConfirm(uint8_t *buf) const { return WriteCapsFrame(buf, kBeastCapsConfirm, pending_bits_); }
    void Confirm();
    void Abandon();
    void Reset() { *this = CapsSession(); }

    State state() const { return state_; }
    uint16_t offered() const { return offered_; }
    // Bits in effect for this connection's frames; 0 unless kAgreed.
    uint16_t bits() const { return state_ == kAgreed ? pending_bits_ : 0; }

   private:
    State state_ = kPlain;
    uint16_t offered_ = 0;
    uint16_t pending_bits_ = 0;
    uint32_t start_timestamp_ms_ = 0;
    CapsAnswerScanner scanner_;
};

/**
 * Picks the sinks of one Beast group: those with this protocol and these frame format bits (kBeastCapsFrameFormat).
 * @param[in] sinks Sink descriptors with .sink, .protocol and .beast_caps (CommsManager::ReportSinkInfo).
 * @param[out] selected The .sink of each selected descriptor; at least num_sinks entries.
 * @retval Number of sinks selected.
 */
template <typename SinkInfo, typename Protocol, typename Sink>
uint16_t SelectBeastGroup(const SinkInfo *sinks, uint16_t num_sinks, Protocol protocol, uint16_t frame_format,
                          Sink *selected) {
    uint16_t num_selected = 0;
    for (uint16_t i = 0; i < num_sinks; i++) {
        if (sinks[i].protocol == protocol && (sinks[i].beast_caps & kBeastCapsFrameFormat) == frame_format) {
            selected[num_selected++] = sinks[i].sink;
        }
    }
    return num_selected;
}

}  // namespace BeastReporter
