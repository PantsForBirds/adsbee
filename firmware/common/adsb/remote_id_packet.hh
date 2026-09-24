#pragma once

#include <cstdint>

/**
 * RawRemoteIDPacket holds a single Broadcast Remote ID (ASTM F3411 / ASD-STAN prEN 4709-002) message or message pack
 * as received on the ESP32-S3 over Bluetooth LE or WiFi, together with the transport metadata needed to decode and
 * de-duplicate it. It is intentionally free of any dependency on the opendroneid-core-c library so that it can be
 * embedded in CompositeArrays and shuffled across the SPI bus without pulling the codec into every translation unit.
 * The actual decode (into an ODID_UAS_Data) happens where the packet is ingested into an AircraftDictionary, on both
 * the ESP32 and the RP2040.
 *
 * The payload holds the raw Open Drone ID bytes exactly as they appear on the wire, starting at the first ODID message
 * byte (i.e. after the transport-specific framing has been stripped by the receiver):
 *   - A single 25-byte ODID message (BasicID / Location / SelfID / System / OperatorID / Auth), OR
 *   - A full ODID message pack (ODID_MessagePack_encoded: a 3-byte pack header followed by up to 9 x 25-byte messages,
 *     for a maximum of 3 + 9 * 25 = 228 bytes).
 * The message type nibble in payload[0] discriminates the two (0xF == message pack); see GetIsMessagePack().
 */
class RawRemoteIDPacket {
   public:
    // Maximum ODID payload: a full message pack (ODID_MessagePack_encoded). Kept in sync with the vendored
    // opendroneid-core-c ODID_MESSAGE_SIZE(25) * ODID_PACK_MAX_MESSAGES(9) + 3-byte pack header.
    static constexpr uint16_t kMaxPayloadLenBytes = 228;
    static constexpr uint16_t kSingleMessageLenBytes = 25;

    // ODID message type nibble (high nibble of the first payload byte) that indicates a message pack rather than a
    // single message. Matches ODID_MESSAGETYPE_PACKED in opendroneid.h.
    static constexpr uint8_t kMessageTypePacked = 0xF;

    enum Transport : uint8_t {
        kTransportInvalid = 0,
        kTransportBT4Legacy = 1,    // Bluetooth 4 legacy advertising (1M PHY), ASTM Service Data UUID 0xFFFA.
        kTransportBT5LongRange = 2,  // Bluetooth 5 Long Range (Coded PHY S=8) extended advertising.
        kTransportWiFiBeacon = 3,    // WiFi beacon frame, vendor-specific IE (OUI FA:0B:BC, type 0x0D).
    };

    RawRemoteIDPacket() = default;

    /**
     * Returns true if payload holds an ODID message pack rather than a single 25-byte message.
     */
    bool GetIsMessagePack() const {
        if (payload_len_bytes == 0) return false;
        return (payload[0] >> 4) == kMessageTypePacked;
    }

    uint32_t received_timestamp_ms = 0;  // Local receiver timestamp when the advertisement was received.
    uint8_t source_mac[6] = {0};         // Transmitter MAC / BLE address. Used as the de-dup key and provisional UID.
    int8_t rssi_dbm = 0;                 // Received signal strength, in dBm (0 if unknown).
    Transport transport = kTransportInvalid;
    uint8_t channel = 0;              // WiFi channel (1-13), or BLE PHY code, for diagnostics.
    uint8_t payload_len_bytes = 0;   // Number of valid bytes in payload.
    uint8_t reserved[2] = {0};       // Padding to keep the struct word-aligned (sizeof % 4 == 0).
    uint8_t payload[kMaxPayloadLenBytes] = {0};
};

// Keep word-aligned so CompositeArray can do direct (memcpy-free) array access on the SPI buffer.
static_assert(sizeof(RawRemoteIDPacket) % 4 == 0, "RawRemoteIDPacket must be word-aligned for CompositeArray access.");
