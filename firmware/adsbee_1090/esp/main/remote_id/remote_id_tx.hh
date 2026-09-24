#pragma once

#include <cstdint>

#include "remote_id_packet.hh"
#include "settings.hh"  // SettingsManager::RxPosition::PositionSource.

/**
 * Builds the Open Drone ID (ASTM F3411) messages that the ADSBee transmits when it acts as a Broadcast Remote ID
 * transmitter, either as a bench test transmitter for checking Remote ID receiver performance, or mounted on a drone
 * as a combined ADS-B receiver / Remote ID transmitter.
 *
 * This class owns only the *content* of what is transmitted; the radios are driven by RemoteIDManager
 * (remote_id_ble.cpp advertises the bytes over BLE, remote_id_wifi_tx.cpp injects them in a WiFi beacon).
 *
 * Message content is assembled from:
 *   - The UA (drone) position: ObjectDictionary::CompositeDeviceStatus::rp2040.rx_position, which the RP2040 pushes at
 *     1 Hz. Only the position sources that describe THIS device are transmitted (see
 *     SettingsManager::RxPosition::MayBeTransmittedAsOwnPosition): a fixed coordinate for a bench transmitter
 *     (AT+RX_POSITION=FIXED) or this device's own GNSS fix on a drone. When no position is available -- because none
 *     has been acquired, or because the configured source is not one we may transmit -- the Location message is still
 *     transmitted, with ODID "unknown" sentinels, so receivers can still see and identify the transmitter.
 *   - Identity settings: remote_id_tx_uas_id / _uas_id_type / _ua_type / _operator_id. An empty UAS ID falls back to a
 *     serial derived from this device's own part code, so an unconfigured unit is still a usable test transmitter.
 *
 * Transmit cadence follows ASTM F3411: the dynamic Location/Vector message goes out at >= 1 Hz, while the static
 * identity messages (Basic ID, System, Operator ID) are interleaved at a lower rate. BLE legacy advertising (BT4) can
 * only carry ONE 25-byte ODID message per advertisement, so GetNextLegacyMessage() round-robins the message types with
 * Location in every other slot. BT5 Long Range and WiFi beacons carry a full message pack, built by BuildMessagePack().
 */
class RemoteIDTransmitter {
   public:
    // A single 25-byte Open Drone ID message (ODID_MESSAGE_SIZE).
    static constexpr uint8_t kSingleMessageLenBytes = 25;
    // Max message pack: 3-byte pack header + up to 9 messages. Matches RawRemoteIDPacket::kMaxPayloadLenBytes.
    static constexpr uint16_t kMaxPackLenBytes = RawRemoteIDPacket::kMaxPayloadLenBytes;

    /**
     * Refreshes the cached ODID data (position, identity) from the object dictionary and settings. Call once per
     * transmit tick, before building messages. Returns true if a valid UA position was available AND its source is one
     * we may transmit; when false, the Location message is still built but carries ODID "unknown" values.
     */
    bool RefreshFromDeviceState();

    /**
     * Whether the position source seen by the last RefreshFromDeviceState() was one we may transmit. Lets the caller
     * tell "configured correctly but no position yet" (e.g. waiting on a GNSS fix) apart from "the configured position
     * source may never be transmitted", which needs the user to change a setting.
     */
    bool PositionSourceAllowed() const { return position_source_allowed_; }

    /**
     * Builds a full Open Drone ID message pack (Basic ID + Location + System + Operator ID, as available) into buf.
     * Used for BT5 Long Range extended advertising and WiFi beacon frames, which can carry the whole pack.
     * @param[out] buf Buffer of at least kMaxPackLenBytes.
     * @retval Number of bytes written, or 0 on failure.
     */
    uint16_t BuildMessagePack(uint8_t* buf);

    /**
     * Builds the next single 25-byte message in the round-robin schedule, for BT4 legacy advertising (whose 31-byte
     * advertising payload only fits one message). Location is returned in every other call so it stays >= 1 Hz at the
     * normal advertising cadence; the static messages share the remaining slots.
     * @param[out] buf Buffer of at least kSingleMessageLenBytes.
     * @retval Number of bytes written (kSingleMessageLenBytes), or 0 on failure.
     */
    uint16_t BuildNextSingleMessage(uint8_t* buf);

    /**
     * Returns the ODID message counter for the next transmission on a given transport and increments it. Remote ID
     * receivers use this to detect dropped/duplicate advertisements; each transport keeps its own counter.
     */
    uint8_t NextMessageCounter(RawRemoteIDPacket::Transport transport);

    /**
     * Copies the effective UAS ID (the configured one, or the device-serial-derived fallback) into out, NUL terminated.
     * Exposed so the console can report what is actually being advertised.
     */
    void GetEffectiveUASID(char* out, uint16_t out_len_bytes);

   private:
    // Round-robin slot for BT4 legacy single-message advertising.
    enum SingleMessageSlot : uint8_t {
        kSlotLocation0 = 0,
        kSlotBasicID,
        kSlotLocation1,
        kSlotSystem,
        kSlotLocation2,
        kSlotOperatorID,
        kNumSingleMessageSlots
    };

    // Populates the cached ODID_UAS_Data (declared opaquely here to keep opendroneid.h out of this header).
    void PopulateBasicID();
    void PopulateLocation(bool position_valid);
    void PopulateSystem(bool position_valid);
    bool PopulateOperatorID();  // Returns false when no operator ID is configured (message is then not transmitted).

    uint8_t single_message_slot_ = 0;
    uint8_t message_counters_[4] = {0};  // Indexed by RawRemoteIDPacket::Transport.
    bool has_operator_id_ = false;
    bool position_valid_ = false;
    // Starts false so that, before the first refresh, nothing reports a transmittable position source.
    bool position_source_allowed_ = false;
};
