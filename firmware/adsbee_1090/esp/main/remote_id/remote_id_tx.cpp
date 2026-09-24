#include "remote_id_tx.hh"

#include <cstdio>
#include <cstring>

#include "device_info.hh"        // GetESP32DeviceInfo (base MAC) for the default UAS ID.
#include "object_dictionary.hh"  // rx_position pushed up from the RP2040.
#include "opendroneid.h"         // Vendored Open Drone ID codec (encode path).
#include "settings.hh"
#include "unit_conversions.hh"

// The working Open Drone ID data set. Kept at file scope rather than as a class member so that opendroneid.h stays out
// of remote_id_tx.hh (ODID_UAS_Data is ~900 B, and only this translation unit needs it).
static ODID_UAS_Data s_uas_data;

// Conversions from the ADSBee-standard units carried in RxPosition (feet, knots) to the ODID wire units (meters, m/s).
static constexpr float kMetersPerFoot = 0.3048f;
static constexpr float kMetersPerSecondPerKnot = 0.514444f;

void RemoteIDTransmitter::GetEffectiveUASID(char* out, uint16_t out_len_bytes) {
    if (out == nullptr || out_len_bytes == 0) return;
    const char* configured = settings_manager.settings.remote_id_tx_uas_id;
    if (configured[0] != '\0') {
        strncpy(out, configured, out_len_bytes - 1);
        out[out_len_bytes - 1] = '\0';
        return;
    }
    // No UAS ID configured: derive a stable one from this device's base MAC so an unconfigured unit is still a usable
    // test transmitter with a unique identity. Formatted like an ANSI/CTA-2063-A style serial ("ADSBee" + MAC digits).
    ObjectDictionary::ESP32DeviceInfo device_info = GetESP32DeviceInfo();
    snprintf(out, out_len_bytes, "ADSBEE%02X%02X%02X%02X%02X%02X", device_info.base_mac[0], device_info.base_mac[1],
             device_info.base_mac[2], device_info.base_mac[3], device_info.base_mac[4], device_info.base_mac[5]);
}

void RemoteIDTransmitter::PopulateBasicID() {
    ODID_BasicID_data& basic_id = s_uas_data.BasicID[0];
    memset(&basic_id, 0, sizeof(basic_id));
    basic_id.IDType = static_cast<ODID_idtype_t>(settings_manager.settings.remote_id_tx_uas_id_type);
    basic_id.UAType = static_cast<ODID_uatype_t>(settings_manager.settings.remote_id_tx_ua_type);
    GetEffectiveUASID(basic_id.UASID, sizeof(basic_id.UASID));
    s_uas_data.BasicIDValid[0] = 1;
}

void RemoteIDTransmitter::PopulateLocation(bool position_valid) {
    const SettingsManager::RxPosition& pos = object_dictionary.composite_device_status.rp2040.rx_position;
    ODID_Location_data& loc = s_uas_data.Location;
    memset(&loc, 0, sizeof(loc));

    // Accuracy fields are reported as unknown: the position comes from a fixed configuration or from a GNSS fix, and
    // RxPosition carries no accuracy estimate for either.
    loc.HorizAccuracy = ODID_HOR_ACC_UNKNOWN;
    loc.VertAccuracy = ODID_VER_ACC_UNKNOWN;
    loc.BaroAccuracy = ODID_VER_ACC_UNKNOWN;
    loc.SpeedAccuracy = ODID_SPEED_ACC_UNKNOWN;
    loc.TSAccuracy = ODID_TIME_ACC_UNKNOWN;
    loc.HeightType = ODID_HEIGHT_REF_OVER_TAKEOFF;
    loc.Height = INV_ALT;
    loc.TimeStamp = INV_TIMESTAMP;

    if (!position_valid) {
        // Transmit an "unknown position" Location so receivers still see and identify the transmitter.
        loc.Status = ODID_STATUS_UNDECLARED;
        loc.Direction = INV_DIR;
        loc.SpeedHorizontal = INV_SPEED_H;
        loc.SpeedVertical = INV_SPEED_V;
        loc.Latitude = 0.0;
        loc.Longitude = 0.0;
        loc.AltitudeBaro = INV_ALT;
        loc.AltitudeGeo = INV_ALT;
        s_uas_data.LocationValid = 1;
        return;
    }

    // A moving transmitter is declared airborne; a stationary bench transmitter is declared "on ground".
    loc.Status = (pos.speed_kts > 0) ? ODID_STATUS_AIRBORNE : ODID_STATUS_GROUND;
    loc.Latitude = pos.latitude_deg;
    loc.Longitude = pos.longitude_deg;
    loc.AltitudeGeo = static_cast<float>(pos.gnss_altitude_ft) * kMetersPerFoot;
    loc.AltitudeBaro = static_cast<float>(pos.baro_altitude_ft) * kMetersPerFoot;
    loc.SpeedHorizontal = static_cast<float>(pos.speed_kts) * kMetersPerSecondPerKnot;
    loc.SpeedVertical = 0.0f;  // RxPosition carries no vertical rate.
    // ODID direction is 0 <= x < 360; anything else must be sent as invalid.
    loc.Direction = (pos.heading_deg >= 0.0f && pos.heading_deg < 360.0f) ? pos.heading_deg : INV_DIR;

    s_uas_data.LocationValid = 1;
}

void RemoteIDTransmitter::PopulateSystem(bool position_valid) {
    const SettingsManager::RxPosition& pos = object_dictionary.composite_device_status.rp2040.rx_position;
    ODID_System_data& system = s_uas_data.System;
    memset(&system, 0, sizeof(system));

    // The ADSBee transmits its own position as both the UA position and the operator position: as a bench test
    // transmitter the two are genuinely the same point, and on a drone no separate ground-station position is
    // available. Both therefore ride on the same position_valid gate, so a source we may not transmit suppresses the
    // operator location too rather than leaking it here.
    system.OperatorLocationType = ODID_OPERATOR_LOCATION_TYPE_TAKEOFF;
    system.ClassificationType = ODID_CLASSIFICATION_TYPE_UNDECLARED;
    system.CategoryEU = ODID_CATEGORY_EU_UNDECLARED;
    system.ClassEU = ODID_CLASS_EU_UNDECLARED;
    system.AreaCount = 1;
    system.AreaRadius = 0;
    system.AreaCeiling = INV_ALT;
    system.AreaFloor = INV_ALT;
    system.Timestamp = 0;

    if (position_valid) {
        system.OperatorLatitude = pos.latitude_deg;
        system.OperatorLongitude = pos.longitude_deg;
        system.OperatorAltitudeGeo = static_cast<float>(pos.gnss_altitude_ft) * kMetersPerFoot;
    } else {
        system.OperatorLatitude = 0.0;
        system.OperatorLongitude = 0.0;
        system.OperatorAltitudeGeo = INV_ALT;
    }

    s_uas_data.SystemValid = 1;
}

bool RemoteIDTransmitter::PopulateOperatorID() {
    const char* operator_id = settings_manager.settings.remote_id_tx_operator_id;
    if (operator_id[0] == '\0') {
        s_uas_data.OperatorIDValid = 0;
        return false;  // Not configured: the Operator ID message is simply not transmitted.
    }
    ODID_OperatorID_data& op = s_uas_data.OperatorID;
    memset(&op, 0, sizeof(op));
    op.OperatorIdType = ODID_OPERATOR_ID;
    strncpy(op.OperatorId, operator_id, sizeof(op.OperatorId) - 1);
    op.OperatorId[sizeof(op.OperatorId) - 1] = '\0';
    s_uas_data.OperatorIDValid = 1;
    return true;
}

bool RemoteIDTransmitter::RefreshFromDeviceState() {
    odid_initUasData(&s_uas_data);

    // The RP2040 pushes its resolved receiver position at 1 Hz as part of its device status, along with the source it
    // was resolved from. Only sources that describe THIS device may be broadcast as our own Remote ID position (see
    // SettingsManager::RxPosition::MayBeTransmittedAsOwnPosition); anything else is treated exactly like "no position
    // available", so the Location and System messages fall through to their ODID "unknown" branches below and no
    // foreign coordinate can escape. This is the single choke point for every transmit transport: BLE legacy, BLE
    // Coded PHY and the WiFi beacon all encode from the s_uas_data populated here.
    const SettingsManager::RxPosition& pos = object_dictionary.composite_device_status.rp2040.rx_position;
    position_source_allowed_ = SettingsManager::RxPosition::MayBeTransmittedAsOwnPosition(pos.source);
    position_valid_ =
        object_dictionary.composite_device_status.rp2040.rx_position_available && position_source_allowed_;

    PopulateBasicID();
    PopulateLocation(position_valid_);
    PopulateSystem(position_valid_);
    has_operator_id_ = PopulateOperatorID();

    return position_valid_;
}

uint16_t RemoteIDTransmitter::BuildMessagePack(uint8_t* buf) {
    if (buf == nullptr) return 0;

    // Assemble the individual encoded messages, then wrap them in a message pack. We encode by hand rather than using
    // the library's odid_message_build_pack(), which lives in the (non-vendored, non-portable) wifi.c.
    ODID_MessagePack_data pack;
    memset(&pack, 0, sizeof(pack));
    pack.SingleMessageSize = ODID_MESSAGE_SIZE;
    pack.MsgPackSize = 0;

    if (s_uas_data.BasicIDValid[0] &&
        encodeBasicIDMessage(reinterpret_cast<ODID_BasicID_encoded*>(&pack.Messages[pack.MsgPackSize]),
                             &s_uas_data.BasicID[0]) == ODID_SUCCESS) {
        pack.MsgPackSize++;
    }
    if (s_uas_data.LocationValid &&
        encodeLocationMessage(reinterpret_cast<ODID_Location_encoded*>(&pack.Messages[pack.MsgPackSize]),
                              &s_uas_data.Location) == ODID_SUCCESS) {
        pack.MsgPackSize++;
    }
    if (s_uas_data.SystemValid &&
        encodeSystemMessage(reinterpret_cast<ODID_System_encoded*>(&pack.Messages[pack.MsgPackSize]),
                            &s_uas_data.System) == ODID_SUCCESS) {
        pack.MsgPackSize++;
    }
    if (s_uas_data.OperatorIDValid &&
        encodeOperatorIDMessage(reinterpret_cast<ODID_OperatorID_encoded*>(&pack.Messages[pack.MsgPackSize]),
                                &s_uas_data.OperatorID) == ODID_SUCCESS) {
        pack.MsgPackSize++;
    }

    if (pack.MsgPackSize == 0) return 0;

    ODID_MessagePack_encoded encoded_pack;
    if (encodeMessagePack(&encoded_pack, &pack) != ODID_SUCCESS) return 0;

    // A message pack on the wire is the 3-byte pack header followed by MsgPackSize whole messages.
    uint16_t pack_len_bytes = 3 + pack.MsgPackSize * ODID_MESSAGE_SIZE;
    if (pack_len_bytes > kMaxPackLenBytes) return 0;
    memcpy(buf, &encoded_pack, pack_len_bytes);
    return pack_len_bytes;
}

uint16_t RemoteIDTransmitter::BuildNextSingleMessage(uint8_t* buf) {
    if (buf == nullptr) return 0;

    // Walk the round-robin schedule, skipping slots whose message isn't available (e.g. no operator ID configured), so
    // a slot never yields an empty advertisement. Bounded by the slot count so an all-empty set terminates.
    for (uint8_t attempt = 0; attempt < kNumSingleMessageSlots; attempt++) {
        uint8_t slot = single_message_slot_;
        single_message_slot_ = (single_message_slot_ + 1) % kNumSingleMessageSlots;

        switch (slot) {
            case kSlotLocation0:
            case kSlotLocation1:
            case kSlotLocation2:
                if (s_uas_data.LocationValid &&
                    encodeLocationMessage(reinterpret_cast<ODID_Location_encoded*>(buf), &s_uas_data.Location) ==
                        ODID_SUCCESS) {
                    return kSingleMessageLenBytes;
                }
                break;
            case kSlotBasicID:
                if (s_uas_data.BasicIDValid[0] &&
                    encodeBasicIDMessage(reinterpret_cast<ODID_BasicID_encoded*>(buf), &s_uas_data.BasicID[0]) ==
                        ODID_SUCCESS) {
                    return kSingleMessageLenBytes;
                }
                break;
            case kSlotSystem:
                if (s_uas_data.SystemValid &&
                    encodeSystemMessage(reinterpret_cast<ODID_System_encoded*>(buf), &s_uas_data.System) ==
                        ODID_SUCCESS) {
                    return kSingleMessageLenBytes;
                }
                break;
            case kSlotOperatorID:
                if (s_uas_data.OperatorIDValid &&
                    encodeOperatorIDMessage(reinterpret_cast<ODID_OperatorID_encoded*>(buf), &s_uas_data.OperatorID) ==
                        ODID_SUCCESS) {
                    return kSingleMessageLenBytes;
                }
                break;
            default:
                break;
        }
    }
    return 0;
}

uint8_t RemoteIDTransmitter::NextMessageCounter(RawRemoteIDPacket::Transport transport) {
    uint8_t index = static_cast<uint8_t>(transport);
    if (index >= sizeof(message_counters_) / sizeof(message_counters_[0])) index = 0;
    return message_counters_[index]++;
}
