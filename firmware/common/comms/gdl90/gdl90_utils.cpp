#include "gdl90_utils.hh"

#include "comms.hh"

// Initialize static variables.
const uint16_t GDL90Reporter::kGDL90CRC16Table[] = {
    0x0,    0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad,
    0xe1ce, 0xf1ef, 0x1231, 0x210,  0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a,
    0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 0x2462, 0x3443, 0x420,  0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b,
    0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672, 0x1611, 0x630,  0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7, 0x840,  0x1861,
    0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96,
    0x1a71, 0xa50,  0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 0x6ca6, 0x7c87,
    0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0xc60,  0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0xe70,  0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a,
    0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0xa1,   0x30c2, 0x20e3,
    0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x2b1,  0x1290,
    0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x481,  0x7466, 0x6447, 0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e,
    0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x691,  0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d, 0xf90e, 0xe92f,
    0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x8e1,  0x3882, 0x28a3, 0xcb7d, 0xdb5c,
    0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0xaf1,  0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83,
    0x1ce0, 0xcc1,  0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74,
    0x2e93, 0x3eb2, 0xed1,  0x1ef0};

uint16_t GDL90Reporter::WriteGDL90Message(uint8_t* to_buf, uint16_t to_buf_num_bytes, uint8_t* unescaped_message,
                                          uint8_t unescaped_message_len_bytes) {
    uint16_t bytes_written = 0;
    if (to_buf_num_bytes < bytes_written + 1) {
        // Not enough space for starting flag byte.
        return bytes_written;
    }
    to_buf[bytes_written++] = kGDL90FlagByte;  // Beginning flag byte.
    bytes_written += WriteBufferWithGDL90Escapes(to_buf + bytes_written, to_buf_num_bytes - bytes_written,
                                                 unescaped_message, unescaped_message_len_bytes);
    // Calculate the CRC with unescaped message ID and data.
    uint16_t crc = CalculateGDL90CRC16(unescaped_message, unescaped_message_len_bytes);
    uint8_t crc_buf[sizeof(crc)] = {static_cast<uint8_t>(crc & 0xFF), static_cast<uint8_t>(crc >> 8)};  // LSB first.
    bytes_written +=
        WriteBufferWithGDL90Escapes(to_buf + bytes_written, to_buf_num_bytes - bytes_written, crc_buf, sizeof(crc));
    if (to_buf_num_bytes < bytes_written + 1) {
        // Not enough space for ending flag byte.
        return bytes_written;
    }
    to_buf[bytes_written++] = kGDL90FlagByte;  // Ending flag byte.
    return bytes_written;
}

uint16_t GDL90Reporter::WriteBufferWithGDL90Escapes(uint8_t* to_buf, uint16_t to_buf_num_bytes, const uint8_t* from_buf,
                                                    uint16_t from_buf_num_bytes) {
    uint16_t bytes_written = 0;
    for (uint16_t i = 0; i < from_buf_num_bytes && bytes_written + 1 < to_buf_num_bytes; i++) {
        if (from_buf[i] == kGDL90FlagByte || from_buf[i] == kGDL90ControlEscapeChar) {
            // Escape any occurrence of 0x7E and 0x7D.
            to_buf[bytes_written++] = kGDL90ControlEscapeChar;
            if (bytes_written + 1 >= to_buf_num_bytes) {
                // Not enough space to write escaped character.
                break;
            }
            to_buf[bytes_written++] = from_buf[i] ^ 0x20;
        } else {
            // Write other values as normal.
            to_buf[bytes_written++] = from_buf[i];
        }
    }
    return bytes_written;
}

uint16_t GDL90Reporter::WriteGDL90HeartbeatMessage(uint8_t* to_buf, uint16_t to_buf_num_bytes,
                                                   uint32_t timestamp_sec_since_0000z, uint16_t adsb_message_counts,
                                                   uint16_t uat_uplink_message_counts) {
    const uint16_t kMessageBufLenBytes = 7;
    uint8_t message_buf[kMessageBufLenBytes];
    // 1: Message ID
    message_buf[0] = kGDL90MessageIDHeartbeat;
    // 2: Status Byte 1
    message_buf[1] = (gnss_position_valid << 7)  // GPS Position Valid: Set valid when ownship GPS position available.
                     | (maintenance_required << 6)  // Maintenance Required: Set to 1 if indicating an error.
                     | uat_initialized;             // UAT Initialized: always set to 1.
    // 3: Status Byte 2
    message_buf[2] = (timestamp_sec_since_0000z >> 16) << 7  // Timestamp MS bit.
                     | (csa_requested << 6)                  // 1 = Conflict Situational Awareness has been requested.
                     | (csa_not_available << 5)  // 1 = Conflict Situational Awareness not available at this time.
                     | utc_timing_is_valid;      // 1 = UTC timing is valid.
    // 4-5: Timestamp
    message_buf[3] = timestamp_sec_since_0000z & 0xFF;         // Timestamp LSB.
    message_buf[4] = (timestamp_sec_since_0000z >> 8) & 0xFF;  // Timestamp MSB (missing MS bit).
    // 6-7: Message Counts
    adsb_message_counts = MIN(1023, adsb_message_counts);
    uat_uplink_message_counts = MIN(31, uat_uplink_message_counts);
    message_buf[5] = static_cast<uint8_t>(((uat_uplink_message_counts & 0x1F) << 3) |  // bits 7..3
                                          ((adsb_message_counts >> 8) & 0x03)          // bits 1..0
    );                                                                                 // bit 2 left as 0
    message_buf[6] = static_cast<uint8_t>(adsb_message_counts & 0xFF);

    return WriteGDL90Message(to_buf, to_buf_num_bytes, message_buf, kMessageBufLenBytes);
}

uint16_t GDL90Reporter::WriteGDL90InitMessage(uint8_t* to_buf, uint16_t to_buf_num_bytes) {
    const uint16_t kMessageBufLenBytes = 3;
    uint8_t message_buf[kMessageBufLenBytes];
    // 1: Message ID
    message_buf[0] = kGDL90MessageIDInitialization;
    // 2: Configuration Byte 1
    message_buf[1] = (audio_test << 6)       // 1 = Initiate audio test.
                     | (audio_inhibit << 1)  // 1 = Suppress GDL90 audio output.
                     | cdti_ok;              // 1 = Cockpit Traffic Display (CDTI) capability is operating.
    // 3: Configuration Byte 2
    message_buf[2] = (csa_audio_disable << 1)  // 1 = Disable GDL90 audible traffic alerts.
                     | csa_disable;            // 1 = Disable CSA traffic alerting.
    return WriteGDL90Message(to_buf, to_buf_num_bytes, message_buf, kMessageBufLenBytes);
}

uint16_t GDL90Reporter::WriteGDL90UplinkDataMessage(uint8_t* to_buf, uint16_t to_buf_num_bytes,
                                                    const uint8_t* uplink_payload, uint16_t uplink_payload_len_bytes,
                                                    uint32_t tor_80ns_ticks) {
    const uint16_t kUplinkDataMessagePayloadLenBytes = 432;
    if (uplink_payload_len_bytes > kUplinkDataMessagePayloadLenBytes) {
        CONSOLE_ERROR("GDL90Reporter::WriteGDL90UplinkDataMessage",
                      "Received uplink payload of length %d bytes, maximum is %d Bytes.", uplink_payload_len_bytes,
                      kGDL90MessageMaxLenBytes);
        return 0;
    }

    // We can't use the standard WriteGDL90Message function here because the uplink payload may be too long to create a
    // separate buffer for. Instead, we will manually send the flag byte and then write escaped bit fields using the
    // WriteBufferWithGDL90Escapes function.
    uint16_t bytes_written = 0;
    to_buf[bytes_written++] = kGDL90FlagByte;  // Beginning flag byte.

    const uint16_t kHeaderBufLenBytes = sizeof(GDL90MessageID) + 3;
    uint8_t header_buf[kHeaderBufLenBytes];
    header_buf[0] = kGDL90MessageIDUplinkData;
    header_buf[1] = tor_80ns_ticks & 0xFF;  // TOR is least significant Byte first.
    header_buf[2] = (tor_80ns_ticks >> 8) & 0xFF;
    header_buf[3] = (tor_80ns_ticks >> 16) & 0xFF;
    // Calculate partial CRC of the unescaped header, not including the initial flag byte.
    uint16_t crc = CalculateGDL90CRC16(header_buf, kHeaderBufLenBytes);

    bytes_written += WriteBufferWithGDL90Escapes(to_buf + bytes_written, to_buf_num_bytes - bytes_written, header_buf,
                                                 kHeaderBufLenBytes);
    bytes_written += WriteBufferWithGDL90Escapes(to_buf + bytes_written, to_buf_num_bytes - bytes_written,
                                                 uplink_payload, uplink_payload_len_bytes);
    // Calculate the CRC with unescaped header and payload.
    crc = CalculateGDL90CRC16(uplink_payload, uplink_payload_len_bytes, crc);
    uint8_t crc_buf[sizeof(crc)] = {static_cast<uint8_t>(crc & 0xFF), static_cast<uint8_t>(crc >> 8)};  // LSB first.
    bytes_written +=
        WriteBufferWithGDL90Escapes(to_buf + bytes_written, to_buf_num_bytes - bytes_written, crc_buf, sizeof(crc));
    to_buf[bytes_written++] = kGDL90FlagByte;  // Ending flag byte.
    return bytes_written;
}

uint16_t GDL90Reporter::WriteGDL90TargetReportMessage(uint8_t* to_buf, uint16_t to_buf_num_bytes,
                                                      const GDL90TargetReportData& data, bool ownship) {
    const uint16_t kMessageBufLenBytes = 28;
    const float kLatLonDegTo24BitFrac = 46603.3777778f;
    const float kHeadingTrackDegTo8BitFrac = 0.71111111111f;
    uint8_t message_buf[kMessageBufLenBytes];
    message_buf[0] = ownship ? kGDL90MessageIDOwnshipReport : kGDL90MessageIDTrafficReport;
    message_buf[1] = ((data.traffic_alert_status & 0xF) << 4)  // s: Traffic Alert Status
                     | (data.address_type & 0xF);              // t: Address Type
    // aa aa aa: 24-bit Participant Address
    message_buf[2] = (data.participant_address >> 16) & 0xFF;
    message_buf[3] = (data.participant_address >> 8) & 0xFF;
    message_buf[4] = data.participant_address & 0xFF;
    // ll ll ll: Latitude as a 24-bit signed binary fraction. Resolution = 180/2^23 degrees.
    int32_t latitude_frac = static_cast<int32_t>(data.latitude_deg * kLatLonDegTo24BitFrac) & 0x00FFFFFF;
    message_buf[5] = (latitude_frac >> 16) & 0xFF;
    message_buf[6] = (latitude_frac >> 8) & 0xFF;
    message_buf[7] = latitude_frac & 0xFF;
    // nn nn nn: Longitude as a 24-bit signed binary fraction. Resolution = 180/2^23 degrees.
    int32_t longitude_frac = static_cast<int32_t>(data.longitude_deg * kLatLonDegTo24BitFrac) & 0x00FFFFFF;
    message_buf[8] = (longitude_frac >> 16) & 0xFF;
    message_buf[9] = (longitude_frac >> 8) & 0xFF;
    message_buf[10] = longitude_frac & 0xFF;

    // ddd: Altitude as a 12-bit offset integer. Resolution = 25 feet. 0xFFF = invalid/unavailable.
    // Valid encoded range: 0x000 (-1000 ft) to 0xFFE (+101,350 ft).
    uint16_t altitude_frac;
    if (data.altitude_ft < -1000 || data.altitude_ft > 101350) {
        altitude_frac = 0xFFF;  // Out of range: encode as invalid.
    } else {
        altitude_frac = static_cast<uint16_t>((data.altitude_ft + 1000) / 25);
        altitude_frac = MIN(0xFFE, altitude_frac);  // Clamp to valid max; 0xFFF is reserved for invalid.
    }
    message_buf[11] = (altitude_frac >> 4) & 0xFF;    // dd: Altitude MS Byte.
    message_buf[12] = ((altitude_frac & 0xF) << 4) |  // d: Altitude LS nibble.
                      (data.misc_indicators & 0xF);   // m: Miscellaneous indicators.

    message_buf[13] =
        ((data.navigation_integrity_category & 0xF) << 4)      // i: Navigation Integrity Category (NIC).
        | (data.navigation_accuracy_category_position & 0xF);  // a: Navigation Accuracy Category for Position (NACp).

    // hhh: Horizontal Velocity. Resolution = 1kt. 0xFFF = invalid/unavailable, 0xFFE = >= 4094 kt.
    int32_t speed_kts_raw = static_cast<int32_t>(data.speed_kts);
    uint16_t speed_kts;
    if (speed_kts_raw < 0) {
        speed_kts = 0xFFF;  // Negative sentinel: encode as unavailable.
    } else if (speed_kts_raw >= 0xFFE) {
        speed_kts = 0xFFE;  // Clamp to max valid; 0xFFF reserved for unavailable.
    } else {
        speed_kts = static_cast<uint16_t>(speed_kts_raw);
    }

    // vvv: Vertical Velocity. Signed Integer in units of 64fpm. 0x800 = not available.
    uint16_t vertical_rate_encoded;
    if (data.vertical_rate_fpm == GDL90TargetReportData::kVerticalRateUnavailableFpm) {
        vertical_rate_encoded = 0x800;
    } else {
        int32_t vertical_rate_64fpm = static_cast<int32_t>(data.vertical_rate_fpm) / 64;
        if (vertical_rate_64fpm > 0x1FE) {
            vertical_rate_64fpm = 0x1FE;  // > +32,576 fpm climb; railed to +32,640 fpm (0x1FE)
        } else if (vertical_rate_64fpm < -0x1FE) {
            vertical_rate_64fpm = -0x1FE;  // > 32,576 fpm descent; railed to -32,640 fpm (0x1FE)
        }
        vertical_rate_encoded = static_cast<uint16_t>(vertical_rate_64fpm) & 0x0FFF;
    }

    message_buf[14] = (speed_kts >> 4) & 0xFF;                  // hh: MSB of Horizontal Velocity.
    message_buf[15] = ((speed_kts & 0xF) << 4)                  // h: LS nibble of Horizontal Velocity.
                      | ((vertical_rate_encoded >> 8) & 0x0F);  // v: MS nibble of Vertical Rate.
    message_buf[16] = vertical_rate_encoded & 0xFF;             // vv: LSB of Vertical Rate.

    // tt: Track / Heading. 8-bit angular weighted binary. Resolution = 360/256 degrees. 0 = North,
    // 128 = South. Indicate track or heading using misc bit field.
    message_buf[17] = static_cast<uint32_t>(data.direction_deg * kHeadingTrackDegTo8BitFrac) & 0x000000FF;
    // ee: Emitter Category.
    message_buf[18] = data.emitter_category;
    // cc cc cc cc cc cc cc cc: Call sign. 8 ASCII characters, 0-9 and A-Z.
    memcpy(message_buf + 19, data.callsign, 8);                  // message[19] - message[26]
    message_buf[27] = (data.emergency_priority_code & 0xF) << 4  // p: Emergency / Priority Code.
                      | 0b0;                                     // x: Spare
    return WriteGDL90Message(to_buf, to_buf_num_bytes, message_buf, kMessageBufLenBytes);
}

bool GDL90Reporter::IsValidOwnshipSource(SettingsManager::RxPosition::PositionSource source) {
    return source == SettingsManager::RxPosition::kPositionSourceGNSS ||
           source == SettingsManager::RxPosition::kPositionSourceAircraftMatchingICAO;
}

bool GDL90Reporter::BuildOwnshipReportData(GDL90TargetReportData& data,
                                           const SettingsManager::RxPosition& rx_position,
                                           bool rx_position_available) {
    data = {};
    memcpy(data.callsign, "ADSBEE  ", sizeof(data.callsign) - 1);
    data.address_type = GDL90TargetReportData::kAddressTypeADSBWithSelfAssignedAddress;

    bool have_position = rx_position_available && IsValidOwnshipSource(rx_position.source);
    if (!have_position) {
        return false;
    }

    bool from_tracked_aircraft = rx_position.source == SettingsManager::RxPosition::kPositionSourceAircraftMatchingICAO;
    data.latitude_deg = rx_position.latitude_deg;
    data.longitude_deg = rx_position.longitude_deg;
    // The ownship report altitude is pressure altitude. A GNSS fix carries no baro data, so report it invalid
    // (INT32_MIN encodes as 0xFFF) rather than substituting geometric altitude or a stale value.
    data.altitude_ft = from_tracked_aircraft ? rx_position.baro_altitude_ft : INT32_MIN;
    data.speed_kts = rx_position.speed_kts;
    data.direction_deg = rx_position.heading_deg;
    // Only carry a real ICAO address when bootstrapping off a tracked aircraft; a GNSS ownship is self-assigned.
    data.participant_address = from_tracked_aircraft ? rx_position.icao_address : 0x0;
    // Non-zero NIC so EFBs treat the position as valid. Conservative fixed accuracy category (<92.6 m).
    data.navigation_integrity_category = 8;
    data.navigation_accuracy_category_position = 8;
    data.SetMiscIndicator(GDL90TargetReportData::kMiscIndicatorTTIsTrueTrackAngle, false,
                          rx_position.speed_kts > kOwnshipAirborneSpeedKts);
    return true;
}

uint16_t GDL90Reporter::WriteGDL90TargetReportMessage(uint8_t* to_buf, uint16_t to_buf_num_bytes,
                                                      const ModeSAircraft& aircraft, bool ownship) {
    GDL90TargetReportData data;

    // NOTE: Traffic Alert Status currently not used.
    data.participant_address = aircraft.icao_address;
    data.address_type = GDL90TargetReportData::kAddressTypeADSBWithICAOAddress;

    data.latitude_deg = aircraft.latitude_deg;
    data.longitude_deg = aircraft.longitude_deg;
    data.altitude_ft = aircraft.baro_altitude_ft;
    data.direction_deg = aircraft.direction_deg;

    GDL90TargetReportData::MiscIndicatorTrackOrHeadingValue track_heading_value;
    if (!aircraft.HasBitFlag(ModeSAircraft::kBitFlagPositionValid)) {
        // No valid position.
        track_heading_value = GDL90TargetReportData::kMiscIndicatorTTNotValid;
    } else {
        // Valid position: indicate what kind of value the track angle / heading field is.
        if (aircraft.HasBitFlag(ModeSAircraft::kBitFlagDirectionIsHeading)) {
            // Aircraft is reporting heading instead of track.
            track_heading_value = aircraft.HasBitFlag(ModeSAircraft::kBitFlagHeadingUsesMagneticNorth)
                                      ? GDL90TargetReportData::kMiscIndicatorTTIsMagneticHeading
                                      : GDL90TargetReportData::kMiscIndicatorTTIsTrueHeading;
        } else {
            // Aircraft is reporting track angle.
            track_heading_value = GDL90TargetReportData::kMiscIndicatorTTIsTrueTrackAngle;
        }
    }
    bool aircraft_updated_position = aircraft.HasBitFlag(ModeSAircraft::kBitFlagUpdatedBaroAltitude) ||
                                     aircraft.HasBitFlag(ModeSAircraft::kBitFlagUpdatedGNSSAltitude) ||
                                     aircraft.HasBitFlag(ModeSAircraft::kBitFlagUpdatedHorizontalSpeed) ||
                                     aircraft.HasBitFlag(ModeSAircraft::kBitFlagUpdatedBaroVerticalRate) ||
                                     aircraft.HasBitFlag(ModeSAircraft::kBitFlagUpdatedGNSSVerticalRate) ||
                                     aircraft.HasBitFlag(ModeSAircraft::kBitFlagUpdatedPosition) ||
                                     aircraft.HasBitFlag(ModeSAircraft::kBitFlagUpdatedDirection);
    data.SetMiscIndicator(track_heading_value,
                          aircraft_updated_position,                              // Aircraft report updated?
                          aircraft.HasBitFlag(ModeSAircraft::kBitFlagIsAirborne)  // Aircraft is airborne?
    );
    data.navigation_integrity_category = aircraft.navigation_integrity_category;
    data.navigation_accuracy_category_position = aircraft.navigation_accuracy_category_position;
    data.speed_kts = aircraft.speed_kts;
    // Prefer baro vertical rate for GDL90 applications (e.g. electronic flight bag).
    data.vertical_rate_fpm = aircraft.HasBitFlag(ModeSAircraft::kBitFlagBaroVerticalRateValid)
                                 ? aircraft.baro_vertical_rate_fpm
                                 : aircraft.gnss_vertical_rate_fpm;
    data.direction_deg = aircraft.direction_deg;
    // Use the decoded enum, not emitter_category_raw: the Mode S raw value is (TypeCode<<3|Category)
    // which is not a valid GDL90 emitter category code.
    data.emitter_category = aircraft.emitter_category == ADSBTypes::kEmitterCategoryInvalid
                                ? 0u
                                : static_cast<uint8_t>(aircraft.emitter_category);
    // GDL90 does not provide space for an EOS character, since it only provides 8 Bytes for the callsign.
    memcpy(data.callsign, aircraft.callsign, ModeSAircraft::kCallSignMaxNumChars);
    // NOTE: Emergency Priority code currently not used.

    return WriteGDL90TargetReportMessage(to_buf, to_buf_num_bytes, data, ownship);
}

uint16_t GDL90Reporter::WriteGDL90TargetReportMessage(uint8_t* to_buf, uint16_t to_buf_num_bytes,
                                                      const UATAircraft& aircraft, bool ownship) {
    GDL90TargetReportData data;

    // NOTE: Traffic Alert Status currently not used.
    data.participant_address = aircraft.icao_address;
    data.latitude_deg = aircraft.latitude_deg;
    data.longitude_deg = aircraft.longitude_deg;
    data.altitude_ft = aircraft.baro_altitude_ft;
    data.direction_deg = aircraft.direction_deg;
    switch (aircraft.GetAddressQualifier()) {
        case UATAircraft::kADSBTargetWithICAO24BitAddress:
            data.address_type = GDL90TargetReportData::kAddressTypeADSBWithICAOAddress;
            break;
        case UATAircraft::kADSBTargetWithSelfAssignedTemporaryAddress:
            data.address_type = GDL90TargetReportData::kAddressTypeADSBWithSelfAssignedAddress;
            break;
        case UATAircraft::kTISBTargetWithICAO24BitAddress:
            data.address_type = GDL90TargetReportData::kAddressTypeTISBWithICAOAddress;
            break;
        case UATAircraft::kTISBTargetWithTrackFileIdentifier:
            data.address_type = GDL90TargetReportData::kAddressTypeTISBWithTrackFileID;
            break;
        case UATAircraft::kSurfaceVehicle:
            data.address_type = GDL90TargetReportData::kAddressTypeSurfaceVehicle;
            break;
        case UATAircraft::kFixedADSBBeacon:
            data.address_type = GDL90TargetReportData::kAddressTypeGroundStationBeacon;
            break;
        default:
            data.address_type = GDL90TargetReportData::kAddressTypeADSBWithICAOAddress;
            break;
    }

    GDL90TargetReportData::MiscIndicatorTrackOrHeadingValue track_heading_value;
    if (!aircraft.HasBitFlag(UATAircraft::kBitFlagPositionValid)) {
        // No valid position.
        track_heading_value = GDL90TargetReportData::kMiscIndicatorTTNotValid;
    } else {
        // Valid position: indicate what kind of value the track angle / heading field is.
        if (aircraft.HasBitFlag(UATAircraft::kBitFlagDirectionIsHeading)) {
            // Aircraft is reporting heading instead of track.
            track_heading_value = aircraft.HasBitFlag(UATAircraft::kBitFlagHeadingUsesMagneticNorth)
                                      ? GDL90TargetReportData::kMiscIndicatorTTIsMagneticHeading
                                      : GDL90TargetReportData::kMiscIndicatorTTIsTrueHeading;
        } else {
            // Aircraft is reporting track angle.
            track_heading_value = GDL90TargetReportData::kMiscIndicatorTTIsTrueTrackAngle;
        }
    }
    bool aircraft_updated_position = aircraft.HasBitFlag(UATAircraft::kBitFlagUpdatedBaroAltitude) ||
                                     aircraft.HasBitFlag(UATAircraft::kBitFlagUpdatedGNSSAltitude) ||
                                     aircraft.HasBitFlag(UATAircraft::kBitFlagUpdatedHorizontalSpeed) ||
                                     aircraft.HasBitFlag(UATAircraft::kBitFlagUpdatedBaroVerticalRate) ||
                                     aircraft.HasBitFlag(UATAircraft::kBitFlagUpdatedGNSSVerticalRate) ||
                                     aircraft.HasBitFlag(UATAircraft::kBitFlagUpdatedPosition) ||
                                     aircraft.HasBitFlag(UATAircraft::kBitFlagUpdatedDirection);
    data.SetMiscIndicator(track_heading_value,
                          aircraft_updated_position,                            // Aircraft report updated?
                          aircraft.HasBitFlag(UATAircraft::kBitFlagIsAirborne)  // Aircraft is airborne?
    );
    data.navigation_integrity_category = aircraft.navigation_integrity_category;
    data.navigation_accuracy_category_position = aircraft.navigation_accuracy_category_position;
    data.speed_kts = aircraft.speed_kts;
    // Prefer baro vertical rate for GDL90 applications (e.g. electronic flight bag).
    data.vertical_rate_fpm = aircraft.HasBitFlag(UATAircraft::kBitFlagBaroVerticalRateValid)
                                 ? aircraft.baro_vertical_rate_fpm
                                 : aircraft.gnss_vertical_rate_fpm;
    data.direction_deg = aircraft.direction_deg;
    data.emitter_category = aircraft.emitter_category_raw;
    // GDL90 does not provide space for an EOS character, since it only provides 8 Bytes for the callsign.
    memcpy(data.callsign, aircraft.callsign, UATAircraft::kCallSignMaxNumChars);
    // NOTE: Emergency Priority code currently not used.

    return WriteGDL90TargetReportMessage(to_buf, to_buf_num_bytes, data, ownship);
}

uint16_t GDL90Reporter::WriteGDL90TargetReportMessage(uint8_t* to_buf, uint16_t to_buf_num_bytes,
                                                      const RemoteIDAircraft& aircraft, bool ownship) {
    GDL90TargetReportData data;

    data.participant_address = aircraft.address;
    // Remote ID uses self-assigned (non-ICAO) identifiers.
    data.address_type = GDL90TargetReportData::kAddressTypeADSBWithSelfAssignedAddress;
    data.latitude_deg = aircraft.latitude_deg;
    data.longitude_deg = aircraft.longitude_deg;
    // Drones report geometric (WGS84) altitude; GDL90 altitude is nominally pressure altitude, but geometric is the best
    // value we have for a drone.
    data.altitude_ft =
        aircraft.HasBitFlag(RemoteIDAircraft::kBitFlagGNSSAltitudeValid) ? aircraft.gnss_altitude_ft : INT32_MIN;
    data.direction_deg = aircraft.direction_deg;

    // Only claim the track/heading field holds a true track angle when a direction was actually received: Remote ID
    // reports direction independently of position, so a drone with a valid position but no direction would otherwise be
    // reported as tracking 0 degrees (or a stale value) rather than "unknown".
    GDL90TargetReportData::MiscIndicatorTrackOrHeadingValue track_heading_value =
        (aircraft.HasBitFlag(RemoteIDAircraft::kBitFlagPositionValid) &&
         aircraft.HasBitFlag(RemoteIDAircraft::kBitFlagDirectionValid))
            ? GDL90TargetReportData::kMiscIndicatorTTIsTrueTrackAngle
            : GDL90TargetReportData::kMiscIndicatorTTNotValid;
    data.SetMiscIndicator(track_heading_value, /*report_is_extrapolated=*/false,
                          aircraft.HasBitFlag(RemoteIDAircraft::kBitFlagIsAirborne));

    // NIC/NAC unknown for Remote ID.
    data.navigation_integrity_category = 0;
    data.navigation_accuracy_category_position = 0;
    data.speed_kts =
        aircraft.HasBitFlag(RemoteIDAircraft::kBitFlagHorizontalSpeedValid) ? (float)aircraft.speed_kts : -1.0f;
    // A Remote ID Location message may omit vertical speed; report it as unavailable (0x800) rather than a real 0 fpm.
    data.vertical_rate_fpm = aircraft.HasBitFlag(RemoteIDAircraft::kBitFlagVerticalRateValid)
                                 ? aircraft.gnss_vertical_rate_fpm
                                 : GDL90TargetReportData::kVerticalRateUnavailableFpm;
    data.emitter_category = ADSBTypes::kEmitterCategoryUnmannedAerialVehicle;  // 14 = UAV.

    // GDL90 callsign is 8 chars; use the tail of the UAS serial (the distinctive part), padded with spaces.
    memset(data.callsign, ' ', 8);
    data.callsign[8] = '\0';
    if (aircraft.HasBitFlag(RemoteIDAircraft::kBitFlagBasicIDValid)) {
        size_t id_len = strnlen(aircraft.uas_id, RemoteIDAircraft::kIDLenChars);
        size_t copy_len = id_len < 8 ? id_len : 8;
        memcpy(data.callsign, aircraft.uas_id + (id_len - copy_len), copy_len);
    }

    return WriteGDL90TargetReportMessage(to_buf, to_buf_num_bytes, data, ownship);
}