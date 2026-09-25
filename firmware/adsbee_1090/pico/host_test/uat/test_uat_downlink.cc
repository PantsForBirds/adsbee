#include "aircraft_dictionary.hh"
#include "buffer_utils.hh"
#include "fec.hh"
#include "gtest/gtest.h"
#include "hal_god_powers.hh" /* For faking timestamps */
#include "uat_packet.hh"
#include "uat_test_data.h"

#define USE_FEC

#ifdef USE_FEC
/**
 * Encodes a UAT downlink test frame into a byte buffer with FEC applied.
 * @param[in] frame Pointer to the test frame to encode.
 * @param[out] encoded_data_frame Pointer to the buffer to write the encoded frame to. Must be at least
 * RawUATADSBPacket::kADSBMessageMaxSizeBytes in size.
 * @retval Length of the encoded frame in bytes, including FEC parity bytes.
 */
uint16_t EncodeDownlinkFrame(const uat_downlink_test_frame_t* frame, uint8_t* encoded_data_frame) {
    HexStringToByteBuffer(encoded_data_frame, frame->frame_data_hex, frame->frame_length);
    uint16_t frame_length_with_fec;
    if (frame->frame_length == RawUATADSBPacket::kShortADSBMessagePayloadNumBytes) {
        uat_rs.EncodeShortADSBMessage(encoded_data_frame);
        frame_length_with_fec = RawUATADSBPacket::kShortADSBMessageNumBytes;
    } else {
        uat_rs.EncodeLongADSBMessage(encoded_data_frame);
        frame_length_with_fec = RawUATADSBPacket::kLongADSBMessageNumBytes;
    }
    return frame_length_with_fec;
}
#endif

// Test downlink frames
TEST(UATDecoderTest, DownlinkFrames) {
    int count = get_uat_downlink_test_frames_count();
    for (int i = 0; i < count; i++) {
        const uat_downlink_test_frame_t* frame = get_uat_downlink_test_frame(i);
        char message[200] = {"\0"};
        sprintf(message, "Test frame %d: %s, icao=%08X", i, frame->test_name, frame->address);
        SCOPED_TRACE(message);

        AircraftDictionary dictionary;
        UATAircraft aircraft;

#ifdef USE_FEC
        // Create encoded data frame.
        uint8_t encoded_data_frame[RawUATADSBPacket::kADSBMessageMaxSizeBytes] = {0};
        uint16_t frame_length_with_fec = EncodeDownlinkFrame(frame, encoded_data_frame);
        PrintByteBuffer("Encoded downlink message:", encoded_data_frame, frame_length_with_fec);
        int16_t sigs_dbm = -10;                // Dummy signal strength.
        int16_t sigq_bits = 0;                 // Dummy signal quality.
        uint64_t mlat_48mhz_64bit_counts = 0;  // Dummy timestamp.
        DecodedUATADSBPacket packet(
            RawUATADSBPacket(encoded_data_frame, frame_length_with_fec, sigs_dbm, sigq_bits, mlat_48mhz_64bit_counts));
#else
        // Create the packet, force it as valid (no FEC included in test data), ingest into dictionary.
        DecodedUATADSBPacket packet(frame->frame_data_hex);
        packet.ReconstructWithoutFEC();
#endif

        EXPECT_TRUE(dictionary.IngestDecodedUATADSBPacket(packet));
        // Increment the packet timestamp and ingest it again to satisfy the position filter.
        // NOTE: Ingest timestamps are done with system time, so fake it with HAL god powers.
        inc_time_since_boot_ms(1000);
        EXPECT_TRUE(dictionary.IngestDecodedUATADSBPacket(packet));

        // Ensure the dictionary has a matching aircraft entry and extract it.
        uint32_t address = frame->address | (frame->address_qualifier << Aircraft::kAddressQualifierBitShift);
        EXPECT_TRUE(dictionary.GetAircraft(Aircraft::ICAOToUID(address, Aircraft::kAircraftTypeUAT), aircraft));

        // Compare fields in aircraft entry against fields in test data.
        if (frame->has_sv) {
            EXPECT_TRUE(packet.has_state_vector);
            if (frame->position_valid) {
                EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagPositionValid));
                EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagUpdatedPosition));
                EXPECT_FLOAT_EQ(aircraft.latitude_deg, frame->lat);
                EXPECT_FLOAT_EQ(aircraft.longitude_deg, frame->lon);
            } else {
                EXPECT_FALSE(aircraft.HasBitFlag(UATAircraft::kBitFlagPositionValid));
                EXPECT_FALSE(aircraft.HasBitFlag(UATAircraft::kBitFlagUpdatedPosition));
            }

            // Subtract 1 from frame's altitude_type to convert to enum format.
            ADSBTypes::AltitudeSource altitude_source =
                static_cast<ADSBTypes::AltitudeSource>(frame->altitude_type - 1);
            switch (altitude_source) {
                case (ADSBTypes::kAltitudeSourceBaro):
                    EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagBaroAltitudeValid));
                    EXPECT_EQ(frame->has_auxsv && frame->sec_altitude != 0,
                              aircraft.HasBitFlag(UATAircraft::kBitFlagGNSSAltitudeValid));

                    EXPECT_EQ(aircraft.baro_altitude_ft, frame->altitude);

                    if (frame->has_auxsv) {
                        EXPECT_EQ(aircraft.gnss_altitude_ft, frame->sec_altitude);
                    }
                    break;
                case (ADSBTypes::kAltitudeSourceGNSS):
                    EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagGNSSAltitudeValid));
                    EXPECT_EQ(frame->has_auxsv && frame->sec_altitude != 0,
                              aircraft.HasBitFlag(UATAircraft::kBitFlagBaroAltitudeValid));

                    EXPECT_EQ(aircraft.gnss_altitude_ft, frame->altitude);

                    if (frame->has_auxsv) {
                        EXPECT_EQ(aircraft.baro_altitude_ft, frame->sec_altitude);
                    }
                    break;
                default:
                    // Handle kAltitudeSourceNotAvailable and kAltitudeSourceNotSet.
                    break;
            }

            // Check NIC
            EXPECT_EQ(aircraft.navigation_integrity_category, frame->nic);

            // Byte 17 bits 5-8: UTC coupled flag (ADS-B) or TIS-B site ID (address qualifier 2 or 3).
            if (frame->address_qualifier == UATAircraft::kTISBTargetWithICAO24BitAddress ||
                frame->address_qualifier == UATAircraft::kTISBTargetWithTrackFileIdentifier) {
                EXPECT_EQ(aircraft.tis_b_site_id, frame->tisb_site_id);
            } else {
                EXPECT_EQ(aircraft.utc_coupled, frame->utc_coupled != 0);
            }

            // Check Air/Ground State and Horizontal Velocity

            EXPECT_EQ(aircraft.HasBitFlag(UATAircraft::kBitFlagIsAirborne), (frame->airground_state & 0b10) == 0);

            if (frame->airground_state & 0b1) {
                // Aircraft is on ground. Reporting heading and ground speed.

                // WARNING: This test case doesn't get reached in the current data.
                EXPECT_TRUE(false);

                // Ground speed and heading format (aircraft on ground).
                EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagHorizontalSpeedValid));
                EXPECT_FALSE(aircraft.HasBitFlag(UATAircraft::kBitFlagUpdatedHorizontalSpeed));

                EXPECT_EQ(aircraft.speed_kts, frame->speed);

                EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagDirectionValid));
                EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagUpdatedDirection));

                EXPECT_NEAR(aircraft.direction_deg, frame->track, 0.5f);  // Test data is rounded.

                // TODO: Check length/width code.

            } else if (frame->ns_vel_valid && frame->ew_vel_valid) {
                // Airborne. Track provided in North Velocity + East Velocity format.
                EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagHorizontalSpeedValid));
                EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagUpdatedHorizontalSpeed));

                EXPECT_EQ(aircraft.speed_kts, frame->speed);

                EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagDirectionValid));
                EXPECT_FALSE(aircraft.HasBitFlag(UATAircraft::kBitFlagDirectionIsHeading));
                EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagUpdatedDirection));

                EXPECT_NEAR(aircraft.direction_deg, frame->track, 0.5f);  // Test data is rounded.

                // Vertical speed
                switch (frame->vert_rate_source) {
                    case UAT_ALT_BARO:
                        EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagBaroVerticalRateValid));
                        EXPECT_EQ(aircraft.baro_vertical_rate_fpm, frame->vert_rate);
                        EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagUpdatedBaroVerticalRate));
                        EXPECT_FALSE(aircraft.HasBitFlag(UATAircraft::kBitFlagGNSSVerticalRateValid));
                        EXPECT_FALSE(aircraft.HasBitFlag(UATAircraft::kBitFlagUpdatedGNSSVerticalRate));
                        break;
                    case UAT_ALT_GEO:
                        EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagGNSSVerticalRateValid));
                        EXPECT_EQ(aircraft.gnss_vertical_rate_fpm, frame->vert_rate);
                        EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagUpdatedGNSSVerticalRate));
                        EXPECT_FALSE(aircraft.HasBitFlag(UATAircraft::kBitFlagBaroVerticalRateValid));
                        EXPECT_FALSE(aircraft.HasBitFlag(UATAircraft::kBitFlagUpdatedBaroVerticalRate));
                        break;
                    default:
                        // No vertical speed information available.
                        EXPECT_FALSE(aircraft.HasBitFlag(UATAircraft::kBitFlagBaroVerticalRateValid));
                        EXPECT_FALSE(aircraft.HasBitFlag(UATAircraft::kBitFlagGNSSVerticalRateValid));
                        EXPECT_FALSE(aircraft.HasBitFlag(UATAircraft::kBitFlagUpdatedBaroVerticalRate));
                        EXPECT_FALSE(aircraft.HasBitFlag(UATAircraft::kBitFlagUpdatedGNSSVerticalRate));
                        break;
                }

            } else {
                // No horizontal velocity information available.
                EXPECT_FALSE(aircraft.HasBitFlag(UATAircraft::kBitFlagHorizontalSpeedValid));
                EXPECT_FALSE(aircraft.HasBitFlag(UATAircraft::kBitFlagUpdatedHorizontalSpeed));
                EXPECT_FALSE(aircraft.HasBitFlag(UATAircraft::kBitFlagDirectionValid));
                EXPECT_FALSE(aircraft.HasBitFlag(UATAircraft::kBitFlagUpdatedDirection));
            }

            switch (frame->track_type) {
                case UAT_TT_TRACK:
                    EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagDirectionValid));
                    EXPECT_FALSE(aircraft.HasBitFlag(UATAircraft::kBitFlagDirectionIsHeading));
                    break;
                case UAT_TT_MAG_HEADING:
                    EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagDirectionValid));
                    EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagDirectionIsHeading));
                    EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagHeadingUsesMagneticNorth));
                    break;
                case UAT_TT_TRUE_HEADING:
                    EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagDirectionValid));
                    EXPECT_FALSE(aircraft.HasBitFlag(UATAircraft::kBitFlagDirectionIsHeading));
                    EXPECT_FALSE(aircraft.HasBitFlag(UATAircraft::kBitFlagHeadingUsesMagneticNorth));
                    break;
                default:
                    // UAT_TT_INVALID and others.
                    EXPECT_FALSE(aircraft.HasBitFlag(UATAircraft::kBitFlagDirectionValid));
                    break;
            }
        }

        if (frame->has_ms) {
            EXPECT_TRUE(packet.has_mode_status);

            if (frame->callsign_type == UAT_CS_CALLSIGN) {
                char callsign[UATAircraft::kCallSignMaxNumChars + 1];
                strncpy(callsign, aircraft.callsign, UATAircraft::kCallSignMaxNumChars);
                // Change aircraft callsign to trim off trailing spaces.
                for (int j = 0; j < UATAircraft::kCallSignMaxNumChars; j++) {
                    if (callsign[j] == ' ') {
                        callsign[j] = '\0';
                    }
                }
                EXPECT_STREQ(callsign, frame->callsign);
            } else if (frame->callsign_type == UAT_CS_SQUAWK) {
                EXPECT_EQ(aircraft.squawk, strtoul(frame->callsign, NULL, 10));
            }

            EXPECT_EQ(aircraft.emitter_category, frame->emitter_category);

            EXPECT_EQ(aircraft.emergency_priority_status, frame->emergency_status);

            EXPECT_EQ(aircraft.uat_version, frame->uat_version);

            EXPECT_EQ(aircraft.surveillance_integrity_level, frame->sil);

            EXPECT_EQ(aircraft.transmit_mso, frame->transmit_mso);

            EXPECT_EQ(aircraft.navigation_accuracy_category_position, frame->nac_p);

            EXPECT_EQ(aircraft.navigation_accuracy_category_velocity, frame->nac_v);

            EXPECT_EQ(aircraft.navigation_integrity_category_baro, frame->nic_baro);

            // Capability Codes
            // Test data can have 1 or -1 in bitfield to indicate true.
            EXPECT_EQ(aircraft.raw_capability_codes, (frame->has_cdti << 1) | frame->has_acas);
            // Has CDTI capability.
            EXPECT_EQ(aircraft.HasBitFlag(UATAircraft::kBitFlagHasCDTI), frame->has_cdti);
            // Has TCAS/ACAS operational and installed.
            EXPECT_EQ(aircraft.HasBitFlag(UATAircraft::kBitFlagTCASOperational), frame->has_acas);

            // Operational Modes
            EXPECT_EQ(aircraft.raw_operational_modes,
                      (frame->acas_ra_active << 2) | (frame->ident_active << 1) | frame->atc_services);
            EXPECT_EQ(aircraft.HasBitFlag(UATAircraft::kBitFlagTCASRA), frame->acas_ra_active);
            EXPECT_EQ(aircraft.HasBitFlag(UATAircraft::kBitFlagIdent), frame->ident_active);
            EXPECT_EQ(aircraft.HasBitFlag(UATAircraft::kBitFlagReceivingATCServices), frame->atc_services);

            // True/Magnetic Heading
            EXPECT_EQ(aircraft.HasBitFlag(UATAircraft::kBitFlagHeadingUsesMagneticNorth),
                      frame->heading_type == UAT_HT_MAGNETIC);
        }
    }
}

// Helper function to convert hex string to bytes:
void hex_string_to_bytes(const char* hex_string, uint8_t* bytes, int length) {
    for (int i = 0; i < length; i++) {
        sscanf(hex_string + (i * 2), "%2hhx", &bytes[i]);
    }
}