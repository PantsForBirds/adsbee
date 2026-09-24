#include "fec.hh"
#include "gdl90_utils.hh"
#include "gtest/gtest.h"

GDL90Reporter gdl90;

TEST(GDL90Utils, TestWriteBufferWithEscapes) {
    uint8_t to_buf[GDL90Reporter::kGDL90MessageMaxLenBytes];

    // These examples are from page 5 of the GDL90 spec below:
    // https://www.faa.gov/sites/faa.gov/files/air_traffic/technology/adsb/archival/GDL90_Public_ICD_RevA.PDF

    // Start of message ID #2 with second byte 0x7E.
    // Omit actual start of message flag since that doesn't get passed to WriteBuffer.
    uint8_t from_buf_0[] = {0x02, 0x7E, 0x05};
    uint16_t from_buf_0_len_bytes = 3;
    ASSERT_EQ(4, gdl90.WriteBufferWithGDL90Escapes(to_buf, sizeof(to_buf), from_buf_0, from_buf_0_len_bytes));
    EXPECT_EQ(to_buf[0], 0x02);
    EXPECT_EQ(to_buf[1], 0x7D);
    EXPECT_EQ(to_buf[2], 0x5E);
    EXPECT_EQ(to_buf[3], 0x05);

    // Start of message ID #3 with second byte 0x7D.
    // Omit actual start of message flag since that doesn't get passed to WriteBuffer.
    uint8_t from_buf_1[] = {0x03, 0x7D, 0x06};
    uint16_t from_buf_1_len_bytes = 3;
    ASSERT_EQ(4, gdl90.WriteBufferWithGDL90Escapes(to_buf, sizeof(to_buf), from_buf_1, from_buf_1_len_bytes));
    EXPECT_EQ(to_buf[0], 0x03);
    EXPECT_EQ(to_buf[1], 0x7D);
    EXPECT_EQ(to_buf[2], 0x5D);
    EXPECT_EQ(to_buf[3], 0x06);

    // End of message with CRC value of 0x7D 0x7E.
    // Omit end of message flag since that doesn't get passed to the WriteBuffer function.
    uint8_t from_buf_2[] = {0x01, 0x09, 0x10, 0x7D, 0x7E};
    uint16_t from_buf_2_len_bytes = 5;
    ASSERT_EQ(7, gdl90.WriteBufferWithGDL90Escapes(to_buf, sizeof(to_buf), from_buf_2, from_buf_2_len_bytes));
    EXPECT_EQ(to_buf[0], 0x01);
    EXPECT_EQ(to_buf[1], 0x09);
    EXPECT_EQ(to_buf[2], 0x10);
    EXPECT_EQ(to_buf[3], 0x7D);
    EXPECT_EQ(to_buf[4], 0x5D);
    EXPECT_EQ(to_buf[5], 0x7D);
    EXPECT_EQ(to_buf[6], 0x5E);
}

void crc_init(uint16_t crc16_table[]) {
    unsigned int i, bitctr, crc;
    for (i = 0; i < 256; i++) {
        crc = (i << 8);
        for (bitctr = 0; bitctr < 8; bitctr++) {
            crc = (crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0);
        }
        crc16_table[i] = crc;
    }
}

TEST(GDL90Utils, PrintCRCTable) {
    uint16_t crc16_table[256];
    crc_init(crc16_table);
    printf("uint16_t crc16_table[256] = {");
    for (uint16_t i = 0; i < 256; i++) {
        printf("0x%x%s", crc16_table[i], i < 255 ? ", " : "};\r\n");
    }
}

TEST(GDL90Utils, CalcCRC16) {
    // 0x7E 0x00 0x81 0x41 0xDB 0xD0 0x08 0x02 0xB3 0x8B 0x7E
    uint8_t unescaped_message[] = {0x00, 0x81, 0x41, 0xDB, 0xD0, 0x08, 0x02};
    ASSERT_EQ(0x8BB3, gdl90.CalculateGDL90CRC16(unescaped_message, 7));

    uint8_t crc_unescaped[] = {0xB3, 0x8B};  // LSB first.
    uint8_t crc_escaped[4];
    ASSERT_EQ(2, gdl90.WriteBufferWithGDL90Escapes(crc_escaped, sizeof(crc_escaped), crc_unescaped, 2));
    EXPECT_EQ(crc_escaped[0], 0xB3);
    EXPECT_EQ(crc_escaped[1], 0x8B);
}

TEST(GDL90Utils, WriteGDL90Message) {
    // 0x7E 0x00 0x81 0x41 0xDB 0xD0 0x08 0x02 0xB3 0x8B 0x7E
    uint8_t buf[GDL90Reporter::kGDL90MessageMaxLenBytes];
    memset(buf, 0xFF, GDL90Reporter::kGDL90MessageMaxLenBytes);  // For easier debugging.
    uint8_t unescaped_message[] = {0x00, 0x81, 0x41, 0xDB, 0xD0, 0x08, 0x02};
    ASSERT_EQ(11, gdl90.WriteGDL90Message(buf, GDL90Reporter::kGDL90MessageMaxLenBytes, unescaped_message,
                                          sizeof(unescaped_message)));
    EXPECT_EQ(buf[0], 0x7E);
    EXPECT_EQ(buf[1], 0x00);
    EXPECT_EQ(buf[2], 0x81);
    EXPECT_EQ(buf[3], 0x41);
    EXPECT_EQ(buf[4], 0xDB);
    EXPECT_EQ(buf[5], 0xD0);
    EXPECT_EQ(buf[6], 0x08);
    EXPECT_EQ(buf[7], 0x02);
    EXPECT_EQ(buf[8], 0xB3);
    EXPECT_EQ(buf[9], 0x8B);
    EXPECT_EQ(buf[10], 0x7E);
}

TEST(GDL90Utils, HeartBeatMessage) {
    // 0x7E 0x00 0x81 0x41 0xDB 0xD0 0x08 0x02 0xB3 0x8B 0x7E
    uint8_t buf[GDL90Reporter::kGDL90MessageMaxLenBytes];
    // Set Status Byte 1 = 0b10000001.
    gdl90.gnss_position_valid = true;
    gdl90.maintenance_required = false;
    gdl90.csa_requested = true;
    gdl90.csa_not_available = false;
    // Set Status Byte 2 = 0b01000001.
    gdl90.utc_timing_is_valid = true;
    // Timestamp must have MS bit = 0b0.
    uint32_t timestamp = (0b0 << 16) | 0xD0DB;  // Set timestamp to match sample message.
    uint16_t mode_s_message_counts = 2;
    uint16_t uat_message_counts = 1;
    ASSERT_EQ(11, gdl90.WriteGDL90HeartbeatMessage(buf, GDL90Reporter::kGDL90MessageMaxLenBytes, timestamp,
                                                   mode_s_message_counts, uat_message_counts));
    EXPECT_EQ(buf[0], 0x7E);
    EXPECT_EQ(buf[1], 0x00);
    EXPECT_EQ(buf[2], 0x81);
    EXPECT_EQ(buf[3], 0x41);
    EXPECT_EQ(buf[4], 0xDB);
    EXPECT_EQ(buf[5], 0xD0);
    EXPECT_EQ(buf[6], 0x08);
    EXPECT_EQ(buf[7], 0x02);
    EXPECT_EQ(buf[8], 0xB3);
    EXPECT_EQ(buf[9], 0x8B);
    EXPECT_EQ(buf[10], 0x7E);

    // UAT count clamped to 5 bits (max 31). Spec §3.1.4: bits 7..3 of byte 6.
    gdl90.WriteGDL90HeartbeatMessage(buf, GDL90Reporter::kGDL90MessageMaxLenBytes, timestamp, 0, 31);
    EXPECT_EQ(buf[6] & 0xF8, 31 << 3);  // All 5 UAT bits set.
    gdl90.WriteGDL90HeartbeatMessage(buf, GDL90Reporter::kGDL90MessageMaxLenBytes, timestamp, 0, 32);
    EXPECT_EQ(buf[6] & 0xF8, 31 << 3);  // 32 should clamp to 31.
    gdl90.WriteGDL90HeartbeatMessage(buf, GDL90Reporter::kGDL90MessageMaxLenBytes, timestamp, 0, 256);
    EXPECT_EQ(buf[6] & 0xF8, 31 << 3);  // 256 would wrap to 0 if narrowed before clamping; must clamp to 31.

    // Mode S count clamped to 10 bits (max 1023). Spec §3.1.4: bits 1..0 of byte 6 (MS) + byte 7 (LS).
    gdl90.WriteGDL90HeartbeatMessage(buf, GDL90Reporter::kGDL90MessageMaxLenBytes, timestamp, 1023, 0);
    EXPECT_EQ(buf[6] & 0x03, 0x03);  // MS 2 bits = 0b11.
    EXPECT_EQ(buf[7], 0xFF);         // LS 8 bits = 0xFF.
    gdl90.WriteGDL90HeartbeatMessage(buf, GDL90Reporter::kGDL90MessageMaxLenBytes, timestamp, 1024, 0);
    EXPECT_EQ(buf[6] & 0x03, 0x03);  // 1024 clamps to 1023.
    EXPECT_EQ(buf[7], 0xFF);
}

TEST(GDL90Utils, InitMessage) {
    // TODO: Add tests here!
}

/**
 * Compare two buffers, removing GDL90 escape characters from the first buffer as they are encountered.
 * @param[in] unescaped_buffer Buffer containing unescaped data to compare against.
 * @param[in] unescaped_buffer_length Length of the unescaped buffer to compare, in bytes.
 * @param[in] escaped_buffer Buffer containing GDL90-escaped data.
 * @retval The number of characters parsed in the escape buffer corresponding to the unescaped buffer length that was
 * scanned.
 */
uint16_t CheckBuffersEqualInjectEscapes(uint8_t* unescaped_buffer, uint16_t unescaped_buffer_length,
                                        uint8_t* escaped_buffer) {
    uint16_t escaped_index = 0;
    for (uint16_t i = 0; i < unescaped_buffer_length; i++) {
        if (unescaped_buffer[i] == GDL90Reporter::kGDL90ControlEscapeChar ||
            unescaped_buffer[i] == GDL90Reporter::kGDL90ControlEscapeChar) {
            EXPECT_EQ(escaped_buffer[escaped_index], GDL90Reporter::kGDL90ControlEscapeChar)
                << "Escape char mismatch at escaped_index=" << escaped_index << " unescaped_index=" << i;
            escaped_index++;
            EXPECT_EQ(escaped_buffer[escaped_index], unescaped_buffer[i] ^ 0x20)
                << "Escaped value mismatch at escaped_index=" << escaped_index << " unescaped_index=" << i;
            escaped_index++;
        } else {
            EXPECT_EQ(escaped_buffer[escaped_index], unescaped_buffer[i])
                << "Value mismatch at escaped_index=" << escaped_index << " unescaped_index=" << i;
            escaped_index++;
        }
    }
    return escaped_index;
}

TEST(GDL90Utils, UplinkDataMessage) {
    // TODO: Add tests here!
    const char* frame_data_hex =
        "3514c952d65ca7b0158000210de09082102d30cb00082f0d1e012d30cb000000000000000fd900011710120118173ba9c9635e4c001580"
        "00210e9e0082102cf04b00082f521e012cf04b000000000000000fd900011a0f00011f0001a916435a6800278000350e1d682210000000"
        "ff004491387c4d5060cb4c74d35833d75db9c337f2d38df87d07d27f3cb0ca030f5dfc75c31cb4c74d357f1d70c72d70c73c1fc30c1fc7"
        "8c1f05f65f7f3cb0c8c3d77df780288000350e1d682210000000ff004691347c4d5060cb4c74d35833d75db9c317f2d70db37d07d27f3c"
        "b0ca02091c87f1d70c72d31d34d5fc75c31cb5c31cf07f1e307f2e707c17d97dfcf2c322091c87df78002d00067408605c93844e008316"
        "0cb5c30c306a080651c5f1cb0c30707c78c30c1c0f2d30c30703cf0c30c1c133d30c30820cf9c30c1c65e718cf5cb2af0c20cf6cf1b71c"
        "e0c31d31b72de0c33d70d36830d36db5da0cf6d72d7879d000000000000000000000000000000000000000000000000000000000000000"
        "0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000";  // frame_data_hex
    uint16_t frame_length_bytes = 432;  // frame_length_bytes

    // Create encoded data frame.
    uint8_t decoded_data_frame[RawUATUplinkPacket::kUplinkMessagePayloadNumBytes] = {0};
    HexStringToByteBuffer(decoded_data_frame, frame_data_hex, frame_length_bytes);
    uint8_t encoded_data_frame[RawUATUplinkPacket::kUplinkMessageNumBytes] = {0};
    uat_rs.EncodeUplinkMessage(encoded_data_frame, decoded_data_frame);
    PrintByteBuffer("Decoded uplink payload:", decoded_data_frame, DecodedUATUplinkPacket::kDecodedPayloadNumBytes);
    PrintByteBuffer("Encoded uplink message:", encoded_data_frame, RawUATUplinkPacket::kUplinkMessageNumBytes);
    int16_t sigs_dbm = -10;                                 // Dummy signal strength.
    int16_t sigq_bits = 0;                                  // Dummy signal quality.
    uint64_t mlat_48mhz_64bit_counts = 0x1234567812345678;  // Dummy timestamp.
    DecodedUATUplinkPacket packet(RawUATUplinkPacket(encoded_data_frame, RawUATUplinkPacket::kUplinkMessageNumBytes,
                                                     sigs_dbm, sigq_bits, mlat_48mhz_64bit_counts));

    uint8_t gdl90_uplink_message[600];
    memset(gdl90_uplink_message, 0xFF, sizeof(gdl90_uplink_message));
    // First try writing to a buffer that's too small.
    uint16_t gdl90_uplink_message_len_bytes = gdl90.WriteGDL90UplinkDataMessage(
        gdl90_uplink_message, 10, packet.decoded_payload, DecodedUATUplinkPacket::kDecodedPayloadNumBytes);
    EXPECT_LE(gdl90_uplink_message_len_bytes, 10);  // Abbreviated write for buffer that is too short.
    for (uint16_t i = 10; i < sizeof(gdl90_uplink_message); i++) {
        EXPECT_EQ(0xFF, gdl90_uplink_message[i]);  // Ensure no writes past buffer end.
    }

    // Now try with a sufficiently large buffer.
    gdl90_uplink_message_len_bytes =
        gdl90.WriteGDL90UplinkDataMessage(gdl90_uplink_message, sizeof(gdl90_uplink_message), packet.decoded_payload,
                                          DecodedUATUplinkPacket::kDecodedPayloadNumBytes,
                                          GDL90Reporter::MLAT48MHz64BitCountsToUATTORTicks(mlat_48mhz_64bit_counts));
    EXPECT_GT(gdl90_uplink_message_len_bytes,
              DecodedUATUplinkPacket::kDecodedPayloadNumBytes);  // Should be larger than payload due to message ID and
                                                                 // escaping.
    // Check flag byte.
    EXPECT_EQ(0x7E, gdl90_uplink_message[0]);
    // Check message ID.
    EXPECT_EQ(GDL90Reporter::kGDL90MessageIDUplinkData, gdl90_uplink_message[1]);
    uint16_t gdl90_msg_index = 2;  // Start after flag byte and message ID.
    // Check 3-byte TOR.
    uint32_t tor = GDL90Reporter::MLAT48MHz64BitCountsToUATTORTicks(
        mlat_48mhz_64bit_counts);  // Should match the value passed in above.
    uint8_t tor_bytes[3] = {static_cast<uint8_t>(tor & 0xFF), static_cast<uint8_t>((tor >> 8) & 0xFF),
                            static_cast<uint8_t>((tor >> 16) & 0xFF)};
    gdl90_msg_index += CheckBuffersEqualInjectEscapes(tor_bytes, 3, &gdl90_uplink_message[gdl90_msg_index]);
    // Check message contents byte by byte including escapes after every 0x7E.
    gdl90_msg_index +=
        CheckBuffersEqualInjectEscapes(packet.decoded_payload, DecodedUATUplinkPacket::kDecodedPayloadNumBytes,
                                       &gdl90_uplink_message[gdl90_msg_index]);
    // CRC is calculated on the unescaped header and payload, excluding the initial flag byte.
    uint8_t unescaped_message_for_crc[1 + 3 + DecodedUATUplinkPacket::kDecodedPayloadNumBytes];
    uint16_t unescaped_len = 0;
    unescaped_message_for_crc[unescaped_len++] = GDL90Reporter::kGDL90MessageIDUplinkData;
    unescaped_message_for_crc[unescaped_len++] = tor_bytes[0];
    unescaped_message_for_crc[unescaped_len++] = tor_bytes[1];
    unescaped_message_for_crc[unescaped_len++] = tor_bytes[2];
    memcpy(&unescaped_message_for_crc[unescaped_len], packet.decoded_payload,
           DecodedUATUplinkPacket::kDecodedPayloadNumBytes);
    unescaped_len += DecodedUATUplinkPacket::kDecodedPayloadNumBytes;
    uint16_t expected_crc = gdl90.CalculateGDL90CRC16(unescaped_message_for_crc, unescaped_len);
    // Check CRC bytes including escapes.
    uint8_t crc_bytes[2] = {static_cast<uint8_t>(expected_crc & 0xFF), static_cast<uint8_t>(expected_crc >> 8)};
    gdl90_msg_index += CheckBuffersEqualInjectEscapes(crc_bytes, 2, &gdl90_uplink_message[gdl90_msg_index]);
    // Check ending flag byte.
    EXPECT_EQ(0x7E, gdl90_uplink_message[gdl90_msg_index++]);
}

TEST(GDL90Utils, WriteGDL90MessageBufferOverrunProtection) {
    // Test that WriteGDL90Message properly respects buffer boundaries and doesn't overrun.

    // Create a simple message (Message ID 0x00 with 2 bytes of data)
    uint8_t unescaped_message[] = {0x00, 0x01, 0x02};
    uint8_t unescaped_message_len = sizeof(unescaped_message);

    // Expected minimum size for this message without escapes:
    // 1 (start flag) + 3 (message) + 2 (CRC) + 1 (end flag) = 7 bytes

    // Test 1: Buffer size of 0 - should write nothing
    {
        uint8_t buf[10];
        memset(buf, 0xAA, sizeof(buf));  // Fill with pattern to detect writes
        uint16_t written = gdl90.WriteGDL90Message(buf, 0, unescaped_message, unescaped_message_len);
        EXPECT_EQ(0, written);
        // Verify no bytes were written
        for (size_t i = 0; i < sizeof(buf); i++) {
            EXPECT_EQ(0xAA, buf[i]) << "Buffer modified at index " << i;
        }
    }

    // Test 2: Buffer size of 1 - should only write start flag, then stop
    {
        uint8_t buf[10];
        memset(buf, 0xAA, sizeof(buf));
        uint16_t written = gdl90.WriteGDL90Message(buf, 1, unescaped_message, unescaped_message_len);
        EXPECT_LE(written, 1);
        // Verify no writes past buffer end
        for (size_t i = 1; i < sizeof(buf); i++) {
            EXPECT_EQ(0xAA, buf[i]) << "Buffer overrun at index " << i;
        }
    }

    // Test 3: Buffer size exactly for start flag + message (no CRC, no end flag)
    {
        uint8_t buf[10];
        memset(buf, 0xAA, sizeof(buf));
        uint16_t written = gdl90.WriteGDL90Message(buf, 4, unescaped_message, unescaped_message_len);
        EXPECT_LE(written, 4);
        // Verify no writes past buffer end
        for (size_t i = 4; i < sizeof(buf); i++) {
            EXPECT_EQ(0xAA, buf[i]) << "Buffer overrun at index " << i;
        }
    }

    // Test 4: Buffer size almost enough (missing 1 byte for end flag)
    {
        uint8_t buf[10];
        memset(buf, 0xAA, sizeof(buf));
        uint16_t written = gdl90.WriteGDL90Message(buf, 6, unescaped_message, unescaped_message_len);
        EXPECT_LE(written, 6);
        // Verify no writes past buffer end
        for (size_t i = 6; i < sizeof(buf); i++) {
            EXPECT_EQ(0xAA, buf[i]) << "Buffer overrun at index " << i;
        }
    }

    // Test 5: Buffer with exact size needed (should succeed)
    {
        uint8_t buf[20];
        memset(buf, 0xAA, sizeof(buf));
        uint16_t written = gdl90.WriteGDL90Message(buf, 7, unescaped_message, unescaped_message_len);
        EXPECT_EQ(7, written);
        EXPECT_EQ(0x7E, buf[0]);  // Start flag
        EXPECT_EQ(0x7E, buf[6]);  // End flag
        // Verify no writes past buffer end
        for (size_t i = 7; i < sizeof(buf); i++) {
            EXPECT_EQ(0xAA, buf[i]) << "Buffer overrun at index " << i;
        }
    }

    // Test 6: Message with escape characters requiring more space
    {
        // Message containing 0x7E (flag byte) which requires escaping
        uint8_t escape_message[] = {0x00, 0x7E, 0x02};
        uint8_t buf[20];
        memset(buf, 0xAA, sizeof(buf));

        // With escaping, the 0x7E becomes 0x7D 0x5E, so message grows by 1 byte
        // Expected: 1 (start) + 1 (0x00) + 2 (escaped 0x7E) + 1 (0x02) + 2 (CRC) + 1 (end) = 8 bytes minimum
        // But CRC might also need escaping, so could be more

        // Try with insufficient buffer (7 bytes - would fit unescaped but not escaped)
        uint16_t written = gdl90.WriteGDL90Message(buf, 7, escape_message, sizeof(escape_message));
        EXPECT_LE(written, 7);
        // Verify no writes past buffer end
        for (size_t i = 7; i < sizeof(buf); i++) {
            EXPECT_EQ(0xAA, buf[i]) << "Buffer overrun at index " << i;
        }
    }

    // Test 7: Multiple escape characters
    {
        // Message with multiple bytes requiring escapes
        uint8_t multi_escape[] = {0x00, 0x7E, 0x7D, 0x7E};
        uint8_t buf[20];
        memset(buf, 0xAA, sizeof(buf));

        // Try with various buffer sizes
        for (uint16_t buf_size = 0; buf_size < 15; buf_size++) {
            memset(buf, 0xAA, sizeof(buf));
            uint16_t written = gdl90.WriteGDL90Message(buf, buf_size, multi_escape, sizeof(multi_escape));
            EXPECT_LE(written, buf_size) << "Overwrote buffer with size " << buf_size;
            // Verify no writes past buffer end
            for (size_t i = buf_size; i < sizeof(buf); i++) {
                EXPECT_EQ(0xAA, buf[i]) << "Buffer overrun at index " << i << " with buffer size " << buf_size;
            }
        }
    }

    // Test 8: Large message with small buffer
    {
        uint8_t large_message[100];
        for (size_t i = 0; i < sizeof(large_message); i++) {
            large_message[i] = i & 0xFF;
        }

        uint8_t buf[20];
        memset(buf, 0xAA, sizeof(buf));
        uint16_t written = gdl90.WriteGDL90Message(buf, 10, large_message, sizeof(large_message));
        EXPECT_LE(written, 10);
        // Verify no writes past buffer end
        for (size_t i = 10; i < sizeof(buf); i++) {
            EXPECT_EQ(0xAA, buf[i]) << "Buffer overrun at index " << i;
        }
    }
}

// Decodes a 24-bit signed binary fraction (resolution 180/2^23 deg) from three big-endian message bytes.
static float DecodeLatLon24(const uint8_t* bytes) {
    int32_t raw = (bytes[0] << 16) | (bytes[1] << 8) | bytes[2];
    if (raw & 0x800000) raw -= 0x1000000;  // Sign extend.
    return raw * (180.0f / 8388608.0f);
}

TEST(GDL90Utils, OwnshipReportFromGNSS) {
    SettingsManager::RxPosition rx_position;
    rx_position.source = SettingsManager::RxPosition::kPositionSourceGNSS;
    rx_position.latitude_deg = 37.5f;
    rx_position.longitude_deg = -122.0f;
    rx_position.gnss_altitude_ft = 1500;
    rx_position.baro_altitude_ft = 1234;  // Stale: must NOT leak into a GNSS ownship report.
    rx_position.heading_deg = 90.0f;
    rx_position.speed_kts = 45;
    rx_position.icao_address = 0xABCDEF;  // Irrelevant for GNSS: ownship must be self-assigned.

    GDL90Reporter::GDL90TargetReportData data;
    ASSERT_TRUE(GDL90Reporter::BuildOwnshipReportData(data, rx_position, /*rx_position_available=*/true));
    EXPECT_EQ(data.address_type, GDL90Reporter::GDL90TargetReportData::kAddressTypeADSBWithSelfAssignedAddress);
    EXPECT_EQ(data.participant_address, 0u);
    EXPECT_FLOAT_EQ(data.latitude_deg, 37.5f);
    EXPECT_FLOAT_EQ(data.longitude_deg, -122.0f);
    EXPECT_EQ(data.altitude_ft, INT32_MIN);  // Pressure altitude unknown from GNSS: invalid, never GNSS altitude.
    EXPECT_FLOAT_EQ(data.speed_kts, 45.0f);
    EXPECT_FLOAT_EQ(data.direction_deg, 90.0f);
    EXPECT_EQ(data.navigation_integrity_category, 8);
    EXPECT_EQ(data.navigation_accuracy_category_position, 8);
    EXPECT_EQ(data.misc_indicators, 0b1001);  // Airborne (45 kt > 30 kt), true track angle.
    EXPECT_STREQ(data.callsign, "ADSBEE  ");

    // Encode it and check the wire format. None of the payload bytes below need escaping.
    uint8_t buf[GDL90Reporter::kGDL90MessageMaxLenBytes];
    uint16_t len = gdl90.WriteGDL90TargetReportMessage(buf, sizeof(buf), data, /*ownship=*/true);
    ASSERT_EQ(len, 32);  // 1 flag + 28 payload + 2 CRC + 1 flag.
    EXPECT_EQ(buf[0], 0x7E);
    EXPECT_EQ(buf[1], GDL90Reporter::kGDL90MessageIDOwnshipReport);
    EXPECT_EQ(buf[2], 0x01);  // No traffic alert, self-assigned address type.
    EXPECT_EQ(buf[3], 0x00);  // Participant address 0x000000.
    EXPECT_EQ(buf[4], 0x00);
    EXPECT_EQ(buf[5], 0x00);
    EXPECT_NEAR(DecodeLatLon24(buf + 6), 37.5f, 1e-4f);
    EXPECT_NEAR(DecodeLatLon24(buf + 9), -122.0f, 1e-4f);
    EXPECT_EQ(buf[12], 0xFF);  // Altitude 0xFFF = invalid.
    EXPECT_EQ(buf[13], 0xF9);  // Altitude LS nibble + misc indicators 0b1001.
    EXPECT_EQ(buf[14], 0x88);  // NIC 8, NACp 8.
    EXPECT_EQ(buf[15], 0x02);  // Horizontal velocity 45 kt = 0x02D.
    EXPECT_EQ(buf[16] & 0xF0, 0xD0);
    EXPECT_EQ(buf[18], 64);  // 90 deg * 256/360.
    EXPECT_EQ(memcmp(buf + 20, "ADSBEE  ", 8), 0);
    EXPECT_EQ(buf[31], 0x7E);
}

TEST(GDL90Utils, OwnshipReportFromTrackedAircraft) {
    SettingsManager::RxPosition rx_position;
    rx_position.source = SettingsManager::RxPosition::kPositionSourceAircraftMatchingICAO;
    rx_position.latitude_deg = 40.0f;
    rx_position.longitude_deg = -75.0f;
    rx_position.gnss_altitude_ft = 3100;
    rx_position.baro_altitude_ft = 3000;
    rx_position.heading_deg = 180.0f;
    rx_position.speed_kts = 5;
    rx_position.icao_address = 0xABCDEF;

    GDL90Reporter::GDL90TargetReportData data;
    ASSERT_TRUE(GDL90Reporter::BuildOwnshipReportData(data, rx_position, /*rx_position_available=*/true));
    EXPECT_EQ(data.participant_address, 0xABCDEFu);  // Bootstrapping off a real aircraft: carry its ICAO.
    EXPECT_EQ(data.altitude_ft, 3000);              // Aircraft supplies a pressure altitude.
    EXPECT_EQ(data.misc_indicators, 0b0001);        // On the ground (5 kt), true track angle.
}

TEST(GDL90Utils, OwnshipReportWithoutPosition) {
    SettingsManager::RxPosition rx_position;
    rx_position.latitude_deg = 37.5f;
    rx_position.longitude_deg = -122.0f;
    rx_position.speed_kts = 100;
    rx_position.icao_address = 0xABCDEF;

    // A GNSS source with no fresh fix must not send its last coordinates.
    rx_position.source = SettingsManager::RxPosition::kPositionSourceGNSS;
    GDL90Reporter::GDL90TargetReportData data;
    data.latitude_deg = 1.0f;  // Prove the builder resets stale contents.
    EXPECT_FALSE(GDL90Reporter::BuildOwnshipReportData(data, rx_position, /*rx_position_available=*/false));
    EXPECT_FLOAT_EQ(data.latitude_deg, 0.0f);
    EXPECT_FLOAT_EQ(data.longitude_deg, 0.0f);
    EXPECT_EQ(data.navigation_integrity_category, 0);  // GDL90 "no position" convention: lat, lon, NIC all zero.
    EXPECT_EQ(data.participant_address, 0u);
    EXPECT_EQ(data.address_type, GDL90Reporter::GDL90TargetReportData::kAddressTypeADSBWithSelfAssignedAddress);
    EXPECT_STREQ(data.callsign, "ADSBEE  ");

    // Receiver locations that are not this device's own movement are never ownship, even when available.
    for (auto source : {SettingsManager::RxPosition::kPositionSourceNone, SettingsManager::RxPosition::kPositionSourceFixed,
                        SettingsManager::RxPosition::kPositionSourceLowestAircraft}) {
        rx_position.source = source;
        EXPECT_FALSE(GDL90Reporter::IsValidOwnshipSource(source));
        EXPECT_FALSE(GDL90Reporter::BuildOwnshipReportData(data, rx_position, /*rx_position_available=*/true))
            << "source " << static_cast<int>(source);
        EXPECT_FLOAT_EQ(data.latitude_deg, 0.0f);
        EXPECT_EQ(data.navigation_integrity_category, 0);
    }
    EXPECT_TRUE(GDL90Reporter::IsValidOwnshipSource(SettingsManager::RxPosition::kPositionSourceGNSS));
    EXPECT_TRUE(GDL90Reporter::IsValidOwnshipSource(SettingsManager::RxPosition::kPositionSourceAircraftMatchingICAO));
}

TEST(GDL90Utils, TrafficReport) {
    // TODO: Add tests here!
}

TEST(GDL90Utils, ModeSEmitterCategoryUsesDecodedEnum) {
    // Mode S emitter_category_raw = (TypeCode<<3)|Category (values 8–39), not a valid GDL90
    // emitter category code. The GDL90 serializer must use the decoded emitter_category enum.
    ModeSAircraft ac;
    ac.emitter_category = ADSBTypes::kEmitterCategoryHeavy;
    ac.emitter_category_raw = 37;  // TC=4, Cat=5 → raw Mode S value, wrong for GDL90.

    uint8_t buf[GDL90Reporter::kGDL90MessageMaxLenBytes];
    memset(buf, 0xFF, sizeof(buf));
    ASSERT_GT(gdl90.WriteGDL90TargetReportMessage(buf, sizeof(buf), ac, /*ownship=*/false), 19u);

    // message_buf[18] = emitter_category. No GDL90 escape bytes precede it for a zero-initialized
    // aircraft, so it maps directly to buf[19].
    EXPECT_EQ(buf[19], static_cast<uint8_t>(ADSBTypes::kEmitterCategoryHeavy));
    EXPECT_NE(buf[19], ac.emitter_category_raw);
}

// Decodes a GDL90 frame (flag bytes + byte stuffing) back into the raw message buffer so tests can assert on field
// offsets without depending on which bytes happened to need escaping. Returns the number of message bytes recovered
// (excluding the trailing 2-byte CRC), or 0 if the frame is malformed.
static uint16_t DecodeGDL90Frame(const uint8_t* frame, uint16_t frame_len, uint8_t* out, uint16_t out_len) {
    if (frame_len < 4 || frame[0] != 0x7E) return 0;
    uint16_t out_i = 0;
    for (uint16_t i = 1; i < frame_len; i++) {
        if (frame[i] == 0x7E) {                   // Closing flag.
            return (out_i >= 2) ? out_i - 2 : 0;  // Strip the trailing CRC16.
        }
        if (out_i >= out_len) return 0;
        if (frame[i] == 0x7D) {  // Escape: next byte is XORed with 0x20.
            if (++i >= frame_len) return 0;
            out[out_i++] = frame[i] ^ 0x20;
        } else {
            out[out_i++] = frame[i];
        }
    }
    return 0;  // No closing flag.
}

// Encodes a RemoteIDAircraft as a GDL90 traffic report and returns the de-escaped message buffer.
static uint16_t EncodeRemoteIDTrafficReport(const RemoteIDAircraft& aircraft, uint8_t* msg, uint16_t msg_len) {
    uint8_t frame[GDL90Reporter::kGDL90MessageMaxLenBytes];
    uint16_t frame_len = gdl90.WriteGDL90TargetReportMessage(frame, sizeof(frame), aircraft, /*ownship=*/false);
    return DecodeGDL90Frame(frame, frame_len, msg, msg_len);
}

TEST(GDL90Utils, RemoteIDTrackValidOnlyWhenDirectionValid) {
    // Remote ID reports direction independently of position, so a drone with a valid position but no direction must
    // NOT be reported as having a true track angle — otherwise a stale/default heading reads as a real track.
    RemoteIDAircraft ac(0x123456);
    ac.latitude_deg = 37.5f;
    ac.longitude_deg = -122.3f;
    ac.direction_deg = 90.0f;  // Stale value that must not be presented as a valid track.
    ac.WriteBitFlag(RemoteIDAircraft::kBitFlagPositionValid, true);

    uint8_t msg[32];
    ASSERT_GE(EncodeRemoteIDTrafficReport(ac, msg, sizeof(msg)), 18u);
    // message_buf[12] low nibble = misc indicators; bits [1:0] are the track/heading type.
    EXPECT_EQ(msg[12] & 0b11, GDL90Reporter::GDL90TargetReportData::kMiscIndicatorTTNotValid);

    // Once a direction is actually received, the track angle is advertised as valid.
    ac.WriteBitFlag(RemoteIDAircraft::kBitFlagDirectionValid, true);
    ASSERT_GE(EncodeRemoteIDTrafficReport(ac, msg, sizeof(msg)), 18u);
    EXPECT_EQ(msg[12] & 0b11, GDL90Reporter::GDL90TargetReportData::kMiscIndicatorTTIsTrueTrackAngle);
}

TEST(GDL90Utils, RemoteIDVerticalRateUnavailableWhenNotReceived) {
    // A Remote ID Location message may omit vertical speed. That must encode as the GDL90 "not available" sentinel
    // (0x800), not as a real 0 fpm.
    RemoteIDAircraft ac(0x123456);
    ac.latitude_deg = 37.5f;
    ac.longitude_deg = -122.3f;
    ac.WriteBitFlag(RemoteIDAircraft::kBitFlagPositionValid, true);
    ac.gnss_vertical_rate_fpm = 0;  // Default/stale: no vertical rate was ever received.

    uint8_t msg[32];
    ASSERT_GE(EncodeRemoteIDTrafficReport(ac, msg, sizeof(msg)), 18u);
    // Vertical velocity is 12 bits: low nibble of message_buf[15] (MSBs) followed by message_buf[16].
    uint16_t vertical_rate_encoded = ((msg[15] & 0x0F) << 8) | msg[16];
    EXPECT_EQ(vertical_rate_encoded, 0x800);

    // With a received vertical rate, the real value is encoded in units of 64 fpm.
    ac.gnss_vertical_rate_fpm = 640;
    ac.WriteBitFlag(RemoteIDAircraft::kBitFlagVerticalRateValid, true);
    ASSERT_GE(EncodeRemoteIDTrafficReport(ac, msg, sizeof(msg)), 18u);
    vertical_rate_encoded = ((msg[15] & 0x0F) << 8) | msg[16];
    EXPECT_EQ(vertical_rate_encoded, 640 / 64);
}