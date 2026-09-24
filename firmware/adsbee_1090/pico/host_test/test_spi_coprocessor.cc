#include "gtest/gtest.h"
#include "spi_coprocessor.hh"
#include "spi_coprocessor_packet.hh"

TEST(SPICoprocessor, SCWritePacket) {
    SPICoprocessorPacket::SCWritePacket packet;
    packet.cmd = ObjectDictionary::SCCommand::kCmdWriteToSlave;
    packet.addr = ObjectDictionary::Address::kAddrRawModeSPacket;
    RawModeSPacket tpacket = RawModeSPacket((const char*)"8D7C1BE8581B66E9BD8CEEDC1C9F");
    packet.len = sizeof(RawModeSPacket);
    memcpy(packet.data, &tpacket, packet.len);
    // Calculate CRC and add it to the data buffer.
    uint16_t crc =
        CalculateCRC16(packet.GetBuf(), packet.GetBufLenBytes() - SPICoprocessorPacket::SCWritePacket::kCRCLenBytes);
    memcpy(packet.data + packet.len, &crc, sizeof(uint16_t));
    EXPECT_TRUE(packet.IsValid());

    packet.PopulateCRC();
    EXPECT_TRUE(packet.IsValid());

    EXPECT_EQ(packet.GetBufLenBytes(), sizeof(RawModeSPacket) + sizeof(ObjectDictionary::SCCommand) +
                                           sizeof(ObjectDictionary::Address) + 2 * sizeof(uint16_t) +
                                           SPICoprocessorPacket::SCWritePacket::kCRCLenBytes);

    SPICoprocessorPacket::SCWritePacket packet_copy =
        SPICoprocessorPacket::SCWritePacket(packet.GetBuf(), packet.GetBufLenBytes());
    EXPECT_EQ(packet.cmd, packet_copy.cmd);
    EXPECT_EQ(packet.addr, packet_copy.addr);
    EXPECT_EQ(packet.IsValid(), packet_copy.IsValid());
    RawModeSPacket* tpacket_copy = (RawModeSPacket*)packet_copy.data;
    EXPECT_EQ(tpacket.buffer_len_bytes, tpacket_copy->buffer_len_bytes);
    EXPECT_EQ(tpacket.buffer[0], tpacket_copy->buffer[0]);
    EXPECT_EQ(tpacket.buffer[1], tpacket_copy->buffer[1]);
    EXPECT_EQ(tpacket.buffer[2], tpacket_copy->buffer[2]);
    EXPECT_EQ(tpacket.buffer[3], tpacket_copy->buffer[3]);

    // Poke packet and make checksum fail.
    packet.data[0] = ~packet.data[0];
    EXPECT_FALSE(packet.IsValid());
}

// TEST(SPICoprocessor, SCReadRequestPacket) {
//     // Test packet creation.
//     SPICoprocessorPacket::SCReadRequestPacket packet;
//     packet.cmd = ObjectDictionary::SCCommand::kCmdReadFromMaster;
//     packet.addr = ObjectDictionary::Address::kAddrSettingsData;
//     packet.offset = 0xFEBC;
//     packet.len = 40;
//     // Make sure that CRC generation works as expected.
//     packet.PopulateCRC();
//     EXPECT_TRUE(packet.IsValid());
//     EXPECT_EQ(CalculateCRC16(packet.GetBuf(),
//                              packet.GetBufLenBytes() - SPICoprocessorPacket::SCReadRequestPacket::kCRCLenBytes),
//               packet.crc);

//     // Test packet ingestion.
//     SPICoprocessorPacket::SCReadRequestPacket packet_copy =
//         SPICoprocessorPacket::SCReadRequestPacket(packet.GetBuf(), packet.GetBufLenBytes());
//     EXPECT_EQ(packet_copy.cmd, packet.cmd);
//     EXPECT_EQ(packet_copy.addr, packet.addr);
//     EXPECT_EQ(packet_copy.offset, packet.offset);
//     EXPECT_EQ(packet_copy.len, packet.len);
//     EXPECT_TRUE(packet_copy.IsValid());
//     // Changes to packet should not affect packet_copy.
//     packet.GetBuf()[0] = ~packet.GetBuf()[0];
//     EXPECT_FALSE(packet.IsValid());
//     EXPECT_TRUE(packet_copy.IsValid());
// }

TEST(SPICoprocessor, SCResponsePacket) {
    // Test packet creation.
    SPICoprocessorPacket::SCResponsePacket packet;
    packet.cmd = ObjectDictionary::SCCommand::kCmdDataBlock;
    EXPECT_EQ(packet.GetBuf()[0], ObjectDictionary::SCCommand::kCmdDataBlock);
    RawModeSPacket tpacket = RawModeSPacket((const char*)"8D7C1BE8581B66E9BD8CEEDC1C9F");
    packet.data_len_bytes = sizeof(RawModeSPacket);
    memcpy(packet.data, &tpacket, packet.data_len_bytes);
    EXPECT_FALSE(packet.IsValid());
    packet.PopulateCRC();
    EXPECT_TRUE(packet.IsValid());

    // Test packet ingestion.
    SPICoprocessorPacket::SCResponsePacket packet_copy =
        SPICoprocessorPacket::SCResponsePacket(packet.GetBuf(), packet.GetBufLenBytes());
    EXPECT_TRUE(packet_copy.IsValid());
    EXPECT_EQ(packet_copy.cmd, packet.cmd);
    RawModeSPacket* tpacket_copy = (RawModeSPacket*)packet_copy.data;
    EXPECT_EQ(tpacket_copy->buffer[0], 0x8D7C1BE8u);
    EXPECT_EQ(tpacket_copy->buffer[1], 0x581B66E9u);
    EXPECT_EQ(tpacket_copy->buffer[2], 0xBD8CEEDCu);
    EXPECT_EQ(tpacket_copy->buffer[3], 0x1C9Fu << 16);
    // Poke the checksum and see it fail.
    tpacket_copy->buffer[0] = tpacket_copy->buffer[0] << 1;
    EXPECT_FALSE(packet_copy.IsValid());
    // Make sure original packet was not affected.
    EXPECT_TRUE(packet.IsValid());
}

void BuildAckPacket(SPICoprocessorPacket::SCAckPacket& packet, bool success) {
    packet.ack = success ? 1 : 0;
    packet.PopulateCRC();
}

TEST(SPICoprocessor, SCAckPacket) {
    // Basic send/receive round-trip.
    SPICoprocessorPacket::SCAckPacket packet;
    EXPECT_EQ(packet.cmd, ObjectDictionary::SCCommand::kCmdAck);  // Default must be kCmdAck.
    EXPECT_EQ(packet.GetBufLenBytes(), SPICoprocessorPacket::SCAckPacket::kBufLenBytes);
    BuildAckPacket(packet, true);
    EXPECT_TRUE(packet.IsValid());
    EXPECT_EQ(packet.ack, 1);

    // NACK.
    SPICoprocessorPacket::SCAckPacket nack_packet;
    BuildAckPacket(nack_packet, false);
    EXPECT_TRUE(nack_packet.IsValid());
    EXPECT_EQ(nack_packet.ack, 0);

    // Poking any field must invalidate the CRC.
    nack_packet.ack = ~nack_packet.ack;
    EXPECT_FALSE(nack_packet.IsValid());
}

TEST(SPICoprocessor, SCResponsePacketScrambleBuf) {
    // Regression: pre-scrambling the data buffer must not corrupt the CRC when only a subset of bytes is populated.
    // GetCRCPtr() uses data_len_bytes to locate the CRC, so bytes beyond data_len_bytes are irrelevant.
    SCOPED_TRACE("SCResponsePacketScrambleBuf");

    SPICoprocessorPacket::SCResponsePacket packet;
    memset(packet.data, 0xFF, SPICoprocessorPacket::SCResponsePacket::kDataMaxLenBytes);
    packet.cmd = ObjectDictionary::SCCommand::kCmdDataBlock;
    packet.data_len_bytes = 1;
    packet.data[0] = 1;
    packet.PopulateCRC();
    EXPECT_TRUE(packet.IsValid());
}

TEST(SPICoprocessor, LogMessagesOffsetChunkedReassembly) {
    // Regression for the FIFO chunked-read offset bug that broke OTA when kSPITransactionMaxLenBytes was reduced
    // 4096->2048. A log burst whose packed size exceeds one SPI transaction is read by the master in multiple chunks
    // with an increasing byte offset; PackLogMessages must serve each offset window so the master's reassembly
    // (memcpy of each chunk at its offset) reconstructs the full packed stream and UnpackLogMessages recovers every
    // message. Before the fix, PackLogMessages ignored offset and re-served the head of the queue, so the tail of a
    // multi-chunk read was a duplicate of the head.
    ObjectDictionary od;

    PFBQueue<ObjectDictionary::LogMessage> in_queue =
        PFBQueue<ObjectDictionary::LogMessage>({.buf_len_num_elements = 16, .buffer = nullptr});
    const uint16_t kNumMessages = 8;
    for (uint16_t m = 0; m < kNumMessages; m++) {
        ObjectDictionary::LogMessage msg;
        msg.log_level = static_cast<SettingsManager::LogLevel>(m % 3);
        msg.num_chars = 400 + m;  // ~400+ chars each: total packed clearly exceeds one 2048-Byte transaction.
        for (uint16_t i = 0; i < msg.num_chars; i++) {
            msg.message[i] = static_cast<char>('A' + ((m + i) % 26));
        }
        msg.message[msg.num_chars] = '\0';
        ASSERT_TRUE(in_queue.Enqueue(msg));
    }

    // Ground-truth single-shot pack (offset 0, buffer large enough for everything).
    uint8_t full_buf[ObjectDictionary::kLogMessageMaxNumChars * 16] = {0};
    uint16_t total_packed = od.PackLogMessages(full_buf, sizeof(full_buf), in_queue, kNumMessages, 0);
    // Sanity: the burst must span more than one transaction so the multi-chunk path is actually exercised.
    ASSERT_GT(total_packed, SPICoprocessorPacket::SCResponsePacket::kDataMaxLenBytes);

    // Simulate the master's chunked read + reassembly (mirrors SPICoprocessor::Read + PartialRead's memcpy at offset).
    uint8_t reassembled[sizeof(full_buf)] = {0};
    for (uint16_t offset = 0; offset < total_packed;
         offset += SPICoprocessorPacket::SCResponsePacket::kDataMaxLenBytes) {
        uint16_t this_len = SPICoprocessorPacket::SCResponsePacket::kDataMaxLenBytes;
        if (total_packed - offset < this_len) {
            this_len = total_packed - offset;
        }
        uint8_t chunk[SPICoprocessorPacket::SCResponsePacket::kDataMaxLenBytes] = {0};
        od.PackLogMessages(chunk, this_len, in_queue, kNumMessages, offset);
        memcpy(reassembled + offset, chunk, this_len);
    }

    // Chunked reassembly must be byte-identical to the single-shot pack.
    EXPECT_EQ(0, memcmp(full_buf, reassembled, total_packed));

    // And UnpackLogMessages must recover every message identically.
    PFBQueue<ObjectDictionary::LogMessage> out_queue =
        PFBQueue<ObjectDictionary::LogMessage>({.buf_len_num_elements = 16, .buffer = nullptr});
    uint16_t num_unpacked = od.UnpackLogMessages(reassembled, total_packed, out_queue, kNumMessages);
    EXPECT_EQ(num_unpacked, kNumMessages);
    for (uint16_t m = 0; m < kNumMessages; m++) {
        ObjectDictionary::LogMessage msg;
        ASSERT_TRUE(out_queue.Dequeue(msg));
        EXPECT_EQ(msg.num_chars, static_cast<uint16_t>(400 + m));
        EXPECT_EQ(msg.log_level, static_cast<SettingsManager::LogLevel>(m % 3));
        for (uint16_t i = 0; i < msg.num_chars; i++) {
            EXPECT_EQ(msg.message[i], static_cast<char>('A' + ((m + i) % 26)));
        }
    }
}