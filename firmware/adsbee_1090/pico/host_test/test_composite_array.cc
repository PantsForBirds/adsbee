#include "composite_array.hh"
#include "gtest/gtest.h"

TEST(CompositeArray, PackRawPacketsBufferNoQueues) {
    uint8_t buffer[100] = {0};
    CompositeArray::RawPackets packets =
        CompositeArray::PackRawPacketsBuffer(buffer, sizeof(buffer), nullptr, nullptr, nullptr);

    // Expect only the header to be present with zero counts
    EXPECT_EQ(packets.len_bytes, sizeof(CompositeArray::RawPackets::Header));
    EXPECT_EQ(packets.header->num_mode_s_packets, 0);
    EXPECT_EQ(packets.header->num_uat_adsb_packets, 0);
    EXPECT_EQ(packets.header->num_uat_uplink_packets, 0);
    EXPECT_EQ(packets.mode_s_packets, nullptr);
    EXPECT_EQ(packets.uat_adsb_packets, nullptr);
    EXPECT_EQ(packets.uat_uplink_packets, nullptr);
}

TEST(CompositeArray, PackRawPacketsBufferTooManyPackets) {
    uint8_t buffer[sizeof(CompositeArray::RawPackets::Header) + 5 * sizeof(RawModeSPacket)] = {0};

    PFBQueue<RawModeSPacket>::PFBQueueConfig queue_config = {
        .buf_len_num_elements = 10, .buffer = nullptr, .overwrite_when_full = false};
    PFBQueue<RawModeSPacket> mode_s_queue = PFBQueue<RawModeSPacket>(queue_config);

    RawModeSPacket dummy_packet;
    for (int i = 0; i < queue_config.buf_len_num_elements; i++) {
        dummy_packet.buffer[0] = i;  // Differentiate packets by first word
        mode_s_queue.Enqueue(dummy_packet);
    }
    CompositeArray::RawPackets packets =
        CompositeArray::PackRawPacketsBuffer(buffer, sizeof(buffer), &mode_s_queue, nullptr, nullptr);

    // Expect only 5 packets to be packed due to buffer size
    EXPECT_EQ(packets.len_bytes, sizeof(CompositeArray::RawPackets::Header) + 5 * sizeof(RawModeSPacket));
    EXPECT_EQ(packets.header->num_mode_s_packets, 5);
    EXPECT_EQ(packets.header->num_uat_adsb_packets, 0);
    EXPECT_EQ(packets.header->num_uat_uplink_packets, 0);
    EXPECT_NE(packets.mode_s_packets, nullptr);
    EXPECT_EQ(packets.uat_adsb_packets, nullptr);
    EXPECT_EQ(packets.uat_uplink_packets, nullptr);

    for (uint16_t i = 0; i < packets.header->num_mode_s_packets; i++) {
        EXPECT_EQ(packets.mode_s_packets[i].buffer[0], i);
    }
}

TEST(CompositeArray, PackRawPacketsBufferTooManyMixedPackets) {
    const uint16_t num_mode_s_packets_to_enqueue = 4;
    const uint16_t num_uat_adsb_packets_to_enqueue = 3;
    const uint16_t num_uat_uplink_packets_to_enqueue = 6;

    uint8_t buffer[sizeof(CompositeArray::RawPackets::Header) + num_mode_s_packets_to_enqueue * sizeof(RawModeSPacket) +
                   num_uat_adsb_packets_to_enqueue * sizeof(RawUATADSBPacket) +
                   (num_uat_uplink_packets_to_enqueue - 1) * sizeof(RawUATUplinkPacket)] = {0};

    PFBQueue<RawModeSPacket>::PFBQueueConfig mode_s_queue_config = {
        .buf_len_num_elements = 10, .buffer = nullptr, .overwrite_when_full = false};
    PFBQueue<RawModeSPacket> mode_s_queue = PFBQueue<RawModeSPacket>(mode_s_queue_config);
    PFBQueue<RawUATADSBPacket>::PFBQueueConfig uat_adsb_queue_config = {
        .buf_len_num_elements = 10, .buffer = nullptr, .overwrite_when_full = false};
    PFBQueue<RawUATADSBPacket> uat_adsb_queue = PFBQueue<RawUATADSBPacket>(uat_adsb_queue_config);
    PFBQueue<RawUATUplinkPacket>::PFBQueueConfig uat_uplink_queue_config = {
        .buf_len_num_elements = 10, .buffer = nullptr, .overwrite_when_full = false};
    PFBQueue<RawUATUplinkPacket> uat_uplink_queue = PFBQueue<RawUATUplinkPacket>(uat_uplink_queue_config);

    RawModeSPacket dummy_mode_s_packet;
    for (int i = 0; i < num_mode_s_packets_to_enqueue; i++) {
        dummy_mode_s_packet.buffer[0] = i;  // Differentiate packets by first word
        mode_s_queue.Enqueue(dummy_mode_s_packet);
    }
    RawUATADSBPacket dummy_uat_adsb_packet;
    for (int i = 0; i < num_uat_adsb_packets_to_enqueue; i++) {
        dummy_uat_adsb_packet.buffer[0] = i;  // Differentiate packets by first word
        uat_adsb_queue.Enqueue(dummy_uat_adsb_packet);
    }
    RawUATUplinkPacket dummy_uat_uplink_packet;
    for (int i = 0; i < num_uat_uplink_packets_to_enqueue; i++) {
        dummy_uat_uplink_packet.encoded_message[0] = i;  // Differentiate packets by first word
        uat_uplink_queue.Enqueue(dummy_uat_uplink_packet);
    }
    CompositeArray::RawPackets packets =
        CompositeArray::PackRawPacketsBuffer(buffer, sizeof(buffer), &mode_s_queue, &uat_adsb_queue, &uat_uplink_queue);

    // Expect only 3 packets to be packed due to buffer size
    EXPECT_EQ(packets.len_bytes, sizeof(CompositeArray::RawPackets::Header) +
                                     num_mode_s_packets_to_enqueue * sizeof(RawModeSPacket) +
                                     num_uat_adsb_packets_to_enqueue * sizeof(RawUATADSBPacket) +
                                     (num_uat_uplink_packets_to_enqueue - 1) * sizeof(RawUATUplinkPacket));
    EXPECT_EQ(packets.header->num_mode_s_packets, num_mode_s_packets_to_enqueue);
    EXPECT_EQ(packets.header->num_uat_adsb_packets, num_uat_adsb_packets_to_enqueue);
    EXPECT_EQ(packets.header->num_uat_uplink_packets, num_uat_uplink_packets_to_enqueue - 1);
    EXPECT_NE(packets.mode_s_packets, nullptr);
    EXPECT_NE(packets.uat_adsb_packets, nullptr);
    EXPECT_NE(packets.uat_uplink_packets, nullptr);

    for (uint16_t i = 0; i < packets.header->num_mode_s_packets; i++) {
        EXPECT_EQ(packets.mode_s_packets[i].buffer[0], i);
    }
    for (uint16_t i = 0; i < packets.header->num_uat_adsb_packets; i++) {
        EXPECT_EQ(packets.uat_adsb_packets[i].buffer[0], i);
    }
    for (uint16_t i = 0; i < packets.header->num_uat_uplink_packets; i++) {
        EXPECT_EQ(packets.uat_uplink_packets[i].encoded_message[0], i);
    }
}

TEST(CompositeArray, PackUnpackRawPacketsBufferMixedPackets) {
    const uint16_t num_mode_s_packets_to_enqueue = 4;
    const uint16_t num_uat_adsb_packets_to_enqueue = 3;
    const uint16_t num_uat_uplink_packets_to_enqueue = 6;

    uint8_t buffer[sizeof(CompositeArray::RawPackets::Header) + num_mode_s_packets_to_enqueue * sizeof(RawModeSPacket) +
                   num_uat_adsb_packets_to_enqueue * sizeof(RawUATADSBPacket) +
                   (num_uat_uplink_packets_to_enqueue) * sizeof(RawUATUplinkPacket)] = {0};

    SCOPED_TRACE("PackUnpackRawPacketsBufferMixedPackets");

    PFBQueue<RawModeSPacket>::PFBQueueConfig mode_s_queue_config = {
        .buf_len_num_elements = 10, .buffer = nullptr, .overwrite_when_full = false};
    PFBQueue<RawModeSPacket> mode_s_queue = PFBQueue<RawModeSPacket>(mode_s_queue_config);
    PFBQueue<RawUATADSBPacket>::PFBQueueConfig uat_adsb_queue_config = {
        .buf_len_num_elements = 10, .buffer = nullptr, .overwrite_when_full = false};
    PFBQueue<RawUATADSBPacket> uat_adsb_queue = PFBQueue<RawUATADSBPacket>(uat_adsb_queue_config);
    PFBQueue<RawUATUplinkPacket>::PFBQueueConfig uat_uplink_queue_config = {
        .buf_len_num_elements = 10, .buffer = nullptr, .overwrite_when_full = false};
    PFBQueue<RawUATUplinkPacket> uat_uplink_queue = PFBQueue<RawUATUplinkPacket>(uat_uplink_queue_config);

    RawModeSPacket dummy_mode_s_packet;
    for (int i = 0; i < num_mode_s_packets_to_enqueue; i++) {
        dummy_mode_s_packet.buffer[0] = i;  // Differentiate packets by first word
        mode_s_queue.Enqueue(dummy_mode_s_packet);
    }
    RawUATADSBPacket dummy_uat_adsb_packet;
    for (int i = 0; i < num_uat_adsb_packets_to_enqueue; i++) {
        dummy_uat_adsb_packet.buffer[0] = i;  // Differentiate packets by first word
        uat_adsb_queue.Enqueue(dummy_uat_adsb_packet);
    }
    RawUATUplinkPacket dummy_uat_uplink_packet;
    for (int i = 0; i < num_uat_uplink_packets_to_enqueue; i++) {
        dummy_uat_uplink_packet.encoded_message[0] = i;  // Differentiate packets by first word
        uat_uplink_queue.Enqueue(dummy_uat_uplink_packet);
    }
    CompositeArray::RawPackets packets =
        CompositeArray::PackRawPacketsBuffer(buffer, sizeof(buffer), &mode_s_queue, &uat_adsb_queue, &uat_uplink_queue);

    EXPECT_TRUE(mode_s_queue.IsEmpty());
    EXPECT_TRUE(uat_adsb_queue.IsEmpty());
    EXPECT_TRUE(uat_uplink_queue.IsEmpty());

    // Make sure we get an error if we try to unpack without providing all queues.
    EXPECT_FALSE(CompositeArray::UnpackRawPacketsBufferToQueues(buffer, packets.len_bytes, &mode_s_queue,
                                                                &uat_adsb_queue, nullptr));
    EXPECT_FALSE(CompositeArray::UnpackRawPacketsBufferToQueues(buffer, packets.len_bytes, nullptr, &uat_adsb_queue,
                                                                &uat_uplink_queue));
    EXPECT_FALSE(CompositeArray::UnpackRawPacketsBufferToQueues(buffer, packets.len_bytes, &mode_s_queue, nullptr,
                                                                &uat_uplink_queue));

    EXPECT_TRUE(CompositeArray::UnpackRawPacketsBufferToQueues(buffer, packets.len_bytes, &mode_s_queue,
                                                               &uat_adsb_queue, &uat_uplink_queue));
    EXPECT_EQ(mode_s_queue.Length(), num_mode_s_packets_to_enqueue);
    EXPECT_EQ(uat_adsb_queue.Length(), num_uat_adsb_packets_to_enqueue);
    EXPECT_EQ(uat_uplink_queue.Length(), num_uat_uplink_packets_to_enqueue);

    RawModeSPacket dequeued_mode_s_packet;
    for (uint16_t i = 0; i < num_mode_s_packets_to_enqueue; i++) {
        EXPECT_TRUE(mode_s_queue.Dequeue(dequeued_mode_s_packet));
        EXPECT_EQ(dequeued_mode_s_packet.buffer[0], i);
    }
    RawUATADSBPacket dequeued_uat_adsb_packet;
    for (uint16_t i = 0; i < num_uat_adsb_packets_to_enqueue; i++) {
        EXPECT_TRUE(uat_adsb_queue.Dequeue(dequeued_uat_adsb_packet));
        EXPECT_EQ(dequeued_uat_adsb_packet.buffer[0], i);
    }
    RawUATUplinkPacket dequeued_uat_uplink_packet;
    for (uint16_t i = 0; i < num_uat_uplink_packets_to_enqueue; i++) {
        EXPECT_TRUE(uat_uplink_queue.Dequeue(dequeued_uat_uplink_packet));
        EXPECT_EQ(dequeued_uat_uplink_packet.encoded_message[0], i);
    }
}

TEST(CompositeArray, MergeRawPacketsBuffersEmptyIntoEmpty) {
    uint8_t dst_buf[CompositeArray::RawPackets::kMaxLenBytes] = {0};
    uint8_t src_buf[CompositeArray::RawPackets::kMaxLenBytes] = {0};

    // Pack empty composite arrays into both buffers.
    CompositeArray::PackRawPacketsBuffer(dst_buf, sizeof(dst_buf), nullptr, nullptr, nullptr);
    CompositeArray::PackRawPacketsBuffer(src_buf, sizeof(src_buf), nullptr, nullptr, nullptr);

    EXPECT_TRUE(CompositeArray::MergeRawPacketsBuffers(dst_buf, src_buf));

    auto* hdr = reinterpret_cast<CompositeArray::RawPackets::Header*>(dst_buf);
    EXPECT_EQ(hdr->num_mode_s_packets, 0);
    EXPECT_EQ(hdr->num_uat_adsb_packets, 0);
    EXPECT_EQ(hdr->num_uat_uplink_packets, 0);
}

TEST(CompositeArray, MergeRawPacketsBuffersModeS) {
    const uint16_t kDstCount = 3;
    const uint16_t kSrcCount = 4;

    uint8_t dst_buf[CompositeArray::RawPackets::kMaxLenBytes] = {0};
    uint8_t src_buf[CompositeArray::RawPackets::kMaxLenBytes] = {0};

    PFBQueue<RawModeSPacket>::PFBQueueConfig cfg = {
        .buf_len_num_elements = 10, .buffer = nullptr, .overwrite_when_full = false};
    PFBQueue<RawModeSPacket> dst_q(cfg);
    PFBQueue<RawModeSPacket> src_q(cfg);

    RawModeSPacket pkt;
    for (int i = 0; i < kDstCount; i++) {
        pkt.buffer[0] = i;
        dst_q.Enqueue(pkt);
    }
    for (int i = 0; i < kSrcCount; i++) {
        pkt.buffer[0] = 100 + i;
        src_q.Enqueue(pkt);
    }

    CompositeArray::PackRawPacketsBuffer(dst_buf, sizeof(dst_buf), &dst_q, nullptr, nullptr);
    CompositeArray::PackRawPacketsBuffer(src_buf, sizeof(src_buf), &src_q, nullptr, nullptr);

    EXPECT_TRUE(CompositeArray::MergeRawPacketsBuffers(dst_buf, src_buf));

    auto* hdr = reinterpret_cast<CompositeArray::RawPackets::Header*>(dst_buf);
    EXPECT_EQ(hdr->num_mode_s_packets, kDstCount + kSrcCount);
    EXPECT_EQ(hdr->num_uat_adsb_packets, 0);
    EXPECT_EQ(hdr->num_uat_uplink_packets, 0);

    // Verify dst packets come first, then src packets.
    auto* ms = reinterpret_cast<RawModeSPacket*>(dst_buf + sizeof(CompositeArray::RawPackets::Header));
    for (int i = 0; i < kDstCount; i++) {
        EXPECT_EQ(ms[i].buffer[0], (uint32_t)i);
    }
    for (int i = 0; i < kSrcCount; i++) {
        EXPECT_EQ(ms[kDstCount + i].buffer[0], (uint32_t)(100 + i));
    }
}

TEST(CompositeArray, MergeRawPacketsBuffersMixed) {
    // Keep uplink counts small: sizeof(RawUATUplinkPacket)=568, so 3 total = 1704B of uplink alone.
    // Use kDstUPL=0, kSrcUPL=1 so combined = 8 + 3*32 + 5*68 + 1*568 = 1012B, well under 2000.
    const uint16_t kDstMS = 2, kDstADSB = 3, kDstUPL = 0;
    const uint16_t kSrcMS = 1, kSrcADSB = 2, kSrcUPL = 1;

    uint8_t dst_buf[CompositeArray::RawPackets::kMaxLenBytes] = {0};
    uint8_t src_buf[CompositeArray::RawPackets::kMaxLenBytes] = {0};

    auto make_ms_q = [](uint16_t count, uint32_t base) {
        PFBQueue<RawModeSPacket>::PFBQueueConfig cfg = {
            .buf_len_num_elements = 10, .buffer = nullptr, .overwrite_when_full = false};
        PFBQueue<RawModeSPacket> q(cfg);
        RawModeSPacket pkt;
        for (uint16_t i = 0; i < count; i++) {
            pkt.buffer[0] = base + i;
            q.Enqueue(pkt);
        }
        return q;
    };
    auto make_adsb_q = [](uint16_t count, uint8_t base) {
        PFBQueue<RawUATADSBPacket>::PFBQueueConfig cfg = {
            .buf_len_num_elements = 10, .buffer = nullptr, .overwrite_when_full = false};
        PFBQueue<RawUATADSBPacket> q(cfg);
        RawUATADSBPacket pkt;
        for (uint16_t i = 0; i < count; i++) {
            pkt.buffer[0] = base + i;
            q.Enqueue(pkt);
        }
        return q;
    };
    auto make_upl_q = [](uint16_t count, uint8_t base) {
        PFBQueue<RawUATUplinkPacket>::PFBQueueConfig cfg = {
            .buf_len_num_elements = 10, .buffer = nullptr, .overwrite_when_full = false};
        PFBQueue<RawUATUplinkPacket> q(cfg);
        RawUATUplinkPacket pkt;
        for (uint16_t i = 0; i < count; i++) {
            pkt.encoded_message[0] = base + i;
            q.Enqueue(pkt);
        }
        return q;
    };

    auto dst_ms_q = make_ms_q(kDstMS, 0);
    auto dst_adsb_q = make_adsb_q(kDstADSB, 10);
    auto dst_upl_q = make_upl_q(kDstUPL, 20);
    auto src_ms_q = make_ms_q(kSrcMS, 100);
    auto src_adsb_q = make_adsb_q(kSrcADSB, 110);
    auto src_upl_q = make_upl_q(kSrcUPL, 120);

    CompositeArray::PackRawPacketsBuffer(dst_buf, sizeof(dst_buf), &dst_ms_q, &dst_adsb_q, &dst_upl_q);
    CompositeArray::PackRawPacketsBuffer(src_buf, sizeof(src_buf), &src_ms_q, &src_adsb_q, &src_upl_q);

    EXPECT_TRUE(CompositeArray::MergeRawPacketsBuffers(dst_buf, src_buf));

    CompositeArray::RawPackets result;
    ASSERT_TRUE(CompositeArray::UnpackRawPacketsBuffer(result, dst_buf, sizeof(dst_buf)));

    EXPECT_EQ(result.header->num_mode_s_packets, kDstMS + kSrcMS);
    EXPECT_EQ(result.header->num_uat_adsb_packets, kDstADSB + kSrcADSB);
    EXPECT_EQ(result.header->num_uat_uplink_packets, kDstUPL + kSrcUPL);

    // Mode S: dst first, then src.
    for (int i = 0; i < kDstMS; i++) EXPECT_EQ(result.mode_s_packets[i].buffer[0], (uint32_t)i);
    for (int i = 0; i < kSrcMS; i++) EXPECT_EQ(result.mode_s_packets[kDstMS + i].buffer[0], (uint32_t)(100 + i));

    // ADS-B: dst first, then src.
    for (int i = 0; i < kDstADSB; i++) EXPECT_EQ(result.uat_adsb_packets[i].buffer[0], (uint8_t)(10 + i));
    for (int i = 0; i < kSrcADSB; i++) EXPECT_EQ(result.uat_adsb_packets[kDstADSB + i].buffer[0], (uint8_t)(110 + i));

    // Uplink: dst first, then src.
    for (int i = 0; i < kDstUPL; i++)
        EXPECT_EQ(result.uat_uplink_packets[i].encoded_message[0], (uint8_t)(20 + i));
    for (int i = 0; i < kSrcUPL; i++)
        EXPECT_EQ(result.uat_uplink_packets[kDstUPL + i].encoded_message[0], (uint8_t)(120 + i));
}

TEST(CompositeArray, MergeRawPacketsBuffersNoModificationOnOverflow) {
    // Fill dst_buf to near capacity with Mode S packets, then attempt a merge that would overflow.
    // dst: as many Mode S packets as possible just under kMaxLenBytes
    // src: one Mode S packet (should push combined size over limit)
    const uint16_t max_ms =
        (CompositeArray::RawPackets::kMaxLenBytes - sizeof(CompositeArray::RawPackets::Header)) / sizeof(RawModeSPacket);

    uint8_t dst_buf[CompositeArray::RawPackets::kMaxLenBytes] = {0};
    uint8_t src_buf[CompositeArray::RawPackets::kMaxLenBytes] = {0};

    PFBQueue<RawModeSPacket>::PFBQueueConfig cfg = {
        .buf_len_num_elements = max_ms + 1, .buffer = nullptr, .overwrite_when_full = false};
    PFBQueue<RawModeSPacket> dst_q(cfg);
    PFBQueue<RawModeSPacket> src_q(cfg);

    RawModeSPacket pkt;
    pkt.buffer[0] = 0xDEADBEEF;
    for (uint16_t i = 0; i < max_ms; i++) dst_q.Enqueue(pkt);

    pkt.buffer[0] = 0xCAFEBABE;
    src_q.Enqueue(pkt);

    CompositeArray::PackRawPacketsBuffer(dst_buf, sizeof(dst_buf), &dst_q, nullptr, nullptr);
    CompositeArray::PackRawPacketsBuffer(src_buf, sizeof(src_buf), &src_q, nullptr, nullptr);

    EXPECT_FALSE(CompositeArray::MergeRawPacketsBuffers(dst_buf, src_buf));

    // dst_buf must be unmodified: header count unchanged, first packet data intact.
    auto* hdr = reinterpret_cast<CompositeArray::RawPackets::Header*>(dst_buf);
    EXPECT_EQ(hdr->num_mode_s_packets, max_ms);
    auto* ms = reinterpret_cast<RawModeSPacket*>(dst_buf + sizeof(CompositeArray::RawPackets::Header));
    EXPECT_EQ(ms[0].buffer[0], (uint32_t)0xDEADBEEF);
}

TEST(CompositeArray, MergeRawPacketsBuffersResultIsValid) {
    uint8_t dst_buf[CompositeArray::RawPackets::kMaxLenBytes] = {0};
    uint8_t src_buf[CompositeArray::RawPackets::kMaxLenBytes] = {0};

    PFBQueue<RawModeSPacket>::PFBQueueConfig ms_cfg = {
        .buf_len_num_elements = 10, .buffer = nullptr, .overwrite_when_full = false};
    PFBQueue<RawUATADSBPacket>::PFBQueueConfig adsb_cfg = {
        .buf_len_num_elements = 10, .buffer = nullptr, .overwrite_when_full = false};
    PFBQueue<RawModeSPacket> dst_ms_q(ms_cfg), src_ms_q(ms_cfg);
    PFBQueue<RawUATADSBPacket> dst_adsb_q(adsb_cfg), src_adsb_q(adsb_cfg);

    RawModeSPacket ms_pkt;
    ms_pkt.buffer[0] = 1;
    dst_ms_q.Enqueue(ms_pkt);
    ms_pkt.buffer[0] = 2;
    src_ms_q.Enqueue(ms_pkt);

    RawUATADSBPacket adsb_pkt;
    adsb_pkt.buffer[0] = 3;
    dst_adsb_q.Enqueue(adsb_pkt);
    adsb_pkt.buffer[0] = 4;
    src_adsb_q.Enqueue(adsb_pkt);

    CompositeArray::PackRawPacketsBuffer(dst_buf, sizeof(dst_buf), &dst_ms_q, &dst_adsb_q, nullptr);
    CompositeArray::PackRawPacketsBuffer(src_buf, sizeof(src_buf), &src_ms_q, &src_adsb_q, nullptr);

    EXPECT_TRUE(CompositeArray::MergeRawPacketsBuffers(dst_buf, src_buf));

    CompositeArray::RawPackets result;
    EXPECT_TRUE(CompositeArray::UnpackRawPacketsBuffer(result, dst_buf, sizeof(dst_buf)));
    EXPECT_TRUE(result.IsValid());
    EXPECT_EQ(result.header->num_mode_s_packets, 2);
    EXPECT_EQ(result.header->num_uat_adsb_packets, 2);
    EXPECT_EQ(result.header->num_uat_uplink_packets, 0);
    EXPECT_EQ(result.mode_s_packets[0].buffer[0], (uint32_t)1);
    EXPECT_EQ(result.mode_s_packets[1].buffer[0], (uint32_t)2);
    EXPECT_EQ(result.uat_adsb_packets[0].buffer[0], (uint8_t)3);
    EXPECT_EQ(result.uat_adsb_packets[1].buffer[0], (uint8_t)4);
}

TEST(CompositeArray, PackUnpackRawPacketsBufferWithRemoteID) {
    const uint16_t num_mode_s = 2;
    const uint16_t num_uat_adsb = 3;
    const uint16_t num_uat_uplink = 1;
    const uint16_t num_remote_id = 4;

    uint8_t buffer[CompositeArray::RawPackets::kMaxLenBytes] = {0};

    PFBQueue<RawModeSPacket>::PFBQueueConfig ms_cfg = {
        .buf_len_num_elements = 10, .buffer = nullptr, .overwrite_when_full = false};
    PFBQueue<RawUATADSBPacket>::PFBQueueConfig adsb_cfg = {
        .buf_len_num_elements = 10, .buffer = nullptr, .overwrite_when_full = false};
    PFBQueue<RawUATUplinkPacket>::PFBQueueConfig upl_cfg = {
        .buf_len_num_elements = 10, .buffer = nullptr, .overwrite_when_full = false};
    PFBQueue<RawRemoteIDPacket>::PFBQueueConfig rid_cfg = {
        .buf_len_num_elements = 10, .buffer = nullptr, .overwrite_when_full = false};
    PFBQueue<RawModeSPacket> mode_s_queue(ms_cfg);
    PFBQueue<RawUATADSBPacket> uat_adsb_queue(adsb_cfg);
    PFBQueue<RawUATUplinkPacket> uat_uplink_queue(upl_cfg);
    PFBQueue<RawRemoteIDPacket> remote_id_queue(rid_cfg);

    RawModeSPacket ms_pkt;
    for (int i = 0; i < num_mode_s; i++) {
        ms_pkt.buffer[0] = i;
        mode_s_queue.Enqueue(ms_pkt);
    }
    RawUATADSBPacket adsb_pkt;
    for (int i = 0; i < num_uat_adsb; i++) {
        adsb_pkt.buffer[0] = i;
        uat_adsb_queue.Enqueue(adsb_pkt);
    }
    RawUATUplinkPacket upl_pkt;
    for (int i = 0; i < num_uat_uplink; i++) {
        upl_pkt.encoded_message[0] = i;
        uat_uplink_queue.Enqueue(upl_pkt);
    }
    RawRemoteIDPacket rid_pkt;
    for (int i = 0; i < num_remote_id; i++) {
        rid_pkt.payload[0] = 30 + i;         // Differentiate by payload byte.
        rid_pkt.source_mac[5] = 30 + i;      // ...and by MAC.
        rid_pkt.transport = RawRemoteIDPacket::kTransportBT5LongRange;
        remote_id_queue.Enqueue(rid_pkt);
    }

    CompositeArray::RawPackets packets = CompositeArray::PackRawPacketsBuffer(
        buffer, sizeof(buffer), &mode_s_queue, &uat_adsb_queue, &uat_uplink_queue, &remote_id_queue);

    EXPECT_EQ(packets.header->num_mode_s_packets, num_mode_s);
    EXPECT_EQ(packets.header->num_uat_adsb_packets, num_uat_adsb);
    EXPECT_EQ(packets.header->num_uat_uplink_packets, num_uat_uplink);
    EXPECT_EQ(packets.header->num_remote_id_packets, num_remote_id);
    EXPECT_TRUE(packets.IsValid());

    // Unpacking must fail if the Remote ID queue is omitted but the header claims Remote ID packets.
    EXPECT_FALSE(CompositeArray::UnpackRawPacketsBufferToQueues(buffer, packets.len_bytes, &mode_s_queue,
                                                               &uat_adsb_queue, &uat_uplink_queue, nullptr));

    EXPECT_TRUE(CompositeArray::UnpackRawPacketsBufferToQueues(buffer, packets.len_bytes, &mode_s_queue,
                                                              &uat_adsb_queue, &uat_uplink_queue, &remote_id_queue));
    EXPECT_EQ(remote_id_queue.Length(), num_remote_id);
    RawRemoteIDPacket dequeued;
    for (uint16_t i = 0; i < num_remote_id; i++) {
        EXPECT_TRUE(remote_id_queue.Dequeue(dequeued));
        EXPECT_EQ(dequeued.payload[0], (uint8_t)(30 + i));
        EXPECT_EQ(dequeued.source_mac[5], (uint8_t)(30 + i));
        EXPECT_EQ(dequeued.transport, RawRemoteIDPacket::kTransportBT5LongRange);
    }
}

TEST(CompositeArray, MergeRawPacketsBuffersWithRemoteID) {
    // Uplink packets are large (568B), so keep uplink + Remote ID counts small to stay under kMaxLenBytes.
    const uint16_t kDstMS = 2, kDstADSB = 2, kDstUPL = 1, kDstRID = 1;
    const uint16_t kSrcMS = 1, kSrcADSB = 1, kSrcUPL = 1, kSrcRID = 1;

    uint8_t dst_buf[CompositeArray::RawPackets::kMaxLenBytes] = {0};
    uint8_t src_buf[CompositeArray::RawPackets::kMaxLenBytes] = {0};

    auto make_ms_q = [](uint16_t count, uint32_t base) {
        PFBQueue<RawModeSPacket>::PFBQueueConfig cfg = {
            .buf_len_num_elements = 10, .buffer = nullptr, .overwrite_when_full = false};
        PFBQueue<RawModeSPacket> q(cfg);
        RawModeSPacket pkt;
        for (uint16_t i = 0; i < count; i++) {
            pkt.buffer[0] = base + i;
            q.Enqueue(pkt);
        }
        return q;
    };
    auto make_adsb_q = [](uint16_t count, uint8_t base) {
        PFBQueue<RawUATADSBPacket>::PFBQueueConfig cfg = {
            .buf_len_num_elements = 10, .buffer = nullptr, .overwrite_when_full = false};
        PFBQueue<RawUATADSBPacket> q(cfg);
        RawUATADSBPacket pkt;
        for (uint16_t i = 0; i < count; i++) {
            pkt.buffer[0] = base + i;
            q.Enqueue(pkt);
        }
        return q;
    };
    auto make_upl_q = [](uint16_t count, uint8_t base) {
        PFBQueue<RawUATUplinkPacket>::PFBQueueConfig cfg = {
            .buf_len_num_elements = 10, .buffer = nullptr, .overwrite_when_full = false};
        PFBQueue<RawUATUplinkPacket> q(cfg);
        RawUATUplinkPacket pkt;
        for (uint16_t i = 0; i < count; i++) {
            pkt.encoded_message[0] = base + i;
            q.Enqueue(pkt);
        }
        return q;
    };
    auto make_rid_q = [](uint16_t count, uint8_t base) {
        PFBQueue<RawRemoteIDPacket>::PFBQueueConfig cfg = {
            .buf_len_num_elements = 10, .buffer = nullptr, .overwrite_when_full = false};
        PFBQueue<RawRemoteIDPacket> q(cfg);
        RawRemoteIDPacket pkt;
        for (uint16_t i = 0; i < count; i++) {
            pkt.payload[0] = base + i;
            q.Enqueue(pkt);
        }
        return q;
    };

    auto dst_ms_q = make_ms_q(kDstMS, 0);
    auto dst_adsb_q = make_adsb_q(kDstADSB, 10);
    auto dst_upl_q = make_upl_q(kDstUPL, 20);
    auto dst_rid_q = make_rid_q(kDstRID, 40);
    auto src_ms_q = make_ms_q(kSrcMS, 100);
    auto src_adsb_q = make_adsb_q(kSrcADSB, 110);
    auto src_upl_q = make_upl_q(kSrcUPL, 120);
    auto src_rid_q = make_rid_q(kSrcRID, 140);

    CompositeArray::PackRawPacketsBuffer(dst_buf, sizeof(dst_buf), &dst_ms_q, &dst_adsb_q, &dst_upl_q, &dst_rid_q);
    CompositeArray::PackRawPacketsBuffer(src_buf, sizeof(src_buf), &src_ms_q, &src_adsb_q, &src_upl_q, &src_rid_q);

    EXPECT_TRUE(CompositeArray::MergeRawPacketsBuffers(dst_buf, src_buf));

    CompositeArray::RawPackets result;
    ASSERT_TRUE(CompositeArray::UnpackRawPacketsBuffer(result, dst_buf, sizeof(dst_buf)));
    EXPECT_TRUE(result.IsValid());

    EXPECT_EQ(result.header->num_mode_s_packets, kDstMS + kSrcMS);
    EXPECT_EQ(result.header->num_uat_adsb_packets, kDstADSB + kSrcADSB);
    EXPECT_EQ(result.header->num_uat_uplink_packets, kDstUPL + kSrcUPL);
    EXPECT_EQ(result.header->num_remote_id_packets, kDstRID + kSrcRID);

    // Every type keeps dst packets ahead of src packets.
    EXPECT_EQ(result.mode_s_packets[0].buffer[0], (uint32_t)0);
    EXPECT_EQ(result.mode_s_packets[kDstMS].buffer[0], (uint32_t)100);
    EXPECT_EQ(result.uat_adsb_packets[0].buffer[0], (uint8_t)10);
    EXPECT_EQ(result.uat_adsb_packets[kDstADSB].buffer[0], (uint8_t)110);
    EXPECT_EQ(result.uat_uplink_packets[0].encoded_message[0], (uint8_t)20);
    EXPECT_EQ(result.uat_uplink_packets[kDstUPL].encoded_message[0], (uint8_t)120);
    EXPECT_EQ(result.remote_id_packets[0].payload[0], (uint8_t)40);
    EXPECT_EQ(result.remote_id_packets[kDstRID].payload[0], (uint8_t)140);
}

TEST(CompositeArray, MergeRawPacketsBuffersRemoteIDOnlyIntoEmpty) {
    uint8_t dst_buf[CompositeArray::RawPackets::kMaxLenBytes] = {0};
    uint8_t src_buf[CompositeArray::RawPackets::kMaxLenBytes] = {0};

    PFBQueue<RawRemoteIDPacket>::PFBQueueConfig cfg = {
        .buf_len_num_elements = 10, .buffer = nullptr, .overwrite_when_full = false};
    PFBQueue<RawRemoteIDPacket> src_rid_q(cfg);
    RawRemoteIDPacket pkt;
    for (int i = 0; i < 3; i++) {
        pkt.payload[0] = 200 + i;
        src_rid_q.Enqueue(pkt);
    }

    CompositeArray::PackRawPacketsBuffer(dst_buf, sizeof(dst_buf), nullptr, nullptr, nullptr, nullptr);
    CompositeArray::PackRawPacketsBuffer(src_buf, sizeof(src_buf), nullptr, nullptr, nullptr, &src_rid_q);

    EXPECT_TRUE(CompositeArray::MergeRawPacketsBuffers(dst_buf, src_buf));

    CompositeArray::RawPackets result;
    ASSERT_TRUE(CompositeArray::UnpackRawPacketsBuffer(result, dst_buf, sizeof(dst_buf)));
    EXPECT_EQ(result.header->num_mode_s_packets, 0);
    EXPECT_EQ(result.header->num_uat_adsb_packets, 0);
    EXPECT_EQ(result.header->num_uat_uplink_packets, 0);
    EXPECT_EQ(result.header->num_remote_id_packets, 3);
    EXPECT_EQ(result.remote_id_packets[0].payload[0], (uint8_t)200);
    EXPECT_EQ(result.remote_id_packets[2].payload[0], (uint8_t)202);
}

TEST(CompositeArray, RawRemoteIDPacketMessagePackDiscrimination) {
    RawRemoteIDPacket single;
    single.payload_len_bytes = RawRemoteIDPacket::kSingleMessageLenBytes;
    single.payload[0] = 0x02;  // Location message: high nibble 0x0 (not a pack).
    EXPECT_FALSE(single.GetIsMessagePack());

    RawRemoteIDPacket pack;
    pack.payload_len_bytes = 100;
    pack.payload[0] = 0xF2;  // High nibble 0xF => message pack.
    EXPECT_TRUE(pack.GetIsMessagePack());

    RawRemoteIDPacket empty;
    EXPECT_FALSE(empty.GetIsMessagePack());  // Zero-length payload is never a pack.
}

TEST(CompositeArray, RawPacketsHeaderIsValid) {
    CompositeArray::RawPackets packets;
    char error_msg[CompositeArray::RawPackets::kErrorMessageMaxLen];

    // Null header
    packets.header = nullptr;
    EXPECT_FALSE(packets.IsValid(error_msg));
    EXPECT_STREQ(error_msg, "Invalid CompositeArray::RawPackets: null header.");

    // Insufficient length for header
    CompositeArray::RawPackets::Header header = {};
    packets.header = &header;
    EXPECT_FALSE(packets.IsValid(error_msg));
    EXPECT_STREQ(error_msg, "Invalid CompositeArray::RawPackets: insufficient length for header.");

    // Insufficient length for packets
    packets.header->num_mode_s_packets = 2;
    packets.header->num_uat_adsb_packets = 1;
    packets.header->num_uat_uplink_packets = 1;
    packets.len_bytes = 8;  // Too small to hold the packets
    // uint16_t expected_len_bytes = sizeof(CompositeArray::RawPackets::Header) +
    //                               packets.header->num_mode_s_packets * sizeof(RawModeSPacket) +
    //                               packets.header->num_uat_adsb_packets * sizeof(RawUATADSBPacket) +
    //                               packets.header->num_uat_uplink_packets * sizeof(RawUATUplinkPacket);
    EXPECT_FALSE(packets.IsValid(error_msg));
    EXPECT_STREQ(
        error_msg,
        "Invalid CompositeArray::RawPackets: insufficient length for 2 Mode S packet(s), 1 UAT ADSB packet(s), 1 "
        "UAT Uplink packet(s), 0 Remote ID packet(s) (expected 704 bytes, got 8 bytes).");

    // Valid case
    packets.len_bytes = sizeof(CompositeArray::RawPackets::Header) + 2 * sizeof(RawModeSPacket) +
                        1 * sizeof(RawUATADSBPacket) + 1 * sizeof(RawUATUplinkPacket);
    EXPECT_TRUE(packets.IsValid());
}
