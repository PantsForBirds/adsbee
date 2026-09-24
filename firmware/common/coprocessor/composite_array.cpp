#include "composite_array.hh"

#include <cstring>  // For memcpy.

#include "comms.hh"
#include "spi_coprocessor_packet.hh"

// Lock the composite array to fit within a single SPI transaction's data capacity. The RawPackets buffer is the largest
// single object transferred over the coprocessor link; keeping it <= the write packet's data capacity (the tighter of
// write/response, offset 6 B) means it is never chunked, exactly as designed. If kSPITransactionMaxLenBytes is ever
// reduced below what RawPackets needs (or kMaxLenBytes grows past it), this fails the build instead of silently
// truncating raw-packet forwarding.
static_assert(CompositeArray::RawPackets::kMaxLenBytes <= SPICoprocessorPacket::SCWritePacket::kDataMaxLenBytes,
              "CompositeArray::RawPackets must fit in one SPI transaction (see kSPITransactionMaxLenBytes).");

CompositeArray::RawPackets CompositeArray::PackRawPacketsBuffer(uint8_t* buf, uint16_t buf_len_bytes,
                                                                PFBQueue<RawModeSPacket>* mode_s_queue,
                                                                PFBQueue<RawUATADSBPacket>* uat_adsb_queue,
                                                                PFBQueue<RawUATUplinkPacket>* uat_uplink_queue,
                                                                PFBQueue<RawRemoteIDPacket>* remote_id_queue) {
    RawPackets packets_to_report = {
        .len_bytes = sizeof(RawPackets::Header),
        .header = reinterpret_cast<RawPackets::Header*>(buf),
        .mode_s_packets = nullptr,
        .uat_adsb_packets = nullptr,
        .uat_uplink_packets = nullptr,
        .remote_id_packets = nullptr,
    };

    packets_to_report.header->num_mode_s_packets = 0;
    packets_to_report.header->num_uat_adsb_packets = 0;
    packets_to_report.header->num_uat_uplink_packets = 0;
    packets_to_report.header->num_remote_id_packets = 0;

    // Reserve room for a couple of uplink packets up front: Mode S and UAT ADS-B pack first, and under
    // sustained traffic they would otherwise fill every batch and starve the low-rate uplink queue forever.
    // Never reserve more than the buffer could actually hold past the header, so a buffer too small for
    // any uplink still packs Mode S / UAT ADS-B instead of holding space no uplink can use.
    constexpr uint16_t kMaxReservedUplinkPackets = 2;
    uint16_t reserved_uplink_bytes = 0;
    if (uat_uplink_queue) {
        uint16_t max_uplinks_that_fit =
            buf_len_bytes > sizeof(RawPackets::Header)
                ? (buf_len_bytes - sizeof(RawPackets::Header)) / sizeof(RawUATUplinkPacket)
                : 0;
        uint16_t num_uplinks = uat_uplink_queue->Length();
        if (num_uplinks > kMaxReservedUplinkPackets) num_uplinks = kMaxReservedUplinkPackets;
        if (num_uplinks > max_uplinks_that_fit) num_uplinks = max_uplinks_that_fit;
        reserved_uplink_bytes = num_uplinks * sizeof(RawUATUplinkPacket);
    }

    // Fill up the CompositeArray::RawPackets header and associated buffers with as many packets as we can report.
    if (mode_s_queue) {
        // Stuff with Mode S packets.
        RawModeSPacket mode_s_packet;
        packets_to_report.mode_s_packets = reinterpret_cast<RawModeSPacket*>(buf + packets_to_report.len_bytes);
        while (packets_to_report.len_bytes + sizeof(RawModeSPacket) + reserved_uplink_bytes <= buf_len_bytes &&
               mode_s_queue->Dequeue(mode_s_packet)) {
            memcpy(&packets_to_report.mode_s_packets[packets_to_report.header->num_mode_s_packets], &mode_s_packet,
                   sizeof(RawModeSPacket));
            packets_to_report.header->num_mode_s_packets++;
            packets_to_report.len_bytes += sizeof(RawModeSPacket);
        }
    }
    if (uat_adsb_queue) {
        // Stuff with UAT ADS-B packets.
        RawUATADSBPacket uat_adsb_packet;
        packets_to_report.uat_adsb_packets = reinterpret_cast<RawUATADSBPacket*>(buf + packets_to_report.len_bytes);
        while (packets_to_report.len_bytes + sizeof(RawUATADSBPacket) + reserved_uplink_bytes <= buf_len_bytes &&
               uat_adsb_queue->Dequeue(uat_adsb_packet)) {
            memcpy(&packets_to_report.uat_adsb_packets[packets_to_report.header->num_uat_adsb_packets],
                   &uat_adsb_packet, sizeof(RawUATADSBPacket));
            packets_to_report.header->num_uat_adsb_packets++;
            packets_to_report.len_bytes += sizeof(RawUATADSBPacket);
        }
    }
    if (uat_uplink_queue) {
        // Stuff with UAT Uplink packets.
        RawUATUplinkPacket uat_uplink_packet;
        packets_to_report.uat_uplink_packets = reinterpret_cast<RawUATUplinkPacket*>(buf + packets_to_report.len_bytes);
        while (packets_to_report.len_bytes + sizeof(RawUATUplinkPacket) <= buf_len_bytes &&
               uat_uplink_queue->Dequeue(uat_uplink_packet)) {
            memcpy(&packets_to_report.uat_uplink_packets[packets_to_report.header->num_uat_uplink_packets],
                   &uat_uplink_packet, sizeof(RawUATUplinkPacket));
            packets_to_report.header->num_uat_uplink_packets++;
            packets_to_report.len_bytes += sizeof(RawUATUplinkPacket);
        }
    }
    if (remote_id_queue) {
        // Stuff with Remote ID packets.
        RawRemoteIDPacket remote_id_packet;
        packets_to_report.remote_id_packets = reinterpret_cast<RawRemoteIDPacket*>(buf + packets_to_report.len_bytes);
        while (packets_to_report.len_bytes + sizeof(RawRemoteIDPacket) <= buf_len_bytes &&
               remote_id_queue->Dequeue(remote_id_packet)) {
            memcpy(&packets_to_report.remote_id_packets[packets_to_report.header->num_remote_id_packets],
                   &remote_id_packet, sizeof(RawRemoteIDPacket));
            packets_to_report.header->num_remote_id_packets++;
            packets_to_report.len_bytes += sizeof(RawRemoteIDPacket);
        }
    }

    return packets_to_report;
}

bool CompositeArray::UnpackRawPacketsBuffer(CompositeArray::RawPackets& packets, uint8_t* buf, uint16_t buf_len_bytes) {
#ifdef ON_PICO
    // Ensure that buf is word-aligned.
    if (reinterpret_cast<uintptr_t>(buf) % 4 != 0) {
        CONSOLE_ERROR("CompositeArray::UnpackRawPacketsBuffer", "Buffer is not word-aligned.");
        return false;  // Buffer must be word-aligned.
    }
#endif

    if (buf_len_bytes < sizeof(RawPackets::Header)) {
        // Buffer too small to contain header.
        CONSOLE_ERROR("CompositeArray::UnpackRawPacketsBuffer", "Buffer too small to contain header.");
        return false;
    }

    packets.header = reinterpret_cast<RawPackets::Header*>(buf);

    packets.mode_s_packets = reinterpret_cast<RawModeSPacket*>(buf + sizeof(RawPackets::Header));
    packets.uat_adsb_packets =
        reinterpret_cast<RawUATADSBPacket*>(packets.mode_s_packets + packets.header->num_mode_s_packets);
    packets.uat_uplink_packets =
        reinterpret_cast<RawUATUplinkPacket*>(packets.uat_adsb_packets + packets.header->num_uat_adsb_packets);
    packets.remote_id_packets =
        reinterpret_cast<RawRemoteIDPacket*>(packets.uat_uplink_packets + packets.header->num_uat_uplink_packets);

    // Extract the header and make sure buf is big enough to contain the claimed number of packets.
    packets.len_bytes = sizeof(RawPackets::Header) + packets.header->num_mode_s_packets * sizeof(RawModeSPacket) +
                        packets.header->num_uat_adsb_packets * sizeof(RawUATADSBPacket) +
                        packets.header->num_uat_uplink_packets * sizeof(RawUATUplinkPacket) +
                        packets.header->num_remote_id_packets * sizeof(RawRemoteIDPacket);

    if (buf_len_bytes < packets.len_bytes) {
        CONSOLE_ERROR(
            "CompositeArray::UnpackRawPacketsBuffer",
            "Buffer too small for claimed number of packets: expected %u bytes from header, buffer was %u bytes.",
            packets.len_bytes, buf_len_bytes);
        return false;  // Buffer is too short to contain the claimed number of packets.
    }

    return true;
}

bool CompositeArray::UnpackRawPacketsBufferToQueues(uint8_t* buf, uint16_t buf_len_bytes,
                                                    PFBQueue<RawModeSPacket>* mode_s_queue,
                                                    PFBQueue<RawUATADSBPacket>* uat_adsb_queue,
                                                    PFBQueue<RawUATUplinkPacket>* uat_uplink_queue,
                                                    PFBQueue<RawRemoteIDPacket>* remote_id_queue) {
    CompositeArray::RawPackets packets;
    if (!UnpackRawPacketsBuffer(packets, buf, buf_len_bytes)) {
        CONSOLE_ERROR("CompositeArray::UnpackRawPacketsBufferToQueues", "Failed to unpack buffer.");
        return false;
    }

    if (mode_s_queue == nullptr && packets.header->num_mode_s_packets > 0) {
        CONSOLE_ERROR("CompositeArray::UnpackRawPacketsBufferToQueues",
                      "No Mode S queue provided but header claims %d packets.", packets.header->num_mode_s_packets);
        return false;  // No Mode S queue provided but header claims packets.
    }
    if (uat_adsb_queue == nullptr && packets.header->num_uat_adsb_packets > 0) {
        CONSOLE_ERROR("CompositeArray::UnpackRawPacketsBufferToQueues",
                      "No UAT ADS-B queue provided but header claims %d packets.",
                      packets.header->num_uat_adsb_packets);
        return false;  // No UAT ADS-B queue provided but header claims packets.
    }
    if (uat_uplink_queue == nullptr && packets.header->num_uat_uplink_packets > 0) {
        CONSOLE_ERROR("CompositeArray::UnpackRawPacketsBufferToQueues",
                      "No UAT Uplink queue provided but header claims %d packets.",
                      packets.header->num_uat_uplink_packets);
        return false;  // No UAT Uplink queue provided but header claims packets.
    }
    if (remote_id_queue == nullptr && packets.header->num_remote_id_packets > 0) {
        CONSOLE_ERROR("CompositeArray::UnpackRawPacketsBufferToQueues",
                      "No Remote ID queue provided but header claims %d packets.",
                      packets.header->num_remote_id_packets);
        return false;  // No Remote ID queue provided but header claims packets.
    }

    for (uint16_t i = 0; i < packets.header->num_mode_s_packets; i++) {
        if (!mode_s_queue->Enqueue(packets.mode_s_packets[i])) {
            CONSOLE_ERROR("CompositeArray::UnpackRawPacketsBuffer", "Mode S queue full, cannot enqueue packet %d / %d.",
                          i, packets.header->num_mode_s_packets);
            return false;  // Queue full, cannot enqueue packet.
        }
    }

    for (uint16_t i = 0; i < packets.header->num_uat_adsb_packets; i++) {
        if (!uat_adsb_queue->Enqueue(packets.uat_adsb_packets[i])) {
            CONSOLE_ERROR("CompositeArray::UnpackRawPacketsBuffer",
                          "UAT ADS-B queue full, cannot enqueue packet %d / %d.", i,
                          packets.header->num_uat_adsb_packets);
            return false;  // Queue full, cannot enqueue packet.
        }
    }

    for (uint16_t i = 0; i < packets.header->num_uat_uplink_packets; i++) {
        if (!uat_uplink_queue->Enqueue(packets.uat_uplink_packets[i])) {
            CONSOLE_ERROR("CompositeArray::UnpackRawPacketsBuffer",
                          "UAT Uplink queue full, cannot enqueue packet %d / %d.", i,
                          packets.header->num_uat_uplink_packets);
            return false;  // Queue full, cannot enqueue packet.
        }
    }

    for (uint16_t i = 0; i < packets.header->num_remote_id_packets; i++) {
        if (!remote_id_queue->Enqueue(packets.remote_id_packets[i])) {
            CONSOLE_ERROR("CompositeArray::UnpackRawPacketsBuffer",
                          "Remote ID queue full, cannot enqueue packet %d / %d.", i,
                          packets.header->num_remote_id_packets);
            return false;  // Queue full, cannot enqueue packet.
        }
    }

    return true;  // All packets successfully enqueued.
}

bool CompositeArray::MergeRawPacketsBuffers(uint8_t* dst_buf, const uint8_t* src_buf) {
    auto* dst_hdr = reinterpret_cast<RawPackets::Header*>(dst_buf);
    auto* src_hdr = reinterpret_cast<const RawPackets::Header*>(src_buf);

    // dst counts (N/M/K/L) and src counts (P/Q/R/S) for Mode S / UAT ADS-B / UAT Uplink / Remote ID respectively.
    const uint16_t N = dst_hdr->num_mode_s_packets;
    const uint16_t M = dst_hdr->num_uat_adsb_packets;
    const uint16_t K = dst_hdr->num_uat_uplink_packets;
    const uint16_t L = dst_hdr->num_remote_id_packets;
    const uint16_t P = src_hdr->num_mode_s_packets;
    const uint16_t Q = src_hdr->num_uat_adsb_packets;
    const uint16_t R = src_hdr->num_uat_uplink_packets;
    const uint16_t S = src_hdr->num_remote_id_packets;

    const uint32_t ms = sizeof(RawModeSPacket);
    const uint32_t ad = sizeof(RawUATADSBPacket);
    const uint32_t up = sizeof(RawUATUplinkPacket);
    const uint32_t ri = sizeof(RawRemoteIDPacket);

    const uint32_t combined_size = sizeof(RawPackets::Header) + (uint32_t)(N + P) * ms + (uint32_t)(M + Q) * ad +
                                   (uint32_t)(K + R) * up + (uint32_t)(L + S) * ri;
    if (combined_size > RawPackets::kMaxLenBytes) {
        return false;
    }

    // Pointers into src's sections (layout: [Header|ModeS|ADSB|Uplink|RemoteID]).
    const uint8_t* src_ms_ptr = src_buf + sizeof(RawPackets::Header);
    const uint8_t* src_adsb_ptr = src_ms_ptr + P * ms;
    const uint8_t* src_upl_ptr = src_adsb_ptr + Q * ad;
    const uint8_t* src_rid_ptr = src_upl_ptr + R * up;

    // Each step inserts src's section immediately after dst's section of the same type, shifting all later sections
    // forward to make room, so within each type dst's packets stay ahead of src's. Processing front-to-back keeps the
    // yet-to-be-inserted src pointers valid (src_buf is never modified).

    // Step 1: insert src Mode S after dst Mode S; shift dst's ADS-B + Uplink + Remote ID forward by P*ms.
    const uint32_t adsb_start = sizeof(RawPackets::Header) + N * ms;
    const uint32_t tail1 = M * ad + K * up + L * ri;
    if (tail1 > 0) memmove(dst_buf + adsb_start + P * ms, dst_buf + adsb_start, tail1);
    if (P > 0) memcpy(dst_buf + adsb_start, src_ms_ptr, P * ms);
    dst_hdr->num_mode_s_packets = N + P;

    // Step 2: insert src ADS-B after dst ADS-B; shift dst's Uplink + Remote ID forward by Q*ad.
    const uint32_t upl_start = adsb_start + P * ms + M * ad;
    const uint32_t tail2 = K * up + L * ri;
    if (tail2 > 0) memmove(dst_buf + upl_start + Q * ad, dst_buf + upl_start, tail2);
    if (Q > 0) memcpy(dst_buf + upl_start, src_adsb_ptr, Q * ad);
    dst_hdr->num_uat_adsb_packets = M + Q;

    // Step 3: insert src Uplink after dst Uplink; shift dst's Remote ID forward by R*up.
    const uint32_t rid_start = upl_start + Q * ad + K * up;
    const uint32_t tail3 = L * ri;
    if (tail3 > 0) memmove(dst_buf + rid_start + R * up, dst_buf + rid_start, tail3);
    if (R > 0) memcpy(dst_buf + rid_start, src_upl_ptr, R * up);
    dst_hdr->num_uat_uplink_packets = K + R;

    // Step 4: append src Remote ID after dst Remote ID (which now sits at rid_start + R*up, length L*ri).
    if (S > 0) memcpy(dst_buf + rid_start + R * up + L * ri, src_rid_ptr, S * ri);
    dst_hdr->num_remote_id_packets = L + S;

    return true;
}

uint16_t CompositeArray::CalculateRawPacketsBufferLength(PFBQueue<RawModeSPacket>* mode_s_queue,
                                                         PFBQueue<RawUATADSBPacket>* uat_adsb_queue,
                                                         PFBQueue<RawUATUplinkPacket>* uat_uplink_queue,
                                                         PFBQueue<RawRemoteIDPacket>* remote_id_queue) {
    uint16_t total_len_bytes = sizeof(RawPackets::Header);

    if (mode_s_queue) {
        total_len_bytes += mode_s_queue->Length() * sizeof(RawModeSPacket);
    }
    if (uat_adsb_queue) {
        total_len_bytes += uat_adsb_queue->Length() * sizeof(RawUATADSBPacket);
    }
    if (uat_uplink_queue) {
        total_len_bytes += uat_uplink_queue->Length() * sizeof(RawUATUplinkPacket);
    }
    if (remote_id_queue) {
        total_len_bytes += remote_id_queue->Length() * sizeof(RawRemoteIDPacket);
    }

    return total_len_bytes;
}