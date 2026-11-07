#pragma once

#include "data_structures.hh"
#include "uat_packet.hh"

class UATPacketDecoder {
   public:
    // Raw queues are filled from the RF-core ISR and drained once per main-loop pass, so they are
    // sized to absorb a burst of back-to-back receptions when a loop iteration runs long. The decoded
    // queues sit between two main-loop stages (Update() -> IngestAndForwardPackets()) and never
    // accumulate more than one pass worth of packets, so they stay shallow.
    static constexpr uint16_t kRawUATADSBPacketQueueDepth = 32;
    // Every ground uplink slot of one UAT frame: uplinks with errors take longer to decode than a slot lasts.
    static constexpr uint16_t kRawUATUplinkPacketQueueDepth = 32;
    static constexpr uint16_t kDecodedUATADSBPacketQueueDepth = 10;
    static constexpr uint16_t kDecodedUATUplinkPacketQueueDepth = 2;

    UATPacketDecoder() = default;
    ~UATPacketDecoder() = default;

    // Input queues — filled by SubGHzRadio from the RF interrupt callback.
    PFBQueue<RawUATADSBPacket> raw_uat_adsb_packet_queue =
        PFBQueue<RawUATADSBPacket>({.buf_len_num_elements = kRawUATADSBPacketQueueDepth,
                                    .buffer = raw_uat_adsb_packet_buffer_,
                                    .overwrite_when_full = true});

    PFBQueue<RawUATUplinkPacket> raw_uat_uplink_packet_queue =
        PFBQueue<RawUATUplinkPacket>({.buf_len_num_elements = kRawUATUplinkPacketQueueDepth,
                                      .buffer = raw_uat_uplink_packet_buffer_,
                                      .overwrite_when_full = false});  // The RF callback counts refused packets.

    // Uplinks refused by raw_uat_uplink_packet_queue. Counted in the RF callback, logged by Update().
    volatile uint32_t num_raw_uat_uplink_packets_dropped = 0;

    // Output queues — drained by ADSBee::IngestAndForwardPackets().
    PFBQueue<DecodedUATADSBPacket> decoded_uat_adsb_packet_out_queue =
        PFBQueue<DecodedUATADSBPacket>({.buf_len_num_elements = kDecodedUATADSBPacketQueueDepth,
                                        .buffer = decoded_uat_adsb_packet_buffer_,
                                        .overwrite_when_full = true});

    PFBQueue<DecodedUATUplinkPacket> decoded_uat_uplink_packet_out_queue =
        PFBQueue<DecodedUATUplinkPacket>({.buf_len_num_elements = kDecodedUATUplinkPacketQueueDepth,
                                          .buffer = decoded_uat_uplink_packet_buffer_,
                                          .overwrite_when_full = true});

    static inline uint16_t SyncWordLSBAndMDBTypeCodeToMessageLenBytes(uint8_t sync_word_ls4, uint8_t mdb_type_code) {
        // Nearest-pattern sync classification, host tested in common code.
        return UATMessageLenBytesFromSyncAndPayloadType(sync_word_ls4, mdb_type_code);
    }

    /**
     * Decodes every waiting UAT ADS-B (or ADS-L) packet and at most one uplink, so that a queue of uplinks delays neither the
     * ADS-B packets nor the rest of the main loop.
     */
    bool Update();

   private:
    void DecodeADSBPackets();
    // ADS-L packets share the ADS-B queues (RawUATADSBPacket::protocol): erasure repair and CRC instead of FEC.
    void DecodeADSLPacket(const RawUATADSBPacket& packet);
    void DecodeUplinkPacket();

    uint32_t num_raw_uat_uplink_packets_dropped_logged_ = 0;
    RawUATADSBPacket raw_uat_adsb_packet_buffer_[kRawUATADSBPacketQueueDepth] = {};
    RawUATUplinkPacket raw_uat_uplink_packet_buffer_[kRawUATUplinkPacketQueueDepth] = {};
    DecodedUATADSBPacket decoded_uat_adsb_packet_buffer_[kDecodedUATADSBPacketQueueDepth] = {};
    DecodedUATUplinkPacket decoded_uat_uplink_packet_buffer_[kDecodedUATUplinkPacketQueueDepth] = {};
};

extern UATPacketDecoder uat_packet_decoder;