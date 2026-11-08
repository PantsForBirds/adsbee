#pragma once

#include "data_structures.hh"
#include "uat_packet.hh"

class UATPacketDecoder {
   public:
    static constexpr uint16_t kRawUATADSBPacketQueueDepth = 10;
    // Every ground uplink slot of one UAT frame: uplinks with errors take longer to decode than a slot lasts.
    static constexpr uint16_t kRawUATUplinkPacketQueueDepth = 32;

    UATPacketDecoder() = default;
    ~UATPacketDecoder() = default;

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

    static inline uint16_t SyncWordLSBAndMDBTypeCodeToMessageLenBytes(uint8_t sync_word_ls4, uint8_t mdb_type_code) {
        // Nearest-pattern sync classification, host tested in common code.
        return UATMessageLenBytesFromSyncAndPayloadType(sync_word_ls4, mdb_type_code);
    }

    /**
     * Decodes every waiting UAT ADS-B packet and at most one uplink, so that a queue of uplinks delays neither the
     * ADS-B packets nor the rest of the main loop.
     */
    bool Update();

   private:
    void DecodeADSBPackets();
    void DecodeUplinkPacket();

    uint32_t num_raw_uat_uplink_packets_dropped_logged_ = 0;
    uint32_t num_uat_adsb_packets_overwritten_ = 0;  // Decoded packets dropped from the full queue for the RP2040.
    uint32_t num_uat_adsb_packets_overwritten_logged_ = 0;
    uint32_t overwritten_log_timestamp_ms_ = 0;

    RawUATADSBPacket raw_uat_adsb_packet_buffer_[kRawUATADSBPacketQueueDepth] = {};
    RawUATUplinkPacket raw_uat_uplink_packet_buffer_[kRawUATUplinkPacketQueueDepth] = {};
};

extern UATPacketDecoder uat_packet_decoder;