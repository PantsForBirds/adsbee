#pragma once

#include "data_structures.hh"
#include "uat_packet.hh"

class UATPacketDecoder {
   public:
    static constexpr uint16_t kRawUATADSBPacketQueueDepth = 10;
    static constexpr uint16_t kRawUATUplinkPacketQueueDepth = 2;

    UATPacketDecoder() = default;
    ~UATPacketDecoder() = default;

    PFBQueue<RawUATADSBPacket> raw_uat_adsb_packet_queue =
        PFBQueue<RawUATADSBPacket>({.buf_len_num_elements = kRawUATADSBPacketQueueDepth,
                                    .buffer = raw_uat_adsb_packet_buffer_,
                                    .overwrite_when_full = true});

    PFBQueue<RawUATUplinkPacket> raw_uat_uplink_packet_queue =
        PFBQueue<RawUATUplinkPacket>({.buf_len_num_elements = kRawUATUplinkPacketQueueDepth,
                                      .buffer = raw_uat_uplink_packet_buffer_,
                                      .overwrite_when_full = true});

    static inline uint16_t SyncWordLSBAndMDBTypeCodeToMessageLenBytes(uint8_t sync_word_ls4, uint8_t mdb_type_code) {
        // Nearest-pattern sync classification, host tested in common code.
        return UATMessageLenBytesFromSyncAndPayloadType(sync_word_ls4, mdb_type_code);
    }
    bool Update();

   private:
    RawUATADSBPacket raw_uat_adsb_packet_buffer_[kRawUATADSBPacketQueueDepth] = {};
    RawUATUplinkPacket raw_uat_uplink_packet_buffer_[kRawUATUplinkPacketQueueDepth] = {};
};

extern UATPacketDecoder uat_packet_decoder;