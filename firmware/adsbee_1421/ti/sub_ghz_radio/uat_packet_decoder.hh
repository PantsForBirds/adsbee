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
    static constexpr uint16_t kRawUATUplinkPacketQueueDepth = 4;
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
                                      .overwrite_when_full = true});

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
        if (sync_word_ls4 == RawUATADSBPacket::kSyncWordLS4) {
            return mdb_type_code == 0 ? RawUATADSBPacket::kShortADSBMessageNumBytes
                                      : RawUATADSBPacket::kLongADSBMessageNumBytes;
        } else {
            return RawUATUplinkPacket::kUplinkMessageNumBytes;
        }
    }
    bool Update();

   private:
    RawUATADSBPacket raw_uat_adsb_packet_buffer_[kRawUATADSBPacketQueueDepth] = {};
    RawUATUplinkPacket raw_uat_uplink_packet_buffer_[kRawUATUplinkPacketQueueDepth] = {};
    DecodedUATADSBPacket decoded_uat_adsb_packet_buffer_[kDecodedUATADSBPacketQueueDepth] = {};
    DecodedUATUplinkPacket decoded_uat_uplink_packet_buffer_[kDecodedUATUplinkPacketQueueDepth] = {};
};

extern UATPacketDecoder uat_packet_decoder;