#pragma once

#include "data_structures.hh"
#include "mode_s_packet.hh"

class PacketDecoder {
   public:
    static constexpr uint16_t kRawModeSPacketQueueDepth = 100;
    static constexpr uint16_t kDecodedModeSPacketQueueDepth = 100;

    PacketDecoder();

    bool Update();

    PFBQueue<RawModeSPacket> raw_mode_s_packet_queue;

    PFBQueue<DecodedModeSPacket> decoded_mode_s_packet_out_queue;

    // Health / diagnostics counters, reported and reset via AT+RX_STATS.
    uint32_t raw_queue_overflow_count = 0;  // Raw packets overwritten because raw_mode_s_packet_queue was full.
    uint32_t bitflips_fixed_count = 0;      // Extended squitter frames recovered via single-bit CRC correction.

   private:
    RawModeSPacket raw_mode_s_packet_queue_buffer_[kRawModeSPacketQueueDepth];
    DecodedModeSPacket decoded_mode_s_packet_out_queue_buffer_[kDecodedModeSPacketQueueDepth];
};

extern PacketDecoder packet_decoder;
