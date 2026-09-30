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
    uint32_t decode_max_cycles = 0;         // Longest decode of one raw packet, in CPU cycles (correction included).

   private:
    // Decodes one raw packet (with single-bit correction) into decoded_mode_s_packet_out_queue.
    void DecodeOne(RawModeSPacket& raw_packet, bool df17_mode);

    RawModeSPacket raw_mode_s_packet_queue_buffer_[kRawModeSPacketQueueDepth];
    DecodedModeSPacket decoded_mode_s_packet_out_queue_buffer_[kDecodedModeSPacketQueueDepth];
};

extern PacketDecoder packet_decoder;
