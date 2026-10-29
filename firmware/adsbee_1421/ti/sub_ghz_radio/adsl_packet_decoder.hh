#pragma once

#include "adsl_packet.hh"
#include "data_structures.hh"

/**
 * Decodes ADS-L chips captured by the RF callback (Manchester, erasure repair, CRC) in the main loop. Valid packets go
 * to adsl_packet_out_queue for ADSBee::IngestAndForwardPackets().
 */
class ADSLPacketDecoder {
   public:
    static constexpr uint16_t kChipsQueueDepth = 8;
    static constexpr uint16_t kADSLPacketOutQueueDepth = 8;

    struct Chips {
        uint8_t chips[RawADSLPacket::kMaxChipBytes];
        uint8_t num_chip_bytes = 0;
        uint8_t channel = RawADSLPacket::kChannelUnknown;
        int8_t rssi_dbm = 0;
        uint32_t timestamp = 0;
    };

    /**
     * Queues chips from the RF callback. Overwrites the oldest entry when full.
     */
    void EnqueueChips(const uint8_t *chips, uint16_t num_chip_bytes, int8_t rssi_dbm, uint32_t timestamp,
                      uint8_t channel);

    /**
     * Decodes every queued reception.
     */
    bool Update();

    PFBQueue<RawADSLPacket> adsl_packet_out_queue = PFBQueue<RawADSLPacket>({.buf_len_num_elements = kADSLPacketOutQueueDepth,
                                                                            .buffer = adsl_packet_out_buffer_,
                                                                            .overwrite_when_full = true});

    volatile uint32_t num_raw_packets = 0;  // Receptions after an ADS-L sync word, since boot. Counted in the ISR.
    uint32_t num_valid_packets = 0;         // Receptions that passed the CRC, since boot.

   private:
    uint32_t num_raw_packets_recorded_ = 0;
    PFBQueue<Chips> chips_queue_ = PFBQueue<Chips>(
        {.buf_len_num_elements = kChipsQueueDepth, .buffer = chips_queue_buffer_, .overwrite_when_full = true});
    Chips chips_queue_buffer_[kChipsQueueDepth] = {};
    RawADSLPacket adsl_packet_out_buffer_[kADSLPacketOutQueueDepth] = {};
};

extern ADSLPacketDecoder adsl_packet_decoder;
