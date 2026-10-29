#pragma once

#include "adsl_packet.hh"
#include "data_structures.hh"

/**
 * Decodes ADS-L chips captured by the RF callback (Manchester, erasure repair, CRC) in the main loop and queues valid
 * packets for the RP2040.
 */
class ADSLPacketDecoder {
   public:
    static constexpr uint16_t kChipsQueueDepth = 8;

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

   private:
    PFBQueue<Chips> chips_queue_ = PFBQueue<Chips>(
        {.buf_len_num_elements = kChipsQueueDepth, .buffer = chips_queue_buffer_, .overwrite_when_full = true});
    Chips chips_queue_buffer_[kChipsQueueDepth] = {};
};

extern ADSLPacketDecoder adsl_packet_decoder;
