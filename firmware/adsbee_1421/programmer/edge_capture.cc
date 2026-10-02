#include "edge_capture.hh"

#include "autobaud.hh"
#include "autobaud_edges.pio.h"  // Generated from autobaud_edges.pio by pico_generate_pio_header().
#include "board.hh"
#include "hardware/dma.h"
#include "hardware/pio.h"

static constexpr uint kRingBits = 14;                          // 16 kB.
static constexpr size_t kRingEntries = (1u << kRingBits) / 4;  // 4096 edges: 2.7 ms of "UU" at 3 Mbaud.
// Entries never read: the ones the DMA could overwrite while a copy runs (1024 edges take at least 0.68 ms).
static constexpr size_t kReadMargin = 1024;
static constexpr uint32_t kTransferCount = 0xFFFFFFFFu;  // Per DMA run (48 minutes of square wave at 3 Mbaud).

static const PIO kCapturePio = pio1;

class PioEdgeSource : public EdgeSource {
   public:
    void Init() {
        uint offset = pio_add_program(kCapturePio, &autobaud_edges_program);
        sm_ = pio_claim_unused_sm(kCapturePio, true);
        data_dma_ = dma_claim_unused_channel(true);
        rearm_dma_ = dma_claim_unused_channel(true);

        // The UART keeps the pin; PIO reads the pad through the jmp pin.
        pio_sm_config config = autobaud_edges_program_get_default_config(offset);
        sm_config_set_jmp_pin(&config, kPinUartRx);
        sm_config_set_in_shift(&config, false, true, 32);  // Autopush at every `in x, 32`.
        sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_RX);
        // Edge 0 is falling if the machine starts at `high`. If the pin changes before it runs, its first loop pushes
        // that edge, so the polarity still holds.
        first_falling_ = gpio_get(kPinUartRx);
        uint start = offset + (first_falling_ ? autobaud_edges_offset_high : autobaud_edges_offset_low);
        pio_sm_init(kCapturePio, sm_, start, &config);
        pio_sm_exec(kCapturePio, sm_, pio_encode_mov_not(pio_x, pio_null));  // X = ~0.

        // Data: PIO RX FIFO -> ring, wrapping at the ring's aligned size; chains to the re-arm channel when done.
        dma_channel_config data = dma_channel_get_default_config(data_dma_);
        channel_config_set_transfer_data_size(&data, DMA_SIZE_32);
        channel_config_set_read_increment(&data, false);
        channel_config_set_write_increment(&data, true);
        channel_config_set_ring(&data, true, kRingBits);
        channel_config_set_dreq(&data, pio_get_dreq(kCapturePio, sm_, false));
        channel_config_set_chain_to(&data, rearm_dma_);
        dma_channel_configure(data_dma_, &data, ring_, &kCapturePio->rxf[sm_], kTransferCount, false);

        // Re-arm: writes the count to the data channel's trigger alias. The data channel keeps its write address, so
        // the ring carries on where it was.
        dma_channel_config rearm = dma_channel_get_default_config(rearm_dma_);
        channel_config_set_transfer_data_size(&rearm, DMA_SIZE_32);
        channel_config_set_read_increment(&rearm, false);
        channel_config_set_write_increment(&rearm, false);
        dma_channel_configure(rearm_dma_, &rearm, &dma_hw->ch[data_dma_].al1_transfer_count_trig, &kTransferCount, 1,
                              false);

        dma_channel_start(data_dma_);
        pio_sm_set_enabled(kCapturePio, sm_, true);
    }

    uint64_t Count() override {
        uint32_t remaining = dma_channel_hw_addr(data_dma_)->transfer_count;
        if (remaining > last_remaining_) runs_++;  // Re-armed since the last call.
        last_remaining_ = remaining;
        return runs_ * (uint64_t)kTransferCount + (kTransferCount - remaining);
    }

    size_t Read(uint64_t from, uint64_t to, uint32_t* cycles, size_t max_edges, uint64_t* first_index) override {
        size_t limit = max_edges < kRingEntries - kReadMargin ? max_edges : kRingEntries - kReadMargin;
        uint64_t first = to - from > limit ? to - limit : from;
        *first_index = first;
        size_t n = (size_t)(to - first);
        for (size_t i = 0; i < n; i++) {
            uint64_t index = first + i;
            cycles[i] = Autobaud::CountToCycles(ring_[index % kRingEntries], index);
        }
        return n;
    }

    bool FirstFalling() const override { return first_falling_; }

   private:
    alignas(1u << kRingBits) uint32_t ring_[kRingEntries];
    uint sm_ = 0;
    uint data_dma_ = 0;
    uint rearm_dma_ = 0;
    bool first_falling_ = true;
    uint32_t last_remaining_ = kTransferCount;
    uint64_t runs_ = 0;
};

static PioEdgeSource edge_source;

void EdgeCaptureInit() { edge_source.Init(); }

EdgeSource& EdgeCapture() { return edge_source; }
