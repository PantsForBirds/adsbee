// IRQ-paced LR2021 RX FIFO drain chain.
//
// The LR2021's DIO6 is configured (SetOokADSB) as an IRQ output for kIrqRxFifo, which latches when
// the RX FIFO crosses kIrqDrainThresholdBytes (9 packets). The line lands on the CC1314's LR_IRQ pin
// (rising-edge GPIO interrupt). When the edge fires, this chain empties the FIFO entirely from
// interrupt context -- no main-loop involvement, no busy-waiting:
//
//   LR_IRQ edge (GPIO HWI) ─ claim drain, post ReadRxFifo(2+126) DMA into a staging slot
//   SPI completion (SWI)   ─ mark slot filled, arm BUSY-fall interrupt, raise NSS
//   LR_BUSY falls (GPIO HWI) ─ post the next frame: GetAndClearFifoIrqFlags (2 frames, overflow
//                              check on the readback) -> GetAndClearIrq (2 frames)
//   final SWI              ─ drop to kIdle, or restart into the other slot if the line is still high
//
// The inter-frame BUSY pulses are absorbed by hardware edges instead of CPU spins. Each GPIO handler
// is a few microseconds; that matters because GPIO HWIs (0xC0) preempt the UAT RF SWI's ~15 us
// CMD_PROP_SET_LEN deadline. The main loop parses filled slots later (ADSBee::UpdateLR2021 ->
// NextFilledSlot/ReleaseSlot) and backstops the chain (missed-edge re-post + wedge timeout) in
// ServiceRxDrain.
//
// Why no level read: a fresh edge with the line STILL HIGH means the FIFO crossed the threshold and
// has not since been drained below it, so >= 126 bytes are present, and because every drain path
// pops whole 14-byte packets the leading 126 bytes are exactly 9 whole packets. HandleIrqLine
// enforces the "still high" part with a live line-level check -- the GPIO EVFLAGS latch outlives the
// chip-side line, and GPIO HWIs are the lowest NVIC priority (not 0xC0; ti_drivers_config.c sets
// ~0), so a delayed dispatch can arrive after the loop drain already cleared and drained. Restarts
// are gated on the line being high again AFTER this chain's own clear, which re-establishes the same
// guarantee.
//
// BUSY edge ordering: the BUSY-fall interrupt is armed (clearInt + enableInt) BEFORE NSS is raised,
// and BUSY cannot rise until NSS rises -- so the falling edge is always catchable. If a command
// produces no BUSY pulse at all, the thread backstop in ServiceRxDrain posts the frame within one
// main-loop iteration (irq_thread_assist_posts counts these).
//
// The chain never touches async_tx_buf_/async_rx_buf_ (they may hold an unparsed loop-drain payload)
// and never relies on async_done_ (a stale flag can be written by a preempted loop-drain SWI that a
// steal interrupted); chain progress is driven purely by SPICallback dispatch on drain_state_.

#include <ti/drivers/dpl/HwiP.h>

#include "hal.hh"
#include "lr2021.hh"

// Minimal Stat validation for ISR context: command-status field (bits 11:9) must be CMD_OK / CMD_DAT.
// last_stat_ is thread-owned and must not be written from here.
static inline bool ChainStatOk(const uint8_t* rx) {
    uint8_t cs = (static_cast<uint16_t>((rx[0] << 8) | rx[1]) >> 9) & 0x7;
    return cs == LR2021::CommandStatus::kOk || cs == LR2021::CommandStatus::kDat;
}

void LR2021::ArmBusyInterrupt() {
    // Stale latched edges accumulate from every polled transaction: always clear before enabling.
    GPIO_clearInt(config_.gpio_busy);
    GPIO_enableInt(config_.gpio_busy);
}

void LR2021::DisarmBusyInterrupt() { GPIO_disableInt(config_.gpio_busy); }

void LR2021::AbortChainFromIsr() {
    DisarmBusyInterrupt();
    drain_state_ = DrainState::kIdle;
    SetNSS(true);  // Harmless if tri-stated.
}

bool LR2021::PostChainFrame() {
    // Caller guarantees atomicity (GPIO HWI, or HwiP_disable from SWI/thread) and a ready chip.
    bool posted = false;
    DrainState next = DrainState::kIdle;
    switch (drain_state_) {
        case DrainState::kChainFifoWait:
            posted = PostAsyncFrame(chain_fifo_tx_buf_, 2u + kIrqDrainThresholdBytes,
                                    irq_slots_[irq_slot_fill_idx_].buf);
            next = DrainState::kChainFifoRead;
            break;
        case DrainState::kChainClearFifoWait:
            posted = PostAsyncFrame(chain_clear_tx_buf_, 2, chain_rx_scratch_);
            next = DrainState::kChainClearFifo;
            break;
        case DrainState::kChainClearFifoRspWait:
            posted = PostAsyncFrame(nullptr, 4, chain_rx_scratch_);  // 2 (stat) + rx/tx flags.
            next = DrainState::kChainClearFifoRsp;
            break;
        case DrainState::kChainIrqCmdWait:
            posted = PostAsyncFrame(chain_irq_tx_buf_, 2, chain_rx_scratch_);
            next = DrainState::kChainIrqCmd;
            break;
        case DrainState::kChainIrqRspWait:
            posted = PostAsyncFrame(nullptr, 6, chain_rx_scratch_);  // 2 (stat) + 4 (irq_flags).
            next = DrainState::kChainIrqRsp;
            break;
        default:
            return false;  // Not a chain wait state; nothing to post.
    }
    if (!posted) {
        irq_chain_errors = irq_chain_errors + 1;
        AbortChainFromIsr();
        return false;
    }
    drain_state_ = next;
    return true;
}

void LR2021::StartChainSegment() {
    // Caller has claimed drain_state_ (any value is about to be overwritten) and stamped the slot
    // timestamp. Runs in GPIO HWI or SPI SWI.
    chain_start_ms_ = get_time_since_boot_ms();
    drain_state_ = DrainState::kChainFifoWait;
    if (!IsBusy()) {
        PostChainFrame();
    } else {
        // Chip mid-BUSY pulse (e.g. tail of a stolen command): the falling edge posts the frame.
        ArmBusyInterrupt();
    }
}

void LR2021::HandleIrqLine() {
    // GPIO HWI (0xC0): budget a few microseconds. Thread-side drain transitions are HwiP-guarded and
    // the SPI SWI cannot preempt an HWI, so drain_state_ reads here are stable snapshots -- with one
    // benign exception: this HWI can land mid-way through a loop-drain SWI completion, but the steal
    // precondition (!async_in_flight_) is only true after that SWI has already raised NSS.
    if (abort_requested_ || spi_handle_ == nullptr) {
        return;
    }
    if (!IrqLineHigh()) {
        // Stale latched edge: the GPIO module's EVFLAGS latch outlives the chip-side line. GPIO HWIs
        // sit at the LOWEST NVIC priority (ti_drivers_config.c GPIO intPriority = ~0), so RF-core /
        // SPI HWIs (0xa0) can delay this dispatch long enough for the loop drain to run its clear
        // tail (dropping the line) and drain the FIFO below the threshold. Starting a chain here
        // would blind-read 126 bytes from a FIFO that may hold fewer, popping the read pointer past
        // the write pointer and tearing the stream.
        irq_stale_edges = irq_stale_edges + 1;
        return;
    }
    IrqDrainSlot& slot = irq_slots_[irq_slot_fill_idx_];
    if (slot.filled) {
        irq_drain_skip_slots = irq_drain_skip_slots + 1;
        return;
    }
    DrainState s = drain_state_;
    if (IsChainState(s)) {
        return;  // Chain already running; it re-checks the line at completion.
    }
    if (s != DrainState::kIdle) {
        // Loop drain active. Steal it only when parked with nothing on the bus; never steal terminal
        // states (kDataReady holds an unparsed payload, kError an unreported failure), and never
        // steal kClearFifoRspWait: GetAndClearFifoIrqFlags frame 1 already destructively cleared the
        // FIFO sub-flags, and the pending frame 2 holds their only copy -- stealing here destroys
        // the overflow/tear evidence the resync depends on.
        if (async_in_flight_ || s == DrainState::kDataReady || s == DrainState::kError ||
            s == DrainState::kClearFifoRspWait) {
            irq_drain_skip_busy = irq_drain_skip_busy + 1;
            return;
        }
        // Stolen: the discarded drain is restarted fresh by the next UpdateLR2021 pass. If the steal
        // dropped a pending read-response, the chip discards it when the next command frame arrives
        // (pipelined protocol).
    }
    irq_drain_count = irq_drain_count + 1;
    slot.timestamp_us = get_time_since_boot_us();
    StartChainSegment();
}

void LR2021::HandleBusyFall() {
    // GPIO HWI (0xC0): fires only while armed, i.e. a chain frame is pending. Post it.
    DisarmBusyInterrupt();
    if (abort_requested_) {
        AbortChainFromIsr();
        return;
    }
    switch (drain_state_) {
        case DrainState::kChainFifoWait:
        case DrainState::kChainClearFifoWait:
        case DrainState::kChainClearFifoRspWait:
        case DrainState::kChainIrqCmdWait:
        case DrainState::kChainIrqRspWait:
            PostChainFrame();
            break;
        default:
            break;  // Disarmed late / state already advanced (thread assist): nothing to do.
    }
}

void LR2021::HandleChainFrameComplete() {
    // SPI completion SWI, chain in-flight states only. async_ok_ was set by the caller (SPICallback).
    // Budget: a few microseconds -- shares the non-preemptive SWI band with the UAT RF deadline.
    if (abort_requested_) {
        AbortChainFromIsr();
        return;
    }
    DrainState next_wait;
    switch (drain_state_) {
        case DrainState::kChainFifoRead: {
            IrqDrainSlot& slot = irq_slots_[irq_slot_fill_idx_];
            if (!async_ok_ || !ChainStatOk(slot.buf)) {
                irq_chain_errors = irq_chain_errors + 1;
                AbortChainFromIsr();
                return;
            }
            slot.len = kIrqDrainThresholdBytes;
            slot.filled = true;  // Written last; the thread reads filled before len/buf.
            irq_slot_fill_idx_ = (irq_slot_fill_idx_ + 1) % kNumIrqDrainSlots;
            next_wait = DrainState::kChainClearFifoWait;
            break;
        }
        case DrainState::kChainClearFifo:
            // Stat in an opcode frame reflects the previous command (the FIFO read): already validated.
            next_wait = DrainState::kChainClearFifoRspWait;
            break;
        case DrainState::kChainClearFifoRsp:
            // GetAndClearFifoIrqFlags readback: the only place the FIFO overflow sub-flag is
            // observable on the chain path. Single-byte inspection, within the SWI budget.
            if (!async_ok_ || !ChainStatOk(chain_rx_scratch_)) {
                irq_chain_errors = irq_chain_errors + 1;
                AbortChainFromIsr();
                return;
            }
            if (chain_rx_scratch_[2] & kFifoIrqFlagsResyncMask) {
                // Full/Overflow/Underflow may have torn the stream. The chain never reads the FIFO
                // level, so the Full sub-flag is its only fullness signal (2 x 126 == 252 < 256:
                // chain reads alone strand the last partial packet of a full FIFO). Flag for the
                // thread-level flush/realign in ADSBee::UpdateLR2021.
                fifo_overflow_count = fifo_overflow_count + 1;
                fifo_overflow_pending = true;
            }
            next_wait = DrainState::kChainIrqCmdWait;
            break;
        case DrainState::kChainIrqCmd:
            next_wait = DrainState::kChainIrqRspWait;
            break;
        case DrainState::kChainIrqRsp: {
            // Final frame: the latched IRQ register (incl. kIrqRxFifo) is now clear, so the line
            // drops unless the FIFO has already re-crossed the threshold.
            if (!async_ok_ || !ChainStatOk(chain_rx_scratch_)) {
                irq_chain_errors = irq_chain_errors + 1;
                AbortChainFromIsr();
                return;
            }
            SetNSS(true);
            IrqDrainSlot& next_slot = irq_slots_[irq_slot_fill_idx_];
            if (IrqLineHigh() && !next_slot.filled && !abort_requested_) {
                // FIFO re-crossed the threshold during this chain (line re-latched after our clear):
                // go straight into another read. Same >= threshold guarantee as a fresh edge.
                irq_drain_restarts = irq_drain_restarts + 1;
                next_slot.timestamp_us = get_time_since_boot_us();
                StartChainSegment();
            } else {
                drain_state_ = DrainState::kIdle;
            }
            return;
        }
        default:
            return;  // Cancelled/raced: nothing to do.
    }
    if (!async_ok_) {
        irq_chain_errors = irq_chain_errors + 1;
        AbortChainFromIsr();
        return;
    }
    // Advance to the next frame: arm the BUSY falling-edge interrupt BEFORE raising NSS (BUSY cannot
    // rise until NSS does, so the edge is always catchable), then end the frame.
    drain_state_ = next_wait;
    busy_wait_start_ms_ = get_time_since_boot_ms();
    ArmBusyInterrupt();
    SetNSS(true);
}

LR2021::IrqDrainSlot* LR2021::NextFilledSlot() {
    IrqDrainSlot& slot = irq_slots_[irq_slot_parse_idx_];
    return slot.filled ? &slot : nullptr;
}

void LR2021::ReleaseSlot() {
    irq_slots_[irq_slot_parse_idx_].filled = false;
    irq_slot_parse_idx_ = (irq_slot_parse_idx_ + 1) % kNumIrqDrainSlots;
}
