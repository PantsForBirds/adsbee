// Async (DMA/callback-driven) RX drain for the LR2021 -- thread-paced loop drain.
//
// Replaces the blocking steady-state poll sequence with a state machine whose long SPI clocking
// happens via uDMA in the background:
//
//   - Thread level (ServiceRxDrain, called once per main-loop iteration) does all BUSY waiting,
//     protocol parsing, and state advancement for the LOOP drain. While a frame's DMA is in flight or
//     the chip is processing a command (BUSY high), the main loop is free to run the UAT re-arm,
//     decoders, and comms instead of spinning.
//   - SWI level (SPICallback) does the minimum: raise NSS to end the frame (so the LR2021's BUSY
//     pulse overlaps main-loop time) and set the completion flags. NoRTOS SWIs are non-preemptive,
//     and the UAT RF SWI must issue CMD_PROP_SET_LEN within ~15 us of sync detect (see
//     sub_ghz_radio.cpp) -- nothing heavier may run in the SWI band.
//
// The loop drain is an UNCONDITIONAL level-read sweep: GetRxFifoLevel -> ReadRxFifo -> (only when the
// LR_IRQ line reads high) GetAndClearFifoIrqFlags + GetAndClearIrq. It deliberately does not gate on the
// kIrqRxFifo flag: with the FIFO high threshold at kIrqDrainThresholdBytes (9 packets), sub-threshold
// packets would otherwise never be read. The clear tail keeps the latched IRQ line (and its edge
// generation for the IRQ-paced chain, lr2021_irq_drain.cpp) healthy whenever it is seen high.
//
// Torn-packet guard: every FIFO read pops a multiple of kOokFifoPacketLenBytes so the FIFO front
// stays packet-aligned, without exception (256 % 14 != 0, so a "full flush" would itself misalign
// the stream). When a tear must be assumed (FIFO read back full, or any of the Full/Overflow/
// Underflow sub-flags observed in a drain tail), fifo_overflow_pending schedules the thread-level
// ClearRxFifo flush in ADSBee::UpdateLR2021 -- the one true realignment mechanism.
//
// Concurrency: the IRQ-paced chain (lr2021_irq_drain.cpp) may STEAL a parked loop drain from a GPIO
// HWI whenever no DMA is in flight and the drain is not in a terminal state. Every thread-side
// drain_state_ transition is therefore HwiP_disable-guarded and re-checks the snapshotted state; a
// stolen drain is simply discarded and restarted fresh by the next UpdateLR2021() pass.

#include <ti/drivers/dpl/HwiP.h>

#include "hal.hh"
#include "lr2021.hh"

void LR2021::SPICallback(SPI_Handle handle, SPI_Transaction* transaction) {
    (void)handle;
    // SWI context: keep tiny. No logging, no busy-waiting, no heavy parsing.
    LR2021* self = static_cast<LR2021*>(transaction->arg);
    if (self == nullptr) {
        return;
    }
    if (transaction == &self->async_txn_) {
        if (IsChainState(self->drain_state_)) {
            // IRQ-paced chain frame: completion handling (slot marking, next-frame arm/post, restart
            // decision) lives in lr2021_irq_drain.cpp. It raises NSS itself with arm-before-NSS
            // ordering.
            self->async_ok_ = (transaction->status == SPI_TRANSFER_COMPLETED);
            self->async_in_flight_ = false;
            self->HandleChainFrameComplete();
        } else {
            // Loop-drain frame: end the NSS frame immediately so the chip starts processing (BUSY
            // pulse) during main-loop time instead of waiting for the next ServiceRxDrain() call.
            self->SetNSS(true);
            self->async_ok_ = (transaction->status == SPI_TRANSFER_COMPLETED);
            self->async_in_flight_ = false;
            self->async_done_ = true;  // Written last: thread level reads async_done_ before async_ok_.
        }
    } else {
        // Synchronous shim transfer (SPITransfer in lr2021_ll.cpp): the caller owns NSS framing at
        // thread level; only report completion.
        self->sync_status_ = transaction->status;
        self->sync_done_ = true;
    }
}

bool LR2021::PostAsyncFrame(const uint8_t* tx_buf, size_t len, uint8_t* rx_buf) {
    if (spi_handle_ == nullptr) {
        return false;
    }
    async_done_ = false;
    async_ok_ = false;
    async_txn_ = {
        .count = len,
        .txBuf = const_cast<uint8_t*>(tx_buf),  // nullptr clocks the driver's 0x00 default.
        .rxBuf = (rx_buf != nullptr) ? rx_buf : async_rx_buf_,
        .arg = this,
        .status = SPI_TRANSFER_QUEUED,
    };
    SetNSS(false);
    async_in_flight_ = true;
    if (!SPI_transfer(spi_handle_, &async_txn_)) {
        async_in_flight_ = false;
        SetNSS(true);
        return false;
    }
    return true;
}

void LR2021::EnterBusyWait(DrainState state) {
    busy_wait_start_ms_ = get_time_since_boot_ms();
    drain_state_ = state;
}

LR2021::DrainResult LR2021::EnterDrainError(const char* reason) {
    drain_error_str_ = reason;
    drain_state_ = DrainState::kError;
    return DrainResult::kError;
}

LR2021::DrainResult LR2021::AsyncStillInFlight() {
    // busy_wait_start_ms_ was stamped on entering the preceding wait state, so the deadline spans the
    // BUSY wait plus the DMA flight -- generous against a worst-case ~0.2 ms 258-byte frame.
    if (get_time_since_boot_ms() - busy_wait_start_ms_ <= kBusyTimeoutMs) {
        return DrainResult::kInProgress;
    }
    // Lost SPI completion: without this the drain returns kInProgress forever and the IRQ chain
    // refuses to steal (async_in_flight_ stays true) -- a silent, permanent 1090 wedge. Publish the
    // terminal state BEFORE cancelling: SPI_transferCancel clears async_in_flight_ via the callback,
    // and until a terminal (never-stolen) state is visible an LR_IRQ edge could steal what it sees as
    // a parked wait state. HwiP-guarded re-check first: a completion SWI (or a completion followed by
    // a chain steal) landing between the caller's async_done_ read and here must not be clobbered
    // with kError.
    uintptr_t key = HwiP_disable();
    if (async_done_ || IsChainState(drain_state_)) {
        HwiP_restore(key);
        return DrainResult::kInProgress;  // Raced: completion (and possibly a steal) arrived.
    }
    drain_dma_timeouts = drain_dma_timeouts + 1;
    drain_error_str_ = "DMA completion timeout";
    drain_state_ = DrainState::kError;
    HwiP_restore(key);
    if (spi_handle_ != nullptr) {
        SPI_transferCancel(spi_handle_);
    }
    SetNSS(true);
    return DrainResult::kError;
}

bool LR2021::IrqLineHigh() { return GPIO_read(config_.gpio_irq) != 0; }

bool LR2021::StartRxDrain() {
    if (spi_handle_ == nullptr || abort_requested_) {
        return false;
    }
    uintptr_t key = HwiP_disable();
    if (drain_state_ != DrainState::kIdle) {
        HwiP_restore(key);
        return false;
    }
    EnterBusyWait(DrainState::kLevelCmdWait);
    HwiP_restore(key);
    // Field resets after the claim: if the chain steals the drain before they matter, they are
    // re-initialized by the next StartRxDrain anyway.
    drain_irq_flags_ = 0;
    drain_fifo_level_ = 0;
    drain_fifo_full_ = false;
    drain_error_str_ = nullptr;
    return true;
}

LR2021::DrainResult LR2021::ServiceRxDrain() {
    while (true) {
        DrainState s = drain_state_;  // Snapshot: the chain may steal/advance concurrently.

        // IRQ-paced chain: the thread only backstops -- re-post a frame whose BUSY edge was missed,
        // and cancel a wedged chain.
        if (IsChainState(s)) {
            if (s == DrainState::kChainFifoWait || s == DrainState::kChainClearFifoWait ||
                s == DrainState::kChainClearFifoRspWait || s == DrainState::kChainIrqCmdWait ||
                s == DrainState::kChainIrqRspWait) {
                if (!IsBusy() && !abort_requested_) {
                    uintptr_t key = HwiP_disable();
                    if (drain_state_ == s) {
                        DisarmBusyInterrupt();
                        if (PostChainFrame()) {
                            irq_thread_assist_posts = irq_thread_assist_posts + 1;
                        }
                    }
                    HwiP_restore(key);
                    return DrainResult::kInProgress;
                }
            }
            if (abort_requested_ || (get_time_since_boot_ms() - chain_start_ms_ > kIrqChainTimeoutMs)) {
                // Wedged (or aborted) chain: cancel at thread level.
                uintptr_t key = HwiP_disable();
                bool claimed = (drain_state_ == s);
                if (claimed) {
                    DisarmBusyInterrupt();
                    drain_state_ = DrainState::kIdle;
                    if (!abort_requested_) {
                        irq_chain_timeouts = irq_chain_timeouts + 1;
                    }
                }
                HwiP_restore(key);
                if (claimed) {
                    if (async_in_flight_ && spi_handle_ != nullptr) {
                        SPI_transferCancel(spi_handle_);  // Canceled callback lands in the legacy branch.
                    }
                    SetNSS(true);
                }
                return DrainResult::kIdle;
            }
            return DrainResult::kInProgress;
        }

        if (abort_requested_ && s != DrainState::kIdle && s != DrainState::kDataReady && s != DrainState::kError) {
            // The SYNC ISR handed the bus to the host mid-drain: unwind silently (expected, not an
            // error, matching WaitUntilReady's abort semantics).
            CancelAsync();
            return DrainResult::kIdle;
        }

        switch (s) {
            case DrainState::kIdle:
                return DrainResult::kIdle;
            case DrainState::kDataReady:
                return DrainResult::kDataReady;
            case DrainState::kError:
                return DrainResult::kError;

            // BUSY-wait states: post the next frame once the chip is ready. Post + transition are
            // HwiP-guarded so a chain steal can't interleave.
            case DrainState::kLevelCmdWait:
            case DrainState::kLevelRspWait:
            case DrainState::kFifoWait:
            case DrainState::kClearFifoWait:
            case DrainState::kClearFifoRspWait:
            case DrainState::kIrqCmdWait:
            case DrainState::kIrqRspWait: {
                if (IsBusy()) {
                    if (get_time_since_boot_ms() - busy_wait_start_ms_ > kBusyTimeoutMs) {
                        return EnterDrainError("BUSY timeout");
                    }
                    return DrainResult::kInProgress;
                }
                uintptr_t key = HwiP_disable();
                if (drain_state_ != s) {  // Stolen by the chain: re-evaluate.
                    HwiP_restore(key);
                    break;
                }
                bool posted = false;
                DrainState next = s;
                switch (s) {
                    case DrainState::kLevelCmdWait:
                        PackU16(async_tx_buf_, kOpcodeGetRxFifoLevel);
                        posted = PostAsyncFrame(async_tx_buf_, 2);
                        next = DrainState::kLevelCmd;
                        break;
                    case DrainState::kLevelRspWait:
                        posted = PostAsyncFrame(nullptr, 4);  // 2 (stat) + 2 (level).
                        next = DrainState::kLevelRsp;
                        break;
                    case DrainState::kFifoWait:
                        // Single NSS frame: opcode at [0:1], then [2..] clocks zeros while the payload
                        // streams back (async_tx_buf_[2..] is kept zeroed).
                        PackU16(async_tx_buf_, kOpcodeReadRxFifo);
                        posted = PostAsyncFrame(async_tx_buf_, 2u + drain_fifo_level_);
                        next = DrainState::kFifoRead;
                        break;
                    case DrainState::kClearFifoWait:
                        // Read-and-clear the (sticky) FIFO sub-flags BEFORE the global latch, so
                        // stale sub-flags can't instantly re-latch kIrqRxFifo. The readback (frame 2)
                        // is the only place the FIFO overflow sub-flag is observable.
                        PackU16(tail_tx_buf_, kOpcodeGetAndClearFifoIrqFlags);
                        posted = PostAsyncFrame(tail_tx_buf_, 2, tail_rx_buf_);
                        next = DrainState::kClearFifoCmd;
                        break;
                    case DrainState::kClearFifoRspWait:
                        posted = PostAsyncFrame(nullptr, 4, tail_rx_buf_);  // 2 (stat) + rx/tx flags.
                        next = DrainState::kClearFifoRsp;
                        break;
                    case DrainState::kIrqCmdWait:
                        PackU16(tail_tx_buf_, kOpcodeGetAndClearIrq);
                        posted = PostAsyncFrame(tail_tx_buf_, 2, tail_rx_buf_);
                        next = DrainState::kIrqCmd;
                        break;
                    default:  // kIrqRspWait
                        posted = PostAsyncFrame(nullptr, 6, tail_rx_buf_);  // 2 (stat) + 4 (irq_flags).
                        next = DrainState::kIrqRsp;
                        break;
                }
                if (posted) {
                    drain_state_ = next;
                }
                HwiP_restore(key);
                if (!posted) {
                    return EnterDrainError("SPI_transfer post failed");
                }
                return DrainResult::kInProgress;
            }

            // DMA-in-flight states: parse and advance on completion. Transitions are HwiP-guarded
            // (once async_in_flight_ drops, the chain may steal this drain).
            case DrainState::kLevelCmd: {
                if (!async_done_) {
                    return AsyncStillInFlight();
                }
                if (!async_ok_) {
                    return EnterDrainError("GetRxFifoLevel opcode frame failed");
                }
                ParseStat(UnpackU16(async_rx_buf_));
                uintptr_t key = HwiP_disable();
                if (drain_state_ == s) {
                    EnterBusyWait(DrainState::kLevelRspWait);
                }
                HwiP_restore(key);
                break;  // Loop: BUSY may already be low, or we were stolen.
            }
            case DrainState::kLevelRsp: {
                if (!async_done_) {
                    return AsyncStillInFlight();
                }
                if (!async_ok_) {
                    return EnterDrainError("GetRxFifoLevel response frame failed");
                }
                ParseStat(UnpackU16(async_rx_buf_));
                if (last_stat_.command_status != CommandStatus::kDat &&
                    last_stat_.command_status != CommandStatus::kOk) {
                    return EnterDrainError("GetRxFifoLevel bad command status");
                }
                uint16_t level = UnpackU16(async_rx_buf_ + 2);
                // Full FIFO means the drain fell behind and the radio almost certainly dropped frames
                // on top of this; the consumer counts it (see ADSBee::UpdateLR2021).
                bool full = (level >= kRxFifoMaxDepthBytes);
                uint16_t read_len = (level > kRxFifoMaxDepthBytes) ? kRxFifoMaxDepthBytes : level;
                // Torn-packet guard: keep the FIFO front packet-aligned by only popping whole
                // packets -- WITH NO FULL-FIFO EXCEPTION. Popping all 256 bytes would itself
                // misalign the stream: 256 % 14 == 4, and the parser's discarded remainder is the
                // head of a packet whose tail is still in the FIFO. Realignment after a tear is
                // solely the thread-level ClearRxFifo resync, scheduled below.
                read_len -= read_len % kOokFifoPacketLenBytes;
                if (full) {
                    // A full FIFO means a tear must be assumed (writes raced a hard ceiling even if
                    // the overflow sub-flag hasn't latched yet): schedule the flush/realign.
                    fifo_overflow_pending = true;
                }
                uintptr_t key = HwiP_disable();
                if (drain_state_ == s) {
                    drain_fifo_full_ = full;
                    drain_fifo_level_ = read_len;
                    if (read_len > 0) {
                        EnterBusyWait(DrainState::kFifoWait);
                    } else if (IrqLineHigh()) {
                        // Nothing (whole) to read but the latched IRQ line is asserted: run the clear
                        // tail so edge generation recovers, then finish as kNoData.
                        EnterBusyWait(DrainState::kClearFifoWait);
                    } else {
                        drain_state_ = DrainState::kIdle;
                        HwiP_restore(key);
                        return DrainResult::kNoData;
                    }
                }
                HwiP_restore(key);
                break;
            }
            case DrainState::kFifoRead: {
                if (!async_done_) {
                    return AsyncStillInFlight();
                }
                if (!async_ok_) {
                    return EnterDrainError("ReadRxFifo frame failed");
                }
                ParseStat(UnpackU16(async_rx_buf_));
                if (last_stat_.command_status != CommandStatus::kDat &&
                    last_stat_.command_status != CommandStatus::kOk) {
                    return EnterDrainError("ReadRxFifo bad command status");
                }
                uintptr_t key = HwiP_disable();
                if (drain_state_ == s) {
                    if (IrqLineHigh()) {
                        // Payload is parked in async_rx_buf_ while the tail frames run into the
                        // dedicated tail buffers; kDataReady is entered at tail completion.
                        EnterBusyWait(DrainState::kClearFifoWait);
                    } else {
                        drain_state_ = DrainState::kDataReady;
                        HwiP_restore(key);
                        return DrainResult::kDataReady;
                    }
                }
                HwiP_restore(key);
                break;
            }
            case DrainState::kClearFifoCmd: {
                if (!async_done_) {
                    return AsyncStillInFlight();
                }
                if (!async_ok_) {
                    return EnterDrainError("GetAndClearFifoIrqFlags opcode frame failed");
                }
                // Stat in an opcode frame reflects the PREVIOUS command; informational only here.
                ParseStat(UnpackU16(tail_rx_buf_));
                uintptr_t key = HwiP_disable();
                if (drain_state_ == s) {
                    EnterBusyWait(DrainState::kClearFifoRspWait);
                }
                HwiP_restore(key);
                break;
            }
            case DrainState::kClearFifoRsp: {
                if (!async_done_) {
                    return AsyncStillInFlight();
                }
                if (!async_ok_) {
                    return EnterDrainError("GetAndClearFifoIrqFlags response frame failed");
                }
                ParseStat(UnpackU16(tail_rx_buf_));
                if (last_stat_.command_status != CommandStatus::kDat &&
                    last_stat_.command_status != CommandStatus::kOk) {
                    return EnterDrainError("GetAndClearFifoIrqFlags bad command status");
                }
                if (tail_rx_buf_[2] & kFifoIrqFlagsResyncMask) {
                    // Full/Overflow/Underflow may have torn the stream, leaving every subsequent
                    // 14-byte slice misaligned: flag for the thread-level FIFO flush/realign in
                    // ADSBee::UpdateLR2021.
                    fifo_overflow_count = fifo_overflow_count + 1;
                    fifo_overflow_pending = true;
                }
                uintptr_t key = HwiP_disable();
                if (drain_state_ == s) {
                    EnterBusyWait(DrainState::kIrqCmdWait);
                }
                HwiP_restore(key);
                break;
            }
            case DrainState::kIrqCmd: {
                if (!async_done_) {
                    return AsyncStillInFlight();
                }
                if (!async_ok_) {
                    return EnterDrainError("GetAndClearIrq opcode frame failed");
                }
                ParseStat(UnpackU16(tail_rx_buf_));
                uintptr_t key = HwiP_disable();
                if (drain_state_ == s) {
                    EnterBusyWait(DrainState::kIrqRspWait);
                }
                HwiP_restore(key);
                break;
            }
            case DrainState::kIrqRsp: {
                if (!async_done_) {
                    return AsyncStillInFlight();
                }
                if (!async_ok_) {
                    return EnterDrainError("GetAndClearIrq response frame failed");
                }
                ParseStat(UnpackU16(tail_rx_buf_));
                if (last_stat_.command_status != CommandStatus::kDat &&
                    last_stat_.command_status != CommandStatus::kOk) {
                    return EnterDrainError("GetAndClearIrq bad command status");
                }
                drain_irq_flags_ = UnpackU32(tail_rx_buf_ + 2);
                uintptr_t key = HwiP_disable();
                if (drain_state_ == s) {
                    if (drain_fifo_level_ > 0) {
                        drain_state_ = DrainState::kDataReady;
                        HwiP_restore(key);
                        return DrainResult::kDataReady;
                    }
                    drain_state_ = DrainState::kIdle;
                    HwiP_restore(key);
                    return DrainResult::kNoData;
                }
                HwiP_restore(key);
                break;
            }
            default:
                // Unreachable: chain states are handled above.
                return DrainResult::kInProgress;
        }
    }
}

void LR2021::FinishRxDrain() {
    if (drain_state_ == DrainState::kDataReady || drain_state_ == DrainState::kError) {
        drain_state_ = DrainState::kIdle;
    }
}

void LR2021::CancelAsync() {
    // Thread level only (SPI_transferCancel must not be called from an ISR here). If a DMA is in
    // flight, cancel it -- SPI_transferCancel invokes SPICallback with a canceled status, which clears
    // async_in_flight_. A completion racing the check is harmless: cancelling with nothing queued is a
    // no-op. The critical section keeps the flag reset atomic against a late-landing callback.
    DisarmBusyInterrupt();
    if (spi_handle_ != nullptr && async_in_flight_) {
        SPI_transferCancel(spi_handle_);
    }
    uintptr_t key = HwiP_disable();
    async_done_ = false;
    async_ok_ = false;
    drain_state_ = DrainState::kIdle;
    HwiP_restore(key);
    SetNSS(true);  // Harmless DOUT update if the pin was tri-stated by the SYNC ISR.
}
