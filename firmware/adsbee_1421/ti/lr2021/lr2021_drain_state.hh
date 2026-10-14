#pragma once

// LR2021 RX drain states, and when the IRQ-paced chain may take the bus from the loop drain.
// Header-only and SDK-free for the host tests.

#include <cstdint>

enum class LR2021DrainState : uint8_t {
    kIdle = 0,
    // Thread-paced loop drain (advanced by ServiceRxDrain): unconditional level-read sweep, then a
    // conditional IRQ-latch clear tail whenever the LR_IRQ line reads high.
    kLevelCmdWait,   // Waiting for BUSY low to post the GetRxFifoLevel opcode frame.
    kLevelCmd,       // GetRxFifoLevel frame 1 (2 B) DMA in flight.
    kLevelRspWait,   // Waiting for BUSY low to post the GetRxFifoLevel response frame.
    kLevelRsp,       // GetRxFifoLevel frame 2 (4 B) DMA in flight.
    kFifoWait,       // Waiting for BUSY low to post the ReadRxFifo frame.
    kFifoRead,       // ReadRxFifo single frame (2 + level B) DMA in flight.
    kClearFifoWait,     // (Tail) waiting for BUSY low to post the GetAndClearFifoIrqFlags opcode frame.
    kClearFifoCmd,      // (Tail) GetAndClearFifoIrqFlags frame 1 (2 B) DMA in flight.
    kClearFifoRspWait,  // (Tail) waiting for BUSY low to post the GetAndClearFifoIrqFlags response frame.
    kClearFifoRsp,      // (Tail) GetAndClearFifoIrqFlags frame 2 (4 B) DMA in flight; overflow check.
    kIrqCmdWait,        // (Tail) waiting for BUSY low to post the GetAndClearIrq opcode frame.
    kIrqCmd,            // (Tail) GetAndClearIrq frame 1 (2 B) DMA in flight.
    kIrqRspWait,        // (Tail) waiting for BUSY low to post the GetAndClearIrq response frame.
    kIrqRsp,            // (Tail) GetAndClearIrq frame 2 (6 B) DMA in flight.
    kDataReady,         // Terminal until FinishRxDrain().
    kError,             // Terminal until FinishRxDrain().
    // IRQ-paced chain (advanced from the GPIO HWIs and the SPI completion SWI; the thread only
    // backstops). Must stay the LAST block: LR2021DrainIsChainState() tests >= kChainFifoWait.
    kChainFifoWait,          // Waiting (BUSY edge / thread assist) to post ReadRxFifo(threshold).
    kChainFifoRead,          // ReadRxFifo (2 + 126 B) DMA into a staging slot in flight.
    kChainClearFifoWait,     // Waiting to post the GetAndClearFifoIrqFlags opcode frame.
    kChainClearFifo,         // GetAndClearFifoIrqFlags frame 1 (2 B) in flight.
    kChainClearFifoRspWait,  // Waiting to post the GetAndClearFifoIrqFlags response frame.
    kChainClearFifoRsp,      // GetAndClearFifoIrqFlags frame 2 (4 B) in flight; overflow check.
    kChainIrqCmdWait,        // Waiting to post GetAndClearIrq opcode frame.
    kChainIrqCmd,            // GetAndClearIrq frame 1 (2 B) in flight.
    kChainIrqRspWait,        // Waiting to post GetAndClearIrq response frame.
    kChainIrqRsp,            // GetAndClearIrq frame 2 (6 B) in flight; completion decides restart/finish.
};

constexpr bool LR2021DrainIsChainState(LR2021DrainState s) { return s >= LR2021DrainState::kChainFifoWait; }

// The loop drain has popped read_len_bytes from the FIFO and the main loop has not parsed them yet. They exist only
// in the async buffer from the ReadRxFifo frame until FinishRxDrain(), through the whole clear tail.
constexpr bool LR2021LoopDrainHoldsRead(LR2021DrainState s, uint16_t read_len_bytes) {
    return read_len_bytes > 0 && s >= LR2021DrainState::kFifoRead && s <= LR2021DrainState::kDataReady;
}

// Whether the chain may take the bus from a loop drain in state s (not kIdle, not a chain state).
enum class LR2021DrainSteal : uint8_t {
    kAllowed = 0,
    // A frame is on the bus, the state is terminal (kError holds an unreported failure), or kClearFifoRspWait:
    // GetAndClearFifoIrqFlags frame 1 cleared the FIFO sub-flags and the pending frame 2 holds their only copy, the
    // overflow evidence the resync depends on.
    kBusy,
    // Allowed, parked between frames with a read not parsed yet: the read stays in the async buffer for the main
    // loop and no loop drain may start until it is parsed.
    kKeepRead,
};
constexpr LR2021DrainSteal LR2021ChainStealCheck(LR2021DrainState s, bool frame_in_flight, uint16_t read_len_bytes) {
    if (frame_in_flight || s == LR2021DrainState::kDataReady || s == LR2021DrainState::kError ||
        s == LR2021DrainState::kClearFifoRspWait) {
        return LR2021DrainSteal::kBusy;
    }
    return LR2021LoopDrainHoldsRead(s, read_len_bytes) ? LR2021DrainSteal::kKeepRead : LR2021DrainSteal::kAllowed;
}

// Whether the chain takes over a loop drain whose frame in state s just completed, to run the clear tail: after a
// good ReadRxFifo frame with the LR_IRQ line high.
constexpr bool LR2021ChainTakesLoopTail(LR2021DrainState s, bool frame_ok, bool irq_line_high) {
    return s == LR2021DrainState::kFifoRead && frame_ok && irq_line_high;
}

// Whether an LR_IRQ edge skipped as kBusy in state s is retried when the frame on the bus completes (kDataReady: when
// the main loop is done parsing). Only where nothing was popped from the FIFO since the edge, so the chain's read of
// a full threshold still holds: not after a ReadRxFifo frame.
constexpr bool LR2021ChainRetriesSkippedEdge(LR2021DrainState s) {
    return s == LR2021DrainState::kLevelCmd || s == LR2021DrainState::kLevelRsp || s == LR2021DrainState::kIrqRsp ||
           s == LR2021DrainState::kDataReady;
}

// Whether a synchronous command may take the bus with the drain in state s: no drain frame or chain on the bus and
// no loop drain mid-sequence (only idle or a terminal state).
constexpr bool LR2021SyncMayTakeBus(LR2021DrainState s) {
    return s == LR2021DrainState::kIdle || s == LR2021DrainState::kDataReady || s == LR2021DrainState::kError;
}

// Synchronous command ownership of the LR2021 bus, from the request frame through the BUSY wait to the response
// frame. Holds nest (a command inside a held sequence). While owned, an LR_IRQ edge defers the chain until Release().
class LR2021SyncBus {
   public:
    // Thread level. The outermost acquire runs with interrupts off. False: a drain or chain is mid-sequence; advance
    // it and retry.
    bool TryAcquire(LR2021DrainState s) {
        if (depth_ > 0) {
            depth_++;
            return true;
        }
        if (!LR2021SyncMayTakeBus(s)) {
            return false;
        }
        depth_ = 1;
        owned_ = true;
        return true;
    }
    // Thread level, interrupts off for the outermost release. True when the outermost hold ended: retry an edge
    // that was deferred.
    bool Release() {
        if (depth_ == 0 || --depth_ > 0) {
            return false;
        }
        owned_ = false;
        return true;
    }
    // Read by the GPIO HWIs.
    bool owned() const { return owned_; }
    uint8_t depth() const { return depth_; }

   private:
    volatile bool owned_ = false;
    uint8_t depth_ = 0;  // Thread only.
};
