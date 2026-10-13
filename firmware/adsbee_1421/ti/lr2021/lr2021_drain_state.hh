#pragma once

// LR2021 RX drain states, and when the IRQ-paced chain may take the bus from the loop drain.
// Header-only and SDK-free for the host tests.

#include <cstdint>

enum class LR2021DrainState : uint8_t {
    kIdle = 0,
    // MLAT prototype (t-0394): optional frames of the loop drain ahead of the level read.
    kSyncClearWait,  // Waiting for BUSY low to post ClearIrq(SyncwordValid).
    kSyncClear,      // ClearIrq frame (6 B) DMA in flight.
    kTsCmdWait,      // Waiting for BUSY low to post the GetTimestampValue opcode frame.
    kTsCmd,          // GetTimestampValue frame 1 (3 B) DMA in flight.
    kTsRspWait,      // Waiting for BUSY low to post the GetTimestampValue response frame.
    kTsRsp,          // GetTimestampValue frame 2 (6 B) DMA in flight.
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

// Whether the chain may discard a loop drain in state s (not kIdle, not a chain state) and take the bus.
enum class LR2021DrainSteal : uint8_t {
    kAllowed = 0,
    // A frame is on the bus, the state is terminal (kError holds an unreported failure), or kClearFifoRspWait:
    // GetAndClearFifoIrqFlags frame 1 cleared the FIFO sub-flags and the pending frame 2 holds their only copy, the
    // overflow evidence the resync depends on.
    kBusy,
    kUnparsedRead,  // Parked between frames with a read not parsed yet.
};
constexpr LR2021DrainSteal LR2021ChainStealCheck(LR2021DrainState s, bool frame_in_flight, uint16_t read_len_bytes) {
    if (frame_in_flight || s == LR2021DrainState::kDataReady || s == LR2021DrainState::kError ||
        s == LR2021DrainState::kClearFifoRspWait) {
        return LR2021DrainSteal::kBusy;
    }
    return LR2021LoopDrainHoldsRead(s, read_len_bytes) ? LR2021DrainSteal::kUnparsedRead : LR2021DrainSteal::kAllowed;
}
