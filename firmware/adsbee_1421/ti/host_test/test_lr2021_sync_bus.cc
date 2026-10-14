// Synchronous bus ownership (lr2021_drain_state.hh LR2021SyncBus): no drain or chain frame between a command's
// request and response frames.
#include <vector>

#include "gtest/gtest.h"
#include "lr2021_drain_state.hh"

using S = LR2021DrainState;

TEST(LR2021SyncBus, TakesTheBusOnlyWithNoSequenceInFlight) {
    for (uint8_t i = 0; i <= static_cast<uint8_t>(S::kChainIrqRsp); i++) {
        S s = static_cast<S>(i);
        bool free = (s == S::kIdle || s == S::kDataReady || s == S::kError);
        EXPECT_EQ(LR2021SyncMayTakeBus(s), free) << static_cast<int>(s);
        LR2021SyncBus bus;
        EXPECT_EQ(bus.TryAcquire(s), free) << static_cast<int>(s);
        EXPECT_EQ(bus.owned(), free) << static_cast<int>(s);
    }
}

TEST(LR2021SyncBus, NestedHoldsReleaseOnce) {
    LR2021SyncBus bus;
    ASSERT_TRUE(bus.TryAcquire(S::kIdle));
    // A nested command acquires whatever the drain state reads: the outer hold already excludes the chain.
    ASSERT_TRUE(bus.TryAcquire(S::kChainFifoWait));
    EXPECT_EQ(bus.depth(), 2);
    EXPECT_FALSE(bus.Release());
    EXPECT_TRUE(bus.owned());
    EXPECT_TRUE(bus.Release());
    EXPECT_FALSE(bus.owned());
    EXPECT_FALSE(bus.Release());  // Unbalanced release: no effect.
    EXPECT_EQ(bus.depth(), 0);
}

// Model of the pipelined protocol: a command frame queues its answer, the next frame reads it (and replaces it with
// its own). A synchronous GetVersion is two frames; the IRQ-paced chain is five (ReadRxFifo with its payload, then
// two two-frame clears). An LR_IRQ edge lands before step `edge_at` of the thread's sequence.
namespace {
constexpr int kVersion = 0x0118;
constexpr int kNoAnswer = 0;

struct Model {
    bool arbitrate = true;  // false: the code before bus ownership.
    LR2021SyncBus bus;
    S drain = S::kIdle;
    int queued = kNoAnswer;  // Answer the next frame clocks out.
    bool edge_pending = false;
    int chains = 0;
    int fifo_packets = 9;

    int Frame(int answer) {
        int out = queued;
        queued = answer;
        return out;
    }
    void RunChain() {
        chains++;
        Frame(kNoAnswer);  // ReadRxFifo: payload in the same frame.
        fifo_packets -= 9;
        Frame(kNoAnswer);  // GetAndClearFifoIrqFlags request.
        Frame(kNoAnswer);  // ... response.
        Frame(kNoAnswer);  // GetAndClearIrq request.
        Frame(kNoAnswer);  // ... response.
        drain = S::kIdle;
    }
    // HandleIrqLine with the line high and a free slot.
    void Edge() {
        if (arbitrate && bus.owned()) {
            edge_pending = true;
            return;
        }
        if (drain == S::kIdle) {
            drain = S::kChainFifoWait;
            RunChain();
        }
    }
    int GetVersion(int edge_at) {
        int step = 0;
        auto maybe_edge = [&] {
            if (step++ == edge_at) {
                Edge();
            }
        };
        maybe_edge();
        if (arbitrate) {
            EXPECT_TRUE(bus.TryAcquire(drain));
        }
        maybe_edge();
        Frame(kVersion);  // Request; its Stat answers the frame before.
        maybe_edge();     // BUSY wait between the frames.
        int answer = Frame(kNoAnswer);
        maybe_edge();
        if (arbitrate && bus.Release() && edge_pending) {
            edge_pending = false;
            Edge();
        }
        maybe_edge();
        return answer;
    }
};
}  // namespace

TEST(LR2021SyncBus, EdgeAnywhereInACommandKeepsItsAnswerAndRunsTheChain) {
    for (int edge_at = 0; edge_at < 5; edge_at++) {
        Model m;
        EXPECT_EQ(m.GetVersion(edge_at), kVersion) << edge_at;
        EXPECT_EQ(m.chains, 1) << edge_at;  // Deferred, never dropped: the FIFO is read.
        EXPECT_EQ(m.fifo_packets, 0) << edge_at;
        EXPECT_FALSE(m.bus.owned());
    }
}

TEST(LR2021SyncBus, WithoutOwnershipAChainBetweenTheFramesTakesTheAnswer) {
    std::vector<int> wrong;
    for (int edge_at = 0; edge_at < 5; edge_at++) {
        Model m;
        m.arbitrate = false;
        if (m.GetVersion(edge_at) != kVersion) {
            wrong.push_back(edge_at);
        }
    }
    EXPECT_EQ(wrong, std::vector<int>({2}));  // The BUSY wait between request and response.
}
