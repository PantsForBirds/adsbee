// CdcTx: writes to the USB host wait while the host reads, but never forever when it stops.
#include <stdint.h>

#include <vector>

#include "cdc_tx.hh"
#include "gtest/gtest.h"

// A TX FIFO the host drains by `read_per_wait` bytes each time the caller waits; each wait takes `ms_per_wait`.
struct FakeHost {
    uint32_t fifo_free = 512;
    uint32_t read_per_wait = 64;
    uint32_t ms_per_wait = 1;
    bool connected = true;
    uint32_t now_ms = 1000;
    uint32_t waits = 0;
    uint32_t taken = 0;

    uint32_t Write(const uint8_t*, uint32_t len) {
        uint32_t n = len < fifo_free ? len : fifo_free;
        fifo_free -= n;
        taken += n;
        return n;
    }
    void Wait() {
        waits++;
        now_ms += ms_per_wait;
        fifo_free += read_per_wait;
        if (fifo_free > 512) fifo_free = 512;
    }
    bool Connected() { return connected; }
    uint32_t NowMs() { return now_ms; }
};

static const std::vector<uint8_t> kData(4096, 0x55);

TEST(CdcTx, ReadingHostGetsEverything) {
    FakeHost host;
    CdcTx tx;
    EXPECT_EQ(tx.Write(host, kData.data(), kData.size()), kData.size());
    EXPECT_EQ(host.taken, kData.size());
    EXPECT_FALSE(tx.stalled());
    EXPECT_EQ(tx.TakeDropped(), 0u);
}

TEST(CdcTx, HostThatStopsReadingCostsOneStallTimeoutThenDropsAtOnce) {
    FakeHost host;
    host.fifo_free = 100;
    host.read_per_wait = 0;  // Stopped reader, or a port closed with DTR still high.
    CdcTx tx;
    uint32_t start_ms = host.now_ms;
    EXPECT_EQ(tx.Write(host, kData.data(), kData.size()), 100u);
    EXPECT_TRUE(tx.stalled());
    EXPECT_GE(host.now_ms - start_ms, CdcTx::kStallMs);
    EXPECT_LE(host.now_ms - start_ms, CdcTx::kStallMs + host.ms_per_wait);
    EXPECT_EQ(tx.TakeDropped(), kData.size() - 100);

    // Known stalled: the next write doesn't wait at all.
    uint32_t waits = host.waits;
    EXPECT_EQ(tx.Write(host, kData.data(), 64), 0u);
    EXPECT_EQ(host.waits, waits);
    EXPECT_EQ(tx.TakeDropped(), 64u);
    EXPECT_EQ(tx.TakeDropped(), 0u);
}

TEST(CdcTx, HostThatReadsAgainEndsTheStall) {
    FakeHost host;
    host.fifo_free = 0;
    host.read_per_wait = 0;
    CdcTx tx;
    tx.Write(host, kData.data(), 64);
    ASSERT_TRUE(tx.stalled());
    tx.TakeDropped();

    host.fifo_free = 512;  // The host read the FIFO.
    host.read_per_wait = 64;
    EXPECT_EQ(tx.Write(host, kData.data(), kData.size()), kData.size());
    EXPECT_FALSE(tx.stalled());
    EXPECT_EQ(tx.TakeDropped(), 0u);
}

TEST(CdcTx, SlowHostIsWaitedForAsLongAsItMakesProgress) {
    FakeHost host;
    host.fifo_free = 0;
    host.read_per_wait = 8;
    host.ms_per_wait = CdcTx::kStallMs / 2;  // Progress on every wait, just slowly.
    CdcTx tx;
    EXPECT_EQ(tx.Write(host, kData.data(), 1024), 1024u);
    EXPECT_FALSE(tx.stalled());
    EXPECT_EQ(tx.TakeDropped(), 0u);
}

TEST(CdcTx, ProgressRestartsTheStallTimeout) {
    CdcTx tx;
    // The host reads 64 bytes after 90 ms, then stops: the write may wait up to 90 + kStallMs ms in total.
    struct OnceHost : FakeHost {
        uint32_t wait_count = 0;
        void Wait() {
            FakeHost::Wait();
            if (++wait_count == 9) fifo_free += 64;
        }
    } once;
    once.fifo_free = 0;
    once.read_per_wait = 0;
    once.ms_per_wait = 10;
    uint32_t start_ms = once.now_ms;
    EXPECT_EQ(tx.Write(once, kData.data(), 256), 64u);
    EXPECT_TRUE(tx.stalled());
    EXPECT_GE(once.now_ms - start_ms, 90 + CdcTx::kStallMs);
}

TEST(CdcTx, DisconnectedHostIsNeverWaitedFor) {
    FakeHost host;
    host.fifo_free = 0;
    host.connected = false;  // DTR low (TinyUSB then overwrites instead) or USB not ready.
    CdcTx tx;
    EXPECT_EQ(tx.Write(host, kData.data(), 64), 0u);
    EXPECT_EQ(host.waits, 0u);
    EXPECT_EQ(tx.TakeDropped(), 64u);
}

TEST(CdcTx, TimeoutSurvivesMillisecondCounterWrap) {
    FakeHost host;
    host.now_ms = UINT32_MAX - 20;
    host.fifo_free = 0;
    host.read_per_wait = 0;
    CdcTx tx;
    EXPECT_EQ(tx.Write(host, kData.data(), 64), 0u);
    EXPECT_TRUE(tx.stalled());
    EXPECT_LT(host.waits, CdcTx::kStallMs + 2);  // Ended by the timeout, not by an early or missed comparison.
    EXPECT_GE(host.waits, CdcTx::kStallMs);
}
