#include <cstdlib>
#include <string>

#include "feed_send_buffer.hh"
#include "gtest/gtest.h"

// Stream socket stand-in: takes up to `room` bytes per call and records what it took.
struct FakeFeedSocket {
    std::string stream;
    int room = 0;
    bool broken = false;
    int num_writes = 0;

    static int Write(void* ctx, const uint8_t* buf, uint16_t buf_len) {
        FakeFeedSocket* sock = static_cast<FakeFeedSocket*>(ctx);
        sock->num_writes++;
        if (sock->broken) {
            return -1;
        }
        int num_taken = buf_len < sock->room ? buf_len : sock->room;
        sock->stream.append(reinterpret_cast<const char*>(buf), num_taken);
        sock->room -= num_taken;
        return num_taken;
    }
};

static int num_allocs = 0;
static void* CountingAlloc(size_t num_bytes) {
    num_allocs++;
    return malloc(num_bytes);
}
static void* FailingAlloc(size_t num_bytes) { return nullptr; }
static int num_deallocs = 0;
static void CountingDealloc(void* ptr) {
    num_deallocs++;
    free(ptr);
}

static FeedSendBuffer::Config MakeConfig(FakeFeedSocket& sock, uint16_t backlog_capacity_bytes) {
    FeedSendBuffer::Config config;
    config.write = FakeFeedSocket::Write;
    config.ctx = &sock;
    config.backlog_capacity_bytes = backlog_capacity_bytes;
    config.alloc = CountingAlloc;
    config.dealloc = CountingDealloc;
    return config;
}

static FeedSendBuffer::Result Write(FeedSendBuffer& buf, const char* str, uint32_t timestamp_ms) {
    return buf.Write(reinterpret_cast<const uint8_t*>(str), strlen(str), timestamp_ms);
}

TEST(FeedSendBuffer, HealthySocketHoldsNoMemory) {
    FakeFeedSocket sock;
    sock.room = 1000;
    FeedSendBuffer buf;
    num_allocs = num_deallocs = 0;
    buf.Open(MakeConfig(sock, 16), 0);
    EXPECT_EQ(Write(buf, "aaaa", 1), FeedSendBuffer::kSent);
    EXPECT_EQ(Write(buf, "bbbb", 2), FeedSendBuffer::kSent);
    EXPECT_EQ(sock.stream, "aaaabbbb");
    EXPECT_EQ(num_allocs, 2);  // A reserve per batch, freed once the socket took it all.
    EXPECT_EQ(num_deallocs, 2);
    EXPECT_EQ(buf.BacklogLenBytes(), 0);
    EXPECT_EQ(buf.StalledMs(1000), 0u);
}

TEST(FeedSendBuffer, CutBatchIsFinishedBeforeNewerBatches) {
    FakeFeedSocket sock;
    sock.room = 3;
    FeedSendBuffer buf;
    buf.Open(MakeConfig(sock, 16), 0);
    EXPECT_EQ(Write(buf, "aaaaa", 1), FeedSendBuffer::kQueued);  // "aaa" sent, "aa" waits.
    EXPECT_EQ(buf.BacklogLenBytes(), 2);
    EXPECT_EQ(Write(buf, "bbbb", 2), FeedSendBuffer::kQueued);  // Socket still full: joins the backlog.
    EXPECT_EQ(sock.stream, "aaa");
    sock.room = 4;
    EXPECT_TRUE(buf.Flush(3));
    EXPECT_EQ(sock.stream, "aaaaabb");
    sock.room = 100;
    EXPECT_EQ(Write(buf, "cc", 4), FeedSendBuffer::kSent);  // Backlog goes out first, then the new batch.
    EXPECT_EQ(sock.stream, "aaaaabbbbcc");
    EXPECT_EQ(buf.BacklogLenBytes(), 0);
}

TEST(FeedSendBuffer, BacklogIsHeldOnlyWhileBehind) {
    FakeFeedSocket sock;
    FeedSendBuffer buf;
    num_allocs = num_deallocs = 0;
    buf.Open(MakeConfig(sock, 16), 0);
    EXPECT_EQ(Write(buf, "aaaa", 1), FeedSendBuffer::kQueued);
    EXPECT_EQ(Write(buf, "bbbb", 2), FeedSendBuffer::kQueued);
    EXPECT_EQ(num_allocs - num_deallocs, 1);  // The backlog; the first batch's reserve is already freed.
    sock.room = 5;
    EXPECT_TRUE(buf.Flush(3));
    EXPECT_EQ(num_allocs - num_deallocs, 1);  // Still three bytes behind.
    sock.room = 100;
    EXPECT_TRUE(buf.Flush(4));
    EXPECT_EQ(num_allocs - num_deallocs, 0);  // Caught up: the memory goes back to the heap.
    EXPECT_EQ(Write(buf, "cccc", 5), FeedSendBuffer::kSent);
    sock.room = 0;
    EXPECT_EQ(Write(buf, "dddd", 6), FeedSendBuffer::kQueued);
    EXPECT_EQ(num_allocs - num_deallocs, 1);
    buf.Close();
    EXPECT_EQ(num_allocs - num_deallocs, 0);
    EXPECT_EQ(sock.stream, "aaaabbbbcccc");
}

TEST(FeedSendBuffer, FullBacklogDropsWholeBatchesOnly) {
    FakeFeedSocket sock;
    FeedSendBuffer buf;
    buf.Open(MakeConfig(sock, 8), 0);
    EXPECT_EQ(Write(buf, "aaaa", 1), FeedSendBuffer::kQueued);
    EXPECT_EQ(Write(buf, "bbbb", 2), FeedSendBuffer::kQueued);
    EXPECT_FALSE(buf.HasRoomFor(1));
    EXPECT_EQ(Write(buf, "cccc", 3), FeedSendBuffer::kDropped);
    EXPECT_EQ(Write(buf, "dd", 4), FeedSendBuffer::kDropped);
    sock.room = 2;
    EXPECT_TRUE(buf.Flush(4));
    EXPECT_TRUE(buf.HasRoomFor(2));
    EXPECT_FALSE(buf.HasRoomFor(3));
    sock.room = 0;
    sock.room = 100;
    EXPECT_EQ(Write(buf, "ee", 5), FeedSendBuffer::kSent);
    EXPECT_EQ(sock.stream, "aaaabbbbee");  // Every batch is either whole in the stream or absent.
    EXPECT_TRUE(buf.HasRoomFor(1000));     // With no backlog the socket is tried first.
}

TEST(FeedSendBuffer, BatchLargerThanBacklogIsDroppedOrFinishedFromItsReserve) {
    FakeFeedSocket sock;
    FeedSendBuffer buf;
    buf.Open(MakeConfig(sock, 4), 0);
    EXPECT_EQ(Write(buf, "aaaaaaaa", 1), FeedSendBuffer::kDropped);
    EXPECT_EQ(sock.stream, "");
    // Part of the batch went out and the backlog is too small for the rest: the reserve holds it.
    sock.room = 2;
    EXPECT_EQ(Write(buf, "bbbbbbbb", 2), FeedSendBuffer::kQueued);
    EXPECT_EQ(buf.BacklogLenBytes(), 6);
    sock.room = 100;
    EXPECT_TRUE(buf.Flush(3));
    EXPECT_EQ(sock.stream, "bbbbbbbb");
}

TEST(FeedSendBuffer, NoMemoryDropsWholeBatchesWithoutTouchingTheSocket) {
    FakeFeedSocket sock;
    FeedSendBuffer buf;
    FeedSendBuffer::Config config = MakeConfig(sock, 16);
    config.alloc = FailingAlloc;
    buf.Open(config, 0);
    EXPECT_EQ(Write(buf, "aaaa", 1), FeedSendBuffer::kDropped);
    sock.room = 1;
    EXPECT_EQ(Write(buf, "bbbb", 2), FeedSendBuffer::kDropped);
    EXPECT_EQ(sock.num_writes, 0);
    EXPECT_EQ(sock.stream, "");
}

static int num_tail_allocs = 0;
static size_t last_tail_alloc_bytes = 0;
static void* CountingTailAlloc(size_t num_bytes) {
    num_tail_allocs++;
    last_tail_alloc_bytes = num_bytes;
    return malloc(num_bytes);
}

TEST(FeedSendBuffer, CutTailIsKeptWhenThereIsNoBacklogMemory) {
    FakeFeedSocket sock;
    FeedSendBuffer buf;
    FeedSendBuffer::Config config = MakeConfig(sock, 16);
    config.alloc = FailingAlloc;
    config.alloc_tail = CountingTailAlloc;
    num_tail_allocs = 0;
    buf.Open(config, 0);
    sock.room = 1;
    // The socket takes 1 byte: the other 3 stay in the batch's reserve instead of losing the connection.
    EXPECT_EQ(Write(buf, "bbbb", 1), FeedSendBuffer::kQueued);
    EXPECT_EQ(num_tail_allocs, 1);
    EXPECT_EQ(last_tail_alloc_bytes, 4u);  // Reserved for the whole batch before the socket saw it.
    EXPECT_EQ(buf.BacklogLenBytes(), 3);
    // No room for newer batches until the tail is out: they are dropped whole.
    EXPECT_FALSE(buf.HasRoomFor(1));
    EXPECT_EQ(Write(buf, "cccc", 2), FeedSendBuffer::kDropped);
    sock.room = 100;
    EXPECT_TRUE(buf.Flush(3));
    EXPECT_EQ(buf.BacklogLenBytes(), 0);
    EXPECT_EQ(Write(buf, "dddd", 4), FeedSendBuffer::kSent);
    EXPECT_EQ(sock.stream, "bbbbdddd");
}

TEST(FeedSendBuffer, ReserveIsFreedUnlessItHoldsACutTail) {
    FakeFeedSocket sock;
    FeedSendBuffer buf;
    FeedSendBuffer::Config config = MakeConfig(sock, 16);
    config.alloc = FailingAlloc;
    config.alloc_tail = CountingTailAlloc;
    num_tail_allocs = num_deallocs = 0;
    buf.Open(config, 0);
    // Nothing went out: the batch is dropped whole and its reserve freed.
    EXPECT_EQ(Write(buf, "aaaa", 1), FeedSendBuffer::kDropped);
    EXPECT_EQ(num_tail_allocs, 1);
    EXPECT_EQ(num_deallocs, 1);
    // With a backlog available, a cut tail goes there and the reserve is freed.
    config.alloc = CountingAlloc;
    buf.Open(config, 2);
    sock.room = 1;
    EXPECT_EQ(Write(buf, "bbbb", 3), FeedSendBuffer::kQueued);
    EXPECT_EQ(num_deallocs, 2);
    EXPECT_TRUE(buf.HasRoomFor(12));
}

TEST(FeedSendBuffer, NoMemoryAtAllDropsInsteadOfCutting) {
    FakeFeedSocket sock;
    FeedSendBuffer buf;
    FeedSendBuffer::Config config = MakeConfig(sock, 16);
    config.alloc = FailingAlloc;
    config.alloc_tail = FailingAlloc;
    buf.Open(config, 0);
    sock.room = 1;
    EXPECT_EQ(Write(buf, "bbbb", 1), FeedSendBuffer::kDropped);
    EXPECT_EQ(sock.num_writes, 0);
}

// Allocations that fail at random, as when other tasks take the heap between batches.
static uint32_t rng = 1;
static uint32_t NextRandom() { return rng = rng * 1103515245u + 12345u; }
static void* FlakyAlloc(size_t num_bytes) { return (NextRandom() >> 16) % 3 == 0 ? nullptr : CountingAlloc(num_bytes); }

TEST(FeedSendBuffer, LowMemoryNeverCutsTheStream) {
    FakeFeedSocket sock;
    FeedSendBuffer buf;
    FeedSendBuffer::Config config = MakeConfig(sock, 24);
    config.alloc = FlakyAlloc;
    config.alloc_tail = FlakyAlloc;
    num_allocs = num_deallocs = 0;
    buf.Open(config, 0);
        for (int i = 0; i < 2000; i++) {
        // Batch i is "<len><letter...>|": the stream can be split back into batches and checked.
        std::string batch = std::string(1, 'a' + i % 26) + std::string(1 + (NextRandom() >> 16) % 12, 'a' + i % 26) + "|";
        sock.room = (NextRandom() >> 16) % 20;
        FeedSendBuffer::Result r = Write(buf, batch.c_str(), i);
        ASSERT_NE(r, FeedSendBuffer::kError);
    }
    sock.room = 100000;
    ASSERT_TRUE(buf.Flush(3000));
    // Every '|'-terminated piece is one batch: all one letter.
    size_t start = 0, num_batches = 0;
    for (size_t end = sock.stream.find('|'); end != std::string::npos; end = sock.stream.find('|', start)) {
        std::string piece = sock.stream.substr(start, end - start);
        ASSERT_GE(piece.size(), 2u);
        ASSERT_EQ(piece.find_first_not_of(piece[0]), std::string::npos) << "cut batch at " << start;
        start = end + 1;
        num_batches++;
    }
    EXPECT_EQ(start, sock.stream.size());
    EXPECT_GT(num_batches, 100u);
    buf.Close();
    EXPECT_EQ(num_allocs, num_deallocs);
}

TEST(FeedSendBuffer, FullBacklogComesBackAfterATailBuffer) {
    FakeFeedSocket sock;
    FeedSendBuffer buf;
    FeedSendBuffer::Config config = MakeConfig(sock, 16);
    config.alloc = FailingAlloc;
    config.alloc_tail = CountingTailAlloc;
    buf.Open(config, 0);
    sock.room = 2;
    EXPECT_EQ(Write(buf, "bbbb", 1), FeedSendBuffer::kQueued);
    sock.room = 2;
    EXPECT_TRUE(buf.Flush(2));
    EXPECT_EQ(buf.BacklogLenBytes(), 0);
    // Freed when sent: the next cut gets the full backlog again once alloc() has memory.
    EXPECT_TRUE(buf.HasRoomFor(16));
    EXPECT_EQ(sock.stream, "bbbb");
}

TEST(FeedSendBuffer, SocketErrors) {
    FakeFeedSocket sock;
    FeedSendBuffer buf;
    buf.Open(MakeConfig(sock, 16), 0);
    EXPECT_EQ(Write(buf, "aaaa", 1), FeedSendBuffer::kQueued);
    sock.broken = true;
    EXPECT_FALSE(buf.Flush(2));
    EXPECT_EQ(Write(buf, "bbbb", 3), FeedSendBuffer::kError);
    FeedSendBuffer unopened;
    EXPECT_EQ(Write(unopened, "cccc", 4), FeedSendBuffer::kError);
}

TEST(FeedSendBuffer, StallTimeCountsFromLastProgress) {
    FakeFeedSocket sock;
    sock.room = 100;
    FeedSendBuffer buf;
    buf.Open(MakeConfig(sock, 8), 0);
    EXPECT_EQ(Write(buf, "aaaa", 50000), FeedSendBuffer::kSent);
    sock.room = 0;
    // The wait starts when the socket first refuses data, however long it was idle before.
    EXPECT_EQ(Write(buf, "bbbb", 90000), FeedSendBuffer::kQueued);
    EXPECT_EQ(buf.StalledMs(90000), 0u);
    EXPECT_EQ(buf.StalledMs(93000), 3000u);
    EXPECT_EQ(Write(buf, "cccc", 94000), FeedSendBuffer::kQueued);
    EXPECT_EQ(Write(buf, "dddd", 95000), FeedSendBuffer::kDropped);
    EXPECT_EQ(buf.StalledMs(96000), 6000u);
    sock.room = 1;  // One byte of progress restarts the timer; the feed is still behind.
    EXPECT_TRUE(buf.Flush(97000));
    EXPECT_EQ(buf.StalledMs(98000), 1000u);
    sock.room = 100;
    EXPECT_TRUE(buf.Flush(99000));
    EXPECT_EQ(buf.StalledMs(120000), 0u);
}

TEST(FeedSendBuffer, StallIsSeenWithoutABacklog) {
    // Batches too large for the backlog are dropped whole; the refusing socket must still count as stalled.
    FakeFeedSocket sock;
    FeedSendBuffer buf;
    buf.Open(MakeConfig(sock, 2), 1000);
    EXPECT_EQ(Write(buf, "aaaa", 2000), FeedSendBuffer::kDropped);
    EXPECT_EQ(Write(buf, "bbbb", 5000), FeedSendBuffer::kDropped);
    EXPECT_EQ(buf.StalledMs(6000), 4000u);
}

TEST(FeedSendBuffer, OpenStartsAFreshConnection) {
    FakeFeedSocket old_sock, new_sock;
    FeedSendBuffer buf;
    buf.Open(MakeConfig(old_sock, 16), 0);
    EXPECT_EQ(Write(buf, "aaaa", 1), FeedSendBuffer::kQueued);
    new_sock.room = 100;
    buf.Open(MakeConfig(new_sock, 16), 20000);
    EXPECT_EQ(buf.BacklogLenBytes(), 0);
    EXPECT_EQ(buf.StalledMs(20000), 0u);
    EXPECT_EQ(Write(buf, "bbbb", 20001), FeedSendBuffer::kSent);
    EXPECT_EQ(new_sock.stream, "bbbb");  // Bytes queued for the old connection never reach the new one.
    EXPECT_EQ(old_sock.stream, "");
}
