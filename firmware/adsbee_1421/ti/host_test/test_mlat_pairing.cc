// MLAT time base and pairing rule (mlat/mlat_pairing.hh).
#include "gtest/gtest.h"
#include "mlat/mlat_pairing.hh"

static constexpr uint64_t kUs = MlatTimebase::kTicksPerUs;
static constexpr uint64_t kPeriodTicks = 1u << MlatTimebase::kCounterBits;

// Edge at t: the timestamp names the same event, kAgreeCenterHalfTicks earlier.
static int64_t TimestampOf(uint64_t edge_ticks) {
    return static_cast<int64_t>(edge_ticks * 2) - MlatPairer::kAgreeCenterHalfTicks;
}

// A sweep whose level request goes out at request_ticks; the answer comes 170 us later (one main-loop pass).
static MlatPairer::Sweep SweepAt(uint64_t request_ticks, bool has_timestamp = false, int64_t timestamp = 0) {
    MlatPairer::Sweep sweep;
    sweep.level_request_ticks = request_ticks;
    sweep.level_answer_ticks = request_ticks + 170 * kUs;
    sweep.has_timestamp = has_timestamp;
    sweep.timestamp_half_ticks = timestamp;
    return sweep;
}

struct PairTest : testing::Test {
    MlatPairer pairer;
    uint64_t stamp = 0;

    bool Pair(uint64_t request_ticks, uint16_t num_packets, bool has_timestamp = false, int64_t timestamp = 0) {
        MlatPairer::Sweep sweep = SweepAt(request_ticks, has_timestamp, timestamp);
        return pairer.Pair(sweep, num_packets, sweep.level_answer_ticks + 500 * kUs, &stamp);
    }
};

TEST_F(PairTest, LonePacketNamedByEdgeAndTimestampIsStamped) {
    const uint64_t edge = 10000 * kUs;
    pairer.AddEdge(edge);
    ASSERT_TRUE(Pair(edge + 200 * kUs, 1, true, TimestampOf(edge)));
    EXPECT_EQ(stamp, edge);
    EXPECT_EQ(pairer.counters().stamped, 1u);
}

TEST_F(PairTest, EdgeAloneDoesNotStamp) {
    const uint64_t edge = 10000 * kUs;
    pairer.AddEdge(edge);
    EXPECT_FALSE(Pair(edge + 200 * kUs, 1));
    EXPECT_EQ(pairer.counters().unstamped_no_event, 1u);
}

TEST_F(PairTest, TimestampAloneDoesNotStamp) {
    // The latch was still set from an earlier packet, so this one made no edge.
    const uint64_t event = 10000 * kUs;
    EXPECT_FALSE(Pair(event + 200 * kUs, 1, true, TimestampOf(event)));
    EXPECT_EQ(pairer.counters().unstamped_no_event, 1u);
}

TEST_F(PairTest, EdgeAndTimestampMustAgree) {
    // A second packet followed the first before the timestamp was read: the timestamp names the second.
    const uint64_t first = 10000 * kUs, second = first + 150 * kUs;
    pairer.AddEdge(first);
    EXPECT_FALSE(Pair(first + 200 * kUs, 1, true, TimestampOf(second)));
    EXPECT_EQ(pairer.counters().unstamped_disagree, 1u);
    EXPECT_EQ(pairer.counters().stamped, 0u);
}

TEST_F(PairTest, AgreementWindow) {
    const int64_t tolerance = MlatPairer::kAgreeToleranceHalfTicks;
    uint64_t edge = 10000 * kUs;
    for (int64_t offset : {-tolerance, int64_t{0}, tolerance}) {
        pairer.AddEdge(edge);
        EXPECT_TRUE(Pair(edge + 200 * kUs, 1, true, TimestampOf(edge) + offset)) << offset;
        edge += 5000 * kUs;
    }
    for (int64_t offset : {-tolerance - 1, tolerance + 1}) {
        pairer.AddEdge(edge);
        EXPECT_FALSE(Pair(edge + 200 * kUs, 1, true, TimestampOf(edge) + offset)) << offset;
        edge += 5000 * kUs;
    }
}

TEST_F(PairTest, SeveralPacketsInAReadAreNotStamped) {
    // The edge names the first packet and the timestamp the last: no event is named by both.
    const uint64_t first = 10000 * kUs, last = first + 300 * kUs;
    pairer.AddEdge(first);
    EXPECT_FALSE(Pair(last + 200 * kUs, 2, true, TimestampOf(last)));
    EXPECT_EQ(pairer.counters().unstamped_multiple, 2u);
    // The last packet of the read is not stamped from the timestamp alone either.
    EXPECT_EQ(pairer.counters().stamped, 0u);
}

TEST_F(PairTest, EventTooCloseToTheLevelReadIsAmbiguous) {
    // Between kHereTicks before the request and kLaterTicks before the answer the packet may be in either read.
    const uint64_t request = 10000 * kUs;
    for (uint64_t age_us : {144u, 100u, 46u}) {
        MlatPairer fresh;
        const uint64_t edge = request - age_us * kUs;
        fresh.AddEdge(edge);
        MlatPairer::Sweep sweep = SweepAt(request, true, TimestampOf(edge));
        EXPECT_FALSE(fresh.Pair(sweep, 1, request + 1000 * kUs, &stamp)) << age_us;
        EXPECT_EQ(fresh.counters().unstamped_ambiguous, 1u) << age_us;
        EXPECT_EQ(fresh.num_pending(), 0) << age_us;  // Never used for a later read.
    }
}

TEST_F(PairTest, EventAtTheHardBoundIsInTheRead) {
    const uint64_t request = 10000 * kUs, edge = request - 145 * kUs;
    pairer.AddEdge(edge);
    EXPECT_TRUE(Pair(request, 1, true, TimestampOf(edge)));
}

TEST_F(PairTest, EventJustBeforeTheAnswerWaitsForTheNextRead) {
    // The packet of an event less than kLaterTicks before the level answer is not in this read.
    const uint64_t request = 10000 * kUs, edge = request + 170 * kUs - 100 * kUs;
    pairer.AddEdge(edge);
    EXPECT_FALSE(Pair(request, 0));
    EXPECT_EQ(pairer.num_pending(), 1);
    ASSERT_TRUE(Pair(request + 600 * kUs, 1, true, TimestampOf(edge)));
    EXPECT_EQ(stamp, edge);
}

TEST_F(PairTest, LaterEventDoesNotStampAnEarlierPacket) {
    // The read holds the packet of an event that made no edge. A later edge, whose packet is still on its way,
    // must not stamp it.
    const uint64_t request = 10000 * kUs, later = request + 170 * kUs - 20 * kUs;
    pairer.AddEdge(later);
    EXPECT_FALSE(Pair(request, 1, true, TimestampOf(later)));
    EXPECT_EQ(pairer.counters().stamped, 0u);
    // Its own packet arrives alone in the next read.
    ASSERT_TRUE(Pair(request + 600 * kUs, 1, true, TimestampOf(later)));
    EXPECT_EQ(stamp, later);
}

TEST_F(PairTest, EventThatMayBeInAnEarlierReadIsDropped) {
    const uint64_t first_request = 10000 * kUs;
    ASSERT_FALSE(Pair(first_request, 1));  // A read with no known event; it sets the boundary.
    // An edge old enough to have been in that read (reported late by the capture ring).
    const uint64_t edge = first_request + 170 * kUs - 130 * kUs;
    pairer.AddEdge(edge);
    EXPECT_FALSE(Pair(first_request + 600 * kUs, 1, true, TimestampOf(edge)));
    EXPECT_EQ(pairer.counters().stamped, 0u);
}

TEST_F(PairTest, EmptyReadKeepsTheEventForTheNextRead) {
    const uint64_t request = 10000 * kUs, edge = request - 100 * kUs;  // Ambiguous for this read.
    pairer.AddEdge(edge);
    EXPECT_FALSE(Pair(request, 0));
    ASSERT_TRUE(Pair(request + 600 * kUs, 1, true, TimestampOf(edge)));
    EXPECT_EQ(stamp, edge);
}

TEST_F(PairTest, EventWithoutPacketIsCounted) {
    const uint64_t edge = 10000 * kUs;
    pairer.AddEdge(edge);
    EXPECT_FALSE(Pair(edge + 200 * kUs, 0));
    EXPECT_EQ(pairer.counters().orphan_events, 1u);
    EXPECT_EQ(pairer.num_pending(), 0);
}

TEST_F(PairTest, TwoEdgesForOnePacketAreNotUsed) {
    const uint64_t first = 10000 * kUs, second = first + 200 * kUs;
    pairer.AddEdge(first);
    pairer.AddEdge(second);
    EXPECT_FALSE(Pair(second + 200 * kUs, 1, true, TimestampOf(second)));
    EXPECT_EQ(pairer.counters().stamped, 0u);
    EXPECT_EQ(pairer.counters().orphan_events, 1u);
}

TEST_F(PairTest, BurstStampsOnlyLoneAgreedPackets) {
    // Sweeps every 1 ms. Packets: one alone, three in one sweep, one alone.
    uint64_t t = 100000 * kUs;
    pairer.AddEdge(t);
    ASSERT_TRUE(Pair(t + 400 * kUs, 1, true, TimestampOf(t)));
    EXPECT_EQ(stamp, t);

    t += 1000 * kUs;
    pairer.AddEdge(t);  // One edge only: the latch stays set for the two packets after it.
    EXPECT_FALSE(Pair(t + 800 * kUs, 3, true, TimestampOf(t + 500 * kUs)));

    t += 2000 * kUs;
    pairer.AddEdge(t);
    ASSERT_TRUE(Pair(t + 400 * kUs, 1, true, TimestampOf(t)));
    EXPECT_EQ(stamp, t);
    EXPECT_EQ(pairer.counters().stamped, 2u);
    EXPECT_EQ(pairer.counters().unstamped_multiple, 3u);
}

TEST_F(PairTest, NothingIsStampedAcrossAnUnpairedRead) {
    // FIFO overflow resync or an IRQ-paced chain read: events before it may belong to packets it took.
    const uint64_t edge = 10000 * kUs;
    pairer.AddEdge(edge);
    pairer.Break(edge + 300 * kUs);
    pairer.CountUnpaired(9);
    EXPECT_FALSE(Pair(edge + 600 * kUs, 1, true, TimestampOf(edge)));
    EXPECT_EQ(pairer.counters().stamped, 0u);
    EXPECT_EQ(pairer.counters().unstamped_multiple, 9u);

    // An edge that is younger than the break but may still have been in the unpaired read.
    const uint64_t near_edge = edge + 300 * kUs - 130 * kUs;
    pairer.AddEdge(near_edge);
    EXPECT_FALSE(Pair(edge + 1200 * kUs, 1, true, TimestampOf(near_edge)));

    // Pairing resumes with the next event.
    const uint64_t next = edge + 5000 * kUs;
    pairer.AddEdge(next);
    ASSERT_TRUE(Pair(next + 300 * kUs, 1, true, TimestampOf(next)));
    EXPECT_EQ(stamp, next);
}

TEST_F(PairTest, StalledMainLoopLeavesOldEventsUnstamped) {
    // The UAT radio or the console held the main loop: the sweep runs long after the event.
    const uint64_t edge = 10000 * kUs;
    pairer.AddEdge(edge);
    MlatPairer::Sweep sweep = SweepAt(edge + 150000 * kUs, true, TimestampOf(edge));
    EXPECT_FALSE(pairer.Pair(sweep, 1, sweep.level_answer_ticks, &stamp));
    EXPECT_EQ(pairer.counters().stamped, 0u);
}

TEST_F(PairTest, YoungEdgeHoldsTheSweepBack) {
    const uint64_t edge = 10000 * kUs;
    EXPECT_FALSE(pairer.HasYoungEdge(edge));
    pairer.AddEdge(edge);
    EXPECT_TRUE(pairer.HasYoungEdge(edge + 20 * kUs));
    EXPECT_TRUE(pairer.HasYoungEdge(edge + 144 * kUs));
    // From here a sweep's level request finds the packet surely in the read.
    EXPECT_FALSE(pairer.HasYoungEdge(edge + 145 * kUs));
    ASSERT_TRUE(Pair(edge + 145 * kUs, 1, true, TimestampOf(edge)));
    EXPECT_EQ(stamp, edge);
}

TEST_F(PairTest, PendingQueueOverflowDropsEverything) {
    uint64_t t = 10000 * kUs;
    for (uint16_t i = 0; i <= MlatPairer::kMaxPendingEdges; i++) {
        pairer.AddEdge(t);
        t += 200 * kUs;
    }
    EXPECT_EQ(pairer.num_pending(), 1);
    EXPECT_EQ(pairer.counters().dropped_edges, MlatPairer::kMaxPendingEdges);
}

TEST(MlatTimebase, CountsAcrossRollovers) {
    MlatTimebase timebase;
    uint32_t now_us = 123456;
    uint64_t true_ticks = 5 * kPeriodTicks - 1000;  // Starts just before a wrap.
    timebase.Start(static_cast<uint32_t>(true_ticks), now_us, true_ticks);
    for (int i = 0; i < 5000; i++) {  // 170 us passes over 0.85 s: two wraps.
        now_us += 170;
        true_ticks += 170 * kUs;
        ASSERT_TRUE(timebase.Update(static_cast<uint32_t>(true_ticks) & MlatTimebase::kCounterMask, now_us));
        ASSERT_EQ(timebase.ticks(), true_ticks);
    }
}

TEST(MlatTimebase, LongGapsAreBridgedByTheSystemClock) {
    // Gaps of several counter periods, with the system clock 3 % fast or slow and 100 us coarse.
    for (double clock_error : {-0.03, 0.0, 0.03}) {
        MlatTimebase timebase;
        uint32_t now_us = 0xFFFF0000;  // The system clock wraps too.
        uint64_t true_ticks = 777;
        timebase.Start(static_cast<uint32_t>(true_ticks), now_us, true_ticks);
        for (uint32_t gap_us : {100u, 349000u, 349600u, 350000u, 524000u, 700000u, 1234567u, 1900000u, 170u}) {
            true_ticks += static_cast<uint64_t>(gap_us) * kUs;
            now_us += static_cast<uint32_t>(gap_us * (1.0 + clock_error)) / 100 * 100;
            ASSERT_TRUE(timebase.Update(static_cast<uint32_t>(true_ticks) & MlatTimebase::kCounterMask, now_us))
                << gap_us;
            ASSERT_EQ(timebase.ticks(), true_ticks) << gap_us << " " << clock_error;
            EXPECT_EQ(timebase.gap_ticks(), static_cast<uint64_t>(gap_us) * kUs);
        }
    }
}

TEST(MlatTimebase, GapBeyondTheLimitIsRefused) {
    MlatTimebase timebase;
    timebase.Start(0, 1000, 0);
    EXPECT_TRUE(timebase.Update(100, 1000 + MlatTimebase::kMaxGapUs));
    EXPECT_FALSE(timebase.Update(200, 1000 + 2 * MlatTimebase::kMaxGapUs + 1));
}

TEST(MlatTimebase, ExtendsCapturesBeforeTheLastUpdate) {
    MlatTimebase timebase;
    const uint64_t start = 3 * kPeriodTicks + 100;
    timebase.Start(100, 0, start);
    // A capture 2 ms before an update that crossed a wrap.
    const uint64_t capture = start + kPeriodTicks - 100 - 2000 * kUs;
    const uint64_t update = capture + 2000 * kUs + 500;
    ASSERT_TRUE(timebase.Update(static_cast<uint32_t>(update) & MlatTimebase::kCounterMask,
                                static_cast<uint32_t>((update - start) / kUs)));
    EXPECT_EQ(timebase.Extend(static_cast<uint32_t>(capture) & MlatTimebase::kCounterMask), capture);
}

// Count pairing (PairBatch), used with the per-packet uDMA clear.
struct BatchTest : testing::Test {
    MlatPairer pairer;
    uint64_t stamps[MlatPairer::kMaxBatchPackets] = {};
    bool stamped[MlatPairer::kMaxBatchPackets] = {};

    uint16_t Read(uint64_t request_ticks, uint16_t num_packets, bool with_ts = false, bool has_ts = false,
                  int64_t ts = 0) {
        MlatPairer::Sweep sweep = SweepAt(request_ticks, has_ts, ts);
        return pairer.PairBatch(sweep, num_packets, sweep.level_answer_ticks + 500 * kUs, with_ts, stamps, stamped);
    }
};

TEST_F(BatchTest, EdgesMatchingPacketsAreStampedInOrder) {
    const uint64_t a = 10000 * kUs, b = a + 140 * kUs, c = b + 300 * kUs;
    pairer.AddEdge(a);
    pairer.AddEdge(b);
    pairer.AddEdge(c);
    ASSERT_EQ(Read(c + 200 * kUs, 3), 3);
    EXPECT_EQ(stamps[0], a);
    EXPECT_EQ(stamps[1], b);
    EXPECT_EQ(stamps[2], c);
    EXPECT_EQ(pairer.counters().batches_stamped, 1u);
}

TEST_F(BatchTest, SyncWithoutEdgeDropsTheBatch) {
    // A second sync came while the latch was still set: two packets, one edge.
    const uint64_t a = 10000 * kUs;
    pairer.AddEdge(a);
    EXPECT_EQ(Read(a + 600 * kUs, 2), 0);
    EXPECT_FALSE(stamped[0]);
    EXPECT_FALSE(stamped[1]);
    EXPECT_EQ(pairer.counters().batches_more_packets, 1u);
    EXPECT_EQ(pairer.counters().unstamped_mismatch, 2u);
}

TEST_F(BatchTest, YoungEdgeIsLeftForTheNextRead) {
    // b is too young to be in the read; the read holds a only.
    const uint64_t a = 10000 * kUs, b = a + 300 * kUs;
    pairer.AddEdge(a);
    pairer.AddEdge(b);
    ASSERT_EQ(Read(b + 50 * kUs, 1), 1);
    EXPECT_EQ(stamps[0], a);
    ASSERT_EQ(Read(b + 1000 * kUs, 1), 1);
    EXPECT_EQ(stamps[0], b);
}

TEST_F(BatchTest, AmbiguousEdgeCarriesOverAfterAMatchedRead) {
    // b sits between "surely in" and "surely later" for the first read. The counts agree (1 packet, 1 sure edge), so
    // b's packet was not in it and b stays usable.
    const uint64_t a = 10000 * kUs, b = a + 300 * kUs;
    pairer.AddEdge(a);
    pairer.AddEdge(b);
    ASSERT_EQ(Read(b + 100 * kUs, 1), 1);  // b + 145 > request, b + 125 <= answer (request + 170).
    ASSERT_EQ(Read(b + 2000 * kUs, 1), 1);
    EXPECT_EQ(stamps[0], b);
}

TEST_F(BatchTest, AmbiguousEdgeIsDroppedAfterAMismatch) {
    const uint64_t a = 10000 * kUs, b = a + 300 * kUs;
    pairer.AddEdge(a);
    pairer.AddEdge(b);
    EXPECT_EQ(Read(b + 100 * kUs, 2), 0);  // b's packet may be the second one.
    EXPECT_EQ(Read(b + 2000 * kUs, 1), 0);  // b is unknown now: its packet may have been read.
    EXPECT_EQ(pairer.counters().batches_more_packets, 2u);
}

TEST_F(BatchTest, EdgeWithoutPacketIsCounted) {
    const uint64_t a = 10000 * kUs, b = a + 200 * kUs;
    pairer.AddEdge(a);
    pairer.AddEdge(b);
    EXPECT_EQ(Read(b + 600 * kUs, 1), 0);
    EXPECT_EQ(pairer.counters().batches_more_edges, 1u);
    EXPECT_EQ(pairer.counters().orphan_events, 1u);
}

TEST_F(BatchTest, TimestampVetoesAMatchThatDisagrees) {
    // Count agrees, but the latest event (surely in the read) is not the last edge.
    const uint64_t a = 10000 * kUs, missed = a + 200 * kUs;
    pairer.AddEdge(a);
    EXPECT_EQ(Read(missed + 400 * kUs, 1, true, true, TimestampOf(missed)), 0);
    EXPECT_EQ(pairer.counters().timestamp_vetoes, 1u);
}

TEST_F(BatchTest, TimestampOfALaterEventDoesNotVeto) {
    const uint64_t a = 10000 * kUs, later = a + 500 * kUs;
    pairer.AddEdge(a);
    EXPECT_EQ(Read(later + 50 * kUs, 1, true, true, TimestampOf(later)), 1);
}

TEST_F(BatchTest, TimestampReanchorsTheLastPacketOfAMismatch) {
    // The second sync of three made no edge; the timestamp names the third, surely in the read.
    const uint64_t a = 10000 * kUs, c = a + 400 * kUs;
    pairer.AddEdge(a);
    pairer.AddEdge(c);
    ASSERT_EQ(Read(c + 300 * kUs, 3, true, true, TimestampOf(c)), 1);
    EXPECT_FALSE(stamped[0]);
    EXPECT_FALSE(stamped[1]);
    ASSERT_TRUE(stamped[2]);
    EXPECT_EQ(stamps[2], c);
    EXPECT_EQ(pairer.counters().reanchored, 1u);
}

TEST_F(BatchTest, EmptyReadKeepsEdges) {
    const uint64_t a = 10000 * kUs;
    pairer.AddEdge(a);
    EXPECT_EQ(Read(a + 50 * kUs, 0), 0);
    ASSERT_EQ(Read(a + 400 * kUs, 1), 1);
    EXPECT_EQ(stamps[0], a);
}

TEST_F(BatchTest, BreakDropsEdgesBeforeIt) {
    const uint64_t a = 10000 * kUs;
    pairer.AddEdge(a);
    pairer.Break(a + 300 * kUs);
    EXPECT_EQ(Read(a + 600 * kUs, 1), 0);
}
