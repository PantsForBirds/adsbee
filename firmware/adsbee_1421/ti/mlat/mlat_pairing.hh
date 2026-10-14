#pragma once

// MLAT timestamps for LR2021 packets: 64-bit time base and the rule that pairs a sync event with a packet.
// Header-only and SDK-free for the host tests.
//
// Two sources mark a packet's sync event:
//   - Edge: SyncwordValid on LR2021 DIO5, latched until cleared, timed by a 48 MHz capture timer. One edge per
//     clear, and the loop drain sweep clears once.
//   - Timestamp: the LR2021's own time of its latest sync event, read by the sweep after the FIFO level.
// A packet is stamped only when its FIFO read holds that packet alone and both sources name one event that is
// surely in the read. Every other packet stays unstamped.

#include <cstdint>

// Extends the 24-bit capture counter to 64 bits, with the system clock picking the number of wraps between updates.
class MlatTimebase {
   public:
    static constexpr uint32_t kTicksPerUs = 48;
    static constexpr uint32_t kCounterBits = 24;  // Wraps every 349.5 ms.
    static constexpr uint32_t kCounterMask = (1u << kCounterBits) - 1;
    // Longest gap between updates the system clock may bridge. Its error over the gap must stay under half a
    // counter period: 2 s allows 8 %.
    static constexpr uint32_t kMaxGapUs = 2000000;

    // Starts counting from ticks at counter value counter and system time now_us.
    void Start(uint32_t counter, uint32_t now_us, uint64_t ticks) {
        counter_ = counter & kCounterMask;
        now_us_ = now_us;
        ticks_ = ticks;
        gap_ticks_ = 0;
    }

    // Call at least every kMaxGapUs. False: the gap was longer and the count is lost, so Start() again.
    bool Update(uint32_t counter, uint32_t now_us) {
        counter &= kCounterMask;
        const uint32_t elapsed_us = now_us - now_us_;
        if (elapsed_us > kMaxGapUs) {
            return false;
        }
        const uint64_t expected_ticks = static_cast<uint64_t>(elapsed_us) * kTicksPerUs;
        const uint32_t counter_ticks = (counter - counter_) & kCounterMask;
        // The count nearest to the system clock's that matches the counter.
        const uint64_t half_period = 1u << (kCounterBits - 1);
        const uint64_t wraps =
            expected_ticks + half_period > counter_ticks ? (expected_ticks + half_period - counter_ticks) >> kCounterBits
                                                         : 0;
        gap_ticks_ = counter_ticks + (wraps << kCounterBits);
        ticks_ += gap_ticks_;
        counter_ = counter;
        now_us_ = now_us;
        return true;
    }

    uint64_t ticks() const { return ticks_; }
    // Time between the last two updates. A capture is extended correctly only if it is younger than one counter
    // period at the update after it.
    uint64_t gap_ticks() const { return gap_ticks_; }

    // 64-bit time of a counter value captured less than one counter period before the last update.
    uint64_t Extend(uint32_t counter) const { return ticks_ - ((counter_ - counter) & kCounterMask); }

   private:
    uint32_t counter_ = 0;
    uint32_t now_us_ = 0;
    uint64_t ticks_ = 0;
    uint64_t gap_ticks_ = 0;
};

class MlatPairer {
   public:
    static constexpr uint32_t kTicksPerUs = MlatTimebase::kTicksPerUs;
    // A packet shows in the FIFO level 135 to 140 us after its sync event (112 bits on air, then processing).
    // An event at least kHereTicks before the level request: its packet is in the read.
    static constexpr uint32_t kHereTicks = 145 * kTicksPerUs;
    // An event less than kLaterTicks before the level answer: its packet is not in the read.
    static constexpr uint32_t kLaterTicks = 125 * kTicksPerUs;
    // Edges older than this are dropped. Must stay under one counter period, less the longest gap between polls.
    static constexpr uint32_t kStaleTicks = 100000 * kTicksPerUs;
    // Edge time minus timestamp time for one event, in half ticks (1/96 us): 0.83 us on the bench. Two events
    // are at least a packet (120 us) apart.
    static constexpr int32_t kAgreeCenterHalfTicks = 80;
    static constexpr int32_t kAgreeToleranceHalfTicks = 96;

    static constexpr uint16_t kMaxPendingEdges = 32;
    static constexpr uint16_t kMaxBatchPackets = 20;  // A full FIFO holds 18.

    struct Counters {
        uint32_t stamped = 0;
        uint32_t unstamped_no_event = 0;   // No edge, or no timestamp, surely in the read.
        uint32_t unstamped_ambiguous = 0;  // An edge may belong to this read or to a neighbor.
        uint32_t unstamped_multiple = 0;   // Several packets in the read, or a read that is never paired.
        uint32_t unstamped_disagree = 0;   // Edge and timestamp name different events.
        uint32_t orphan_events = 0;        // An edge surely in a read with fewer packets than edges.
        uint32_t dropped_edges = 0;        // Pending queue full.
        // Count pairing (PairBatch).
        uint32_t batches_stamped = 0;
        uint32_t batches_more_packets = 0;  // Fewer edges surely in the read than packets: a sync without an edge.
        uint32_t batches_more_edges = 0;    // More edges surely in the read than packets: an edge without a packet.
        uint32_t unstamped_mismatch = 0;    // Packets of those batches.
        uint32_t reanchored = 0;            // Last packet of a mismatched batch stamped through the timestamp.
        uint32_t timestamp_vetoes = 0;      // Matched batch dropped: the timestamp contradicts the edges.
        int32_t bench_diff_min = 0x7FFFFFFF, bench_diff_max = -0x7FFFFFFF;
    };

    // One loop drain sweep. Times are 64-bit ticks.
    struct Sweep {
        uint64_t level_request_ticks = 0;  // NSS falling edge of the GetRxFifoLevel request.
        uint64_t level_answer_ticks = 0;   // The level was sampled before this.
        bool has_timestamp = false;
        int64_t timestamp_half_ticks = 0;  // Time of the LR2021's latest sync event, in half ticks.
    };

    void AddEdge(uint64_t ticks) {
        if (num_pending_ == kMaxPendingEdges) {
            // More edges than sweeps can use: none of them is trusted.
            counters_.dropped_edges += num_pending_;
            num_pending_ = 0;
            boundary_ticks_ = ticks;
        }
        pending_[num_pending_++] = ticks;
    }

    // A FIFO read that is not paired was made, or the FIFO or the time base was reset: no event before
    // now_ticks is used.
    void Break(uint64_t now_ticks) {
        num_pending_ = 0;
        boundary_ticks_ = now_ticks;
    }

    // True while a pending edge is younger than kHereTicks: a sweep started now could not place its packet. The
    // caller holds the sweep back, for one edge at most (the next edge needs a clear, which only a sweep sends).
    bool HasYoungEdge(uint64_t now_ticks) const {
        return num_pending_ > 0 && now_ticks - pending_[num_pending_ - 1] < kHereTicks;
    }

    // Packets of a read that is never paired (IRQ-paced chain, FIFO overflow).
    void CountUnpaired(uint16_t num_packets) { counters_.unstamped_multiple += num_packets; }

    // Call for every completed sweep, with the number of whole packets it read (0: none). True: the read's one
    // packet gets *stamp_ticks. now_ticks is the current time.
    bool Pair(const Sweep& sweep, uint16_t num_packets, uint64_t now_ticks, uint64_t* stamp_ticks) {
        // Sort the pending edges by where their packet is.
        uint16_t num_here = 0, num_kept = 0;
        bool ambiguous = false;
        uint64_t here_ticks = 0;
        for (uint16_t i = 0; i < num_pending_; i++) {
            const uint64_t t = pending_[i];
            if (now_ticks - t > kStaleTicks || t + kLaterTicks <= boundary_ticks_) {
                continue;  // Too old, or possibly in an earlier read.
            }
            if (t + kHereTicks <= sweep.level_request_ticks) {
                if (num_here++ == 0) {
                    here_ticks = t;
                }
            } else if (t + kLaterTicks > sweep.level_answer_ticks) {
                pending_[num_kept++] = t;  // Surely not read yet.
            } else {
                ambiguous = true;
                if (num_packets == 0) {
                    pending_[num_kept++] = t;  // Nothing was read, so its packet is still to come.
                }
            }
        }
        num_pending_ = num_kept;
        if (num_here > num_packets) {
            counters_.orphan_events += num_here - num_packets;
        }
        if (num_packets == 0) {
            return false;  // Nothing was popped: the boundary stays.
        }
        boundary_ticks_ = sweep.level_answer_ticks;
        if (num_packets > 1) {
            counters_.unstamped_multiple += num_packets;
            return false;
        }
        if (ambiguous) {
            counters_.unstamped_ambiguous++;
            return false;
        }
        if (num_here != 1 || !sweep.has_timestamp) {
            counters_.unstamped_no_event++;
            return false;
        }
        // The edge is the first event after a clear and the timestamp is the latest event of all. When they are
        // one event, no other event came between the clear and the read, so the read's packet is this event's.
        const int64_t diff_half_ticks = static_cast<int64_t>(here_ticks * 2) - sweep.timestamp_half_ticks;
        if (diff_half_ticks > -100000 && diff_half_ticks < 100000) {
            if (diff_half_ticks < counters_.bench_diff_min) counters_.bench_diff_min = static_cast<int32_t>(diff_half_ticks);
            if (diff_half_ticks > counters_.bench_diff_max) counters_.bench_diff_max = static_cast<int32_t>(diff_half_ticks);
        }
        if (diff_half_ticks < kAgreeCenterHalfTicks - kAgreeToleranceHalfTicks ||
            diff_half_ticks > kAgreeCenterHalfTicks + kAgreeToleranceHalfTicks) {
            counters_.unstamped_disagree++;
            return false;
        }
        counters_.stamped++;
        *stamp_ticks = here_ticks;
        return true;
    }

    // Count pairing for reads of any size, with a per-packet clear: every sync event raises an edge unless it comes
    // while the latch is still set. The read's packets are exactly the packets of the edges surely in the read when
    // the counts agree, because every such edge has its packet in the read (it was in the FIFO at the level
    // sample, and not in an earlier read). Then the n-th edge is the n-th packet. Otherwise no packet is stamped.
    // with_timestamp: the sweep's timestamp names the latest sync event. A matched batch whose last edge disagrees
    // with a timestamp that is surely in the read is dropped; a mismatched batch whose last edge agrees with such a
    // timestamp gets its last packet stamped (nothing can come between the latest event and the read's end).
    // Returns how many packets got stamps: stamped[i] says which, stamps[i] holds them.
    uint16_t PairBatch(const Sweep& sweep, uint16_t num_packets, uint64_t now_ticks, bool with_timestamp,
                       uint64_t* stamps, bool* stamped) {
        for (uint16_t i = 0; i < num_packets && i < kMaxBatchPackets; i++) {
            stamped[i] = false;
        }
        uint64_t here[kMaxBatchPackets + 1];
        uint16_t num_here = 0, num_kept = 0;
        uint64_t oldest_ambiguous = 0;
        bool has_ambiguous = false;
        for (uint16_t i = 0; i < num_pending_; i++) {
            const uint64_t t = pending_[i];
            if (now_ticks - t > kStaleTicks || t + kLaterTicks <= boundary_ticks_) {
                continue;  // Too old, or possibly in an earlier read.
            }
            if (t + kHereTicks <= sweep.level_request_ticks) {
                if (num_here <= kMaxBatchPackets) here[num_here] = t;
                num_here++;
            } else {
                // Surely later, or ambiguous: kept. An ambiguous edge is resolved below.
                if (t + kLaterTicks <= sweep.level_answer_ticks && !has_ambiguous) {
                    has_ambiguous = true;
                    oldest_ambiguous = t;
                }
                pending_[num_kept++] = t;
            }
        }
        num_pending_ = num_kept;
        if (num_packets == 0) {
            if (num_here > 0) counters_.orphan_events += num_here;
            return 0;  // Nothing was popped: the boundary stays.
        }
        // Ambiguous edges: when the counts agree their packets are surely not in this read, so they stay usable for
        // the next one. Otherwise they may be in this read and the boundary drops them.
        const bool match = num_here == num_packets && num_packets <= kMaxBatchPackets;
        boundary_ticks_ = sweep.level_answer_ticks;
        if (match && has_ambiguous && oldest_ambiguous + kLaterTicks - 1 < boundary_ticks_) {
            boundary_ticks_ = oldest_ambiguous + kLaterTicks - 1;
        }
        // The timestamp's event is surely in this read when it is old enough at the level request.
        bool ts_here = false;
        int64_t last_diff = 0;
        if (with_timestamp && sweep.has_timestamp && num_here > 0 && num_here <= kMaxBatchPackets) {
            ts_here = sweep.timestamp_half_ticks + 2 * static_cast<int64_t>(kHereTicks) <=
                      static_cast<int64_t>(sweep.level_request_ticks * 2);
            last_diff = static_cast<int64_t>(here[num_here - 1] * 2) - sweep.timestamp_half_ticks;
        }
        const bool last_agrees = last_diff >= kAgreeCenterHalfTicks - kAgreeToleranceHalfTicks &&
                                 last_diff <= kAgreeCenterHalfTicks + kAgreeToleranceHalfTicks;
        if (match) {
            if (ts_here && !last_agrees) {
                counters_.timestamp_vetoes++;
                counters_.unstamped_mismatch += num_packets;
                return 0;
            }
            for (uint16_t i = 0; i < num_packets; i++) {
                stamps[i] = here[i];
                stamped[i] = true;
            }
            counters_.batches_stamped++;
            counters_.stamped += num_packets;
            return num_packets;
        }
        if (num_here < num_packets) {
            counters_.batches_more_packets++;
        } else {
            counters_.batches_more_edges++;
            counters_.orphan_events += num_here - num_packets;
        }
        if (ts_here && last_agrees && num_packets <= kMaxBatchPackets) {
            stamps[num_packets - 1] = here[num_here - 1];
            stamped[num_packets - 1] = true;
            counters_.reanchored++;
            counters_.stamped++;
            counters_.unstamped_mismatch += num_packets - 1;
            return 1;
        }
        counters_.unstamped_mismatch += num_packets;
        return 0;
    }

    // True while the youngest pending edge is younger than kHereTicks.
    bool HasYoungEdgeAny(uint64_t now_ticks) const {
        return num_pending_ > 0 && now_ticks - pending_[num_pending_ - 1] < kHereTicks;
    }

    const Counters& counters() const { return counters_; }
    void ResetCounters() { counters_ = Counters(); }
    uint16_t num_pending() const { return num_pending_; }

   private:
    uint64_t pending_[kMaxPendingEdges] = {};  // Edge times not yet used or dropped, oldest first.
    uint16_t num_pending_ = 0;
    uint64_t boundary_ticks_ = 0;  // Latest time the last paired or unpaired FIFO read may have sampled the FIFO.
    Counters counters_;
};
