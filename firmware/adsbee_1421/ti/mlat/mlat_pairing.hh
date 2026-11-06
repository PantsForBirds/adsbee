#pragma once

// MLAT timestamps for LR2021 packets: 64-bit time base and the rule that pairs a sync event with a packet.
// Header-only and SDK-free for the host tests.
//
// SyncwordValid on LR2021 DIO5 is latched until cleared and timed by a 48 MHz capture timer. A uDMA chain clears the
// latch after every edge (mlat/mlat_clear_dma.hh), so every sync event raises an edge unless it comes while the
// latch is still set. Packets are paired with edges by count, per FIFO read: when the edges surely in the read
// equal its packets, the n-th edge is the n-th packet. Otherwise no packet of the read is stamped.

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
    // 64-bit time of a counter value read less than one counter period after the last update.
    uint64_t ExtendForward(uint32_t counter) const { return ticks_ + ((counter - counter_) & kCounterMask); }

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

    static constexpr uint16_t kMaxPendingEdges = 32;
    static constexpr uint16_t kMaxReadPackets = 20;  // A full FIFO holds 18.

    struct Counters {
        uint32_t stamped = 0;
        uint32_t unstamped_more_packets = 0;  // Packets of reads with fewer edges than packets (a sync without an edge).
        uint32_t unstamped_more_edges = 0;    // Packets of reads with more edges than packets.
        uint32_t unstamped_unpaired = 0;      // Packets of reads that are never paired (IRQ-paced chain, FIFO overflow).
        uint32_t orphan_events = 0;           // Edges surely in a read that had no packet for them.
        uint32_t dropped_edges = 0;           // Pending queue full.
    };

    // One loop drain sweep. Times are 64-bit ticks.
    struct Sweep {
        uint64_t level_request_ticks = 0;  // NSS falling edge of the GetRxFifoLevel request.
        uint64_t level_answer_ticks = 0;   // The level was sampled before this.
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

    // True while a pending edge is younger than kHereTicks: a sweep started now could not count its packet.
    bool HasYoungEdge(uint64_t now_ticks) const {
        return num_pending_ > 0 && now_ticks - pending_[num_pending_ - 1] < kHereTicks;
    }

    // Packets of a read that is never paired (IRQ-paced chain, FIFO overflow).
    void CountUnpaired(uint16_t num_packets) { counters_.unstamped_unpaired += num_packets; }

    // Call for every completed sweep, with the number of whole packets it read (0: none) and now_ticks the current
    // time. Every edge surely in the read has its packet in the read: the packet was in the FIFO at the level sample,
    // and not in an earlier read. So when the counts agree, the read's packets are exactly those edges' packets, in
    // order. Returns how many packets got stamps: stamped[i] says which, stamps[i] holds them (num_packets entries).
    uint16_t Pair(const Sweep& sweep, uint16_t num_packets, uint64_t now_ticks, uint64_t* stamps, bool* stamped) {
        for (uint16_t i = 0; i < num_packets && i < kMaxReadPackets; i++) {
            stamped[i] = false;
        }
        uint64_t here[kMaxReadPackets];
        uint16_t num_here = 0, num_kept = 0;
        uint64_t oldest_ambiguous = 0;
        bool has_ambiguous = false;
        for (uint16_t i = 0; i < num_pending_; i++) {
            const uint64_t t = pending_[i];
            if (now_ticks - t > kStaleTicks || t + kLaterTicks <= boundary_ticks_) {
                continue;  // Too old, or possibly in an earlier read.
            }
            if (t + kHereTicks <= sweep.level_request_ticks) {
                if (num_here < kMaxReadPackets) {
                    here[num_here] = t;
                }
                num_here++;
            } else {
                // Surely later, or ambiguous (its packet may or may not be in this read): kept, resolved below.
                if (t + kLaterTicks <= sweep.level_answer_ticks && !has_ambiguous) {
                    has_ambiguous = true;
                    oldest_ambiguous = t;
                }
                pending_[num_kept++] = t;
            }
        }
        num_pending_ = num_kept;
        if (num_packets == 0) {
            counters_.orphan_events += num_here;
            return 0;  // Nothing was popped: the boundary stays.
        }
        // When the counts agree, the ambiguous edges' packets are surely not in this read: they stay usable for the
        // next one. Otherwise they may be, and the boundary drops them.
        const bool match = num_here == num_packets && num_packets <= kMaxReadPackets;
        boundary_ticks_ = sweep.level_answer_ticks;
        if (match && has_ambiguous && oldest_ambiguous + kLaterTicks - 1 < boundary_ticks_) {
            boundary_ticks_ = oldest_ambiguous + kLaterTicks - 1;
        }
        if (!match) {
            if (num_here < num_packets) {
                counters_.unstamped_more_packets += num_packets;
            } else {
                counters_.unstamped_more_edges += num_packets;
                counters_.orphan_events += num_here - num_packets;
            }
            return 0;
        }
        for (uint16_t i = 0; i < num_packets; i++) {
            stamps[i] = here[i];
            stamped[i] = true;
        }
        counters_.stamped += num_packets;
        return num_packets;
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
