#pragma once

#include <stddef.h>
#include <stdint.h>

// Keeps the Programmer's UART at the module console's baud rate; the host's USB baud rate is ignored. The module
// sends "UU" at its rate after every rate change, at boot and after a break (ti/comms/console_autobaud.hh):
//   Watch:   look for signs of another rate (framing/break errors, edges off the current bit grid).
//   Resolve: on a sign, hold host data, find the newest "UU" in the recent edges and retune to its rate.
//   Ask:     if that fails or the rate is unknown, send a break and lock onto the "UU" answer.
// Console bytes are forwarded kConsoleDelayUs late, so bytes garbled by a rate change can be dropped first.
// Pure logic, host-tested in host_test/test_rate_watch.cc.

// The edge timer: edge_capture.hh on the Programmer, a simulated wire in the host tests.
class EdgeSource {
   public:
    // Edges captured since the capture started.
    virtual uint64_t Count() = 0;
    // Writes edge times (clock cycles, mod 2^32) of edges [first, to) to `cycles`; first is `from`, or later if more
    // than max_edges lie in between. Returns to - first.
    virtual size_t Read(uint64_t from, uint64_t to, uint32_t* cycles, size_t max_edges, uint64_t* first_index) = 0;
    // True if edge 0 was falling (edges alternate).
    virtual bool FirstFalling() const = 0;
};

class RateWatch {
   public:
    static constexpr uint32_t kBreakUs = 2000;  // Longer than a frame at 9600 baud.
    // Idle between the NUL sent before a break and the break, so a frame at 9600 baud can finish.
    static constexpr uint32_t kPreBreakIdleUs = 1100;
    static constexpr uint32_t kAskAnswerMs = 30;
    static constexpr uint32_t kMaxAsks = 3;
    static constexpr uint32_t kUnknownAskIntervalMs = 1000;
    // The module applies its saved rate about 50 ms after reset.
    static constexpr uint32_t kBootAnswerMs = 150;
    static constexpr uint32_t kResolveMs = 6;
    // Edges before a hint to search for the "UU", which can precede the first sign of a new rate by a few frames.
    static constexpr size_t kHintLookback = 256;
    static constexpr size_t kMaxAnalyzedEdges = 512;
    // The module's rate right after reset. Data at this rate means it rebooted: wait for its boot "UU".
    static constexpr uint32_t kFactoryBaud = 1000000;
    // While the rate is unknown, a hint resolves at most this often, and never leads to an ask.
    static constexpr uint32_t kUnknownHintIntervalMs = 100;
    static constexpr size_t kMaxCheckedEdges = 32;     // Newest edges checked for a hint,
    static constexpr uint32_t kCheckIntervalUs = 100;  // at most this often (or once the line goes quiet).
    static constexpr uint32_t kQuietUs = 50;  // Analyze after this long without a new edge (or kAnalyzeEvery edges).
    // Longer than one 'U' at 9600 baud plus a check interval.
    static constexpr uint32_t kConsoleDelayUs = 1500;
    static constexpr uint32_t kForwardSampleUs = 20;
    static constexpr size_t kForwardSamples = 128;  // 2.56 ms of arrival times: more than kConsoleDelayUs.
    // On a retune, console bytes that arrived at least this long before the "UU" are forwarded; the rest are dropped.
    static constexpr uint32_t kKeepMarginUs = 60;
    static constexpr size_t kAnalyzeEvery = 32;

    enum class Phase {
        kLocked,       // At the console's rate; watching.
        kBootloader,   // After a reset into the ROM bootloader: at its rate; never asks.
        kUnknown,      // Lock failed; watching, and asking every kUnknownAskIntervalMs.
        kResolving,    // A hint: looking for the "UU" that came with it.
        kAsking,       // Break sent; waiting for "UU".
        kAwaitingBoot  // Reset into the application; waiting for its boot "UU".
    };

    struct Action {
        enum Kind {
            kNone,
            kRetune,      // Forward console bytes up to count `keep`, drop the rest, set the UART to `baud`.
            kBreakStart,  // Drive TX low (after what the UART is still sending).
            kBreakEnd,    // Hand TX back to the UART.
        } kind = kNone;
        uint32_t baud = 0;
        uint32_t keep = 0;
        // Send a NUL after every lock and before every break, so the module detects back-to-back breaks
        // (console_autobaud.hh).
        bool send_nul = false;
    };

    // Statistics for status messages and tests.
    struct LockInfo {
        uint32_t measured_baud = 0;       // From the "UU".
        uint32_t baud = 0;                // The UART's new rate (the measurement snapped to a console rate).
        uint32_t elapsed_us = 0;          // From the reset, the first break or the hint.
        uint32_t asks = 0;                // Breaks sent for it.
        uint32_t answer_us = 0;           // Last break's start to the first edge after it (0: no break).
        uint32_t uu_to_lock_us = 0;       // The "UU"'s first edge to the lock.
        uint32_t quiet_before_uu_us = 0;  // The line was idle this long before the "UU" (up to 1 s).
        bool retuned = false;
    };
    struct Stats {
        uint32_t locks = 0;  // Finished with a "UU" (last_lock() has the latest).
        uint32_t hints = 0;
        uint32_t retunes = 0;
        uint32_t asks = 0;
        uint32_t failed_locks = 0;
    };

    RateWatch(EdgeSource& edges, uint32_t clock_hz) : edges_(edges), clock_hz_(clock_hz) {}

    // The UART runs at `baud`, the console's rate as far as the caller knows.
    void Start(uint32_t baud, uint64_t now_us);
    // The caller reset the module into the application (RESET_N released at now_us).
    void OnAppReset(uint64_t now_us);
    // The caller reset the module into the ROM bootloader and set the UART to its rate.
    void OnBootloaderReset(uint32_t bootloader_baud, uint64_t now_us);
    // Ask now (when the caller knows the console's rate is unknown).
    void Ask(uint64_t now_us);
    // UART framing and break errors received since the last call.
    void OnRxErrors(uint32_t count);

    // Call every loop. sync_high: module may be asleep; ignore UART errors and send no break. rx_received: free-running
    // count of console bytes received.
    Action Poll(uint64_t now_us, bool sync_high, uint32_t rx_received);

    // Console bytes (as an rx_received count) old enough to forward, unless held or dropped.
    uint32_t Forwardable() const { return forwardable_; }

    Phase phase() const { return phase_; }
    uint32_t baud() const { return baud_; }
    bool Locking() const {
        return phase_ == Phase::kResolving || phase_ == Phase::kAsking || phase_ == Phase::kAwaitingBoot;
    }
    bool BreakActive() const { return break_pending_ || break_active_; }
    bool HoldHostData() const { return Locking() || BreakActive(); }
    bool HoldConsoleData() const { return phase_ == Phase::kResolving; }
    bool DropConsoleData() const { return phase_ == Phase::kAsking || phase_ == Phase::kAwaitingBoot; }
    const LockInfo& last_lock() const { return last_lock_; }
    const Stats& stats() const { return stats_; }

   private:
    void StartResolve(uint64_t now_us);
    Action StartAsk(uint64_t now_us);
    // Looks for the newest "UU" in edges [mark_, Count()); on success finishes the lock and returns true.
    bool Analyze(uint64_t now_us, Action* action);
    // Ends a lock in `phase` (kLocked, or kUnknown / back to watching after a failure).
    void EndLock(Phase phase);
    bool EdgesHint(uint64_t to);
    bool EdgesFitRate(uint32_t baud);
    void SampleArrivals(uint32_t received, uint64_t edges, uint64_t now_us);
    // rx_received at the newest arrival sample taken at or before us (Forwardable() if none is that old).
    uint32_t ReceivedBy(uint64_t us) const;
    // Time of the oldest arrival sample that had seen edge `index` (now_us if none had).
    uint64_t EdgeSeenAt(uint64_t index, uint64_t now_us) const;

    EdgeSource& edges_;
    uint32_t clock_hz_;
    Phase phase_ = Phase::kUnknown;
    Phase watch_phase_ = Phase::kUnknown;  // Where a lock returns to when it finds nothing.
    uint32_t baud_ = 0;
    uint32_t pending_errors_ = 0;
    uint64_t checked_ = 0;     // Count() at the last hint check.
    uint64_t check_from_ = 0;  // Edges before this (before the last lock) aren't checked.
    uint64_t last_check_us_ = 0;
    uint64_t mark_ = 0;            // First edge the current lock may use.
    uint64_t analyzed_count_ = 0;  // Count() at the last analysis.
    uint64_t last_count_ = 0;
    uint64_t last_edge_us_ = 0;  // When Count() last changed.
    uint64_t lock_start_us_ = 0;
    uint64_t deadline_us_ = 0;
    uint64_t break_start_us_ = 0;
    uint64_t break_end_us_ = 0;
    bool break_pending_ = false;  // The NUL went out; the break starts at break_start_us_.
    bool sync_high_ = false;      // As of the last Poll().
    bool break_active_ = false;
    uint32_t asks_ = 0;  // In the current lock.
    uint64_t ask_start_us_ = 0;
    uint64_t answer_us_ = 0;    // First edge after the last break (0: none yet).
    uint64_t next_ask_us_ = 0;  // kUnknown.
    uint64_t last_resolve_us_ = 0;
    struct ForwardSample {
        uint32_t received;  // rx_received
        uint64_t edges;     // EdgeSource::Count()
        uint64_t us;
    };
    ForwardSample forward_samples_[kForwardSamples] = {};
    size_t forward_newest_ = 0;
    bool forward_started_ = false;
    uint32_t forwardable_ = 0;
    uint32_t received_ = 0;
    LockInfo last_lock_;
    Stats stats_;
    uint32_t cycles_[kMaxAnalyzedEdges];
};
