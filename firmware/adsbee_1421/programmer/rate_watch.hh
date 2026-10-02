#pragma once

#include <stddef.h>
#include <stdint.h>

// Keeps the Programmer's UART at the module console's baud rate. Pure logic with no SDK dependencies so it can be
// host-tested (host_test/test_rate_watch.cc, which runs it against a bit-level model of the wire and the module);
// console_lock.cc and bridge.cc feed it the clock, the UART's error count and the edge timer, and carry out the
// actions it returns.
//
// The USB CDC baud rate is virtual: the host opens the port at any rate, and the UART runs at the console's rate. The
// module says "UU" at its console rate after every rate change, at boot and in answer to a break
// (firmware/adsbee_1421/ti/comms/console_autobaud.hh), so the Programmer never reads the host's AT commands:
//
//   Watch:   cheap hints that the console is at another rate: UART framing or break errors, or the newest
//            kMaxCheckedEdges edges showing data at another rate (Autobaud::OtherRateHint(): intervals too short or
//            off the bit grid, or a 'U' at another rate), checked every kCheckIntervalUs while edges come in and once
//            the line goes quiet. edge_capture.hh times every edge in hardware, so a hint can look back.
//   Resolve: on a hint, hold host data, look for the newest "UU" in the edges around and after the hint, snap its rate
//            to one the module's UART generates and retune. If none turns up within kResolveMs, and the edges don't
//            fit the current rate either, ask.
//   Ask:     when unsure (after a reset if the boot "UU" doesn't come, after a hint that didn't resolve and isn't a
//            module rebooting at kFactoryBaud, while the rate is unknown), send a break (TX low for kBreakUs, longer
//            than a frame at 9600 baud) and lock onto the
//            "UU" the module answers with. Up to kMaxAsks tries, then the rate is unknown: one more ask every
//            kUnknownAskIntervalMs, host data flowing in between.
//
// Host data waits only while a lock is in progress (resolve, ask, waiting for the boot "UU"). Console bytes reach the
// host kConsoleDelayUs after they arrive (ForwardableCount()): long enough for the watch to look at their edges first,
// so the bytes a "UU" at a slower rate makes at the old rate (0x00, 0x80, ... with no framing error) never get through.
// The console bytes received while a hint resolves stay in the UART's RX buffer (forwarded if the rate holds, dropped
// by the retune if it changes); during an ask or a reset they are dropped (the module's answer, or boot output at the
// wrong rate).
//
// Host rule, as on any wire: after AT+BAUD_RATE=CONSOLE,<n>, wait for the OK before sending the next command. The
// module drains the OK, reopens its UART at <n> and flushes its RX, so bytes sent before it announces the new rate
// are lost.

// The edge timer: edge_capture.hh on the Programmer, a simulated wire in the host tests.
class EdgeSource {
   public:
    // Edges captured since the capture started.
    virtual uint64_t Count() = 0;
    // Writes the times (clock cycles, modulo 2^32) of edges [first, to) to `cycles`, where first is `from` or, if more
    // than max_edges (or more than the source keeps) lie in between, the oldest edge that fits. Returns to - first.
    virtual size_t Read(uint64_t from, uint64_t to, uint32_t* cycles, size_t max_edges, uint64_t* first_index) = 0;
    // Edge 0 was falling. Edges alternate, so edge n is falling if n is even and this is true, or n odd and false.
    virtual bool FirstFalling() const = 0;
};

class RateWatch {
   public:
    static constexpr uint32_t kBreakUs = 2000;  // A frame at 9600 baud is 1.04 ms.
    // An ask sends a NUL first (Action::send_nul), then waits this long before the break: the NUL can start a frame at
    // a slower module rate, which has to end (1.04 ms at 9600 baud) before the break so the break reads as one.
    static constexpr uint32_t kPreBreakIdleUs = 1100;
    // An ask's answer: the module's main loop notices the break (a few ms in dense traffic), then 2.1 ms of "UU" at
    // 9600 baud.
    static constexpr uint32_t kAskAnswerMs = 30;
    static constexpr uint32_t kMaxAsks = 3;
    static constexpr uint32_t kUnknownAskIntervalMs = 1000;
    // Boot "UU": the module applies its saved rate about 50 ms after RESET_N is released.
    static constexpr uint32_t kBootAnswerMs = 150;
    // A hint's "UU": already on the line, or 2.1 ms long at 9600 baud.
    static constexpr uint32_t kResolveMs = 6;
    // Edges before the hint that may belong to the "UU": data at exactly half the old rate frames without errors half
    // the time and fits the old bit grid, so the hint can come a few frames after the "UU".
    static constexpr size_t kHintLookback = 256;
    static constexpr size_t kMaxAnalyzedEdges = 512;
    // The module prints at its factory rate from reset until it applies its saved rate (about 50 ms after reset). A
    // hint whose edges fit this rate is a module rebooting (AT+REBOOT, a watchdog reset, a power cycle): wait for its
    // boot "UU" instead of asking.
    static constexpr uint32_t kFactoryBaud = 1000000;
    // While the rate is unknown, a hint resolves at most this often, and never leads to an ask.
    static constexpr uint32_t kUnknownHintIntervalMs = 100;
    static constexpr size_t kMaxCheckedEdges = 32;  // Newest edges checked for a hint,
    static constexpr uint32_t kCheckIntervalUs = 100;  // at most this often (or once the line goes quiet).
    static constexpr uint32_t kQuietUs = 50;  // Analyze after this long without a new edge (or kAnalyzeEvery edges).
    // A 'U' at 9600 baud takes 1.04 ms to show its 10 edges, and a hint check runs every kCheckIntervalUs.
    static constexpr uint32_t kConsoleDelayUs = 1500;
    static constexpr uint32_t kForwardSampleUs = 20;
    static constexpr size_t kForwardSamples = 128;  // 2.56 ms of arrival times: more than kConsoleDelayUs.
    // On a retune the Programmer forwards the console bytes that arrived kKeepMarginUs or more before it saw the "UU"
    // start, and drops the rest. Both are seen by the bridge loop, a few tens of us late; the module is quiet for at
    // least 0.14 ms (measured) between its OK and the "UU", so the OK's last byte is in.
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
            kRetune,      // Forward the console bytes received up to `keep` (free-running count), then set the
                          // UART to `baud`, which drops the rest (received at the old rate after the change).
            kBreakStart,  // Drive TX low (after what the UART is still sending).
            kBreakEnd,    // Hand TX back to the UART.
        } kind = kNone;
        uint32_t baud = 0;
        uint32_t keep = 0;
        // Send a NUL at the UART's (new) rate: after every lock, and ahead of every break. The module ignores NULs, and
        // a character between two breaks lets its UART flag the second one (console_autobaud.hh).
        bool send_nul = false;
    };

    // Statistics (status messages, test builds' timing output).
    struct LockInfo {
        uint32_t measured_baud = 0;  // From the "UU".
        uint32_t baud = 0;           // The UART's new rate (the measurement snapped to a console rate).
        uint32_t elapsed_us = 0;     // From the reset, the first break or the hint.
        uint32_t asks = 0;           // Breaks sent for it.
        uint32_t answer_us = 0;      // Last break's start to the first edge after it (0: no break).
        uint32_t uu_to_lock_us = 0;  // The "UU"'s first edge to the lock.
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

    // Call every loop. sync_high: SYNC is high, so the module may be asleep: UART errors are ignored and no break is
    // sent (edges still count). rx_received: console bytes the UART has received so far (a free-running count).
    Action Poll(uint64_t now_us, bool sync_high, uint32_t rx_received);

    // The console bytes the bridge may forward (unless HoldConsoleData() or DropConsoleData()): up to this count (of
    // the rx_received passed to Poll()) they arrived at least kConsoleDelayUs ago.
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
    uint64_t checked_ = 0;          // Count() at the last hint check.
    uint64_t check_from_ = 0;       // Edges before this (before the last lock) aren't checked.
    uint64_t last_check_us_ = 0;
    uint64_t mark_ = 0;             // First edge the current lock may use.
    uint64_t analyzed_count_ = 0;   // Count() at the last analysis.
    uint64_t last_count_ = 0;
    uint64_t last_edge_us_ = 0;     // When Count() last changed.
    uint64_t lock_start_us_ = 0;
    uint64_t deadline_us_ = 0;
    uint64_t break_start_us_ = 0;
    uint64_t break_end_us_ = 0;
    bool break_pending_ = false;  // The NUL went out; the break starts at break_start_us_.
    bool sync_high_ = false;      // As of the last Poll().
    bool break_active_ = false;
    uint32_t asks_ = 0;             // In the current lock.
    uint64_t ask_start_us_ = 0;
    uint64_t answer_us_ = 0;        // First edge after the last break (0: none yet).
    uint64_t next_ask_us_ = 0;      // kUnknown.
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
