// Host tests for RateWatch (rate_watch.hh): the Programmer following the module console's baud rate, played through a
// bit-level model of both UART lines. Each line is a list of level changes in time; a PL011-like receiver samples it
// at its own rate (mid-bit, 8N1, framing errors, breaks), so bytes sent at one rate and received at another garble
// as on the bench. The module model answers breaks and announces rate changes with console_autobaud.hh, the code the
// firmware runs, and runs AT commands the way cppAT does. The Programmer model is bridge.cc's loop around the real
// RateWatch, reading edge times from the module's TX line.
#include <stdint.h>
#include <stdio.h>

#include <algorithm>
#include <deque>
#include <initializer_list>
#include <limits>
#include <string>
#include <vector>

#include "console_autobaud.hh"
#include "console_baud.hh"
#include "gtest/gtest.h"
#include "host_line_coding.hh"
#include "rate_watch.hh"

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr uint32_t kClockHz = 125000000;
constexpr double kProgrammerLoopNs = 5000;  // Bridge loop iteration.
constexpr double kModuleLoopNs = 100000;    // Module main loop iteration.
constexpr double kModuleReopenNs = 150000;  // Module: OK drained -> UART closed, reopened, "UU" queued.
constexpr double kHostTurnaroundNs = 2e6;   // Host: reply seen -> next command at the Programmer.

double BitNs(double baud) { return 1e9 / baud; }

// One direction of a UART line; idles high.
struct Wire {
    std::vector<double> t;    // Level changes, in time order.
    std::vector<bool> level;  // Level after each change.
    bool last = true;

    void Set(double when, bool to) {
        if (to == last) return;
        if (!t.empty() && when < t.back()) when = t.back();
        t.push_back(when);
        level.push_back(to);
        last = to;
    }
    bool LevelAt(double when) const {
        size_t i = std::upper_bound(t.begin(), t.end(), when) - t.begin();
        return i == 0 ? true : level[i - 1];
    }
    // First change after `when` (strictly) to `to`, or kInf.
    double NextChange(double when, bool to) const {
        size_t i = std::upper_bound(t.begin(), t.end(), when) - t.begin();
        for (; i < t.size(); i++) {
            if (level[i] == to) return t[i];
        }
        return kInf;
    }
    // Sends one 8N1 frame from `start`; returns its end.
    double SendByte(double start, double baud, uint8_t byte) {
        double bit = BitNs(baud);
        Set(start, false);
        for (int i = 0; i < 8; i++) Set(start + (1 + i) * bit, (byte >> i) & 1);
        Set(start + 9 * bit, true);
        return start + 10 * bit;
    }
};

// PL011-like receiver: a start bit is the line low (level, not edge) once idle, checked again mid-bit; data and stop
// bits are sampled mid-bit. A frame that is all zeros with the line low for a whole frame is a break: one NUL with
// the break flag, then nothing until the line goes high. As on the CC1314, a break that follows another with no
// valid character in between arrives as a plain NUL with no flag (console_autobaud.hh).
struct Receiver {
    const Wire* wire = nullptr;
    double baud = 1000000;
    double scan_from = 0;
    bool waiting_high = false;
    bool break_armed = true;

    struct Byte {
        uint8_t value;
        bool framing_error;
        bool break_error;
    };

    void SetBaud(double new_baud, double now) {
        baud = new_baud;
        if (scan_from < now) scan_from = now;
        waiting_high = false;
    }

    void Run(double now, std::vector<Byte>& out) {
        double bit = BitNs(baud);
        while (true) {
            if (waiting_high) {
                double rise = wire->LevelAt(scan_from) ? scan_from : wire->NextChange(scan_from, true);
                if (rise > now) return;
                scan_from = rise;
                waiting_high = false;
            }
            double start = wire->LevelAt(scan_from) ? wire->NextChange(scan_from, false) : scan_from;
            if (start + 10 * bit > now) {
                if (start != kInf)
                    scan_from = start;
                else if (wire->LevelAt(now))
                    scan_from = now;
                return;
            }
            if (wire->LevelAt(start + 0.5 * bit)) {  // False start.
                scan_from = start + 0.5 * bit;
                continue;
            }
            uint8_t value = 0;
            for (int i = 0; i < 8; i++) {
                if (wire->LevelAt(start + (1.5 + i) * bit)) value |= 1 << i;
            }
            bool stop = wire->LevelAt(start + 9.5 * bit);
            bool brk = !stop && value == 0 && wire->NextChange(start, true) >= start + 10 * bit;
            if (stop) break_armed = true;
            if (brk && !break_armed) {
                out.push_back({0, false, false});
            } else {
                out.push_back({value, !stop, brk});
            }
            if (brk) break_armed = false;
            scan_from = start + 9.5 * bit;
            if (brk) {
                scan_from = start + 10 * bit;
                waiting_high = true;
            }
        }
    }
};

// Byte transmitter with a queue, sending back to back at its rate.
struct Transmitter {
    Wire* wire = nullptr;
    double baud = 1000000;
    std::deque<uint8_t> queue;
    double free_at = 0;  // End of the frame on the wire.

    // Called every loop: frames that start before the next call go out now, back to back as DMA sends them.
    void Run(double now) {
        while (!queue.empty() && free_at <= now + kProgrammerLoopNs) {
            double start = std::max(free_at, now);
            free_at = wire->SendByte(start, baud, queue.front());
            queue.pop_front();
        }
    }
    void Queue(const std::string& s) {
        for (char c : s) queue.push_back((uint8_t)c);
    }
    bool Idle(double now) const { return queue.empty() && free_at <= now; }
};

// The ADSBee 1421 console.
struct Module {
    uint32_t baud = 1000000;        // Nominal console rate.
    uint32_t saved_baud = 1000000;  // AT+SETTINGS=SAVE.
    Receiver rx;
    Transmitter tx;
    std::deque<Receiver::Byte> fifo;
    bool break_flag = false;
    std::string line;
    std::vector<std::string> executed;
    double next_loop = 0;
    double switch_at = kInf;  // Pending rate switch (after the OK drains).
    uint32_t switch_to = 0;
    double boot_at = kInf;  // Pending boot steps.
    int boot_step = 0;
    bool silent = false;  // Old firmware: never says "UU".
    bool alive = true;
    double stream_every_ns = 0;  // Report output.
    double next_stream = 0;
    uint32_t stream_seq = 0;

    void Init(const Wire* in, Wire* out) {
        rx.wire = in;
        tx.wire = out;
        SetUart(baud, 0);
    }
    void SetUart(uint32_t nominal, double now) {
        baud = nominal;
        rx.SetBaud(ConsoleBaud::CC1314ActualBaud(nominal), now);
        tx.baud = ConsoleBaud::CC1314ActualBaud(nominal);
    }
    void Announce() {
        if (!silent) tx.Queue(ConsoleAutobaud::kAnswer);
    }
    // Reset: boot output at the factory rate, then the saved rate applied and announced.
    void Reset(double now) {
        tx.queue.clear();
        fifo.clear();
        line.clear();
        switch_at = kInf;
        SetUart(1000000, now);
        boot_at = now + 20e6;
        boot_step = 0;
    }
    void Execute(const std::string& cmd, double now) {
        executed.push_back(cmd);
        if (cmd.rfind("AT+BAUD_RATE=CONSOLE,", 0) == 0) {
            uint32_t n = (uint32_t)strtoul(cmd.c_str() + 21, nullptr, 10);
            if (!ConsoleBaud::IsSupported(n)) {
                tx.Queue("ERROR\r\n");
                return;
            }
            tx.Queue("OK\r\n");
            if (n != baud) {
                switch_to = n;
                switch_at = 0;  // When the OK has drained.
            }
        } else if (cmd == "AT+BAUD_RATE?") {
            tx.Queue("BAUD_RATE=CONSOLE," + std::to_string(baud) + "\r\n");
        } else if (cmd == "AT+UPTIME?") {
            tx.Queue("UPTIME=" + std::to_string((unsigned long)(now / 1e6)) + "\r\n");
        } else if (cmd == "AT+SETTINGS=SAVE") {
            saved_baud = baud;
            tx.Queue("OK\r\n");
        } else if (cmd == "AT+REBOOT") {
            Reset(now);
        } else if (cmd.rfind("AT+LOG_LEVEL=", 0) == 0) {
            tx.Queue("OK\r\n");
        } else {
            tx.Queue("ERROR\r\n");
        }
    }
    // cppAT: a line runs at '\n'; every AT+ command on it (CR-separated) runs.
    void RunLine(double now) {
        size_t pos = 0;
        while (pos < line.size()) {
            size_t end = line.find_first_of("\r\n", pos);
            if (end == std::string::npos) end = line.size();
            std::string cmd = line.substr(pos, end - pos);
            if (cmd.rfind("AT+", 0) == 0) Execute(cmd, now);
            pos = end + 1;
        }
        line.clear();
    }
    uint32_t answers = 0;
    void AnswerBreak() {
        break_flag = false;
        answers++;
        tx.queue.clear();  // DropQueuedConsoleTx(): the frame on the wire still finishes.
        Announce();
    }
    void Step(double now) {
        if (!alive) {  // Its transmitter still sends what a test queues by hand.
            tx.Run(now);
            return;
        }
        std::vector<Receiver::Byte> bytes;
        rx.Run(now, bytes);
        for (const Receiver::Byte& b : bytes) {
            if (b.break_error) break_flag = true;  // The raw interrupt status latches as the NUL enters the FIFO.
            fifo.push_back(b);
        }
        if (boot_at <= now) {
            if (boot_step == 0) {
                tx.Queue("\r\nADSBee 1421 booting\r\n");  // At the factory rate, before Apply().
                boot_step = 1;
                boot_at = now + 25e6;
            } else if (tx.Idle(now)) {
                SetUart(saved_baud, now);  // Apply(), then AnnounceBootRate(): one "UU" either way.
                fifo.clear();
                break_flag = false;
                Announce();
                boot_at = kInf;
            }
        }
        if (switch_at == 0 && tx.Idle(now)) switch_at = now + kModuleReopenNs;
        if (switch_at != 0 && switch_at <= now) {
            SetUart(switch_to, now);
            fifo.clear();
            break_flag = false;
            Announce();
            switch_at = kInf;
        }
        // SetBaudRate() blocks the main loop from the OK until the "UU" is queued.
        if (now >= next_loop && boot_at == kInf && switch_at == kInf) {
            next_loop = now + kModuleLoopNs;
            if (break_flag) AnswerBreak();
            while (!fifo.empty() && switch_at == kInf) {
                Receiver::Byte b = fifo.front();
                fifo.pop_front();
                char c = (char)b.value;
                if (c == '\0' && break_flag) AnswerBreak();
                if (ConsoleAutobaud::IgnoredConsoleByte(c)) continue;
                line.push_back(c);
                if (c == '\n') RunLine(now);
            }
            if (stream_every_ns > 0 && now >= next_stream) {
                next_stream = now + stream_every_ns;
                char report[48];
                snprintf(report, sizeof(report), "#MDS%06u;8D4840D6202CC371C32CE0576098\r\n", (unsigned)stream_seq++);
                tx.Queue(report);
            }
        }
        tx.Run(now);
    }
};

// The module's TX line as the Programmer's edge timer sees it: 125 MHz, sampled every 2 cycles.
struct WireEdges : public EdgeSource {
    const Wire* wire = nullptr;
    double now = 0;
    uint64_t Count() override { return std::upper_bound(wire->t.begin(), wire->t.end(), now) - wire->t.begin(); }
    size_t Read(uint64_t from, uint64_t to, uint32_t* cycles, size_t max_edges, uint64_t* first_index) override {
        uint64_t first = to - from > max_edges ? to - max_edges : from;
        *first_index = first;
        for (uint64_t i = first; i < to; i++) {
            uint64_t c = (uint64_t)(wire->t[i] * (kClockHz / 1e9));
            cycles[i - first] = (uint32_t)(c & ~1ull);
        }
        return (size_t)(to - first);
    }
    bool FirstFalling() const override { return true; }
};

// bridge.cc's loop.
struct Programmer {
    WireEdges edges;
    RateWatch watch{edges, kClockHz};
    uint32_t baud = 1000000;
    Receiver rx;
    Transmitter tx;
    std::deque<uint8_t> rx_ring;
    uint32_t rx_received = 0;     // Free-running, as target_uart.cc's ring head.
    std::deque<uint8_t> host_in;  // CDC FIFO.
    std::string host_out;
    uint32_t errors = 0;
    bool tx_low = false;
    bool sync_high = false;
    std::vector<RateWatch::Action> actions;

    void Init(const Wire* in, Wire* out) {
        edges.wire = in;
        rx.wire = in;
        tx.wire = out;
        SetUart(baud, 0);
    }
    void SetUart(uint32_t nominal, double now) {
        baud = nominal;
        rx.SetBaud(ProgrammerActualBaud(nominal), now);
        tx.baud = ProgrammerActualBaud(nominal);
        rx_ring.clear();
    }
    void Apply(const RateWatch::Action& action, double now) {
        if (action.kind != RateWatch::Action::kNone) actions.push_back(action);
        switch (action.kind) {
            case RateWatch::Action::kRetune:
                for (int32_t n = (int32_t)(action.keep - (rx_received - (uint32_t)rx_ring.size()));
                     n > 0 && !rx_ring.empty(); n--) {
                    host_out.push_back((char)rx_ring.front());
                    rx_ring.pop_front();
                }
                SetUart(action.baud, now);
                errors = 0;
                break;
            case RateWatch::Action::kBreakStart:
                if (!tx_low) {
                    tx.queue.clear();
                    tx.wire->Set(std::max(now, tx.free_at), false);
                    tx_low = true;
                }
                break;
            case RateWatch::Action::kBreakEnd:
                if (tx_low) {
                    tx.wire->Set(now, true);
                    tx.free_at = now;
                    tx_low = false;
                }
                break;
            case RateWatch::Action::kNone:
                break;
        }
        if (action.send_nul) {
            tx.queue.push_front(0);  // Ahead of host data; after a break that is still running.
            if (!tx_low) tx.Run(now);
        }
    }
    void Step(double now) {
        edges.now = now;
        std::vector<Receiver::Byte> bytes;
        rx.Run(now, bytes);
        for (const Receiver::Byte& b : bytes) {
            if (b.framing_error || b.break_error) errors++;
            rx_ring.push_back(b.value);
            rx_received++;
        }
        watch.OnRxErrors(errors);
        errors = 0;
        Apply(watch.Poll((uint64_t)(now / 1000), sync_high, rx_received), now);
        uint32_t forwardable = watch.Forwardable();
        if (watch.DropConsoleData()) {
            rx_ring.clear();
        } else if (!watch.HoldConsoleData()) {
            uint32_t consumed = rx_received - (uint32_t)rx_ring.size();
            int32_t n = (int32_t)(forwardable - consumed);
            for (; n > 0 && !rx_ring.empty(); n--) {
                host_out.push_back((char)rx_ring.front());
                rx_ring.pop_front();
            }
        }
        if (!watch.HoldHostData() && !tx_low) {
            while (!host_in.empty() && tx.queue.size() < 64) {
                tx.queue.push_back(host_in.front());
                host_in.pop_front();
            }
        }
        if (!tx_low) tx.Run(now);
    }
};

struct Sim {
    Wire to_module, to_programmer;
    Module module;
    Programmer programmer;
    double now = 0;

    explicit Sim(uint32_t module_baud = 1000000, uint32_t programmer_baud = 1000000) {
        module.baud = module.saved_baud = module_baud;
        programmer.baud = programmer_baud;
        module.Init(&to_module, &to_programmer);
        programmer.Init(&to_programmer, &to_module);
        programmer.watch.Start(programmer_baud, 0);
    }
    // The bridge loop takes 5 to 65 us (USB servicing, forwarding), so the Programmer sees edges and bytes late.
    double next_programmer = 0;
    uint32_t jitter = 12345;
    void Run(double ms) {
        double end = now + ms * 1e6;
        while (now < end) {
            now += kProgrammerLoopNs;
            module.Step(now);
            if (now >= next_programmer) {
                programmer.Step(now);
                jitter = jitter * 1103515245u + 12345u;
                next_programmer = now + kProgrammerLoopNs * (1 + (jitter >> 16) % 13);
            }
        }
    }
    // Runs until `pred` or `ms` passes; returns whether pred came true.
    template <typename Pred>
    bool RunUntil(Pred pred, double ms) {
        double end = now + ms * 1e6;
        while (now < end) {
            if (pred()) return true;
            Run(kProgrammerLoopNs / 1e6);
        }
        return pred();
    }
    // The Programmer's startup lock (ConsoleLock(kReset)): reset into the application, wait for the lock.
    bool ResetLock(double ms = 500) {
        module.Reset(now);
        programmer.watch.OnAppReset((uint64_t)(now / 1000));
        return RunUntil([&] { return !programmer.watch.Locking() && !programmer.watch.BreakActive(); }, ms);
    }
    void HostWrite(const std::string& s) { programmer.host_in.insert(programmer.host_in.end(), s.begin(), s.end()); }
    // Sends a command and waits for a reply line containing `expect` (or `ms`). Returns what the host read.
    std::string Command(const std::string& cmd, const std::string& expect, double ms = 300) {
        size_t start = programmer.host_out.size();
        HostWrite(cmd);
        RunUntil([&] { return programmer.host_out.find(expect, start) != std::string::npos; }, ms);
        Run(kHostTurnaroundNs / 1e6);
        return programmer.host_out.substr(start);
    }
    std::string QueryBaud() { return Command("AT+BAUD_RATE?\r\n", "\r\n", 500); }
    static std::string BaudLine(uint32_t baud) { return "BAUD_RATE=CONSOLE," + std::to_string(baud) + "\r\n"; }
};

const std::initializer_list<uint32_t> kRates = {9600, 57600, 115200, 123457, 460800, 1000000, 2000000, 3000000};

}  // namespace

// Startup: reset into the application at any saved rate, lock onto the boot "UU" in about 50 ms, no break needed.
TEST(RateWatch, ResetLock) {
    for (uint32_t saved : kRates) {
        Sim sim(saved, 1000000);
        ASSERT_TRUE(sim.ResetLock()) << saved;
        EXPECT_EQ(sim.programmer.watch.phase(), RateWatch::Phase::kLocked) << saved;
        EXPECT_EQ(sim.programmer.baud, ConsoleBaud::CC1314ActualBaud(saved)) << saved;
        const RateWatch::LockInfo& lock = sim.programmer.watch.last_lock();
        EXPECT_EQ(lock.asks, 0u) << saved;
        EXPECT_LT(lock.elapsed_us, 60000u) << saved;
        EXPECT_EQ(sim.programmer.host_out, "") << saved;  // Boot output at the wrong rate and the "UU" are dropped.
        EXPECT_EQ(sim.QueryBaud(), Sim::BaudLine(saved)) << saved;
    }
}

// The host changes the rate on the open port, waits for the OK, and talks on. The Programmer follows the "UU"; the host
// sees exactly the replies, no "UU" and no garbage.
TEST(RateWatch, FollowsRateChange) {
    for (uint32_t from : kRates) {
        for (uint32_t to : kRates) {
            if (from == to) continue;
            Sim sim(from, from);
            std::string reply = sim.Command("AT+BAUD_RATE=CONSOLE," + std::to_string(to) + "\r\n", "OK\r\n");
            EXPECT_EQ(reply, "OK\r\n") << from << " -> " << to;
            EXPECT_EQ(sim.QueryBaud(), Sim::BaudLine(to)) << from << " -> " << to;
            EXPECT_EQ(sim.programmer.watch.stats().asks, 0u) << from << " -> " << to;
            EXPECT_EQ(sim.programmer.baud, ConsoleBaud::CC1314ActualBaud(to)) << from << " -> " << to;
            EXPECT_EQ(sim.Command("AT+UPTIME?\r\n", "\r\n").rfind("UPTIME=", 0), 0u) << from << " -> " << to;
        }
    }
}

// Former limit 1: an OK still owed to an earlier command on the same line. Both OKs arrive intact; the retune follows
// the "UU", whatever the line held before.
TEST(RateWatch, EarlierOkOnTheSameLine) {
    for (uint32_t to : {9600u, 115200u, 3000000u}) {
        Sim sim(1000000, 1000000);
        std::string reply =
            sim.Command("AT+LOG_LEVEL=INFO\rAT+BAUD_RATE=CONSOLE," + std::to_string(to) + "\r\n", "OK\r\nOK\r\n");
        EXPECT_EQ(reply, "OK\r\nOK\r\n") << to;
        EXPECT_EQ(sim.QueryBaud(), Sim::BaudLine(to)) << to;
    }
}

// Former limit 2: a reset with SYNC high when the module's bootloader backdoor is off boots the application at its
// saved rate; the Programmer, at the ROM bootloader's 1 M, follows its boot "UU", which comes while the Programmer
// still holds SYNC high (ModemLines::kBackdoorHoldMs).
TEST(RateWatch, BackdoorOffBootloaderReset) {
    for (uint32_t saved : {9600u, 115200u, 921600u, 3000000u}) {
        Sim sim(saved, saved);
        sim.programmer.SetUart(1000000, sim.now);
        sim.programmer.watch.OnBootloaderReset(1000000, (uint64_t)(sim.now / 1000));
        sim.module.Reset(sim.now);
        sim.programmer.sync_high = true;
        sim.Run(300);
        sim.programmer.sync_high = false;
        sim.Run(10);
        EXPECT_EQ(sim.programmer.watch.phase(), RateWatch::Phase::kLocked) << saved;
        EXPECT_EQ(sim.programmer.baud, ConsoleBaud::CC1314ActualBaud(saved)) << saved;
        EXPECT_EQ(sim.programmer.watch.stats().asks, 0u) << saved;  // The ROM bootloader is never sent a break.
        EXPECT_EQ(sim.QueryBaud(), Sim::BaudLine(saved)) << saved;
    }
}

// A real ROM bootloader session at 1 M is left alone: its traffic fits the rate, nothing retunes or asks.
TEST(RateWatch, RomBootloaderLeftAlone) {
    Sim sim(115200, 115200);
    sim.programmer.SetUart(1000000, sim.now);
    sim.programmer.watch.OnBootloaderReset(1000000, 0);
    sim.module.alive = false;  // The ROM bootloader: answers ACKs at 1 M.
    sim.module.tx.baud = 1000000;
    for (int i = 0; i < 50; i++) {
        sim.module.tx.queue.push_back(0x00);
        sim.module.tx.queue.push_back(0xCC);
        sim.Run(2);
    }
    sim.Run(2);  // RateWatch::kConsoleDelayUs.
    EXPECT_EQ(sim.programmer.watch.phase(), RateWatch::Phase::kBootloader);
    EXPECT_TRUE(sim.programmer.actions.empty());
    EXPECT_EQ(sim.programmer.host_out.size(), 100u);
}

// AT+REBOOT: the module comes back at its saved rate and the Programmer follows its boot "UU" (unsaved change lost,
// saved change kept).
TEST(RateWatch, Reboot) {
    {
        Sim sim(1000000, 1000000);
        sim.Command("AT+BAUD_RATE=CONSOLE,460800\r\n", "OK\r\n");
        sim.Command("AT+REBOOT\r\n", "", 1);
        sim.Run(150);
        EXPECT_EQ(sim.QueryBaud(), Sim::BaudLine(1000000));
    }
    {
        Sim sim(1000000, 1000000);
        sim.Command("AT+BAUD_RATE=CONSOLE,57600\r\n", "OK\r\n");
        EXPECT_EQ(sim.Command("AT+SETTINGS=SAVE\r\n", "OK\r\n"), "OK\r\n");
        sim.Command("AT+REBOOT\r\n", "", 1);
        sim.Run(150);
        EXPECT_EQ(sim.QueryBaud(), Sim::BaudLine(57600));
        EXPECT_EQ(sim.programmer.watch.stats().asks, 0u);
    }
}

// Break -> "UU": the Programmer asks with no idea of the rate (a module that changed rate without saying so), locks
// in one break, and the break leaves no trace in the module's AT parser: a command split around it still runs.
TEST(RateWatch, BreakLock) {
    for (uint32_t hidden : kRates) {
        for (uint32_t programmer : {9600u, 1000000u, 3000000u}) {
            if (hidden == programmer) continue;
            Sim sim(hidden, programmer);
            sim.HostWrite("AT+UPT");
            sim.Run(1);
            sim.programmer.watch.Ask((uint64_t)(sim.now / 1000));
            ASSERT_TRUE(sim.RunUntil(
                [&] {
                    return sim.programmer.watch.phase() == RateWatch::Phase::kLocked &&
                           !sim.programmer.watch.BreakActive();
                },
                100))
                << hidden << " from " << programmer;
            EXPECT_EQ(sim.programmer.baud, ConsoleBaud::CC1314ActualBaud(hidden));
            EXPECT_EQ(sim.programmer.watch.last_lock().asks, 1u);
            EXPECT_LT(sim.programmer.watch.last_lock().elapsed_us, 6000u) << hidden;
            // The partial line, if it reached the module at its rate, continues; otherwise it is garbage on its own
            // line.
            sim.module.line.clear();
            EXPECT_EQ(sim.Command("\r\nAT+UPTIME?\r\n", "\r\n").rfind("UPTIME=", 0), 0u) << hidden;
        }
    }
    // At the right rate the break leaves the partial command intact.
    Sim sim(115200, 115200);
    sim.HostWrite("AT+UPT");
    sim.Run(1);
    sim.programmer.watch.Ask((uint64_t)(sim.now / 1000));
    sim.Run(20);
    EXPECT_EQ(sim.programmer.watch.phase(), RateWatch::Phase::kLocked);
    EXPECT_EQ(sim.Command("IME?\r\n", "\r\n").rfind("UPTIME=", 0), 0u);
    EXPECT_EQ(sim.module.executed.back(), "AT+UPTIME?");
}

// A rate change the module doesn't announce (old firmware): the Programmer sees errors or short intervals, finds no
// "UU", the edges don't fit its rate, so it asks; that module never answers, and the Programmer reports the rate
// unknown and asks again every second while host data flows.
TEST(RateWatch, SilentModuleIsUnknown) {
    Sim sim(1000000, 1000000);
    sim.module.silent = true;
    sim.module.stream_every_ns = 1e6;
    sim.module.SetUart(115200, sim.now);
    sim.Run(200);
    EXPECT_EQ(sim.programmer.watch.phase(), RateWatch::Phase::kUnknown);
    EXPECT_EQ(sim.programmer.watch.stats().asks, RateWatch::kMaxAsks);
    sim.Run(1100);
    EXPECT_GE(sim.programmer.watch.stats().asks, 2 * RateWatch::kMaxAsks);
    // Once it answers, the next ask locks.
    sim.module.silent = false;
    sim.Run(1100);
    EXPECT_EQ(sim.programmer.watch.phase(), RateWatch::Phase::kLocked);
    EXPECT_EQ(sim.programmer.baud, ConsoleBaud::CC1314ActualBaud(115200));
}

// The same hidden change with a module that answers breaks: one ask after the hint.
TEST(RateWatch, HiddenChangeAsks) {
    for (uint32_t to : {9600u, 57600u, 460800u, 2000000u}) {
        Sim sim(1000000, 1000000);
        sim.module.stream_every_ns = 2e6;
        sim.Run(10);
        sim.module.SetUart(to, sim.now);  // No announcement.
        sim.Run(100);
        EXPECT_EQ(sim.programmer.watch.phase(), RateWatch::Phase::kLocked) << to;
        EXPECT_EQ(sim.programmer.baud, ConsoleBaud::CC1314ActualBaud(to)) << to;
        EXPECT_GE(sim.programmer.watch.stats().asks, 1u) << to;
    }
}

// Report output streaming across a rate change, with no host involvement beyond the command: reports before and after
// it reach the host intact; only those on the wire around the switch are lost.
TEST(RateWatch, StreamingAcrossChange) {
    for (uint32_t from : {115200u, 1000000u, 3000000u}) {
        for (uint32_t to : {57600u, 921600u, 3000000u}) {
            if (from == to) continue;
            Sim sim(from, from);
            // A 44-byte report every 1 ms, or as often as the slower rate carries with room to spare.
            sim.module.stream_every_ns = std::max(1e6, 1.5 * 440 * 1e9 / std::min(from, to));
            sim.Run(100);
            sim.HostWrite("AT+BAUD_RATE=CONSOLE," + std::to_string(to) + "\r\n");
            sim.Run(400);
            const std::string& out = sim.programmer.host_out;
            // Count intact reports and look for the first missing sequence number.
            unsigned last = 0, intact = 0, missing = 0;
            bool first = true;
            for (size_t p = out.find("#MDS"); p != std::string::npos; p = out.find("#MDS", p + 1)) {
                unsigned seq = 0;
                if (sscanf(out.c_str() + p, "#MDS%06u;8D4840D6202CC371C32CE0576098\r\n", &seq) != 1) continue;
                if (out.compare(p + 10, 30, ";8D4840D6202CC371C32CE0576098\r") != 0) continue;
                if (!first && seq > last + 1) missing += seq - last - 1;
                first = false;
                last = seq;
                intact++;
            }
            EXPECT_GT(intact, 20u) << from << " -> " << to;
            EXPECT_LE(missing, 2u) << from << " -> " << to;
            EXPECT_GE(last + 3, sim.module.stream_seq) << from << " -> " << to;  // On the wire, or in the delay.
            EXPECT_EQ(sim.programmer.baud, ConsoleBaud::CC1314ActualBaud(to)) << from << " -> " << to;
            EXPECT_EQ(sim.programmer.watch.stats().asks, 0u) << from << " -> " << to;
        }
    }
}

// Spurious hints (a framing error at the right rate) change nothing: the edges fit the rate.
TEST(RateWatch, SpuriousHint) {
    Sim sim(115200, 115200);
    sim.module.stream_every_ns = 1e6;
    sim.Run(20);
    sim.programmer.watch.OnRxErrors(1);
    sim.Run(50);
    EXPECT_EQ(sim.programmer.watch.phase(), RateWatch::Phase::kLocked);
    EXPECT_EQ(sim.programmer.watch.stats().hints, 1u);
    EXPECT_EQ(sim.programmer.watch.stats().asks, 0u);
    EXPECT_TRUE(sim.programmer.actions.empty());
    EXPECT_EQ(sim.programmer.host_out.find("#MDS"), 0u);
}

// While SYNC is high the module may be asleep: UART errors are ignored and no break goes out, but a "UU" still counts.
TEST(RateWatch, SyncHigh) {
    {
        Sim sim(115200, 115200);
        sim.programmer.sync_high = true;
        sim.programmer.watch.OnRxErrors(5);
        sim.module.silent = true;
        sim.module.SetUart(57600, sim.now);  // No "UU": the edges hint, nothing resolves, nothing is asked.
        sim.module.stream_every_ns = 1e6;
        sim.Run(100);
        EXPECT_EQ(sim.programmer.watch.stats().asks, 0u);
        EXPECT_EQ(sim.programmer.baud, 115200u);
        sim.programmer.sync_high = false;  // Awake: the next hint asks; this module never answers.
        sim.Run(200);
        EXPECT_GE(sim.programmer.watch.stats().asks, 1u);
    }
    {
        Sim sim(115200, 115200);
        sim.programmer.sync_high = true;
        sim.Command("AT+BAUD_RATE=CONSOLE,460800\r\n", "OK\r\n");  // Announced: followed.
        EXPECT_EQ(sim.programmer.baud, ConsoleBaud::CC1314ActualBaud(460800));
        EXPECT_EQ(sim.programmer.watch.stats().asks, 0u);
    }
}

// Host data waits only while a lock runs, and in order.
TEST(RateWatch, HostDataHeldDuringLock) {
    Sim sim(1000000, 1000000);
    sim.module.Reset(sim.now);
    sim.programmer.watch.OnAppReset((uint64_t)(sim.now / 1000));
    sim.HostWrite("AT+UPTIME?\r\n");
    sim.Run(5);
    EXPECT_EQ(sim.programmer.host_in.size(), 12u);  // Held: the module is booting.
    sim.Run(100);
    EXPECT_TRUE(sim.programmer.host_in.empty());
    EXPECT_EQ(sim.programmer.host_out.rfind("UPTIME=", 0), 0u);
}

// A module that isn't there: the lock gives up after kMaxAsks breaks.
TEST(RateWatch, NoModule) {
    Sim sim(1000000, 1000000);
    sim.module.alive = false;
    EXPECT_TRUE(sim.ResetLock(1000));
    EXPECT_EQ(sim.programmer.watch.phase(), RateWatch::Phase::kUnknown);
    EXPECT_EQ(sim.programmer.watch.stats().asks, RateWatch::kMaxAsks);
    EXPECT_EQ(sim.programmer.watch.stats().failed_locks, 1u);
    // Host data flows while the rate is unknown.
    sim.HostWrite("AT\r\n");
    sim.Run(5);
    EXPECT_TRUE(sim.programmer.host_in.empty());
}

// Asks back to back with no host data in between: the NULs around each break let the module's UART flag every one.
TEST(RateWatch, RepeatedBreaks) {
    for (uint32_t baud : {9600u, 115200u, 3000000u}) {
        Sim sim(baud, baud);
        for (int i = 0; i < 5; i++) {
            sim.programmer.watch.Ask((uint64_t)(sim.now / 1000));
            sim.Run(200);
            EXPECT_EQ(sim.programmer.watch.phase(), RateWatch::Phase::kLocked) << baud << " #" << i;
            EXPECT_EQ(sim.programmer.watch.last_lock().asks, 1u) << baud << " #" << i;
        }
        EXPECT_EQ(sim.module.answers, 5u) << baud;
        EXPECT_EQ(sim.module.line, "") << baud;  // The NULs never reached the AT parser.
        EXPECT_EQ(sim.QueryBaud(), Sim::BaudLine(baud)) << baud;
    }
}

// The model's UART behaves like the CC1314's: without the NULs, the second break goes unflagged.
TEST(RateWatch, RepeatedBreakNeedsACharacter) {
    Wire w;
    Receiver rx;
    rx.wire = &w;
    rx.baud = 115200;
    for (double t : {1e6, 3e6}) {
        w.Set(t, false);
        w.Set(t + 2e6, true);  // 2 ms.
    }
    std::vector<Receiver::Byte> bytes;
    rx.Run(10e6, bytes);
    ASSERT_EQ(bytes.size(), 2u);
    EXPECT_TRUE(bytes[0].break_error);
    EXPECT_FALSE(bytes[1].break_error);
    EXPECT_EQ(bytes[1].value, 0u);
}

// The module side alone.
TEST(ConsoleAutobaud, Constants) {
    EXPECT_EQ(std::string(ConsoleAutobaud::kAnswer), "UU");
    EXPECT_EQ(ConsoleAutobaud::kAnswerLen, 2u);
    EXPECT_TRUE(ConsoleAutobaud::IgnoredConsoleByte('\0'));
    EXPECT_FALSE(ConsoleAutobaud::IgnoredConsoleByte('A'));
}
