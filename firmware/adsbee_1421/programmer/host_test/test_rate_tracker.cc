// Host tests for the console baud range (ti/comms/console_baud.hh) and RateTracker: host commands that move the
// module console's rate, played through a byte-level model of the host, the bridge loop (bridge.cc) and the module.
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <deque>
#include <initializer_list>
#include <string>

#include "console_baud.hh"
#include "gtest/gtest.h"
#include "host_line_coding.hh"
#include "rate_tracker.hh"

// ---- Baud arithmetic ----

// The divider code of driverlib UARTConfigSetExpClk() (CC13x4, no clamping) and pico-sdk uart_set_baudrate() (RP2040,
// clamps the integer part to 1..65535), transcribed independently of console_baud.hh.
static uint32_t TiActual(uint32_t clk, uint32_t baud) {
    uint32_t div = (((clk * 8) / baud) + 1) / 2;
    uint32_t ibrd = div / 64, fbrd = div % 64;
    if (ibrd == 0 || ibrd > 0xFFFF) return 0;
    return (uint32_t)((uint64_t)clk * 4 / (64 * ibrd + fbrd));
}
static uint32_t PicoActual(uint32_t clk, uint32_t baud) {
    uint32_t div = (8 * clk / baud) + 1;
    uint32_t ibrd = div >> 7, fbrd;
    if (ibrd == 0) {
        ibrd = 1;
        fbrd = 0;
    } else if (ibrd >= 65535) {
        ibrd = 65535;
        fbrd = 0;
    } else {
        fbrd = (div & 0x7f) >> 1;
    }
    return (4 * clk) / (64 * ibrd + fbrd);
}

// PL011 divider matches driverlib and pico-sdk.
TEST(ConsoleBaud, DividerMatchesDrivers) {
    for (uint32_t baud = ConsoleBaud::kMin; baud <= ConsoleBaud::kMax; baud += 997) {
        EXPECT_EQ(ConsoleBaud::CC1314ActualBaud(baud), TiActual(ConsoleBaud::kCC1314UartClockHz, baud));
        EXPECT_EQ(ProgrammerActualBaud(baud), PicoActual(kProgrammerUartClockHz, baud));
    }
    EXPECT_EQ(ConsoleBaud::CC1314ActualBaud(115200), 115176u);  // Divisor 1667/64.
    EXPECT_EQ(ConsoleBaud::CC1314ActualBaud(3000000), 3000000u);
    EXPECT_EQ(ConsoleBaud::CC1314ActualBaud(3000001), 3000000u);  // Rounds to the same divisor.
    EXPECT_EQ(ConsoleBaud::CC1314ActualBaud(3100000), 0u);        // Integer part 0: can't be generated.
    EXPECT_EQ(ConsoleBaud::Pl011ActualBaud(48000000, 45u), 0u);   // Integer part > 65535.
    EXPECT_EQ(ConsoleBaud::Pl011ActualBaud(48000000, 0u), 0u);
}

// Console accepts 9600..3000000 within 2%, and the Programmer's UART matches every one of them.
TEST(ConsoleBaud, ConsoleRange) {
    // The old whitelist stays valid, so saved settings keep working.
    for (uint32_t baud : {115200u, 230400u, 460800u, 921600u, 1000000u}) {
        EXPECT_TRUE(ConsoleBaud::IsSupported(baud));
    }
    for (uint32_t baud : {9600u, 19200u, 38400u, 57600u, 76800u, 250000u, 500000u, 2000000u, 3000000u, 123457u}) {
        EXPECT_TRUE(ConsoleBaud::IsSupported(baud));
    }
    for (uint32_t baud : {0u, 300u, 1200u, 9599u, 3000001u, 4000000u, 0xDEADBEEu, 0xFFFFFFFFu}) {
        EXPECT_FALSE(ConsoleBaud::IsSupported(baud));
    }
    // Every rate in range is generated well inside the 2% limit, on both ends of the link.
    uint32_t worst_cc1314 = 0, worst_link = 0;
    for (uint32_t baud = ConsoleBaud::kMin; baud <= ConsoleBaud::kMax; baud += 101) {
        uint32_t cc1314 = ConsoleBaud::CC1314ActualBaud(baud);
        uint32_t e = ConsoleBaud::ErrorPpm(baud, cc1314);
        if (e > worst_cc1314) worst_cc1314 = e;
        e = ConsoleBaud::ErrorPpm(cc1314, ProgrammerActualBaud(baud));
        if (e > worst_link) worst_link = e;
        EXPECT_TRUE(ConsoleBaud::IsSupported(baud));
    }
    printf("  worst CC1314 error %u ppm, worst CC1314 vs Programmer mismatch %u ppm\n", (unsigned)worst_cc1314,
           (unsigned)worst_link);
    EXPECT_TRUE(worst_cc1314 < 8000);
    EXPECT_TRUE(worst_link < 11000);  // ~1%, near 3 M where the RP2040 divisor is smallest.
}

// ---- RateTracker ----

namespace {

// The host, the bridge loop as in BridgeRun() and a model of the module console. A byte crosses the UART intact only
// when both ends run at the same rate; otherwise it arrives as garbage with a framing error.
struct Sim {
    RateTracker t;
    uint32_t now = 5000;
    bool sync_high = false;

    // Module.
    uint32_t console = 1000000;  // Live console rate.
    uint32_t saved = 1000000;    // Saved console rate (AT+SETTINGS=SAVE).
    bool accept = true;          // AT+BAUD_RATE=CONSOLE,<n> in range is accepted.
    bool answer = true;          // The module answers at all (false: the OK is lost).
    std::string module_line;
    std::string module_got;  // Every byte the module received.
    // After a switch the module reopens its UART and flushes RX: bytes arriving until then are lost.
    uint32_t module_deaf_until = 0;
    int module_lost = 0;

    // Programmer.
    uint32_t uart = 1000000;
    std::deque<std::pair<char, uint32_t>> wire;  // Module -> Programmer: byte and the rate it was sent at.
    uint8_t pending[64];
    size_t pending_len = 0, pending_off = 0;
    int wake_locks = 0, reset_locks = 0, retunes = 0;
    uint32_t errors = 0;

    // Host.
    std::string host_in;   // Written by the host, not yet read from the CDC FIFO.
    std::string host_out;  // Received by the host.

    void Start() {
        uart = console;
        t.Start(console, now);
    }

    void ModuleOut(const std::string& text) {
        for (char c : text) wire.push_back({c, console});
    }
    void ModuleRun(const std::string& line) {
        size_t at = line.find("AT");
        if (at == std::string::npos) return;
        std::string cmd = line.substr(at);
        while (!cmd.empty() && cmd.back() == '\r') cmd.pop_back();
        static const std::string kBaud = "AT+BAUD_RATE=CONSOLE,";
        if (cmd.compare(0, kBaud.size(), kBaud) == 0) {
            uint32_t baud = (uint32_t)strtoul(cmd.c_str() + kBaud.size(), nullptr, 10);
            if (!ConsoleBaud::IsSupported(baud) || !accept) {
                ModuleOut("ERROR Baud not supported.\r\n");
                return;
            }
            if (answer) ModuleOut("OK\r\n");
            console = baud;
            module_deaf_until = now + 3;
        } else if (cmd == "AT+BAUD_RATE?") {
            ModuleOut("BAUD_RATE=CONSOLE," + std::to_string(console) + "\r\n");
        } else if (cmd == "AT+SETTINGS=SAVE") {
            saved = console;
            ModuleOut("OK\r\n");
        } else if (cmd == "AT+SETTINGS=RESET") {
            ModuleOut("OK\r\n");
            console = saved = 1000000;
            module_deaf_until = now + 3;
        } else if (cmd == "AT+REBOOT") {
            console = saved;  // Reboots; its boot output is lost in the reset the bridge does.
        } else if (cmd.compare(0, 3, "AT+") == 0 && isupper((unsigned char)cmd[3])) {
            ModuleOut("OK\r\n");
        } else {
            ModuleOut("CppAT::ParseMessage: Unable to match AT command.\r\n");
        }
    }
    void ModuleReceive(char c, uint32_t rate) {
        if (now < module_deaf_until) {
            module_lost++;
            return;
        }
        if (rate != console) c = '\x01';
        module_got += c;
        if (c == '\n') {
            ModuleRun(module_line);
            module_line.clear();
        } else {
            module_line += c;
        }
    }

    void Lock(bool reset, uint32_t fallback) {
        (void)fallback;
        if (reset) {
            reset_locks++;
            console = saved;
        } else {
            wake_locks++;
        }
        wire.clear();
        uart = console;
        t.OnLocked(console, now);
    }

    // One pass of the bridge loop.
    void Loop() {
        if (errors) t.OnRxErrors(errors, now);
        errors = 0;
        RateTracker::Step step = t.Poll(now, sync_high);
        if (step.kind == RateTracker::Step::kWakeLock) return Lock(false, step.baud);
        if (step.kind == RateTracker::Step::kResetAndLock) return Lock(true, step.baud);
        EXPECT_NE(step.kind, RateTracker::Step::kRetune);  // Only right after OnConsoleBytes().

        uint8_t buf[64];
        size_t len = 0;
        while (len < sizeof(buf) && !wire.empty()) {
            auto [c, rate] = wire.front();
            wire.pop_front();
            if (rate != uart) {
                c = '~';
                errors++;
            }
            buf[len++] = (uint8_t)c;
        }
        if (len > 0) {
            len = t.OnConsoleBytes(buf, len, now);
            step = t.Poll(now, sync_high);
            if (step.kind == RateTracker::Step::kRetune) {
                retunes++;
                uart = step.baud;
                wire.clear();  // TargetUartSetBaud() flushes the RX ring.
                t.OnRetuned(now);
            }
            host_out.append((const char*)buf, len);
        }

        if (!t.HoldHostData()) {
            if (pending_off == pending_len && !host_in.empty()) {
                pending_len = host_in.copy((char*)pending, sizeof(pending));
                host_in.erase(0, pending_len);
                pending_off = 0;
            }
            size_t take = pending_len - pending_off;
            if (take > 16) take = 16;  // A small UART TX ring, so commands split across passes too.
            if (take > 0) {
                size_t send = t.OnHostBytes(pending + pending_off, take, now);
                for (size_t i = 0; i < send; i++) ModuleReceive((char)pending[pending_off + i], uart);
                pending_off += send;
            }
        }
        now++;
    }
    void Run(uint32_t ms) {
        for (uint32_t i = 0; i < ms; i++) Loop();
    }
    void Host(const std::string& text) { host_in += text; }
    bool InSync() const { return uart == console && t.console_baud() == console; }
    bool Clean() const {
        return host_out.find('~') == std::string::npos && module_got.find('\x01') == std::string::npos &&
               module_lost == 0;
    }
};

}  // namespace

// The host changes the rate and keeps talking in the same write: the command after it waits for the switch, then
// gets its answer at the new rate. Every rate in the range, both ways.
TEST(RateTracker, SwitchOnTheFly) {
    for (uint32_t baud : {115200u, 9600u, 3000000u, 57600u, 123457u, 1000000u}) {
        Sim s;
        s.Start();
        s.Host("AT+BAUD_RATE=CONSOLE," + std::to_string(baud) + "\r\nAT+BAUD_RATE?\r\nAT+BAUD_RATE?\r\n");
        s.Run(50);
        EXPECT_TRUE(s.InSync()) << baud;
        EXPECT_EQ(s.console, baud);
        EXPECT_TRUE(s.Clean()) << s.host_out;
        std::string answer = "BAUD_RATE=CONSOLE," + std::to_string(baud) + "\r\n";
        EXPECT_EQ(s.host_out, "OK\r\n" + answer + answer);
        EXPECT_EQ(s.saved, 1000000u);  // Nothing persisted.
        EXPECT_EQ(s.wake_locks + s.reset_locks, 0);
    }
}

// Report output keeps flowing around the switch; the host loses none of it.
TEST(RateTracker, ReportsAroundSwitch) {
    Sim s;
    s.Start();
    std::string expected;
    for (int i = 0; i < 20; i++) {
        std::string report = "#A" + std::to_string(i) + ",1234\r\n";
        s.ModuleOut(report);
        expected += report;
        if (i == 5) s.Host("AT+BAUD_RATE=CONSOLE,57600\r\nAT+X\r\n");
        s.Run(3);
    }
    s.Run(20);
    EXPECT_TRUE(s.InSync());
    EXPECT_TRUE(s.Clean());
    // Every report, in order; the two OKs fall between them wherever the module sent them.
    std::string reports = s.host_out;
    for (int i = 0; i < 2; i++) {
        size_t ok = reports.find("OK\r\n");
        ASSERT_NE(ok, std::string::npos);
        reports.erase(ok, 4);
    }
    EXPECT_EQ(reports, expected);
}

// A human typing the command one key at a time, with CR and LF in separate writes.
TEST(RateTracker, TypedCommand) {
    Sim s;
    s.Start();
    for (char c : std::string("AT+BAUD_RATE=CONSOLE,230400\r")) {
        s.Host(std::string(1, c));
        s.Run(2);
    }
    EXPECT_FALSE(s.t.HoldHostData());  // The module runs the line at its '\n'.
    s.Host("\n");
    s.Run(1);
    s.Host("AT+BAUD_RATE?\n");
    s.Run(20);
    EXPECT_TRUE(s.InSync());
    EXPECT_TRUE(s.Clean());
    EXPECT_EQ(s.host_out, "OK\r\nBAUD_RATE=CONSOLE,230400\r\n");
}

// The OK split across reads of the UART.
TEST(RateTracker, OkSplitAcrossReads) {
    RateTracker t;
    t.Start(1000000, 0);
    const char* cmd = "AT+BAUD_RATE=CONSOLE,115200\r\nAT+X\r\n";
    EXPECT_EQ(t.OnHostBytes((const uint8_t*)cmd, strlen(cmd), 1), strlen("AT+BAUD_RATE=CONSOLE,115200\r\n"));
    EXPECT_TRUE(t.HoldHostData());
    EXPECT_EQ(t.OnHostBytes((const uint8_t*)"AT+X\r\n", 6, 2), 0u);
    EXPECT_EQ(t.OnConsoleBytes((const uint8_t*)"#rep\r\nO", 7, 3), 7u);
    EXPECT_EQ(t.Poll(3, false).kind, RateTracker::Step::kNone);
    EXPECT_EQ(t.OnConsoleBytes((const uint8_t*)"K\r\n~~~", 6, 4), 3u);  // The rest came at the new rate.
    RateTracker::Step step = t.Poll(4, false);
    EXPECT_EQ(step.kind, RateTracker::Step::kRetune);
    EXPECT_EQ(step.baud, 115200u);
    t.OnRetuned(4);
    EXPECT_EQ(t.console_baud(), 115200u);
    // The module is still reopening its UART.
    EXPECT_TRUE(t.HoldHostData());
    EXPECT_EQ(t.OnHostBytes((const uint8_t*)"AT+X\r\n", 6, 4), 0u);
    EXPECT_EQ(t.Poll(4 + RateTracker::kSwitchSettleMs - 1, false).kind, RateTracker::Step::kNone);
    EXPECT_TRUE(t.HoldHostData());
    EXPECT_EQ(t.Poll(4 + RateTracker::kSwitchSettleMs, false).kind, RateTracker::Step::kNone);
    EXPECT_FALSE(t.HoldHostData());
}

// A rate the console refuses: ERROR, no retune, host data flows again.
TEST(RateTracker, Refused) {
    Sim s;
    s.accept = false;
    s.Start();
    s.Host("AT+BAUD_RATE=CONSOLE,57600\r\nAT+BAUD_RATE?\r\n");
    s.Run(20);
    EXPECT_EQ(s.retunes, 0);
    EXPECT_TRUE(s.InSync() && s.console == 1000000);
    EXPECT_TRUE(s.Clean());
    EXPECT_EQ(s.host_out, "ERROR Baud not supported.\r\nBAUD_RATE=CONSOLE,1000000\r\n");
}

// Commands the tracker leaves alone: out of range (the console answers ERROR), wrong case or syntax (the module doesn't
// run them), queries, other commands, overlong lines. No hold, no AT traffic of the Programmer's own.
TEST(RateTracker, IgnoredCommands) {
    for (const char* cmd : {"AT+BAUD_RATE=CONSOLE,1200", "AT+BAUD_RATE=CONSOLE,4000000", "at+baud_rate=console,57600",
                            "AT+BAUD_RATE=CONSOLE,57600x", "AT+BAUD_RATE=CONSOLE,", "AT+BAUD_RATE=GNSS,57600",
                            "AT+BAUD_RATE?", "AT+SETTINGS=SAVE", "AT+SETTINGS=LOAD", "AT+REBOOTX",
                            "AT+BAUD_RATE=CONSOLE,99999999999999999999", "overlong"}) {
        RateTracker t;
        t.Start(1000000, 0);
        // Longer than the module's AT line buffer: it drops the line, and so does the tracker.
        std::string line = strcmp(cmd, "overlong") == 0 ? std::string(1000, 'X') + "AT+BAUD_RATE=CONSOLE,57600\r\n"
                                                         : std::string(cmd) + "\r\n";
        EXPECT_EQ(t.OnHostBytes((const uint8_t*)line.data(), line.size(), 1), line.size()) << cmd;
        EXPECT_FALSE(t.HoldHostData()) << cmd;
        EXPECT_EQ(t.Poll(100000, false).kind, RateTracker::Step::kNone) << cmd;
    }
}

// Forms the module's parser (cppAT) also runs: whitespace and '+' around the number, repeated operator characters,
// several commands on one line (a terminal that ends lines with CR alone), any operator after REBOOT.
TEST(RateTracker, CommandFormsTheModuleAccepts) {
    struct Case {
        const char* line;
        uint32_t target;  // 0: AT+REBOOT.
    };
    for (Case c : {Case{"AT+BAUD_RATE=CONSOLE, 57600 ", 57600}, Case{"AT+BAUD_RATE=CONSOLE,+57600", 57600},
                   Case{"AT+BAUD_RATE==CONSOLE,57600", 57600}, Case{"AT+BAUD_RATE=CONSOLE,0057600", 57600},
                   Case{"AT+UPTIME?\rAT+BAUD_RATE=CONSOLE,19200\r", 19200}, Case{"  AT+BAUD_RATE=CONSOLE,9600", 9600},
                   Case{"AT+SETTINGS=RESET", 1000000}, Case{"AT+REBOOT", 0}, Case{"AT+REBOOT?", 0},
                   Case{"AT+REBOOT=", 0}}) {
        RateTracker t;
        t.Start(115200, 0);
        std::string line = std::string(c.line) + "\n";
        t.OnHostBytes((const uint8_t*)line.data(), line.size(), 1);
        EXPECT_TRUE(t.HoldHostData()) << c.line;
        if (c.target == 0) {
            EXPECT_EQ(t.Poll(1 + RateTracker::kRebootSettleMs, false).kind, RateTracker::Step::kResetAndLock) << c.line;
        } else {
            EXPECT_EQ(t.OnConsoleBytes((const uint8_t*)"OK\r\n", 4, 2), 4u);
            RateTracker::Step step = t.Poll(2, false);
            EXPECT_EQ(step.kind, RateTracker::Step::kRetune) << c.line;
            EXPECT_EQ(step.baud, c.target) << c.line;
        }
    }
    // And forms it refuses (ERROR, or not a command at all).
    for (const char* cmd : {"AT+BAUD_RATE =CONSOLE,57600", "AT+BAUD_RATE=CONSOLE ,57600", "AT+BAUD_RATE=CONSOLE,57600,1",
                            "AT+BAUD_RATE=CONSOLE,-57600", "AT+BAUD_RATE=CONSOLE,5 7600", "AT+REBOOT=1",
                            "AT+SETTINGS=RESET2", "AT+SETTINGS?", "AT+BAUD_RATEX=CONSOLE,57600"}) {
        RateTracker t;
        t.Start(115200, 0);
        std::string line = std::string(cmd) + "\r\n";
        t.OnHostBytes((const uint8_t*)line.data(), line.size(), 1);
        EXPECT_FALSE(t.HoldHostData()) << cmd;
    }
}

// Garbage ahead of the command on the same line (a wrong-rate byte, a stray CR): the module still runs it, so the
// tracker does too. After a NUL (a held-low line) the module sees nothing more of the line, and neither does the
// tracker.
TEST(RateTracker, GarbageBeforeCommand) {
    RateTracker t;
    t.Start(1000000, 0);
    const uint8_t line[] = "\x01\xfe\rAT+BAUD_RATE=CONSOLE,19200\r\n";
    EXPECT_EQ(t.OnHostBytes(line, sizeof(line) - 1, 1), sizeof(line) - 1);
    EXPECT_TRUE(t.HoldHostData());
    RateTracker n;
    n.Start(1000000, 0);
    const uint8_t nul[] = "\0\rAT+BAUD_RATE=CONSOLE,19200\r\n";
    EXPECT_EQ(n.OnHostBytes(nul, sizeof(nul) - 1, 1), sizeof(nul) - 1);
    EXPECT_FALSE(n.HoldHostData());
    // A long line of junk ahead of it too.
    RateTracker u;
    u.Start(1000000, 0);
    std::string junk = std::string(900, 'X') + "AT+BAUD_RATE=CONSOLE,19200\r\n";
    EXPECT_EQ(u.OnHostBytes((const uint8_t*)junk.data(), junk.size(), 1), junk.size());
    EXPECT_TRUE(u.HoldHostData());
}

// The console's rate moved without the tracker seeing it and the module is quiet (no framing errors): the host's next
// command reaches it as garbage, nothing comes back, and the silence asks for a wake lock.
TEST(RateTracker, SilenceRelocks) {
    Sim s;
    s.Start();
    s.Run(RateTracker::kRelockIntervalMs);
    s.console = 57600;  // E.g. a module power-cycled back to its saved rate.
    s.Host("AT+BAUD_RATE?\r\n");
    s.Run(RateTracker::kNoReplyMs - 10);
    EXPECT_EQ(s.wake_locks, 0);
    s.Run(20);
    EXPECT_EQ(s.wake_locks, 1);
    EXPECT_TRUE(s.InSync());
    s.host_out.clear();
    s.module_got.clear();
    s.Host("AT+BAUD_RATE?\r\n");
    s.Run(10);
    EXPECT_EQ(s.host_out, "BAUD_RATE=CONSOLE,57600\r\n");
}

// Silence doesn't relock while SYNC is high (the module may be asleep), for lines that aren't AT commands, or when the
// console answers.
TEST(RateTracker, SilenceExceptions) {
    {
        RateTracker t;
        t.Start(1000000, 0);
        uint32_t now = RateTracker::kRelockIntervalMs;
        t.OnHostBytes((const uint8_t*)"AT+UPTIME?\n", 11, now);
        EXPECT_EQ(t.Poll(now + 100, true).kind, RateTracker::Step::kNone);
        t.Poll(now + 5000, false);
        EXPECT_EQ(t.Poll(now + 5000, false).kind, RateTracker::Step::kNone);
    }
    {
        RateTracker t;
        t.Start(1000000, 0);
        uint32_t now = RateTracker::kRelockIntervalMs;
        t.OnHostBytes((const uint8_t*)"hello\n\n", 7, now);
        t.Poll(now + 5000, false);
        EXPECT_EQ(t.Poll(now + 5000, false).kind, RateTracker::Step::kNone);
    }
    {
        RateTracker t;
        t.Start(1000000, 0);
        uint32_t now = RateTracker::kRelockIntervalMs;
        t.OnHostBytes((const uint8_t*)"AT+UPTIME?\n", 11, now);
        t.OnConsoleBytes((const uint8_t*)"UPTIME=5\r\n", 10, now + 3);
        t.Poll(now + 5000, false);
        EXPECT_EQ(t.Poll(now + 5000, false).kind, RateTracker::Step::kNone);
    }
    {
        // Right after a lock: waits out the rate limit, then relocks if the console is still silent.
        RateTracker t;
        t.Start(1000000, 0);
        t.OnHostBytes((const uint8_t*)"AT+UPTIME?\n", 11, 10);
        t.Poll(RateTracker::kNoReplyMs + 10, false);
        EXPECT_EQ(t.Poll(RateTracker::kNoReplyMs + 10, false).kind, RateTracker::Step::kNone);
        t.Poll(RateTracker::kRelockIntervalMs, false);
        EXPECT_EQ(t.Poll(RateTracker::kRelockIntervalMs, false).kind, RateTracker::Step::kWakeLock);
    }
}

// The OK never comes (lost, or an image that doesn't answer): after the timeout the console is found with a wake lock,
// and the held host data goes on at whatever rate it is at.
TEST(RateTracker, LostReply) {
    Sim s;
    s.answer = false;
    s.Start();
    s.Host("AT+BAUD_RATE=CONSOLE,57600\r\nAT+BAUD_RATE?\r\n");
    s.Run(10);
    EXPECT_TRUE(s.t.HoldHostData());
    // 500 ms + twice 8 kB at 1 M.
    s.Run(500 + 2 * 81920000 / 1000000 + 10);
    EXPECT_EQ(s.wake_locks, 1);
    s.Run(10);
    EXPECT_TRUE(s.InSync() && s.console == 57600);
    EXPECT_EQ(s.host_out, "BAUD_RATE=CONSOLE,57600\r\n");
    EXPECT_TRUE(s.Clean());
}

// The reply timeout scales with the old rate (the OK queues behind the console's TX ring) and is capped.
TEST(RateTracker, ReplyTimeout) {
    for (uint32_t from : {3000000u, 1000000u, 115200u, 9600u}) {
        RateTracker t;
        t.Start(from, 0);
        const char* cmd = "AT+BAUD_RATE=CONSOLE,57600\n";
        t.OnHostBytes((const uint8_t*)cmd, strlen(cmd), 0);
        uint64_t expected = RateTracker::kReplyBaseMs + 2ull * 81920000 / from;
        if (expected > RateTracker::kReplyMaxMs) expected = RateTracker::kReplyMaxMs;
        EXPECT_EQ(t.Poll((uint32_t)expected - 1, false).kind, RateTracker::Step::kNone) << from;
        EXPECT_TRUE(t.HoldHostData());
        t.Poll((uint32_t)expected, false);
        RateTracker::Step step = t.Poll((uint32_t)expected, false);
        EXPECT_EQ(step.kind, RateTracker::Step::kWakeLock) << from;
        EXPECT_EQ(step.baud, 57600u);  // Where the UART goes if the lock finds nothing.
    }
}

// Save, then AT+REBOOT: the bridge resets and locks at the saved rate.
TEST(RateTracker, SaveAndReboot) {
    Sim s;
    s.Start();
    s.Host("AT+BAUD_RATE=CONSOLE,115200\r\nAT+SETTINGS=SAVE\r\nAT+REBOOT\r\nAT+BAUD_RATE?\r\n");
    s.Run(RateTracker::kRebootSettleMs + 50);
    EXPECT_EQ(s.reset_locks, 1);
    EXPECT_TRUE(s.InSync() && s.console == 115200 && s.saved == 115200);
    EXPECT_EQ(s.host_out, "OK\r\nOK\r\nBAUD_RATE=CONSOLE,115200\r\n");
    EXPECT_TRUE(s.Clean());
}

// An unsaved change is gone after AT+REBOOT, and the Programmer follows it back.
TEST(RateTracker, UnsavedRevertsOnReboot) {
    Sim s;
    s.Start();
    s.Host("AT+BAUD_RATE=CONSOLE,9600\r\nAT+REBOOT\r\nAT+BAUD_RATE?\r\n");
    s.Run(RateTracker::kRebootSettleMs + 50);
    EXPECT_EQ(s.reset_locks, 1);
    EXPECT_TRUE(s.InSync() && s.console == 1000000);
    EXPECT_EQ(s.host_out, "OK\r\nBAUD_RATE=CONSOLE,1000000\r\n");
    EXPECT_TRUE(s.Clean());
}

// AT+REBOOT waits while SYNC is high (a reset then would start the ROM bootloader).
TEST(RateTracker, RebootWaitsForSyncLow) {
    RateTracker t;
    t.Start(1000000, 0);
    t.OnHostBytes((const uint8_t*)"AT+REBOOT\n", 10, 0);
    EXPECT_TRUE(t.HoldHostData());
    EXPECT_EQ(t.Poll(RateTracker::kRebootSettleMs - 1, false).kind, RateTracker::Step::kNone);
    EXPECT_EQ(t.Poll(RateTracker::kRebootSettleMs + 50, true).kind, RateTracker::Step::kNone);
    EXPECT_EQ(t.Poll(RateTracker::kRebootSettleMs + 60, false).kind, RateTracker::Step::kResetAndLock);
}

// AT+SETTINGS=RESET puts the console back at the factory 1 M after its OK.
TEST(RateTracker, SettingsReset) {
    Sim s;
    s.console = s.saved = 250000;
    s.Start();
    s.Host("AT+SETTINGS=RESET\r\nAT+BAUD_RATE?\r\n");
    s.Run(20);
    EXPECT_TRUE(s.InSync() && s.console == 1000000);
    EXPECT_EQ(s.host_out, "OK\r\nBAUD_RATE=CONSOLE,1000000\r\n");
    EXPECT_TRUE(s.Clean());
}

// The console's rate changes behind the tracker's back (here: another module at another rate): the framing errors ask
// for a wake lock, at most once every kRelockIntervalMs.
TEST(RateTracker, FramingErrorsRelock) {
    Sim s;
    s.Start();
    s.Run(RateTracker::kRelockIntervalMs);
    s.console = 57600;
    s.ModuleOut("#report one\r\n");
    s.Run(5);
    EXPECT_EQ(s.wake_locks, 1);
    EXPECT_TRUE(s.InSync());
    s.ModuleOut("#report two\r\n");
    s.Run(5);
    EXPECT_EQ(s.host_out.substr(s.host_out.size() - 13), "#report two\r\n");

    // Again right away: rate-limited.
    s.console = 115200;
    s.ModuleOut("#report three\r\n");
    s.Run(5);
    EXPECT_EQ(s.wake_locks, 1);
    s.Run(RateTracker::kRelockIntervalMs);
    s.ModuleOut("#report four\r\n");
    s.Run(5);
    EXPECT_EQ(s.wake_locks, 2);
    EXPECT_TRUE(s.InSync());
}

// A few errors (line noise) or errors spread out in time don't relock.
TEST(RateTracker, IsolatedErrorsDontRelock) {
    RateTracker t;
    t.Start(1000000, 0);
    uint32_t now = RateTracker::kRelockIntervalMs;
    for (int i = 0; i < 50; i++) {
        t.OnRxErrors(RateTracker::kRelockErrors - 1, now);
        now += RateTracker::kErrorWindowMs + 1;
        EXPECT_EQ(t.Poll(now, false).kind, RateTracker::Step::kNone);
    }
    t.OnRxErrors(RateTracker::kRelockErrors, now);
    EXPECT_EQ(t.Poll(now, true).kind, RateTracker::Step::kNone);  // Waits for SYNC low.
    EXPECT_EQ(t.Poll(now, false).kind, RateTracker::Step::kWakeLock);
}

// ROM bootloader: bytes pass untouched both ways and nothing is held or relocked until a reset with SYNC low.
TEST(RateTracker, RomBootloaderHandsOff) {
    RateTracker t;
    t.Start(115200, 0);
    t.OnReset(true, 0, 10);
    EXPECT_TRUE(t.in_rom_bootloader());
    const char* data = "\x55\x55" "AT+BAUD_RATE=CONSOLE,57600\nAT+REBOOT\n";
    EXPECT_EQ(t.OnHostBytes((const uint8_t*)data, strlen(data), 11), strlen(data));
    EXPECT_FALSE(t.HoldHostData());
    EXPECT_EQ(t.OnConsoleBytes((const uint8_t*)"\x00\xcc" "OK\r\n", 6, 12), 6u);
    t.OnRxErrors(1000, 13);
    EXPECT_EQ(t.Poll(100000, false).kind, RateTracker::Step::kNone);
    t.OnReset(false, 1000000, 100001);
    EXPECT_FALSE(t.in_rom_bootloader());
    EXPECT_EQ(t.console_baud(), 1000000u);
}

// A host reset in the middle of a switch starts over from the lock.
TEST(RateTracker, ResetDuringSwitch) {
    RateTracker t;
    t.Start(1000000, 0);
    t.OnHostBytes((const uint8_t*)"AT+BAUD_RATE=CONSOLE,57600\n", 27, 1);
    EXPECT_TRUE(t.HoldHostData());
    t.OnReset(false, 1000000, 2);
    EXPECT_FALSE(t.HoldHostData());
    EXPECT_EQ(t.OnConsoleBytes((const uint8_t*)"OK\r\n", 4, 3), 4u);
    EXPECT_EQ(t.Poll(100000, false).kind, RateTracker::Step::kNone);
}

// Timeouts across the millisecond counter's wrap.
TEST(RateTracker, ClockWrap) {
    RateTracker t;
    uint32_t start = 0xFFFFFFF0u;
    t.Start(1000000, start);
    t.OnHostBytes((const uint8_t*)"AT+BAUD_RATE=CONSOLE,57600\n", 27, start);
    EXPECT_EQ(t.Poll(start + 100, false).kind, RateTracker::Step::kNone);  // Wrapped, 100 ms later.
    t.Poll(start + 1000, false);
    EXPECT_EQ(t.Poll(start + 1000, false).kind, RateTracker::Step::kWakeLock);
}
