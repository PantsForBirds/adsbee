// Host tests for the console baud range (ti/comms/console_baud.hh), the rates the ADSBee 1421 Programmer renegotiates
// (host_line_coding.hh), and BaudFollower: the line-coding / reset / command sequences real hosts
// produce, played back against the follower the way bridge.cc drives it.
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <initializer_list>

#include "gtest/gtest.h"
#include "baud_follower.hh"
#include "console_baud.hh"
#include "host_line_coding.hh"


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
TEST(BaudFollower, DividerMatchesDrivers) {
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

// Console accepts 9600..3000000 within 2%.
TEST(BaudFollower, ConsoleRange) {
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
        EXPECT_TRUE(IsRenegotiableBaud(baud));
    }
    printf("  worst CC1314 error %u ppm, worst CC1314 vs Programmer mismatch %u ppm\n", (unsigned)worst_cc1314,
           (unsigned)worst_link);
    EXPECT_TRUE(worst_cc1314 < 8000);
    EXPECT_TRUE(worst_link < 11000);  // ~1%, near 3 M where the RP2040 divisor is smallest.
}

// Renegotiable host rates.
TEST(BaudFollower, RenegotiableRates) {
    for (uint32_t baud : {9600u, 19200u, 57600u, 76800u, 115200u, 123457u, 250000u, 1000000u, 2000000u, 3000000u}) {
        EXPECT_TRUE(IsRenegotiableBaud(baud));
    }
    // pymavlink's 1200 probe, the reboot magic, the ROM-only and out-of-range rates go straight to the UART.
    for (uint32_t baud : {0u, 300u, 1200u, 4800u, 3500000u, kRebootToBootselBaud}) {
        EXPECT_FALSE(IsRenegotiableBaud(baud));
    }
}

// ---- BaudFollower ----

// How long the bridge's reset + autobaud lock takes (ConsoleLock(kReset)).
static constexpr uint32_t kLockMs = 450;

namespace {

// Mirrors BridgeRun(): callbacks feed the follower, each loop iteration polls it and carries out the step.
struct Bridge {
    BaudFollower f;
    uint32_t now = 5000;
    bool sync_high = false;
    uint32_t uart = 0;            // Programmer UART rate.
    uint32_t console = 1000000;   // The module console's actual rate.
    uint32_t boot = 1000000;      // The module's saved rate.
    bool console_up = true;       // Answering (false: no autobaud answer and not at a probed rate either).
    int renegotiations = 0;
    int resets = 0;
    uint32_t last_target = 0;
    bool accept = true;           // The console accepts AT+BAUD_RATE.

    void Start(uint32_t host_baud) {
        uart = console;
        f.Start(console, host_baud, now);
    }
    void HostBaud(uint32_t baud) { f.OnHostBaud(baud, now); }
    // A host DTR edge: ResetAndLock() (SYNC low) or a plain reset pulse (SYNC high).
    void Reset(bool sync) {
        resets++;
        if (sync) {
            f.OnReset(true, 0);
            return;
        }
        console = boot;
        now += kLockMs;
        uart = console_up ? console : uart;
        f.OnReset(false, console_up ? console : 0);
    }
    void Send(const char* text) { f.OnHostBytes((const uint8_t*)text, strlen(text), now); }
    // What ConsoleRenegotiate() would do against this console.
    uint32_t Renegotiate(uint32_t target) {
        renegotiations++;
        last_target = target;
        if (!console_up) {
            if (target) uart = target;
            return 0;
        }
        if (target != 0 && accept) console = target;
        uart = console;
        return console;
    }
    void Loop() {
        BaudFollower::Step step = f.Poll(now, sync_high);
        if (step.kind == BaudFollower::Step::kApplyDirect) uart = step.baud;
        if (step.kind == BaudFollower::Step::kRenegotiate) f.OnRenegotiated(Renegotiate(step.baud));
        if (step.kind == BaudFollower::Step::kReset) Reset(false);
    }
    void Wait(uint32_t ms) {
        for (uint32_t i = 0; i < ms; i++) {
            now++;
            Loop();
        }
    }
    bool Hold() { return f.HoldHostData(sync_high); }
    bool InSync() { return uart == console && f.console_baud() == console; }
};

}  // namespace

// pymavlink: the kernel open asserts DTR (a reset into the app), then 1200, then the tool's rate.
// Pymavlink open at 57600: reset, 1200, 57600 -> one renegotiation after the lock.
TEST(BaudFollower, PymavlinkOpen) {
    Bridge b;
    b.Start(0);
    // The rates arrive while the bridge locks after the reset (tud_task runs inside ConsoleLock()).
    b.HostBaud(1200);
    b.now++;
    b.HostBaud(57600);
    b.Reset(false);
    EXPECT_TRUE(b.Hold());
    b.Wait(1);
    EXPECT_EQ(b.renegotiations, 1);
    EXPECT_EQ(b.last_target, 57600u);
    EXPECT_TRUE(b.console == 57600 && b.InSync());
    EXPECT_FALSE(b.Hold());
    b.Wait(2000);
    EXPECT_EQ(b.renegotiations, 1);
}

// Several rates in quick succession -> one renegotiation to the last.
TEST(BaudFollower, RapidChanges) {
    Bridge b;
    b.Start(1000000);
    for (uint32_t baud : {115200u, 57600u, 230400u, 1200u, 250000u}) {
        b.HostBaud(baud);
        b.Wait(20);
    }
    b.Wait(BaudFollower::kSettleMs);
    EXPECT_EQ(b.renegotiations, 1);
    EXPECT_TRUE(b.console == 250000 && b.InSync());
}

// Host rate equal to the console's -> UART only, no AT traffic.
TEST(BaudFollower, SameRateNoTraffic) {
    Bridge b;
    b.Start(0);
    b.HostBaud(1000000);
    b.Wait(500);
    EXPECT_EQ(b.renegotiations, 0);
    EXPECT_EQ(b.uart, 1000000u);
}

// Rate change while a renegotiation runs -> another one follows.
TEST(BaudFollower, ChangeDuringRenegotiation) {
    Bridge b;
    b.Start(1000000);
    b.HostBaud(57600);
    b.Wait(BaudFollower::kSettleMs - 1);
    // The renegotiation is about to start; the host changes again from inside it (tud_task in the AT client's waits).
    BaudFollower::Step step = b.f.Poll(++b.now, false);
    EXPECT_TRUE(step.kind == BaudFollower::Step::kRenegotiate && step.baud == 57600);
    b.HostBaud(115200);
    b.f.OnRenegotiated(b.Renegotiate(step.baud));
    EXPECT_TRUE(b.Hold());
    b.Wait(BaudFollower::kSettleMs + 1);
    EXPECT_EQ(b.renegotiations, 2);
    EXPECT_TRUE(b.console == 115200 && b.InSync());
}

// ROM bootloader: host rates go straight to the UART until a reset with SYNC low.
TEST(BaudFollower, RomBootloaderDirect) {
    Bridge b;
    b.Start(115200);
    b.Reset(true);
    b.Loop();
    EXPECT_EQ(b.uart, 115200u);  // Immediately, no settle.
    EXPECT_TRUE(b.f.in_rom_bootloader());
    b.HostBaud(1000000);
    EXPECT_FALSE(b.Hold());
    b.Loop();
    EXPECT_EQ(b.uart, 1000000u);
    b.HostBaud(460800);
    b.Loop();
    EXPECT_EQ(b.uart, 460800u);
    EXPECT_EQ(b.renegotiations, 0);
    // Back to the app, which boots at its saved 1 M: the host is at 460800, so the console is moved once locked.
    b.Reset(false);
    b.Wait(BaudFollower::kSettleMs);
    EXPECT_FALSE(b.f.in_rom_bootloader());
    EXPECT_EQ(b.renegotiations, 1);
    EXPECT_TRUE(b.console == 460800 && b.InSync());
}

// Host reset with the host at the boot rate -> UART retuned at once, no AT traffic.
TEST(BaudFollower, ResetAtBootRate) {
    Bridge b;
    b.Start(1000000);
    b.HostBaud(57600);
    b.Wait(BaudFollower::kSettleMs + 1);
    EXPECT_TRUE(b.console == 57600 && b.renegotiations == 1);
    // Reopen at 1 M: the open resets the console (it boots at 1 M) before the host sets 1 M.
    b.Reset(false);
    EXPECT_TRUE(b.Hold());  // The host still says 57600: a renegotiation is coming unless the rate changes.
    b.HostBaud(1000000);
    b.Wait(BaudFollower::kSettleMs + 1);
    EXPECT_EQ(b.uart, 1000000u);
    EXPECT_FALSE(b.Hold());
    b.Wait(2000);
    EXPECT_EQ(b.renegotiations, 1);
    EXPECT_TRUE(b.InSync());
    // Reset with the host already at the boot rate: retuned in the same loop iteration.
    Bridge c;
    c.Start(1000000);
    c.uart = 1;
    c.Reset(false);
    c.Loop();
    EXPECT_EQ(c.uart, 1000000u);
    c.Wait(2000);
    EXPECT_EQ(c.renegotiations, 0);
}

// Host reset at a non-boot rate -> console moved back to the host's rate after the lock.
TEST(BaudFollower, ResetBackToBootRate) {
    Bridge b;
    b.Start(57600);
    b.Wait(BaudFollower::kSettleMs + 1);
    EXPECT_EQ(b.console, 57600u);
    b.Reset(false);  // Reopen at the same rate: the console comes back at 1 M.
    EXPECT_TRUE(b.uart == 1000000 && b.f.console_baud() == 1000000);
    b.Wait(1);
    EXPECT_EQ(b.renegotiations, 2);
    EXPECT_TRUE(b.console == 57600 && b.InSync());
}

// Rates the console doesn't accept go straight to the UART.
TEST(BaudFollower, UnsupportedRatesDirect) {
    for (uint32_t baud : {1200u, 300u, 4000000u}) {
        Bridge b;
        b.Start(0);
        b.HostBaud(baud);
        EXPECT_FALSE(b.Hold());
        b.Wait(BaudFollower::kSettleMs + 1);
        EXPECT_EQ(b.uart, baud);
        EXPECT_EQ(b.renegotiations, 0);
        EXPECT_EQ(b.console, 1000000u);
    }
}

// SYNC high (console asleep) defers the renegotiation, and host data isn't held meanwhile.
TEST(BaudFollower, SyncHighDefers) {
    Bridge b;
    b.Start(1000000);
    b.sync_high = true;
    b.HostBaud(57600);
    EXPECT_FALSE(b.Hold());
    b.Wait(1000);
    EXPECT_EQ(b.renegotiations, 0);
    b.sync_high = false;
    b.Wait(1);
    EXPECT_EQ(b.renegotiations, 1);
    EXPECT_TRUE(b.console == 57600 && b.InSync());
}

// Renegotiation that fails leaves pass-through running and isn't retried in a loop.
TEST(BaudFollower, RenegotiationFailure) {
    Bridge b;
    b.Start(1000000);
    b.accept = false;
    b.HostBaud(57600);
    b.Wait(BaudFollower::kSettleMs + 1);
    EXPECT_EQ(b.renegotiations, 1);
    EXPECT_TRUE(b.uart == 1000000 && b.f.console_baud() == 1000000);  // Found, but it stayed at 1 M.
    EXPECT_FALSE(b.Hold());
    b.Wait(5000);
    EXPECT_EQ(b.renegotiations, 1);
    b.accept = true;
    b.HostBaud(115200);  // The next change tries again.
    b.Wait(BaudFollower::kSettleMs + 1);
    EXPECT_EQ(b.renegotiations, 2);
    EXPECT_EQ(b.console, 115200u);
}

// Host never set a rate: the UART follows the console.
TEST(BaudFollower, HostNeverSetRate) {
    Bridge b;
    b.Start(0);
    b.Wait(1000);
    EXPECT_EQ(b.renegotiations, 0);
    b.Reset(false);
    b.Loop();
    EXPECT_EQ(b.uart, 1000000u);
    b.Wait(2000);
    EXPECT_EQ(b.renegotiations, 0);
    // Saved at another rate: the lock finds it, the UART follows, no AT traffic.
    Bridge c;
    c.boot = 57600;
    c.Start(0);
    c.Reset(false);
    c.Wait(2000);
    EXPECT_EQ(c.renegotiations, 0);
    EXPECT_TRUE(c.uart == 57600 && c.f.console_baud() == 57600);
    // The lock failed (console dead): one attempt to find it, not a loop.
    Bridge d;
    d.Start(115200);
    d.console_up = false;
    d.Reset(false);
    d.Wait(5000);
    EXPECT_TRUE(d.renegotiations == 1 && d.f.console_baud() == 0);
}

// A rate the host set while the Programmer was busy is applied when pass-through starts.
TEST(BaudFollower, StartWithHostRate) {
    Bridge b;
    b.Start(76800);
    EXPECT_TRUE(b.Hold());
    b.Wait(BaudFollower::kSettleMs + 1);
    EXPECT_TRUE(b.console == 76800 && b.InSync());
    Bridge c;
    c.Start(1200);
    c.Wait(1000);
    EXPECT_TRUE(c.uart == 1000000 && c.renegotiations == 0);
}

// Snooped AT+BAUD_RATE=CONSOLE,<n> is probed first; the web console's reopen then needs no move.
TEST(BaudFollower, SnoopBaudHint) {
    Bridge b;
    b.Start(1000000);
    b.Send("AT+BAUD_RATE=CONSOLE,");
    b.Send("115200\r\n");  // Split across USB packets.
    EXPECT_EQ(b.f.hinted_baud(), 115200u);
    b.console = 115200;  // The console switched after its OK.
    b.HostBaud(115200);
    b.Wait(BaudFollower::kSettleMs + 1);
    EXPECT_TRUE(b.renegotiations == 1 && b.last_target == 115200);
    EXPECT_EQ(b.f.hinted_baud(), 0u);
    EXPECT_TRUE(b.InSync());

    BaudFollower f;
    f.Start(1000000, 1000000, 0);
    const char* ignored[] = {"AT+BAUD_RATE=CONSOLE,1200\r",  "AT+BAUD_RATE=CONSOLE,57600X\r",
                             "AT+BAUD_RATE=CONSOLE,99999999999\r", "XAT+BAUD_RATE=CONSOLE,57600\r",
                             "AT+BAUD_RATE?\r"};
    for (const char* line : ignored) {
        f.OnHostBytes((const uint8_t*)line, strlen(line), 0);
        EXPECT_EQ(f.hinted_baud(), 0u);
    }
    const char* lower = "at+baud_rate=console,57600  \n";
    f.OnHostBytes((const uint8_t*)lower, strlen(lower), 0);
    EXPECT_EQ(f.hinted_baud(), 57600u);
    // A line too long to be a command is dropped whole, including a command-like tail.
    BaudFollower g;
    g.Start(1000000, 1000000, 0);
    char longline[128];
    memset(longline, 'x', sizeof(longline));
    g.OnHostBytes((const uint8_t*)longline, sizeof(longline), 0);
    const char* tail = "AT+BAUD_RATE=CONSOLE,57600\r";
    g.OnHostBytes((const uint8_t*)tail, strlen(tail), 0);
    EXPECT_EQ(g.hinted_baud(), 0u);
    g.OnHostBytes((const uint8_t*)tail, strlen(tail), 0);
    EXPECT_EQ(g.hinted_baud(), 57600u);
}

// Snooped AT+SETTINGS=RESET: the console drops to 1 M and is moved back.
TEST(BaudFollower, SnoopSettingsReset) {
    Bridge b;
    b.Start(57600);
    b.Wait(BaudFollower::kSettleMs + 1);
    b.Send("AT+SETTINGS=RESET\r\n");
    b.console = 1000000;
    EXPECT_TRUE(b.Hold());
    EXPECT_EQ(b.f.hinted_baud(), 1000000u);
    b.Wait(BaudFollower::kSettleMs + 1);
    EXPECT_TRUE(b.renegotiations == 2 && b.last_target == 57600);
    EXPECT_TRUE(b.console == 57600 && b.InSync());
}

// Snooped AT+REBOOT: reset and lock, then moved to the host's rate.
TEST(BaudFollower, SnoopReboot) {
    Bridge b;
    b.Start(230400);
    b.Wait(BaudFollower::kSettleMs + 1);
    EXPECT_EQ(b.renegotiations, 1);
    b.Send("AT+REBOOT\r\n");
    b.console = b.boot;  // Rebooting at its saved rate.
    EXPECT_TRUE(b.Hold());
    b.Wait(BaudFollower::kSettleMs - 1);
    EXPECT_EQ(b.resets, 0);
    b.Wait(1);
    EXPECT_EQ(b.resets, 1);
    b.Wait(1);
    EXPECT_EQ(b.renegotiations, 2);
    EXPECT_TRUE(b.console == 230400 && b.InSync());
    // Not while SYNC is high (that reset would start the ROM bootloader).
    Bridge c;
    c.Start(1000000);
    c.Send("AT+REBOOT\r\n");
    c.sync_high = true;
    c.Wait(1000);
    EXPECT_EQ(c.resets, 0);
    c.sync_high = false;
    c.Wait(1);
    EXPECT_TRUE(c.resets == 1 && c.InSync());
}

// Ms counter wrap.
TEST(BaudFollower, ClockWrap) {
    Bridge b;
    b.now = 0xFFFFFF00u;
    b.Start(0);
    b.HostBaud(57600);
    b.Reset(false);
    b.Wait(BaudFollower::kSettleMs);
    EXPECT_EQ(b.renegotiations, 1);
    EXPECT_TRUE(b.console == 57600 && b.InSync());
}

