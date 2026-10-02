// Host tests for the console baud range (ti/comms/console_baud.hh), the rates the ADSBee 1421 Programmer renegotiates
// (host_line_coding.hh), the probe order, and BaudFollower: the line-coding / reset / command sequences real hosts
// produce, played back against the follower the way bridge.cc drives it.
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <initializer_list>

#include "baud_follower.hh"
#include "console_baud.hh"
#include "host_line_coding.hh"

static int failures = 0;

#define EXPECT(cond)                                                     \
    do {                                                                 \
        if (!(cond)) {                                                   \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
            failures++;                                                  \
        }                                                                \
    } while (0)

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

static void TestDividerMatchesDrivers() {
    printf("PL011 divider matches driverlib and pico-sdk\n");
    for (uint32_t baud = ConsoleBaud::kMin; baud <= ConsoleBaud::kMax; baud += 997) {
        EXPECT(ConsoleBaud::CC1314ActualBaud(baud) == TiActual(ConsoleBaud::kCC1314UartClockHz, baud));
        EXPECT(ProgrammerActualBaud(baud) == PicoActual(kProgrammerUartClockHz, baud));
    }
    EXPECT(ConsoleBaud::CC1314ActualBaud(115200) == 115176);  // Divisor 1667/64.
    EXPECT(ConsoleBaud::CC1314ActualBaud(3000000) == 3000000);
    EXPECT(ConsoleBaud::CC1314ActualBaud(3000001) == 3000000);  // Rounds to the same divisor.
    EXPECT(ConsoleBaud::CC1314ActualBaud(3100000) == 0);        // Integer part 0: can't be generated.
    EXPECT(ConsoleBaud::Pl011ActualBaud(48000000, 45) == 0);   // Integer part > 65535.
    EXPECT(ConsoleBaud::Pl011ActualBaud(48000000, 0) == 0);
}

static void TestConsoleRange() {
    printf("console accepts 9600..3000000 within 2%%\n");
    // The old whitelist stays valid, so saved settings keep working.
    for (uint32_t baud : {115200u, 230400u, 460800u, 921600u, 1000000u}) EXPECT(ConsoleBaud::IsSupported(baud));
    for (uint32_t baud : {9600u, 19200u, 38400u, 57600u, 76800u, 250000u, 500000u, 2000000u, 3000000u, 123457u}) {
        EXPECT(ConsoleBaud::IsSupported(baud));
    }
    for (uint32_t baud : {0u, 300u, 1200u, 9599u, 3000001u, 4000000u, 0xDEADBEEu, 0xFFFFFFFFu}) {
        EXPECT(!ConsoleBaud::IsSupported(baud));
    }
    // Every rate in range is generated well inside the 2% limit, on both ends of the link.
    uint32_t worst_cc1314 = 0, worst_link = 0;
    for (uint32_t baud = ConsoleBaud::kMin; baud <= ConsoleBaud::kMax; baud += 101) {
        uint32_t cc1314 = ConsoleBaud::CC1314ActualBaud(baud);
        uint32_t e = ConsoleBaud::ErrorPpm(baud, cc1314);
        if (e > worst_cc1314) worst_cc1314 = e;
        e = ConsoleBaud::ErrorPpm(cc1314, ProgrammerActualBaud(baud));
        if (e > worst_link) worst_link = e;
        EXPECT(ConsoleBaud::IsSupported(baud));
        EXPECT(IsRenegotiableBaud(baud));
    }
    printf("  worst CC1314 error %u ppm, worst CC1314 vs Programmer mismatch %u ppm\n", (unsigned)worst_cc1314,
           (unsigned)worst_link);
    EXPECT(worst_cc1314 < 8000);
    EXPECT(worst_link < 11000);  // ~1%, near 3 M where the RP2040 divisor is smallest.
}

static void TestRenegotiableRates() {
    printf("renegotiable host rates\n");
    for (uint32_t baud : kCommonConsoleBauds) EXPECT(IsRenegotiableBaud(baud));
    // pymavlink's 1200 probe, the reboot magic, the ROM-only and out-of-range rates go straight to the UART.
    for (uint32_t baud : {0u, 300u, 1200u, 4800u, 3500000u, kRebootToBootselBaud}) EXPECT(!IsRenegotiableBaud(baud));
}

static void TestCandidates() {
    printf("probe order: preferred first, no duplicates, only console rates\n");
    uint32_t out[32];
    const uint32_t preferred[] = {76801, 0, 1200, 1000000, 76801, 57600};
    size_t n = BuildConsoleBaudCandidates(preferred, 6, out, 32);
    EXPECT(n == 1 + sizeof(kCommonConsoleBauds) / sizeof(kCommonConsoleBauds[0]));
    EXPECT(out[0] == 76801);
    EXPECT(out[1] == 1000000);
    EXPECT(out[2] == 57600);
    EXPECT(out[3] == 921600);  // Then kCommonConsoleBauds in order, minus the ones already listed.
    for (size_t i = 0; i < n; i++) {
        EXPECT(IsRenegotiableBaud(out[i]));
        for (size_t j = 0; j < i; j++) EXPECT(out[i] != out[j]);
    }
    EXPECT(BuildConsoleBaudCandidates(preferred, 6, out, 2) == 2);
    EXPECT(BuildConsoleBaudCandidates(nullptr, 0, out, 32) ==
           sizeof(kCommonConsoleBauds) / sizeof(kCommonConsoleBauds[0]));
    EXPECT(out[0] == 1000000);
}

// ---- BaudFollower ----

static constexpr uint32_t kBootWaitMs = 800;

// Mirrors BridgeRun(): callbacks feed the follower, each loop iteration polls it and carries out the step.
struct Bridge {
    BaudFollower f;
    uint32_t now = 5000;
    bool sync_high = false;
    uint32_t uart = 0;            // Programmer UART rate.
    uint32_t console = 1000000;   // The module console's actual rate.
    uint32_t boot = 1000000;      // The module's saved rate.
    bool console_up = true;       // Booted and answering.
    uint32_t boot_done = 0;
    int renegotiations = 0;
    uint32_t last_target = 0;
    bool accept = true;           // The console accepts AT+BAUD_RATE.

    void Start(uint32_t host_baud) {
        uart = console;
        f.Start(console, boot, host_baud, now);
    }
    void HostBaud(uint32_t baud) { f.OnHostBaud(baud, now); }
    void Reset(bool sync) {
        f.OnReset(sync, now, kBootWaitMs);
        if (!sync) {
            console = boot;
            console_up = false;
            boot_done = now + 600;  // Boots a bit faster than the Programmer waits.
        }
    }
    void Send(const char* text) { f.OnHostBytes((const uint8_t*)text, strlen(text), now, kBootWaitMs); }
    // What AtRenegotiateConsole() would do against this console.
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
        if (!console_up && (int32_t)(now - boot_done) >= 0) console_up = true;
        BaudFollower::Step step = f.Poll(now, sync_high);
        if (step.kind == BaudFollower::Step::kApplyDirect) uart = step.baud;
        if (step.kind == BaudFollower::Step::kRenegotiate) f.OnRenegotiated(Renegotiate(step.baud));
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

// pymavlink: the kernel open asserts DTR (a reset into the app), then 1200, then the tool's rate.
static void TestPymavlinkOpen() {
    printf("pymavlink open at 57600: reset, 1200, 57600 -> one renegotiation after the boot\n");
    Bridge b;
    b.Start(0);
    b.Reset(false);
    b.HostBaud(1200);
    b.now++;
    b.HostBaud(57600);
    EXPECT(b.Hold());
    b.Wait(BaudFollower::kSettleMs + 50);
    EXPECT(b.renegotiations == 0);  // The console is still booting.
    EXPECT(b.Hold());
    b.Wait(kBootWaitMs);
    EXPECT(b.renegotiations == 1);
    EXPECT(b.last_target == 57600);
    EXPECT(b.console == 57600 && b.InSync());
    EXPECT(!b.Hold());
    b.Wait(2000);
    EXPECT(b.renegotiations == 1);
}

static void TestRapidChanges() {
    printf("several rates in quick succession -> one renegotiation to the last\n");
    Bridge b;
    b.Start(1000000);
    for (uint32_t baud : {115200u, 57600u, 230400u, 1200u, 250000u}) {
        b.HostBaud(baud);
        b.Wait(20);
    }
    b.Wait(BaudFollower::kSettleMs);
    EXPECT(b.renegotiations == 1);
    EXPECT(b.console == 250000 && b.InSync());
}

static void TestSameRateNoTraffic() {
    printf("host rate equal to the console's -> UART only, no AT traffic\n");
    Bridge b;
    b.Start(0);
    b.HostBaud(1000000);
    b.Wait(500);
    EXPECT(b.renegotiations == 0);
    EXPECT(b.uart == 1000000);
}

static void TestChangeDuringRenegotiation() {
    printf("rate change while a renegotiation runs -> another one follows\n");
    Bridge b;
    b.Start(1000000);
    b.HostBaud(57600);
    b.Wait(BaudFollower::kSettleMs - 1);
    // The renegotiation is about to start; the host changes again from inside it (tud_task in the AT client's waits).
    BaudFollower::Step step = b.f.Poll(++b.now, false);
    EXPECT(step.kind == BaudFollower::Step::kRenegotiate && step.baud == 57600);
    b.HostBaud(115200);
    b.f.OnRenegotiated(b.Renegotiate(step.baud));
    EXPECT(b.Hold());
    b.Wait(BaudFollower::kSettleMs + 1);
    EXPECT(b.renegotiations == 2);
    EXPECT(b.console == 115200 && b.InSync());
}

static void TestRomBootloaderDirect() {
    printf("ROM bootloader: host rates go straight to the UART until a reset with SYNC low\n");
    Bridge b;
    b.Start(115200);
    b.Reset(true);
    b.Loop();
    EXPECT(b.uart == 115200);  // Immediately, no settle.
    EXPECT(b.f.in_rom_bootloader());
    b.HostBaud(1000000);
    EXPECT(!b.Hold());
    b.Loop();
    EXPECT(b.uart == 1000000);
    b.HostBaud(460800);
    b.Loop();
    EXPECT(b.uart == 460800);
    EXPECT(b.renegotiations == 0);
    // Back to the app, which boots at its saved 1 M: the host is at 460800, so the console is moved once up.
    b.Reset(false);
    b.Wait(kBootWaitMs + BaudFollower::kSettleMs);
    EXPECT(!b.f.in_rom_bootloader());
    EXPECT(b.renegotiations == 1);
    EXPECT(b.console == 460800 && b.InSync());
}

static void TestResetAtBootRate() {
    printf("host reset with the host at the boot rate -> UART retuned at once, no AT traffic\n");
    Bridge b;
    b.Start(1000000);
    b.HostBaud(57600);
    b.Wait(BaudFollower::kSettleMs + 1);
    EXPECT(b.console == 57600 && b.renegotiations == 1);
    // Reopen at 1 M: the open resets the console (it boots at 1 M) before the host sets 1 M.
    b.Reset(false);
    b.Loop();
    EXPECT(b.Hold());  // The host still says 57600: a renegotiation is coming after the boot.
    b.HostBaud(1000000);
    b.Wait(BaudFollower::kSettleMs + 1);
    EXPECT(b.uart == 1000000);
    EXPECT(!b.Hold());
    b.Wait(2000);
    EXPECT(b.renegotiations == 1);
    EXPECT(b.InSync());
    // Reset with the host already at the boot rate: retuned in the same loop iteration.
    Bridge c;
    c.Start(1000000);
    c.uart = 1;
    c.Reset(false);
    c.Loop();
    EXPECT(c.uart == 1000000);
    c.Wait(2000);
    EXPECT(c.renegotiations == 0);
}

static void TestResetBackToBootRate() {
    printf("host reset at a non-boot rate -> console moved back to the host's rate after the boot\n");
    Bridge b;
    b.Start(57600);
    b.Wait(BaudFollower::kSettleMs + 1);
    EXPECT(b.console == 57600);
    b.Reset(false);  // Reopen at the same rate: the console comes back at 1 M.
    b.Wait(kBootWaitMs - 1);
    EXPECT(b.renegotiations == 1);
    b.Wait(2);
    EXPECT(b.renegotiations == 2);
    EXPECT(b.console == 57600 && b.InSync());
}

static void TestUnsupportedRatesDirect() {
    printf("rates the console doesn't accept go straight to the UART\n");
    for (uint32_t baud : {1200u, 300u, 4000000u}) {
        Bridge b;
        b.Start(0);
        b.HostBaud(baud);
        EXPECT(!b.Hold());
        b.Wait(BaudFollower::kSettleMs + 1);
        EXPECT(b.uart == baud);
        EXPECT(b.renegotiations == 0);
        EXPECT(b.console == 1000000);
    }
}

static void TestSyncHighDefers() {
    printf("SYNC high (console asleep) defers the renegotiation, and host data isn't held meanwhile\n");
    Bridge b;
    b.Start(1000000);
    b.sync_high = true;
    b.HostBaud(57600);
    EXPECT(!b.Hold());
    b.Wait(1000);
    EXPECT(b.renegotiations == 0);
    b.sync_high = false;
    b.Wait(1);
    EXPECT(b.renegotiations == 1);
    EXPECT(b.console == 57600 && b.InSync());
}

static void TestRenegotiationFailure() {
    printf("renegotiation that fails leaves pass-through running and isn't retried in a loop\n");
    Bridge b;
    b.Start(1000000);
    b.accept = false;
    b.HostBaud(57600);
    b.Wait(BaudFollower::kSettleMs + 1);
    EXPECT(b.renegotiations == 1);
    EXPECT(b.uart == 1000000 && b.f.console_baud() == 1000000);  // Found, but it stayed at 1 M.
    EXPECT(!b.Hold());
    b.Wait(5000);
    EXPECT(b.renegotiations == 1);
    b.accept = true;
    b.HostBaud(115200);  // The next change tries again.
    b.Wait(BaudFollower::kSettleMs + 1);
    EXPECT(b.renegotiations == 2);
    EXPECT(b.console == 115200);
}

static void TestHostNeverSetRate() {
    printf("host never set a rate: the UART follows the console\n");
    Bridge b;
    b.Start(0);
    b.Wait(1000);
    EXPECT(b.renegotiations == 0);
    b.Reset(false);
    b.Loop();
    EXPECT(b.uart == 1000000);
    b.Wait(2000);
    EXPECT(b.renegotiations == 0);
    // Boot rate unknown: find the console once it has booted.
    Bridge c;
    c.boot = 57600;
    c.f.Start(1000000, 0, 0, c.now);
    c.uart = 1000000;
    c.Reset(false);
    c.Wait(kBootWaitMs + 1);
    EXPECT(c.renegotiations == 1 && c.last_target == 0);
    EXPECT(c.uart == 57600 && c.f.console_baud() == 57600);
}

static void TestStartWithHostRate() {
    printf("a rate the host set while the Programmer was busy is applied when pass-through starts\n");
    Bridge b;
    b.Start(76800);
    EXPECT(b.Hold());
    b.Wait(BaudFollower::kSettleMs + 1);
    EXPECT(b.console == 76800 && b.InSync());
    Bridge c;
    c.Start(1200);
    c.Wait(1000);
    EXPECT(c.uart == 1000000 && c.renegotiations == 0);
}

static void TestSnoopBaudHint() {
    printf("snooped AT+BAUD_RATE=CONSOLE,<n> is probed first; the web console's reopen then needs no move\n");
    Bridge b;
    b.Start(1000000);
    b.Send("AT+BAUD_RATE=CONSOLE,");
    b.Send("115200\r\n");  // Split across USB packets.
    EXPECT(b.f.hinted_baud() == 115200);
    b.console = 115200;  // The console switched after its OK.
    b.HostBaud(115200);
    b.Wait(BaudFollower::kSettleMs + 1);
    EXPECT(b.renegotiations == 1 && b.last_target == 115200);
    EXPECT(b.f.hinted_baud() == 0);
    EXPECT(b.InSync());

    BaudFollower f;
    f.Start(1000000, 1000000, 1000000, 0);
    const char* ignored[] = {"AT+BAUD_RATE=CONSOLE,1200\r",  "AT+BAUD_RATE=CONSOLE,57600X\r",
                             "AT+BAUD_RATE=CONSOLE,99999999999\r", "XAT+BAUD_RATE=CONSOLE,57600\r",
                             "AT+BAUD_RATE?\r"};
    for (const char* line : ignored) {
        f.OnHostBytes((const uint8_t*)line, strlen(line), 0, kBootWaitMs);
        EXPECT(f.hinted_baud() == 0);
    }
    const char* lower = "at+baud_rate=console,57600  \n";
    f.OnHostBytes((const uint8_t*)lower, strlen(lower), 0, kBootWaitMs);
    EXPECT(f.hinted_baud() == 57600);
    // A line too long to be a command is dropped whole, including a command-like tail.
    BaudFollower g;
    g.Start(1000000, 1000000, 1000000, 0);
    char longline[128];
    memset(longline, 'x', sizeof(longline));
    g.OnHostBytes((const uint8_t*)longline, sizeof(longline), 0, kBootWaitMs);
    const char* tail = "AT+BAUD_RATE=CONSOLE,57600\r";
    g.OnHostBytes((const uint8_t*)tail, strlen(tail), 0, kBootWaitMs);
    EXPECT(g.hinted_baud() == 0);
    g.OnHostBytes((const uint8_t*)tail, strlen(tail), 0, kBootWaitMs);
    EXPECT(g.hinted_baud() == 57600);
}

static void TestSnoopSave() {
    printf("snooped AT+SETTINGS=SAVE makes the live rate the boot rate\n");
    Bridge b;
    b.Start(57600);
    b.Wait(BaudFollower::kSettleMs + 1);
    EXPECT(b.console == 57600);
    b.Send("AT+SETTINGS=SAVE\r\n");
    b.boot = 57600;
    EXPECT(b.f.boot_baud() == 57600 && b.f.boot_baud_changed());
    b.f.ClearBootBaudChanged();
    b.Reset(false);  // Reopen: the console boots at 57600, which is what the host wants.
    b.Loop();
    EXPECT(b.uart == 57600);
    b.Wait(2000);
    EXPECT(b.renegotiations == 1);
    EXPECT(b.InSync());
    // Saving the same rate again changes nothing.
    b.Send("AT+SETTINGS=SAVE\r\n");
    EXPECT(!b.f.boot_baud_changed());
    // An unconfirmed AT+BAUD_RATE makes the saved rate unknown.
    b.Send("AT+BAUD_RATE=CONSOLE,230400\r\nAT+SETTINGS=SAVE\r\n");
    EXPECT(b.f.boot_baud() == 0 && !b.f.boot_baud_changed());
}

static void TestSnoopSettingsReset() {
    printf("snooped AT+SETTINGS=RESET: the console drops to 1 M, live and saved, and is moved back\n");
    Bridge b;
    b.Start(57600);
    b.Wait(BaudFollower::kSettleMs + 1);
    b.Send("AT+SETTINGS=RESET\r\n");
    b.console = 1000000;
    b.boot = 1000000;
    EXPECT(b.f.boot_baud() == 1000000 && !b.f.boot_baud_changed());  // Already 1 M.
    EXPECT(b.Hold());
    b.Wait(BaudFollower::kSettleMs + 1);
    EXPECT(b.renegotiations == 2 && b.last_target == 57600);
    EXPECT(b.console == 57600 && b.InSync());
}

static void TestSnoopReboot() {
    printf("snooped AT+REBOOT: back at the boot rate, moved to the host's after the boot\n");
    Bridge b;
    b.Start(230400);
    b.Wait(BaudFollower::kSettleMs + 1);
    b.Send("AT+REBOOT\r\n");
    b.console = b.boot;
    b.console_up = false;
    b.boot_done = b.now + 600;
    b.Wait(kBootWaitMs - 1);
    EXPECT(b.renegotiations == 1);
    b.Wait(2);
    EXPECT(b.renegotiations == 2);
    EXPECT(b.console == 230400 && b.InSync());
}

static void TestClockWrap() {
    printf("ms counter wrap\n");
    Bridge b;
    b.now = 0xFFFFFF00u;
    b.Start(0);
    b.Reset(false);
    b.HostBaud(57600);
    b.Wait(kBootWaitMs + BaudFollower::kSettleMs);
    EXPECT(b.renegotiations == 1);
    EXPECT(b.console == 57600 && b.InSync());
}

int main() {
    TestDividerMatchesDrivers();
    TestConsoleRange();
    TestRenegotiableRates();
    TestCandidates();
    TestPymavlinkOpen();
    TestRapidChanges();
    TestSameRateNoTraffic();
    TestChangeDuringRenegotiation();
    TestRomBootloaderDirect();
    TestResetAtBootRate();
    TestResetBackToBootRate();
    TestUnsupportedRatesDirect();
    TestSyncHighDefers();
    TestRenegotiationFailure();
    TestHostNeverSetRate();
    TestStartWithHostRate();
    TestSnoopBaudHint();
    TestSnoopSave();
    TestSnoopSettingsReset();
    TestSnoopReboot();
    TestClockWrap();
    if (failures) {
        printf("%d failure(s)\n", failures);
        return 1;
    }
    printf("all passed\n");
    return 0;
}
