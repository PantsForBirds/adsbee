// Host tests for ModemLines: the SET_CONTROL_LINE_STATE sequences real hosts send, played back
// against the bridge's SYNC / reset logic. Each (dtr, rts) pair is one control request.
#include <stdint.h>
#include <stdio.h>

#include "modem_lines.hh"

static int failures = 0;

#define EXPECT(cond)                                                     \
    do {                                                                 \
        if (!(cond)) {                                                   \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
            failures++;                                                  \
        }                                                                \
    } while (0)

// Mirrors BridgeRun(): the callback updates the lines, the loop performs a pending reset
// (RESET_N released kResetPulseMs later) and re-evaluates SYNC.
struct Bridge {
    static constexpr uint32_t kResetPulseMs = 50;

    ModemLines lines;
    uint32_t now = 1000;
    int resets = 0;
    bool last_reset_sync_high = false;
    bool sync_ever_high = false;  // Since the last ClearWatch().

    Bridge() { lines.Start(); }

    void Lines(bool dtr, bool rts) {
        lines.OnLineState(dtr, rts);
        Watch();
    }
    // One bridge loop iteration.
    void Loop() {
        if (lines.reset_pending()) {
            last_reset_sync_high = lines.reset_sync_high();
            Watch();
            now += kResetPulseMs;
            lines.OnResetDone(now);
            resets++;
        }
        Watch();
    }
    void Wait(uint32_t ms) {
        for (uint32_t i = 0; i < ms; i++) {
            now++;
            Loop();
        }
    }
    bool Sync() { return lines.SyncHigh(now); }
    void Watch() { sync_ever_high |= Sync(); }
    void ClearWatch() { sync_ever_high = false; }
};

// Kernel open asserts DTR and RTS: a DTR edge with SYNC low, i.e. a reset into the app.
static void OpenPort(Bridge& b) {
    b.Lines(true, true);
    b.Loop();
}

static void TestPlainTerminalOpenAndClose() {
    printf("plain terminal open / close\n");
    Bridge b;
    b.ClearWatch();
    OpenPort(b);
    EXPECT(b.resets == 1);
    EXPECT(!b.last_reset_sync_high);  // App boot.
    b.Wait(1000);
    b.Lines(false, false);  // Close with HUPCL: DTR and RTS dropped together.
    b.Wait(5000);
    EXPECT(!b.sync_ever_high);
    EXPECT(b.resets == 1);
}

static void TestCloseDropsIntentionalSleep() {
    printf("close while intentionally asleep wakes the module\n");
    Bridge b;
    OpenPort(b);
    b.Lines(true, false);  // RTS deasserted with the port open: sleep.
    EXPECT(b.Sync());
    b.Wait(2000);
    EXPECT(b.Sync());
    b.Lines(false, false);  // Close.
    EXPECT(!b.Sync());
    b.Wait(1000);
    EXPECT(!b.Sync());
    EXPECT(b.resets == 1);
}

static void TestIntentionalSleepAndWake() {
    printf("intentional sleep via RTS on an open port\n");
    Bridge b;
    OpenPort(b);
    b.Lines(true, false);
    b.Wait(10000);
    EXPECT(b.Sync());  // Held for as long as the host keeps it.
    b.Lines(true, true);
    EXPECT(!b.Sync());
    EXPECT(b.resets == 1);  // Toggling RTS never resets.
}

// Web console: connect parks the lines at SIGNALS_RUN (RTS asserted, DTR deasserted); Enter
// bootloader deasserts RTS, then DTR false -> true (50 ms) -> false; after flashing it asserts RTS
// and sends the ROM RESET command.
static void TestWebConsoleBootloaderEntry() {
    printf("web console Enter bootloader\n");
    Bridge b;
    OpenPort(b);
    b.Lines(false, true);  // SIGNALS_RUN.
    b.Wait(2000);
    b.ClearWatch();
    b.Lines(false, false);  // _setSync(true): RTS deasserted, DTR still low.
    b.Loop();
    b.Lines(false, false);  // _pulseReset: DTR false (no change).
    b.Loop();
    EXPECT(!b.sync_ever_high);  // No sleep glitch before the pulse.
    b.Lines(true, false);  // DTR assert edge.
    EXPECT(b.lines.reset_pending());
    EXPECT(b.Sync());
    b.Loop();
    EXPECT(b.resets == 2);
    EXPECT(b.last_reset_sync_high);  // Backdoor entry.
    EXPECT(b.Sync());
    b.Wait(50 - Bridge::kResetPulseMs + 1);
    b.Lines(false, false);  // DTR released.
    EXPECT(b.Sync());       // Still in the post-reset hold: the ROM samples SYNC high.
    b.Wait(ModemLines::kBackdoorHoldMs);
    EXPECT(!b.Sync());  // ROM bootloader running; SYNC no longer matters and returns low.
    b.Wait(30000);      // Flashing.
    EXPECT(!b.Sync());
    b.Lines(false, true);  // _leaveBootloader: SYNC low, then the ROM RESET command.
    EXPECT(!b.Sync());
    EXPECT(b.resets == 2);
    b.Lines(false, false);  // Close.
    b.Wait(1000);
    EXPECT(!b.Sync());
}

// Edge and release in the same tud_task() pass, before the loop performs the reset.
static void TestEdgeAndReleaseBeforeReset() {
    printf("DTR edge and release before the reset runs\n");
    Bridge b;
    OpenPort(b);
    b.Lines(false, false);
    b.Lines(true, false);
    b.Lines(false, false);
    EXPECT(b.lines.reset_pending());
    EXPECT(b.Sync());  // Latched at the edge.
    b.Loop();
    EXPECT(b.resets == 2);
    EXPECT(b.last_reset_sync_high);
    EXPECT(b.Sync());  // Hold.
    b.Wait(ModemLines::kBackdoorHoldMs + 1);
    EXPECT(!b.Sync());
}

// The pyserial snippet in firmware/adsbee_1421/README.md: rts = dtr = False before open. The kernel
// asserts both on open, pyserial then clears DTR and RTS, and the snippet pulses DTR.
static void TestPyserialReadmeSequence() {
    printf("pyserial README snippet\n");
    Bridge b;
    OpenPort(b);            // Kernel open: reset into the app.
    b.Lines(false, true);   // pyserial _update_dtr_state().
    b.Lines(false, false);  // pyserial _update_rts_state().
    b.Loop();
    EXPECT(!b.Sync());
    b.Wait(100);           // time.sleep(0.1)
    b.Lines(true, false);  // pulse_reset(): dtr = True
    b.Loop();
    EXPECT(b.resets == 2);
    EXPECT(b.last_reset_sync_high);  // Into the ROM bootloader.
    b.Wait(50 - Bridge::kResetPulseMs + 1);
    b.Lines(false, false);  // dtr = False
    EXPECT(b.Sync());
    b.Wait(100);
    EXPECT(b.Sync());
    b.Wait(ModemLines::kBackdoorHoldMs);
    EXPECT(!b.Sync());
    b.Wait(20000);  // Program.
    b.Lines(false, true);  // s.rts = True: SYNC low.
    b.Lines(true, true);   // pulse_reset()
    b.Loop();
    EXPECT(b.resets == 3);
    EXPECT(!b.last_reset_sync_high);  // Back into the app.
    b.Wait(50);
    b.Lines(false, true);
    b.Wait(100);
    b.ClearWatch();
    b.Lines(false, false);  // s.close() with HUPCL.
    b.Wait(5000);
    EXPECT(!b.sync_ever_high);
    EXPECT(b.resets == 3);
}

// A second backdoor entry long after the hold expired, RTS still deasserted throughout.
static void TestRepeatedBackdoorEntry() {
    printf("repeated backdoor entry\n");
    Bridge b;
    OpenPort(b);
    b.Lines(false, false);
    for (int i = 0; i < 3; i++) {
        b.Wait(5000);
        EXPECT(!b.Sync());
        b.Lines(true, false);
        b.Loop();
        EXPECT(b.last_reset_sync_high);
        b.Lines(false, false);
        EXPECT(b.Sync());
    }
    EXPECT(b.resets == 4);
}

static void TestRtsAssertEndsHold() {
    printf("asserting RTS during the hold drops SYNC\n");
    Bridge b;
    OpenPort(b);
    b.Lines(false, false);
    b.Lines(true, false);
    b.Loop();
    b.Lines(false, false);
    EXPECT(b.Sync());
    b.Lines(false, true);
    EXPECT(!b.Sync());
}

static void TestLinesBeforePassThrough() {
    printf("lines set before pass-through are not acted on\n");
    ModemLines lines;
    lines.Track(true, false);  // Port opened and RTS dropped while the Programmer was still flashing.
    lines.Start();
    EXPECT(!lines.SyncHigh(0));  // SYNC stays low until the host changes a line.
    lines.OnLineState(true, false);
    EXPECT(!lines.reset_pending());  // DTR was already asserted: no edge.
    EXPECT(lines.SyncHigh(0));
    lines.Start();  // Next session (BOOTSEL recheck).
    EXPECT(!lines.SyncHigh(0));
    lines.OnLineState(false, false);
    lines.OnLineState(true, true);
    EXPECT(lines.reset_pending());
    EXPECT(!lines.reset_sync_high());
}

static void TestHoldAcrossCounterWrap() {
    printf("hold across the ms counter wrap\n");
    ModemLines lines;
    lines.Start();
    lines.OnLineState(false, false);
    lines.OnLineState(true, false);
    uint32_t t = UINT32_MAX - 100;
    lines.OnResetDone(t);
    lines.OnLineState(false, false);
    EXPECT(lines.SyncHigh(t + 50));
    EXPECT(lines.SyncHigh(t + ModemLines::kBackdoorHoldMs - 1));  // Wrapped past zero.
    EXPECT(!lines.SyncHigh(t + ModemLines::kBackdoorHoldMs));
    EXPECT(!lines.SyncHigh(t + 50));  // Expired holds stay expired.
}

int main() {
    TestPlainTerminalOpenAndClose();
    TestCloseDropsIntentionalSleep();
    TestIntentionalSleepAndWake();
    TestWebConsoleBootloaderEntry();
    TestEdgeAndReleaseBeforeReset();
    TestPyserialReadmeSequence();
    TestRepeatedBackdoorEntry();
    TestRtsAssertEndsHold();
    TestLinesBeforePassThrough();
    TestHoldAcrossCounterWrap();
    if (failures) {
        printf("%d failure(s)\n", failures);
        return 1;
    }
    printf("all passed\n");
    return 0;
}
