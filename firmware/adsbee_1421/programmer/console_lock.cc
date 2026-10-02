#include "console_lock.hh"

#include "autobaud.hh"
#include "autobaud_edges.pio.h"  // Generated from autobaud_edges.pio by pico_generate_pio_header().
#include "board.hh"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "status.hh"
#include "target_ctl.hh"
#include "target_uart.hh"
#include "tusb.h"

// Trigger timing. The module checks its RX line early in its boot (CommsManager::Init(), done 12 to 15 ms after RESET_N
// is released, 5 ms debounce included) and on a SYNC wake once the LR2021 is back up (CommsManager::Resume(), done 15
// to 20 ms after SYNC drops), as measured. The holds are about twice that.
static constexpr uint32_t kResetHoldMs = 30;  // RX held low after RESET_N is released.
static constexpr uint32_t kSyncPulseMs = 20;  // SYNC high: the module goes to sleep.
static constexpr uint32_t kWakeHoldMs = 40;   // RX held low after SYNC drops.
// How long the "UU" may take after the line is released: the console is ready (saved rate applied) about 48 ms after
// RESET_N is released, so 18 ms after the release, and it answers a wake at once. Both include a 50 ms margin.
static constexpr uint32_t kResetAnswerMs = 70;
static constexpr uint32_t kWakeAnswerMs = 50;

// "UU" is 20 edges; the buffer also catches whatever comes ahead of it.
static constexpr size_t kMaxEdges = 64;
// No edge for this long ends a burst: longer than any gap inside "UU" (one bit, 104 us at 9600 baud).
static constexpr uint32_t kBurstGapUs = 1000;

// The rates module firmware without the trigger (0.3.11-rc3 and earlier) accepts, factory default first.
static constexpr uint32_t kLegacyConsoleBauds[] = {1000000, 921600, 460800, 230400, 115200};

static ConsoleLockInfo last_lock;

const ConsoleLockInfo& LastConsoleLock() { return last_lock; }

// ---- Edge capture: autobaud_edges.pio on PIO1 (the WS2812 LED uses PIO0), drained by DMA ----

static const PIO kCapturePio = pio1;
static int capture_sm = -1;
static uint capture_offset = 0;
static int capture_dma = -1;
static uint32_t capture_buf[kMaxEdges];
static bool capture_first_edge_falling = true;

static void CaptureStart() {
    if (capture_sm < 0) {
        capture_offset = pio_add_program(kCapturePio, &autobaud_edges_program);
        capture_sm = (int)pio_claim_unused_sm(kCapturePio, true);
        capture_dma = dma_claim_unused_channel(true);
    }
    pio_sm_set_enabled(kCapturePio, capture_sm, false);
    dma_channel_abort(capture_dma);

    // The UART keeps the pin; PIO reads the pad through the jmp pin.
    pio_sm_config config = autobaud_edges_program_get_default_config(capture_offset);
    sm_config_set_jmp_pin(&config, kPinUartRx);
    sm_config_set_in_shift(&config, false, true, 32);  // Autopush at every `in x, 32`.
    sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_RX);
    capture_first_edge_falling = gpio_get(kPinUartRx);
    uint start = capture_offset + (capture_first_edge_falling ? autobaud_edges_offset_high : autobaud_edges_offset_low);
    pio_sm_init(kCapturePio, capture_sm, start, &config);
    pio_sm_exec(kCapturePio, capture_sm, pio_encode_mov_not(pio_x, pio_null));

    dma_channel_config dma_config = dma_channel_get_default_config(capture_dma);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_config, false);
    channel_config_set_write_increment(&dma_config, true);
    channel_config_set_dreq(&dma_config, pio_get_dreq(kCapturePio, capture_sm, false));
    dma_channel_configure(capture_dma, &dma_config, capture_buf, &kCapturePio->rxf[capture_sm], kMaxEdges, true);
    pio_sm_set_enabled(kCapturePio, capture_sm, true);
}

static size_t CaptureCount() { return kMaxEdges - dma_channel_hw_addr(capture_dma)->transfer_count; }

static void CaptureStop() {
    pio_sm_set_enabled(kCapturePio, capture_sm, false);
    dma_channel_abort(capture_dma);
}

// Waits up to timeout_ms for a "UU" on the RX pin and returns its rate, or 0.
static uint32_t CaptureMeasure(uint32_t timeout_ms, AtAbortFn abort) {
    absolute_time_t deadline = delayed_by_ms(get_absolute_time(), timeout_ms);
    absolute_time_t last_edge = get_absolute_time();
    size_t seen = 0;
    while (absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
        tud_task();
        StatusUpdate();
        if (abort != nullptr && abort()) return 0;
        size_t count = CaptureCount();
        if (count != seen) {
            seen = count;
            last_edge = get_absolute_time();
        }
        bool burst_over = seen == kMaxEdges || absolute_time_diff_us(last_edge, get_absolute_time()) > kBurstGapUs;
        if (seen <= Autobaud::kMinIntervals || !burst_over) continue;
        uint32_t cycles[kMaxEdges];
        Autobaud::CountsToCycles(capture_buf, seen, cycles);
        Autobaud::Measurement m =
            Autobaud::Measure(cycles, seen, capture_first_edge_falling, clock_get_hz(clk_sys));
        if (m.baud != 0) return m.baud;
        CaptureStart();  // Something else: look at whatever comes next.
        seen = 0;
    }
    return 0;
}

// sleep_ms that keeps USB and the status LED serviced. Returns false if abort() returned true.
static bool WaitMs(uint32_t ms, AtAbortFn abort) {
    absolute_time_t deadline = delayed_by_ms(get_absolute_time(), ms);
    while (absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
        tud_task();
        StatusUpdate();
        if (abort != nullptr && abort()) return false;
    }
    return true;
}

uint32_t ConsoleLock(ConsoleTrigger trigger, AtAbortFn abort) {
    absolute_time_t start = get_absolute_time();
    last_lock = {};

    TargetUartHoldTxLow(true);
    bool waited;
    uint32_t answer_ms;
    if (trigger == ConsoleTrigger::kReset) {
        TargetResetIntoApp();
        waited = WaitMs(kResetHoldMs, abort);
        answer_ms = kResetAnswerMs;
    } else {
        TargetSetSync(true);
        waited = WaitMs(kSyncPulseMs, nullptr);
        TargetSetSync(false);
        waited = waited && WaitMs(kWakeHoldMs, abort);
        answer_ms = kWakeAnswerMs;
    }
    CaptureStart();
    TargetUartHoldTxLow(false);
    uint32_t measured = waited ? CaptureMeasure(answer_ms, abort) : 0;
    CaptureStop();
    if (abort != nullptr && abort()) return 0;

    uint32_t console = 0;
    if (measured != 0) {
        last_lock.measured_baud = measured;
        TargetUartSetBaud(Autobaud::SnapToConsoleRate(measured));
        console = AtProbeConsoleBaud();
        if (console == 0) console = AtProbeConsoleBaud();
    }
    for (size_t i = 0; console == 0 && i < sizeof(kLegacyConsoleBauds) / sizeof(kLegacyConsoleBauds[0]); i++) {
        if (abort != nullptr && abort()) return 0;
        TargetUartSetBaud(kLegacyConsoleBauds[i]);
        console = AtProbeConsoleBaud();
    }
    if (console != 0 && console != TargetUartGetBaud()) TargetUartSetBaud(console);
    last_lock.console_baud = console;
    last_lock.elapsed_ms = (uint32_t)(absolute_time_diff_us(start, get_absolute_time()) / 1000);
    return console;
}

// Lets the console finish switching (it closes and reopens its UART after the OK) while USB stays serviced.
static void WaitForSwitch() {
    for (int i = 0; i < 5; i++) (void)TargetUartReadByteTimeout(10);
}

// With the UART at the console's rate, moves it to `target` and confirms it answers there.
static bool MoveConsole(uint32_t target) {
    if (!AtSetConsoleBaud(target)) return false;
    WaitForSwitch();
    TargetUartSetBaud(target);
    return AtProbeAlive() || AtProbeAlive();
}

uint32_t ConsoleRenegotiate(uint32_t target_baud, const uint32_t* likely, size_t num_likely, AtAbortFn abort) {
    // Where the console probably is: one exchange each, and straight to the move when it answers.
    for (size_t i = 0; i < num_likely; i++) {
        uint32_t from = likely[i];
        if (from == 0) continue;
        bool tried = false;
        for (size_t j = 0; j < i; j++) tried |= likely[j] == from;
        if (tried) continue;
        if (abort != nullptr && abort()) return 0;
        TargetUartSetBaud(from);
        if (target_baud == 0 || from == target_baud) {
            if (AtProbeAlive()) return from;
            continue;
        }
        // An OK proves the console was at `from`. Without one it may be elsewhere (a probe would cost as much as the
        // command), so move on.
        if (MoveConsole(target_baud)) return target_baud;
    }

    // Lost: find it with the trigger, then move it from wherever it is.
    uint32_t found = ConsoleLock(ConsoleTrigger::kSyncWake, abort);
    if (found == 0) {
        if (target_baud != 0) TargetUartSetBaud(target_baud);
        return 0;
    }
    if (target_baud == 0 || found == target_baud) return found;
    if (abort != nullptr && abort()) return found;
    if (MoveConsole(target_baud)) return target_baud;
    // Refused (an image that doesn't accept the rate) or lost on the way: back to where it was found.
    TargetUartSetBaud(found);
    return AtProbeAlive() ? found : 0;
}
