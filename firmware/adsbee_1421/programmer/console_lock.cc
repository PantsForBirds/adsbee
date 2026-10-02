#include "console_lock.hh"

#include "edge_capture.hh"
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "status.hh"
#include "target_ctl.hh"
#include "target_uart.hh"
#include "tusb.h"

static RateWatch* watch = nullptr;

void ConsoleWatchInit() {
    EdgeCaptureInit();
    static RateWatch instance(EdgeCapture(), clock_get_hz(clk_sys));
    watch = &instance;
    watch->Start(TargetUartGetBaud(), time_us_64());
}

RateWatch& ConsoleWatch() { return *watch; }

void ConsoleWatchApply(const RateWatch::Action& action) {
    switch (action.kind) {
        case RateWatch::Action::kRetune:
            TargetUartSetBaud(action.baud);  // Also drops what came in at the old rate.
            (void)TargetUartTakeRxErrors();
            break;
        case RateWatch::Action::kBreakStart:
            TargetUartHoldTxLow(true);
            break;
        case RateWatch::Action::kBreakEnd:
            TargetUartHoldTxLow(false);
            break;
        case RateWatch::Action::kNone:
            break;
    }
}

uint32_t ConsoleLock(AtAbortFn abort) {
    TargetResetIntoApp();
    watch->OnAppReset(time_us_64());
    uint8_t discard[64];
    while (true) {
        tud_task();
        StatusUpdate();
        if (abort != nullptr && abort()) break;
        TargetUartPollRx();
        watch->OnRxErrors(TargetUartTakeRxErrors());
        ConsoleWatchApply(watch->Poll(time_us_64(), false, TargetUartRxReceived()));
        while (TargetUartRead(discard, sizeof(discard)) > 0) {
        }
        if (watch->BreakActive()) continue;  // The answer can come before the break ends.
        if (watch->phase() == RateWatch::Phase::kLocked) return watch->baud();
        if (!watch->Locking()) return 0;  // kUnknown: the asks ran out.
    }
    TargetUartHoldTxLow(false);  // Aborted: in case a break was running.
    return 0;
}
