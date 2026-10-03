#include "bridge.hh"

#include "board.hh"
#include "bootsel.hh"
#include "console_lock.hh"
#include "host_line_coding.hh"
#include "modem_lines.hh"
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "status.hh"
#include "target_ctl.hh"
#include "target_uart.hh"
#include "tusb.h"

// Main loop and tud_task() callbacks only; never touched from interrupts.
static bool bridge_active = false;
static ModemLines lines;

static uint32_t NowMs() { return to_ms_since_boot(get_absolute_time()); }

static_assert(kRebootToBootselBaud != kBootloaderBaud && !ConsoleBaud::IsSupported(kRebootToBootselBaud),
              "The reboot-to-BOOTSEL baud must not be a rate the module uses");

extern "C" void tud_cdc_line_coding_cb(uint8_t itf, const cdc_line_coding_t* coding) {
    (void)itf;
    // Checked in every state, even mid-flash: the next boot's CRC check redoes an interrupted flash.
    if (ClassifyHostBaud(coding->bit_rate) == HostBaudAction::kRebootToBootsel) reset_usb_boot(0, 0);
}

extern "C" void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void)itf;
    if (!bridge_active) {
        lines.Track(dtr, rts);
        return;
    }
    lines.OnLineState(dtr, rts);
    TargetSetSync(lines.SyncHigh(NowMs()));
}

// Forwards the console bytes in the RX ring up to `limit` (TargetUartRxReceived()'s count) to the host.
static void ForwardConsole(uint32_t limit) {
    uint8_t buf[64];
    int32_t pending = (int32_t)(limit - TargetUartRxConsumed());
    while (pending > 0) {
        size_t len = TargetUartRead(buf, pending < (int32_t)sizeof(buf) ? (size_t)pending : sizeof(buf));
        if (len == 0) break;
        pending -= (int32_t)len;
        size_t written = 0;
        while (written < len) {
            written += tud_cdc_write(buf + written, (uint32_t)(len - written));
            if (written < len) {
                tud_cdc_write_flush();
                tud_task();  // FIFO full: let USB drain.
            }
        }
    }
}

BridgeExit BridgeRun() {
    StatusSet(Status::kPassthrough);
    lines.Start();
    RateWatch& watch = ConsoleWatch();
    watch.Start(TargetUartGetBaud(), time_us_64());
    (void)TargetUartTakeRxErrors();
    bridge_active = true;

    absolute_time_t next_bootsel_poll = get_absolute_time();
    uint8_t buf[64];
    bool was_locking = false;
    bool was_unknown = false;

    while (true) {
        tud_task();
        StatusUpdate();
        TargetUartPumpTx();

        if (lines.reset_pending()) {
            // SYNC high: the ROM bootloader at kBootloaderBaud (or the app, if the backdoor is off; the watch
            // follows it). SYNC low: the app at its saved rate; lock onto its boot "UU".
            TargetUartHoldTxLow(false);  // Ends a break in progress.
            if (lines.reset_sync_high()) {
                TargetPulseReset();  // SYNC already holds the level latched at the DTR edge.
                lines.OnResetDone(NowMs());
                if (TargetUartGetBaud() != kBootloaderBaud) TargetUartSetBaud(kBootloaderBaud);
                watch.OnBootloaderReset(kBootloaderBaud, time_us_64());
            } else {
                TargetResetIntoApp();
                lines.OnResetDone(NowMs());
                watch.OnAppReset(time_us_64());
            }
            (void)TargetUartTakeRxErrors();
            continue;
        }

        bool sync_high = lines.SyncHigh(NowMs());
        TargetSetSync(sync_high);  // Ends the post-reset backdoor hold.

        TargetUartPollRx();
        watch.OnRxErrors(TargetUartTakeRxErrors());
        RateWatch::Action action = watch.Poll(time_us_64(), sync_high, TargetUartRxReceived());
        if (action.kind == RateWatch::Action::kRetune) ForwardConsole(action.keep);  // Arrived before the change.
        ConsoleWatchApply(action);
        bool locking = watch.HoldHostData();
        if (locking != was_locking) {
            StatusSet(locking ? Status::kNegotiating : Status::kPassthrough);
            was_locking = locking;
        }
        bool unknown = watch.phase() == RateWatch::Phase::kUnknown;
        if (unknown && !was_unknown) {
            CdcPrintf(
                "\r\n[ADSBee 1421 Programmer] Console not found; UART left at %lu baud, asking again every "
                "%lu s.\r\n",
                (unsigned long)TargetUartGetBaud(), (unsigned long)(RateWatch::kUnknownAskIntervalMs / 1000));
        }
        was_unknown = unknown;

        // Device -> host, kConsoleDelayUs after arrival. Held while a rate hint resolves; dropped during an ask or
        // reset, since they arrive at the wrong rate.
        if (watch.DropConsoleData()) {
            while (TargetUartRead(buf, sizeof(buf)) > 0) {
            }
        } else if (!watch.HoldConsoleData()) {
            ForwardConsole(watch.Forwardable());
        }
        tud_cdc_write_flush();

        // Host -> device. Only take what the UART TX ring can hold so the CDC FIFO provides natural backpressure to
        // the host. Held during a lock.
        if (!watch.HoldHostData()) {
            size_t take = TargetUartTxFree();
            if (take > sizeof(buf)) take = sizeof(buf);
            if (take > 0 && tud_cdc_available()) {
                size_t len = tud_cdc_read(buf, (uint32_t)take);
                TargetUartWriteNonblocking(buf, len);
            }
        }

        if (absolute_time_diff_us(get_absolute_time(), next_bootsel_poll) <= 0) {
            next_bootsel_poll = delayed_by_ms(get_absolute_time(), kBootselPollMs);
            BootselEvent event = PollBootsel();
            if (event != BootselEvent::kNone) {
                while (GetBootselButton()) sleep_ms(20);
                sleep_ms(50);   // Debounce the release.
                PollBootsel();  // Consume the release so the next loop's poll sees no stale edge.
                TargetUartHoldTxLow(false);
                bridge_active = false;
                return event == BootselEvent::kLongPress ? BridgeExit::kEraseSettings : BridgeExit::kRecheck;
            }
        }
    }
}
