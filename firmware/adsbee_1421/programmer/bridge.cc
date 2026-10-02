#include "bridge.hh"

#include "board.hh"
#include "bootsel.hh"
#include "console_lock.hh"
#include "host_line_coding.hh"
#include "modem_lines.hh"
#include "rate_tracker.hh"
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "status.hh"
#include "target_ctl.hh"
#include "target_uart.hh"
#include "tusb.h"

// Only touched from the main loop and the tud_task() callbacks it runs (the bridge loop, and the waits inside a lock),
// never from interrupts.
static bool bridge_active = false;
static uint32_t expected_console_baud = kConsoleBaud;
static ModemLines lines;
static RateTracker tracker;

// Host bytes read from the CDC FIFO that the tracker held back (they follow a command that switches the console's
// rate); they go first once the switch is done.
static uint8_t host_pending[64];
static size_t host_pending_len = 0;
static size_t host_pending_off = 0;

static uint32_t NowMs() { return to_ms_since_boot(get_absolute_time()); }

static_assert(kRebootToBootselBaud != kBootloaderBaud && !ConsoleBaud::IsSupported(kRebootToBootselBaud),
              "The reboot-to-BOOTSEL baud must not be a rate the module uses");

extern "C" void tud_cdc_line_coding_cb(uint8_t itf, const cdc_line_coding_t* coding) {
    (void)itf;
    // The USB baud is virtual (rate_tracker.hh): only the magic baud does anything. Checked in every Programmer
    // state, including while it flashes the module: an interrupted flash is redone by the CRC check on the next boot.
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

void BridgeSetExpectedConsoleBaud(uint32_t baud) { expected_console_baud = baud; }

// A reset requested by the host outranks a lock in progress: a ROM bootloader client syncs right after it.
static bool LockAborted() { return lines.reset_pending(); }

// Finds the console with the autobaud trigger. On failure the UART goes to fallback_baud and the host gets one line
// saying so (the console may be gone: unplugged, or a module without the trigger at a rate the legacy probe misses).
static void Lock(ConsoleTrigger trigger, uint32_t fallback_baud) {
    StatusSet(Status::kNegotiating);
    uint32_t found = ConsoleLock(trigger, LockAborted);
    if (found == 0 && !LockAborted()) {
        if (fallback_baud != 0) TargetUartSetBaud(fallback_baud);
        CdcPrintf("\r\n[ADSBee 1421 Programmer] Console not found; UART left at %lu baud.\r\n",
                  (unsigned long)TargetUartGetBaud());
    }
    (void)TargetUartTakeRxErrors();  // The trigger and the probes leave errors behind.
    tracker.OnLocked(found, NowMs());
    StatusSet(Status::kPassthrough);
}

BridgeExit BridgeRun() {
    StatusSet(Status::kPassthrough);
    lines.Start();
    tracker.Start(expected_console_baud, NowMs());
    (void)TargetUartTakeRxErrors();
    bridge_active = true;

    absolute_time_t next_bootsel_poll = get_absolute_time();
    uint8_t buf[64];

    while (true) {
        tud_task();
        StatusUpdate();
        TargetUartPumpTx();

        if (lines.reset_pending()) {
            // SYNC high: the ROM bootloader, which auto-bauds to the rate it is sent at, so the UART goes to the
            // bootloader rate whatever the host's line coding. SYNC low: the application, whose console comes back at
            // its saved rate; lock onto it.
            if (lines.reset_sync_high()) {
                TargetPulseReset();  // SYNC already holds the level latched at the DTR edge.
                lines.OnResetDone(NowMs());
                if (TargetUartGetBaud() != kBootloaderBaud) TargetUartSetBaud(kBootloaderBaud);
                tracker.OnReset(true, 0, NowMs());
            } else {
                lines.OnResetDone(NowMs());
                StatusSet(Status::kNegotiating);
                uint32_t found = ConsoleLock(ConsoleTrigger::kReset, LockAborted);
                (void)TargetUartTakeRxErrors();
                tracker.OnReset(false, found, NowMs());  // Restarts if the host asks for another reset meanwhile.
                StatusSet(Status::kPassthrough);
            }
            continue;
        }

        bool sync_high = lines.SyncHigh(NowMs());
        TargetSetSync(sync_high);  // Ends the post-reset backdoor hold.

        uint32_t errors = TargetUartTakeRxErrors();
        if (!sync_high) tracker.OnRxErrors(errors, NowMs());  // A sleeping module's line says nothing about the rate.

        RateTracker::Step step = tracker.Poll(NowMs(), sync_high);
        if (step.kind == RateTracker::Step::kWakeLock) {
            Lock(ConsoleTrigger::kSyncWake, step.baud);
            continue;
        } else if (step.kind == RateTracker::Step::kResetAndLock) {
            Lock(ConsoleTrigger::kReset, step.baud);
            continue;
        }

        // Device -> host.
        size_t len = TargetUartRead(buf, sizeof(buf));
        if (len > 0) {
            // Ends at the console's OK when a rate switch is acknowledged: what came after it is at the new rate.
            len = tracker.OnConsoleBytes(buf, len, NowMs());
            step = tracker.Poll(NowMs(), sync_high);
            if (step.kind == RateTracker::Step::kRetune) {
                // At once, before the console's next byte at the new rate. Also drops what was received meanwhile.
                TargetUartSetBaud(step.baud);
                (void)TargetUartTakeRxErrors();
                tracker.OnRetuned(NowMs());
            }
            size_t written = 0;
            while (written < len) {
                written += tud_cdc_write(buf + written, (uint32_t)(len - written));
                tud_cdc_write_flush();
                if (written < len) tud_task();  // FIFO full: let USB drain.
            }
        } else {
            tud_cdc_write_flush();
        }

        // Host -> device. Only take what the UART TX ring can hold so the CDC FIFO provides natural backpressure to
        // the host. Held while the console switches rates.
        if (!tracker.HoldHostData()) {
            if (host_pending_off == host_pending_len && tud_cdc_available()) {
                host_pending_len = tud_cdc_read(host_pending, sizeof(host_pending));
                host_pending_off = 0;
            }
            size_t avail = host_pending_len - host_pending_off;
            size_t tx_free = TargetUartTxFree();
            size_t take = avail < tx_free ? avail : tx_free;
            if (take > 0) {
                size_t send = tracker.OnHostBytes(host_pending + host_pending_off, take, NowMs());
                TargetUartWriteNonblocking(host_pending + host_pending_off, send);
                host_pending_off += send;
            }
        }

        if (absolute_time_diff_us(get_absolute_time(), next_bootsel_poll) <= 0) {
            next_bootsel_poll = delayed_by_ms(get_absolute_time(), kBootselPollMs);
            BootselEvent event = PollBootsel();
            if (event != BootselEvent::kNone) {
                while (GetBootselButton()) sleep_ms(20);
                sleep_ms(50);  // Debounce the release.
                PollBootsel();  // Consume the release so the next loop's poll sees no stale edge.
                bridge_active = false;
                return event == BootselEvent::kLongPress ? BridgeExit::kEraseSettings : BridgeExit::kRecheck;
            }
        }
    }
}
