#include "bridge.hh"

#include "board.hh"
#include "bootsel.hh"
#include "host_line_coding.hh"
#include "modem_lines.hh"
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "status.hh"
#include "target_ctl.hh"
#include "target_uart.hh"
#include "tusb.h"

static volatile bool bridge_active = false;
static volatile uint32_t host_baud = 0;
static volatile uint32_t expected_console_baud = kConsoleBaud;
static volatile bool baud_change_pending = false;
static ModemLines lines;  // Only touched from tud_task() callbacks and the bridge loop.

static uint32_t NowMs() { return to_ms_since_boot(get_absolute_time()); }

static constexpr bool IsConsoleOrBootloaderBaud(uint32_t baud) {
    if (baud == kBootloaderBaud) return true;
    for (uint32_t candidate : kConsoleBaudCandidates) {
        if (baud == candidate) return true;
    }
    return false;
}
static_assert(!IsConsoleOrBootloaderBaud(kRebootToBootselBaud),
              "The reboot-to-BOOTSEL baud must not be a rate pass-through tools use");

extern "C" void tud_cdc_line_coding_cb(uint8_t itf, const cdc_line_coding_t* coding) {
    (void)itf;
    switch (ClassifyHostBaud(coding->bit_rate)) {
        case HostBaudAction::kIgnore:
            return;
        case HostBaudAction::kRebootToBootsel:
            // Checked in every Programmer state, including while it flashes the module: an interrupted flash is redone
            // by the CRC check on the next boot. Does not return, so the magic baud never reaches host_baud or the UART.
            reset_usb_boot(0, 0);
            return;
        case HostBaudAction::kApply:
            host_baud = coding->bit_rate;
            if (bridge_active) baud_change_pending = true;
            return;
    }
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

uint32_t BridgeHostBaud() { return host_baud; }

void BridgeSetExpectedConsoleBaud(uint32_t baud) { expected_console_baud = baud; }

BridgeExit BridgeRun() {
    StatusSet(Status::kPassthrough);
    baud_change_pending = false;
    lines.Start();
    bridge_active = true;

    absolute_time_t next_bootsel_poll = get_absolute_time();
    uint8_t buf[64];

    while (true) {
        tud_task();
        StatusUpdate();
        TargetUartPumpTx();

        if (baud_change_pending) {
            baud_change_pending = false;
            TargetUartSetBaud(host_baud);
        }

        if (lines.reset_pending()) {
            bool sync_low_at_reset = !lines.reset_sync_high();
            TargetPulseReset();  // SYNC already holds the level latched at the DTR edge.
            lines.OnResetDone(NowMs());
            // A reset with SYNC low reboots into the app, whose console comes up at its saved
            // baud (factory default 1 M). If the host's line coding differs from the rate the
            // console was last negotiated to, hand back to the caller to re-negotiate. A host
            // whose rate matches is assumed in sync; a device saved at some other rate is
            // recovered by the host probing (line-coding changes retune the Programmer's UART live) or
            // by the BOOTSEL recheck.
            if (sync_low_at_reset && host_baud != 0 && host_baud != expected_console_baud) {
                bridge_active = false;
                return BridgeExit::kHostResetTarget;
            }
        }

        TargetSetSync(lines.SyncHigh(NowMs()));  // Ends the post-reset backdoor hold.

        // Device -> host.
        size_t len = TargetUartRead(buf, sizeof(buf));
        if (len > 0) {
            size_t written = 0;
            while (written < len) {
                written += tud_cdc_write(buf + written, (uint32_t)(len - written));
                tud_cdc_write_flush();
                if (written < len) tud_task();  // FIFO full: let USB drain.
            }
        } else {
            tud_cdc_write_flush();
        }

        // Host -> device. Only take what the UART TX ring can hold so the CDC FIFO provides
        // natural backpressure to the host.
        size_t tx_free = TargetUartTxFree();
        if (tx_free > 0 && tud_cdc_available()) {
            uint32_t take = (uint32_t)(tx_free < sizeof(buf) ? tx_free : sizeof(buf));
            uint32_t got = tud_cdc_read(buf, take);
            if (got > 0) TargetUartWriteNonblocking(buf, got);
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
