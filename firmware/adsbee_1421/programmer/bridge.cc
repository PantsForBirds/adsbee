#include "bridge.hh"

#include "board.hh"
#include "bootsel.hh"
#include "pico/stdlib.h"
#include "status.hh"
#include "target_ctl.hh"
#include "target_uart.hh"
#include "tusb.h"

static volatile bool bridge_active = false;
static volatile uint32_t host_baud = 0;
static volatile uint32_t expected_console_baud = kConsoleBaud;
static volatile bool baud_change_pending = false;
static volatile bool reset_pending = false;
static volatile bool sync_low_at_reset = false;
static bool last_dtr = false;

extern "C" void tud_cdc_line_coding_cb(uint8_t itf, const cdc_line_coding_t* coding) {
    (void)itf;
    if (coding->bit_rate == 0) return;
    host_baud = coding->bit_rate;
    if (bridge_active) baud_change_pending = true;
}

extern "C" void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void)itf;
    if (!bridge_active) {
        last_dtr = dtr;
        return;
    }
    // Adapter emulation: an asserted modem-control bit drives the physical pin low.
    TargetSetSync(!rts);
    if (dtr && !last_dtr) {
        sync_low_at_reset = rts;  // RTS asserted = SYNC pin low = normal app boot after reset.
        reset_pending = true;
    }
    last_dtr = dtr;
}

uint32_t BridgeHostBaud() { return host_baud; }

void BridgeSetExpectedConsoleBaud(uint32_t baud) { expected_console_baud = baud; }

BridgeExit BridgeRun() {
    StatusSet(Status::kPassthrough);
    baud_change_pending = false;
    reset_pending = false;
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

        if (reset_pending) {
            reset_pending = false;
            TargetPulseReset();  // SYNC already tracks RTS, so this honors backdoor entry.
            // A reset with SYNC low reboots into the app, whose console comes up at its saved
            // baud (factory default 1 M). If the host's line coding differs from the rate the
            // console was last negotiated to, hand back to the caller to re-negotiate. A host
            // whose rate matches is assumed in sync; a device saved at some other rate is
            // recovered by the host probing (line-coding changes retune the jig UART live) or
            // by the BOOTSEL recheck.
            if (sync_low_at_reset && host_baud != 0 && host_baud != expected_console_baud) {
                bridge_active = false;
                return BridgeExit::kHostResetTarget;
            }
        }

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
