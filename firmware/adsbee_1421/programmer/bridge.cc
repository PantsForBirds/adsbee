#include "bridge.hh"

#include "baud_follower.hh"
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

// Only touched from the main loop and the tud_task() callbacks it runs (the bridge loop, and the AT client's waits
// during a renegotiation), never from interrupts.
static bool bridge_active = false;
static uint32_t host_baud = 0;
static uint32_t expected_console_baud = kConsoleBaud;
static ModemLines lines;
static BaudFollower follower;

static uint32_t NowMs() { return to_ms_since_boot(get_absolute_time()); }

static_assert(kRebootToBootselBaud != kBootloaderBaud && !IsRenegotiableBaud(kRebootToBootselBaud),
              "The reboot-to-BOOTSEL baud must not be a rate pass-through tools use");
static_assert(IsRenegotiableBaud(kConsoleBaud), "The factory console rate must be renegotiable");

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
            if (bridge_active) follower.OnHostBaud(host_baud, NowMs());
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

// A reset requested by the host outranks a renegotiation in progress: a ROM bootloader client syncs right after it.
static bool RenegotiationAborted() { return lines.reset_pending(); }

// Carries out a BaudFollower::Step::kRenegotiate. Host data stays in the CDC FIFO meanwhile (HoldHostData()); console
// output read during it is consumed by the AT client.
static void Renegotiate(uint32_t target_baud) {
    StatusSet(Status::kNegotiating);
    uint32_t likely[] = {follower.hinted_baud(), follower.console_baud(), TargetUartGetBaud()};
    uint32_t found = ConsoleRenegotiate(target_baud, likely, sizeof(likely) / sizeof(likely[0]), RenegotiationAborted);
    follower.OnRenegotiated(found);
    if (found != target_baud && !RenegotiationAborted()) {
        // Pass-through carries on either way. The CDC baud is virtual, so a host reading the console at another rate
        // still gets clean data; the warning says why the rate didn't follow.
        if (found == 0) {
            CdcPrintf("\r\n[ADSBee 1421 Programmer] Console not found; UART set to %lu baud without it.\r\n",
                      (unsigned long)target_baud);
        } else {
            CdcPrintf("\r\n[ADSBee 1421 Programmer] Console stays at %lu baud: AT+BAUD_RATE=CONSOLE,%lu failed.\r\n",
                      (unsigned long)found, (unsigned long)target_baud);
        }
    }
    StatusSet(Status::kPassthrough);
}

// Resets the module into the application (SYNC low) and finds its console with the autobaud lock.
static void ResetAndLock() {
    StatusSet(Status::kNegotiating);
    uint32_t found = ConsoleLock(ConsoleTrigger::kReset, RenegotiationAborted);
    follower.OnReset(false, found);
    StatusSet(Status::kPassthrough);
}

BridgeExit BridgeRun() {
    StatusSet(Status::kPassthrough);
    lines.Start();
    follower.Start(expected_console_baud, host_baud, NowMs());
    bridge_active = true;

    absolute_time_t next_bootsel_poll = get_absolute_time();
    uint8_t buf[64];

    while (true) {
        tud_task();
        StatusUpdate();
        TargetUartPumpTx();

        if (lines.reset_pending()) {
            // SYNC high: the ROM bootloader, which takes the host's rate as it is. SYNC low: the application, whose
            // console comes back at its saved rate; the follower retunes, or renegotiates to the host's rate.
            if (lines.reset_sync_high()) {
                TargetPulseReset();  // SYNC already holds the level latched at the DTR edge.
                lines.OnResetDone(NowMs());
                follower.OnReset(true, 0);
            } else {
                lines.OnResetDone(NowMs());
                ResetAndLock();  // Restarts if the host asks for another reset meanwhile.
            }
            continue;
        }

        bool sync_high = lines.SyncHigh(NowMs());
        TargetSetSync(sync_high);  // Ends the post-reset backdoor hold.

        BaudFollower::Step step = follower.Poll(NowMs(), sync_high);
        if (step.kind == BaudFollower::Step::kApplyDirect) {
            if (step.baud != TargetUartGetBaud()) TargetUartSetBaud(step.baud);
        } else if (step.kind == BaudFollower::Step::kRenegotiate) {
            Renegotiate(step.baud);
        } else if (step.kind == BaudFollower::Step::kReset) {
            ResetAndLock();
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
        if (tx_free > 0 && tud_cdc_available() && !follower.HoldHostData(sync_high)) {
            uint32_t take = (uint32_t)(tx_free < sizeof(buf) ? tx_free : sizeof(buf));
            uint32_t got = tud_cdc_read(buf, take);
            if (got > 0) {
                follower.OnHostBytes(buf, got, NowMs());
                TargetUartWriteNonblocking(buf, got);
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
