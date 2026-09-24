// ADSBee 1421 programmer + pass-through jig for the Waveshare RP2040-Zero.
//
// At power-up the jig enters the CC1314's ROM UART bootloader (SYNC backdoor + reset pulse) and
// compares the on-chip flash against the baked-in adsbee_1421 image using the bootloader's CRC32
// command; on any mismatch (or a blank device) it reflashes and verifies. Once the device is
// confirmed up to date it resets it into the app, finds the console by sweeping the whitelisted
// baud rates (the app boots at its saved console baud; factory default 1 M), and becomes a
// transparent USB-CDC serial adapter (see bridge.cc for the modem-control-line emulation
// contract).
//
// Hold BOOTSEL at power-up to force a reflash; tap BOOTSEL during pass-through to rerun the
// check (also the recovery for in-band AT+REBOOT / AT+BAUD_RATE desyncs). Hold BOOTSEL for 3 s --
// at any point, including while the jig is stuck reporting a dead console -- to arm a settings
// erase, which runs at the next bootloader entry and factory-resets the device. Wiring in board.hh.

#include <stdio.h>
#include <string.h>

#include "at_client.hh"
#include "board.hh"
#include "bootsel.hh"
#include "bridge.hh"
#include "cc13x4_bootloader.hh"
#include "firmware_image.hh"
#include "flasher.hh"
#include "pico/stdlib.h"
#include "status.hh"
#include "target_ctl.hh"
#include "target_uart.hh"
#include "tusb.h"

enum class State { kCheck, kFlash, kNegotiate, kPassthrough };

// Last bootloader-entry diagnosis, re-printed periodically in the wait loop so a CDC terminal
// attached after the fact still sees it (CdcPrintf output is dropped with no host connected).
static char last_diagnosis[192] = "";

// Set by a BOOTSEL long press; consumed by State::kCheck once the ROM bootloader is up. Going
// through the bootloader is what makes this work when the app console is dead, which is the case it
// exists for: a persisted settings blob that stops the console coming up cannot be cleared with
// AT+SETTINGS=RESET or AT+BOOT_UART_BOOTLOADER, since both need a console that already answers.
static bool erase_settings_armed = false;

// Arms the settings erase and tells the user. Idempotent, so repeated long presses are harmless.
static void ArmSettingsErase() {
    if (erase_settings_armed) return;
    erase_settings_armed = true;
    StatusSet(Status::kSettingsEraseArmed);
    CdcPrintf("BOOTSEL held: settings erase ARMED. The settings sectors (0x000FC000..0x000FDFFF) will "
              "be erased at the next bootloader entry and the device will boot with factory defaults. "
              "Device info / OTA keys are not touched.\r\n");
}

// sleep_ms that keeps USB enumeration and the LED alive. Also polls the BOOTSEL gesture, so the
// settings erase can be armed from the bootloader-entry retry loop and the console-negotiation
// failure wait -- the two places the jig sits when a device is bricked.
static void IdleMs(uint32_t ms) {
    absolute_time_t deadline = delayed_by_ms(get_absolute_time(), ms);
    absolute_time_t next_bootsel_poll = get_absolute_time();
    while (absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
        tud_task();
        StatusUpdate();
        // Modest cadence: GetBootselButton() stalls flash access with interrupts off.
        if (absolute_time_diff_us(get_absolute_time(), next_bootsel_poll) <= 0) {
            next_bootsel_poll = delayed_by_ms(get_absolute_time(), kBootselPollMs);
            if (PollBootsel() == BootselEvent::kLongPress) ArmSettingsErase();
        }
        sleep_ms(1);
    }
}

// Enters the ROM bootloader and completes auto-baud sync at 1 M. Reports each attempt's
// outcome over CDC.
static bool EnterBootloader(Cc13x4Bootloader& bl) {
    TargetUartSetBaud(kBootloaderBaud);
    for (int attempt = 0; attempt < kBootloaderEntryAttempts; attempt++) {
        TargetResetIntoBootloader();
        absolute_time_t deadline = delayed_by_ms(get_absolute_time(), 1500);
        while (absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
            if (bl.SyncBaud(300)) {
                CdcPrintf("SBL sync at %lu baud.\r\n", (unsigned long)kBootloaderBaud);
                return true;
            }
        }
        // Report what (if anything) came back: garbage bytes point at baud/framing, silence
        // at reset/UART/SYNC. SyncBaud flushes input per try, so also catch late stragglers.
        sleep_ms(100);
        uint8_t junk[8];
        size_t junk_len = TargetUartRead(junk, sizeof(junk));
        char junk_hex[3 * sizeof(junk) + 1] = "";
        for (size_t i = 0; i < junk_len; i++) {
            snprintf(junk_hex + 3 * i, 4, " %02X", junk[i]);
        }
        CdcPrintf("SBL entry attempt %d/%d @ %lu baud failed (%s)%s%s\r\n", attempt + 1,
                  kBootloaderEntryAttempts, (unsigned long)kBootloaderBaud, bl.LastError(),
                  junk_len ? "; late bytes:" : "", junk_hex);
    }
    return false;
}

// After a full entry failure, work out whether the module is alive at all and record a
// human-readable diagnosis (report-only; never auto-erases the app).
static void DiagnoseEntryFailure() {
    TargetResetIntoApp();
    IdleMs(kBootWaitMs);
    // Sweep the whitelist: a device with a non-default saved baud must not misdiagnose as dead.
    uint32_t app_baud = AtFindConsoleBaud();
    if (app_baud != 0) {
        snprintf(last_diagnosis, sizeof(last_diagnosis),
                 "App console responds at %lu baud - reset+UART wiring OK, but SBL entry "
                 "fails. Check SYNC (GP%u -> module pin 28) and the firmware's CCFG backdoor.",
                 (unsigned long)app_baud, kPinSync);
    } else {
        // Raw pad levels (readable regardless of pin function) localize the dead leg:
        // RESET_N is released here, so low means a wiring/drive conflict; UART RX should
        // idle high whenever the module is powered and its TX is connected.
        bool reset_high = gpio_get(kPinResetN);
        bool rx_idle_high = gpio_get(kPinUartRx);
        snprintf(last_diagnosis, sizeof(last_diagnosis),
                 "No response from module. RESET_N(GP%u)=%s%s, UART RX(GP%u)=%s%s",
                 kPinResetN, reset_high ? "high" : "LOW",
                 reset_high ? "" : " (stuck in reset - wiring?)", kPinUartRx,
                 rx_idle_high ? "high (link plausible)" : "LOW",
                 rx_idle_high ? "" : " (module TX not driving - power/wiring?)");
    }
    CdcPrintf("%s\r\n", last_diagnosis);
}

// Brings the console up after a reset into the app: wait out the boot, then find the device by
// sweeping the whitelisted rates (it boots at its saved console baud; factory default 1 M).
// `target_baud` is kConsoleBaud normally, or the host's rate when the bridge is reconciling to
// a host that expects another rate after a host-driven reset. Only the live rate is moved; the
// jig never issues AT+SETTINGS=SAVE, so the device's persisted baud is untouched.
static bool NegotiateConsole(uint32_t target_baud, bool print_version) {
    StatusSet(Status::kNegotiating);
    IdleMs(kBootWaitMs);

    // Generous window: first boot after a flash may rewrite the settings sectors (only when the
    // settings version changed -- the flash preserves them otherwise) and re-inits radios.
    // Three passes over the 5-rate sweep ~= the old fixed-rate 10 x 500 ms window.
    uint32_t found_baud = 0;
    for (int pass = 0; pass < 3 && !found_baud; pass++) found_baud = AtFindConsoleBaud();
    if (found_baud == 0) {
        CdcPrintf("Device console not responding at any whitelisted baud rate. If this persists with a "
                  "CRC-verified image, the saved settings may be the cause: hold BOOTSEL for 3 s to erase "
                  "them and boot with factory defaults.\r\n");
        return false;
    }

    if (print_version) {
        char version[32];
        if (AtQueryVersion(version, sizeof(version))) {
            CdcPrintf("Device firmware version: %s (baked: %s)\r\n", version,
                      kFirmwareVersionStr);
        }
    }

    if (target_baud != found_baud) {
        CdcPrintf("Console found at %lu baud; moving to %lu.\r\n", (unsigned long)found_baud,
                  (unsigned long)target_baud);
        if (!AtSetConsoleBaud(target_baud)) {
            CdcPrintf("AT+BAUD_RATE=CONSOLE,%lu not acknowledged.\r\n",
                      (unsigned long)target_baud);
            return false;
        }
        sleep_ms(50);  // Let the device finish switching.
        TargetUartSetBaud(target_baud);
        if (!AtProbeAlive(500) && !AtProbeAlive(500)) {
            CdcPrintf("Device unresponsive at %lu baud.\r\n", (unsigned long)target_baud);
            return false;
        }
    }

    CdcPrintf("Console up at %lu baud; entering pass-through.\r\n", (unsigned long)target_baud);
    BridgeSetExpectedConsoleBaud(target_baud);
    return true;
}

int main() {
    StatusInit();
    TargetCtlInit();
    TargetUartInit(kConsoleBaud);
    tusb_init();

    bool force_flash = GetBootselButton();
    if (force_flash) StatusSet(Status::kForcedFlashArmed);

    Cc13x4Bootloader bl;
    State state = State::kCheck;
    uint32_t negotiate_baud = kConsoleBaud;
    bool print_version = true;
    absolute_time_t next_diag_print = get_absolute_time();

    while (true) {
        switch (state) {
            case State::kCheck: {
                // The armed indication outranks the idle one: it is the only feedback that a
                // destructive action is pending, and this loop is where the user is watching.
                if (erase_settings_armed) {
                    StatusSet(Status::kSettingsEraseArmed);
                } else if (!force_flash) {
                    StatusSet(Status::kWaitingForDevice);
                }
                if (!EnterBootloader(bl)) {
                    DiagnoseEntryFailure();
                    // Retry forever (the module may be attached later), repeating the last
                    // diagnosis every few seconds for late-attached terminals.
                    next_diag_print = delayed_by_ms(get_absolute_time(), 5000);
                    IdleMs(1000);
                    break;
                }
                if (!bl.Ping()) {
                    CdcPrintf("SBL synced but ping failed (%s); retrying.\r\n", bl.LastError());
                    IdleMs(1000);
                    break;
                }
                // Before the CRC check: the erase must still happen on a device whose image is
                // already up to date, which is exactly the bricked case.
                if (erase_settings_armed) {
                    if (EraseSettingsRegion(bl) != FlashResult::kOk) {
                        // Stay armed: a fresh bootloader entry is exactly what the retry needs, and
                        // disarming here would silently drop the request the user made.
                        StatusSet(Status::kError);
                        CdcPrintf("Settings erase failed (%s); retrying.\r\n", bl.LastError());
                        IdleMs(2000);
                        break;
                    }
                    erase_settings_armed = false;
                }
                if (force_flash) {
                    CdcPrintf("BOOTSEL held at power-up: forcing reflash.\r\n");
                    state = State::kFlash;
                    break;
                }
                StatusSet(Status::kCrcCheck);
                if (BakedImageMatches(bl)) {
                    CdcPrintf("Firmware up to date (version %s).\r\n", kFirmwareVersionStr);
                    TargetResetIntoApp();
                    negotiate_baud = kConsoleBaud;
                    print_version = true;
                    state = State::kNegotiate;
                } else {
                    CdcPrintf("Firmware differs from baked %s image; flashing.\r\n",
                              kFirmwareVersionStr);
                    state = State::kFlash;
                }
                break;
            }

            case State::kFlash: {
                StatusSet(Status::kFlashing);
                FlashResult result = FlashBakedImage(bl);
                if (result == FlashResult::kOk) {
                    force_flash = false;
                    TargetResetIntoApp();
                    negotiate_baud = kConsoleBaud;
                    print_version = true;
                    state = State::kNegotiate;
                } else {
                    StatusSet(Status::kError);
                    CdcPrintf("Flash failed (%s); retrying.\r\n", bl.LastError());
                    IdleMs(2000);
                    state = State::kCheck;
                }
                break;
            }

            case State::kNegotiate: {
                if (NegotiateConsole(negotiate_baud, print_version)) {
                    state = State::kPassthrough;
                } else {
                    StatusSet(Status::kError);
                    IdleMs(2000);
                    state = State::kCheck;  // A fresh reset re-syncs everything.
                }
                print_version = false;
                break;
            }

            case State::kPassthrough: {
                BridgeExit exit_reason = BridgeRun();
                if (exit_reason == BridgeExit::kEraseSettings) {
                    ArmSettingsErase();
                    force_flash = false;
                    negotiate_baud = kConsoleBaud;
                    state = State::kCheck;
                } else if (exit_reason == BridgeExit::kRecheck) {
                    force_flash = false;
                    negotiate_baud = kConsoleBaud;
                    state = State::kCheck;
                } else {  // kHostResetTarget: match the console to what the host expects.
                    negotiate_baud = BridgeHostBaud();
                    state = State::kNegotiate;
                }
                break;
            }
        }

        // Periodic diagnosis re-print while stuck waiting for a device.
        if (state == State::kCheck && last_diagnosis[0] != '\0' &&
            absolute_time_diff_us(get_absolute_time(), next_diag_print) <= 0) {
            CdcPrintf("[waiting for device] %s\r\n", last_diagnosis);
            next_diag_print = delayed_by_ms(get_absolute_time(), 5000);
        }
    }
}
