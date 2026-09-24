#include "adsbee.hh"

#include <ti/devices/cc13x4_cc26x4/driverlib/ccfgread.h>
#include <ti/devices/cc13x4_cc26x4/driverlib/osc.h>
#include <ti/devices/cc13x4_cc26x4/driverlib/sys_ctrl.h>
#include <ti/devices/cc13x4_cc26x4/driverlib/watchdog.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

#include "buffer_utils.hh"
#include "comms.hh"
#include "flash_utils.hh"
#include "led.hh"
#include "packet_decoder.hh"
#include "sub_ghz_radio.hh"
#include "uat_packet_decoder.hh"

ADSBee::ADSBee(ADSBeeConfig config) : lr2021(config.lr2021_config), config_(config) {}

// Set from the SYNC rising-edge ISR; cleared in EnterSyncSleep() once SYNC has dropped. The level
// fallback in SyncSleepRequested() covers the cases where no edge ever fires (SYNC already high at
// boot) or the interrupt is momentarily disarmed.
static volatile bool sync_sleep_requested_ = false;

// SYNC line ISR, armed for a RISING edge while awake and a FALLING edge while asleep (CC26XX/CC13x4
// GPIO interrupts are edge-only). Reads the live pin level rather than trusting the armed edge, so
// stale latched EVFLAGS events and bounces are benign, and re-running the handoff is idempotent.
//
// SYNC high: performs the time-critical half of sync sleep -- the electrical handoff of the shared
// LR2021 bus to the external host -- within microseconds of the edge, even if an SPI transfer is in
// flight (the DMA completes internally against the un-muxed pins). Everything here is GPIO register
// work and volatile stores, all HWI-safe. The graceful half (draining output, suspending peripherals,
// STANDBY) runs from the main loop via SyncSleepRequested().
//
// SYNC low: nothing to do -- its only job is waking the MCU from STANDBY; execution resumes right
// after the PowerCC26XX_standbyPolicy() call in EnterSyncSleep().
static void SyncLineCallback(uint_least8_t /*index*/) {
    if (GPIO_read(bsp.kSyncPin) == 1) {
        sync_sleep_requested_ = true;
        adsbee.lr2021.RequestAbort();       // Unwind any in-progress command sequence at the next boundary.
        adsbee.lr2021.TristateInterface();  // Release NSS/ENABLE/SCLK/PICO to the host immediately.
    }
}

// LR2021 IRQ line (LR2021 DIO6 -> LR_IRQ pin) rising edge: the RX FIFO crossed its high threshold.
// Kicks the ISR-paced drain chain (lr2021_irq_drain.cpp). GPIO HWI context; all GPIO callbacks share
// one HWI vector, so this never races SyncLineCallback or LRBusyLineCallback.
static void LRIrqLineCallback(uint_least8_t /*index*/) { adsbee.lr2021.HandleIrqLine(); }

// LR_BUSY falling edge ("chip ready"): armed only while the drain chain has a frame pending; posts it.
static void LRBusyLineCallback(uint_least8_t /*index*/) { adsbee.lr2021.HandleBusyFall(); }

bool ADSBee::Init() {
    // Arm the SYNC rising-edge interrupt (the SysConfig default pin config) before touching the LR2021,
    // so a host asserting SYNC mid-init still gets the bus handed off promptly. Clear any stale latched
    // edge first: neither GPIO_setConfig nor GPIO_enableInt clears EVFLAGS.
    GPIO_setCallback(bsp.kSyncPin, SyncLineCallback);
    GPIO_clearInt(bsp.kSyncPin);
    GPIO_enableInt(bsp.kSyncPin);
    // Seed the LR2021 interface enable from the persisted setting (loaded before adsbee.Init() in
    // main()) so a device saved with AT+LR_ENABLE=0 parks the bus on the first receiver config instead
    // of briefly initializing the chip and driving the bus before SettingsManager::Apply() runs.
    lr2021_enabled_ = settings_manager.settings.lr2021_enabled;
    return ApplyReceiverConfig();
}

bool ADSBee::SyncSleepRequested() { return sync_sleep_requested_ || GPIO_read(bsp.kSyncPin) == 1; }

bool ADSBee::SetRx1090Enabled(bool enabled) {
    if (enabled == rx_1090_enabled_) {
        return true;  // No change (also keeps SettingsManager::Apply() from re-initing a live radio).
    }
    rx_1090_enabled_ = enabled;
    bool success = ApplyReceiverConfig();  // Honors the flag: powers the LR2021 down or brings it up.
    if (!enabled) {
        // Discard any packets the IRQ-paced drain chain staged before the teardown, so stale traffic
        // doesn't surface when reception is re-enabled later.
        while (lr2021.NextFilledSlot() != nullptr) {
            lr2021.ReleaseSlot();
        }
    }
    return success;
}

bool ADSBee::SetLR2021Enabled(bool enabled) {
    if (enabled == lr2021_enabled_) {
        return true;  // No change (also keeps SettingsManager::Apply() from re-initing a live radio).
    }
    lr2021_enabled_ = enabled;
    bool success = ApplyReceiverConfig();  // Honors the flag: parks the bus or re-drives + re-inits.
    if (!enabled) {
        // Discard any packets the IRQ-paced drain chain staged before the teardown, so stale traffic
        // doesn't surface when the interface is re-enabled later.
        while (lr2021.NextFilledSlot() != nullptr) {
            lr2021.ReleaseSlot();
        }
    }
    return success;
}

bool ADSBee::ApplyReceiverConfig() {
    bool success = ApplyReceiverConfigInner();
    receiver_config_ok_ = success;
    if (!success) {
        lr2021_config_fail_count++;
    }
    // Restart the RX-health clocks on BOTH outcomes so retries are paced by the recovery backoff (a
    // failed bring-up self-heals via the health ladder in UpdateLR2021 instead of dying silently).
    uint32_t now_ms = get_time_since_boot_ms();
    lr2021_last_rx_ok_ms_ = now_ms;
    lr2021_last_recovery_ms_ = now_ms;
    lr2021_rearm_attempts_ = 0;
    return success;
}

bool ADSBee::ApplyReceiverConfigInner() {
    // Quiesce the LR2021 interrupt lines while the chip is reset/reconfigured: the IRQ line's meaning
    // is undefined until SetOokADSB re-routes it, and the BUSY interrupt must only ever be armed by an
    // active drain chain. (DeInit -> CancelAsync also disarms BUSY; this covers every path.)
    GPIO_disableInt(bsp.kLR2021IrqPin);
    GPIO_disableInt(bsp.kLR2021BusyPin);
    // LR2021 interface disabled (AT+LR_ENABLE=0): release the shared bus to an external host. DeInit
    // first (cancels any in-flight async DMA before SPI_close, drives ENABLE low for a crisp reset
    // edge), then TristateInterface parks NSS (input, pull-up) and ENABLE/SCLK/PICO (inputs,
    // pull-down). Both are idempotent, so every caller (boot, AT commands, sync-sleep wake, R1090_*
    // setters) can funnel through here.
    if (!lr2021_enabled_) {
        lr2021.DeInit();
        lr2021.TristateInterface();
        return true;
    }
    // Re-drive NSS/ENABLE in case a prior AT+LR_ENABLE=0 or sync sleep parked them (GPIO_resetConfig
    // restores the SysConfig OUT_HIGH defaults; the SPI pins re-mux on SPI_open in Init()). Harmless
    // when already driven: the DeInit/Init below performs a full reset cycle regardless.
    lr2021.RestoreInterface();
    // 1090 RX disabled (AT+RX_ENABLE): hold the LR2021 in reset instead of configuring it. DeInit
    // cancels any in-flight async transfer, closes SPI, and drives ENABLE low, so no reception and no
    // interrupts. Every RX-arming path funnels through here (boot, gain/preamble/boost changes,
    // sync-sleep wake), so none of them can re-enable a user-disabled receiver.
    if (!rx_1090_enabled_) {
        lr2021.DeInit();
        return true;
    }
    // Reconfigure from a clean hardware reset every time. The LR2021's RF/AGC/detector calibration
    // must be set up from standby (kStdbyRC); reconfiguring while the radio is in continuous RX
    // leaves it demodulating but never validating. DeInit()+Init() reproduces the exact known-good
    // boot sequence (reset -> kStdbyRC) before configuring, for both boot and live AT changes.
    lr2021.DeInit();  // Safe even if never inited (SPI_close is guarded).
    if (!lr2021.Init()) {
        return false;
    }
    if (!lr2021.SetOokADSB(r1090_preamble_mode_, r1090_gain_, r1090_rx_boost_)) {
        return false;
    }
    // Arm the LR2021 IRQ rising-edge interrupt now that the chip side is routing kIrqRxFifo to it.
    // Clear any stale latched edge first: neither GPIO_setConfig nor GPIO_enableInt clears EVFLAGS.
    // The BUSY callback is registered here too, but its interrupt stays disarmed until a chain frame
    // is pending.
    GPIO_setCallback(bsp.kLR2021IrqPin, LRIrqLineCallback);
    GPIO_setCallback(bsp.kLR2021BusyPin, LRBusyLineCallback);
    GPIO_clearInt(bsp.kLR2021IrqPin);
    GPIO_enableInt(bsp.kLR2021IrqPin);
    return true;
}

bool ADSBee::SetRxSubGHzEnabled(bool enabled) { return subg_radio.SetRxEnabled(enabled); }

bool ADSBee::RxSubGHzIsEnabled() const { return subg_radio.RxIsEnabled(); }

void ADSBee::SetR1090PreambleMode(SettingsManager::R1090PreambleMode mode) {
    if (mode >= SettingsManager::kNumR1090PreambleModes) {
        // Guard against stale persisted settings holding a removed enum value.
        mode = SettingsManager::kR1090PreambleModeModeS;
    }
    r1090_preamble_mode_ = mode;
    ApplyReceiverConfig();
}

void ADSBee::SetR1090Gain(uint8_t gain_step) {
    r1090_gain_ = gain_step;
    ApplyReceiverConfig();
}

void ADSBee::SetR1090RxBoost(uint8_t rx_boost) {
    r1090_rx_boost_ = rx_boost;
    ApplyReceiverConfig();
}

void ADSBee::EnterSyncSleep() {
    // Disarm the rising-edge interrupt while we reconfigure the pin for wake. The time-critical bus
    // handoff has already run in SyncLineCallback (or is redone idempotently below, for the level-poll
    // entry path where no edge ever fired, e.g. SYNC already high at boot).
    GPIO_disableInt(bsp.kSyncPin);

    CONSOLE_INFO("ADSBee::EnterSyncSleep", "SYNC asserted; powering down LR2021 and entering STANDBY.");

    // Disarm the LR2021 interrupt lines for the handoff: an enabled DIO interrupt is automatically a
    // STANDBY wake source on CC13x4, so host bus traffic wiggling LR_IRQ/BUSY during sync sleep would
    // wake the MCU continuously. Re-armed by ApplyReceiverConfig() on wake. Any filled drain slots
    // survive and are parsed after wake.
    GPIO_disableInt(bsp.kLR2021IrqPin);
    GPIO_disableInt(bsp.kLR2021BusyPin);

    // Finish powering the external radio down. The ISR already tri-stated the bus; DeInit()'s enable-low
    // write is then a harmless DOUT update on an input pin, and SPI_close leaves SCLK/PICO hi-Z (their
    // SysConfig not-in-use configs are inputs). ApplyReceiverConfig() below re-runs the exact boot
    // sequence on wake.
    lr2021.DeInit();

    // Re-assert the tri-state (idempotent): covers the poll-path entry where SyncLineCallback never ran.
    // On wake, ApplyReceiverConfigInner() re-drives NSS/ENABLE via RestoreInterface() — but only when
    // the LR2021 interface is enabled, so a device with AT+LR_ENABLE=0 keeps the bus parked.
    lr2021.TristateInterface();

    // Re-arm SYNC (DIO_5) as the STANDBY wake source on a FALLING edge (SYNC going low). CC26XX/CC13x4
    // hardware does not support level-triggered GPIO interrupts, so an edge is the only option. The
    // lost-edge race (host drops SYNC between the GPIO_read below and entering STANDBY) is closed by the
    // re-check loop: with the interrupt enabled, the edge latches as a pending NVIC interrupt, the
    // policy's WFI returns immediately, and the loop re-reads LOW and exits. Keep the fail-safe pull-down,
    // and clear any stale latched edge before enabling (EVFLAGS is not cleared by setConfig/enableInt).
    GPIO_setConfig(bsp.kSyncPin, GPIO_CFG_INPUT_INTERNAL | GPIO_CFG_IN_INT_FALLING | GPIO_CFG_PULL_DOWN_INTERNAL);
    GPIO_setCallback(bsp.kSyncPin, SyncLineCallback);
    GPIO_clearInt(bsp.kSyncPin);
    GPIO_enableInt(bsp.kSyncPin);

    // Let the console line go fully idle before sampling constraints below. UART2 holds
    // PowerCC26XX_DISALLOW_STANDBY from the start of a TX until its EOT interrupt, which lands after the
    // last stop bit -- and console output is queued asynchronously in a TX ring. Without this the
    // CONSOLE_INFO above guarantees a spurious "STANDBY blocked" report on every sleep cycle.
    comms_manager.DrainConsoleTx();

    // On CC13x4 an enabled DIO interrupt is automatically a STANDBY wake source (the GPIO driver routes
    // PAD events to MCU WU1 at init), so no explicit AON/IOC wakeup config is needed.
    //
    // PowerCC26XX_standbyPolicy() is what performs the constraint-checked WFI/IDLE/STANDBY transition,
    // but it short-circuits and returns immediately unless the GLOBAL power policy is enabled
    // (PowerCC26X2_nortos.c: `if (!PowerCC26X2_module.enablePolicy) return;` is its first statement),
    // and main.cpp deliberately disables the policy at boot for debugger stability. Without the enable
    // below the loop is a pure busy-spin that never sleeps at all. Scope the enable to the sleep loop
    // and restore the caller's setting on wake, so the rest of the firmware keeps running with the
    // policy off. Power_disablePolicy() returns the previous state, so it doubles as the read.
    bool policy_was_enabled = Power_disablePolicy();
    Power_enablePolicy();

    // Re-check SYNC in a loop because the policy can return from a plain WFI/IDLE (e.g. a pending ClockP
    // tick) without having reached STANDBY, or while a power constraint is still momentarily held (e.g. a
    // UART/SPI transfer draining). The caller must have quiesced the persistent constraint holders (the
    // SubGHz RF core and the console UART RX) first, or the policy will never descend to STANDBY.
    //
    // Constraint diagnostics: a briefly-held constraint (the CONSOLE_INFO above draining, an SPI transfer
    // finishing) clears on its own and is not worth reporting, so only start complaining once STANDBY has
    // been blocked continuously for kConstraintGraceMs, then repeat at kConstraintRepeatMs so a genuinely
    // stuck holder keeps reporting without flooding the console.
    static constexpr uint32_t kConstraintGraceMs = 250;
    static constexpr uint32_t kConstraintRepeatMs = 1000;
    uint32_t unblocked_timestamp_ms = get_time_since_boot_ms();
    uint32_t next_warning_timestamp_ms = unblocked_timestamp_ms + kConstraintGraceMs;
    while (GPIO_read(bsp.kSyncPin) == 1) {
        uint32_t constraint_mask = Power_getConstraintMask();
        uint32_t timestamp_ms = get_time_since_boot_ms();
        if (!(constraint_mask & (1u << PowerCC26XX_DISALLOW_STANDBY))) {
            unblocked_timestamp_ms = timestamp_ms;
            next_warning_timestamp_ms = timestamp_ms + kConstraintGraceMs;
        } else if (static_cast<int32_t>(timestamp_ms - next_warning_timestamp_ms) >= 0) {
            // Report the LF clock state alongside the mask. Power_init() takes DISALLOW_STANDBY pending
            // the SCLK_LF switch and only releases it once the LF clock actually reaches RCOSC_LF or
            // XOSC_LF, so a CCFG that asks for LF XOSC (option 2) on a board whose 32.768 kHz crystal
            // is absent or not oscillating pins this constraint from boot forever. lf_src is the live
            // OSC_{RCOSC,XOSC}_{HF,LF} value; ccfg_lf is the configured CCFG SCLK_LF_OPTION.
            CONSOLE_WARNING("ADSBee::EnterSyncSleep",
                            "STANDBY blocked for %lums; constraint mask = 0x%lx, lf_src = %lu, ccfg_lf = %lu.",
                            (unsigned long)(timestamp_ms - unblocked_timestamp_ms), (unsigned long)constraint_mask,
                            (unsigned long)OSCClockSourceGet(OSC_SRC_CLK_LF),
                            (unsigned long)CCFGRead_SCLK_LF_OPTION());
            next_warning_timestamp_ms = timestamp_ms + kConstraintRepeatMs;
            // The warning itself is a TX, which re-takes the very constraint we are reporting on. Drain
            // it so the next pass measures the real holder instead of confirming its own echo.
            comms_manager.DrainConsoleTx();
        }
        PowerCC26XX_standbyPolicy();
        // The CC13x4 watchdog keeps counting (on SCLK_LF) through STANDBY, so feed it on every policy
        // return or a long enough sleep watchdog-resets the MCU mid-sleep.
        FeedWatchdog();
    }

    if (!policy_was_enabled) {
        Power_disablePolicy();
    }

    // SYNC is now low: swap the wake interrupt back to the rising-edge sleep-request config (keeping the
    // fail-safe pull-down), clear the request flag, then re-initialize the receiver to resume where we
    // left off. If SYNC bounces high during the re-init, the re-armed ISR re-tristates and sets the flag
    // again, the config aborts, and the main loop re-enters sync sleep.
    GPIO_disableInt(bsp.kSyncPin);
    sync_sleep_requested_ = false;
    GPIO_setConfig(bsp.kSyncPin, GPIO_CFG_INPUT_INTERNAL | GPIO_CFG_IN_INT_RISING | GPIO_CFG_PULL_DOWN_INTERNAL);
    GPIO_clearInt(bsp.kSyncPin);
    GPIO_enableInt(bsp.kSyncPin);
    FeedWatchdog();

    CONSOLE_INFO("ADSBee::EnterSyncSleep", "SYNC released; re-initializing LR2021.");
    // ApplyReceiverConfigInner() re-drives NSS/ENABLE via RestoreInterface() before Init()'s GPIO
    // writes — unless AT+LR_ENABLE=0, in which case it re-parks the bus without ever driving it
    // (calling RestoreInterface() here unconditionally would glitch RESET/NSS high on wake).
    ApplyReceiverConfig();
}

void ADSBee::FeedWatchdog() {
    if (WatchdogRunning()) {
        WatchdogIntClear();
    }
}

bool ADSBee::Update() {
    FeedWatchdog();
    UpdateLR2021();
    PruneAircraftDictionary();
    UpdateRxPosition();
    IngestAndForwardPackets();
    return true;
}

void ADSBee::UpdateRxPosition() {
    // Rate limiting.
    uint32_t timestamp_ms = get_time_since_boot_ms();
    if (timestamp_ms - last_rx_position_update_timestamp_ms_ < config_.rx_position_update_interval_ms) {
        return;
    }
    last_rx_position_update_timestamp_ms_ = timestamp_ms;

    // Update rx position based on selected position source. Note this only handles periodic automatic updates.
    // One-time changes are handled in AT comms.
    switch (rx_position.source) {
        case SettingsManager::RxPosition::PositionSource::kPositionSourceNone:
            // No position available.
            rx_position_available = false;
            break;
        case SettingsManager::RxPosition::PositionSource::kPositionSourceLowestAircraft: {
            // Use position of lowest aircraft in dictionary.
            // Make variables to write into since we can't bind directly to the packed struct members.
            float latitude_deg, longitude_deg, heading_deg;
            int32_t gnss_altitude_ft, baro_altitude_ft, speed_kts;
            if (aircraft_dictionary.GetLowestAircraftPosition(latitude_deg, longitude_deg, gnss_altitude_ft,
                                                              baro_altitude_ft, heading_deg, speed_kts)) {
                rx_position.latitude_deg = latitude_deg;
                rx_position.longitude_deg = longitude_deg;
                rx_position.gnss_altitude_ft = gnss_altitude_ft;
                rx_position.baro_altitude_ft = baro_altitude_ft;
                rx_position.heading_deg = heading_deg;
                rx_position.speed_kts = speed_kts;
                rx_position_available = true;
            } else {
                // No valid aircraft position available. Don't update receiver position.
                rx_position_available = false;
            }
            break;
        }
        case SettingsManager::RxPosition::PositionSource::kPositionSourceGNSS:
            // No GNSS receiver on this board; position must come from another source.
            rx_position_available = false;
            break;
        case SettingsManager::RxPosition::PositionSource::kPositionSourceFixed:
            // Fixed position is always available.
            rx_position_available = true;
            break;
        case SettingsManager::RxPosition::PositionSource::kPositionSourceAircraftMatchingICAO: {
            // Try to find the aircraft with the matching ICAO address in the dictionary.
            // Check both Mode S and UAT, and use the most recently seen aircraft if both exist.
            uint32_t mode_s_uid = Aircraft::ICAOToUID(rx_position.icao_address, Aircraft::kAircraftTypeModeS);
            ModeSAircraft* mode_s_aircraft = aircraft_dictionary.GetAircraftPtr<ModeSAircraft>(mode_s_uid, false);
            bool mode_s_valid = mode_s_aircraft && mode_s_aircraft->HasBitFlag(ModeSAircraft::kBitFlagPositionValid);

            uint32_t uat_uid = Aircraft::ICAOToUID(rx_position.icao_address, Aircraft::kAircraftTypeUAT);
            UATAircraft* uat_aircraft = aircraft_dictionary.GetAircraftPtr<UATAircraft>(uat_uid, false);
            bool uat_valid = uat_aircraft && uat_aircraft->HasBitFlag(UATAircraft::kBitFlagPositionValid);

            // Determine which aircraft to use based on validity and recency.
            Aircraft* selected_aircraft = nullptr;
            if (mode_s_valid && uat_valid) {
                // Both exist and have valid positions; use the most recently seen.
                if (mode_s_aircraft->last_message_timestamp_ms >= uat_aircraft->last_message_timestamp_ms) {
                    selected_aircraft = mode_s_aircraft;
                } else {
                    selected_aircraft = uat_aircraft;
                }
            } else if (mode_s_valid) {
                selected_aircraft = mode_s_aircraft;
            } else if (uat_valid) {
                selected_aircraft = uat_aircraft;
            }

            if (selected_aircraft) {
                rx_position.latitude_deg = selected_aircraft->latitude_deg;
                rx_position.longitude_deg = selected_aircraft->longitude_deg;
                rx_position.gnss_altitude_ft = selected_aircraft->gnss_altitude_ft;
                rx_position.baro_altitude_ft = selected_aircraft->baro_altitude_ft;
                rx_position.heading_deg = selected_aircraft->direction_deg;
                rx_position.speed_kts = selected_aircraft->speed_kts;
                rx_position_available = true;
            } else {
                // No aircraft found with the matching ICAO address or no valid position.
                rx_position_available = false;
            }
            break;
        }
        default:
            CONSOLE_ERROR("ADSBee::UpdateRxPosition", "Unknown Rx position source %d.",
                          static_cast<int>(rx_position.source));
    }

    // Update the aircraft dictionary's reference location. Clearing it when unavailable pauses Mode S
    // surface position decoding, which would otherwise resolve CPR against a stale or bogus reference.
    if (rx_position_available) {
        aircraft_dictionary.SetReferencePosition(rx_position.latitude_deg, rx_position.longitude_deg);
    } else {
        aircraft_dictionary.ClearReferencePosition();
    }
}

bool ADSBee::SetWatchdogTimeoutSec(uint32_t timeout_sec) {
    // timeout_sec == 0 means disabled; don't start the watchdog. Note: if the watchdog is already
    // running it cannot be stopped in hardware, so Update() will continue feeding it via WatchdogRunning().
    if (timeout_sec == 0) {
        watchdog_timeout_sec_ = 0;
        return true;
    }

    // Guard against overflow: max load value is UINT32_MAX ticks at GET_MCU_CLOCK Hz. Clamp rather than
    // bail out -- returning early here used to leave the watchdog switched off entirely, so a corrupt
    // persisted timeout would silently remove the one mechanism that turns a boot-path hang into a
    // visible reset loop instead of a brick.
    if (timeout_sec > UINT32_MAX / GET_MCU_CLOCK) {
        CONSOLE_WARNING("ADSBee::SetWatchdogTimeoutSec", "Timeout %lu s out of range; clamping to %lu s.",
                        (unsigned long)timeout_sec, (unsigned long)kDefaultWatchdogTimeoutSec);
        timeout_sec = kDefaultWatchdogTimeoutSec;
    }

    watchdog_timeout_sec_ = timeout_sec;

    WatchdogUnlock();
    WatchdogReloadSet(timeout_sec * GET_MCU_CLOCK);
    if (!WatchdogRunning()) {
        WatchdogResetEnable();
        WatchdogEnable();
    }
    WatchdogLock();

    return true;
}

void ADSBee::Reboot() {
    CONSOLE_INFO("ADSBee::Reboot", "Rebooting.");
    // Console output is queued asynchronously; get it (and any pending AT response) onto the wire before the reset.
    comms_manager.DrainConsoleTx();
    lr2021.DeInit();
    SysCtrlSystemReset();
}

void ADSBee::EnterUARTBootloader() {
    CONSOLE_INFO("ADSBee::EnterUARTBootloader", "Invalidating image and entering ROM UART bootloader.");
    // Console output is queued asynchronously; get it (and any pending AT response) onto the wire before the reset.
    comms_manager.DrainConsoleTx();
    lr2021.DeInit();
    // Mask interrupts and keep them masked through the reset: we are about to erase the vector
    // table sector, so no interrupt may fire afterwards. FlashSectorErase() and
    // SysCtrlSystemReset() both execute from ROM, so erasing flash sector 0 from here is safe.
    // Do not add any work between the erase and the reset.
    FlashUtils::FlashSafe();
    // Result deliberately ignored: nothing can be reported from here (interrupts are masked and the
    // console is already drained), and a failed erase simply means the app boots normally instead.
    (void)FlashUtils::EraseSector(0x0);  // erase the vector-table sector -> invalid reset vector
    // With CCFG IMAGE_VALID_CONF == 0, the ROM validates the reset vector at flash address 0x0
    // on boot. With it erased (0xFFFFFFFF) the ROM treats the image as invalid and enters the
    // serial bootloader on DIO2 (RX) / DIO3 (TX) -- the same pins as SUBG_UART.
    SysCtrlSystemReset();
    // Does not return.
}

/**
 * Private Functions
 */

void ADSBee::IngestAndForwardPackets() {
    // Ingest new Mode S packets into dictionary and report them.
    DecodedModeSPacket decoded_packet;
    // Explicitly count the number of packets to process so that we don't get stuck in this loop if the decoder
    // keeps outputting packets.
    uint16_t num_decoded_mode_s_packets = packet_decoder.decoded_mode_s_packet_out_queue.Length();
    for (uint16_t i = 0;
         i < num_decoded_mode_s_packets && packet_decoder.decoded_mode_s_packet_out_queue.Dequeue(decoded_packet);
         i++) {
        // CONSOLE_INFO("ADSBee::IngestAndForwardPackets", "\tdf=%d icao_address=0x%06x",
        // decoded_packet.downlink_format,
        //              decoded_packet.icao_address);

        if (aircraft_dictionary.IngestDecodedModeSPacket(decoded_packet)) {
            // Packet was used to update the dictionary or was silently ignored (but presumed to be valid).
            leds.FlashLED(bsp.k1090LEDPin, 10);
            // CONSOLE_INFO("ADSBee::IngestAndForwardPackets", "\taircraft_dictionary: %d aircraft",
            //              aircraft_dictionary.GetNumAircraft());
        }
        // Check for any new valid packets and push all decoded packets to the reporting queue, even if the aircraft
        // dictionary didn't know what to do with them.
        if (decoded_packet.is_valid) {
            lr2021_frames_since_valid_ = 0;  // Healthy stream: reset the validity watchdog.
            // Counted, not logged: at high packet rates a per-packet CONSOLE_ERROR here floods the UART
            // TX ring and stalls the main loop -- the very thing that makes the queue overflow.
            if (comms_manager.mode_s_packet_reporting_queue.IsFull()) {
                comms_manager.mode_s_report_queue_ovf_count++;
            }
            comms_manager.mode_s_packet_reporting_queue.Enqueue(decoded_packet.raw);  // Overwrites oldest when full.
        }
    }

    // Ingest decoded UAT ADS-B packets.
    DecodedUATADSBPacket decoded_uat_adsb_packet;
    uint16_t num_decoded_uat_adsb = uat_packet_decoder.decoded_uat_adsb_packet_out_queue.Length();
    for (uint16_t i = 0; i < num_decoded_uat_adsb &&
                         uat_packet_decoder.decoded_uat_adsb_packet_out_queue.Dequeue(decoded_uat_adsb_packet);
         i++) {
        aircraft_dictionary.IngestDecodedUATADSBPacket(decoded_uat_adsb_packet);
        aircraft_dictionary.RecordSubGHzMetrics(0, 1, 0, 0);
        if (!comms_manager.uat_adsb_packet_reporting_queue.Enqueue(decoded_uat_adsb_packet.raw)) {
            CONSOLE_ERROR("ADSBee::IngestAndForwardPackets", "UAT ADS-B packet reporting queue overflowed.");
        }
    }

    // Ingest decoded UAT uplink packets. The raw packet has already been FEC-corrected in place by the
    // decoder, so downstream reporters (e.g. GDL90) can re-decode it without another correction pass.
    DecodedUATUplinkPacket decoded_uat_uplink_packet;
    uint16_t num_decoded_uat_uplink = uat_packet_decoder.decoded_uat_uplink_packet_out_queue.Length();
    for (uint16_t i = 0; i < num_decoded_uat_uplink &&
                         uat_packet_decoder.decoded_uat_uplink_packet_out_queue.Dequeue(decoded_uat_uplink_packet);
         i++) {
        aircraft_dictionary.RecordSubGHzMetrics(0, 0, 0, 1);
        if (!comms_manager.uat_uplink_packet_reporting_queue.Enqueue(decoded_uat_uplink_packet.raw)) {
            CONSOLE_ERROR("ADSBee::IngestAndForwardPackets", "UAT uplink packet reporting queue overflowed.");
        }
    }
}

void ADSBee::PruneAircraftDictionary() {
    // Prune aircraft dictionary. Need to do this up front so that we don't end up with a negative timestamp delta
    // caused by packets being ingested more recently than the timestamp we take at the beginning of this function.
    uint32_t timestamp_ms = get_time_since_boot_ms();
    if (timestamp_ms - last_aircraft_dictionary_update_timestamp_ms_ > config_.aircraft_dictionary_update_interval_ms) {
        aircraft_dictionary.Update(timestamp_ms);

        last_aircraft_dictionary_update_timestamp_ms_ = timestamp_ms;
    }
}

bool ADSBee::UpdateLR2021() {
    // Once the SYNC ISR has handed the bus to the host, issue no further transactions and stay quiet:
    // an in-progress drain unwinds via the abort flag inside ServiceRxDrain(), which is expected, not
    // an error. The main loop enters sync sleep at the top of its next iteration.
    if (SyncSleepRequested()) {
        return true;
    }
    if (!lr2021_enabled_) {
        return true;  // Bus released to an external host (AT+LR_ENABLE=0): issue no transactions.
    }
    if (!rx_1090_enabled_) {
        return true;  // 1090 RX user-disabled: the LR2021 is held in reset; nothing to drain.
    }
    uint32_t start_us = get_time_since_boot_us();

    // Consume staging slots filled by the IRQ-paced drain chain (lr2021_irq_drain.cpp) first: they
    // predate anything the loop drain is currently reading, and their timestamps were captured at the
    // IRQ edge.
    LR2021::IrqDrainSlot* slot;
    while ((slot = lr2021.NextFilledSlot()) != nullptr) {
        ParseLR2021RxFifo(slot->buf + 2, slot->len, slot->timestamp_us);
        lr2021.ReleaseSlot();
    }

    // FIFO tear hazard observed (level read back full, or a Full/Overflow/Underflow sub-flag seen by
    // either drain tail): the FIFO byte stream may be torn mid-packet, and
    // because every read pops whole 14-byte packets, the misalignment would persist forever (every
    // slice failing CRC -- reception looks dead while the chip-side counters keep climbing). Flush the
    // FIFO to realign, then clear the (possibly re-latched) FIFO sub-flags and the global IRQ latch so
    // edge generation recovers. LR_IRQ is disarmed around the sequence so a fresh edge can't start a
    // chain between the commands; each command's BeginTransaction pumps any in-flight drain/chain to
    // quiescence first. Already-staged data is still parsed: slices from before the tear are good, the
    // rest fail CRC harmlessly.
    if (lr2021.fifo_overflow_pending) {
        GPIO_disableInt(bsp.kLR2021IrqPin);
        LR2021::GetAndClearIrqRsp irq_rsp;
        // Standby first: it halts the packet engine, so a packet mid-demodulation can't keep
        // streaming its tail into the freshly flushed FIFO (a flush landing mid-write would re-tear
        // the stream from byte 0). RX re-arms from standby exactly as SetOokADSB does; the config
        // registers persist through standby (only sleep loses them).
        bool resync_ok = lr2021.SetStandby(LR2021::kSysStandbyRc) && lr2021.ClearRxFifo() &&
                         lr2021.ClearFifoIrqFlags(0x3F, 0) && lr2021.GetAndClearIrq(&irq_rsp) &&
                         lr2021.SetRxAdv(0xFFFFFF);
        if (resync_ok) {
            lr2021.fifo_overflow_pending = false;
            lr2021_fifo_resync_count++;
        }  // On failure the flag stays set and the resync is retried next pass.
        GPIO_clearInt(bsp.kLR2021IrqPin);
        GPIO_enableInt(bsp.kLR2021IrqPin);
    }

    // Async RX drain: kick one off when idle, then advance it as far as the DMA/BUSY state allows.
    // Each call does at most one phase's worth of parsing plus posting the next DMA frame -- the long
    // SPI clocking (up to 258 B) happens in the background while the rest of the loop runs.
    if (lr2021.DrainIdle() && lr2021.StartRxDrain()) {
        lr2021_drain_start_us_ = start_us;
    }

    bool success = true;
    LR2021::DrainResult drain_result = lr2021.ServiceRxDrain();
    switch (drain_result) {
        case LR2021::DrainResult::kDataReady: {
            uint32_t drain_us = get_time_since_boot_us() - lr2021_drain_start_us_;
            if (drain_us > lr2021_drain_max_us) {
                lr2021_drain_max_us = drain_us;
            }
            if (lr2021.drain_fifo_was_full()) {
                // FIFO read back completely full: the poll loop fell behind and the radio almost
                // certainly dropped frames on top of this. Counted (not just logged) so benches can
                // see it via AT+RX_STATS even with logging off.
                lr2021_fifo_full_count++;
            }
            ParseLR2021RxFifo(lr2021.drain_payload(), lr2021.drain_level(), get_time_since_boot_us());
            lr2021.FinishRxDrain();
            break;
        }
        case LR2021::DrainResult::kError:
            // Rate-limited: a persistently failing drain would otherwise log every loop iteration,
            // and sustained console errors are themselves a main-loop stall vector (TX ring spin).
            if (!SyncSleepRequested() &&
                get_time_since_boot_ms() - last_drain_error_log_ms_ > 1000) {
                last_drain_error_log_ms_ = get_time_since_boot_ms();
                CONSOLE_ERROR("ADSBee::Update", "LR2021 RX drain failed: %s.", lr2021.drain_error_str());
            }
            lr2021.FinishRxDrain();
            success = false;
            break;
        default:
            // kIdle / kInProgress / kNoData: nothing to consume this iteration.
            break;
    }

    // RX health ladder. Continuous RX is armed only when the receiver config is applied; if the chip
    // ever leaves RX (or a config attempt failed and left the SPI closed), the drain just polls an
    // empty FIFO with no error. Every COMPLETED drain sweep refreshes last_stat() from a live Stat
    // readback, so gate the "healthy" stamp on completion -- a dead SPI never completes a sweep, so a
    // stale-but-kRx last_stat() can't mask a dead receiver.
    uint32_t now_ms = get_time_since_boot_ms();
    if ((drain_result == LR2021::DrainResult::kDataReady || drain_result == LR2021::DrainResult::kNoData) &&
        lr2021.last_stat().chip_mode == LR2021::ChipMode::kRx) {
        lr2021_last_rx_ok_ms_ = now_ms;
        lr2021_rearm_attempts_ = 0;
    } else if (now_ms - lr2021_last_rx_ok_ms_ > kRxHealthTimeoutMs &&
               now_ms - lr2021_last_recovery_ms_ > kRxRecoveryBackoffMs) {
        lr2021_last_recovery_ms_ = now_ms;
        if (receiver_config_ok_ && lr2021_rearm_attempts_ < kMaxRearmAttempts) {
            // Config intact but the chip left RX: re-arm continuous RX in place. LR_IRQ is disarmed
            // around the command -- an LR_IRQ HWI would otherwise post a chain frame onto the SPI
            // driver's queue mid-command (the TI driver queues a second SPI_transfer and clocks it
            // back-to-back under the same NSS frame, merging the two into one corrupted command).
            lr2021_rearm_attempts_++;
            lr2021_rx_rearm_count++;
            CONSOLE_WARNING("ADSBee::UpdateLR2021", "No confirmed RX for %lu ms; re-arming LR2021 RX.",
                            (unsigned long)(now_ms - lr2021_last_rx_ok_ms_));
            GPIO_disableInt(bsp.kLR2021IrqPin);
            lr2021.SetRxAdv(0xFFFFFF);  // Continuous RX (no timeout), as armed by SetOokADSB.
            GPIO_clearInt(bsp.kLR2021IrqPin);
            GPIO_enableInt(bsp.kLR2021IrqPin);
        } else {
            // Re-arm didn't stick, or the config never applied: full receiver bring-up (resets the
            // health clocks on both outcomes, so a hard failure retries forever on the backoff).
            lr2021_rx_reconfig_count++;
            CONSOLE_WARNING("ADSBee::UpdateLR2021", "No confirmed RX for %lu ms; reconfiguring receiver.",
                            (unsigned long)(now_ms - lr2021_last_rx_ok_ms_));
            ApplyReceiverConfig();
        }
    }

    // Validity watchdog: a mis-framed FIFO stream produces endless frames that all fail CRC while
    // the chip reports kRx and sweeps complete -- invisible to the chip-mode ladder above. Real
    // traffic statistically cannot produce this many consecutive frames with zero CRC passes, so
    // treat it as a wedged receiver and re-run the full bring-up (the AT+RX_ENABLE-toggle
    // equivalent, which provably recovers). In dead-quiet airspace, noise frames accumulate slowly,
    // so a spurious reconfig is rare and costs nothing (there is no traffic to lose).
    if (lr2021_frames_since_valid_ >= kMaxFramesWithoutValid &&
        now_ms - lr2021_last_recovery_ms_ > kRxRecoveryBackoffMs) {
        lr2021_frames_since_valid_ = 0;
        lr2021_validity_reconfig_count++;
        CONSOLE_WARNING("ADSBee::UpdateLR2021", "%lu frames without a valid packet; reconfiguring receiver.",
                        (unsigned long)kMaxFramesWithoutValid);
        ApplyReceiverConfig();  // Stamps lr2021_last_recovery_ms_ for the backoff.
    }

    uint32_t elapsed_us = get_time_since_boot_us() - start_us;
    if (elapsed_us > max_lr2021_us) {
        max_lr2021_us = elapsed_us;
    }
    return success;
}

void ADSBee::ParseLR2021RxFifo(const uint8_t* rx_buf, uint16_t rx_len_bytes, uint64_t mlat_timestamp_us) {
    // The FIFO packet length depends on the preamble mode: DF17 mode captures a shorter
    // remainder because the detector consumed the preamble + DF17 header bits.
    const bool df17_mode = LR2021::IsOokDF17PreambleMode(r1090_preamble_mode_);
    const uint16_t packet_len_bytes = LR2021::GetOokRxPacketLenBytes(r1090_preamble_mode_);
    uint16_t num_packets = packet_len_bytes ? (rx_len_bytes / packet_len_bytes) : 0;
    for (uint16_t i = 0; i < num_packets; i++) {
        const uint8_t* packet_start = rx_buf + i * packet_len_bytes;

        uint32_t rx_word_buf[RawModeSPacket::kMaxPacketLenWords32] = {0};
        if (df17_mode) {
            // Reconstruct the full 112-bit frame by prepending the known DF=17 header bits
            // (which the detector consumed) in front of the captured remainder, so the decoder +
            // software CRC validate the whole frame.
            const LR2021::OokDetectorConfig& detector = LR2021::kOokDF17Detector;
            SetNBitsInWordBuffer(detector.header_len_bits, detector.header_bits, 0, rx_word_buf);
            uint32_t remainder_words[RawModeSPacket::kMaxPacketLenWords32] = {0};
            ByteBufferToWordBuffer(packet_start, remainder_words, packet_len_bytes);
            const uint16_t remainder_bits = packet_len_bytes * 8;
            for (uint16_t b = 0; b < remainder_bits; b += 8) {
                uint16_t chunk = (remainder_bits - b) < 8 ? (remainder_bits - b) : 8;
                uint32_t val = GetNBitsFromWordBuffer(chunk, b, remainder_words);
                SetNBitsInWordBuffer(chunk, val, detector.header_len_bits + b, rx_word_buf);
            }
        } else {
            ByteBufferToWordBuffer(packet_start, rx_word_buf, packet_len_bytes);
        }
        RawModeSPacket raw_packet(rx_word_buf, RawModeSPacket::kExtendedSquitterPacketNumWords32);
        raw_packet.mlat_48mhz_64bit_counts = mlat_timestamp_us * 48;
        lr2021_frames_since_valid_++;  // Zeroed on any valid decode; feeds the validity watchdog.
        // Record demod + raw frame metrics (valid frames are recorded during dictionary
        // ingestion). All LR2021 captures are 112-bit extended-squitter-length frames.
        aircraft_dictionary.Record1090Demod();
        aircraft_dictionary.Record1090RawExtendedSquitterFrame();
        if (packet_decoder.raw_mode_s_packet_queue.IsFull()) {
            packet_decoder.raw_queue_overflow_count++;
        }
        packet_decoder.raw_mode_s_packet_queue.Enqueue(raw_packet);
    }
}