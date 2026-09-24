#include <stddef.h>
#include <stdint.h>

extern "C" {
// Make sure these are linked in C.
#include <posix/unistd.h>

#include "NoRTOS.h"
#include "ti/drivers/Board.h"
#include "ti/drivers/GPIO.h"
#include "ti/drivers/Power.h"
#include "ti/drivers/SPI.h"
}

#include "adsbee.hh"
#include "bsp.hh"
#include "comms.hh"
#include "hal.hh"  // For get_time_since_boot_us.
#include "led.hh"
#include "object_dictionary.hh"
#include "packet_decoder.hh"
#include "sub_ghz_radio.hh"
#include "uat_packet_decoder.hh"

// #include "unistd.h" // For usleep.
#include <cstring>  // For malloc

// TemperatureCC26X2_Config TemperatureCC26X2_config = {
//     .intPriority = (7 << 5)  // Lowest priority.
// };

// BSP bsp;
// CPUMonitor user_core_monitor = CPUMonitor({
//     .idle_ticks_per_update_interval = 480000, // Arbitrary, assume 100 instructions per idle loop at 48MHz.
//     .full_usage_update_frequency_hz = 100,    // Minimum 100Hz update rate.
//     .update_interval_ms = 1000                // Update stats every second.
// });
ObjectDictionary object_dictionary;
// Pico pico_ll = Pico({});
// SPICoprocessor pico = SPICoprocessor({.interface = pico_ll});
SettingsManager settings_manager;
CommsManager comms_manager = CommsManager({});
LEDs leds = LEDs({
    .pins = {bsp.k1090LEDPin, bsp.kSubGLEDPin},
    .num_leds = 2,
});

ADSBee adsbee = ADSBee({});
PacketDecoder packet_decoder;
SubGHzRadio subg_radio({});
UATPacketDecoder uat_packet_decoder;

/**
 * A note on interrupt priorities (configured via SysConfig in syscfg/adsbee_1421.syscfg):
 *
 * RF driver (UAT): hwiPriority 5 (0xa0), swiPriority 1. The RF SWI must run ahead of every other SWI:
 * its RX callback has ~15 us after sync detect to issue CMD_PROP_SET_LEN or the packet is lost (see the
 * maxPktLen comment in sub_ghz_radio.cpp).
 * SPI (LR2021) + its DMA: interrupt priority 5 (0xa0), swiPriority 0 (default).
 *
 * NoRTOS SWIs are NON-preemptive: a higher-priority pending SWI runs next, but never preempts a running
 * one. The SPI completion callback (LR2021::SPICallback) therefore shares the SWI band with the UAT
 * deadline and must stay trivial -- see lr2021_async.cpp.
 */

void exception_handler() {
    // 1 Morse unit = 200ms. Dit = 1u on, dah = 3u on, element gap = 1u off,
    // letter gap = 3u off, word gap = 7u off.
    static constexpr uint32_t kUnitUs = 200000;

    auto flash = [](uint32_t units) {
        GPIO_write(bsp.kSubGLEDPin, 1);
        usleep(units * kUnitUs);
        GPIO_write(bsp.kSubGLEDPin, 0);
        usleep(kUnitUs);  // inter-element gap
    };

    while (1) {
        flash(1);
        flash(1);
        flash(1);             // S
        usleep(2 * kUnitUs);  // inter-letter gap (3u total, 1u already elapsed)
        flash(3);
        flash(3);
        flash(3);  // O
        usleep(2 * kUnitUs);
        flash(1);
        flash(1);
        flash(1);             // S
        usleep(6 * kUnitUs);  // inter-word gap (7u total, 1u already elapsed)
    }
}

/*
 *  ======== main ========
 */
int main(void) {
    Power_disablePolicy();  // Stop aggressive clock gating that messes with the debugger.

    NoRTOS_Config cfg;
    NoRTOS_getConfig(&cfg);
    cfg.clockTickPeriod = 100;  // Other values cause crash.
    NoRTOS_setConfig(&cfg);

    /* Call driver init functions */
    Board_init();
    leds.Init();
    comms_manager.Init();

    SPI_init();

    // Start NoRTOS AFTER system initialization.
    NoRTOS_start();

    // Arm the watchdog at the default timeout before touching settings, so a hang anywhere in the
    // load/init/apply path reboots into a visible retry loop instead of wedging the board with no
    // console. Apply() re-applies the persisted timeout once the settings are known good.
    adsbee.SetWatchdogTimeoutSec(SettingsManager::Settings::kDefaultWatchdogTimeoutSec);

    // Load settings before adsbee.Init() so it can seed the LR2021 interface enable (AT+LR_ENABLE)
    // from the persisted value — a device saved with the bus released must not drive it during boot.
    // Apply() stays after subg_radio.Init() since it configures the SubGHz RF core.
    settings_manager.Load();
    // Feed between steps so the watchdog bounds each one individually rather than their sum: the point
    // is to catch a wedged step, not to impose a deadline on a slow-but-progressing boot.
    adsbee.FeedWatchdog();
    adsbee.Init();
    adsbee.FeedWatchdog();
    subg_radio.Init();
    adsbee.FeedWatchdog();
    settings_manager.Apply();

    leds.FlashLED(bsp.k1090LEDPin, 100);  // Flash the LED for 100ms.

#ifdef RFI_LF_S11_SCAN
    uint32_t rf_frequency = 800e6;
    const uint32_t rf_frequency_min = 800e6;
    const uint32_t rf_frequency_max = 1090e6;
    const uint32_t rf_frequency_step = 10e6;
    while (true) {
        adsbee.lr2021.SetRfFrequency(rf_frequency);
        rf_frequency += rf_frequency_step;
        if (rf_frequency > rf_frequency_max) {
            rf_frequency = rf_frequency_min;
        }
        usleep(100000);  // Sleep for 100ms.
    }
#endif

    while (true) {
        // An external host can force a low-power sleep by driving the SYNC line HIGH. The SYNC rising-
        // edge ISR has already handed the shared LR2021 bus off (tri-state + command abort) within
        // microseconds of the edge; this branch is the graceful half. Checking at the top of the loop
        // lets in-progress work (e.g. draining UART output) complete first. Quiesce the SubGHz RF core
        // (releases its STANDBY power constraint), then EnterSyncSleep() finishes the LR2021 power-down
        // and puts the MCU into STANDBY (SRAM retained) until SYNC drops, re-initializing the LR2021 on
        // wake. Resume() restarts UAT reception. Restart the loop cleanly afterward.
        if (adsbee.SyncSleepRequested()) {
            subg_radio.Suspend();     // release the RF core's STANDBY power constraint
            comms_manager.Suspend();  // release the UART RX STANDBY power constraint
            adsbee.EnterSyncSleep();
            comms_manager.Resume();   // re-arm UART RX
            subg_radio.Resume();      // restart UAT reception
            continue;
        }

        uint64_t loop_start_us = get_time_since_boot_us();
        leds.Update();
        // leds.FlashLED(bsp.k1090LEDPin, 100);  // Flash the LED for 100ms.
        // usleep(1000000);                        // Sleep for 1 second.
        // mode_s_radio.Update();
        adsbee.Update();
        packet_decoder.Update();
        subg_radio.Update();
        uat_packet_decoder.Update();
        comms_manager.Update();
        // Track the longest loop iteration (AT+RX_STATS max_loop_us) as the receiver's stall gauge.
        uint32_t loop_us = static_cast<uint32_t>(get_time_since_boot_us() - loop_start_us);
        if (loop_us > adsbee.max_loop_us) {
            adsbee.max_loop_us = loop_us;
        }
    }
}