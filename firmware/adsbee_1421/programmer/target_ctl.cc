#include "target_ctl.hh"

#include "board.hh"
#include "hardware/gpio.h"

void TargetCtlInit() {
    // SYNC: drive low before enabling the output so the target never sees a glitch sleep
    // request. Pads reset with pull-down enabled; pointless on an output.
    gpio_init(kPinSync);
    gpio_disable_pulls(kPinSync);
    gpio_put(kPinSync, 0);
    gpio_set_dir(kPinSync, GPIO_OUT);

    // RESET_N released: input, pulled up. RP2040 pads reset with the PULL-DOWN enabled and
    // gpio_init() does not clear it — left in place it fights the module's weak internal
    // RESET_N pull-up and holds the module in reset. Our internal pull-up reinforces the
    // module's instead, while staying open-drain so a JTAG probe is never fought.
    gpio_init(kPinResetN);
    gpio_pull_up(kPinResetN);
    gpio_put(kPinResetN, 0);  // Preload low so switching to output pulls reset immediately.
    gpio_set_dir(kPinResetN, GPIO_IN);
}

void TargetSetSync(bool high) { gpio_put(kPinSync, high); }

void TargetPulseReset() {
    gpio_set_dir(kPinResetN, GPIO_OUT);  // Drives the preloaded low level.
    sleep_ms(kResetPulseMs);
    gpio_set_dir(kPinResetN, GPIO_IN);
}

void TargetResetIntoApp() {
    TargetSetSync(false);
    TargetPulseReset();
}

void TargetResetIntoBootloader() {
    TargetSetSync(true);
    TargetPulseReset();
    // SYNC stays high so any further reset re-enters the bootloader; it is only sampled at boot.
}
