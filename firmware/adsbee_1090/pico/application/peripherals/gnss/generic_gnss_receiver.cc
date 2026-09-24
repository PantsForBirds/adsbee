#include "generic_gnss_receiver.hh"

#include "comms.hh"
#include "hal.hh"

bool GenericGNSSReceiver::Init() {
    // Hold the receiver off while uart0 and its pins are configured. This prevents startup bytes
    // from arriving at an old baud rate or while the UART peripheral is being reset.
    SetEnable(false);
    ClaimUart();
    comms_manager.SetBaudRate(SettingsManager::kGNSSUART, GetDefaultBaudrate());
    EnableRxInterrupt();

    if (config_.pps_pin != UINT16_MAX) {
        gpio_init(config_.pps_pin);
        gpio_set_dir(config_.pps_pin, GPIO_IN);
        gpio_pull_down(config_.pps_pin);
        pps_last_level_ = gpio_get(config_.pps_pin);
    }

    // Arm a non-blocking pre-power delay. Update() will assert module power only after the UART,
    // interrupt handler, and ring buffer have remained ready for the full settling interval.
    initializing_ = false;
    suspended_ = false;
    healthy_ = true;
    active_ = true;
    power_enable_pending_ = true;
    uart_ready_timestamp_ms_ = get_time_since_boot_ms();
    notify_observed_valid_ = false;
    notify_last_emitted_valid_ = false;
    notify_pending_ = false;
    notify_has_emitted_ = false;
    notify_last_timestamp_ms_ = 0;
    return true;
}
