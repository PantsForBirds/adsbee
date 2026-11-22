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
