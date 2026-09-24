#include "led.hh"

#include "hal.hh"

void LEDs::TimerCallback(uintptr_t arg) { static_cast<LEDs*>(reinterpret_cast<void*>(arg))->Update(); }

void LEDs::Init() {
    // GPIO pins are pre-configured via SysConfig; just turn them all off.
    for (uint16_t i = 0; i < config_.num_leds; i++) {
        GPIO_write(config_.pins[i], 0);
    }
    // Schedule Update() to be called from a 1ms repeating ClockP.
    ClockP_Params params;
    ClockP_Params_init(&params);
    params.period = 1;
    params.arg = reinterpret_cast<uintptr_t>(this);
    params.startFlag = true;
    clock_handle_ = ClockP_construct(&clock_struct_, TimerCallback, 1, &params);
}

void LEDs::DeInit() {
    ClockP_stop(clock_handle_);
    ClockP_destruct(&clock_struct_);
    for (uint16_t i = 0; i < config_.num_leds; i++) {
        GPIO_write(config_.pins[i], 0);
    }
}

void LEDs::Update() {
    uint32_t now_ms = get_time_since_boot_ms();
    for (uint16_t i = 0; i < config_.num_leds; i++) {
        if (led_on_[i] && now_ms - led_on_timestamp_ms_[i] >= led_on_duration_ms_[i]) {
            GPIO_write(config_.pins[i], 0);  // Turn off the LED
            led_on_[i] = false;
        }
    }
}

void LEDs::FlashLED(uint16_t pin, uint32_t duration_ms) {
    for (uint16_t i = 0; i < config_.num_leds; i++) {
        if (config_.pins[i] == pin) {
            GPIO_write(pin, 1);  // Turn on the LED
            led_on_timestamp_ms_[i] = get_time_since_boot_ms();
            led_on_duration_ms_[i] = duration_ms;
            led_on_[i] = true;
            break;
        }
    }
}