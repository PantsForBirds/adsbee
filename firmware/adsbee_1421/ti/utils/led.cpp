#include "led.hh"

#include "hal.hh"
#include "settings.hh"

#include <ti/drivers/dpl/HwiP.h>

void LEDs::TimerCallback(uintptr_t arg) { static_cast<LEDs*>(reinterpret_cast<void*>(arg))->Update(); }

void LEDs::Init() {
    // GPIO pins are pre-configured via SysConfig; just turn them all off.
    for (uint16_t i = 0; i < config_.num_leds; i++) {
        GPIO_write(config_.pins[i], 0);
    }
    // Schedule Update() to be called from a 1ms repeating ClockP. ClockP periods are in system ticks,
    // which main() sets to 100 us.
    uint32_t period_ticks = 1000 / ClockP_getSystemTickPeriod();
    ClockP_Params params;
    ClockP_Params_init(&params);
    params.period = period_ticks;
    params.arg = reinterpret_cast<uintptr_t>(this);
    params.startFlag = true;
    clock_handle_ = ClockP_construct(&clock_struct_, TimerCallback, period_ticks, &params);
}

void LEDs::DeInit() {
    ClockP_stop(clock_handle_);
    ClockP_destruct(&clock_struct_);
    for (uint16_t i = 0; i < config_.num_leds; i++) {
        GPIO_write(config_.pins[i], 0);
    }
}

void LEDs::Suspend() {
    ClockP_stop(clock_handle_);
    for (uint16_t i = 0; i < config_.num_leds; i++) {
        GPIO_write(config_.pins[i], 0);
        led_on_[i] = false;
        led_forced_[i] = false;
    }
}

void LEDs::Resume() { ClockP_start(clock_handle_); }

void LEDs::Update() {
    uint32_t now_ms = get_time_since_boot_ms();
    bool enabled = settings_manager.settings.led_enabled;
    for (uint16_t i = 0; i < config_.num_leds; i++) {
        // AT+LED_ENABLE=0 turns a lit LED off right away, unless it's a forced blink.
        if (led_on_[i] &&
            (now_ms - led_on_timestamp_ms_[i] >= led_on_duration_ms_[i] || (!enabled && !led_forced_[i]))) {
            GPIO_write(config_.pins[i], 0);
            led_on_[i] = false;
            led_forced_[i] = false;
        }
    }
}

int16_t LEDs::IndexOf(uint16_t pin) const {
    for (uint16_t i = 0; i < config_.num_leds; i++) {
        if (config_.pins[i] == pin) {
            return i;
        }
    }
    return -1;
}

void LEDs::Light(uint16_t i, uint32_t duration_ms, bool force) {
    uint32_t now_ms = get_time_since_boot_ms();
    // Update() runs from a clock interrupt: keep it from seeing a half-updated LED.
    uintptr_t key = HwiP_disable();
    // Later off-deadline wins, so a short packet flash doesn't cut a longer blink short.
    if (!led_on_[i] ||
        (int32_t)((now_ms + duration_ms) - (led_on_timestamp_ms_[i] + led_on_duration_ms_[i])) > 0) {
        led_on_timestamp_ms_[i] = now_ms;
        led_on_duration_ms_[i] = duration_ms;
    }
    if (force) {
        led_forced_[i] = true;
    }
    led_on_[i] = true;
    GPIO_write(config_.pins[i], 1);
    HwiP_restore(key);
}

void LEDs::FlashLED(uint16_t pin, uint32_t duration_ms) {
    int16_t i = IndexOf(pin);
    if (i < 0 || !settings_manager.settings.led_enabled) {
        return;
    }
    Light(i, duration_ms, false);
}

void LEDs::ForceBlinkLED(uint16_t pin, uint32_t duration_ms) {
    int16_t i = IndexOf(pin);
    if (i >= 0) {
        Light(i, duration_ms, true);
    }
}
