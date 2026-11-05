#pragma once

#include <ti/drivers/dpl/ClockP.h>

#include "bsp.hh"

class LEDs {
   public:
    static constexpr uint32_t kMaxNumLEDs = 4;

    struct LEDConfig {
        uint16_t pins[kMaxNumLEDs];
        uint16_t num_leds;
    };

    LEDs(const LEDConfig& config) : config_(config) {}

    void Init();
    void DeInit();
    void Update();
    // Stops the Update() clock and turns the LEDs off for SYNC sleep: a periodic ClockP keeps the MCU out of
    // STANDBY, and outputs stay latched in STANDBY.
    void Suspend();
    // Restarts the Update() clock after SYNC sleep.
    void Resume();

    // Status/activity flash. Ignored while AT+LED_ENABLE=0. A flash never shortens one already in progress.
    void FlashLED(uint16_t pin, uint32_t duration_ms);
    // AT+LED_BLINK: lights the LED for duration_ms even while AT+LED_ENABLE=0.
    void ForceBlinkLED(uint16_t pin, uint32_t duration_ms);

   private:
    static void TimerCallback(uintptr_t arg);

    // Index of pin in config_.pins, or -1.
    int16_t IndexOf(uint16_t pin) const;
    void Light(uint16_t i, uint32_t duration_ms, bool force);

    LEDConfig config_;
    ClockP_Struct clock_struct_;
    ClockP_Handle clock_handle_;

    volatile uint32_t led_on_timestamp_ms_[kMaxNumLEDs] = {0};
    volatile uint32_t led_on_duration_ms_[kMaxNumLEDs] = {0};
    volatile bool led_on_[kMaxNumLEDs] = {false};
    volatile bool led_forced_[kMaxNumLEDs] = {false};  // Forced blink in progress: ignores AT+LED_ENABLE=0.
};

extern LEDs leds;