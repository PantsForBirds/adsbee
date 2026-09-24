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

    void FlashLED(uint16_t pin, uint32_t duration_ms);

   private:
    static void TimerCallback(uintptr_t arg);

    LEDConfig config_;
    ClockP_Struct clock_struct_;
    ClockP_Handle clock_handle_;

    volatile uint32_t led_on_timestamp_ms_[kMaxNumLEDs] = {0};
    volatile uint32_t led_on_duration_ms_[kMaxNumLEDs] = {0};
    volatile bool led_on_[kMaxNumLEDs] = {false};
};

extern LEDs leds;