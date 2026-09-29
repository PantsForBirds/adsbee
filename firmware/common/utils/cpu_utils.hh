#pragma once

#include <cstdint>

#include "comms.hh"
#include "hal.hh"
#include "macros.hh"
#include "unit_conversions.hh"

#ifdef ON_PICO
#include "hardware/adc.h"
#include "pico/stdlib.h"
#elif defined(ON_TI)
extern "C" {
#include "ti/drivers/temperature/TemperatureCC26X2.h"
}
#elif defined(ON_ESP32)
#include "driver/temperature_sensor.h"
#endif

class CPUMonitor {
   public:
#ifdef ON_PICO
    static constexpr uint16_t kTempSensorADCChannel = 4;  // ADC channel for internal temperature sensor.
#endif

    struct CPUMonitorConfig {
#ifdef ON_ESP32
#else
        uint32_t idle_ticks_per_update_interval = 100e3;  // Loop ticks per interval during idle activity.
        uint32_t full_usage_update_frequency_hz =
            100;                             // If the loop slows down to this frequency or lower, CPU usage is 100%.
        uint32_t update_interval_ms = 1000;  // Interval at which to update CPU usage percentage.
#endif
    };

    CPUMonitor(const CPUMonitorConfig &config) : config_(config) {
#ifdef ON_ESP32
#else
        full_usage_ticks_per_update_interval_ =
            config_.update_interval_ms * config_.full_usage_update_frequency_hz / kMsPerSec;
#endif
    };

    static void Init();

    /**
     * Converts the idle time and total time elapsed over one measurement interval into a CPU usage percentage.
     * Both arguments are deltas of free-running counters (e.g. FreeRTOS run time stats), so the caller must subtract
     * the previous sample with unsigned arithmetic to stay wrap-safe.
     * @param[in] idle_delta Time spent in the idle task during the interval.
     * @param[in] total_delta Total time elapsed during the interval, in the same units.
     * @retval CPU usage percentage (0-100%). 0 if no time has elapsed.
     */
    static uint8_t UsagePercentFromIdleDelta(uint32_t idle_delta, uint32_t total_delta) {
        if (total_delta == 0) {
            return 0;
        }
        uint64_t idle_percent = static_cast<uint64_t>(idle_delta) * 100 / total_delta;
        return static_cast<uint8_t>(100 - (idle_percent > 100 ? 100 : idle_percent));
    }

#ifdef ON_ESP32
    /**
     * Reads the CPU usage of each core since the previous call, from FreeRTOS run time stats.
     */
    void ReadCPUUsage(uint8_t &core_0_usage_percent, uint8_t &core_1_usage_percent);
#else
    /**
     * Records an idle tick. Call once per main loop iteration.
     */
    inline void Tick() { ticks_since_last_update_++; }

    /**
     * Updates the CPU usage percentage. Self-regulates to run only at the configured CPU usage update interval.
     * Call frequently (at least once per CPU usage update interval).
     */
    void Update();

    /**
     * Returns the CPU usage as a percentage (0-100%). CPU usage is based on the number of idle ticks recorded vs
     * the configured maximum number of idle ticks that could be recorded if the core was doing nothing interesting
     * other than running the main loop. The maximum number of idle ticks is used to represent "0%" utilization. A
     * configurable value for the minimum permisible loop frequency is used to calculate the number of ticks that
     * represents "100%" utilization. Utilization is calculated by finding the proportion of excess idle ticks (over
     * 100% utilization tick count) recorded by the CPU.
     * @retval CPU usage percentage (0-100%).
     */
    inline uint8_t GetUsagePercent() const { return cpu_usage_percent_; }
#endif

    /**
     * Reads the internal temperature sensor and returns the temperature in degrees Celsius.
     * @retval Temperature in degrees Celsius.
     */
    static int8_t ReadTemperatureDegC();

   private:
    CPUMonitorConfig config_;
    uint32_t full_usage_ticks_per_update_interval_;

    uint32_t last_update_timestamp_ms_ = 0;
    uint32_t ticks_since_last_update_ = 0;

    uint8_t cpu_usage_percent_ = 0;

#ifdef ON_ESP32
    // Run time stats counters at the previous ReadCPUUsage() call.
    uint32_t last_total_run_time_ = 0;
    uint32_t last_idle_run_time_core_0_ = 0;
    uint32_t last_idle_run_time_core_1_ = 0;

    static temperature_sensor_handle_t temp_sensor_handle_;
    static bool temp_sensor_initialized_;
#endif
};