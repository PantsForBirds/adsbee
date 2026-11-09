#include "esp32_mlat_sync.hh"

#include "comms.hh"  // Logging.
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_private/systimer.h"  // SYSTIMER_COUNTER_ESPTIMER
#include "esp_timer.h"
#include "hal/systimer_ll.h"
#include "object_dictionary.hh"
#include "soc/systimer_struct.h"

ESP32MLATSync esp32_mlat_sync;

namespace {

bool IRAM_ATTR OnEdgeCapture(mcpwm_cap_channel_handle_t, const mcpwm_capture_event_data_t* edata, void*) {
    esp32_mlat_sync.HandleEdgeCapture(edata->cap_value);
    return false;
}

bool IRAM_ATTR OnSoftCapture(mcpwm_cap_channel_handle_t, const mcpwm_capture_event_data_t* edata, void*) {
    esp32_mlat_sync.HandleSoftCapture(edata->cap_value);
    return false;
}

}  // namespace

bool ESP32MLATSync::Init() {
    // Input only, no pulls, before the capture channel connects to it.
    gpio_config_t io_config = {.pin_bit_mask = 1ull << kSyncPinGPIO,
                               .mode = GPIO_MODE_INPUT,
                               .pull_up_en = GPIO_PULLUP_DISABLE,
                               .pull_down_en = GPIO_PULLDOWN_DISABLE,
                               .intr_type = GPIO_INTR_DISABLE};
    if (gpio_config(&io_config) != ESP_OK) {
        CONSOLE_ERROR("ESP32MLATSync::Init", "Failed to configure the SYNC pin as an input.");
        return false;
    }

    mcpwm_capture_timer_config_t timer_config = {};
    timer_config.group_id = 0;
    timer_config.clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT;  // APB, 80 MHz.
    mcpwm_capture_channel_config_t edge_config = {};
    edge_config.gpio_num = kSyncPinGPIO;
    edge_config.prescale = 1;
    edge_config.flags.pos_edge = true;  // pull_up, pull_down and io_loop_back stay off.
    mcpwm_capture_channel_config_t soft_config = {};
    soft_config.gpio_num = -1;  // Software captures only.
    soft_config.prescale = 1;
    mcpwm_capture_event_callbacks_t edge_callbacks = {.on_cap = OnEdgeCapture};
    mcpwm_capture_event_callbacks_t soft_callbacks = {.on_cap = OnSoftCapture};
    uint32_t resolution_hz = 0;
    if (mcpwm_new_capture_timer(&timer_config, &capture_timer_) != ESP_OK ||
        mcpwm_capture_timer_get_resolution(capture_timer_, &resolution_hz) != ESP_OK ||
        resolution_hz != esp32_clock_math::kAPBTicksPerSystimerTick * esp32_clock_math::kSystimerTicksPerUs * 1'000'000 ||
        mcpwm_new_capture_channel(capture_timer_, &edge_config, &edge_channel_) != ESP_OK ||
        mcpwm_new_capture_channel(capture_timer_, &soft_config, &soft_channel_) != ESP_OK ||
        mcpwm_capture_channel_register_event_callbacks(edge_channel_, &edge_callbacks, nullptr) != ESP_OK ||
        mcpwm_capture_channel_register_event_callbacks(soft_channel_, &soft_callbacks, nullptr) != ESP_OK ||
        mcpwm_capture_channel_enable(edge_channel_) != ESP_OK || mcpwm_capture_channel_enable(soft_channel_) != ESP_OK ||
        mcpwm_capture_timer_enable(capture_timer_) != ESP_OK || mcpwm_capture_timer_start(capture_timer_) != ESP_OK) {
        CONSOLE_ERROR("ESP32MLATSync::Init", "Failed to start the SYNC pulse capture (capture clock %lu Hz).",
                      resolution_hz);
        return false;
    }
    return true;
}

uint64_t IRAM_ATTR ESP32MLATSync::SystimerNow() {
    // Same sequence as systimer_hal_get_counter_value(): re-read until the low word is stable.
    systimer_ll_counter_snapshot(&SYSTIMER, SYSTIMER_COUNTER_ESPTIMER);
    while (!systimer_ll_is_counter_value_valid(&SYSTIMER, SYSTIMER_COUNTER_ESPTIMER)) {
    }
    uint32_t lo_start = systimer_ll_get_counter_value_low(&SYSTIMER, SYSTIMER_COUNTER_ESPTIMER);
    uint32_t lo, hi;
    do {
        lo = lo_start;
        hi = systimer_ll_get_counter_value_high(&SYSTIMER, SYSTIMER_COUNTER_ESPTIMER);
        lo_start = systimer_ll_get_counter_value_low(&SYSTIMER, SYSTIMER_COUNTER_ESPTIMER);
    } while (lo_start != lo);
    return (static_cast<uint64_t>(hi) << 32) | lo;
}

void IRAM_ATTR ESP32MLATSync::HandleEdgeCapture(uint32_t capture) {
    // The capture timer can't be read directly: latch it with a software capture right next to a system timer read.
    portENTER_CRITICAL_ISR(&lock_);
    mcpwm_capture_channel_trigger_soft_catch(soft_channel_);
    pending_edge_systimer_now_ = SystimerNow();
    pending_edge_capture_ = capture;
    edge_pending_ = true;
    portEXIT_CRITICAL_ISR(&lock_);
}

void IRAM_ATTR ESP32MLATSync::HandleSoftCapture(uint32_t capture_now) {
    portENTER_CRITICAL_ISR(&lock_);
    if (edge_pending_) {
        edge_systimer_ = esp32_clock_math::CaptureToSystimerTicks(pending_edge_capture_, capture_now,
                                                                  pending_edge_systimer_now_);
        edge_count_ = edge_count_ + 1;
        edge_pending_ = false;
    }
    portEXIT_CRITICAL_ISR(&lock_);
}

void ESP32MLATSync::HandleSyncPulse(uint64_t mlat_48mhz_counts, uint32_t pulse_num) {
    portENTER_CRITICAL(&lock_);
    uint64_t edge_systimer = edge_systimer_;
    uint16_t edge_count = edge_count_;
    portEXIT_CRITICAL(&lock_);
    uint64_t edge_age_us = (SystimerNow() - edge_systimer) / esp32_clock_math::kSystimerTicksPerUs;
    if (!sync_pulse_pairing_.Pair(edge_count, pulse_num, edge_age_us > UINT32_MAX ? UINT32_MAX : edge_age_us,
                                  ObjectDictionary::kSyncPulseMaxAgeMs * 1000,
                                  ObjectDictionary::kSyncPulseMaxMatchedAgeMs * 1000)) {
        return;
    }
    portENTER_CRITICAL(&lock_);
    bool was_synced = time_sync_.IsSynced();
    MLATTimeSync::Result result = time_sync_.AddSyncPoint(static_cast<uint32_t>(edge_systimer), mlat_48mhz_counts);
    if (result != MLATTimeSync::kSkipped) {
        last_sync_point_systimer_ = edge_systimer;
    }
    SyncEvent event = result == MLATTimeSync::kSkipped       ? kSyncEventSkipped
                      : result == MLATTimeSync::kRestarted   ? kSyncEventRestarted
                      : time_sync_.IsSynced() && !was_synced ? kSyncEventSynced
                                                             : kSyncEventNone;
    if (event != kSyncEventNone) {
        sync_event_ = event;
        sync_event_pulse_num_ = pulse_num;
        sync_event_rate_error_ppb_ = static_cast<int32_t>(time_sync_.GetRateErrorPPB());
    }
    portEXIT_CRITICAL(&lock_);
}

void ESP32MLATSync::Update() {
    portENTER_CRITICAL(&lock_);
    SyncEvent event = sync_event_;
    uint32_t pulse_num = sync_event_pulse_num_;
    int32_t rate_error_ppb = sync_event_rate_error_ppb_;
    sync_event_ = kSyncEventNone;
    portEXIT_CRITICAL(&lock_);
    switch (event) {
        case kSyncEventSynced:
            CONSOLE_INFO("ESP32MLATSync::Update", "Remote ID timestamps on the MLAT clock (%ld ppb).", rate_error_ppb);
            break;
        case kSyncEventSkipped:
            CONSOLE_WARNING("ESP32MLATSync::Update", "Sync pulse %lu is off the fit: skipped.", pulse_num);
            break;
        case kSyncEventRestarted:
            CONSOLE_WARNING("ESP32MLATSync::Update", "Sync pulse %lu doesn't fit the previous ones: re-syncing.",
                            pulse_num);
            break;
        default:
            break;
    }
}

uint8_t ESP32MLATSync::GetMLATSyncFlags() {
    uint64_t systimer_now = SystimerNow();
    portENTER_CRITICAL(&lock_);
    uint64_t sync_point_age_ticks = systimer_now - last_sync_point_systimer_;
    if (time_sync_.GetNumSyncPoints() > 0 &&
        sync_point_age_ticks > static_cast<uint64_t>(kSyncPointMaxAgeUs) * esp32_clock_math::kSystimerTicksPerUs) {
        time_sync_.Restart();  // Pulses stopped (CC1312 reset or flashing, RP2040 restart).
    }
    bool synced = time_sync_.IsSynced();
    // A pulse went missing: ask for one before the fit gets too old.
    bool overdue = synced && sync_point_age_ticks > static_cast<uint64_t>(ObjectDictionary::kSyncPulseOverdueMs) *
                                                        1000 * esp32_clock_math::kSystimerTicksPerUs;
    uint8_t flags = ObjectDictionary::MLATSyncFlags(synced, time_sync_.WantsPoint() || overdue);
    portEXIT_CRITICAL(&lock_);
    return flags;
}

uint64_t ESP32MLATSync::SystimerToMLAT48MHzCounts(uint64_t systimer_ticks) {
    uint64_t mlat_48mhz_counts = 0;
    portENTER_CRITICAL(&lock_);
    uint64_t ticks_since_sync_point = systimer_ticks - last_sync_point_systimer_;
    uint64_t max_age_ticks = static_cast<uint64_t>(kSyncPointMaxAgeUs) * esp32_clock_math::kSystimerTicksPerUs;
    if (ticks_since_sync_point <= max_age_ticks || -ticks_since_sync_point <= max_age_ticks) {
        mlat_48mhz_counts = time_sync_.ToMLAT48MHzCounts(static_cast<uint32_t>(systimer_ticks));
    }
    portEXIT_CRITICAL(&lock_);
    return mlat_48mhz_counts;
}

uint64_t ESP32MLATSync::WiFiRxToMLAT48MHzCounts(uint32_t rx_timestamp_us, uint8_t rate, uint16_t len_bytes) {
    int64_t rx_us = wifi_rx_clock_.ToEspTimerUs(rx_timestamp_us, esp_timer_get_time(),
                                                esp32_clock_math::LegacyPSDUAirTimeUs(rate, len_bytes));
    return SystimerToMLAT48MHzCounts(static_cast<uint64_t>(rx_us) * esp32_clock_math::kSystimerTicksPerUs);
}
