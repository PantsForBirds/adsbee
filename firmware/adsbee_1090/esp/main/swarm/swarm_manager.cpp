#include "swarm_manager.hh"

#include <cctype>
#include <cstring>
#include <new>

#include "adsbee_server.hh"
#include "auto_update_manager.hh"
#include "comms.hh"
#include "esp_heap_caps.h"
#include "esp_random.h"
#include "hal.hh"
#include "hardware_capabilities.hh"
#include "object_dictionary.hh"
#include "settings.hh"

SwarmManager swarm_manager;

static void swarm_task(void* pvParameters) {
    swarm_manager.Task();
    vTaskDelete(NULL);
}

void SwarmManager::Stop() {
    adsbee_server.aircraft_dictionary.SetSwarmRecorder(nullptr);
    xSemaphoreTake(mutex_, portMAX_DELAY);
    next_seq_ = reporter_->seq();
    delete reporter_;
    reporter_ = nullptr;
    xSemaphoreGive(mutex_);
    delete live_;
    live_ = nullptr;
    low_memory_ = false;
}

SwarmListenState SwarmManager::ListenState() {
    const SettingsManager::Settings& settings = settings_manager.settings;
    SwarmListenState state;
    state.r1090_rx_enabled = settings.r1090_rx_enabled;
    state.subg_enabled = settings.subg_enabled;
    state.subg_rx_enabled = settings.subg_rx_enabled;
    state.subg_mode = settings.subg_mode;
    state.remote_id_status = object_dictionary.device_status.remote_id_status;
    return state;
}

void SwarmManager::Update() {
    uint32_t now_ms = NowMs();
    if (now_ms - last_update_timestamp_ms_ < kUpdateIntervalMs) return;
    last_update_timestamp_ms_ = now_ms;
    if (live_ != nullptr) {
        // Whole seconds since the last count, also while a report is being sent.
        uint32_t seconds = (now_ms - last_listen_timestamp_ms_) / 1000;
        if (seconds > 0) {
            live_->AddListening(ListenState(), seconds);
            last_listen_timestamp_ms_ += seconds * 1000;
        }
    }
    if (task_ != nullptr) return;  // A report is being sent; it owns the pending report.
    const SettingsManager::Settings& settings = settings_manager.settings;
    if (!settings.swarm_enabled) {
        if (reporter_ != nullptr) Stop();
        return;
    }

    if (reporter_ == nullptr) {
        char version[AutoUpdatePlatform::kVersionMaxLen + 1];
        AutoUpdateManager::RunningVersion(version, sizeof(version));
        SwarmRecorder* live = new (std::nothrow) SwarmRecorder();
        SwarmReporter* reporter = new (std::nothrow) SwarmReporter(*this, version, SwarmReporter::Config(), next_seq_);
        if (live == nullptr || reporter == nullptr) {
            delete live;
            delete reporter;
            return;
        }
        live_ = live;
        last_listen_timestamp_ms_ = now_ms;
        reporter->Restart(*live_, now_ms);
        next_report_timestamp_ms_ = now_ms + reporter->FirstDelayMs(now_ms);
        strncpy(token_seen_, settings.swarm_token, sizeof(token_seen_) - 1);
        strncpy(server_seen_, settings.swarm_server, sizeof(server_seen_) - 1);
        xSemaphoreTake(mutex_, portMAX_DELAY);
        reporter_ = reporter;
        xSemaphoreGive(mutex_);
        adsbee_server.aircraft_dictionary.SetSwarmRecorder(live_);
        CONSOLE_INFO("SwarmManager::Update", "Swarm reports on, to %s.", settings.swarm_server);
    }
    if (strncmp(token_seen_, settings.swarm_token, sizeof(token_seen_) - 1) != 0 ||
        strncmp(server_seen_, settings.swarm_server, sizeof(server_seen_) - 1) != 0) {
        // A new token undoes a 410; neither inherits the old setup's backoff. Next report as if just turned on.
        strncpy(token_seen_, settings.swarm_token, sizeof(token_seen_) - 1);
        strncpy(server_seen_, settings.swarm_server, sizeof(server_seen_) - 1);
        reporter_->Reconfigure();
        next_report_timestamp_ms_ = now_ms + reporter_->FirstDelayMs(now_ms);
    }

    // Aircraft-derived receiver positions move with the aircraft: no range map from them.
    const ObjectDictionary::RP2040DeviceStatus& rp2040 = object_dictionary.composite_device_status.rp2040;
    SettingsManager::RxPosition::PositionSource source = rp2040.rx_position.source;
    bool range_valid = SwarmReporter::UsablePosition(rp2040.rx_position_available, source,
                                                     rp2040.rx_position.latitude_deg, rp2040.rx_position.longitude_deg);
    live_->record_range = range_valid;
    if (range_valid) live_->SetReceiverAltitude(FeetToMeters(rp2040.rx_position.FilterAltitudeFt()));

    if (static_cast<int32_t>(now_ms - next_report_timestamp_ms_) < 0) return;
    if (auto_update_manager.Busy()) {
        next_report_timestamp_ms_ = now_ms + kBusyRetryMs;  // One TLS session at a time.
        return;
    }
    uint32_t free_bytes = HardwareCapabilities::GetInternalFreeBytes();
    if (free_bytes < kMinFreeHeapBytesToRun) {
        if (!low_memory_) {
            CONSOLE_WARNING("SwarmManager::Update", "Not sending swarm report: %lu Bytes of heap free, need %lu.",
                            (unsigned long)free_bytes, (unsigned long)kMinFreeHeapBytesToRun);
        }
        low_memory_ = true;
        next_report_timestamp_ms_ = now_ms + kLowMemoryRetryMs;
        return;
    }
    low_memory_ = false;

    SwarmReporter::DeviceState state;
    state.auto_update = settings.auto_update_enabled;
    state.rx_position_valid = range_valid;
    state.latitude_deg = rp2040.rx_position.latitude_deg;
    state.longitude_deg = rp2040.rx_position.longitude_deg;
    state.altitude_m = FeetToMeters(rp2040.rx_position.FilterAltitudeFt());
    char source_str[SettingsManager::RxPosition::kPositionSourceStrMaxLen] = {0};
    if (source < SettingsManager::RxPosition::kNumPositionSources) {
        const char* s = SettingsManager::RxPosition::kPositionSourceStrs[source];
        for (size_t i = 0; s[i] && i + 1 < sizeof(source_str); i++) source_str[i] = tolower((unsigned char)s[i]);
    }
    state.rx_position_source = source_str;
    state.range_valid = range_valid;
    reporter_->TakePeriod(*live_, now_ms, state);

    strncpy(server_, settings.swarm_server, sizeof(server_) - 1);
    strncpy(token_, settings.swarm_token, sizeof(token_) - 1);
    if (xTaskCreate(swarm_task, "swarm", kTaskStackSizeBytes, NULL, kTaskPriority, &task_) != pdPASS) {
        task_ = nullptr;
        next_report_timestamp_ms_ = now_ms + kLowMemoryRetryMs;
        CONSOLE_ERROR("SwarmManager::Update", "Not enough memory for the swarm report task.");
    }
}

void SwarmManager::Task() {
    uint32_t start_ms = NowMs();
    uint32_t delay_ms = reporter_->Send(server_, token_);
    http_.Close();  // Hourly: not worth holding a TLS session's buffers.
    const SwarmReporter::Status& status = reporter_->status();
    CONSOLE_INFO("SwarmManager::Task",
                 "Report %lu: %s (HTTP %d, %u Bytes, %lu ms), next in %lu s. Stack headroom %u Bytes, lowest free heap "
                 "since boot %u Bytes.",
                 (unsigned long)status.seq, status.result, status.last_http_status, status.last_body_len,
                 (unsigned long)(NowMs() - start_ms), (unsigned long)(delay_ms / 1000),
                 (unsigned)uxTaskGetStackHighWaterMark(NULL),
                 (unsigned)heap_caps_get_minimum_free_size(HardwareCapabilities::kInternalHeapCaps));
    next_report_timestamp_ms_ = NowMs() + delay_ms;
    task_ = nullptr;
}

void SwarmManager::GetStatus(Status& status) {
    status = Status();
    status.enabled = settings_manager.settings.swarm_enabled;
    uint32_t now_s = NowMs() / 1000;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    if (reporter_ != nullptr) {
        const SwarmReporter::Status& s = reporter_->status();
        status.result = low_memory_ ? "low memory" : task_ != nullptr ? "sending" : s.stopped ? "removed" : s.result;
        status.http_status = s.last_http_status;
        status.last_report_s_ago = s.last_ok_s ? now_s - s.last_ok_s : UINT32_MAX;
        status.last_attempt_s_ago = s.last_attempt_s ? now_s - s.last_attempt_s : UINT32_MAX;
        int32_t next_ms = static_cast<int32_t>(next_report_timestamp_ms_ - NowMs());
        status.next_report_s = next_ms > 0 ? next_ms / 1000 : 0;
        status.seq = s.seq;
        status.body_len = s.last_body_len;
    }
    xSemaphoreGive(mutex_);
}

uint32_t SwarmManager::NowMs() { return get_time_since_boot_ms(); }

uint32_t SwarmManager::Random() { return esp_random(); }

bool SwarmManager::Online() { return comms_manager.HasIP(); }

bool SwarmManager::GetIdentity(AutoUpdatePlatform::Identity& identity) {
    if (auto_update_manager.Busy()) return false;  // It uses the same RP2040 request.
    bool ok = auto_update_manager.GetIdentity(identity);
    // Keep the OTA keys only as long as the report needs them (the updater keeps its own copy).
    object_dictionary.device_identity_valid = false;
    memset(&object_dictionary.device_identity, 0, sizeof(object_dictionary.device_identity));
    return ok;
}

bool SwarmManager::Http(const AutoUpdatePlatform::HttpRequest& request, AutoUpdatePlatform::HttpResult& result,
                        AutoUpdatePlatform::BodySink* sink) {
    return http_.Request(request.path, request, result, sink);
}

void SwarmManager::SetFirmwareTarget(const char* tag, const char* channel) {
    auto_update_manager.SetSwarmTarget(tag);
}
