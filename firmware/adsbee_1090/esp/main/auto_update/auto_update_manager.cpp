#include "auto_update_manager.hh"

#include <cstring>
#include <new>

#include "comms.hh"
#include "device_info.hh"
#include "esp_random.h"
#include "swarm_manager.hh"
#include "esp_heap_caps.h"
#include "hal.hh"
#include "hardware_capabilities.hh"
#include "nvs.h"
#include "object_dictionary.hh"
#include "settings.hh"

#ifndef AUTO_UPDATE_API_BASE
#define AUTO_UPDATE_API_BASE "https://adsbee.aero/api/fw/v1/"
#endif
const char AutoUpdateManager::kAPIBase[] = AUTO_UPDATE_API_BASE;

static const char kNVSNamespace[] = "auto_update";
static const char kNVSStateKey[] = "state";

AutoUpdateManager auto_update_manager;

static void auto_update_task(void* pvParameters) {
    auto_update_manager.Task();
    vTaskDelete(NULL);
}

void AutoUpdateManager::RunningVersion(char* buf, size_t buf_size) {
    if (ObjectDictionary::kFirmwareVersionReleaseCandidate == 0) {
        snprintf(buf, buf_size, "%d.%d.%d", ObjectDictionary::kFirmwareVersionMajor,
                 ObjectDictionary::kFirmwareVersionMinor, ObjectDictionary::kFirmwareVersionPatch);
    } else {
        snprintf(buf, buf_size, "%d.%d.%d-rc%d", ObjectDictionary::kFirmwareVersionMajor,
                 ObjectDictionary::kFirmwareVersionMinor, ObjectDictionary::kFirmwareVersionPatch,
                 ObjectDictionary::kFirmwareVersionReleaseCandidate);
    }
}

void AutoUpdateManager::Update() {
    if (task_ != nullptr) return;  // A check is running.
    uint32_t now_ms = NowMs();
    if (!Enabled()) {
        if (updater_ != nullptr) {
            xSemaphoreTake(updater_mutex_, portMAX_DELAY);
            delete updater_;
            updater_ = nullptr;
            xSemaphoreGive(updater_mutex_);
            // The OTA keys are only kept while the feature is on.
            object_dictionary.device_identity_valid = false;
            memset(&object_dictionary.device_identity, 0, sizeof(object_dictionary.device_identity));
            low_memory_ = false;
        }
        return;
    }
    if (updater_ == nullptr) {
        char version[kVersionMaxLen + 1];
        RunningVersion(version, sizeof(version));
        AutoUpdater::Config config;
#ifdef AUTO_UPDATE_FIRST_CHECK_S
        config.first_check_delay_ms = AUTO_UPDATE_FIRST_CHECK_S * 1000;  // Bench builds only.
        config.first_check_jitter_ms = 0;
#endif
        AutoUpdater* updater = new (std::nothrow) AutoUpdater(*this, version, config);
        if (console_rx_buf_ == nullptr) console_rx_buf_ = xStreamBufferCreate(kConsoleRxBufLenBytes, 1);
        if (updater == nullptr || console_rx_buf_ == nullptr) {
            delete updater;
            return;
        }
        xSemaphoreTake(updater_mutex_, portMAX_DELAY);
        updater_ = updater;
        xSemaphoreGive(updater_mutex_);
        next_run_timestamp_ms_ = now_ms + updater->FirstDelayMs(now_ms);
        CONSOLE_INFO("AutoUpdateManager::Update", "Automatic updates on, running %s.", version);
    }
    if (static_cast<int32_t>(now_ms - next_run_timestamp_ms_) < 0) return;
    xSemaphoreTake(updater_mutex_, portMAX_DELAY);
    updater_->SetTargetPending(settings_manager.settings.swarm_enabled && !swarm_answered_);
    uint32_t target_wait_ms = updater_->TargetWaitMs();
    xSemaphoreGive(updater_mutex_);
    if (target_wait_ms > 0) {
        // No task meanwhile: SetSwarmTarget() brings the check forward.
        static constexpr uint32_t kTargetWaitStepMs = 30 * 1000;
        next_run_timestamp_ms_ = now_ms + (target_wait_ms < kTargetWaitStepMs ? target_wait_ms : kTargetWaitStepMs);
        return;
    }

    // A check costs about 40 kB of internal RAM at its peak (task stack plus the TLS handshake). Without that much to
    // spare, WiFi and Ethernet allocations start failing, so wait for a better moment.
    uint32_t free_bytes = HardwareCapabilities::GetInternalFreeBytes();
    if (free_bytes < kMinFreeHeapBytesToRun) {
        if (!low_memory_) {
            CONSOLE_WARNING("AutoUpdateManager::Update",
                            "Not checking for updates: %lu Bytes of heap free, need %lu. Turning off the WiFi access "
                            "point or station frees memory.",
                            (unsigned long)free_bytes, (unsigned long)kMinFreeHeapBytesToRun);
        }
        low_memory_ = true;
        next_run_timestamp_ms_ = now_ms + kLowMemoryRetryMs;
        return;
    }
    low_memory_ = false;
    if (swarm_manager.Busy()) return;  // One TLS session at a time.
    if (xTaskCreate(auto_update_task, "auto_update", kTaskStackSizeBytes, NULL, kTaskPriority, &task_) != pdPASS) {
        task_ = nullptr;
        next_run_timestamp_ms_ = now_ms + kLowMemoryRetryMs;
        CONSOLE_ERROR("AutoUpdateManager::Update", "Not enough memory for the automatic update task.");
    }
}

void AutoUpdateManager::Task() {
    // One check (and install, when one is offered). The task and its TLS buffers exist only for this long.
    xStreamBufferReset(console_rx_buf_);
    tap_line_len_ = 0;
    capturing_ = true;
    xSemaphoreTake(updater_mutex_, portMAX_DELAY);
    updater_->SetTarget(settings_manager.settings.swarm_enabled ? swarm_target_ : nullptr);
    xSemaphoreGive(updater_mutex_);
    uint32_t delay_ms = updater_->RunOnce();
    capturing_ = false;
    reads_console_ = false;
    http_.Close();
    if (updater_->state() == AutoUpdater::kFailed) {
        CONSOLE_WARNING("AutoUpdateManager::Task", "Update to %s failed: %s.", updater_->version(), updater_->detail());
    }
    CONSOLE_INFO("AutoUpdateManager::Task",
                 "State %s, next check in %lu s. Stack headroom %u Bytes, lowest free heap since boot %u Bytes.",
                 AutoUpdater::StateToStr(updater_->state()), (unsigned long)(delay_ms / 1000),
                 (unsigned)uxTaskGetStackHighWaterMark(NULL),
                 (unsigned)heap_caps_get_minimum_free_size(HardwareCapabilities::kInternalHeapCaps));
    next_run_timestamp_ms_ = NowMs() + delay_ms;
    task_ = nullptr;
}

void AutoUpdateManager::GetStatus(Status& status) {
    status = Status();
    status.enabled = Enabled();
    xSemaphoreTake(updater_mutex_, portMAX_DELAY);
    if (updater_ != nullptr) {
        status.state = low_memory_ ? "low memory" : AutoUpdater::StateToStr(updater_->state());
        strncpy(status.version, updater_->version(), kVersionMaxLen);
        status.progress_percent = updater_->progress_percent();
    }
    xSemaphoreGive(updater_mutex_);
}

void AutoUpdateManager::SetSwarmTarget(const char* tag) {
    if (tag == nullptr) tag = "";
    xSemaphoreTake(updater_mutex_, portMAX_DELAY);
    bool changed = strncmp(swarm_target_, tag, sizeof(swarm_target_) - 1) != 0 || !swarm_answered_;
    strncpy(swarm_target_, tag, sizeof(swarm_target_) - 1);
    swarm_target_[sizeof(swarm_target_) - 1] = '\0';
    swarm_answered_ = true;
    xSemaphoreGive(updater_mutex_);
    if (!changed) return;
    CONSOLE_INFO("AutoUpdateManager::SetSwarmTarget", "Swarm firmware target: %s.", tag[0] ? tag : "none");
    static constexpr uint32_t kTargetCheckDelayMs = 30 * 1000;
    uint32_t soon_ms = NowMs() + kTargetCheckDelayMs;
    if (updater_ != nullptr && task_ == nullptr && static_cast<int32_t>(next_run_timestamp_ms_ - soon_ms) > 0) {
        next_run_timestamp_ms_ = soon_ms;
    }
}

void AutoUpdateManager::NoteUserOTA(const char* buf, size_t len, const char* marker) {
    size_t marker_len = strlen(marker);
    for (size_t i = 0; i + marker_len <= len; i++) {
        if (buf[i] == marker[0] && memcmp(buf + i, marker, marker_len) == 0) {
            last_user_ota_timestamp_ms_ = get_time_since_boot_ms();
            user_ota_seen_ = true;
            return;
        }
    }
}

void AutoUpdateManager::OnConsoleOutput(const uint8_t* buf, size_t len) {
    if (capturing_) {
        // Keep only the replies the updater waits for, so a flood of log lines can't push them out of the buffer.
        for (size_t i = 0; i < len; i++) {
            char c = static_cast<char>(buf[i]);
            if (c != '\n') {
                if (c != '\r' && tap_line_len_ < sizeof(tap_line_)) tap_line_[tap_line_len_++] = c;
                continue;
            }
            bool overflowed = tap_line_len_ >= sizeof(tap_line_);
            tap_line_[overflowed ? sizeof(tap_line_) - 1 : tap_line_len_] = '\0';
            if (!overflowed && (strcmp(tap_line_, "OK") == 0 || strcmp(tap_line_, "READY") == 0 ||
                                strncmp(tap_line_, "ERROR", 5) == 0 || strncmp(tap_line_, "Partition: ", 11) == 0)) {
                tap_line_[tap_line_len_++] = '\n';
                xStreamBufferSend(console_rx_buf_, tap_line_, tap_line_len_, 0);
            }
            tap_line_len_ = 0;
        }
    } else if (updater_ != nullptr) {
        // An OTA driven from USB or the web shows up as the RP2040's replies to its writes.
        NoteUserOTA(reinterpret_cast<const char*>(buf), len, "Bytes to partition");
    }
}

void AutoUpdateManager::OnConsoleInput(const char* buf, size_t len) {
    if (updater_ != nullptr) NoteUserOTA(buf, len, "AT+OTA=");
}

uint32_t AutoUpdateManager::NowMs() { return get_time_since_boot_ms(); }

void AutoUpdateManager::SleepMs(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

uint32_t AutoUpdateManager::Random() { return esp_random(); }

bool AutoUpdateManager::Enabled() { return settings_manager.settings.auto_update_enabled; }

bool AutoUpdateManager::Online() { return comms_manager.HasIP(); }

bool AutoUpdateManager::GetIdentity(Identity& identity) {
    object_dictionary.device_identity_valid = false;
    if (!object_dictionary.RequestSCCommand(ObjectDictionary::SCCommandRequest{
            .command = ObjectDictionary::SCCommand::kCmdWriteToSlaveRequireAck,
            .addr = ObjectDictionary::Address::kAddrDeviceIdentity,
            .offset = 0,
            .len = sizeof(SettingsManager::DeviceIdentity)})) {
        return false;
    }
    uint32_t start_ms = NowMs();
    while (!object_dictionary.device_identity_valid) {
        if (NowMs() - start_ms > kIdentityTimeoutMs) {
            CONSOLE_ERROR("AutoUpdateManager::GetIdentity", "No device identity from the RP2040.");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    const SettingsManager::DeviceIdentity& id = object_dictionary.device_identity;
    snprintf(identity.unique_id, sizeof(identity.unique_id), "%02X%02X%02X%02X%02X%02X%02X%02X", id.unique_id[0],
             id.unique_id[1], id.unique_id[2], id.unique_id[3], id.unique_id[4], id.unique_id[5], id.unique_id[6],
             id.unique_id[7]);
    ObjectDictionary::ESP32DeviceInfo esp32_info = GetESP32DeviceInfo();
    snprintf(identity.mac_address, sizeof(identity.mac_address), "%02X:%02X:%02X:%02X:%02X:%02X",
             esp32_info.base_mac[0], esp32_info.base_mac[1], esp32_info.base_mac[2], esp32_info.base_mac[3],
             esp32_info.base_mac[4], esp32_info.base_mac[5]);
    strncpy(identity.part_code, id.part_code, sizeof(identity.part_code) - 1);
    for (uint16_t i = 0; i < SettingsManager::DeviceInfo::kNumOTAKeys; i++) {
        strncpy(identity.ota_keys[i], id.ota_keys[i], sizeof(identity.ota_keys[i]) - 1);
    }
    return true;
}

bool AutoUpdateManager::Http(const HttpRequest& request, HttpResult& result, BodySink* sink) {
    char url[sizeof(kAPIBase) + 160];
    snprintf(url, sizeof(url), "%s%s", kAPIBase, request.path);
    return http_.Request(url, request, result, sink);
}

void AutoUpdateManager::ConsoleSend(const uint8_t* data, size_t len) {
    reads_console_ = true;
    // Same queue the /console websocket feeds; the RP2040 pulls from it over SPI.
    static constexpr uint32_t kTimeoutMs = 10000;
    uint32_t start_ms = NowMs();
    size_t sent = 0;
    while (sent < len && NowMs() - start_ms < kTimeoutMs) {
        xSemaphoreTake(object_dictionary.network_console_rx_queue_mutex, portMAX_DELAY);
        // All or nothing, so a payload is never interleaved with a web client's typing.
        bool fits = static_cast<size_t>(object_dictionary.network_console_rx_queue.MaxNumElements() -
                                        object_dictionary.network_console_rx_queue.Length()) >= len;
        while (fits && sent < len &&
               object_dictionary.network_console_rx_queue.Enqueue(static_cast<char>(data[sent]))) {
            sent++;
        }
        xSemaphoreGive(object_dictionary.network_console_rx_queue_mutex);
        if (sent < len) vTaskDelay(pdMS_TO_TICKS(20));
    }
}

bool AutoUpdateManager::ConsoleReadLine(char* line, size_t line_size, uint32_t timeout_ms) {
    uint32_t start_ms = NowMs();
    size_t len = 0;
    for (;;) {
        uint32_t elapsed_ms = NowMs() - start_ms;
        if (elapsed_ms >= timeout_ms) return false;
        char c;
        if (xStreamBufferReceive(console_rx_buf_, &c, 1, pdMS_TO_TICKS(timeout_ms - elapsed_ms)) != 1) {
            return false;
        }
        if (c == '\n') {
            line[len] = '\0';
            return true;
        }
        if (len + 1 < line_size) line[len++] = c;  // Longer lines are cut.
    }
}

const char* AutoUpdateManager::ConsoleReportingProtocol() {
    SettingsManager::ReportingProtocol protocol =
        settings_manager.settings.reporting_protocols[SettingsManager::SerialInterface::kConsole];
    if (protocol == SettingsManager::ReportingProtocol::kNoReports ||
        protocol >= SettingsManager::ReportingProtocol::kNumProtocols) {
        return nullptr;
    }
    return SettingsManager::kReportingProtocolStrs[protocol];
}

bool AutoUpdateManager::UserOTAActive() {
    return user_ota_seen_ && NowMs() - last_user_ota_timestamp_ms_ < kUserOTAHoldoffMs;
}

void AutoUpdateManager::LoadState(SavedState& state) {
    state = SavedState();
    nvs_handle_t handle;
    if (nvs_open(kNVSNamespace, NVS_READONLY, &handle) != ESP_OK) return;
    SavedState stored;
    size_t len = sizeof(stored);
    if (nvs_get_blob(handle, kNVSStateKey, &stored, &len) == ESP_OK && len == sizeof(stored)) {
        state = stored;
    }
    nvs_close(handle);
}

void AutoUpdateManager::SaveState(const SavedState& state) {
    nvs_handle_t handle;
    if (nvs_open(kNVSNamespace, NVS_READWRITE, &handle) != ESP_OK) return;
    if (nvs_set_blob(handle, kNVSStateKey, &state, sizeof(state)) == ESP_OK) nvs_commit(handle);
    nvs_close(handle);
}
