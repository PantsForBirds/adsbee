#pragma once

#include "auto_updater.hh"
#include "https_client.hh"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"

/**
 * Runs AutoUpdater (common/auto_update) on the ESP32: HTTPS to the firmware service, the RP2040 console through the
 * network console queues, and state in NVS. With the AUTO_UPDATE setting off nothing is allocated. With it on, a task
 * exists only for the duration of a check.
 */
class AutoUpdateManager : public AutoUpdatePlatform {
   public:
    // The firmware service. A bench build can point somewhere else: idf.py -DAUTO_UPDATE_API_BASE=http://host/api/fw/v1/
    static const char kAPIBase[];

    // Task stack is internal SRAM. The TLS handshake is the deep part.
    static constexpr uint32_t kTaskStackSizeBytes = 8192;
    static constexpr UBaseType_t kTaskPriority = tskIDLE_PRIORITY;  // Same as the feed task: never ahead of SPI or feeds.
    static constexpr uint16_t kConsoleRxBufLenBytes = 512;  // Holds replies only (see OnConsoleOutput).
    static constexpr uint32_t kIdentityTimeoutMs = 5000;
    // A manual AT+OTA seen this recently means an update is in progress.
    static constexpr uint32_t kUserOTAHoldoffMs = 10 * 60 * 1000;
    // Free internal heap needed to start a check: its 40 kB peak plus what the network stacks need meanwhile.
    static constexpr uint32_t kMinFreeHeapBytesToRun = 56 * 1024;
    static constexpr uint32_t kLowMemoryRetryMs = 10 * 60 * 1000;

    struct Status {
        bool enabled = false;
        const char* state = "idle";
        char version[kVersionMaxLen + 1] = {0};
        uint8_t progress_percent = 0;
    };

    /**
     * Starts a check when one is due. Call from the main loop.
     */
    void Update();

    /**
     * Console bytes from the RP2040. Called from the SPI receive task: never blocks.
     */
    void OnConsoleOutput(const uint8_t* buf, size_t len);

    /**
     * Console bytes a /console client sent to the RP2040.
     */
    void OnConsoleInput(const char* buf, size_t len);

    void GetStatus(Status& status);

    /**
     * Firmware the swarm account staged (SwarmManager), or nullptr for none: called with every swarm answer. Installed
     * instead of the recommendation, only while AUTO_UPDATE and AT+SWARM are on. RAM only: after a reboot the first
     * check waits for the first swarm answer (AutoUpdater::SetTargetPending). A new target is checked within a minute.
     */
    void SetSwarmTarget(const char* tag);

    // Running firmware version, e.g. 0.9.1-rc9.
    static void RunningVersion(char* buf, size_t buf_size);

    // A check or install is running.
    bool Busy() const { return task_ != nullptr; }

    /**
     * The updater is talking to the RP2040 console. Counted as a /console client in the status the RP2040 reads, so
     * the RP2040 keeps sending console output to the network: without a client it does so only for 10 s after the
     * last network input, and an erase takes longer than that.
     */
    bool ReadsConsole() const { return reads_console_; }

    void Task();

    // AutoUpdatePlatform
    uint32_t NowMs() override;
    void SleepMs(uint32_t ms) override;
    uint32_t Random() override;
    bool Enabled() override;
    bool Online() override;
    bool GetIdentity(Identity& identity) override;
    bool Http(const HttpRequest& request, HttpResult& result, BodySink* sink) override;
    void ConsoleSend(const uint8_t* data, size_t len) override;
    bool ConsoleReadLine(char* line, size_t line_size, uint32_t timeout_ms) override;
    const char* ConsoleReportingProtocol() override;
    bool UserOTAActive() override;
    void LoadState(SavedState& state) override;
    void SaveState(const SavedState& state) override;

   private:
    void NoteUserOTA(const char* buf, size_t len, const char* marker);

    TaskHandle_t task_ = nullptr;
    SemaphoreHandle_t updater_mutex_ = xSemaphoreCreateMutex();
    AutoUpdater* updater_ = nullptr;  // Exists while the setting is on.
    uint32_t next_run_timestamp_ms_ = 0;
    volatile bool low_memory_ = false;
    char swarm_target_[kVersionMaxLen + 16] = {0};
    volatile bool swarm_answered_ = false;  // A swarm answer since boot.

    HttpsClient http_;

    StreamBufferHandle_t console_rx_buf_ = nullptr;
    volatile bool capturing_ = false;  // The updater is reading the console.
    volatile bool reads_console_ = false;
    char tap_line_[96] = {0};          // Console line being assembled on the SPI receive task.
    size_t tap_line_len_ = 0;
    volatile uint32_t last_user_ota_timestamp_ms_ = 0;
    volatile bool user_ota_seen_ = false;
};

extern AutoUpdateManager auto_update_manager;
