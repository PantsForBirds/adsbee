#pragma once

#include <cstddef>
#include <cstdint>

#include "sha256.hh"

/**
 * Opt-in automatic firmware updates from the adsbee.aero firmware service (API v1, see docs/guides/update-firmware.md).
 *
 * Runs on the ESP32 in its own task. It asks the service what this device should run and, when that differs from the
 * running version, downloads the .ota file in small HTTP ranges and feeds the RP2040 through the network console with
 * the same AT+OTA commands the web updater sends. Only what the service recommends is installed, and an older version
 * only when the service says rollback.
 *
 * Everything the logic touches goes through AutoUpdatePlatform, so host tests run it against fakes.
 */
class AutoUpdatePlatform {
   public:
    static constexpr size_t kVersionMaxLen = 23;

    struct Identity {
        char unique_id[17] = {0};    // RP2040 flash unique ID, 16 hex characters.
        char mac_address[18] = {0};  // ESP32 base MAC, AA:BB:CC:DD:EE:FF.
        char part_code[27] = {0};
        char ota_keys[2][129] = {{0}};
    };

    struct HttpRequest {
        const char* method = "GET";
        const char* path = "";         // Relative to the API base.
        const char* token = nullptr;   // Bearer token, or nullptr.
        const char* body = nullptr;    // JSON body, or nullptr.
        int32_t range_first = -1;      // Range: bytes=first-last when >= 0.
        int32_t range_last = -1;
    };

    struct HttpResult {
        int status = 0;
        uint32_t retry_after_s = 0;
    };

    // Receives the response body in pieces, with the response's status. Return false to abort the transfer.
    class BodySink {
       public:
        virtual ~BodySink() {}
        virtual bool OnBody(int status, const uint8_t* data, size_t len) = 0;
    };

    // Survives a reboot.
    struct SavedState {
        char target[kVersionMaxLen + 1] = {0};   // Version the attempts below are for.
        uint8_t attempts = 0;                    // Install attempts for target in the current window.
        char pending[kVersionMaxLen + 1] = {0};  // Version that was told to boot; reported once it runs.
    };

    virtual ~AutoUpdatePlatform() {}

    virtual uint32_t NowMs() = 0;
    virtual void SleepMs(uint32_t ms) = 0;
    virtual uint32_t Random() = 0;
    // The AUTO_UPDATE setting.
    virtual bool Enabled() = 0;
    // WiFi station or Ethernet has an address.
    virtual bool Online() = 0;
    virtual bool GetIdentity(Identity& identity) = 0;
    // One HTTP exchange. False means no response (DNS, TCP, TLS, timeout or an aborted body).
    virtual bool Http(const HttpRequest& request, HttpResult& result, BodySink* sink) = 0;

    // The RP2040 console, as a /console client sees it.
    virtual void ConsoleSend(const uint8_t* data, size_t len) = 0;
    // Next console output line without its line ending. False after timeout_ms without one.
    virtual bool ConsoleReadLine(char* line, size_t line_size, uint32_t timeout_ms) = 0;
    // The reporting protocol on the console (binary reports would mix into the replies), or nullptr for none.
    virtual const char* ConsoleReportingProtocol() = 0;
    // Somebody else is using AT+OTA.
    virtual bool UserOTAActive() = 0;

    virtual void LoadState(SavedState& state) = 0;
    virtual void SaveState(const SavedState& state) = 0;
};

class AutoUpdater {
   public:
    enum State : uint8_t { kIdle = 0, kChecking, kDownloading, kInstalling, kFailed, kHeld, kNoKeys, kNumStates };

    struct Config {
        const char* product = "adsbee_1090";
        const char* channel = "stable";
        uint32_t first_check_delay_ms = 3 * 60 * 1000;
        uint32_t first_check_jitter_ms = 2 * 60 * 1000;
        uint32_t check_interval_ms = 24 * 60 * 60 * 1000;
        uint32_t check_jitter_ms = 60 * 60 * 1000;
        uint32_t offline_retry_ms = 60 * 1000;
        // After a request that got no usable answer: doubles per failure up to the maximum.
        uint32_t error_retry_min_ms = 5 * 60 * 1000;
        uint32_t error_retry_max_ms = 2 * 60 * 60 * 1000;
        uint32_t busy_retry_ms = 10 * 60 * 1000;  // A user-driven OTA is running.
        // Install attempts per target per check interval, and the wait after each failed one.
        uint8_t max_attempts = 3;
        uint32_t failure_retry_ms[2] = {15 * 60 * 1000, 60 * 60 * 1000};
        uint32_t token_lifetime_ms = 50 * 60 * 1000;  // The service issues 1 h tokens.
        uint32_t target_wait_ms = 15 * 60 * 1000;     // Uptime until which a pending swarm target is waited for.

        uint32_t range_len_bytes = 4096;  // Per HTTP request. Also bounds the TLS record the ESP32 must buffer.
        // A download gives up after this long without receiving a byte.
        uint32_t download_stall_limit_ms = 10 * 60 * 1000;
        uint32_t download_retry_min_ms = 2000;
        uint32_t download_retry_max_ms = 60 * 1000;

        uint8_t max_attempts_per_write = 5;
        uint32_t cmd_timeout_ms = 6000;
        uint32_t ready_timeout_ms = 6000;
        uint32_t write_timeout_ms = 8000;  // Longer than the RP2040's 5 s payload timeout, so its ERROR comes first.
        // Bytes per AT+OTA=ERASE, a multiple of 4096. Without a /console client the RP2040 sends console output to
        // the network only for 10 s after the last network input, and a whole image takes longer than that to erase.
        uint32_t erase_step_bytes = 256 * 1024;
        uint32_t erase_timeout_ms = 30000;  // Per step.
        uint32_t verify_timeout_ms = 30000;
        uint32_t drain_quiet_ms = 2000;
        uint32_t drain_max_ms = 20000;
        uint32_t reboot_wait_ms = 2 * 60 * 1000;
    };

    static constexpr uint32_t kFlashChunkLenBytes = 4096;  // One flash sector per AT+OTA=WRITE.
    static constexpr uint32_t kOTAHeaderLenBytes = 20;
    static constexpr uint32_t kOTAAppOffsetBytes = 4096;
    static constexpr uint32_t kOTAMagic = 0x0AD5BEEE;
    static constexpr uint32_t kOTAMaxNumPartitions = 8;
    static constexpr size_t kTokenMaxLen = 383;
    static constexpr size_t kDetailMaxLen = 79;

    AutoUpdater(AutoUpdatePlatform& platform, const char* running_version, const Config& config);
    AutoUpdater(AutoUpdatePlatform& platform, const char* running_version)
        : AutoUpdater(platform, running_version, Config()) {}

    // Delay before the first RunOnce(), given the time since boot.
    uint32_t FirstDelayMs(uint32_t uptime_ms);

    // One check, and one install when the service recommends another version. Blocks while it works.
    // @retval Milliseconds until the next call.
    uint32_t RunOnce();

    /**
     * Firmware a swarm server staged for this device (a version such as 0.9.2, or a release tag), installed instead of
     * the service's recommendation. nullptr or "" goes back to the recommendation.
     */
    void SetTarget(const char* version_or_tag);
    const char* target() const { return target_; }

    /**
     * True while a swarm server may still stage a target (AT+SWARM on, no answer yet since boot). The first check then
     * waits up to Config::target_wait_ms of uptime, and falls back to the recommendation after that.
     */
    void SetTargetPending(bool pending) { target_pending_ = pending; }
    // Milliseconds until a check may start because of SetTargetPending(), 0 if none.
    uint32_t TargetWaitMs();

    State state() const { return state_; }
    // Version the state is about (the update target), or "".
    const char* version() const { return version_; }
    uint8_t progress_percent() const { return progress_percent_; }
    // Why the last install failed, or "".
    const char* detail() const { return detail_; }

    static const char* StateToStr(State state);
    // <0, 0, >0 like strcmp for versions such as 0.9.1-rc8 and 0.9.1 (a release is newer than its candidates).
    static int CompareVersions(const char* a, const char* b);

   private:
    struct Offer {
        bool parsed = false;
        bool hold = false;
        bool rollback = false;
        char recommended[AutoUpdatePlatform::kVersionMaxLen + 1] = {0};
        char tag[64] = {0};
        bool has_asset = false;
        char url[128] = {0};
        char sha256[65] = {0};
        uint32_t size = 0;
    };

    class OfferReader;
    class TokenReader;
    class Download;

    enum Reply : uint8_t { kReplyOK, kReplyNoAnswer, kReplyRateLimited, kReplyRejected, kReplyError };

    Reply Request(const AutoUpdatePlatform::HttpRequest& request, AutoUpdatePlatform::BodySink* sink,
                  bool authenticated);
    Reply Authenticate();
    Reply GetOffer(Offer& offer);
    void Checkin(const char* state, const char* version, const char* detail);
    // Downloads and flashes the offer. Returns nullptr once the image is verified and ready to boot, else the reason.
    const char* Install(const Offer& offer);

    bool ConsoleCommand(const char* command, const char* want, uint32_t timeout_ms, char* match = nullptr,
                        size_t match_size = 0);
    bool WaitLine(const char* want, uint32_t timeout_ms, char* match = nullptr, size_t match_size = 0);
    void DrainConsole();
    bool FlashErase(uint32_t offset, uint32_t len);
    bool FlashWrite(uint32_t offset, const uint8_t* data, uint32_t len);

    void SetState(State state, const char* version = nullptr);
    uint32_t Jitter(uint32_t max_ms);
    uint32_t ErrorRetryMs();
    uint32_t NextCheckMs();

    AutoUpdatePlatform& platform_;
    Config config_;
    char running_version_[AutoUpdatePlatform::kVersionMaxLen + 1] = {0};

    State state_ = kIdle;
    char version_[AutoUpdatePlatform::kVersionMaxLen + 1] = {0};
    uint8_t progress_percent_ = 0;
    char detail_[kDetailMaxLen + 1] = {0};

    AutoUpdatePlatform::SavedState saved_;
    char token_[kTokenMaxLen + 1] = {0};
    uint32_t token_timestamp_ms_ = 0;
    int last_status_ = 0;
    uint32_t retry_after_s_ = 0;
    uint8_t num_errors_ = 0;           // Consecutive requests without a usable answer.
    uint32_t window_start_ms_ = 0;     // Start of the current attempt window.
    bool have_identity_ = false;
    AutoUpdatePlatform::Identity identity_;
    char target_[AutoUpdatePlatform::kVersionMaxLen + 1] = {0};
    bool target_pending_ = false;
};
