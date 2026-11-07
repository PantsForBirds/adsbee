#pragma once

#include <cstddef>
#include <cstdint>

#include "auto_updater.hh"  // AutoUpdatePlatform: HTTP types and device identity.
#include "settings.hh"
#include "swarm_recorder.hh"

/**
 * What SwarmReporter needs from the device. The ESP32 implements it in esp/main/swarm; host tests use fakes.
 */
class SwarmPlatform {
   public:
    virtual ~SwarmPlatform() {}
    virtual uint32_t NowMs() = 0;
    virtual uint32_t Random() = 0;
    // WiFi station or Ethernet has an address.
    virtual bool Online() = 0;
    virtual bool GetIdentity(AutoUpdatePlatform::Identity& identity) = 0;
    // One HTTP exchange; request.path is the full URL. False means no response.
    virtual bool Http(const AutoUpdatePlatform::HttpRequest& request, AutoUpdatePlatform::HttpResult& result,
                      AutoUpdatePlatform::BodySink* sink) = 0;
    // Firmware the swarm account staged for this device, or nullptr for none. AUTO_UPDATE decides whether it installs.
    virtual void SetFirmwareTarget(const char* tag, const char* channel) = 0;
};

/**
 * Swarm device protocol v1: one POST {server}/api/swarm/v1/report per period with the range maps and counts. A report
 * that fails stays pending with its seq and start; later periods merge into it until the server takes it.
 */
class SwarmReporter {
   public:
    struct Config {
        const char* product = "adsbee_1090";
        uint32_t period_s = 3600;
        uint32_t jitter_s = 60;  // +- around each period.
        uint32_t first_min_s = 120;
        uint32_t first_max_s = 300;
        uint32_t min_period_s = 60;  // Bounds for the server's next_report_s.
        uint32_t max_period_s = 86400;
        uint32_t retry_min_s = 60;  // Doubles per failure, up to the period.
    };

    // Receiver state at the end of a period.
    struct DeviceState {
        bool auto_update = false;
        bool rx_position_valid = false;
        float latitude_deg = 0;
        float longitude_deg = 0;
        int32_t altitude_m = 0;
        const char* rx_position_source = "";
        bool range_valid = false;  // UsablePosition() held all period, so the range map means something.
    };

    struct Status {
        uint32_t seq = 0;
        uint32_t last_attempt_s = 0;  // Uptime, 0 = none yet.
        uint32_t last_ok_s = 0;
        int last_http_status = 0;
        const char* result = "waiting";
        uint16_t last_body_len = 0;
        bool stopped = false;  // 410: the device was removed from the account.
    };

    enum Result : uint8_t { kOK, kNoAnswer, kNoIdentity, kRejected, kRateLimited, kServerError, kRemoved, kBadRequest };

    // Largest body sent; each report allocates only its own length. Three full range maps with OTA keys: 2,898 Bytes.
    static constexpr uint16_t kBodyMaxLen = 3072;

    SwarmReporter(SwarmPlatform& platform, const char* version, const Config& config, uint32_t first_seq = 0);
    SwarmReporter(SwarmPlatform& platform, const char* version) : SwarmReporter(platform, version, Config()) {}

    // Delay before the first report: 2-5 min after boot, or after uptime_ms if that is later.
    uint32_t FirstDelayMs(uint32_t uptime_ms);

    /**
     * Ends the live period: moves it into the pending report (merging if one is waiting) and clears it. Call from the
     * task that feeds live, never while Send() runs.
     */
    void TakePeriod(SwarmRecorder& live, uint32_t now_ms, const DeviceState& state);

    /**
     * Sends the pending report. Blocks while it works.
     * @retval Milliseconds until the next TakePeriod() and Send().
     */
    uint32_t Send(const char* server, const char* token);

    // Starts a new period at now_ms without reporting the data so far (e.g. while stopped).
    void Restart(SwarmRecorder& live, uint32_t now_ms);

    // A new server or token: undoes a 410 and the old server's backoff, period and protocol list. A pending report is kept.
    void Reconfigure() {
        status_.stopped = false;
        num_failures_ = 0;
        period_s_ = config_.period_s;
        accepted_protocols_ = kSwarmV1ProtocolMask;
    }

    bool has_pending() const { return have_pending_; }
    uint32_t seq() const { return seq_; }
    const Status& status() const { return status_; }
    const SwarmRecorder& pending() const { return pending_; }
    // Bit per kSwarmProtocols entry whose range map the server takes: its last reply's "protocols", else the v1 set.
    uint32_t accepted_protocols() const { return accepted_protocols_; }

    /**
     * Whether a receiver position may go into reports: FIXED or GNSS (positions of received aircraft move with them),
     * and a real coordinate (FIXED at 0, 0 means none was entered).
     */
    static bool UsablePosition(bool available, SettingsManager::RxPosition::PositionSource source, float latitude_deg,
                               float longitude_deg) {
        if (!available || !SettingsManager::RxPosition::MayBeTransmittedAsOwnPosition(source)) return false;
        if (!(latitude_deg >= -90.0f && latitude_deg <= 90.0f && longitude_deg >= -180.0f && longitude_deg <= 180.0f)) {
            return false;  // Also NaN.
        }
        return !(latitude_deg == 0.0f && longitude_deg == 0.0f);
    }

    // True for https://adsbee.aero (any path): the only server that gets the OTA keys.
    static bool IsDefaultServer(const char* server);
    // Standard base64 with padding. dest needs 4 * ceil(len / 3) + 1 Bytes.
    static size_t Base64Encode(const uint8_t* data, size_t len, char* dest);
    // Builds the report JSON. Returns its length, or 0 if it doesn't fit. A null buf only measures the length.
    size_t BuildBody(char* buf, size_t buf_size, const AutoUpdatePlatform::Identity& identity, bool include_ota_keys,
                     uint32_t uptime_s) const;
    static const char* ResultToStr(Result result);

   private:
    class ResponseReader;

    uint32_t Jittered(uint32_t s);
    uint32_t RetryMs(uint32_t retry_after_s);
    Result Finish(Result result, int http_status);

    SwarmPlatform& platform_;
    Config config_;
    char version_[AutoUpdatePlatform::kVersionMaxLen + 1] = {0};

    SwarmRecorder pending_;
    bool have_pending_ = false;
    uint32_t seq_ = 0;
    uint32_t pending_start_s_ = 0;
    uint32_t pending_len_s_ = 0;
    uint32_t live_start_s_ = 0;
    DeviceState state_;
    char rx_source_[24] = {0};

    uint32_t period_s_;
    uint8_t num_failures_ = 0;
    uint32_t accepted_protocols_ = kSwarmV1ProtocolMask;
    Status status_;
};
