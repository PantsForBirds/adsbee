#include "swarm_reporter.hh"

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>

#include "json_scanner.hh"

namespace {

void CopyStr(char* dest, size_t dest_size, const char* src) {
    strncpy(dest, src ? src : "", dest_size - 1);
    dest[dest_size - 1] = '\0';
}

// Bounded JSON text writer. len() is 0 once something didn't fit. With a null buffer it only counts.
class Writer {
   public:
    Writer(char* buf, size_t size) : buf_(buf), size_(size) {
        if (buf_) buf_[0] = '\0';
    }
    void Printf(const char* format, ...) __attribute__((format(printf, 2, 3))) {
        if (!ok_) return;
        va_list args;
        va_start(args, format);
        int n = buf_ ? vsnprintf(buf_ + len_, size_ - len_, format, args) : vsnprintf(nullptr, 0, format, args);
        va_end(args);
        if (n < 0 || (buf_ && static_cast<size_t>(n) >= size_ - len_)) {
            ok_ = false;
            return;
        }
        len_ += n;
    }
    void Safe(const char* text) {
        for (; ok_ && *text; text++) {
            if ((unsigned char)*text < 0x20 || *text == '"' || *text == '\\') continue;
            if (!buf_) {
                len_++;
                continue;
            }
            if (len_ + 1 >= size_) {
                ok_ = false;
                return;
            }
            buf_[len_++] = *text;
        }
        if (buf_) buf_[len_] = '\0';
    }
    void Base64(const uint8_t* data, size_t n) {
        size_t need = 4 * ((n + 2) / 3);
        if (!ok_ || (buf_ && len_ + need >= size_)) {
            ok_ = false;
            return;
        }
        len_ += buf_ ? SwarmReporter::Base64Encode(data, n, buf_ + len_) : need;
    }
    size_t len() const { return ok_ ? len_ : 0; }

   private:
    char* buf_;
    size_t size_;
    size_t len_ = 0;
    bool ok_ = true;
};

}  // namespace

// Reads {"next_report_s": N, "firmware": {"tag": "..", "channel": ".."} | null, "protocols": ["mode_s", ..]}.
class SwarmReporter::ResponseReader : public AutoUpdatePlatform::BodySink, public JsonScanner::Handler {
   public:
    bool OnBody(int status, const uint8_t* data, size_t len) override {
        if (status == 200) scanner_.Feed(reinterpret_cast<const char*>(data), len);
        return true;
    }
    void OnValue(const char* key, const char* value, bool is_string, uint8_t depth) override {
        if (depth == 1 && strcmp(key, "next_report_s") == 0 && !is_string) {
            next_report_s = strtoul(value, nullptr, 10);
        } else if (depth == 2 && in_protocols_ && is_string) {
            uint8_t p = SwarmProtocolIndex(value);
            if (p != kNoSwarmProtocol) protocols |= 1u << p;  // Names this firmware doesn't know are skipped.
        } else if (depth == 2 && in_firmware_ && is_string) {
            if (strcmp(key, "tag") == 0) CopyStr(tag, sizeof(tag), value);
            if (strcmp(key, "channel") == 0) CopyStr(channel, sizeof(channel), value);
        }
    }
    void OnOpen(char bracket, const char* key, uint8_t depth) override {
        if (depth != 2) return;
        in_firmware_ = bracket == '{' && strcmp(key, "firmware") == 0;
        in_protocols_ = bracket == '[' && strcmp(key, "protocols") == 0;
        if (in_protocols_) {
            has_protocols = true;
            protocols = 0;
        }
    }
    void OnClose(char bracket, uint8_t depth) override {
        if (depth == 2) in_firmware_ = in_protocols_ = false;
    }
    bool complete() const { return scanner_.complete(); }

    uint32_t next_report_s = 0;
    char tag[AutoUpdatePlatform::kVersionMaxLen + 16] = {0};
    char channel[16] = {0};
    bool has_protocols = false;
    uint32_t protocols = 0;

   private:
    JsonScanner scanner_{*this};
    bool in_firmware_ = false;
    bool in_protocols_ = false;
};

SwarmReporter::SwarmReporter(SwarmPlatform& platform, const char* version, const Config& config, uint32_t first_seq)
    : platform_(platform), config_(config), seq_(first_seq), period_s_(config.period_s) {
    CopyStr(version_, sizeof(version_), version);
    status_.seq = seq_;
}

const char* SwarmReporter::ResultToStr(Result result) {
    static const char* const kStrs[] = {"ok",       "no answer",    "no identity", "rejected",
                                        "slow down", "server error", "removed",     "bad request"};
    return result < sizeof(kStrs) / sizeof(kStrs[0]) ? kStrs[result] : "?";
}

bool SwarmReporter::IsDefaultServer(const char* server) {
    static const char kScheme[] = "https://";
    static const char kHost[] = "adsbee.aero";
    if (strncasecmp(server, kScheme, sizeof(kScheme) - 1) != 0) return false;
    const char* host = server + sizeof(kScheme) - 1;
    size_t host_len = strcspn(host, "/:?#");
    if (host[host_len] == ':' && strncmp(host + host_len, ":443", 4) != 0) return false;
    return host_len == sizeof(kHost) - 1 && strncasecmp(host, kHost, host_len) == 0;
}

size_t SwarmReporter::Base64Encode(const uint8_t* data, size_t len, char* dest) {
    static const char kAlphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t n = 0;
    for (size_t i = 0; i < len; i += 3) {
        uint32_t v = static_cast<uint32_t>(data[i]) << 16;
        if (i + 1 < len) v |= static_cast<uint32_t>(data[i + 1]) << 8;
        if (i + 2 < len) v |= data[i + 2];
        dest[n++] = kAlphabet[(v >> 18) & 0x3F];
        dest[n++] = kAlphabet[(v >> 12) & 0x3F];
        dest[n++] = i + 1 < len ? kAlphabet[(v >> 6) & 0x3F] : '=';
        dest[n++] = i + 2 < len ? kAlphabet[v & 0x3F] : '=';
    }
    dest[n] = '\0';
    return n;
}

uint32_t SwarmReporter::Jittered(uint32_t s) {
    uint32_t jitter_s = config_.jitter_s < s ? config_.jitter_s : s / 2;
    uint32_t span_ms = 2 * jitter_s * 1000 + 1;
    return s * 1000 - jitter_s * 1000 + platform_.Random() % span_ms;
}

uint32_t SwarmReporter::FirstDelayMs(uint32_t uptime_ms) {
    uint32_t span_ms = (config_.first_max_s - config_.first_min_s) * 1000 + 1;
    uint32_t at_ms = config_.first_min_s * 1000 + platform_.Random() % span_ms;
    // Turned on later than that: the same delay from now, so the first period isn't empty.
    return at_ms > uptime_ms ? at_ms - uptime_ms : at_ms;
}

uint32_t SwarmReporter::RetryMs(uint32_t retry_after_s) {
    uint32_t s = config_.retry_min_s;
    for (uint8_t i = 1; i < num_failures_ && s < period_s_; i++) s *= 2;
    if (s > period_s_) s = period_s_;
    if (retry_after_s > s) s = retry_after_s;
    return s * 1000 + platform_.Random() % (s * 250 + 1);  // Up to +25 %.
}

void SwarmReporter::Restart(SwarmRecorder& live, uint32_t now_ms) {
    live.Clear();
    live_start_s_ = now_ms / 1000;
}

void SwarmReporter::TakePeriod(SwarmRecorder& live, uint32_t now_ms, const DeviceState& state) {
    bool range_valid = state.range_valid;
    if (!have_pending_) {
        pending_ = static_cast<SwarmRecorder&&>(live);  // Takes live's heap maps; Restart() clears live.
        pending_start_s_ = live_start_s_;
        have_pending_ = true;
    } else {
        pending_.Merge(live);  // A retry: same seq and start, longer period.
        range_valid = range_valid && state_.range_valid;  // Only if every merged period had a fixed or GNSS position.
    }
    pending_len_s_ = now_ms / 1000 - pending_start_s_;
    state_ = state;
    state_.range_valid = range_valid;
    CopyStr(rx_source_, sizeof(rx_source_), state.rx_position_source);
    state_.rx_position_source = rx_source_;
    Restart(live, now_ms);
}

size_t SwarmReporter::BuildBody(char* buf, size_t buf_size, const AutoUpdatePlatform::Identity& identity,
                                bool include_ota_keys, uint32_t uptime_s) const {
    Writer w(buf, buf_size);
    w.Printf("{\"v\":1,\"product\":\"");
    w.Safe(config_.product);
    w.Printf("\",\"unique_id\":\"");
    w.Safe(identity.unique_id);
    w.Printf("\",\"mac_address\":\"");
    w.Safe(identity.mac_address);
    w.Printf("\",");
    if (include_ota_keys) {
        w.Printf("\"ota_keys\":[\"");
        w.Safe(identity.ota_keys[0]);
        w.Printf("\",\"");
        w.Safe(identity.ota_keys[1]);
        w.Printf("\"],");
    }
    w.Printf("\"versions\":{\"rp2040\":\"");
    w.Safe(version_);
    w.Printf("\",\"esp32\":\"");
    w.Safe(version_);
    w.Printf("\"},\"auto_update\":%s,\"uptime_s\":%lu,\"seq\":%lu,", state_.auto_update ? "true" : "false",
             (unsigned long)uptime_s, (unsigned long)seq_);
    w.Printf("\"period\":{\"start_uptime_s\":%lu,\"len_s\":%lu},", (unsigned long)pending_start_s_,
             (unsigned long)pending_len_s_);
    if (state_.rx_position_valid) {
        w.Printf("\"rx_position\":{\"lat\":%.5f,\"lon\":%.5f,\"alt_m\":%ld,\"source\":\"", state_.latitude_deg,
                 state_.longitude_deg, (long)state_.altitude_m);
        w.Safe(state_.rx_position_source);
        w.Printf("\"},");
    }
    // Without listen time (it was never sampled) every v1 stats key goes, as in v1. With it, only protocols listened
    // to or heard: "listen_s" seconds, v1 "stats" keys, "frames" for the rest (Mode S's one total included).
    const bool listen_known = pending_.listen_known;
    bool first = true;
    if (listen_known) {
        w.Printf("\"listen_s\":{");
        for (uint8_t p = 0; p < kNumSwarmProtocols; p++) {
            if (pending_.listen_s[p] == 0) continue;
            w.Printf("%s\"%s\":%lu", first ? "" : ",", kSwarmProtocols[p].name, (unsigned long)pending_.listen_s[p]);
            first = false;
        }
        w.Printf("},");
    }
    w.Printf("\"stats\":{");
    for (uint8_t p = 0; p < kNumSwarmProtocols; p++) {
        if (listen_known && !pending_.Reported(p)) continue;
        for (const SwarmProtocol::V1Stat& v1 : kSwarmProtocols[p].v1_stats) {
            if (v1.key) w.Printf("\"%s\":%lu,", v1.key, (unsigned long)(pending_.stats.*v1.field));
        }
    }
    w.Printf("\"aircraft_max\":%u,\"aircraft_unique\":%u}", (unsigned)pending_.stats.aircraft_max,
             (unsigned)pending_.UniqueAircraft());
    // "frames": protocols counted without v1 keys. Without listen time, the v1 ones (Mode S) always, as v1 sent them.
    first = true;
    for (uint8_t p = 0; p < kNumSwarmProtocols; p++) {
        const SwarmProtocol& info = kSwarmProtocols[p];
        if (info.v1_stats[0].key || !info.counts_frames) continue;
        if (listen_known ? !pending_.Reported(p) : !(kSwarmV1ProtocolMask & (1u << p))) continue;
        w.Printf("%s\"%s\":%lu", first ? ",\"frames\":{" : ",", info.name, (unsigned long)pending_.frames[p]);
        first = false;
    }
    if (!first) w.Printf("}");
    // Range only from a fixed or GNSS position that every merged period had, and not when too much was dropped.
    if (!state_.rx_position_valid || !state_.range_valid) {
        w.Printf(",\"range_withheld\":\"no_position\"");
    } else if (pending_.RangeImplausible()) {
        w.Printf(",\"range_withheld\":\"implausible\",\"range_dropped\":%lu",
                 (unsigned long)(pending_.guard.num_dropped + pending_.guard.num_no_map));
    } else {
        w.Printf(",\"range_dropped\":%lu,\"ranges\":{",
                 (unsigned long)(pending_.guard.num_dropped + pending_.guard.num_no_map));
        // One map per protocol with data that the server takes; none at all gives an empty object.
        uint8_t bitmap[RangeMap::kBitmapLenBytes];
        uint8_t values[RangeMap::kNumBins];
        first = true;
        for (uint8_t p = 0; p < kNumSwarmProtocols; p++) {
            const RangeMap* map = pending_.ranges.Find(p);
            if (map == nullptr || !(accepted_protocols_ & (1u << p))) continue;
            uint16_t num_values = map->Encode(bitmap, values);
            if (num_values == 0) continue;
            w.Printf("%s\"%s\":{\"sectors\":%u,\"bands\":%u,\"unit_m\":%lu,\"changed\":\"", first ? "" : ",",
                     kSwarmProtocols[p].name, RangeMap::kNumSectors, RangeMap::kNumBands,
                     (unsigned long)kSwarmProtocols[p].unit_m);
            first = false;
            w.Base64(bitmap, sizeof(bitmap));
            w.Printf("\",\"max\":\"");
            w.Base64(values, num_values);
            w.Printf("\"}");
        }
        w.Printf("}");
    }
    w.Printf("}");
    return w.len();
}

SwarmReporter::Result SwarmReporter::Finish(Result result, int http_status) {
    status_.result = ResultToStr(result);
    status_.last_http_status = http_status;
    status_.seq = seq_;
    return result;
}

uint32_t SwarmReporter::Send(const char* server, const char* token) {
    status_.last_attempt_s = platform_.NowMs() / 1000;
    if (status_.stopped) {
        have_pending_ = false;  // Nobody takes it.
        Finish(kRemoved, 410);
        return period_s_ * 1000;
    }
    if (!have_pending_) return Jittered(period_s_);
    if (!platform_.Online()) {
        if (num_failures_ < 255) num_failures_++;
        Finish(kNoAnswer, 0);
        return RetryMs(0);
    }

    AutoUpdatePlatform::Identity* identity = new (std::nothrow) AutoUpdatePlatform::Identity();
    char* body = nullptr;
    ResponseReader* reader = new (std::nothrow) ResponseReader();
    char* url = new (std::nothrow) char[strlen(server) + 32];
    Result result = kNoIdentity;
    AutoUpdatePlatform::HttpResult http;
    if (identity && reader && url && platform_.GetIdentity(*identity)) {
        // Counts first, then takes only what this report needs (often under 1 kB): keeps heap free for Remote ID.
        const bool ota_keys = IsDefaultServer(server);
        const uint32_t uptime_s = platform_.NowMs() / 1000;
        size_t body_size = BuildBody(nullptr, 0, *identity, ota_keys, uptime_s) + 1;
        if (body_size > kBodyMaxLen) body_size = kBodyMaxLen;
        body = new (std::nothrow) char[body_size];
        size_t len = body ? BuildBody(body, body_size, *identity, ota_keys, uptime_s) : 0;
        status_.last_body_len = static_cast<uint16_t>(len);
        size_t server_len = strlen(server);
        while (server_len > 0 && server[server_len - 1] == '/') server_len--;
        snprintf(url, server_len + 32, "%.*s/api/swarm/v1/report", (int)server_len, server);
        AutoUpdatePlatform::HttpRequest request;
        request.method = "POST";
        request.path = url;
        request.token = token;
        request.body = body;
        if (!body) {
            result = kNoIdentity;  // Out of RAM: kept for the next try, like a failed identity.
        } else if (len == 0) {
            result = kBadRequest;
        } else if (!platform_.Http(request, http, reader)) {
            result = kNoAnswer;
        } else if (http.status == 200) {
            result = kOK;
        } else if (http.status == 410) {
            result = kRemoved;
        } else if (http.status == 429) {
            result = kRateLimited;
        } else if (http.status == 400) {
            result = kBadRequest;
        } else if (http.status >= 400 && http.status < 500) {
            result = kRejected;  // 401 bad token, 403 key check failed.
        } else {
            result = kServerError;
        }
    }
    delete identity;

    uint32_t next_ms;
    switch (result) {
        case kOK: {
            uint32_t s = reader->next_report_s ? reader->next_report_s : config_.period_s;
            period_s_ = s < config_.min_period_s ? config_.min_period_s : s > config_.max_period_s ? config_.max_period_s : s;
            platform_.SetFirmwareTarget(reader->tag[0] ? reader->tag : nullptr, reader->channel);
            accepted_protocols_ = reader->has_protocols ? reader->protocols : kSwarmV1ProtocolMask;
            have_pending_ = false;
            seq_++;
            num_failures_ = 0;
            status_.last_ok_s = status_.last_attempt_s;
            next_ms = Jittered(period_s_);
            break;
        }
        case kRemoved:
            status_.stopped = true;
            have_pending_ = false;
            next_ms = period_s_ * 1000;
            break;
        case kBadRequest:
            // The server will never take this report: drop it and start a new one.
            have_pending_ = false;
            seq_++;
            next_ms = Jittered(period_s_);
            break;
        case kRejected:
            num_failures_ = 0;
            next_ms = Jittered(period_s_);  // Waits for a settings change; keeps merging meanwhile.
            break;
        default:
            if (num_failures_ < 255) num_failures_++;
            // 429, and 5xx such as 503 while the server can't check keys: Retry-After when given, never a stop.
            next_ms = RetryMs(result == kRateLimited || result == kServerError ? http.retry_after_s : 0);
            break;
    }
    delete reader;
    delete[] body;
    delete[] url;
    Finish(result, http.status);
    return next_ms;
}
