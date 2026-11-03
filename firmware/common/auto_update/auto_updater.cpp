#include "auto_updater.hh"

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

uint32_t ReadLE32(const uint8_t* p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

// Same CRC-32 as FirmwareUpdateManager::CalculateCRC32 on the RP2040 (IEEE 802.3).
uint32_t CRC32(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) crc = (crc >> 1) ^ (0xEDB88320 & (0u - (crc & 1)));
    }
    return ~crc;
}

// Appends src to a JSON string value, dropping characters that would need escaping.
void AppendJSONSafe(char* dest, size_t dest_size, const char* src) {
    size_t n = strlen(dest);
    for (; *src && n + 1 < dest_size; src++) {
        if ((unsigned char)*src >= 0x20 && *src != '"' && *src != '\\') dest[n++] = *src;
    }
    dest[n] = '\0';
}

}  // namespace

// Reads {"token": "..."} from the auth response.
class AutoUpdater::TokenReader : public AutoUpdatePlatform::BodySink, public JsonScanner::Handler {
   public:
    bool OnBody(int status, const uint8_t* data, size_t len) override {
        if (status == 200) scanner_.Feed(reinterpret_cast<const char*>(data), len);
        return true;
    }
    void OnValue(const char* key, const char* value, bool is_string, uint8_t depth) override {
        if (depth == 1 && is_string && strcmp(key, "token") == 0) CopyStr(token, sizeof(token), value);
    }
    char token[JsonScanner::kTokenMaxLen + 1] = {0};

   private:
    JsonScanner scanner_{*this};
};

// Reads what the releases listing offers this device: the recommendation, and the .ota asset of the release with the
// recommended tag.
class AutoUpdater::OfferReader : public AutoUpdatePlatform::BodySink, public JsonScanner::Handler {
   public:
    // want_tag: a tag learned from an earlier pass, for a listing that names the recommendation after the releases.
    // force: install want_tag whatever the listing recommends.
    OfferReader(Offer& offer, const char* want_tag, bool force = false)
        : offer_(offer), want_tag_(want_tag), force_(force) {}

    bool OnBody(int status, const uint8_t* data, size_t len) override {
        if (status == 200) scanner_.Feed(reinterpret_cast<const char*>(data), len);
        return true;
    }

    void OnValue(const char* key, const char* value, bool is_string, uint8_t depth) override {
        if (depth == 1) {
            if (strcmp(key, "recommended") == 0 && is_string) {
                CopyStr(offer_.recommended, sizeof(offer_.recommended), value);
            } else if (strcmp(key, "recommended_tag") == 0 && is_string && !force_) {
                CopyStr(offer_.tag, sizeof(offer_.tag), value);
            } else if (strcmp(key, "rollback") == 0) {
                offer_.rollback = strcmp(value, "true") == 0;
            } else if (strcmp(key, "hold") == 0) {
                offer_.hold = strcmp(value, "true") == 0;
            }
        } else if (depth == kReleaseDepth && in_releases_) {
            if (strcmp(key, "tag") == 0) CopyStr(release_tag_, sizeof(release_tag_), value);
        } else if (depth == kAssetDepth && in_releases_) {
            if (strcmp(key, "kind") == 0) {
                asset_is_ota_ = strcmp(value, "ota") == 0;
            } else if (strcmp(key, "url") == 0) {
                CopyStr(asset_url_, sizeof(asset_url_), value);
            } else if (strcmp(key, "sha256") == 0) {
                CopyStr(asset_sha256_, sizeof(asset_sha256_), value);
            } else if (strcmp(key, "size") == 0) {
                asset_size_ = strtoul(value, nullptr, 10);
            }
        }
    }

    void OnOpen(char bracket, const char* key, uint8_t depth) override {
        if (depth == 2 && bracket == '[' && strcmp(key, "releases") == 0) {
            in_releases_ = true;
        } else if (depth == kReleaseDepth && in_releases_) {
            release_tag_[0] = '\0';
            release_has_ota_ = false;
        } else if (depth == kAssetDepth && in_releases_) {
            asset_is_ota_ = false;
            asset_url_[0] = asset_sha256_[0] = '\0';
            asset_size_ = 0;
        }
    }

    void OnClose(char bracket, uint8_t depth) override {
        if (depth == 1) {
            offer_.parsed = true;
        } else if (depth == 2) {
            in_releases_ = false;
        } else if (depth == kAssetDepth && in_releases_ && asset_is_ota_ && !release_has_ota_) {
            release_has_ota_ = true;
            CopyStr(release_url_, sizeof(release_url_), asset_url_);
            CopyStr(release_sha256_, sizeof(release_sha256_), asset_sha256_);
            release_size_ = asset_size_;
        } else if (depth == kReleaseDepth && in_releases_ && release_has_ota_) {
            const char* tag = offer_.tag[0] ? offer_.tag : want_tag_;
            if (tag && tag[0] && strcmp(tag, release_tag_) == 0) {
                CopyStr(offer_.tag, sizeof(offer_.tag), tag);
                offer_.has_asset = true;
                CopyStr(offer_.url, sizeof(offer_.url), release_url_);
                CopyStr(offer_.sha256, sizeof(offer_.sha256), release_sha256_);
                offer_.size = release_size_;
            }
        }
    }

   private:
    // {"releases": [ {release: "assets": [ {asset} ] } ] }
    static constexpr uint8_t kReleaseDepth = 3;
    static constexpr uint8_t kAssetDepth = 5;

    Offer& offer_;
    const char* want_tag_;
    bool force_;
    JsonScanner scanner_{*this};
    bool in_releases_ = false;
    char release_tag_[64] = {0};
    bool release_has_ota_ = false;
    char release_url_[128] = {0};
    char release_sha256_[65] = {0};
    uint32_t release_size_ = 0;
    bool asset_is_ota_ = false;
    char asset_url_[128] = {0};
    char asset_sha256_[65] = {0};
    uint32_t asset_size_ = 0;
};

// Streams the .ota file from the service into the RP2040's other flash partition.
//
// The file is [number of partitions][offset per partition][image per partition], an image being a 20 byte header plus
// the application. The whole file is read in order so its SHA-256 can be checked; only this device's image is written:
// header at flash offset 0, application in 4096 byte writes from offset 4096.
class AutoUpdater::Download : public AutoUpdatePlatform::BodySink {
   public:
    Download(AutoUpdater& updater, const Offer& offer, uint32_t partition)
        : up_(updater), offer_(offer), partition_(partition) {}

    bool wrote_header() const { return wrote_header_; }

    const char* Run() {
        const Config& config = up_.config_;
        AutoUpdatePlatform& platform = up_.platform_;
        uint32_t last_progress_ms = platform.NowMs();
        uint32_t retry_ms = config.download_retry_min_ms;
        while (pos_ < offer_.size) {
            if (!platform.Enabled()) return "turned off";
            if (platform.UserOTAActive()) return "manual update started";

            AutoUpdatePlatform::HttpRequest request;
            request.path = offer_.url;
            request.range_first = static_cast<int32_t>(pos_);
            uint32_t end = pos_ + config.range_len_bytes;
            if (end > offer_.size) end = offer_.size;
            request.range_last = static_cast<int32_t>(end - 1);
            range_left_ = end - pos_;

            uint32_t pos_before = pos_;
            Reply reply = up_.Request(request, this, true);
            if (reason_) return reason_;
            if (pos_ > pos_before) {
                last_progress_ms = platform.NowMs();
                retry_ms = config.download_retry_min_ms;
                up_.progress_percent_ = static_cast<uint8_t>((uint64_t)pos_ * 100 / offer_.size);
                if (reply == kReplyOK) continue;
            } else if (reply == kReplyOK) {
                return up_.last_status_ == 206 ? "empty range" : "no range support";
            }
            if (reply == kReplyRejected) return "file not available";

            // No answer, a server error or a rate limit: wait, then ask again from the last byte received.
            if (platform.NowMs() - last_progress_ms >= config.download_stall_limit_ms) return "download stalled";
            uint32_t wait_ms = retry_ms;
            if (reply == kReplyRateLimited && up_.retry_after_s_ * 1000 > wait_ms) {
                wait_ms = up_.retry_after_s_ * 1000;
                if (wait_ms > config.download_retry_max_ms) wait_ms = config.download_retry_max_ms;
            }
            retry_ms = retry_ms * 2 > config.download_retry_max_ms ? config.download_retry_max_ms : retry_ms * 2;
            platform.SleepMs(wait_ms);
        }
        if (!image_done_) return "no image for this partition";
        char hex[2 * SHA256::kDigestLenBytes + 1];
        sha_.FinalHex(hex);
        if (strcmp(hex, offer_.sha256) != 0) return "sha256 mismatch";
        return nullptr;
    }

    bool OnBody(int status, const uint8_t* data, size_t len) override {
        if (status != 206) return true;  // Not the range that was asked for; Run() sees the status.
        if (len > range_left_) len = range_left_;
        range_left_ -= len;
        while (len > 0 && !reason_) {
            size_t n = Consume(data, len);
            sha_.Update(data, n);
            pos_ += n;
            data += n;
            len -= n;
        }
        return reason_ == nullptr;
    }

   private:
    // Takes the leading bytes of data that belong to one part of the file. Returns how many.
    size_t Consume(const uint8_t* data, size_t len) {
        if (pos_ < table_end_) {
            table_[pos_] = data[0];
            if (pos_ + 1 == 4) {
                uint32_t num_partitions = ReadLE32(table_);
                if (num_partitions < 1 || num_partitions > kOTAMaxNumPartitions || partition_ >= num_partitions) {
                    reason_ = "bad ota file";
                    return 1;
                }
                table_end_ = 4 + 4 * num_partitions;
            } else if (pos_ + 1 == table_end_ && pos_ + 1 > 4) {
                image_start_ = ReadLE32(table_ + 4 + 4 * partition_);
                if (image_start_ < table_end_ || image_start_ + kOTAHeaderLenBytes >= offer_.size) {
                    reason_ = "bad ota file";
                }
            }
            return 1;
        }
        if (pos_ < image_start_) {
            return Min(len, image_start_ - pos_);  // Another partition's image.
        }
        if (pos_ < image_start_ + kOTAHeaderLenBytes) {
            header_[pos_ - image_start_] = data[0];
            if (pos_ + 1 == image_start_ + kOTAHeaderLenBytes) BeginImage();
            return 1;
        }
        if (pos_ < image_end_) {
            size_t n = Min(Min(len, image_end_ - pos_), kFlashChunkLenBytes - chunk_len_);
            memcpy(chunk_ + chunk_len_, data, n);
            chunk_len_ += n;
            bool last = pos_ + n == image_end_;
            if (chunk_len_ == kFlashChunkLenBytes || last) {
                memset(chunk_ + chunk_len_, 0xFF, kFlashChunkLenBytes - chunk_len_);
                if (!up_.FlashWrite(kOTAAppOffsetBytes + num_chunks_written_ * kFlashChunkLenBytes, chunk_,
                                    kFlashChunkLenBytes)) {
                    reason_ = "flash write failed";
                }
                num_chunks_written_++;
                chunk_len_ = 0;
                image_done_ = last && !reason_;
            }
            return n;
        }
        return len;  // Past this partition's image.
    }

    void BeginImage() {
        uint32_t app_len = ReadLE32(header_ + 8);
        if (ReadLE32(header_) != kOTAMagic || app_len == 0 ||
            (uint64_t)image_start_ + kOTAHeaderLenBytes + app_len > offer_.size) {
            reason_ = "bad ota file";
            return;
        }
        image_end_ = image_start_ + kOTAHeaderLenBytes + app_len;
        if (!up_.FlashErase(0, kOTAAppOffsetBytes + app_len + kFlashChunkLenBytes)) {
            reason_ = "flash erase failed";
            return;
        }
        wrote_header_ = true;
        if (!up_.FlashWrite(0, header_, kOTAHeaderLenBytes)) reason_ = "flash write failed";
    }

    static size_t Min(size_t a, size_t b) { return a < b ? a : b; }

    AutoUpdater& up_;
    const Offer& offer_;
    uint32_t partition_;
    SHA256 sha_;
    uint32_t pos_ = 0;  // File bytes received so far, which is also where the next range starts.
    uint32_t range_left_ = 0;
    const char* reason_ = nullptr;

    uint8_t table_[4 + 4 * kOTAMaxNumPartitions] = {0};
    uint32_t table_end_ = 4;
    uint32_t image_start_ = UINT32_MAX;
    uint32_t image_end_ = 0;
    uint8_t header_[kOTAHeaderLenBytes] = {0};
    bool wrote_header_ = false;
    bool image_done_ = false;
    uint32_t num_chunks_written_ = 0;
    uint32_t chunk_len_ = 0;
    uint8_t chunk_[kFlashChunkLenBytes];
};

AutoUpdater::AutoUpdater(AutoUpdatePlatform& platform, const char* running_version, const Config& config)
    : platform_(platform), config_(config) {
    CopyStr(running_version_, sizeof(running_version_), running_version);
    platform_.LoadState(saved_);
    saved_.target[sizeof(saved_.target) - 1] = '\0';
    saved_.pending[sizeof(saved_.pending) - 1] = '\0';
}

const char* AutoUpdater::StateToStr(State state) {
    static const char* const kStrs[kNumStates] = {"idle",   "checking", "downloading", "installing",
                                                  "failed", "held",     "no keys"};
    return state < kNumStates ? kStrs[state] : "idle";
}

int AutoUpdater::CompareVersions(const char* a, const char* b) {
    long va[4], vb[4];
    for (int k = 0; k < 2; k++) {
        const char* s = k == 0 ? a : b;
        long* v = k == 0 ? va : vb;
        char* end = nullptr;
        for (int i = 0; i < 3; i++) {
            v[i] = strtol(s, &end, 10);
            s = *end == '.' ? end + 1 : end;
        }
        v[3] = strncmp(s, "-rc", 3) == 0 ? strtol(s + 3, nullptr, 10) : 999999;  // No suffix: the release itself.
    }
    for (int i = 0; i < 4; i++) {
        if (va[i] != vb[i]) return va[i] < vb[i] ? -1 : 1;
    }
    return 0;
}

void AutoUpdater::SetState(State state, const char* version) {
    state_ = state;
    if (version != version_) CopyStr(version_, sizeof(version_), version);
}

uint32_t AutoUpdater::Jitter(uint32_t max_ms) { return max_ms == 0 ? 0 : platform_.Random() % max_ms; }

uint32_t AutoUpdater::NextCheckMs() { return config_.check_interval_ms + Jitter(config_.check_jitter_ms); }

uint32_t AutoUpdater::ErrorRetryMs() {
    uint32_t ms = config_.error_retry_min_ms;
    for (uint8_t i = 1; i < num_errors_ && ms < config_.error_retry_max_ms; i++) ms *= 2;
    if (ms > config_.error_retry_max_ms) ms = config_.error_retry_max_ms;
    if (retry_after_s_ * 1000 > ms) ms = retry_after_s_ * 1000;
    if (ms > config_.check_interval_ms) ms = config_.check_interval_ms;
    return ms + Jitter(ms / 4);
}

uint32_t AutoUpdater::FirstDelayMs(uint32_t uptime_ms) {
    static constexpr uint32_t kMinDelayMs = 30 * 1000;
    uint32_t delay_ms = config_.first_check_delay_ms > uptime_ms ? config_.first_check_delay_ms - uptime_ms : 0;
    // An install is waiting to be reported: don't keep the service waiting.
    if (saved_.pending[0] && delay_ms > 2 * kMinDelayMs) delay_ms = 2 * kMinDelayMs;
    if (delay_ms < kMinDelayMs && config_.first_check_delay_ms >= kMinDelayMs) delay_ms = kMinDelayMs;
    return delay_ms + Jitter(saved_.pending[0] ? kMinDelayMs : config_.first_check_jitter_ms);
}

AutoUpdater::Reply AutoUpdater::Request(const AutoUpdatePlatform::HttpRequest& request_in,
                                        AutoUpdatePlatform::BodySink* sink, bool authenticated) {
    AutoUpdatePlatform::HttpRequest request = request_in;
    for (int attempt = 0; attempt < 2; attempt++) {
        if (authenticated) {
            if (token_[0] == '\0' || platform_.NowMs() - token_timestamp_ms_ >= config_.token_lifetime_ms) {
                Reply reply = Authenticate();
                if (reply != kReplyOK) return reply;
            }
            request.token = token_;
        }
        AutoUpdatePlatform::HttpResult result;
        last_status_ = 0;
        retry_after_s_ = 0;
        if (!platform_.Http(request, result, sink)) return kReplyNoAnswer;
        last_status_ = result.status;
        retry_after_s_ = result.retry_after_s;
        if (result.status >= 200 && result.status < 300) return kReplyOK;
        if (result.status == 429) return kReplyRateLimited;
        if (result.status == 401 && authenticated && attempt == 0) {
            token_[0] = '\0';  // Expired or revoked: authenticate again, once.
            continue;
        }
        return result.status >= 400 && result.status < 500 ? kReplyRejected : kReplyError;
    }
    return kReplyRejected;
}

AutoUpdater::Reply AutoUpdater::Authenticate() {
    if (!have_identity_) {
        have_identity_ = platform_.GetIdentity(identity_);
        if (!have_identity_) return kReplyNoAnswer;
    }
    // About 520 characters with full length keys.
    static constexpr size_t kBodyMaxLen = 640;
    char* body = new (std::nothrow) char[kBodyMaxLen];
    if (!body) return kReplyNoAnswer;
    body[0] = '\0';
    auto add = [&](const char* text, const char* value) {
        size_t n = strlen(body);
        snprintf(body + n, kBodyMaxLen - n, "%s", text);
        AppendJSONSafe(body, kBodyMaxLen, value);
    };
    add("{\"client\":\"esp32_auto\",\"product\":\"", config_.product);
    add("\",\"unique_id\":\"", identity_.unique_id);
    add("\",\"mac_address\":\"", identity_.mac_address);
    add("\",\"part_code\":\"", identity_.part_code);
    add("\",\"ota_keys\":[\"", identity_.ota_keys[0]);
    add("\",\"", identity_.ota_keys[1]);
    add("\"],\"versions\":{\"rp2040\":\"", running_version_);
    add("\",\"esp32\":\"", running_version_);
    add("\"}}", "");

    TokenReader* reader = new (std::nothrow) TokenReader();
    Reply reply = kReplyNoAnswer;
    if (reader) {
        AutoUpdatePlatform::HttpRequest request;
        request.method = "POST";
        request.path = "auth";
        request.body = body;
        reply = Request(request, reader, false);
        if (reply == kReplyOK) {
            if (reader->token[0] && strlen(reader->token) <= kTokenMaxLen) {
                CopyStr(token_, sizeof(token_), reader->token);
                token_timestamp_ms_ = platform_.NowMs();
            } else {
                reply = kReplyError;
            }
        }
        delete reader;
    }
    delete[] body;
    return reply;
}

void AutoUpdater::SetTarget(const char* version_or_tag) {
    const char* version = version_or_tag ? version_or_tag : "";
    size_t product_len = strlen(config_.product);
    if (strncmp(version, config_.product, product_len) == 0 && version[product_len] == '-') {
        version += product_len + 1;  // A release tag: adsbee_1090-0.9.2.
    }
    CopyStr(target_, sizeof(target_), version);
}

AutoUpdater::Reply AutoUpdater::GetOffer(Offer& offer) {
    if (target_[0]) {
        // The rc listing has every release the service keeps, so a staged RC is found too.
        char tag[sizeof(offer.tag)];
        snprintf(tag, sizeof(tag), "%s-%s", config_.product, target_);
        offer = Offer();
        OfferReader* reader = new (std::nothrow) OfferReader(offer, tag, true);
        if (!reader) return kReplyNoAnswer;
        AutoUpdatePlatform::HttpRequest request;
        request.path = "releases?channel=rc";
        Reply reply = Request(request, reader, true);
        delete reader;
        if (reply != kReplyOK) return reply;
        if (!offer.parsed) return kReplyError;
        CopyStr(offer.recommended, sizeof(offer.recommended), target_);
        offer.rollback = true;  // The swarm owner chose this version, older or not.
        return kReplyOK;
    }
    char path[48];
    snprintf(path, sizeof(path), "releases?channel=%s", config_.channel);
    char tag[sizeof(offer.tag)] = {0};
    // A second pass only when the listing named the recommended tag after the release that carries it.
    for (int pass = 0; pass < 2; pass++) {
        offer = Offer();
        OfferReader* reader = new (std::nothrow) OfferReader(offer, tag);
        if (!reader) return kReplyNoAnswer;
        AutoUpdatePlatform::HttpRequest request;
        request.path = path;
        Reply reply = Request(request, reader, true);
        delete reader;
        if (reply != kReplyOK) return reply;
        if (!offer.parsed) return kReplyError;
        if (offer.has_asset || !offer.recommended[0]) break;
        if (offer.tag[0]) {
            CopyStr(tag, sizeof(tag), offer.tag);
        } else {
            snprintf(tag, sizeof(tag), "%s-%s", config_.product, offer.recommended);  // How release tags are named.
        }
    }
    return kReplyOK;
}

void AutoUpdater::Checkin(const char* state, const char* version, const char* detail) {
    char body[96 + AutoUpdatePlatform::kVersionMaxLen + kDetailMaxLen];
    snprintf(body, sizeof(body), "{\"state\":\"%s\",\"version\":\"", state);
    AppendJSONSafe(body, sizeof(body), version);
    size_t n = strlen(body);
    snprintf(body + n, sizeof(body) - n, "\",\"detail\":\"");
    AppendJSONSafe(body, sizeof(body) - 2, detail);
    n = strlen(body);
    snprintf(body + n, sizeof(body) - n, "\"}");
    AutoUpdatePlatform::HttpRequest request;
    request.method = "POST";
    request.path = "checkin";
    request.body = body;
    Request(request, nullptr, true);  // Best effort: an update never depends on a check-in.
}

bool AutoUpdater::WaitLine(const char* want, uint32_t timeout_ms, char* match, size_t match_size) {
    char line[160];
    uint32_t start_ms = platform_.NowMs();
    size_t want_len = strlen(want);
    for (;;) {
        uint32_t elapsed_ms = platform_.NowMs() - start_ms;
        if (elapsed_ms >= timeout_ms || !platform_.ConsoleReadLine(line, sizeof(line), timeout_ms - elapsed_ms)) {
            return false;
        }
        char* text = line;
        while (*text == ' ') text++;
        size_t len = strlen(text);
        while (len > 0 && (text[len - 1] == '\r' || text[len - 1] == ' ')) text[--len] = '\0';
        // With match, want is a prefix and the whole line is returned. Otherwise the line must equal want: echoed
        // payload and log lines can contain "OK".
        if (match ? strncmp(text, want, want_len) == 0 : strcmp(text, want) == 0) {
            if (match) CopyStr(match, match_size, text);
            return true;
        }
        if (strncmp(text, "ERROR", 5) == 0) return false;
    }
}

bool AutoUpdater::ConsoleCommand(const char* command, const char* want, uint32_t timeout_ms, char* match,
                                 size_t match_size) {
    char line[96];
    int len = snprintf(line, sizeof(line), "%s\r\n", command);
    platform_.ConsoleSend(reinterpret_cast<const uint8_t*>(line), static_cast<size_t>(len));
    return WaitLine(want, timeout_ms, match, match_size);
}

void AutoUpdater::DrainConsole() {
    char line[160];
    uint32_t start_ms = platform_.NowMs();
    while (platform_.NowMs() - start_ms < config_.drain_max_ms &&
           platform_.ConsoleReadLine(line, sizeof(line), config_.drain_quiet_ms)) {
    }
}

bool AutoUpdater::FlashErase(uint32_t offset, uint32_t len) {
    uint32_t step = config_.erase_step_bytes / kFlashChunkLenBytes * kFlashChunkLenBytes;
    if (step == 0) step = kFlashChunkLenBytes;
    for (uint32_t done = 0; done < len; done += step) {
        uint32_t n = len - done < step ? len - done : step;
        char command[48];
        snprintf(command, sizeof(command), "AT+OTA=ERASE,%lx,%lu", (unsigned long)(offset + done), (unsigned long)n);
        if (!ConsoleCommand(command, "OK", config_.erase_timeout_ms)) return false;
    }
    return true;
}

bool AutoUpdater::FlashWrite(uint32_t offset, const uint8_t* data, uint32_t len) {
    char command[64];
    snprintf(command, sizeof(command), "AT+OTA=WRITE,%lx,%lu,%lx", (unsigned long)offset, (unsigned long)len,
             (unsigned long)CRC32(data, len));
    for (uint8_t attempt = 0; attempt < config_.max_attempts_per_write; attempt++) {
        if (attempt > 0) {
            // The RP2040 may still be reading (or have misread) the last payload: let it finish, then start clean.
            DrainConsole();
            if (!FlashErase(offset, len)) continue;
        }
        if (!ConsoleCommand(command, "READY", config_.ready_timeout_ms)) continue;
        platform_.ConsoleSend(data, len);
        if (WaitLine("OK", config_.write_timeout_ms)) return true;
    }
    return false;
}

const char* AutoUpdater::Install(const Offer& offer) {
    char protocol[24] = {0};
    CopyStr(protocol, sizeof(protocol), platform_.ConsoleReportingProtocol());
    DrainConsole();
    if (protocol[0] && !ConsoleCommand("AT+PROTOCOL_OUT=CONSOLE,NONE", "OK", config_.cmd_timeout_ms)) {
        return "console busy";
    }

    const char* reason = nullptr;
    char line[32];
    bool wrote_header = false;
    if (!ConsoleCommand("AT+OTA=GET_PARTITION", "Partition: ", config_.cmd_timeout_ms, line, sizeof(line))) {
        reason = "no partition reply";
    } else {
        WaitLine("OK", config_.cmd_timeout_ms);
        Download* download = new (std::nothrow) Download(*this, offer, strtoul(line + strlen("Partition: "), nullptr, 10));
        if (!download) {
            reason = "out of memory";
        } else {
            reason = download->Run();
            wrote_header = download->wrote_header();
            delete download;
        }
    }
    if (!reason && !ConsoleCommand("AT+OTA=VERIFY", "OK", config_.verify_timeout_ms)) reason = "verify failed";
    if (reason) {
        // Leave nothing that looks like an image, and put the console back.
        if (wrote_header) FlashErase(0, kOTAAppOffsetBytes);
        if (protocol[0]) {
            char command[64];
            snprintf(command, sizeof(command), "AT+PROTOCOL_OUT=CONSOLE,%s", protocol);
            ConsoleCommand(command, "OK", config_.cmd_timeout_ms);
        }
    }
    return reason;
}

uint32_t AutoUpdater::TargetWaitMs() {
    uint32_t now_ms = platform_.NowMs();
    return target_pending_ && now_ms < config_.target_wait_ms ? config_.target_wait_ms - now_ms : 0;
}

uint32_t AutoUpdater::RunOnce() {
    if (uint32_t wait_ms = TargetWaitMs()) return wait_ms;
    if (!platform_.Online()) return config_.offline_retry_ms;

    uint32_t now_ms = platform_.NowMs();
    if (now_ms - window_start_ms_ >= config_.check_interval_ms) {
        window_start_ms_ = now_ms;
        if (saved_.attempts != 0) {
            saved_.attempts = 0;
            platform_.SaveState(saved_);
        }
    }

    State state_before = state_;
    if (state_ != kFailed) SetState(kChecking, version_);

    Offer offer;
    Reply reply = GetOffer(offer);
    if (reply != kReplyOK) {
        if (reply == kReplyRejected && token_[0] == '\0') {
            // The service does not accept this device's keys. Asking more often won't change that.
            SetState(kNoKeys);
            num_errors_ = 0;
            return NextCheckMs();
        }
        if (num_errors_ < 255) num_errors_++;
        if (state_ == kChecking) SetState(state_before == kChecking ? kIdle : state_before, version_);
        return ErrorRetryMs();
    }
    num_errors_ = 0;

    // The outcome of an install that ended in a reboot.
    if (saved_.pending[0]) {
        if (strcmp(saved_.pending, running_version_) == 0) {
            Checkin("installed", running_version_, "");
            saved_.attempts = 0;
            saved_.target[0] = '\0';
        } else {
            char detail[kDetailMaxLen + 1];
            snprintf(detail, sizeof(detail), "running %s after reboot", running_version_);
            Checkin("failed", saved_.pending, detail);
        }
        saved_.pending[0] = '\0';
        platform_.SaveState(saved_);
    }

    if (offer.hold) {
        SetState(kHeld);
        return NextCheckMs();
    }
    if (target_[0] && !offer.has_asset && strcmp(target_, running_version_) != 0) {
        // Never falls back to the recommendation: the swarm asked for this version.
        CopyStr(detail_, sizeof(detail_), "target not offered");
        SetState(kFailed, target_);
        return NextCheckMs();
    }
    int newer = offer.recommended[0] ? CompareVersions(offer.recommended, running_version_) : 0;
    if (newer == 0 || (newer < 0 && !offer.rollback)) {
        SetState(kIdle);
        detail_[0] = '\0';
        return NextCheckMs();
    }

    if (strcmp(saved_.target, offer.recommended) != 0) {
        CopyStr(saved_.target, sizeof(saved_.target), offer.recommended);
        saved_.attempts = 0;
    }
    if (saved_.attempts >= config_.max_attempts) {
        SetState(kFailed, offer.recommended);
        return NextCheckMs();
    }
    if (platform_.UserOTAActive()) {
        SetState(kIdle);
        return config_.busy_retry_ms;
    }

    saved_.attempts++;
    platform_.SaveState(saved_);  // Before the work, so a crash or power cut in the middle still counts.
    SetState(kDownloading, offer.recommended);
    progress_percent_ = 0;
    const char* reason = offer.has_asset && offer.size > 0 ? nullptr : "no ota file in the listing";
    if (!reason) {
        Checkin("downloading", offer.recommended, "");
        reason = Install(offer);
    }
    if (reason) {
        CopyStr(detail_, sizeof(detail_), reason);
        Checkin("failed", offer.recommended, reason);
        SetState(kFailed, offer.recommended);
        if (saved_.attempts >= config_.max_attempts) return NextCheckMs();
        uint32_t wait_ms = config_.failure_retry_ms[saved_.attempts >= 2 ? 1 : 0];
        return wait_ms + Jitter(wait_ms / 4);
    }

    // Verified. Record it first: the RP2040 resets the ESP32 as it reboots, and the next boot reports the result.
    CopyStr(saved_.pending, sizeof(saved_.pending), offer.recommended);
    platform_.SaveState(saved_);
    SetState(kInstalling, offer.recommended);
    progress_percent_ = 100;
    detail_[0] = '\0';
    static const char kBoot[] = "AT+OTA=BOOT\r\n";
    platform_.ConsoleSend(reinterpret_cast<const uint8_t*>(kBoot), sizeof(kBoot) - 1);
    platform_.SleepMs(config_.reboot_wait_ms);

    // Still here: the RP2040 did not reboot.
    saved_.pending[0] = '\0';
    platform_.SaveState(saved_);
    CopyStr(detail_, sizeof(detail_), "no reboot");
    Checkin("failed", offer.recommended, detail_);
    SetState(kFailed, offer.recommended);
    return saved_.attempts >= config_.max_attempts ? NextCheckMs() : config_.failure_retry_ms[1];
}
