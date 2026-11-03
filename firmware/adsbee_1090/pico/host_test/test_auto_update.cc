#include <cstdio>
#include <cstring>
#include <deque>
#include <map>
#include <string>
#include <vector>

#include "auto_updater.hh"
#include "gtest/gtest.h"
#include "json_scanner.hh"
#include "sha256.hh"

namespace {

std::string Sha256Hex(const std::string& data) {
    SHA256 sha;
    sha.Update(reinterpret_cast<const uint8_t*>(data.data()), data.size());
    char hex[65];
    sha.FinalHex(hex);
    return hex;
}

uint32_t Crc32(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) crc = (crc >> 1) ^ (0xEDB88320 & (0u - (crc & 1)));
    }
    return ~crc;
}

void PutLE32(std::string& s, uint32_t v) {
    for (int i = 0; i < 4; i++) s.push_back(static_cast<char>(v >> (8 * i)));
}

// An .ota file with two partition images of app_len bytes each (different contents).
std::string MakeOTA(uint32_t app_len) {
    std::string file;
    PutLE32(file, 2);
    PutLE32(file, 12);
    PutLE32(file, 12 + 20 + app_len);
    for (int partition = 0; partition < 2; partition++) {
        std::string app(app_len, '\0');
        for (uint32_t i = 0; i < app_len; i++) app[i] = static_cast<char>((i * 31 + partition * 7 + (i >> 8)) & 0xFF);
        PutLE32(file, AutoUpdater::kOTAMagic);
        PutLE32(file, 0);
        PutLE32(file, app_len);
        PutLE32(file, Crc32(reinterpret_cast<const uint8_t*>(app.data()), app.size()));
        PutLE32(file, 0xFFFFFFFF);
        file += app;
    }
    return file;
}

// The firmware service and the RP2040 console, as the updater sees them.
class FakePlatform : public AutoUpdatePlatform {
   public:
    // Service.
    std::string recommended = "0.9.1-rc8";
    bool recommended_null = false;
    std::string recommendation;  // Listed as recommended instead of `recommended`, whose release stays listed.
    bool rollback = false;
    bool hold = false;
    bool tag_after_releases = false;
    std::string file;
    std::string listed_sha256;  // Empty: the real one.
    bool offline = false;       // No answer to any request.
    int auth_status = 200;
    int rate_limit_next = 0;    // Number of requests to answer with 429.
    int drop_file_request = -1; // Index of the file request that loses its connection after drop_after_bytes.
    uint32_t drop_after_bytes = 0;
    int num_no_answer_after_drop = 0;
    bool ignore_range = false;
    std::vector<std::string> requests;
    std::vector<std::string> checkins;
    std::string auth_body;
    int num_auth = 0;
    int num_file_requests = 0;

    // Device.
    uint32_t now_ms = 0;
    bool enabled = true;
    bool online = true;
    bool user_ota = false;
    const char* protocol = nullptr;
    SavedState stored;
    int num_saves = 0;
    uint32_t partition = 1;
    std::string flash = std::string(64 * 1024, '\xEE');
    int fail_writes = 0;       // Number of AT+OTA=WRITE payloads to answer with ERROR.
    int fail_erase = -1;       // Index of the AT+OTA=ERASE to answer with ERROR.
    int num_erases = 0;
    uint32_t erase_ms_per_sector = 0;
    // Like the RP2040 without a /console client: output later than this after the command is not sent. 0: no limit.
    uint32_t reply_window_ms = 0;
    int num_writes = 0;
    bool verified = false;
    bool booted = false;
    bool reboot_on_boot = false;  // AT+OTA=BOOT stops the test run like a reset would.
    std::vector<std::string> commands;

    uint32_t NowMs() override { return now_ms; }
    void SleepMs(uint32_t ms) override { now_ms += ms; }
    uint32_t Random() override { return 0; }
    bool Enabled() override { return enabled; }
    bool Online() override { return online; }
    bool GetIdentity(Identity& identity) override {
        snprintf(identity.unique_id, sizeof(identity.unique_id), "0123456789ABCDEF");
        snprintf(identity.mac_address, sizeof(identity.mac_address), "02:00:00:AB:CD:EF");
        snprintf(identity.part_code, sizeof(identity.part_code), "010250002G-20260101-000001");
        memset(identity.ota_keys[0], 'a', 128);
        memset(identity.ota_keys[1], 'b', 128);
        return true;
    }

    bool Http(const HttpRequest& request, HttpResult& result, BodySink* sink) override {
        std::string path = request.path;
        requests.push_back(std::string(request.method) + " " + path);
        now_ms += 50;
        if (offline) return false;
        if (rate_limit_next > 0) {
            rate_limit_next--;
            result.status = 429;
            result.retry_after_s = 30;
            return true;
        }
        if (path == "auth") {
            num_auth++;
            auth_body = request.body ? request.body : "";
            result.status = auth_status;
            std::string body = auth_status == 200 ? "{\"token\": \"tok-" + std::to_string(num_auth) + "\", \"expires_in\": 3600}"
                                                  : "{\"error\": \"bad_key\"}";
            return Deliver(result.status, body, sink, 7);
        }
        if (!request.token || std::string(request.token) != "tok-" + std::to_string(num_auth)) {
            result.status = 401;
            return true;
        }
        if (path == "checkin") {
            checkins.push_back(request.body);
            result.status = 204;
            return true;
        }
        if (path.rfind("releases", 0) == 0) {
            result.status = 200;
            return Deliver(200, Listing(), sink, 13);
        }
        if (path == "files/adsbee_1090-" + recommended + "/adsbee_1090.ota") {
            int index = num_file_requests++;
            if (ignore_range || request.range_first < 0) {
                result.status = 200;
                return Deliver(200, file, sink, 1024);
            }
            if (static_cast<size_t>(request.range_first) >= file.size()) {
                result.status = 416;
                return true;
            }
            if (num_no_answer_after_drop > 0 && index > drop_file_request && drop_file_request >= 0) {
                num_no_answer_after_drop--;
                return false;
            }
            std::string body = file.substr(request.range_first, request.range_last - request.range_first + 1);
            result.status = 206;
            if (index == drop_file_request) {
                Deliver(206, body.substr(0, drop_after_bytes), sink, 100);
                return false;
            }
            return Deliver(206, body, sink, 1000);
        }
        result.status = 404;
        return true;
    }

    void ConsoleSend(const uint8_t* data, size_t len) override {
        for (size_t i = 0; i < len; i++) {
            if (payload_left_ > 0) {
                payload_.push_back(static_cast<char>(data[i]));
                if (--payload_left_ == 0) FinishWrite();
            } else if (data[i] == '\n') {
                Command(line_);
                line_.clear();
            } else if (data[i] != '\r') {
                line_.push_back(static_cast<char>(data[i]));
            }
        }
    }

    bool ConsoleReadLine(char* line, size_t line_size, uint32_t timeout_ms) override {
        if (out_.empty()) {
            now_ms += timeout_ms;
            return false;
        }
        now_ms += 5;
        snprintf(line, line_size, "%s", out_.front().c_str());
        out_.pop_front();
        return true;
    }

    const char* ConsoleReportingProtocol() override { return protocol; }
    bool UserOTAActive() override { return user_ota; }
    void LoadState(SavedState& state) override { state = stored; }
    void SaveState(const SavedState& state) override {
        stored = state;
        num_saves++;
    }

    int NumRequests(const std::string& prefix) {
        int n = 0;
        for (const std::string& r : requests) n += r.rfind(prefix, 0) == 0;
        return n;
    }

   private:
    bool Deliver(int status, const std::string& body, BodySink* sink, size_t piece) {
        for (size_t i = 0; sink && i < body.size(); i += piece) {
            size_t n = std::min(piece, body.size() - i);
            if (!sink->OnBody(status, reinterpret_cast<const uint8_t*>(body.data()) + i, n)) return false;
        }
        return true;
    }

    std::string Listing() {
        std::string tag = "adsbee_1090-" + recommended;
        std::string rec = recommendation.empty() ? recommended : recommendation;
        std::string top = std::string("\"recommended\": ") + (recommended_null ? "null" : "\"" + rec + "\"") +
                          ", \"recommended_tag\": " + (recommended_null ? "null" : "\"adsbee_1090-" + rec + "\"") +
                          ", \"rollback\": " + (rollback ? "true" : "false") + ", \"reason\": \"rollout\", \"hold\": " +
                          (hold ? "true" : "false") + ", \"pinned\": false";
        std::string sha = listed_sha256.empty() ? Sha256Hex(file) : listed_sha256;
        std::string releases =
            "\"releases\": [{\"version\": \"0.9.0\", \"tag\": \"adsbee_1090-0.9.0\", \"prerelease\": false, \"assets\": ["
            "{\"name\": \"adsbee_1090.ota\", \"kind\": \"ota\", \"size\": 5, \"sha256\": \"00\", \"url\": \"files/x/y.ota\"}]}, "
            "{\"version\": \"" + recommended + "\", \"assets\": [{\"name\": \"combined.uf2\", \"kind\": \"uf2\", \"size\": 9, "
            "\"sha256\": \"11\", \"url\": \"files/" + tag + "/combined.uf2\"}, {\"name\": \"adsbee_1090.ota\", \"kind\": "
            "\"ota\", \"size\": " + std::to_string(file.size()) + ", \"sha256\": \"" + sha + "\", \"url\": \"files/" + tag +
            "/adsbee_1090.ota\"}], \"notes_url\": \"https://example.com/a,b\", \"tag\": \"" + tag + "\"}]";
        return "{\"product\": \"adsbee_1090\", \"current\": \"0.9.1-rc7\", " +
               (tag_after_releases ? releases + ", " + top : top + ", " + releases) + "}";
    }

    void Command(const std::string& line) {
        commands.push_back(line);
        unsigned long offset = 0, len = 0, crc = 0;
        if (line == "AT+OTA=GET_PARTITION") {
            out_.push_back("Partition: " + std::to_string(partition));
            out_.push_back("OK");
        } else if (sscanf(line.c_str(), "AT+OTA=WRITE,%lx,%lu,%lx", &offset, &len, &crc) == 3) {
            write_offset_ = offset;
            write_crc_ = crc;
            payload_left_ = len;
            payload_.clear();
            out_.push_back("READY");
        } else if (sscanf(line.c_str(), "AT+OTA=ERASE,%lx,%lu", &offset, &len) == 2) {
            out_.push_back("Erasing...");
            if (num_erases++ == fail_erase) {
                out_.push_back("ERROR Failed partial erase of complementary flash partition.");
                return;
            }
            flash.replace(offset, len, std::string(len, '\xFF'));
            uint32_t erase_ms = (len + 4095) / 4096 * erase_ms_per_sector;
            now_ms += erase_ms;
            if (reply_window_ms == 0 || erase_ms < reply_window_ms) out_.push_back("OK");
        } else if (line == "AT+OTA=VERIFY") {
            verified = true;
            out_.push_back("OK");
        } else if (line == "AT+OTA=BOOT") {
            booted = true;
            if (reboot_on_boot) throw Reboot();
        } else {
            out_.push_back("OK");
        }
    }

    void FinishWrite() {
        num_writes++;
        if (fail_writes > 0) {
            fail_writes--;
            out_.push_back("ERROR Timed out after 5000 ms. Received 0 Bytes.");
            return;
        }
        if (Crc32(reinterpret_cast<const uint8_t*>(payload_.data()), payload_.size()) != write_crc_) {
            out_.push_back("ERROR Calculated CRC did not match.");
            return;
        }
        flash.replace(write_offset_, payload_.size(), payload_);
        out_.push_back("Writing...");
        out_.push_back("OK");
    }

   public:
    struct Reboot {};

   private:
    std::string line_;
    std::string payload_;
    size_t payload_left_ = 0;
    uint32_t write_offset_ = 0;
    uint32_t write_crc_ = 0;
    std::deque<std::string> out_;
};

constexpr uint32_t kAppLen = 10000;
constexpr uint32_t kDayMs = 24 * 60 * 60 * 1000;

struct Fixture {
    FakePlatform p;
    Fixture() { p.file = MakeOTA(kAppLen); }

    // The image the fake RP2040 should hold for its partition: header, then the application at 4096.
    void ExpectFlashed() {
        size_t image = 12 + p.partition * (20 + kAppLen);
        EXPECT_EQ(p.flash.substr(0, 20), p.file.substr(image, 20));
        EXPECT_EQ(p.flash.substr(4096, kAppLen), p.file.substr(image + 20, kAppLen));
        EXPECT_EQ(p.flash[4096 + kAppLen], '\xFF');  // Padding of the last write.
    }
};

}  // namespace

TEST(SHA256, KnownVectors) {
    EXPECT_EQ(Sha256Hex(""), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    EXPECT_EQ(Sha256Hex("abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    EXPECT_EQ(Sha256Hex(std::string(1000000, 'a')), "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
    // Lengths around the padding boundary.
    EXPECT_EQ(Sha256Hex(std::string(55, 'a')), "9f4390f8d30c2dd92ec9f095b65e2b9ae9b0a925a5258e241c9f1e910f734318");
    EXPECT_EQ(Sha256Hex(std::string(56, 'a')), "b35439a4ac6f0948b6d6f9e3c6af0f5f590ce20f1bde7090ef7970686ec6738a");
    EXPECT_EQ(Sha256Hex(std::string(64, 'a')), "ffe054fe7ae0cb6dc65c3af9b61d5209f439851db43d0ba5997337df154668eb");
}

TEST(JsonScanner, ReportsValuesAndNesting) {
    struct Log : JsonScanner::Handler {
        std::string text;
        void OnValue(const char* key, const char* value, bool is_string, uint8_t depth) override {
            text += std::to_string(depth) + ":" + key + "=" + (is_string ? "'" : "") + value + (is_string ? "'" : "") + " ";
        }
        void OnOpen(char bracket, const char* key, uint8_t depth) override { text += std::string(key) + bracket + " "; }
        void OnClose(char bracket, uint8_t depth) override { text += std::string(1, bracket) + " "; }
    } log;
    JsonScanner scanner(log);
    const char* json = "{\"a\": \"x,}\\\"y\", \"b\": null, \"c\": [1, -2.5, {\"d\": true}], \"e\": \"\"}";
    for (const char* c = json; *c; c++) scanner.Feed(c, 1);  // One byte at a time.
    EXPECT_EQ(log.text, "{ 1:a='x,}\"y' 1:b=null c[ 2:=1 2:=-2.5 { 3:d=true } ] 1:e='' } ");
    EXPECT_TRUE(scanner.complete());
}

TEST(AutoUpdater, CompareVersions) {
    EXPECT_EQ(AutoUpdater::CompareVersions("0.9.1-rc8", "0.9.1-rc8"), 0);
    EXPECT_LT(AutoUpdater::CompareVersions("0.9.1-rc8", "0.9.1-rc10"), 0);
    EXPECT_LT(AutoUpdater::CompareVersions("0.9.1-rc8", "0.9.1"), 0);
    EXPECT_GT(AutoUpdater::CompareVersions("0.10.0-rc1", "0.9.1"), 0);
    EXPECT_GT(AutoUpdater::CompareVersions("1.0.0", "0.99.99"), 0);
}

TEST(AutoUpdater, OfflineDoesNothing) {
    Fixture f;
    f.p.online = false;
    AutoUpdater updater(f.p, "0.9.1-rc8");
    EXPECT_EQ(updater.RunOnce(), 60u * 1000);
    EXPECT_TRUE(f.p.requests.empty());
}

TEST(AutoUpdater, UpToDateChecksOncePerDay) {
    Fixture f;
    AutoUpdater updater(f.p, "0.9.1-rc8");
    EXPECT_EQ(updater.FirstDelayMs(0), 3u * 60 * 1000);
    EXPECT_EQ(updater.FirstDelayMs(10 * 60 * 1000), 30u * 1000);
    EXPECT_EQ(updater.RunOnce(), kDayMs);
    EXPECT_EQ(updater.state(), AutoUpdater::kIdle);
    EXPECT_EQ(f.p.requests, (std::vector<std::string>{"POST auth", "GET releases?channel=stable"}));
    EXPECT_TRUE(f.p.commands.empty());
    EXPECT_TRUE(f.p.checkins.empty());
    // What the service is told about the device.
    EXPECT_NE(f.p.auth_body.find("\"client\":\"esp32_auto\",\"product\":\"adsbee_1090\",\"unique_id\":\"0123456789ABCDEF\","
                                 "\"mac_address\":\"02:00:00:AB:CD:EF\",\"part_code\":\"010250002G-20260101-000001\","
                                 "\"ota_keys\":[\"aaaa"),
              std::string::npos);
    EXPECT_NE(f.p.auth_body.find("\"versions\":{\"rp2040\":\"0.9.1-rc8\",\"esp32\":\"0.9.1-rc8\"}}"), std::string::npos);
}

TEST(AutoUpdater, InstallsTheRecommendedVersion) {
    for (uint32_t partition = 0; partition < 2; partition++) {
        SCOPED_TRACE(partition);
        Fixture f;
        f.p.partition = partition;
        f.p.recommended = "0.9.1-rc9";
        f.p.reboot_on_boot = true;
        AutoUpdater updater(f.p, "0.9.1-rc8");
        EXPECT_THROW(updater.RunOnce(), FakePlatform::Reboot);
        f.ExpectFlashed();
        EXPECT_TRUE(f.p.verified);
        EXPECT_TRUE(f.p.booted);
        EXPECT_EQ(f.p.commands.back(), "AT+OTA=BOOT");
        EXPECT_STREQ(f.p.stored.pending, "0.9.1-rc9");
        ASSERT_GE(f.p.checkins.size(), 1u);
        EXPECT_EQ(f.p.checkins[0], "{\"state\":\"downloading\",\"version\":\"0.9.1-rc9\",\"detail\":\"\"}");
        // Small ranges, in order, covering the file once.
        EXPECT_EQ(f.p.num_file_requests, static_cast<int>((f.p.file.size() + 4095) / 4096));
    }
}

TEST(AutoUpdater, ReportsInstalledAfterTheReboot) {
    Fixture f;
    f.p.recommended = "0.9.1-rc9";
    f.p.reboot_on_boot = true;
    {
        AutoUpdater updater(f.p, "0.9.1-rc8");
        EXPECT_THROW(updater.RunOnce(), FakePlatform::Reboot);
    }
    EXPECT_STREQ(f.p.stored.pending, "0.9.1-rc9");
    f.p.checkins.clear();
    f.p.commands.clear();
    AutoUpdater updater(f.p, "0.9.1-rc9");  // The new firmware.
    EXPECT_EQ(updater.FirstDelayMs(0), 60u * 1000);
    EXPECT_EQ(updater.RunOnce(), kDayMs);
    EXPECT_EQ(f.p.checkins, (std::vector<std::string>{"{\"state\":\"installed\",\"version\":\"0.9.1-rc9\",\"detail\":\"\"}"}));
    EXPECT_STREQ(f.p.stored.pending, "");
    EXPECT_EQ(f.p.stored.attempts, 0);
    EXPECT_TRUE(f.p.commands.empty());
    EXPECT_EQ(updater.state(), AutoUpdater::kIdle);
}

TEST(AutoUpdater, ReportsFailedWhenTheOldVersionComesBack) {
    Fixture f;
    f.p.recommended = "0.9.1-rc9";
    snprintf(f.p.stored.pending, sizeof(f.p.stored.pending), "0.9.1-rc9");
    snprintf(f.p.stored.target, sizeof(f.p.stored.target), "0.9.1-rc9");
    f.p.stored.attempts = 3;
    AutoUpdater updater(f.p, "0.9.1-rc8");
    EXPECT_EQ(updater.RunOnce(), kDayMs);
    ASSERT_EQ(f.p.checkins.size(), 1u);
    EXPECT_EQ(f.p.checkins[0], "{\"state\":\"failed\",\"version\":\"0.9.1-rc9\",\"detail\":\"running 0.9.1-rc8 after reboot\"}");
    EXPECT_EQ(updater.state(), AutoUpdater::kFailed);  // Out of attempts until the next day.
    EXPECT_TRUE(f.p.commands.empty());
}

TEST(AutoUpdater, ReportsFailedWhenTheRP2040DoesNotReboot) {
    Fixture f;
    f.p.recommended = "0.9.1-rc9";
    AutoUpdater updater(f.p, "0.9.1-rc8");
    EXPECT_EQ(updater.RunOnce(), 60u * 60 * 1000);
    EXPECT_TRUE(f.p.booted);
    EXPECT_STREQ(f.p.stored.pending, "");
    EXPECT_EQ(f.p.checkins.back(), "{\"state\":\"failed\",\"version\":\"0.9.1-rc9\",\"detail\":\"no reboot\"}");
}

TEST(AutoUpdater, HoldAndNoRecommendationInstallNothing) {
    Fixture f;
    f.p.recommended = "0.9.1-rc9";
    f.p.hold = true;
    AutoUpdater updater(f.p, "0.9.1-rc8");
    EXPECT_EQ(updater.RunOnce(), kDayMs);
    EXPECT_EQ(updater.state(), AutoUpdater::kHeld);
    f.p.hold = false;
    f.p.recommended_null = true;
    EXPECT_EQ(updater.RunOnce(), kDayMs);
    EXPECT_EQ(updater.state(), AutoUpdater::kIdle);
    EXPECT_TRUE(f.p.commands.empty());
    EXPECT_EQ(f.p.num_file_requests, 0);
}

TEST(AutoUpdater, OlderVersionOnlyWithRollback) {
    Fixture f;
    f.p.recommended = "0.9.1-rc7";
    AutoUpdater updater(f.p, "0.9.1-rc8");
    EXPECT_EQ(updater.RunOnce(), kDayMs);
    EXPECT_TRUE(f.p.commands.empty());
    f.p.rollback = true;
    f.p.reboot_on_boot = true;
    EXPECT_THROW(updater.RunOnce(), FakePlatform::Reboot);
    f.ExpectFlashed();
    EXPECT_TRUE(f.p.booted);
    EXPECT_STREQ(f.p.stored.pending, "0.9.1-rc7");
}

TEST(AutoUpdater, ListingWithTheRecommendationLast) {
    Fixture f;
    f.p.recommended = "0.9.1-rc9";
    f.p.tag_after_releases = true;
    AutoUpdater updater(f.p, "0.9.1-rc8");
    updater.RunOnce();
    f.ExpectFlashed();
    EXPECT_EQ(f.p.NumRequests("GET releases"), 2);
}

TEST(AutoUpdater, BadKeysStopUntilTheNextDay) {
    Fixture f;
    f.p.auth_status = 401;
    AutoUpdater updater(f.p, "0.9.1-rc8");
    EXPECT_EQ(updater.RunOnce(), kDayMs);
    EXPECT_EQ(updater.state(), AutoUpdater::kNoKeys);
    EXPECT_EQ(f.p.requests, (std::vector<std::string>{"POST auth"}));
}

TEST(AutoUpdater, ExpiredTokenAuthenticatesAgainOnce) {
    Fixture f;
    AutoUpdater updater(f.p, "0.9.1-rc8");
    updater.RunOnce();
    f.p.num_auth = 5;  // The service forgets the token.
    f.p.requests.clear();
    EXPECT_EQ(updater.RunOnce(), kDayMs);
    EXPECT_EQ(f.p.requests,
              (std::vector<std::string>{"GET releases?channel=stable", "POST auth", "GET releases?channel=stable"}));
}

TEST(AutoUpdater, RateLimitAndOutageBackOff) {
    Fixture f;
    f.p.rate_limit_next = 1;
    AutoUpdater updater(f.p, "0.9.1-rc8");
    EXPECT_EQ(updater.RunOnce(), 5u * 60 * 1000);  // Retry-After (30 s) is below the minimum wait.
    f.p.offline = true;
    EXPECT_EQ(updater.RunOnce(), 10u * 60 * 1000);
    EXPECT_EQ(updater.RunOnce(), 20u * 60 * 1000);
    for (int i = 0; i < 10; i++) updater.RunOnce();
    EXPECT_EQ(updater.RunOnce(), 2u * 60 * 60 * 1000);  // Capped.
    f.p.offline = false;
    EXPECT_EQ(updater.RunOnce(), kDayMs);
    EXPECT_EQ(updater.state(), AutoUpdater::kIdle);
}

TEST(AutoUpdater, ResumesAfterADroppedConnection) {
    Fixture f;
    f.p.recommended = "0.9.1-rc9";
    f.p.drop_file_request = 2;
    f.p.drop_after_bytes = 1234;
    f.p.num_no_answer_after_drop = 3;
    AutoUpdater updater(f.p, "0.9.1-rc8");
    updater.RunOnce();
    f.ExpectFlashed();
    EXPECT_TRUE(f.p.booted);
    // One extra request for the rest of the interrupted range, plus the unanswered ones.
    EXPECT_EQ(f.p.num_file_requests, static_cast<int>((f.p.file.size() - 2 * 4096 - 1234 + 4095) / 4096) + 3 + 3);
}

TEST(AutoUpdater, StalledDownloadFailsAndBacksOff) {
    Fixture f;
    f.p.recommended = "0.9.1-rc9";
    f.p.drop_file_request = 1;
    f.p.drop_after_bytes = 10;
    f.p.num_no_answer_after_drop = 1000;
    AutoUpdater updater(f.p, "0.9.1-rc8");
    uint32_t start_ms = f.p.now_ms;
    EXPECT_EQ(updater.RunOnce(), 15u * 60 * 1000);
    EXPECT_GE(f.p.now_ms - start_ms, 10u * 60 * 1000);
    EXPECT_LT(f.p.now_ms - start_ms, 13u * 60 * 1000);
    EXPECT_EQ(updater.state(), AutoUpdater::kFailed);
    EXPECT_STREQ(updater.detail(), "download stalled");
    EXPECT_FALSE(f.p.verified);
    EXPECT_FALSE(f.p.booted);
    EXPECT_EQ(f.p.checkins.back(), "{\"state\":\"failed\",\"version\":\"0.9.1-rc9\",\"detail\":\"download stalled\"}");
}

TEST(AutoUpdater, ChecksumMismatchIsRejectedWithoutBooting) {
    Fixture f;
    f.p.recommended = "0.9.1-rc9";
    f.p.listed_sha256 = Sha256Hex(f.p.file);
    f.p.file[f.p.file.size() - 100] ^= 0x01;  // One bit, in the last partition's image.
    AutoUpdater updater(f.p, "0.9.1-rc8");
    EXPECT_EQ(updater.RunOnce(), 15u * 60 * 1000);
    EXPECT_EQ(updater.state(), AutoUpdater::kFailed);
    EXPECT_STREQ(updater.detail(), "sha256 mismatch");
    EXPECT_FALSE(f.p.verified);
    EXPECT_FALSE(f.p.booted);
    EXPECT_STREQ(f.p.stored.pending, "");
    EXPECT_EQ(f.p.flash.substr(0, 20), std::string(20, '\xFF'));  // Header erased again.
    EXPECT_EQ(f.p.checkins.back(), "{\"state\":\"failed\",\"version\":\"0.9.1-rc9\",\"detail\":\"sha256 mismatch\"}");
}

TEST(AutoUpdater, AtMostThreeAttemptsPerDayPerTarget) {
    Fixture f;
    f.p.recommended = "0.9.1-rc9";
    f.p.listed_sha256 = std::string(64, '0');
    AutoUpdater updater(f.p, "0.9.1-rc8");
    EXPECT_EQ(updater.RunOnce(), 15u * 60 * 1000);
    EXPECT_EQ(updater.RunOnce(), 60u * 60 * 1000);
    EXPECT_EQ(updater.RunOnce(), kDayMs);
    EXPECT_EQ(f.p.stored.attempts, 3);
    int file_requests = f.p.num_file_requests;
    // Out of attempts: checks still run, nothing is downloaded, also after a reboot.
    EXPECT_EQ(updater.RunOnce(), kDayMs);
    AutoUpdater rebooted(f.p, "0.9.1-rc8");
    EXPECT_EQ(rebooted.RunOnce(), kDayMs);
    EXPECT_EQ(rebooted.state(), AutoUpdater::kFailed);
    EXPECT_EQ(f.p.num_file_requests, file_requests);
    // A day later the budget is back.
    f.p.now_ms += kDayMs;
    EXPECT_EQ(rebooted.RunOnce(), 15u * 60 * 1000);
    EXPECT_GT(f.p.num_file_requests, file_requests);
    // A different target starts a new count.
    f.p.recommended = "0.9.1-rc10";
    f.p.listed_sha256.clear();
    rebooted.RunOnce();
    EXPECT_TRUE(f.p.booted);
}

TEST(AutoUpdater, RetriesAFailedFlashWrite) {
    Fixture f;
    f.p.recommended = "0.9.1-rc9";
    f.p.fail_writes = 2;
    AutoUpdater updater(f.p, "0.9.1-rc8");
    updater.RunOnce();
    f.ExpectFlashed();
    EXPECT_TRUE(f.p.booted);
    f = Fixture();
    f.p.recommended = "0.9.1-rc9";
    f.p.fail_writes = 1000;
    AutoUpdater failing(f.p, "0.9.1-rc8");
    failing.RunOnce();
    EXPECT_STREQ(failing.detail(), "flash write failed");
    EXPECT_EQ(f.p.num_writes, 5);
    EXPECT_FALSE(f.p.booted);
}

std::vector<std::string> EraseCommands(const std::vector<std::string>& commands) {
    std::vector<std::string> erases;
    for (const std::string& c : commands) {
        if (c.rfind("AT+OTA=ERASE", 0) == 0) erases.push_back(c);
    }
    return erases;
}

TEST(AutoUpdater, ErasesInStepsThatAnswerWithinTheReplyWindow) {
    // The image (4096 + 10000 + 4096 Bytes, 5 sectors) takes 10 s to erase at once, the length of the window.
    Fixture f;
    f.p.recommended = "0.9.1-rc9";
    f.p.reboot_on_boot = true;
    f.p.erase_ms_per_sector = 2000;
    f.p.reply_window_ms = 10000;
    AutoUpdater::Config config;
    config.erase_step_bytes = 10000;  // Rounded down to whole sectors.
    AutoUpdater updater(f.p, "0.9.1-rc8", config);
    EXPECT_THROW(updater.RunOnce(), FakePlatform::Reboot);
    EXPECT_EQ(EraseCommands(f.p.commands), (std::vector<std::string>{"AT+OTA=ERASE,0,8192", "AT+OTA=ERASE,2000,8192",
                                                                     "AT+OTA=ERASE,4000,1808"}));
    f.ExpectFlashed();
    EXPECT_TRUE(f.p.verified);

    // A whole image per command, as before: the reply is never sent.
    Fixture g;
    g.p.recommended = "0.9.1-rc9";
    g.p.erase_ms_per_sector = 2000;
    g.p.reply_window_ms = 10000;
    config.erase_step_bytes = 1u << 30;
    AutoUpdater whole(g.p, "0.9.1-rc8", config);
    whole.RunOnce();
    EXPECT_STREQ(whole.detail(), "flash erase failed");
    EXPECT_EQ(EraseCommands(g.p.commands), (std::vector<std::string>{"AT+OTA=ERASE,0,18192"}));
    EXPECT_EQ(g.p.num_writes, 0);
    EXPECT_EQ(g.p.checkins.back(), "{\"state\":\"failed\",\"version\":\"0.9.1-rc9\",\"detail\":\"flash erase failed\"}");
}

TEST(AutoUpdater, DefaultEraseStepsAreWholeSectors) {
    AutoUpdater::Config config;
    EXPECT_EQ(config.erase_step_bytes % AutoUpdater::kFlashChunkLenBytes, 0u);
    EXPECT_GE(config.erase_step_bytes, AutoUpdater::kFlashChunkLenBytes);
    EXPECT_LE(config.erase_step_bytes, 256u * 1024);  // About 1.3 s on a bench unit, 8 s at the flash's slowest.
}

TEST(AutoUpdater, StopsAtAFailedEraseStep) {
    Fixture f;
    f.p.recommended = "0.9.1-rc9";
    f.p.fail_erase = 1;
    AutoUpdater::Config config;
    config.erase_step_bytes = 8192;
    AutoUpdater updater(f.p, "0.9.1-rc8", config);
    updater.RunOnce();
    EXPECT_STREQ(updater.detail(), "flash erase failed");
    EXPECT_EQ(EraseCommands(f.p.commands), (std::vector<std::string>{"AT+OTA=ERASE,0,8192", "AT+OTA=ERASE,2000,8192"}));
    EXPECT_EQ(f.p.num_writes, 0);
    EXPECT_FALSE(f.p.verified);
    EXPECT_FALSE(f.p.booted);
}

TEST(AutoUpdater, WaitsForAManualUpdateAndStopsWhenTurnedOff) {
    Fixture f;
    f.p.recommended = "0.9.1-rc9";
    f.p.user_ota = true;
    AutoUpdater updater(f.p, "0.9.1-rc8");
    EXPECT_EQ(updater.RunOnce(), 10u * 60 * 1000);
    EXPECT_TRUE(f.p.commands.empty());
    EXPECT_EQ(f.p.stored.attempts, 0);
    f.p.user_ota = false;
    f.p.enabled = false;
    updater.RunOnce();
    EXPECT_STREQ(updater.detail(), "turned off");
    EXPECT_FALSE(f.p.booted);
}

TEST(AutoUpdater, ServerWithoutRangesAndBadFilesAreRejected) {
    Fixture f;
    f.p.recommended = "0.9.1-rc9";
    f.p.ignore_range = true;
    AutoUpdater updater(f.p, "0.9.1-rc8");
    updater.RunOnce();
    EXPECT_STREQ(updater.detail(), "no range support");
    EXPECT_TRUE(f.p.commands.size() <= 1);  // Only the partition query.

    Fixture g;
    g.p.recommended = "0.9.1-rc9";
    g.p.file[12] ^= 0xFF;  // Partition 0's magic word; this device uses partition 0.
    g.p.partition = 0;
    AutoUpdater bad(g.p, "0.9.1-rc8");
    bad.RunOnce();
    EXPECT_STREQ(bad.detail(), "bad ota file");
    EXPECT_EQ(g.p.num_writes, 0);
}

TEST(AutoUpdater, SilencesConsoleReportsAndRestoresThemOnFailure) {
    Fixture f;
    f.p.recommended = "0.9.1-rc9";
    f.p.protocol = "BEAST";
    f.p.listed_sha256 = std::string(64, '0');
    AutoUpdater updater(f.p, "0.9.1-rc8");
    updater.RunOnce();
    EXPECT_EQ(f.p.commands.front(), "AT+PROTOCOL_OUT=CONSOLE,NONE");
    EXPECT_EQ(f.p.commands.back(), "AT+PROTOCOL_OUT=CONSOLE,BEAST");
}

TEST(AutoUpdater, SwarmTargetReplacesTheRecommendation) {
    Fixture f;
    f.p.recommended = "0.9.1-rc7";        // Listed with its .ota file.
    f.p.recommendation = "0.9.1-rc9";     // What the cohort is offered.
    f.p.reboot_on_boot = true;
    AutoUpdater updater(f.p, "0.9.1-rc8");
    updater.SetTarget("adsbee_1090-0.9.1-rc7");
    EXPECT_STREQ(updater.target(), "0.9.1-rc7");
    EXPECT_THROW(updater.RunOnce(), FakePlatform::Reboot);  // Older, and installed: the swarm owner chose it.
    f.ExpectFlashed();
    EXPECT_STREQ(f.p.stored.pending, "0.9.1-rc7");
    bool asked_rc = false;
    for (const std::string& r : f.p.requests) asked_rc |= r == "GET releases?channel=rc";
    EXPECT_TRUE(asked_rc);
}

TEST(AutoUpdater, SwarmTargetNotListedInstallsNothing) {
    Fixture f;
    f.p.recommended = "0.9.1-rc9";
    AutoUpdater updater(f.p, "0.9.1-rc8");
    updater.SetTarget("0.9.3");
    EXPECT_EQ(updater.RunOnce(), kDayMs);
    EXPECT_EQ(updater.state(), AutoUpdater::kFailed);
    EXPECT_STREQ(updater.detail(), "target not offered");
    EXPECT_TRUE(f.p.commands.empty());

    // The running version as target: nothing to do.
    updater.SetTarget("0.9.1-rc8");
    EXPECT_EQ(updater.RunOnce(), kDayMs);
    EXPECT_EQ(updater.state(), AutoUpdater::kIdle);
    EXPECT_TRUE(f.p.commands.empty());

    // No target: the recommendation again.
    updater.SetTarget(nullptr);
    f.p.reboot_on_boot = true;
    EXPECT_THROW(updater.RunOnce(), FakePlatform::Reboot);
    EXPECT_STREQ(f.p.stored.pending, "0.9.1-rc9");
}

TEST(AutoUpdater, WaitsForTheSwarmTargetOnlyWhileSwarmIsOn) {
    // AT+SWARM off: the first check happens at once, as before.
    {
        Fixture f;
        f.p.now_ms = 3 * 60 * 1000;
        f.p.recommended = "0.9.1-rc9";
        f.p.reboot_on_boot = true;
        AutoUpdater updater(f.p, "0.9.1-rc8");
        updater.SetTargetPending(false);
        EXPECT_EQ(updater.TargetWaitMs(), 0u);
        EXPECT_THROW(updater.RunOnce(), FakePlatform::Reboot);
        EXPECT_STREQ(f.p.stored.pending, "0.9.1-rc9");
    }
    // AT+SWARM on, the server stages a target in time: that target, never the recommendation.
    {
        Fixture f;
        f.p.now_ms = 3 * 60 * 1000;
        f.p.recommended = "0.9.1-rc7";
        f.p.recommendation = "0.9.1-rc9";
        f.p.reboot_on_boot = true;
        AutoUpdater updater(f.p, "0.9.1-rc8");
        updater.SetTargetPending(true);
        EXPECT_EQ(updater.RunOnce(), 12 * 60 * 1000u);
        EXPECT_TRUE(f.p.requests.empty());
        updater.SetTarget("0.9.1-rc7");
        updater.SetTargetPending(false);
        EXPECT_THROW(updater.RunOnce(), FakePlatform::Reboot);
        EXPECT_STREQ(f.p.stored.pending, "0.9.1-rc7");
    }
    // AT+SWARM on, no answer: after 15 min of uptime the recommendation, as without swarm.
    {
        Fixture f;
        f.p.now_ms = 3 * 60 * 1000;
        f.p.recommended = "0.9.1-rc9";
        f.p.reboot_on_boot = true;
        AutoUpdater updater(f.p, "0.9.1-rc8");
        updater.SetTargetPending(true);
        uint32_t wait_ms = updater.RunOnce();
        EXPECT_EQ(wait_ms, 12 * 60 * 1000u);
        EXPECT_TRUE(f.p.requests.empty());
        f.p.now_ms += wait_ms;
        EXPECT_EQ(updater.TargetWaitMs(), 0u);
        EXPECT_THROW(updater.RunOnce(), FakePlatform::Reboot);
        EXPECT_STREQ(f.p.stored.pending, "0.9.1-rc9");
    }
}
