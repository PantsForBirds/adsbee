#include <cstring>
#include <string>
#include <vector>

#include "aircraft_dictionary.hh"
#include "fec.hh"
#include "geo_utils.hh"
#include "gtest/gtest.h"
#include "hal.hh"
#include "opendroneid.h"
#include "range_map.hh"
#include "swarm_reporter.hh"
#include "uat_packet.hh"

namespace {

uint16_t DegToBam16(float deg) { return static_cast<uint16_t>(deg * (65536.0f / 360.0f)); }

// Standard base64 decoder for checking what the reporter sends.
std::vector<uint8_t> Base64Decode(const std::string& text) {
    static const std::string kAlphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::vector<uint8_t> out;
    uint32_t v = 0;
    int bits = 0;
    for (char c : text) {
        if (c == '=') break;
        v = (v << 6) | kAlphabet.find(c);
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            out.push_back((v >> bits) & 0xFF);
        }
    }
    return out;
}

// The string value of "key":"..." in a JSON text, or "" if absent.
std::string JsonString(const std::string& json, const std::string& key) {
    size_t pos = json.find("\"" + key + "\":\"");
    if (pos == std::string::npos) return "";
    pos += key.size() + 4;
    return json.substr(pos, json.find('"', pos) - pos);
}

// The number after "key": in a JSON text, or -1 if absent.
long JsonNumber(const std::string& json, const std::string& key) {
    size_t pos = json.find("\"" + key + "\":");
    if (pos == std::string::npos) return -1;
    return strtol(json.c_str() + pos + key.size() + 3, nullptr, 10);
}

class FakeSwarmPlatform : public SwarmPlatform {
   public:
    uint32_t now_ms = 0;
    bool online = true;
    bool answer = true;
    int status = 200;
    uint32_t retry_after_s = 0;
    std::string response = "{\"next_report_s\": 3600, \"firmware\": null}";
    std::vector<std::string> urls;
    std::vector<std::string> bodies;
    std::vector<std::string> tokens;
    std::string target = "(unset)";
    std::string target_channel;

    uint32_t NowMs() override { return now_ms; }
    uint32_t Random() override { return 0; }
    bool Online() override { return online; }
    bool GetIdentity(AutoUpdatePlatform::Identity& identity) override {
        snprintf(identity.unique_id, sizeof(identity.unique_id), "E6614C311B5B6F2A");
        snprintf(identity.mac_address, sizeof(identity.mac_address), "02:00:00:AB:CD:EF");
        memset(identity.ota_keys[0], 'a', 128);
        memset(identity.ota_keys[1], 'b', 128);
        return true;
    }
    bool Http(const AutoUpdatePlatform::HttpRequest& request, AutoUpdatePlatform::HttpResult& result,
              AutoUpdatePlatform::BodySink* sink) override {
        urls.push_back(request.path);
        bodies.push_back(request.body ? request.body : "");
        tokens.push_back(request.token ? request.token : "");
        if (!answer) return false;
        result.status = status;
        result.retry_after_s = retry_after_s;
        // In two pieces, like a body split across reads.
        size_t half = response.size() / 2;
        sink->OnBody(status, reinterpret_cast<const uint8_t*>(response.data()), half);
        sink->OnBody(status, reinterpret_cast<const uint8_t*>(response.data()) + half, response.size() - half);
        return true;
    }
    void SetFirmwareTarget(const char* tag, const char* channel) override {
        target = tag ? tag : "";
        target_channel = channel ? channel : "";
    }
};

SwarmReporter::DeviceState FixedPosition() {
    SwarmReporter::DeviceState state;
    state.auto_update = true;
    state.rx_position_valid = true;
    state.latitude_deg = 37.1f;
    state.longitude_deg = -122.1f;
    state.altitude_m = 30;
    state.rx_position_source = "fixed";
    state.range_valid = true;
    return state;
}

void AddSecond(SwarmRecorder& r, uint32_t modes, uint32_t es, uint32_t uat, uint32_t uplink, uint16_t aircraft) {
    r.AddFrames(modes, es, uat, uplink);
    r.NoteAircraftCount(aircraft);
}

const char kToken[] = "0123456789abcdef0123456789abcdef";

}  // namespace

TEST(SwarmRange, AltitudeBands) {
    struct {
        int32_t ft;
        uint8_t band;
    } cases[] = {{-500, 0},  {0, 0},      {999, 0},    {1000, 1},   {1999, 1},   {2000, 2},  {3999, 2},
                 {4000, 3},  {5999, 3},   {6000, 4},   {9999, 4},   {10000, 5},  {14999, 5}, {15000, 6},
                 {19999, 6}, {20000, 7},  {24999, 7},  {25000, 8},  {29999, 8},  {30000, 9}, {34999, 9},
                 {35000, 10}, {39999, 10}, {40000, 11}, {60000, 11}, {INT32_MAX, 11}};
    for (const auto& c : cases) EXPECT_EQ(RangeMap::AltitudeBand(c.ft), c.band) << c.ft << " ft";
}

TEST(SwarmRange, SectorsAndDistanceValues) {
    EXPECT_EQ(RangeMap::Sector(0), 0);
    EXPECT_EQ(RangeMap::Sector(DegToBam16(9.99f)), 0);
    EXPECT_EQ(RangeMap::Sector(DegToBam16(10.01f)), 1);
    EXPECT_EQ(RangeMap::Sector(DegToBam16(90.5f)), 9);
    EXPECT_EQ(RangeMap::Sector(DegToBam16(359.9f)), 35);
    EXPECT_EQ(RangeMap::Sector(UINT16_MAX), 35);
    for (uint32_t bam = 0; bam <= UINT16_MAX; bam += 7) {
        ASSERT_EQ(RangeMap::Sector(bam), static_cast<uint8_t>(bam * 360.0 / 65536.0 / 10.0)) << bam;
    }

    EXPECT_EQ(RangeMap::DistanceValue(0), 1);
    EXPECT_EQ(RangeMap::DistanceValue(1), 1);
    EXPECT_EQ(RangeMap::DistanceValue(2000), 1);
    EXPECT_EQ(RangeMap::DistanceValue(2001), 2);
    EXPECT_EQ(RangeMap::DistanceValue(509999), 255);
    EXPECT_EQ(RangeMap::DistanceValue(510000), 255);
    EXPECT_EQ(RangeMap::DistanceValue(UINT32_MAX), 255);
}

TEST(SwarmRange, RecordKeepsTheMaximumPerBin) {
    RangeMap map;
    map.Record(50000, DegToBam16(45.0f), 12000);   // band 5, sector 4: 25
    map.Record(30000, DegToBam16(45.0f), 12000);   // smaller: kept 25
    map.Record(70500, DegToBam16(44.0f), 14999);   // same bin: 36
    map.Record(1000, DegToBam16(355.0f), 41000);   // band 11, sector 35: 1
    EXPECT_EQ(map.bins[5 * 36 + 4], 36);
    EXPECT_EQ(map.bins[11 * 36 + 35], 1);
    int num_set = 0;
    for (uint8_t b : map.bins) num_set += b != 0;
    EXPECT_EQ(num_set, 2);
}

TEST(SwarmRange, EncodeBitmapIsLSBFirstInBinOrder) {
    RangeMap map;
    map.bins[0] = 7;
    map.bins[9] = 200;
    map.bins[431] = 3;
    uint8_t bitmap[RangeMap::kBitmapLenBytes];
    uint8_t values[RangeMap::kNumBins];
    ASSERT_EQ(map.Encode(bitmap, values), 3);
    EXPECT_EQ(sizeof(bitmap), 54u);
    EXPECT_EQ(bitmap[0], 0x01);
    EXPECT_EQ(bitmap[1], 0x02);  // Bin 9: byte 1, bit 1.
    EXPECT_EQ(bitmap[53], 0x80);
    for (int i = 2; i < 53; i++) EXPECT_EQ(bitmap[i], 0) << i;
    EXPECT_EQ(values[0], 7);
    EXPECT_EQ(values[1], 200);
    EXPECT_EQ(values[2], 3);
}

TEST(SwarmRange, Base64MatchesTheStandardAlphabetWithPadding) {
    char out[16];
    EXPECT_EQ(SwarmReporter::Base64Encode(reinterpret_cast<const uint8_t*>(""), 0, out), 0u);
    EXPECT_STREQ(out, "");
    SwarmReporter::Base64Encode(reinterpret_cast<const uint8_t*>("f"), 1, out);
    EXPECT_STREQ(out, "Zg==");
    SwarmReporter::Base64Encode(reinterpret_cast<const uint8_t*>("fo"), 2, out);
    EXPECT_STREQ(out, "Zm8=");
    SwarmReporter::Base64Encode(reinterpret_cast<const uint8_t*>("foobar"), 6, out);
    EXPECT_STREQ(out, "Zm9vYmFy");
    const uint8_t high[] = {0xFB, 0xFF, 0xBF};
    SwarmReporter::Base64Encode(high, 3, out);
    EXPECT_STREQ(out, "+/+/");
}

TEST(SwarmRecorder, UniqueAircraftEstimate) {
    SwarmRecorder recorder;
    EXPECT_EQ(recorder.UniqueAircraft(), 0);
    for (int repeat = 0; repeat < 3; repeat++) {
        for (uint32_t icao = 0xA00000; icao < 0xA00000 + 500; icao++) recorder.MarkAircraft(icao);
    }
    EXPECT_NEAR(recorder.UniqueAircraft(), 500, 25);
    SwarmRecorder later;
    for (uint32_t icao = 0xA00000 + 250; icao < 0xA00000 + 1500; icao++) later.MarkAircraft(icao);
    recorder.Merge(later);  // Union: 1500 different aircraft.
    EXPECT_NEAR(recorder.UniqueAircraft(), 1500, 75);
}

TEST(SwarmRecorder, MergeTakesMaximaAndSums) {
    SwarmRecorder a, b;
    AddSecond(a, 10, 20, 3, 4, 50);
    a.ranges[SwarmRecorder::kModeS].bins[7] = 10;
    AddSecond(b, 1, 2, 3, 4, 70);
    AddSecond(b, 1, 2, 3, 4, 60);
    b.ranges[SwarmRecorder::kModeS].bins[7] = 5;
    b.ranges[SwarmRecorder::kModeS].bins[8] = 9;
    a.Merge(b);
    EXPECT_EQ(a.stats.modes_valid, 12u);
    EXPECT_EQ(a.stats.es_valid, 24u);
    EXPECT_EQ(a.stats.uat_adsb, 9u);
    EXPECT_EQ(a.stats.uat_uplink, 12u);
    EXPECT_EQ(a.stats.aircraft_max, 70);
    EXPECT_EQ(a.ranges[SwarmRecorder::kModeS].bins[7], 10);
    EXPECT_EQ(a.ranges[SwarmRecorder::kModeS].bins[8], 9);
}

TEST(SwarmReporter, DefaultServerOnly) {
    EXPECT_TRUE(SwarmReporter::IsDefaultServer("https://adsbee.aero"));
    EXPECT_TRUE(SwarmReporter::IsDefaultServer("https://ADSBee.aero/"));
    EXPECT_TRUE(SwarmReporter::IsDefaultServer("https://adsbee.aero:443"));
    EXPECT_FALSE(SwarmReporter::IsDefaultServer("http://adsbee.aero"));
    EXPECT_FALSE(SwarmReporter::IsDefaultServer("https://adsbee.aero.example.com"));
    EXPECT_FALSE(SwarmReporter::IsDefaultServer("https://adsbee.aero:8443"));
    EXPECT_FALSE(SwarmReporter::IsDefaultServer("https://evil.example/adsbee.aero"));
    EXPECT_FALSE(SwarmReporter::IsDefaultServer("https://swarm.example.com"));
}

TEST(SwarmReporter, ReportBodyFollowsProtocolV1) {
    FakeSwarmPlatform platform;
    SwarmReporter reporter(platform, "0.9.1-rc9");
    SwarmRecorder live;
    AddSecond(live, 100, 200, 3, 4, 17);
    live.MarkAircraft(0xABCDEF);
    live.RecordPosition(SwarmRecorder::kModeS, 50000, DegToBam16(45.0f), 12000, 1);
    platform.now_ms = 3700 * 1000;
    reporter.Restart(live, 100 * 1000);  // Period starts at uptime 100 s.
    AddSecond(live, 100, 200, 3, 4, 17);
    live.RecordPosition(SwarmRecorder::kModeS, 50000, DegToBam16(45.0f), 12000, 1);
    reporter.TakePeriod(live, 3700 * 1000, FixedPosition());
    EXPECT_EQ(live.stats.modes_valid, 0u);  // The live period starts over.

    platform.now_ms = 3701 * 1000;
    reporter.Send("https://adsbee.aero/", kToken);
    ASSERT_EQ(platform.bodies.size(), 1u);
    EXPECT_EQ(platform.urls[0], "https://adsbee.aero/api/swarm/v1/report");
    EXPECT_EQ(platform.tokens[0], kToken);
    const std::string& body = platform.bodies[0];
    EXPECT_EQ(body.rfind("{\"v\":1,\"product\":\"adsbee_1090\",\"unique_id\":\"E6614C311B5B6F2A\"", 0), 0u) << body;
    EXPECT_NE(body.find("\"ota_keys\":[\"" + std::string(128, 'a') + "\",\"" + std::string(128, 'b') + "\"]"),
              std::string::npos);
    EXPECT_NE(body.find("\"versions\":{\"rp2040\":\"0.9.1-rc9\",\"esp32\":\"0.9.1-rc9\"}"), std::string::npos);
    EXPECT_NE(body.find("\"auto_update\":true"), std::string::npos);
    EXPECT_EQ(JsonNumber(body, "uptime_s"), 3701);
    EXPECT_EQ(JsonNumber(body, "seq"), 0);
    EXPECT_NE(body.find("\"period\":{\"start_uptime_s\":100,\"len_s\":3600}"), std::string::npos);
    EXPECT_NE(body.find("\"rx_position\":{\"lat\":37.10000,\"lon\":-122.10000,\"alt_m\":30,\"source\":\"fixed\"}"),
              std::string::npos);
    EXPECT_NE(body.find("\"stats\":{\"modes_valid\":100,\"es_valid\":200,\"uat_adsb\":3,\"uat_uplink\":4,"
                        "\"aircraft_max\":17,\"aircraft_unique\":0}"),
              std::string::npos);
    EXPECT_NE(body.find(",\"range_dropped\":0,\"ranges\":{\"mode_s\":{\"sectors\":36,\"bands\":12,\"unit_m\":2000,"),
              std::string::npos);
    EXPECT_EQ(body.find("\"range\":"), std::string::npos);  // Replaced by ranges.
    EXPECT_EQ(body.find("\"uat\""), std::string::npos);    // No data: left out.
    EXPECT_EQ(body.find("\"remote_id\""), std::string::npos);
    std::vector<uint8_t> bitmap = Base64Decode(JsonString(body, "changed"));
    std::vector<uint8_t> values = Base64Decode(JsonString(body, "max"));
    ASSERT_EQ(bitmap.size(), 54u);
    ASSERT_EQ(values.size(), 1u);
    const int bin = 5 * 36 + 4;
    EXPECT_EQ(bitmap[bin / 8], 1 << (bin % 8));
    EXPECT_EQ(values[0], 25);
    EXPECT_EQ(body.back(), '}');
    EXPECT_EQ(reporter.seq(), 1u);
    EXPECT_FALSE(reporter.has_pending());
}

TEST(SwarmReporter, NoOTAKeysToOtherServersAndNoPositionFromAircraft) {
    FakeSwarmPlatform platform;
    SwarmReporter reporter(platform, "0.9.1-rc9");
    SwarmRecorder live;
    live.RecordPosition(SwarmRecorder::kModeS, 50000, 0, 0, 1);
    reporter.TakePeriod(live, 60000, FixedPosition());
    reporter.Send("http://192.168.1.10:8000", kToken);
    ASSERT_EQ(platform.bodies.size(), 1u);
    EXPECT_EQ(platform.urls[0], "http://192.168.1.10:8000/api/swarm/v1/report");
    EXPECT_EQ(platform.bodies[0].find("ota_keys"), std::string::npos);
    EXPECT_NE(platform.bodies[0].find("\"ranges\":{\"mode_s\":{"), std::string::npos);

    // An aircraft-derived position (UsablePosition() false): no rx_position, no range, stats still go.
    reporter.TakePeriod(live, 120000, SwarmReporter::DeviceState());
    reporter.Send("http://192.168.1.10:8000", kToken);
    const std::string& body = platform.bodies[1];
    EXPECT_EQ(body.find("rx_position"), std::string::npos);
    EXPECT_EQ(body.find("\"ranges\""), std::string::npos);
    EXPECT_EQ(body.find("range_dropped"), std::string::npos);
    EXPECT_NE(body.find("\"stats\":{"), std::string::npos);
    EXPECT_NE(body.find(",\"range_withheld\":\"no_position\"}"), std::string::npos) << body;
}

TEST(SwarmReporter, FailedReportMergesWithSameSeqAndStart) {
    FakeSwarmPlatform platform;
    SwarmReporter reporter(platform, "0.9.1-rc9");
    SwarmRecorder live;
    reporter.Restart(live, 200 * 1000);
    AddSecond(live, 10, 20, 0, 0, 5);
    live.RecordPosition(SwarmRecorder::kModeS, 10000, DegToBam16(5.0f), 500, 1);   // bin 0: 5
    live.MarkAircraft(1);
    reporter.TakePeriod(live, 3800 * 1000, FixedPosition());

    platform.answer = false;
    platform.now_ms = 3800 * 1000;
    uint32_t retry_ms = reporter.Send("https://adsbee.aero", kToken);
    EXPECT_EQ(retry_ms, 60 * 1000u);  // First retry after a minute.
    EXPECT_TRUE(reporter.has_pending());
    EXPECT_EQ(reporter.seq(), 0u);
    EXPECT_STREQ(reporter.status().result, "no answer");

    // The next period merges into the pending report.
    AddSecond(live, 1, 2, 0, 0, 9);
    live.RecordPosition(SwarmRecorder::kModeS, 4000, DegToBam16(5.0f), 500, 1);    // Smaller: bin 0 stays 5.
    live.RecordPosition(SwarmRecorder::kModeS, 30000, DegToBam16(15.0f), 500, 1);  // bin 1: 15
    live.MarkAircraft(2);
    reporter.TakePeriod(live, 3860 * 1000, FixedPosition());
    platform.status = 503;
    platform.answer = true;
    retry_ms = reporter.Send("https://adsbee.aero", kToken);
    EXPECT_EQ(retry_ms, 120 * 1000u);  // Doubles.
    EXPECT_STREQ(reporter.status().result, "server error");

    reporter.TakePeriod(live, 3980 * 1000, FixedPosition());
    platform.status = 200;
    reporter.Send("https://adsbee.aero", kToken);
    ASSERT_EQ(platform.bodies.size(), 3u);
    for (const std::string& body : platform.bodies) {
        EXPECT_EQ(JsonNumber(body, "seq"), 0) << body;
        EXPECT_EQ(JsonNumber(body, "start_uptime_s"), 200) << body;
    }
    EXPECT_EQ(JsonNumber(platform.bodies[0], "len_s"), 3600);
    EXPECT_EQ(JsonNumber(platform.bodies[1], "len_s"), 3660);
    EXPECT_EQ(JsonNumber(platform.bodies[2], "len_s"), 3780);
    const std::string& last = platform.bodies[2];
    EXPECT_EQ(JsonNumber(last, "modes_valid"), 11);  // Counts cover the whole widened period.
    EXPECT_EQ(JsonNumber(last, "es_valid"), 22);
    EXPECT_EQ(JsonNumber(last, "aircraft_max"), 9);
    EXPECT_EQ(JsonNumber(last, "aircraft_unique"), 2);
    std::vector<uint8_t> values = Base64Decode(JsonString(last, "max"));
    ASSERT_EQ(values.size(), 2u);
    EXPECT_EQ(values[0], 5);
    EXPECT_EQ(values[1], 15);

    // Only a 200 takes a new seq; the next period starts where the merged one ended.
    EXPECT_EQ(reporter.seq(), 1u);
    reporter.TakePeriod(live, 7580 * 1000, FixedPosition());
    reporter.Send("https://adsbee.aero", kToken);
    EXPECT_EQ(JsonNumber(platform.bodies[3], "seq"), 1);
    EXPECT_EQ(JsonNumber(platform.bodies[3], "start_uptime_s"), 3980);
    EXPECT_EQ(JsonNumber(platform.bodies[3], "modes_valid"), 0);
    EXPECT_NE(platform.bodies[3].find("\"range_dropped\":0,\"ranges\":{}}"), std::string::npos);  // No data.
}

TEST(SwarmReporter, RangeStaysOutIfAnyMergedPeriodLackedAFixedPosition) {
    FakeSwarmPlatform platform;
    platform.answer = false;
    SwarmReporter reporter(platform, "0.9.1-rc9");
    SwarmRecorder live;
    SwarmReporter::DeviceState no_fix = FixedPosition();
    no_fix.range_valid = false;
    reporter.TakePeriod(live, 3600 * 1000, no_fix);
    reporter.Send("https://adsbee.aero", kToken);
    reporter.TakePeriod(live, 3660 * 1000, FixedPosition());
    platform.answer = true;
    reporter.Send("https://adsbee.aero", kToken);
    EXPECT_EQ(platform.bodies[1].find("\"ranges\""), std::string::npos);
}

TEST(SwarmReporter, ResponseSetsPeriodAndFirmwareTarget) {
    FakeSwarmPlatform platform;
    SwarmReporter reporter(platform, "0.9.1-rc9");
    SwarmRecorder live;
    platform.response = "{\"next_report_s\": 60, \"firmware\": {\"tag\": \"0.9.2\", \"channel\": \"stable\"}}";
    reporter.TakePeriod(live, 1000, FixedPosition());
    EXPECT_EQ(reporter.Send("https://adsbee.aero", kToken), 0u + 60 * 1000 - 30 * 1000);  // Jitter at most half.
    EXPECT_EQ(platform.target, "0.9.2");
    EXPECT_EQ(platform.target_channel, "stable");

    // Out-of-range periods are clamped; firmware null clears the target.
    platform.response = "{\"next_report_s\": 5, \"firmware\": null}";
    reporter.TakePeriod(live, 2000, FixedPosition());
    EXPECT_EQ(reporter.Send("https://adsbee.aero", kToken), 30 * 1000u);
    EXPECT_EQ(platform.target, "");
    platform.response = "{\"next_report_s\": 999999}";
    reporter.TakePeriod(live, 3000, FixedPosition());
    EXPECT_EQ(reporter.Send("https://adsbee.aero", kToken), (86400u - 60) * 1000);
    EXPECT_EQ(platform.target, "");
}

TEST(SwarmReporter, ErrorAnswers) {
    FakeSwarmPlatform platform;
    SwarmReporter reporter(platform, "0.9.1-rc9");
    SwarmRecorder live;

    // 429: Retry-After when longer than the backoff.
    platform.status = 429;
    platform.retry_after_s = 600;
    reporter.TakePeriod(live, 1000, FixedPosition());
    EXPECT_EQ(reporter.Send("https://adsbee.aero", kToken), 600 * 1000u);
    EXPECT_TRUE(reporter.has_pending());

    // 503 (keys can't be checked right now): Retry-After too, the report stays pending.
    platform.status = 503;
    platform.retry_after_s = 900;
    EXPECT_EQ(reporter.Send("https://adsbee.aero", kToken), 900 * 1000u);
    EXPECT_TRUE(reporter.has_pending());
    EXPECT_FALSE(reporter.status().stopped);
    platform.retry_after_s = 0;

    // 401: the report waits a period, still pending.
    platform.status = 401;
    EXPECT_EQ(reporter.Send("https://adsbee.aero", kToken), (3600u - 60) * 1000);
    EXPECT_TRUE(reporter.has_pending());
    EXPECT_STREQ(reporter.status().result, "rejected");
    EXPECT_EQ(reporter.status().last_http_status, 401);

    // 400: dropped, new seq.
    platform.status = 400;
    reporter.Send("https://adsbee.aero", kToken);
    EXPECT_FALSE(reporter.has_pending());
    EXPECT_EQ(reporter.seq(), 1u);

    // 410: stop until the token changes.
    platform.status = 410;
    reporter.TakePeriod(live, 2000, FixedPosition());
    reporter.Send("https://adsbee.aero", kToken);
    EXPECT_TRUE(reporter.status().stopped);
    reporter.TakePeriod(live, 3000, FixedPosition());
    size_t num_requests = platform.bodies.size();
    reporter.Send("https://adsbee.aero", kToken);
    EXPECT_EQ(platform.bodies.size(), num_requests);
    reporter.Reconfigure();
    platform.status = 200;
    reporter.TakePeriod(live, 4000, FixedPosition());
    reporter.Send("https://adsbee.aero", kToken);
    EXPECT_EQ(platform.bodies.size(), num_requests + 1);
    EXPECT_STREQ(reporter.status().result, "ok");
}

TEST(SwarmReporter, NewServerForgetsTheOldBackoff) {
    FakeSwarmPlatform platform;
    SwarmReporter reporter(platform, "0.9.1-rc9");
    SwarmRecorder live;
    platform.response = "{\"next_report_s\": 600, \"firmware\": null}";
    reporter.TakePeriod(live, 1000, FixedPosition());
    reporter.Send("https://adsbee.aero", kToken);
    reporter.TakePeriod(live, 2000, FixedPosition());
    platform.answer = false;
    EXPECT_EQ(reporter.Send("https://adsbee.aero", kToken), 60 * 1000u);
    EXPECT_EQ(reporter.Send("https://adsbee.aero", kToken), 120 * 1000u);
    EXPECT_EQ(reporter.Send("https://adsbee.aero", kToken), 240 * 1000u);
    EXPECT_EQ(reporter.Send("https://adsbee.aero", kToken), 480 * 1000u);
    EXPECT_EQ(reporter.Send("https://adsbee.aero", kToken), 600 * 1000u);  // Up to the server's period.
    uint32_t seq = reporter.seq();

    reporter.Reconfigure();
    EXPECT_TRUE(reporter.has_pending());
    EXPECT_EQ(reporter.Send("https://swarm.example", kToken), 60 * 1000u);
    EXPECT_EQ(reporter.seq(), seq);
    for (int i = 0; i < 10; i++) reporter.Send("https://swarm.example", kToken);
    EXPECT_EQ(reporter.Send("https://swarm.example", kToken), 3600 * 1000u);  // Back to the default period.
}

TEST(SwarmReporter, FirstReportTwoToFiveMinutesAfterBoot) {
    FakeSwarmPlatform platform;
    SwarmReporter reporter(platform, "0.9.1-rc9");
    EXPECT_EQ(reporter.FirstDelayMs(0), 120 * 1000u);  // Random() == 0.
    EXPECT_EQ(reporter.FirstDelayMs(100 * 1000), 20 * 1000u);
    EXPECT_EQ(reporter.FirstDelayMs(400 * 1000), 120 * 1000u);  // Turned on later: 2-5 min from now.
}

TEST(SwarmReporter, FullBodyFits) {
    FakeSwarmPlatform platform;
    SwarmReporter reporter(platform, "0.9.1-rc9");
    SwarmRecorder live;
    for (RangeMap& map : live.ranges) memset(map.bins, 255, sizeof(map.bins));
    AddSecond(live, UINT32_MAX / 2, UINT32_MAX / 2, UINT32_MAX / 2, UINT32_MAX / 2, UINT16_MAX);
    reporter.TakePeriod(live, 4000000000u, FixedPosition());
    reporter.Send("https://adsbee.aero", kToken);
    ASSERT_EQ(platform.bodies.size(), 1u);
    EXPECT_GT(platform.bodies[0].size(), 1000u);
    EXPECT_LT(platform.bodies[0].size(), SwarmReporter::kBodyMaxLen);
    for (const char* protocol : SwarmRecorder::kProtocolNames) {
        std::string map = platform.bodies[0].substr(platform.bodies[0].find(std::string("\"") + protocol + "\":{"));
        EXPECT_EQ(Base64Decode(JsonString(map, "max")).size(), 432u) << protocol;
    }
    printf("Largest body: %zu Bytes\n", platform.bodies[0].size());
}

// Each report allocates only its own length: the measured length is exact, one Byte less doesn't fit.
TEST(SwarmReporter, BodyLengthMeasuredExactly) {
    FakeSwarmPlatform platform;
    SwarmReporter reporter(platform, "0.9.1-rc9");
    AutoUpdatePlatform::Identity identity;
    ASSERT_TRUE(platform.GetIdentity(identity));
    std::vector<char> buf(SwarmReporter::kBodyMaxLen);
    for (int fill : {0, 1, 3}) {
        SwarmRecorder live;
        for (int p = 0; p < fill; p++) memset(live.ranges[p].bins, 200, sizeof(live.ranges[p].bins) / (p + 1));
        AddSecond(live, 1000, 500, 20, 3, 40);
        reporter.TakePeriod(live, 3600, FixedPosition());
        for (bool keys : {false, true}) {
            size_t measured = reporter.BuildBody(nullptr, 0, identity, keys, 3600);
            ASSERT_GT(measured, 0u);
            EXPECT_EQ(reporter.BuildBody(buf.data(), buf.size(), identity, keys, 3600), measured);
            EXPECT_EQ(strlen(buf.data()), measured);
            EXPECT_EQ(reporter.BuildBody(buf.data(), measured + 1, identity, keys, 3600), measured);
            EXPECT_EQ(reporter.BuildBody(buf.data(), measured, identity, keys, 3600), 0u);
        }
    }
}

// Positions reach the range map once an aircraft has two accepted decodes.
TEST(SwarmRecorder, DictionaryRecordsConfirmedPositions) {
    AircraftDictionary dictionary;
    SwarmRecorder recorder;
    dictionary.SetSwarmRecorder(&recorder);
    dictionary.SetReferencePosition(48.0f, 18.0f);
    auto ingest = [&](const char* hex) {
        DecodedModeSPacket packet = DecodedModeSPacket((char*)hex);
        inc_time_since_boot_ms(500);
        packet.raw.mlat_48mhz_64bit_counts = get_time_since_boot_ms() * 48'000;
        EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(packet));
    };
    auto num_bins = [&]() {
        int n = 0;
        for (uint8_t b : recorder.ranges[SwarmRecorder::kModeS].bins) n += b != 0;
        return n;
    };
    // 48.3 N 18.2 E: one decode, not recorded yet.
    ingest("8DBEE001589B803335F17E3806C1");
    ingest("8DBEE001589B87A9D1D79C757A00");
    dictionary.Update(get_time_since_boot_ms());
    EXPECT_EQ(num_bins(), 0);
    EXPECT_EQ(recorder.stats.es_valid, 0u);  // Frame counts come from the RP2040, not the dictionary.
    EXPECT_EQ(recorder.stats.aircraft_max, 1);
    EXPECT_EQ(recorder.UniqueAircraft(), 1);

    // Moved to 48.4 N: confirmed, recorded on the next Update().
    for (int i = 0; i < 2; i++) {
        ingest("8DBEE001589B804445F17E39A3D0");
        ingest("8DBEE001589B87BA99D79C52EE84");
    }
    dictionary.Update(get_time_since_boot_ms());
    ModeSAircraft* aircraft =
        dictionary.GetAircraftPtr<ModeSAircraft>(Aircraft::ICAOToUID(0xBEE001, Aircraft::kAircraftTypeModeS), false);
    ASSERT_TRUE(aircraft);
    EXPECT_EQ(aircraft->num_position_decodes, 2);
    ASSERT_EQ(num_bins(), 1);
    uint8_t band = RangeMap::AltitudeBand(aircraft->baro_altitude_ft);
    EXPECT_EQ(recorder.ranges[SwarmRecorder::kModeS].bins[band * 36 + 1], 24);  // 46897 m at 18.41 deg.
    EXPECT_EQ(recorder.UniqueAircraft(), 1);

    // Without a recorder nothing is touched.
    dictionary.SetSwarmRecorder(nullptr);
    recorder.Clear();
    ingest("8DBEE001589B803335F17E3806C1");
    ingest("8DBEE001589B87A9D1D79C757A00");
    dictionary.Update(get_time_since_boot_ms());
    EXPECT_EQ(num_bins(), 0);
    EXPECT_EQ(recorder.stats.aircraft_max, 0);
}

TEST(SwarmRecorder, OnlyDirectPositionsCountForRange) {
    ModeSAircraft mode_s(0xA46E6A);
    const ModeSAircraft::AddressType direct[] = {ModeSAircraft::kAddressTypeADSBICAO,
                                                 ModeSAircraft::kAddressTypeADSBICAONonTransponder,
                                                 ModeSAircraft::kAddressTypeADSBOther};
    const ModeSAircraft::AddressType relayed[] = {ModeSAircraft::kAddressTypeADSRICAO, ModeSAircraft::kAddressTypeTISBICAO,
                                                  ModeSAircraft::kAddressTypeADSROther,
                                                  ModeSAircraft::kAddressTypeTISBTrackFile,
                                                  ModeSAircraft::kAddressTypeTISBOther};
    for (auto type : direct) {
        mode_s.position_address_type = type;
        EXPECT_TRUE(AircraftDictionary::IsDirectPosition(mode_s)) << type;
    }
    for (auto type : relayed) {
        mode_s.position_address_type = type;
        EXPECT_FALSE(AircraftDictionary::IsDirectPosition(mode_s)) << type;
    }
    for (int q = 0; q <= 6; q++) {
        UATAircraft uat(0xA00001 | (q << Aircraft::kAddressQualifierBitShift));
        EXPECT_EQ(AircraftDictionary::IsDirectPosition(uat), q == 0 || q == 1 || q == 4 || q == 5) << q;
    }
}

TEST(SwarmReporter, UsablePositionIsFixedOrGNSSAndReal) {
    typedef SettingsManager::RxPosition P;
    EXPECT_TRUE(SwarmReporter::UsablePosition(true, P::kPositionSourceFixed, 37.1f, -122.1f));
    EXPECT_TRUE(SwarmReporter::UsablePosition(true, P::kPositionSourceGNSS, -33.9f, 151.2f));
    EXPECT_TRUE(SwarmReporter::UsablePosition(true, P::kPositionSourceFixed, 0.0f, 10.0f));
    EXPECT_FALSE(SwarmReporter::UsablePosition(false, P::kPositionSourceGNSS, 37.1f, -122.1f));  // No fix.
    EXPECT_FALSE(SwarmReporter::UsablePosition(true, P::kPositionSourceLowestAircraft, 37.1f, -122.1f));
    EXPECT_FALSE(SwarmReporter::UsablePosition(true, P::kPositionSourceAircraftMatchingICAO, 37.1f, -122.1f));
    EXPECT_FALSE(SwarmReporter::UsablePosition(true, P::kPositionSourceNone, 37.1f, -122.1f));
    EXPECT_FALSE(SwarmReporter::UsablePosition(true, P::kPositionSourceFixed, 0.0f, 0.0f));
    EXPECT_FALSE(SwarmReporter::UsablePosition(true, P::kPositionSourceFixed, 91.0f, 0.0f));
    EXPECT_FALSE(SwarmReporter::UsablePosition(true, P::kPositionSourceFixed, 10.0f, -181.0f));
    EXPECT_FALSE(SwarmReporter::UsablePosition(true, P::kPositionSourceFixed, NAN, 1.0f));
}

namespace {

// Horizon limit in km for a receiver at rx_m and a band top at top_ft, as the protocol defines it.
double HorizonKm(double rx_m, double top_ft) {
    return 1.15 * 4.12 * (sqrt(rx_m > 0 ? rx_m : 0) + sqrt(top_ft * 0.3048)) + 20;
}

// Great circle distance in meters and initial bearing in 1/65536 turns.
void DistanceAndBearing(double lat1, double lon1, double lat2, double lon2, uint32_t& distance_m, uint16_t& bam) {
    const double r = M_PI / 180;
    double dlat = (lat2 - lat1) * r, dlon = (lon2 - lon1) * r;
    double a = sin(dlat / 2) * sin(dlat / 2) + cos(lat1 * r) * cos(lat2 * r) * sin(dlon / 2) * sin(dlon / 2);
    distance_m = static_cast<uint32_t>(6371000 * 2 * atan2(sqrt(a), sqrt(1 - a)));
    double y = sin(dlon) * cos(lat2 * r), x = cos(lat1 * r) * sin(lat2 * r) - sin(lat1 * r) * cos(lat2 * r) * cos(dlon);
    double deg = fmod(atan2(y, x) / r + 360, 360);
    bam = static_cast<uint16_t>(deg / 360 * 65536);
}

// One hour of traffic around a receiver: mostly en route above 20,000 ft, a third low near airports. Positions are
// within the radio horizon of the real receiver. Feeds the distances as seen from `rx`.
void FeedTraffic(SwarmRecorder& recorder, double rx_lat, double rx_lon) {
    const double true_lat = 37.0, true_lon = -122.0;
    uint32_t seed = 12345;
    auto rnd = [&]() { return (seed = seed * 1103515245u + 12345u) / 4294967296.0; };
    for (uint32_t aircraft = 0; aircraft < 300; aircraft++) {
        bool low = aircraft % 3 == 0;
        int32_t alt_ft = low ? 500 + static_cast<int32_t>(rnd() * 5500) : 20000 + static_cast<int32_t>(rnd() * 19000);
        double top_ft = SwarmRecorder::kBandTopFt[RangeMap::AltitudeBand(alt_ft)];
        double max_km = low ? 60 : 300;  // Traffic density, not the horizon.
        double range_km = rnd() * (max_km < HorizonKm(30, top_ft) - 20 ? max_km : HorizonKm(30, top_ft) - 20);
        double brg = rnd() * 2 * M_PI;
        double lat = true_lat + range_km / 111.2 * cos(brg);
        double lon = true_lon + range_km / (111.2 * cos(true_lat * M_PI / 180)) * sin(brg);
        for (int sample = 0; sample < 20; sample++) {
            uint32_t distance_m;
            uint16_t bam;
            DistanceAndBearing(rx_lat, rx_lon, lat, lon, distance_m, bam);
            recorder.RecordPosition(SwarmRecorder::kModeS, distance_m, bam, alt_ft, 0xA00000 + aircraft);
        }
    }
}

}  // namespace

TEST(SwarmRecorder, HorizonLimitPerBand) {
    for (int32_t rx_m : {-50, 0, 30, 100, 1000}) {
        SwarmRecorder recorder;
        recorder.SetReceiverAltitude(rx_m);
        for (uint8_t band = 0; band < RangeMap::kNumBands; band++) {
            uint32_t limit_m = static_cast<uint32_t>(HorizonKm(rx_m, SwarmRecorder::kBandTopFt[band]) * 1000);
            int32_t alt_ft = band == 0 ? 0 : SwarmRecorder::kBandTopFt[band - 1];  // Bottom of the band.
            recorder.Clear();
            recorder.RecordPosition(SwarmRecorder::kModeS, limit_m - 100, 0, alt_ft, 1);
            recorder.RecordPosition(SwarmRecorder::kModeS, limit_m + 100, 0, alt_ft, 2);
            EXPECT_EQ(recorder.guard.num_positions, 2u) << rx_m << " m, band " << int(band);
            EXPECT_EQ(recorder.guard.num_dropped, 1u) << rx_m << " m, band " << int(band);
            EXPECT_EQ(recorder.ranges[SwarmRecorder::kModeS].bins[band * 36], RangeMap::DistanceValue(limit_m - 100));
        }
    }
    // 30 m antenna, 0-1k ft band: 1.15 * 4.12 * (5.48 + 17.46) + 20 = 128.7 km.
    SwarmRecorder recorder;
    recorder.SetReceiverAltitude(30);
    recorder.RecordPosition(SwarmRecorder::kModeS, 128000, 0, 500, 1);
    recorder.RecordPosition(SwarmRecorder::kModeS, 129500, 0, 500, 2);
    EXPECT_EQ(recorder.guard.num_dropped, 1u);
}

// The horizon guard catches gross position errors. A position 100 km off still sees most traffic inside each band's
// horizon (129 km below 1,000 ft, 163 km below 2,000 ft, ...), so it is kept; 300 km off is withheld.
TEST(SwarmRecorder, GrossPositionErrorWithholdsTheMap) {
    SwarmRecorder right;
    right.SetReceiverAltitude(30);
    FeedTraffic(right, 37.0, -122.0);
    EXPECT_EQ(right.guard.num_positions, 6000u);
    EXPECT_EQ(right.guard.num_dropped, 0u);
    EXPECT_FALSE(right.RangeImplausible());

    SwarmRecorder off_100_km;  // 0.9 degrees north.
    off_100_km.SetReceiverAltitude(30);
    FeedTraffic(off_100_km, 37.9, -122.0);
    EXPECT_LT(off_100_km.guard.num_dropped * 10, off_100_km.guard.num_positions);
    EXPECT_FALSE(off_100_km.RangeImplausible());

    SwarmRecorder off_300_km;  // 2.7 degrees north.
    off_300_km.SetReceiverAltitude(30);
    FeedTraffic(off_300_km, 39.7, -122.0);
    EXPECT_GE(off_300_km.guard.num_dropped * 10, off_300_km.guard.num_positions) << off_300_km.guard.num_dropped;
    EXPECT_EQ(off_300_km.guard.num_dropped_aircraft, SwarmRecorder::kWithholdDroppedAircraft);
    EXPECT_TRUE(off_300_km.RangeImplausible());
}

TEST(SwarmRecorder, WithholdNeedsTenPercentFromFiveAircraft) {
    SwarmRecorder recorder;
    recorder.SetReceiverAltitude(0);
    auto feed = [&](int good, int bad, int bad_aircraft) {
        recorder.Clear();
        for (int i = 0; i < good; i++) recorder.RecordPosition(SwarmRecorder::kModeS, 10000, 0, 30000, 100 + i % 7);
        for (int i = 0; i < bad; i++) recorder.RecordPosition(SwarmRecorder::kModeS, 400000, 0, 500, 1 + i % bad_aircraft);
    };
    feed(90, 10, 5);  // 10 %, 5 aircraft.
    EXPECT_TRUE(recorder.RangeImplausible());
    feed(91, 9, 5);  // 9 %.
    EXPECT_FALSE(recorder.RangeImplausible());
    feed(10, 90, 4);  // One aircraft short.
    EXPECT_FALSE(recorder.RangeImplausible());
    feed(10, 0, 1);
    EXPECT_FALSE(recorder.RangeImplausible());
    feed(0, 0, 1);
    EXPECT_FALSE(recorder.RangeImplausible());

    // A merged retry sums the counts and joins the aircraft.
    SwarmRecorder a, b;
    a.SetReceiverAltitude(0);
    b.SetReceiverAltitude(0);
    for (int i = 0; i < 3; i++) a.RecordPosition(SwarmRecorder::kModeS, 400000, 0, 500, 1 + i);
    for (int i = 0; i < 3; i++) b.RecordPosition(SwarmRecorder::kModeS, 400000, 0, 500, 3 + i);  // Aircraft 3 in both.
    for (int i = 0; i < 20; i++) b.RecordPosition(SwarmRecorder::kModeS, 1000, 0, 500, 9);
    a.Merge(b);
    EXPECT_EQ(a.guard.num_positions, 26u);
    EXPECT_EQ(a.guard.num_dropped, 6u);
    EXPECT_EQ(a.guard.num_dropped_aircraft, 5);
    EXPECT_TRUE(a.RangeImplausible());
}

TEST(SwarmReporter, WithheldMapSaysWhyAndCountsDrops) {
    FakeSwarmPlatform platform;
    SwarmReporter reporter(platform, "0.9.1-rc9");
    SwarmRecorder live;
    live.SetReceiverAltitude(30);
    for (int i = 0; i < 50; i++) live.RecordPosition(SwarmRecorder::kModeS, 20000, 0, 30000, 100);
    for (int i = 0; i < 10; i++) live.RecordPosition(SwarmRecorder::kModeS, 300000, 0, 500, 1 + i);
    reporter.TakePeriod(live, 60000, FixedPosition());
    reporter.Send("https://adsbee.aero", kToken);
    const std::string& body = platform.bodies[0];
    EXPECT_NE(body.find("\"rx_position\":{"), std::string::npos);
    EXPECT_NE(body.find(",\"range_withheld\":\"implausible\",\"range_dropped\":10}"), std::string::npos) << body;
    EXPECT_EQ(body.find("\"ranges\""), std::string::npos);

    // A clean period: the map, with its drop count.
    for (int i = 0; i < 50; i++) live.RecordPosition(SwarmRecorder::kModeS, 20000, 0, 30000, 100);
    live.RecordPosition(SwarmRecorder::kModeS, 300000, 0, 500, 1);
    reporter.TakePeriod(live, 120000, FixedPosition());
    reporter.Send("https://adsbee.aero", kToken);
    EXPECT_NE(platform.bodies[1].find(",\"range_dropped\":1,\"ranges\":{\"mode_s\":{"), std::string::npos) << platform.bodies[1];
    EXPECT_EQ(platform.bodies[1].find("range_withheld"), std::string::npos);
}

TEST(SwarmRange, DroneHeightBands) {
    struct {
        int32_t ft;
        uint8_t band;
    } cases[] = {{-300, 0}, {0, 0},     {99, 0},    {100, 1},   {199, 1},   {200, 2},   {299, 2},   {300, 3},
                 {399, 3},  {400, 4},   {499, 4},   {500, 5},   {699, 5},   {700, 6},   {999, 6},   {1000, 7},
                 {1499, 7}, {1500, 8},  {1999, 8},  {2000, 9},  {2999, 9},  {3000, 10}, {4999, 10}, {5000, 11},
                 {9999, 11}, {INT32_MAX, 11}};
    for (const auto& c : cases) EXPECT_EQ(RangeMap::HeightBand(c.ft), c.band) << c.ft << " ft";
    // Band tops are the first height of the next band.
    for (uint8_t band = 0; band + 1 < RangeMap::kNumBands; band++) {
        EXPECT_EQ(RangeMap::HeightBand(SwarmRecorder::kDroneBandTopFt[band]), band + 1) << int(band);
        EXPECT_EQ(RangeMap::HeightBand(SwarmRecorder::kDroneBandTopFt[band] - 1), band) << int(band);
        EXPECT_EQ(RangeMap::AltitudeBand(SwarmRecorder::kBandTopFt[band]), band + 1) << int(band);
    }
}

TEST(SwarmRange, DroneDistanceValuesIn50MeterSteps) {
    EXPECT_EQ(RangeMap::DistanceValue(0, 50), 1);
    EXPECT_EQ(RangeMap::DistanceValue(50, 50), 1);
    EXPECT_EQ(RangeMap::DistanceValue(51, 50), 2);
    EXPECT_EQ(RangeMap::DistanceValue(1234, 50), 25);
    EXPECT_EQ(RangeMap::DistanceValue(12750, 50), 255);
    EXPECT_EQ(RangeMap::DistanceValue(12751, 50), 255);  // Clamped: max 12.75 km.
    EXPECT_EQ(RangeMap::DistanceValue(40000, 50), 255);
}

TEST(SwarmRecorder, DronesBinByHeightAboveTheReceiver) {
    SwarmRecorder recorder;
    recorder.SetReceiverAltitude(300);  // 984 ft.
    recorder.RecordDrone(1234, DegToBam16(95.0f), 984 + 150, 7);    // 150 ft up: band 1, sector 9.
    recorder.RecordDrone(800, DegToBam16(95.0f), 984 + 199, 7);     // Same bin, shorter: kept 25.
    recorder.RecordDrone(300, DegToBam16(205.0f), 500, 8);          // Below the receiver: band 0.
    recorder.RecordDrone(9000, DegToBam16(0.0f), 984 + 6000, 9);    // Band 11.
    const RangeMap& map = recorder.ranges[SwarmRecorder::kRemoteID];
    EXPECT_EQ(map.bins[1 * 36 + 9], 25);
    EXPECT_EQ(map.bins[0 * 36 + 20], 6);
    EXPECT_EQ(map.bins[11 * 36 + 0], 180);
    int num_set = 0;
    for (uint8_t b : map.bins) num_set += b != 0;
    EXPECT_EQ(num_set, 3);
    EXPECT_TRUE(recorder.ranges[SwarmRecorder::kModeS].Empty());
    EXPECT_TRUE(recorder.ranges[SwarmRecorder::kUAT].Empty());
    EXPECT_EQ(recorder.guard.num_positions, 4u);

    // Not while the receiver position isn't fixed or GNSS.
    recorder.Clear();
    recorder.record_range = false;
    recorder.RecordDrone(1234, 0, 1000, 7);
    EXPECT_TRUE(recorder.RangeEmpty());
    EXPECT_EQ(recorder.guard.num_positions, 0u);
}

TEST(SwarmRecorder, ProtocolsKeepTheirOwnMaps) {
    SwarmRecorder recorder;
    recorder.RecordPosition(SwarmRecorder::kModeS, 50000, DegToBam16(45.0f), 12000, 1);
    recorder.RecordPosition(SwarmRecorder::kUAT, 30000, DegToBam16(45.0f), 12000, 2);
    EXPECT_EQ(recorder.ranges[SwarmRecorder::kModeS].bins[5 * 36 + 4], 25);
    EXPECT_EQ(recorder.ranges[SwarmRecorder::kUAT].bins[5 * 36 + 4], 15);
    EXPECT_TRUE(recorder.ranges[SwarmRecorder::kRemoteID].Empty());

    SwarmRecorder later;
    later.RecordPosition(SwarmRecorder::kUAT, 40000, DegToBam16(45.0f), 12000, 2);
    later.RecordDrone(500, 0, 50, 3);
    recorder.Merge(later);
    EXPECT_EQ(recorder.ranges[SwarmRecorder::kModeS].bins[5 * 36 + 4], 25);
    EXPECT_EQ(recorder.ranges[SwarmRecorder::kUAT].bins[5 * 36 + 4], 20);
    EXPECT_EQ(recorder.ranges[SwarmRecorder::kRemoteID].bins[0], 10);
    EXPECT_FALSE(recorder.RangeEmpty());
    recorder.Clear();
    EXPECT_TRUE(recorder.RangeEmpty());
}

TEST(SwarmRecorder, DroneHorizonLimitPerBand) {
    for (int32_t rx_m : {-50, 0, 30, 1000}) {
        SwarmRecorder recorder;
        recorder.SetReceiverAltitude(rx_m);
        int32_t rx_ft = static_cast<int32_t>(lround(rx_m / 0.3048));
        for (uint8_t band = 0; band < RangeMap::kNumBands; band++) {
            uint32_t limit_m = static_cast<uint32_t>(HorizonKm(rx_m, SwarmRecorder::kDroneBandTopFt[band]) * 1000);
            int32_t height_ft = band == 0 ? 0 : SwarmRecorder::kDroneBandTopFt[band - 1];  // Bottom of the band.
            recorder.guard = SwarmRecorder::Guard();
            recorder.RecordDrone(limit_m - 100, 0, rx_ft + height_ft, 1);
            recorder.RecordDrone(limit_m + 100, 0, rx_ft + height_ft, 2);
            EXPECT_EQ(recorder.guard.num_positions, 2u) << rx_m << " m, band " << int(band);
            EXPECT_EQ(recorder.guard.num_dropped, 1u) << rx_m << " m, band " << int(band);
        }
    }
    // 30 m antenna, drone 100-200 ft up: 1.15 * 4.12 * (sqrt(30) + sqrt(61)) + 20 = 83 km.
    SwarmRecorder recorder;
    recorder.SetReceiverAltitude(30);
    recorder.RecordDrone(82000, 0, 98 + 150, 1);
    recorder.RecordDrone(84000, 0, 98 + 150, 2);
    EXPECT_EQ(recorder.guard.num_dropped, 1u);
    EXPECT_EQ(recorder.ranges[SwarmRecorder::kRemoteID].bins[36], 255);  // Kept, clamped to 12.75 km.
}

TEST(SwarmRecorder, DroneDropsCountTowardsWithholding) {
    // The guard is shared: 4 aircraft and 1 drone dropped make the 5 that withhold the map.
    SwarmRecorder recorder;
    recorder.SetReceiverAltitude(0);
    for (int i = 0; i < 4; i++) recorder.RecordPosition(SwarmRecorder::kModeS, 400000, 0, 500, 1 + i);
    for (int i = 0; i < 20; i++) recorder.RecordPosition(SwarmRecorder::kUAT, 10000, 0, 30000, 100);
    EXPECT_FALSE(recorder.RangeImplausible());
    recorder.RecordDrone(200000, 0, 100, Aircraft::ICAOToUID(5, Aircraft::kAircraftTypeRemoteID));
    EXPECT_EQ(recorder.guard.num_dropped, 5u);
    EXPECT_EQ(recorder.guard.num_dropped_aircraft, 5);
    EXPECT_TRUE(recorder.RangeImplausible());
}

TEST(SwarmReporter, RangesPerProtocolInOrderWithTheirUnits) {
    FakeSwarmPlatform platform;
    SwarmReporter reporter(platform, "0.9.1-rc10");
    SwarmRecorder live;
    live.SetReceiverAltitude(30);
    live.RecordPosition(SwarmRecorder::kModeS, 50000, DegToBam16(45.0f), 12000, 1);  // Bin 184: 25.
    live.RecordDrone(1234, DegToBam16(95.0f), 98 + 150, 3);                           // Bin 45: 25.
    live.RecordDrone(60, DegToBam16(5.0f), 98, 3);                                    // Bin 0: 2.
    reporter.TakePeriod(live, 3600 * 1000, FixedPosition());
    reporter.Send("https://adsbee.aero", kToken);
    std::string body = platform.bodies.back();
    size_t ranges = body.find(",\"range_dropped\":0,\"ranges\":{\"mode_s\":{\"sectors\":36,\"bands\":12,\"unit_m\":2000,");
    ASSERT_NE(ranges, std::string::npos) << body;
    size_t remote_id = body.find("},\"remote_id\":{\"sectors\":36,\"bands\":12,\"unit_m\":50,\"changed\":\"");
    ASSERT_NE(remote_id, std::string::npos) << body;
    EXPECT_EQ(body.find("\"uat\""), std::string::npos);
    EXPECT_EQ(body.find("\"range\":"), std::string::npos);
    EXPECT_EQ(body.compare(body.size() - 4, 4, "\"}}}"), 0) << body;  // ranges is the last field.

    std::string mode_s = body.substr(ranges, remote_id - ranges);
    std::vector<uint8_t> values = Base64Decode(JsonString(mode_s, "max"));
    ASSERT_EQ(values, std::vector<uint8_t>({25}));
    std::string drones = body.substr(remote_id);
    std::vector<uint8_t> bitmap = Base64Decode(JsonString(drones, "changed"));
    values = Base64Decode(JsonString(drones, "max"));
    ASSERT_EQ(bitmap.size(), 54u);
    EXPECT_EQ(bitmap[0], 0x01);
    EXPECT_EQ(bitmap[45 / 8], 1 << (45 % 8));
    EXPECT_EQ(values, std::vector<uint8_t>({2, 25}));

    // UAT only.
    live.RecordPosition(SwarmRecorder::kUAT, 2001, 0, 0, 2);
    reporter.TakePeriod(live, 7200 * 1000, FixedPosition());
    reporter.Send("https://adsbee.aero", kToken);
    body = platform.bodies.back();
    EXPECT_NE(body.find(",\"ranges\":{\"uat\":{\"sectors\":36,\"bands\":12,\"unit_m\":2000,\"changed\":\"AQAA"),
              std::string::npos)
        << body;
    EXPECT_EQ(body.find("mode_s"), std::string::npos);
}

TEST(SwarmReporter, FailedReportMergesEveryProtocol) {
    FakeSwarmPlatform platform;
    platform.answer = false;
    SwarmReporter reporter(platform, "0.9.1-rc10");
    SwarmRecorder live;
    live.RecordPosition(SwarmRecorder::kModeS, 10000, 0, 500, 1);
    live.RecordDrone(100, 0, 0, 2);
    reporter.TakePeriod(live, 60000, FixedPosition());
    reporter.Send("https://adsbee.aero", kToken);
    live.RecordPosition(SwarmRecorder::kUAT, 20000, 0, 500, 3);
    live.RecordDrone(300, 0, 0, 2);
    reporter.TakePeriod(live, 120000, FixedPosition());
    EXPECT_EQ(reporter.pending().ranges[SwarmRecorder::kModeS].bins[0], 5);
    EXPECT_EQ(reporter.pending().ranges[SwarmRecorder::kUAT].bins[0], 10);
    EXPECT_EQ(reporter.pending().ranges[SwarmRecorder::kRemoteID].bins[0], 6);
    platform.answer = true;
    reporter.Send("https://adsbee.aero", kToken);
    const std::string& body = platform.bodies.back();
    EXPECT_NE(body.find("\"mode_s\":{"), std::string::npos);
    EXPECT_NE(body.find("\"uat\":{"), std::string::npos);
    EXPECT_NE(body.find("\"remote_id\":{"), std::string::npos);
    EXPECT_EQ(JsonNumber(body, "seq"), 0);
}

namespace {

// A basic UAT ADS-B message (UAT Tech Manual Table 2-11) from an ICAO-addressed aircraft at a baro altitude.
DecodedUATADSBPacket UATPosition(uint32_t address, double lat, double lon, int32_t altitude_ft) {
    uint8_t buf[RawUATADSBPacket::kShortADSBMessageNumBytes] = {0};
    auto set_bits = [&](uint32_t first_bit, uint16_t n, uint32_t value) {
        for (uint16_t i = 0; i < n; i++) {
            uint32_t bit = first_bit + i;
            if ((value >> (n - 1 - i)) & 1) buf[bit / 8] |= 0x80 >> (bit % 8);
        }
    };
    set_bits(8, 24, address);
    set_bits(32, 23, static_cast<uint32_t>(llround(lat * 16777216.0 / 360.0)) & 0x7FFFFF);
    set_bits(55, 24, static_cast<uint32_t>(llround(lon * 16777216.0 / 360.0)) & 0xFFFFFF);
    set_bits(80, 12, (altitude_ft + 1000) / 25 + 1);
    set_bits(92, 4, 8);  // NIC.
    uat_rs.EncodeShortADSBMessage(buf);
    return DecodedUATADSBPacket(RawUATADSBPacket(buf, sizeof(buf)));
}

// One Open Drone ID Location message, as a Bluetooth 4 advertisement from mac.
RawRemoteIDPacket DroneLocation(uint8_t mac_last, double lat, double lon, float geodetic_altitude_m) {
    ODID_Location_data location;
    odid_initLocationData(&location);
    location.Status = ODID_STATUS_AIRBORNE;
    location.Latitude = lat;
    location.Longitude = lon;
    location.AltitudeGeo = geodetic_altitude_m;
    RawRemoteIDPacket packet;
    const uint8_t mac[6] = {0x02, 0x11, 0x22, 0x33, 0x44, mac_last};
    memcpy(packet.source_mac, mac, sizeof(mac));
    packet.transport = RawRemoteIDPacket::kTransportBT4Legacy;
    packet.rssi_dbm = -60;
    EXPECT_EQ(encodeLocationMessage(reinterpret_cast<ODID_Location_encoded*>(packet.payload), &location), ODID_SUCCESS);
    packet.payload_len_bytes = RawRemoteIDPacket::kSingleMessageLenBytes;
    return packet;
}

int NumBins(const RangeMap& map) {
    int n = 0;
    for (uint8_t b : map.bins) n += b != 0;
    return n;
}

}  // namespace

TEST(SwarmRecorder, DictionaryRecordsUATPositions) {
    AircraftDictionary dictionary;
    SwarmRecorder recorder;
    dictionary.SetSwarmRecorder(&recorder);
    dictionary.SetReferencePosition(37.0f, -122.0f);
    inc_time_since_boot_ms(1000);
    ASSERT_TRUE(dictionary.IngestDecodedUATADSBPacket(UATPosition(0xA12345, 37.3, -122.0, 12000)));
    dictionary.Update(get_time_since_boot_ms());
    EXPECT_TRUE(recorder.RangeEmpty());  // One decode.
    inc_time_since_boot_ms(1000);
    ASSERT_TRUE(dictionary.IngestDecodedUATADSBPacket(UATPosition(0xA12345, 37.31, -122.0, 12000)));
    dictionary.Update(get_time_since_boot_ms());
    EXPECT_TRUE(recorder.ranges[SwarmRecorder::kModeS].Empty());
    EXPECT_TRUE(recorder.ranges[SwarmRecorder::kRemoteID].Empty());
    ASSERT_EQ(NumBins(recorder.ranges[SwarmRecorder::kUAT]), 1);
    EXPECT_EQ(recorder.ranges[SwarmRecorder::kUAT].bins[5 * 36 + 0], 18);  // 34.5 km north at 12,000 ft.
    dictionary.SetSwarmRecorder(nullptr);
}

TEST(SwarmRecorder, DictionaryRecordsDronesAfterTwoLocationMessages) {
    AircraftDictionary dictionary;
    SwarmRecorder recorder;
    recorder.SetReceiverAltitude(30);  // 98 ft.
    dictionary.SetSwarmRecorder(&recorder);
    dictionary.SetReferencePosition(37.0f, -122.0f);

    // Hovering 1.11 km north, 120 m geodetic (= 296 ft above the receiver): band 2.
    inc_time_since_boot_ms(100);
    ASSERT_TRUE(dictionary.IngestRawRemoteIDPacket(DroneLocation(1, 37.01, -122.0, 120)));
    dictionary.Update(get_time_since_boot_ms());
    EXPECT_TRUE(recorder.RangeEmpty());  // One decode.
    inc_time_since_boot_ms(100);
    ASSERT_TRUE(dictionary.IngestRawRemoteIDPacket(DroneLocation(1, 37.01, -122.0, 120)));  // Same position.
    const RangeMap& drones = recorder.ranges[SwarmRecorder::kRemoteID];
    ASSERT_EQ(NumBins(drones), 1);
    EXPECT_EQ(drones.bins[2 * 36 + 0], 23);  // 1112 m.
    EXPECT_TRUE(recorder.ranges[SwarmRecorder::kModeS].Empty());
    EXPECT_TRUE(recorder.ranges[SwarmRecorder::kUAT].Empty());

    // Next period: still hovering, recorded again from its next message.
    recorder.Clear();
    inc_time_since_boot_ms(100);
    dictionary.Update(get_time_since_boot_ms());
    EXPECT_TRUE(recorder.RangeEmpty());
    ASSERT_TRUE(dictionary.IngestRawRemoteIDPacket(DroneLocation(1, 37.01, -122.0, 120)));
    ASSERT_EQ(NumBins(drones), 1);
    EXPECT_EQ(drones.bins[2 * 36 + 0], 23);

    // Flies on: the distance is recomputed once a second and recorded.
    inc_time_since_boot_ms(100);
    ASSERT_TRUE(dictionary.IngestRawRemoteIDPacket(DroneLocation(1, 37.02, -122.0, 120)));
    dictionary.Update(get_time_since_boot_ms());
    EXPECT_EQ(drones.bins[2 * 36 + 0], 45);  // 2224 m.

    // Without a geodetic altitude: left out.
    for (int i = 0; i < 3; i++) {
        inc_time_since_boot_ms(100);
        ASSERT_TRUE(dictionary.IngestRawRemoteIDPacket(DroneLocation(2, 37.0, -121.99, INV_ALT)));
    }
    dictionary.Update(get_time_since_boot_ms());
    EXPECT_EQ(NumBins(drones), 1);

    // Drones don't count as aircraft in the stats.
    EXPECT_EQ(recorder.stats.aircraft_max, 0);
    EXPECT_EQ(recorder.UniqueAircraft(), 0);
    dictionary.SetSwarmRecorder(nullptr);
}

namespace {

std::string ReadFile(const std::string& path) {
    FILE* f = fopen(path.c_str(), "rb");
    if (f == nullptr) return "";
    std::string text;
    char buf[1024];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) text.append(buf, n);
    fclose(f);
    return text;
}

void FillBins(RangeMap& map, int n) {
    for (int i = 0; i < n; i++) map.bins[(i * 7) % RangeMap::kNumBins] = 10 + i % 200;
}

// A typical hour (design t-1057): bins per protocol and frame counts.
struct BodyCase {
    const char* name;
    int mode_s_bins, uat_bins, remote_id_bins;
    uint32_t modes_valid, es_valid, uat_adsb, uat_uplink;
    bool position;
};
const BodyCase kBodyCases[] = {
    {"a_europe", 150, 0, 0, 1200000, 500000, 0, 0, true},
    {"b_us", 150, 40, 5, 1200000, 500000, 30000, 8000, true},
    {"c_uat_only", 0, 40, 0, 0, 0, 30000, 8000, true},
    {"d_no_position", 150, 0, 0, 1200000, 500000, 0, 0, false},
    {"e_nothing", 0, 0, 0, 0, 0, 0, 0, true},
    {"f_full", 432, 432, 432, 1200000, 500000, 30000, 8000, true},
};

class GoldenPlatform : public FakeSwarmPlatform {
   public:
    GoldenPlatform() { now_ms = 7200000; }
    bool GetIdentity(AutoUpdatePlatform::Identity& identity) override {
        FakeSwarmPlatform::GetIdentity(identity);
        snprintf(identity.mac_address, sizeof(identity.mac_address), "DC:54:75:D2:6C:24");
        return true;
    }
};

// Fills live like an hour of the case, optionally with listen time.
void FillCase(SwarmRecorder& live, const BodyCase& c) {
    if (c.mode_s_bins) FillBins(live.ranges[SwarmRecorder::kModeS], c.mode_s_bins);
    if (c.uat_bins) FillBins(live.ranges[SwarmRecorder::kUAT], c.uat_bins);
    if (c.remote_id_bins) FillBins(live.ranges[SwarmRecorder::kRemoteID], c.remote_id_bins);
    live.AddFrames(c.modes_valid, c.es_valid, c.uat_adsb, c.uat_uplink);
    live.NoteAircraftCount(40);
    for (uint32_t a = 0; a < 300; a++) live.MarkAircraft(0xA00000 + a);
}

SwarmReporter::DeviceState CaseState(bool position) {
    SwarmReporter::DeviceState s;
    s.auto_update = true;
    s.rx_position_valid = position;
    s.range_valid = position;
    s.latitude_deg = 37.1234f;
    s.longitude_deg = -122.1234f;
    s.altitude_m = 30;
    s.rx_position_source = "fixed";
    return s;
}

std::string Body(const SwarmReporter& reporter, bool ota_keys) {
    GoldenPlatform platform;
    AutoUpdatePlatform::Identity identity;
    platform.GetIdentity(identity);
    char buf[SwarmReporter::kBodyMaxLen + 1];
    size_t len = reporter.BuildBody(buf, sizeof(buf), identity, ota_keys, 7200);
    return std::string(buf, len);
}

SwarmListenState ListenAll() {
    SwarmListenState s;
    s.r1090_rx_enabled = true;
    s.subg_rx_enabled = true;
    s.subg_mode = SettingsManager::kSubGHzRadioModeUATRx;
    s.remote_id_status = RemoteIDGuard::kStatusBLEActive;
    return s;
}

std::string JsonObject(const std::string& json, const std::string& key) {
    size_t pos = json.find("\"" + key + "\":{");
    if (pos == std::string::npos) return "";
    pos += key.size() + 3;
    return json.substr(pos, json.find('}', pos) - pos + 1);
}

}  // namespace

TEST(SwarmReporter, BodiesWithoutListenTimeMatchMainByteForByte) {
    for (const BodyCase& c : kBodyCases) {
        GoldenPlatform platform;
        SwarmReporter reporter(platform, "0.9.1-rc11");
        SwarmRecorder live;
        FillCase(live, c);
        reporter.TakePeriod(live, 3600000, CaseState(c.position));
        for (bool ota : {false, true}) {
            std::string path = std::string(SWARM_BODIES_DIR) + "/" + c.name + (ota ? "_ota" : "") + ".json";
            std::string golden = ReadFile(path);
            ASSERT_FALSE(golden.empty()) << path;
            EXPECT_EQ(Body(reporter, ota), golden) << path;
        }
    }
}

TEST(SwarmRecorder, RegistryMatchesTheV1Tables) {
    ASSERT_EQ(kNumSwarmProtocols, 3);  // ADS-L comes in phase 3.
    EXPECT_STREQ(kSwarmProtocols[SwarmRecorder::kModeS].name, "mode_s");
    EXPECT_STREQ(kSwarmProtocols[SwarmRecorder::kUAT].name, "uat");
    EXPECT_STREQ(kSwarmProtocols[SwarmRecorder::kRemoteID].name, "remote_id");
    EXPECT_EQ(kSwarmProtocols[SwarmRecorder::kModeS].unit_m, 2000u);
    EXPECT_EQ(kSwarmProtocols[SwarmRecorder::kRemoteID].unit_m, 50u);
    EXPECT_EQ(AircraftDictionary::ProtocolOf<ModeSAircraft>(), SwarmRecorder::kModeS);
    EXPECT_EQ(AircraftDictionary::ProtocolOf<UATAircraft>(), SwarmRecorder::kUAT);
    EXPECT_EQ(AircraftDictionary::ProtocolOf<RemoteIDAircraft>(), SwarmRecorder::kRemoteID);
    EXPECT_EQ(kSwarmV1ProtocolMask, 0b111u);
    EXPECT_EQ(SwarmProtocolIndex("ads_l"), kNoSwarmProtocol);
}

// The value after "key": (any spacing) up to the next , } or ] at the same level, as text.
std::string JsonField(const std::string& json, const std::string& key) {
    size_t pos = json.find("\"" + key + "\":");
    if (pos == std::string::npos) return "";
    pos += key.size() + 3;
    while (json[pos] == ' ') pos++;
    size_t end = pos;
    int depth = 0;
    for (; end < json.size(); end++) {
        char c = json[end];
        if (c == '[' || c == '{') depth++;
        if ((c == ']' || c == '}') && depth-- == 0) break;
        if (c == ',' && depth == 0) break;
    }
    return json.substr(pos, end - pos);
}

// The registry and band tables against the server's table (software/adsbee_swarm/adsbee_swarm/protocols.json).
TEST(SwarmRecorder, RegistryMatchesServerProtocolsJson) {
    std::string json = ReadFile(std::string(SWARM_BODIES_DIR) +
                                "/../../../../../software/adsbee_swarm/adsbee_swarm/protocols.json");
    if (json.empty()) GTEST_SKIP() << "No protocols.json in this tree yet.";
    EXPECT_EQ(JsonField(json, "sectors"), std::to_string(RangeMap::kNumSectors));
    EXPECT_EQ(JsonField(json, "bands"), std::to_string(RangeMap::kNumBands));
    std::vector<std::string> reported;
    for (size_t pos = json.find("{\"name\""); pos != std::string::npos; pos = json.find("{\"name\"", pos + 1)) {
        std::string entry = json.substr(pos, json.find('}', pos) - pos + 1);
        if (JsonField(entry, "reported") != "true") continue;
        std::string name = JsonField(entry, "name");
        name = name.substr(1, name.size() - 2);
        reported.push_back(name);
        uint8_t p = SwarmProtocolIndex(name.c_str());
        ASSERT_NE(p, kNoSwarmProtocol) << name;
        const SwarmProtocol& info = kSwarmProtocols[p];
        EXPECT_EQ(JsonField(entry, "unit_m"), std::to_string(info.unit_m)) << name;
        EXPECT_EQ(JsonField(entry, "basis"),
                  info.basis == SwarmBandBasis::kAltitude ? "\"altitude\"" : "\"height_above_rx\"")
            << name;
        EXPECT_EQ(JsonField(entry, "aircraft"), info.counts_as_aircraft ? "true" : "false") << name;
        const int32_t* tops = SwarmRecorder::kBandTopFtByBasis[static_cast<uint8_t>(info.basis)];
        std::string floors = "[0";
        for (uint16_t b = 0; b + 1 < RangeMap::kNumBands; b++) floors += ", " + std::to_string(tops[b]);
        EXPECT_EQ(JsonField(entry, "floors_ft"), floors + "]") << name;
        EXPECT_EQ(JsonField(entry, "top_ft"), std::to_string(tops[RangeMap::kNumBands - 1])) << name;
        std::string stats = "[";
        for (const SwarmProtocol::V1Stat& v1 : info.v1_stats) {
            if (v1.key) stats += std::string(stats.size() > 1 ? ", " : "") + "\"" + v1.key + "\"";
        }
        EXPECT_EQ(JsonField(entry, "stats"), stats + "]") << name;
    }
    ASSERT_EQ(reported.size(), kNumSwarmProtocols);  // Same set, same order.
    for (uint8_t p = 0; p < kNumSwarmProtocols; p++) EXPECT_EQ(reported[p], kSwarmProtocols[p].name);
}

TEST(SwarmRecorder, FourthMapComesFromTheHeap) {
    SwarmRecorder recorder(2);  // Two pool slots: the third protocol heard needs the heap.
    recorder.RecordPosition(SwarmRecorder::kModeS, 10000, 0, 500, 1);
    recorder.RecordPosition(SwarmRecorder::kUAT, 20000, 0, 500, 2);
    recorder.RecordDrone(1000, 0, 100, 3);
    EXPECT_EQ(recorder.ranges[SwarmRecorder::kModeS].bins[0], 5);
    EXPECT_EQ(recorder.ranges[SwarmRecorder::kUAT].bins[0], 10);
    EXPECT_EQ(recorder.ranges[SwarmRecorder::kRemoteID].bins[1 * 36], 20);
    EXPECT_EQ(recorder.guard.num_no_map, 0u);

    // Copies and moves keep every map; merging adds them.
    SwarmRecorder copy = recorder;
    EXPECT_EQ(copy.ranges[SwarmRecorder::kRemoteID].bins[1 * 36], 20);
    SwarmRecorder moved(2);
    moved = static_cast<SwarmRecorder&&>(copy);
    EXPECT_EQ(moved.ranges[SwarmRecorder::kRemoteID].bins[1 * 36], 20);
    EXPECT_EQ(copy.ranges.Find(SwarmRecorder::kRemoteID), nullptr);
    moved.Merge(recorder);
    EXPECT_EQ(moved.ranges[SwarmRecorder::kRemoteID].bins[1 * 36], 20);

    // The reporter sends the heap map like any other.
    FakeSwarmPlatform platform;
    SwarmReporter reporter(platform, "0.9.1");
    SwarmRecorder live;
    live.Merge(recorder);
    reporter.TakePeriod(live, 3600000, FixedPosition());
    reporter.Send("https://swarm.example", kToken);
    EXPECT_NE(platform.bodies[0].find("\"remote_id\":{"), std::string::npos);
    recorder.Clear();
    EXPECT_EQ(recorder.ranges.Find(SwarmRecorder::kRemoteID), nullptr);
}

TEST(SwarmRecorder, NoHeapForAMapCountsAsDropped) {
    RangeMap* (*allocate)() = SwarmRangeMaps::allocate;
    SwarmRangeMaps::allocate = []() -> RangeMap* { return nullptr; };
    SwarmRecorder recorder(2);
    recorder.RecordPosition(SwarmRecorder::kModeS, 10000, 0, 500, 1);
    recorder.RecordPosition(SwarmRecorder::kUAT, 20000, 0, 500, 2);
    for (int i = 0; i < 3; i++) recorder.RecordDrone(1000, 0, 100, 3);
    EXPECT_EQ(recorder.ranges.Find(SwarmRecorder::kRemoteID), nullptr);
    EXPECT_EQ(recorder.guard.num_no_map, 3u);
    EXPECT_EQ(recorder.guard.num_dropped, 0u);  // Not a horizon drop: doesn't count towards withholding.
    EXPECT_FALSE(recorder.RangeImplausible());

    // Merging a map the heap can't hold counts its bins.
    SwarmRecorder other(2);
    SwarmRangeMaps::allocate = allocate;
    other.RecordDrone(1000, 0, 100, 3);
    other.RecordDrone(1000, DegToBam16(90.0f), 100, 3);
    SwarmRangeMaps::allocate = []() -> RangeMap* { return nullptr; };
    recorder.Merge(other);
    EXPECT_EQ(recorder.guard.num_no_map, 5u);
    SwarmRangeMaps::allocate = allocate;

    // range_dropped reports them; the other maps still go.
    FakeSwarmPlatform platform;
    SwarmReporter reporter(platform, "0.9.1");
    SwarmRecorder live;
    live.Merge(recorder);
    reporter.TakePeriod(live, 3600000, FixedPosition());
    reporter.Send("https://swarm.example", kToken);
    EXPECT_EQ(JsonNumber(platform.bodies[0], "range_dropped"), 5);
    EXPECT_NE(platform.bodies[0].find("\"mode_s\":{"), std::string::npos);
}

TEST(SwarmReporter, OldServerReplyKeepsTheV1Set) {
    FakeSwarmPlatform platform;
    SwarmReporter reporter(platform, "0.9.1");
    EXPECT_EQ(reporter.accepted_protocols(), kSwarmV1ProtocolMask);
    SwarmRecorder live;
    live.RecordPosition(SwarmRecorder::kModeS, 10000, 0, 500, 1);
    live.RecordDrone(1000, 0, 100, 3);
    reporter.TakePeriod(live, 3600000, FixedPosition());
    reporter.Send("https://swarm.example", kToken);  // Today's reply: no "protocols".
    EXPECT_STREQ(reporter.status().result, "ok");
    EXPECT_EQ(reporter.accepted_protocols(), kSwarmV1ProtocolMask);
    live.RecordPosition(SwarmRecorder::kModeS, 10000, 0, 500, 1);
    live.RecordDrone(1000, 0, 100, 3);
    reporter.TakePeriod(live, 7200000, FixedPosition());
    reporter.Send("https://swarm.example", kToken);
    EXPECT_NE(platform.bodies[1].find("\"mode_s\":{"), std::string::npos);
    EXPECT_NE(platform.bodies[1].find("\"remote_id\":{"), std::string::npos);
}

TEST(SwarmReporter, ReplyProtocolsGateTheMaps) {
    FakeSwarmPlatform platform;
    platform.response = "{\"next_report_s\": 3600, \"firmware\": null, \"protocols\": [\"mode_s\", \"ads_l\", \"uat\"]}";
    SwarmReporter reporter(platform, "0.9.1");
    SwarmRecorder live;
    reporter.TakePeriod(live, 3600000, FixedPosition());
    reporter.Send("https://swarm.example", kToken);
    // Unknown names skipped; Remote ID not listed.
    EXPECT_EQ(reporter.accepted_protocols(), (1u << SwarmRecorder::kModeS) | (1u << SwarmRecorder::kUAT));

    live.AddListening(ListenAll(), 3600);
    live.RecordPosition(SwarmRecorder::kModeS, 10000, 0, 500, 1);
    live.RecordDrone(1000, 0, 100, 3);
    reporter.TakePeriod(live, 7200000, FixedPosition());
    platform.response = "{\"next_report_s\": 3600, \"protocols\": []}";
    reporter.Send("https://swarm.example", kToken);
    const std::string& body = platform.bodies[1];
    EXPECT_NE(body.find("\"mode_s\":{"), std::string::npos);
    EXPECT_EQ(body.find("\"remote_id\":{"), std::string::npos);
    EXPECT_EQ(JsonNumber(JsonObject(body, "listen_s"), "remote_id"), 3600);  // Its listen time still goes.
    EXPECT_EQ(reporter.accepted_protocols(), 0u);  // An empty list: no maps at all.

    // A reply without the list, or a new server, brings back the v1 set.
    platform.response = "{\"next_report_s\": 3600}";
    reporter.TakePeriod(live, 10800000, FixedPosition());
    reporter.Send("https://swarm.example", kToken);
    EXPECT_EQ(reporter.accepted_protocols(), kSwarmV1ProtocolMask);
    platform.response = "{\"protocols\": [\"mode_s\"]}";
    reporter.TakePeriod(live, 14400000, FixedPosition());
    reporter.Send("https://swarm.example", kToken);
    EXPECT_EQ(reporter.accepted_protocols(), 1u << SwarmRecorder::kModeS);
    reporter.Reconfigure();
    EXPECT_EQ(reporter.accepted_protocols(), kSwarmV1ProtocolMask);

    // Errors keep the list.
    platform.response = "{\"protocols\": [\"mode_s\"]}";
    reporter.TakePeriod(live, 18000000, FixedPosition());
    reporter.Send("https://swarm.example", kToken);
    platform.status = 503;
    reporter.TakePeriod(live, 21600000, FixedPosition());
    reporter.Send("https://swarm.example", kToken);
    EXPECT_EQ(reporter.accepted_protocols(), 1u << SwarmRecorder::kModeS);
}

TEST(SwarmRecorder, ListenTimeFollowsTheReceivers) {
    SwarmListenState s = ListenAll();
    SwarmRecorder r;
    EXPECT_FALSE(r.listen_known);
    r.AddListening(s, 1800);
    s.subg_mode = static_cast<SettingsManager::SubGHzRadioMode>(1);  // Another sub-GHz mode: UAT stops.
    s.remote_id_status = RemoteIDGuard::kStatusBlockedByRAM;           // Requested, not running.
    r.AddListening(s, 1200);
    s.subg_mode = SettingsManager::kSubGHzRadioModeUATRx;
    s.subg_rx_enabled = false;
    s.remote_id_status = RemoteIDGuard::kStatusWiFiSnifferActive;
    r.AddListening(s, 300);
    s.r1090_rx_enabled = false;
    r.AddListening(s, 300);
    EXPECT_TRUE(r.listen_known);
    EXPECT_EQ(r.listen_s[SwarmRecorder::kModeS], 3300u);
    EXPECT_EQ(r.listen_s[SwarmRecorder::kUAT], 1800u);
    EXPECT_EQ(r.listen_s[SwarmRecorder::kRemoteID], 2400u);

    // A retry sends the merged totals.
    SwarmRecorder later;
    later.AddListening(ListenAll(), 3600);
    r.Merge(later);
    EXPECT_EQ(r.listen_s[SwarmRecorder::kUAT], 5400u);
    r.Clear();
    EXPECT_FALSE(r.listen_known);
    EXPECT_EQ(r.listen_s[SwarmRecorder::kModeS], 0u);
}

TEST(SwarmReporter, ListenTimeAcrossAModeSwitch) {
    FakeSwarmPlatform platform;
    SwarmReporter reporter(platform, "0.9.1");
    SwarmRecorder live;
    SwarmListenState s = ListenAll();
    s.remote_id_status = 0;
    live.AddListening(s, 1800);
    live.RecordPosition(SwarmRecorder::kUAT, 20000, 0, 500, 2);
    live.AddFrames(1000, 500, 300, 20);
    s.subg_mode = static_cast<SettingsManager::SubGHzRadioMode>(1);
    live.AddListening(s, 1800);
    reporter.TakePeriod(live, 3600000, FixedPosition());
    reporter.Send("https://swarm.example", kToken);
    const std::string& body = platform.bodies[0];
    EXPECT_EQ(JsonObject(body, "listen_s"), "{\"mode_s\":3600,\"uat\":1800}");
    EXPECT_NE(JsonObject(body, "stats").find("\"uat_adsb\":300,\"uat_uplink\":20,"), std::string::npos);
    EXPECT_NE(body.find("\"uat\":{\"sectors\""), std::string::npos);
    EXPECT_EQ(body.find("\"frames\""), std::string::npos);  // No protocol without v1 stats keys counts frames yet.
}

TEST(SwarmReporter, EuropeUnitSendsNoUATKeys) {
    FakeSwarmPlatform platform;
    SwarmReporter reporter(platform, "0.9.1");
    SwarmRecorder live;
    SwarmListenState s = ListenAll();
    s.subg_rx_enabled = false;
    s.remote_id_status = 0;
    live.AddListening(s, 3600);
    live.AddFrames(1200000, 500000, 0, 0);
    live.RecordPosition(SwarmRecorder::kModeS, 10000, 0, 500, 1);
    reporter.TakePeriod(live, 3600000, FixedPosition());
    reporter.Send("https://swarm.example", kToken);
    const std::string& body = platform.bodies[0];
    EXPECT_EQ(JsonObject(body, "listen_s"), "{\"mode_s\":3600}");
    EXPECT_EQ(JsonObject(body, "stats"), "{\"modes_valid\":1200000,\"es_valid\":500000,\"aircraft_max\":0,\"aircraft_unique\":0}");
    EXPECT_EQ(body.find("uat"), std::string::npos);
    EXPECT_EQ(body.find("remote_id"), std::string::npos);
}

TEST(SwarmReporter, ListenedAndHeardNothingStillReports) {
    FakeSwarmPlatform platform;
    SwarmReporter reporter(platform, "0.9.1");
    SwarmRecorder live;
    live.AddListening(ListenAll(), 3600);
    reporter.TakePeriod(live, 3600000, FixedPosition());
    reporter.Send("https://swarm.example", kToken);
    const std::string& body = platform.bodies[0];
    EXPECT_EQ(JsonObject(body, "listen_s"), "{\"mode_s\":3600,\"uat\":3600,\"remote_id\":3600}");
    EXPECT_EQ(JsonObject(body, "stats"),
              "{\"modes_valid\":0,\"es_valid\":0,\"uat_adsb\":0,\"uat_uplink\":0,\"aircraft_max\":0,"
              "\"aircraft_unique\":0}");
    EXPECT_NE(body.find("\"ranges\":{}"), std::string::npos);
}

TEST(SwarmReporter, ListenedBodiesFitAndMeasureExactly) {
    for (const BodyCase& c : kBodyCases) {
        GoldenPlatform platform;
        SwarmReporter reporter(platform, "0.9.1-rc11");
        SwarmRecorder live;
        FillCase(live, c);
        live.AddListening(ListenAll(), 3600);
        reporter.TakePeriod(live, 3600000, CaseState(c.position));
        AutoUpdatePlatform::Identity identity;
        platform.GetIdentity(identity);
        size_t measured = reporter.BuildBody(nullptr, 0, identity, true, 7200);
        std::string body = Body(reporter, true);
        EXPECT_EQ(measured, body.size()) << c.name;
        EXPECT_GT(body.size(), 0u) << c.name;
        EXPECT_LE(body.size(), SwarmReporter::kBodyMaxLen) << c.name;
        printf("%-14s listened: %4zu B with OTA keys\n", c.name, body.size());
    }
}
