// GPS time sync frame 0xe6 (capability bit 0): the encoder against the spec's vectors (Beast GPS time sync spec v2,
// Part 3, approved 2026-11-26; the same hex as worker-6's gps_time_vectors.json), which feeds get it, once per seq.
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <cstring>

#include "beast_utils.hh"
#include "gtest/gtest.h"

using namespace BeastReporter;

namespace {

std::string ToHex(const uint8_t* buf, uint16_t len) {
    static const char* kDigits = "0123456789abcdef";
    std::string out;
    for (uint16_t i = 0; i < len; i++) {
        out += kDigits[buf[i] >> 4];
        out += kDigits[buf[i] & 0xF];
    }
    return out;
}

// A PPSTime (ObjectDictionary::PPSTime's fields) whose edge is at 12 MHz tick ts + sub1 / 256.
struct PPSTime {
    uint64_t edge_mlat_48mhz_counts = 0;
    uint16_t edge_sub_counts = 0;
    uint64_t utc_seconds = 0;
    int32_t rate_ppb = 0;
    uint8_t step = 0;
    uint8_t seq = 0;
};

PPSTime Edge(uint64_t ts, uint8_t sub1, uint64_t utc, int32_t rate_ppb, bool step, uint8_t seq = 1) {
    uint64_t e = (ts << 12) | (static_cast<uint64_t>(sub1) << 4);  // In 1/1024 of a 48 MHz count.
    return {e >> 10, static_cast<uint16_t>(e & 0x3FF), utc, rate_ppb, static_cast<uint8_t>(step), seq};
}

std::string Frame(const PPSTime& t) {
    uint8_t buf[kGPSTimeBeastFrameMaxLenBytes];
    return ToHex(buf, WriteGPSTimeFrame(buf, t.edge_mlat_48mhz_counts, t.edge_sub_counts, t.utc_seconds, t.rate_ppb,
                                        t.step));
}

// Spec Part 3 (and gps_time_vectors.json).
const char* kVectorGPSTime = "1ae65000123456789b80006b04e82d80004394";
const char* kVectorEscapedStep = "1ae65000001a1a1a1a001a1a1a1a006b04e82e81001a1a1a1a";
const char* kVectorRateUnknown = "1ae650001235c4b03ab3006b04e82f80800000";
const char* kVectorStreamFeeder =  // stream_query_answer_gps_time, the feeder's frames after the answer.
    "1a320012345a0000045d4840d6abcdef1ae65000123456789b80006b04e82d800043941a32001234600000045d4840d6abcdef";

struct Sink {
    uint16_t sink;
    int protocol;
    uint16_t beast_caps;
};

struct Sent {
    std::vector<uint16_t> to;
    std::string hex;
};

std::vector<Sent> Report(const std::vector<Sink>& sinks, const PPSTime& t, uint8_t& last_seq) {
    std::vector<Sent> sent;
    uint16_t selected[8];
    auto send = [&](const uint16_t* to, uint16_t n, const uint8_t* buf, uint16_t len, uint16_t num_frames) {
        EXPECT_EQ(num_frames, 1);
        sent.push_back({std::vector<uint16_t>(to, to + n), ToHex(buf, len)});
        return true;
    };
    EXPECT_TRUE(ReportGPSTimeFrame(sinks.data(), sinks.size(), t, last_seq, selected, send));
    return sent;
}

}  // namespace

// The hex of a vector in readsbee_vectors/vectors.json (readsbee tests/vectors/feed_vectors.json).
std::string VectorsHex(const std::string& name) {
    std::ifstream file(READSBEE_VECTORS_JSON);
    std::stringstream ss;
    ss << file.rdbuf();
    std::string json = ss.str();
    size_t start = json.find("\"name\": \"" + name + "\",");
    if (start == std::string::npos) return "missing " + name;
    start = json.find("\"hex\": \"", start) + strlen("\"hex\": \"");
    return json.substr(start, json.find('"', start) - start);
}

TEST(BeastGPSTime, SpecVectorsAreReadsbeesVectors) {
    EXPECT_EQ(VectorsHex("gps_time"), kVectorGPSTime);
    EXPECT_EQ(VectorsHex("gps_time_escaped_step"), kVectorEscapedStep);
    EXPECT_EQ(VectorsHex("gps_time_rate_unknown"), kVectorRateUnknown);
    EXPECT_NE(VectorsHex("stream_query_answer_gps_time").find(kVectorStreamFeeder), std::string::npos);
}

TEST(BeastGPSTime, SpecVectors) {
    // 17.3 ppm fast; the edge at 0x00123456789b + 128/256.
    EXPECT_EQ(Frame(Edge(0x00123456789b, 128, 1795483693, 17300, false)), kVectorGPSTime);
    // 0x1a in ts6, sub1 and rate3 is escaped; flags 0x81 (step) never is.
    EXPECT_EQ(Frame(Edge(437911578, 26, 1795483694, 6682, true)), kVectorEscapedStep);
    // We always measure a rate, so 80 00 00 (unknown) is never ours: the rest of that vector matches.
    std::string unknown = kVectorRateUnknown;
    std::string ours = Frame(Edge(78211493946, 179, 1795483695, 0, false));
    EXPECT_EQ(ours.substr(0, ours.size() - 6), unknown.substr(0, unknown.size() - 6));
    EXPECT_EQ(ours.substr(ours.size() - 6), "000000");
    // stream_query_answer_gps_time: the sync frame between the Mode S frames is kVectorGPSTime.
    EXPECT_NE(std::string(kVectorStreamFeeder).find(Frame(Edge(0x00123456789b, 128, 1795483693, 17300, false))),
              std::string::npos);
}

TEST(BeastGPSTime, SameShiftAsModeS) {
    // A Mode S timestamp is the 48 MHz count >> 2: an edge on a whole 48 MHz count has the same ts6 and sub1 = its
    // remainder in 1/256 of a 12 MHz tick.
    PPSTime t;
    t.edge_mlat_48mhz_counts = 0x123456789ABull * 4 + 3;
    t.edge_sub_counts = 512;  // + half a 48 MHz count.
    std::string hex = Frame(t);
    EXPECT_EQ(hex.substr(6, 12), "0123456789ab");
    EXPECT_EQ(hex.substr(18, 2), "e0");  // (3 + 0.5) / 4 of a tick = 224 / 256.
    // ts6 wraps at 48 bits like the Mode S frames.
    t.edge_mlat_48mhz_counts = (1ull << 50) + 4;
    t.edge_sub_counts = 0;
    EXPECT_EQ(Frame(t).substr(6, 14), "00000000000100");
}

TEST(BeastGPSTime, FlagsAndRate) {
    for (bool step : {false, true}) {
        for (int32_t rate : {0, 1, -1, 200000, -200000, 0x7FFFFF, -0x7FFFFF, INT32_MAX, INT32_MIN}) {
            uint8_t buf[kGPSTimeBeastFrameMaxLenBytes];
            uint16_t len = WriteGPSTimeFrame(buf, 0x1a1a1a1a1a1aull, 0x3FF, 0x1a1a1a1a1aull, rate, step);
            EXPECT_LE(len, kGPSTimeBeastFrameMaxLenBytes);
            // Unescape: 16 bytes after the letter, the 13th is the flags.
            std::vector<uint8_t> body;
            for (uint16_t i = 3; i < len; i++) {
                body.push_back(buf[i]);
                if (buf[i] == 0x1a) i++;
            }
            ASSERT_EQ(body.size(), 16u);
            EXPECT_EQ(body[12], step ? 0x81 : 0x80);
            int32_t got = (body[13] << 16) | (body[14] << 8) | body[15];
            if (got & 0x800000) got -= 0x1000000;
            int32_t want = rate > 0x7FFFFF ? 0x7FFFFF : (rate < -0x7FFFFF ? -0x7FFFFF : rate);
            EXPECT_EQ(got, want) << rate;
            EXPECT_NE(got, -0x800000) << "never 80 00 00 (unknown)";
        }
    }
}

TEST(BeastGPSTime, OnlyFeedsThatAgreedBit0) {
    std::vector<Sink> sinks = {{0, 1, kBeastCapGPSTime | kBeastCapRemoteID},
                               {1, 1, kBeastCapRemoteID},  // Answered 8E (readsbee today): no bit 0.
                               {2, 1, 0},                  // Plain Beast (stock readsb, no answer).
                               {3, 1, kBeastCapGPSTime}};
    uint8_t last_seq = 0;
    std::vector<Sent> sent = Report(sinks, Edge(0x00123456789b, 128, 1795483693, 17300, false, 7), last_seq);
    ASSERT_EQ(sent.size(), 1u) << "built once, one send to the agreed feeds";
    EXPECT_EQ(sent[0].to, (std::vector<uint16_t>{0, 3}));
    EXPECT_EQ(sent[0].hex, kVectorGPSTime);
    // Nobody agreed: nothing sent, the seq still taken.
    std::vector<Sink> plain = {{1, 1, kBeastCapRemoteID}, {2, 1, 0}};
    EXPECT_TRUE(Report(plain, Edge(1, 0, 2, 0, false, 8), last_seq).empty());
    EXPECT_EQ(last_seq, 8);
}

TEST(BeastGPSTime, OneFramePerSeq) {
    std::vector<Sink> sinks = {{0, 1, kBeastCapGPSTime}};
    uint8_t last_seq = 0;
    PPSTime t = Edge(0x00123456789b, 128, 1795483693, 17300, false, 0);
    EXPECT_TRUE(Report(sinks, t, last_seq).empty()) << "seq 0: no edge yet";
    t.seq = 1;
    EXPECT_EQ(Report(sinks, t, last_seq).size(), 1u);
    for (int i = 0; i < 20; i++) EXPECT_TRUE(Report(sinks, t, last_seq).empty()) << "status repeats every ~100 ms";
    t.seq = 2;
    t.utc_seconds++;
    EXPECT_EQ(Report(sinks, t, last_seq).size(), 1u);
    // seq wraps 255 -> 1 (never 0).
    last_seq = 255;
    t.seq = 1;
    EXPECT_EQ(Report(sinks, t, last_seq).size(), 1u);
}

TEST(BeastGPSTime, CapsOfferedAndKnown) {
    EXPECT_EQ(kBeastCapGPSTime, 1);
    EXPECT_TRUE(kBeastCapsKnown & kBeastCapGPSTime);
    EXPECT_TRUE(kBeastCapsSendable & kBeastCapGPSTime);
    // An answer of 8f (bits 0-3) to our query (bits 0 and 2) agrees bits 0 and 2.
    uint8_t buf[kBeastCapsFrameLenBytes];
    CapsSession session;
    EXPECT_EQ(ToHex(buf, session.Start(buf, kBeastCapGPSTime | kBeastCapRemoteID, 0)), "1ae551818580");
    const uint8_t answer[] = {0x1a, 0xe5, 0x41, 0x81, 0x8f, 0x80};
    ASSERT_TRUE(session.Receive(answer, sizeof(answer), 10));
    EXPECT_EQ(session.bits(), kBeastCapGPSTime | kBeastCapRemoteID);
    // GNSS off: the query has no bit 0, so a server's bit 0 agrees nothing.
    session.Start(buf, kBeastCapRemoteID, 0);
    ASSERT_TRUE(session.Receive(answer, sizeof(answer), 10));
    EXPECT_EQ(session.bits(), kBeastCapRemoteID);
}

TEST(BeastGPSTime, Bit0OfferedOnlyWithGNSS) {
    EXPECT_EQ(CapsToOffer(false, false), 0);
    EXPECT_EQ(CapsToOffer(true, false), kBeastCapRemoteID);
    EXPECT_EQ(CapsToOffer(false, true), kBeastCapGPSTime);
    EXPECT_EQ(CapsToOffer(true, true), kBeastCapGPSTime | kBeastCapRemoteID);
}
