// Beast capabilities check and the frames it enables (ADSBee Beast extensions (2026-11-06)). Expected bytes are read from
// their golden vectors, readsbee_vectors/vectors.json (checked by readsbee_vectors/readsbee_proto.py).
#include <climits>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "beast_utils.hh"
#include "composite_array.hh"
#include "gtest/gtest.h"
#include "mode_s_packet.hh"
#include "settings.hh"
#include "uat_packet.hh"

using namespace BeastReporter;

namespace {

const std::string& VectorsJSON() {
    static std::string json = [] {
        std::ifstream file(READSBEE_VECTORS_JSON);
        std::stringstream ss;
        ss << file.rdbuf();
        return ss.str();
    }();
    return json;
}

// The text of one vector's object in vectors.json (written by json.dumps with indent=1).
std::string VectorBlock(const std::string& name) {
    const std::string& json = VectorsJSON();
    size_t start = json.find("\"name\": \"" + name + "\",");
    if (start == std::string::npos) return "";
    size_t end = json.find("\"name\": ", start + 1);
    return json.substr(start, end == std::string::npos ? std::string::npos : end - start);
}

std::string VectorHex(const std::string& name) {
    std::string block = VectorBlock(name);
    size_t start = block.find("\"hex\": \"");
    if (start == std::string::npos) return "missing vector " + name;
    start += strlen("\"hex\": \"");
    return block.substr(start, block.find('"', start) - start);
}

int VectorInt(const std::string& block, const std::string& key) {
    size_t start = block.find("\"" + key + "\": ");
    return start == std::string::npos ? -1 : std::stoi(block.substr(start + key.size() + 4));
}

std::vector<uint8_t> Hex(const std::string& hex) {
    std::vector<uint8_t> out;
    for (size_t i = 0; i + 1 < hex.size(); i += 2) out.push_back(std::stoi(hex.substr(i, 2), nullptr, 16));
    return out;
}

std::string ToHex(const uint8_t* buf, uint16_t len) {
    static const char* kDigits = "0123456789abcdef";
    std::string out;
    for (uint16_t i = 0; i < len; i++) {
        out += kDigits[buf[i] >> 4];
        out += kDigits[buf[i] & 0xf];
    }
    return out;
}

// readsbee_proto.build_vectors inputs.
const uint64_t kTs = 0x0123451A6789;  // Contains 0x1a on purpose.
const char* kShortMsg = "5d4840d6abcdef";
const char* kLongMsg = "8d4840d6202cc371c32ce0576098";
const uint8_t kMAC[6] = {0xc0, 0xff, 0xee, 0x1a, 0x00, 0x01};
const uint8_t kPackMAC[6] = {0x6a, 0x1a, 0x2b, 0x3c, 0x4d, 0x5e};
const uint8_t kFeedId[8] = {0x0b, 0xee, 0x00, 0x03, 0x81, 0x72, 0xd1, 0x8c};
const uint16_t kAllBits = kBeastCapADSL | kBeastCapRemoteID | kBeastCapUATModeSClock;  // 0x0e; bit 0 is reserved.

std::vector<uint8_t> ADSLOnAir() {
    std::vector<uint8_t> body = Hex("0002" "1a" "3c4d5e6f" "1a1a" "00112233445566778899aabb" "0102" "a1b2c3");
    body.insert(body.begin(), body.size());  // L = 26 = 0x1a: the length byte itself is escaped.
    return body;
}

std::vector<uint8_t> ODIDMessage(uint8_t type, uint8_t fill) {
    std::vector<uint8_t> msg(RawRemoteIDPacket::kSingleMessageLenBytes, fill);
    msg[0] = type << 4 | 0x2;
    return msg;
}

std::vector<uint8_t> ODIDPack(const std::vector<std::vector<uint8_t>>& messages) {
    std::vector<uint8_t> pack = {0xF2, RawRemoteIDPacket::kSingleMessageLenBytes, static_cast<uint8_t>(messages.size())};
    for (const auto& m : messages) pack.insert(pack.end(), m.begin(), m.end());
    return pack;
}

std::vector<uint8_t> VectorPack() { return ODIDPack({ODIDMessage(0, 0x1a), ODIDMessage(1, 0x11), ODIDMessage(4, 0x44)}); }

std::string ModeS(const char* msg, uint64_t ts, int16_t dbm) {
    uint8_t buf[kModeSBeastFrameMaxLenBytes];
    DecodedModeSPacket packet(msg, 0, dbm, 50, ts << 2);
    return ToHex(buf, BuildModeSBeastFrame(buf, packet));
}

// caps: kBeastCapUATModeSClock gives the upper-case letter; any other bit changes nothing.
std::string UATADSB(const std::vector<uint8_t>& payload, uint64_t ts, int16_t dbm, uint16_t caps) {
    uint8_t buf[kUATADSBBeastFrameMaxLenBytes];
    std::vector<uint8_t> copy = payload;
    RawUATADSBPacket packet(copy.data(), copy.size(), dbm, 0, ts << 2);
    return ToHex(buf, BuildUATADSBBeastFrame(buf, packet, caps));
}

std::string UATUplink(uint64_t ts, int16_t dbm, uint16_t caps) {
    std::vector<uint8_t> payload(RawUATUplinkPacket::kUplinkMessageNumBytes);
    for (size_t i = 0; i < payload.size(); i++) payload[i] = (i * 7) & 0xff;
    RawUATUplinkPacket packet(payload.data(), payload.size(), dbm, 0, ts << 2);
    static uint8_t buf[kUATUplinkBeastFrameMaxLenBytes];
    return ToHex(buf, BuildUATUplinkBeastFrame(buf, packet, caps));
}

std::string ADSL(ADSLChannelChar channel, uint64_t ts, int16_t dbm, uint8_t repaired) {
    uint8_t buf[kADSLBeastFrameMaxLenBytes];
    std::vector<uint8_t> onair = ADSLOnAir();
    return ToHex(buf, WriteADSLBeastFrame(buf, channel, ts, dbm, repaired, onair.data(), onair.size()));
}

std::string RemoteID(char transport, int16_t dbm, const uint8_t mac[6], const std::vector<uint8_t>& odid) {
    uint8_t buf[kRemoteIDBeastFrameMaxLenBytes];
    return ToHex(buf, WriteRemoteIDBeastFrame(buf, transport, 0, dbm, mac, odid.data(), odid.size()));
}

std::string Caps(BeastCapsLetter letter, uint16_t bits) {
    uint8_t buf[kBeastCapsFrameLenBytes];
    return ToHex(buf, WriteCapsFrame(buf, letter, bits));
}

std::string FeedStart() {
    uint8_t buf[2 + kUuidNumChars];
    uint8_t id[8];
    memcpy(id, kFeedId, 8);
    return ToHex(buf, BuildFeedStartFrame(buf, id));
}

// The hex of each step of an exchange vector (query -> answer -> frames), in order.
std::vector<std::string> ExchangeHex(const std::string& name) {
    std::string block = VectorBlock(name);
    std::vector<std::string> steps;
    size_t at = block.find("\"exchange\": [");
    while (at != std::string::npos && (at = block.find("\"hex\": \"", at)) != std::string::npos) {
        at += strlen("\"hex\": \"");
        steps.push_back(block.substr(at, block.find('"', at) - at));
    }
    return steps;
}

}  // namespace

TEST(BeastCapsVectors, FileIsThere) {
    ASSERT_FALSE(VectorsJSON().empty()) << READSBEE_VECTORS_JSON;
    EXPECT_EQ(VectorHex("caps_query"), "1ae551818e80");
    // dBm and the confirm frame are gone from the spec (2026-11-07).
    EXPECT_EQ(VectorBlock("caps_confirm_dbm"), "");
    EXPECT_EQ(VectorsJSON().find("\"dbm\""), std::string::npos);
}

TEST(BeastCapsVectors, CapabilitiesFrames) {
    EXPECT_EQ(Caps(kBeastCapsQuery, kAllBits), VectorHex("caps_query"));
    EXPECT_EQ(Caps(kBeastCapsQuery, 0), VectorHex("caps_query_none"));
    // What this firmware offers: Remote ID while receive is on. Never 0x1a.
    EXPECT_EQ(Caps(kBeastCapsQuery, kBeastCapsSendable), "1ae551818480");
    EXPECT_EQ(Caps(kBeastCapsAnswer, 1 << 13 | 1), "1ae54181" "81c0");
}

// Bit 0 is reserved; bits 1 and 3 wait for ADS-L (#409) and sub-GHz Mode S timestamps (#428) on main.
TEST(BeastCapsVectors, OnlyImplementedBitsAreOffered) {
    EXPECT_EQ(kBeastCapsKnown, 0x0e);
    EXPECT_EQ(kBeastCapsSendable, kBeastCapRemoteID);
    EXPECT_EQ(kBeastCapsSendable & (kBeastCapADSL | kBeastCapUATModeSClock), 0);
}

TEST(BeastCapsVectors, ModeSPowerByte) {
    EXPECT_EQ(ModeS(kShortMsg, 0x1234, -87), VectorHex("mode_s_short_power_m87"));
    EXPECT_EQ(ModeS(kShortMsg, 0x1234, -105), VectorHex("mode_s_short_power_m105"));
    EXPECT_EQ(ModeS(kShortMsg, 0x1234, -20), VectorHex("mode_s_short_power_m20"));
    EXPECT_EQ(ModeS(kShortMsg, 0x1234, INT16_MIN), VectorHex("mode_s_short_power_none"));
    EXPECT_EQ(ModeS(kLongMsg, kTs, -60), VectorHex("mode_s_long_power_m60"));
}

TEST(BeastCapsVectors, UAT) {
    std::vector<uint8_t> uat_s;
    for (uint8_t i = 0x10; i < 0x10 + 30; i++) uat_s.push_back(i);
    EXPECT_EQ(UATADSB(uat_s, 0x55, -95, 0), VectorHex("uat_s_power_m95"));
    EXPECT_EQ(UATUplink(kTs, -70, kBeastCapUATModeSClock), VectorHex("uat_U_power_m70"));
    EXPECT_EQ(UATUplink(kTs, -52, kBeastCapUATModeSClock), VectorHex("uat_U_uplink"));  // Power byte 90.
    // Bit 3 alone changes only the letter; the other bits change nothing.
    std::string lower = UATADSB(uat_s, kTs, -95, 0), upper = UATADSB(uat_s, kTs, -95, kBeastCapUATModeSClock);
    EXPECT_EQ(lower.substr(0, 4) + "53" + lower.substr(6), upper);
    EXPECT_EQ(UATADSB(uat_s, kTs, -95, kBeastCapADSL | kBeastCapRemoteID | 1), lower);
}

TEST(BeastCapsVectors, ADSL) {
    EXPECT_EQ(ADSL(kADSLChannel868_4, kTs, -87, 2), VectorHex("adsl_868_4_power"));
    EXPECT_EQ(ADSL(kADSLChannel868_2, 0, -108, 0), VectorHex("adsl_868_2_power_floor"));
    EXPECT_EQ(ADSL(kADSLChannelUnknown, kTs, INT16_MIN, kADSLRepairedUnknown), VectorHex("adsl_power_none"));
}

TEST(BeastCapsVectors, ADSLRefusesOnAirBytesADecoderCouldNotBound) {
    uint8_t buf[kADSLBeastFrameMaxLenBytes];
    std::vector<uint8_t> onair = ADSLOnAir();
    EXPECT_EQ(WriteADSLBeastFrame(buf, kADSLChannel868_2, 0, -80, 0, onair.data(), onair.size() - 1), 0);
    std::vector<uint8_t> too_long(33, 0x1a);
    too_long[0] = 32;
    EXPECT_EQ(WriteADSLBeastFrame(buf, kADSLChannel868_2, 0, -80, 0, too_long.data(), too_long.size()), 0);
    // Worst case: L = 31, every byte 0x1a (the power byte never is).
    std::vector<uint8_t> worst(32, 0x1a);
    worst[0] = 31;
    uint16_t len = WriteADSLBeastFrame(buf, kADSLChannelUnknown, 0x1a1a1a1a1a1a, -66, 0x1a, worst.data(), worst.size());
    EXPECT_EQ(len, 3 + 2 * 6 + 1 + 2 + 1 + 2 * 31);
    EXPECT_LE(len, kADSLBeastFrameMaxLenBytes);
    EXPECT_EQ(kADSLBeastFrameMaxLenBytes, 83);  // FEED-PROTOCOL.md section 2.
}

TEST(BeastCapsVectors, RemoteID) {
    EXPECT_EQ(RemoteID(kRemoteIDTransportBT4Legacy, -72, kMAC, ODIDMessage(1, 0x1a)), VectorHex("rid_bt4_single_power"));
    EXPECT_EQ(RemoteID(kRemoteIDTransportBT4Legacy, -34, kMAC, ODIDMessage(1, 0x1a)),
              VectorHex("rid_bt4_power_saturated"));
    EXPECT_EQ(RemoteID(kRemoteIDTransportWiFiBeacon, -60, kPackMAC, VectorPack()),
              VectorHex("rid_wifi_beacon_pack_power"));
    EXPECT_EQ(RemoteID(kRemoteIDTransportBT5Coded, -95, kMAC, ODIDMessage(0, 0x22)), VectorHex("rid_bt5_coded_power"));
}

// Whole feeds, both directions: query, answer, then the agreed frames from the moment the answer is read.
TEST(BeastCapsVectors, Streams) {
    std::vector<uint8_t> uat_s;
    for (uint8_t i = 0x10; i < 0x10 + 30; i++) uat_s.push_back(i);
    for (const char* name : {"stream_query_answer", "stream_query_answer_rid_only"}) {
        std::vector<std::string> steps = ExchangeHex(name);
        ASSERT_EQ(steps.size(), 3u) << name;
        uint8_t query[kBeastCapsFrameLenBytes];
        CapsSession session;
        std::string before = FeedStart() + ToHex(query, session.Start(query, kAllBits, 0)) + ModeS(kShortMsg, 1, -87);
        EXPECT_EQ(before, steps[0]) << name;
        std::vector<uint8_t> answer = Hex(steps[1]);
        ASSERT_TRUE(session.Receive(answer.data(), answer.size(), 100)) << name;
        const uint16_t bits = session.bits();
        std::string after = ModeS(kShortMsg, 3, -87);
        if (bits & kBeastCapADSL) after += ADSL(kADSLChannel868_2, kTs, -87, 1);
        if (bits & kBeastCapRemoteID) after += RemoteID(kRemoteIDTransportBT4Legacy, -87, kMAC, ODIDMessage(1, 0x33));
        after += UATADSB(uat_s, kTs, -87, bits);
        EXPECT_EQ(after, steps[2]) << name;
        EXPECT_EQ(before + after, VectorHex(name)) << name;
    }
}

// A server that never answers gets today's main output plus the 6 query bytes, and the query is the only addition.
TEST(BeastCapsPlain, StreamIsMainPlusTheQuery) {
    std::vector<uint8_t> payload =
        Hex("00a66ef135445d525a0c05191190212048006cb82bc4d53a5b2bb0a8ec6e");
    std::string frames = ModeS("8d495066587f469bb826d21ad767", 0xABABFF1AFFFFFF1A, -80) +
                         ModeS("5d4840d6abcdef", 0x0000001A2B3C, -55) + UATADSB(payload, 0x1a3456, -60, 0);
    // Main's bytes for these packets (main's BuildModeSBeastFrame / BuildUATADSBBeastFrame).
    const std::string main_frames =
        "1a33ff1a1affffff1a1a088d495066587f469bb826d21a1ad7671a320000001a1a2b3c465d4840d6abcdef1aec730000001a1a34"
        "562d00a66ef135445d525a0c05191190212048006cb82bc4d53a5b2bb0a8ec6e";
    EXPECT_EQ(frames, main_frames);
    uint8_t query[kBeastCapsFrameLenBytes];
    CapsSession session;
    std::string stream = FeedStart() + ToHex(query, session.Start(query, kBeastCapsSendable, 0));
    session.Receive(Hex("1a50001a1a05").data(), 6, 1000);  // A readsb ping.
    session.Update(kBeastCapsAnswerTimeoutMs);
    ASSERT_EQ(session.bits(), 0);
    stream += ModeS("8d495066587f469bb826d21ad767", 0xABABFF1AFFFFFF1A, -80) +
              ModeS("5d4840d6abcdef", 0x0000001A2B3C, -55) + UATADSB(payload, 0x1a3456, -60, session.bits());
    EXPECT_EQ(stream, FeedStart() + "1ae551818480" + main_frames);
    EXPECT_EQ(FeedStart(), "1ae430626565303030332d383137322d643138632d306265652d303030333831373264313863");
}

// The fix that goes with the Beast extensions: no level is power byte 0 (main sent 1, the first table entry).
TEST(BeastCapsPlain, SignalBytes) {
    uint8_t buf[2];
    auto power = [&](int16_t dbm) { return WriteRSSIdBmAsBeastPowerLevel(buf, dbm) == 1 ? buf[0] : 0x1ff; };
    EXPECT_EQ(power(INT16_MIN), 0);
    EXPECT_EQ(power(-100), 1);
    EXPECT_EQ(power(-130), 1);
    EXPECT_EQ(power(-40), 255);
    EXPECT_EQ(power(0), 255);
    EXPECT_EQ(power(-87), 4);
    EXPECT_EQ(power(-66), 0x1b);  // The nearest to 0x1a: no power byte is 0x1a, so it is never escaped.
    for (int rssi = -300; rssi <= 300; rssi++) EXPECT_EQ(WriteRSSIdBmAsBeastPowerLevel(buf, rssi), 1) << rssi;
}

// vectors.json server_to_feeder: what the scanner finds, all at once and one byte at a time.
TEST(BeastCapsScanner, MatchesTheVectors) {
    const char* names[] = {"answer_all",          "answer_none",       "answer_rid_only",  "answer_after_pings",
                           "answer_future",       "echo_of_query",     "answer_malformed", "answer_cut_by_frame",
                           "stock_readsb_silence", "answer_bit0_set"};
    for (const char* name : names) {
        std::string block = VectorBlock(name);
        ASSERT_NE(block.find("server_to_feeder"), std::string::npos) << name;
        bool want = block.find("\"answer\": null") == std::string::npos;
        std::vector<uint8_t> wire = Hex(VectorHex(name));
        CapsAnswerScanner whole, slow;
        EXPECT_EQ(whole.Scan(wire.data(), wire.size()), want) << name;
        int found = 0;
        for (uint8_t x : wire) found += slow.Scan(&x, 1);
        EXPECT_EQ(found, want ? 1 : 0) << name;
        if (!want) continue;
        EXPECT_EQ(whole.version(), VectorInt(block, "version")) << name;
        EXPECT_EQ(whole.bits(), VectorInt(block, "bits")) << name;
        EXPECT_EQ(slow.bits(), whole.bits()) << name;
    }
}

TEST(BeastCapsScanner, FindsTheAnswerInPingsHeartbeatsAndGarbage) {
    std::mt19937 rng(1095);
    std::vector<uint8_t> answer = Hex("1ae541818480");
    const std::vector<uint8_t> ping = Hex("1a50001a1a05"), heartbeat = Hex("1a31000000000000000000"),
                               echo = Hex("1ae551818480"), unknown = Hex("1af5711a1a33000000");
    for (int run = 0; run < 200; run++) {
        std::vector<uint8_t> wire;
        int parts = rng() % 20;
        for (int p = 0; p < parts; p++) {
            switch (rng() % 5) {
                case 0: wire.insert(wire.end(), ping.begin(), ping.end()); break;
                case 1: wire.insert(wire.end(), heartbeat.begin(), heartbeat.end()); break;
                case 2: wire.insert(wire.end(), echo.begin(), echo.end()); break;
                case 3: wire.insert(wire.end(), unknown.begin(), unknown.end()); break;
                default:
                    // Line noise: anything but 0x1a, whose meaning depends on what came before.
                    for (int i = rng() % 64; i > 0; i--) {
                        uint8_t x = rng();
                        wire.push_back(x == 0x1a ? 0x00 : x);
                    }
            }
        }
        CapsAnswerScanner noise_only;
        EXPECT_FALSE(noise_only.Scan(wire.data(), wire.size())) << run;
        wire.insert(wire.end(), answer.begin(), answer.end());
        std::vector<uint8_t> after(100, 0xaa);
        after.insert(after.end(), answer.begin(), answer.end());  // A second answer: ignored.
        wire.insert(wire.end(), after.begin(), after.end());
        CapsAnswerScanner scanner;
        int found = 0;
        // Uneven chunks, like recv().
        for (size_t i = 0; i < wire.size();) {
            size_t n = std::min<size_t>(1 + rng() % 64, wire.size() - i);
            found += scanner.Scan(wire.data() + i, n);
            i += n;
        }
        EXPECT_EQ(found, 1) << run;
        EXPECT_EQ(scanner.bits(), kBeastCapsSendable) << run;
    }
}

TEST(BeastCapsScanner, HoldsOnlyAFewBytes) { EXPECT_LE(sizeof(CapsAnswerScanner), 12u); }

TEST(BeastCapsSession, AnswerInTimeAgreesAtOnce) {
    uint8_t buf[kBeastCapsFrameLenBytes];
    CapsSession session;
    EXPECT_EQ(session.state(), CapsSession::kPlain);
    EXPECT_EQ(ToHex(buf, session.Start(buf, kBeastCapsSendable, 1000)), "1ae551818480");
    EXPECT_EQ(session.state(), CapsSession::kWaiting);
    EXPECT_EQ(session.bits(), 0);
    std::vector<uint8_t> answer = Hex(VectorHex("answer_all"));  // Server accepts bits 1-3.
    EXPECT_TRUE(session.Receive(answer.data(), answer.size(), 1000 + kBeastCapsAnswerTimeoutMs - 1));
    EXPECT_EQ(session.state(), CapsSession::kAgreed);
    EXPECT_EQ(session.bits(), kBeastCapRemoteID);  // Query bits AND answer bits, for the next frame on.
    // One answer per connection.
    EXPECT_FALSE(session.Receive(answer.data(), answer.size(), 2000));
    session.Update(1000000);
    EXPECT_EQ(session.bits(), kBeastCapRemoteID);
}

TEST(BeastCapsSession, KnownBitsOnlyAndEmptyAnswers) {
    uint8_t buf[kBeastCapsFrameLenBytes];
    CapsSession session;
    session.Start(buf, kBeastCapsSendable, 0);
    std::vector<uint8_t> future = Hex(VectorHex("answer_future"));  // Version 2, Remote ID and unknown bit 9.
    ASSERT_TRUE(session.Receive(future.data(), future.size(), 10));
    EXPECT_EQ(session.bits(), kBeastCapRemoteID);

    session.Start(buf, kBeastCapsSendable, 0);
    std::vector<uint8_t> none = Hex(VectorHex("answer_none"));
    ASSERT_TRUE(session.Receive(none.data(), none.size(), 10));
    EXPECT_EQ(session.state(), CapsSession::kAgreed);
    EXPECT_EQ(session.bits(), 0);

    // Reserved bit 0 (readsbee before 2026-11-07 answered 8f): never offered, never used.
    session.Start(buf, kBeastCapsSendable | 1, 0);
    EXPECT_EQ(session.offered(), kBeastCapsSendable);
    std::vector<uint8_t> bit0 = Hex(VectorHex("answer_bit0_set"));
    ASSERT_TRUE(session.Receive(bit0.data(), bit0.size(), 10));
    EXPECT_EQ(session.bits(), kBeastCapRemoteID);
    session.Start(buf, kAllBits, 0);
    ASSERT_TRUE(session.Receive(bit0.data(), bit0.size(), 10));
    EXPECT_EQ(session.bits(), kAllBits);
}

TEST(BeastCapsSession, FiveSecondTimeout) {
    uint8_t buf[kBeastCapsFrameLenBytes];
    std::vector<uint8_t> answer = Hex(VectorHex("answer_all"));
    CapsSession session;
    const uint32_t t0 = UINT32_MAX - 1000;  // Across the millisecond counter's wrap.
    session.Start(buf, kBeastCapsSendable, t0);
    session.Update(t0 + kBeastCapsAnswerTimeoutMs - 1);
    EXPECT_EQ(session.state(), CapsSession::kWaiting);
    session.Update(t0 + kBeastCapsAnswerTimeoutMs);
    EXPECT_EQ(session.state(), CapsSession::kPlain);
    // A late answer is ignored, also when no Update() ran in between.
    EXPECT_FALSE(session.Receive(answer.data(), answer.size(), t0 + 6000));
    session.Start(buf, kBeastCapsSendable, t0);
    EXPECT_FALSE(session.Receive(answer.data(), answer.size(), t0 + kBeastCapsAnswerTimeoutMs));
    EXPECT_EQ(session.state(), CapsSession::kPlain);
    EXPECT_EQ(session.bits(), 0);
}

TEST(BeastCapsSession, Reconnect) {
    uint8_t buf[kBeastCapsFrameLenBytes];
    std::vector<uint8_t> answer = Hex(VectorHex("answer_all"));
    CapsSession session;
    session.Start(buf, kBeastCapsSendable, 0);
    ASSERT_TRUE(session.Receive(answer.data(), answer.size(), 10));
    EXPECT_EQ(session.bits(), kBeastCapRemoteID);
    // A new connection asks again; nothing is cached.
    session.Start(buf, kBeastCapsSendable, 100);
    EXPECT_EQ(session.state(), CapsSession::kWaiting);
    EXPECT_EQ(session.bits(), 0);
    ASSERT_TRUE(session.Receive(answer.data(), answer.size(), 200));
    EXPECT_EQ(session.bits(), kBeastCapRemoteID);
    session.Reset();
    EXPECT_EQ(session.state(), CapsSession::kPlain);
    EXPECT_EQ(session.bits(), 0);
}

// Every sink of a Beast protocol gets that protocol's one batch, whatever its capabilities.
TEST(BeastCapsGroups, ByProtocolOnly) {
    typedef SettingsManager S;
    struct Info {  // Same members as CommsManager::ReportSinkInfo.
        uint16_t sink;
        S::ReportingProtocol protocol;
        uint16_t beast_caps;
    };
    const Info sinks[] = {{0, S::kBeast, 0},
                          {1, S::kBeast, kBeastCapRemoteID},
                          {2, S::kBeastNoUAT, kBeastCapRemoteID},
                          {3, S::kBeast, kAllBits},
                          {4, S::kCSBee, kBeastCapRemoteID}};
    uint16_t selected[5];
    ASSERT_EQ(SelectBeastSinks(sinks, 5, S::kBeast, selected), 3);
    EXPECT_EQ(selected[0], 0);
    EXPECT_EQ(selected[1], 1);
    EXPECT_EQ(selected[2], 3);
    ASSERT_EQ(SelectBeastSinks(sinks, 5, S::kBeastNoUAT, selected), 1);
    EXPECT_EQ(selected[0], 2);
    EXPECT_EQ(SelectBeastSinks(sinks, 5, S::kBeastNoUATUplink, selected), 0);
}

// Remote ID frames from received packets (MAC order, unknown level, transports, bounds).
TEST(BeastCapsRemoteID, BuildsFromAReceivedPacket) {
    uint8_t buf[kRemoteIDBeastFrameMaxLenBytes];
    RawRemoteIDPacket packet;
    std::vector<uint8_t> msg = ODIDMessage(1, 0x1a);
    memcpy(packet.payload, msg.data(), msg.size());
    packet.payload_len_bytes = msg.size() + 3;  // Trailing bytes past the message are not sent.
    packet.rssi_dbm = -72;
    // NimBLE holds BLE addresses least significant byte first; the frame carries display order.
    for (int i = 0; i < 6; i++) packet.source_mac[i] = kMAC[5 - i];
    packet.transport = RawRemoteIDPacket::kTransportBT4Legacy;
    EXPECT_EQ(ToHex(buf, BuildRemoteIDBeastFrame(buf, packet)), VectorHex("rid_bt4_single_power"));

    packet.transport = RawRemoteIDPacket::kTransportBT5LongRange;
    std::string coded = ToHex(buf, BuildRemoteIDBeastFrame(buf, packet));
    EXPECT_EQ(coded.substr(0, 6), "1aef63");
    EXPECT_EQ(coded.substr(20, 14), "c0ffee1a1a0001");

    // A WiFi source address is already in frame order. Level 0 means unknown.
    memcpy(packet.source_mac, kMAC, 6);
    packet.transport = RawRemoteIDPacket::kTransportWiFiBeacon;
    packet.rssi_dbm = 0;
    std::string wifi = ToHex(buf, BuildRemoteIDBeastFrame(buf, packet));
    EXPECT_EQ(wifi.substr(0, 6), "1aef77");
    EXPECT_EQ(wifi.substr(18, 2), "00");  // No level: power byte 0.
    EXPECT_EQ(wifi.substr(20, 14), "c0ffee1a1a0001");

    packet.transport = RawRemoteIDPacket::kTransportInvalid;
    EXPECT_EQ(BuildRemoteIDBeastFrame(buf, packet), 0);
    packet.transport = RawRemoteIDPacket::kTransportWiFiBeacon;
    packet.payload_len_bytes = 0;
    EXPECT_EQ(BuildRemoteIDBeastFrame(buf, packet), 0);
}

TEST(BeastCapsRemoteID, OtherLettersAndTheLargestFrame) {
    std::vector<std::vector<uint8_t>> nine;
    for (int i = 0; i < 9; i++) nine.push_back(ODIDMessage(i % 6, i == 4 ? 0x1a : 0x30 + i));
    // readsbee_proto.enc_rid(pack9, 0, 0, "e", mac) and enc_rid(msg, 0, 255, "n", 1a x 6).
    EXPECT_EQ(RemoteID(kRemoteIDTransportBT5Extended1M, INT16_MIN, kMAC, ODIDPack(nine)),
              "1aef6500000000000000c0ffee1a1a0001f2190902303030303030303030303030303030303030303030303030123131313131"
              "313131313131313131313131313131313131312232323232323232323232323232323232323232323232323232333333333333"
              "333333333333333333333333333333333333421a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a"
              "1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a52353535353535353535353535353535353535353535353535023636363636363636"
              "363636363636363636363636363636361237373737373737373737373737373737373737373737373722383838383838383838"
              "383838383838383838383838383838");
    const uint8_t all_escapes[6] = {0x1a, 0x1a, 0x1a, 0x1a, 0x1a, 0x1a};
    EXPECT_EQ(RemoteID(kRemoteIDTransportWiFiNaN, 26, all_escapes, ODIDMessage(2, 0x55)),
              "1aef6e000000000000ff1a1a1a1a1a1a1a1a1a1a1a1a22555555555555555555555555555555555555555555555555");
    std::vector<uint8_t> pack = ODIDPack(std::vector<std::vector<uint8_t>>(9, ODIDMessage(1, 0x1a)));
    uint8_t buf[kRemoteIDBeastFrameMaxLenBytes];
    uint16_t len = WriteRemoteIDBeastFrame(buf, kRemoteIDTransportBT4Legacy, 0x1a1a1a1a1a1a, -66, all_escapes,
                                           pack.data(), pack.size());
    EXPECT_EQ(len, 3 + 12 + 1 + 12 + 3 + 9 * (1 + 2 * 24));
    EXPECT_GE(kRemoteIDBeastFrameMaxLenBytes, 485);  // FEED-PROTOCOL.md section 2.
    EXPECT_LE(len, kRemoteIDBeastFrameMaxLenBytes);
}

TEST(BeastCapsRemoteID, RefusesODIDBytesADecoderCouldNotBound) {
    std::vector<uint8_t> msg = ODIDMessage(1, 0);
    EXPECT_EQ(RemoteIDODIDLenBytes(msg.data(), 25), 25);
    EXPECT_EQ(RemoteIDODIDLenBytes(msg.data(), 24), 0);
    EXPECT_EQ(RemoteIDODIDLenBytes(msg.data(), 0), 0);
    std::vector<uint8_t> pack = ODIDPack(std::vector<std::vector<uint8_t>>(10, ODIDMessage(1, 0)));
    EXPECT_EQ(RemoteIDODIDLenBytes(pack.data(), pack.size()), 0);  // Count over 9.
    pack[2] = 0;
    EXPECT_EQ(RemoteIDODIDLenBytes(pack.data(), pack.size()), 0);
    pack[2] = 2;
    EXPECT_EQ(RemoteIDODIDLenBytes(pack.data(), pack.size()), 53);
    pack[1] = 24;
    EXPECT_EQ(RemoteIDODIDLenBytes(pack.data(), pack.size()), 0);
    pack[1] = 25;
    EXPECT_EQ(RemoteIDODIDLenBytes(pack.data(), 52), 0);
}

// The READSBEE protocol value is gone: feeds stay BEAST and its variants.
TEST(BeastCapsSettings, NoReadsbeeProtocol) {
    typedef SettingsManager S;
    for (uint16_t p = 0; p < S::kNumProtocols; p++) EXPECT_STRNE(S::kReportingProtocolStrs[p], "READSBEE");
    EXPECT_EQ(S::kNumProtocols, 13);
    // A stored 13 (READSBEE on unreleased t1069 builds) is out of range and sanitized.
    SettingsManager manager;
    manager.settings.feed_protocols[1] = static_cast<S::ReportingProtocol>(13);
    EXPECT_TRUE(manager.Sanitize());
    EXPECT_EQ(manager.settings.feed_protocols[1], S::kNoReports);
}

namespace {
// A received Remote ID packet as the ESP32 queues it, with a timestamp in 48 MHz counts (0 = not synced).
RawRemoteIDPacket RemoteIDPacket(RawRemoteIDPacket::Transport transport, const uint8_t mac[6],
                                 const std::vector<uint8_t>& odid, int16_t dbm, uint64_t mlat_48mhz_counts) {
    RawRemoteIDPacket packet;
    memcpy(packet.payload, odid.data(), odid.size());
    packet.payload_len_bytes = odid.size();
    packet.rssi_dbm = dbm;
    packet.transport = transport;
    for (int i = 0; i < 6; i++) {
        // NimBLE holds BLE addresses least significant byte first; WiFi addresses are in frame order.
        packet.source_mac[i] = transport == RawRemoteIDPacket::kTransportWiFiBeacon ? mac[i] : mac[5 - i];
    }
    packet.mlat_48mhz_64bit_counts = mlat_48mhz_counts;
    return packet;
}

std::string Build(const RawRemoteIDPacket& packet) {
    uint8_t buf[kRemoteIDBeastFrameMaxLenBytes];
    return ToHex(buf, BuildRemoteIDBeastFrame(buf, packet));
}
}  // namespace

// Synced (#428): upper-case letter and the Mode S clock; not synced: lower case and 0. Vector-exact both ways.
TEST(BeastCapsRemoteID, TimestampOnTheModeSClock) {
    const auto kBT4 = RawRemoteIDPacket::kTransportBT4Legacy;
    const auto kWiFi = RawRemoteIDPacket::kTransportWiFiBeacon;
    EXPECT_EQ(Build(RemoteIDPacket(kBT4, kMAC, ODIDMessage(1, 0x1a), -72, kTs << 2)),
              VectorHex("rid_bt4_modes_clock_power"));
    EXPECT_EQ(Build(RemoteIDPacket(kWiFi, kPackMAC, VectorPack(), -60, kTs << 2)),
              VectorHex("rid_wifi_beacon_pack_modes_clock"));
    EXPECT_EQ(Build(RemoteIDPacket(kBT4, kMAC, ODIDMessage(1, 0x1a), -72, 0)), VectorHex("rid_bt4_single_power"));
    EXPECT_EQ(Build(RemoteIDPacket(kWiFi, kPackMAC, VectorPack(), -60, 0)), VectorHex("rid_wifi_beacon_pack_power"));
    // The other letters go upper case the same way.
    EXPECT_EQ(
        Build(RemoteIDPacket(RawRemoteIDPacket::kTransportBT5LongRange, kMAC, ODIDMessage(0, 0x22), -95, kTs << 2))
            .substr(0, 6),
        "1aef43");
    EXPECT_EQ(Build(RemoteIDPacket(RawRemoteIDPacket::kTransportBT5LongRange, kMAC, ODIDMessage(0, 0x22), -95, 0)),
              VectorHex("rid_bt5_coded_power"));
}

// 48 MHz -> 12 MHz exactly as Mode S frames do it (GetMLAT12MHzCounter): counts / 4, low 48 bits.
TEST(BeastCapsRemoteID, TimestampConversionMatchesModeS) {
    const uint64_t counts[] = {4,
                               7,
                               kTs << 2,
                               (kTs << 2) | 3,
                               0x3FFFFFFFFFFFFull,
                               0x0004000000000008ull,
                               0xABABFF1AFFFFFF1Aull,
                               0xFFFFFFFFFFFFFFFFull,
                               123456789ull * 48};
    for (uint64_t c : counts) {
        std::string rid =
            Build(RemoteIDPacket(RawRemoteIDPacket::kTransportBT4Legacy, kMAC, ODIDMessage(1, 0x33), -70, c));
        uint8_t buf[kModeSBeastFrameMaxLenBytes];
        DecodedModeSPacket mode_s(kShortMsg, 0, -70, 50, c);
        std::string ms = ToHex(buf, BuildModeSBeastFrame(buf, mode_s));
        // Both escape the same 6 timestamp bytes: same hex from after the letter (Mode S has none) to the signal byte.
        size_t ts_hex_len = ms.size() - 4 - 2 - 2 * 7;  // 1a 32, signal byte, 7-byte message (no 1a in kShortMsg).
        EXPECT_EQ(rid.substr(6, ts_hex_len), ms.substr(4, ts_hex_len)) << std::hex << c;
        EXPECT_EQ(rid.substr(4, 2), "42") << std::hex << c;  // Upper case: every one of these is nonzero at 12 MHz.
    }
    // Counts below 4 are 0 at 12 MHz: lower case, so an upper-case frame never says "not synced".
    for (uint64_t c : {1ull, 3ull, 1ull << 50}) {
        std::string rid =
            Build(RemoteIDPacket(RawRemoteIDPacket::kTransportBT4Legacy, kMAC, ODIDMessage(1, 0x1a), -72, c));
        EXPECT_EQ(rid, VectorHex("rid_bt4_single_power")) << c;
    }
}

// The 12 MHz counter wraps at 48 bits like the Mode S frames': ffffffffffff, then 000000000002.
TEST(BeastCapsRemoteID, TimestampWrapsAt48Bits) {
    const auto kBT4 = RawRemoteIDPacket::kTransportBT4Legacy;
    std::string before = Build(RemoteIDPacket(kBT4, kMAC, ODIDMessage(1, 0x33), -70, 0xFFFFFFFFFFFFull << 2 | 3));
    std::string after = Build(RemoteIDPacket(kBT4, kMAC, ODIDMessage(1, 0x33), -70, (1ull << 50) + (2 << 2)));
    EXPECT_EQ(before + after, VectorHex("rid_bt4_modes_clock_wrap_power"));
}

namespace {
struct SentTo {
    uint16_t sink;
    std::string hex;
    uint16_t num_frames;
};

struct Info {  // Same members as CommsManager::ReportSinkInfo.
    uint16_t sink;
    SettingsManager::ReportingProtocol protocol;
    uint16_t beast_caps;
};

std::vector<SentTo> ReportRemoteID(const std::vector<Info>& sinks, const std::vector<RawRemoteIDPacket>& packets,
                                   uint16_t buf_size, bool* ret = nullptr, uint16_t* failures = nullptr) {
    std::vector<SentTo> sent;
    std::vector<uint8_t> buf(buf_size);
    std::vector<uint16_t> selected(sinks.size() + 1);
    uint16_t num_build_failures = 0;
    bool ok = ReportRemoteIDFrames(
        sinks.data(), sinks.size(), packets.data(), packets.size(), buf.data(), buf_size, selected.data(),
        [&](const uint16_t* to, uint16_t num_to, const uint8_t* b, uint16_t len, uint16_t num_frames) {
            for (uint16_t i = 0; i < num_to; i++) sent.push_back({to[i], ToHex(b, len), num_frames});
            return true;
        },
        num_build_failures);
    if (ret) *ret = ok;
    if (failures) *failures = num_build_failures;
    return sent;
}
}  // namespace

// Remote ID frames go only to the feeds that agreed on bit 2, after the Beast batch, built once.
TEST(BeastCapsRemoteID, OnlyToFeedsThatAgreedOnBit2) {
    typedef SettingsManager S;
    const std::vector<Info> sinks = {{0, S::kBeast, 0},
                                     {1, S::kBeast, kBeastCapRemoteID},
                                     {2, S::kBeastNoUAT, kBeastCapRemoteID},
                                     {3, S::kBeast, kBeastCapADSL},
                                     {4, S::kBeast, kBeastCapUATModeSClock | kBeastCapADSL},
                                     {5, S::kRaw, 0},
                                     {6, S::kBeastNoUATUplink, kBeastCapsKnown}};
    std::vector<RawRemoteIDPacket> packets = {
        RemoteIDPacket(RawRemoteIDPacket::kTransportBT4Legacy, kMAC, ODIDMessage(1, 0x1a), -72, kTs << 2),
        RemoteIDPacket(RawRemoteIDPacket::kTransportWiFiBeacon, kPackMAC, VectorPack(), -60, kTs << 2)};
    ASSERT_EQ(Build(packets[0]), VectorHex("rid_bt4_modes_clock_power"));
    ASSERT_EQ(Build(packets[1]), VectorHex("rid_wifi_beacon_pack_modes_clock"));
    std::string batch = Build(packets[0]) + Build(packets[1]);

    bool ok = false;
    std::vector<SentTo> sent = ReportRemoteID(sinks, packets, kRemoteIDBeastFrameMaxLenBytes * 4, &ok);
    EXPECT_TRUE(ok);
    ASSERT_EQ(sent.size(), 3u);  // Sinks 1, 2 and 6, one send of the same batch; never 0, 3, 4 or 5.
    EXPECT_EQ(sent[0].sink, 1);
    EXPECT_EQ(sent[1].sink, 2);
    EXPECT_EQ(sent[2].sink, 6);
    for (const SentTo& s : sent) {
        EXPECT_EQ(s.hex, batch);
        EXPECT_EQ(s.num_frames, 2);
    }

    // Nobody agreed on bit 2, or no packets (the RP2040 and the 1421): nothing is sent.
    EXPECT_TRUE(ReportRemoteID({sinks[0], sinks[3], sinks[4], sinks[5]}, packets, 2048).empty());
    EXPECT_TRUE(ReportRemoteID(sinks, {}, 2048).empty());
}

// A buffer too small for the round: sent in parts, each holding whole frames; too small for one frame: refused.
TEST(BeastCapsRemoteID, SplitsABatchThatDoesNotFit) {
    typedef SettingsManager S;
    const std::vector<Info> sinks = {{7, S::kBeast, kBeastCapRemoteID}, {8, S::kBeastNoUAT, kBeastCapRemoteID}};
    std::vector<RawRemoteIDPacket> packets;
    for (int i = 0; i < 5; i++) {
        packets.push_back(
            RemoteIDPacket(RawRemoteIDPacket::kTransportWiFiBeacon, kPackMAC, VectorPack(), -60 + i, (kTs + i) << 2));
    }
    RawRemoteIDPacket bad = packets[0];
    bad.transport = RawRemoteIDPacket::kTransportInvalid;
    packets.insert(packets.begin() + 2, bad);
    uint16_t failures = 0;
    bool ok = false;
    // Room for exactly two of these frames per send: a frame starts only where a worst-case one would fit.
    const uint16_t frame_len = Build(packets[0]).size() / 2;
    std::vector<SentTo> sent =
        ReportRemoteID(sinks, packets, kRemoteIDBeastFrameMaxLenBytes + frame_len, &ok, &failures);
    EXPECT_TRUE(ok);
    EXPECT_EQ(failures, 1);
    std::string all_7, all_8;
    uint16_t frames_7 = 0;
    for (const SentTo& s : sent) {
        (s.sink == 7 ? all_7 : all_8) += s.hex;
        if (s.sink == 7) frames_7 += s.num_frames;
    }
    std::string want;
    for (const RawRemoteIDPacket& p : packets) want += Build(p);
    EXPECT_EQ(sent.size(), 6u);  // Three sends, to two sinks each.
    EXPECT_EQ(frames_7, 5);
    EXPECT_EQ(all_7, want);
    EXPECT_EQ(all_8, want);

    sent = ReportRemoteID(sinks, packets, kRemoteIDBeastFrameMaxLenBytes - 1, &ok);
    EXPECT_FALSE(ok);
    EXPECT_TRUE(sent.empty());
}

namespace {
// A CompositeArray::RawPackets that holds only Remote ID packets, laid out as on the SPI bus.
struct RemoteIDArray {
    alignas(4) uint8_t buf[sizeof(CompositeArray::RawPackets::Header) + 4 * sizeof(RawRemoteIDPacket)] = {0};
    CompositeArray::RawPackets packets;
    explicit RemoteIDArray(const std::vector<RawRemoteIDPacket>& rid) {
        packets.header = reinterpret_cast<CompositeArray::RawPackets::Header*>(buf);
        packets.header->num_remote_id_packets = rid.size();
        packets.remote_id_packets = reinterpret_cast<RawRemoteIDPacket*>(buf + sizeof(*packets.header));
        for (size_t i = 0; i < rid.size(); i++) packets.remote_id_packets[i] = rid[i];
        packets.len_bytes = sizeof(*packets.header) + rid.size() * sizeof(RawRemoteIDPacket);
    }
};

std::vector<SentTo> ReportRound(const std::vector<Info>& sinks, const CompositeArray::RawPackets& packets,
                                bool validated, bool* ret, std::string* error = nullptr) {
    std::vector<SentTo> sent;
    uint8_t buf[kRemoteIDBeastFrameMaxLenBytes * 4];
    std::vector<uint16_t> selected(sinks.size() + 1);
    uint16_t num_build_failures = 0;
    char error_msg[CompositeArray::RawPackets::kErrorMessageMaxLen] = {0};
    *ret = ReportRemoteIDRound(
        sinks.data(), sinks.size(), packets, validated, buf, sizeof(buf), selected.data(),
        [&](const uint16_t* to, uint16_t num_to, const uint8_t* b, uint16_t len, uint16_t num_frames) {
            for (uint16_t i = 0; i < num_to; i++) sent.push_back({to[i], ToHex(b, len), num_frames});
            return true;
        },
        num_build_failures, error_msg);
    if (error) *error = error_msg;
    return sent;
}
}  // namespace

// The Remote ID part of ReportBeast reads the packets only when a feed agreed on bit 2, and only once they pass the
// Beast batch's check: a null header, or a round nobody takes, must neither crash nor send.
TEST(BeastCapsRemoteID, RoundChecksThePacketsOnlyWhenSomeoneTakesThem) {
    typedef SettingsManager S;
    const std::vector<Info> rid_sinks = {{1, S::kBeast, kBeastCapRemoteID}, {2, S::kBeastNoUAT, 0}};
    const std::vector<Info> no_rid_sinks = {{1, S::kBeast, 0}, {2, S::kBeast, kBeastCapADSL}, {3, S::kRaw, 0}};
    std::vector<RawRemoteIDPacket> rid = {
        RemoteIDPacket(RawRemoteIDPacket::kTransportBT4Legacy, kMAC, ODIDMessage(1, 0x1a), -72, kTs << 2)};
    bool ok = false;
    std::string error;

    // Null header, never validated: no bit 2 sink means it is never looked at.
    CompositeArray::RawPackets null_header;
    null_header.header = nullptr;
    null_header.len_bytes = 0;
    for (bool validated : {false, true}) {
        EXPECT_TRUE(ReportRound(no_rid_sinks, null_header, validated, &ok).empty());
        EXPECT_TRUE(ok);
        EXPECT_TRUE(ReportRound({}, null_header, validated, &ok).empty());
        EXPECT_TRUE(ok);
    }
    // Null header with a bit 2 sink: refused with the reason, nothing sent, even if the caller claims it validated.
    EXPECT_TRUE(ReportRound(rid_sinks, null_header, false, &ok, &error).empty());
    EXPECT_FALSE(ok);
    EXPECT_NE(error.find("null header"), std::string::npos);
    EXPECT_TRUE(ReportRound(rid_sinks, null_header, true, &ok).empty());
    EXPECT_FALSE(ok);

    // A header with Remote ID packets but no bit 2 sink: nothing sent.
    RemoteIDArray array(rid);
    EXPECT_TRUE(ReportRound(no_rid_sinks, array.packets, false, &ok).empty());
    EXPECT_TRUE(ok);

    // A header that claims more packets than the array holds: refused before any packet is read.
    RemoteIDArray short_array(rid);
    short_array.packets.header->num_remote_id_packets = 3;
    EXPECT_TRUE(ReportRound(rid_sinks, short_array.packets, false, &ok, &error).empty());
    EXPECT_FALSE(ok);
    EXPECT_NE(error.find("insufficient length"), std::string::npos);

    // Valid, with a bit 2 sink: the frame goes to that sink only.
    std::vector<SentTo> sent = ReportRound(rid_sinks, array.packets, false, &ok, &error);
    EXPECT_TRUE(ok);
    EXPECT_TRUE(error.empty());
    ASSERT_EQ(sent.size(), 1u);
    EXPECT_EQ(sent[0].sink, 1);
    EXPECT_EQ(sent[0].hex, Build(rid[0]));

    // Valid with no Remote ID packets: nothing sent.
    RemoteIDArray empty({});
    EXPECT_TRUE(ReportRound(rid_sinks, empty.packets, false, &ok).empty());
    EXPECT_TRUE(ok);
}
