// Beast capabilities check and the frames it enables (ADSBee Beast extensions (2026-11-06)). Expected bytes are read from
// their golden vectors, readsbee_vectors/vectors.json (checked by readsbee_vectors/readsbee_proto.py).
#include <climits>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "beast_utils.hh"
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
const uint16_t kAllBits = kBeastCapsKnown;  // 0x0f: the confirm frame of the dBm vectors.
const uint16_t kNoDbmBits = kBeastCapADSL | kBeastCapRemoteID | kBeastCapUATModeSClock;  // 0x0e.

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

std::string ModeS(const char* msg, uint64_t ts, int16_t dbm, uint16_t caps) {
    uint8_t buf[kModeSBeastFrameMaxLenBytes];
    DecodedModeSPacket packet(msg, 0, dbm, 50, ts << 2);
    return ToHex(buf, BuildModeSBeastFrame(buf, packet, caps));
}

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

std::string ADSL(ADSLChannelChar channel, uint64_t ts, int16_t dbm, uint8_t repaired, uint16_t caps) {
    uint8_t buf[kADSLBeastFrameMaxLenBytes];
    std::vector<uint8_t> onair = ADSLOnAir();
    return ToHex(buf, WriteADSLBeastFrame(buf, channel, ts, dbm, repaired, onair.data(), onair.size(), caps));
}

std::string RemoteID(char transport, int16_t dbm, const uint8_t mac[6], const std::vector<uint8_t>& odid,
                     uint16_t caps) {
    uint8_t buf[kRemoteIDBeastFrameMaxLenBytes];
    return ToHex(buf, WriteRemoteIDBeastFrame(buf, transport, 0, dbm, mac, odid.data(), odid.size(), caps));
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

const std::string kConfirmHex = "1ae543818f80";

}  // namespace

TEST(BeastCapsVectors, FileIsThere) {
    ASSERT_FALSE(VectorsJSON().empty()) << READSBEE_VECTORS_JSON;
    EXPECT_EQ(VectorHex("caps_confirm_dbm"), kConfirmHex);
}

TEST(BeastCapsVectors, CapabilitiesFrames) {
    EXPECT_EQ(Caps(kBeastCapsQuery, kNoDbmBits), VectorHex("caps_query"));
    EXPECT_EQ(Caps(kBeastCapsQuery, 0), VectorHex("caps_query_none"));
    EXPECT_EQ(Caps(kBeastCapsQuery, kAllBits), VectorHex("caps_query_dbm"));
    EXPECT_EQ(Caps(kBeastCapsConfirm, kAllBits), VectorHex("caps_confirm_dbm"));
    // What this firmware offers: dBm, plus Remote ID while receive is on. Never 0x1a.
    EXPECT_EQ(Caps(kBeastCapsQuery, kBeastCapsSendable), "1ae551818580");
    EXPECT_EQ(Caps(kBeastCapsQuery, kBeastCapSignalDbm), "1ae551818180");
    EXPECT_EQ(Caps(kBeastCapsAnswer, 1 << 13 | 1), "1ae54181" "81c0");
}

// Bits 1 and 3 wait for ADS-L (#409) and sub-GHz Mode S timestamps (#428) on main.
TEST(BeastCapsVectors, OnlyImplementedBitsAreOffered) {
    EXPECT_EQ(kBeastCapsSendable, kBeastCapSignalDbm | kBeastCapRemoteID);
    EXPECT_EQ(kBeastCapsSendable & (kBeastCapADSL | kBeastCapUATModeSClock), 0);
}

TEST(BeastCapsVectors, ModeSPowerByte) {
    EXPECT_EQ(ModeS(kShortMsg, 0x1234, -87, 0), VectorHex("mode_s_short_power_m87"));
    EXPECT_EQ(ModeS(kShortMsg, 0x1234, -105, 0), VectorHex("mode_s_short_power_m105"));
    EXPECT_EQ(ModeS(kShortMsg, 0x1234, -20, 0), VectorHex("mode_s_short_power_m20"));
    EXPECT_EQ(ModeS(kShortMsg, 0x1234, INT16_MIN, 0), VectorHex("mode_s_short_power_none"));
    EXPECT_EQ(ModeS(kLongMsg, kTs, -60, 0), VectorHex("mode_s_long_power_m60"));
}

TEST(BeastCapsVectors, ModeSDbm) {
    EXPECT_EQ(kConfirmHex + ModeS(kShortMsg, 0x1234, -87, kAllBits), VectorHex("mode_s_short_dbm_m87"));
    EXPECT_EQ(kConfirmHex + ModeS(kShortMsg, 0x1234, -105, kAllBits), VectorHex("mode_s_short_dbm_m105"));
    EXPECT_EQ(kConfirmHex + ModeS(kShortMsg, 0x1234, -20, kAllBits), VectorHex("mode_s_short_dbm_m20"));
    EXPECT_EQ(kConfirmHex + ModeS(kShortMsg, 0x1234, INT16_MIN, kAllBits), VectorHex("mode_s_short_dbm_none"));
    EXPECT_EQ(kConfirmHex + ModeS(kLongMsg, kTs, -60, kAllBits), VectorHex("mode_s_long_dbm_m60"));
    EXPECT_EQ(kConfirmHex + ModeS(kShortMsg, 0x1234, 26, kAllBits), VectorHex("mode_s_dbm_plus26_escaped"));
}

TEST(BeastCapsVectors, UAT) {
    std::vector<uint8_t> uat_s;
    for (uint8_t i = 0x10; i < 0x10 + 30; i++) uat_s.push_back(i);
    EXPECT_EQ(UATADSB(uat_s, 0x55, -95, 0), VectorHex("uat_s_power_m95"));
    EXPECT_EQ(kConfirmHex + UATADSB(uat_s, 0x55, -95, kBeastCapSignalDbm), VectorHex("uat_s_dbm_m95"));
    EXPECT_EQ(kConfirmHex + UATADSB(uat_s, kTs, -95, kBeastCapsFrameFormat), VectorHex("uat_S_dbm_m95"));
    EXPECT_EQ(UATUplink(kTs, -70, kBeastCapUATModeSClock), VectorHex("uat_U_power_m70"));
    EXPECT_EQ(kConfirmHex + UATUplink(kTs, -70, kBeastCapsFrameFormat), VectorHex("uat_U_dbm_m70"));
    EXPECT_EQ(UATUplink(kTs, -52, kBeastCapUATModeSClock), VectorHex("uat_U_uplink"));  // Power byte 90.
    // Bit 3 alone changes only the letter.
    std::string lower = UATADSB(uat_s, kTs, -95, 0), upper = UATADSB(uat_s, kTs, -95, kBeastCapUATModeSClock);
    EXPECT_EQ(lower.substr(0, 4) + "53" + lower.substr(6), upper);
}

TEST(BeastCapsVectors, ADSL) {
    EXPECT_EQ(ADSL(kADSLChannel868_4, kTs, -87, 2, kBeastCapSignalDbm), VectorHex("adsl_868_4"));
    EXPECT_EQ(ADSL(kADSLChannel868_2, 0, -108, 0, kBeastCapSignalDbm), VectorHex("adsl_868_2_unsynced"));
    EXPECT_EQ(ADSL(kADSLChannelUnknown, kTs, INT16_MIN, kADSLRepairedUnknown, kBeastCapSignalDbm),
              VectorHex("adsl_channel_unknown"));
    EXPECT_EQ(ADSL(kADSLChannel868_4, kTs, -87, 2, 0), VectorHex("adsl_868_4_power"));
    EXPECT_EQ(ADSL(kADSLChannel868_2, 0, -108, 0, 0), VectorHex("adsl_868_2_power_floor"));
    EXPECT_EQ(ADSL(kADSLChannelUnknown, kTs, INT16_MIN, kADSLRepairedUnknown, 0), VectorHex("adsl_power_none"));
}

TEST(BeastCapsVectors, ADSLRefusesOnAirBytesADecoderCouldNotBound) {
    uint8_t buf[kADSLBeastFrameMaxLenBytes];
    std::vector<uint8_t> onair = ADSLOnAir();
    EXPECT_EQ(WriteADSLBeastFrame(buf, kADSLChannel868_2, 0, -80, 0, onair.data(), onair.size() - 1, 0), 0);
    std::vector<uint8_t> too_long(33, 0x1a);
    too_long[0] = 32;
    EXPECT_EQ(WriteADSLBeastFrame(buf, kADSLChannel868_2, 0, -80, 0, too_long.data(), too_long.size(), 0), 0);
    // Worst case: L = 31, every byte 0x1a.
    std::vector<uint8_t> worst(32, 0x1a);
    worst[0] = 31;
    uint16_t len = WriteADSLBeastFrame(buf, kADSLChannelUnknown, 0x1a1a1a1a1a1a, 26, 0x1a, worst.data(), worst.size(),
                                       kBeastCapSignalDbm);
    EXPECT_EQ(len, 3 + 2 * 6 + 2 + 2 + 1 + 2 * 31);
    EXPECT_LE(len, kADSLBeastFrameMaxLenBytes);
    EXPECT_EQ(kADSLBeastFrameMaxLenBytes, 83);  // FEED-PROTOCOL.md section 2.
}

TEST(BeastCapsVectors, RemoteID) {
    EXPECT_EQ(RemoteID(kRemoteIDTransportBT4Legacy, -72, kMAC, ODIDMessage(1, 0x1a), kBeastCapSignalDbm),
              VectorHex("rid_bt4_single"));
    EXPECT_EQ(RemoteID(kRemoteIDTransportWiFiBeacon, -60, kPackMAC, VectorPack(), kBeastCapSignalDbm),
              VectorHex("rid_wifi_beacon_pack"));
    EXPECT_EQ(RemoteID(kRemoteIDTransportBT5Coded, -95, kMAC, ODIDMessage(0, 0x22), kBeastCapSignalDbm),
              VectorHex("rid_bt5_coded"));
    EXPECT_EQ(RemoteID(kRemoteIDTransportBT4Legacy, -72, kMAC, ODIDMessage(1, 0x1a), 0),
              VectorHex("rid_bt4_single_power"));
    EXPECT_EQ(RemoteID(kRemoteIDTransportBT4Legacy, -34, kMAC, ODIDMessage(1, 0x1a), 0),
              VectorHex("rid_bt4_power_saturated"));
    EXPECT_EQ(RemoteID(kRemoteIDTransportWiFiBeacon, -60, kPackMAC, VectorPack(), 0),
              VectorHex("rid_wifi_beacon_pack_power"));
    EXPECT_EQ(kConfirmHex + RemoteID(kRemoteIDTransportBT4Legacy, -72, kMAC, ODIDMessage(1, 0x1a), kAllBits),
              VectorHex("rid_bt4_dbm_after_confirm"));
}

// Whole streams: what a feed sends from connect on.
TEST(BeastCapsVectors, Streams) {
    std::string query = Caps(kBeastCapsQuery, kAllBits);
    EXPECT_EQ(FeedStart() + query + ModeS(kShortMsg, 1, -87, 0) + Caps(kBeastCapsConfirm, kAllBits) +
                  ModeS(kShortMsg, 2, -87, kAllBits) +
                  RemoteID(kRemoteIDTransportBT4Legacy, -70, kMAC, ODIDMessage(1, 0x33), kAllBits),
              VectorHex("stream_feed_start_dbm"));
    EXPECT_EQ(Caps(kBeastCapsConfirm, kNoDbmBits) + ModeS(kShortMsg, 3, -87, kNoDbmBits) +
                  ADSL(kADSLChannel868_2, kTs, -87, 1, kNoDbmBits) +
                  RemoteID(kRemoteIDTransportBT4Legacy, -87, kMAC, ODIDMessage(1, 0x33), kNoDbmBits),
              VectorHex("stream_no_dbm"));
    EXPECT_EQ(Caps(kBeastCapsConfirm, kAllBits) + ModeS(kShortMsg, 3, -87, kAllBits) +
                  ADSL(kADSLChannel868_2, kTs, -87, 1, kAllBits) +
                  RemoteID(kRemoteIDTransportBT4Legacy, -87, kMAC, ODIDMessage(1, 0x33), kAllBits),
              VectorHex("stream_dbm"));
}

// A server that never answers gets today's main output plus the 6 query bytes, and the query is the only addition.
TEST(BeastCapsPlain, StreamIsMainPlusTheQuery) {
    std::vector<uint8_t> payload =
        Hex("00a66ef135445d525a0c05191190212048006cb82bc4d53a5b2bb0a8ec6e");
    std::string frames = ModeS("8d495066587f469bb826d21ad767", 0xABABFF1AFFFFFF1A, -80, 0) +
                         ModeS("5d4840d6abcdef", 0x0000001A2B3C, -55, 0) + UATADSB(payload, 0x1a3456, -60, 0);
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
    stream += ModeS("8d495066587f469bb826d21ad767", 0xABABFF1AFFFFFF1A, -80, session.bits()) +
              ModeS("5d4840d6abcdef", 0x0000001A2B3C, -55, session.bits()) +
              UATADSB(payload, 0x1a3456, -60, session.bits());
    EXPECT_EQ(stream, FeedStart() + "1ae551818580" + main_frames);
    EXPECT_EQ(FeedStart(), "1ae430626565303030332d383137322d643138632d306265652d303030333831373264313863");
}

// The fix that goes with the Beast extensions: no level is power byte 0 (main sent 1, the first table entry).
TEST(BeastCapsPlain, SignalBytes) {
    uint8_t buf[2];
    auto power = [&](int16_t dbm) { return WriteRSSIdBmAsBeastPowerLevel(buf, dbm) == 1 ? buf[0] : 0x1ff; };
    auto dbm = [&](int16_t dbm) { return WriteRSSIdBmAsInt8(buf, dbm) == 1 ? buf[0] : 0x1ff; };
    EXPECT_EQ(power(INT16_MIN), 0);
    EXPECT_EQ(power(-100), 1);
    EXPECT_EQ(power(-130), 1);
    EXPECT_EQ(power(-40), 255);
    EXPECT_EQ(power(0), 255);
    EXPECT_EQ(power(-87), 4);
    EXPECT_EQ(dbm(INT16_MIN), 0x80);
    EXPECT_EQ(dbm(-127), 0x81);
    EXPECT_EQ(dbm(-200), 0x81);
    EXPECT_EQ(dbm(200), 0x7f);
    EXPECT_EQ(dbm(-87), 0xa9);
    EXPECT_EQ(dbm(0), 0x00);
    EXPECT_EQ(WriteRSSIdBmAsInt8(buf, 26), 2);  // 0x1a is escaped.
    EXPECT_EQ(WriteSignalByte(buf, -87, kBeastCapSignalDbm), 1);
    EXPECT_EQ(buf[0], 0xa9);
    EXPECT_EQ(WriteSignalByte(buf, -87, kBeastCapRemoteID), 1);  // Bits other than 0 keep the power byte.
    EXPECT_EQ(buf[0], 4);
}

// vectors.json server_to_feeder: what the scanner finds, all at once and one byte at a time.
TEST(BeastCapsScanner, MatchesTheVectors) {
    const char* names[] = {"answer_all",          "answer_none",       "answer_rid_only",  "answer_after_pings",
                           "answer_future",       "echo_of_query",     "answer_malformed", "answer_cut_by_frame",
                           "stock_readsb_silence", "answer_all_dbm",   "answer_no_dbm"};
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
    std::vector<uint8_t> answer = Hex("1ae541818580");
    const std::vector<uint8_t> ping = Hex("1a50001a1a05"), heartbeat = Hex("1a31000000000000000000"),
                               echo = Hex("1ae551818580"), unknown = Hex("1af5711a1a33000000");
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

TEST(BeastCapsSession, AnswerInTimeThenConfirm) {
    uint8_t buf[kBeastCapsFrameLenBytes];
    CapsSession session;
    EXPECT_EQ(session.state(), CapsSession::kPlain);
    EXPECT_EQ(ToHex(buf, session.Start(buf, kBeastCapsSendable, 1000)), "1ae551818580");
    EXPECT_EQ(session.state(), CapsSession::kWaiting);
    EXPECT_EQ(session.bits(), 0);
    std::vector<uint8_t> answer = Hex(VectorHex("answer_all_dbm"));  // Server accepts bits 0-3.
    EXPECT_TRUE(session.Receive(answer.data(), answer.size(), 1000 + kBeastCapsAnswerTimeoutMs - 1));
    EXPECT_EQ(session.state(), CapsSession::kAnswered);
    EXPECT_EQ(session.bits(), 0);  // Frames stay plain until the confirm is queued.
    EXPECT_EQ(ToHex(buf, session.WriteConfirm(buf)), "1ae543818580");  // Query bits AND answer bits.
    session.Confirm();
    EXPECT_EQ(session.state(), CapsSession::kAgreed);
    EXPECT_EQ(session.bits(), kBeastCapSignalDbm | kBeastCapRemoteID);
    // One answer per connection.
    EXPECT_FALSE(session.Receive(answer.data(), answer.size(), 2000));
    session.Update(1000000);
    EXPECT_EQ(session.bits(), kBeastCapSignalDbm | kBeastCapRemoteID);
}

TEST(BeastCapsSession, KnownBitsOnlyAndEmptyAnswers) {
    uint8_t buf[kBeastCapsFrameLenBytes];
    CapsSession session;
    session.Start(buf, kBeastCapsSendable, 0);
    std::vector<uint8_t> future = Hex(VectorHex("answer_future"));  // Version 2, Remote ID and unknown bit 9.
    ASSERT_TRUE(session.Receive(future.data(), future.size(), 10));
    EXPECT_EQ(ToHex(buf, session.WriteConfirm(buf)), "1ae543818480");
    session.Confirm();
    EXPECT_EQ(session.bits(), kBeastCapRemoteID);

    session.Start(buf, kBeastCapsSendable, 0);
    std::vector<uint8_t> none = Hex(VectorHex("answer_none"));
    ASSERT_TRUE(session.Receive(none.data(), none.size(), 10));
    EXPECT_EQ(ToHex(buf, session.WriteConfirm(buf)), "1ae543818080");  // Confirm is sent for 0 too.
    session.Confirm();
    EXPECT_EQ(session.state(), CapsSession::kAgreed);
    EXPECT_EQ(session.bits(), 0);

    // A v3 server (no bit 0): power bytes stay.
    session.Start(buf, kBeastCapsSendable, 0);
    std::vector<uint8_t> v3 = Hex(VectorHex("answer_no_dbm"));
    ASSERT_TRUE(session.Receive(v3.data(), v3.size(), 10));
    session.Confirm();
    EXPECT_EQ(session.bits(), kBeastCapRemoteID);
}

TEST(BeastCapsSession, FiveSecondTimeout) {
    uint8_t buf[kBeastCapsFrameLenBytes];
    std::vector<uint8_t> answer = Hex(VectorHex("answer_all_dbm"));
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

TEST(BeastCapsSession, AbandonAndReconnect) {
    uint8_t buf[kBeastCapsFrameLenBytes];
    std::vector<uint8_t> answer = Hex(VectorHex("answer_all_dbm"));
    CapsSession session;
    session.Start(buf, kBeastCapsSendable, 0);
    ASSERT_TRUE(session.Receive(answer.data(), answer.size(), 10));
    session.Abandon();  // The confirm couldn't be queued.
    EXPECT_EQ(session.state(), CapsSession::kPlain);
    EXPECT_EQ(session.bits(), 0);
    session.Confirm();  // No effect after Abandon.
    EXPECT_EQ(session.bits(), 0);
    // A new connection asks again; nothing is cached.
    session.Start(buf, kBeastCapSignalDbm, 100);
    EXPECT_EQ(session.state(), CapsSession::kWaiting);
    EXPECT_EQ(session.bits(), 0);
    ASSERT_TRUE(session.Receive(answer.data(), answer.size(), 200));
    session.Confirm();
    EXPECT_EQ(session.bits(), kBeastCapSignalDbm);
    session.Reset();
    EXPECT_EQ(session.state(), CapsSession::kPlain);
    EXPECT_EQ(session.bits(), 0);
}

// Feeds are grouped by protocol and the bits that change frame bytes (0 and 3); Remote ID doesn't split a group.
TEST(BeastCapsGroups, ByProtocolAndFrameFormat) {
    typedef SettingsManager S;
    const uint16_t sinks[] = {0, 1, 2, 3, 4, 5};
    const S::ReportingProtocol protocols[] = {S::kBeast, S::kBeast, S::kBeast, S::kBeastNoUAT, S::kBeast, S::kCSBee};
    const uint16_t caps[] = {0, kBeastCapSignalDbm | kBeastCapRemoteID, kBeastCapRemoteID, kBeastCapSignalDbm,
                             kBeastCapSignalDbm, kBeastCapSignalDbm};
    uint16_t selected[6];
    ASSERT_EQ(SelectBeastGroup(sinks, protocols, caps, 6, S::kBeast, uint16_t(0), selected), 2);
    EXPECT_EQ(selected[0], 0);
    EXPECT_EQ(selected[1], 2);
    ASSERT_EQ(SelectBeastGroup(sinks, protocols, caps, 6, S::kBeast, kBeastCapSignalDbm, selected), 2);
    EXPECT_EQ(selected[0], 1);
    EXPECT_EQ(selected[1], 4);
    ASSERT_EQ(SelectBeastGroup(sinks, protocols, caps, 6, S::kBeastNoUAT, kBeastCapSignalDbm, selected), 1);
    EXPECT_EQ(selected[0], 3);
    EXPECT_EQ(SelectBeastGroup(sinks, protocols, caps, 6, S::kBeast, kBeastCapUATModeSClock, selected), 0);
    EXPECT_EQ(SelectBeastGroup(sinks, protocols, caps, 6, S::kBeastNoUATUplink, uint16_t(0), selected), 0);
    // No caps array (serial interfaces, other platforms): one plain group per protocol, as before.
    EXPECT_EQ(SelectBeastGroup(sinks, protocols, static_cast<const uint16_t*>(nullptr), 6, S::kBeast, uint16_t(0),
                               selected),
              4);
    EXPECT_EQ(SelectBeastGroup(sinks, protocols, static_cast<const uint16_t*>(nullptr), 6, S::kBeast,
                               kBeastCapSignalDbm, selected),
              0);
    // Every sink lands in exactly one of the four formats.
    int total = 0;
    for (uint16_t format : {uint16_t(0), kBeastCapSignalDbm, kBeastCapUATModeSClock, kBeastCapsFrameFormat}) {
        total += SelectBeastGroup(sinks, protocols, caps, 6, S::kBeast, format, selected);
    }
    EXPECT_EQ(total, 4);
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
    EXPECT_EQ(ToHex(buf, BuildRemoteIDBeastFrame(buf, packet, kBeastCapSignalDbm)), VectorHex("rid_bt4_single"));
    EXPECT_EQ(ToHex(buf, BuildRemoteIDBeastFrame(buf, packet)), VectorHex("rid_bt4_single_power"));

    packet.transport = RawRemoteIDPacket::kTransportBT5LongRange;
    std::string coded = ToHex(buf, BuildRemoteIDBeastFrame(buf, packet));
    EXPECT_EQ(coded.substr(0, 6), "1aef63");
    EXPECT_EQ(coded.substr(20, 14), "c0ffee1a1a0001");

    // A WiFi source address is already in frame order. Level 0 means unknown.
    memcpy(packet.source_mac, kMAC, 6);
    packet.transport = RawRemoteIDPacket::kTransportWiFiBeacon;
    packet.rssi_dbm = 0;
    std::string wifi = ToHex(buf, BuildRemoteIDBeastFrame(buf, packet, kBeastCapSignalDbm));
    EXPECT_EQ(wifi.substr(0, 6), "1aef77");
    EXPECT_EQ(wifi.substr(18, 2), "80");
    EXPECT_EQ(wifi.substr(20, 14), "c0ffee1a1a0001");
    EXPECT_EQ(ToHex(buf, BuildRemoteIDBeastFrame(buf, packet)).substr(18, 2), "00");

    packet.transport = RawRemoteIDPacket::kTransportInvalid;
    EXPECT_EQ(BuildRemoteIDBeastFrame(buf, packet), 0);
    packet.transport = RawRemoteIDPacket::kTransportWiFiBeacon;
    packet.payload_len_bytes = 0;
    EXPECT_EQ(BuildRemoteIDBeastFrame(buf, packet), 0);
}

TEST(BeastCapsRemoteID, OtherLettersAndTheLargestFrame) {
    std::vector<std::vector<uint8_t>> nine;
    for (int i = 0; i < 9; i++) nine.push_back(ODIDMessage(i % 6, i == 4 ? 0x1a : 0x30 + i));
    // readsbee_proto.enc_rid(pack9, 0, None, "e", mac) and enc_rid(msg, 0, 26, "n", 1a x 6).
    EXPECT_EQ(RemoteID(kRemoteIDTransportBT5Extended1M, INT16_MIN, kMAC, ODIDPack(nine), kBeastCapSignalDbm),
              "1aef6500000000000080c0ffee1a1a0001f2190902303030303030303030303030303030303030303030303030123131313131"
              "313131313131313131313131313131313131312232323232323232323232323232323232323232323232323232333333333333"
              "333333333333333333333333333333333333421a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a"
              "1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a52353535353535353535353535353535353535353535353535023636363636363636"
              "363636363636363636363636363636361237373737373737373737373737373737373737373737373722383838383838383838"
              "383838383838383838383838383838");
    const uint8_t all_escapes[6] = {0x1a, 0x1a, 0x1a, 0x1a, 0x1a, 0x1a};
    EXPECT_EQ(RemoteID(kRemoteIDTransportWiFiNaN, 26, all_escapes, ODIDMessage(2, 0x55), kBeastCapSignalDbm),
              "1aef6e0000000000001a1a1a1a1a1a1a1a1a1a1a1a1a1a22555555555555555555555555555555555555555555555555");
    std::vector<uint8_t> pack = ODIDPack(std::vector<std::vector<uint8_t>>(9, ODIDMessage(1, 0x1a)));
    uint8_t buf[kRemoteIDBeastFrameMaxLenBytes];
    uint16_t len = WriteRemoteIDBeastFrame(buf, kRemoteIDTransportBT4Legacy, 0, 26, all_escapes, pack.data(),
                                           pack.size(), kBeastCapSignalDbm);
    EXPECT_EQ(len, 3 + 6 + 2 + 12 + 3 + 9 * (1 + 2 * 24));
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
