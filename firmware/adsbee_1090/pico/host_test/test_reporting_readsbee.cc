// READSBEE feeds (readsbee feed protocol v2): the Remote ID frame 0xef, and BEAST feeds unchanged.
// Expected bytes come from readsbee's reference encoder (readsbee_proto.py enc_rid) and its vectors.json.
#include <string>
#include <vector>

#include "beast_utils.hh"
#include "feed_datagram.hh"
#include "gtest/gtest.h"
#include "settings.hh"

namespace {

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

// readsbee_proto._odid_msg: message type in the high nibble, protocol version 2, then 24 fill bytes.
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

std::string RemoteIDFrameHex(BeastReporter::RemoteIDTransportChar transport, int8_t dbm, const std::string& mac,
                             const std::vector<uint8_t>& odid) {
    uint8_t buf[BeastReporter::kRemoteIDBeastFrameMaxLenBytes];
    uint16_t len = BeastReporter::WriteRemoteIDBeastFrame(buf, transport, dbm, Hex(mac).data(), odid.data(), odid.size());
    return ToHex(buf, len);
}

const uint8_t kVectorMAC[6] = {0xc0, 0xff, 0xee, 0x1a, 0x00, 0x01};

}  // namespace

// vectors.json: rid_bt4_single, rid_wifi_beacon_pack, rid_bt5_coded.
TEST(ReadsbeeRemoteID, MatchesTheReferenceVectors) {
    EXPECT_EQ(RemoteIDFrameHex(BeastReporter::kRemoteIDTransportBT4Legacy, -72, "c0ffee1a0001", ODIDMessage(1, 0x1a)),
              "1aef62000000000000b8c0ffee1a1a0001121a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a");
    EXPECT_EQ(RemoteIDFrameHex(BeastReporter::kRemoteIDTransportWiFiBeacon, -60, "6a1a2b3c4d5e",
                               ODIDPack({ODIDMessage(0, 0x1a), ODIDMessage(1, 0x11), ODIDMessage(4, 0x44)})),
              "1aef77000000000000c46a1a1a2b3c4d5ef21903021a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1211111111111111111111111111111111111111111111111142444444444444444444444444444444444444444444444444");
    EXPECT_EQ(RemoteIDFrameHex(BeastReporter::kRemoteIDTransportBT5Coded, -95, "c0ffee1a0001", ODIDMessage(0, 0x22)),
              "1aef63000000000000a1c0ffee1a1a000102222222222222222222222222222222222222222222222222");
}

// The other transport letters, a pack of nine, no level, and a dBm byte and MAC that need escapes.
TEST(ReadsbeeRemoteID, MatchesTheReferenceEncoderOnEdgeCases) {
    std::vector<std::vector<uint8_t>> nine;
    for (int i = 0; i < 9; i++) nine.push_back(ODIDMessage(i % 6, i == 4 ? 0x1a : 0x30 + i));
    std::string pack9 = RemoteIDFrameHex(BeastReporter::kRemoteIDTransportBT5Extended1M, BeastReporter::kBeastNoLevelDbm,
                                         "c0ffee1a0001", ODIDPack(nine));
    EXPECT_EQ(pack9, "1aef6500000000000080c0ffee1a1a0001f2190902303030303030303030303030303030303030303030303030123131313131313131313131313131313131313131313131312232323232323232323232323232323232323232323232323232333333333333333333333333333333333333333333333333421a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a52353535353535353535353535353535353535353535353535023636363636363636363636363636363636363636363636361237373737373737373737373737373737373737373737373722383838383838383838383838383838383838383838383838");
    EXPECT_EQ(pack9.size() / 2, 2u + 1 + 6 + 1 + 6 + 3 + 9 * 25 + 1 + 24);  // Escapes: one in the MAC, 24 in a message.
    EXPECT_EQ(RemoteIDFrameHex(BeastReporter::kRemoteIDTransportWiFiNaN, 26, "1a1a1a1a1a1a", ODIDMessage(2, 0x55)),
              "1aef6e0000000000001a1a1a1a1a1a1a1a1a1a1a1a1a1a22555555555555555555555555555555555555555555555555");
    EXPECT_EQ(RemoteIDFrameHex(BeastReporter::kRemoteIDTransportBT4Legacy, -1, "000000000000", ODIDMessage(4, 0x00)),
              "1aef62000000000000ff00000000000042000000000000000000000000000000000000000000000000");
}

TEST(ReadsbeeRemoteID, LargestFrameFitsTheBuffer) {
    // Nine messages of 0x1a fill, and a dBm byte and MAC of 0x1a: everything that can need an escape does.
    std::vector<uint8_t> pack = ODIDPack(std::vector<std::vector<uint8_t>>(9, ODIDMessage(1, 0x1a)));
    const uint8_t mac[6] = {0x1a, 0x1a, 0x1a, 0x1a, 0x1a, 0x1a};
    uint8_t buf[BeastReporter::kRemoteIDBeastFrameMaxLenBytes];
    uint16_t len = BeastReporter::WriteRemoteIDBeastFrame(buf, BeastReporter::kRemoteIDTransportBT4Legacy, 0x1a, mac,
                                                          pack.data(), pack.size());
    EXPECT_EQ(len, 3 + 6 + 2 + 12 + 3 + 9 * (1 + 2 * 24));
    // Spec section 2's bound (2 + 1 + 2 x 241 = 485) with room for a 228-byte payload.
    EXPECT_GE(BeastReporter::kRemoteIDBeastFrameMaxLenBytes, 485);
    EXPECT_LE(len, BeastReporter::kRemoteIDBeastFrameMaxLenBytes);
}

TEST(ReadsbeeRemoteID, RefusesODIDBytesADecoderCouldNotBound) {
    uint8_t buf[BeastReporter::kRemoteIDBeastFrameMaxLenBytes];
    std::vector<uint8_t> msg = ODIDMessage(1, 0);
    EXPECT_EQ(BeastReporter::RemoteIDODIDLenBytes(msg.data(), 25), 25);
    EXPECT_EQ(BeastReporter::RemoteIDODIDLenBytes(msg.data(), 24), 0);  // Short message.
    EXPECT_EQ(BeastReporter::RemoteIDODIDLenBytes(msg.data(), 0), 0);
    std::vector<std::vector<uint8_t>> ten(10, ODIDMessage(1, 0));
    std::vector<uint8_t> pack = ODIDPack(ten);
    EXPECT_EQ(BeastReporter::RemoteIDODIDLenBytes(pack.data(), pack.size()), 0);  // Count over 9.
    pack[2] = 0;
    EXPECT_EQ(BeastReporter::RemoteIDODIDLenBytes(pack.data(), pack.size()), 0);  // Count 0.
    pack[2] = 2;
    EXPECT_EQ(BeastReporter::RemoteIDODIDLenBytes(pack.data(), pack.size()), 53);  // Only the counted messages.
    pack[1] = 24;
    EXPECT_EQ(BeastReporter::RemoteIDODIDLenBytes(pack.data(), pack.size()), 0);  // Message size not 25.
    pack[1] = 25;
    EXPECT_EQ(BeastReporter::RemoteIDODIDLenBytes(pack.data(), 52), 0);  // Pack cut short.
    EXPECT_EQ(BeastReporter::WriteRemoteIDBeastFrame(buf, BeastReporter::kRemoteIDTransportBT4Legacy, -50, kVectorMAC,
                                                     pack.data(), 52),
              0);
}

TEST(ReadsbeeRemoteID, BuildsFromAReceivedPacket) {
    uint8_t buf[BeastReporter::kRemoteIDBeastFrameMaxLenBytes];
    RawRemoteIDPacket packet;
    std::vector<uint8_t> msg = ODIDMessage(1, 0x1a);
    memcpy(packet.payload, msg.data(), msg.size());
    packet.payload_len_bytes = msg.size() + 3;  // Trailing bytes past the message are not sent.
    packet.rssi_dbm = -72;
    // NimBLE holds BLE addresses least significant byte first; the frame carries display order.
    for (int i = 0; i < 6; i++) packet.source_mac[i] = kVectorMAC[5 - i];
    packet.transport = RawRemoteIDPacket::kTransportBT4Legacy;
    EXPECT_EQ(ToHex(buf, BeastReporter::BuildRemoteIDBeastFrame(buf, packet)), "1aef62000000000000b8c0ffee1a1a0001121a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a");

    packet.transport = RawRemoteIDPacket::kTransportBT5LongRange;
    std::string coded = ToHex(buf, BeastReporter::BuildRemoteIDBeastFrame(buf, packet));
    EXPECT_EQ(coded.substr(0, 6), "1aef63");
    EXPECT_EQ(coded.substr(20, 14), "c0ffee1a1a0001");

    // A WiFi source address is already in frame order.
    memcpy(packet.source_mac, kVectorMAC, 6);
    packet.transport = RawRemoteIDPacket::kTransportWiFiBeacon;
    packet.rssi_dbm = 0;  // Unknown: no level.
    std::string wifi = ToHex(buf, BeastReporter::BuildRemoteIDBeastFrame(buf, packet));
    EXPECT_EQ(wifi.substr(0, 6), "1aef77");
    EXPECT_EQ(wifi.substr(18, 2), "80");
    EXPECT_EQ(wifi.substr(20, 14), "c0ffee1a1a0001");

    packet.transport = RawRemoteIDPacket::kTransportInvalid;
    EXPECT_EQ(BeastReporter::BuildRemoteIDBeastFrame(buf, packet), 0);
    packet.transport = RawRemoteIDPacket::kTransportWiFiBeacon;
    packet.payload_len_bytes = 0;
    EXPECT_EQ(BeastReporter::BuildRemoteIDBeastFrame(buf, packet), 0);
}

// UDP feeds split READSBEE batches at frame starts, the same as Beast.
TEST(ReadsbeeRemoteID, DatagramsSplitAtFrameStarts) {
    std::vector<uint8_t> batch = Hex("1aef62000000000000b8c0ffee1a1a0001121a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a"), coded = Hex("1aef63000000000000a1c0ffee1a1a000102222222222222222222222222222222222222222222222222");
    batch.insert(batch.end(), coded.begin(), coded.end());
    EXPECT_EQ(FeedDatagram::FrameLen(SettingsManager::kReadsbee, batch.data(), batch.size()), Hex("1aef62000000000000b8c0ffee1a1a0001121a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a1a").size());
    EXPECT_EQ(FeedDatagram::FrameLen(SettingsManager::kReadsbee, batch.data(), batch.size()),
              FeedDatagram::FrameLen(SettingsManager::kBeast, batch.data(), batch.size()));
}

TEST(ReadsbeeFeed, IsAFeedProtocolThatCarriesTheBeastFrames) {
    typedef SettingsManager S;
    EXPECT_STREQ(S::kReportingProtocolStrs[S::kReadsbee], "READSBEE");
    // Appended: stored values of the other protocols keep their meaning.
    EXPECT_EQ(S::kReadsbee, 13);
    EXPECT_EQ(S::kReadsbee, S::kNumProtocols - 1);
    EXPECT_EQ(S::DefaultFeedTransport(S::kReadsbee), S::kFeedTransportTCP);
    EXPECT_EQ(S::BeastFramesOf(S::kReadsbee), S::kBeast);
    for (uint16_t p = 0; p < S::kNumProtocols; p++) {
        if (p == S::kReadsbee) continue;
        EXPECT_EQ(S::BeastFramesOf(static_cast<S::ReportingProtocol>(p)), p) << S::kReportingProtocolStrs[p];
    }
}

// AT+FEED: the dump names READSBEE, the parser's name lookup maps it back, and the value survives Sanitize.
TEST(ReadsbeeFeed, FeedDumpReplaysAndSanitizeKeepsIt) {
    SettingsManager manager;
    manager.settings.feed_protocols[1] = SettingsManager::kReadsbee;
    strcpy(manager.settings.feed_uris[1], "feed.example.net");
    manager.settings.feed_ports[1] = 30005;
    manager.settings.feed_is_active[1] = true;
    testing::internal::CaptureStdout();
    manager.PrintAT();
    std::string dump = testing::internal::GetCapturedStdout();
    const std::string line = "AT+FEED=1,feed.example.net,30005,1,READSBEE\r\n";
    ASSERT_NE(dump.find(line), std::string::npos);
    // CommsManager::ATFeedCallback looks the fifth argument up in kReportingProtocolStrs.
    std::string name = line.substr(line.rfind(',') + 1, strlen("READSBEE"));
    uint16_t i = 0;
    while (i < SettingsManager::kNumProtocols && name != SettingsManager::kReportingProtocolStrs[i]) i++;
    EXPECT_EQ(i, SettingsManager::kReadsbee);

    EXPECT_FALSE(manager.Sanitize());
    EXPECT_EQ(manager.settings.feed_protocols[1], SettingsManager::kReadsbee);
    EXPECT_TRUE(SettingsManager::ProtocolInUse(manager.settings, SettingsManager::kReadsbee));
    manager.settings.feeds_enabled = false;
    EXPECT_FALSE(SettingsManager::ProtocolInUse(manager.settings, SettingsManager::kReadsbee));
}

TEST(ReadsbeeFeed, SerialInterfacesGetBeastInstead) {
    SettingsManager manager;
    manager.settings.reporting_protocols[SettingsManager::kCommsUART] = SettingsManager::kReadsbee;
    EXPECT_TRUE(manager.Sanitize());
    EXPECT_EQ(manager.settings.reporting_protocols[SettingsManager::kCommsUART], SettingsManager::kBeast);
}

// A BEAST feed's frames, fixed. READSBEE feeds carry these same bytes (BeastFramesOf), then 0xef frames.
TEST(ReadsbeeFeed, BeastFramesAreUnchanged) {
    uint8_t buf[BeastReporter::kUATADSBBeastFrameMaxLenBytes];
    std::string batch;
    DecodedModeSPacket long_frame("8d495066587f469bb826d21ad767", 0, -80, 50, 0xABABFF1AFFFFFF1A << 2);
    batch += ToHex(buf, BeastReporter::BuildModeSBeastFrame(buf, long_frame));
    DecodedModeSPacket short_frame("5d4840d6abcdef", 0, -55, 50, 0x0000001A2B3C << 2);
    batch += ToHex(buf, BeastReporter::BuildModeSBeastFrame(buf, short_frame));
    RawUATADSBPacket uat("00a66ef135445d525a0c05191190212048006cb82bc4d53a5b2bb0a8ec6e", -60, 0, 0x1a3456 << 2);
    batch += ToHex(buf, BeastReporter::BuildUATADSBBeastFrame(buf, uat));
    // Same bytes as readsbee_proto.py enc_mode_s and enc_uat (lower-case letter) for these packets.
    EXPECT_EQ(batch,
              "1a33ff1a1affffff1a1a088d495066587f469bb826d21a1ad7671a320000001a1a2b3c465d4840d6abcdef1aec730000001a1a34"
              "562d00a66ef135445d525a0c05191190212048006cb82bc4d53a5b2bb0a8ec6e");
}
