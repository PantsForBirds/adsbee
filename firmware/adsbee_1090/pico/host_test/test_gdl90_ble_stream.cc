#include <vector>

#include "gdl90_ble_stream.hh"
#include "gtest/gtest.h"

namespace {

// A framed message: flag, n payload bytes (never 0x7E), flag.
std::vector<uint8_t> Frame(uint8_t first_byte, uint16_t num_payload_bytes) {
    std::vector<uint8_t> f = {0x7E};
    for (uint16_t i = 0; i < num_payload_bytes; i++) f.push_back(static_cast<uint8_t>((first_byte + i) % 0x7E));
    f.push_back(0x7E);
    return f;
}

std::vector<uint8_t> Cat(std::initializer_list<std::vector<uint8_t>> parts) {
    std::vector<uint8_t> out;
    for (const auto& p : parts) out.insert(out.end(), p.begin(), p.end());
    return out;
}

// Sends the whole queue in payloads of max_len and returns each payload.
std::vector<std::vector<uint8_t>> Drain(GDL90BLEStream& stream, uint16_t max_len) {
    std::vector<std::vector<uint8_t>> chunks;
    uint8_t buf[512];
    while (!stream.Empty()) {
        uint16_t n = stream.Peek(buf, max_len);
        EXPECT_GT(n, 0);
        EXPECT_LE(n, max_len);
        chunks.emplace_back(buf, buf + n);
        stream.Consume(n);
    }
    return chunks;
}

}  // namespace

TEST(GDL90BLEStream, PacksWholeMessagesIntoEachPayload) {
    GDL90BLEStream stream(1024);
    auto heartbeat = Frame(0x00, 9);   // 11 B
    auto ownship = Frame(0x10, 30);    // 32 B
    auto traffic = Frame(0x20, 30);    // 32 B
    auto all = Cat({heartbeat, ownship, traffic});
    ASSERT_TRUE(stream.Push(all.data(), all.size()));

    // 60 B payloads: heartbeat + ownship fit (43 B), traffic goes alone.
    auto chunks = Drain(stream, 60);
    ASSERT_EQ(chunks.size(), 2u);
    EXPECT_EQ(chunks[0], Cat({heartbeat, ownship}));
    EXPECT_EQ(chunks[1], traffic);
}

TEST(GDL90BLEStream, SplitsOnlyMessagesLongerThanAPayload) {
    GDL90BLEStream stream(2048);
    auto uplink = Frame(0x01, 440);  // A FIS-B uplink, 442 B.
    auto traffic = Frame(0x20, 30);
    auto all = Cat({uplink, traffic});
    ASSERT_TRUE(stream.Push(all.data(), all.size()));

    // Default ATT MTU: 20 B payloads. The uplink is split; traffic starts a payload of its own.
    auto chunks = Drain(stream, 20);
    std::vector<uint8_t> joined;
    bool payload_starts_at_traffic = false;
    for (const auto& c : chunks) {
        if (joined.size() == uplink.size()) payload_starts_at_traffic = true;
        joined.insert(joined.end(), c.begin(), c.end());
    }
    EXPECT_EQ(joined, all);
    EXPECT_TRUE(payload_starts_at_traffic);
    EXPECT_EQ(chunks.back().size(), 12u);  // 32 B of traffic = 20 + 12.
}

TEST(GDL90BLEStream, KeepsMessageBoundariesAfterASplit) {
    GDL90BLEStream stream(1024);
    auto a = Frame(0x01, 50);  // 52 B
    auto b = Frame(0x40, 5);   // 7 B
    auto c = Frame(0x50, 5);   // 7 B
    auto all = Cat({a, b, c});
    ASSERT_TRUE(stream.Push(all.data(), all.size()));

    // 40 B payloads: a is split 40 + 12. The 12 B tail ends a; b and c follow whole in the same payload.
    auto chunks = Drain(stream, 40);
    ASSERT_EQ(chunks.size(), 2u);
    EXPECT_EQ(chunks[0], std::vector<uint8_t>(a.begin(), a.begin() + 40));
    EXPECT_EQ(chunks[1], Cat({std::vector<uint8_t>(a.begin() + 40, a.end()), b, c}));
}

TEST(GDL90BLEStream, RefusesWhatDoesNotFitAndCountsIt) {
    GDL90BLEStream stream(100);
    auto m = Frame(0x00, 58);  // 60 B
    EXPECT_TRUE(stream.Push(m.data(), m.size()));
    EXPECT_FALSE(stream.Push(m.data(), m.size()));
    EXPECT_EQ(stream.Size(), 60u);
    EXPECT_EQ(stream.NumDroppedBytes(), 60u);
}

TEST(GDL90BLEStream, DroppableDataKeepsHalfTheQueueForTraffic) {
    GDL90BLEStream stream(100);
    auto small = Frame(0x00, 18);  // 20 B
    auto uplink = Frame(0x00, 38);  // 40 B
    EXPECT_TRUE(stream.Push(small.data(), small.size()));
    EXPECT_FALSE(stream.Push(uplink.data(), uplink.size(), /*droppable=*/true));  // 60 B > half.
    EXPECT_TRUE(stream.Push(uplink.data(), uplink.size()));                     // Traffic may fill it.
}

TEST(GDL90BLEStream, WrapsAroundTheBuffer) {
    GDL90BLEStream stream(64);
    auto m = Frame(0x00, 22);  // 24 B
    for (int round = 0; round < 10; round++) {
        ASSERT_TRUE(stream.Push(m.data(), m.size()));
        ASSERT_TRUE(stream.Push(m.data(), m.size()));
        auto chunks = Drain(stream, 30);
        ASSERT_EQ(chunks.size(), 2u);
        EXPECT_EQ(chunks[0], m);
        EXPECT_EQ(chunks[1], m);
    }
}

TEST(GDL90BLEStream, ClearForgetsAPartlySentMessage) {
    GDL90BLEStream stream(256);
    auto a = Frame(0x01, 50);
    auto b = Frame(0x40, 5);
    ASSERT_TRUE(stream.Push(a.data(), a.size()));
    uint8_t buf[64];
    stream.Consume(stream.Peek(buf, 20));  // Half of a sent.
    stream.Clear();
    ASSERT_TRUE(stream.Push(b.data(), b.size()));
    auto chunks = Drain(stream, 60);
    ASSERT_EQ(chunks.size(), 1u);
    EXPECT_EQ(chunks[0], b);
}

TEST(GDL90BLEStream, DropUnsentKeepsOnlyThePartlySentMessage) {
    GDL90BLEStream stream(256);
    auto a = Frame(0x01, 50);
    auto b = Frame(0x40, 5);
    auto c = Frame(0x50, 5);
    auto all = Cat({a, b, c});
    ASSERT_TRUE(stream.Push(all.data(), all.size()));
    uint8_t buf[64];
    stream.Consume(stream.Peek(buf, 20));  // 20 B of a sent.
    stream.DropUnsent();
    EXPECT_EQ(stream.Size(), a.size() - 20);
    EXPECT_EQ(stream.NumDroppedBytes(), b.size() + c.size());
    ASSERT_TRUE(stream.Push(c.data(), c.size()));
    auto chunks = Drain(stream, 60);
    ASSERT_EQ(chunks.size(), 1u);
    EXPECT_EQ(chunks[0], Cat({std::vector<uint8_t>(a.begin() + 20, a.end()), c}));

    // Nothing partly sent: everything goes.
    ASSERT_TRUE(stream.Push(b.data(), b.size()));
    stream.DropUnsent();
    EXPECT_TRUE(stream.Empty());
}

TEST(ReportRotation, StartsWhereTheQueueFilledAndWraps) {
    ReportRotation rotation;
    EXPECT_EQ(rotation.Start(), 0);
    rotation.End(10);  // Everything fit.
    EXPECT_EQ(rotation.Start(), 0);

    rotation.Refused(6);
    rotation.Refused(7);  // Only the first refusal counts.
    rotation.End(10);
    EXPECT_EQ(rotation.Start(), 6);

    // Sent 6..9 then 0..5; 2 didn't fit.
    rotation.Refused(2);
    rotation.End(10);
    EXPECT_EQ(rotation.Start(), 2);

    // Fewer aircraft than the start: back to 0.
    rotation.Refused(8);
    rotation.End(5);
    EXPECT_EQ(rotation.Start(), 0);
}

TEST(ReportRotation, EveryAircraftGoesOutWhenOnlyHalfFit) {
    // 10 aircraft, room for 5 per report: each aircraft is sent at least every other report.
    ReportRotation rotation;
    int sent[10] = {};
    for (int report = 0; report < 10; report++) {
        int room = 5;
        const uint16_t start = rotation.Start();
        for (int k = 0; k < 10; k++) {
            const uint16_t index = (start + k) % 10;
            if (room > 0) {
                room--;
                sent[index]++;
            } else {
                rotation.Refused(index);
            }
        }
        rotation.End(10);
    }
    for (int i = 0; i < 10; i++) EXPECT_EQ(sent[i], 5) << i;
}

TEST(GDL90BLEStream, AllocatesItsBuffer) {
    GDL90BLEStream stream(3 * 1024);
    EXPECT_TRUE(stream.Valid());
    auto m = Frame(0x00, 100);
    for (int i = 0; i < 30; i++) ASSERT_TRUE(stream.Push(m.data(), m.size()));  // 30 x 102 B = 3060 B.
    EXPECT_FALSE(stream.Push(m.data(), m.size()));
}
