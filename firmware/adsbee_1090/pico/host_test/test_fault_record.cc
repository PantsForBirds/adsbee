#include "gtest/gtest.h"
#include "utils/fault_record.hh"

using namespace FaultRecord;

namespace {
Record MakeFault(uint8_t core, uint32_t pc, uint32_t lr) {
    Record r;
    r.core = core;
    r.frame_valid = true;
    r.ipsr = core ? 0 : 29;
    r.pc = pc;
    r.lr = lr;
    r.sp = core ? 0x20040de0 : 0x20041e80;
    return r;
}
State RoundTrip(const State& s) {
    uint32_t w[kNumWords];
    Encode(s, w);
    return Decode(w);
}
}  // namespace

TEST(FaultRecord, PowerOnZerosAreNoRecord) {
    uint32_t w[kNumWords] = {0, 0, 0, 0};
    State s = Decode(w);
    EXPECT_FALSE(s.has_record);
    EXPECT_EQ(s.count, 0);
}

TEST(FaultRecord, RoundTripKeepsEveryField) {
    State s = AddFault(State(), MakeFault(1, 0x1006c10a, 0x2000013b));
    State d = RoundTrip(s);
    ASSERT_TRUE(d.has_record);
    EXPECT_EQ(d.count, 1);
    EXPECT_EQ(d.record.core, 1);
    EXPECT_TRUE(d.record.frame_valid);
    EXPECT_EQ(d.record.ipsr, 0);
    EXPECT_EQ(d.record.pc, 0x1006c10au);
    EXPECT_EQ(d.record.lr, 0x2000013bu);
    EXPECT_EQ(d.record.sp, 0x20040de0u);
    State c0 = RoundTrip(AddFault(State(), MakeFault(0, 0x1002e94c, 0x09662602)));
    EXPECT_EQ(c0.record.core, 0);
    EXPECT_EQ(c0.record.ipsr, 29);
}

TEST(FaultRecord, AnyFlippedBitIsRejectedOrHarmless) {
    State s = AddFault(State(), MakeFault(1, 0x1006c10a, 0x2000013b));
    uint32_t w[kNumWords];
    Encode(s, w);
    int accepted_changed = 0;
    for (uint16_t word = 0; word < kNumWords; word++) {
        for (uint16_t bit = 0; bit < 32; bit++) {
            uint32_t c[kNumWords] = {w[0], w[1], w[2], w[3]};
            c[word] ^= 1u << bit;
            State d = Decode(c);
            if (d.has_record) accepted_changed++;
        }
    }
    EXPECT_EQ(accepted_changed, 0);  // Every single-bit error is caught by the check byte.
}

TEST(FaultRecord, SecondFaultBeforeReportOnlyCounts) {
    State first = AddFault(State(), MakeFault(1, 0x1006c10a, 0x2000013b));
    State second = RoundTrip(AddFault(RoundTrip(first), MakeFault(0, 0x1002e94c, 0x0)));
    ASSERT_TRUE(second.has_record);
    EXPECT_EQ(second.count, 2);
    EXPECT_EQ(second.record.core, 1);
    EXPECT_EQ(second.record.pc, 0x1006c10au);
}

TEST(FaultRecord, ReportedKeepsCountUntilPowerOn) {
    State s = RoundTrip(MarkReported(RoundTrip(AddFault(State(), MakeFault(0, 0x10001000, 0x10002001)))));
    EXPECT_FALSE(s.has_record);
    EXPECT_EQ(s.count, 1);
    State next = RoundTrip(AddFault(s, MakeFault(1, 0x10003000, 0x10004001)));
    ASSERT_TRUE(next.has_record);
    EXPECT_EQ(next.count, 2);
    EXPECT_EQ(next.record.pc, 0x10003000u);
}

TEST(FaultRecord, CountSaturates) {
    State s;
    for (int i = 0; i < 40; i++) s = RoundTrip(MarkReported(RoundTrip(AddFault(s, MakeFault(0, 0x10001000, 0)))));
    EXPECT_EQ(s.count, kMaxCount);
}

TEST(FaultRecord, ForeignScratchContentIsNoRecord) {
    // e.g. some other code's values in scratch 0-3.
    uint32_t w[kNumWords] = {0xDEADBEEF, 0x12345678, 0x0, 0xFFFFFFFF};
    EXPECT_FALSE(Decode(w).has_record);
    EXPECT_EQ(Decode(w).count, 0);
    uint32_t reported_with_junk[kNumWords];
    Encode(State(), reported_with_junk);
    reported_with_junk[1] = 5;
    EXPECT_EQ(Decode(reported_with_junk).count, 0);
}

TEST(FaultRecord, FormatForDeviceInfo) {
    char buf[80];
    Format(State(), buf, sizeof(buf));
    EXPECT_STREQ(buf, "none");
    State s = AddFault(State(), MakeFault(1, 0x1006c10a, 0x2000013b));
    Format(s, buf, sizeof(buf));
    EXPECT_STREQ(buf, "core 1 PC 0x1006c10a LR 0x2000013b (1 since power-on)");
    Format(MarkReported(s), buf, sizeof(buf));
    EXPECT_STREQ(buf, "none (1 since power-on)");
    Record bad;
    bad.core = 0;
    bad.sp = 0x1fffffe0;
    Format(AddFault(State(), bad), buf, sizeof(buf));
    EXPECT_STREQ(buf, "core 0, bad stack SP 0x1fffffe0 (1 since power-on)");
}

TEST(FaultRecord, Core1StallRoundTripsAndReads) {
    Record stall;
    stall.core = 1;
    stall.stalled = true;
    stall.pc = 2004;      // ms since core 1's loop last moved
    stall.lr = 123456;    // its passes
    State s = RoundTrip(AddFault(State(), stall));
    ASSERT_TRUE(s.has_record);
    EXPECT_TRUE(s.record.stalled);
    EXPECT_EQ(s.record.core, 1);
    EXPECT_EQ(s.record.pc, 2004u);
    EXPECT_EQ(s.record.lr, 123456u);
    char buf[80];
    Format(s, buf, sizeof(buf));
    EXPECT_STREQ(buf, "core 1 stalled 2004 ms (1 since power-on)");
    EXPECT_FALSE(RoundTrip(AddFault(State(), MakeFault(1, 0x1006c10a, 0x2000013b))).record.stalled);
    // A stall after an unreported fault only counts, like a second fault.
    State both = RoundTrip(AddFault(RoundTrip(AddFault(State(), MakeFault(0, 0x10001000, 0))), stall));
    EXPECT_FALSE(both.record.stalled);
    EXPECT_EQ(both.count, 2);
}

TEST(FaultRecord, Core1AliveWhileItsLoopMoves) {
    uint32_t last_passes = 0, last_move = 0;
    EXPECT_TRUE(Core1Alive(1, true, 100, 2000, last_passes, last_move));
    EXPECT_TRUE(Core1Alive(1, true, 2099, 2000, last_passes, last_move));     // 1999 ms without a pass
    EXPECT_FALSE(Core1Alive(1, true, 2100, 2000, last_passes, last_move));    // 2000 ms: stalled
    EXPECT_TRUE(Core1Alive(2, true, 2200, 2000, last_passes, last_move));     // it moved again
    EXPECT_EQ(last_move, 2200u);
}

TEST(FaultRecord, Core1StoppedOnPurposeIsNeverStalled) {
    uint32_t last_passes = 5, last_move = 0;
    EXPECT_TRUE(Core1Alive(5, false, 60000, 2000, last_passes, last_move));   // StopCore1: flash write, ESP32 flash
    // Restarted (StartCore1 counts a pass) but no loop pass yet: the clock starts from the restart.
    EXPECT_TRUE(Core1Alive(6, true, 60100, 2000, last_passes, last_move));
    EXPECT_TRUE(Core1Alive(6, true, 62000, 2000, last_passes, last_move));
    EXPECT_FALSE(Core1Alive(6, true, 62100, 2000, last_passes, last_move));
}

TEST(FaultRecord, Core1AliveAcrossTheMillisecondWrap) {
    uint32_t last_passes = 0, last_move = 0;
    EXPECT_TRUE(Core1Alive(1, true, 0xFFFFFF00u, 2000, last_passes, last_move));
    EXPECT_TRUE(Core1Alive(1, true, 0x00000500u, 2000, last_passes, last_move));   // 1536 ms later
    EXPECT_FALSE(Core1Alive(1, true, 0x00000800u, 2000, last_passes, last_move));  // 2304 ms later
}
