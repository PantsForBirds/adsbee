// LR2021StatusTracker (lr2021_status_tracker.hh): which command a Stat word reports on.
#include "gtest/gtest.h"
#include "lr2021_status_tracker.hh"

// Opcodes as in lr2021.hh.
static constexpr uint16_t kSetOokSyncWord = 0x0284;
static constexpr uint16_t kSetOokDetector = 0x0288;
static constexpr uint16_t kConfigFifoIrq = 0x011A;
static constexpr uint16_t kGetStatus = 0x0100;
static constexpr uint16_t kGetOokRxStats = 0x0286;

TEST(LR2021StatusTracker, ACommandFrameReportsTheCommandBeforeIt) {
    LR2021StatusTracker tracker;
    tracker.OnFrame(kSetOokSyncWord);
    tracker.OnFrame(kSetOokDetector);
    EXPECT_EQ(tracker.status_opcode(), kSetOokSyncWord);
    // A detector rejected with CMD_PERR shows up in the next command's frame: that frame names the detector.
    tracker.OnFrame(kConfigFifoIrq);
    EXPECT_EQ(tracker.status_opcode(), kSetOokDetector);
}

TEST(LR2021StatusTracker, TheLastCommandIsReportedByAStatusFrameAfterIt) {
    LR2021StatusTracker tracker;
    tracker.OnFrame(kSetOokDetector);
    tracker.OnFrame(kGetStatus);  // LR2021::CheckLastCommandStatus.
    EXPECT_EQ(tracker.status_opcode(), kSetOokDetector);
}

TEST(LR2021StatusTracker, AReadsDataFrameReportsTheRead) {
    LR2021StatusTracker tracker;
    tracker.OnFrame(kSetOokDetector);
    tracker.OnFrame(kGetOokRxStats);
    EXPECT_EQ(tracker.status_opcode(), kSetOokDetector);
    tracker.OnFrame(0);  // Data frame: zeros clocked out.
    EXPECT_EQ(tracker.status_opcode(), kGetOokRxStats);
    // The next command frame reports the read as well.
    tracker.OnFrame(kSetOokSyncWord);
    EXPECT_EQ(tracker.status_opcode(), kGetOokRxStats);
}

TEST(LR2021StatusTracker, NothingBeforeTheFirstCommandAfterAReset) {
    LR2021StatusTracker tracker;
    tracker.OnFrame(kSetOokSyncWord);
    tracker.OnFrame(kSetOokDetector);
    tracker.Reset();
    EXPECT_EQ(tracker.status_opcode(), 0);
    tracker.OnFrame(kSetOokSyncWord);
    EXPECT_EQ(tracker.status_opcode(), 0) << "the first command after a reset has no command before it";
}
