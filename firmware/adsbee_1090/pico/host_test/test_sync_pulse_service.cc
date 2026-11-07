#include <atomic>
#include <chrono>
#include <thread>

#include "gtest/gtest.h"
#include "mlat_time_sync.hh"
#include "sync_pulse_service.hh"

namespace {

std::atomic<uint64_t> g_mlat{1000};
std::atomic<int> g_num_emitted{0};
uint64_t FakeEmit() {
    g_num_emitted++;
    return g_mlat += 48'000'000;
}

std::atomic<bool> g_in_pulse{false};
uint64_t SlowEmit() {
    g_in_pulse = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    g_in_pulse = false;
    return 7;
}

using S = SyncPulseService;
constexpr uint8_t kWanted = ObjectDictionary::kMLATSyncFlagPulseWanted;
constexpr uint8_t kSynced = ObjectDictionary::kMLATSyncFlagSynced;

// One main loop pass at timestamp_ms, with core 1 serving the request at once.
void Pass(S& s, uint32_t timestamp_ms) {
    s.Update(timestamp_ms);
    s.Service();
    s.Update(timestamp_ms);  // Records the pulse just sent.
}

}  // namespace

TEST(SyncPulseService, OneHertzOnceAllowed) {
    g_num_emitted = 0;
    S s(&FakeEmit);
    for (uint32_t t = 0; t < 3000; t += 10) Pass(s, 1000 + t);
    EXPECT_EQ(g_num_emitted, 0);  // Not allowed yet.
    s.Allow();
    for (uint32_t t = 4000; t <= 9000; t += 10) Pass(s, t);
    EXPECT_EQ(g_num_emitted, 6);  // 4000, 5000, ... 9000.
    EXPECT_EQ(s.GetLastPulse().pulse_num, 6u);
}

TEST(SyncPulseService, NoEmitterNoPulses) {
    S s;
    s.Allow();
    for (uint32_t t = 0; t < 5000; t += 10) Pass(s, t);
    EXPECT_EQ(s.GetLastPulse().pulse_num, 0u);
}

TEST(SyncPulseService, EarlyPulseWhenWantedAfterTheMinInterval) {
    g_num_emitted = 0;
    S s(&FakeEmit);
    s.Allow();
    Pass(s, 10'000);
    ASSERT_EQ(g_num_emitted, 1);
    s.ReportFlags(S::kSubscriberESP32, kWanted);  // E.g. the ESP32 just rebooted.
    for (uint32_t t = 10'000; t < 10'000 + ObjectDictionary::kSyncPulseMinIntervalMs; t += 10) Pass(s, t);
    EXPECT_EQ(g_num_emitted, 1);  // Not before the min interval.
    Pass(s, 10'000 + ObjectDictionary::kSyncPulseMinIntervalMs);
    EXPECT_EQ(g_num_emitted, 2);
    // Still wanted (no rate yet would clear it; here it stays): every min interval, not faster.
    s.ReportFlags(S::kSubscriberESP32, kSynced);
    for (uint32_t t = 10'300; t < 11'300; t += 10) Pass(s, t);
    EXPECT_EQ(g_num_emitted, 2);
    Pass(s, 11'300);
    EXPECT_EQ(g_num_emitted, 3);  // Back to 1 Hz.
}

TEST(SyncPulseService, BlockedMeansNoPulseAndNoPendingRequest) {
    g_num_emitted = 0;
    S s(&FakeEmit);
    s.Allow();
    s.Update(5000);  // Requested.
    s.Block();       // E.g. CC1312 reset.
    s.Service();     // Core 1 comes around: nothing goes out.
    EXPECT_EQ(g_num_emitted, 0);
    s.Allow();
    s.Service();  // The request was dropped by Block().
    EXPECT_EQ(g_num_emitted, 0);
    Pass(s, 5010);
    EXPECT_EQ(g_num_emitted, 0);  // Next one on the schedule, or early when a coprocessor asks.
    s.ReportFlags(S::kSubscriberCC1312, kWanted);
    Pass(s, 5000 + ObjectDictionary::kSyncPulseMinIntervalMs);
    EXPECT_EQ(g_num_emitted, 1);
}

TEST(SyncPulseService, BlockWaitsOutAPulseInFlight) {
    S s(&SlowEmit);
    s.Allow();
    s.Update(5000);
    std::thread core1([&] { s.Service(); });
    while (!g_in_pulse) {
    }
    s.Block();
    EXPECT_FALSE(g_in_pulse);  // Returned only after the pulse ended.
    core1.join();
}

TEST(SyncPulseService, AStuckRequestTimesOut) {
    g_num_emitted = 0;
    S s(&FakeEmit);
    s.Allow();
    s.Update(5000);  // Requested, but core 1 never serves it.
    s.Update(5000 + S::kRequestTimeoutMs + 1);
    s.Update(6000);  // A new request.
    s.Service();
    EXPECT_EQ(g_num_emitted, 1);
}

TEST(SyncPulseService, EverySubscriberGetsEveryPulseOnce) {
    S s(&FakeEmit);
    s.Allow();
    Pass(s, 2000);
    ObjectDictionary::SyncPulse cc, esp;
    EXPECT_TRUE(s.HasNewPulse(S::kSubscriberCC1312));
    EXPECT_TRUE(s.HasNewPulse(S::kSubscriberESP32));
    ASSERT_TRUE(s.TakeNewPulse(S::kSubscriberCC1312, cc));
    EXPECT_FALSE(s.TakeNewPulse(S::kSubscriberCC1312, cc));  // Once.
    ASSERT_TRUE(s.TakeNewPulse(S::kSubscriberESP32, esp));
    EXPECT_EQ(cc.pulse_num, esp.pulse_num);
    EXPECT_EQ(cc.mlat_48mhz_counts, esp.mlat_48mhz_counts);
    // A subscriber that missed a pulse (link busy) gets the latest one, numbered so it can tell.
    Pass(s, 3000);
    Pass(s, 4000);
    ASSERT_TRUE(s.TakeNewPulse(S::kSubscriberESP32, esp));
    EXPECT_EQ(esp.pulse_num, cc.pulse_num + 2);
    ASSERT_TRUE(s.TakeNewPulse(S::kSubscriberCC1312, cc));
    EXPECT_EQ(cc.pulse_num, esp.pulse_num);
}

TEST(SyncPulseService, ESP32PulseWantedPath) {
    // The ESP32's flags come from its own MLATTimeSync: wanted after a reboot, cleared after one point (rate unknown),
    // synced after two; after its fit is lost (Restart) wanted again, and one point re-syncs.
    MLATTimeSync esp(MLATTimeSync::kMLATTicksPerESP32SystimerTick, 2);
    auto flags = [&] { return ObjectDictionary::MLATSyncFlags(esp.IsSynced(), esp.WantsPoint()); };
    EXPECT_EQ(flags(), kWanted);
    esp.AddSyncPoint(0, 0);
    EXPECT_EQ(flags(), 0);
    esp.AddSyncPoint(16'000'000, 48'000'000);
    EXPECT_EQ(flags(), kSynced);
    esp.Restart();
    EXPECT_EQ(flags(), kWanted);
    esp.AddSyncPoint(32'000'000 + 5, 96'000'000);
    EXPECT_EQ(flags(), kSynced);

    // And the service turns that into an early pulse.
    g_num_emitted = 0;
    S s(&FakeEmit);
    s.Allow();
    Pass(s, 20'000);
    s.ReportFlags(S::kSubscriberCC1312, kSynced);
    s.ReportFlags(S::kSubscriberESP32, kWanted);
    Pass(s, 20'000 + ObjectDictionary::kSyncPulseMinIntervalMs);
    EXPECT_EQ(g_num_emitted, 2);
}
