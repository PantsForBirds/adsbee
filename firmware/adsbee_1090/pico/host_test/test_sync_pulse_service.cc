#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <thread>
#include <vector>

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
    s.ReportFlags(S::kSubscriberCC1312, 0);  // Answers status reads, no fit yet.
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

TEST(SyncPulseService, NoNewPulseInThePassThatRecordsOne) {
    g_num_emitted = 0;
    S s(&FakeEmit);
    s.Allow();
    s.Update(1000);  // Requested.
    s.Service();
    ASSERT_EQ(g_num_emitted, 1);
    s.Update(2200);  // A long pass: records pulse 1, and the next one is due, but not before the subscribers take it.
    s.Service();
    EXPECT_EQ(g_num_emitted, 1);
    ObjectDictionary::SyncPulse pulse;
    ASSERT_TRUE(s.TakeNewPulse(S::kSubscriberESP32, pulse));
    EXPECT_EQ(pulse.pulse_num, 1u);
    // The CC1312 doesn't take it in time: it is dropped when the next pulse is requested, never sent after its edge.
    EXPECT_TRUE(s.HasNewPulse(S::kSubscriberCC1312));
    s.Update(3400);
    EXPECT_FALSE(s.HasNewPulse(S::kSubscriberCC1312));
    s.Service();
    EXPECT_EQ(g_num_emitted, 2);
}

// The main loop's order (adsbee.Update() with the CC1312 poll, then Update(), then esp32.Update()) on passes shorter and
// longer than the pulse interval: a coprocessor always gets a count before the next pulse's edge, so its last edge is
// the pulse's own, and SyncPulsePairing pairs every count it gets.
TEST(SyncPulseService, CountsArriveBeforeTheNextEdge) {
    for (uint32_t pass_ms : {5u, 400u, 1100u, 2500u}) {
        SCOPED_TRACE(pass_ms);
        g_num_emitted = 0;
        S s(&FakeEmit);
        s.Allow();
        s.ReportFlags(S::kSubscriberCC1312, 0);  // No fit: a pulse a second.
        s.ReportFlags(S::kSubscriberESP32, 0);
        SyncPulsePairing pairing[S::kNumSubscribers];
        uint32_t num_paired[S::kNumSubscribers] = {};
        auto take = [&](S::Subscriber subscriber) {
            ObjectDictionary::SyncPulse pulse;
            if (!s.TakeNewPulse(subscriber, pulse)) return;
            uint16_t edge_count = static_cast<uint16_t>(g_num_emitted.load());
            EXPECT_EQ(edge_count, pulse.pulse_num);  // The last edge is this pulse's.
            if (pairing[subscriber].Pair(edge_count, pulse.pulse_num, 0, 1, 1)) num_paired[subscriber]++;
        };
        for (uint32_t t = 1000; t < 60'000; t += pass_ms) {
            take(S::kSubscriberCC1312);  // In adsbee.Update().
            s.Update(t);
            take(S::kSubscriberESP32);
            s.Service();  // Core 1 serves the request at once.
        }
        EXPECT_GT(g_num_emitted, 10);
        EXPECT_GT(num_paired[S::kSubscriberESP32], 10u);
        EXPECT_GT(num_paired[S::kSubscriberCC1312], 10u);
    }
}

namespace {

// Request times of the pulses sent from start_ms to end_ms, in 10 ms passes.
std::vector<uint32_t> PulseTimes(S& s, uint32_t start_ms, uint32_t end_ms) {
    std::vector<uint32_t> times;
    for (uint32_t t = start_ms; t < end_ms; t += 10) {
        int before = g_num_emitted;
        Pass(s, t);
        if (g_num_emitted != before) times.push_back(t);
    }
    return times;
}

}  // namespace

TEST(SyncPulseService, SlowPulsesOnceEverySubscriberIsSynced) {
    g_num_emitted = 0;
    S s(&FakeEmit);
    s.Allow();
    s.ReportFlags(S::kSubscriberCC1312, kWanted);
    s.ReportFlags(S::kSubscriberESP32, kWanted);
    EXPECT_EQ(PulseTimes(s, 1000, 1010), std::vector<uint32_t>({1000}));
    // One with a fit, one without: a pulse a second.
    s.ReportFlags(S::kSubscriberCC1312, kSynced);
    s.ReportFlags(S::kSubscriberESP32, 0);
    EXPECT_EQ(PulseTimes(s, 1010, 3010), std::vector<uint32_t>({2000, 3000}));
    // Both synced: kFastPulsesWhenSynced more at 1 s, then the interval doubles up to the slow one.
    s.ReportFlags(S::kSubscriberESP32, kSynced);
    std::vector<uint32_t> times = PulseTimes(s, 3010, 200'000);
    std::vector<uint32_t> expected;
    uint32_t t = 3000, interval = 1000;
    for (int i = 0; i < S::kFastPulsesWhenSynced; i++) expected.push_back(t += 1000);
    while (true) {
        t += interval;
        if (t >= 200'000) break;
        expected.push_back(t);
        interval = std::min(interval * 2, S::kSlowIntervalMs);
    }
    EXPECT_EQ(times, expected);
    EXPECT_EQ(s.GetIntervalMs(), S::kSlowIntervalMs);
    // A coprocessor that loses its fit (restart, SUBG_MODE change) asks: a pulse right away, then 1 s again.
    s.ReportFlags(S::kSubscriberCC1312, kWanted);
    uint32_t last = expected.back();
    times = PulseTimes(s, 200'000, 200'010);
    EXPECT_EQ(times, std::vector<uint32_t>({200'000}));
    s.ReportFlags(S::kSubscriberCC1312, 0);
    EXPECT_EQ(PulseTimes(s, 200'010, 202'010), std::vector<uint32_t>({201'000, 202'000}));
    EXPECT_GT(200'000u, last);
}

TEST(SyncPulseService, ASubscriberThatDoesntAnswerDoesntHoldFastPulses) {
    g_num_emitted = 0;
    S s(&FakeEmit);
    s.Allow();
    s.ReportFlags(S::kSubscriberCC1312, kSynced);
    s.ReportFlags(S::kSubscriberESP32, 0);
    PulseTimes(s, 1000, 20'000);
    EXPECT_EQ(s.GetIntervalMs(), S::kIntervalMs);
    s.ReportAbsent(S::kSubscriberESP32);  // ESP32 link down.
    PulseTimes(s, 20'000, 120'000);
    EXPECT_EQ(s.GetIntervalMs(), S::kSlowIntervalMs);
    s.ReportFlags(S::kSubscriberESP32, kWanted);  // Back, rebooted.
    int before = g_num_emitted;
    PulseTimes(s, 120'000, 120'010);
    EXPECT_EQ(g_num_emitted, before + 1);
    EXPECT_EQ(s.GetIntervalMs(), S::kIntervalMs);
}

// The whole path under output overload: main loop passes of 1.1 and 2.5 s, counts delivered a pass after their edge.
// Both coprocessors sync, ramp to slow pulses and keep timestamps well under a microsecond.
TEST(SyncPulseService, CoprocessorsSyncUnderOverload) {
    for (uint32_t pass_ms : {10u, 1100u, 2500u}) {
        SCOPED_TRACE(pass_ms);
        g_num_emitted = 0;
        g_mlat = 1000;
        S s(&FakeEmit);
        s.Allow();
        struct Copro {
            MLATTimeSync sync = MLATTimeSync(MLATTimeSync::kMLATTicksPerRATTick, 2);
            SyncPulsePairing pairing;
            uint16_t edge_count = 0;
            uint32_t edge_local = 0;
            uint64_t edge_ms = 0;
        } copro[S::kNumSubscribers];
        // FakeEmit steps the MLAT count by 48e6 a pulse; the fake local clock (4 MHz, 20 ppm slow vs MLAT) follows it.
        auto local_of_mlat = [](uint64_t mlat) { return static_cast<uint32_t>(mlat / 12 * (1 - 20e-6)); };
        uint32_t edges_seen = 0;
        int max_error = 0;
        for (uint32_t t = 1000; t < 400'000; t += pass_ms) {
            auto take = [&](S::Subscriber sub) {
                Copro& c = copro[sub];
                s.ReportFlags(sub, ObjectDictionary::MLATSyncFlags(c.sync.IsSynced(), c.sync.WantsPoint()));
                ObjectDictionary::SyncPulse pulse;
                if (!s.TakeNewPulse(sub, pulse)) return;
                uint32_t age_us = static_cast<uint32_t>((t - c.edge_ms) * 1000);
                if (c.pairing.Pair(c.edge_count, pulse.pulse_num, age_us, ObjectDictionary::kSyncPulseMaxAgeMs * 1000,
                                   ObjectDictionary::kSyncPulseMaxMatchedAgeMs * 1000)) {
                    EXPECT_NE(MLATTimeSync::kRestarted, c.sync.AddSyncPoint(c.edge_local, pulse.mlat_48mhz_counts));
                    if (c.sync.IsSynced()) {
                        int e = static_cast<int>(static_cast<int64_t>(c.sync.ToMLAT48MHzCounts(c.edge_local)) -
                                                 static_cast<int64_t>(pulse.mlat_48mhz_counts));
                        max_error = std::max(max_error, std::abs(e));
                    }
                }
            };
            take(S::kSubscriberCC1312);
            s.Update(t);
            take(S::kSubscriberESP32);
            s.Service();
            if (static_cast<uint32_t>(g_num_emitted) != edges_seen) {
                edges_seen = g_num_emitted;
                for (Copro& c : copro) {
                    c.edge_count++;
                    c.edge_local = local_of_mlat(g_mlat);
                    c.edge_ms = t;
                }
            }
        }
        EXPECT_TRUE(copro[S::kSubscriberCC1312].sync.IsSynced());
        EXPECT_TRUE(copro[S::kSubscriberESP32].sync.IsSynced());
        EXPECT_EQ(s.GetIntervalMs(), S::kSlowIntervalMs);
        EXPECT_LE(max_error, 12);  // One radio timer tick.
        // A pulse a second would be ~400.
        EXPECT_LT(g_num_emitted, 400 / (S::kSlowIntervalMs / 1000) + 20);
    }
}
