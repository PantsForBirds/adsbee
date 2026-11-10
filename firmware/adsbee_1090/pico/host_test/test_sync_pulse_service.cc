#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <random>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "mlat_time_sync.hh"
#include "sync_pulse_service.hh"

namespace {

// True time in us; the RP2040's MLAT counter runs 10 ppm fast.
std::atomic<uint64_t> g_now_us{0};
uint64_t MlatAt(double t_us) { return 5'000'000 + static_cast<uint64_t>(t_us * 48 * (1 + 10e-6)); }
std::atomic<int> g_num_emitted{0};
std::atomic<uint64_t> g_last_emit_us{0};
uint64_t FakeEmit() {
    g_num_emitted++;
    g_last_emit_us = g_now_us.load();
    return MlatAt(g_now_us);
}

std::atomic<bool> g_in_pulse{false};
uint64_t SlowEmit() {
    g_in_pulse = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    g_in_pulse = false;
    return 7;
}

using S = SyncPulseService;

// Calls Service() every 10 us from start_ms to end_ms; returns the times (ms, rounded down) of the pulses sent.
std::vector<uint32_t> PulsesFrom(S& s, uint32_t start_ms, uint32_t end_ms) {
    std::vector<uint32_t> times;
    for (uint32_t t_us = start_ms * 1000; t_us < end_ms * 1000; t_us += 10) {
        g_now_us = t_us;
        int before = g_num_emitted;
        s.Service(t_us);
        if (g_num_emitted != before) times.push_back(t_us / 1000);
    }
    return times;
}

// Pulse times a second apart (plus the stagger, under 1 ms) from first_ms.
void ExpectOneHertz(const std::vector<uint32_t>& times, uint32_t first_ms, size_t num) {
    ASSERT_EQ(times.size(), num);
    EXPECT_EQ(times[0], first_ms);
    for (size_t i = 1; i < times.size(); i++) {
        EXPECT_GE(times[i] - times[i - 1], 1000u);
        EXPECT_LE(times[i] - times[i - 1], 1001u);
    }
}

}  // namespace

TEST(SyncPulseService, OneHertzFromTheMomentItIsAllowed) {
    g_num_emitted = 0;
    S s(&FakeEmit);
    EXPECT_TRUE(PulsesFrom(s, 0, 3000).empty());  // Not allowed yet.
    EXPECT_EQ(s.GetLastPulse().mlat_48mhz_counts, 0u);
    s.Allow();
    std::vector<uint32_t> times = PulsesFrom(s, 3500, 6800);
    ExpectOneHertz(times, 3500, 4);
    EXPECT_EQ(s.GetLastPulse().mlat_48mhz_counts, MlatAt(g_last_emit_us));
}

TEST(SyncPulseService, NoEmitterNoPulses) {
    S s;
    s.Allow();
    PulsesFrom(s, 0, 5000);
    EXPECT_EQ(s.GetLastPulse().mlat_48mhz_counts, 0u);
}

TEST(SyncPulseService, BlockedNoPulsesThenOneAsSoonAsAllowed) {
    S s(&FakeEmit);
    s.Allow();
    PulsesFrom(s, 0, 2100);  // Pulses at 0, 1000, 2000.
    s.Block();
    EXPECT_TRUE(PulsesFrom(s, 2100, 4000).empty());
    s.Allow();  // E.g. the CC1312 application answered its first status read after a reset.
    ExpectOneHertz(PulsesFrom(s, 4321, 6000), 4321, 2);
    // Allow() again (every status read) doesn't add pulses.
    s.Allow();
    EXPECT_EQ(PulsesFrom(s, 6000, 7000).size(), 1u);
}

TEST(SyncPulseService, BlockWaitsOutAPulseInFlight) {
    S s(&SlowEmit);
    s.Allow();
    std::thread core1([&] { s.Service(0); });
    while (!g_in_pulse) {
    }
    s.Block();
    EXPECT_FALSE(g_in_pulse);  // Returned only after the pulse ended.
    core1.join();
}

TEST(SyncPulseService, TheCC1312GetsEachPulseOnceTheESP32StatusAlwaysTheLatest) {
    S s(&FakeEmit);
    s.Allow();
    ObjectDictionary::SyncPulse pulse;
    EXPECT_FALSE(s.TakeNewPulse(pulse));  // None yet.
    PulsesFrom(s, 0, 10);
    EXPECT_TRUE(s.TakeNewPulse(pulse));
    EXPECT_EQ(pulse.mlat_48mhz_counts, MlatAt(0));
    EXPECT_FALSE(s.TakeNewPulse(pulse));
    EXPECT_EQ(s.GetLastPulse().mlat_48mhz_counts, MlatAt(0));
    // Missed a pulse (a long main loop pass): only the latest is given.
    EXPECT_EQ(PulsesFrom(s, 10, 2500).size(), 2u);
    EXPECT_TRUE(s.TakeNewPulse(pulse));
    EXPECT_EQ(pulse.mlat_48mhz_counts, MlatAt(g_last_emit_us));
    EXPECT_EQ(s.GetLastPulse().mlat_48mhz_counts, MlatAt(g_last_emit_us));
}

namespace {
std::atomic<uint64_t> g_seq_count{0};
uint64_t WordsChangeTogether() { return g_seq_count += 0x1'0000'0001ull; }
}  // namespace

TEST(SyncPulseService, CoreZeroNeverReadsATornCount) {
    // The 64-bit count is written on one core and read on the other: both words always belong to the same pulse.
    S s(&WordsChangeTogether);
    s.Allow();
    std::atomic<bool> stop{false};
    std::thread core1([&] {
        uint32_t t = 0;
        while (!stop) s.Service(t += S::kIntervalUs);
    });
    int num_reads = 0;
    while (num_reads < 2'000'000) {
        uint64_t c = s.GetLastPulse().mlat_48mhz_counts;
        ASSERT_EQ(c >> 32, c & 0xFFFF'FFFF);
        num_reads++;
    }
    stop = true;
    core1.join();
    EXPECT_GT(g_seq_count.load(), 1000u * 0x1'0000'0001ull);
}

// The whole path on main loop passes from 10 ms to 2.5 s (output overload): core 1 pulses about every second; core 0
// hands the latest count to the CC1312 after its status read (handled in its main loop 1 ms later) and to the ESP32 in
// its status writes (every pass, at most every 100 ms). Counts then arrive late, after the next edge, twice, or not at
// all. Midway the CC1312 is reset (pulses blocked, then allowed again) and its radio timer restarts.
//
// Points come from the pulse's own edge, except possibly a first point whose count arrived in the 1 ms after the next
// edge: the stagger then gets that fit restarted within a few pulses. Timestamps end well under a microsecond.
TEST(SyncPulseService, CoprocessorsSyncOnAnyMainLoopPass) {
    for (uint32_t pass_ms : {10u, 400u, 1100u, 2500u}) {
        SCOPED_TRACE(pass_ms);
        g_num_emitted = 0;
        S s(&FakeEmit);
        std::mt19937 rng(pass_ms);
        struct Copro {
            const char* name;
            double ticks_per_us;
            double ppm;
            uint32_t local_offset;
            MLATTimeSync sync;
            SyncPulsePairing pairing;
            int num_points = 0, num_wrong = 0, num_off_fit = 0, num_bad_timestamps = 0;
            uint32_t first_sync_ms = 0;
            uint32_t Local(double t_us) const {
                return local_offset +
                       static_cast<uint32_t>(static_cast<uint64_t>(t_us * ticks_per_us * (1 + ppm * 1e-6)));
            }
        };
        Copro copro[S::kNumSubscribers] = {
            {"CC1312", 4, -52, 0xFFF0'0000, MLATTimeSync(MLATTimeSync::kMLATTicksPerRATTick, 2)},
            {"ESP32", 16, 1.6, 0x8000'0000, MLATTimeSync(MLATTimeSync::kMLATTicksPerESP32SystimerTick, 2)},
        };
        std::vector<std::pair<uint64_t, double>> pulses;  // MLAT count, true time.
        auto handle = [&](Copro& c, uint64_t count, double now_us) {
            uint32_t edge_local = 0;
            if (!c.pairing.Pair(c.sync, count, c.Local(now_us), edge_local)) return;
            auto it = std::find_if(pulses.begin(), pulses.end(), [&](auto& p) { return p.first == count; });
            ASSERT_NE(it, pulses.end());
            if (static_cast<uint32_t>(edge_local - c.Local(it->second) + 1) > 2) c.num_wrong++;
            if (c.sync.AddSyncPoint(edge_local, count) != MLATTimeSync::kAdded) c.num_off_fit++;
            c.num_points++;
            if (c.sync.IsSynced()) {
                if (c.first_sync_ms == 0) c.first_sync_ms = static_cast<uint32_t>(now_us / 1000);
                int64_t error = static_cast<int64_t>(c.sync.ToMLAT48MHzCounts(c.Local(now_us)) - MlatAt(now_us));
                if (std::abs(error) > 48) c.num_bad_timestamps++;  // Off by more than 1 us.
            }
        };
        bool cc1312_up = true;
        uint64_t cc1312_pending = 0;  // Written over SPI, handled in the CC1312's main loop 1 ms later.
        uint32_t cc1312_pending_ms = 0;
        uint32_t next_pass_ms = 0, last_esp32_status_ms = 0;
        constexpr uint32_t kEndMs = 400'000, kResetMs = 200'000, kBackMs = 203'000;
        for (uint32_t t_us = 0; t_us < kEndMs * 1000; t_us += 100) {
            uint32_t t = t_us / 1000;
            g_now_us = t_us;
            int before = g_num_emitted;
            s.Service(t_us);  // Core 1.
            if (g_num_emitted != before) {
                pulses.push_back({s.GetLastPulse().mlat_48mhz_counts, static_cast<double>(t_us)});
                for (Copro& c : copro) {
                    if (&c == &copro[0] && !cc1312_up) continue;
                    c.pairing.AddEdge(c.Local(t_us) + static_cast<int>(rng() % 3) - 1);  // Capture noise.
                }
            }
            if (t_us % 1000 != 0) continue;
            if (cc1312_pending != 0 && t > cc1312_pending_ms) {
                handle(copro[0], cc1312_pending, t_us);
                cc1312_pending = 0;
            }
            if (t == kResetMs) {
                s.Block();  // CC1312 reset.
                s.ReportAbsent(S::kSubscriberCC1312);
                cc1312_up = false;
            }
            if (t == kBackMs) {  // The radio timer starts from a new value.
                cc1312_up = true;
                copro[0].local_offset += 0x1234'5678;
                copro[0].pairing.Clear();
                copro[0].sync.Restart();
            }
            if (t < next_pass_ms) continue;
            next_pass_ms = t + pass_ms * (90 + rng() % 21) / 100;  // Passes vary by +-10 %.
            // Core 0 main loop pass.
            if (cc1312_up) {  // Status read.
                s.ReportFlags(S::kSubscriberCC1312,
                              copro[0].sync.IsSynced() ? ObjectDictionary::kMLATSyncFlagSynced : 0);
                s.Allow();
                ObjectDictionary::SyncPulse pulse;
                if (s.TakeNewPulse(pulse)) {
                    cc1312_pending = pulse.mlat_48mhz_counts;
                    cc1312_pending_ms = t;
                }
            }
            if (t - last_esp32_status_ms >= 100 || pass_ms >= 100) {
                last_esp32_status_ms = t;
                handle(copro[1], s.GetLastPulse().mlat_48mhz_counts, t_us);
            }
        }
        // A pulse about a second, less the CC1312 reset.
        EXPECT_NEAR(g_num_emitted, (kEndMs - (kBackMs - kResetMs)) / 1000, 3);
        for (Copro& c : copro) {
            SCOPED_TRACE(c.name);
            printf(
                "    %u ms passes, %s: %d pulses, %d points, %d wrong edge, %d off the fit, %d bad timestamps, "
                "synced at %u ms\n",
                pass_ms, c.name, g_num_emitted.load(), c.num_points, c.num_wrong, c.num_off_fit, c.num_bad_timestamps,
                c.first_sync_ms);
            EXPECT_LE(c.num_wrong, 3);  // At most one first point a pulse off, caught within a few pulses.
            EXPECT_LE(c.num_bad_timestamps, 2);
            EXPECT_TRUE(c.sync.IsSynced());
            double t_us = kEndMs * 1000.0 - 300e3;
            int64_t error = static_cast<int64_t>(c.sync.ToMLAT48MHzCounts(c.Local(t_us)) - MlatAt(t_us));
            EXPECT_LE(std::abs(error), 24);  // 0.5 us.
            // Nearly every count delivered gives a point: one a pulse, or one a pass when passes are longer.
            int num_deliveries = std::min<int>(g_num_emitted, kEndMs / pass_ms);
            EXPECT_GE(c.num_points, num_deliveries * 8 / 10);
            // The first point needs a count within 250 ms of its edge: on long passes that takes a few passes.
            EXPECT_LE(c.first_sync_ms, pass_ms <= 10 ? 2'500u : pass_ms <= 1100 ? 8'000u : 60'000u);
        }
    }
}
