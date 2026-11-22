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

// Pulse times a second apart from first_ms.
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
// hands the latest count to the CC1312 after its status read (handled in its main loop 0.5-3 ms later) and to the ESP32
// in its status writes (every pass, at most every 100 ms). Counts then arrive late, after the next edge, twice, or not
// at all. 1 % of edges are missed and 1 % of pulses bring a glitch edge somewhere in the next second. Every 50 s the
// CC1312 is reset (pulses blocked for 1.5 s) and its radio timer restarts, so each coprocessor's fit starts many times.
//
// Timestamps, checked every 10 ms while synced, stay within 1 us (rms under 0.2 us; capture noise 0.11 us rms on the
// CC1312, 0.04 us on the ESP32). Points from a wrong edge (a glitch, or a first count
// that arrived just after the next edge) only ever start the fit again.
TEST(SyncPulseService, CoprocessorsSyncOnAnyMainLoopPass) {
    for (uint32_t seed = 1; seed <= 4; seed++) {
        for (uint32_t pass_ms : {10u, 400u, 1100u, 2500u}) {
            SCOPED_TRACE(testing::Message() << pass_ms << " ms passes, seed " << seed);
            g_num_emitted = 0;
            S s(&FakeEmit);
            std::mt19937 rng(pass_ms * 100 + seed);
            std::normal_distribution<double> noise(0, 1);
            struct Copro {
                const char* name;
                double ticks_per_us;
                double ppm;
                double capture_noise_us;  // rms, on top of the tick (t-1185: 0.11 / 0.04 us rms in all).
                uint32_t local_offset;
                MLATTimeSync sync;
                SyncPulsePairing pairing;
                int num_points = 0, num_wrong = 0, num_restarts = 0, num_starts = 0, num_bad_timestamps = 0;
                int num_checks = 0, num_synced_checks = 0;
                double worst_error_us = 0, sum_sq_error_us = 0;
                uint32_t start_ms = 0, sync_ms_sum = 0, worst_sync_ms = 0;
                bool was_synced = false;
                uint32_t Local(double t_us) const {
                    return local_offset +
                           static_cast<uint32_t>(static_cast<uint64_t>(t_us * ticks_per_us * (1 + ppm * 1e-6)));
                }
            };
            Copro copro[S::kNumSubscribers] = {
                {"CC1312", 4, -52, 0.08, 0xFFF0'0000, MLATTimeSync(MLATTimeSync::kMLATTicksPerRATTick, 2)},
                {"ESP32", 16, 1.6, 0.03, 0x8000'0000, MLATTimeSync(MLATTimeSync::kMLATTicksPerESP32SystimerTick, 2)},
            };
            std::vector<std::pair<uint64_t, double>> pulses;  // MLAT count, true time.
            uint32_t last_point_ms[S::kNumSubscribers] = {};
            auto handle = [&](Copro& c, uint64_t count, double now_us) {
                uint32_t edge_local = 0;
                if (!c.pairing.Pair(c.sync, count, c.Local(now_us), edge_local)) return;
                auto it = std::find_if(pulses.begin(), pulses.end(), [&](auto& p) { return p.first == count; });
                ASSERT_NE(it, pulses.end());
                int32_t ticks_off = static_cast<int32_t>(edge_local - c.Local(it->second));
                if (std::abs(ticks_off) > 10 * c.ticks_per_us) c.num_wrong++;  // Not this pulse's edge.
                if (!c.sync.AddSyncPoint(edge_local, count)) c.num_restarts++;
                c.num_points++;
                last_point_ms[&c - copro] = static_cast<uint32_t>(now_us / 1000);
            };
            // Glitch edges to come: true time, coprocessor.
            std::vector<std::pair<double, int>> glitches;
            bool cc1312_up = true;
            uint64_t cc1312_pending = 0;  // Written over SPI, handled in the CC1312's main loop 0.5-3 ms later.
            double cc1312_pending_us = 0;
            uint32_t next_pass_ms = 0, last_esp32_status_ms = 0;
            constexpr uint32_t kEndMs = 600'000, kResetEveryMs = 50'000, kResetMs = 1'500;
            for (uint32_t t_us = 0; t_us < kEndMs * 1000; t_us += 100) {
                uint32_t t = t_us / 1000;
                g_now_us = t_us;
                int before = g_num_emitted;
                s.Service(t_us);  // Core 1.
                if (g_num_emitted != before) {
                    pulses.push_back({s.GetLastPulse().mlat_48mhz_counts, static_cast<double>(t_us)});
                    for (Copro& c : copro) {
                        if (&c == &copro[0] && !cc1312_up) continue;
                        if (rng() % 100 == 0) continue;  // Missed edge.
                        c.pairing.AddEdge(c.Local(t_us + noise(rng) * c.capture_noise_us));
                        if (rng() % 100 == 0) glitches.push_back({t_us + (rng() % 1'000'000), int(&c - copro)});
                    }
                }
                for (auto it = glitches.begin(); it != glitches.end();) {
                    if (it->first > t_us) {
                        ++it;
                        continue;
                    }
                    Copro& c = copro[it->second];
                    if (&c != &copro[0] || cc1312_up) c.pairing.AddEdge(c.Local(t_us));
                    it = glitches.erase(it);
                }
                if (cc1312_pending != 0 && t_us >= cc1312_pending_us) {
                    handle(copro[0], cc1312_pending, t_us);
                    cc1312_pending = 0;
                }
                if (t_us % 1000 != 0) continue;
                for (Copro& c : copro) {
                    // Drivers: a fit with no point for kSyncPointMaxAgeMs is dropped.
                    if (c.sync.GetNumSyncPoints() > 0 &&
                        t - last_point_ms[&c - copro] > ObjectDictionary::kSyncPointMaxAgeMs) {
                        c.sync.Restart();
                    }
                    bool up = &c != &copro[0] || cc1312_up;
                    bool synced = c.sync.IsSynced();
                    if (!synced && c.was_synced) c.start_ms = t;
                    if (synced && !c.was_synced) {
                        uint32_t took = t - c.start_ms;
                        c.sync_ms_sum += took;
                        c.worst_sync_ms = std::max(c.worst_sync_ms, took);
                        c.num_starts++;
                    }
                    c.was_synced = synced;
                    if (t % 10 != 0 || !up || t < 2000) continue;
                    c.num_checks++;
                    if (!synced) continue;
                    c.num_synced_checks++;
                    double error_us =
                        (static_cast<double>(c.sync.ToMLAT48MHzCounts(c.Local(t_us))) - MlatAt(t_us)) / 48;
                    c.worst_error_us = std::max(c.worst_error_us, std::abs(error_us));
                    c.sum_sq_error_us += error_us * error_us;
                    if (std::abs(error_us) > 1) c.num_bad_timestamps++;
                }
                if (t % kResetEveryMs == kResetEveryMs / 2) {
                    s.Block();  // CC1312 reset.
                    s.ReportAbsent(S::kSubscriberCC1312);
                    cc1312_up = false;
                    cc1312_pending = 0;
                }
                if (!cc1312_up && t % kResetEveryMs == kResetEveryMs / 2 + kResetMs) {  // A new radio timer start.
                    cc1312_up = true;
                    copro[0].local_offset += 0x1234'5678;
                    copro[0].pairing.Clear();
                    copro[0].sync.Restart();
                    copro[0].start_ms = t;
                    copro[0].was_synced = false;
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
                        cc1312_pending_us = t_us + 500 + rng() % 2500;
                    }
                }
                if (t - last_esp32_status_ms >= 100 || pass_ms >= 100) {
                    last_esp32_status_ms = t;
                    handle(copro[1], s.GetLastPulse().mlat_48mhz_counts, t_us);
                }
            }
            for (Copro& c : copro) {
                SCOPED_TRACE(c.name);
                printf(
                    "    seed %u, %4u ms passes, %-6s: %d pulses, %d points, %d wrong edge, %d restarts, %d syncs "
                    "(mean %u ms, worst %u ms), timestamps %.1f %% of the time, error rms %.3f us max %.3f us, "
                    "%d > 1 us\n",
                    seed, pass_ms, c.name, g_num_emitted.load(), c.num_points, c.num_wrong, c.num_restarts,
                    c.num_starts, c.num_starts ? c.sync_ms_sum / c.num_starts : 0, c.worst_sync_ms,
                    100.0 * c.num_synced_checks / c.num_checks, std::sqrt(c.sum_sq_error_us / c.num_synced_checks),
                    c.worst_error_us, c.num_bad_timestamps);
                EXPECT_EQ(c.num_bad_timestamps, 0);
                EXPECT_LT(std::sqrt(c.sum_sq_error_us / c.num_synced_checks), 0.2);
                // A wrong edge only ever starts the fit again.
                EXPECT_LE(c.num_wrong, c.num_restarts);
                EXPECT_TRUE(c.sync.IsSynced());
                // Timestamps nearly all the time, less the CC1312 resets and, on long passes, the time to sync again.
                EXPECT_GT(c.num_synced_checks, c.num_checks * (pass_ms <= 400 ? 95 : pass_ms <= 1100 ? 90 : 70) / 100);
            }
        }
    }
}
