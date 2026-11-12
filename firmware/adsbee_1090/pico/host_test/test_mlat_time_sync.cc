#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

#include "esp32_clock_math.hh"
#include "gtest/gtest.h"
#include "mlat_time_sync.hh"
#include "object_dictionary.hh"

namespace {

// A local clock (ticks_per_us, 32 bits) against an RP2040-like 48 MHz MLAT counter whose crystal runs rate_error_ppm
// fast, both read at the same true time t_us.
struct TwoClocks {
    double rate_error_ppm;
    uint32_t local_start;
    uint64_t mlat_start;
    uint32_t local_ticks_per_us = 4;  // CC1312 radio timer.
    uint32_t Local(double t_us) const {
        return local_start + static_cast<uint32_t>(static_cast<uint64_t>(t_us * local_ticks_per_us));
    }
    uint64_t Mlat(double t_us) const {
        return mlat_start + static_cast<uint64_t>(t_us * 48 * (1 + rate_error_ppm * 1e-6));
    }
};

int64_t AbsError(uint64_t a, uint64_t b) {
    int64_t e = static_cast<int64_t>(a) - static_cast<int64_t>(b);
    return e < 0 ? -e : e;
}

}  // namespace

TEST(MLATTimeSync, NoTimestampBeforeTheFirstSyncPoint) {
    MLATTimeSync sync(MLATTimeSync::kMLATTicksPerRATTick);
    EXPECT_FALSE(sync.IsSynced());
    EXPECT_EQ(sync.ToMLAT48MHzCounts(12345), 0u);
}

TEST(MLATTimeSync, OnePointIsAnExactOffsetAcrossTheLocalClockWrap) {
    // The m1421 case: one crystal, so 12 MLAT counts per radio timer tick exactly.
    MLATTimeSync sync(MLATTimeSync::kMLATTicksPerRATTick);
    sync.AddSyncPoint(0xFFFFFF00, 1'000'000'000);
    EXPECT_EQ(sync.ToMLAT48MHzCounts(0xFFFFFF00), 1'000'000'000u);
    EXPECT_EQ(sync.ToMLAT48MHzCounts(0xFFFFFF00 - 4000), 1'000'000'000u - 48'000);  // 1 ms earlier.
    EXPECT_EQ(sync.ToMLAT48MHzCounts(0x00000100), 1'000'000'000u + 0x200 * 12);     // After the wrap.
}

TEST(MLATTimeSync, TracksTwoCrystalsToWellUnderAMicrosecond) {
    // The 1090 case: a sync pulse per second, the capture quantized to one local tick, the MLAT count read with a little
    // jitter. Packets between pulses must land within 0.5 us (24 counts) of the true time. Radio timer (CC1312) and
    // system timer (ESP32), both wrapping during the run.
    for (uint32_t ticks_per_us : {4u, 16u}) {
        for (double ppm : {-40.0, 0.0, 25.0}) {
            TwoClocks clocks = {ppm, 0xF0000000, 5'000'000'000ull, ticks_per_us};
            MLATTimeSync sync(MLATTimeSync::kMLATTicksPerUs / ticks_per_us, 2);
            std::mt19937 rng(7);
            std::uniform_real_distribution<double> jitter_us(-0.1, 0.1);
            std::uniform_real_distribution<double> within_s(0, 1'000'000);
            int64_t worst_error = 0;
            for (int pulse = 0; pulse < 600; pulse++) {
                double t_us = pulse * 1'000'000.0 + jitter_us(rng) + 0.3;
                EXPECT_TRUE(sync.AddSyncPoint(clocks.Local(t_us), clocks.Mlat(t_us + jitter_us(rng))));
                if (pulse < 1) continue;
                for (int i = 0; i < 20; i++) {
                    double packet_us = t_us + within_s(rng);  // Before the next pulse.
                    worst_error = std::max(
                        worst_error, AbsError(sync.ToMLAT48MHzCounts(clocks.Local(packet_us)), clocks.Mlat(packet_us)));
                }
            }
            EXPECT_LE(worst_error, 24) << ppm << " ppm, " << ticks_per_us << " ticks/us";
            EXPECT_NEAR(sync.GetRateErrorPPB() / 1000.0, ppm, 0.5) << ppm << " ppm, " << ticks_per_us << " ticks/us";
        }
    }
}

TEST(MLATTimeSync, TimestampsIncreaseWithTheLocalClock) {
    TwoClocks clocks = {25.0, 100, 7};
    MLATTimeSync sync(MLATTimeSync::kMLATTicksPerRATTick, 2);
    sync.AddSyncPoint(clocks.Local(0), clocks.Mlat(0));
    sync.AddSyncPoint(clocks.Local(1e6), clocks.Mlat(1e6));
    uint64_t last = 0;
    for (double t_us = 0; t_us < 3e6; t_us += 500'000) {  // A sender at 2 Hz.
        uint64_t mlat = sync.ToMLAT48MHzCounts(clocks.Local(t_us));
        EXPECT_GT(mlat, last);
        if (last != 0) EXPECT_NEAR(static_cast<double>(mlat - last), 24'000'000 * (1 + 25e-6), 2);
        last = mlat;
    }
}

TEST(MLATTimeSync, TwoCrystalsWaitForTheSecondPoint) {
    MLATTimeSync sync(MLATTimeSync::kMLATTicksPerRATTick, 2);
    sync.AddSyncPoint(0, 1000);
    EXPECT_FALSE(sync.IsSynced());
    EXPECT_EQ(sync.ToMLAT48MHzCounts(10), 0u);
    sync.AddSyncPoint(4'000'000, 1000 + 48'000'000 + 480);  // 10 ppm.
    EXPECT_TRUE(sync.IsSynced());
    EXPECT_EQ(sync.GetRateErrorPPB(), 10'000);
    EXPECT_EQ(sync.ToMLAT48MHzCounts(8'000'000), 1000u + 96'000'000 + 960);
}

TEST(MLATTimeSync, ARestartWaitsForTwoNewPoints) {
    // An RF re-open (AT+SUBG_MODE switch) restarts the radio timer with an unknown step.
    TwoClocks clocks = {-30.0, 0, 1'000'000};
    MLATTimeSync sync(MLATTimeSync::kMLATTicksPerRATTick, 2);
    for (int pulse = 0; pulse < 10; pulse++) sync.AddSyncPoint(clocks.Local(pulse * 1e6), clocks.Mlat(pulse * 1e6));
    ASSERT_TRUE(sync.IsSynced());
    sync.Restart();
    EXPECT_FALSE(sync.IsSynced());
    EXPECT_EQ(sync.ToMLAT48MHzCounts(clocks.Local(9.5e6)), 0u);  // Old offset is gone.
    clocks.local_start = 0x12345678;  // The step.
    sync.AddSyncPoint(clocks.Local(10.2e6), clocks.Mlat(10.2e6));
    EXPECT_FALSE(sync.IsSynced());
    sync.AddSyncPoint(clocks.Local(11.2e6), clocks.Mlat(11.2e6));
    EXPECT_TRUE(sync.IsSynced());
    EXPECT_LE(AbsError(sync.ToMLAT48MHzCounts(clocks.Local(11.5e6)), clocks.Mlat(11.5e6)), 3);
    EXPECT_NEAR(sync.GetRateErrorPPB(), -30'000, 1);
}

TEST(MLATTimeSync, ARateBeyond200PPMStartsTheFitAgain) {
    MLATTimeSync sync(MLATTimeSync::kMLATTicksPerRATTick, 2);
    EXPECT_TRUE(sync.AddSyncPoint(0, 0));
    // 1 ms off in one second (1000 ppm): not a crystal, e.g. an edge paired with the wrong pulse.
    EXPECT_FALSE(sync.AddSyncPoint(4'000'000, 48'000'000 + 48'000));
    EXPECT_FALSE(sync.IsSynced());
    EXPECT_EQ(sync.GetNumSyncPoints(), 1);  // The fit starts again from the new point.
    // 10 ppm from that point.
    EXPECT_TRUE(sync.AddSyncPoint(8'000'000, 96'000'000 + 48'000 + 480));
    EXPECT_TRUE(sync.IsSynced());
    EXPECT_EQ(sync.GetRateErrorPPB(), 10'000);
    // 199 ppm agrees, 201 ppm doesn't.
    MLATTimeSync edge(MLATTimeSync::kMLATTicksPerRATTick, 2);
    edge.AddSyncPoint(0, 0);
    EXPECT_TRUE(edge.AddSyncPoint(4'000'000, 48'000'000 + 199 * 48));
    MLATTimeSync beyond(MLATTimeSync::kMLATTicksPerRATTick, 2);
    beyond.AddSyncPoint(0, 0);
    EXPECT_FALSE(beyond.AddSyncPoint(4'000'000, 48'000'000 - 201 * 48));
}

TEST(MLATTimeSync, APointOffThePredictionStartsTheFitAgain) {
    MLATTimeSync sync(MLATTimeSync::kMLATTicksPerRATTick, 2);
    EXPECT_TRUE(sync.AddSyncPoint(0, 0));
    EXPECT_TRUE(sync.AddSyncPoint(4'000'000, 48'000'000 + 480));  // 10 ppm.
    // 49 us off the prediction one second on: agrees. The rate moves a quarter of the way to the 59 ppm measured.
    uint64_t mlat = 96'000'000 + 960 + 49 * 48;
    EXPECT_TRUE(sync.AddSyncPoint(8'000'000, mlat));
    EXPECT_TRUE(sync.IsSynced());
    EXPECT_EQ(sync.GetRateErrorPPB(), 22'250);
    // 101 us off the prediction (123 ppm from the last point, within 200 ppm of nominal): the fit starts again from it.
    mlat += 48'000'000 + 1068 + 101 * 48;
    EXPECT_FALSE(sync.AddSyncPoint(12'000'000, mlat));
    EXPECT_FALSE(sync.IsSynced());
    EXPECT_EQ(sync.ToMLAT48MHzCounts(12'000'000), 0u);
    // The next point agrees with it: synced again, at the rate between the two.
    EXPECT_TRUE(sync.AddSyncPoint(16'000'000, mlat + 48'000'000 + 480));
    EXPECT_TRUE(sync.IsSynced());
    EXPECT_EQ(sync.GetRateErrorPPB(), 10'000);
}

TEST(MLATTimeSync, OneHertzPulsesKeepTheFitAtTheCaptureNoise) {
    // Two crystals whose rate wanders like the bench 1090U's (a random walk, ~45 ppb rms change per minute, around
    // -52 ppm), the radio timer capture quantized to 0.25 us, pulses every second. Packets in between keep to about the
    // quantized capture and packet times.
    std::mt19937 rng(11);
    std::normal_distribution<double> step_ppb(0, 45.0 / std::sqrt(60.0));
    const double kRatTicksPerUs = 4;
    double rate_ppb = -52'000, t_us = 0, local_us = 0;  // local_us: the radio timer's view of t_us.
    MLATTimeSync sync(MLATTimeSync::kMLATTicksPerRATTick, 2);
    auto local = [&](double l_us) { return static_cast<uint32_t>(static_cast<uint64_t>(l_us * kRatTicksPerUs)); };
    auto mlat = [&](double us) { return static_cast<uint64_t>(us * 48); };
    std::vector<double> errors_us;
    const uint32_t interval_ms = ObjectDictionary::kSyncPulseIntervalMs;
    for (int pulse = 0; t_us < 3600e6; pulse++) {
        sync.AddSyncPoint(local(local_us), mlat(t_us));
        for (uint32_t ms = 0; ms < interval_ms; ms++) {
            // 1 ms steps: the rate walks, the radio timer runs at (1 + rate) of true time... seen from the MLAT clock.
            rate_ppb += step_ppb(rng) / std::sqrt(1000.0);
            t_us += 1000;
            local_us += 1000 / (1 + rate_ppb * 1e-9);
            if (pulse > 20 && ms % 97 == 0) {
                errors_us.push_back(
                    (static_cast<double>(sync.ToMLAT48MHzCounts(local(local_us))) - static_cast<double>(mlat(t_us))) /
                    48);
            }
        }
    }
    double sum_sq = 0, worst = 0;
    for (double e : errors_us) {
        sum_sq += e * e;
        worst = std::max(worst, std::abs(e));
    }
    double rms = std::sqrt(sum_sq / errors_us.size());
    printf("  %zu packets: rms %.3f us, max %.3f us\n", errors_us.size(), rms, worst);
    EXPECT_LT(rms, 0.15);
    EXPECT_LT(worst, 0.6);
}

TEST(MLATTimeSync, APointAtTheSameTimeOrBackwardsStartsTheFitAgain) {
    MLATTimeSync sync(MLATTimeSync::kMLATTicksPerRATTick, 2);
    sync.AddSyncPoint(4'000'000, 48'000'000);
    EXPECT_FALSE(sync.AddSyncPoint(4'000'000, 96'000'000));  // Same edge, another count.
    EXPECT_FALSE(sync.AddSyncPoint(8'000'000, 48'000'000));  // Count backwards.
    EXPECT_FALSE(sync.IsSynced());
}

TEST(MLATTimeSync, EachPointMovesTheRateAQuarterOfTheWay) {
    MLATTimeSync sync(MLATTimeSync::kMLATTicksPerRATTick, 2);
    uint64_t t_us = 0;
    for (int i = 0; i < 10; i++, t_us += 1'000'000) sync.AddSyncPoint(static_cast<uint32_t>(t_us * 4), t_us * 48);
    EXPECT_EQ(sync.GetRateErrorPPB(), 0);
    t_us += 1'000'000;  // A pulse missed; this one 1 us late: 500 ppb over the 2 s since the last point.
    EXPECT_TRUE(sync.AddSyncPoint(static_cast<uint32_t>(t_us * 4), t_us * 48 + 48));
    EXPECT_EQ(sync.GetRateErrorPPB(), 125);
    // The first rate after a restart is the measurement itself.
    sync.Restart();
    sync.AddSyncPoint(0, 0);
    sync.AddSyncPoint(4'000'000, 48'000'000 + 480);
    EXPECT_EQ(sync.GetRateErrorPPB(), 10'000);
}

TEST(MLATTimeSync, NoTimestampsFarFromTheLastSyncPoint) {
    MLATTimeSync sync(MLATTimeSync::kMLATTicksPerESP32SystimerTick);
    sync.AddSyncPoint(1000, 1'000'000'000'000ull);
    uint32_t limit = MLATTimeSync::kMaxExtrapolationUs * 16;
    EXPECT_NE(sync.ToMLAT48MHzCounts(1000 + limit), 0u);
    EXPECT_NE(sync.ToMLAT48MHzCounts(1000 - limit), 0u);
    EXPECT_EQ(sync.ToMLAT48MHzCounts(1000 + limit + 1), 0u);
    EXPECT_EQ(sync.ToMLAT48MHzCounts(1000 - limit - 1), 0u);
}

namespace {

// A coprocessor pairing sync pulse counts with the edges it captured. Pulse k goes out at true time k s (plus
// pulse_offset_us); local clock: 4 MHz radio timer, MLAT crystal 30 ppm fast, capture noise up to +-1 tick.
struct Coprocessor {
    TwoClocks clocks = {30.0, 0xFFF0'0000, 5'000'000};  // The local clock wraps early on.
    MLATTimeSync sync = MLATTimeSync(MLATTimeSync::kMLATTicksPerRATTick, 2);
    SyncPulsePairing pairing;
    std::mt19937 rng{1};
    double pulse_offset_us = 0;
    int num_added = 0, num_wrong = 0, num_restarts = 0;

    double T(double pulse) const { return pulse_offset_us + pulse * 1e6; }
    void Edge(double t_us) { pairing.AddEdge(clocks.Local(t_us) + static_cast<int>(rng() % 3) - 1); }
    void EdgeOf(int pulse) { Edge(T(pulse)); }
    uint64_t CountOf(int pulse) const { return clocks.Mlat(T(pulse)); }
    // Pulse's count arrives at now_us. True if it made a sync point.
    bool Count(int pulse, double now_us) {
        uint32_t edge_local = 0;
        if (!pairing.Pair(sync, CountOf(pulse), clocks.Local(now_us), edge_local)) return false;
        uint32_t ticks_off = edge_local - clocks.Local(T(pulse));
        if (ticks_off + 1 > 2) num_wrong++;  // Not within the capture noise of the pulse's edge.
        if (!sync.AddSyncPoint(edge_local, CountOf(pulse))) num_restarts++;
        num_added++;
        return true;
    }
    // Two points from counts 1 ms after their edges.
    void SyncAt(int pulse) {
        EdgeOf(pulse);
        Count(pulse, T(pulse) + 1e3);
        EdgeOf(pulse + 1);
        Count(pulse + 1, T(pulse + 1) + 1e3);
        ASSERT_TRUE(sync.IsSynced());
    }
    int64_t ErrorTicks(double t_us) const {
        return static_cast<int64_t>(sync.ToMLAT48MHzCounts(clocks.Local(t_us)) - clocks.Mlat(t_us));
    }
};

constexpr double kYoungUs = SyncPulsePairing::kMaxFirstEdgeAgeUs;

}  // namespace

TEST(SyncPulsePairing, TheFirstTwoPointsNeedAYoungEdge) {
    Coprocessor c;
    EXPECT_FALSE(c.Count(0, 50e3));  // No edge yet.
    c.EdgeOf(1);
    EXPECT_FALSE(c.Count(1, c.T(1) + kYoungUs + 50e3));  // Too old: if pulse 1's edge was missed, it's pulse 0's.
    EXPECT_EQ(c.sync.GetNumSyncPoints(), 0);
    c.EdgeOf(2);
    EXPECT_TRUE(c.Count(2, c.T(2) + kYoungUs - 50e3));
    c.EdgeOf(3);
    EXPECT_FALSE(c.Count(3, c.T(3) + kYoungUs + 50e3));  // No rate yet: a young edge again.
    c.EdgeOf(4);
    EXPECT_TRUE(c.Count(4, c.T(4) + 10e3));
    EXPECT_TRUE(c.sync.IsSynced());
    // From here on the prediction pairs, also a count that arrives after the next edge.
    c.EdgeOf(5);
    c.EdgeOf(6);
    EXPECT_TRUE(c.Count(5, c.T(6) + 1e3));
    EXPECT_EQ(c.num_wrong, 0);
    EXPECT_EQ(c.num_restarts, 0);
}

TEST(SyncPulsePairing, LateCountsPairWithTheirOwnEdge) {
    // Under output overload (RP2040 main loop passes over a second) a count can arrive after the next edge.
    for (double delay_us : {1e3, 400e3, 990e3, 1.001e6, 1.5e6, 1.99e6}) {
        SCOPED_TRACE(delay_us);
        Coprocessor c;
        c.pulse_offset_us = 123'456;
        c.SyncAt(0);
        int next_edge = 2;
        for (int pulse = 2; pulse < 300; pulse++) {
            double now = c.T(pulse) + delay_us;
            while (c.T(next_edge) <= now) c.EdgeOf(next_edge++);
            EXPECT_TRUE(c.Count(pulse, now));
            EXPECT_LE(std::abs(c.ErrorTicks(c.T(pulse) + 500e3)), 36);  // 0.75 us (capture noise here is +-0.25 us).
        }
        EXPECT_EQ(c.num_wrong, 0);
        EXPECT_EQ(c.num_restarts, 0);
    }
}

TEST(SyncPulsePairing, ALateCountWhoseEdgeWasMissedStartsTheFitAgain) {
    // Pulse 2's edge is missed and its count arrives just after pulse 3's: no kept edge is predicted, and pulse 3's is
    // young, so the count pairs with it. That point doesn't fit: the fit starts again from it, no timestamps until two
    // points agree.
    Coprocessor c;
    c.SyncAt(0);
    c.EdgeOf(3);
    EXPECT_TRUE(c.Count(2, c.T(3) + 1e3));
    EXPECT_FALSE(c.sync.IsSynced());
    for (int pulse = 3; pulse < 6; pulse++) {
        if (pulse > 3) c.EdgeOf(pulse);
        c.Count(pulse, c.T(pulse) + 2e3);
        EXPECT_EQ(c.sync.IsSynced(), pulse >= 4);
        if (c.sync.IsSynced()) EXPECT_LE(std::abs(c.ErrorTicks(c.T(pulse) + 500e3)), 48);
    }
    EXPECT_EQ(c.num_wrong, 1);
    EXPECT_EQ(c.num_restarts, 2);
}

TEST(SyncPulsePairing, ARepeatedCountCountsOnce) {
    // The ESP32 gets the latest count in every RP2040 status write, ~10 a second.
    Coprocessor c;
    c.EdgeOf(0);
    EXPECT_TRUE(c.Count(0, c.T(0) + 10e3));
    for (int i = 0; i < 9; i++) EXPECT_FALSE(c.Count(0, c.T(0) + 110e3 + i * 100e3));
    c.EdgeOf(1);
    EXPECT_TRUE(c.Count(1, c.T(1) + 10e3));
    EXPECT_EQ(c.sync.GetNumSyncPoints(), 2);
    EXPECT_EQ(c.num_restarts, 0);
}

TEST(SyncPulsePairing, AMissedEdgeGivesNoPoint) {
    Coprocessor c;
    for (int pulse = 0; pulse < 5; pulse++) {
        c.EdgeOf(pulse);
        c.Count(pulse, c.T(pulse) + 5e3);
    }
    // No edge for pulse 5: its count pairs with neither kept edge (a second and more away), and the latest is too old
    // for a new first point.
    EXPECT_FALSE(c.Count(5, c.T(5) + 5e3));
    c.EdgeOf(6);
    c.EdgeOf(7);
    EXPECT_TRUE(c.Count(6, c.T(7) + 5e3));
    EXPECT_TRUE(c.Count(7, c.T(7) + 6e3));
    EXPECT_EQ(c.num_wrong, 0);
    EXPECT_EQ(c.num_restarts, 0);
    EXPECT_EQ(c.num_added, 7);
}

TEST(SyncPulsePairing, AnExtraEdgeIsIgnored) {
    Coprocessor c;
    c.SyncAt(0);
    for (int pulse = 2; pulse < 20; pulse++) {
        if (pulse % 2) c.Edge(c.T(pulse) - 2e3);  // A glitch 2 ms before the edge...
        c.EdgeOf(pulse);
        if (pulse % 2 == 0) c.Edge(c.T(pulse) + 2e3);  // ... or after it, before the count.
        EXPECT_TRUE(c.Count(pulse, c.T(pulse) + 600e3));
    }
    EXPECT_EQ(c.num_wrong, 0);
    EXPECT_EQ(c.num_restarts, 0);
    // Before there is a rate, a glitch before the edge doesn't matter either: the count takes the latest edge.
    Coprocessor d;
    d.EdgeOf(0);
    d.Count(0, d.T(0) + 5e3);
    d.Edge(d.T(1) - 2e3);
    d.EdgeOf(1);
    EXPECT_TRUE(d.Count(1, d.T(1) + 5e3));
    EXPECT_EQ(d.num_wrong, 0);
    EXPECT_TRUE(d.sync.IsSynced());
}

TEST(SyncPulsePairing, TwoExtraEdgesCostTwoPointsButGiveNoWrongTimestamp) {
    // Two glitches after an edge push it out of the two kept: the count takes the latest glitch, which doesn't fit, so
    // the fit starts again from it; the next point doesn't agree with that one either, the one after does.
    Coprocessor c;
    c.SyncAt(0);
    c.EdgeOf(2);
    c.Edge(c.T(2) + 100e3);
    c.Edge(c.T(2) + 150e3);
    EXPECT_TRUE(c.Count(2, c.T(2) + 160e3));
    EXPECT_FALSE(c.sync.IsSynced());
    for (int pulse = 3; pulse < 10; pulse++) {
        c.EdgeOf(pulse);
        c.Count(pulse, c.T(pulse) + 5e3);
        if (c.sync.IsSynced()) EXPECT_LE(std::abs(c.ErrorTicks(c.T(pulse) + 500e3)), 48);
        EXPECT_EQ(c.sync.IsSynced(), pulse >= 4);
    }
    EXPECT_EQ(c.num_wrong, 1);
    EXPECT_EQ(c.num_restarts, 2);
}

TEST(SyncPulsePairing, ALocalClockRestartClearsTheEdges) {
    // The CC1312's radio timer restarts with its RF client (sub-GHz receiver restart, mode switch).
    Coprocessor c;
    c.SyncAt(0);
    c.EdgeOf(2);
    c.clocks.local_start += 0x4321'0000;  // The step.
    c.pairing.Clear();
    c.sync.Restart();
    EXPECT_FALSE(c.Count(2, c.T(2) + 100e3));  // Its edge was on the old clock.
    c.EdgeOf(3);
    EXPECT_TRUE(c.Count(3, c.T(3) + 50e3));  // A young edge: the first point after the restart.
    EXPECT_FALSE(c.sync.IsSynced());
    c.EdgeOf(4);
    EXPECT_TRUE(c.Count(4, c.T(4) + 100e3));
    EXPECT_TRUE(c.sync.IsSynced());
    EXPECT_LE(std::abs(c.ErrorTicks(c.T(4) + 0.5e6)), 24);
    EXPECT_EQ(c.num_wrong, 0);
    EXPECT_EQ(c.num_restarts, 0);
}

namespace {

// Pulses a second apart from first_us. The first num_late counts arrive 1 ms after the next pulse's edge (so a first
// point pairs with that edge), the others 5 ms after their own. Returns the number of counts after which the fit was
// synced but its timestamps more than 1 us off.
int CountsSyncedAPulseOff(double first_us, int num_late) {
    TwoClocks clocks = {30.0, 0, 5'000'000};
    auto t = [&](int pulse) { return first_us + pulse * 1e6; };
    MLATTimeSync sync(MLATTimeSync::kMLATTicksPerRATTick, 2);
    SyncPulsePairing pairing;
    int next_edge = 0, num_off = 0;
    for (int pulse = 0; pulse < 20; pulse++) {
        double arrival = pulse < num_late ? t(pulse + 1) + 1e3 : t(pulse) + 5e3;
        while (t(next_edge) <= arrival) pairing.AddEdge(clocks.Local(t(next_edge++)));
        uint32_t edge = 0;
        if (pairing.Pair(sync, clocks.Mlat(t(pulse)), clocks.Local(arrival), edge)) {
            sync.AddSyncPoint(edge, clocks.Mlat(t(pulse)));
        }
        if (sync.IsSynced() && AbsError(sync.ToMLAT48MHzCounts(clocks.Local(arrival)), clocks.Mlat(arrival)) > 48) {
            num_off++;
        }
    }
    EXPECT_TRUE(sync.IsSynced());
    return num_off;
}

}  // namespace

TEST(SyncPulsePairing, AFitAPulseOffEndsWithTheFirstCountInTime) {
    // A first count that arrives just after the next pulse's edge pairs with that edge (all a first point can go by is
    // a young edge). If the next count is in time, its point doesn't agree and the fit starts again. If it is late in
    // the same way, the two points are a pulse off but agree, and timestamps are a second off until the first count
    // that arrives before the next edge: the fit predicts neither kept edge for it, so it starts again from its own
    // (young) edge. Only with RP2040 main loop passes over a second can a count arrive in those few milliseconds.
    for (double first_us : {0.0, 12'345.0, 999'999.0}) {
        EXPECT_EQ(CountsSyncedAPulseOff(first_us, 0), 0);
        EXPECT_EQ(CountsSyncedAPulseOff(first_us, 1), 0);
        EXPECT_EQ(CountsSyncedAPulseOff(first_us, 2), 1);
        EXPECT_EQ(CountsSyncedAPulseOff(first_us, 5), 4);
    }
}

TEST(SyncPulseMessage, WireFormat) {
    // RP2040 and coprocessors exchange it as raw bytes over SPI.
    EXPECT_EQ(sizeof(ObjectDictionary::SyncPulse), 8u);
    ObjectDictionary::SyncPulse pulse = {.mlat_48mhz_counts = 0x0102030405060708ull};
    uint8_t bytes[sizeof(pulse)];
    memcpy(bytes, &pulse, sizeof(pulse));
    EXPECT_EQ(bytes[0], 0x08);
    EXPECT_EQ(bytes[7], 0x01);
}

TEST(ESP32ClockMath, CaptureToSystimerTicks) {
    using namespace esp32_clock_math;
    // Edge 1 us before the software capture: 80 APB ticks, 16 system timer ticks.
    EXPECT_EQ(CaptureToSystimerTicks(1000, 1080, 1'000'000), 1'000'000u - 16);
    // Rounded to the nearest system timer tick.
    EXPECT_EQ(CaptureToSystimerTicks(1000, 1082, 1'000'000), 1'000'000u - 16);
    EXPECT_EQ(CaptureToSystimerTicks(1000, 1083, 1'000'000), 1'000'000u - 17);
    // The capture timer wrapped between the edge and the software capture.
    EXPECT_EQ(CaptureToSystimerTicks(0xFFFFFFF6, 0x00000046, 1ull << 40), (1ull << 40) - 16);
}

TEST(ESP32ClockMath, WiFiRxClockRemovesTheCallbackLatency) {
    esp32_clock_math::WiFiRxClock clock;
    std::mt19937 rng(3);
    // Mostly a quick callback (40-45 us), sometimes a busy WiFi task (up to 900 us).
    std::uniform_int_distribution<int> quick_us(40, 45), busy_us(46, 900), pick(0, 9);
    auto latency_us = [&](std::mt19937& r) { return pick(r) < 7 ? quick_us(r) : busy_us(r); };
    const int64_t kMacMinusEspTimerUs = -123'456'789;
    int64_t worst_error = 0;
    for (int i = 0; i < 2000; i++) {
        int64_t frame_us = 5'000'000 + i * 10'240;  // ~100 beacons/s.
        uint32_t rx_timestamp = static_cast<uint32_t>(frame_us + kMacMinusEspTimerUs);
        int64_t out_us = clock.ToEspTimerUs(rx_timestamp, frame_us + 2400 + latency_us(rng), 2400);
        if (i < 256) continue;  // Let the shortest latency show up.
        worst_error = std::max<int64_t>(worst_error, std::abs(out_us - (frame_us + 40)));
    }
    EXPECT_LE(worst_error, 5);
}

TEST(ESP32ClockMath, LegacyPSDUAirTime) {
    using esp32_clock_math::LegacyPSDUAirTimeUs;
    EXPECT_EQ(LegacyPSDUAirTimeUs(0x00, 300), 2400u);  // 1 Mbps.
    EXPECT_EQ(LegacyPSDUAirTimeUs(0x03, 100), 73u);    // 11 Mbps, rounded up.
    EXPECT_EQ(LegacyPSDUAirTimeUs(0x0B, 100), 140u);   // 6 Mbps: 35 OFDM symbols (16 + 800 + 6 bits).
    EXPECT_EQ(LegacyPSDUAirTimeUs(0x0C, 100), 16u);    // 54 Mbps: 4 symbols of 216 bits.
    EXPECT_EQ(LegacyPSDUAirTimeUs(0x04, 100), 0u);     // Not a rate.
    EXPECT_EQ(LegacyPSDUAirTimeUs(0x1F, 100), 0u);
}

TEST(ESP32ClockMath, WiFiRxClockIgnoresFrameLength) {
    // Beacons of many APs at 1 Mbps, 60 to 400 bytes: the callback runs after the frame, but the offset mustn't follow
    // whichever frame is shortest at the moment (on the bench it moved by up to 1.8 ms after channel hops).
    esp32_clock_math::WiFiRxClock clock;
    std::mt19937 rng(5);
    std::uniform_int_distribution<int> len(60, 400), latency_us(30, 33), pick(0, 3);
    int64_t worst_error = 0;
    for (int i = 0; i < 5000; i++) {
        int64_t frame_us = 1'000'000 + i * 2'000;
        uint16_t frame_len = (i / 300) % 2 ? 60 + pick(rng) : len(rng);  // Stretches of short-frame-only channels.
        uint32_t air_us = esp32_clock_math::LegacyPSDUAirTimeUs(0x00, frame_len);
        int64_t out_us = clock.ToEspTimerUs(static_cast<uint32_t>(frame_us + 12345), frame_us + air_us + latency_us(rng),
                                            air_us);
        if (i > 300) worst_error = std::max<int64_t>(worst_error, std::abs(out_us - (frame_us + 30)));
    }
    EXPECT_LE(worst_error, 3);
}

TEST(ESP32ClockMath, WiFiRxClockConvertsHTFramesWithoutLearningFromThem) {
    esp32_clock_math::WiFiRxClock clock;
    EXPECT_EQ(clock.ToEspTimerUs(5000, 77'000, 0), 77'000);  // Nothing learned yet: callback time.
    clock.ToEspTimerUs(1000, 1'000'000 + 500 + 40, 500);    // Offset: 999'000 + 40.
    // An HT frame with a long callback delay: converted with the learned offset, offset unchanged.
    EXPECT_EQ(clock.ToEspTimerUs(2000, 1'001'000 + 9'000, 0), 1'001'040);
    EXPECT_EQ(clock.ToEspTimerUs(3000, 1'002'000 + 100 + 40, 100), 1'002'040);
}

TEST(ESP32ClockMath, WiFiRxClockFollowsMACClockSteps) {
    esp32_clock_math::WiFiRxClock clock;
    int64_t offset_us = 1000;
    auto frame = [&](int64_t frame_us, uint32_t latency_us) {
        return clock.ToEspTimerUs(static_cast<uint32_t>(frame_us + offset_us), frame_us + 300 + latency_us, 300);
    };
    for (int i = 0; i < 100; i++) EXPECT_EQ(frame(1'000'000 + i * 1000, 50 + i % 7), 1'000'000 + i * 1000 + 50);
    // The MAC clock jumps 5 ms ahead: frames look earlier, taken at once (the first one with its own latency).
    offset_us += 5000;
    EXPECT_EQ(frame(2'000'000, 60), 2'000'000 + 60);
    EXPECT_EQ(frame(2'001'000, 50), 2'001'000 + 50);
    // The MAC clock falls back 300 ms: past the latency limit, start over.
    offset_us -= 300'000;
    EXPECT_EQ(frame(3'000'000, 70), 3'000'000 + 70);
    EXPECT_EQ(frame(3'001'000, 50), 3'001'000 + 50);
    // A small fall back (200 us) is followed within two windows; up to kMinRiseUs may stay.
    offset_us -= 200;
    int64_t error = 0;
    for (int i = 0; i < 2 * esp32_clock_math::WiFiRxClock::kWindowNumFrames + 1; i++) {
        error = frame(4'000'000 + i * 1000, 50) - (4'000'000 + i * 1000 + 50);
    }
    EXPECT_EQ(error, 0);
    // The 32-bit MAC clock wraps (every 71 minutes).
    offset_us = 0x100000000ll - 4'000'000 - 500;
    for (int i = 0; i < 200; i++) {
        error = frame(4'000'000 + i * 10, 50) - (4'000'000 + i * 10 + 50);
    }
    EXPECT_EQ(error, 0);
}

TEST(ESP32ClockMath, RemoteIDTimestampsEndToEnd) {
    // An ESP32 whose crystal runs 18 ppm slow against the RP2040: sync pulses captured on the 80 MHz capture timer and
    // read back through the software capture, a fit on the 16 MHz system timer, WiFi frames converted from MAC time.
    const double kESP32PPM = -18.0;
    auto systimer_at = [&](double t_us) { return static_cast<uint64_t>((1e9 + t_us) * 16 * (1 + kESP32PPM * 1e-6)); };
    auto apb_at = [&](double t_us) { return static_cast<uint32_t>(static_cast<uint64_t>((1e9 + t_us) * 80 * (1 + kESP32PPM * 1e-6))); };
    auto mlat_at = [](double t_us) { return static_cast<uint64_t>(t_us * 48); };
    MLATTimeSync sync(MLATTimeSync::kMLATTicksPerESP32SystimerTick, 2);
    esp32_clock_math::WiFiRxClock wifi;
    int64_t worst_error = 0;
    for (int pulse = 1; pulse < 120; pulse++) {
        double edge_us = pulse * 1e6 + 0.37;
        double isr_us = edge_us + 3.5 + (pulse % 5);  // Interrupt latency varies; the capture doesn't.
        uint64_t edge = esp32_clock_math::CaptureToSystimerTicks(apb_at(edge_us), apb_at(isr_us), systimer_at(isr_us));
        sync.AddSyncPoint(static_cast<uint32_t>(edge), mlat_at(edge_us));
        if (pulse < 5) continue;
        for (int f = 0; f < 10; f++) {
            double frame_us = edge_us + 50'000 + f * 90'000;
            uint32_t mac_us = static_cast<uint32_t>(systimer_at(frame_us) / 16 + 777);
            int64_t now_us = static_cast<int64_t>(systimer_at(frame_us + 300 + 35 + f % 3) / 16);  // 300 us on air.
            int64_t rx_us = wifi.ToEspTimerUs(mac_us, now_us, 300);
            uint64_t mlat = sync.ToMLAT48MHzCounts(static_cast<uint32_t>(static_cast<uint64_t>(rx_us) * 16));
            if (pulse > 10) worst_error = std::max(worst_error, AbsError(mlat, mlat_at(frame_us + 35)));
        }
    }
    EXPECT_LE(worst_error, 48 + 24);  // 1 us of MAC clock resolution, plus the fit.
}
