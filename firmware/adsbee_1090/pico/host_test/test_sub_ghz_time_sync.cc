#include <algorithm>
#include <cstdlib>
#include <random>

#include "gtest/gtest.h"
#include "sub_ghz_time_sync.hh"

namespace {

// A CC1312-like radio timer (4 MHz, 32 bits) against an RP2040-like 48 MHz MLAT counter whose crystal runs
// rate_error_ppm fast, both read at the same true time t_us.
struct TwoClocks {
    double rate_error_ppm;
    uint32_t rat_start;
    uint64_t mlat_start;
    uint32_t Rat(double t_us) const { return rat_start + static_cast<uint32_t>(static_cast<uint64_t>(t_us * 4)); }
    uint64_t Mlat(double t_us) const {
        return mlat_start + static_cast<uint64_t>(t_us * 48 * (1 + rate_error_ppm * 1e-6));
    }
};

}  // namespace

TEST(SubGHzTimeSync, NoTimestampBeforeTheFirstSyncPoint) {
    SubGHzTimeSync sync;
    EXPECT_FALSE(sync.IsSynced());
    EXPECT_EQ(sync.ToMLAT48MHzCounts(12345), 0u);
}

TEST(SubGHzTimeSync, OnePointIsAnExactOffsetAcrossTheRadioTimerWrap) {
    // The m1421 case: one crystal, so 12 MLAT counts per radio timer tick exactly.
    SubGHzTimeSync sync;
    sync.AddSyncPoint(0xFFFFFF00, 1'000'000'000);
    EXPECT_EQ(sync.ToMLAT48MHzCounts(0xFFFFFF00), 1'000'000'000u);
    EXPECT_EQ(sync.ToMLAT48MHzCounts(0xFFFFFF00 - 4000), 1'000'000'000u - 48'000);  // 1 ms earlier.
    EXPECT_EQ(sync.ToMLAT48MHzCounts(0x00000100), 1'000'000'000u + 0x200 * 12);     // After the wrap.
}

TEST(SubGHzTimeSync, TracksTwoCrystalsToWellUnderAMicrosecond) {
    // The 1090 case: a sync pulse per second, the capture quantized to one radio timer tick, the MLAT count read with
    // a little jitter. Packets between pulses must land within 0.5 us (24 counts) of the true time.
    for (double ppm : {-40.0, 0.0, 25.0}) {
        TwoClocks clocks = {ppm, 0xF0000000, 5'000'000'000ull};  // The radio timer wraps during the run.
        SubGHzTimeSync sync;
        std::mt19937 rng(7);
        std::uniform_real_distribution<double> jitter_us(-0.1, 0.1);
        std::uniform_real_distribution<double> within_s(0, 1'000'000);
        int64_t worst_error = 0;
        for (int pulse = 0; pulse < 600; pulse++) {
            double t_us = pulse * 1'000'000.0 + jitter_us(rng) + 0.3;
            sync.AddSyncPoint(clocks.Rat(t_us), clocks.Mlat(t_us + jitter_us(rng)));
            if (pulse < 8) continue;  // Let the rate estimate settle.
            for (int i = 0; i < 20; i++) {
                double packet_us = t_us + within_s(rng);  // Before the next pulse.
                int64_t error = static_cast<int64_t>(sync.ToMLAT48MHzCounts(clocks.Rat(packet_us))) -
                                static_cast<int64_t>(clocks.Mlat(packet_us));
                worst_error = std::max<int64_t>(worst_error, error < 0 ? -error : error);
            }
        }
        EXPECT_LE(worst_error, 24) << ppm << " ppm";
        EXPECT_NEAR(sync.GetRateErrorPPB() / 1000.0, ppm, 0.5) << ppm << " ppm";
    }
}

TEST(SubGHzTimeSync, TimestampsIncreaseWithTheRadioTimer) {
    TwoClocks clocks = {25.0, 100, 7};
    SubGHzTimeSync sync;
    sync.AddSyncPoint(clocks.Rat(0), clocks.Mlat(0));
    sync.AddSyncPoint(clocks.Rat(1e6), clocks.Mlat(1e6));
    uint64_t last = 0;
    for (double t_us = 0; t_us < 3e6; t_us += 500'000) {  // A sender at 2 Hz.
        uint64_t mlat = sync.ToMLAT48MHzCounts(clocks.Rat(t_us));
        EXPECT_GT(mlat, last);
        if (last != 0) EXPECT_NEAR(static_cast<double>(mlat - last), 24'000'000 * (1 + 25e-6), 2);
        last = mlat;
    }
}

TEST(SubGHzTimeSync, ABadPointRestartsTheFit) {
    SubGHzTimeSync sync;
    sync.AddSyncPoint(0, 0);
    sync.AddSyncPoint(4'000'000, 48'000'000 + 480);  // 10 ppm.
    EXPECT_EQ(sync.GetNumSyncPoints(), 2);
    EXPECT_EQ(sync.GetRateErrorPPB(), 10'000);
    // 1 ms off in one second (1000 ppm): not a crystal, e.g. a missed edge. Start over from this point.
    sync.AddSyncPoint(8'000'000, 96'000'000 + 48'000);
    EXPECT_EQ(sync.GetNumSyncPoints(), 1);
    EXPECT_EQ(sync.GetRateErrorPPB(), 0);
    EXPECT_EQ(sync.ToMLAT48MHzCounts(8'000'000), 96'048'000u);
    // So do a point more than 10 s later and one going backwards.
    sync.AddSyncPoint(8'000'000 + 41'000'000, 500'000'000);
    EXPECT_EQ(sync.GetNumSyncPoints(), 1);
    sync.AddSyncPoint(49'000'000 + 4'000'000, 100);
    EXPECT_EQ(sync.GetNumSyncPoints(), 1);
    sync.Reset();
    EXPECT_FALSE(sync.IsSynced());
}
