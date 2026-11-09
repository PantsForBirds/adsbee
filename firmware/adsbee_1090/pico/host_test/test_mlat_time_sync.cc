#include <algorithm>
#include <cstdlib>
#include <random>

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
                EXPECT_EQ(MLATTimeSync::kAdded, sync.AddSyncPoint(clocks.Local(t_us), clocks.Mlat(t_us + jitter_us(rng))));
                if (pulse < 8) continue;  // Let the rate estimate settle.
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

TEST(MLATTimeSync, ARestartKeepsTheRateSoOnePointResyncs) {
    // An RF re-open (AT+SUBG_MODE switch) restarts the radio timer with an unknown step.
    TwoClocks clocks = {-30.0, 0, 1'000'000};
    MLATTimeSync sync(MLATTimeSync::kMLATTicksPerRATTick, 2);
    for (int pulse = 0; pulse < 10; pulse++) sync.AddSyncPoint(clocks.Local(pulse * 1e6), clocks.Mlat(pulse * 1e6));
    ASSERT_TRUE(sync.IsSynced());
    sync.Restart();
    EXPECT_FALSE(sync.IsSynced());
    EXPECT_EQ(sync.ToMLAT48MHzCounts(clocks.Local(9.5e6)), 0u);  // Old offset is gone.
    EXPECT_TRUE(sync.HasRate());
    clocks.local_start = 0x12345678;  // The step.
    sync.AddSyncPoint(clocks.Local(10.2e6), clocks.Mlat(10.2e6));
    EXPECT_TRUE(sync.IsSynced());  // One point.
    EXPECT_LE(AbsError(sync.ToMLAT48MHzCounts(clocks.Local(11e6)), clocks.Mlat(11e6)), 3);
    EXPECT_NEAR(sync.GetRateErrorPPB(), -30'000, 1);
}

TEST(MLATTimeSync, ARestartBeforeTheRateIsKnownStillNeedsTwoPoints) {
    MLATTimeSync sync(MLATTimeSync::kMLATTicksPerRATTick, 2);
    sync.AddSyncPoint(0, 0);
    sync.Restart();
    sync.AddSyncPoint(100, 1000);
    EXPECT_FALSE(sync.IsSynced());
}

TEST(MLATTimeSync, AResetForgetsTheRate) {
    MLATTimeSync sync(MLATTimeSync::kMLATTicksPerRATTick, 2);
    sync.AddSyncPoint(0, 0);
    sync.AddSyncPoint(4'000'000, 48'000'000 + 480);
    ASSERT_TRUE(sync.HasRate());
    sync.Reset();  // CC1312 reset.
    EXPECT_FALSE(sync.HasRate());
    EXPECT_FALSE(sync.IsSynced());
    sync.AddSyncPoint(4'000, 7);
    EXPECT_FALSE(sync.IsSynced());
}

TEST(MLATTimeSync, ABadPointBeforeSyncStopsTimestampsUntilTwoPointsAgree) {
    MLATTimeSync sync(MLATTimeSync::kMLATTicksPerRATTick, 2);
    EXPECT_EQ(MLATTimeSync::kAdded, sync.AddSyncPoint(0, 0));
    // 1 ms off in one second (1000 ppm): not a crystal, e.g. an edge paired with the wrong pulse.
    EXPECT_EQ(MLATTimeSync::kRestarted, sync.AddSyncPoint(4'000'000, 48'000'000 + 48'000));
    EXPECT_FALSE(sync.IsSynced());
    EXPECT_EQ(MLATTimeSync::kAdded, sync.AddSyncPoint(8'000'000, 96'000'000 + 48'000 + 480));  // 10 ppm.
    EXPECT_TRUE(sync.IsSynced());
    EXPECT_EQ(sync.GetRateErrorPPB(), 10'000);
}

TEST(MLATTimeSync, OneBadPointOnceSyncedIsSkipped) {
    MLATTimeSync sync(MLATTimeSync::kMLATTicksPerRATTick, 2);
    EXPECT_EQ(MLATTimeSync::kAdded, sync.AddSyncPoint(0, 0));
    EXPECT_EQ(MLATTimeSync::kAdded, sync.AddSyncPoint(4'000'000, 48'000'000 + 480));  // 10 ppm.
    EXPECT_EQ(sync.GetRateErrorPPB(), 10'000);
    EXPECT_FALSE(sync.WantsPoint());
    // 1 ms off: skipped, timestamps go on from the fit, and the next point is wanted soon.
    EXPECT_EQ(MLATTimeSync::kSkipped, sync.AddSyncPoint(8'000'000, 96'000'000 + 48'000));
    EXPECT_TRUE(sync.IsSynced());
    EXPECT_TRUE(sync.WantsPoint());
    EXPECT_EQ(sync.ToMLAT48MHzCounts(8'000'000), 96'000'960u);
    EXPECT_EQ(sync.GetRateErrorPPB(), 10'000);
    // The next good point agrees with the fit (two intervals from its last point).
    EXPECT_EQ(MLATTimeSync::kAdded, sync.AddSyncPoint(12'000'000, 144'000'000 + 1440));
    EXPECT_FALSE(sync.WantsPoint());
    EXPECT_EQ(sync.ToMLAT48MHzCounts(12'000'000), 144'001'440u);
    // 30 us off after 1 s is off the fit too (a crystal moves well under 1 ppm in a second).
    EXPECT_EQ(MLATTimeSync::kSkipped, sync.AddSyncPoint(16'000'000, 192'000'000 + 1920 + 30 * 48));
    // Two in a row: the fit restarts from the second; its rate stays.
    EXPECT_EQ(MLATTimeSync::kRestarted, sync.AddSyncPoint(20'000'000, 240'000'000 + 2400 + 40 * 48));
    EXPECT_FALSE(sync.IsSynced());
    EXPECT_EQ(sync.GetRateErrorPPB(), 10'000);
    EXPECT_EQ(MLATTimeSync::kAdded, sync.AddSyncPoint(24'000'000, 288'000'000 + 2880 + 40 * 48));
    EXPECT_TRUE(sync.IsSynced());
}

TEST(MLATTimeSync, PointsFarApartOrBackwardsRestart) {
    MLATTimeSync sync(MLATTimeSync::kMLATTicksPerRATTick, 2);
    sync.AddSyncPoint(0, 0);
    // Further apart than kMaxSyncIntervalUs.
    uint32_t far = (MLATTimeSync::kMaxSyncIntervalUs + 1'000'000) * 4;
    EXPECT_EQ(MLATTimeSync::kRestarted, sync.AddSyncPoint(far, static_cast<uint64_t>(far) * 12));
    // Backwards.
    EXPECT_EQ(MLATTimeSync::kRestarted, sync.AddSyncPoint(far + 4'000'000, 100));
    EXPECT_FALSE(sync.IsSynced());
}

TEST(MLATTimeSync, TheRateIsTheSlopeOverTheLongestRecentIntervals) {
    // A 1 us error in one point moves the rate by 1 ppm over a 1 s interval, but only by 1/30 ppm over 30 s.
    MLATTimeSync sync(MLATTimeSync::kMLATTicksPerRATTick, 2);
    uint64_t t_us = 0;
    sync.AddSyncPoint(0, 0);
    for (uint32_t interval_s : {1u, 2u, 4u, 8u, 16u, 30u, 30u, 30u, 30u}) {
        t_us += interval_s * 1'000'000ull;
        sync.AddSyncPoint(static_cast<uint32_t>(t_us * 4), t_us * 48);
    }
    EXPECT_EQ(sync.GetRateErrorPPB(), 0);
    t_us += 30'000'000;
    EXPECT_EQ(MLATTimeSync::kAdded, sync.AddSyncPoint(static_cast<uint32_t>(t_us * 4), t_us * 48 + 48));  // 1 us late.
    EXPECT_GT(sync.GetRateErrorPPB(), 0);
    EXPECT_LE(sync.GetRateErrorPPB(), 1'000'000'000 / 30'000'000 + 1);
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

TEST(SyncPulsePairing, PairsOneEdgePerPulse) {
    SyncPulsePairing pairing;
    constexpr uint32_t kMaxAgeUs = ObjectDictionary::kSyncPulseMaxAgeMs * 1000;
    constexpr uint32_t kMatchedUs = ObjectDictionary::kSyncPulseMaxMatchedAgeMs * 1000;
    EXPECT_TRUE(pairing.Pair(1, 1, 3'000, kMaxAgeUs, kMatchedUs));
    EXPECT_TRUE(pairing.Pair(2, 2, 3'000, kMaxAgeUs, kMatchedUs));
    // No edge for this pulse (the radio was closed): the old edge isn't reused.
    EXPECT_FALSE(pairing.Pair(2, 3, 1'003'000, kMaxAgeUs, kMatchedUs));
    // An extra edge (two edges, one pulse).
    EXPECT_FALSE(pairing.Pair(4, 4, 3'000, kMaxAgeUs, kMatchedUs));
    EXPECT_TRUE(pairing.Pair(5, 5, 3'000, kMaxAgeUs, kMatchedUs));
    // A lost message: two pulses and two edges since the last one, the last edge is this pulse's.
    EXPECT_TRUE(pairing.Pair(7, 7, 3'000, kMaxAgeUs, kMatchedUs));
    // A late message (a long main loop pass): one edge for one pulse since the last message still pairs.
    EXPECT_TRUE(pairing.Pair(8, 8, kMaxAgeUs + 1, kMaxAgeUs, kMatchedUs));
    // Too late even for that.
    EXPECT_FALSE(pairing.Pair(9, 9, kMatchedUs + 1, kMaxAgeUs, kMatchedUs));
    // A missed edge after a lost message: one edge for two pulses is the older pulse's, and too old.
    EXPECT_FALSE(pairing.Pair(10, 11, 1'000'000, kMaxAgeUs, kMatchedUs));
    // Back in step.
    EXPECT_TRUE(pairing.Pair(11, 12, 3'000, kMaxAgeUs, kMatchedUs));
    // Edge count wraps.
    pairing.SkipEdges(0xFFFF);
    EXPECT_TRUE(pairing.Pair(0, 13, 3'000, kMaxAgeUs, kMatchedUs));
}

TEST(SyncPulsePairing, EdgesBeforeAClockRestartDontCount) {
    SyncPulsePairing pairing;
    EXPECT_TRUE(pairing.Pair(1, 1, 3'000, 250'000, 5'000'000));
    pairing.SkipEdges(2);  // Edge 2 came in on the old clock.
    EXPECT_FALSE(pairing.Pair(2, 2, 3'000, 250'000, 5'000'000));
    EXPECT_TRUE(pairing.Pair(3, 3, 3'000, 250'000, 5'000'000));
}

TEST(SyncPulsePairing, TheRP2040RestartingItsPulseCount) {
    SyncPulsePairing pairing;
    EXPECT_TRUE(pairing.Pair(500, 500, 3'000, 250'000, 5'000'000));
    EXPECT_TRUE(pairing.Pair(501, 1, 3'000, 250'000, 5'000'000));  // RP2040 rebooted, one edge since: still its pulse.
}

TEST(SyncPulsePairing, ALateFirstMessageOnlySetsTheHistory) {
    // Under output overload every count arrives a second or more after its edge: the first can't pair (nothing says its
    // edge is the pulse's), the next ones do.
    SyncPulsePairing pairing;
    EXPECT_FALSE(pairing.Pair(1, 40, 1'100'000, 250'000, 5'000'000));
    EXPECT_TRUE(pairing.Pair(2, 41, 1'100'000, 250'000, 5'000'000));
    pairing.SkipEdges(2);  // The coprocessor's clock restarted: history gone.
    EXPECT_FALSE(pairing.Pair(3, 42, 1'100'000, 250'000, 5'000'000));
    EXPECT_TRUE(pairing.Pair(4, 43, 1'100'000, 250'000, 5'000'000));
}

TEST(SyncPulseMessage, WireFormat) {
    // RP2040 and coprocessors exchange it as raw bytes over SPI.
    EXPECT_EQ(sizeof(ObjectDictionary::SyncPulse), 12u);
    ObjectDictionary::SyncPulse pulse = {.mlat_48mhz_counts = 0x0102030405060708ull, .pulse_num = 0x0A0B0C0D};
    uint8_t bytes[sizeof(pulse)];
    memcpy(bytes, &pulse, sizeof(pulse));
    EXPECT_EQ(bytes[0], 0x08);
    EXPECT_EQ(bytes[7], 0x01);
    EXPECT_EQ(bytes[8], 0x0D);
    EXPECT_EQ(bytes[11], 0x0A);
    EXPECT_GT(ObjectDictionary::kSyncPulseMinIntervalMs, ObjectDictionary::kSyncPulseMaxAgeMs);
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
