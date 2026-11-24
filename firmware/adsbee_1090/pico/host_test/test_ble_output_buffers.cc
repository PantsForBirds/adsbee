#include <cstdlib>
#include <new>

#include "ble_output_buffers.hh"
#include "gtest/gtest.h"

// Fails the n-th (1-based) nothrow array allocation from here on, as on a heap that is too full or fragmented.
// Replaces the global nothrow operator new[] for the whole test binary; inactive (0) outside these tests.
static int g_fail_nothrow_array_new_at = 0;

void* operator new[](std::size_t size, const std::nothrow_t&) noexcept {
    if (g_fail_nothrow_array_new_at > 0 && --g_fail_nothrow_array_new_at == 0) return nullptr;
    try {
        return ::operator new[](size);
    } catch (...) {
        return nullptr;
    }
}

TEST(FLARMTrafficList, SortsNearestFirstAndStopsWhenFull) {
    FLARMTrafficList list(3);
    ASSERT_TRUE(list.Valid());
    static const AircraftDictionary::AircraftEntry e[4];
    EXPECT_TRUE(list.Add(300, e + 0));
    EXPECT_TRUE(list.Add(100, e + 1));
    EXPECT_TRUE(list.Add(200, e + 2));
    EXPECT_FALSE(list.Add(50, e + 3));
    list.SortNearestFirst();
    ASSERT_EQ(list.Size(), 3);
    EXPECT_EQ(list[0].distance_m, 100u);
    EXPECT_EQ(list[0].entry, e + 1);
    EXPECT_EQ(list[1].distance_m, 200u);
    EXPECT_EQ(list[2].distance_m, 300u);
    list.Clear();
    EXPECT_EQ(list.Size(), 0);
    EXPECT_TRUE(list.Add(50, e + 3));
}

TEST(FLARMTrafficList, HoldsTheWholeDictionary) {
    FLARMTrafficList list;
    for (uint16_t i = 0; i < AircraftDictionary::kMaxNumAircraft; i++) ASSERT_TRUE(list.Add(i, nullptr));
    EXPECT_FALSE(list.Add(0, nullptr));
}

TEST(BLEOutputBuffers, FLARMModeAllocatesTheTrafficListGDL90DoesNot) {
    BLEOutputBuffers buffers;
    EXPECT_FALSE(buffers.Allocated());
    EXPECT_EQ(buffers.stream(), nullptr);
    EXPECT_EQ(buffers.flarm_traffic(), nullptr);

    ASSERT_TRUE(buffers.Allocate(SettingsManager::kBLEOutputGDL90, 1024));
    EXPECT_TRUE(buffers.Allocated());
    EXPECT_NE(buffers.stream(), nullptr);
    EXPECT_EQ(buffers.flarm_traffic(), nullptr);
    buffers.Free();
    EXPECT_FALSE(buffers.Allocated());

    // Mode switch: stop (free), then start in the other mode.
    ASSERT_TRUE(buffers.Allocate(SettingsManager::kBLEOutputFLARMNMEA, 1024));
    ASSERT_NE(buffers.stream(), nullptr);
    ASSERT_NE(buffers.flarm_traffic(), nullptr);
    EXPECT_TRUE(buffers.flarm_traffic()->Valid());
    // Line framing: a sentence ending in LF is one message.
    const uint8_t line[] = {'$', 'A', '\r', '\n', '$', 'B', '\r', '\n'};
    ASSERT_TRUE(buffers.stream()->Push(line, sizeof(line)));
    uint8_t out[8];
    EXPECT_EQ(buffers.stream()->Peek(out, 6), 4);
    buffers.Free();
    EXPECT_FALSE(buffers.Allocated());
    EXPECT_EQ(buffers.stream(), nullptr);
    EXPECT_EQ(buffers.flarm_traffic(), nullptr);

    ASSERT_TRUE(buffers.Allocate(SettingsManager::kBLEOutputGDL90, 1024));
    EXPECT_EQ(buffers.flarm_traffic(), nullptr);
}

TEST(BLEOutputBuffers, FailedAllocationLeavesNothingAllocated) {
    BLEOutputBuffers buffers;
    // 1: the stream's queue, 2: the FLARM traffic array.
    for (int fail_at : {1, 2}) {
        g_fail_nothrow_array_new_at = fail_at;
        EXPECT_FALSE(buffers.Allocate(SettingsManager::kBLEOutputFLARMNMEA, 1024)) << fail_at;
        g_fail_nothrow_array_new_at = 0;
        EXPECT_FALSE(buffers.Allocated()) << fail_at;
        EXPECT_EQ(buffers.stream(), nullptr) << fail_at;
        EXPECT_EQ(buffers.flarm_traffic(), nullptr) << fail_at;
    }
    g_fail_nothrow_array_new_at = 1;
    EXPECT_FALSE(buffers.Allocate(SettingsManager::kBLEOutputGDL90, 1024));
    g_fail_nothrow_array_new_at = 0;
    EXPECT_FALSE(buffers.Allocated());
    // A later start succeeds.
    EXPECT_TRUE(buffers.Allocate(SettingsManager::kBLEOutputFLARMNMEA, 1024));
    EXPECT_NE(buffers.flarm_traffic(), nullptr);
}
