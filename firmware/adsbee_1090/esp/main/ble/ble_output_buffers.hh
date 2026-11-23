#pragma once

#include <algorithm>
#include <cstdint>
#include <new>

#include "aircraft_dictionary.hh"
#include "gdl90_ble_stream.hh"
#include "settings.hh"

/**
 * A FLARM NMEA report's aircraft, nearest first: each aircraft's distance from ownship and its dictionary entry. One
 * report fills it, sorts it and writes PFLAA from it; the next report starts over with Clear().
 */
class FLARMTrafficList {
   public:
    struct Ref {
        uint32_t distance_m;
        const AircraftDictionary::AircraftEntry* entry;
    };

    // The array is allocated here; check Valid().
    explicit FLARMTrafficList(uint16_t capacity = AircraftDictionary::kMaxNumAircraft)
        : capacity_(capacity), refs_(new (std::nothrow) Ref[capacity]) {}
    ~FLARMTrafficList() { delete[] refs_; }
    FLARMTrafficList(const FLARMTrafficList&) = delete;
    FLARMTrafficList& operator=(const FLARMTrafficList&) = delete;

    bool Valid() const { return refs_ != nullptr; }

    void Clear() { size_ = 0; }

    // False when full.
    bool Add(uint32_t distance_m, const AircraftDictionary::AircraftEntry* entry) {
        if (size_ >= capacity_) return false;
        refs_[size_++] = {.distance_m = distance_m, .entry = entry};
        return true;
    }

    void SortNearestFirst() {
        std::sort(refs_, refs_ + size_, [](const Ref& a, const Ref& b) { return a.distance_m < b.distance_m; });
    }

    uint16_t Size() const { return size_; }
    const Ref& operator[](uint16_t i) const { return refs_[i]; }

   private:
    const uint16_t capacity_;
    Ref* const refs_;
    uint16_t size_ = 0;
};

/**
 * The heap a running Bluetooth output holds: the notification queue, and in FLARM NMEA mode the report's traffic list.
 * Allocated together when the output starts and freed together when it stops, so a stopped output (or a GDL90 one, for
 * the list) holds none.
 */
class BLEOutputBuffers {
   public:
    BLEOutputBuffers() = default;
    ~BLEOutputBuffers() { Free(); }
    BLEOutputBuffers(const BLEOutputBuffers&) = delete;
    BLEOutputBuffers& operator=(const BLEOutputBuffers&) = delete;

    /**
     * Allocates what an output in mode needs. Allocated() must be false.
     * @retval False if an allocation failed; nothing stays allocated then.
     */
    bool Allocate(SettingsManager::BLEOutputMode mode, uint16_t stream_capacity_bytes) {
        const bool flarm = mode == SettingsManager::kBLEOutputFLARMNMEA;
        stream_ = new (std::nothrow) GDL90BLEStream(
            stream_capacity_bytes, flarm ? GDL90BLEStream::kFramingLines : GDL90BLEStream::kFramingGDL90);
        if (stream_ == nullptr || !stream_->Valid()) {
            Free();
            return false;
        }
        if (flarm) {
            flarm_traffic_ = new (std::nothrow) FLARMTrafficList();
            if (flarm_traffic_ == nullptr || !flarm_traffic_->Valid()) {
                Free();
                return false;
            }
        }
        return true;
    }

    void Free() {
        delete stream_;
        stream_ = nullptr;
        delete flarm_traffic_;
        flarm_traffic_ = nullptr;
    }

    bool Allocated() const { return stream_ != nullptr; }

    GDL90BLEStream* stream() const { return stream_; }
    // Null unless allocated for FLARM NMEA.
    FLARMTrafficList* flarm_traffic() const { return flarm_traffic_; }

   private:
    GDL90BLEStream* stream_ = nullptr;
    FLARMTrafficList* flarm_traffic_ = nullptr;
};
