#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>

/**
 * Range per direction and height band for swarm reports (AT+SWARM), as the swarm protocol v1 defines it: 36 sectors
 * of 10 degrees (0 = north) by 12 bands, each holding the largest distance seen in the protocol's unit.
 */
class RangeMap {
   public:
    static constexpr uint16_t kNumSectors = 36;
    static constexpr uint16_t kNumBands = 12;
    static constexpr uint16_t kNumBins = kNumSectors * kNumBands;
    static constexpr uint16_t kBitmapLenBytes = kNumBins / 8;
    static constexpr uint32_t kUnitMeters = 2000;     // Aircraft.
    static constexpr uint32_t kDroneUnitMeters = 50;  // Drones.

    // Aircraft band per thousand feet (baro, else geometric) below 40,000 ft. Bands: [0,1k) [1k,2k) [2k,4k) [4k,6k)
    // [6k,10k) [10k,15k) [15k,20k) [20k,25k) [25k,30k) [30k,35k) [35k,40k) [40k,inf).
    static constexpr uint8_t kBandByKft[40] = {0, 1, 2, 2, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6,
                                               7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10};
    // Drone band per hundred feet above the receiver below 5,000 ft. Bands: [0,100) [100,200) [200,300) [300,400)
    // [400,500) [500,700) [700,1000) [1000,1500) [1500,2000) [2000,3000) [3000,5000) [5000,inf).
    static constexpr uint8_t kBandByHundredFt[50] = {0, 1, 2, 3, 4, 5, 5, 6, 6, 6, 7, 7, 7, 7, 7, 8, 8,
                                                     8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10,
                                                     10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10};

    static inline uint8_t AltitudeBand(int32_t altitude_ft) {
        if (altitude_ft < 0) return 0;
        if (altitude_ft >= 40000) return kNumBands - 1;
        return kBandByKft[altitude_ft / 1000];
    }

    static inline uint8_t HeightBand(int32_t height_ft) {
        if (height_ft < 0) return 0;
        if (height_ft >= 5000) return kNumBands - 1;
        return kBandByHundredFt[height_ft / 100];
    }

    // Bearing in 1/65536 turns to a 10 degree sector.
    static inline uint8_t Sector(uint16_t bearing_bam16) {
        return static_cast<uint8_t>((static_cast<uint32_t>(bearing_bam16) * kNumSectors) >> 16);
    }

    // Distance in meters to a bin value: ceil(distance / unit), clamped to 1..255.
    static inline uint8_t DistanceValue(uint32_t distance_m, uint32_t unit_m = kUnitMeters) {
        uint32_t units = distance_m / unit_m + (distance_m % unit_m != 0);
        return units < 1 ? 1 : units > 255 ? 255 : static_cast<uint8_t>(units);
    }

    inline void Record(uint8_t band, uint16_t bearing_bam16, uint8_t value) {
        uint8_t& bin = bins[band * kNumSectors + Sector(bearing_bam16)];
        if (value > bin) bin = value;
    }

    // Aircraft (Mode S, UAT): band by altitude, 2 km unit.
    inline void Record(uint32_t distance_m, uint16_t bearing_bam16, int32_t altitude_ft) {
        Record(AltitudeBand(altitude_ft), bearing_bam16, DistanceValue(distance_m));
    }

    void Merge(const RangeMap& other) {
        for (uint16_t i = 0; i < kNumBins; i++) {
            if (other.bins[i] > bins[i]) bins[i] = other.bins[i];
        }
    }

    void Clear() { memset(bins, 0, sizeof(bins)); }

    bool Empty() const {
        for (uint16_t i = 0; i < kNumBins; i++) {
            if (bins[i]) return false;
        }
        return true;
    }

    /**
     * Writes the protocol's two arrays: a bitmap with bit i (byte i / 8, bit i % 8, LSB first) set for each bin with
     * data, and one value byte per set bit in bin order.
     * @param[out] bitmap kBitmapLenBytes Bytes.
     * @param[out] values kNumBins Bytes at most.
     * @retval Number of value Bytes written.
     */
    uint16_t Encode(uint8_t* bitmap, uint8_t* values) const {
        memset(bitmap, 0, kBitmapLenBytes);
        uint16_t num_values = 0;
        for (uint16_t i = 0; i < kNumBins; i++) {
            if (bins[i] == 0) continue;
            bitmap[i / 8] |= 1 << (i % 8);
            values[num_values++] = bins[i];
        }
        return num_values;
    }

    uint8_t bins[kNumBins] = {0};  // index = band * kNumSectors + sector; 0 = no data.
};
