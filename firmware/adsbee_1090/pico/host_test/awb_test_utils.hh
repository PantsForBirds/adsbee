#pragma once

#include <cmath>
#include <cstdint>

#include "geo_utils.hh"

// Test-only conversions between AWB (see geo_utils.hh) and degrees, in host double precision.
inline double AWBToDeg(uint32_t awb) { return static_cast<int32_t>(awb) * (360.0 / 4294967296.0); }
inline uint32_t DegToAWB(double deg) {
    return static_cast<uint32_t>(static_cast<int64_t>(llround(deg * (4294967296.0 / 360.0))));
}
