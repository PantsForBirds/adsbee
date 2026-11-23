#pragma once

#include <cstdint>

// Conditionally define the MAX macro because the pico platform includes it by default in pico/platform.h.
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a, b) ((b) > (a) ? (a) : (b))
#endif

#ifndef ABS
#define ABS(x) ((x) < 0 ? -(x) : (x))
#endif

// v limited to lo..hi.
constexpr inline int32_t Clamp(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }

// numerator / denominator rounded half away from zero, for denominator > 0.
constexpr inline int32_t DivRound(int32_t numerator, int32_t denominator) {
    return numerator < 0 ? -((-numerator + denominator / 2) / denominator)
                         : (numerator + denominator / 2) / denominator;
}

inline uint32_t safe_cast_float_to_uint32(float value) {
    if (value < 0.0f) {
        value = -value;  // Convert negative to positive.
        uint32_t uint_value = static_cast<uint32_t>(value);
        return UINT32_MAX - uint_value + 1;  // Return the complement for negative values.
    } else {
        return static_cast<uint32_t>(value);
    }
}