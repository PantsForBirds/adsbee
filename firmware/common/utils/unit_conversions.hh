#ifndef UNIT_CONVERSIONS_HH_
#define UNIT_CONVERSIONS_HH_

#include <cstddef>
#include <cstdint>

#include "macros.hh"

static const int kUsPerMs = 1000;
static const int kMsPerSec = 1000;
static const int kBytesPerWord = 4;
static const int kBitsPerByte = 8;
static const int kBitsPerNibble = 4;
static const int kNibblesPerByte = 2;

constexpr inline uint16_t CeilBitsToBytes(uint16_t bits) { return (bits + kBitsPerByte - 1) / kBitsPerByte; }

inline int FeetToMeters(int feet) { return feet * 1000 / 3280; }
// Feet to meters (0.3048 m/ft, exact), rounded. Clamped to +-200000 ft first so the product fits.
inline int32_t FeetToMetersRounded(int32_t ft) { return DivRound(Clamp(ft, -200000, 200000) * 3048, 10000); }
// Centimeters to feet (x 25 / 762, exact), rounded. Clamped to +-10000000 cm (100 km) first so the product fits.
inline int32_t CentimetersToFeetRounded(int32_t cm) { return DivRound(Clamp(cm, -10000000, 10000000) * 25, 762); }

inline int MetersToFeet(int meters) {
    if (meters < INT32_MAX / 328084) {
        // Enable high resolution conversion when we don't have risk of overflowing 32 bits.
        return meters * 328084 / 100000;
    } else {
        return meters * 3280 / 1000;
    }
}

inline int KtsToMps(int kts) { return kts * 5144 / 10000; }

inline int MpsToKts(int mps) { return mps * 10000 / 5144; }

inline int FpmToMps(int fpm) { return fpm * 508 / 100000; }

static const uint32_t kMetersPerNauticalMile = 1852;

/**
 * Tenths of a nautical mile to meters, rounded. Exact in 32 bits up to 2,319,000 tenths (231,900 nm).
 */
constexpr inline uint32_t DeciNmToMeters(uint32_t deci_nm) { return (deci_nm * kMetersPerNauticalMile + 5) / 10; }

/**
 * Parses a decimal number of nautical miles ("50", "50.5", "0.25") into tenths of a nautical mile, rounded to the
 * nearest tenth. Digits and at most one '.' only: no sign, exponent or whitespace.
 * @param[in] str Characters to parse (need not be null-terminated).
 * @param[in] len Number of characters.
 * @param[out] deci_nm Result in tenths of a nautical mile. Untouched on failure.
 * @param[in] max_deci_nm Largest accepted result.
 * @retval True if str was a number no larger than max_deci_nm.
 */
inline bool ParseDeciNm(const char* str, size_t len, uint32_t& deci_nm, uint32_t max_deci_nm) {
    uint64_t whole = 0;
    uint32_t tenths = 0;
    bool round_up = false;
    bool seen_digit = false, seen_point = false;
    size_t fraction_digits = 0;
    for (size_t i = 0; i < len; i++) {
        char c = str[i];
        if (c == '.') {
            if (seen_point) return false;
            seen_point = true;
            continue;
        }
        if (c < '0' || c > '9') return false;
        seen_digit = true;
        if (!seen_point) {
            whole = whole * 10 + static_cast<uint64_t>(c - '0');
            if (whole > max_deci_nm) return false;  // Keeps whole * 10 from overflowing.
        } else if (fraction_digits++ == 0) {
            tenths = static_cast<uint32_t>(c - '0');
        } else if (fraction_digits == 2) {
            round_up = c >= '5';
        }
    }
    if (!seen_digit) return false;
    uint64_t result = whole * 10 + tenths + (round_up ? 1 : 0);
    if (result > max_deci_nm) return false;
    deci_nm = static_cast<uint32_t>(result);
    return true;
}

#endif