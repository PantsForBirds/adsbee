#pragma once

#include <cstddef>
#include <string_view>

#include "fixedmath/fixed_math.hpp"
#include "stdint.h"

/**
 * Latitudes and longitudes are held as uint32_t Angular Weighted Binary (AWB) everywhere in firmware: 2^32 per turn,
 * read as int32_t for the sign. Latitude -90..90 deg is -2^30..2^30, longitude -180..180 deg is -2^31..2^31 (east and
 * north positive). Degrees appear only at human or decimal-protocol edges, converted with the integer helpers below.
 */

/**
 * Distance and bearing on the WGS-84 ellipsoid in 32-bit fixed point, without lookup tables or floating point (the
 * RP2040 has no FPU). All targets run the same integer code, so they get identical results.
 *
 * Positions are in Angular Weighted Binary (AWB): a full circle is 2^32, so latitudes -90..90 deg are the signed range
 * -2^30..2^30 and longitude differences wrap across the antimeridian on their own. The central angle and bearing come
 * from exact spherical formulas written without cancellation; the distance multiplies the central angle by the
 * ellipsoid's radius of curvature in the direction of travel at the mid latitude, and the bearing is corrected for the
 * ellipsoid's shape (east against north scaled by N / M). Error against the WGS-84 geodesic at any latitude: distance
 * < 0.015% + 1 m up to 600 km (< 0.15% up to 5000 km), bearing as BAM16 (rounded to 0.0027 deg) < 0.004 deg from 1 km
 * to 100 km and < 0.012 deg up to 600 km.
 */

/**
 * sin and cos of an angle in AWB, as Q30 fixed point (1.0 = 2^30), accurate to 5e-9.
 * @param[in] angle_awb Angle, where 2^32 is a full circle.
 * @param[out] sin_out sin(angle), Q30.
 * @param[out] cos_out cos(angle), Q30.
 */
void SinCosQ30(uint32_t angle_awb, int32_t &sin_out, int32_t &cos_out);

/**
 * A point that many distances are measured from (the receiver). Caches the trig of its latitude, so each distance and
 * bearing to it costs two sin/cos evaluations.
 */
struct GeoReference {
    uint32_t lat_awb = 0;
    uint32_t lon_awb = 0;
    int32_t sin_lat_q30 = 0;
    int32_t cos_lat_q30 = 1 << 30;

    GeoReference() {}
    GeoReference(uint32_t lat_awb_in, uint32_t lon_awb_in) { Set(lat_awb_in, lon_awb_in); }
    void Set(uint32_t lat_awb_in, uint32_t lon_awb_in);
};

// Bearings are stored in 1/65536 of a turn (binary angle measurement, BAM).
inline float Bam16ToDeg(uint16_t bearing_bam16) { return bearing_bam16 * (360.0f / 65536.0f); }

// A track or heading rounded to whole degrees, 0-359.
inline int RoundTrackDeg(float track_deg) {
    int t = static_cast<int>(track_deg < 0 ? track_deg - 0.5f : track_deg + 0.5f) % 360;
    return t < 0 ? t + 360 : t;
}

/**
 * Writes an AWB latitude or longitude as an NMEA 0183 field pair: "ddmm.mmmm,N" or "dddmm.mmmm,E" (signed AWB:
 * negative is south or west), rounded to 1/10000 minute.
 * @param[out] buf Output; 12 characters for a latitude, 13 for a longitude, plus the terminator.
 * @param[in] len Size of buf.
 * @param[in] awb Latitude or longitude in AWB.
 * @param[in] is_lat True for a latitude.
 */
void WriteNMEALatLon(char* buf, size_t len, uint32_t awb, bool is_lat);

/**
 * Distance and initial bearing from a reference point to another point.
 * @param[in] ref Reference point (start).
 * @param[in] lat_awb Latitude of the other point, in AWB.
 * @param[in] lon_awb Longitude of the other point, in AWB.
 * @param[out] distance_m Distance along the surface, in meters.
 * @param[out] bearing_bam16 Initial bearing from the reference to the point, clockwise from north, in 1/65536 turns.
 */
void CalculateDistanceAndBearing(const GeoReference &ref, uint32_t lat_awb, uint32_t lon_awb, uint32_t &distance_m,
                                 uint16_t &bearing_bam16);

/**
 * A distance and bearing (as CalculateDistanceAndBearing gives them) split into meters along true north and east: the
 * azimuthal equidistant projection centered on the reference, which keeps every distance from it true. Each component
 * is rounded once; against the same split of the WGS-84 geodesic it is off by the distance's rounding (0.5 m) plus the
 * bearing's (half a BAM16, 0.48 m at 10 km) and error: < 1.2 m up to 10 km and < 1.6 m up to 20 km, at any latitude.
 * @param[in] distance_m Distance along the surface, in meters.
 * @param[in] bearing_bam16 Bearing clockwise from north, in 1/65536 turns.
 * @param[out] north_m Meters north, negative to the south.
 * @param[out] east_m Meters east, negative to the west.
 */
void SplitNorthEastMeters(uint32_t distance_m, uint16_t bearing_bam16, int32_t &north_m, int32_t &east_m);

/**
 * An angle in float degrees as AWB, exactly (to 1 AWB, 8e-8 deg) and without floating point arithmetic: negative
 * angles wrap, so it serves latitudes and longitudes alike.
 */
inline uint32_t FloatDegToAWB(float deg) {
    uint32_t bits;
    __builtin_memcpy(&bits, &deg, sizeof(bits));  // Inlined even where memcpy isn't (a call on the ESP32).
    const int exponent = static_cast<int>((bits >> 23) & 0xFFu);
    if (exponent == 0 || exponent == 0xFF) return 0;  // Zero, subnormal (< 1e-38 deg), infinity or NaN.
    // |deg| = mantissa 2^(exponent - 150), so deg 2^32 / 360 = mantissa (2^40 / 360) 2^(exponent - 158).
    const uint32_t mantissa = (bits & 0x7FFFFFu) | 0x800000u;
    const int shift = 158 - exponent;
    constexpr uint32_t kAWBPerDegQ8 = 3054198966u;  // 2^40 / 360, rounded down (by 0.04, 1.3e-11 of itself).
    if (shift <= 0 || shift >= 64) return 0;  // |deg| >= 2^31 (not an angle) or below 1e-9 AWB.
    // product / 2^shift, rounded, in 32-bit halves: variable 64-bit shifts are library calls on both targets.
    const uint64_t product = static_cast<uint64_t>(mantissa) * kAWBPerDegQ8;
    uint32_t lo = static_cast<uint32_t>(product), hi = static_cast<uint32_t>(product >> 32);
    uint32_t awb;
    if (shift < 32) {  // |deg| >= 1.
        const uint32_t half = 1u << (shift - 1);
        lo += half;
        hi += lo < half;
        awb = (lo >> shift) | (hi << (32 - shift));
    } else if (shift == 32) {
        awb = hi + (lo >> 31);
    } else {
        awb = (hi + (1u << (shift - 33))) >> (shift - 32);
    }
    return (bits >> 31) ? 0u - awb : awb;
}

// Whether an AWB latitude lies within -90..90 deg.
inline bool AWBIsLatitude(uint32_t lat_awb) {
    const int32_t lat = static_cast<int32_t>(lat_awb);
    return lat >= -(1 << 30) && lat <= (1 << 30);
}

/**
 * An AWB angle in degrees times 10^decimals, rounded to nearest (half away from zero), -180..180 deg.
 * @param[in] decimals 0 to 7.
 */
int32_t AWBToScaledDeg(uint32_t awb, uint8_t decimals);

// An AWB angle in 1e-7 degrees (MAVLink, Remote ID).
inline int32_t AWBToDegE7(uint32_t awb) { return AWBToScaledDeg(awb, 7); }

// An angle in 1e-7 degrees as AWB, exactly rounded to nearest. Negative angles wrap.
inline uint32_t DegE7ToAWB(int32_t deg_e7) {
    // deg_e7 2^32 / 3.6e9 = deg_e7 2^22 / 3515625. The divisor is odd, so there are no ties.
    constexpr int64_t kDivisor = 3515625;
    const int64_t scaled = static_cast<int64_t>(deg_e7) * (1 << 22);
    return static_cast<uint32_t>((scaled + (scaled >= 0 ? kDivisor / 2 : -(kDivisor / 2))) / kDivisor);
}

/**
 * An angle in units_per_turn steps as AWB, exactly rounded to nearest. Negative angles wrap.
 * @param[in] value Angle, |value| < units_per_turn.
 * @param[in] units_per_turn Steps per full circle, < 2^47.
 */
uint32_t ScaledToAWB(int64_t value, uint64_t units_per_turn);

// Buffer size for FormatAWBDeg: "-180.0000000" and the terminator.
static constexpr size_t kAWBDegStrMaxLen = 13;

/**
 * Writes an AWB angle as signed decimal degrees, like printf("%.<decimals>f") of the exact value.
 * @param[in] decimals 0 to 7.
 * @retval Characters written (excluding the terminator), as snprintf.
 */
int FormatAWBDeg(char* buf, size_t len, uint32_t awb, uint8_t decimals);

/**
 * Parses decimal degrees ("-122.3", "37", "1e-7") to 1e-7 degrees, rounded to nearest.
 * @param[out] deg_e7 Angle in 1e-7 degrees. Unchanged on failure.
 * @retval False if the text is not a number or does not fit (beyond +/-214 deg).
 */
bool ParseDegE7(std::string_view text, int32_t& deg_e7);

/**
 * Distance from a reference point to another point, in meters. Same accuracy as CalculateDistanceAndBearing.
 */
uint32_t CalculateDistanceMeters(const GeoReference &ref, uint32_t lat_awb, uint32_t lon_awb);

/**
 * @brief Calculates the distance along the surface of the earth between two lat/long coordinate sets.
 * @param[in] lat_a_awb Latitude of coordinate A, in Angular Weighted Binary format.
 * @param[in] lon_a_awb Longitude of coordinate A, in Angular Weighted Binary format.
 * @param[in] lat_b_awb Latitude of coordinate B, in Angular Weighted Binary format.
 * @param[in] lon_b_awb Longitude of coordinate B, in Angular Weighted Binary format.
 * @retval Distance between points A and B, in meters.
 */
uint32_t CalculateGeoidalDistanceMetersAWB(uint32_t lat_a_awb, uint32_t lon_a_awb, uint32_t lat_b_awb,
                                           uint32_t lon_b_awb);

/**
 * Whether two points are farther apart than a threshold. Exact integer bounds on the latitude and longitude differences
 * settle most calls in a few cycles; the full distance calculation runs only when the answer is close.
 * @retval True if the distance exceeds threshold_m. Bounds and full calculation can disagree only within the distance
 *         error (0.015%) of the threshold.
 */
bool IsDistanceGreaterThan(uint32_t lat_a_awb, uint32_t lon_a_awb, uint32_t lat_b_awb, uint32_t lon_b_awb,
                           uint32_t threshold_m);

void CalculateTrackAndSpeedFromNEVelocities(int32_t n_vel_kts, int32_t e_vel_kts, float &track_deg, int32_t &speed_kts);
