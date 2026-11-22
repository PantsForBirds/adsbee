#include "geo_utils.hh"

#include <cstring>  // memcpy

#include "awb_utils.h"
#include "fixedmath/fixed_math.hpp"
#include "macros.hh"

namespace {

// Fixed point values are Q30: 1.0 is 2^30.
constexpr int32_t kQ30One = 1 << 30;

/**
 * (a * b) >> 30, rounded. The Cortex-M0+ has no 32x32 -> 64 bit multiply, so it builds the product from four 16x16 bit
 * multiplies inline instead of calling __aeabi_lmul (about 3x faster); that version is within 2 LSB (2e-9) of the exact
 * one used elsewhere.
 */
inline int32_t MulQ30(int32_t a, int32_t b) {
#if defined(__ARM_ARCH_6M__) || defined(GEO_UTILS_SPLIT_MULTIPLY)
    int32_t a_hi = a >> 16, b_hi = b >> 16;
    int32_t a_lo = static_cast<int32_t>(static_cast<uint32_t>(a) & 0xFFFFu);
    int32_t b_lo = static_cast<int32_t>(static_cast<uint32_t>(b) & 0xFFFFu);
    // a b = a_hi b_hi 2^32 + (a_hi b_lo + a_lo b_hi) 2^16 + a_lo b_lo. Cross terms are halved so their sum fits.
    int32_t cross_half = ((a_hi * b_lo) >> 1) + ((a_lo * b_hi) >> 1) +
                         static_cast<int32_t>((static_cast<uint32_t>(a_lo) * static_cast<uint32_t>(b_lo)) >> 17);
    return a_hi * b_hi * 4 + ((cross_half + (1 << 12)) >> 13);
#else
    return static_cast<int32_t>((static_cast<int64_t>(a) * b + (1 << 29)) >> 30);
#endif
}

// Polynomial fits (minimax weighted) for sin and cos on [-pi/4, pi/4], in Q30.
constexpr int32_t kSin1 = 1073741821, kSin3 = -178956794, kSin5 = 8946435, kSin7 = -209400;
constexpr int32_t kCos0 = 1073741824, kCos2 = -536870908, kCos4 = 44739189, kCos6 = -1491064, kCos8 = 26178;
constexpr int32_t kHalfPiQ30 = 1686629713;

// atan(2^-i) in AWB, for CORDIC (coordinate rotation digital computer) vectoring.
constexpr int kCordicIterations = 15;
constexpr uint32_t kCordicAtanAWB[kCordicIterations] = {536870912, 316933406, 167458907, 85004756, 42667331,
                                                        21354465,  10679838,  5340245,   2670163,  1335087,
                                                        667544,    333772,    166886,    83443,    41722};
// CORDIC magnitudes come out this much larger than the input (1.6467602581), Q30.
constexpr int32_t kCordicGainQ30 = 1768195004;
constexpr int32_t kAWBPerRadianQ28Q14 = 41722;  // AWB per 2^-28 rad, Q14: 2^32 / (2 pi) / 2^28 * 2^14.

/**
 * CORDIC vectoring: rotates (x, y) onto the positive x axis, then adds the small angle that is left, y / x, with one
 * division (atan(r) = r to 1e-13 once |r| < 2^-14).
 * @param[in,out] x X component, >= 0, |(x, y)| < 2^30; the vector's magnitude times kCordicGain on return.
 * @param[in,out] y Y component.
 * @retval Angle of the input vector from the x axis, in AWB.
 */
inline uint32_t CordicAtan2(int32_t &x, int32_t &y) {
    uint32_t angle = 0;
    for (int i = 0; i < kCordicIterations; i++) {
        int32_t x_shifted = x >> i, y_shifted = y >> i;
        if (y > 0) {
            x += y_shifted;
            y -= x_shifted;
            angle += kCordicAtanAWB[i];
        } else {
            x -= y_shifted;
            y += x_shifted;
            angle -= kCordicAtanAWB[i];
        }
    }
    // |y| < x 2^-14 < 2^17 and x >= 2^27 here. (y 2^14) / (x 2^-14) is y / x in units of 2^-28 rad, to 1e-4 of itself.
    int32_t divisor = x >> 14;
    if (divisor == 0) {
        return angle;
    }
    int32_t residual_rad_q28 = (y * (1 << 14)) / divisor;
    return angle + static_cast<uint32_t>((residual_rad_q28 * kAWBPerRadianQ28Q14) >> 14);
}

// Scales a vector up so its larger component is in [2^27, 2^28): headroom for the CORDIC gain, full precision when small.
inline int NormalizeShift(int32_t a, int32_t b) {
    uint32_t m = static_cast<uint32_t>(a < 0 ? -a : a) | static_cast<uint32_t>(b < 0 ? -b : b);
    int shift = 0;
    if (m == 0) return 0;
    while (m >= (1u << 28)) {
        m >>= 1;
        shift--;
    }
    if (m < (1u << 12)) {
        m <<= 16;
        shift += 16;
    }
    if (m < (1u << 20)) {
        m <<= 8;
        shift += 8;
    }
    while (m < (1u << 27)) {
        m <<= 1;
        shift++;
    }
    return shift;
}

inline int32_t Shift(int32_t v, int shift) { return shift >= 0 ? v * (1 << shift) : v >> -shift; }

// WGS-84 ellipsoid.
constexpr int32_t kSemiMajorAxisMeters = 6378137;
constexpr int32_t kMeridianRadiusEquatorMeters = 6335439;  // a (1 - e^2).
constexpr int32_t kEccentricitySquaredQ30 = 7188036;
constexpr int32_t kSecondEccentricitySquaredQ30 = 7236480;  // e^2 / (1 - e^2).
constexpr uint32_t kHalfMeridianMeters = 20003931;  // Pole to pole.
constexpr uint32_t kTwoPiQ28 = 1686629713;  // 2 * pi in Q28.

}  // namespace

void SinCosQ30(uint32_t angle_awb, int32_t &sin_out, int32_t &cos_out) {
    // Reduce to the nearest quarter turn and a remainder in [-pi/4, pi/4).
    uint32_t quadrant = (angle_awb + 0x20000000u) >> 30;
    int32_t remainder = static_cast<int32_t>(angle_awb - (quadrant << 30));  // AWB, |remainder| <= 2^29.
    int32_t x = MulQ30(remainder, kHalfPiQ30);                                 // Radians, Q30.
    int32_t x2 = MulQ30(x, x);
    int32_t s = MulQ30(x, kSin1 + MulQ30(x2, kSin3 + MulQ30(x2, kSin5 + MulQ30(x2, kSin7))));
    int32_t c = kCos0 + MulQ30(x2, kCos2 + MulQ30(x2, kCos4 + MulQ30(x2, kCos6 + MulQ30(x2, kCos8))));
    switch (quadrant & 3) {
        case 0:
            sin_out = s;
            cos_out = c;
            break;
        case 1:
            sin_out = c;
            cos_out = -s;
            break;
        case 2:
            sin_out = -s;
            cos_out = -c;
            break;
        default:
            sin_out = -c;
            cos_out = s;
            break;
    }
}

void GeoReference::Set(uint32_t lat_awb_in, uint32_t lon_awb_in) {
    lat_awb = lat_awb_in;
    lon_awb = lon_awb_in;
    SinCosQ30(lat_awb, sin_lat_q30, cos_lat_q30);
}

namespace {

/**
 * CalculateDistanceAndBearing with the bearing at full AWB resolution.
 * @param[in] ellipsoidal_bearing False: the azimuth on a sphere through both geodetic latitudes, as
 *            CalculateDistanceAndBearing has always returned (up to 0.2 deg off the ellipsoid's). True: corrected for
 *            the ellipsoid's shape near the reference (east scaled by N / M against north), to 0.001 deg up to 20 km.
 * @param[out] distance_q37_m Distance in 2^-37 m, unrounded, so each caller rounds once.
 */
void DistanceAndBearingAWB(const GeoReference &ref, uint32_t lat_awb, uint32_t lon_awb, uint64_t &distance_q37_m,
                           uint32_t &bearing_awb_out, bool ellipsoidal_bearing) {
    // Point B in the reference's east-north-up frame on a unit sphere. Written with half-angle sines so nearby points
    // don't lose precision to cancellation. Signed differences wrap across the antimeridian.
    int32_t dlat = static_cast<int32_t>(lat_awb - ref.lat_awb);
    int32_t dlon = static_cast<int32_t>(lon_awb - ref.lon_awb);
    int32_t sin_half_dlat, cos_half_dlat, sin_half_dlon, cos_half_dlon;
    SinCosQ30(static_cast<uint32_t>(dlat >> 1), sin_half_dlat, cos_half_dlat);
    SinCosQ30(static_cast<uint32_t>(dlon >> 1), sin_half_dlon, cos_half_dlon);
    int32_t sin_dlat = 2 * MulQ30(sin_half_dlat, cos_half_dlat);
    int32_t cos_dlat = kQ30One - 2 * MulQ30(sin_half_dlat, sin_half_dlat);
    int32_t cos_lat_b = MulQ30(ref.cos_lat_q30, cos_dlat) - MulQ30(ref.sin_lat_q30, sin_dlat);
    // cos(lat_b) sin^2(dlon / 2): half of what cos(dlon) takes away from the same-meridian terms. Doubling it can reach
    // 2.0 (2^31), so the doubled terms are added in wrapping unsigned arithmetic; north and up themselves fit.
    int32_t half_t = MulQ30(cos_lat_b, MulQ30(sin_half_dlon, sin_half_dlon));
    int32_t east = 2 * MulQ30(cos_lat_b, MulQ30(sin_half_dlon, cos_half_dlon));
    int32_t north = static_cast<int32_t>(static_cast<uint32_t>(sin_dlat) +
                                         2u * static_cast<uint32_t>(MulQ30(ref.sin_lat_q30, half_t)));
    int32_t up = static_cast<int32_t>(static_cast<uint32_t>(cos_dlat) -
                                      2u * static_cast<uint32_t>(MulQ30(ref.cos_lat_q30, half_t)));

    if (east == 0 && north == 0) {
        distance_q37_m = up > 0 ? 0 : static_cast<uint64_t>(kHalfMeridianMeters) << 37;  // Same point or antipode.
        bearing_awb_out = 0;
        return;
    }

    // Bearing and horizontal magnitude (sine of the central angle) of (north, east).
    int shift = NormalizeShift(north, east);
    int32_t x = Shift(north, shift), y = Shift(east, shift);
    uint32_t bearing_awb = 0;
    if (x < 0) {  // CORDIC converges within +-99 deg: start from the opposite direction.
        x = -x;
        y = -y;
        bearing_awb = 0x80000000u;
    }
    const uint32_t half_turn = bearing_awb;
    int32_t north_n = x, east_n = y;  // Normalized, for the direction's share of north-south below.
    bearing_awb += CordicAtan2(x, y);
    bearing_awb_out = bearing_awb;
    int32_t sin_mid_lat = MulQ30(ref.sin_lat_q30, cos_half_dlat) + MulQ30(ref.cos_lat_q30, sin_half_dlat);
    if (ellipsoidal_bearing) {
        // A meter east spans N / M = 1 + e'^2 cos^2(lat) times the angle of a meter north (prime vertical over meridian
        // radius of curvature), which the sphere above leaves out. The scaled vector stays below 2^28.5, so CORDIC's
        // gain keeps it below 2^30.
        int32_t cos2_mid_lat = kQ30One - MulQ30(sin_mid_lat, sin_mid_lat);
        int32_t xe = north_n, ye = MulQ30(east_n, kQ30One + MulQ30(kSecondEccentricitySquaredQ30, cos2_mid_lat));
        bearing_awb_out = half_turn + CordicAtan2(xe, ye);
    }

    // Central angle = atan2(horizontal, up), with both scaled by the CORDIC gain and kept below 2^30.
    int32_t horizontal = Shift(x, -shift - 1);  // gain * |(north, east)|, Q29.
    int32_t up_scaled = MulQ30(up, kCordicGainQ30) >> 1;  // gain * up, Q29.
    uint32_t central_angle_awb = 0;
    int32_t cx = up_scaled, cy = horizontal;
    if (cx < 0) {  // Past 90 deg: rotate by -90 deg first.
        int32_t tmp = cx;
        cx = cy;
        cy = -tmp;
        central_angle_awb = 0x40000000u;
    }
    central_angle_awb += CordicAtan2(cx, cy);
    if (central_angle_awb > 0xC0000000u) {
        central_angle_awb = 0;  // CORDIC can settle a few AWB below zero for coincident points.
    }

    // Radius of curvature in the direction of travel at the mid latitude: N - (N - M) cos^2(bearing), where M (meridian)
    // and N (prime vertical) come from a second order series in w = e^2 sin^2(mid latitude).
    int32_t w = MulQ30(MulQ30(sin_mid_lat, sin_mid_lat), kEccentricitySquaredQ30);  // < 2^23
    int32_t w_m = MulQ30(w, kSemiMajorAxisMeters);                                     // a w, meters (< 43 km).
    int32_t w2_m = MulQ30(w, w_m);                                                     // a w^2, meters.
    int32_t prime_vertical_m = kSemiMajorAxisMeters + w_m / 2 + (3 * w2_m) / 8;
    int32_t meridian_m = kMeridianRadiusEquatorMeters + MulQ30(w, kMeridianRadiusEquatorMeters) * 3 / 2 +
                         MulQ30(MulQ30(w, w), kMeridianRadiusEquatorMeters) * 15 / 8;
    int32_t n16 = north_n >> 13, e16 = east_n >> 13;  // |.| < 2^15.
    uint32_t nn = static_cast<uint32_t>(n16 * n16), ee = static_cast<uint32_t>(e16 * e16);
    uint32_t cos2_q15 = (nn >> 2) / (((nn + ee) >> 17) | 1);  // Divisor >= 2^11: 0.05% is plenty here.
    int32_t radius_m = prime_vertical_m - static_cast<int32_t>(((prime_vertical_m - meridian_m) * cos2_q15) >> 15);

    // distance = central angle (rad) * radius = central_angle_awb * 2 pi / 2^32 * radius.
    uint32_t radius_2pi_q5 = static_cast<uint32_t>((static_cast<uint64_t>(radius_m) * kTwoPiQ28) >> 23);  // < 2^31
    distance_q37_m = static_cast<uint64_t>(central_angle_awb) * radius_2pi_q5;
}

}  // namespace

void CalculateDistanceAndBearing(const GeoReference &ref, uint32_t lat_awb, uint32_t lon_awb, uint32_t &distance_m,
                                 uint16_t &bearing_bam16) {
    uint64_t distance_q37_m;
    uint32_t bearing_awb;
    DistanceAndBearingAWB(ref, lat_awb, lon_awb, distance_q37_m, bearing_awb, false);
    distance_m = static_cast<uint32_t>((distance_q37_m + (1ull << 36)) >> 37);
    bearing_bam16 = static_cast<uint16_t>((bearing_awb + 0x8000u) >> 16);
}

void CalculateNorthEastMeters(const GeoReference &ref, uint32_t lat_awb, uint32_t lon_awb, int32_t &north_m,
                              int32_t &east_m, uint32_t &distance_m) {
    uint64_t distance_q37_m;
    uint32_t bearing_awb;
    DistanceAndBearingAWB(ref, lat_awb, lon_awb, distance_q37_m, bearing_awb, true);
    distance_m = static_cast<uint32_t>((distance_q37_m + (1ull << 36)) >> 37);
    // 1/16 m keeps the product with a Q30 sine below 2^63.
    const int64_t distance_q4_m = static_cast<int64_t>((distance_q37_m + (1ull << 32)) >> 33);
    int32_t sin_q30, cos_q30;
    SinCosQ30(bearing_awb, sin_q30, cos_q30);
    // Arithmetic shifts round halves up for both signs: each component is rounded once, to 0.5 m.
    north_m = static_cast<int32_t>((distance_q4_m * cos_q30 + (1ll << 33)) >> 34);
    east_m = static_cast<int32_t>((distance_q4_m * sin_q30 + (1ll << 33)) >> 34);
}

uint32_t FloatDegToAWB(float deg) {
    uint32_t bits;
    memcpy(&bits, &deg, sizeof(bits));
    const int exponent = static_cast<int>((bits >> 23) & 0xFFu);
    if (exponent == 0 || exponent == 0xFF) return 0;  // Zero, subnormal (< 1e-38 deg), infinity or NaN.
    // |deg| = mantissa 2^(exponent - 150), so deg 2^32 / 360 = mantissa (2^40 / 360) 2^(exponent - 158).
    const uint64_t mantissa = (bits & 0x7FFFFFu) | 0x800000u;
    const int shift = 158 - exponent;
    constexpr uint64_t kAWBPerDegQ8 = 3054198966u;  // 2^40 / 360, rounded down (by 0.04, 1.3e-11 of itself).
    if (shift <= 0 || shift >= 64) return 0;  // |deg| >= 2^31 (not an angle) or below 1e-9 AWB.
    const uint32_t awb = static_cast<uint32_t>((mantissa * kAWBPerDegQ8 + (1ull << (shift - 1))) >> shift);
    return (bits >> 31) ? 0u - awb : awb;
}

uint32_t CalculateDistanceMeters(const GeoReference &ref, uint32_t lat_awb, uint32_t lon_awb) {
    uint32_t distance_m;
    uint16_t bearing_bam16;
    CalculateDistanceAndBearing(ref, lat_awb, lon_awb, distance_m, bearing_bam16);
    return distance_m;
}

uint32_t CalculateGeoidalDistanceMetersAWB(uint32_t lat_a_awb, uint32_t lon_a_awb, uint32_t lat_b_awb,
                                           uint32_t lon_b_awb) {
    return CalculateDistanceMeters(GeoReference(lat_a_awb, lon_a_awb), lat_b_awb, lon_b_awb);
}

bool IsDistanceGreaterThan(uint32_t lat_a_awb, uint32_t lon_a_awb, uint32_t lat_b_awb, uint32_t lon_b_awb,
                           uint32_t threshold_m) {
    // AWB per meter at the smallest (6335439 m, meridian at the equator) and largest (6399594 m, at the poles) radius
    // of curvature, Q16, rounded so both bounds stay conservative.
    constexpr uint64_t kAWBPerMeterMaxQ16 = 7071039;
    constexpr uint64_t kAWBPerMeterMinQ16 = 7000152;
    int32_t dlat = static_cast<int32_t>(lat_b_awb - lat_a_awb);
    int32_t dlon = static_cast<int32_t>(lon_b_awb - lon_a_awb);
    uint64_t abs_dlat = dlat < 0 ? 0u - static_cast<uint32_t>(dlat) : static_cast<uint32_t>(dlat);
    uint64_t abs_dlon = dlon < 0 ? 0u - static_cast<uint32_t>(dlon) : static_cast<uint32_t>(dlon);
    // The distance is at least the meridian arc between the two latitudes...
    if (abs_dlat > ((threshold_m * kAWBPerMeterMaxQ16) >> 16) + 1) {
        return true;
    }
    // ...and at most that arc plus the arc along the parallel.
    if (abs_dlat + abs_dlon <= ((threshold_m * kAWBPerMeterMinQ16) >> 16)) {
        return false;
    }
    return CalculateGeoidalDistanceMetersAWB(lat_a_awb, lon_a_awb, lat_b_awb, lon_b_awb) > threshold_m;
}

uint32_t CalculateGeoidalDistanceMetersDeg(float lat_a_deg, float lon_a_deg, float lat_b_deg, float lon_b_deg) {
    // Cast floats to signed ints, then to unsigned ints, to avoid undefined behavior with negative floats.
    uint32_t lat_a_awb = safe_cast_float_to_uint32(INV_RESOLUTION * lat_a_deg);
    uint32_t lon_a_awb = safe_cast_float_to_uint32(INV_RESOLUTION * lon_a_deg);
    uint32_t lat_b_awb = safe_cast_float_to_uint32(INV_RESOLUTION * lat_b_deg);
    uint32_t lon_b_awb = safe_cast_float_to_uint32(INV_RESOLUTION * lon_b_deg);

    return CalculateGeoidalDistanceMetersAWB(lat_a_awb, lon_a_awb, lat_b_awb, lon_b_awb);
}

#ifndef HAS_FPU
inline fixedmath::fixed_t wrapped_atan2(fixedmath::fixed_t y, fixedmath::fixed_t x) {
    fixedmath::fixed_t val = fixedmath::func::atan2(y, x);
    return val < fixedmath::fixed_t(0) ? (val + fixedmath::fixed_t(2.0f * M_PI)) : val;
}
const fixedmath::fixed_t kDegPerRadian = fixedmath::fixed_t{180.0f / M_PI};  // Conversion factor from radians to degrees.
#endif

void CalculateTrackAndSpeedFromNEVelocities(int32_t n_vel_kts, int32_t e_vel_kts, float &track_deg,
                                            int32_t &speed_kts) {
#ifdef HAS_FPU
    float n = (float)n_vel_kts;
    float e = (float)e_vel_kts;
    float track_rad = atan2f(e, n);
    if (track_rad < 0.0f) track_rad += 2.0f * (float)M_PI;
    track_deg = track_rad * (180.0f / (float)M_PI);
    speed_kts = (int32_t)(sqrtf(n * n + e * e) + 0.5f);
#else
    track_deg = static_cast<float>(
        wrapped_atan2(static_cast<fixedmath::fixed_t>(e_vel_kts), static_cast<fixedmath::fixed_t>(n_vel_kts)) *
        kDegPerRadian);
    fixedmath::fixed_t speed_kts_fixed =
        fixedmath::func::sqrt(static_cast<fixedmath::fixed_t>(n_vel_kts * n_vel_kts + e_vel_kts * e_vel_kts));
    // Round speed_kts_fixed.
    if (speed_kts_fixed % fixedmath::fixed_t(1) >= fixedmath::fixed_t(0.5f)) {
        speed_kts_fixed += fixedmath::fixed_t(1);
    }
    speed_kts = static_cast<int16_t>(speed_kts_fixed);
#endif
}
