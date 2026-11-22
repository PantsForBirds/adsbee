#include <cmath>
#include <random>

#include "awb_utils.h"
#include "geo_utils.hh"
#include "gtest/gtest.h"
#include "macros.hh"

static const float kTrackToleranceDeg = 0.1f;
static const int32_t kSpeedToleranceKts = 1;

static uint32_t DegToAWB(double deg) { return static_cast<uint32_t>(static_cast<int64_t>(llround(deg / 360.0 * 4294967296.0))); }

/**
 * Reference geodesic distances (m) and initial azimuths (deg) on WGS-84 from GeographicLib (Karney's algorithm, accurate
 * to nanometers). Cases cover the equator, mid latitudes, near the poles, across the antimeridian and up to 600 km.
 */
struct GeodesicCase {
    double lat_a, lon_a, lat_b, lon_b, distance_m, azimuth_deg;
};
static const GeodesicCase kGeodesicCases[] = {
    // lat_a, lon_a, lat_b, lon_b, s12 (m), azi1 (deg)
    {0, 0, 1, 0, 110574.389, 0.000},
    {0, 0, 0, 1, 111319.491, 90.000},
    {37.4, -122, 37.5, -122, 11098.610, 0.000},
    {37.4, -122, 38.4, -121, 141614.458, 38.087},
    {37.4, -122, 32, -117.5, 726986.975, 144.157},
    {52, 4.5, 51, 6, 152400.157, 136.298},
    {-33.9, 151.2, -34.9, 148.9, 238801.501, 241.675},
    {60, 179.5, 60.5, -179, 100014.545, 55.503},    // Across the antimeridian.
    {-10, -179.9, -11, 179, 163505.459, 227.331},
    {85, 0, 86, 90, 714820.065, 38.741},            // Near the north pole.
    {89.5, 10, 89.5, -170, 111693.951, 0.000},      // Across the north pole.
    {-88, 45, -86, -135, 670155.630, 180.000},      // Across the south pole.
    {12.34, 56.78, 12.341, 56.781, 155.137, 44.514},  // Short hop.
};

TEST(GeoUtils, DistanceAndBearingMatchGeodesic) {
    for (const auto &c : kGeodesicCases) {
        GeoReference ref(DegToAWB(c.lat_a), DegToAWB(c.lon_a));
        uint32_t distance_m;
        uint16_t bearing_bam16;
        CalculateDistanceAndBearing(ref, DegToAWB(c.lat_b), DegToAWB(c.lon_b), distance_m, bearing_bam16);
        // 0.015% + 1 m up to 600 km, 0.15% beyond.
        double tolerance_m = c.distance_m < 600e3 ? 0.00015 * c.distance_m + 1.0 : 0.0015 * c.distance_m;
        EXPECT_NEAR(distance_m, c.distance_m, tolerance_m) << c.lat_a << "," << c.lon_a << " -> " << c.lat_b << ","
                                                           << c.lon_b;
        double bearing_error_deg = fmod(Bam16ToDeg(bearing_bam16) - c.azimuth_deg + 540.0, 360.0) - 180.0;
        EXPECT_LT(fabs(bearing_error_deg), 0.2) << c.lat_a << "," << c.lon_a << " -> " << c.lat_b << "," << c.lon_b;
        EXPECT_EQ(distance_m, CalculateGeoidalDistanceMetersAWB(DegToAWB(c.lat_a), DegToAWB(c.lon_a), DegToAWB(c.lat_b),
                                                                DegToAWB(c.lon_b)));
    }
}

TEST(GeoUtils, DistanceEdgeCases) {
    // Same point, anywhere.
    for (double lat : {-90.0, -45.0, 0.0, 45.0, 90.0}) {
        EXPECT_EQ(CalculateGeoidalDistanceMetersAWB(DegToAWB(lat), DegToAWB(10.0), DegToAWB(lat), DegToAWB(10.0)), 0u);
    }
    // The poles are one point whatever the longitude.
    EXPECT_LE(CalculateGeoidalDistanceMetersAWB(DegToAWB(90.0), 0, DegToAWB(90.0), DegToAWB(123.0)), 1u);
    // Antipodes: half the meridian (20003931 m).
    EXPECT_NEAR(CalculateGeoidalDistanceMetersAWB(0, 0, 0, DegToAWB(180.0)), 20003931, 1);
    EXPECT_NEAR(CalculateGeoidalDistanceMetersAWB(DegToAWB(90.0), 0, DegToAWB(-90.0), 0), 20003931, 1);
    // Symmetric within the error bound (the radius of curvature follows the bearing at the start point).
    EXPECT_NEAR(CalculateGeoidalDistanceMetersAWB(DegToAWB(40.0), DegToAWB(-100.0), DegToAWB(42.0), DegToAWB(-97.0)),
                CalculateGeoidalDistanceMetersAWB(DegToAWB(42.0), DegToAWB(-97.0), DegToAWB(40.0), DegToAWB(-100.0)),
                0.0003 * 336000);
}

TEST(GeoUtils, CalculateGeoidalDistanceDeg) {
    EXPECT_NEAR(CalculateGeoidalDistanceMetersDeg(20.35f, -13.78f, 19.35f, -13.78f), 110702, 20);
    EXPECT_NEAR(CalculateGeoidalDistanceMetersDeg(19.35f, -13.78f, 19.35f, -14.78f), 105070, 20);
}

TEST(GeoUtils, SinCosQ30) {
    for (double deg = -360.0; deg <= 360.0; deg += 0.0137) {
        int32_t s, c;
        SinCosQ30(DegToAWB(deg), s, c);
        double rad = deg * M_PI / 180.0;
        EXPECT_NEAR(s / 1073741824.0, sin(rad), 5e-9) << deg;
        EXPECT_NEAR(c / 1073741824.0, cos(rad), 5e-9) << deg;
    }
}

TEST(GeoUtils, BearingCardinalDirections) {
    GeoReference ref(DegToAWB(10.0), DegToAWB(20.0));
    uint32_t d;
    uint16_t b;
    CalculateDistanceAndBearing(ref, DegToAWB(10.5), DegToAWB(20.0), d, b);
    EXPECT_NEAR(Bam16ToDeg(b), 0.0f, 0.01f);
    CalculateDistanceAndBearing(ref, DegToAWB(10.0), DegToAWB(20.5), d, b);
    EXPECT_NEAR(Bam16ToDeg(b), 89.957f, 0.01f);  // A parallel is not a great circle: slightly north of east.
    CalculateDistanceAndBearing(ref, DegToAWB(9.5), DegToAWB(20.0), d, b);
    EXPECT_NEAR(Bam16ToDeg(b), 180.0f, 0.01f);
    CalculateDistanceAndBearing(ref, DegToAWB(10.0), DegToAWB(19.5), d, b);
    EXPECT_NEAR(Bam16ToDeg(b), 270.043f, 0.01f);
}

TEST(GeoUtils, IsDistanceGreaterThanAgreesWithDistance) {
    // Random pairs around thresholds: the integer bounds must never contradict the full calculation by more than its
    // error, and must agree exactly away from the threshold.
    std::mt19937 rng(455);
    std::uniform_real_distribution<double> lat_dist(-89.9, 89.9), lon_dist(-180.0, 180.0), off(-3.0, 3.0);
    for (int i = 0; i < 20000; i++) {
        double lat = lat_dist(rng), lon = lon_dist(rng);
        uint32_t a_lat = DegToAWB(lat), a_lon = DegToAWB(lon);
        uint32_t b_lat = DegToAWB(fmax(-90.0, fmin(90.0, lat + off(rng)))), b_lon = DegToAWB(lon + 2 * off(rng));
        uint32_t d = CalculateGeoidalDistanceMetersAWB(a_lat, a_lon, b_lat, b_lon);
        for (uint32_t threshold_m : {20000u, 50000u, 200000u}) {
            bool greater = IsDistanceGreaterThan(a_lat, a_lon, b_lat, b_lon, threshold_m);
            if (d > threshold_m * 1.0002 + 1) {
                EXPECT_TRUE(greater) << d << " vs " << threshold_m;
            } else if (d < threshold_m * 0.9998 - 1) {
                EXPECT_FALSE(greater) << d << " vs " << threshold_m;
            }
        }
    }
}

/**
 * Vincenty's inverse formula on WGS-84 in double precision (within 0.5 mm of the geodesic away from antipodes):
 * distance (m) and initial azimuth (deg), as the reference for CalculateNorthEastMeters.
 */
static void VincentyInverse(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg, double &s_m,
                            double &azimuth_deg) {
    const double a = 6378137.0, f = 1 / 298.257223563, b = a * (1 - f), rad = M_PI / 180.0;
    const double L = remainder(lon2_deg - lon1_deg, 360.0) * rad;
    const double U1 = atan((1 - f) * tan(lat1_deg * rad)), U2 = atan((1 - f) * tan(lat2_deg * rad));
    const double sinU1 = sin(U1), cosU1 = cos(U1), sinU2 = sin(U2), cosU2 = cos(U2);
    double lambda = L, sin_sigma = 0, cos_sigma = 1, sigma = 0, cos2_alpha = 1, cos_2sigma_m = 0;
    for (int i = 0; i < 1000; i++) {
        const double sin_lambda = sin(lambda), cos_lambda = cos(lambda);
        sin_sigma = hypot(cosU2 * sin_lambda, cosU1 * sinU2 - sinU1 * cosU2 * cos_lambda);
        if (sin_sigma == 0) {
            s_m = 0;
            azimuth_deg = 0;
            return;
        }
        cos_sigma = sinU1 * sinU2 + cosU1 * cosU2 * cos_lambda;
        sigma = atan2(sin_sigma, cos_sigma);
        const double sin_alpha = cosU1 * cosU2 * sin_lambda / sin_sigma;
        cos2_alpha = 1 - sin_alpha * sin_alpha;
        cos_2sigma_m = cos2_alpha != 0 ? cos_sigma - 2 * sinU1 * sinU2 / cos2_alpha : 0;
        const double C = f / 16 * cos2_alpha * (4 + f * (4 - 3 * cos2_alpha));
        const double previous = lambda;
        const double cos_term = cos_2sigma_m + C * cos_sigma * (-1 + 2 * cos_2sigma_m * cos_2sigma_m);
        lambda = L + (1 - C) * f * sin_alpha * (sigma + C * sin_sigma * cos_term);
        if (fabs(lambda - previous) < 1e-14) break;
    }
    const double u2 = cos2_alpha * (a * a - b * b) / (b * b);
    const double A = 1 + u2 / 16384 * (4096 + u2 * (-768 + u2 * (320 - 175 * u2)));
    const double B = u2 / 1024 * (256 + u2 * (-128 + u2 * (74 - 47 * u2)));
    const double delta_sigma =
        B * sin_sigma *
        (cos_2sigma_m + B / 4 *
                            (cos_sigma * (-1 + 2 * cos_2sigma_m * cos_2sigma_m) -
                             B / 6 * cos_2sigma_m * (-3 + 4 * sin_sigma * sin_sigma) *
                                 (-3 + 4 * cos_2sigma_m * cos_2sigma_m)));
    s_m = b * A * (sigma - delta_sigma);
    azimuth_deg = atan2(cosU2 * sin(lambda), cosU1 * sinU2 - sinU1 * cosU2 * cos(lambda)) / rad;
}

static double AWBToDeg(uint32_t awb) { return static_cast<int32_t>(awb) * (360.0 / 4294967296.0); }

TEST(GeoUtils, VincentyReferenceMatchesGeographicLib) {
    for (const auto &c : kGeodesicCases) {
        if (fabs(c.lat_a) > 89 || fabs(c.lat_b) > 89) continue;  // Across the poles: azimuths are degenerate there.
        double s, azimuth;
        VincentyInverse(c.lat_a, c.lon_a, c.lat_b, c.lon_b, s, azimuth);
        EXPECT_NEAR(s, c.distance_m, 0.002);
        EXPECT_NEAR(fmod(azimuth + 360.0, 360.0), c.azimuth_deg, 0.0006);
    }
}

TEST(GeoUtils, NorthEastMatchesGeodesicAtFLARMRanges) {
    // Targets out to 20 km in every direction from ownship anywhere: every latitude band including near the poles,
    // and longitudes that straddle the antimeridian. Reference: the geodesic distance split by its initial azimuth.
    std::mt19937 rng(1293);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    double max_component_error_m = 0, max_distance_error_m = 0;
    for (int i = 0; i < 200000; i++) {
        double lat = i % 4 == 0 ? (unit(rng) < 0.5 ? -1 : 1) * (80.0 + 9.7 * unit(rng)) : -80.0 + 160.0 * unit(rng);
        double lon = i % 4 == 1 ? 179.9 + 0.2 * unit(rng) : -180.0 + 360.0 * unit(rng);
        const double range_m = i % 8 == 0 ? 500.0 * unit(rng) : 20000.0 * unit(rng);
        const double bearing = 2 * M_PI * unit(rng);
        // Target by the spherical direct formula: only roughly at range_m, the reference measures it exactly.
        const double delta = range_m / 6371008.8, phi = lat * M_PI / 180.0;
        const double phi2 = asin(sin(phi) * cos(delta) + cos(phi) * sin(delta) * cos(bearing));
        const double lon2 =
            lon + atan2(sin(bearing) * sin(delta) * cos(phi), cos(delta) - sin(phi) * sin(phi2)) * 180.0 / M_PI;
        const uint32_t own_lat = DegToAWB(lat), own_lon = DegToAWB(remainder(lon, 360.0));
        const uint32_t lat_b = DegToAWB(phi2 * 180.0 / M_PI), lon_b = DegToAWB(remainder(lon2, 360.0));
        double s, azimuth;
        VincentyInverse(AWBToDeg(own_lat), AWBToDeg(own_lon), AWBToDeg(lat_b), AWBToDeg(lon_b), s, azimuth);
        int32_t north, east;
        uint32_t distance;
        CalculateNorthEastMeters(GeoReference(own_lat, own_lon), lat_b, lon_b, north, east, distance);
        const double north_error = north - s * cos(azimuth * M_PI / 180.0);
        const double east_error = east - s * sin(azimuth * M_PI / 180.0);
        max_component_error_m = fmax(max_component_error_m, fmax(fabs(north_error), fabs(east_error)));
        max_distance_error_m = fmax(max_distance_error_m, fabs(distance - s));
        ASSERT_LT(fabs(north_error), 0.65) << lat << "," << lon << " " << range_m << " m at " << bearing;
        ASSERT_LT(fabs(east_error), 0.65) << lat << "," << lon << " " << range_m << " m at " << bearing;
        ASSERT_LT(fabs(distance - s), 0.65) << lat << "," << lon << " " << range_m << " m at " << bearing;
    }
    printf("CalculateNorthEastMeters to 20 km: max north/east error %.3f m, max distance error %.3f m\n",
           max_component_error_m, max_distance_error_m);
}

TEST(GeoUtils, NorthEastAgreesWithDistanceAndBearing) {
    GeoReference ref(DegToAWB(47.3), DegToAWB(8.5));
    int32_t north, east;
    uint32_t distance, distance2;
    uint16_t bearing;
    CalculateNorthEastMeters(ref, DegToAWB(47.3), DegToAWB(8.5), north, east, distance);
    EXPECT_EQ(north, 0);
    EXPECT_EQ(east, 0);
    EXPECT_EQ(distance, 0u);
    CalculateNorthEastMeters(ref, DegToAWB(47.4), DegToAWB(8.4), north, east, distance);
    CalculateDistanceAndBearing(ref, DegToAWB(47.4), DegToAWB(8.4), distance2, bearing);
    EXPECT_EQ(distance, distance2);
    EXPECT_GT(north, 0);
    EXPECT_LT(east, 0);
    // The ellipsoid's azimuth (GeographicLib: 325.8362 deg), where CalculateDistanceAndBearing keeps the spherical one.
    EXPECT_NEAR(atan2(east, north) * 180.0 / M_PI + 360.0, 325.8362, 0.005);
    EXPECT_NEAR(Bam16ToDeg(bearing), 325.8362, 0.2);
}

TEST(GeoUtils, FloatDegToAWBIsExact) {
    std::mt19937 rng(410);
    std::uniform_real_distribution<float> deg(-360.0f, 360.0f);
    for (int i = 0; i < 200000; i++) {
        const float d = i < 1000 ? deg(rng) * 1e-6f : deg(rng);
        // The float is exact in double; 2^32 / 360 of it, rounded and wrapped to 32 bits.
        const int64_t exact = llround(static_cast<double>(d) * (4294967296.0 / 360.0));
        const int32_t error = static_cast<int32_t>(FloatDegToAWB(d) - static_cast<uint32_t>(exact));
        ASSERT_LE(abs(error), 1) << d;
    }
    EXPECT_EQ(FloatDegToAWB(0.0f), 0u);
    EXPECT_EQ(FloatDegToAWB(-0.0f), 0u);
    EXPECT_EQ(FloatDegToAWB(90.0f), 0x40000000u);
    EXPECT_EQ(FloatDegToAWB(-90.0f), 0xC0000000u);
    EXPECT_EQ(FloatDegToAWB(180.0f), 0x80000000u);
    EXPECT_EQ(FloatDegToAWB(-180.0f), 0x80000000u);
    EXPECT_EQ(FloatDegToAWB(-122.1f), DegToAWB(-122.1f));
    EXPECT_EQ(FloatDegToAWB(NAN), 0u);
}

/** Test track and speed calculation from NE velocity components. **/
void expect_track_and_speed(int32_t n_vel_kts, int32_t e_vel_kts, float expected_track_deg,
                            int32_t expected_speed_kts) {
    float track_deg = 0.0f;
    int32_t speed_kts = 0;
    CalculateTrackAndSpeedFromNEVelocities(n_vel_kts, e_vel_kts, track_deg, speed_kts);
    EXPECT_NEAR(expected_track_deg, track_deg, kTrackToleranceDeg)
        << "Track failed for n=" << n_vel_kts << " e=" << e_vel_kts;
    EXPECT_NEAR(expected_speed_kts, speed_kts, kSpeedToleranceKts)
        << "Speed failed for n=" << n_vel_kts << " e=" << e_vel_kts;
}

TEST(GeoUtils, CalculateTrackAndSpeed_Table) {
    expect_track_and_speed(100, 0, 0.0f, 100);     // Due north
    expect_track_and_speed(0, 100, 90.0f, 100);    // Due east
    expect_track_and_speed(-100, 0, 180.0f, 100);  // Due south
    expect_track_and_speed(0, -100, 270.0f, 100);  // Due west
    expect_track_and_speed(100, 100, 45.0f, 141);  // NE diagonal: speed = sqrt(2)*100 ≈ 141
}
