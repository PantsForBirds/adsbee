#include <cmath>
#include <cstring>

#include "aircraft_dictionary.hh"
#include "fec.hh"
#include "gtest/gtest.h"
#include "hal_god_powers.hh"  // for faking timestamps
#include "uat_packet.hh"

/**
 * Tests that build UAT basic ADS-B messages field by field (per the UAT Tech Manual STATE VECTOR layout, Table 2-11),
 * FEC-encode them, and run them through the full decode + aircraft dictionary ingest path.
 */

namespace {

struct UATStateVectorFields {
    uint8_t payload_type = 0;
    uint8_t address_qualifier = 0;
    uint32_t address = 0xABCDEF;
    uint32_t latitude_awb23 = 0;
    uint32_t longitude_awb24 = 0;
    bool altitude_is_geometric = false;
    uint16_t altitude_encoded = 0;
    uint8_t nic = 0;
    uint8_t air_ground_state = 0;
    uint16_t north_velocity_or_ground_speed = 0;  // 11 bits.
    uint16_t east_velocity_or_track = 0;          // 11 bits.
    uint16_t vertical_velocity_or_av_size = 0;    // 11 bits.
    uint8_t utc_or_tis_b_site_id = 0;             // 4 bits.
};

void SetBits(uint8_t* buf, uint32_t first_bit, uint16_t n, uint32_t value) {
    for (uint16_t i = 0; i < n; i++) {
        uint32_t bit = first_bit + i;
        bool set = (value >> (n - 1 - i)) & 0b1;
        if (set) {
            buf[bit / 8] |= (0x80 >> (bit % 8));
        } else {
            buf[bit / 8] &= ~(0x80 >> (bit % 8));
        }
    }
}

uint32_t LatDegToAWB23(double lat_deg) {
    // 24-bit angular weighted binary with the MSB dropped (Table 2-12 note 1).
    return static_cast<uint32_t>(llround(lat_deg * 16777216.0 / 360.0)) & 0x7FFFFF;
}

uint32_t LonDegToAWB24(double lon_deg) {
    return static_cast<uint32_t>(llround(lon_deg * 16777216.0 / 360.0)) & 0xFFFFFF;
}

// Builds and FEC-encodes a basic (payload type 0) UAT ADS-B message and decodes it through the normal FEC path.
DecodedUATADSBPacket BuildBasicPacket(const UATStateVectorFields& f) {
    uint8_t buf[RawUATADSBPacket::kShortADSBMessageNumBytes] = {0};
    SetBits(buf, 0, 5, f.payload_type);
    SetBits(buf, 5, 3, f.address_qualifier);
    SetBits(buf, 8, 24, f.address);
    SetBits(buf, 32, 23, f.latitude_awb23);
    SetBits(buf, 55, 24, f.longitude_awb24);
    SetBits(buf, 79, 1, f.altitude_is_geometric);
    SetBits(buf, 80, 12, f.altitude_encoded);
    SetBits(buf, 92, 4, f.nic);
    SetBits(buf, 96, 2, f.air_ground_state);
    SetBits(buf, 99, 11, f.north_velocity_or_ground_speed);
    SetBits(buf, 110, 11, f.east_velocity_or_track);
    SetBits(buf, 121, 11, f.vertical_velocity_or_av_size);
    SetBits(buf, 132, 4, f.utc_or_tis_b_site_id);
    uat_rs.EncodeShortADSBMessage(buf);
    return DecodedUATADSBPacket(RawUATADSBPacket(buf, sizeof(buf)));
}

UATAircraft IngestAndGet(AircraftDictionary& dictionary, const DecodedUATADSBPacket& packet) {
    // The position filter only accepts a report with a strictly newer timestamp than the last one it saw (starting
    // from 0), so step the fake system clock before every ingest.
    inc_time_since_boot_ms(1000);
    EXPECT_TRUE(packet.IsValid());
    EXPECT_TRUE(dictionary.IngestDecodedUATADSBPacket(packet));
    UATAircraft aircraft;
    EXPECT_TRUE(dictionary.GetAircraft(Aircraft::ICAOToUID(packet.GetICAOAddress(), Aircraft::kAircraftTypeUAT),
                                       aircraft));
    return aircraft;
}

const float kAWB24ResolutionDeg = 360.0f / 16777216.0f;

}  // namespace

TEST(UATStateVector, PositionAllQuadrants) {
    const struct {
        double lat;
        double lon;
    } positions[] = {
        {37.6188, -122.3754},   // KSFO: north / west.
        {-33.9461, 151.1772},   // YSSY: south / east.
        {-54.8433, -68.2958},   // SAWH: south / west.
        {64.1300, -21.9406},    // BIRK: north / west.
        {1.3502, 103.9940},     // WSSS: north / east.
        {-0.0005, 0.0005},      // Just south of the equator.
        {-89.9990, 179.9990},   // Near the south pole / antimeridian.
        {89.9990, -179.9990},   // Near the north pole / antimeridian.
    };
    for (const auto& p : positions) {
        char msg[64];
        snprintf(msg, sizeof(msg), "lat=%f lon=%f", p.lat, p.lon);
        SCOPED_TRACE(msg);
        AircraftDictionary dictionary;
        UATStateVectorFields f;
        f.latitude_awb23 = LatDegToAWB23(p.lat);
        f.longitude_awb24 = LonDegToAWB24(p.lon);
        f.nic = 8;
        UATAircraft aircraft = IngestAndGet(dictionary, BuildBasicPacket(f));
        EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagPositionValid));
        EXPECT_NEAR(aircraft.latitude_deg, p.lat, 2 * kAWB24ResolutionDeg + 1e-5);
        EXPECT_NEAR(aircraft.longitude_deg, p.lon, 2 * kAWB24ResolutionDeg + 1e-5);
    }
}

TEST(UATStateVector, PositionFilterAcceptsEquatorCrossing) {
    // Consecutive reports 1 s apart, ~110 m apart, straddling the equator. The position filter must see them as close
    // together (it works on 32-bit AWB positions, so the 23-bit UAT latitude has to be sign-restored correctly).
    AircraftDictionary dictionary;
    UATStateVectorFields f;
    f.nic = 8;
    f.latitude_awb23 = LatDegToAWB23(0.0005);
    f.longitude_awb24 = LonDegToAWB24(-30.0);
    IngestAndGet(dictionary, BuildBasicPacket(f));

    f.latitude_awb23 = LatDegToAWB23(-0.0005);
    UATAircraft aircraft = IngestAndGet(dictionary, BuildBasicPacket(f));
    EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagUpdatedPosition));
    EXPECT_NEAR(aircraft.latitude_deg, -0.0005, 2 * kAWB24ResolutionDeg);
}

namespace {
// A/G state values (Table 2-16).
const uint8_t kAGAirborneSubsonic = 0;
const uint8_t kAGOnGround = 2;

// Ground "Track Angle/Heading" subfield (Table 2-27): 2-bit type then 9-bit angle (360/512 deg per LSB).
uint16_t GroundTrack(uint8_t type, uint16_t angle_ticks) { return (type << 9) | (angle_ticks & 0x1FF); }

// "A/V Length and Width" form of the vertical velocity field (Table 2-34): 4-bit L/W code, 1-bit POA, 6 reserved.
uint16_t AVSize(uint8_t lw_code, bool poa) { return (lw_code << 7) | (poa << 6); }
}  // namespace

TEST(UATStateVector, OnGroundSpeedWithoutTrack) {
    // Ground speed is independent of the Track Angle/Heading subfield: a surface target may report a speed with the
    // track type "not available" (common for vehicles and stationary aircraft).
    AircraftDictionary dictionary;
    UATStateVectorFields f;
    f.air_ground_state = kAGOnGround;
    f.north_velocity_or_ground_speed = 11;  // 10 kts.
    f.east_velocity_or_track = GroundTrack(0, 0);
    UATAircraft aircraft = IngestAndGet(dictionary, BuildBasicPacket(f));
    EXPECT_FALSE(aircraft.HasBitFlag(UATAircraft::kBitFlagIsAirborne));
    EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagHorizontalSpeedValid));
    EXPECT_EQ(aircraft.speed_kts, 10);
    EXPECT_FALSE(aircraft.HasBitFlag(UATAircraft::kBitFlagDirectionValid));
}

TEST(UATStateVector, OnGroundTrackWithoutSpeed) {
    // "Ground speed not available" (0) with a valid true track: direction is valid, speed must not be (it would
    // otherwise be reported as INT32_MIN knots).
    AircraftDictionary dictionary;
    UATStateVectorFields f;
    f.air_ground_state = kAGOnGround;
    f.north_velocity_or_ground_speed = 0;
    f.east_velocity_or_track = GroundTrack(ADSBTypes::kDirectionTypeTrueTrackAngle, 128);  // 90 deg.
    UATAircraft aircraft = IngestAndGet(dictionary, BuildBasicPacket(f));
    EXPECT_FALSE(aircraft.HasBitFlag(UATAircraft::kBitFlagHorizontalSpeedValid));
    EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagDirectionValid));
    EXPECT_FALSE(aircraft.HasBitFlag(UATAircraft::kBitFlagDirectionIsHeading));
    EXPECT_NEAR(aircraft.direction_deg, 90.0f, 0.01f);
}

TEST(UATStateVector, OnGroundHeadingTypes) {
    const struct {
        uint8_t type;
        bool is_heading;
        bool magnetic;
    } cases[] = {{ADSBTypes::kDirectionTypeTrueTrackAngle, false, false},
                 {ADSBTypes::kDirectionTypeMagneticHeading, true, true},
                 {ADSBTypes::kDirectionTypeTrueHeading, true, false}};
    for (const auto& c : cases) {
        SCOPED_TRACE(c.type);
        AircraftDictionary dictionary;
        UATStateVectorFields f;
        f.air_ground_state = kAGOnGround;
        f.north_velocity_or_ground_speed = 6;                    // 5 kts.
        f.east_velocity_or_track = GroundTrack(c.type, 256 + 64);  // 225 deg.
        UATAircraft aircraft = IngestAndGet(dictionary, BuildBasicPacket(f));
        EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagDirectionValid));
        EXPECT_EQ(aircraft.HasBitFlag(UATAircraft::kBitFlagDirectionIsHeading), c.is_heading);
        EXPECT_EQ(aircraft.HasBitFlag(UATAircraft::kBitFlagHeadingUsesMagneticNorth), c.magnetic);
        EXPECT_NEAR(aircraft.direction_deg, 225.0f, 0.01f);
        EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagHorizontalSpeedValid));
        EXPECT_EQ(aircraft.speed_kts, 5);
    }
}

TEST(UATStateVector, OnGroundDimensionsWithPositionOffsetApplied) {
    // The POA bit (Table 2-36) only says the reported position was normalized to the ADS-B reference point. The field
    // still carries the A/V length/width code (Table 2-35), never a GNSS antenna offset.
    for (bool poa : {false, true}) {
        SCOPED_TRACE(poa ? "POA=1" : "POA=0");
        AircraftDictionary dictionary;
        UATStateVectorFields f;
        f.air_ground_state = kAGOnGround;
        f.vertical_velocity_or_av_size = AVSize(5, poa);  // 25 < L <= 35 m, 33 < W <= 38 m.
        UATAircraft aircraft = IngestAndGet(dictionary, BuildBasicPacket(f));
        EXPECT_EQ(aircraft.length_m, 35);
        EXPECT_EQ(aircraft.width_m, 38);
        EXPECT_EQ(aircraft.gnss_antenna_offset_forward_of_reference_point_m, 0);
        EXPECT_EQ(aircraft.gnss_antenna_offset_right_of_reference_point_m, 0);
    }
}

TEST(UATStateVector, AirborneZeroVelocityHasNoTrack) {
    // N/S and E/W velocity both "zero" (encoded 1): speed is 0 kts and valid, but the track angle is undefined.
    AircraftDictionary dictionary;
    UATStateVectorFields f;
    f.air_ground_state = kAGAirborneSubsonic;
    f.north_velocity_or_ground_speed = 1;
    f.east_velocity_or_track = 1;
    UATAircraft aircraft = IngestAndGet(dictionary, BuildBasicPacket(f));
    EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagHorizontalSpeedValid));
    EXPECT_EQ(aircraft.speed_kts, 0);
    EXPECT_FALSE(aircraft.HasBitFlag(UATAircraft::kBitFlagDirectionValid));
}

TEST(UATStateVector, AirborneVelocitySigns) {
    // N/S sign 1 = south, E/W sign 1 = west (Tables 2-20, 2-25). 100 kts south + 100 kts west = track 225.
    AircraftDictionary dictionary;
    UATStateVectorFields f;
    f.air_ground_state = kAGAirborneSubsonic;
    f.north_velocity_or_ground_speed = (1 << 10) | 101;
    f.east_velocity_or_track = (1 << 10) | 101;
    UATAircraft aircraft = IngestAndGet(dictionary, BuildBasicPacket(f));
    EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagDirectionValid));
    EXPECT_NEAR(aircraft.direction_deg, 225.0f, 0.1f);
    EXPECT_NEAR(aircraft.speed_kts, 141, 1);
}

namespace {
// Base-40 digit for a callsign character (Table 2-41). ' ' = 36; '\x25' (37) = "not available".
uint16_t B40(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'Z') return c - 'A' + 10;
    if (c == ' ') return 36;
    return 37;
}

// Builds and FEC-encodes a long payload type 1 message (HDR | SV | MS | AUX SV) with the given 8-character callsign
// field and CSID bit. The state vector is left "not available".
DecodedUATADSBPacket BuildModeStatusPacket(uint32_t address, uint8_t emitter_category, const char callsign[8],
                                           bool csid) {
    uint8_t buf[RawUATADSBPacket::kLongADSBMessageNumBytes] = {0};
    SetBits(buf, 0, 5, 1);  // Payload type 1.
    SetBits(buf, 8, 24, address);
    const uint32_t ms = 17 * 8;  // Mode Status starts at payload byte 18.
    SetBits(buf, ms + 0, 16, emitter_category * 1600 + B40(callsign[0]) * 40 + B40(callsign[1]));
    SetBits(buf, ms + 16, 16, B40(callsign[2]) * 1600 + B40(callsign[3]) * 40 + B40(callsign[4]));
    SetBits(buf, ms + 32, 16, B40(callsign[5]) * 1600 + B40(callsign[6]) * 40 + B40(callsign[7]));
    SetBits(buf, ms + 78, 1, csid);
    uat_rs.EncodeLongADSBMessage(buf);
    return DecodedUATADSBPacket(RawUATADSBPacket(buf, sizeof(buf)));
}
}  // namespace

TEST(UATModeStatus, CallsignNotAvailableKeepsLastCallsign) {
    AircraftDictionary dictionary;
    UATAircraft aircraft = IngestAndGet(dictionary, BuildModeStatusPacket(0x123456, 1, "N12345  ", true));
    EXPECT_STREQ(aircraft.callsign, "N12345  ");
    EXPECT_EQ(aircraft.emitter_category, ADSBTypes::kEmitterCategoryLight);

    // All eight characters = base-40 digit 37 means "call sign not available" (§3.2.1.5.4.2); that must not blank
    // out the callsign we already know.
    const char kNotAvailable[8] = {'\x25', '\x25', '\x25', '\x25', '\x25', '\x25', '\x25', '\x25'};
    aircraft = IngestAndGet(dictionary, BuildModeStatusPacket(0x123456, 1, kNotAvailable, true));
    EXPECT_STREQ(aircraft.callsign, "N12345  ");
}

TEST(UATModeStatus, SquawkFromFlightPlanID) {
    AircraftDictionary dictionary;
    UATAircraft aircraft = IngestAndGet(dictionary, BuildModeStatusPacket(0x654321, 0, "1200    ", false));
    EXPECT_EQ(aircraft.squawk, 1200);

    // A non-numeric flight plan ID must not be turned into a bogus squawk.
    aircraft = IngestAndGet(dictionary, BuildModeStatusPacket(0x654321, 0, "12A4    ", false));
    EXPECT_EQ(aircraft.squawk, 1200);
}

TEST(UATStateVector, AltitudeAboveMaxIsFiniteAndValid) {
    // Altitude code 4095 means "> 101,337.5 ft" (Table 2-14). It used to decode to INT32_MAX and was stored as a valid
    // altitude of 2147483647 ft.
    AircraftDictionary dictionary;
    UATStateVectorFields f;
    f.altitude_encoded = 4095;
    UATAircraft aircraft = IngestAndGet(dictionary, BuildBasicPacket(f));
    EXPECT_TRUE(aircraft.HasBitFlag(UATAircraft::kBitFlagBaroAltitudeValid));
    EXPECT_EQ(aircraft.baro_altitude_ft, 101350);
}
