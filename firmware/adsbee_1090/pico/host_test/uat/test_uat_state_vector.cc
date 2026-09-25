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
