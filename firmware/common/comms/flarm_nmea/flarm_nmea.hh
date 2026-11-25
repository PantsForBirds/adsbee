#pragma once

#include <cstdint>

#include "adsb_types.hh"
#include "aircraft_dictionary.hh"
#include "geo_utils.hh"
#include "settings.hh"

/**
 * FLARM data port sentences (FLARM FTD-012 Data Port ICD v7.21, data port version 9) for received traffic, as read by
 * XCSoar, LK8000, SeeYou Navigator, SkyDemon, Enroute and other apps with a FLARM input.
 *
 * ADSBee transmits no FLARM and computes no collision alarms: PFLAU reports TX 0 and alarm level 0, and every PFLAA has
 * alarm level 0. PFLAA positions are relative to ownship, so traffic is sent only while ownship is known (GNSS fix, or
 * the tracked aircraft that matches the configured ICAO address; see GDL90Reporter::IsValidOwnshipSource).
 *
 * PFLAA vertical separation never mixes altitude references: baro against baro, else geometric against geometric. ADS-B
 * and Remote ID geometric altitudes are heights above the WGS-84 ellipsoid (HAE); a GNSS receiver's altitude is above
 * mean sea level, which differs from HAE by the local geoid height (up to about 100 m). A GNSS ownship therefore has a
 * geometric altitude only when the receiver reports its geoid separation (HAE = MSL + separation, as the GDL90 Ownship
 * Geometric Altitude message, GDL90Reporter::GNSSHeightAboveEllipsoidFt()); without one it is not compared with them. Baro and geometric altitudes differ by the weather and temperature, so they are compared only through a
 * measured difference: ADS-B airborne velocity messages carry each aircraft's GNSS minus baro difference, from which
 * the dictionary derives its geometric altitude. A target with no altitude in ownship's references gets no PFLAA
 * (FTD-012 defines no "unknown" RelativeVertical, and XCSoar drops a PFLAA without one); it is still counted in PFLAU.
 *
 * Traffic positions come from the AircraftDictionary's distance and bearing to each aircraft (one geodesy routine for
 * every user, see geo_utils.hh), which it measures from the receiver position: the caller passes only targets whose
 * values are current and measured from ownship (AircraftDictionary::DistanceIsCurrent(), DistanceReferenceIs()). All
 * arithmetic is integer, so it costs the same on the ESP32 and the RP2040.
 *
 * Writers return the sentence length including "$", "*hh" and CR LF, or 0 if nothing was written.
 */
class FLARMNMEAReporter {
   public:
    // FTD-012: at most 80 characters between "$" and CR LF.
    static constexpr uint16_t kMaxSentenceLenBytes = 83;
    static constexpr int32_t kMaxRelativeHorizontalM = 20000000;  // PFLAA <RelativeNorth> / <RelativeEast> range.

    enum IDType : uint8_t { kIDTypeRandom = 0, kIDTypeICAO = 1 };
    enum Source : uint8_t {
        kSourceADSB = 1,
        kSourceADSR = 3,
        kSourceTISB = 4,
        kSourceNone = 0xFF,  // No FTD-012 source code (Remote ID): <Source> and <RSSI> are omitted.
    };
    enum AircraftType : uint8_t {
        kAircraftTypeGlider = 0x1,
        kAircraftTypeRotorcraft = 0x3,
        kAircraftTypeSkydiver = 0x4,
        kAircraftTypeHangGlider = 0x6,
        kAircraftTypePiston = 0x8,
        kAircraftTypeJet = 0x9,
        kAircraftTypeUnknown = 0xA,
        kAircraftTypeBalloon = 0xB,
        kAircraftTypeUAV = 0xD,
        kAircraftTypeStaticObstacle = 0xF,
    };

    struct UTCTime {
        bool time_valid = false;
        uint8_t hour = 0, minute = 0, second = 0;
        bool date_valid = false;  // GPRMC leaves the date empty without it.
        uint16_t year = 0;        // Four digits.
        uint8_t month = 0, day = 0;
    };

    struct Ownship {
        uint32_t lat_awb = 0;  // Receiver position, as the dictionary measures from it.
        uint32_t lon_awb = 0;
        bool msl_altitude_valid = false;
        int32_t msl_altitude_ft = 0;  // GNSS altitude above mean sea level, for GPGGA.
        bool geoid_separation_valid = false;
        int32_t geoid_separation_cm = 0;  // GNSS geoid height above the WGS-84 ellipsoid, for GPGGA.
        bool geometric_altitude_valid = false;
        int32_t geometric_altitude_ft = 0;  // Height above the WGS-84 ellipsoid.
        bool baro_altitude_valid = false;
        int32_t baro_altitude_ft = 0;
        float track_deg = 0;
        int32_t speed_kts = 0;
        bool airborne = false;
    };

    struct Target {
        uint32_t id = 0;  // 24 bits.
        IDType id_type = kIDTypeICAO;
        // The dictionary's distance and bearing from the receiver (Aircraft::distance_m, bearing_bam16).
        bool distance_valid = false;
        uint32_t distance_m = 0;
        uint16_t bearing_bam16 = 0;
        bool geometric_altitude_valid = false;
        int32_t geometric_altitude_ft = 0;  // Height above the WGS-84 ellipsoid.
        bool baro_altitude_valid = false;
        int32_t baro_altitude_ft = 0;
        bool track_valid = false;
        float track_deg = 0;
        bool speed_valid = false;
        int32_t speed_kts = 0;
        bool on_ground = false;
        bool vertical_rate_valid = false;
        int32_t vertical_rate_fpm = 0;
        AircraftType type = kAircraftTypeUnknown;
        Source source = kSourceADSB;
        bool rssi_valid = false;
        int16_t rssi_dbm = 0;
    };

    // Ownship-relative position of a target, in meters.
    struct RelativePosition {
        int32_t north_m = 0;
        int32_t east_m = 0;
        int32_t vertical_m = 0;   // Above ownship.
        uint32_t distance_m = 0;  // Horizontal, along the surface.
    };

    /**
     * Ownship from the receiver position. A GNSS fix gives the mean sea level altitude (GPGGA) and, with the receiver's
     * geoid separation, a geometric altitude (MSL + separation) to compare traffic with; without one, none. For the tracked aircraft with the configured ICAO address, RxPosition carries altitudes
     * without validity flags: they are left invalid, and OwnshipAltitudesFromAircraft() sets them from the aircraft.
     * @param[in] geoid_separation_valid Whether the GNSS receiver reported a geoid separation (GNSS source only).
     * @param[in] geoid_separation_cm Geoid height above the WGS-84 ellipsoid, in centimeters.
     * @retval False if the position isn't this aircraft's own (GDL90Reporter::IsValidOwnshipSource) or unavailable.
     */
    static bool OwnshipFromRxPosition(Ownship& own, const SettingsManager::RxPosition& rx_position,
                                      bool rx_position_available, bool geoid_separation_valid = false,
                                      int32_t geoid_separation_cm = 0);

    // Baro and geometric altitudes of the tracked aircraft that is ownship, as valid as the aircraft has them.
    static void OwnshipAltitudesFromAircraft(Ownship& own, const ModeSAircraft& aircraft);
    static void OwnshipAltitudesFromAircraft(Ownship& own, const UATAircraft& aircraft);

    /**
     * PFLAA targets from dictionary entries.
     * @retval False if the entry has no valid position.
     */
    static bool TargetFromAircraft(Target& target, const ModeSAircraft& aircraft);
    static bool TargetFromAircraft(Target& target, const UATAircraft& aircraft);
    static bool TargetFromAircraft(Target& target, const RemoteIDAircraft& aircraft);

    /**
     * Ownship-relative position: north and east split from the target's distance and bearing (SplitNorthEastMeters(),
     * within 1.2 m of the WGS-84 geodesic to 10 km), vertical baro against baro, else geometric against geometric (see
     * the class comment).
     * @retval False without a distance, without an altitude in a reference ownship has, or beyond the PFLAA range.
     */
    static bool Relative(const Ownship& own, const Target& target, RelativePosition& relative);

    // FLARM aircraft type for an ADS-B emitter category.
    static AircraftType AircraftTypeFromEmitterCategory(ADSBTypes::EmitterCategory category);

    /**
     * PFLAU heartbeat, once per second.
     * @param[in] num_received Aircraft currently reported (clamped to 99).
     * @param[in] gps 0 without ownship, 1 ownship on ground, 2 ownship airborne.
     */
    static uint16_t WritePFLAU(char* buf, uint16_t buf_len, uint16_t num_received, uint8_t gps);
    // 0 if Relative() fails.
    static uint16_t WritePFLAA(char* buf, uint16_t buf_len, const Ownship& own, const Target& target);
    // 0 without a valid UTC time.
    static uint16_t WriteGPRMC(char* buf, uint16_t buf_len, const Ownship& own, const UTCTime& utc);
    // Fix quality 1. Satellites and HDOP empty; altitude (MSL) only with an MSL altitude, geoid separation (field 11)
    // only with one. 0 without a
    // valid UTC time.
    static uint16_t WriteGPGGA(char* buf, uint16_t buf_len, const Ownship& own, const UTCTime& utc);
    // Barometric altitude in feet. 0 without an ownship baro altitude.
    static uint16_t WritePGRMZ(char* buf, uint16_t buf_len, const Ownship& own);

   private:
    // Position, altitudes, track, speed, vertical rate, type and signal, shared by Mode S and UAT aircraft.
    template <typename T>
    static void FillMotion(Target& target, const T& aircraft);
    template <typename T>
    static void FillOwnshipAltitudes(Ownship& own, const T& aircraft);
    // Appends "*hh\r\n" to the sentence body in buf, which starts with '$'.
    static uint16_t Finish(char* buf, uint16_t buf_len, int body_len);
};
