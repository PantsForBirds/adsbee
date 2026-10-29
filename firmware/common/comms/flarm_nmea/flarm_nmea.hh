#pragma once

#include <cstdint>

#include "adsb_types.hh"
#include "aircraft_dictionary.hh"
#include "settings.hh"

/**
 * FLARM data port sentences (FLARM FTD-012 Data Port ICD v7.21, data port version 9) for received traffic, as read by
 * XCSoar, LK8000, SeeYou Navigator, SkyDemon, Enroute and other apps with a FLARM input.
 *
 * ADSBee transmits no FLARM and computes no collision alarms: PFLAU reports TX 0 and alarm level 0, and every PFLAA has
 * alarm level 0. PFLAA positions are relative to ownship, so traffic is sent only while ownship is known (GNSS fix, or
 * the tracked aircraft that matches the configured ICAO address; see GDL90Reporter::IsValidOwnshipSource).
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
        double latitude_deg = 0;
        double longitude_deg = 0;
        bool gnss_altitude_valid = false;
        int32_t gnss_altitude_ft = 0;  // Mean sea level.
        bool baro_altitude_valid = false;
        int32_t baro_altitude_ft = 0;
        float track_deg = 0;
        int32_t speed_kts = 0;
        bool airborne = false;
    };

    struct Target {
        uint32_t id = 0;  // 24 bits.
        IDType id_type = kIDTypeICAO;
        double latitude_deg = 0;
        double longitude_deg = 0;
        bool gnss_altitude_valid = false;
        int32_t gnss_altitude_ft = 0;
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

    /**
     * Ownship from the receiver position.
     * @retval False if the position isn't this aircraft's own (GDL90Reporter::IsValidOwnshipSource) or unavailable.
     */
    static bool OwnshipFromRxPosition(Ownship& own, const SettingsManager::RxPosition& rx_position,
                                      bool rx_position_available);

    /**
     * PFLAA targets from dictionary entries.
     * @retval False if the entry has no valid position.
     */
    static bool TargetFromAircraft(Target& target, const ModeSAircraft& aircraft);
    static bool TargetFromAircraft(Target& target, const UATAircraft& aircraft);
    static bool TargetFromAircraft(Target& target, const RemoteIDAircraft& aircraft);

    /**
     * Ownship-relative position. Vertical separation uses baro against baro, else GNSS against GNSS (ADS-B geometric
     * altitude is height above the ellipsoid, ownship GNSS is above mean sea level: off by the local geoid separation),
     * else target baro against ownship GNSS.
     * @retval False without a common altitude reference, or beyond the PFLAA range.
     */
    static bool Relative(const Ownship& own, const Target& target, int32_t& north_m, int32_t& east_m,
                         int32_t& vertical_m);

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
    // Fix quality 1. Satellites, HDOP and geoid separation empty. 0 without a valid UTC time.
    static uint16_t WriteGPGGA(char* buf, uint16_t buf_len, const Ownship& own, const UTCTime& utc);
    // Barometric altitude in feet. 0 without an ownship baro altitude.
    static uint16_t WritePGRMZ(char* buf, uint16_t buf_len, const Ownship& own);

   private:
    // Appends "*hh\r\n" to the sentence body in buf, which starts with '$'.
    static uint16_t Finish(char* buf, uint16_t buf_len, int body_len);
};
