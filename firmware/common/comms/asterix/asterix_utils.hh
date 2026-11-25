#pragma once

#include <cstdint>

#include "aircraft_dictionary.hh"

// EUROCONTROL ASTERIX encoder: CAT021 (ADS-B target reports) and CAT023 (ground station and service status). Writes
// records into caller buffers and knows nothing about transports. Formats follow CAT021 Edition 2.6 and CAT023
// Edition 1.3; see README.md in this folder.

class ASTERIXReporter {
   public:
    static constexpr uint8_t kCategoryADSBTargetReports = 21;
    static constexpr uint8_t kCategoryGroundStationStatus = 23;
    static constexpr uint16_t kDataBlockHeaderLenBytes = 3;  // Category and 16-bit length.
    static constexpr uint16_t kCAT021RecordMaxLenBytes = 96;
    static constexpr uint16_t kCAT023RecordMaxLenBytes = 16;

    static constexpr uint32_t kTimeOfDayTicksPerSecond = 128;
    static constexpr uint32_t kMsPerDay = 86400000;

    // CAT021 service identification (I021/015) and CAT023 service ID (I023/015, 4 bits) of each receiver. TIS-B and
    // ADS-R targets, which CAT021 has no flag for, get their own services.
    static constexpr uint8_t kServiceID1090ES = 1;
    static constexpr uint8_t kServiceIDUAT = 2;
    static constexpr uint8_t kServiceID1090ESRebroadcast = 3;  // DF18 TIS-B and ADS-R.
    static constexpr uint8_t kServiceIDUATRebroadcast = 4;     // UAT TIS-B and ADS-R.
    // CAT023 service types (I023/015 STYP). CAT023 has no ADS-R type: rebroadcasts are reported as TIS-B.
    static constexpr uint8_t kServiceType1090ES = 2;
    static constexpr uint8_t kServiceTypeUAT = 3;
    static constexpr uint8_t kServiceTypeTISB1090ES = 5;
    static constexpr uint8_t kServiceTypeTISBUAT = 6;

    // I021/040 address type (ATP).
    enum AddressType : uint8_t {
        kAddressTypeICAO = 0,
        kAddressTypeDuplicate = 1,
        kAddressTypeSurfaceVehicle = 2,
        kAddressTypeAnonymous = 3
    };
    // I021/040 altitude reporting capability (ARC).
    enum AltitudeReportingCapability : uint8_t { kARC25Ft = 0, kARC100Ft = 1, kARCUnknown = 2 };
    // I021/210 link technology type (LTT).
    enum LinkTechnology : uint8_t { kLinkTechnologyUAT = 1, kLinkTechnology1090ES = 2 };
    // I023/110 service status (STAT).
    enum ServiceStatus : uint8_t { kServiceStatusDisabled = 2, kServiceStatusNormal = 4 };

    // Converts receiver time to the ASTERIX time of day (1/128 s since midnight UTC).
    struct Clock {
        // Where UTC came from, in increasing priority.
        enum Source : uint8_t { kSourceNone = 0, kSourceNetwork, kSourceGNSS };

        bool utc_valid = false;
        Source source = kSourceNone;
        // UTC milliseconds of day minus receiver milliseconds since boot. With no UTC the offset stays 0 and times
        // count from boot, modulo one day.
        int64_t utc_minus_local_ms = 0;

        /**
         * Sets the offset from a UTC time of day observed at a receiver timestamp, unless UTC from a higher priority
         * source is already set.
         * @param[in] utc_ms_of_day UTC milliseconds since midnight.
         * @param[in] local_ms Receiver milliseconds since boot at which that UTC time held.
         * @param[in] utc_source Where the UTC time came from.
         */
        void SetUTC(uint32_t utc_ms_of_day, uint32_t local_ms, Source utc_source = kSourceGNSS) {
            if (utc_source < source) {
                return;
            }
            utc_valid = true;
            source = utc_source;
            utc_minus_local_ms = static_cast<int64_t>(utc_ms_of_day) - static_cast<int64_t>(local_ms);
        }

        /**
         * @param[in] local_ms Receiver milliseconds since boot.
         * @retval Time of day in 1/128 s, 0 to 11059199.
         */
        uint32_t TimeOfDay(uint32_t local_ms) const;
    };

    static uint32_t UTCMsOfDay(uint8_t hour, uint8_t minute, uint8_t second, uint16_t millisecond) {
        return ((hour * 60u + minute) * 60u + second) * 1000u + millisecond;
    }

    /**
     * Contents of one CAT021 record. Only the items whose has_ flag is set are written; I021/010, I021/040, I021/080
     * and I021/090 are always written.
     */
    struct CAT021TargetReport {
        uint8_t sac = 0;
        uint8_t sic = 0;

        // I021/040 Target Report Descriptor.
        AddressType address_type = kAddressTypeICAO;
        AltitudeReportingCapability altitude_reporting_capability = kARCUnknown;
        bool from_field_monitor = false;  // RAB: fixed beacon.
        bool on_ground = false;           // GBS.
        bool no_go = false;               // NOGO: mirrors I023/100, set while there is no UTC.

        bool has_track_number = false;
        uint16_t track_number = 0;  // 12 bits.

        bool has_service_id = false;
        uint8_t service_id = 0;

        bool has_position = false;
        uint32_t latitude_awb = 0;  // AWB, see geo_utils.hh.
        uint32_t longitude_awb = 0;
        uint32_t position_time_of_day = 0;  // I021/073.

        bool has_velocity_time = false;
        uint32_t velocity_time_of_day = 0;  // I021/075.

        uint32_t address = 0;  // I021/080, 24 bits.

        bool has_geometric_height = false;
        int32_t geometric_height_ft = 0;

        // I021/090 Quality Indicators.
        uint8_t nac_v = 0;
        uint8_t nic = 0;
        uint8_t nic_baro = 0;
        uint8_t sil = 0;
        uint8_t sil_supplement = 0;
        uint8_t nac_p = 0;
        uint8_t sda = 0;
        uint8_t gva = 0;
        uint8_t pic = 0;  // Position Integrity Category, see NICToPIC().

        bool has_mops_version = false;
        uint8_t mops_version = 0;
        LinkTechnology link_technology = kLinkTechnology1090ES;

        bool has_squawk = false;
        uint16_t squawk = 0;  // Decimal digits as read, e.g. 7700.

        bool has_flight_level = false;
        int32_t baro_altitude_ft = 0;

        bool has_magnetic_heading = false;
        float magnetic_heading_deg = 0.0f;

        // I021/200 Target Status.
        bool has_target_status = false;
        uint8_t priority_status = 0;      // PS.
        uint8_t surveillance_status = 0;  // SS: 1 permanent alert, 2 temporary alert, 3 SPI.

        bool has_baro_vertical_rate = false;
        int32_t baro_vertical_rate_fpm = 0;
        bool has_geometric_vertical_rate = false;
        int32_t geometric_vertical_rate_fpm = 0;

        bool has_ground_vector = false;
        float ground_speed_kts = 0.0f;
        float track_angle_deg = 0.0f;

        bool has_indicated_airspeed = false;
        float indicated_airspeed_kts = 0.0f;
        bool has_true_airspeed = false;
        int32_t true_airspeed_kts = 0;

        bool has_report_time = false;
        uint32_t report_time_of_day = 0;  // I021/077.

        bool has_callsign = false;
        char callsign[9] = "        ";

        bool has_emitter_category = false;
        uint8_t emitter_category = 0;  // I021/020 ECAT, see EmitterCategoryToECAT().

        bool has_amplitude = false;
        int32_t amplitude_dbm = 0;

        // I021/008 Aircraft Operational Status, written if a bit is set.
        bool tcas_ra_active = false;
        bool single_antenna = false;
    };

    // Per-round values shared by every aircraft record.
    struct ReportContext {
        uint8_t sac = 0;
        uint8_t sic = 0;
        Clock clock;
        uint32_t timestamp_ms = 0;  // Receiver milliseconds since boot at the time of the report.
    };

    /**
     * Assigns stable 12-bit track numbers (I021/161) to aircraft UIDs. A number is freed once its aircraft has gone
     * unreported for kStaleRounds rounds.
     */
    class TrackNumbers {
       public:
        static constexpr uint16_t kNumSlots = 2 * kAircraftDictionaryMaxNumAircraft;
        static constexpr uint16_t kStaleRounds = 60;
        static_assert(kNumSlots <= 4096, "Track numbers are 12 bits.");

        // Call once at the start of each reporting round.
        void StartRound() { round_++; }

        /**
         * @param[in] uid Aircraft UID.
         * @param[out] track_number Track number for the UID.
         * @retval False if the table is full of live aircraft.
         */
        bool Get(uint32_t uid, uint16_t& track_number);

       private:
        struct Slot {
            uint32_t uid = 0;
            uint16_t last_round = 0;  // Compared modulo 2^16.
            bool used = false;
        };
        Slot slots_[kNumSlots];
        uint16_t round_ = 0;
    };

    /**
     * Fills a CAT021 report from a Mode S aircraft.
     * @retval False if the aircraft isn't reported in CAT021: no position, or no extended squitter.
     */
    static bool BuildCAT021Report(CAT021TargetReport& report, const ModeSAircraft& aircraft, const ReportContext& ctx);

    /**
     * Fills a CAT021 report from a UAT aircraft.
     * @retval False if the aircraft isn't reported in CAT021: no position.
     */
    static bool BuildCAT021Report(CAT021TargetReport& report, const UATAircraft& aircraft, const ReportContext& ctx);

    /**
     * Writes one CAT021 record (FSPEC and items, no data block header).
     * @retval Bytes written, or 0 if buf is too small.
     */
    static uint16_t WriteCAT021Record(uint8_t* buf, uint16_t buf_len, const CAT021TargetReport& report);

    /**
     * Writes a CAT023 Ground Station Status record (report type 1).
     * @param[in] time_of_day I023/070.
     * @param[in] clock_valid False sets NOGO and TSV (time source invalid).
     * @param[in] track_numbering_restarted RN bit, for the first report after boot.
     * @param[in] period_s Reporting period (GSSP), 1 to 127 s.
     * @retval Bytes written, or 0 if buf is too small.
     */
    static uint16_t WriteCAT023GroundStationStatusRecord(uint8_t* buf, uint16_t buf_len, uint8_t sac, uint8_t sic,
                                                         uint32_t time_of_day, bool clock_valid,
                                                         bool track_numbering_restarted, uint8_t period_s);

    /**
     * Writes a CAT023 Service Status record (report type 2).
     * @param[in] report_period_half_s CAT021 report period (RP) in 0.5 s, 0 for data driven.
     * @param[in] period_s Service status reporting period (SSRP), 1 to 127 s.
     * @retval Bytes written, or 0 if buf is too small.
     */
    static uint16_t WriteCAT023ServiceStatusRecord(uint8_t* buf, uint16_t buf_len, uint8_t sac, uint8_t sic,
                                                   uint8_t service_id, uint8_t service_type, uint32_t time_of_day,
                                                   uint8_t report_period_half_s, uint8_t period_s,
                                                   ServiceStatus status);

    /**
     * Writes a data block header. Append records after it, then call FinishDataBlock().
     * @retval kDataBlockHeaderLenBytes, or 0 if buf is too small.
     */
    static uint16_t BeginDataBlock(uint8_t* buf, uint16_t buf_len, uint8_t category);

    // Writes the total block length (header included) into the header at buf.
    static void FinishDataBlock(uint8_t* buf, uint16_t block_len_bytes);

    // Maps ADSBTypes::EmitterCategory to I021/020 ECAT. Returns 0 (no information) for reserved values.
    static uint8_t EmitterCategoryToECAT(ADSBTypes::EmitterCategory category);

    // Maps a NIC (version 1 and up) to the Position Integrity Category of I021/090, using the largest containment
    // radius a NIC value can mean.
    static uint8_t NICToPIC(uint8_t nic);

    // Converts a squawk read as decimal digits (e.g. 7700) to the 12-bit Mode 3/A code. False if a digit is above 7.
    static bool SquawkToMode3A(uint16_t squawk, uint16_t& mode_3a);
};
