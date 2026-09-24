#pragma once

#include <cstdio>
#include <cstring>
#include <memory_resource>
#include <unordered_map>
#include <variant>

#include "adsb_types.hh"
#include "aircraft_dictionary_config.hh"
// Can't include comms.hh or we will get a circular dependency.
#include "hal.hh"
#include "json_utils.hh"
#include "macros.hh"
#include "mode_s_packet.hh"
#include "remote_id_packet.hh"
#include "uat_packet.hh"

// #define ADSB_VERBOSE_PACKET_WARNINGS  // Uncomment to print warnings about unexpected packet formats / content.
#define FILTER_CPR_POSITIONS

/**
 * Virtual base class for aircraft.
 * This class is used to define the common interface for aircraft types that can be stored in the aircraft dictionary.
 *
 * Everything that is common to child classes should be defined here. Ideally reporters that don't need differentiation
 * between protocol types for contacts can access everything through this common base class.
 */
class Aircraft {
   public:
    static const uint8_t kAircraftTypeShiftBits = 28;
    static const uint32_t kAircraftTypeMask = 0xF0000000;  // Mask to extract the aircraft type from the UID.

    // Aircraft can have an optional address qualifier that is used to differentiate between aircraft subtypes within
    // the same aircraft type.
    static const uint32_t kAddressQualifierMask = 0x0F000000;
    static const uint8_t kAddressQualifierBitShift = 24;

    // Note: these need to match the variant indices in AircraftEntry for all valid variants.
    // Aircraft types must be < 4 bits in length (16 in decimal).
    enum AircraftType : int8_t {
        kAircraftTypeInvalid = 0xF,
        kAircraftTypeModeS = 0,
        kAircraftTypeUAT = 1,
        kAircraftTypeRemoteID = 2,  // Broadcast Remote ID (ASTM F3411) drone, received over BLE / WiFi on the ESP32.
        // TODO: Add FLARM, ADS-L, etc.
    };

    virtual ~Aircraft() = default;
    virtual void UpdateMetrics() = 0;

    /**
     * Converts an ICAO address to a unique identifier for the aircraft.
     * Note that the ICAO address is nominally 24 bits, but can be up to 28 bits including an address qualifier.
     */
    static inline uint32_t ICAOToUID(uint32_t icao_address, AircraftType type) {
        switch (type) {
            case kAircraftTypeModeS:
                return (icao_address & ~kAircraftTypeMask) | (kAircraftTypeModeS << kAircraftTypeShiftBits);
            case kAircraftTypeUAT:
                return (icao_address & ~kAircraftTypeMask) | (kAircraftTypeUAT << kAircraftTypeShiftBits);
            case kAircraftTypeRemoteID:
                return (icao_address & ~kAircraftTypeMask) | (kAircraftTypeRemoteID << kAircraftTypeShiftBits);
            default:
                // We can't print here or else we cause a circular dependency with comms.hh.
                // CONSOLE_ERROR("Aircraft::ICAOToUID", "Invalid aircraft type %d for ICAO address 0x%lx.", type,
                //               icao_address);
                return 0;  // Invalid aircraft type.
        }
    }

    /**
     * Returns a unique identifier for the aircraft that can differentiate it from other aircraft of different types
     * with the same ICAO address.
     * @retval Unique identifier for the aircraft.
     */
    virtual inline uint32_t GetUID() const = 0;

    /**
     * Determine which type of aircraft a UID belongs to.
     * @param uid Unique identifier for the aircraft.
     * @retval AircraftType enum value indicating the type of aircraft.
     */
    static inline AircraftType UIDToAircraftType(uint32_t uid) {
        return static_cast<AircraftType>((uid & kAircraftTypeMask) >> kAircraftTypeShiftBits);
    }

    /**
     * Strips off the UID bits from the UID to get the ICAO address.
     * @param uid Unique identifier for the aircraft.
     * @retval ICAO address of the aircraft.
     */
    static inline uint32_t UIDToICAOAddress(uint32_t uid) {
        return uid & ~kAircraftTypeMask;  // Clear the aircraft type bits to get the ICAO address.
    }

    // We store values that all aircraft are guaranteed to have (not protocol specific) in the base class. These values
    // are critical for aircraft tracking. Enums for AltitudeSource, SpeedSource, and VerticalRateSource match
    // encodings in Mode S and UAT standards (e.g. you can cast a binary value from a a packet directly to these enums).
    // Values from other protocols will need to be translated.

    uint32_t last_message_timestamp_ms = 0;

    float latitude_deg = 0.0f;
    float longitude_deg = 0.0f;

    int32_t baro_altitude_ft = 0;
    int32_t gnss_altitude_ft = 0;

    float direction_deg = 0;
    int32_t speed_kts = 0;  // Provide full 32 bit depth so we can square it without issues.

    int32_t baro_vertical_rate_fpm = 0;
    int32_t gnss_vertical_rate_fpm = 0;
};

/**
 * Mode S aircraft class.
 */
class ModeSAircraft : public Aircraft {
   public:
    // These variables define filter bounds for time between CPR packets. If the time between packets is greater than
    // the time delta limit, the old CPR packet is discarded and the CPR packet pair is not used for position decoding.
    static constexpr uint32_t kDefaultCPRIntervalMs = 10e3;  // CPR interval when starting from scratch or stale track.
    static constexpr uint32_t kRefCPRIntervalMs = 19e3;      // Reference interval for rejecting CPR packet pairs.
    static constexpr uint32_t kMaxCPRIntervalMs = 30e3;  // Never accept CPR packet pairs more than 30 seconds apart.
    static constexpr uint32_t kMaxTrackUpdateIntervalMs = 20e3;  // Tracks older than this are considered stale.

    static constexpr uint16_t kCallSignMaxNumChars = 8;
    static constexpr uint16_t kCallSignMinNumChars = 3;  // Callsigns must be at this long to be valid.

    enum BitFlag : uint32_t {
        kBitFlagIsAirborne = 0,  // Received messages or flags indicating the aircraft is airborne.
        kBitFlagBaroAltitudeValid = 1,
        kBitFlagGNSSAltitudeValid = 2,
        kBitFlagPositionValid = 3,
        kBitFlagDirectionValid = 4,
        kBitFlagHorizontalSpeedValid = 5,
        kBitFlagBaroVerticalRateValid = 6,
        kBitFlagGNSSVerticalRateValid = 7,
        kBitFlagIsMilitary = 8,              // Received at least one military ES message from the aircraft.
        kBitFlagIsClassB2GroundVehicle = 9,  // Is a class B2 ground vehicle transmitting at <70W.
        kBitFlagHas1090ESIn = 10,            // Aircraft is equipped with 1090MHz Extended Squitter receive capability.
        kBitFlagHasUATIn = 11,               // Aircraft can receive UAT.
        kBitFlagTCASOperational = 12,        // TCAS system on aircraft is functional.
        kBitFlagSingleAntenna = 13,       // Indicates that the aircraft is using a single antenna. Transmissions may be
                                          // intermittent.
        kBitFlagDirectionIsHeading = 14,  // Direction is aircraft heading and not track angle.
        kBitFlagHeadingUsesMagneticNorth = 15,  // Heading in surface and airborne position messages is magnetic north,
                                                // not true north.
        kBitFlagIdent = 16,                     // IDENT switch is currently active.
        kBitFlagAlert = 17,                     // Aircraft is indicating an alert.
        kBitFlagTCASRA = 18,                    // Indicates a TCAS resolution advisory is active.
        kBitFlagIsNonTransponder = 19,          // Aircraft is being rebroadcast (this is not a primary source).
        kBitFlagReserved0 = 20,
        kBitFlagReserved1 = 21,
        kBitFlagReserved2 = 22,
        // Flags after kBitFlagUpdatedBaroAltitude are cleared at the end of every reporting interval.
        kBitFlagUpdatedBaroAltitude = 23,
        kBitFlagUpdatedGNSSAltitude = 24,
        kBitFlagUpdatedPosition = 25,
        kBitFlagUpdatedDirection = 26,
        kBitFlagUpdatedHorizontalSpeed = 27,
        kBitFlagUpdatedBaroVerticalRate = 28,
        kBitFlagUpdatedGNSSVerticalRate = 29,
        kBitFlagReserved3 = 30,
        kBitFlagReserved4 = 31,
        kBitFlagNumFlagBits
    };

    static const uint8_t kBitFlagEndOfPersistentFlags = kBitFlagUpdatedBaroAltitude;

    struct Metrics {
        // We can only confirm that valid frames came from this aircraft. Not sure who invalid frames are from.
        uint16_t valid_squitter_frames = 0;
        uint16_t valid_extended_squitter_frames = 0;
    };

    ModeSAircraft(uint32_t icao_address_in);
    ModeSAircraft();

    /**
     * Override Functions
     */

    /**
     * Returns a unique identifier for the aircraft that can differentiate it from other aircraft of different types
     * with the same ICAO address.
     * @retval Unique identifier for the aircraft.
     */
    inline uint32_t GetUID() const { return ICAOToUID(icao_address, kAircraftTypeModeS); }

    /**
     * Standard Functions
     */

    /**
     * Checks to see if the aircraft position can be decoded. Requires that both an odd and an even packet have been
     * received, and they aren't separated by too large of a time interval.
     * @retval True if the aircraft position can be decoded, false otherwise.
     */
    bool CanDecodePosition();

    /**
     * Clears the CPR packet cache. Used when too much time has elapsed since the last CPR packet was received, to avoid
     * decoding CPR location with invalid packet pairings.
     */
    inline void ClearCPRPackets() {
        // Clearing received timestamps causes the packet pair to be rejected during the decoding stage, so it's as good
        // as wiping all of the received packet contents.
        last_odd_packet_.received_timestamp_ms = 0;
        last_even_packet_.received_timestamp_ms = 0;
    }

    /**
     * Wrapper around DecodePosition for airborne positions (reference latitude not needed).
     * @param[in] filter_cpr_position True if the CPR position filter should be run (defaults to true).
     * @retval True if position was decoded successfully, false otherwise.
     */
    inline bool DecodeAirbornePosition(bool filter_cpr_position = true) {
        return DecodePosition(true, 0, 0, filter_cpr_position);
    }

    /**
     * Wrapper around DecodePosition for surface positions.
     * @param[in] ref_lat_awb32 Reference latitude in 32-bit angular weighted binary for surface position decoding.
     * @param[in] ref_lon_awb32 Reference longitude in 32-bit angular weighted binary for surface position decoding.
     * @param[in] filter_cpr_position True if the CPR position filter should be run (defaults to true).
     * @retval True if position was decoded successfully, false otherwise.
     */
    inline bool DecodeSurfacePosition(uint32_t ref_lat_awb32, uint32_t ref_lon_awb32, bool filter_cpr_position = true) {
        return DecodePosition(false, ref_lat_awb32, ref_lon_awb32, filter_cpr_position);
    }

    /**
     * Returns the maximum time delta between CPR packets that will be accepted for decoding.
     * @retval Maximum allowed time delta between CPR packets.
     */
    uint32_t GetMaxAllowedCPRIntervalMs() const {
        if (speed_source == ADSBTypes::kSpeedSourceNotSet || speed_source == ADSBTypes::kSpeedSourceNotAvailable ||
            get_time_since_boot_ms() - last_track_update_timestamp_ms > kMaxTrackUpdateIntervalMs) {
            return kDefaultCPRIntervalMs;
        }
        // Scale time delta threshold based on the velocity of the aircraft relative to 500kts, but clamp the result to
        // the max time delta thresholds.
        // Protect against divide by 0.
        return speed_kts == 0 ? kMaxCPRIntervalMs : MIN(kRefCPRIntervalMs * 500 / speed_kts, kMaxCPRIntervalMs);
    }

    /**
     * Checks whether a flag bit is set.
     * @param[in] bit Position of bit to check.
     * @retval True if bit has been set, false if bit has been cleared.
     */
    inline bool HasBitFlag(BitFlag bit) const { return flags & (0b1 << bit) ? true : false; }

    /**
     * Indicate that a frame has been received by incrementing the corresponding frame counter.
     * @param[in] is_extended_squitter Set to true if the frame received was a Mode S frame.
     */
    inline void IncrementNumFramesReceived(bool is_extended_squitter = false) {
        is_extended_squitter ? metrics_counter_.valid_extended_squitter_frames++
                             : metrics_counter_.valid_squitter_frames++;
    }

    /**
     * Returns whether a NIC supplement bit has been written to. Used to decide when to use NIC supplement bit values to
     * determine NIC value based on received TypeCodes.
     * @param[in] bit NIC supplement bit to check.
     * @retval True if bit has been written to, false otherwise.
     */
    inline bool NICBitIsValid(ADSBTypes::NICBit bit) { return nic_bits_valid & (0b1 << bit); }

    /**
     * Resets just the flag bits that show that something updated within the last reporting interval.
     */
    inline void ResetUpdatedBitFlags() { flags &= ~(~0b0 << kBitFlagEndOfPersistentFlags); }

    /**
     * Set an aircraft's position in Compact Position Reporting (CPR) format. Takes either an even or odd set of lat/lon
     * coordinates and uses them to set the aircraft's position.
     * @param[in] n_lat_cpr 17-bit latitude count.
     * @param[in] n_lon_cpr 17-bit longitude count.
     * @param[in] odd Boolean indicating that the position update is relative to an odd grid reference (if true) or an
     * even grid reference.
     * @param[in] received_timestamp_ms Timestamp in milliseconds when the position packet was received. Should be
     * derived from the MLAT counter, but the precision can be pretty lax since it is only used when deciding whether to
     * reject CPR packet pairings due to too much time elapsed between even and odd packets.
     * @retval True if coordinates were parsed successfully, false if not. NOTE: Invalid positions can still be
     * considered a successful parse.
     */
    bool SetCPRLatLon(uint32_t n_lat_cpr, uint32_t n_lon_cpr, bool odd, uint32_t received_timestamp_ms);

    /**
     * Roll the metrics counter over to the public metrics field.
     */
    inline void UpdateMetrics() {
        metrics = metrics_counter_;
        metrics_counter_ = Metrics();
    }

    /**
     * Set or clear a bit on the Aircraft.
     */
    inline void WriteBitFlag(BitFlag bit, bool value) {
        if (bit == kBitFlagIsAirborne && HasBitFlag(kBitFlagIsAirborne) != value) {
            // Reset CPR packets when the airborne state changes, since position decoding may be invalid now.
            ClearCPRPackets();
        }
        value ? flags |= (0b1 << bit) : flags &= ~(0b1 << bit);
    }

    /**
     * Write a value for a NIC supplement bit. Used to piece together a NIC from separate messages, so that the NIC can
     * be determined based on a received TypeCode.
     * @param[in] bit NIC supplement bit to write.
     * @param[in] value Value to write to the bit.
     */
    inline void WriteNICBit(ADSBTypes::NICBit bit, bool value) {
        value ? nic_bits |= (0b1 << bit) : nic_bits &= ~(0b1 << bit);
        // FIXME: Permanently setting NIC bits valid like this can cause invalid navigation integrity values to be read
        // if a stale NIC supplement bit is being used. Hopefully this isn't a big problem if NIC values don't change
        // frequently.
        nic_bits_valid |= (0b1 << bit);
    }

    uint32_t flags = 0b0;

    int16_t last_message_signal_strength_dbm = 0;  // Voltage of RSSI signal during message receipt.
    int16_t last_message_signal_quality_db = 0;    // Ratio of RSSI to noise floor during message receipt.
    uint32_t last_track_update_timestamp_ms = 0;   // Timestamp of the last time that the position was updated.
    Metrics metrics;

    uint16_t transponder_capability = 0;
    uint32_t icao_address = 0;
    char callsign[kCallSignMaxNumChars + 1] = "?";  // put extra EOS character at end
    uint16_t squawk = ADSBTypes::kSquawkCodeNotYetReceived;
    ADSBTypes::EmitterCategory emitter_category = ADSBTypes::kEmitterCategoryNoCategoryInfo;
    uint8_t emitter_category_raw = 0;  // Non-enum category in case we want the value without a many to one mapping.

    ADSBTypes::SpeedSource speed_source = ADSBTypes::kSpeedSourceNotSet;  // Most recent reported speed.
    // Used to keep track of the most recently reported altitude, for use in adjusting the complementary altitude with
    // an offset.
    ADSBTypes::AltitudeSource altitude_source = ADSBTypes::kAltitudeSourceNotSet;

    // Aircraft Operation Status Message
    // Navigation Integrity Category (NIC)
    uint8_t nic_bits_valid = 0b000;  // MSb to LSb: nic_c_valid nic_b_valid nic_a_valid.
    uint8_t nic_bits = 0b000;        // MSb to LSb: nic_c nic_b nic_a.
    ADSBTypes::NICRadiusOfContainment navigation_integrity_category = ADSBTypes::kROCUnknown;  // 4 bits.
    ADSBTypes::NICBarometricAltitudeIntegrity navigation_integrity_category_baro =
        ADSBTypes::kBAIGillhamInputNotCrossChecked;  // 1 bit. Default to worst case.
    // Navigation Accuracy Category (NAC)
    ADSBTypes::NACHorizontalVelocityError navigation_accuracy_category_velocity =
        ADSBTypes::kHVEUnknownOrGreaterThanOrEqualTo10MetersPerSecond;  // 3 bits.
    ADSBTypes::NACEstimatedPositionUncertainty navigation_accuracy_category_position =
        ADSBTypes::kEPUUnknownOrGreaterThanOrEqualTo10NauticalMiles;  // 4 bits.
    // Geometric Vertical Accuracy (GVA)
    ADSBTypes::GVA geometric_vertical_accuracy = ADSBTypes::kGVAUnknownOrGreaterThan150Meters;  // 2 bits.
    ADSBTypes::SILProbabilityOfExceedingNICRadiusOfContainmnent surveillance_integrity_level =
        ADSBTypes::kPOERCUnknownOrGreaterThan1em3PerFlightHour;  // 3 bits.
    // System Design Assurance
    ADSBTypes::SystemDesignAssurance system_design_assurance =
        ADSBTypes::kSDASupportedFailureUnknownOrNoSafetyEffect;  // 2 bits.
    // GPS Antenna Offset
    int8_t gnss_antenna_offset_right_of_reference_point_m =
        INT8_MAX;  // Defaults to INT8_MAX to indicate it hasn't been read yet.
    // Aircraft dimensions (on the ground).
    uint16_t length_m = 0;
    uint16_t width_m = 0;

    int8_t adsb_version = 0;

    /**
     * GENERIC COMMENT FOR ALL MESSAGE INGESTION HELPERS
     * Ingests a <Message Type> ADS-B message. Called by IngestModeSADSBPacket, which makes sure that the packet
     * is valid and has the correct Downlink Format.
     * @param[in] packet ModeSADSBPacket to ingest.
     * @param[in] ref_lat_awb32 Reference latitude in 32-bit angular weighted binary for surface position decoding.
     * @param[in] ref_lon_awb32 Reference longitude in 32-bit angular weighted binary for surface position decoding.
     * @param[in] filter_cpr_position True if the CPR position filter should be run.
     * @retval True if message was ingested successfully, false otherwise.
     */

    bool ApplyAircraftIDMessage(const ModeSADSBPacket& packet);
    // ref_position_valid gates only the CPR position decode: surface CPR resolves its 90x90 degree ambiguity by
    // picking the solution nearest the reference position, so decoding against an unset reference (0, 0) would
    // silently produce a wrong position. With ref_position_valid false the rest of the message (air/ground state,
    // movement, heading, CPR frame caching) still applies and the message is treated as successfully ingested.
    bool ApplySurfacePositionMessage(const ModeSADSBPacket& packet, uint32_t ref_lat_awb32, uint32_t ref_lon_awb32,
                                     bool filter_cpr_position = true, bool ref_position_valid = true);
    bool ApplyAirbornePositionMessage(const ModeSADSBPacket& packet, bool filter_cpr_position = true);
    bool ApplyAirborneVelocitiesMessage(const ModeSADSBPacket& packet);
    bool ApplyAircraftStatusMessage(const ModeSADSBPacket& packet);
    bool ApplyTargetStateAndStatusInfoMessage(const ModeSADSBPacket& packet);
    bool ApplyAircraftOperationStatusMessage(const ModeSADSBPacket& packet);

   private:
    struct CPRPacket {
        // SetCPRLatLon values.
        uint32_t received_timestamp_ms = 0;  // [ms] time since boot when packet was recorded
        uint32_t n_lat = 0;                  // 17-bit latitude count
        uint32_t n_lon = 0;                  // 17-bit longitude count
    };

    /**
     * Decodes the aircraft position using last_odd_packet_ and last_even_packet_.
     * @param[in] is_airborne True if decoding airborne position, false for surface position.
     * @param[in] ref_lat_awb32 Reference latitude in 32-bit angular weighted binary for surface position decoding
     * (defaults to 0, unused for airborne positions).
     * @param[in] ref_lon_awb32 Reference longitude in 32-bit angular weighted binary for surface position decoding.
     * @param[in] filter_cpr_position True if the CPR position filter should be run (defaults to true).
     * @retval True if position was decoded successfully, false otherwise.
     */
    bool DecodePosition(bool is_airborne = true, uint32_t ref_lat_awb32 = 0, uint32_t ref_lon_awb32 = 0,
                        bool filter_cpr_position = true);

    CPRPacket last_odd_packet_;
    CPRPacket last_even_packet_;

#ifdef FILTER_CPR_POSITIONS
    // Position in Angular Weighted Binary. This gets set to a candidate position which may not match the actual
    // displayed latitude_deg and logitude_deg. Format is in AWB to enable fast fixed point operations for screening
    // candidate positions.
    uint32_t last_filter_received_timestamp_ms_ =
        0;  // received_timestamp_ms for the last packet that was fed to the filter.
    uint32_t lat_awb32_ = 0;
    uint32_t lon_awb32_ = 0;
#endif

    Metrics metrics_counter_;
};

/**
 * UAT aircraft class.
 */
class UATAircraft : public Aircraft {
   public:
    static constexpr uint16_t kCallSignMaxNumChars = 8;
    static constexpr uint16_t kCallSignMinNumChars = 3;  // Callsigns must be at this long to be valid.
    static constexpr uint16_t kSquawkNumDigits = 4;

    enum AddressQualifier : int8_t {
        kAddressQualifierInvalid = -1,  // Address qualifier has not been set.
        kADSBTargetWithICAO24BitAddress = 0,
        kADSBTargetWithSelfAssignedTemporaryAddress = 1,
        kTISBTargetWithICAO24BitAddress = 2,
        kTISBTargetWithTrackFileIdentifier = 3,
        kSurfaceVehicle = 4,
        kFixedADSBBeacon = 5
    };

    enum BitFlag : uint32_t {
        kBitFlagIsAirborne = 0,  // Received messages or flags indicating the aircraft is airborne.
        kBitFlagBaroAltitudeValid = 1,
        kBitFlagGNSSAltitudeValid = 2,
        kBitFlagPositionValid = 3,
        kBitFlagDirectionValid = 4,
        kBitFlagHorizontalSpeedValid = 5,
        kBitFlagBaroVerticalRateValid = 6,
        kBitFlagGNSSVerticalRateValid = 7,
        kBitFlagHasCDTI = 8,              // Aircraft is equipped with a Cockpit Display of Traffic Information (CDTI).
        kBitFlagTCASOperational = 9,      // TCAS system on aircraft is functional.
        kBitFlagDirectionIsHeading = 10,  // Direction is aircraft heading and not track angle.
        kBitFlagHeadingUsesMagneticNorth = 11,  // Heading in surface and airborne position messages is magnetic north,
                                                // not true north.
        kBitFlagIdent = 12,                     // IDENT switch is currently active.
        kBitFlagReserved0 = 13,
        kBitFlagReserved1 = 14,
        kBitFlagTCASRA = 15,                // Indicates a TCAS resolution advisory is active.
        kBitFlagReceivingATCServices = 16,  // Aircraft is receiving ATC services.
        kBitFlagReserved2 = 17,
        kBitFlagReserved3 = 18,
        kBitFlagReserved4 = 19,
        kBitFlagReserved5 = 20,
        kBitFlagReserved6 = 21,
        kBitFlagReserved7 = 22,
        // Flags after kBitFlagUpdatedBaroAltitude are cleared at the end of every reporting interval.
        kBitFlagUpdatedBaroAltitude = 23,
        kBitFlagUpdatedGNSSAltitude = 24,
        kBitFlagUpdatedPosition = 25,
        kBitFlagUpdatedDirection = 26,
        kBitFlagUpdatedHorizontalSpeed = 27,
        kBitFlagUpdatedBaroVerticalRate = 28,
        kBitFlagUpdatedGNSSVerticalRate = 29,
        kBitFlagReserved8 = 30,
        kBitFlagReserved9 = 31,
        kBitFlagNumFlagBits
    };

    static const uint8_t kBitFlagEndOfPersistentFlags = kBitFlagUpdatedBaroAltitude;

    enum EmergencyPriorityStatus : uint8_t {
        kEmergencyPriorityStatusNone = 0,
        kEmergencyStatusGeneralEmergency = 1,
        kEmergencyStatusLifeguardMedicalEmergency = 2,
        kEmergencyStatusMinimumFuel = 3,
        kEmergencyStatusNoCommunications = 4,
        kEmergencyStatusUnlawfulInterference = 5,
        kEmergencyStatusDownedAircraft = 6,
        kEmergencyStatusReserved = 7
    };

    struct Metrics {
        uint16_t valid_frames = 0;  // Number of valid UAT frames received.
    };

    UATAircraft(uint32_t icao_address_in);
    UATAircraft() {};
    ~UATAircraft();

    /**
     * Override Functions
     */

    /**
     * Returns a unique identifier for the aircraft that can differentiate it from other aircraft of different types
     * with the same ICAO address.
     * @retval Unique identifier for the aircraft.
     */
    inline uint32_t GetUID() const { return ICAOToUID(icao_address, kAircraftTypeUAT); }

    /**
     * Extracts the address qualifier from the ICAO address.
     */
    inline AddressQualifier GetAddressQualifier() const {
        return static_cast<AddressQualifier>((icao_address & kAddressQualifierMask) >> kAddressQualifierBitShift);
    }

    /**
     * Standard Functions
     */

    /**
     * Checks whether a flag bit is set.
     * @param[in] bit Position of bit to check.
     * @retval True if bit has been set, false if bit has been cleared.
     */
    inline bool HasBitFlag(BitFlag bit) const { return flags & (0b1 << bit) ? true : false; }

    /**
     * Increments the number of valid frames received.
     */
    inline void IncrementNumFramesReceived() { metrics_counter_.valid_frames++; }

    /**
     * Resets just the flag bits that show that something updated within the last reporting interval.
     */
    inline void ResetUpdatedBitFlags() { flags &= ~(~0b0 << kBitFlagEndOfPersistentFlags); }

    /**
     * Roll the metrics counter over to the public metrics field.
     */
    inline void UpdateMetrics() {
        metrics = metrics_counter_;
        metrics_counter_ = Metrics();
    }

    /**
     * Set or clear a bit on the Aircraft.
     */
    inline void WriteBitFlag(BitFlag bit, bool value) { value ? flags |= (0b1 << bit) : flags &= ~(0b1 << bit); }

    /**
     * UAT state vector contains latitude and longitude as 24-bit Angular Weighted Binary (AWB). Convert to 32-bit
     * angular weighted binary.
     * @param[in] uat_lat_awb 24-bit UAT latitude in AWB format.
     * @retval 32-bit latitude in AWB format.
     */
    inline uint32_t UATAWBToAWB32(uint32_t uat_awb) {
        return (uat_awb << 8);  // Shift left to convert from 24-bit to 32-bit AWB.
    }

    /**
     * Filters the position in the state vector using a position filter to reject outlier positions.
     * @param[in] state_vector State vector to filter position for.
     * @param[in] filter_position True if the position filter should be applied, false to skip filtering.
     * @retval True if the position in the state vector passed the filter, false if it needs to be rejected.
     */
    bool DecodePosition(const DecodedUATADSBPacket::UATStateVector& state_vector, bool filter_position = true);

    // Application functions use pointers to portions of the decoded UAT ADS-B message payload that can be directly
    // interpreted.
    bool ApplyUATADSBStateVector(const DecodedUATADSBPacket::UATStateVector& state_vector);
    bool ApplyUATADSBModeStatus(const DecodedUATADSBPacket::UATModeStatus& mode_status);
    bool ApplyUATADSBTargetState(const DecodedUATADSBPacket::UATTargetState& target_state);
    bool ApplyUATADSBTrajectoryChange(const DecodedUATADSBPacket::UATTrajectoryChange& trajectory_change);
    bool ApplyUATADSBAuxiliaryStateVector(const DecodedUATADSBPacket::UATStateVector& state_vector,
                                          const DecodedUATADSBPacket::UATAuxiliaryStateVector& auxiliary_state_vector);

    uint32_t flags = 0b0;

    int16_t last_message_signal_strength_dbm = 0;  // Voltage of RSSI signal during last message reception.
    int16_t last_message_signal_quality_bits = 0;  // Number of bits corrected with FEC during last message reception.
    uint32_t last_track_update_timestamp_ms = 0;   // Timestamp of the last time that the position was updated.
    Metrics metrics;

    uint16_t transponder_capability = 0;
    uint32_t icao_address = 0;
    char callsign[ModeSAircraft::kCallSignMaxNumChars + 1] = "?";  // put extra EOS character at end
    uint16_t squawk = ADSBTypes::kSquawkCodeNotYetReceived;
    ADSBTypes::EmitterCategory emitter_category = ADSBTypes::kEmitterCategoryNoCategoryInfo;
    uint8_t emitter_category_raw = 0;  // Non-enum category.
    EmergencyPriorityStatus emergency_priority_status =
        kEmergencyPriorityStatusNone;  // Emergency priority status of the aircraft.

    ADSBTypes::NICRadiusOfContainment navigation_integrity_category = ADSBTypes::kROCUnknown;  // 4 bits.
    ADSBTypes::NICBarometricAltitudeIntegrity navigation_integrity_category_baro =
        ADSBTypes::kBAIGillhamInputNotCrossChecked;  // 1 bit. Default to worst case.
    // Navigation Accuracy Category (NAC)
    ADSBTypes::NACHorizontalVelocityError navigation_accuracy_category_velocity =
        ADSBTypes::kHVEUnknownOrGreaterThanOrEqualTo10MetersPerSecond;  // 3 bits.
    ADSBTypes::NACEstimatedPositionUncertainty navigation_accuracy_category_position =
        ADSBTypes::kEPUUnknownOrGreaterThanOrEqualTo10NauticalMiles;  // 4 bits.
    // Geometric Vertical Accuracy (GVA)
    ADSBTypes::GVA geometric_vertical_accuracy = ADSBTypes::kGVAUnknownOrGreaterThan150Meters;  // 2 bits.
    ADSBTypes::SILProbabilityOfExceedingNICRadiusOfContainmnent surveillance_integrity_level =
        ADSBTypes::kPOERCUnknownOrGreaterThan1em3PerFlightHour;  // 3 bits.
    // System Design Assurance
    ADSBTypes::SystemDesignAssurance system_design_assurance =
        ADSBTypes::kSDASupportedFailureUnknownOrNoSafetyEffect;  // 2 bits.

    uint8_t raw_capability_codes = 0;
    uint8_t raw_operational_modes = 0;

    // 6 LSBs of Message Start Opportunity used for last transmission.
    uint8_t transmit_mso = 0;

    // Aircraft dimensions (on the ground).
    uint16_t length_m = 0;
    uint16_t width_m = 0;

    // Aircraft GNSS sensor offset (transmitted on the ground).
    int16_t gnss_antenna_offset_right_of_reference_point_m = 0;
    int16_t gnss_antenna_offset_forward_of_reference_point_m = 0;

    int8_t tis_b_site_id = 0;
    bool utc_coupled = false;
    int8_t uat_version = -1;

   private:
    Metrics metrics_counter_;

#ifdef FILTER_CPR_POSITIONS
    // Position in Angular Weighted Binary. This gets set to a candidate position which may not match the actual
    // displayed latitude_deg and logitude_deg. Format is in AWB to enable fast fixed point operations for screening
    // candidate positions.
    uint32_t last_filter_received_timestamp_ms_ =
        0;  // received_timestamp_ms for the last packet that was fed to the filter.
    uint32_t lat_awb32_ = 0;
    uint32_t lon_awb32_ = 0;
#endif
};

/**
 * Broadcast Remote ID (ASTM F3411 / ASD-STAN prEN 4709-002) drone.
 *
 * Populated on the ESP32 (and mirrored on the RP2040) from Open Drone ID messages received over Bluetooth LE and WiFi.
 * The protocol-agnostic position/altitude/speed/track live in the Aircraft base class (converted to the ADSBee-standard
 * feet / knots / degrees); the fields here hold the Remote-ID-specific identity and operator information.
 *
 * UID / dictionary key: unlike Mode S and UAT (which key on a real ICAO-style address contained in every message), a
 * Remote ID transmitter has no address in most message types (a Location message on its own carries no serial number).
 * We therefore key on a 24-bit CRC of the transmitter's MAC / BLE address (see MACToAddress). This is stable across all
 * message types from one transmitter, so Location messages that arrive before the Basic ID still land on the right
 * entry with no provisional-UID migration dance. The trade-off is that a drone which rotates its (randomized) BLE MAC
 * mid-flight starts a new track; tracks expire on the normal prune interval, so this is acceptable. The human-readable
 * UAS serial / registration is stored in uas_id once a Basic ID message arrives.
 * TODO: optionally coalesce tracks that share a uas_id across MAC rotations.
 */
class RemoteIDAircraft : public Aircraft {
   public:
    static constexpr uint16_t kIDLenChars = 20;          // ODID_ID_SIZE. UAS ID / Operator ID length (no null).
    static constexpr uint16_t kDescriptionLenChars = 23;  // ODID_STR_SIZE. Self-ID free-text description length.

    enum BitFlag : uint32_t {
        kBitFlagIsAirborne = 0,          // Operational status indicates the UA is airborne.
        kBitFlagPositionValid = 1,       // A valid UA Location has been received.
        kBitFlagDirectionValid = 2,      // A valid track direction has been received.
        kBitFlagHorizontalSpeedValid = 3,
        kBitFlagGNSSAltitudeValid = 4,   // Geometric (WGS84) UA altitude is valid.
        kBitFlagHeightValid = 5,         // Height above takeoff / ground is valid.
        kBitFlagOperatorPositionValid = 6,  // A valid operator (System message) location has been received.
        kBitFlagBasicIDValid = 7,        // A Basic ID message has populated uas_id / uas_id_type / ua_type.
        kBitFlagOperatorIDValid = 8,     // An Operator ID message has populated operator_id.
        kBitFlagSelfIDValid = 9,         // A Self ID message has populated self_id_description.
        kBitFlagVerticalRateValid = 10,  // A valid vertical speed has been received (Location may omit it).
        kBitFlagNumFlagBits
    };

    /**
     * Computes the 24-bit dictionary address (low 24 bits of the UID) from a 6-byte transmitter MAC / BLE address.
     * Implemented in aircraft_dictionary.cpp (uses the shared crc24 utility).
     */
    static uint32_t MACToAddress(const uint8_t mac[6]);

    RemoteIDAircraft(uint32_t address_in) : address(address_in) {}
    RemoteIDAircraft() {}

    /**
     * Override Functions
     */
    inline uint32_t GetUID() const { return ICAOToUID(address, kAircraftTypeRemoteID); }

    inline void UpdateMetrics() {
        metrics = metrics_counter_;
        metrics_counter_ = Metrics();
    }

    /**
     * Standard Functions
     */
    inline bool HasBitFlag(BitFlag bit) const { return flags & (0b1 << bit) ? true : false; }
    inline void WriteBitFlag(BitFlag bit, bool value) { value ? flags |= (0b1 << bit) : flags &= ~(0b1 << bit); }
    inline void IncrementNumFramesReceived() { metrics_counter_.valid_frames++; }

    struct Metrics {
        uint16_t valid_frames = 0;  // Number of valid Remote ID messages received this reporting interval.
    };

    uint32_t flags = 0b0;
    uint32_t address = 0;  // 24-bit MAC-derived dictionary address (see MACToAddress). Top nibble stays 0.

    char uas_id[kIDLenChars + 1] = "";           // UAS ID: serial number (ANSI/CTA-2063-A), CAA registration, etc.
    char operator_id[kIDLenChars + 1] = "";      // Operator ID (e.g. FAA / CAA operator registration).

    uint8_t uas_id_type = 0;  // ODID_idtype_t: 1=serial, 2=CAA registration, 3=UTM (UUID), 4=specific session ID.
    uint8_t ua_type = 0;      // ODID_uatype_t: aircraft category (aeroplane, multirotor, helicopter, ...).
    uint8_t operational_status = 0;  // ODID_status_t: 0=undeclared, 1=ground, 2=airborne, 3=emergency, 4=remote-id-fail.
    uint8_t height_type = 0;         // ODID_Height_reference_t: 0=above takeoff, 1=above ground.

    int8_t last_message_signal_strength_dbm = 0;  // RSSI of the most recent advertisement, in dBm.
    RawRemoteIDPacket::Transport transport = RawRemoteIDPacket::kTransportInvalid;  // Last transport the UA was heard on.
    uint8_t source_mac[6] = {0};  // Transmitter MAC / BLE address the UID was derived from.

    int32_t height_above_takeoff_ft = 0;  // UA height above takeoff or ground per height_type. Valid iff flag set.

    float operator_latitude_deg = 0.0f;
    float operator_longitude_deg = 0.0f;
    int32_t operator_altitude_geo_ft = 0;  // Operator (System message) geometric altitude, WGS84.

    Metrics metrics;

   private:
    Metrics metrics_counter_;
};

class AircraftDictionary {
   public:
    static constexpr uint16_t kMaxNumAircraft = kAircraftDictionaryMaxNumAircraft;
    static constexpr uint16_t kMaxNumSources = 4;  // One per 1090 demodulator state machine.

#ifdef FILTER_CPR_POSITIONS
    // Table-based inverse haversine isn't super accurate at small distances, and aircraft velocity may be super
    // accurate, so we always allow a position jump of less than or equal to this distance.
    static constexpr uint32_t kPositionFilterDeadbandMeters = 20e3;
#endif

    typedef std::variant<ModeSAircraft, UATAircraft, RemoteIDAircraft> AircraftEntry;

    // Ensure that valid variant indices matche the AircraftType enum value so that we can use them interchangeably.
    static_assert(
        std::is_same_v<std::variant_alternative_t<Aircraft::kAircraftTypeModeS, AircraftEntry>, ModeSAircraft>,
        "ModeSAircraft variant index must match Aircraft::kAircraftTypeModeS");
    static_assert(std::is_same_v<std::variant_alternative_t<Aircraft::kAircraftTypeUAT, AircraftEntry>, UATAircraft>,
                  "UATAircraft variant index must match Aircraft::kAircraftTypeUAT");
    static_assert(
        std::is_same_v<std::variant_alternative_t<Aircraft::kAircraftTypeRemoteID, AircraftEntry>, RemoteIDAircraft>,
        "RemoteIDAircraft variant index must match Aircraft::kAircraftTypeRemoteID");
    // Keep RemoteIDAircraft from growing the variant (and therefore the fixed node pool on both the RP2040 and ESP32,
    // which costs kMaxNumAircraft * delta bytes on each). If this fails, trim RemoteIDAircraft's fields.
    static_assert(sizeof(RemoteIDAircraft) <= sizeof(ModeSAircraft),
                  "RemoteIDAircraft must not exceed ModeSAircraft or it grows the AircraftEntry pool on both MCUs.");

    struct AircraftDictionaryConfig {
        uint32_t aircraft_prune_interval_ms = 60e3;
#ifdef FILTER_CPR_POSITIONS
        // CPR position filter checks each new aircraft location against the previous location and requires two
        // consecutive packets within a geographic radius to confirm large jumps in aircraft location. This reduces the
        // likelihood that an aircraft might "jump" but increases CPU load.
        bool enable_cpr_position_filter = true;
#endif
    };

    struct Metrics {
        static const uint16_t kMetricsJSONMaxLen = 1000;  // Includes null terminator.

        uint32_t raw_squitter_frames = 0;
        uint32_t valid_squitter_frames = 0;
        uint32_t raw_extended_squitter_frames = 0;
        uint32_t valid_extended_squitter_frames = 0;
        uint32_t demods_1090 = 0;

        uint16_t raw_squitter_frames_by_source[kMaxNumSources] = {0};
        uint16_t valid_squitter_frames_by_source[kMaxNumSources] = {0};
        uint16_t raw_extended_squitter_frames_by_source[kMaxNumSources] = {0};
        uint16_t valid_extended_squitter_frames_by_source[kMaxNumSources] = {0};
        uint16_t demods_1090_by_source[kMaxNumSources] = {0};

        uint16_t raw_uat_adsb_frames = 0;
        uint16_t valid_uat_adsb_frames = 0;

        uint16_t raw_uat_uplink_frames = 0;
        uint16_t valid_uat_uplink_frames = 0;

        uint16_t num_mode_s_aircraft = 0;
        uint16_t num_uat_aircraft = 0;
        uint16_t num_remote_id_aircraft = 0;

        uint16_t raw_remote_id_frames = 0;    // Remote ID advertisements received (pre-decode).
        uint16_t valid_remote_id_frames = 0;  // Remote ID advertisements that decoded successfully.

        /**
         * Formats the metrics dictionary into a JSON packet with the following structure.
         * {
         *      "raw_squitter_frames": 10,
         *      "valid_squitter_frames": 7,
         *      "raw_extended_squitter_frames": 30,
         *      "valid_extended_squitter_frames": 16,
         *      "demods_1090": 50,
         *      "raw_squitter_frames_by_source": [3, 3, 4],
         *      "valid_squitter_frames_by_source": [2, 2, 3],
         *      "raw_extended_squitter_frames_by_source": [10, 11, 9],
         *      "valid_squitter_frames_by_source": [4, 4, 8],
         *      "demods_1090_by_source": [20, 10, 20]
         * }
         * @param[in] buf Buffer to write the JSON string to.
         * @param[in] buf_len Length of the buffer, including the null terminator.
         */
        inline uint16_t ToJSON(char* buf, size_t buf_len) {
            uint16_t message_max_len = buf_len - 1;  // Leave space for null terminator.
            // Add aggregate 1090 stats.
            uint16_t chars_written = snprintf(buf, message_max_len - strnlen(buf, kMetricsJSONMaxLen),
                                              "{ \"raw_squitter_frames\": %lu, \"valid_squitter_frames\": %lu, "
                                              "\"raw_extended_squitter_frames\": %lu, "
                                              "\"valid_extended_squitter_frames\": %lu, \"demods_1090\": %lu, ",
                                              raw_squitter_frames, valid_squitter_frames, raw_extended_squitter_frames,
                                              valid_extended_squitter_frames, demods_1090);
            // Add 1090 stats by source.
            chars_written += ArrayToJSON(buf + chars_written, buf_len - chars_written, "raw_squitter_frames_by_source",
                                         raw_squitter_frames_by_source, "%u", true);
            chars_written +=
                ArrayToJSON(buf + chars_written, buf_len - chars_written, "valid_squitter_frames_by_source",
                            valid_squitter_frames_by_source, "%u", true);
            chars_written +=
                ArrayToJSON(buf + chars_written, buf_len - chars_written, "raw_extended_squitter_frames_by_source",
                            raw_extended_squitter_frames_by_source, "%u", true);
            chars_written +=
                ArrayToJSON(buf + chars_written, buf_len - chars_written, "valid_extended_squitter_frames_by_source",
                            valid_extended_squitter_frames_by_source, "%u", true);
            chars_written += ArrayToJSON(buf + chars_written, buf_len - chars_written, "demods_1090_by_source",
                                         demods_1090_by_source, "%u", true);
            // Add UAT stats.
            chars_written +=
                snprintf(buf + chars_written, buf_len - chars_written,
                         "\"raw_uat_adsb_frames\": %u, \"valid_uat_adsb_frames\": %u"
                         ", \"raw_uat_uplink_frames\": %u, \"valid_uat_uplink_frames\": %u",
                         raw_uat_adsb_frames, valid_uat_adsb_frames, raw_uat_uplink_frames, valid_uat_uplink_frames);

            // Add Remote ID stats.
            chars_written += snprintf(buf + chars_written, buf_len - chars_written,
                                      ", \"raw_remote_id_frames\": %u, \"valid_remote_id_frames\": %u",
                                      raw_remote_id_frames, valid_remote_id_frames);

            // Add dictionary stats.
            chars_written +=
                snprintf(buf + chars_written, buf_len - chars_written,
                         ", \"num_mode_s_aircraft\": %u, \"num_uat_aircraft\": %u, \"num_remote_id_aircraft\": %u",
                         num_mode_s_aircraft, num_uat_aircraft, num_remote_id_aircraft);
            chars_written += snprintf(buf + chars_written, buf_len - chars_written, "}");
            return chars_written;
        }
    };

    /**
     * Default constructor. Uses default config values.
     */
    AircraftDictionary() {
        // Pre-allocate kMaxNumAircraft buckets so that no rehash (and no additional bucket-array
        // allocation from the monotonic region) occurs while we stay within capacity.
        dict.max_load_factor(1.0);
        dict.reserve(kMaxNumAircraft);
    };

    /**
     * Constructor with config values specified.
     */
    AircraftDictionary(AircraftDictionaryConfig config_in) : config_(config_in) {};

    /**
     * Removes all aircraft from the aircraft dictionary.
     */
    void Init();

    /**
     * Prunes stale aircraft from the dictionary.
     * @param[in] timestamp_us Current timestamp, in microseconds, to use for pruning. Aircraft older than timestamp_us
     * minus the pruning interval will be removed.
     */
    void Update(uint32_t timestamp_us);

    /**
     * Log an attempted demodulation on 1090MHz. Used to record performance statistics. Note that the increment won't be
     * visible until the next dictionary update occurs.
     */
    inline void Record1090Demod(int16_t source = -1) {
        metrics_counter_.demods_1090++;
        if (source >= 0 && source < kMaxNumSources) {
            metrics_counter_.demods_1090_by_source[source]++;
        }
    }

    /**
     * Log a received 56-bit squitter frame. Used to record performance statistics. Valid frames are automatically
     * recorded during ingestion, this function is broken out so that raw frames with no ingestion attempted can stil be
     * recorded. Note that the increment won't be visible until the next dictionary update occurs.
     */
    inline void Record1090RawSquitterFrame(int16_t source = -1) {
        metrics_counter_.raw_squitter_frames++;
        if (source >= 0 && source < kMaxNumSources) {
            metrics_counter_.raw_squitter_frames_by_source[source]++;
        }
    }

    /**
     * Log a received 112-bit extended squitter frame. Used to record performance statistics. Valid frames are
     * automatically recorded during ingestion, this function is broken out so that raw frames with no ingestion
     * attempted can stil be recorded. Note that the increment won't be visible until the next dictionary update occurs.
     */
    inline void Record1090RawExtendedSquitterFrame(int16_t source = -1) {
        metrics_counter_.raw_extended_squitter_frames++;
        if (source >= 0 && source < kMaxNumSources) {
            metrics_counter_.raw_extended_squitter_frames_by_source[source]++;
        }
    }

    /**
     * Record receiver metrics from the SubGHz radio.
     * @param[in] raw_uat_adsb_frames Number of raw UAT ADS-B frames received.
     * @param[in] valid_uat_adsb_frames Number of UAT ADS-B frames that passed FEC.
     * @param[in] raw_uat_uplink_frames Number of raw UAT Uplink frames received.
     * @param[in] valid_uat_uplink_frames Number of UAT Uplink frames that passed FEC.
     */
    inline void RecordSubGHzMetrics(uint16_t raw_uat_adsb_frames, uint16_t valid_uat_adsb_frames,
                                    uint16_t raw_uat_uplink_frames, uint16_t valid_uat_uplink_frames) {
        metrics_counter_.raw_uat_adsb_frames += raw_uat_adsb_frames;
        metrics_counter_.valid_uat_adsb_frames += valid_uat_adsb_frames;
        metrics_counter_.raw_uat_uplink_frames += raw_uat_uplink_frames;
        metrics_counter_.valid_uat_uplink_frames += valid_uat_uplink_frames;
    }

    /**
     * Mode S Packet Decoding
     */

    /**
     * Ingests a DecodedModeSPacket and uses it to insert and update the relevant aircraft.
     * @param[in] packet DecodedModeSPacket to ingest. Can be 56-bit (Squitter) or 112-bit (Extended Squitter).
     * Passed as a reference, since packets can be marked as valid by this function.
     * @retval True if successful, false if something broke.
     */
    bool IngestDecodedModeSPacket(DecodedModeSPacket& packet);

    /**
     * Ingests an Identity Surveillance Reply packet and uses it to update the relevant aircraft. Exposed for
     * testing, but usually called by IngestDecodedModeSPacket.
     * Note: this function requires that the packet be marked as valid using the ForceValid() function. If the packet is
     * valid and does not match an ICAO in the aircraft dictionary, a new aircraft will be inserted.
     * @param[in] packet ModeSIdentityReplyPacket to ingest.
     * @retval True if successful, false if something broke.
     */
    bool IngestModeSIdentityReplyPacket(const ModeSIdentityReplyPacket& packet);

    /**
     * Ingests an Altitude Surveillance Reply packet and uses it to update the relevant aircraft. Exposed for
     * testing, but usually called by IngestDecodedModeSPacket.
     * Note: this function requires that the packet be marked as valid using the ForceValid() function. If the packet is
     * valid and does not match an ICAO in the aircraft dictionary, a new aircraft will be inserted.
     * @param[in] packet ModeSAltitudeReplyPacket to ingest.
     * @retval True if successful, false if something broke.
     */
    bool IngestModeSAltitudeReplyPacket(const ModeSAltitudeReplyPacket& packet);

    /**
     * Ingests an All Call Reply packet and uses it to update the relevant aircraft. Exposed for testing, but usually
     * called by IngestDecodedModeSPacket.
     *
     * Currently, we only accept all call reply packets with an interrogator ID of 0 (replies to spontaneous acquisition
     * squitters), since we don't have a way to know the interrogator ID of ground based surveillance stations.
     */
    bool IngestModeSAllCallReplyPacket(const ModeSAllCallReplyPacket& packet);

    /**
     * Ingests an ModeSADSBPacket directly. Exposed for testing, but usually this gets called by
     * IngestDecodedModeSPacket and should not get touched directly.
     * @param[in] packet ModeSADSBPacket to ingest. Derived from a DecodedModeSPacket with DF=17-19.
     * @retval True if successful, false if something broke.
     */
    bool IngestModeSADSBPacket(const ModeSADSBPacket& packet);

    /**
     * UAT Packet Decoding
     */

    bool IngestDecodedUATADSBPacket(const DecodedUATADSBPacket& packet);

    /**
     * Remote ID (Broadcast Drone ID) Decoding
     */

    /**
     * Ingests a raw Broadcast Remote ID packet (an Open Drone ID message or message pack, as received over BLE/WiFi on
     * the ESP32). Decodes the payload with the vendored opendroneid-core-c library and inserts/updates the corresponding
     * RemoteIDAircraft, keyed on a CRC of the source MAC. Records raw/valid frame metrics.
     * @param[in] packet Raw Remote ID packet to ingest (payload + transport metadata).
     * @retval True if the packet decoded and updated an aircraft, false otherwise.
     */
    bool IngestRawRemoteIDPacket(const RawRemoteIDPacket& packet);

    /**
     * Returns the number of aircraft currently in the dictionary.
     * @retval Number of aircraaft that are currently in the dictionary.
     */
    inline uint16_t GetNumAircraft() { return dict.size(); }

    /**
     * Adds an Aircraft object to the aircraft dictionary, hashed by ICAO address.
     * @param[in] aircraft Aircraft to insert.
     * @retval Pointer to the aircraft that was added, or nullptr if the aircraft could not be added. If the aircraft
     * already exists in the dictionary, it will be overwritten and a pointer to the existing aircraft will be returned.
     * @note The aircraft must be derived from the Aircraft class, since the UID function is used to hash the aircraft.
     */
    template <typename T>
    T* InsertAircraft(const T& aircraft) {
        // Extract UID from the underlying Aircraft virtual class.
        uint32_t uid = static_cast<const Aircraft&>(aircraft).GetUID();
        auto itr = dict.find(uid);
        if (itr != dict.end()) {
            // Overwriting an existing aircraft
            itr->second = aircraft;
            return std::get_if<T>(&itr->second);
        }

        if (dict.size() >= kMaxNumAircraft) {
            dictionary_overflowed = true;
            RemoveOldestAircraft();
        }

        dict[uid] = aircraft;               // add the new aircraft to the dictionary
        return std::get_if<T>(&dict[uid]);  // return a pointer to the aircraft that was added
    }

    /**
     * Remove an aircraft from the dictionary, by ICAO address.
     * @param[in] uid UID of the aircraft to remove from the dictionary.
     * @retval True if removal succeeded, false if aircraft was not found.
     */
    bool RemoveAircraft(uint32_t uid);

    /**
     * Remove the aircraft with the oldest last_message_timestamp_ms from the dictionary.
     * @retval True if an aircraft was removed, false if the dictionary was empty.
     */
    bool RemoveOldestAircraft();

    /**
     * Retrieve an aircraft from the dictionary.
     * @param[in] uid Address to use for looking up the aircraft.
     * @param[out] aircraft_out Aircraft reference to put the retrieved aircraft into if successful.
     * @retval True if aircraft was found and retrieved, false if aircraft was not in the dictionary.
     */
    template <typename T>
    bool GetAircraft(uint32_t uid, T& aircraft_out) {
        // Don't create and insert the aircraft if it is not in the dictionary.
        T* aircraft_ptr = GetAircraftPtr<T>(uid, false);
        if (aircraft_ptr) {
            aircraft_out = *aircraft_ptr;
            return true;
        }
        return false;
    }

    /**
     * Returns a pointer to an aircraft in the dictionary.
     * @param[in] uid Address to use for looking up the aircraft.
     * @param[in] insert_if_not_found If true, a new aircraft will be created and inserted into the dictionary if it is
     * not found. If false, nullptr will be returned if the aircraft is not found.
     * @retval Pointer to the aircraft if found or inserted, nullptr if not found.
     */
    template <typename T>
    T* GetAircraftPtr(uint32_t uid, bool insert_if_not_found = true) {
        auto itr = dict.find(uid);
        if (itr != dict.end()) {
            // Aircraft was found in the dictionary. Get a pointer to it if the type is correct.
            T* aircraft_ptr = std::get_if<T>(&(itr->second));
            if (aircraft_ptr) {
                return aircraft_ptr;  // Found the aircraft, return it.
            }
        } else if (insert_if_not_found) {
            // Aircraft was not found but should be inserted.
            T new_aircraft(Aircraft::UIDToICAOAddress(uid));
            return InsertAircraft(new_aircraft);
        }
        // Aircraft not found with the given UID.
        return nullptr;
    }

    /**
     * Get the position of the lowest aircraft in the dictionary with a valid GNSS altitude. This can be used to
     * approximate the receiver position.
     * @param[out] latitude_deg Latitude of the lowest aircraft, in degrees.
     * @param[out] longitude_deg Longitude of the lowest aircraft, in degrees.
     * @param[out] gnss_altitude_ft GNSS altitude of the lowest aircraft, in feet.
     * @param[out] baro_altitude_ft Barometric altitude of the lowest aircraft, in feet.
     * @param[out] heading_deg Heading of the lowest aircraft, in degrees. NAN if not available.
     * @param[out] speed_kts Speed of the lowest aircraft, in knots. NAN if not available.
     * @retval True if a lowest aircraft was found, false if no aircraft with a valid GNSS altitude exists in the
     * dictionary.
     */
    bool GetLowestAircraftPosition(float& latitude_deg, float& longitude_deg, int32_t& gnss_altitude_ft,
                                   int32_t& baro_altitude_ft, float& heading_deg, int32_t& speed_kts);

    /**
     * Check if an aircraft is contained in the dictionary.
     * @param[in] uid Address to use for looking up the aircraft.
     * @retval True if aircraft is in the dictionary, false if not.
     */
    bool ContainsAircraft(uint32_t uid) const;

    /**
     * Used to enable or disable the CPR position filter.
     * @param[in] enabled True to enable the filter, false to disable it.
     */
    inline void SetCPRPositionFilterEnabled(bool enabled) { config_.enable_cpr_position_filter = enabled; }

    /**
     * Sets a reference position used when decoding Mode S surface position packets. Should be called whenever the
     * reference position (i.e. the receiver location) changes.
     * @param[in] latitude_deg Latitude of the reference position, in degrees.
     * @param[in] longitude_deg Longitude of the reference position, in degrees.
     */
    void SetReferencePosition(float latitude_deg, float longitude_deg);

    /**
     * Marks the reference position as unavailable. While the reference is invalid, Mode S surface position packets
     * are ingested (air/ground state, movement, heading) but their CPR position decode is skipped, since surface
     * CPR decoded against a bogus reference silently yields a wrong position.
     */
    inline void ClearReferencePosition() { reference_position_valid_ = false; }

    /**
     * Check whether a valid reference position has been set.
     * @retval True if SetReferencePosition has been called and not subsequently cleared.
     */
    inline bool ReferencePositionValid() const { return reference_position_valid_; }

    /**
     * Check if the CPR position filter is enabled.
     * @retval True if the filter is enabled, false if it is disabled.
     */
    inline bool CPRPositionFilterIsEnabled() { return config_.enable_cpr_position_filter; }

    std::pmr::unordered_map<uint32_t, AircraftEntry> dict{&pool_};

    Metrics metrics;

    AircraftEntry* lowest_aircraft_entry = nullptr;  // Pointer to the aircraft entry with the lowest valid GNSS
                                                     // altitude. Useful for approximating receiver position.
    bool dictionary_overflowed = false;

   private:
    /**
     * Custom fixed-pool memory resource for AircraftDictionary.
     *
     * Manages two allocation regions:
     *  - A free-list pool for hash-map node allocations (one slot per aircraft). Freed slots are
     *    returned to the pool and reused, so up to kMaxNumAircraft insertions are always
     *    possible regardless of how many prior insert/erase cycles have occurred.
     *  - A one-shot monotonic region for the hash-table bucket array (allocated once via
     *    reserve(), never rehashed while within capacity bounds).
     *
     * null_memory_resource() is used as the monotonic upstream, so any true overflow terminates
     * cleanly rather than silently falling back to the heap.
     */
    class FixedPoolResource : public std::pmr::memory_resource {
       public:
        // Upper bound on a single unordered_map node:
        //   sizeof(void*)                                  — next-node pointer in the linked list
        //   sizeof(pair<const uint32_t, AircraftEntry>)   — key-value pair stored in the node
        //   alignof(pair<const uint32_t, AircraftEntry>)  — worst-case intra-node alignment slack
        // Rounded up to the nearest multiple of alignof(AircraftEntry) so every block in
        // node_blocks_ starts on a suitably aligned boundary.
        static constexpr size_t kNodeBlockRaw = sizeof(void*) + sizeof(std::pair<const uint32_t, AircraftEntry>) +
                                                alignof(std::pair<const uint32_t, AircraftEntry>);
        static constexpr size_t kNodeBlockSize =
            (kNodeBlockRaw + alignof(AircraftEntry) - 1) & ~(alignof(AircraftEntry) - 1);

        // Bucket buffer sized to survive repeated doublings from 1 → kMaxNumAircraft buckets.
        // The geometric series sums to at most 2*kMaxNumAircraft pointer-widths, plus a margin.
        static constexpr size_t kBucketBufferSize = kMaxNumAircraft * sizeof(void*) * 2 + 512;

        FixedPoolResource() {
            // Link every block into the initial free list.
            free_head_ = nullptr;
            for (int i = 0; i < kMaxNumAircraft; ++i) {
                auto* blk = reinterpret_cast<FreeNode*>(node_blocks_[i]);
                blk->next = free_head_;
                free_head_ = blk;
            }
        }

        FixedPoolResource(const FixedPoolResource&) = delete;
        FixedPoolResource& operator=(const FixedPoolResource&) = delete;

       protected:
        void* do_allocate(size_t bytes, size_t alignment) override {
            if (bytes <= kNodeBlockSize) {
                if (free_head_ != nullptr) {
                    void* ptr = free_head_;
                    free_head_ = free_head_->next;
                    return ptr;
                }
                // Free-list exhausted — InsertAircraft's capacity guard should have prevented
                // this. Fall through to the monotonic region so behaviour remains defined.
            }
            // Large allocation (bucket array) or pool overflow: served from the monotonic
            // region. null_memory_resource() upstream ensures clean termination on overflow.
            return bucket_mono_.allocate(bytes, alignment);
        }

        void do_deallocate(void* p, size_t bytes, size_t alignment) override {
            if (bytes <= kNodeBlockSize) {
                // Return the node slot to the free list for reuse.
                auto* blk = reinterpret_cast<FreeNode*>(p);
                blk->next = free_head_;
                free_head_ = blk;
                return;
            }
            // Large allocations (bucket arrays) come from the monotonic region and cannot be
            // individually reclaimed.
        }

        bool do_is_equal(const std::pmr::memory_resource& other) const noexcept override { return this == &other; }

       private:
        struct FreeNode {
            FreeNode* next;
        };
        static_assert(sizeof(FreeNode) <= kNodeBlockSize,
                      "kNodeBlockSize is too small to hold a FreeNode — increase the size formula");

        // Pre-allocated, aligned node storage: one block per aircraft.
        alignas(alignof(AircraftEntry)) uint8_t node_blocks_[kMaxNumAircraft][kNodeBlockSize];
        FreeNode* free_head_ = nullptr;

        // One-shot monotonic storage for the hash-table bucket array (and free-list overflow).
        uint8_t bucket_buffer_[kBucketBufferSize];
        std::pmr::monotonic_buffer_resource bucket_mono_{bucket_buffer_, kBucketBufferSize,
                                                         std::pmr::null_memory_resource()};
    };

    // Helper functions for ingesting specific ADS-B packet types, called by IngestModeSADSBPacket.

    AircraftDictionaryConfig config_;
    // Counters in metrics_counter_ are incremented, then metrics_counter_ is swapped into metrics during the dictionary
    // update. This ensures that the public metrics struct always has valid data.
    Metrics metrics_counter_;

    // Reference position for decoding Mode S surface position packets, in angular weighted binary format.
    uint32_t reference_latitude_awb32_ = 0;
    uint32_t reference_longitude_awb32_ = 0;
    bool reference_position_valid_ = false;  // Set by SetReferencePosition, cleared by ClearReferencePosition.

    FixedPoolResource pool_;  // Free-list pool + monotonic bucket region; must outlive dict.
};