#pragma once

#include <cstdio>
#include <cstring>
#include <memory_resource>
#include <unordered_map>
#include <variant>

#include "adsb_types.hh"
#include "aircraft_dictionary_config.hh"
#include "altitude_filter.hh"
#include "geo_utils.hh"
// Can't include comms.hh or we will get a circular dependency.
#include "hal.hh"
#include "json_utils.hh"
#include "macros.hh"
#include "mode_s_packet.hh"
#include "range_map.hh"
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
    // Local time (get_time_since_boot_ms()) of the last accepted position, comparable across aircraft types. The
    // protocol-specific last_track_update_timestamp_ms holds packet MLAT time for Mode S.
    uint32_t last_position_update_ms = 0;
    uint32_t last_velocity_update_ms = 0;  // Local time of the last velocity, 0 until one arrives.

    float latitude_deg = 0.0f;
    float longitude_deg = 0.0f;

    int32_t baro_altitude_ft = 0;
    int32_t gnss_altitude_ft = 0;

    float direction_deg = 0;
    int32_t speed_kts = 0;  // Provide full 32 bit depth so we can square it without issues.

    int32_t baro_vertical_rate_fpm = 0;
    int32_t gnss_vertical_rate_fpm = 0;

    // Distance and bearing from the receiver, kept up to date by the AircraftDictionary (at most 1 s behind the
    // position). kDistanceUnknown while the aircraft's or the receiver's position is unknown.
    static constexpr uint32_t kDistanceUnknown = UINT32_MAX;
    uint32_t distance_m = kDistanceUnknown;
    uint16_t bearing_bam16 = 0;              // Clockwise from north, in 1/65536 turns. See Bam16ToDeg().
    uint16_t distance_reference_epoch = 0;  // AircraftDictionary reference epoch distance_m was computed in.

    /**
     * Whether the distance filter (AT+FILTER_DISTANCE_NM) leaves this aircraft out of aircraft reports. Raw packets are
     * never filtered. Aircraft at an unknown distance are kept.
     * @param[in] max_distance_m Largest distance from the receiver to keep, in meters. 0 turns the filter off.
     * @retval True if the aircraft is known to be farther than max_distance_m.
     */
    inline bool IsBeyondDistance(uint32_t max_distance_m) const {
        return max_distance_m != 0 && distance_m != kDistanceUnknown && distance_m > max_distance_m;
    }
};

/**
 * Mode S aircraft class.
 */
class ModeSAircraft : public Aircraft {
   public:
    // These variables define filter bounds for time between CPR packets. If the time between packets is greater than
    // the time delta limit, the old CPR packet is discarded and the CPR packet pair is not used for position decoding.
    static constexpr uint32_t kDefaultCPRIntervalMs = 10e3;  // CPR interval when starting from scratch or stale track.
    // DO-260C 2.2.10.3.1: even/odd airborne position pairs are accepted within MIN(20, 10000 / ground speed kts) seconds.
    static constexpr uint32_t kRefCPRIntervalMs = 20e3;  // Reference interval for rejecting CPR packet pairs at 500kts.
    static constexpr uint32_t kMaxCPRIntervalMs = 20e3;  // Never accept CPR packet pairs more than 20 seconds apart.
    static constexpr uint32_t kMaxTrackUpdateIntervalMs = 20e3;  // Tracks older than this are considered stale.

    static constexpr uint16_t kCallSignMaxNumChars = 8;
    static constexpr uint16_t kCallSignMinNumChars = 3;  // Callsigns must be at this long to be valid.

    // ADS-B position and velocity messages decide air/ground; Mode S replies only once none has arrived for this long.
    static constexpr uint32_t kADSBAirGroundHoldMs = 30e3;
    // Above this, a Mode S reply saying "on the ground" is a transponder fault (highest airport: 14,472 ft).
    static constexpr int32_t kMaxGroundAltitudeFt = 18000;

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
        kBitFlagIsNonTransponder = 19,          // Reported address type is a DF18 one (GetAddressType()).
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

    // How the aircraft is heard, in readsb's order: a lower value is a more direct source. DO-260C Table 2-7.
    enum AddressType : uint8_t {
        kAddressTypeADSBICAO = 0,                // DF17.
        kAddressTypeADSBICAONonTransponder = 1,  // DF18 CF=0.
        kAddressTypeADSRICAO = 2,                // DF18 CF=6, rebroadcast of UAT ADS-B.
        kAddressTypeTISBICAO = 3,                // DF18 CF=2, radar or multilateration track.
        kAddressTypeModeS = 4,                   // Mode S replies or DF19 only.
        // Addresses below aren't ICAO addresses and get their own dictionary entries.
        kAddressTypeADSBOther = 5,      // DF18 CF=1: anonymous, ground vehicle or obstruction address.
        kAddressTypeADSROther = 6,      // DF18 CF=6 with IMF (ICAO/Mode A flag) set.
        kAddressTypeTISBTrackFile = 7,  // DF18 CF=2 with IMF set: Mode A code and track file number.
        kAddressTypeTISBOther = 8,      // DF18 CF=5.
        kAddressTypeInvalid = 0xF
    };
    static constexpr AddressType kFirstNonICAOAddressType = kAddressTypeADSBOther;
    // A less direct source can't replace position, velocity or other data from a more direct one for this long.
    static constexpr uint32_t kAddressTypeHoldMs = 15e3;

    /**
     * Returns the address type of an extended squitter, or kAddressTypeInvalid if it doesn't describe a target
     * (DF18 CF=3, 4 and 7). The ICAO/Mode A flag position depends on the type code (DO-260C 2.2.17, 2.2.18).
     */
    static AddressType GetAddressType(const ModeSADSBPacket& packet);

    // TIS-B or ADS-R (DF18 CF=2, 5, 6).
    static inline bool IsRebroadcast(AddressType type) {
        return type == kAddressTypeADSRICAO || type == kAddressTypeTISBICAO || type >= kAddressTypeADSROther;
    }
    static inline bool IsTISB(AddressType type) {
        return type == kAddressTypeTISBICAO || type == kAddressTypeTISBTrackFile || type == kAddressTypeTISBOther;
    }
    static inline bool IsADSR(AddressType type) {
        return type == kAddressTypeADSRICAO || type == kAddressTypeADSROther;
    }

    /**
     * Dictionary key of a Mode S address: non-ICAO address types go in the address qualifier bits so they never
     * collide with an ICAO address.
     */
    static inline uint32_t AddressToUID(uint32_t address, AddressType type) {
        uint32_t qualifier = type >= kFirstNonICAOAddressType && type != kAddressTypeInvalid ? type : 0;
        return ICAOToUID((address & 0xFFFFFF) | (qualifier << kAddressQualifierBitShift), kAircraftTypeModeS);
    }

    struct Metrics {
        // We can only confirm that valid frames came from this aircraft. Not sure who invalid frames are from.
        uint16_t valid_squitter_frames = 0;
        uint16_t valid_extended_squitter_frames = 0;
    };

    /**
     * @param[in] icao_address_in 24-bit address. A non-ICAO address carries its address type in the address
     * qualifier bits (AddressToUID).
     */
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
    inline uint32_t GetUID() const { return AddressToUID(icao_address, static_cast<AddressType>(last_address_type)); }

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
     * @param[in] received_timestamp_ms MLAT timestamp of the newest CPR packet, in ms. Track age is measured against
     * this timestamp, because last_track_update_timestamp_ms holds packet (MLAT) time, which is the RP2040's clock. On
     * the ESP32, get_time_since_boot_ms() counts from the ESP32's own boot, so comparing it with packet time made every
     * track look stale and pinned the interval at kDefaultCPRIntervalMs.
     * @retval Maximum allowed time delta between CPR packets.
     */
    uint32_t GetMaxAllowedCPRIntervalMs(uint32_t received_timestamp_ms) const {
        uint32_t track_age_ms = received_timestamp_ms > last_track_update_timestamp_ms
                                    ? received_timestamp_ms - last_track_update_timestamp_ms
                                    : 0;  // Packets can arrive slightly out of order.
        if (speed_source == ADSBTypes::kSpeedSourceNotSet || speed_source == ADSBTypes::kSpeedSourceNotAvailable ||
            track_age_ms > kMaxTrackUpdateIntervalMs) {
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
        num_frames_received++;
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
    inline void WriteBitFlag(BitFlag bit, bool value) { value ? flags |= (0b1 << bit) : flags &= ~(0b1 << bit); }

    /**
     * Air/ground from an ADS-B position or velocity message. A change clears the CPR packets, since airborne and
     * surface positions don't pair.
     */
    inline void SetAirGroundFromADSB(bool airborne, uint32_t timestamp_ms) {
        if (HasBitFlag(kBitFlagIsAirborne) != airborne) {
            ClearCPRPackets();
        }
        WriteBitFlag(kBitFlagIsAirborne, airborne);
        last_adsb_air_ground_ms = timestamp_ms;
        adsb_air_ground_heard_ = true;
        air_ground_known_ = true;
    }

    /**
     * On the ground, for every output (JSON on_ground, ASTERIX GBS, GDL90 airborne). False until a message has given
     * air/ground: kBitFlagIsAirborne starts cleared, which would otherwise show every new aircraft on the ground.
     */
    inline bool IsOnGround() const { return air_ground_known_ && !HasBitFlag(kBitFlagIsAirborne); }

    /**
     * Air/ground from a Mode S reply (flight status, vertical status or capability). Ignored while ADS-B decides
     * (kADSBAirGroundHoldMs), and "on the ground" above kMaxGroundAltitudeFt is ignored. Leaves the CPR packets alone.
     * @param[in] airborne Air/ground state the reply gives.
     * @param[in] altitude_ft Altitude in the same reply, or INT32_MIN if it has none (the last baro altitude is used).
     * @param[in] timestamp_ms Local time of the reply.
     * @retval True if the state was taken.
     */
    bool SetAirGroundFromModeS(bool airborne, int32_t altitude_ft, uint32_t timestamp_ms);

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
    uint32_t num_frames_received = 0;  // Valid frames received since the aircraft was added, never reset.

    uint16_t transponder_capability = 0;
    uint32_t last_adsb_air_ground_ms = 0;  // Local time of the last ADS-B message that gave air/ground.
    uint32_t icao_address = 0;
    char callsign[kCallSignMaxNumChars + 1] = "?";  // put extra EOS character at end
    uint8_t num_position_decodes = 0;  // Accepted positions in a row, up to 2. Sits in padding.
    uint16_t squawk = ADSBTypes::kSquawkCodeNotYetReceived;  // Squawk digits as a decimal value, e.g. 7700.
    ADSBTypes::EmitterCategory emitter_category = ADSBTypes::kEmitterCategoryNoCategoryInfo;
    uint8_t emitter_category_raw = 0;  // Non-enum category in case we want the value without a many to one mapping.

    ADSBTypes::SpeedSource speed_source = ADSBTypes::kSpeedSourceNotSet;  // Most recent reported speed.
    // Used to keep track of the most recently reported altitude, for use in adjusting the complementary altitude with
    // an offset.
    ADSBTypes::AltitudeSource altitude_source = ADSBTypes::kAltitudeSourceNotSet;

    // Aircraft Operation Status Message
    // Navigation Integrity Category (NIC)
    uint8_t nic_bits_valid = 0b000;  // MSb to LSb: nic_d1_valid nic_d0_valid nic_c_valid nic_b_valid nic_a_valid.
    uint8_t nic_bits = 0b000;        // MSb to LSb: nic_d1 nic_d0 nic_c nic_b nic_a.
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

    AddressType position_address_type = kAddressTypeADSBICAO;  // Source of the position.
    AddressType velocity_address_type = kAddressTypeADSBICAO;  // Source of the velocity.
    uint8_t cpr_address_type  : 4 = kAddressTypeADSBICAO;      // Source of the cached CPR packets.
    uint8_t last_address_type : 4 = kAddressTypeModeS;         // Source of the last extended squitter applied.

    /**
     * Returns how the aircraft is heard: the source of its position, or of the last extended squitter applied if it
     * has no position. kAddressTypeModeS if no extended squitter was applied.
     */
    inline AddressType GetAddressType() const {
        return HasBitFlag(kBitFlagPositionValid) ? position_address_type : static_cast<AddressType>(last_address_type);
    }

    /**
     * Decides whether an extended squitter may update the aircraft, and records its source. Like readsb, a less
     * direct source is ignored for kAddressTypeHoldMs after a more direct one updated the same data: position
     * messages are held against the position, velocity messages against the velocity, and others against the
     * position.
     * @param[in] type Address type of the message.
     * @param[in] type_code Type code of the message.
     * @param[in] timestamp_ms Local time (get_time_since_boot_ms()).
     * @retval True if the message should be applied.
     */
    bool AcceptAddressType(AddressType type, uint16_t type_code, uint32_t timestamp_ms);

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
    // TC = 0: sent in place of a position message while the aircraft has no position. Applies the altitude, if any.
    bool ApplyNoPositionMessage(const ModeSADSBPacket& packet);
    bool ApplyAirborneVelocitiesMessage(const ModeSADSBPacket& packet);
    bool ApplyAircraftStatusMessage(const ModeSADSBPacket& packet);
    bool ApplyTargetStateAndStatusInfoMessage(const ModeSADSBPacket& packet);
    bool ApplyAircraftOperationStatusMessage(const ModeSADSBPacket& packet);

   private:
    bool adsb_air_ground_heard_ = false;  // An ADS-B message has given air/ground (last_adsb_air_ground_ms is set).
    bool air_ground_known_ = false;       // An ADS-B message or Mode S reply has given air/ground.

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
    // Received timestamps of the even and odd packets that produced the pending, unconfirmed jump candidate in
    // lat_awb32_ / lon_awb32_ (0 if none). A jump is only confirmed by a packet pair that shares neither packet, so a
    // single corrupted CPR packet can't confirm itself when it is decoded against two consecutive complementary packets.
    uint32_t candidate_even_received_timestamp_ms_ = 0;
    uint32_t candidate_odd_received_timestamp_ms_ = 0;
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
        kFixedADSBBeacon = 5,
        kADSRTargetWithNonICAOAddress = 6  // DO-282B; dump978's ADSR_OTHER.
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
    inline void IncrementNumFramesReceived() {
        metrics_counter_.valid_frames++;
        num_frames_received++;
    }

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
     * UAT state vector contains longitude as 24-bit Angular Weighted Binary (AWB). Convert to 32-bit angular weighted
     * binary.
     * @param[in] uat_lon_awb 24-bit UAT longitude in AWB format.
     * @retval 32-bit longitude in AWB format.
     */
    static inline uint32_t UATLonAWBToAWB32(uint32_t uat_lon_awb) {
        return (uat_lon_awb << 8);  // Shift left to convert from 24-bit to 32-bit AWB.
    }

    /**
     * UAT state vector contains latitude as 24-bit AWB with the MSB omitted (23 bits, UAT Tech Manual Table 2-12 note
     * 1): valid latitudes are 0x000000-0x400000 (0 to 90N) and 0xC00000-0xFFFFFF (90S to 0), so the two MSBs are
     * always equal apart from the north pole. Restore the MSB, then convert to 32-bit AWB.
     * @param[in] uat_lat_awb 23-bit UAT latitude.
     * @retval 32-bit latitude in AWB format.
     */
    static inline uint32_t UATLatAWBToAWB32(uint32_t uat_lat_awb) {
        uint32_t lat_awb24 = uat_lat_awb > 0x400000 ? (uat_lat_awb | 0x800000) : uat_lat_awb;
        return (lat_awb24 << 8);
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
    uint32_t num_frames_received = 0;  // Valid frames received since the aircraft was added, never reset.

    uint16_t transponder_capability = 0;
    uint32_t icao_address = 0;
    char callsign[ModeSAircraft::kCallSignMaxNumChars + 1] = "?";  // put extra EOS character at end
    uint16_t squawk = ADSBTypes::kSquawkCodeNotYetReceived;  // Squawk digits as a decimal value, e.g. 7700.
    ADSBTypes::EmitterCategory emitter_category = ADSBTypes::kEmitterCategoryNoCategoryInfo;
    uint8_t emitter_category_raw = 0;  // Non-enum category.
    uint8_t num_position_decodes = 0;  // Accepted positions in a row, up to 2. Sits in padding.
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
#ifdef SWARM_RECORDING
    // Location messages with a position in a row, up to 2. Sits in padding; swarm builds only, so others are unchanged.
    uint8_t num_position_decodes = 0;
#endif

    int32_t height_above_takeoff_ft = 0;  // UA height above takeoff or ground per height_type. Valid iff flag set.

    float operator_latitude_deg = 0.0f;
    float operator_longitude_deg = 0.0f;
    int32_t operator_altitude_geo_ft = 0;  // Operator (System message) geometric altitude, WGS84.

    Metrics metrics;

   private:
    Metrics metrics_counter_;
};

/**
 * Which aircraft go into aircraft reports, resolved once per reporting round by AircraftDictionary::GetReportFilter():
 * the distance filter (AT+FILTER_DISTANCE_NM) and the altitude filter (AT+FILTER_ALTITUDE_FT). An aircraft is reported
 * only if it passes both. Raw packets are never filtered.
 */
struct AircraftReportFilter {
    uint32_t max_distance_m = 0;  // 0: no distance filter.
    AltitudeFilterBounds altitude;

    // Altitude the altitude filter compares: barometric, else GNSS. False if neither is known.
    template <typename T>
    static bool ReportAltitudeFt(const T &aircraft, int32_t &altitude_ft) {
        if (aircraft.HasBitFlag(T::kBitFlagBaroAltitudeValid)) {
            altitude_ft = aircraft.baro_altitude_ft;
            return true;
        }
        if (aircraft.HasBitFlag(T::kBitFlagGNSSAltitudeValid)) {
            altitude_ft = aircraft.gnss_altitude_ft;
            return true;
        }
        return false;
    }
    static bool ReportAltitudeFt(const RemoteIDAircraft &aircraft, int32_t &altitude_ft) {
        altitude_ft = aircraft.gnss_altitude_ft;
        return aircraft.HasBitFlag(RemoteIDAircraft::kBitFlagGNSSAltitudeValid);
    }

    /**
     * Whether an aircraft is left out of aircraft reports. Aircraft with an unknown distance or altitude pass that
     * filter.
     */
    template <typename T>
    bool Excludes(const T &aircraft) const {
        if (aircraft.IsBeyondDistance(max_distance_m)) return true;
        if (!altitude.enabled) return false;
        int32_t altitude_ft = 0;
        bool known = ReportAltitudeFt(aircraft, altitude_ft);
        return altitude.Excludes(known, altitude_ft);
    }
};

class AircraftDictionary {
   public:
    static constexpr uint16_t kMaxNumAircraft = kAircraftDictionaryMaxNumAircraft;

    // Receives range, message and aircraft counts for swarm reports (AT+SWARM), or nullptr for none. ESP32 only.
    void SetSwarmRecorder(SwarmRecorder *recorder) { swarm_recorder_ = recorder; }

    // Heard directly: TIS-B and ADS-R positions say where a ground station hears the aircraft, not this receiver.
    static inline bool IsDirectPosition(const ModeSAircraft &aircraft) {
        return aircraft.position_address_type == ModeSAircraft::kAddressTypeADSBICAO ||
               aircraft.position_address_type == ModeSAircraft::kAddressTypeADSBICAONonTransponder ||
               aircraft.position_address_type == ModeSAircraft::kAddressTypeADSBOther;
    }
    static inline bool IsDirectPosition(const UATAircraft &aircraft) {
        UATAircraft::AddressQualifier q = aircraft.GetAddressQualifier();
        return q == UATAircraft::kADSBTargetWithICAO24BitAddress ||
               q == UATAircraft::kADSBTargetWithSelfAssignedTemporaryAddress || q == UATAircraft::kSurfaceVehicle ||
               q == UATAircraft::kFixedADSBBeacon;
    }
    static constexpr uint16_t kMaxNumSources = 4;  // One per 1090 demodulator state machine.

#ifdef FILTER_CPR_POSITIONS
    // Reported velocities can be inaccurate, so a position jump of up to this distance is always allowed.
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
        // Address parity replies that match no aircraft get a one or two bit correction if it gives a unique tracked
        // aircraft with a plausible altitude or squawk.
        bool enable_address_parity_correction = false;
#ifdef FILTER_CPR_POSITIONS
        // CPR position filter checks each new aircraft location against the previous location and requires two
        // consecutive packets within a geographic radius to confirm large jumps in aircraft location. This reduces the
        // likelihood that an aircraft might "jump" but increases CPU load.
        bool enable_cpr_position_filter = true;
#endif
    };

    // Mode S frames accepted after a bit error correction, and corrected frames turned down.
    enum CorrectionKind : uint8_t {
        kCorrectionExtendedSquitter1Bit = 0,
        kCorrectionExtendedSquitter2Bit,
        kCorrectionAddressParity1Bit,
        kCorrectionAddressParity2Bit,
        kCorrectionRejected,
        kCorrectionSkipped,  // Searches not made: over the per-update limit, or under load.
        // Messages the 1090 demodulator read again at another start offset: DF11/17/18 that then passed their CRC,
        // and address/parity replies that then named a tracked aircraft.
        kCorrectionOffsetRetry,
        kCorrectionOffsetRetryAddressParity,
        kNumCorrectionKinds
    };
    static constexpr const char* kCorrectionKindStrs[kNumCorrectionKinds] = {
        "extended_squitter_1_bit", "extended_squitter_2_bit", "address_parity_1_bit", "address_parity_2_bit",
        "rejected", "skipped", "offset_retry", "offset_retry_address_parity"};
    // Address parity searches per dictionary update (1 s). Each one checks every tracked aircraft.
    static constexpr uint16_t kMaxAddressParityCorrectionAttemptsPerUpdate = 250;

    struct Metrics {
        static const uint16_t kMetricsJSONMaxLen = 1100;  // Includes null terminator.

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

        uint16_t corrected_frames[kNumCorrectionKinds] = {0};  // By CorrectionKind.

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
            chars_written += snprintf(buf + chars_written, buf_len - chars_written, ", \"corrected_frames\": {");
            for (uint16_t i = 0; i < kNumCorrectionKinds; i++) {
                chars_written += snprintf(buf + chars_written, buf_len - chars_written, "%s\"%s\": %u", i ? ", " : "",
                                          kCorrectionKindStrs[i], corrected_frames[i]);
            }
            chars_written += snprintf(buf + chars_written, buf_len - chars_written, "}}");
            return chars_written;
        }
    };

    /**
     * Default constructor. Uses default config values.
     */
    AircraftDictionary() : AircraftDictionary(AircraftDictionaryConfig()) {};

    /**
     * Constructor with config values specified.
     */
    AircraftDictionary(AircraftDictionaryConfig config_in) : config_(config_in) {
        // Pre-allocate kMaxNumAircraft buckets so that no rehash (and no additional bucket-array
        // allocation from the monotonic region) occurs while we stay within capacity.
        dict.max_load_factor(1.0);
        dict.reserve(kMaxNumAircraft);
    };

    inline void SetAddressParityCorrection(bool enable) { config_.enable_address_parity_correction = enable; }
    // While paused (e.g. a backlog of decoded packets), address parity searches are skipped.
    inline void PauseAddressParityCorrection(bool paused) {
        address_parity_correction_budget_ = paused ? 0 : kUnlimitedAddressParityCorrections;
    }
    // Allows at most num address parity searches until the next call, e.g. one per main loop pass on a single core.
    inline void LimitAddressParityCorrection(uint16_t num) { address_parity_correction_budget_ = num; }
    // Counts searches skipped elsewhere, e.g. the decoder's 2 bit searches.
    inline void RecordSkippedCorrections(uint32_t num) { metrics_counter_.corrected_frames[kCorrectionSkipped] += num; }
    inline void RecordOffsetRetries(uint32_t num_crc, uint32_t num_address_parity) {
        metrics_counter_.corrected_frames[kCorrectionOffsetRetry] += num_crc;
        metrics_counter_.corrected_frames[kCorrectionOffsetRetryAddressParity] += num_address_parity;
    }

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
     * Ingests a DF=0 or DF=16 (ACAS air-air surveillance) reply and uses its vertical status and altitude to update the
     * relevant aircraft. Exposed for testing, but usually called by IngestDecodedModeSPacket. The packet must be valid
     * (i.e. its address parity already confirmed against the dictionary).
     * @param[in] packet DecodedModeSPacket with DF=0 or DF=16 to ingest.
     * @retval True if successful, false if something broke.
     */
    bool IngestModeSAirAirSurveillancePacket(const DecodedModeSPacket& packet);

    /**
     * Ingests an All Call Reply packet and uses it to update the relevant aircraft. Exposed for testing, but usually
     * called by IngestDecodedModeSPacket.
     *
     * All call replies with an interrogator code of 0 (acquisition squitters) validate themselves. Replies to
     * interrogators with a nonzero code are only accepted once their ICAO address is in the dictionary.
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
            overflow_evictions_++;
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

    // One ICAO address can have up to three entries: Mode S (direct, ADS-R or TIS-B, see ModeSAircraft::AddressType),
    // UAT ADS-B, and UAT TIS-B/ADS-R. Address-keyed outputs (aircraft JSON, GDL90 traffic, ASTERIX) report one: the
    // best-ranked entry with a position newer than kPreferredReportPositionFreshMs, else the newest position. 15 s
    // covers a TIS-B radar scan.
    static constexpr uint32_t kPreferredReportPositionFreshMs = 15e3;
    enum AddressSourceRank : int8_t {
        kAddressSourceRankDirect = 0,       // DF17, DF18 CF=0, Mode S replies, or UAT ADS-B with an ICAO address.
        kAddressSourceRankRebroadcast = 1,  // 1090 ADS-R (DF18 CF=6) with an ICAO address.
        kAddressSourceRankTISB = 2,         // 1090 TIS-B, or UAT TIS-B/ADS-R, with an ICAO address.
        kAddressSourceRankUnique = -1       // Not an ICAO-addressed entry (Remote ID, track files, anonymous).
    };

    /**
     * Returns the source rank of a dictionary entry for de-duplicating reports of the same ICAO address.
     * @param[in] entry Dictionary entry.
     * @param[out] icao_address 24-bit ICAO address of the entry (only set if the rank isn't kAddressSourceRankUnique).
     * @retval AddressSourceRank of the entry.
     */
    static AddressSourceRank GetAddressSourceRank(const AircraftEntry& entry, uint32_t& icao_address);

    /**
     * Returns whether the entry with the given UID is the one to report for its ICAO address (see
     * kPreferredReportPositionFreshMs). Entries that aren't ICAO-addressed are always reported.
     * @param[in] uid UID of the entry.
     * @param[in] timestamp_ms Current local time, in ms.
     * @retval True if the entry should be reported, false if another entry for the same address is preferred.
     */
    bool IsPreferredReportForAddress(uint32_t uid, uint32_t timestamp_ms) const;

    /**
     * Resolves the aircraft report filters for a reporting round. kAltitudeFilterNearRx uses the receiver
     * altitude from SetReferenceAltitudeFt(); without one it filters nothing.
     * @param[in] max_distance_m Settings::FilterDistanceMeters(): largest distance to report, 0 for no filter.
     * @param[in] altitude_mode AltitudeFilterMode (Settings::filter_altitude_mode).
     * @param[in] altitude_a_ft First altitude argument, in feet.
     * @param[in] altitude_b_ft Second altitude argument, in feet.
     */
    AircraftReportFilter GetReportFilter(uint32_t max_distance_m, uint8_t altitude_mode, int32_t altitude_a_ft,
                                         int32_t altitude_b_ft) const {
        AircraftReportFilter filter;
        filter.max_distance_m = max_distance_m;
        filter.altitude = AltitudeFilterBounds::Of(altitude_mode, altitude_a_ft, altitude_b_ft,
                                                   reference_position_valid_ && reference_altitude_valid_,
                                                   reference_altitude_ft_);
        return filter;
    }

    // GetReportFilter() for a SettingsManager::Settings (either product's).
    template <typename SettingsT>
    AircraftReportFilter GetReportFilter(const SettingsT &settings) const {
        return GetReportFilter(settings.FilterDistanceMeters(), settings.filter_altitude_mode,
                               settings.filter_altitude_a_ft, settings.filter_altitude_b_ft);
    }

    /**
     * Lists the aircraft that aircraft reports (CSBee, MAVLink, GDL90, aircraft JSON) cover in a reporting round: every
     * tracked aircraft except those the report filter leaves out. Raw packets don't go through here and are never
     * filtered.
     * @param[out] uids Filled with up to max_uids aircraft UIDs.
     * @param[in] max_uids Capacity of uids.
     * @param[in] filter From GetReportFilter().
     * @retval Number of UIDs written.
     */
    uint16_t GetReportUIDs(uint32_t uids[], uint16_t max_uids, const AircraftReportFilter &filter) const {
        uint16_t count = 0;
        for (const auto &itr : dict) {
            if (count >= max_uids) {
                break;
            }
            if (!std::visit([&filter](const auto &a) { return filter.Excludes(a); }, itr.second)) {
                uids[count++] = itr.first;
            }
        }
        return count;
    }

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
    void ClearReferencePosition();

    /**
     * Sets the receiver altitude that AT+FILTER_ALTITUDE_FT's near-receiver mode compares against. It counts only
     * while the reference position is valid, and ClearReferencePosition() clears it.
     * @param[in] altitude_ft Receiver altitude, in feet.
     */
    inline void SetReferenceAltitudeFt(int32_t altitude_ft) {
        reference_altitude_ft_ = altitude_ft;
        reference_altitude_valid_ = true;
    }

    // Marks the receiver altitude as unknown: the near-receiver altitude filter then filters nothing.
    inline void ClearReferenceAltitude() { reference_altitude_valid_ = false; }

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

    // A full dictionary logs at most one warning per interval, with the number of aircraft it removed.
    static constexpr uint32_t kOverflowLogIntervalMs = 60e3;

    /**
     * Number of aircraft removed to make room since the last overflow warning, if a warning is due now; else 0.
     * Resets the count when it returns a non-zero value. Called by Update().
     * @param[in] timestamp_ms Current time in milliseconds.
     */
    uint32_t TakeOverflowEvictionsToLog(uint32_t timestamp_ms);

   private:
    uint32_t overflow_evictions_ = 0;
    uint32_t last_overflow_log_timestamp_ms_ = 0;
    bool overflow_logged_ = false;

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

        // Bucket buffer for the one bucket array reserve() allocates in the constructor: the next prime at or above
        // kMaxNumAircraft buckets, plus a margin.
        static constexpr size_t kBucketBufferSize = kMaxNumAircraft * sizeof(void*) + 512;

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
                // this. Fall through to the monotonic region so behavior remains defined.
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

    /**
     * Whether an extended squitter with corrected bits may update the dictionary: its aircraft must be tracked, a one
     * bit correction must not disagree with the track, and a two bit correction must agree with it.
     */
    bool AcceptCorrectedExtendedSquitter(const DecodedModeSPacket& packet);

    enum TrackFit : uint8_t { kTrackFits, kTrackDisagrees, kTrackUnknown };
    /**
     * Compares an extended squitter with the aircraft's callsign, altitude and position, or velocity.
     * @retval kTrackUnknown if the track has nothing recent to compare with.
     */
    TrackFit CheckAgainstTrack(const ModeSADSBPacket& es, const ModeSAircraft& aircraft);

    /**
     * Tries the one or two bit corrections of an address parity reply that give a tracked aircraft's address. Applies
     * the correction if exactly one aircraft fits, with a plausible altitude or the same squawk.
     * @retval True if the packet was corrected.
     */
    bool CorrectAddressParityPacket(DecodedModeSPacket& packet);

    AircraftDictionaryConfig config_;
    uint16_t address_parity_correction_attempts_ = 0;  // Since the last update.
    static constexpr uint16_t kUnlimitedAddressParityCorrections = UINT16_MAX;
    uint16_t address_parity_correction_budget_ = kUnlimitedAddressParityCorrections;  // Searches until the next limit.

    // Uncorrected addresses of recent replies that a fix in the address parity field would have matched.
    static constexpr uint16_t kAddressParityFieldNumBits = 24;
    static constexpr uint16_t kRecentAddressSlotBits = 7;
    static constexpr uint32_t kRecentAddressWindowMs = 60e3;
    static constexpr uint8_t kMaxUncorrectedAddressRepeats = 2;  // From the second reply on, it's another aircraft.
    struct RecentAddress {
        uint32_t address_and_count = 0;  // Address in the low 24 bits, repeats in the top 8.
        uint32_t last_ms = 0;
    };
    RecentAddress recent_uncorrected_addresses_[1 << kRecentAddressSlotBits];

    /**
     * Records an uncorrected address and returns whether it has repeated kMaxUncorrectedAddressRepeats times within
     * kRecentAddressWindowMs.
     */
    bool RepeatsAsUncorrectedAddress(uint32_t address, uint32_t now_ms);
    // Counters in metrics_counter_ are incremented, then metrics_counter_ is swapped into metrics during the dictionary
    // update. This ensures that the public metrics struct always has valid data.
    Metrics metrics_counter_;

    // Reference position for decoding Mode S surface position packets, in angular weighted binary format.
    uint32_t reference_latitude_awb32_ = 0;
    uint32_t reference_longitude_awb32_ = 0;
    bool reference_position_valid_ = false;  // Set by SetReferencePosition, cleared by ClearReferencePosition.
    int32_t reference_altitude_ft_ = 0;
    bool reference_altitude_valid_ = false;  // Set by SetReferenceAltitudeFt, cleared by ClearReferencePosition.
    // Receiver position for aircraft distance and bearing. Moving it more than kReferenceMoveThresholdMeters or
    // clearing it bumps reference_epoch_, and Update() recomputes every aircraft computed in an older epoch or marked
    // stale by a move.
    static constexpr uint32_t kReferenceMoveThresholdMeters = 500;
    // Epoch value that marks an aircraft's distance as stale; reference_epoch_ never takes it.
    static constexpr uint16_t kDistanceEpochStale = UINT16_MAX;
    GeoReference distance_reference_;
    uint16_t reference_epoch_ = 0;

    // Position and validity of an aircraft before a packet is applied, to tell whether its distance needs updating.
    struct PositionSnapshot {
        float latitude_deg;
        float longitude_deg;
        bool valid;
        template <typename T>
        static PositionSnapshot Of(const T &aircraft) {
            return {aircraft.latitude_deg, aircraft.longitude_deg, aircraft.HasBitFlag(T::kBitFlagPositionValid)};
        }
    };

    /**
     * After a packet: an aircraft's first position (or a lost one) updates its distance at once, so the distance filter
     * applies to new aircraft right away. Later moves only mark the distance stale, and Update() recomputes it once a
     * second: the cost follows the number of aircraft, not the position message rate.
     */
    template <typename T>
    void UpdateDistanceAndBearingIfMoved(T &aircraft, const PositionSnapshot &before, bool position_message = false) {
        PositionSnapshot after = PositionSnapshot::Of(aircraft);
        bool record_in_place = false;
        if constexpr (kRecordsRange<T>) {
            bool moved = after.latitude_deg != before.latitude_deg || after.longitude_deg != before.longitude_deg;
            // Remote ID repeats a hovering drone's position unchanged, so every Location message with one counts.
            bool decoded = std::is_same_v<T, RemoteIDAircraft> ? position_message : !before.valid || moved;
            if (!after.valid) {
                aircraft.num_position_decodes = 0;
            } else if (decoded && aircraft.num_position_decodes < 2) {
                record_in_place = ++aircraft.num_position_decodes == 2;
            }
            // A confirmed drone counts in every period it is heard, hovering or not.
            if constexpr (std::is_same_v<T, RemoteIDAircraft>) {
                record_in_place = decoded && aircraft.num_position_decodes == 2;
            }
        }
        if (after.valid != before.valid || (after.valid && aircraft.distance_m == Aircraft::kDistanceUnknown)) {
            UpdateDistanceAndBearing(aircraft, after.valid);
            RecordRange(aircraft);
        } else if (after.latitude_deg != before.latitude_deg || after.longitude_deg != before.longitude_deg) {
            aircraft.distance_reference_epoch = kDistanceEpochStale;
        } else if (record_in_place && aircraft.distance_reference_epoch == reference_epoch_) {
            RecordRange(aircraft);  // Confirmed or heard again in place: the distance won't be recomputed.
        }
    }

    // Mode S, UAT and Remote ID positions go into one swarm range map each. Only builds that report swarm metrics (the
    // ESP32, host tests) define SWARM_RECORDING: elsewhere this compiles to nothing.
#ifdef SWARM_RECORDING
    static constexpr bool kSwarmRecording = true;
#else
    static constexpr bool kSwarmRecording = false;
#endif
    template <typename T>
    static constexpr bool kRecordsRange =
        kSwarmRecording && (std::is_same_v<T, ModeSAircraft> || std::is_same_v<T, UATAircraft> ||
                            std::is_same_v<T, RemoteIDAircraft>);

    /**
     * Adds a freshly computed distance to the protocol's swarm range map, once the position is confirmed by a second
     * decode (the first can be a CPR outlier). Only positions heard directly. Aircraft altitude is barometric, else
     * GNSS; drones need a geodetic altitude. Those without one are left out.
     */
    template <typename T>
    inline void RecordRange(const T &aircraft) {
        if constexpr (kRecordsRange<T>) {
            if (swarm_recorder_ == nullptr || aircraft.num_position_decodes < 2 ||
                aircraft.distance_m == Aircraft::kDistanceUnknown) {
                return;
            }
            if constexpr (std::is_same_v<T, RemoteIDAircraft>) {
                if (aircraft.HasBitFlag(T::kBitFlagGNSSAltitudeValid)) {
                    swarm_recorder_->RecordDrone(aircraft.distance_m, aircraft.bearing_bam16, aircraft.gnss_altitude_ft,
                                                 aircraft.GetUID());
                }
            } else {
                if (!IsDirectPosition(aircraft)) return;
                int32_t altitude_ft;
                if (aircraft.HasBitFlag(T::kBitFlagBaroAltitudeValid)) {
                    altitude_ft = aircraft.baro_altitude_ft;
                } else if (aircraft.HasBitFlag(T::kBitFlagGNSSAltitudeValid)) {
                    altitude_ft = aircraft.gnss_altitude_ft;
                } else {
                    return;
                }
                swarm_recorder_->RecordPosition(
                    std::is_same_v<T, ModeSAircraft> ? SwarmRecorder::kModeS : SwarmRecorder::kUAT,
                    aircraft.distance_m, aircraft.bearing_bam16, altitude_ft, aircraft.GetUID());
            }
        }
    }

    SwarmRecorder *swarm_recorder_ = nullptr;

    void BumpReferenceEpoch();

    /**
     * Recomputes an aircraft's distance and bearing from the receiver.
     * @param[in,out] aircraft Aircraft to update.
     * @param[in] position_valid Whether the aircraft's latitude_deg and longitude_deg hold a position.
     */
    void UpdateDistanceAndBearing(Aircraft &aircraft, bool position_valid);

    FixedPoolResource pool_;  // Free-list pool + monotonic bucket region; must outlive dict.
};