#pragma once
#include "stdint.h"

/**
 * This is a collection of enums that is common across ModeSAircraft, UATAircraft, and others.
 */
class ADSBTypes {
   public:
    enum AirGroundState : uint8_t {
        kAirGroundStateAirborneSubsonic = 0,    // Ground state.
        kAirGroundStateAirborneSupersonic = 1,  // Airborne state.
        kAirGroundStateOnGround = 2,            // Reserved for future use.
        kAirGroundStateReserved = 3             // Invalid state.
    };

    enum AltitudeSource : int16_t {
        kAltitudeSourceNotAvailable = -2,
        kAltitudeSourceNotSet = -1,
        kAltitudeSourceBaro = 0,
        kAltitudeSourceGNSS = 1
    };

    enum AVDimensionsType : uint8_t { kAVDimensionsTypeAVLengthWidth = 0, kAVDimensionsTypeGNSSSensorOffset = 1 };

    enum EmitterCategory : int8_t {
        kEmitterCategoryInvalid = -1,
        kEmitterCategoryNoCategoryInfo = 0,
        kEmitterCategoryLight = 1,    // < 7000kg
        kEmitterCategoryMedium1 = 2,  // 7000kg - 34000kg
        kEmitterCategoryMedium2 = 3,  // 34000kg - 136000kg
        kEmitterCategoryHighVortexAircraft = 4,
        kEmitterCategoryHeavy = 5,            // > 136000kg
        kEmitterCategoryHighPerformance = 6,  // >5g acceleration and high speed
        kEmitterCategoryRotorcraft = 7,
        kEmitterCategoryReserved = 8,
        kEmitterCategoryGliderSailplane = 9,
        kEmitterCategoryLighterThanAir = 10,
        kEmitterCategoryParachutistSkydiver = 11,
        kEmitterCategoryUltralightHangGliderParaglider = 12,
        kEmitterCategoryReserved1 = 13,
        kEmitterCategoryUnmannedAerialVehicle = 14,
        kEmitterCategorySpaceTransatmosphericVehicle = 15,
        kEmitterCategoryReserved2 = 16,  // Reserved for future use.
        kEmitterCategorySurfaceEmergencyVehicle = 17,
        kEmitterCategorySurfaceServiceVehicle = 18,
        kEmitterCategoryPointObstacle = 19,
        kEmitterCategoryClusterObstacle = 20,
        kEmitterCategoryLineObstacle = 21
    };

    enum DirectionType : uint8_t {
        kDirectionTypeNotAvailable = 0,
        kDirectionTypeTrueTrackAngle = 1,
        kDirectionTypeMagneticHeading = 2,
        kDirectionTypeTrueHeading = 3
    };

    enum GVA : uint8_t {
        kGVAUnknownOrGreaterThan150Meters = 0,
        GVALessThanOrEqualTo150Meters = 1,
        GVALessThanOrEqualTo45Meters = 2,
        GVALessThan45Meters = 3
    };

    enum NACEstimatedPositionUncertainty : uint8_t {
        kEPUUnknownOrGreaterThanOrEqualTo10NauticalMiles = 0,
        kEPULessThan10NauticalMiles = 1,
        kEPULessThan4NauticalMiles = 2,
        kEPULessThan2NauticalMiles = 3,
        kEPULessThan1NauticalMile = 4,
        kEPULessThan0p5NauticalMiles = 5,
        kEPULessThan0p3NauticalMiles = 6,
        kEPULessThan0p1NauticalMiles = 7,
        kEPULessThan0p05NauticalMiles = 8,
        kEPULessThan30Meters = 9,
        kEPULessThan10Meters = 10,
        kEPULessThan3Meters = 11
    };

    enum NACHorizontalVelocityError : uint8_t {
        kHVEUnknownOrGreaterThanOrEqualTo10MetersPerSecond = 0b000,
        kHVELessThan10MetersPerSecond = 0b001,
        kHVELessThan3MetersPerSecond = 0b010,
        kHVELessThan1MeterPerSecond = 0b011,
        kHVELessThan0p3MetersPerSecond = 0b100
    };

    // NIC supplement D (version 3 only) is a 2-bit value stored as kNICBitD0 (LSB) and kNICBitD1 (MSB).
    enum NICBit : uint16_t { kNICBitA = 0, kNICBitB = 1, kNICBitC = 2, kNICBitD0 = 3, kNICBitD1 = 4 };

    enum NICRadiusOfContainment : uint8_t {
        kROCUnknown = 0,
        kROCLessThan20NauticalMiles = 1,
        kROCLessThan8NauticalMiles = 2,
        kROCLessThan4NauticalMiles = 3,
        kROCLessThan2NauticalMiles = 4,
        kROCLessThan1NauticalMile = 5,
        kROCLessThan0p6NauticalMiles = 6,  // Lump together with <0.5NM and <0.3NM since they share a NIC value.
        kROCLessThan0p2NauticalMiles = 7,
        kROCLessThan0p1NauticalMiles = 8,
        kROCLessThan75Meters = 9,
        kROCLessThan25Meters = 10,
        kROCLessThan7p5Meters = 11
    };

    enum NICBarometricAltitudeIntegrity : uint8_t {
        kBAIGillhamInputNotCrossChecked = 0,
        kBAIGillHamInputCrossCheckedOrNonGillhamSource = 1
    };

    enum SpeedSource : int16_t {
        kSpeedSourceNotAvailable = -2,
        kSpeedSourceNotSet = -1,
        kSpeedSourceGroundSpeed = 0,
        kSpeedSourceAirspeedTrue = 1,
        kSpeedSourceAirspeedIndicated = 2
    };

    // This composite SIL value is SIL | (SIL_supplement << 2).
    enum SILProbabilityOfExceedingNICRadiusOfContainmnent : uint8_t {
        kPOERCUnknownOrGreaterThan1em3PerFlightHour = 0b000,
        kPOERCLessThanOrEqualTo1em3PerFligthHour = 0b001,
        kPOERCLessThanOrEqualTo1em5PerFlightHour = 0b010,
        kPOERCLessThanOrEqualTo1em7PerFlightHour = 0b011,
        kPOERCUnknownOrGreaterThan1em3PerSample = 0b100,
        kPOERCLessThanOrEqualTo1em3PerSample = 0b101,
        kPOERCLessThanOrEqualTo1em5PerSample = 0b110,
        kPOERCLessThanOrEqualTo1em7PerSample = 0b111,
    };

    enum SquawkCode : uint16_t {
        kSquawkCodeNotYetReceived = UINT16_MAX,
    };

    enum SystemDesignAssurance : uint8_t {
        kSDASupportedFailureUnknownOrNoSafetyEffect = 0b00,
        kSDASupportedFailureMinor = 0b01,
        kSDASupportedFailureMajor = 0b10,
        kSDASupportedFailureHazardous = 0b11
    };

    enum VerticalRateSource : int16_t {
        kVerticalRateSourceNotAvailable = -2,
        kVerticalRateSourceNotSet = -1,
        kVerticalRateSourceGNSS = 0,
        kVerticalRateSourceBaro = 1
    };

    /**
     * Checks if the air-ground state indicates that the aircraft is airborne.
     * @param air_ground_state Air-ground state from the UAT state vector.
     * @return True if the aircraft is airborne, false otherwise.
     */
    static inline bool AirGroundStateIsAirborne(ADSBTypes::AirGroundState air_ground_state) {
        return (air_ground_state & 0b10) == 0;
    }

    /**
     * Checks if the air-ground state indicates that the aircraft is supersonic.
     * @param air_ground_state Air-ground state from the UAT state vector.
     * @return True if the aircraft is supersonic, false otherwise.
     */
    static inline bool AirGroundStateIsSupersonic(ADSBTypes::AirGroundState air_ground_state) {
        return (air_ground_state & 0b1) == 1;
    }
};