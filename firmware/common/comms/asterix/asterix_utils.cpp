#include "asterix_utils.hh"

#include <cmath>
#include <cstring>

namespace {

// Bounded big-endian writer; any overflow makes the whole record fail.
class ByteWriter {
   public:
    ByteWriter(uint8_t* buf, uint16_t buf_len) : buf_(buf), buf_len_(buf_len) {}

    void U8(uint32_t value) {
        if (len_ >= buf_len_) {
            overflow_ = true;
            return;
        }
        buf_[len_++] = static_cast<uint8_t>(value);
    }
    void U16(uint32_t value) {
        U8(value >> 8);
        U8(value);
    }
    void U24(uint32_t value) {
        U8(value >> 16);
        U16(value);
    }
    void U32(uint32_t value) {
        U16(value >> 16);
        U16(value);
    }
    // Reserves bytes to fill in later with At().
    uint16_t Skip(uint16_t num_bytes) {
        uint16_t start = len_;
        for (uint16_t i = 0; i < num_bytes; i++) U8(0);
        return start;
    }
    uint8_t& At(uint16_t index) { return buf_[index]; }

    // Writes an extended item: octets after the first are dropped from the end while zero, and FX links the rest.
    void Extended(uint8_t* octets, uint8_t num_octets) {
        uint8_t used = num_octets;
        while (used > 1 && (octets[used - 1] & 0xFE) == 0) used--;
        for (uint8_t i = 0; i < used; i++) {
            U8((octets[i] & 0xFE) | (i + 1 < used ? 0x01 : 0x00));
        }
    }

    uint16_t Len() const { return overflow_ ? 0 : len_; }

   private:
    uint8_t* buf_;
    uint16_t buf_len_;
    uint16_t len_ = 0;
    bool overflow_ = false;
};

// Field Specification: one presence bit per Field Reference Number (FRN), seven per octet, FX in bit 1.
class FSPEC {
   public:
    static constexpr uint8_t kMaxNumOctets = 7;

    void Set(uint8_t frn) {
        uint8_t octet = (frn - 1) / 7;
        octets_[octet] |= 0x80 >> ((frn - 1) % 7);
        if (octet + 1 > num_octets_) num_octets_ = octet + 1;
    }
    void Write(ByteWriter& writer) const {
        for (uint8_t i = 0; i < num_octets_; i++) {
            writer.U8(octets_[i] | (i + 1 < num_octets_ ? 0x01 : 0x00));
        }
    }

   private:
    uint8_t octets_[kMaxNumOctets] = {0};
    uint8_t num_octets_ = 1;
};

// Two's complement of value in a field of num_bits.
uint32_t Signed(int32_t value, uint8_t num_bits) {
    return num_bits >= 32 ? static_cast<uint32_t>(value) : static_cast<uint32_t>(value) & ((1u << num_bits) - 1);
}

int32_t RoundToInt(double value) { return static_cast<int32_t>(lround(value)); }

// An AWB angle in I021/131 units (180 / 2^30 degrees), rounded half up.
int32_t AWBToAsterixHighRes(uint32_t awb) { return (static_cast<int32_t>(awb) >> 1) + static_cast<int32_t>(awb & 1); }

// Ground speed and indicated airspeed share an LSB of 2^-14 NM/s.
uint32_t KnotsToNMPerSecond14(float kts) { return static_cast<uint32_t>(lround(kts * 16384.0 / 3600.0)); }

uint32_t DegreesToAngle16(float deg) {
    double wrapped = fmod(deg, 360.0);
    if (wrapped < 0) wrapped += 360.0;
    return static_cast<uint32_t>(lround(wrapped * 65536.0 / 360.0)) & 0xFFFF;
}

// Vertical rates (I021/155, I021/157): 15-bit signed, LSB 6.25 ft/min, range exceeded flag in bit 16.
uint16_t VerticalRate(int32_t fpm) {
    int32_t raw = RoundToInt(fpm / 6.25);
    bool range_exceeded = raw > 0x3FFF || raw < -0x4000;
    if (raw > 0x3FFF) raw = 0x3FFF;
    if (raw < -0x4000) raw = -0x4000;
    return (range_exceeded ? 0x8000 : 0) | Signed(raw, 15);
}

uint8_t CallsignCharTo6Bit(char c) {
    if (c >= 'A' && c <= 'Z') return c - 'A' + 1;
    if (c >= 'a' && c <= 'z') return c - 'a' + 1;
    if (c >= '0' && c <= '9') return c;  // 48 to 57.
    return 32;                           // Space, also for characters ICAO doesn't define.
}

bool CallsignIsSet(const char* callsign) { return callsign[0] != '\0' && callsign[0] != '?'; }

// Priority status (I021/200 PS) of the emergency squawks.
uint8_t SquawkToPriorityStatus(uint16_t squawk) {
    switch (squawk) {
        case 7500:
            return 5;  // Unlawful interference.
        case 7600:
            return 4;  // No communications.
        case 7700:
            return 1;  // General emergency.
        default:
            return 0;
    }
}

void FillQualityIndicators(ASTERIXReporter::CAT021TargetReport& report, uint8_t nac_v, uint8_t nic, uint8_t nic_baro,
                           uint8_t sil_and_supplement, uint8_t nac_p, uint8_t sda, uint8_t gva) {
    report.nac_v = nac_v & 0x7;
    report.nic = nic & 0xF;
    report.nic_baro = nic_baro & 0x1;
    report.sil = sil_and_supplement & 0x3;
    report.sil_supplement = (sil_and_supplement >> 2) & 0x1;
    report.nac_p = nac_p & 0xF;
    report.sda = sda & 0x3;
    report.gva = gva & 0x3;
    report.pic = ASTERIXReporter::NICToPIC(report.nic);
}

}  // namespace

uint32_t ASTERIXReporter::Clock::TimeOfDay(uint32_t local_ms) const {
    int64_t ms = (static_cast<int64_t>(local_ms) + utc_minus_local_ms) % kMsPerDay;
    if (ms < 0) ms += kMsPerDay;
    uint32_t ticks = static_cast<uint32_t>((ms * kTimeOfDayTicksPerSecond + 500) / 1000);
    return ticks >= kMsPerDay / 1000 * kTimeOfDayTicksPerSecond ? 0 : ticks;  // Rounding up at midnight wraps to 0.
}

bool ASTERIXReporter::TrackNumbers::Get(uint32_t uid, uint16_t& track_number) {
    uint16_t start = static_cast<uint16_t>(((uid ^ (uid >> 12)) * 2654435761u) % kNumSlots);
    int32_t free_slot = -1;
    for (uint16_t probe = 0; probe < kNumSlots; probe++) {
        uint16_t i = (start + probe) % kNumSlots;
        Slot& slot = slots_[i];
        if (slot.used && slot.uid == uid) {
            slot.last_round = round_;
            track_number = i;
            return true;
        }
        if (!slot.used) {
            if (free_slot < 0) free_slot = i;
            break;  // The UID would have been placed at or before the first never-used slot.
        }
        if (free_slot < 0 && static_cast<uint16_t>(round_ - slot.last_round) > kStaleRounds) {
            free_slot = i;  // Stale slots keep their UID so that probing past them still finds later entries.
        }
    }
    if (free_slot < 0) return false;
    slots_[free_slot] = {.uid = uid, .last_round = round_, .used = true};
    track_number = static_cast<uint16_t>(free_slot);
    return true;
}

uint8_t ASTERIXReporter::EmitterCategoryToECAT(ADSBTypes::EmitterCategory category) {
    switch (category) {
        case ADSBTypes::kEmitterCategoryLight:
            return 1;
        case ADSBTypes::kEmitterCategoryMedium1:
            return 2;
        case ADSBTypes::kEmitterCategoryMedium2:
            return 3;
        case ADSBTypes::kEmitterCategoryHighVortexAircraft:
            return 4;
        case ADSBTypes::kEmitterCategoryHeavy:
            return 5;
        case ADSBTypes::kEmitterCategoryHighPerformance:
            return 6;
        case ADSBTypes::kEmitterCategoryRotorcraft:
            return 10;
        case ADSBTypes::kEmitterCategoryGliderSailplane:
            return 11;
        case ADSBTypes::kEmitterCategoryLighterThanAir:
            return 12;
        case ADSBTypes::kEmitterCategoryUnmannedAerialVehicle:
            return 13;
        case ADSBTypes::kEmitterCategorySpaceTransatmosphericVehicle:
            return 14;
        case ADSBTypes::kEmitterCategoryUltralightHangGliderParaglider:
            return 15;
        case ADSBTypes::kEmitterCategoryParachutistSkydiver:
            return 16;
        case ADSBTypes::kEmitterCategorySurfaceEmergencyVehicle:
            return 20;
        case ADSBTypes::kEmitterCategorySurfaceServiceVehicle:
            return 21;
        case ADSBTypes::kEmitterCategoryPointObstacle:
            return 22;
        case ADSBTypes::kEmitterCategoryClusterObstacle:
            return 23;
        case ADSBTypes::kEmitterCategoryLineObstacle:
            return 24;
        default:
            return 0;
    }
}

uint8_t ASTERIXReporter::NICToPIC(uint8_t nic) {
    // Index: NIC. NIC 6 can mean 0.3, 0.5 or 0.6 NM; use 0.6 NM (PIC 7).
    static const uint8_t kPICForNIC[12] = {0, 1, 3, 4, 5, 6, 7, 10, 11, 12, 13, 14};
    return nic < sizeof(kPICForNIC) ? kPICForNIC[nic] : 0;
}

bool ASTERIXReporter::SquawkToMode3A(uint16_t squawk, uint16_t& mode_3a) {
    if (squawk > 7777) return false;
    mode_3a = 0;
    uint16_t divisor = 1000;
    for (uint8_t i = 0; i < 4; i++) {
        uint16_t digit = (squawk / divisor) % 10;
        if (digit > 7) return false;
        mode_3a = (mode_3a << 3) | digit;
        divisor /= 10;
    }
    return true;
}

bool ASTERIXReporter::BuildCAT021Report(CAT021TargetReport& report, const ModeSAircraft& aircraft,
                                        const ReportContext& ctx) {
    ModeSAircraft::AddressType address_type = aircraft.GetAddressType();
    if (!aircraft.HasBitFlag(ModeSAircraft::kBitFlagPositionValid)) {
        return false;
    }
    report = CAT021TargetReport();
    report.sac = ctx.sac;
    report.sic = ctx.sic;
    report.no_go = !ctx.clock.utc_valid;
    report.address = aircraft.icao_address & 0xFFFFFF;
    // DF18 CF=1 and the other non-ICAO addresses are anonymous (ATP 3), as the I021/040 notes recommend.
    report.address_type =
        address_type >= ModeSAircraft::kFirstNonICAOAddressType ? kAddressTypeAnonymous : kAddressTypeICAO;
    report.altitude_reporting_capability = kARCUnknown;  // The altitude's Q bit isn't kept.
    report.on_ground = aircraft.IsOnGround();
    report.has_service_id = true;
    report.service_id = ModeSAircraft::IsRebroadcast(address_type) ? kServiceID1090ESRebroadcast : kServiceID1090ES;

    report.has_position = true;
    report.latitude_awb = aircraft.latitude_awb;
    report.longitude_awb = aircraft.longitude_awb;
    report.position_time_of_day = ctx.clock.TimeOfDay(aircraft.last_position_update_ms);

    if (aircraft.HasBitFlag(ModeSAircraft::kBitFlagGNSSAltitudeValid)) {
        report.has_geometric_height = true;
        report.geometric_height_ft = aircraft.gnss_altitude_ft;
    }
    if (aircraft.HasBitFlag(ModeSAircraft::kBitFlagBaroAltitudeValid)) {
        report.has_flight_level = true;
        report.baro_altitude_ft = aircraft.baro_altitude_ft;
    }

    FillQualityIndicators(report, aircraft.navigation_accuracy_category_velocity,
                          aircraft.navigation_integrity_category, aircraft.navigation_integrity_category_baro,
                          aircraft.surveillance_integrity_level, aircraft.navigation_accuracy_category_position,
                          aircraft.system_design_assurance, aircraft.geometric_vertical_accuracy);
    // TIS-B messages carry no ADS-B version.
    if (!ModeSAircraft::IsTISB(address_type) && aircraft.adsb_version >= 0 && aircraft.adsb_version <= 3) {
        report.has_mops_version = true;
        report.mops_version = aircraft.adsb_version;
        report.link_technology = kLinkTechnology1090ES;
    }

    if (aircraft.squawk != ADSBTypes::kSquawkCodeNotYetReceived) {
        report.has_squawk = true;
        report.squawk = aircraft.squawk;
    }

    // Airborne velocity only: surface movement belongs in the Reserved Expansion Field, which isn't written.
    if (!report.on_ground && aircraft.last_velocity_update_ms != 0) {
        bool speed_valid = aircraft.HasBitFlag(ModeSAircraft::kBitFlagHorizontalSpeedValid);
        bool direction_valid = aircraft.HasBitFlag(ModeSAircraft::kBitFlagDirectionValid);
        bool direction_is_heading = aircraft.HasBitFlag(ModeSAircraft::kBitFlagDirectionIsHeading);
        if (aircraft.speed_source == ADSBTypes::kSpeedSourceGroundSpeed && speed_valid && direction_valid &&
            !direction_is_heading) {
            report.has_ground_vector = true;
            report.ground_speed_kts = aircraft.speed_kts;
            report.track_angle_deg = aircraft.direction_deg;
        } else if (aircraft.speed_source == ADSBTypes::kSpeedSourceAirspeedIndicated && speed_valid) {
            report.has_indicated_airspeed = true;
            report.indicated_airspeed_kts = aircraft.speed_kts;
        } else if (aircraft.speed_source == ADSBTypes::kSpeedSourceAirspeedTrue && speed_valid) {
            report.has_true_airspeed = true;
            report.true_airspeed_kts = aircraft.speed_kts;
        }
        if (direction_valid && direction_is_heading &&
            aircraft.HasBitFlag(ModeSAircraft::kBitFlagHeadingUsesMagneticNorth)) {
            report.has_magnetic_heading = true;
            report.magnetic_heading_deg = aircraft.direction_deg;
        }
        if (aircraft.HasBitFlag(ModeSAircraft::kBitFlagBaroVerticalRateValid)) {
            report.has_baro_vertical_rate = true;
            report.baro_vertical_rate_fpm = aircraft.baro_vertical_rate_fpm;
        }
        if (aircraft.HasBitFlag(ModeSAircraft::kBitFlagGNSSVerticalRateValid)) {
            report.has_geometric_vertical_rate = true;
            report.geometric_vertical_rate_fpm = aircraft.gnss_vertical_rate_fpm;
        }
        report.has_velocity_time = report.has_ground_vector || report.has_indicated_airspeed ||
                                   report.has_true_airspeed || report.has_magnetic_heading ||
                                   report.has_baro_vertical_rate || report.has_geometric_vertical_rate;
        report.velocity_time_of_day = ctx.clock.TimeOfDay(aircraft.last_velocity_update_ms);
    }

    uint8_t priority_status = SquawkToPriorityStatus(aircraft.squawk);
    uint8_t surveillance_status = 0;
    if (aircraft.HasBitFlag(ModeSAircraft::kBitFlagIdent)) {
        surveillance_status = 3;
    } else if (aircraft.HasBitFlag(ModeSAircraft::kBitFlagAlert)) {
        surveillance_status = priority_status != 0 ? 1 : 2;
    }
    if (priority_status != 0 || surveillance_status != 0) {
        report.has_target_status = true;
        report.priority_status = priority_status;
        report.surveillance_status = surveillance_status;
    }

    if (CallsignIsSet(aircraft.callsign)) {
        report.has_callsign = true;
        memcpy(report.callsign, aircraft.callsign, sizeof(report.callsign) - 1);  // Writer stops at a NUL.
    }
    if (aircraft.emitter_category != ADSBTypes::kEmitterCategoryInvalid) {
        report.has_emitter_category = true;
        report.emitter_category = EmitterCategoryToECAT(aircraft.emitter_category);
    }
    report.has_amplitude = true;
    report.amplitude_dbm = aircraft.last_message_signal_strength_dbm;
    report.tcas_ra_active = aircraft.HasBitFlag(ModeSAircraft::kBitFlagTCASRA);
    report.single_antenna = aircraft.HasBitFlag(ModeSAircraft::kBitFlagSingleAntenna);
    report.has_report_time = true;
    report.report_time_of_day = ctx.clock.TimeOfDay(ctx.timestamp_ms);
    return true;
}

bool ASTERIXReporter::BuildCAT021Report(CAT021TargetReport& report, const UATAircraft& aircraft,
                                        const ReportContext& ctx) {
    if (!aircraft.HasBitFlag(UATAircraft::kBitFlagPositionValid)) {
        return false;
    }
    report = CAT021TargetReport();
    switch (aircraft.GetAddressQualifier()) {
        case UATAircraft::kADSBTargetWithICAO24BitAddress:
            report.address_type = kAddressTypeICAO;
            break;
        case UATAircraft::kADSBTargetWithSelfAssignedTemporaryAddress:
            report.address_type = kAddressTypeAnonymous;
            break;
        case UATAircraft::kSurfaceVehicle:
            report.address_type = kAddressTypeSurfaceVehicle;
            break;
        case UATAircraft::kFixedADSBBeacon:
            report.address_type = kAddressTypeICAO;
            report.from_field_monitor = true;
            break;
        case UATAircraft::kTISBTargetWithICAO24BitAddress:  // TIS-B or ADS-R.
            report.address_type = kAddressTypeICAO;
            break;
        case UATAircraft::kTISBTargetWithTrackFileIdentifier:
        case UATAircraft::kADSRTargetWithNonICAOAddress:
            report.address_type = kAddressTypeAnonymous;
            break;
        default:
            return false;  // Reserved address qualifier.
    }
    UATAircraft::AddressQualifier qualifier = aircraft.GetAddressQualifier();
    bool rebroadcast = qualifier == UATAircraft::kTISBTargetWithICAO24BitAddress ||
                       qualifier == UATAircraft::kTISBTargetWithTrackFileIdentifier ||
                       qualifier == UATAircraft::kADSRTargetWithNonICAOAddress;
    report.sac = ctx.sac;
    report.sic = ctx.sic;
    report.no_go = !ctx.clock.utc_valid;
    report.address = aircraft.icao_address & 0xFFFFFF;
    report.altitude_reporting_capability = kARC25Ft;  // UAT altitudes have 25 ft steps.
    report.on_ground = !aircraft.HasBitFlag(UATAircraft::kBitFlagIsAirborne);
    report.has_service_id = true;
    report.service_id = rebroadcast ? kServiceIDUATRebroadcast : kServiceIDUAT;

    report.has_position = true;
    report.latitude_awb = aircraft.latitude_awb;
    report.longitude_awb = aircraft.longitude_awb;
    report.position_time_of_day = ctx.clock.TimeOfDay(aircraft.last_position_update_ms);

    if (aircraft.HasBitFlag(UATAircraft::kBitFlagGNSSAltitudeValid)) {
        report.has_geometric_height = true;
        report.geometric_height_ft = aircraft.gnss_altitude_ft;
    }
    if (aircraft.HasBitFlag(UATAircraft::kBitFlagBaroAltitudeValid)) {
        report.has_flight_level = true;
        report.baro_altitude_ft = aircraft.baro_altitude_ft;
    }

    FillQualityIndicators(report, aircraft.navigation_accuracy_category_velocity,
                          aircraft.navigation_integrity_category, aircraft.navigation_integrity_category_baro,
                          aircraft.surveillance_integrity_level, aircraft.navigation_accuracy_category_position,
                          aircraft.system_design_assurance, aircraft.geometric_vertical_accuracy);
    if (aircraft.uat_version >= 0) {
        // UAT MOPS versions DO-282, DO-282A and DO-282B line up with the 1090ES versions 0 to 2.
        report.has_mops_version = true;
        report.mops_version = aircraft.uat_version > 2 ? 2 : aircraft.uat_version;
        report.link_technology = kLinkTechnologyUAT;
    }

    if (aircraft.squawk != ADSBTypes::kSquawkCodeNotYetReceived) {
        report.has_squawk = true;
        report.squawk = aircraft.squawk;
    }

    if (!report.on_ground && aircraft.last_velocity_update_ms != 0) {
        if (aircraft.HasBitFlag(UATAircraft::kBitFlagHorizontalSpeedValid) &&
            aircraft.HasBitFlag(UATAircraft::kBitFlagDirectionValid) &&
            !aircraft.HasBitFlag(UATAircraft::kBitFlagDirectionIsHeading)) {
            report.has_ground_vector = true;
            report.ground_speed_kts = aircraft.speed_kts;
            report.track_angle_deg = aircraft.direction_deg;
        }
        if (aircraft.HasBitFlag(UATAircraft::kBitFlagBaroVerticalRateValid)) {
            report.has_baro_vertical_rate = true;
            report.baro_vertical_rate_fpm = aircraft.baro_vertical_rate_fpm;
        }
        if (aircraft.HasBitFlag(UATAircraft::kBitFlagGNSSVerticalRateValid)) {
            report.has_geometric_vertical_rate = true;
            report.geometric_vertical_rate_fpm = aircraft.gnss_vertical_rate_fpm;
        }
        report.has_velocity_time =
            report.has_ground_vector || report.has_baro_vertical_rate || report.has_geometric_vertical_rate;
        report.velocity_time_of_day = ctx.clock.TimeOfDay(aircraft.last_velocity_update_ms);
    }

    uint8_t priority_status = aircraft.emergency_priority_status <= UATAircraft::kEmergencyStatusDownedAircraft
                                  ? aircraft.emergency_priority_status
                                  : 0;
    uint8_t surveillance_status = aircraft.HasBitFlag(UATAircraft::kBitFlagIdent) ? 3 : 0;
    if (priority_status != 0 || surveillance_status != 0) {
        report.has_target_status = true;
        report.priority_status = priority_status;
        report.surveillance_status = surveillance_status;
    }

    if (CallsignIsSet(aircraft.callsign)) {
        report.has_callsign = true;
        memcpy(report.callsign, aircraft.callsign, sizeof(report.callsign) - 1);  // Writer stops at a NUL.
    }
    if (aircraft.emitter_category != ADSBTypes::kEmitterCategoryInvalid) {
        report.has_emitter_category = true;
        report.emitter_category = EmitterCategoryToECAT(aircraft.emitter_category);
    }
    report.has_amplitude = true;
    report.amplitude_dbm = aircraft.last_message_signal_strength_dbm;
    report.tcas_ra_active = aircraft.HasBitFlag(UATAircraft::kBitFlagTCASRA);
    report.has_report_time = true;
    report.report_time_of_day = ctx.clock.TimeOfDay(ctx.timestamp_ms);
    return true;
}

uint16_t ASTERIXReporter::WriteCAT021Record(uint8_t* buf, uint16_t buf_len, const CAT021TargetReport& r) {
    // Field Reference Numbers of the CAT021 standard User Application Profile.
    enum FRN : uint8_t {
        kFRNDataSource = 1,           // I021/010
        kFRNTargetReportDescriptor,   // I021/040
        kFRNTrackNumber,              // I021/161
        kFRNServiceID,                // I021/015
        kFRNPositionTimeApplicable,   // I021/071
        kFRNPosition,                 // I021/130
        kFRNPositionHighRes,          // I021/131
        kFRNVelocityTimeApplicable,   // I021/072
        kFRNAirSpeed,                 // I021/150
        kFRNTrueAirSpeed,             // I021/151
        kFRNTargetAddress,            // I021/080
        kFRNPositionTimeReceived,     // I021/073
        kFRNPositionTimeReceivedFine, // I021/074
        kFRNVelocityTimeReceived,     // I021/075
        kFRNVelocityTimeReceivedFine, // I021/076
        kFRNGeometricHeight,          // I021/140
        kFRNQualityIndicators,        // I021/090
        kFRNMOPSVersion,              // I021/210
        kFRNMode3A,                   // I021/070
        kFRNRollAngle,                // I021/230
        kFRNFlightLevel,              // I021/145
        kFRNMagneticHeading,          // I021/152
        kFRNTargetStatus,             // I021/200
        kFRNBaroVerticalRate,         // I021/155
        kFRNGeometricVerticalRate,    // I021/157
        kFRNAirborneGroundVector,     // I021/160
        kFRNTrackAngleRate,           // I021/165
        kFRNReportTime,               // I021/077
        kFRNTargetIdentification,     // I021/170
        kFRNEmitterCategory,          // I021/020
        kFRNMetInformation,           // I021/220
        kFRNSelectedAltitude,         // I021/146
        kFRNFinalStateSelectedAltitude,  // I021/148
        kFRNTrajectoryIntent,         // I021/110
        kFRNServiceManagement,        // I021/016
        kFRNOperationalStatus,        // I021/008
        kFRNSurfaceCapabilities,      // I021/271
        kFRNMessageAmplitude,         // I021/132
    };

    FSPEC fspec;
    uint16_t mode_3a = 0;
    bool has_mode_3a = r.has_squawk && SquawkToMode3A(r.squawk, mode_3a);
    int32_t flight_level_quarters = RoundToInt(r.baro_altitude_ft / 25.0);
    bool has_flight_level = r.has_flight_level && flight_level_quarters >= -15 * 4 && flight_level_quarters <= 1500 * 4;
    int32_t geometric_height = RoundToInt(r.geometric_height_ft / 6.25);
    bool has_geometric_height = r.has_geometric_height && geometric_height >= -1500 * 4 / 25 &&
                                geometric_height <= 150000 * 4 / 25;
    uint8_t operational_status = (r.tcas_ra_active ? 0x80 : 0) | (r.single_antenna ? 0x01 : 0);

    fspec.Set(kFRNDataSource);
    fspec.Set(kFRNTargetReportDescriptor);
    if (r.has_track_number) fspec.Set(kFRNTrackNumber);
    if (r.has_service_id) fspec.Set(kFRNServiceID);
    if (r.has_position) fspec.Set(kFRNPositionHighRes);
    if (r.has_indicated_airspeed) fspec.Set(kFRNAirSpeed);
    if (r.has_true_airspeed) fspec.Set(kFRNTrueAirSpeed);
    fspec.Set(kFRNTargetAddress);
    if (r.has_position) fspec.Set(kFRNPositionTimeReceived);
    if (r.has_velocity_time) fspec.Set(kFRNVelocityTimeReceived);
    if (has_geometric_height) fspec.Set(kFRNGeometricHeight);
    fspec.Set(kFRNQualityIndicators);
    if (r.has_mops_version) fspec.Set(kFRNMOPSVersion);
    if (has_mode_3a) fspec.Set(kFRNMode3A);
    if (has_flight_level) fspec.Set(kFRNFlightLevel);
    if (r.has_magnetic_heading) fspec.Set(kFRNMagneticHeading);
    if (r.has_target_status) fspec.Set(kFRNTargetStatus);
    if (r.has_baro_vertical_rate) fspec.Set(kFRNBaroVerticalRate);
    if (r.has_geometric_vertical_rate) fspec.Set(kFRNGeometricVerticalRate);
    if (r.has_ground_vector) fspec.Set(kFRNAirborneGroundVector);
    if (r.has_report_time) fspec.Set(kFRNReportTime);
    if (r.has_callsign) fspec.Set(kFRNTargetIdentification);
    if (r.has_emitter_category) fspec.Set(kFRNEmitterCategory);
    if (operational_status != 0) fspec.Set(kFRNOperationalStatus);
    if (r.has_amplitude) fspec.Set(kFRNMessageAmplitude);

    ByteWriter w(buf, buf_len);
    fspec.Write(w);

    // Items in FRN order.
    w.U8(r.sac);
    w.U8(r.sic);

    uint8_t descriptor[3] = {
        static_cast<uint8_t>(((r.address_type & 0x7) << 5) | ((r.altitude_reporting_capability & 0x3) << 3) |
                             (r.from_field_monitor ? 0x02 : 0)),
        static_cast<uint8_t>(r.on_ground ? 0x40 : 0),
        static_cast<uint8_t>(r.no_go ? 0x10 : 0),
    };
    w.Extended(descriptor, sizeof(descriptor));

    if (r.has_track_number) w.U16(r.track_number & 0x0FFF);
    if (r.has_service_id) w.U8(r.service_id);
    if (r.has_position) {
        // LSB 180 / 2^30 degrees: AWB / 2, rounded.
        w.U32(static_cast<uint32_t>(AWBToAsterixHighRes(r.latitude_awb)));
        w.U32(static_cast<uint32_t>(AWBToAsterixHighRes(r.longitude_awb)));
    }
    if (r.has_indicated_airspeed) w.U16(KnotsToNMPerSecond14(r.indicated_airspeed_kts) & 0x7FFF);  // IM = 0: IAS.
    if (r.has_true_airspeed) {
        bool range_exceeded = r.true_airspeed_kts > 0x7FFF;
        w.U16((range_exceeded ? 0x8000 : 0) | (range_exceeded ? 0x7FFF : (r.true_airspeed_kts & 0x7FFF)));
    }
    w.U24(r.address & 0xFFFFFF);
    if (r.has_position) w.U24(r.position_time_of_day);
    if (r.has_velocity_time) w.U24(r.velocity_time_of_day);
    if (has_geometric_height) w.U16(Signed(geometric_height, 16));

    uint8_t quality[4] = {
        static_cast<uint8_t>((r.nac_v << 5) | (r.nic << 1)),
        static_cast<uint8_t>((r.nic_baro << 7) | (r.sil << 5) | (r.nac_p << 1)),
        static_cast<uint8_t>((r.sil_supplement << 5) | (r.sda << 3) | (r.gva << 1)),
        static_cast<uint8_t>(r.pic << 4),
    };
    w.Extended(quality, sizeof(quality));

    if (r.has_mops_version) w.U8(((r.mops_version & 0x7) << 3) | (r.link_technology & 0x7));
    if (has_mode_3a) w.U16(mode_3a);
    if (has_flight_level) w.U16(Signed(flight_level_quarters, 16));
    if (r.has_magnetic_heading) w.U16(DegreesToAngle16(r.magnetic_heading_deg));
    if (r.has_target_status) {
        // LNAV (bit 7) set: not engaged or unknown.
        w.U8(0x40 | ((r.priority_status & 0x7) << 2) | (r.surveillance_status & 0x3));
    }
    if (r.has_baro_vertical_rate) w.U16(VerticalRate(r.baro_vertical_rate_fpm));
    if (r.has_geometric_vertical_rate) w.U16(VerticalRate(r.geometric_vertical_rate_fpm));
    if (r.has_ground_vector) {
        uint32_t speed = KnotsToNMPerSecond14(r.ground_speed_kts < 0 ? 0 : r.ground_speed_kts);
        bool range_exceeded = speed > 0x7FFF;
        w.U16((range_exceeded ? 0x8000 : 0) | (range_exceeded ? 0x7FFF : speed));
        w.U16(DegreesToAngle16(r.track_angle_deg));
    }
    if (r.has_report_time) w.U24(r.report_time_of_day);
    if (r.has_callsign) {
        uint64_t packed = 0;
        bool ended = false;
        for (uint8_t i = 0; i < 8; i++) {
            char c = ended ? ' ' : r.callsign[i];
            if (c == '\0') {
                ended = true;
                c = ' ';
            }
            packed = (packed << 6) | CallsignCharTo6Bit(c);
        }
        w.U16(static_cast<uint32_t>(packed >> 32));
        w.U32(static_cast<uint32_t>(packed));
    }
    if (r.has_emitter_category) w.U8(r.emitter_category);
    if (operational_status != 0) w.U8(operational_status);
    if (r.has_amplitude) {
        int32_t amplitude = r.amplitude_dbm < -128 ? -128 : (r.amplitude_dbm > 127 ? 127 : r.amplitude_dbm);
        w.U8(Signed(amplitude, 8));
    }
    return w.Len();
}

uint16_t ASTERIXReporter::WriteCAT023GroundStationStatusRecord(uint8_t* buf, uint16_t buf_len, uint8_t sac,
                                                               uint8_t sic, uint32_t time_of_day, bool clock_valid,
                                                               bool track_numbering_restarted, uint8_t period_s) {
    FSPEC fspec;
    fspec.Set(1);  // I023/010 Data Source Identifier.
    fspec.Set(2);  // I023/000 Report Type.
    fspec.Set(4);  // I023/070 Time of Day.
    fspec.Set(5);  // I023/100 Ground Station Status.

    ByteWriter w(buf, buf_len);
    fspec.Write(w);
    w.U8(sac);
    w.U8(sic);
    w.U8(1);  // Ground station status report.
    w.U24(time_of_day);
    // NOGO and TSV (time source invalid) go together; GSSP extension always follows.
    w.U8((clock_valid ? 0 : 0x80 | 0x08) | (track_numbering_restarted ? 0x02 : 0) | 0x01);
    w.U8((period_s & 0x7F) << 1);
    return w.Len();
}

uint16_t ASTERIXReporter::WriteCAT023ServiceStatusRecord(uint8_t* buf, uint16_t buf_len, uint8_t sac, uint8_t sic,
                                                         uint8_t service_id, uint8_t service_type,
                                                         uint32_t time_of_day, uint8_t report_period_half_s,
                                                         uint8_t period_s, ServiceStatus status) {
    FSPEC fspec;
    fspec.Set(1);  // I023/010 Data Source Identifier.
    fspec.Set(2);  // I023/000 Report Type.
    fspec.Set(3);  // I023/015 Service Type and Identification.
    fspec.Set(4);  // I023/070 Time of Day.
    fspec.Set(6);  // I023/101 Service Configuration.
    fspec.Set(8);  // I023/110 Service Status.

    ByteWriter w(buf, buf_len);
    fspec.Write(w);
    w.U8(sac);
    w.U8(sic);
    w.U8(2);  // Service status report.
    w.U8(((service_id & 0xF) << 4) | (service_type & 0xF));
    w.U24(time_of_day);
    w.U8(report_period_half_s);
    w.U8(0x01);  // Service class 0 (no information), SSRP extension follows.
    w.U8((period_s & 0x7F) << 1);
    w.U8((status & 0x7) << 1);
    return w.Len();
}

uint16_t ASTERIXReporter::BeginDataBlock(uint8_t* buf, uint16_t buf_len, uint8_t category) {
    if (buf_len < kDataBlockHeaderLenBytes) return 0;
    buf[0] = category;
    buf[1] = 0;
    buf[2] = kDataBlockHeaderLenBytes;
    return kDataBlockHeaderLenBytes;
}

void ASTERIXReporter::FinishDataBlock(uint8_t* buf, uint16_t block_len_bytes) {
    buf[1] = block_len_bytes >> 8;
    buf[2] = block_len_bytes & 0xFF;
}
