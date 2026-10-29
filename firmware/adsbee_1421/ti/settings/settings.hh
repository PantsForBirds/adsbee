#ifndef SETTINGS_HH_
#define SETTINGS_HH_

#include <cstddef>  // for offsetof
#include <cstdint>
#include <functional>  // for strtoull

#include "crc.hh"
#include "macros.hh"
#include "altitude_filter.hh"
#include "unit_conversions.hh"  // DeciNmToMeters
#include "stdio.h"
#include "stdlib.h"  // for strtoull
#include "string.h"  // for memset
#ifdef ON_PICO
#include "pico/rand.h"
#endif

// Change this when settings format changes! A mismatched version resets all settings to factory defaults on boot.
static constexpr uint32_t kSettingsVersion = 6;
static constexpr uint32_t kDeviceInfoVersion = 2;

// Leading word of a valid settings blob. Deliberately not 0x00000000 or 0xFFFFFFFF so that blank,
// bulk-erased, or all-zero flash fails the check immediately rather than looking like a valid
// struct. Paired with the trailing CRC32; see SettingsManager::Settings::IsValid().
static constexpr uint32_t kSettingsMagic = 0x14215E77;

class SettingsManager {
   public:
    // Serial Interface enum and string conversion array.
    enum SerialInterface : uint16_t { kConsole = 0, kNumSerialInterfaces };

    // Maximum number of sinks that can be passed to CommsManager::UpdateReporting() in a single call.
    // The 1421 has no IP feeds, so this is just the serial interface count. Per-protocol sink arrays in
    // the shared comms_reporting.cpp are sized with this.
    static constexpr uint16_t kMaxNumReportSinks = SerialInterface::kNumSerialInterfaces;
    static constexpr uint16_t kSerialInterfaceStrMaxLen = 30;
    static const char kSerialInterfaceStrs[SerialInterface::kNumSerialInterfaces][kSerialInterfaceStrMaxLen];

    enum LogLevel : uint16_t { kSilent = 0, kErrors, kWarnings, kInfo, kNumLogLevels };
    static constexpr uint16_t kConsoleLogLevelStrMaxLen = 30;
    static const char kConsoleLogLevelStrs[LogLevel::kNumLogLevels][kConsoleLogLevelStrMaxLen];

    static const uint16_t kSubGHzModeStrMaxLen = 30;  // Length of Sub-GHz mode strings.

    // Reporting Protocol enum and string conversion array.
    enum ReportingProtocol : uint16_t {
        kNoReports = 0,
        kRaw,
        kBeast,
        kBeastNoUAT,
        kBeastNoUATUplink,
        kCSBee,
        kMAVLINK1,
        kMAVLINK2,
        kGDL90,
        kAircraftJSON,
        kGDL90NoUATUplink,  // Appended (not next to kGDL90): values are flash-persisted, don't renumber.
        kAircraftJSONUATUplink,  // Aircraft JSON plus one line per UAT ground uplink.
        kASTERIX,                // EUROCONTROL ASTERIX CAT021 target reports and CAT023 status.
        kNumProtocols
    };
    static constexpr uint16_t kReportingProtocolStrMaxLen = 30;
    static const char kReportingProtocolStrs[ReportingProtocol::kNumProtocols][kReportingProtocolStrMaxLen];

    enum EnableState : int8_t {
        kEnableStateExternal = -1,  // Enable GPIO pin is high impedance.
        kEnableStateDisabled = 0,
        kEnableStateEnabled = 1
    };

    // Mode setting for the Sub-GHz radio (AT+SUBG_MODE). Values are flash-persisted: append new modes, don't
    // renumber.
    enum SubGHzRadioMode : uint8_t {
        kSubGHzRadioModeUATRx = 0,       // UAT (978MHz): ADS-B + ground uplink reception.
        kSubGHzRadioModeUATRxNoUplink,   // UAT (978MHz): ADS-B only, uplink sync word disabled at the RF core.
        kSubGHzRadioModeADSLRx,          // ADS-L M-band receiver on 868.2 MHz.
        kSubGHzRadioModeADSLRx868_4MHz,  // ADS-L M-band receiver on 868.4 MHz.
        kNumSubGHzRadioModes
    };
    static const char kSubGHzModeStrs[kNumSubGHzRadioModes][kSubGHzModeStrMaxLen];

    // How the LR2021 detects 1090 MHz Mode S packets. Stored in flash: renumbering needs a kSettingsVersion bump.
    enum R1090PreambleMode : uint8_t {
        kR1090PreambleModeModeS = 0,  // Whole preamble, fixed gain: every downlink format. Factory default.
        kR1090PreambleModeDF17 = 1,   // Preamble chips 8-15 + DF17 header bits: DF17 frames only.
        kNumR1090PreambleModes
    };
    static constexpr uint16_t kR1090PreambleModeStrMaxLen = 30;
    static const char kR1090PreambleModeStrs[kNumR1090PreambleModes][kR1090PreambleModeStrMaxLen];

    // Receiver position settings.
    struct __attribute__((packed)) RxPosition {
        enum PositionSource : uint8_t {
            kPositionSourceNone = 0,
            kPositionSourceFixed = 1,
            kPositionSourceGNSS = 2,
            kPositionSourceLowestAircraft = 3,
            kPositionSourceAircraftMatchingICAO = 4,
            kNumPositionSources
        };

        static const uint16_t kPositionSourceStrMaxLen = 32;
        static const char kPositionSourceStrs[kNumPositionSources][kPositionSourceStrMaxLen];

        PositionSource source = kPositionSourceLowestAircraft;
        float latitude_deg = 0.0;      // Degrees, WGS84
        float longitude_deg = 0.0;     // Degrees, WGS84
        int32_t gnss_altitude_ft = 0;  // Meters, WGS84
        int32_t baro_altitude_ft = 0;  // Meters, AMSL
        float heading_deg = 0.0;       // Degrees from true north
        int32_t speed_kts = 0;         // Speed over ground in knots
        uint32_t icao_address =
            0;  // ICAO address to use for position bootstrap when source is kPositionSourceICAO, or the ICAO of the
                // lowest plane being tracked when source is kPositionSourceLowestAircraft.

        /**
         * Receiver altitude for AT+FILTER_ALTITUDE_FT's near-receiver mode: the GNSS altitude for a GNSS fix (it has no
         * barometric altitude), otherwise the barometric altitude, falling back to the GNSS altitude when that is 0
         * (not entered, or not known for the aircraft the position comes from).
         */
        int32_t FilterAltitudeFt() const {
            if (source == kPositionSourceGNSS || baro_altitude_ft == 0) {
                return gnss_altitude_ft;
            }
            return baro_altitude_ft;
        }
    };

    // This struct contains nonvolatile settings that should persist across reboots but may be overwritten during a
    // firmware upgrade if the format of the settings struct changes.
    //
    // Packed on purpose: the integrity CRC is taken over the raw object bytes, so the struct must have no
    // padding. Padding bytes are indeterminate in a stack-local (e.g. ResetToDefaults()'s default_settings),
    // which would make the CRC of two otherwise identical structs differ. Note this is why the struct is not
    // alignas(4) — that would force sizeof() up to a multiple of 4 and reintroduce trailing padding. The
    // in-RAM instance is aligned separately (see the `settings` member below).
    struct __attribute__((packed)) Settings {
        static constexpr uint32_t kDefaultWatchdogTimeoutSec = 10;
        // AT+FILTER_DISTANCE_NM, in tenths of a nautical mile.
        static constexpr uint32_t kDefaultFilterDistanceDeciNm = 1000;  // 100.0 nm.
        static constexpr uint32_t kMinFilterDistanceDeciNm = 1;         // 0.1 nm.
        static constexpr uint32_t kMaxFilterDistanceDeciNm = 100000;    // 10,000.0 nm.
        // AT+FILTER_ALTITUDE_FT defaults: off, with 0 to 10,000 ft ready for range mode.
        static constexpr int32_t kDefaultFilterAltitudeAFt = 0;
        static constexpr int32_t kDefaultFilterAltitudeBFt = 10000;
        // NOTE: Lengths do not include null terminator.
        // Single source of truth for the console boot baud rate: CommsManagerConfig defaults to this,
        // and Apply() falls back to it if the stored rate isn't whitelisted.
        static constexpr uint32_t kDefaultUARTBaudRate = 1000000;

        // Network-related string length constants. The 1421 has no network interfaces, but
        // common/coprocessor/object_dictionary.hh (pulled in via spi_coprocessor_packet.hh from
        // composite_array.cpp) sizes its ESP32NetworkInfo wire struct with these, so they must
        // exist here (this file shadows common/settings/settings.hh) and match the values in
        // common/settings/settings.hh to keep the wire layout identical.
        static constexpr uint16_t kWiFiSSIDMaxLen = 31;
        static constexpr uint16_t kWiFiMaxNumClients = 6;
        static constexpr uint16_t kIPAddrStrLen = 16;   // XXX.XXX.XXX.XXX (does not include null terminator)
        static constexpr uint16_t kMACAddrStrLen = 18;  // XX:XX:XX:XX:XX:XX (does not include null terminator)
        static constexpr uint16_t kMACAddrNumBytes = 6;

        // Actual settings values start here. `magic` must stay first and `crc` last: the CRC covers
        // everything from `magic` up to (not including) `crc`.
        uint32_t magic = kSettingsMagic;
        uint32_t settings_version = kSettingsVersion;

        // ADSBee settings
        bool r1090_rx_enabled = true;
        uint32_t watchdog_timeout_sec = kDefaultWatchdogTimeoutSec;
        R1090PreambleMode r1090_preamble_mode = R1090PreambleMode::kR1090PreambleModeModeS;
        uint8_t r1090_gain = 0;      // 0 = auto AGC, 1..15 manual (13 = max). Default: auto.
        uint8_t r1090_rx_boost = 0;  // LF RX path boost, 0 (off) .. 7 (max).
        // AT+LR_ENABLE: false = LR2021 held in reset with the CC1314-side bus pins parked hi-Z
        // (NSS pull-up; RESET/SCLK/PICO pull-down) so an external host can drive the shared bus.
        bool lr2021_enabled = true;

        // CommunicationsManager settings
        LogLevel log_level = LogLevel::kWarnings;
        ReportingProtocol reporting_protocols[SerialInterface::kNumSerialInterfaces] = {
            ReportingProtocol::kNoReports,  // Console
        };
        uint32_t baud_rates[SerialInterface::kNumSerialInterfaces] = {
            kDefaultUARTBaudRate,  // Console
        };

        // Sub-GHz settings
        bool subg_rx_enabled = true;
        SubGHzRadioMode subg_mode = SubGHzRadioMode::kSubGHzRadioModeUATRx;  // Default to UAT mode (978MHz receiver).

        // MAVLINK settings
        uint8_t mavlink_system_id = 1;
        uint8_t mavlink_component_id = 156;  // Default to MAV_COMP_ID_ADSB (156).

        // Receiver position settings
        RxPosition rx_position;

        // Distance filter (AT+FILTER_DISTANCE_NM), added in version 6. When enabled, aircraft known to be farther than
        // filter_distance_deci_nm tenths of a nautical mile from the receiver are left out of aircraft reports
        // (aircraft JSON, GDL90, MAVLink, CSBee). Raw packets (Raw, Beast, MLAT) are never filtered. Aircraft at an
        // unknown distance are always kept.
        bool filter_distance_enabled = false;
        uint32_t filter_distance_deci_nm = kDefaultFilterDistanceDeciNm;

        // ASTERIX data source identifier (I021/010, I023/010), added in version 6: System Area Code and System
        // Identification Code.
        uint8_t asterix_sac = 0;
        uint8_t asterix_sic = 0;

        // Altitude filter (AT+FILTER_ALTITUDE_FT=<mode>,<a>,<b>), added in version 6: an AltitudeFilterMode with its
        // arguments in feet. Leaves aircraft outside the selected altitudes out of the same aircraft reports as the
        // distance filter. Aircraft with no known altitude are always kept.
        uint8_t filter_altitude_mode = kAltitudeFilterOff;
        int32_t filter_altitude_a_ft = kDefaultFilterAltitudeAFt;
        int32_t filter_altitude_b_ft = kDefaultFilterAltitudeBFt;

        // Integrity check over every preceding byte. Must stay the last member; stamped by Stamp().
        uint32_t crc = 0;

        /**
         * Default constructor.
         */
        Settings() {}

        /**
         * The distance filter as Aircraft::IsBeyondDistance() takes it.
         * @retval Filter distance in meters, or 0 when the filter is off.
         */
        inline uint32_t FilterDistanceMeters() const {
            return filter_distance_enabled ? DeciNmToMeters(filter_distance_deci_nm) : 0;
        }

        /**
         * CRC32 over the struct from `magic` up to (not including) `crc`.
         * @retval Expected value of the crc field for the current contents.
         */
        inline uint32_t ComputeCRC() const {
            return crc32_ieee_802_3(reinterpret_cast<const uint8_t *>(this), sizeof(Settings) - sizeof(crc));
        }

        /**
         * Stamps the magic, version and CRC so the struct is ready to be written to flash. Call immediately
         * before programming; any later field change invalidates the CRC.
         */
        inline void Stamp() {
            magic = kSettingsMagic;
            settings_version = kSettingsVersion;
            crc = ComputeCRC();
        }

        /**
         * Checks a struct read back from flash. Guards against blank flash, a stale settings format, and a
         * torn write that landed the leading words but not the rest (which the version check alone accepts,
         * since settings_version sits at the front of the struct).
         * @retval True if the blob is intact and belongs to this firmware.
         */
        inline bool IsValid() const {
            return magic == kSettingsMagic && settings_version == kSettingsVersion && crc == ComputeCRC();
        }
    };

    // The CRC is taken over raw object bytes, so `crc` must sit flush against the end of the struct with no
    // trailing padding. Trips if the packed attribute is ever dropped or a member is added after `crc`.
    static_assert(offsetof(Settings, crc) == sizeof(Settings) - sizeof(uint32_t),
                  "Settings::crc must be the last member, with no trailing padding.");

    // Version 5 stored the same fields up to rx_position, then the CRC; version 6 appended filter_distance_enabled,
    // filter_distance_deci_nm, asterix_sac, asterix_sic and the altitude filter (filter_altitude_*).
    static constexpr uint16_t kVersion5SettingsSize = 62;
    static_assert(kSettingsVersion == 6 &&
                      offsetof(Settings, filter_distance_enabled) == kVersion5SettingsSize - sizeof(uint32_t) &&
                      sizeof(Settings) == kVersion5SettingsSize + 16,
                  "Settings changed: update MigrateFromVersion5 or add a migration from version 6.");

    /**
     * Converts a version 5 settings blob, so a firmware update keeps the user's settings.
     * @param[in] blob Stored bytes, at least kVersion5SettingsSize long.
     * @param[out] out Migrated and stamped settings. Untouched unless the blob is an intact version 5 blob.
     * @retval True if the blob was an intact version 5 blob.
     */
    static bool MigrateFromVersion5(const uint8_t *blob, Settings &out) {
        constexpr uint16_t kFieldBytes = kVersion5SettingsSize - sizeof(uint32_t);  // Everything before the CRC.
        uint32_t magic, version, crc;
        memcpy(&magic, blob, sizeof(magic));
        memcpy(&version, blob + sizeof(magic), sizeof(version));
        memcpy(&crc, blob + kFieldBytes, sizeof(crc));
        if (magic != kSettingsMagic || version != 5 || crc != crc32_ieee_802_3(blob, kFieldBytes)) {
            return false;
        }
        out = Settings();  // The fields added in version 6 take their defaults.
        memcpy(static_cast<void *>(&out), blob, kFieldBytes);
        out.Stamp();
        return true;
    }

    // This struct contains device information that should persist across firmware upgrades.
    struct DeviceInfo {
        // NOTE: Lengths do not include null terminator.
        static constexpr uint16_t kPartCodeLen = 26;  // NNNNNNNNNR-YYYYMMDD-VVXXXX (not counting end of string char).
        static constexpr uint16_t kPartCodePartNumberLen = 9;  // Not counting rev letter or end of string char.
        static constexpr uint16_t kOTAKeyMaxLen = 128;
        static constexpr uint16_t kNumOTAKeys = 2;

        uint32_t device_info_version = kDeviceInfoVersion;
        char part_code[kPartCodeLen + 1];
        char ota_keys[kNumOTAKeys][kOTAKeyMaxLen + 1];

        enum ADSBeePartNumber : uint32_t {
            // WARNING: These can't have a leading 0, or they will be encoded as octal and not decimal!
            kPNADSBee1090 = 10240002,                  // ADSBee 1090
            kPNADSBee1090U = 10250002,                 // ADSBee 1090U
            kPNADSBee1090UIndoorPoEFeeder = 40250002,  // ADSBee 1090U Indoor PoE Feeder
            kPNADSBeem1090 = 10250007,                 // ADSBee m1090
            kPNADSBeem1090EvalBoard = 10250013,        // ADSBee m1090 Eval Board
            kPNGS3MPoE = 40250001,                     // GS3M PoE
            kPNADSBeem1421 = 10260002                  // ADSBee m1421
        };

        /**
         * Default constructor.
         */
        DeviceInfo() { memset(part_code, '\0', kPartCodeLen + 1); }

        /**
         * Extracts the part number and revision from the part code.
         * @retval Part number as an integer.
         */
        uint32_t GetPartNumber() {
            char part_number_buf[kPartCodePartNumberLen + 1] = {0};
            memcpy(part_number_buf, part_code, kPartCodePartNumberLen);
            return strtoul(
                part_number_buf, nullptr,
                10);  // Convert part number to integer (read part code string as base 10, without the rev letter).
        }

        /**
         * Extracts the part revision character from the part code.
         * @retval Character representing the part revision.
         */
        char GetPartRev() { return part_code[kPartCodePartNumberLen]; }

        /**
         * True if the part code is <part_number><rev>[-...] with rev >= min_rev. Blank or malformed part codes
         * return false, so callers can treat false as "unknown board".
         * @param[in] part_number Part number as an integer (see ADSBeePartNumber).
         * @param[in] min_rev Lowest revision letter that matches.
         */
        bool IsPartAtLeastRev(uint32_t part_number, char min_rev) const {
            uint32_t pn = 0;
            for (uint16_t i = 0; i < kPartCodePartNumberLen; i++) {
                char c = part_code[i];
                if (c < '0' || c > '9') return false;
                pn = pn * 10 + static_cast<uint32_t>(c - '0');
            }
            char rev = part_code[kPartCodePartNumberLen];
            char next = part_code[kPartCodePartNumberLen + 1];
            if (rev < 'A' || rev > 'Z' || (next != '-' && next != '\0')) return false;
            return pn == part_number && rev >= min_rev;
        }
    };

    /**
     * Applies internal settings to the relevant objects. This is only used after the settings struct has been
     * updated by loading it from EEPROM or by overwriting it via the coprocessor SPI bus.
     */
    bool Apply();

    /**
     * Helper function for reconstructing an AT command value for a given EnableState.
     * @param[in] state EnableState to convert to a string.
     * @retval String representation of the EnableState, as it would be used in an AT command.
     */
    static inline const char* EnableStateToATValueStr(EnableState state) {
        switch (state) {
            case kEnableStateExternal:
                return "EXTERNAL";
            case kEnableStateEnabled:
                return "1";
            case kEnableStateDisabled:
                return "0";
            default:
                return "?";
        }
    }

    /**
     * Used to retrieve device information, either directly from EEPROM or via interprocessor SPI bus.
     * @param[in] device_info DeviceInfo struct to set.
     * @retval True if device info was retrieved successfully, false otherwise.
     */
    static bool GetDeviceInfo(DeviceInfo& device_info);

    /**
     * Used to set device information, which is stored in flash and should persist across firmware updates.
     * @param[in] device_info DeviceInfo struct to set.
     * @retval True if device info was set successfully, false otherwise.
     */
    static bool SetDeviceInfo(const DeviceInfo& device_info);

    /**
     * Loads settings from flash. Validates the blob with Settings::IsValid(); on failure (blank flash, a
     * settings format change, or a torn write) it falls back to defaults and rewrites the region.
     * @retval True if succeeded, false if the fallback rewrite failed.
     */
    bool Load();

    /**
     * Print the settings in human-readable format.
     */
    void Print();

    /**
     * Print the settings in AT command format. Used for dumping settings before a firmware update.
     */
    void PrintAT();

    /**
     * Takes a password as a string and fills a buffer with the corresponding number of asterix.
     * @param[in] password_buf Buffer to read the password from. Must be at least password_len+1 chars.
     * @param[out] redacted_password_buf Buffer to write the asterix to. Must be at least password_len+1 chars.
     * @param[in] buf_len Maximum allowable number of characteers in the password. Used to guard against falling off
     * the end of the string. Not used for actually finding ther number of asterix to print.
     */
    static inline void RedactPassword(char* password_buf, char* redacted_password_buf, uint16_t buf_len) {
        uint16_t password_len = MIN(strnlen(password_buf, buf_len), buf_len);
        memset(redacted_password_buf, '*', password_len);
        redacted_password_buf[password_len] = '\0';
    }

    /**
     * Saves settings to flash, stamping the magic/version/CRC first and verifying the write by reading it
     * back. Returns the real outcome, so AT+SETTINGS=SAVE reports ERROR on a failed write rather than a
     * false OK.
     * @retval True if the settings were written and read back intact, false otherwise.
     */
    bool Save();

    /**
     * Copies settings that other objects hold while running (receiver enables, baud rate, receiver position) into
     * the settings struct. Save() does this before writing.
     */
    void CollectLiveSettings();

    /**
     * Restores settings to factory default values.
     */
    void ResetToDefaults();

    // Aligned explicitly: Settings is packed (alignof 1) so the CRC can cover its raw bytes, but the live
    // instance is memcpy'd to and from flash and read field-by-field on every loop, so keep it word aligned.
    alignas(4) Settings settings;

   private:
};

extern SettingsManager settings_manager;

#endif /* SETTINGS_HH_ */