#ifndef SETTINGS_HH_
#define SETTINGS_HH_

#include <cstdint>
#include <functional>  // for strtoull

#include "crc.hh"
#include "macros.hh"
#include "stdio.h"
#include "stdlib.h"  // for strtoull
#include "string.h"  // for memset
#ifdef ON_PICO
#include "pico/rand.h"
#endif

static constexpr uint32_t kSettingsVersion = 15;  // Change this when settings format changes!
static constexpr uint32_t kDeviceInfoVersion = 2;

class SettingsManager {
   public:
    // Serial Interface enum and string conversion array.
    enum SerialInterface : uint16_t { kConsole = 0, kCommsUART, kGNSSUART, kNumSerialInterfaces };
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
        kNumProtocols
    };
    static constexpr uint16_t kReportingProtocolStrMaxLen = 30;
    static const char kReportingProtocolStrs[ReportingProtocol::kNumProtocols][kReportingProtocolStrMaxLen];

    static constexpr uint8_t kWiFiAPChannelMax = 11;  // Operation in channels 12-14 avoided in USA.

    enum EnableState : int8_t {
        kEnableStateExternal = -1,  // Enable GPIO pin is high impedance.
        kEnableStateDisabled = 0,
        kEnableStateEnabled = 1
    };

    enum GNSSReceiverType : uint8_t {
        kGNSSReceiverNone = 0,
        kGNSSReceiverGeneric,
        kGNSSReceiverUBXMIA,
    };

    // Mode setting for the Sub-GHz radio.
    enum SubGHzRadioMode : uint8_t {
        kSubGHzRadioModeUATRx = 0,  // UAT mode (978MHz receiver).
        kNumSubGHzRadioModes
    };
    static const char kSubGHzModeStrs[kNumSubGHzRadioModes][kSubGHzModeStrMaxLen];

    // Broadcast Remote ID transports, as a bitmask stored in Settings::remote_id_transports. Whether a requested
    // transport actually runs additionally depends on the hardware build (PSRAM vs not) and the WiFi/ethernet state;
    // the ESP32 reports the resolved live state back in ObjectDictionary::ESP32DeviceStatus::remote_id_status.
    enum RemoteIDTransport : uint8_t {
        kRemoteIDTransportBLE4 = 1 << 0,        // Bluetooth 4 legacy advertising (1M PHY).
        kRemoteIDTransportBLE5Long = 1 << 1,    // Bluetooth 5 Long Range (Coded PHY S=8) extended advertising.
        kRemoteIDTransportWiFiBeacon = 1 << 2,  // WiFi beacon vendor-specific IE.
    };

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
         * Whether a position resolved from `source` describes THIS device, and may therefore be broadcast as our own
         * position (currently: Remote ID transmit).
         *
         * Only kPositionSourceFixed (a coordinate the operator entered for this device) and kPositionSourceGNSS (this
         * device's own fix) qualify. The aircraft-derived sources carry a *received* aircraft's position:
         * kPositionSourceLowestAircraft and kPositionSourceAircraftMatchingICAO copy the lat/lon of a tracked Mode S /
         * UAT target. Broadcasting one of those would assert "this is where I am" using a real, possibly crewed
         * aircraft's coordinate -- placing a fabricated drone, and a fabricated operator location, on top of it. They
         * are legitimate as a *receiver* reference position (range calculations, CPR decoding), never as a transmitted
         * one. kPositionSourceNone carries no position at all.
         *
         * Lives here, rather than beside the transmitter, because both processors need the same answer: the ESP32
         * enforces it (RemoteIDTransmitter::RefreshFromDeviceState) and the RP2040 warns about it from the AT console.
         * A static member function adds no storage, so the packed layout above is unaffected.
         */
        static bool MayBeTransmittedAsOwnPosition(PositionSource source) {
            return source == kPositionSourceFixed || source == kPositionSourceGNSS;
        }
    };

    // This struct contains nonvolatile settings that should persist across reboots but may be overwritten during a
    // firmware upgrade if the format of the settings struct changes.
    struct alignas(4) Settings {
        static constexpr int kDefaultTLOffsetMV = 600;  // [mV]
        static constexpr uint32_t kDefaultWatchdogTimeoutSec = 10;
        // NOTE: Lengths do not include null terminator.
        static constexpr uint16_t kHostnameMaxLen = 32;
        // For settings struct backwards compatibility. ESP IDF can't handle the full length SSID and password, so length was reduced.
        static constexpr uint16_t kWiFiSSIDMaxLen = 31;
        static constexpr uint16_t kWiFiPasswordMaxLen = 63;
        static constexpr uint16_t kWiFiMaxNumClients = 6;
        static constexpr uint32_t kDefaultCommsUARTBaudrate = 115200;
        static constexpr uint32_t kDefaultGNSSUARTBaudrate = 9600;
        // Open Drone ID UAS ID / Operator ID string length (ODID_ID_SIZE in opendroneid.h).
        static constexpr uint16_t kRemoteIDIDMaxLen = 20;
        static constexpr uint16_t kMaxNumFeeds = 10;
        static constexpr uint16_t kFeedURIMaxNumChars = 63;
        static constexpr uint16_t kFeedReceiverIDNumBytes = 8;
        static constexpr uint16_t kIPAddrStrLen = 16;   // XXX.XXX.XXX.XXX (does not include null terminator)
        static constexpr uint16_t kMACAddrStrLen = 18;  // XX:XX:XX:XX:XX:XX (does not include null terminator)
        static constexpr uint16_t kMACAddrNumBytes = 6;

        /**
         * Core Network Settings struct is used for storing network settings that should remain unchanged through most
         * firmware updates. This allows re-establishing network connectivity with an ADSBee to restore remaining
         * settings via AT commands after a software upgrade changes other parts of the settings struct or associated
         * code.
         */
        struct CoreNetworkSettings {
            // ESP32 settings
            bool esp32_enabled = true;

            char hostname[kHostnameMaxLen + 1];

            bool wifi_ap_enabled = true;
            uint8_t wifi_ap_channel = 1;
            // SSID and password array lengths adjusted to maintain settings struct size.
            char wifi_ap_ssid[kWiFiSSIDMaxLen + 2];
            char wifi_ap_password[kWiFiPasswordMaxLen + 2];

            bool wifi_sta_enabled = false;
            char wifi_sta_ssid[kWiFiSSIDMaxLen + 2];
            char wifi_sta_password[kWiFiPasswordMaxLen + 2];

            bool ethernet_enabled = false;

            uint32_t crc32 = 0;

            CoreNetworkSettings() {
                // Clear out the strings so that checksum calculation is consistent.
                memset(hostname, '\0', sizeof(hostname));
                memset(wifi_ap_ssid, '\0', sizeof(wifi_ap_ssid));
                memset(wifi_ap_password, '\0', sizeof(wifi_ap_password));
                memset(wifi_sta_ssid, '\0', sizeof(wifi_sta_ssid));
                memset(wifi_sta_password, '\0', sizeof(wifi_sta_password));

                strncpy(hostname, "ADSBee1090",
                        sizeof(hostname));  // Default hostname, will be overridden when DeviceInfo is set.
            }

            uint32_t CalculateCRC32() {
                // Don't calculate checksum including the crc32 field itself, silly.
                return crc32_ieee_802_3((uint8_t*)this, sizeof(CoreNetworkSettings) - sizeof(crc32));
            }

            /**
             * Updates the CRC32. Should be called before saving.
             */
            void UpdateCRC32() { crc32 = CalculateCRC32(); }

            /**
             * Checks whether the calculated CRC32 matches the value that was stored. Should be used to check whether
             * the core network settings can be applied.
             */
            bool IsValid() {
                uint32_t calculated_crc32 = CalculateCRC32();
                return calculated_crc32 == crc32;
            }
        };

        uint32_t settings_version = kSettingsVersion;

        CoreNetworkSettings core_network_settings;

        // ADSBee settings
        bool r1090_rx_enabled = true;
        int tl_offset_mv = kDefaultTLOffsetMV;
        bool r1090_bias_tee_enabled = false;
        uint32_t watchdog_timeout_sec = kDefaultWatchdogTimeoutSec;
        bool led_enabled = true;  // Set to false to disable all hardware status/activity LEDs.
        bool gnss_enabled = false;
        GNSSReceiverType gnss_receiver_type = kGNSSReceiverNone;
        bool gnss_notify = false;

        // CommunicationsManager settings
        LogLevel log_level = LogLevel::kWarnings;
        ReportingProtocol reporting_protocols[SerialInterface::kNumSerialInterfaces] = {
            ReportingProtocol::kNoReports,  // Console
            ReportingProtocol::kNoReports,  // Comms UART
            ReportingProtocol::kNoReports   // GNSS UART
        };
        uint32_t baud_rates[SerialInterface::kNumSerialInterfaces] = {
            0,       // Console (virtual COM port)
            115200,  // Comms UART
            9600     // GNSS UART
        };

        // Sub-GHz settings
        EnableState subg_enabled = EnableState::kEnableStateEnabled;  // High impedance state by default.
        bool subg_rx_enabled = true;
        bool subg_bias_tee_enabled = false;
        SubGHzRadioMode subg_mode = SubGHzRadioMode::kSubGHzRadioModeUATRx;  // Default to UAT mode (978MHz receiver).

        // Remote ID (Broadcast Drone ID, ASTM F3411) receive settings. EXPERIMENTAL: not yet a stable part of the
        // system, and these settings may change. Reception happens on the ESP32 over BLE/WiFi; see
        // firmware/adsbee_1090/esp/main/remote_id/. Which transports actually run also depends on the hardware build and
        // the WiFi/ethernet state (the ESP32 reports the live state back via ESP32DeviceStatus::remote_id_status).
        bool remote_id_rx_enabled = false;  // Master enable for Remote ID reception. Off by default.
        // Bitmask of RemoteIDTransport values the user wants enabled (subject to the constraints above).
        uint8_t remote_id_transports = SettingsManager::kRemoteIDTransportBLE4 |
                                       SettingsManager::kRemoteIDTransportBLE5Long |
                                       SettingsManager::kRemoteIDTransportWiFiBeacon;

        // Remote ID transmit settings. EXPERIMENTAL: not yet a stable part of the system, and these settings may
        // change. The ADSBee can act as a Broadcast Remote ID transmitter, either as a bench test transmitter for
        // checking Remote ID receiver performance, or mounted on a drone as a combined ADS-B receiver and Remote ID
        // transmitter. Transmission uses the same radios as reception and needs no WiFi AP/STA.
        // The transmitted UA position comes from rx_position (AT+RX_POSITION), but ONLY from the sources that describe
        // this device: kPositionSourceFixed (a bench transmitter's entered coordinate) and kPositionSourceGNSS (this
        // device's own fix). The aircraft-derived sources carry a *received* aircraft's position and are never
        // transmitted -- see RxPosition::MayBeTransmittedAsOwnPosition.
        bool remote_id_tx_enabled = false;  // Master enable for Remote ID transmission. Off by default.
        // Bitmask of RemoteIDTransport values to advertise on. All methods by default so the device is useful as a
        // test transmitter against any receiver; narrow it to test one method at a time.
        uint8_t remote_id_tx_transports = SettingsManager::kRemoteIDTransportBLE4 |
                                          SettingsManager::kRemoteIDTransportBLE5Long |
                                          SettingsManager::kRemoteIDTransportWiFiBeacon;
        // ODID_idtype_t: 1 = serial number (ANSI/CTA-2063-A), 2 = CAA registration, 3 = UTM UUID, 4 = session ID.
        uint8_t remote_id_tx_uas_id_type = 1;
        // ODID_uatype_t: 1 = aeroplane, 2 = helicopter/multirotor, ... 15 = other.
        uint8_t remote_id_tx_ua_type = 2;
        // UAS ID (serial / registration) advertised in the Basic ID message. Empty means "derive from this device's
        // serial number at runtime" so the unit is a usable test transmitter with no configuration.
        char remote_id_tx_uas_id[kRemoteIDIDMaxLen + 1];
        // Operator ID advertised in the Operator ID message. Empty means the message is not transmitted.
        char remote_id_tx_operator_id[kRemoteIDIDMaxLen + 1];

        // Feed settings
        char feed_uris[kMaxNumFeeds][kFeedURIMaxNumChars + 1];
        uint16_t feed_ports[kMaxNumFeeds];
        bool feed_is_active[kMaxNumFeeds];
        ReportingProtocol feed_protocols[kMaxNumFeeds];
        uint8_t feed_receiver_ids[kMaxNumFeeds][kFeedReceiverIDNumBytes];

        // MAVLINK settings
        uint8_t mavlink_system_id = 1;
        uint8_t mavlink_component_id = 156;  // Default to MAV_COMP_ID_ADSB (156).

        // Receiver position settings
        RxPosition rx_position;

        /**
         * Default constructor.
         */
        Settings() {
            // Remote ID transmit identity strings default to empty: an empty UAS ID means "derive from this device's
            // serial number at runtime" (see the Remote ID transmitter), and an empty operator ID suppresses the
            // Operator ID message.
            memset(remote_id_tx_uas_id, '\0', sizeof(remote_id_tx_uas_id));
            memset(remote_id_tx_operator_id, '\0', sizeof(remote_id_tx_operator_id));

#ifdef ON_PICO
            DeviceInfo device_info;
            if (GetDeviceInfo(device_info)) {
                // If able to load device info from EEPROM, use the last 16 characters in the part code as part of the
                // WiFi SSID.
                device_info.GetDefaultSSID(core_network_settings.wifi_ap_ssid);
                // Reuse the WiFi SSID as the hostname.
                strncpy(core_network_settings.hostname, core_network_settings.wifi_ap_ssid, 32);
                snprintf(core_network_settings.wifi_ap_password, kWiFiPasswordMaxLen, "yummyflowers");
            }

            core_network_settings.wifi_ap_channel =
                (get_rand_32() % 3) * 5 + 1;  // Randomly select a non-overlapping channel on 2.4GHz (1, 6, 11).
#endif

            for (uint16_t i = 0; i < kMaxNumFeeds; i++) {
                memset(feed_uris[i], '\0', kFeedURIMaxNumChars + 1);
                feed_ports[i] = 0;
                feed_is_active[i] = false;
                feed_protocols[i] = kNoReports;
#ifdef ON_PICO
                // Pico has access to EEPROM for receiver ID in device info.
                device_info.GetDefaultFeedReceiverID(feed_receiver_ids[i]);
#else
                // ESP32 will have to query for receiver ID later.
                memset(feed_receiver_ids[i], 0, kFeedReceiverIDNumBytes);
#endif
            }

            // Set default feed URIs.
            // adsb.fi: feed.adsb.fi:30004, Beast
            strncpy(feed_uris[kMaxNumFeeds - 1], "feed.adsb.fi", kFeedURIMaxNumChars);
            feed_uris[kMaxNumFeeds - 1][kFeedURIMaxNumChars] = '\0';
            feed_ports[kMaxNumFeeds - 1] = 30004;
            feed_is_active[kMaxNumFeeds - 1] = true;
            feed_protocols[kMaxNumFeeds - 1] = kBeastNoUAT;
            // airplanes.live: feed.airplanes.live:30004, Beast
            strncpy(feed_uris[kMaxNumFeeds - 2], "feed.airplanes.live", kFeedURIMaxNumChars);
            feed_uris[kMaxNumFeeds - 2][kFeedURIMaxNumChars] = '\0';
            feed_ports[kMaxNumFeeds - 2] = 30004;
            feed_is_active[kMaxNumFeeds - 2] = true;
            feed_protocols[kMaxNumFeeds - 2] = kBeastNoUAT;
            // adsb.lol: feed.adsb.lol:30004, Beast
            strncpy(feed_uris[kMaxNumFeeds - 3], "feed.adsb.lol", kFeedURIMaxNumChars);
            feed_uris[kMaxNumFeeds - 3][kFeedURIMaxNumChars] = '\0';
            feed_ports[kMaxNumFeeds - 3] = 30004;
            feed_is_active[kMaxNumFeeds - 3] = true;
            feed_protocols[kMaxNumFeeds - 3] = kBeastNoUAT;
            // whereplane.xyz: feed.whereplane.xyz:30004, Beast
            strncpy(feed_uris[kMaxNumFeeds - 4], "feed.whereplane.xyz", kFeedURIMaxNumChars);
            feed_uris[kMaxNumFeeds - 4][kFeedURIMaxNumChars] = '\0';
            feed_ports[kMaxNumFeeds - 4] = 30004;
            feed_is_active[kMaxNumFeeds - 4] = true;
            feed_protocols[kMaxNumFeeds - 4] = kBeast;
        }
    };

    // Maximum number of sinks that can be passed to CommsManager::UpdateReporting() in a single call:
    // serial interfaces on the Pico, IP feeds on the ESP32. Per-protocol sink arrays are sized with this.
    static constexpr uint16_t kMaxNumReportSinks = Settings::kMaxNumFeeds > SerialInterface::kNumSerialInterfaces
                                                       ? Settings::kMaxNumFeeds
                                                       : SerialInterface::kNumSerialInterfaces;

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
            kPNADSBeeWinglet = 10260008,               // ADSBee Winglet
        };

        enum ADSBee1090RFFrontendVersion : uint8_t {
            kADSBee1090RFFrontendV1 = 1,  // Original RF frontend used through ADSBee 1090U rev C.
            kADSBee1090RFFrontendV2 = 2,  // Revised FAST net filter resistor to 5.1kOhms.
            kADSBee1090RFFrontendV3 =
                3,  // Added schottky diode between FAST and LEVEL nets for improved pulse shaping.
        };

        /**
         * Default constructor.
         */
        DeviceInfo() {
            memset(part_code, '\0', kPartCodeLen + 1);
            for (uint16_t i = 0; i < kNumOTAKeys; i++) {
                memset(ota_keys[i], '\0', kOTAKeyMaxLen + 1);
            }
        }

        static constexpr uint16_t kDefaultSSIDLenChars = 24;  // ADSBee1090-YYYMMDDVVXXXX
        /**
         * Writes a default value for a network SSID to a buffer. The buffer must be at least kDefaultSSIDLenChars+1 so
         * that there is space for an end of string character. This default network SSID value is intended to not
         * conflict with any other ADSBee devices or future Pants for Birds products.
         * @param[out] buf Buffer to write the network SSID to.
         */
        void GetDefaultSSID(char* buf) {
            memcpy(buf, "ADSBee1090-", 11);       // [0:10] ADSBee1090-
            memcpy(buf + 11, part_code + 12, 7);  // [11:17] YYYMMDD
            memcpy(buf + 18, part_code + 20, 6);  // [18:23] VVXXXX
            buf[kDefaultSSIDLenChars] = '\0';
        }

        /**
         * Writes a default 8 Byte Unique ID to a buffer. The buffer must be at least 8 Bytes long. UID is in binary
         * (not human readable), and is of the form 0xBE 0xE0 NN NN NN NN, where the NN's represent bytes in a unique
         * value formed from the Base-10 integer YYYMMDDVVXXXX extracted from the ADSBee 1090's manufacturing code, sent
         * MSB first. The Unique ID is intended not to conflict between any two ADSBee 1090 devices. New devices in the
         * ADSBee lineup may be prefixed with 0xBE EN, where N is a value greater than 0.
         * @param[out] buf Buffer to write the 8 Byte unique ID to.
         */
        void GetDefaultFeedReceiverID(uint8_t* buf) {
            // 0xBE 0xE0 <6 Byte Binary UID, MSB first.>
            buf[0] = 0xBE;
            buf[1] = 0xE0;
            // Base the rest of the UID off of a 13-digit Base 10 number.
            char uid_digits[14];                        // YYYMMDDVVXXXX
            memcpy(uid_digits, part_code + 12, 7);      // YYYMMDD
            memcpy(uid_digits + 7, part_code + 20, 6);  // VVXXXX
            uid_digits[13] = '\0';
            // log2(10^13) = 43.18, so we need 44 (6 Bytes) bits to store the UID.
            uint64_t uid_value = strtoull(uid_digits, nullptr, 10);
            for (uint16_t i = 0; i < 6; i++) {
                buf[2 + i] = (uid_value >> (8 * (5 - i))) & 0xFF;
            }
        }

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
         * Determines the RF frontend version based on the part number and revision. This is used to tune PIO behavior
         * based on the analog response of the 1090MHz RF receiver frontend.
         * @retval ADSBee1090RFFrontendVersion enum value representing the RF frontend version.
         */
        inline ADSBee1090RFFrontendVersion GetRFFrontendVersion() {
            uint32_t part_number = GetPartNumber();
            char part_rev = GetPartRev();
            switch (part_number) {
                case kPNADSBee1090:
                    return kADSBee1090RFFrontendV1;
                    break;
                case kPNADSBee1090U:
                    if (part_rev <= 'D') {
                        return kADSBee1090RFFrontendV1;
                    } else if (part_rev == 'E') {
                        return kADSBee1090RFFrontendV2;
                    } else {
                        return kADSBee1090RFFrontendV3;
                    }
                    break;
                case kPNGS3MPoE:
                    if (part_rev <= 'E') {
                        return kADSBee1090RFFrontendV2;
                    } else {
                        return kADSBee1090RFFrontendV3;
                    }
                    break;
                case kPNADSBeem1090:
                    if (part_rev == 'A') {
                        return kADSBee1090RFFrontendV1;
                    } else {
                        return kADSBee1090RFFrontendV3;
                    }
                    break;
                case kPNADSBeem1090EvalBoard:
                    if (part_rev <= 'B') {
                        return kADSBee1090RFFrontendV2;
                    } else {
                        return kADSBee1090RFFrontendV3;
                    }
                    break;
                case kPNADSBeeWinglet:
                    return kADSBee1090RFFrontendV3;
                    break;
            }
            // Default to V1 for unknown part numbers, since that's the most common and safest assumption.
            return kADSBee1090RFFrontendV1;
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
     * Helper function for converting a GNSSReceiverType to its AT command value string. Returns "NONE" for
     * out-of-range values (e.g. from a corrupted settings blob) so the AT dump stays replayable.
     * @param[in] type GNSSReceiverType to convert to a string.
     * @retval String representation of the GNSSReceiverType, as it would be used in an AT command.
     */
    static inline const char* GNSSReceiverTypeToStr(GNSSReceiverType type) {
        switch (type) {
            case kGNSSReceiverGeneric:
                return "GENERIC";
            case kGNSSReceiverUBXMIA:
                return "UBX_MIA";
            case kGNSSReceiverNone:
            default:
                return "NONE";
        }
    }

    /**
     * Used to retrieve device information, either directly from EEPROM or via interprocessor SPI bus.
     * @param[in] device_info DeviceInfo struct to set.
     * @retval True if device info was retrieved successfully, false otherwise.
     */
    static bool GetDeviceInfo(DeviceInfo& device_info);

    /**
     * Loads settings from EEPROM. Assumes settings are stored at address 0x0 and doesn't do any integrity check.
     * @retval True if succeeded, false otherwise.
     */
    bool Load();

    /**
     * Clamps every enum-typed field in `settings` to its legal range, defaulting out-of-range values instead of
     * leaving them as-is. Settings migration can only be as trustworthy as the version tag on a stored blob (see the
     * NOTE in settings_migration.hh for a case where that tag was wrong for a real device), and other corruption
     * sources (a bit-flipped flash sector, a partially-written EEPROM page) are always possible too. An out-of-range
     * enum value used to index a kXxxStrs[] lookup table (Print(), PrintAT(), AT command handlers) is a hard fault on
     * the RP2040 and a crash/reboot loop on the ESP32 (which receives `settings` from the RP2040 over SPI at boot and
     * calls Print() immediately) -- i.e. a corrupted settings blob can brick the device on every subsequent boot,
     * surviving even a full reflash, because the settings flash/EEPROM sector isn't touched by an application update.
     * Called unconditionally at the end of Load(), after migration or the factory-defaults fallback, so nothing
     * downstream ever sees an out-of-range value regardless of where it came from.
     * @retval True if any field was out-of-range and got reset (caller should persist the fix), false if nothing
     *         needed changing.
     */
    bool Sanitize();

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
     * Saves settings to EEPROM. Stores settings at address 0x0 and performs no integrity check.
     * @retval True if succeeded, false otherwise.
     */
    bool Save();

    /**
     * Prints an 8-Byte receiver ID to a string buffer.
     * @param[in] receiver_id Pointer to first byte of an 8-Byte receiver ID.
     * @param[in] buf Buffer to write receiver ID string to. Must be at least 17 chars (including null terminator).
     */
    static inline void ReceiverIDToStr(uint8_t* receiver_id, char* buf) {
        snprintf(buf, 2 * SettingsManager::Settings::kFeedReceiverIDNumBytes + 1, "%02x%02x%02x%02x%02x%02x%02x%02x",
                 receiver_id[0], receiver_id[1], receiver_id[2], receiver_id[3], receiver_id[4], receiver_id[5],
                 receiver_id[6], receiver_id[7]);
    }

    /**
     * Restores settings to factory default values.
     */
    void ResetToDefaults();

#ifdef ON_PICO
    /**
     * Used to write device information to EEPROM during manufacturing. Only available on the Pico since it's the
     * one with direct access to EEPROm over I2C.
     * @param[in] device_info Reference to a DeviceInfo struct with the information to set in EEPROM.
     * @retval True if device info was set successfully, false otherwise.
     */
    static bool SetDeviceInfo(const DeviceInfo& device_info);

    /**
     * Sends the settings struct to the ESP32 and CC1312 via SPI. Call this after changing values that need to be
     * propagated to coprocessors, like log level or bias tee settings.
     */
    bool SyncToCoprocessors();
#endif

    Settings settings;

   private:
};

extern SettingsManager settings_manager;

#endif /* SETTINGS_HH_ */
