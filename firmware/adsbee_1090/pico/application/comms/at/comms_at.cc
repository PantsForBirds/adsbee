#include <stdio.h>  // for printing

#include <cctype>    // for isalnum
#include <cstring>   // for strcat
#include <iostream>  // for AT command ingestion

#include "adsbee.hh"
#include "comms.hh"
#include "core1.hh"  // For turning multiprocessor on / off during firmware flashing.
#include "cpu_utils.hh"
#include "eeprom.hh"
#include "esp32.hh"  // Access to ESP32 low level SPICoprocessorSlaveInterface.
#include "esp32_firmware_check.hh"
#include "esp32_flasher.hh"
#include "remote_id_guard.hh"
#include "firmware_update.hh"
#include "flash_utils.hh"
#include "main.hh"
#include "pico/bootrom.h"  // For Jumping to Bootloader
#include "pico/multicore.h"
#include "pico/stdlib.h"  // for getchar etc
#include "pico/unique_id.h"
#include "ram_monitor.hh"
#include "feed_ram_budget.hh"
#include "feed_receiver_id.hh"
#include "ble_output_status.hh"
#include "settings.hh"
#include "settings_hash.hh"
#include "spi_coprocessor.hh"  // For init / de-init before and after flashing ESP32.
#include "gnss_interface.hh"   // For parking/restoring the GNSS UART around ESP32 flashing.

#ifdef HARDWARE_UNIT_TESTS
#include "hardware_unit_tests.hh"
#include "unit_conversions.hh"  // ParseDeciNm
#endif

// For mapping cpp_at_printf
#include <cstdarg>
// #include "printf.h" // for using custom printf defined in printf.h
#include <cstdio>  // for using regular printf

// This is intended to stop people from accidentally modifying serial number information on their device.
const uint32_t kDeviceInfoProgrammingPassword = 0xDEDBEEF;
// Polling interval during OTA update. Faster than the normal ESP32 heartbeat for better transfer bandwidth.
// Heartbeat is required since the ESP32 firmware won't hand off the SPI mutex until it gets poked, so it needs a
// heartbeat between each message (no ACK required).
const uint32_t kOTAHeartbeatMs = 50;

/** CppAT Printf Override **/
int CppAT::cpp_at_printf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    int ret = comms_manager.iface_vprintf(SettingsManager::SerialInterface::kConsole, format, args);
    va_end(args);

    return ret;
}

/** AT Utility Functions **/

int32_t CommsManager::ATReadConsole(char* buf, uint16_t max_len, uint32_t timeout_ms, bool terminate_on_newline) {
    uint16_t len = 0;
    uint32_t start_ms = get_time_since_boot_ms();
    while (len < max_len) {
        int c = getchar_timeout_us(0);
        char ch;
        bool got_char = false;
        if (c >= 0) {
            ch = static_cast<char>(c);
            got_char = true;
        } else if (esp32.IsEnabled()) {
            if (esp32_console_rx_queue.Dequeue(ch)) {
                got_char = true;
            } else {
                esp32.Update();
                // Console output printed while waiting (e.g. by the caller just before) would otherwise stay queued
                // until the wait ends.
                UpdateNetworkConsole();
            }
        }

        if (got_char) {
            if (terminate_on_newline && (ch == '\r' || ch == '\n')) {
                while (getchar_timeout_us(0) >= 0) {
                }
                if (esp32.IsEnabled()) {
                    esp32_console_rx_queue.Clear();
                }
                break;
            }
            buf[len++] = ch;
            continue;
        }

        if (get_time_since_boot_ms() - start_ms > timeout_ms) {
            return -1;
        }
    }
    return static_cast<int32_t>(len);
}

/** AT Command Callback Functions **/

CPP_AT_CALLBACK(CommsManager::ATAutoUpdateCallback) {
    switch (op) {
        case '?':
            CPP_AT_CMD_PRINTF("=%d", settings_manager.settings.auto_update_enabled);
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=':
            if (CPP_AT_HAS_ARG(0)) {
                bool enabled;
                CPP_AT_TRY_ARG2NUM(0, enabled);
                settings_manager.settings.auto_update_enabled = enabled;
                settings_manager.SyncToCoprocessors();  // The ESP32 does the updating.
                CPP_AT_SUCCESS();
            }
            break;
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATBLEOutputCallback) {
    switch (op) {
        case '?':
            CPP_AT_CMD_PRINTF("=%s(MODE),%s(STATUS)",
                              SettingsManager::BLEOutputModeToStr(settings_manager.settings.ble_output_mode),
                              BLEOutputStatusStr(esp32_ll.ble_output_status));
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=': {
            char mode_str[16] = {0};
            if (!CPP_AT_HAS_ARG(0) || args[0].size() >= sizeof(mode_str)) {
                CPP_AT_ERROR("Requires a mode. AT+BLE_OUTPUT=<OFF|GDL90>");
            }
            memcpy(mode_str, args[0].data(), args[0].size());
            SettingsManager::BLEOutputMode mode;
            if (!SettingsManager::BLEOutputModeFromStr(mode_str, mode)) {
                CPP_AT_ERROR("Invalid Bluetooth output mode '%s'. AT+BLE_OUTPUT=<OFF|GDL90>", mode_str);
            }
            settings_manager.settings.ble_output_mode = mode;
            settings_manager.SyncToCoprocessors();  // The ESP32 runs the Bluetooth server.
            CPP_AT_SUCCESS();
            break;
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATBaudRateCallback) {
    switch (op) {
        case '?':
            CPP_AT_CMD_PRINTF("=%d(COMMS_UART),%d(GNSS_UART)",
                              settings_manager.settings.baud_rates[SettingsManager::SerialInterface::kCommsUART],
                              settings_manager.settings.baud_rates[SettingsManager::SerialInterface::kGNSSUART]);
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=':
            if (!(CPP_AT_HAS_ARG(0) && CPP_AT_HAS_ARG(1))) {
                CPP_AT_ERROR(
                    "Requires two arguments: AT+BAUD_RATE=<iface>,<baud_rate> where <iface> can be one of [COMMS_UART, "
                    "GNSS_UART].");
            }
            SettingsManager::SerialInterface iface;
            if (args[0].compare("COMMS_UART") == 0) {
                iface = SettingsManager::kCommsUART;
            } else if (args[0].compare("GNSS_UART") == 0) {
                iface = SettingsManager::kGNSSUART;
            } else {
                CPP_AT_ERROR("Invalid interface. Must be one of [COMMS_UART, GNSS_UART].");
            }
            uint32_t baud_rate;
            CPP_AT_TRY_ARG2NUM(1, baud_rate);
            if (!SetBaudRate(iface, baud_rate)) {
                CPP_AT_ERROR("Unable to set baud rate %d on interface %d.", baud_rate, iface);
            }
            CPP_AT_SUCCESS();
            break;
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATBiasTeeEnableCallback) {
    switch (op) {
        case '?':
            CPP_AT_CMD_PRINTF("=%d,%d", adsbee.BiasTeeIsEnabled(), settings_manager.settings.subg_bias_tee_enabled);
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=':
            bool enabled;
            if (CPP_AT_HAS_ARG(0)) {
                // The bias tee setting for the 1090 radio is applied directly, then scraped by the settings manager
                // during a settings save.
                CPP_AT_TRY_ARG2NUM(0, enabled);
                adsbee.SetBiasTeeEnable(enabled);
            }
            if (CPP_AT_HAS_ARG(1)) {
                // The bias tee setting for the sub-GHz radio is stored in the active settings struct, then synced to
                // the CC1312.
                CPP_AT_TRY_ARG2NUM(1, enabled);
                settings_manager.settings.subg_bias_tee_enabled = enabled;
                settings_manager.SyncToCoprocessors();
            }
            CPP_AT_SUCCESS();
            break;
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATConfigHashCallback) {
    switch (op) {
        case '?': {
            settings_manager.CollectLiveSettings();
            uint32_t settings_fleet = SettingsHash::Compute(settings_manager.settings, SettingsHash::kScopeFleet);
            uint32_t settings_full = SettingsHash::Compute(settings_manager.settings, SettingsHash::kScopeFull);

            // The running image can't change, so its CRC32 is calculated once.
            static bool rp2040_ok = false;
            static uint32_t rp2040_crc = 0;
            static uint32_t rp2040_len_bytes = 0;
            if (!rp2040_ok) {
                rp2040_ok = FirmwareUpdateManager::CalculateOwnImageCRC32(rp2040_crc, rp2040_len_bytes);
            }

            // First 4 Bytes of the SHA-256 of the application the ESP32 runs.
            uint32_t esp32_hash = 0;
            uint8_t esp32_sha256[ESP32FirmwareCheck::kHashLenBytes];
            bool esp32_ok =
                esp32.IsEnabled() && esp32.Read(ObjectDictionary::Address::kAddrESP32FirmwareHash, esp32_sha256);
            if (esp32_ok) {
                esp32_hash =
                    (esp32_sha256[0] << 24) | (esp32_sha256[1] << 16) | (esp32_sha256[2] << 8) | esp32_sha256[3];
            }

            uint32_t cc1312_crc = 0;
            uint32_t ccfg_crc = 0;
            bool cc1312_ok = adsbee.subg_radio_ll.GetFlashCRCs(cc1312_crc, ccfg_crc);

            // A hash that can't be read counts as 0.
            uint32_t firmware_hashes[] = {rp2040_ok ? rp2040_crc : 0, esp32_hash, cc1312_ok ? cc1312_crc : 0};
            uint32_t firmware = ConfigHasher::Combine(firmware_hashes, 3);
            uint32_t config_hashes[] = {settings_fleet, cc1312_ok ? ccfg_crc : 0, firmware};

            CPP_AT_CMD_PRINTF("=0x%08X", ConfigHasher::Combine(config_hashes, 3));
            CPP_AT_PRINTF("SETTINGS_FLEET=0x%08X (settings version %u, %u Bytes)\r\n", settings_fleet,
                          settings_manager.settings.settings_version, sizeof(SettingsManager::Settings));
            CPP_AT_PRINTF("SETTINGS_FULL=0x%08X\r\n", settings_full);
            if (cc1312_ok) {
                CPP_AT_PRINTF("CCFG_CC1312=0x%08X\r\n", ccfg_crc);
            } else {
                CPP_AT_PRINTF("CCFG_CC1312=UNKNOWN\r\n");
            }
            CPP_AT_PRINTF("FIRMWARE=0x%08X\r\n", firmware);
            if (rp2040_ok) {
#ifdef COMBINED_IMAGE
                const char* image_type = "UF2";
#else
                const char* image_type = "OTA";
#endif
                CPP_AT_PRINTF("FIRMWARE_RP2040=0x%08X (%s image, partition %u, %u Bytes)\r\n", rp2040_crc, image_type,
                              FirmwareUpdateManager::GetOwnFlashPartition(), rp2040_len_bytes);
            } else {
                CPP_AT_PRINTF("FIRMWARE_RP2040=UNKNOWN\r\n");
            }
            if (esp32_ok) {
                CPP_AT_PRINTF("FIRMWARE_ESP32=0x%08X\r\n", esp32_hash);
            } else {
                CPP_AT_PRINTF("FIRMWARE_ESP32=UNKNOWN\r\n");
            }
            if (cc1312_ok) {
                CPP_AT_PRINTF("FIRMWARE_CC1312=0x%08X\r\n", cc1312_crc);
            } else {
                CPP_AT_PRINTF("FIRMWARE_CC1312=UNKNOWN\r\n");
            }
            CPP_AT_SILENT_SUCCESS();
            break;
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATLEDBlinkCallback) {
    switch (op) {
        case '=': {
            uint32_t duration_ms;
            CPP_AT_TRY_ARG2NUM(0, duration_ms);
            if (duration_ms == 0 || duration_ms > ObjectDictionary::kLEDBlinkMaxDurationMs) {
                CPP_AT_ERROR("Duration must be 1-%lu ms.", (unsigned long)ObjectDictionary::kLEDBlinkMaxDurationMs);
            }
            // Parse all LED tokens before blinking anything so an invalid token doesn't half-execute the command.
            bool blink_1090 = false, blink_subg = false, blink_network = false;
            for (uint16_t i = 1; i < num_args; i++) {
                if (args[i].compare("1090") == 0) {
                    blink_1090 = true;
                } else if (args[i].compare("SUBG") == 0) {
                    blink_subg = true;
                } else if (args[i].compare("NETWORK") == 0) {
                    blink_network = true;
                } else if (args[i].compare("ALL") == 0) {
                    blink_1090 = blink_subg = blink_network = true;
                } else {
                    CPP_AT_ERROR("Invalid LED '%.*s'. Must be 1090, SUBG, NETWORK, or ALL.", (int)args[i].length(),
                                 args[i].data());
                }
            }
            // Fire all requested LEDs, collecting failures. LEDs that already fired are not rolled back.
            char failures[64] = "";
            if (blink_1090) {
                adsbee.ForceBlinkStatusLED(duration_ms);
            }
            if (blink_subg) {
                if (!adsbee.subg_radio.IsEnabled()) {
                    strcat(failures, " SUBG(disabled)");
                } else if (!adsbee.subg_radio.Write(ObjectDictionary::kAddrLEDBlink, duration_ms, true)) {
                    strcat(failures, " SUBG(write failed)");
                }
            }
            if (blink_network) {
                if (!esp32.IsEnabled()) {
                    strcat(failures, " NETWORK(disabled)");
                } else if (!esp32.Write(ObjectDictionary::kAddrLEDBlink, duration_ms, true)) {
                    strcat(failures, " NETWORK(write failed)");
                }
            }
            if (failures[0] != '\0') {
                CPP_AT_ERROR("Failed to blink:%s.", failures);
            }
            CPP_AT_SUCCESS();
            break;
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATLEDEnableCallback) {
    switch (op) {
        case '?':
            CPP_AT_CMD_PRINTF("=%d", settings_manager.settings.led_enabled);
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=':
            if (CPP_AT_HAS_ARG(0)) {
                bool enabled;
                CPP_AT_TRY_ARG2NUM(0, enabled);
                settings_manager.settings.led_enabled = enabled;
                // Force the status LED off immediately when disabling so it doesn't linger from the last blink.
                if (!enabled) {
                    adsbee.SetStatusLED(false);
                }
                // Sync to coprocessors so the ESP32 network LED honors the setting too.
                settings_manager.SyncToCoprocessors();
                CPP_AT_SUCCESS();
            }
            break;
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATFeedEnableCallback) {
    switch (op) {
        case '?':
            CPP_AT_CMD_PRINTF("=%d", settings_manager.settings.feeds_enabled);
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=':
            if (CPP_AT_HAS_ARG(0)) {
                bool enabled;
                CPP_AT_TRY_ARG2NUM(0, enabled);
                settings_manager.settings.feeds_enabled = enabled;
                // Sync to the ESP32 so it stops (or resumes) opening outbound feed sockets. When disabled, the
                // ESP32 IP WAN task closes all feed sockets so no data is sent to any feed.
                settings_manager.SyncToCoprocessors();
                CPP_AT_SUCCESS();
            }
            break;
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATGNSSCallback) {
    switch (op) {
        case '?':
            CPP_AT_CMD_PRINTF("=%d,%s,%d", settings_manager.settings.gnss_enabled,
                              GNSSModuleTypeToStr(SettingsToGNSSModuleType(settings_manager.settings.gnss_receiver_type)),
                              settings_manager.settings.gnss_notify);
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=': {
            if (!CPP_AT_HAS_ARG(0)) {
                CPP_AT_ERROR("Requires AT+GNSS=<0|1>[,<NONE|GENERIC|UBX_MIA>[,<0|1>]].");
            }

            bool enabled;
            if (args[0].compare("0") == 0) {
                enabled = false;
            } else if (args[0].compare("1") == 0) {
                enabled = true;
            } else {
                CPP_AT_ERROR("GNSS enable must be 0 or 1.");
            }

            // With no explicit type, preserve the configured receiver selection. Enabling is not
            // allowed when the saved type is NONE because there is no receiver implementation to use.
            BSP::GNSSModuleType type =
                SettingsToGNSSModuleType(settings_manager.settings.gnss_receiver_type);
            if (CPP_AT_HAS_ARG(1)) {
                if (args[1].compare("NONE") == 0) {
                    type = BSP::kGNSSModuleNone;
                } else if (args[1].compare("GENERIC") == 0) {
                    type = BSP::kGNSSModuleGeneric;
                } else if (args[1].compare("UBX_MIA") == 0) {
                    type = BSP::kGNSSModuleUbloxMAXM10;
                } else {
                    CPP_AT_ERROR("GNSS type must be NONE, GENERIC, or UBX_MIA.");
                }
            } else if (enabled && type == BSP::kGNSSModuleNone) {
                CPP_AT_ERROR("Cannot enable GNSS while the saved type is NONE; specify GENERIC or UBX_MIA.");
            }

            bool notify = false;
            if (CPP_AT_HAS_ARG(2)) {
                if (args[2].compare("0") == 0) {
                    notify = false;
                } else if (args[2].compare("1") == 0) {
                    notify = true;
                } else {
                    CPP_AT_ERROR("GNSS notify must be 0 or 1.");
                }
            }

            if (!ConfigureGNSSReceiver(enabled, type)) {
                settings_manager.settings.gnss_enabled = enabled;
                settings_manager.settings.gnss_receiver_type = GNSSModuleTypeToSettings(type);
                settings_manager.settings.gnss_notify = notify;
                CPP_AT_ERROR("GNSS receiver did not initialize successfully.");
            }
            settings_manager.settings.gnss_enabled = enabled && type != BSP::kGNSSModuleNone;
            settings_manager.settings.gnss_receiver_type = GNSSModuleTypeToSettings(type);
            settings_manager.settings.gnss_notify = notify;
            CPP_AT_SUCCESS();
            break;
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATGNSSFixCallback) {
    if (op != '?') {
        CPP_AT_ERROR("Only AT+GNSS_FIX? is supported.");
    }

    const NMEAParser::GNSSFix& fix = gnss->fix();
    char utc_time[9] = "--:--:--";
    if (fix.utc_time_valid) {
        snprintf(utc_time, sizeof(utc_time), "%02u:%02u:%02u", fix.utc_hour, fix.utc_minute, fix.utc_second);
    }
    CPP_AT_CMD_PRINTF("=%d,%.6f,%.6f,%ld,%.1f,%ld,%u,%s,%lu", gnss->HasValidFix(), fix.latitude_deg,
                      fix.longitude_deg, static_cast<long>(fix.altitude_ft), fix.heading_deg,
                      static_cast<long>(fix.speed_kts), static_cast<unsigned int>(fix.num_satellites), utc_time,
                      static_cast<unsigned long>(gnss->pps_count()));
    CPP_AT_SILENT_SUCCESS();
}

CPP_AT_CALLBACK(CommsManager::ATBootloader) {
    switch (op) {
        case '=': {
            if (CPP_AT_HAS_ARG(0)) {
                if (args[0].compare("1DEADBEE") == 0) {
                    CPP_AT_CMD_PRINTF(": Rebooting into USB bootloader\r\n");
                    rom_reset_usb_boot(0, 0);
                } else {
                    CPP_AT_ERROR("Invalid argument '%s'.", args[0].data());
                }
            }
        }
        default: {
            CPP_AT_ERROR("Operator %c not supported.", op);
        }
    }
}

CPP_AT_CALLBACK(CommsManager::ATDeviceInfoCallback) {
    switch (op) {
        case '?': {
            SettingsManager::DeviceInfo device_info;
            if (!settings_manager.GetDeviceInfo(device_info)) {
                CPP_AT_ERROR("Error while retrieving device info.");
            }
            device_info.part_code[SettingsManager::DeviceInfo::kPartCodeLen] =
                '\0';  // Use caution in case part code is invalid.
            CPP_AT_PRINTF("Part Code: %s\r\n", device_info.part_code);
            // Get 8-byte flash unique ID from RP2040 flash.
            pico_unique_board_id_t unique_id;
            pico_get_unique_board_id(&unique_id);
            CPP_AT_PRINTF("RP2040 Flash Unique ID: %02X%02X%02X%02X%02X%02X%02X%02X\r\n", unique_id.id[0],
                          unique_id.id[1], unique_id.id[2], unique_id.id[3], unique_id.id[4], unique_id.id[5],
                          unique_id.id[6], unique_id.id[7]);
            if (object_dictionary.kFirmwareVersionReleaseCandidate == 0) {
                // Indicates a finalized release; no need to print release candidate number.
                CPP_AT_PRINTF("RP2040 Firmware Version: %d.%d.%d\r\n", object_dictionary.kFirmwareVersionMajor,
                              object_dictionary.kFirmwareVersionMinor, object_dictionary.kFirmwareVersionPatch);
            } else {
                // Print release candidate number.
                CPP_AT_PRINTF("RP2040 Firmware Version: %d.%d.%d-rc%d\r\n", object_dictionary.kFirmwareVersionMajor,
                              object_dictionary.kFirmwareVersionMinor, object_dictionary.kFirmwareVersionPatch,
                              object_dictionary.kFirmwareVersionReleaseCandidate);
            }
#ifdef ADSBEE_DEBUG_BUILD
            CPP_AT_PRINTF("RP2040 Firmware Build: Debug\r\n");
#else
            CPP_AT_PRINTF("RP2040 Firmware Build: Release\r\n");
#endif
            RAMMonitor::Usage ram = RAMMonitor::GetUsage();
            CPP_AT_PRINTF("RP2040 RAM: %lu B heap, %lu B free, stack peak %lu B core 0, %lu B core 1\r\n",
                          (unsigned long)ram.heap_in_use_bytes, (unsigned long)ram.free_bytes,
                          (unsigned long)ram.core_0_stack_peak_bytes, (unsigned long)ram.core_1_stack_peak_bytes);
            CPP_AT_PRINTF("RP2040 Aircraft Dictionary: %u aircraft max\r\n",
                          (unsigned)AircraftDictionary::kMaxNumAircraft);

            for (uint16_t i = 0; i < SettingsManager::DeviceInfo::kNumOTAKeys; i++) {
                CPP_AT_PRINTF("OTA Key %d: %s\r\n", i, device_info.ota_keys[i]);
            }
            CPP_AT_PRINTF("1090MHz RF Frontend Version: %d\r\n", bsp.r1090_rf_frontend_version);

            if (esp32.IsEnabled()) {
                // Read ESP32 firmware version.
                uint32_t esp32_firmware_version;
                if (!esp32.Read(ObjectDictionary::Address::kAddrFirmwareVersion, esp32_firmware_version)) {
                    CPP_AT_ERROR("ESP32 firmware version read failed!");
                }
                uint8_t esp32_fwv_major = (esp32_firmware_version >> 24) & 0xFF;
                uint8_t esp32_fwv_minor = (esp32_firmware_version >> 16) & 0xFF;
                uint8_t esp32_fwv_patch = (esp32_firmware_version >> 8) & 0xFF;
                uint8_t esp32_fwv_rc = esp32_firmware_version & 0xFF;
                if (esp32_fwv_rc == 0) {
                    // Finalized release version. No need to print release candidate number.
                    CPP_AT_PRINTF("ESP32 Firmware Version: %d.%d.%d\r\n", esp32_fwv_major, esp32_fwv_minor,
                                  esp32_fwv_patch);
                } else {
                    // Print with release candidate number.
                    CPP_AT_PRINTF("ESP32 Firmware Version: %d.%d.%d-rc%d\r\n", esp32_fwv_major, esp32_fwv_minor,
                                  esp32_fwv_patch, esp32_fwv_rc);
                }

                ObjectDictionary::ESP32DeviceInfo esp32_device_info;
                if (!esp32.Read(ObjectDictionary::Address::kAddrESP32DeviceInfo, esp32_device_info,
                                sizeof(esp32_device_info))) {
                    CPP_AT_ERROR("ESP32 device info read failed!");
                }
                CPP_AT_PRINTF("ESP32 Base MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                              esp32_device_info.base_mac[0], esp32_device_info.base_mac[1],
                              esp32_device_info.base_mac[2], esp32_device_info.base_mac[3],
                              esp32_device_info.base_mac[4], esp32_device_info.base_mac[5]);
                CPP_AT_PRINTF("ESP32 WiFi Station MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                              esp32_device_info.wifi_station_mac[0], esp32_device_info.wifi_station_mac[1],
                              esp32_device_info.wifi_station_mac[2], esp32_device_info.wifi_station_mac[3],
                              esp32_device_info.wifi_station_mac[4], esp32_device_info.wifi_station_mac[5]);
                CPP_AT_PRINTF("ESP32 WiFi AP MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                              esp32_device_info.wifi_ap_mac[0], esp32_device_info.wifi_ap_mac[1],
                              esp32_device_info.wifi_ap_mac[2], esp32_device_info.wifi_ap_mac[3],
                              esp32_device_info.wifi_ap_mac[4], esp32_device_info.wifi_ap_mac[5]);
                CPP_AT_PRINTF("ESP32 Bluetooth MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                              esp32_device_info.bluetooth_mac[0], esp32_device_info.bluetooth_mac[1],
                              esp32_device_info.bluetooth_mac[2], esp32_device_info.bluetooth_mac[3],
                              esp32_device_info.bluetooth_mac[4], esp32_device_info.bluetooth_mac[5]);
                CPP_AT_PRINTF("ESP32 Ethernet MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                              esp32_device_info.ethernet_mac[0], esp32_device_info.ethernet_mac[1],
                              esp32_device_info.ethernet_mac[2], esp32_device_info.ethernet_mac[3],
                              esp32_device_info.ethernet_mac[4], esp32_device_info.ethernet_mac[5]);
                if (!(esp32_ll.hardware_capabilities & ObjectDictionary::kESP32HWCapReported)) {
                    CPP_AT_PRINTF("ESP32 PSRAM: UNKNOWN\r\n");
                } else if (esp32_ll.hardware_capabilities & ObjectDictionary::kESP32HWCapPSRAM) {
                    CPP_AT_PRINTF("ESP32 PSRAM: %u KB (%u KB free)\r\n", esp32_ll.psram_total_kb,
                                  esp32_ll.psram_free_kb);
                } else {
                    CPP_AT_PRINTF("ESP32 PSRAM: NONE\r\n");
                }
                CPP_AT_PRINTF("ESP32 Heap: %lu KB free, %lu KB largest block\r\n",
                              (unsigned long)(esp32_ll.heap_free_bytes / 1024),
                              (unsigned long)(esp32_ll.heap_largest_free_block_bytes / 1024));
            } else {
                CPP_AT_PRINTF("ESP32 Disabled\r\n");
            }

            CPP_AT_SILENT_SUCCESS();
            break;
        }
        case '=': {
            // AT+DEVICE_INFO=<programming_password>,<device_info_version>,<part_code>,<ota_key_0>,<ota_key_1>
            if (!CPP_AT_HAS_ARG(0)) {
                CPP_AT_ERROR("Missing programming password.");
            } else {
                uint32_t programming_password;
                CPP_AT_TRY_ARG2NUM_BASE(0, programming_password, 16);
                if (programming_password != kDeviceInfoProgrammingPassword) {
                    CPP_AT_ERROR("Programming password 0x%x does not match.", programming_password);
                }
                // Program device information here.
                SettingsManager::DeviceInfo device_info;

                if (CPP_AT_HAS_ARG(1)) {
                    uint16_t device_info_version;
                    CPP_AT_TRY_ARG2NUM(1, device_info_version);
                    if (device_info_version != kDeviceInfoVersion) {
                        CPP_AT_ERROR("Device info version must be %d, got %d.", device_info_version,
                                     kDeviceInfoVersion);
                    }
                    if (device_info_version == device_info.device_info_version) {
                        // Existing device info is compatible, use existing DeviceInfo struct as a starter.
                        settings_manager.GetDeviceInfo(device_info);
                    }
                }
                // Else (no argument provided for device info), use default values for everything.
                if (CPP_AT_HAS_ARG(2)) {
                    // Copy part code while guarding against part codes that are too long.
                    strncpy(device_info.part_code, args[2].data(), SettingsManager::DeviceInfo::kPartCodeLen);
                    device_info.part_code[SettingsManager::DeviceInfo::kPartCodeLen] = '\0';
                }
                for (uint16_t i = 0; i < SettingsManager::DeviceInfo::kNumOTAKeys; i++) {
                    uint16_t arg_index = i + 3;
                    if (CPP_AT_HAS_ARG(arg_index)) {
                        // Set OTA key 0.
                        strncpy(device_info.ota_keys[i], args[arg_index].data(),
                                SettingsManager::DeviceInfo::kOTAKeyMaxLen);
                        device_info.ota_keys[i][SettingsManager::DeviceInfo::kOTAKeyMaxLen] = '\0';
                    }
                }
                if (settings_manager.SetDeviceInfo(device_info)) {
                    CPP_AT_SUCCESS();
                }
                CPP_AT_ERROR("Error while attempting to set device info.");
            }
            break;
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATESP32EnableCallback) {
    switch (op) {
        case '?':
            CPP_AT_CMD_PRINTF("=%d", esp32.IsEnabled());
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=':
            if (!CPP_AT_HAS_ARG(0)) {
                CPP_AT_ERROR("Requires an argument (0 or 1). AT+ESP32_ENABLED=<enabled>");
            }
            bool enabled;
            bool already_enabled = esp32.IsEnabled();
            CPP_AT_TRY_ARG2NUM(0, enabled);
            if (enabled && !already_enabled) {
                esp32.Init();
            } else if (!enabled && already_enabled) {
                esp32.DeInit();
            }
            CPP_AT_SUCCESS();
            break;
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATESP32FlashCallback) {
    if (!esp32.DeInit()) {
        CPP_AT_ERROR("CommsManager::ATESP32FlashCallback", "Error while de-initializing ESP32 before flashing.");
    }
    // The ESP32 flasher and the GNSS module share uart0 (on different pins). Release the GNSS pins
    // so the module's NMEA/UBX stream can't corrupt the ESP-ROM bootloader handshake. No-op if GNSS
    // is absent/inactive. The module stays powered so it hot-starts on resume.
    gnss->SuspendForUartHandover();
    // Manually stop and start core 1 and watchdog instead of using FlashSafe() and FlashUnsafe() since we aren't
    // actually writing to RP2040 flash memory and we want printouts to work over the USB console.
    StopCore1();
    adsbee.DisableWatchdog();
    bool flashed_successfully = esp32_flasher.FlashESP32();
    adsbee.EnableWatchdog();
    StartCore1();
    // Re-claim uart0 for the GNSS module (FlashESP32() deinit'd it) and resume NMEA output. Do this
    // whether or not the flash succeeded so GNSS always comes back.
    gnss->ResumeAfterUartHandover();
    if (!flashed_successfully) {
        CPP_AT_ERROR("CommsManager::ATESP32FlashCallback", "Error while flashing ESP32.");
    }

    esp32.firmware_update_failed = false;  // The ESP32 now runs this build's firmware.
    if (!esp32.Init()) {
        CPP_AT_ERROR("CommsManager::ATESP32FlashCallback", "Error while re-initializing ESP32 after flashing.");
    }

    CONSOLE_INFO("CommsManager::ATESP32FlashCallback", "ESP32 successfully flashed.");
    CPP_AT_SUCCESS();
}

CPP_AT_CALLBACK(CommsManager::ATESP32RebootInfoCallback) {
    switch (op) {
        case '?': {
            if (!esp32.IsEnabled()) {
                CPP_AT_ERROR("ESP32 is not enabled.");
            }
            ObjectDictionary::ESP32RebootInfo reboot_info;
            if (!esp32.Read(ObjectDictionary::Address::kAddrESP32RebootInfo, reboot_info, sizeof(reboot_info))) {
                CPP_AT_ERROR("Failed to read ESP32 reboot info.");
            }
            CPP_AT_PRINTF("ESP32 Reset Reason: %d (%s)\r\n", reboot_info.reset_reason, reboot_info.reset_reason_str);
            CPP_AT_PRINTF("ESP32 Core Dump To Flash: %s\r\n",
                          reboot_info.core_dump_to_flash_enabled ? "enabled" : "disabled");
            if (reboot_info.core_dump_to_flash_enabled) {
                CPP_AT_PRINTF("ESP32 Core Dump: printing to console log...\r\n");
            }
            CPP_AT_SILENT_SUCCESS();
            break;
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

#ifdef HARDWARE_UNIT_TESTS
CPP_AT_CALLBACK(CommsManager::ATESP32TriggerAbortCallback) {
    if (!esp32.IsEnabled()) {
        CPP_AT_ERROR("ESP32 is not enabled.");
    }
    uint8_t dummy = 0;
    esp32.Write(ObjectDictionary::Address::kAddrESP32TriggerAbort, dummy);
    CPP_AT_PRINTF("ESP32 abort triggered.\r\n");
    CPP_AT_SILENT_SUCCESS();
}
#endif

// The ESP32 image runs on modules with and without PSRAM (ESP32-S3-MINI-1U-N4R2 vs -N8) and reports which one it is in
// ESP32DeviceStatus::hardware_capabilities. Returns true only once the ESP32 has positively reported "no PSRAM"; while
// the capability is unknown (ESP32 disabled, not read yet, or older ESP32 firmware) settings are accepted and the ESP32's
// own runtime gating (RemoteIDManager) remains the backstop.
static bool ESP32KnownToLackPSRAM() {
    return (esp32_ll.hardware_capabilities & ObjectDictionary::kESP32HWCapReported) &&
           !(esp32_ll.hardware_capabilities & ObjectDictionary::kESP32HWCapPSRAM);
}

// Remote ID transports this hardware can run with the enabled network interfaces (all while PSRAM is unknown).
static uint8_t RemoteIDAllowedTransports() {
    const SettingsManager::Settings::CoreNetworkSettings& cns = settings_manager.settings.core_network_settings;
    return RemoteIDGuard::AllowedTransports(!ESP32KnownToLackPSRAM(), cns.wifi_ap_enabled, cns.wifi_sta_enabled,
                                            cns.ethernet_enabled);
}

// Why this hardware can't run the given Remote ID transports with the enabled network interfaces, or nullptr if it can.
static const char* RemoteIDTransportsError(const char* command, uint8_t transports) {
    static char error[300];
    char names[RemoteIDGuard::kTransportsStrMaxLen];
    const uint8_t allowed = RemoteIDAllowedTransports();
    if ((transports & ~allowed) == 0) return nullptr;
    if (allowed == 0) {
        return "Remote ID can't run while WiFi AP or WiFi Station is enabled on this hardware: its ESP32 has no PSRAM, "
               "so there isn't enough RAM for the web server and Remote ID at the same time. Disable WiFi and use "
               "Ethernet or USB, or use hardware with PSRAM.";
    }
    snprintf(error, sizeof(error),
             "Only the Bluetooth transports can run while Ethernet is enabled on this hardware: its ESP32 has no "
             "PSRAM, so there isn't enough RAM for the web server, Bluetooth and the WiFi beacon receiver at the same "
             "time. Use AT+%s=1,%s, disable Ethernet, or use hardware with PSRAM.",
             command, RemoteIDGuard::TransportsStr(allowed, names, sizeof(names)));
    return error;
}

// Warns when a network interface is being enabled while Remote ID uses transports it rules out on hardware without
// PSRAM. Networking wins (it is how users reach the device), so the setting is accepted and those transports pause.
static void WarnIfNetworkPausesRemoteID() {
    const SettingsManager::Settings& s = settings_manager.settings;
    const uint8_t allowed = RemoteIDAllowedTransports();
    if (!(s.remote_id_rx_enabled && (s.remote_id_transports & ~allowed)) &&
        !(s.remote_id_tx_enabled && (s.remote_id_tx_transports & ~allowed))) {
        return;
    }
    CPP_AT_PRINTF(
        "WARNING: this hardware's ESP32 has no PSRAM, so Remote ID will be paused while WiFi is enabled, and only its "
        "Bluetooth transports run while Ethernet is enabled.\r\n");
}

// Explains why requested Remote ID isn't running, from the ESP32's live status.
static void PrintRemoteIDBlockedReason(bool requested, uint16_t status_mask) {
    if (!requested) return;
    const char* reason = RemoteIDGuard::BlockedReason(esp32_ll.remote_id_status & status_mask);
    if (reason != nullptr) CPP_AT_PRINTF("Remote ID %s.\r\n", reason);
}

CPP_AT_CALLBACK(CommsManager::ATEthernetCallback) {
    switch (op) {
        case '?':
            CPP_AT_CMD_PRINTF("=%d", settings_manager.settings.core_network_settings.ethernet_enabled);
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=':
            if (!CPP_AT_HAS_ARG(0)) {
                CPP_AT_ERROR("Requires an argument (0 or 1). AT+ETHERNET=<enabled>");
            }
            bool enabled;
            CPP_AT_TRY_ARG2NUM(0, settings_manager.settings.core_network_settings.ethernet_enabled);
            CPP_AT_CMD_PRINTF(": ethernet_enabled: %d\r\n",
                              settings_manager.settings.core_network_settings.ethernet_enabled);
            if (settings_manager.settings.core_network_settings.ethernet_enabled) WarnIfNetworkPausesRemoteID();
            CPP_AT_SUCCESS();
            break;
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATRemoteIDCallback) {
    switch (op) {
        case '?': {
            // Report the requested settings plus the ESP32's resolved live status (why RID may not be running).
            // remote_id_status is cached on the low-level ESP32 slave interface (esp32_ll) by ESP32::Update(); the
            // high-level SPICoprocessor `esp32` only inherits the base interface, which does not carry it.
            char transports[RemoteIDGuard::kTransportsStrMaxLen], status[RemoteIDGuard::kStatusStrMaxLen],
                allowed[RemoteIDGuard::kTransportsStrMaxLen];
            CPP_AT_CMD_PRINTF(
                "=%d,%s(TRANSPORTS),%s(ESP32_STATUS),%s(ALLOWED_TRANSPORTS)",
                settings_manager.settings.remote_id_rx_enabled,
                RemoteIDGuard::TransportsStr(settings_manager.settings.remote_id_transports, transports,
                                             sizeof(transports)),
                RemoteIDGuard::StatusStr(esp32_ll.remote_id_status & 0x00FF, status, sizeof(status)),
                RemoteIDGuard::TransportsStr(RemoteIDAllowedTransports(), allowed, sizeof(allowed)));
            PrintRemoteIDBlockedReason(settings_manager.settings.remote_id_rx_enabled, 0x00FF);
            CPP_AT_SILENT_SUCCESS();
            break;
        }
        case '=':
            if (!CPP_AT_HAS_ARG(0)) {
                CPP_AT_ERROR("Requires at least one argument. AT+REMOTE_ID=<enabled>[,<transports>]");
            }
            bool rx_enabled;
            CPP_AT_TRY_ARG2NUM(0, rx_enabled);
            uint8_t transports = settings_manager.settings.remote_id_transports;
            if (CPP_AT_HAS_ARG(1) && !RemoteIDGuard::ParseTransports(args[1], transports)) {
                CPP_AT_ERROR(
                    "Invalid transports '%.*s': use BT4, BT5_LR and WIFI_BEACON joined with + (e.g. BT4+BT5_LR), ALL or "
                    "NONE. Numeric masks are no longer accepted.",
                    static_cast<int>(args[1].length()), args[1].data());
            }
            const char* transports_error = RemoteIDTransportsError("REMOTE_ID", transports);
            if (rx_enabled && transports_error != nullptr) {
                CPP_AT_ERROR("%s", transports_error);
            }
            settings_manager.settings.remote_id_rx_enabled = rx_enabled;
            settings_manager.settings.remote_id_transports = transports;
            // Push the new receive configuration to the ESP32, which owns the radios.
            settings_manager.SyncToCoprocessors();
            char names[RemoteIDGuard::kTransportsStrMaxLen];
            CPP_AT_CMD_PRINTF(": remote_id_rx_enabled: %d, transports: %s\r\n",
                              settings_manager.settings.remote_id_rx_enabled,
                              RemoteIDGuard::TransportsStr(transports, names, sizeof(names)));
            CPP_AT_SUCCESS();
            break;
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

// Warns when Remote ID transmit is enabled alongside a receiver position source that may not be broadcast. A Remote ID
// broadcast asserts "this is where I am", so only FIXED (an operator-entered coordinate for this device) and GNSS (this
// device's own fix) are transmitted; the aircraft-derived sources carry a *received* aircraft's position, and the ESP32
// suppresses them, transmitting an unknown position instead. Printed rather than logged so it reaches whichever
// interface issued the command regardless of log level. Advisory only: the setting still takes effect.
static void WarnIfRemoteIDTxPositionSourceNotTransmittable() {
    if (!settings_manager.settings.remote_id_tx_enabled) return;
    SettingsManager::RxPosition::PositionSource source = adsbee.rx_position.source;
    if (SettingsManager::RxPosition::MayBeTransmittedAsOwnPosition(source)) return;
    CPP_AT_PRINTF(
        "WARNING: Remote ID transmit is enabled but AT+RX_POSITION source is %s, which is not this device's own "
        "position and will NOT be transmitted. An unknown position will be broadcast instead. Use AT+RX_POSITION=FIXED "
        "or AT+RX_POSITION=GNSS to transmit a position.\r\n",
        SettingsManager::RxPosition::kPositionSourceStrs[source]);
}

void ATRemoteIDTxHelpCallback() {
    CPP_AT_PRINTF(
        "\t[EXPERIMENTAL] Remote ID is not yet a stable part of the system; its settings and behavior may change in "
        "future firmware.\r\n"
        "\tAT+REMOTE_ID_TX=<enabled>[,<transports>[,<uas_id>[,<id_type>[,<ua_type>[,<operator_id>]]]]]\r\n"
        "\tTransmit Broadcast Remote ID (drone ID) from this device, for use as a Remote ID test transmitter or as a "
        "combined ADS-B receiver / Remote ID transmitter on a drone. No WiFi AP/STA is required.\r\n"
        "\ttransports = BT4, BT5_LR and WIFI_BEACON joined with +, ALL or NONE (default ALL), e.g. BT4+BT5_LR.\r\n"
        "\tuas_id = UAS serial / registration, up to %d chars ('-' to clear and use this device's serial).\r\n"
        "\tid_type = [1 SERIAL, 2 CAA_REGISTRATION, 3 UTM_UUID, 4 SESSION_ID].\r\n"
        "\tua_type = [1 AEROPLANE, 2 HELICOPTER_MULTIROTOR, ... 15 OTHER].\r\n"
        "\toperator_id = operator registration, up to %d chars ('-' to clear).\r\n"
        "\tThe transmitted position comes from AT+RX_POSITION, but ONLY from the FIXED and GNSS sources, which\r\n"
        "\tdescribe this device. The aircraft-derived sources (LOWEST, ICAO) carry a received aircraft's position;\r\n"
        "\tbroadcasting those would place a fabricated drone on a real aircraft, so an unknown position is sent\r\n"
        "\tinstead.\r\n"
        "\tAT+REMOTE_ID_TX?\r\n\tQuery transmit settings, the ESP32's live status and the allowed transports.\r\n"
        "\tTransmit status words: TX_BT4, TX_BT5_LR, TX_WIFI_BEACON, TX_NO_POSITION, TX_BLOCKED,\r\n"
        "\tTX_POSITION_SOURCE_NOT_ALLOWED, plus the receive words of AT+REMOTE_ID?.",
        SettingsManager::Settings::kRemoteIDIDMaxLen, SettingsManager::Settings::kRemoteIDIDMaxLen);
}

CPP_AT_CALLBACK(CommsManager::ATRemoteIDTxCallback) {
    switch (op) {
        case '?': {
            // Report the requested settings plus the ESP32's resolved live status (the high byte carries the transmit
            // state; see RemoteIDGuard::Status).
            char transports[RemoteIDGuard::kTransportsStrMaxLen], status[RemoteIDGuard::kStatusStrMaxLen],
                allowed[RemoteIDGuard::kTransportsStrMaxLen];
            CPP_AT_CMD_PRINTF("=%d,%s(TRANSPORTS),%s(UAS_ID),%d(ID_TYPE),%d(UA_TYPE),%s(OPERATOR_ID),%s(ESP32_STATUS),"
                              "%s(ALLOWED_TRANSPORTS)",
                              settings_manager.settings.remote_id_tx_enabled,
                              RemoteIDGuard::TransportsStr(settings_manager.settings.remote_id_tx_transports,
                                                           transports, sizeof(transports)),
                              settings_manager.settings.remote_id_tx_uas_id[0] == '\0'
                                  ? "(device serial)"
                                  : settings_manager.settings.remote_id_tx_uas_id,
                              settings_manager.settings.remote_id_tx_uas_id_type,
                              settings_manager.settings.remote_id_tx_ua_type,
                              settings_manager.settings.remote_id_tx_operator_id[0] == '\0'
                                  ? "(none)"
                                  : settings_manager.settings.remote_id_tx_operator_id,
                              RemoteIDGuard::StatusStr(esp32_ll.remote_id_status, status, sizeof(status)),
                              RemoteIDGuard::TransportsStr(RemoteIDAllowedTransports(), allowed, sizeof(allowed)));
            PrintRemoteIDBlockedReason(settings_manager.settings.remote_id_tx_enabled, 0xFFFF);
            CPP_AT_SILENT_SUCCESS();
            break;
        }
        case '=': {
            if (!CPP_AT_HAS_ARG(0)) {
                CPP_AT_ERROR("Requires at least one argument. AT+REMOTE_ID_TX=<enabled>[,<transports>]");
            }
            bool tx_enabled;
            CPP_AT_TRY_ARG2NUM(0, tx_enabled);
            uint8_t tx_transports = settings_manager.settings.remote_id_tx_transports;
            if (CPP_AT_HAS_ARG(1) && !RemoteIDGuard::ParseTransports(args[1], tx_transports)) {
                CPP_AT_ERROR(
                    "Invalid transports '%.*s': use BT4, BT5_LR and WIFI_BEACON joined with + (e.g. BT4+BT5_LR), ALL or "
                    "NONE. Numeric masks are no longer accepted.",
                    static_cast<int>(args[1].length()), args[1].data());
            }
            const char* transports_error = RemoteIDTransportsError("REMOTE_ID_TX", tx_transports);
            if (tx_enabled && transports_error != nullptr) {
                CPP_AT_ERROR("%s", transports_error);
            }
            settings_manager.settings.remote_id_tx_enabled = tx_enabled;
            settings_manager.settings.remote_id_tx_transports = tx_transports;
            if (CPP_AT_HAS_ARG(2)) {
                // "-" clears the UAS ID, which makes the transmitter fall back to this device's own serial number.
                if (args[2].compare("-") == 0) {
                    memset(settings_manager.settings.remote_id_tx_uas_id, '\0',
                           sizeof(settings_manager.settings.remote_id_tx_uas_id));
                } else {
                    if (args[2].length() > SettingsManager::Settings::kRemoteIDIDMaxLen) {
                        CPP_AT_ERROR("UAS ID must be %d characters or fewer, got %d.",
                                     SettingsManager::Settings::kRemoteIDIDMaxLen, args[2].length());
                    }
                    memset(settings_manager.settings.remote_id_tx_uas_id, '\0',
                           sizeof(settings_manager.settings.remote_id_tx_uas_id));
                    strncpy(settings_manager.settings.remote_id_tx_uas_id, args[2].data(), args[2].length());
                }
            }
            if (CPP_AT_HAS_ARG(3)) {
                CPP_AT_TRY_ARG2NUM(3, settings_manager.settings.remote_id_tx_uas_id_type);
            }
            if (CPP_AT_HAS_ARG(4)) {
                CPP_AT_TRY_ARG2NUM(4, settings_manager.settings.remote_id_tx_ua_type);
            }
            if (CPP_AT_HAS_ARG(5)) {
                if (args[5].compare("-") == 0) {
                    memset(settings_manager.settings.remote_id_tx_operator_id, '\0',
                           sizeof(settings_manager.settings.remote_id_tx_operator_id));
                } else {
                    if (args[5].length() > SettingsManager::Settings::kRemoteIDIDMaxLen) {
                        CPP_AT_ERROR("Operator ID must be %d characters or fewer, got %d.",
                                     SettingsManager::Settings::kRemoteIDIDMaxLen, args[5].length());
                    }
                    memset(settings_manager.settings.remote_id_tx_operator_id, '\0',
                           sizeof(settings_manager.settings.remote_id_tx_operator_id));
                    strncpy(settings_manager.settings.remote_id_tx_operator_id, args[5].data(), args[5].length());
                }
            }
            // Push the new transmit configuration to the ESP32, which owns the radios.
            settings_manager.SyncToCoprocessors();
            char names[RemoteIDGuard::kTransportsStrMaxLen];
            CPP_AT_CMD_PRINTF(": remote_id_tx_enabled: %d, transports: %s\r\n",
                              settings_manager.settings.remote_id_tx_enabled,
                              RemoteIDGuard::TransportsStr(tx_transports, names, sizeof(names)));
            WarnIfRemoteIDTxPositionSourceNotTransmittable();
            CPP_AT_SUCCESS();
            break;
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

void ATFeedHelpCallback() {
    CPP_AT_PRINTF(
        "\tAT+FEED=<index>,<uri>,<port>,<active>,<protocol>[,<transport>]\r\n\tSet details for a "
        "network feed. Empty arguments keep their value.\r\n\tindex = [0-%d], uri = IP address or hostname (\"-\" "
        "clears it), port = [0-65535], active = [0 1], protocol = [BEAST RAW AIRCRAFT_JSON ...], transport = [TCP UDP "
        "MQTT].\r\n\tWithout a transport, a feed whose protocol changes takes the protocol's default: UDP for GDL90 "
        "and MAVLink, MQTT for AIRCRAFT_JSON, TCP for the others.\r\n\tUDP sends whole frames per datagram to a host, "
        "broadcast or multicast address. MQTT feeds publish to the broker set with AT+MQTT and ignore uri and port."
        "\r\n\tA feed is refused if the active feeds would need more ESP32 RAM than the unit has for them."
        "\r\n\t\r\n\tAT+FEED?\r\n\tPrint details for all feeds.\r\n\t\r\n\tAT+FEED?<index>\r\n\tPrint details for a "
        "specific feed.\r\n\tfeed_index = [0-%d]\r\n",
        SettingsManager::Settings::kMaxNumFeeds - 1, SettingsManager::Settings::kMaxNumFeeds - 1);
}

// Feed RAM budget of this unit, or 0 while the ESP32 hasn't reported whether it has PSRAM (no check then).
static uint32_t FeedRAMBudgetBytes() {
    if (!(esp32_ll.hardware_capabilities & ObjectDictionary::kESP32HWCapReported)) {
        return 0;
    }
    return FeedRAMBudget::BudgetBytes(settings_manager.settings,
                                      esp32_ll.hardware_capabilities & ObjectDictionary::kESP32HWCapPSRAM);
}

// True if feed settings that went from old_used_bytes to the current use fit the RAM budget. Changes that don't add to
// the use always fit.
static bool FeedRAMFits(uint32_t old_used_bytes, uint32_t& used_bytes, uint32_t& budget_bytes) {
    used_bytes = FeedRAMBudget::UsedBytes(settings_manager.settings);
    budget_bytes = FeedRAMBudgetBytes();
    return budget_bytes == 0 || used_bytes <= budget_bytes || used_bytes <= old_used_bytes;
}

#define FEED_RAM_ERROR_FORMAT                                                                                           \
    "Not enough ESP32 RAM: the active feeds would need %lu kB of the %lu kB this unit has for feeds. Turn off a feed, " \
    "or use UDP, or MQTT without TLS."

static void PrintFeed(const CppAT::ATCommandDef_t& def, uint16_t index) {
    const SettingsManager::Settings& s = settings_manager.settings;
    char receiver_id_str[SettingsManager::Settings::kFeedReceiverIDNumBytes * 2 + 1];
    SettingsManager::ReceiverIDToStr(settings_manager.settings.feed_receiver_ids[index], receiver_id_str);
    CPP_AT_CMD_PRINTF("=%d(INDEX),%s(URI),%d(PORT),%d(ACTIVE),%s(PROTOCOL),%s(RECEIVER_ID),%s(TRANSPORT)", index,
                      s.feed_uris[index], s.feed_ports[index], s.feed_is_active[index],
                      SettingsManager::kReportingProtocolStrs[s.feed_protocols[index]], receiver_id_str,
                      SettingsManager::kFeedTransportStrs[s.feed_transports[index]]);
}

CPP_AT_CALLBACK(CommsManager::ATFeedCallback) {
    switch (op) {
        case '?':
            if (CPP_AT_HAS_ARG(0)) {
                // Querying info about a specific feed.
                uint16_t index = UINT16_MAX;
                CPP_AT_TRY_ARG2NUM(0, index);
                if (index >= SettingsManager::Settings::kMaxNumFeeds) {
                    CPP_AT_ERROR("Feed number must be between 0-%d, no details for feed with index %d.",
                                 SettingsManager::Settings::kMaxNumFeeds - 1, index);
                }
                PrintFeed(def, index);
            } else {
                // Querying info about all feeds.
                for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
                    PrintFeed(def, i);
                }
            }
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=': {
            // Setting feed information for a specific feed.
            uint16_t index = UINT16_MAX;
            if (!CPP_AT_HAS_ARG(0)) {
                CPP_AT_ERROR("Feed index is required for setting feed information.");
            }
            CPP_AT_TRY_ARG2NUM(0, index);
            if (index >= SettingsManager::Settings::kMaxNumFeeds) {
                CPP_AT_ERROR("Feed index must be between 0-%d, no details for feed with index %d.",
                             SettingsManager::Settings::kMaxNumFeeds - 1, index);
            }
            // Check the names before changing anything.
            SettingsManager::Settings& s = settings_manager.settings;
            SettingsManager::ReportingProtocol feed_protocol = s.feed_protocols[index];
            if (CPP_AT_HAS_ARG(4)) {
                uint16_t i = 0;
                while (i < SettingsManager::ReportingProtocol::kNumProtocols &&
                       args[4].compare(SettingsManager::kReportingProtocolStrs[i]) != 0) {
                    i++;
                }
                if (i == SettingsManager::ReportingProtocol::kNumProtocols) {
                    CPP_AT_ERROR("Unknown protocol '%.*s'.", (int)args[4].length(), args[4].data());
                }
                feed_protocol = static_cast<SettingsManager::ReportingProtocol>(i);
            }
            SettingsManager::FeedTransport feed_transport = s.feed_transports[index];
            if (CPP_AT_HAS_ARG(5)) {
                char transport_str[SettingsManager::kFeedTransportStrMaxLen] = {0};
                if (args[5].length() >= sizeof(transport_str)) {
                    CPP_AT_ERROR("Unknown transport '%.*s'. Use TCP, UDP or MQTT.", (int)args[5].length(),
                                 args[5].data());
                }
                memcpy(transport_str, args[5].data(), args[5].length());
                if (!SettingsManager::FeedTransportFromStr(transport_str, feed_transport)) {
                    CPP_AT_ERROR("Unknown transport '%s'. Use TCP, UDP or MQTT.", transport_str);
                }
            } else if (feed_protocol != s.feed_protocols[index]) {
                feed_transport = SettingsManager::DefaultFeedTransport(feed_protocol);
            }
            uint16_t feed_port = s.feed_ports[index];
            if (CPP_AT_HAS_ARG(2)) {
                CPP_AT_TRY_ARG2NUM(2, feed_port);
            }
            bool feed_is_active = s.feed_is_active[index];
            if (CPP_AT_HAS_ARG(3)) {
                CPP_AT_TRY_ARG2NUM(3, feed_is_active);
            }

            // Check the RAM budget with the new active flag and transport, then set everything.
            uint32_t old_used_bytes = FeedRAMBudget::UsedBytes(s);
            bool old_is_active = s.feed_is_active[index];
            SettingsManager::FeedTransport old_transport = s.feed_transports[index];
            s.feed_is_active[index] = feed_is_active;
            s.feed_transports[index] = feed_transport;
            uint32_t used_bytes, budget_bytes;
            if (!FeedRAMFits(old_used_bytes, used_bytes, budget_bytes)) {
                s.feed_is_active[index] = old_is_active;
                s.feed_transports[index] = old_transport;
                CPP_AT_ERROR(FEED_RAM_ERROR_FORMAT, (unsigned long)(used_bytes / 1024), (unsigned long)(budget_bytes / 1024));
            }
            // Set FEED_URI. A single "-" is a sentinel meaning "clear the URI to empty".
            if (CPP_AT_HAS_ARG(1)) {
                if (args[1] == "-") {
                    s.feed_uris[index][0] = '\0';
                } else {
                    strncpy(s.feed_uris[index], args[1].data(), SettingsManager::Settings::kFeedURIMaxNumChars);
                    s.feed_uris[index][SettingsManager::Settings::kFeedURIMaxNumChars] = '\0';
                }
            }
            s.feed_ports[index] = feed_port;
            s.feed_protocols[index] = feed_protocol;
            settings_manager.SyncToCoprocessors();
            CPP_AT_SUCCESS();
            break;
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

static void PrintFeedReceiverID(const CppAT::ATCommandDef_t& def, uint16_t index, const char* default_str) {
    char receiver_id_str[SettingsManager::Settings::kFeedReceiverIDNumBytes * 2 + 1];
    SettingsManager::ReceiverIDToStr(settings_manager.settings.feed_receiver_ids[index], receiver_id_str);
    CPP_AT_CMD_PRINTF("=%d(INDEX),%s(RECEIVER_ID),%s(DEFAULT)", index, receiver_id_str, default_str);
}

CPP_AT_CALLBACK(CommsManager::ATFeedReceiverIDCallback) {
    static_assert(FeedReceiverID::kNumBytes == SettingsManager::Settings::kFeedReceiverIDNumBytes);
    // Device default ID, from the part code in EEPROM. Empty if the EEPROM can't be read.
    uint8_t default_id[SettingsManager::Settings::kFeedReceiverIDNumBytes];
    char default_str[SettingsManager::Settings::kFeedReceiverIDNumBytes * 2 + 1] = "";
    SettingsManager::DeviceInfo device_info;
    bool have_default = settings_manager.GetDeviceInfo(device_info);
    if (have_default) {
        device_info.GetDefaultFeedReceiverID(default_id);
        SettingsManager::ReceiverIDToStr(default_id, default_str);
    }
    uint16_t index = UINT16_MAX;
    if (CPP_AT_HAS_ARG(0)) {
        CPP_AT_TRY_ARG2NUM(0, index);
        if (index >= SettingsManager::Settings::kMaxNumFeeds) {
            CPP_AT_ERROR("Feed index must be between 0-%d.", SettingsManager::Settings::kMaxNumFeeds - 1);
        }
    }
    switch (op) {
        case '?':
            if (index != UINT16_MAX) {
                PrintFeedReceiverID(def, index, default_str);
            } else {
                for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
                    PrintFeedReceiverID(def, i, default_str);
                }
            }
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=': {
            if (index == UINT16_MAX || !CPP_AT_HAS_ARG(1)) {
                CPP_AT_ERROR("Usage: AT+FEED_RECEIVER_ID=<index>,<16 hex digits|DEFAULT>");
            }
            uint8_t id[SettingsManager::Settings::kFeedReceiverIDNumBytes];
            switch (FeedReceiverID::Parse(args[1], id)) {
                case FeedReceiverID::kHex:
                    break;
                case FeedReceiverID::kDefault:
                    if (!have_default) {
                        CPP_AT_ERROR("Can't read the device default ID from EEPROM.");
                    }
                    memcpy(id, default_id, sizeof(id));
                    break;
                default:
                    CPP_AT_ERROR("Receiver ID must be 16 hex digits or DEFAULT.");
            }
            memcpy(settings_manager.settings.feed_receiver_ids[index], id, sizeof(id));
            settings_manager.SyncToCoprocessors();
            CPP_AT_SUCCESS();
            break;
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

// Copies an AT argument into a settings string: "-" clears it, an empty argument keeps it.
static bool SetATString(std::string_view arg, char* dest, uint16_t max_len) {
    if (arg.empty()) {
        return true;
    }
    if (arg == "-") {
        dest[0] = '\0';
        return true;
    }
    if (arg.length() > max_len) {
        return false;
    }
    memset(dest, '\0', max_len + 1);
    memcpy(dest, arg.data(), arg.length());
    return true;
}

CPP_AT_CALLBACK(CommsManager::ATMQTTCallback) {
    typedef SettingsManager::Settings Settings;
    switch (op) {
        case '?': {
            const Settings::MQTTBroker& mqtt = settings_manager.settings.mqtt_broker;
            char redacted_password[Settings::kMQTTPasswordMaxLen + 1];
            SettingsManager::RedactPassword(const_cast<char*>(mqtt.password), redacted_password,
                                            Settings::kMQTTPasswordMaxLen);
            CPP_AT_CMD_PRINTF("=%s(HOST),%u(PORT),%s(USERNAME),%s(PASSWORD),%d(TLS),%u(QOS),%s(TOPIC_PREFIX),"
                              "%s(TRANSPORT),%s(PATH)",
                              mqtt.host, mqtt.port, mqtt.username, redacted_password, mqtt.tls, mqtt.qos,
                              mqtt.topic_prefix, SettingsManager::kMQTTTransportStrs[mqtt.transport],
                              mqtt.websocket_path);
            CPP_AT_SILENT_SUCCESS();
            break;
        }
        case '=': {
            Settings::MQTTBroker mqtt = settings_manager.settings.mqtt_broker;
            if (num_args > 0 && !SetATString(args[0], mqtt.host, Settings::kFeedURIMaxNumChars)) {
                CPP_AT_ERROR("Host must be %d characters or fewer.", Settings::kFeedURIMaxNumChars);
            }
            if (CPP_AT_HAS_ARG(1)) {
                CPP_AT_TRY_ARG2NUM(1, mqtt.port);
            }
            if (num_args > 2 && !SetATString(args[2], mqtt.username, Settings::kMQTTUsernameMaxLen)) {
                CPP_AT_ERROR("Username must be %d characters or fewer.", Settings::kMQTTUsernameMaxLen);
            }
            if (num_args > 3 && !SetATString(args[3], mqtt.password, Settings::kMQTTPasswordMaxLen)) {
                CPP_AT_ERROR("Password must be %d characters or fewer.", Settings::kMQTTPasswordMaxLen);
            }
            if (CPP_AT_HAS_ARG(4)) {
                CPP_AT_TRY_ARG2NUM(4, mqtt.tls);
            }
            if (CPP_AT_HAS_ARG(5)) {
                CPP_AT_TRY_ARG2NUM(5, mqtt.qos);
                if (mqtt.qos > Settings::kMQTTMaxQoS) {
                    CPP_AT_ERROR("QoS must be 0, 1 or 2.");
                }
            }
            if (num_args > 6 && !SetATString(args[6], mqtt.topic_prefix, Settings::kMQTTTopicPrefixMaxLen)) {
                CPP_AT_ERROR("Topic prefix must be %d characters or fewer.", Settings::kMQTTTopicPrefixMaxLen);
            }
            if (strpbrk(mqtt.topic_prefix, "+#") != nullptr) {
                CPP_AT_ERROR("Topic prefix can't contain the MQTT wildcards + or #.");
            }
            if (CPP_AT_HAS_ARG(7)) {
                uint8_t i = 0;
                while (i < SettingsManager::kNumMQTTTransports &&
                       !(args[7].length() == strlen(SettingsManager::kMQTTTransportStrs[i]) &&
                         strncasecmp(args[7].data(), SettingsManager::kMQTTTransportStrs[i], args[7].length()) == 0)) {
                    i++;
                }
                if (i == SettingsManager::kNumMQTTTransports) {
                    CPP_AT_ERROR("Unknown transport '%.*s'. Use TCP or WS.", (int)args[7].length(), args[7].data());
                }
                mqtt.transport = static_cast<SettingsManager::MQTTTransport>(i);
            }
            if (num_args > 8 && !SetATString(args[8], mqtt.websocket_path, Settings::kMQTTWebSocketPathMaxLen)) {
                CPP_AT_ERROR("Path must be %d characters or fewer.", Settings::kMQTTWebSocketPathMaxLen);
            }
            if (mqtt.websocket_path[0] != '\0' && mqtt.websocket_path[0] != '/') {
                CPP_AT_ERROR("Path must start with /.");
            }
            // TLS adds to the RAM that active MQTT feeds take.
            uint32_t old_used_bytes = FeedRAMBudget::UsedBytes(settings_manager.settings);
            Settings::MQTTBroker old_mqtt = settings_manager.settings.mqtt_broker;
            settings_manager.settings.mqtt_broker = mqtt;
            uint32_t used_bytes, budget_bytes;
            if (!FeedRAMFits(old_used_bytes, used_bytes, budget_bytes)) {
                settings_manager.settings.mqtt_broker = old_mqtt;
                CPP_AT_ERROR(FEED_RAM_ERROR_FORMAT, (unsigned long)(used_bytes / 1024), (unsigned long)(budget_bytes / 1024));
            }
            settings_manager.SyncToCoprocessors();
            CPP_AT_SUCCESS();
            break;
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

// Token with all but its last 4 characters masked, so it can be told apart without being shown.
static void MaskSwarmToken(const char* token, char* masked, size_t masked_size) {
    size_t len = strnlen(token, masked_size - 1);
    for (size_t i = 0; i < len; i++) masked[i] = i + 4 < len ? '*' : token[i];
    masked[len] = '\0';
}

CPP_AT_CALLBACK(CommsManager::ATSwarmCallback) {
    typedef SettingsManager::Settings Settings;
    Settings& s = settings_manager.settings;
    switch (op) {
        case '?': {
            char masked[Settings::kSwarmTokenMaxLen + 1];
            MaskSwarmToken(s.swarm_token, masked, sizeof(masked));
            CPP_AT_CMD_PRINTF("=%d(ENABLED),%s(SERVER),%s(TOKEN)", s.swarm_enabled, s.swarm_server, masked);
            CPP_AT_SILENT_SUCCESS();
            break;
        }
        case '=': {
            bool enabled = s.swarm_enabled;
            char server[Settings::kSwarmServerMaxLen + 1];
            char token[Settings::kSwarmTokenMaxLen + 1];
            strncpy(server, s.swarm_server, sizeof(server));
            strncpy(token, s.swarm_token, sizeof(token));
            if (CPP_AT_HAS_ARG(0)) {
                CPP_AT_TRY_ARG2NUM(0, enabled);
            }
            if (num_args > 1) {
                if (args[1] == "-") {
                    CPP_AT_ERROR("Server can't be empty.");
                }
                if (!SetATString(args[1], server, Settings::kSwarmServerMaxLen)) {
                    CPP_AT_ERROR("Server must be %d characters or fewer.", Settings::kSwarmServerMaxLen);
                }
            }
            if (strncmp(server, "https://", 8) != 0 && strncmp(server, "http://", 7) != 0) {
                CPP_AT_ERROR("Server must start with https:// or http://.");
            }
            for (size_t n = strlen(server); n > 0 && server[n - 1] == '/'; n--) server[n - 1] = '\0';
            if (num_args > 2 && !SetATString(args[2], token, Settings::kSwarmTokenMaxLen)) {
                CPP_AT_ERROR("Token must be %d characters or fewer.", Settings::kSwarmTokenMaxLen);
            }
            for (const char* c = token; *c; c++) {
                if (!isalnum((unsigned char)*c) && *c != '-' && *c != '_' && *c != '.') {
                    CPP_AT_ERROR("Token may only contain letters, digits, - _ and .");
                }
            }
            if (enabled && token[0] == '\0') {
                CPP_AT_ERROR("Set a token to enable swarm reports.");
            }
            s.swarm_enabled = enabled;
            memset(s.swarm_server, '\0', sizeof(s.swarm_server));
            strncpy(s.swarm_server, server, Settings::kSwarmServerMaxLen);
            memset(s.swarm_token, '\0', sizeof(s.swarm_token));
            strncpy(s.swarm_token, token, Settings::kSwarmTokenMaxLen);
            settings_manager.SyncToCoprocessors();  // The ESP32 sends the reports.
            CPP_AT_SUCCESS();
            break;
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATHostnameCallback) {
    SettingsManager::Settings::CoreNetworkSettings& cns = settings_manager.settings.core_network_settings;
    switch (op) {
        case '?':
            CPP_AT_CMD_PRINTF("=%s", cns.hostname);
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=':
            if (!CPP_AT_HAS_ARG(0)) {
                CPP_AT_ERROR("Requires an argument. AT+HOSTNAME=<hostname>");
            }
            strncpy(cns.hostname, args[0].data(), SettingsManager::Settings::kHostnameMaxLen);
            cns.hostname[SettingsManager::Settings::kHostnameMaxLen] = '\0';
            CPP_AT_SUCCESS();
            break;
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATIPModeCallback) {
    switch (op) {
        case '?':
            CPP_AT_CMD_PRINTF("=%s", SettingsManager::IPModeToStr(settings_manager.settings.ip_mode));
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=': {
            char mode_str[SettingsManager::kIPModeStrMaxLen + 1] = {0};
            if (!CPP_AT_HAS_ARG(0) || args[0].size() > SettingsManager::kIPModeStrMaxLen) {
                CPP_AT_ERROR("Requires an argument. AT+IP_MODE=<IPV4|DUAL|IPV6>");
            }
            memcpy(mode_str, args[0].data(), args[0].size());
            SettingsManager::IPMode mode;
            if (!SettingsManager::IPModeFromStr(mode_str, mode)) {
                CPP_AT_ERROR("Invalid IP mode '%s'. AT+IP_MODE=<IPV4|DUAL|IPV6>", mode_str);
            }
            settings_manager.settings.ip_mode = mode;
            CPP_AT_SUCCESS();
            break;
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATOTACallback) {
    switch (op) {
        case '?':
            CPP_AT_PRINTF("Flash Partition Information\r\n");
            for (uint16_t partition = 0; partition < FirmwareUpdateManager::kNumPartitions; partition++) {
                CPP_AT_PRINTF("\tPartition %u %s\r\n", partition,
                              FirmwareUpdateManager::AmWithinFlashPartition(partition) ? "(ACTIVE)" : "");
                if (FirmwareUpdateManager::flash_partition_headers[partition]->magic_word !=
                    FirmwareUpdateManager::kFlashHeaderMagicWord) {
                    CPP_AT_PRINTF("\t\tNO VALID HEADER\r\n");
                    continue;
                }
                CPP_AT_PRINTF("\t\tLength: %u Bytes\r\n",
                              FirmwareUpdateManager::flash_partition_headers[partition]->app_size_bytes);
                CPP_AT_PRINTF("\t\tApplication CRC: 0x%x\r\n",
                              FirmwareUpdateManager::flash_partition_headers[partition]->app_crc);
                char status_str[FirmwareUpdateManager::kFlashPartitionStatusStrMaxLen];
                FirmwareUpdateManager::FlashPartitionStatusToStr(
                    (FirmwareUpdateManager::FlashPartitionStatus)
                        FirmwareUpdateManager::flash_partition_headers[partition]
                            ->status,
                    status_str);
                CPP_AT_PRINTF("\t\tStatus: %s (0x%x)\r\n", status_str,
                              FirmwareUpdateManager::flash_partition_headers[partition]->status);
                CPP_AT_PRINTF("\t\tSector %s verification.\r\n",
                              FirmwareUpdateManager::VerifyFlashPartition(partition, false) ? "PASSED" : "FAILED");
            }
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=':
            if (CPP_AT_HAS_ARG(0)) {
                uint16_t complementary_partition = FirmwareUpdateManager::GetComplementaryFlashPartition();
                if (args[0].compare("ERASE") == 0) {
                    if (CPP_AT_HAS_ARG(1) && CPP_AT_HAS_ARG(2)) {
                        // Performing a partial erase with AT+OTA=ERASE,<offset>,<len_bytes>
                        uint32_t offset, len_bytes;
                        CPP_AT_TRY_ARG2NUM_BASE(1, offset, 16);
                        CPP_AT_TRY_ARG2NUM_BASE(2, len_bytes, 10);
                        CPP_AT_PRINTF("Erasing %u Bytes at offset 0x%x in partition %u.\r\n", len_bytes, offset,
                                      complementary_partition);
                        bool flash_erase_succeeded =
                            FirmwareUpdateManager::EraseFlashParition(complementary_partition, offset, len_bytes);
                        if (!flash_erase_succeeded) {
                            CPP_AT_ERROR("Failed partial erase of complementary flash partition.");
                        }
                        CPP_AT_SUCCESS();
                    } else {
                        // Performing a full partition erase with AT+OTA=ERASE.
                        CPP_AT_PRINTF("Erasing partition %d.\r\n", complementary_partition);
                        bool flash_erase_succeeded = FirmwareUpdateManager::EraseFlashParition(complementary_partition);
                        if (!flash_erase_succeeded) {
                            CPP_AT_ERROR("Failed full erase of complementary flash partition.");
                        }
                        CPP_AT_SUCCESS();
                    }

                } else if (args[0].compare("GET_PARTITION") == 0) {
                    // Reply with the complementary flash partition number. This is used to select the correct flash
                    // partition from the OTA file.
                    CPP_AT_PRINTF("Partition: %u\r\n", complementary_partition);
                    CPP_AT_SUCCESS();
                } else if (args[0].compare("WRITE") == 0) {
                    // Write a section of the complementary flash partition.
                    // AT+OTA=WRITE,<offset (base 16)>,<len_bytes (base 10)>,<crc (base 16)>
                    bool receiver_was_enabled = adsbee.Receiver1090IsEnabled();
                    adsbee.SetReceiver1090Enable(0);  // Stop ADSB packets from mucking up the SPI bus.
                    uint32_t offset, len_bytes, crc;
                    CPP_AT_TRY_ARG2NUM_BASE(1, offset, 16);
                    CPP_AT_TRY_ARG2NUM_BASE(2, len_bytes, 10);
                    if (len_bytes > FirmwareUpdateManager::kFlashWriteBufMaxLenBytes) {
                        adsbee.SetReceiver1090Enable(receiver_was_enabled);  // Re-enable receiver before exit.
                        CPP_AT_ERROR("Write length %u exceeds maximum %u Bytes.", len_bytes,
                                     FirmwareUpdateManager::kFlashWriteBufMaxLenBytes);
                    }
                    uint8_t buf[len_bytes];

                    // From READY on, the next len_bytes console bytes are payload. The guard keeps any nested
                    // UpdateAT() off the input and discards whatever part of the payload arrives late.
                    at_console_guard_.BeginBinaryPayload(len_bytes);
                    // Send OK to indicate that we're ready to receive data.
                    CPP_AT_PRINTF("READY\r\n");
                    // The sender waits for READY before sending the payload, so send it now. Console output is batched
                    // (see UpdateNetworkConsole()): READY printed right after the previous command's OK stayed queued,
                    // and on an otherwise quiet console the payload read below then timed out.
                    UpdateNetworkConsole(true);

                    uint32_t old_esp32_heartbeat_ms = esp32.update_interval_ms;
                    esp32.update_interval_ms = kOTAHeartbeatMs;  // Faster heartbeat during OTA.
                    int32_t buf_len_bytes = ATReadConsole(reinterpret_cast<char*>(buf), len_bytes, kOTAWriteTimeoutMs);
                    esp32.update_interval_ms = old_esp32_heartbeat_ms;  // Restore old heartbeat.
                    at_console_guard_.EndBinaryPayload(buf_len_bytes, get_time_since_boot_ms());

                    if (buf_len_bytes < 0) {
                        adsbee.SetReceiver1090Enable(receiver_was_enabled);  // Re-enable receiver before exit.
                        CPP_AT_ERROR("Timed out after %u ms. Received %u Bytes.", kOTAWriteTimeoutMs, 0);
                    }

                    bool has_crc = CPP_AT_HAS_ARG(3);
                    CPP_AT_TRY_ARG2NUM_BASE(3, crc, 16);
                    if (has_crc) {
                        // CRC provided: use it to verify the data before writing.
                        CPP_AT_PRINTF("Verifying %d bytes of data with CRC 0x%x.\r\n", len_bytes, crc);
                        uint32_t calculated_crc = FirmwareUpdateManager::CalculateCRC32(buf, len_bytes);
                        if (calculated_crc != crc) {
                            adsbee.SetReceiver1090Enable(receiver_was_enabled);  // Re-enable receiver before exit.
                            CPP_AT_ERROR("Calculated CRC 0x%x did not match provided CRC 0x%x.", calculated_crc, crc);
                        }
                    }
                    CPP_AT_PRINTF("Writing %u Bytes to partition %u at offset 0x%x.\r\n", len_bytes,
                                  complementary_partition, offset);
                    bool flash_write_succeeded = FirmwareUpdateManager::PartialWriteFlashPartition(
                        complementary_partition, offset, len_bytes, buf);
                    if (!flash_write_succeeded) {
                        adsbee.SetReceiver1090Enable(receiver_was_enabled);  // Re-enable receiver before exit.
                        CPP_AT_ERROR("Partial %u Byte write failed in partition %u at offset 0x%x.", len_bytes,
                                     complementary_partition, offset);
                    }
                    comms_manager.ResetConsoleTxTimestamp();
                    if (has_crc) {
                        // CRC provided: verify the flash after writing.
                        CPP_AT_PRINTF("Verifying flash with CRC 0x%x.\r\n", crc);
                        uint32_t calculated_crc = FirmwareUpdateManager::CalculateCRC32(
                            (uint8_t*)(FirmwareUpdateManager::flash_partition_headers[complementary_partition]) +
                                offset,
                            len_bytes);
                        if (calculated_crc != crc) {
                            adsbee.SetReceiver1090Enable(receiver_was_enabled);  // Re-enable receiver before exit.
                            CPP_AT_ERROR("Calculated CRC 0x%x did not match provided CRC 0x%x.", calculated_crc, crc);
                        }
                    }
                    adsbee.SetReceiver1090Enable(receiver_was_enabled);  // Re-enable receiver before exit.
                    CPP_AT_SUCCESS();
                } else if (args[0].compare("VERIFY") == 0) {
                    // Verify the complementary flash partition.
                    CPP_AT_PRINTF(
                        "Verifying partition %u: %u Bytes, status 0x%x, application CRC 0x%x\r\n",
                        complementary_partition,
                        FirmwareUpdateManager::flash_partition_headers[complementary_partition]->app_size_bytes,
                        FirmwareUpdateManager::flash_partition_headers[complementary_partition]->status,
                        FirmwareUpdateManager::flash_partition_headers[complementary_partition]->app_crc);
                    // Modify the partition header.
                    bool ret = FirmwareUpdateManager::VerifyFlashPartition(complementary_partition, true);
                    if (ret) {
                        CPP_AT_SUCCESS();
                    } else {
                        CPP_AT_ERROR("Partition %u failed verification.", complementary_partition);
                    }
                } else if (args[0].compare("BOOT") == 0) {
                    // Boot the complementary flash partition.
                    CPP_AT_PRINTF("Booting partition %u...\r\n", complementary_partition);
                    esp32.Update();  // Send out this last console message.
                    FirmwareUpdateManager::BootPartition(complementary_partition);
                    // Don't return an error here - the boot process will handle any errors
                    CPP_AT_SUCCESS();
                }
            }
            break;
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_HELP_CALLBACK(CommsManager::ATOTAHelpCallback) {
    CPP_AT_PRINTF(
        "AT+OTA?\r\n\tQueries current OTA status.\r\n\tAT+OTA=ERASE\r\n\tErase the sector to "
        "update. Responds with status messages for each erase operation, then OK when "
        "complete.\r\n\tAT+OTA=WRITE,<offset>,<num_bytes>,<checksum>\r\n\tBegin an "
        "OTA write operation of num_bytes to offset bytes from the start of the partition with provided CRC32 "
        "checksum. Will respond with BEGIN, and then OK when complete, or ERROR if checksum doesn't match or timeout "
        "reached.\r\n");
}

CPP_AT_CALLBACK(CommsManager::ATLogLevelCallback) {
    switch (op) {
        case '?':
            // AT+CONFIG mode query.
            CPP_AT_CMD_PRINTF("=%s", SettingsManager::kConsoleLogLevelStrs[settings_manager.settings.log_level]);
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=':
            // AT+CONFIG set command.
            if (!CPP_AT_HAS_ARG(0)) {
                CPP_AT_ERROR("Need to specify a config mode to run.");
            }
            for (uint16_t i = 0; i < SettingsManager::kNumLogLevels; i++) {
                if (args[0].compare(SettingsManager::kConsoleLogLevelStrs[i]) == 0) {
                    settings_manager.settings.log_level = static_cast<SettingsManager::LogLevel>(i);
                    settings_manager.SyncToCoprocessors();
                    CPP_AT_SUCCESS();
                }
            }
            CPP_AT_ERROR("Verbosity level %s not recognized.", args[0].data());
            break;
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATASTERIXSACSICCallback) {
    switch (op) {
        case '?':
            CPP_AT_CMD_PRINTF("=%u,%u", settings_manager.settings.asterix_sac, settings_manager.settings.asterix_sic);
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=':
            if (!(CPP_AT_HAS_ARG(0) && CPP_AT_HAS_ARG(1))) {
                CPP_AT_ERROR("Requires two arguments: AT+ASTERIX_SAC_SIC=<sac>,<sic>.");
            }
            uint16_t sac, sic;
            CPP_AT_TRY_ARG2NUM(0, sac);
            CPP_AT_TRY_ARG2NUM(1, sic);
            if (sac > 255 || sic > 255) {
                CPP_AT_ERROR("SAC and SIC must be between 0 and 255.");
            }
            settings_manager.settings.asterix_sac = static_cast<uint8_t>(sac);
            settings_manager.settings.asterix_sic = static_cast<uint8_t>(sic);
            settings_manager.SyncToCoprocessors();  // The ESP32 encodes ASTERIX for feeds.
            CPP_AT_SUCCESS();
            break;
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATNTPServerCallback) {
    switch (op) {
        case '?':
            CPP_AT_CMD_PRINTF("=%s", settings_manager.settings.ntp_server);
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=': {
            if (!CPP_AT_HAS_ARG(0) || args[0].size() > SettingsManager::Settings::kNTPServerMaxLen) {
                CPP_AT_ERROR("Requires a host name or IP address of up to %u characters. AT+NTP_SERVER=<host>",
                             SettingsManager::Settings::kNTPServerMaxLen);
            }
            for (char c : args[0]) {
                if (!(isalnum(static_cast<unsigned char>(c)) || c == '.' || c == '-' || c == ':')) {
                    CPP_AT_ERROR("Invalid character '%c' in NTP server.", c);
                }
            }
            memset(settings_manager.settings.ntp_server, '\0', sizeof(settings_manager.settings.ntp_server));
            memcpy(settings_manager.settings.ntp_server, args[0].data(), args[0].size());
            settings_manager.SyncToCoprocessors();  // The ESP32 runs the NTP client.
            CPP_AT_SUCCESS();
            break;
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATMAVLINKIDCallback) {
    switch (op) {
        case '?':
            CPP_AT_PRINTF("System ID: %d\r\nComponent ID: %d\r\n", settings_manager.settings.mavlink_system_id,
                          settings_manager.settings.mavlink_component_id);
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=':
            if (!(CPP_AT_HAS_ARG(0) && CPP_AT_HAS_ARG(1))) {
                CPP_AT_ERROR("Requires two arguments: AT+MAVLINK_ID=<system_id>,<component_id>.");
            }
            uint16_t system_id, component_id;
            CPP_AT_TRY_ARG2NUM(0, system_id);
            CPP_AT_TRY_ARG2NUM(1, component_id);
            if (system_id < 1 || system_id > 255) {
                CPP_AT_ERROR("System ID must be between 1 and 255.");
            }
            if (component_id < 1 || component_id > 255) {
                CPP_AT_ERROR("Component ID must be between 1 and 255.");
            }
            settings_manager.settings.mavlink_system_id = static_cast<uint8_t>(system_id);
            settings_manager.settings.mavlink_component_id = static_cast<uint8_t>(component_id);
            CPP_AT_SUCCESS();
            break;
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATFilterAltitudeFtCallback) {
    switch (op) {
        case '?':
            CPP_AT_CMD_PRINTF("=%s,%ld,%ld", AltitudeFilterModeStr(settings_manager.settings.filter_altitude_mode),
                              (long)settings_manager.settings.filter_altitude_a_ft,
                              (long)settings_manager.settings.filter_altitude_b_ft);
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=': {
            uint8_t mode = settings_manager.settings.filter_altitude_mode;
            int32_t a_ft = settings_manager.settings.filter_altitude_a_ft;
            int32_t b_ft = settings_manager.settings.filter_altitude_b_ft;
            if (CPP_AT_HAS_ARG(0) && !ParseAltitudeFilterMode(args[0].data(), args[0].size(), mode)) {
                CPP_AT_ERROR("Mode must be OFF, ABOVE, BELOW, BETWEEN or NEAR_RX.");
            }
            if (CPP_AT_HAS_ARG(1)) {
                CPP_AT_TRY_ARG2NUM(1, a_ft);
            }
            if (CPP_AT_HAS_ARG(2)) {
                CPP_AT_TRY_ARG2NUM(2, b_ft);
            }
            if (a_ft < kAltitudeFilterMinFt || a_ft > kAltitudeFilterMaxFt || b_ft < kAltitudeFilterMinFt ||
                b_ft > kAltitudeFilterMaxFt) {
                CPP_AT_ERROR("Altitudes must be %ld to %ld ft.", (long)kAltitudeFilterMinFt, (long)kAltitudeFilterMaxFt);
            }
            if (mode == kAltitudeFilterBetween && a_ft > b_ft) {
                CPP_AT_ERROR("BETWEEN needs a <= b.");
            }
            if (mode == kAltitudeFilterNearRx && a_ft <= 0) {
                CPP_AT_ERROR("NEAR_RX needs a > 0.");
            }
            settings_manager.settings.filter_altitude_mode = mode;
            settings_manager.settings.filter_altitude_a_ft = a_ft;
            settings_manager.settings.filter_altitude_b_ft = b_ft;
            settings_manager.SyncToCoprocessors();  // The ESP32 filters its web UI, GDL90 and feeds too.
            CPP_AT_SUCCESS();
            break;
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATFilterDistanceNMCallback) {
    using Settings = SettingsManager::Settings;
    switch (op) {
        case '?':
            CPP_AT_CMD_PRINTF("=%d,%lu.%lu", settings_manager.settings.filter_distance_enabled,
                              static_cast<unsigned long>(settings_manager.settings.filter_distance_deci_nm / 10),
                              static_cast<unsigned long>(settings_manager.settings.filter_distance_deci_nm % 10));
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=': {
            bool enabled = settings_manager.settings.filter_distance_enabled;
            uint32_t deci_nm = settings_manager.settings.filter_distance_deci_nm;
            if (CPP_AT_HAS_ARG(0)) {
                CPP_AT_TRY_ARG2NUM(0, enabled);
            }
            if (CPP_AT_HAS_ARG(1) && (!ParseDeciNm(args[1].data(), args[1].size(), deci_nm,
                                                   Settings::kMaxFilterDistanceDeciNm) ||
                                      deci_nm < Settings::kMinFilterDistanceDeciNm)) {
                CPP_AT_ERROR("Distance must be 0.1 to 10000.0 nm, e.g. 50 or 12.5.");
            }
            settings_manager.settings.filter_distance_enabled = enabled;
            settings_manager.settings.filter_distance_deci_nm = deci_nm;
            settings_manager.SyncToCoprocessors();  // The ESP32 filters its web UI, GDL90 and feeds too.
            CPP_AT_SUCCESS();
            break;
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

// Prints an interface's IPv6 addresses from ESP32NetworkInfo, if it has any.
static void PrintIP6Addrs(char* link_local, char* global) {
    link_local[ObjectDictionary::kIP6AddrStrLen] = '\0';
    global[ObjectDictionary::kIP6AddrStrLen] = '\0';
    if (link_local[0] != '\0') {
        CPP_AT_PRINTF("\tIPv6 Link-Local Address: %s\r\n", link_local);
    }
    if (global[0] != '\0') {
        CPP_AT_PRINTF("\tIPv6 Global Address: %s\r\n", global);
    }
}

CPP_AT_CALLBACK(CommsManager::ATNetworkInfoCallback) {
    switch (op) {
        case '?':
            ObjectDictionary::ESP32NetworkInfo network_info;
            if (!esp32.Read(ObjectDictionary::Address::kAddrESP32NetworkInfo, network_info, sizeof(network_info))) {
                CPP_AT_ERROR("Error while reading network info from ESP32.");
            }

            CPP_AT_PRINTF("IP Mode: %s\r\n", SettingsManager::IPModeToStr(settings_manager.settings.ip_mode));
            CPP_AT_PRINTF("Ethernet: %s\r\n", network_info.ethernet_enabled ? "ENABLED" : "DISABLED");
            if (!network_info.ethernet_has_ip) {
                CPP_AT_PRINTF("\tNo IPv4 address assigned.\r\n");
            } else {
                CPP_AT_PRINTF("\tIP Address: %s\r\n", network_info.ethernet_ip);
                CPP_AT_PRINTF("\tSubnet Mask: %s\r\n", network_info.ethernet_netmask);
                CPP_AT_PRINTF("\tGateway: %s\r\n", network_info.ethernet_gateway);
            }
            PrintIP6Addrs(network_info.ethernet_ip6_link_local, network_info.ethernet_ip6_global);

            CPP_AT_PRINTF("WiFi Station: %s\r\n", network_info.wifi_sta_enabled ? "ENABLED" : "DISABLED");
            CPP_AT_PRINTF("\tSSID: %s\r\n", network_info.wifi_sta_ssid);
            if (!network_info.wifi_sta_has_ip) {
                CPP_AT_PRINTF("\tNo IPv4 address assigned.\r\n");
            } else {
                CPP_AT_PRINTF("\tIP Address: %s\r\n", network_info.wifi_sta_ip);
                CPP_AT_PRINTF("\tSubnet Mask: %s\r\n", network_info.wifi_sta_netmask);
                CPP_AT_PRINTF("\tGateway: %s\r\n", network_info.wifi_sta_gateway);
            }
            PrintIP6Addrs(network_info.wifi_sta_ip6_link_local, network_info.wifi_sta_ip6_global);

            CPP_AT_PRINTF("WiFi Access Point: %s\r\n", network_info.wifi_ap_enabled ? "ENABLED" : "DISABLED");
            CPP_AT_PRINTF("\tNum Clients: %u\r\n", network_info.wifi_ap_num_clients);
            for (uint16_t i = 0; i < network_info.wifi_ap_num_clients; i++) {
                CPP_AT_PRINTF("\t\tClient %u | IP: %s MAC: %s\r\n", i, network_info.wifi_ap_client_ips[i],
                              network_info.wifi_ap_client_macs[i]);
            }
            CPP_AT_SILENT_SUCCESS();
            break;
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATProtocolOutCallback) {
    switch (op) {
        case '?':
            // Print out reporting protocols for CONSOLE and COMMS_UART.
            for (uint16_t iface = 0; iface < SettingsManager::SerialInterface::kGNSSUART; iface++) {
                CPP_AT_CMD_PRINTF(
                    "=%s,%s", SettingsManager::kSerialInterfaceStrs[iface],
                    SettingsManager::kReportingProtocolStrs[settings_manager.settings.reporting_protocols[iface]]);
            }
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=':
            // Set the reporting protocol for a given interface.
            if (!(CPP_AT_HAS_ARG(0) && CPP_AT_HAS_ARG(1))) {
                CPP_AT_ERROR("Requires two arguments: AT+PROTOCOL=<iface>,<protocol>.");
            }

            // Match the selected serial interface. Don't allow selection of the GNSS interface.
            SettingsManager::SerialInterface selected_iface = SettingsManager::SerialInterface::kNumSerialInterfaces;
            for (uint16_t iface = 0; iface < SettingsManager::SerialInterface::kGNSSUART; iface++) {
                if (args[0].compare(SettingsManager::kSerialInterfaceStrs[iface]) == 0) {
                    selected_iface = static_cast<SettingsManager::SerialInterface>(iface);
                    break;
                }
            }
            if (selected_iface == SettingsManager::kNumSerialInterfaces) {
                CPP_AT_ERROR("Invalid serial interface %s.", args[0].data());
            }

            // Match the selected protocol.
            SettingsManager::ReportingProtocol selected_protocol = SettingsManager::ReportingProtocol::kNumProtocols;
            for (uint16_t protocol = 0; protocol < SettingsManager::ReportingProtocol::kNumProtocols; protocol++) {
                if (args[1].compare(SettingsManager::kReportingProtocolStrs[protocol]) == 0) {
                    selected_protocol = static_cast<SettingsManager::ReportingProtocol>(protocol);
                    break;
                }
            }
            if (selected_protocol == SettingsManager::kNumProtocols) {
                CPP_AT_ERROR("Invalid reporting protocol %s.", args[1].data());
            }

            // Assign the selected protocol to the selected interface.
            settings_manager.settings.reporting_protocols[selected_iface] = selected_protocol;
            settings_manager.SyncToCoprocessors();  // The ESP32 starts NTP when any output uses ASTERIX.
            CPP_AT_SUCCESS();
            break;
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_HELP_CALLBACK(CommsManager::ATProtocolOutHelpCallback) {
    CPP_AT_PRINTF("\tSet the reporting protocol used on a given serial interface:\r\n");
    CPP_AT_PRINTF("\tAT+PROTOCOL_OUT=<iface>,<protocol>\r\n\t<iface> = ");
    for (uint16_t iface = 0; iface < SettingsManager::kGNSSUART; iface++) {
        CPP_AT_PRINTF("%s ", SettingsManager::kSerialInterfaceStrs[iface]);
    }
    CPP_AT_PRINTF("\r\n\t<protocol> = ");
    for (uint16_t protocol = 0; protocol < SettingsManager::kNumProtocols; protocol++) {
        CPP_AT_PRINTF("\r\n\t\t%s ", SettingsManager::kReportingProtocolStrs[protocol]);
    }
    CPP_AT_PRINTF("\r\n\tQuery the reporting protocol used on all interfaces:\r\n");
    CPP_AT_PRINTF("\tAT+PROTOCOL_OUT?\r\n\tPROTOCOL_OUT=<iface>,<protocol>\r\n\t...\r\n");
}

CPP_AT_CALLBACK(CommsManager::ATRebootCallback) {
    adsbee.Reboot();
    // Note: would need to block here if there was a reboot delay.
    CPP_AT_ERROR("Failed to reboot.");
}

CPP_AT_CALLBACK(CommsManager::ATRxEnableCallback) {
    switch (op) {
        case '=':
            if (CPP_AT_HAS_ARG(0)) {
                // All enabled argument is present. Ignore subsequent args. This is useful so that all radios can be
                // disabled or enabled with one argument.
                bool all_enabled;
                CPP_AT_TRY_ARG2NUM(0, all_enabled);
                adsbee.SetReceiver1090Enable(all_enabled);
                settings_manager.settings.subg_rx_enabled = all_enabled;
            } else {
                if (CPP_AT_HAS_ARG(1)) {
                    bool r1090_enabled;
                    CPP_AT_TRY_ARG2NUM(1, r1090_enabled);
                    adsbee.SetReceiver1090Enable(r1090_enabled);
                }
                if (CPP_AT_HAS_ARG(2)) {
                    CPP_AT_TRY_ARG2NUM(2, settings_manager.settings.subg_rx_enabled);
                }
            }
            settings_manager.SyncToCoprocessors();
            CPP_AT_SUCCESS();
            break;
        case '?':
            CPP_AT_PRINTF("1090 Receiver: %s\r\nSubG Receiver: %s\r\n",
                          adsbee.Receiver1090IsEnabled() ? "ENABLED" : "DISABLED",
                          settings_manager.settings.subg_rx_enabled ? "ENABLED" : "DISABLED");
            CPP_AT_SILENT_SUCCESS();
            break;
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_HELP_CALLBACK(CommsManager::ATRxEnableHelpCallback) {
    CPP_AT_PRINTF(
        "RX_ENABLE=<all_enabled [1,0]>,<1090_enabled [1,0]>,<subg_enabled [1,0]>\r\n\tOK\r\n\tEnables or disables the "
        "receiver(s) "
        "from receiving messages. First arg overrides others if present.\r\n\tAT+RX_ENABLE?\r\n\t1090 Receiver: "
        "<1090_enabled [ENABLED,DISABLED]>\r\n\tSubG Receiver: <subg_en> [ENABLED,DISABLED]>\r\n\tQuery whether the "
        "receiver(s) are enabled.\r\n");
}

void ATRxPositionHelpCallback() {
    CPP_AT_PRINTF(
        "AT+RX_POSITION=<source>,<args>\r\n"
        "\tSet the receiver's position and source.\r\n");
    CPP_AT_PRINTF(
        "\tsource:\r\n"
        "\t\tNONE - No position. No additional args.\r\n"
        "\t\tFIXED - Fixed position set by user. Takes full position as additional args.\r\n"
        "\t\tGNSS - Position from GNSS receiver. Takes no additional args.\r\n"
        "\t\tLOWEST - Bootstrap receiver position from lowest altitude aircraft being tracked. "
        "Takes no additional args.\r\n"
        "\t\tICAO - Bootstrap receiver position from an aircraft with a given ICAO address. Takes an ICAO address as "
        "an additional arg.\r\n");
    CPP_AT_PRINTF(
        "AT+RX_POSITION=FIXED,<lat_deg>,<lon_deg>,<gnss_alt_ft>,<baro_alt_ft>,<heading_deg>,<speed_kts>\r\n"
        "\tlat_deg: Latitude in degrees (-90.0 to 90.0).\r\n"
        "\tlon_deg: Longitude in degrees (-180.0 to 180.0).\r\n"
        "\tgnss_alt_m: GNSS altitude in meters.\r\n"
        "\tbaro_alt_m: Barometric altitude in meters.\r\n"
        "\theading_deg: Heading in degrees [0.0-360.0)."
        "\tspeed_kts: Speed in kts."
        "AT+RX_POSITION=ICAO,<icao>\r\n"
        "\ticao: ICAO address in hex (e.g. 7ABCDEF) of aircraft to use for position bootstrap.\r\n"
        "AT+RX_POSITION?\r\n\tQuery the receiver's position.\r\n");
}

CPP_AT_CALLBACK(CommsManager::ATRxPositionCallback) {
    switch (op) {
        case '?': {
            CPP_AT_PRINTF(
                "Receiver Position:\r\n"
                "\tSource: %s [%s] \r\n"
                "\tLatitude: %.6f deg\r\n"
                "\tLongitude: %.6f deg\r\n"
                "\tGNSS Altitude: %d ft\r\n"
                "\tBarometric Altitude: %d ft\r\n"
                "\tHeading: %.1f deg\r\n"
                "\tSpeed: %d kts\r\n"
                "\tICAO: 0x%06X [%s]\r\n",
                SettingsManager::RxPosition::kPositionSourceStrs[adsbee.rx_position.source],
                adsbee.rx_position_available ? "OK" : "NOT AVAILABLE", adsbee.rx_position.latitude_deg,
                adsbee.rx_position.longitude_deg, adsbee.rx_position.gnss_altitude_ft,
                adsbee.rx_position.baro_altitude_ft, adsbee.rx_position.heading_deg, adsbee.rx_position.speed_kts,
                adsbee.rx_position.icao_address,
                adsbee.rx_position.source ==
                        SettingsManager::RxPosition::PositionSource::kPositionSourceAircraftMatchingICAO
                    ? "IN USE"
                    : "NOT IN USE");
            // if (rx_position.source ==
            //     SettingsManager::RxPosition::PositionSource::kPositionSourceAircraftMatchingICAO) {
            //     CPP_AT_PRINTF("\tICAO: 0x%06X\r\n", rx_position.icao_address);
            // }
            CPP_AT_SILENT_SUCCESS();
            break;
        }
        case '=': {
            if (CPP_AT_HAS_ARG(0)) {
                // Set position source.
                for (uint16_t i = 0; i <= SettingsManager::RxPosition::kNumPositionSources; i++) {
                    if (i == SettingsManager::RxPosition::PositionSource::kNumPositionSources) {
                        CPP_AT_ERROR("Invalid position source %s.", args[0].data());
                    }
                    if (args[0].compare(SettingsManager::RxPosition::kPositionSourceStrs[i]) == 0) {
                        adsbee.rx_position.source = static_cast<SettingsManager::RxPosition::PositionSource>(i);

                        // Post-set actions.
                        switch (adsbee.rx_position.source) {
                            case SettingsManager::RxPosition::PositionSource::kPositionSourceFixed: {
                                // Fixed position is available immediately. The ADSBee::UpdateRxPosition function will
                                // also overwrite this with "true" every loop.
                                adsbee.rx_position_available = true;

                                if (CPP_AT_HAS_ARG(1)) {
                                    // Set latitude.
                                    float latitude_deg;
                                    CPP_AT_TRY_ARG2NUM(1, latitude_deg);
                                    if (latitude_deg < -90.0 || latitude_deg > 90.0) {
                                        CPP_AT_ERROR("Latitude %.6f out of range (-90.0 to 90.0).", latitude_deg);
                                    }
                                    adsbee.rx_position.latitude_deg = latitude_deg;
                                }
                                if (CPP_AT_HAS_ARG(2)) {
                                    // Set longitude.
                                    float longitude_deg;
                                    CPP_AT_TRY_ARG2NUM(2, longitude_deg);
                                    if (longitude_deg < -180.0 || longitude_deg > 180.0) {
                                        CPP_AT_ERROR("Longitude %.6f out of range (-180.0 to 180.0).", longitude_deg);
                                    }
                                    adsbee.rx_position.longitude_deg = longitude_deg;
                                }
                                if (CPP_AT_HAS_ARG(3)) {
                                    // Set GNSS altitude.
                                    int32_t gnss_altitude_ft;
                                    CPP_AT_TRY_ARG2NUM(3, gnss_altitude_ft);
                                    adsbee.rx_position.gnss_altitude_ft = gnss_altitude_ft;
                                }
                                if (CPP_AT_HAS_ARG(4)) {
                                    // Set barometric altitude.
                                    int32_t baro_altitude_ft;
                                    CPP_AT_TRY_ARG2NUM(4, baro_altitude_ft);
                                    adsbee.rx_position.baro_altitude_ft = baro_altitude_ft;
                                }
                                if (CPP_AT_HAS_ARG(5)) {
                                    // Set heading.
                                    float heading_deg;
                                    CPP_AT_TRY_ARG2NUM(5, heading_deg);
                                    if (heading_deg < 0.0 || heading_deg >= 360.0) {
                                        CPP_AT_ERROR("Heading %.1f out of range [0.0 to 360.0).", heading_deg);
                                    }
                                    adsbee.rx_position.heading_deg = heading_deg;
                                }
                                if (CPP_AT_HAS_ARG(6)) {
                                    // Set speed.
                                    int32_t speed_kts;
                                    CPP_AT_TRY_ARG2NUM(6, speed_kts);
                                    if (speed_kts < 0.0) {
                                        CPP_AT_ERROR("Speed %.1f out of range (>= 0.0).", speed_kts);
                                    }
                                    adsbee.rx_position.speed_kts = speed_kts;
                                }
                                break;
                            }
                            case SettingsManager::RxPosition::PositionSource::kPositionSourceAircraftMatchingICAO: {
                                adsbee.rx_position_available =
                                    false;  // ICAO needs to be found in the dict for position to be valid.

                                if (CPP_AT_HAS_ARG(1)) {
                                    // Set ICAO address.
                                    uint32_t icao_address;
                                    CPP_AT_TRY_ARG2NUM_BASE(1, icao_address, 16);
                                    if (icao_address > 0xFFFFFF) {
                                        CPP_AT_ERROR("ICAO address 0x%X out of range (0x000000 to 0xFFFFFF).",
                                                     icao_address);
                                    }
                                    adsbee.rx_position.icao_address = icao_address;
                                }
                                break;
                            }
                            default: {
                                // Other position sources need to be updated to be available.
                                adsbee.rx_position_available = false;
                                break;
                            }
                        }

                        break;
                    }
                }
            }
            settings_manager.SyncToCoprocessors();
            WarnIfRemoteIDTxPositionSourceNotTransmittable();
            CPP_AT_SUCCESS();
            break;
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

// Escapes src for embedding inside a JSON string literal (backslash, double quote;
// control characters are dropped — settings strings shouldn't contain them). Truncates
// to fit dest. Returns dest so it can be used inline in a printf argument list.
static const char* JSONEscapeStr(const char* src, char* dest, size_t dest_size) {
    size_t o = 0;
    for (size_t i = 0; src[i] != '\0'; i++) {
        char c = src[i];
        if (c == '"' || c == '\\') {
            if (o + 3 >= dest_size) break;
            dest[o++] = '\\';
            dest[o++] = c;
        } else if ((unsigned char)c >= 0x20) {
            if (o + 2 >= dest_size) break;
            dest[o++] = c;
        }
    }
    dest[o] = '\0';
    return dest;
}

// Prints all settings as one JSON object on a single line: SETTINGS={"CMD":[...],...}.
// Each value is an array ordered to match the web settings GUI's field list for that
// command (the writable AT= args plus query-only extras like live status, minus the
// redacted WiFi STA password), using the same value representations as the individual
// AT queries (enum strings, 0/1 booleans). Emitted in multiple printf calls to stay
// under kPrintfBufferMaxSize. Keep in sync with SETTINGS_SCHEMA_1090 in
// esp/main/server/web/settings.js when adding commands.
static void PrintSettingsJSON() {
    char esc[2 * SettingsManager::Settings::kWiFiPasswordMaxLen + 2];
    const SettingsManager::Settings& s = settings_manager.settings;
    const SettingsManager::Settings::CoreNetworkSettings& cns = s.core_network_settings;

    CPP_AT_PRINTF("SETTINGS={");
    CPP_AT_PRINTF("\"ASTERIX_SAC_SIC\":[%u,%u],", s.asterix_sac, s.asterix_sic);
    CPP_AT_PRINTF("\"AUTO_UPDATE\":[%d],", s.auto_update_enabled);
    CPP_AT_PRINTF("\"BAUD_RATE\":[%lu,%lu],",
                  (unsigned long)s.baud_rates[SettingsManager::SerialInterface::kCommsUART],
                  (unsigned long)s.baud_rates[SettingsManager::SerialInterface::kGNSSUART]);
    CPP_AT_PRINTF("\"BIAS_TEE_ENABLE\":[%d,%d],", adsbee.BiasTeeIsEnabled(), s.subg_bias_tee_enabled);
    CPP_AT_PRINTF("\"BLE_OUTPUT\":[\"%s\",\"%s\"],", SettingsManager::BLEOutputModeToStr(s.ble_output_mode),
                  BLEOutputStatusStr(esp32_ll.ble_output_status));
    CPP_AT_PRINTF("\"ETHERNET\":[%d],", cns.ethernet_enabled);
    CPP_AT_PRINTF("\"ESP32_ENABLE\":[%d],", esp32.IsEnabled());
    CPP_AT_PRINTF("\"FEED_ENABLE\":[%d],", s.feeds_enabled);
    CPP_AT_PRINTF("\"FILTER_ALTITUDE_FT\":[\"%s\",%ld,%ld],", AltitudeFilterModeStr(s.filter_altitude_mode),
                  (long)s.filter_altitude_a_ft, (long)s.filter_altitude_b_ft);
    CPP_AT_PRINTF("\"FILTER_DISTANCE_NM\":[%d,%lu.%lu],", s.filter_distance_enabled,
                  static_cast<unsigned long>(s.filter_distance_deci_nm / 10),
                  static_cast<unsigned long>(s.filter_distance_deci_nm % 10));
    {
        char masked[SettingsManager::Settings::kSwarmTokenMaxLen + 1];
        MaskSwarmToken(s.swarm_token, masked, sizeof(masked));
        CPP_AT_PRINTF("\"SWARM\":[%d,\"%s\",\"%s\"],", s.swarm_enabled, JSONEscapeStr(s.swarm_server, esc, sizeof(esc)),
                      masked);
    }
    CPP_AT_PRINTF("\"GNSS\":[%d,\"%s\",%d],", s.gnss_enabled,
                  GNSSModuleTypeToStr(SettingsToGNSSModuleType(s.gnss_receiver_type)), s.gnss_notify);
    CPP_AT_PRINTF("\"HOSTNAME\":[\"%s\"],", JSONEscapeStr(cns.hostname, esc, sizeof(esc)));
    CPP_AT_PRINTF("\"IP_MODE\":[\"%s\"],", SettingsManager::IPModeToStr(s.ip_mode));
    CPP_AT_PRINTF("\"LED_ENABLE\":[%d],", s.led_enabled);
    CPP_AT_PRINTF("\"LOG_LEVEL\":[\"%s\"],", SettingsManager::kConsoleLogLevelStrs[s.log_level]);
    CPP_AT_PRINTF("\"MAVLINK_ID\":[%d,%d],", s.mavlink_system_id, s.mavlink_component_id);
    CPP_AT_PRINTF("\"NTP_SERVER\":[\"%s\"],", s.ntp_server);
    CPP_AT_PRINTF("\"PROTOCOL_OUT\":[\"%s\",\"%s\"],",
                  SettingsManager::kReportingProtocolStrs[s.reporting_protocols[SettingsManager::kConsole]],
                  SettingsManager::kReportingProtocolStrs[s.reporting_protocols[SettingsManager::kCommsUART]]);
    CPP_AT_PRINTF("\"REMOTE_ID\":[%d,%u,\"0x%02X\",%d],", s.remote_id_rx_enabled, s.remote_id_transports,
                  esp32_ll.remote_id_status, RemoteIDAllowedTransports());
    CPP_AT_PRINTF("\"REMOTE_ID_TX\":[%d,%u,\"%s\",", s.remote_id_tx_enabled, s.remote_id_tx_transports,
                  JSONEscapeStr(s.remote_id_tx_uas_id, esc, sizeof(esc)));
    CPP_AT_PRINTF("%d,%d,\"%s\",\"0x%04X\",%d],", s.remote_id_tx_uas_id_type, s.remote_id_tx_ua_type,
                  JSONEscapeStr(s.remote_id_tx_operator_id, esc, sizeof(esc)), esp32_ll.remote_id_status,
                  RemoteIDAllowedTransports());
    CPP_AT_PRINTF("\"RX_ENABLE\":[%d,%d],", adsbee.Receiver1090IsEnabled(), s.subg_rx_enabled);
    CPP_AT_PRINTF("\"RX_POSITION\":[\"%s\",\"%s\",%.6f,%.6f,%d,%d,%.1f,%d,\"%06X\"],",
                  SettingsManager::RxPosition::kPositionSourceStrs[adsbee.rx_position.source],
                  adsbee.rx_position_available ? "OK" : "NOT AVAILABLE", adsbee.rx_position.latitude_deg,
                  adsbee.rx_position.longitude_deg, (int)adsbee.rx_position.gnss_altitude_ft,
                  (int)adsbee.rx_position.baro_altitude_ft, adsbee.rx_position.heading_deg,
                  (int)adsbee.rx_position.speed_kts, (unsigned)adsbee.rx_position.icao_address);
    CPP_AT_PRINTF("\"SUBG_ENABLE\":[\"%s\"],",
                  SettingsManager::EnableStateToATValueStr(adsbee.subg_radio_ll.IsEnabledState()));
    int tl_offset_mv = adsbee.GetTLOffsetMilliVolts();
    CPP_AT_PRINTF("\"TL_OFFSET\":[%d,\"%d dBm\"],", tl_offset_mv, adsbee.AD8313MilliVoltsTodBm(tl_offset_mv));
    CPP_AT_PRINTF("\"WATCHDOG\":[%lu],", (unsigned long)adsbee.GetWatchdogTimeoutSec());
    CPP_AT_PRINTF("\"WIFI_AP\":[%d,\"%s\",", cns.wifi_ap_enabled, JSONEscapeStr(cns.wifi_ap_ssid, esc, sizeof(esc)));
    CPP_AT_PRINTF("\"%s\",%d],", JSONEscapeStr(cns.wifi_ap_password, esc, sizeof(esc)), cns.wifi_ap_channel);
    CPP_AT_PRINTF("\"WIFI_STA\":[%d,\"%s\"],", cns.wifi_sta_enabled, JSONEscapeStr(cns.wifi_sta_ssid, esc, sizeof(esc)));
    // Networks 2 and up; network 1 is in WIFI_STA.
    CPP_AT_PRINTF("\"WIFI_STA_NETWORK\":[");
    for (uint16_t i = 0; i < SettingsManager::Settings::kWiFiSTANumExtraNetworks; i++) {
        CPP_AT_PRINTF("%s\"%s\"", i ? "," : "",
                      JSONEscapeStr(settings_manager.settings.wifi_sta_extra_networks[i].ssid, esc, sizeof(esc)));
    }
    CPP_AT_PRINTF("]");
    CPP_AT_PRINTF("}\r\n");
}

CPP_AT_CALLBACK(CommsManager::ATSettingsCallback) {
    switch (op) {
        case '=':
            // Note: Don't allow settings modification from here, do it directly through other commands.
            if (CPP_AT_HAS_ARG(0)) {
                if (args[0].compare("SAVE") == 0) {
                    if (!settings_manager.Save()) {
                        CPP_AT_ERROR("Error while writing to EEPROM.");
                    }
                } else if (args[0].compare("LOAD") == 0) {
                    if (!settings_manager.Load()) {
                        CPP_AT_ERROR("Error while reading from EEPROM.");
                    }
                } else if (args[0].compare("RESET") == 0) {
                    settings_manager.ResetToDefaults();
                    settings_manager.Apply();
                } else {
                    CPP_AT_ERROR("Invalid argument %s.", args[0].data());
                }
                CPP_AT_SUCCESS();
            }
            CPP_AT_ERROR("No arguments provided.");
            break;
        case '?':
            if (CPP_AT_HAS_ARG(0)) {
                if (args[0].compare("DUMP") == 0) {
                    // Print settings in AT format.
                    settings_manager.PrintAT();
                } else if (args[0].compare("JSON") == 0) {
                    // Print settings as a single-line JSON object (used by the web GUI).
                    PrintSettingsJSON();
                } else {
                    CPP_AT_ERROR("Invalid argument %s.", args[0].data());
                }
            } else {
                // Print settings in human readable format.
                settings_manager.Print();
            }
            CPP_AT_SUCCESS();
            break;
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATSubGEnableCallback) {
    switch (op) {
        case '=':
            if (CPP_AT_HAS_ARG(0)) {
                if (args[0].compare("EXTERNAL") == 0) {
                    adsbee.SetSubGRadioEnable(SettingsManager::kEnableStateExternal);
                } else {
                    bool subg_enabled;
                    CPP_AT_TRY_ARG2NUM(0, subg_enabled);
                    adsbee.SetSubGRadioEnable(subg_enabled ? SettingsManager::kEnableStateEnabled
                                                           : SettingsManager::kEnableStateDisabled);
                }
            }
            CPP_AT_SUCCESS();
            break;
        case '?':
            CPP_AT_CMD_PRINTF("=%s\r\n",
                              SettingsManager::EnableStateToATValueStr(adsbee.subg_radio_ll.IsEnabledState()));
            CPP_AT_SILENT_SUCCESS();
            break;
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATSubGFlashCallback) {
    // Disable other core and watchdog, since flashing and verification operations take a while and might trigger a
    // watchdog reboot.
    StopCore1();
    adsbee.DisableWatchdog();
    bool flash_success = adsbee.subg_radio_ll.Flash();
    adsbee.EnableWatchdog();
    StartCore1();
    if (!flash_success) {
        CPP_AT_ERROR("Error while flashing SubG radio.");
    }
    CPP_AT_SUCCESS();
}

CPP_AT_CALLBACK(CommsManager::ATTLReadCallback) {
    switch (op) {
        case '?':
            // Read command.
            int tl_mv = adsbee.ReadTLMilliVolts();
            CPP_AT_CMD_PRINTF("=%dmV (%d dBm)\r\n", tl_mv, adsbee.DetectorMilliVoltsTodBm(tl_mv));
            CPP_AT_SILENT_SUCCESS();
            break;
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

/**
 * AT+TL_OFFSET Callback
 * AT+TL_OFFSET=<tl_offset_mv>
 *  tl_offset_mv = Trigger Level Offset, in milliVolts.
 * AT+TL_OFFSET=LEARN
 *  Learn the offset from the number of valid frames received (about 20 minutes). Setting an offset stops learning.
 * AT+TL_OFFSET?
 * +TL_OFFSET=<tl_offset_mv>mV (<dBm> dBm)[,LEARNING,<cycle>/<num_cycles>,<best_offset_mv>mV]
 */
CPP_AT_CALLBACK(CommsManager::ATTLSetCallback) {
    switch (op) {
        case '?': {
            // AT+TL_SET value query.
            int tl_mv = adsbee.GetTLOffsetMilliVolts();
            const TLLearner &learner = adsbee.GetTLLearner();
            if (learner.IsActive()) {
                // Offset being tried, then learning progress and the best offset so far.
                CPP_AT_CMD_PRINTF("=%dmV (%d dBm),LEARNING,%u/%u,%dmV\r\n", tl_mv, adsbee.AD8313MilliVoltsTodBm(tl_mv),
                                  learner.GetCycle(), learner.GetNumCycles(), learner.GetAcceptedOffsetMV());
            } else {
                CPP_AT_CMD_PRINTF("=%dmV (%d dBm)\r\n", tl_mv, adsbee.AD8313MilliVoltsTodBm(tl_mv));
            }
            CPP_AT_SILENT_SUCCESS();
            break;
        }
        case '=': {
            // Attempt setting LO TL value, in milliVolts, if first argument is not blank.
            if (CPP_AT_HAS_ARG(0)) {
                if (args[0].compare("LEARN") == 0) {
                    // Starting a new trigger level learning session.
                    adsbee.StartTLLearning();
                } else {
                    // Assigning trigger level manually.
                    uint16_t new_tl_offset_mv;
                    CPP_AT_TRY_ARG2NUM(0, new_tl_offset_mv);
                    if (!adsbee.SetTLOffsetMilliVolts(new_tl_offset_mv)) {
                        CPP_AT_ERROR("Failed to set tl_offset_mv.");
                    }
                }
            }
            CPP_AT_SUCCESS();
            break;
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATUptimeCallback) {
    switch (op) {
        case '?':
            // Query uptime.
            CPP_AT_CMD_PRINTF("=%u", get_time_since_boot_ms() / kMsPerSec);
            CPP_AT_SILENT_SUCCESS();
            break;
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATWatchdogCallback) {
    switch (op) {
        case '?': {
            CPP_AT_CMD_PRINTF("=%u\r\n", adsbee.GetWatchdogTimeoutSec());
            CPP_AT_SILENT_SUCCESS();
            break;
        }
        case '=': {
            if (CPP_AT_HAS_ARG(0)) {
                if (args[0].compare("TEST") == 0) {
                    // Block for watchdog timeout + 1 seconds to try triggering a watchdog event.
                    uint32_t begin_timestamp_ms = get_time_since_boot_ms();
                    static constexpr uint32_t kDotPrintIntervalMs = 1 * kMsPerSec;
                    uint32_t last_dot_print_timestamp_ms = begin_timestamp_ms;
                    uint32_t watchdog_test_blocking_interval_sec = adsbee.GetWatchdogTimeoutSec() + 1;
                    CPP_AT_PRINTF("Blocking for %d seconds.\r\n", watchdog_test_blocking_interval_sec);
                    while ((get_time_since_boot_ms() - begin_timestamp_ms) / kMsPerSec <
                           watchdog_test_blocking_interval_sec) {
                        if (get_time_since_boot_ms() - last_dot_print_timestamp_ms > kDotPrintIntervalMs) {
                            CPP_AT_PRINTF(".");
                            last_dot_print_timestamp_ms = get_time_since_boot_ms();
                        }
                    }
                    CPP_AT_ERROR("Watchdog failed to trigger in %d seconds.", watchdog_test_blocking_interval_sec);
                } else {
                    uint32_t watchdog_timeout_sec = 0;
                    CPP_AT_TRY_ARG2NUM(0, watchdog_timeout_sec);
                    if (!adsbee.SetWatchdogTimeoutSec(watchdog_timeout_sec)) {
                        CPP_AT_ERROR("Setting watchdog timeout to %d seconds failed.");
                    }
                }
            }
            CPP_AT_SUCCESS();
            break;
        }
        default: {
            CPP_AT_ERROR("Operator %c not supported.", op);
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

// WPA2 needs 8-63 characters; an empty password means an open network.
static bool WiFiAPPasswordIsValid(std::string_view password) {
    return password.empty() || (password.length() >= 8 && password.length() <= 63);
}

// Stations also accept WEP keys (5 or 13 characters).
static bool WiFiSTAPasswordIsValid(std::string_view password) {
    return WiFiAPPasswordIsValid(password) || password.length() == 5 || password.length() == 13;
}

CPP_AT_CALLBACK(CommsManager::ATWiFiAPCallback) {
    SettingsManager::Settings::CoreNetworkSettings& cns = settings_manager.settings.core_network_settings;
    switch (op) {
        case '?': {
            char redacted_password[SettingsManager::Settings::kWiFiPasswordMaxLen + 1];
            CPP_AT_CMD_PRINTF("=%d,%s,%s,%d\r\n", cns.wifi_ap_enabled, cns.wifi_ap_ssid, cns.wifi_ap_password,
                              cns.wifi_ap_channel);
            CPP_AT_SILENT_SUCCESS();
            break;
        }
        case '=': {
            if (CPP_AT_HAS_ARG(2) && !WiFiAPPasswordIsValid(args[2])) {
                CPP_AT_ERROR("WiFi AP password must be 8-63 characters.");
            }
            if (CPP_AT_HAS_ARG(0)) {
                CPP_AT_TRY_ARG2NUM(0, cns.wifi_ap_enabled);
                CPP_AT_CMD_PRINTF(": wifi_ap_enabled=%d\r\n", cns.wifi_ap_enabled);
                if (cns.wifi_ap_enabled) WarnIfNetworkPausesRemoteID();
            }
            if (CPP_AT_HAS_ARG(1)) {
                strncpy(cns.wifi_ap_ssid, args[1].data(), SettingsManager::Settings::kWiFiSSIDMaxLen);
                cns.wifi_ap_ssid[SettingsManager::Settings::kWiFiSSIDMaxLen] = '\0';
                CPP_AT_CMD_PRINTF(": wifi_ap_ssid=%s\r\n", cns.wifi_ap_ssid);
            }
            if (CPP_AT_HAS_ARG(2)) {
                strncpy(cns.wifi_ap_password, args[2].data(), SettingsManager::Settings::kWiFiPasswordMaxLen);
                cns.wifi_ap_password[SettingsManager::Settings::kWiFiPasswordMaxLen] = '\0';
                CPP_AT_CMD_PRINTF(": wifi_ap_password=%s\r\n", cns.wifi_ap_password);
            }
            if (CPP_AT_HAS_ARG(3)) {
                uint8_t channel;
                CPP_AT_TRY_ARG2NUM(3, channel);
                if (channel == 0 || channel > SettingsManager::kWiFiAPChannelMax) {
                    CPP_AT_ERROR("WiFi channel out of range, must be >0 and <=%d.", SettingsManager::kWiFiAPChannelMax);
                }
                cns.wifi_ap_channel = channel;
                CPP_AT_CMD_PRINTF(": wifi_ap_channel = %d\r\n", cns.wifi_ap_channel);
            }
            CPP_AT_SUCCESS();
            break;
        }
        default: {
            CPP_AT_ERROR("Operator %c not supported.", op);
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATWiFiSTACallback) {
    SettingsManager::Settings::CoreNetworkSettings& cns = settings_manager.settings.core_network_settings;
    switch (op) {
        case '?': {
            char redacted_password[SettingsManager::Settings::kWiFiPasswordMaxLen + 1];
            SettingsManager::RedactPassword(cns.wifi_sta_password, redacted_password,
                                            SettingsManager::Settings::kWiFiPasswordMaxLen);
            CPP_AT_CMD_PRINTF("=%d,%s,%s\r\n", cns.wifi_sta_enabled, cns.wifi_sta_ssid, redacted_password);
            CPP_AT_SILENT_SUCCESS();
            break;
        }
        case '=': {
            if (CPP_AT_HAS_ARG(2) && !WiFiSTAPasswordIsValid(args[2])) {
                CPP_AT_ERROR("WiFi station password must be 8-63 characters (or a 5 or 13 character WEP key).");
            }
            if (CPP_AT_HAS_ARG(0)) {
                CPP_AT_TRY_ARG2NUM(0, cns.wifi_sta_enabled);
                CPP_AT_CMD_PRINTF(": wifi_sta_enabled=%d\r\n", cns.wifi_sta_enabled);
                if (cns.wifi_sta_enabled) WarnIfNetworkPausesRemoteID();
            }
            if (CPP_AT_HAS_ARG(1)) {
                strncpy(cns.wifi_sta_ssid, args[1].data(), SettingsManager::Settings::kWiFiSSIDMaxLen);
                cns.wifi_sta_ssid[SettingsManager::Settings::kWiFiSSIDMaxLen] = '\0';
                CPP_AT_CMD_PRINTF(": sta_ssid=%s\r\n", cns.wifi_sta_ssid);
            }
            if (CPP_AT_HAS_ARG(2)) {
                strncpy(cns.wifi_sta_password, args[2].data(), SettingsManager::Settings::kWiFiPasswordMaxLen);
                cns.wifi_sta_password[SettingsManager::Settings::kWiFiPasswordMaxLen] = '\0';
                char redacted_password[SettingsManager::Settings::kWiFiPasswordMaxLen + 1];
                SettingsManager::RedactPassword(cns.wifi_sta_password, redacted_password,
                                                SettingsManager::Settings::kWiFiPasswordMaxLen);
                CPP_AT_CMD_PRINTF(": sta_password=%s\r\n", redacted_password);
            }

            CPP_AT_SUCCESS();
            break;
        }
        default: {
            CPP_AT_ERROR("Operator %c not supported.", op);
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

// Stored WiFi station network `index` (1-based); network 1 lives in CoreNetworkSettings.
static void WiFiSTANetwork(uint16_t index, char*& ssid, char*& password) {
    SettingsManager::Settings& s = settings_manager.settings;
    if (index == 1) {
        ssid = s.core_network_settings.wifi_sta_ssid;
        password = s.core_network_settings.wifi_sta_password;
    } else {
        ssid = s.wifi_sta_extra_networks[index - 2].ssid;
        password = s.wifi_sta_extra_networks[index - 2].password;
    }
}

CPP_AT_CALLBACK(CommsManager::ATWiFiSTANetworkCallback) {
    char *ssid, *password;
    char redacted_password[SettingsManager::Settings::kWiFiPasswordMaxLen + 1];
    switch (op) {
        case '?': {
            for (uint16_t i = 1; i <= SettingsManager::Settings::kWiFiSTAMaxNumNetworks; i++) {
                WiFiSTANetwork(i, ssid, password);
                SettingsManager::RedactPassword(password, redacted_password,
                                                SettingsManager::Settings::kWiFiPasswordMaxLen);
                CPP_AT_CMD_PRINTF("=%d,%s,%s\r\n", i, ssid, redacted_password);
            }
            CPP_AT_SILENT_SUCCESS();
            break;
        }
        case '=': {
            uint16_t index;
            CPP_AT_TRY_ARG2NUM(0, index);
            if (index < 1 || index > SettingsManager::Settings::kWiFiSTAMaxNumNetworks) {
                CPP_AT_ERROR("Network index must be 1-%d.", SettingsManager::Settings::kWiFiSTAMaxNumNetworks);
            }
            if (CPP_AT_HAS_ARG(2) && !WiFiSTAPasswordIsValid(args[2])) {
                CPP_AT_ERROR("WiFi station password must be 8-63 characters (or a 5 or 13 character WEP key).");
            }
            WiFiSTANetwork(index, ssid, password);
            if (!CPP_AT_HAS_ARG(1)) {
                // Index alone clears the slot.
                memset(ssid, '\0', SettingsManager::Settings::kWiFiSSIDMaxLen + 2);
                memset(password, '\0', SettingsManager::Settings::kWiFiPasswordMaxLen + 2);
                CPP_AT_CMD_PRINTF(": cleared network %d\r\n", index);
                CPP_AT_SUCCESS();
            }
            strncpy(ssid, args[1].data(), SettingsManager::Settings::kWiFiSSIDMaxLen);
            ssid[MIN(args[1].length(), SettingsManager::Settings::kWiFiSSIDMaxLen)] = '\0';
            if (CPP_AT_HAS_ARG(2)) {
                strncpy(password, args[2].data(), SettingsManager::Settings::kWiFiPasswordMaxLen);
                password[MIN(args[2].length(), SettingsManager::Settings::kWiFiPasswordMaxLen)] = '\0';
            }
            SettingsManager::RedactPassword(password, redacted_password, SettingsManager::Settings::kWiFiPasswordMaxLen);
            CPP_AT_CMD_PRINTF(": network %d: ssid=%s password=%s\r\n", index, ssid, redacted_password);
            CPP_AT_SUCCESS();
            break;
        }
        default: {
            CPP_AT_ERROR("Operator %c not supported.", op);
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

const CppAT::ATCommandDef_t at_command_list[] = {
    {.command = "ASTERIX_SAC_SIC",
     .min_args = 0,
     .max_args = 2,
     .help_string = "AT+ASTERIX_SAC_SIC=<sac>,<sic>\r\n\tSet the ASTERIX System Area Code and System Identification "
                    "Code (0-255 each) sent in every ASTERIX record.\r\n\tAT+ASTERIX_SAC_SIC?\r\n\tQuery them.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATASTERIXSACSICCallback, comms_manager)},
    {.command = "AUTO_UPDATE",
     .min_args = 0,
     .max_args = 1,
     .help_string = "AT+AUTO_UPDATE=<enabled>\r\n\tAutomatic firmware updates from adsbee.aero over WiFi station or "
                    "Ethernet. Off by default. When on, the unique ID, part code, OTA keys and firmware versions are "
                    "sent to adsbee.aero.\r\n\tAT+AUTO_UPDATE?\r\n\tQuery whether automatic updates are enabled.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATAutoUpdateCallback, comms_manager)},
    {.command = "BAUD_RATE",
     .min_args = 0,
     .max_args = 2,
     .help_string = "AT+BAUD_RATE=<iface>,<baud_rate>\r\n\tSet the baud rate of a serial "
                    "interface.\r\n\tAT+BAUD_RATE?\r\n\tQuery the baud rate of all serial interfaces.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATBaudRateCallback, comms_manager)},
    {.command = "BIAS_TEE_ENABLE",
     .min_args = 0,
     .max_args = 2,
     .help_string = "AT+BIAS_TEE_ENABLE=<1090_bt_enabled>,<subg_bt_enabled>\r\n\tEnable or disable the bias "
                    "tees.\r\n\tBIAS_TEE_ENABLE?\r\n\tQuery the status of the bias tees.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATBiasTeeEnableCallback, comms_manager)},
    {.command = "BLE_OUTPUT",
     .min_args = 0,
     .max_args = 1,
     .help_string = "AT+BLE_OUTPUT=<OFF|GDL90>\r\n\tStream to one Bluetooth LE client (Nordic UART and HM-10 serial "
                    "services). GDL90: the GDL90 sent to WiFi clients. Off by default. Without PSRAM it needs the WiFi "
                    "access point and station off.\r\n\tAT+BLE_OUTPUT?\r\n\tQuery the mode and its state.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATBLEOutputCallback, comms_manager)},
    {.command = "BOOT_USB_UF2",
     .min_args = 0,
     .max_args = 1,
     .help_string = "Reboot ADSBee into RP2040 USB Bootloader.\r\n\tDo not use unless you have a USB connection to the "
                    "ADSBee\r\n\t"
                    "AT+BOOT_USB_UF2=1DEADBEE",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATBootloader, comms_manager)},
    {.command = "CONFIG_HASH",
     .min_args = 0,
     .max_args = 0,
     .help_string =
         "AT+CONFIG_HASH?\r\n\tPrint hashes of the settings, the CC1312 CCFG and the firmware on each processor, to "
         "check that a unit matches another one.\r\n\tCONFIG_HASH=<hash of SETTINGS_FLEET, CCFG_CC1312 and "
         "FIRMWARE>\r\n\tSETTINGS_FLEET leaves out per-unit settings (hostname, WiFi AP SSID and channel, feed "
         "receiver IDs, Remote ID UAS ID, receiver position); SETTINGS_FULL covers all settings. FIRMWARE is the "
         "hash of the FIRMWARE_<processor> lines. Settings are hashed as running, saved or not. Expected firmware "
         "values: the .hashes.txt file of a release, or firmware/scripts/firmware_hashes.py.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATConfigHashCallback, comms_manager)},
    {.command = "DEVICE_INFO",
     .min_args = 0,
     .max_args = 5,  // TODO: check this value.
     .help_string = "AT+DEVICE_INFO?\r\n\tQuery device information, including RP2040 RAM use and aircraft "
                    "dictionary size.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATDeviceInfoCallback, comms_manager)},
    {.command = "ETHERNET",
     .min_args = 0,
     .max_args = 1,
     .help_string = "AT+ETHERNET=<enabled>\r\n\tEnable or disable the Ethernet "
                    "interface.\r\n\tETHERNET?\r\n\tQuery the status of the Ethernet interface.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATEthernetCallback, comms_manager)},
    {.command = "ESP32_ENABLE",
     .min_args = 0,
     .max_args = 1,
     .help_string = "AT+ESP32_ENABLE=<enabled>\r\n\tEnable or disable the ESP32.\r\n\tAT+ESP32_ENABLE?\r\n\tQuery "
                    "the enable status of the ESP32.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATESP32EnableCallback, comms_manager)},
    {.command = "ESP32_FLASH",
     .min_args = 0,
     .max_args = 0,
     .help_string = "AT+ESP32_FLASH\r\n\tTriggers a firmware update of the ESP32 from the firmware image stored in "
                    "the RP2040's flash memory.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATESP32FlashCallback, comms_manager)},
    {.command = "ESP32_REBOOT_INFO",
     .min_args = 0,
     .max_args = 0,
     .help_string = "AT+ESP32_REBOOT_INFO?\r\n\tQuery the reason the ESP32 last rebooted. "
                    "In debug builds also prints the core dump backtrace if available.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATESP32RebootInfoCallback, comms_manager)},
#ifdef HARDWARE_UNIT_TESTS
    {.command = "ESP32_TRIGGER_ABORT",
     .min_args = 0,
     .max_args = 0,
     .help_string = "AT+ESP32_TRIGGER_ABORT\r\n\tTrigger an abort() on the ESP32 to test core dump functionality. "
                    "Debug builds only.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATESP32TriggerAbortCallback, comms_manager)},
#endif
    {.command = "FEED",
     .min_args = 0,
     .max_args = 6,
     .help_callback = ATFeedHelpCallback,
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATFeedCallback, comms_manager)},
    {.command = "FEED_ENABLE",
     .min_args = 0,
     .max_args = 1,
     .help_string = "AT+FEED_ENABLE=<enabled>\r\n\tMaster switch for all outbound network feeds. Set to 0 to disable "
                    "all feeds, or 1 to allow feeds marked active. Does not change per-feed active flags set with "
                    "AT+FEED.\r\n\tAT+FEED_ENABLE?\r\n\tQuery whether outbound feeds are enabled.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATFeedEnableCallback, comms_manager)},
    {.command = "FEED_RECEIVER_ID",
     .min_args = 0,
     .max_args = 2,
     .help_string = "AT+FEED_RECEIVER_ID=<index>,<receiver_id>\r\n\tSet the receiver ID a feed sends to its "
                    "aggregator (Beast feeds; feed 0's also names MQTT topics). index = [0-9], receiver_id = 16 hex "
                    "digits, or DEFAULT for this device's own ID. AT+SETTINGS=SAVE keeps it.\r\n\t"
                    "AT+FEED_RECEIVER_ID?\r\n\tPrint every feed's receiver ID and the device default.\r\n\t"
                    "AT+FEED_RECEIVER_ID?<index>\r\n\tPrint one feed's.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATFeedReceiverIDCallback, comms_manager)},
    {.command = "FILTER_ALTITUDE_FT",
     .min_args = 0,
     .max_args = 3,
     .help_string = "AT+FILTER_ALTITUDE_FT=<mode>,<a_ft>,<b_ft>\r\n\tLeave aircraft outside an altitude band out of aircraft "
                    "reports (aircraft JSON, the web UI's list and map, GDL90, MAVLink, CSBee). mode: OFF, ABOVE (at or above a), BELOW (at or below "
                    "a), BETWEEN (a to b) or NEAR_RX (within a ft above or below the receiver, see AT+RX_POSITION; "
                    "filters nothing without a receiver altitude), any case. Altitudes are -2000 to 100000 ft; "
                    "aircraft altitude is barometric, else GNSS. Aircraft with no known altitude are always kept. Raw "
                    "packets (Raw, Beast, MLAT and every feed of them) are never filtered. Empty arguments keep their value. Default: "
                    "OFF,0,10000.\r\n\t"
                    "AT+FILTER_ALTITUDE_FT?\r\n\tQuery the mode and its altitudes.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATFilterAltitudeFtCallback, comms_manager)},
    {.command = "FILTER_DISTANCE_NM",
     .min_args = 0,
     .max_args = 2,
     .help_string = "AT+FILTER_DISTANCE_NM=<enabled [0 1]>,<distance_nm>\r\n\tLeave aircraft farther than distance_nm "
                    "nautical miles (0.1 to 10000.0, 0.1 nm steps) from the receiver out of aircraft reports (aircraft JSON, the web UI's list and map, GDL90, MAVLink, CSBee). Raw "
                    "packets (Raw, Beast, MLAT and every feed of them) are never filtered. Aircraft at an unknown distance (no position yet, or "
                    "no receiver position, see AT+RX_POSITION) are always kept. Default: 0,100.0 (off).\r\n\t"
                    "AT+FILTER_DISTANCE_NM?\r\n\tQuery whether the filter is enabled and its distance.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATFilterDistanceNMCallback, comms_manager)},
    {.command = "SWARM",
     .min_args = 0,
     .max_args = 3,
     .help_string = "AT+SWARM=<enabled [0 1]>,<server>,<token>\r\n\tHourly reports to a swarm server over WiFi station "
                    "or Ethernet: range per direction and altitude, message and aircraft counts, receiver position, "
                    "unique ID and firmware versions (plus OTA keys to adsbee.aero only). token: from the swarm "
                    "account page. Empty arguments keep their value. Default: 0,https://adsbee.aero,(none).\r\n\t"
                    "AT+SWARM?\r\n\tQuery the setting; the token is masked.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATSwarmCallback, comms_manager)},
    {.command = "GNSS",
     .min_args = 0,
     .max_args = 3,
     .help_string = "AT+GNSS=enable: <0|1>[, type: <NONE|GENERIC|UBX_MIA>[,notify: <0|1>]]\r\n\tEnable or disable GNSS power, optionally "
                    "select the GNSS type, and enable GNSS_FIX log messages for when the GNSS fix becomes valid or invalid. Notify defaults to off. "
                    "Notifications are INFO-level log messages: they only appear on the console with AT+LOG_LEVEL=INFO. "
                    "Poll AT+GNSS_FIX? to read the fix programmatically. "
                    "Type is optional, when not present the last configured type is reused, default is NONE; GNSS cannot be enabled while the saved type is NONE."
                    "\r\n\tAT+GNSS?\r\n\tQuery the GNSS system state, type, and notification setting",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATGNSSCallback, comms_manager)},
    {.command = "GNSS_FIX",
     .min_args = 0,
     .max_args = 0,
     .help_string = "AT+GNSS_FIX?\r\n\tReturn fix validity, latitude and longitude in degrees, altitude in feet, "
                    "heading in degrees true, groundspeed in knots, active satellites, UTC time, and PPS count "
                    "since the last GNSS enable or disable operation.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATGNSSFixCallback, comms_manager)},
    {.command = "HOSTNAME",
     .min_args = 0,
     .max_args = 1,
     .help_string = "AT+HOSTNAME=<hostname>\r\n\tSet the hostname for all network "
                    "interfaces.\r\n\tAT+HOSTNAME?\r\n\tQuery the "
                    "hostname used for all network interfaces.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATHostnameCallback, comms_manager)},
    {.command = "IP_MODE",
     .min_args = 0,
     .max_args = 1,
     .help_string = "AT+IP_MODE=<IPV4|DUAL|IPV6>\r\n\tSet the IP versions used on Ethernet and WiFi Station: IPV4 "
                    "(DHCP), DUAL (DHCP plus IPv6 link-local and SLAAC, the default) or IPV6 (link-local and SLAAC, no "
                    "DHCP). The WiFi Access Point is always IPv4. Applied with AT+SETTINGS=SAVE.\r\n\tAT+IP_MODE?\r\n\t"
                    "Query the IP mode.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATIPModeCallback, comms_manager)},
    {.command = "LED_BLINK",
     .min_args = 2,
     .max_args = 4,
     .help_string = "AT+LED_BLINK=<duration_ms>,<led>[,<led>...]\r\n\tForce one or more LEDs (1090, SUBG, NETWORK, or "
                    "ALL) solid ON for duration_ms (1-60000) then off.\r\n\tBypasses AT+LED_ENABLE; intended for test "
                    "fixtures. Returns immediately.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATLEDBlinkCallback, comms_manager)},
    {.command = "LED_ENABLE",
     .min_args = 0,
     .max_args = 1,
     .help_string = "AT+LED_ENABLE=<enabled>\r\n\tEnable or disable the hardware status/activity "
                    "LEDs.\r\n\tAT+LED_ENABLE?\r\n\tQuery whether the hardware LEDs are enabled.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATLEDEnableCallback, comms_manager)},
    {.command = "LOG_LEVEL",
     .min_args = 0,
     .max_args = 1,
     .help_string =
         "AT+LOG_LEVEL=<log_level [SILENT ERRORS WARNINGS INFO]>\r\n\tSet how much stuff gets printed to the "
         "console.\r\n\t",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATLogLevelCallback, comms_manager)},
    {.command = "MAVLINK_ID",
     .min_args = 0,
     .max_args = 2,
     .help_string = "AT+MAVLINK_ID=<system_id>,<component_id>\r\n\tSet the MAVLink system and component IDs.\r\n\t"
                    "AT+MAVLINK_ID?\r\n\tQuery the MAVLink system and component IDs.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATMAVLINKIDCallback, comms_manager)},
    {.command = "MQTT",
     .min_args = 0,
     .max_args = 9,
     .help_string = "AT+MQTT=<host>,<port>,<username>,<password>,<tls>,<qos>,<topic_prefix>,<transport>,<path>\r\n\t"
                    "Set the MQTT broker that all feeds with the MQTT transport publish to, over one connection. "
                    "Empty arguments keep their value, \"-\" clears a string.\r\n\thost = hostname or IP address, "
                    "port = [1-65535] (1883, 8883 with TLS), username and password = optional, tls = [0 1], qos = [0 1 "
                    "2], topic_prefix = up to 31 characters (empty: adsbee), transport = [TCP WS] (WS: MQTT over "
                    "WebSocket, for brokers behind web proxies), path = WebSocket path (empty: /mqtt).\r\n\tTopics are "
                    "<topic_prefix>/<receiver ID>/<protocol>, plus status and metrics.\r\n\tAT+MQTT?\r\n\tPrint the "
                    "broker settings (password redacted).",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATMQTTCallback, comms_manager)},
    {.command = "NTP_SERVER",
     .min_args = 0,
     .max_args = 1,
     .help_string = "AT+NTP_SERVER=<host>\r\n\tSet the Network Time Protocol server the ESP32 takes UTC from while an "
                    "output uses ASTERIX and GNSS has no time (default pool.ntp.org).\r\n\tAT+NTP_SERVER?\r\n\tQuery "
                    "it.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATNTPServerCallback, comms_manager)},
    {.command = "NETWORK_INFO",
     .min_args = 0,
     .max_args = 0,
     .help_string = "AT+NETWORK_INFO?\r\n\tQueries network information.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATNetworkInfoCallback, comms_manager)},
    {.command = "OTA",
     .min_args = 0,
     .max_args = 4,
     .help_callback = CPP_AT_BIND_MEMBER_HELP_CALLBACK(CommsManager::ATOTAHelpCallback, comms_manager),
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATOTACallback, comms_manager)},
    {.command = "PROTOCOL_OUT",
     .min_args = 0,
     .max_args = 2,
     .help_callback = CPP_AT_BIND_MEMBER_HELP_CALLBACK(CommsManager::ATProtocolOutHelpCallback, comms_manager),
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATProtocolOutCallback, comms_manager)},
    {.command = "REBOOT",
     .min_args = 0,
     .max_args = 0,
     .help_string = "REBOOT\r\n\tReboots the RP2040.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATRebootCallback, comms_manager)},
    {.command = "REMOTE_ID",
     .min_args = 0,
     .max_args = 2,
     .help_string =
         "[EXPERIMENTAL] Remote ID is not yet a stable part of the system; its settings and behavior may change in "
         "future firmware.\r\n\t"
         "AT+REMOTE_ID=<enabled>[,<transports [BT4 BT5_LR WIFI_BEACON] joined with +, ALL or NONE>]\r\n\t"
         "Enable/disable Broadcast Remote ID (drone) reception on the ESP32, e.g. AT+REMOTE_ID=1,BT4+BT5_LR.\r\n\t"
         "BT4 = Bluetooth 4 legacy, BT5_LR = Bluetooth 5 Long Range, WIFI_BEACON = WiFi beacon (default ALL).\r\n\t"
         "On hardware whose ESP32 has no PSRAM (see AT+DEVICE_INFO?), Remote ID can't run with WiFi AP or WiFi Station "
         "enabled, and with Ethernet enabled only its Bluetooth transports run; with PSRAM it runs alongside them (the "
         "WiFi beacon receiver then listens on the WiFi channel only). Remote ID pauses while ESP32 free memory is below what the web server needs.\r\n\t"
         "AT+REMOTE_ID?\r\n\tQuery Remote ID settings, the ESP32's live receive status and the transports this "
         "hardware can run with the enabled network interfaces, e.g.\r\n\t"
         "REMOTE_ID=1,BT4+BT5_LR(TRANSPORTS),BT_SCANNING+BT5_LR_SCANNING(ESP32_STATUS),BT4+BT5_LR(ALLOWED_TRANSPORTS)"
         "\r\n\tStatus words: BT_SCANNING, BT5_LR_SCANNING, WIFI_BEACON_SCANNING, BLOCKED_NETWORK (no PSRAM and WiFi "
         "or Ethernet on), BLOCKED_RAM, NOT_IN_BUILD, PAUSED_LOW_HEAP, or NONE.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATRemoteIDCallback, comms_manager)},
    {.command = "REMOTE_ID_TX",
     .min_args = 0,
     .max_args = 6,
     .help_callback = ATRemoteIDTxHelpCallback,
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATRemoteIDTxCallback, comms_manager)},
    {.command = "RX_ENABLE",
     .min_args = 0,
     .max_args = 3,
     .help_callback = CPP_AT_BIND_MEMBER_HELP_CALLBACK(CommsManager::ATRxEnableHelpCallback, comms_manager),
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATRxEnableCallback, comms_manager)

    },
    {.command = "RX_POSITION",
     .min_args = 0,
     .max_args = 8,
     .help_callback = ATRxPositionHelpCallback,
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATRxPositionCallback, comms_manager)},
    {.command = "SETTINGS",
     .min_args = 0,
     .max_args = 3,
     .help_string = "Load, save, or reset nonvolatile settings.\r\n\tAT+SETTINGS=<op [LOAD SAVE RESET]>\r\n\t"
                    "Display nonvolatile settings.\r\n\tAT+SETTINGS?\r\n\t+SETTINGS=...\r\n\tDump settings in AT "
                    "command format.\r\n\tAT+SETTINGS?DUMP\r\n\t+SETTINGS=...\r\n\tDump settings as a "
                    "single-line JSON object keyed by AT command.\r\n\tAT+SETTINGS?JSON\r\n\tSETTINGS={...}",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATSettingsCallback, comms_manager)},
    {.command = "SUBG_ENABLE",
     .min_args = 0,
     .max_args = 2,
     .help_string = "AT+SUBG_ENABLE=<enabled [1,0,EXTERNAL]>\r\n\tEnable or disable the sub-GHz receiver. Receiver "
                    "enable line can be driven with a low impedance GPIO output or left high impedance (with "
                    "pulldown) for control via an external device.\r\n\tAT+SUBG_ENABLE?\r\n\t"
                    "Query the status of the sub-GHz receiver.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATSubGEnableCallback, comms_manager)},
    {.command = "SUBG_FLASH",
     .min_args = 0,
     .max_args = 0,
     .help_string = "AT+SUBG_FLASH\r\n\tTriggers a firmware update of the sub-GHz radio from the firmware image "
                    "stored in the RP2040's flash memory.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATSubGFlashCallback, comms_manager)},
#ifdef HARDWARE_UNIT_TESTS
    {.command = "INGEST_MODE_S",
     .min_args = 1,
     .max_args = 4,
     .help_string = "Ingest a Mode S packet into the raw packet queue.\r\n\t"
                    "AT+INGEST_MODE_S=<hex_data>,<sigs_dbm>,<sigq_db>,<mlat_counts>\r\n\t"
                    "hex_data: 14 hex chars (squitter) or 28 hex chars (extended squitter).\r\n\t"
                    "sigs_dbm, sigq_db, mlat_counts are optional signal quality parameters.",
     .callback = ATIngestModeSCallback},
    {.command = "INGEST_UAT",
     .min_args = 1,
     .max_args = 4,
     .help_string = "Ingest a UAT packet into the raw packet queue.\r\n\t"
                    "AT+INGEST_UAT=<hex_data>,<sigs_dbm>,<sigq_bits>,<mlat_counts>\r\n\t"
                    "hex_data: 60 hex chars (short ADS-B), 96 (long ADS-B), or 1104 (uplink).\r\n\t"
                    "sigs_dbm, sigq_bits, mlat_counts are optional signal quality parameters.",
     .callback = ATIngestUATCallback},
    {.command = "TEST",
     .min_args = 0,
     .max_args = 1,
     .help_string = "Run hardware self-tests.",
     .callback = ATTestCallback},
#endif
    {.command = "TL_READ",
     .min_args = 0,
     .max_args = 0,
     .help_string =
         "Read ADC counts and mV value for the minimum trigger level threshold. Call with no ops nor arguments, "
         "AT+TL_READ. Note this reads the trigger level, not the trigger level offset.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATTLReadCallback, comms_manager)},
    {.command = "TL_OFFSET",
     .min_args = 0,
     .max_args = 1,
     .help_string = "Set minimum trigger level offset (trigger level distance above noise floor) for RF power "
                    "detector.\r\n\tAT+TL_OFFSET=<tl_offset_mv>\r\n"
                    "\tLearn it from received traffic (about 20 minutes).\r\n\tAT+TL_OFFSET=LEARN\r\n"
                    "\tQuery trigger level offset.\r\n\tAT+TL_OFFSET?\r\n\t+TL_OFFSET=<tl_offset_mv>.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATTLSetCallback, comms_manager)},
    {.command = "UPTIME",
     .min_args = 0,
     .max_args = 0,
     .help_string = "Get the uptime of the ADSBee 1090 in seconds.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATUptimeCallback, comms_manager)},
    {.command = "WATCHDOG",
     .min_args = 0,
     .max_args = 1,
     .help_string = "Set the watchdog timeout, in seconds, 0-65535. 0 = watchdog disabled, 65535 = "
                    "18.2hrs.\r\n\tAT+WATCHDOG=<timeout_sec>\r\n\tTest watchdog by blocking for timeout_sec+1 "
                    "seconds.\r\n\tAT+WATCHDOG=TEST",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATWatchdogCallback, comms_manager)},
    {.command = "WIFI_AP",
     .min_args = 0,
     .max_args = 4,
     .help_string =
         "Set WiFi access point params.\r\n\tAT+WIFI_AP=<enabled>,<ap_ssid>,<ap_pwd>,<ap_channel>\r\n\t"
         "ap_pwd: 8-63 characters.\r\n\t"
         "Get WiFi access point params.\r\n\tAT+WIFI_AP?\r\n\t+WIFI_AP=<enabled>,<ap_ssid>,<ap_pwd>,<ap_channel>",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATWiFiAPCallback, comms_manager)},
    {.command = "WIFI_STA",
     .min_args = 0,
     .max_args = 3,
     .help_string = "Set WiFi station params.\r\n\tAT+WIFI_STA=<enabled>,<sta_ssid>,<sta_pwd>\r\n\t"
                    "sta_pwd: 8-63 characters, or a 5 or 13 character WEP key.\r\n\t"
                    "Get WiFi station params.\r\n\tAT+WIFI_STA?\r\n\t+WIFI_STA=<enabled>,<sta_ssid>,<sta_pwd>",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATWiFiSTACallback, comms_manager)},
    {.command = "WIFI_STA_NETWORK",
     .min_args = 0,
     .max_args = 3,
     .help_string = "Store WiFi station networks. The station joins the strongest one in range.\r\n\t"
                    "AT+WIFI_STA_NETWORK=<index 1-3>,<ssid>,<pwd>\r\n\t"
                    "Network 1 is the AT+WIFI_STA network. An omitted pwd keeps the stored one.\r\n\t"
                    "Clear a network.\r\n\tAT+WIFI_STA_NETWORK=<index>\r\n\t"
                    "List networks.\r\n\tAT+WIFI_STA_NETWORK?\r\n\t+WIFI_STA_NETWORK=<index>,<ssid>,<pwd>",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATWiFiSTANetworkCallback, comms_manager)},
};
const uint16_t at_command_list_num_commands = sizeof(at_command_list) / sizeof(at_command_list[0]);

bool CommsManager::UpdateAT() {
    // While an AT+OTA=WRITE payload is being read, every console byte belongs to ATReadConsole(). UpdateAT() can be
    // reached from inside that callback (network_console_putc() used to call it), so don't touch the input then.
    if (at_console_guard_.PayloadInProgress()) {
        return true;
    }
    // Check for new AT commands from STDIO.
    int ci = getchar_timeout_us(0);
    while (ci != PICO_ERROR_TIMEOUT) {
        CPUMonitor::WorkScope work;
        switch (stdio_at_line_.Push(static_cast<char>(ci), get_time_since_boot_ms())) {
            case ATLineAssembler<kATCommandBufMaxLen>::kLine:
                at_parser_.ParseMessage(stdio_at_line_.line());
                break;
            case ATLineAssembler<kATCommandBufMaxLen>::kOverflow:
                CONSOLE_ERROR("CommsManager::UpdateAT", "AT command buffer overflow.");
                break;
            case ATLineAssembler<kATCommandBufMaxLen>::kBinaryLine:
                CONSOLE_WARNING("CommsManager::UpdateAT", "Dropped a console line containing binary data.");
                break;
            default:
                break;
        }
        if (at_console_guard_.PayloadInProgress()) {
            return true;  // A command above started reading a binary payload (can't happen after it returns, but be safe).
        }
        ci = getchar_timeout_us(0);
    }

    if (esp32.IsEnabled()) {
        // Receive incoming network console characters.
        char c;
        while (!at_console_guard_.PayloadInProgress() && esp32_console_rx_queue.Dequeue(c)) {
            CPUMonitor::WorkScope work;
            switch (network_at_line_.Push(c, get_time_since_boot_ms())) {
                case ATLineAssembler<kATCommandBufMaxLen>::kLine:
                    CONSOLE_INFO("CommsManager::UpdateAT", "Received network console message: %.*s\r\n",
                                 (int)network_at_line_.line().length(), network_at_line_.line().data());
                    at_parser_.ParseMessage(network_at_line_.line());
                    break;
                case ATLineAssembler<kATCommandBufMaxLen>::kOverflow:
                    CONSOLE_ERROR("CommsManager::UpdateAT", "Network console buffer overflow.");
                    break;
                case ATLineAssembler<kATCommandBufMaxLen>::kBinaryLine:
                    CONSOLE_WARNING("CommsManager::UpdateAT", "Dropped a network console line containing binary data.");
                    break;
                default:
                    break;
            }
        }
    }

    return true;
}
