#include <stdio.h>  // for printing

#include <cstring>   // for strcat
#include <iostream>  // for AT command ingestion

#include "adsbee.hh"
#include "comms.hh"
#include "core1.hh"  // For turning multiprocessor on / off during firmware flashing.
#include "eeprom.hh"
#include "esp32.hh"  // Access to ESP32 low level SPICoprocessorSlaveInterface.
#include "esp32_flasher.hh"
#include "firmware_update.hh"
#include "flash_utils.hh"
#include "main.hh"
#include "pico/bootrom.h"  // For Jumping to Bootloader
#include "pico/multicore.h"
#include "pico/stdlib.h"  // for getchar etc
#include "pico/unique_id.h"
#include "settings.hh"
#include "spi_coprocessor.hh"  // For init / de-init before and after flashing ESP32.
#include "gnss_interface.hh"   // For parking/restoring the GNSS UART around ESP32 flashing.

#ifdef HARDWARE_UNIT_TESTS
#include "hardware_unit_tests.hh"
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
            CPP_AT_SUCCESS();
            break;
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

// The ESP32 image runs on modules with and without PSRAM (ESP32-S3-MINI-1U-N4R2 vs -N8) and reports which one it is in
// ESP32DeviceStatus::hardware_capabilities. Returns true only once the ESP32 has positively reported "no PSRAM"; while
// the capability is unknown (ESP32 disabled, not read yet, or older ESP32 firmware) settings are accepted and the ESP32's
// own runtime gating (RemoteIDManager) remains the backstop.
static bool ESP32KnownToLackPSRAM() {
    return (esp32_ll.hardware_capabilities & ObjectDictionary::kESP32HWCapReported) &&
           !(esp32_ll.hardware_capabilities & ObjectDictionary::kESP32HWCapPSRAM);
}

static bool WiFiAPOrSTAEnabled() {
    const SettingsManager::Settings::CoreNetworkSettings& cns = settings_manager.settings.core_network_settings;
    return cns.wifi_ap_enabled || cns.wifi_sta_enabled;
}

static const char kRemoteIDNeedsPSRAMError[] =
    "Remote ID can't run while WiFi AP or WiFi Station is enabled on this hardware: its ESP32 has no PSRAM, so there "
    "isn't enough RAM for WiFi networking and Remote ID at the same time. Disable both (AT+WIFI_AP=0 and "
    "AT+WIFI_STA=0) and use Ethernet, or use hardware with PSRAM.";

// Warns when WiFi AP/STA is being enabled while Remote ID is on, on hardware without PSRAM. WiFi networking wins (it is
// how most users reach the device), so the setting is accepted and Remote ID pauses; this makes that visible.
static void WarnIfWiFiPausesRemoteID() {
    const SettingsManager::Settings& s = settings_manager.settings;
    if (!ESP32KnownToLackPSRAM() || !WiFiAPOrSTAEnabled()) return;
    if (!s.remote_id_rx_enabled && !s.remote_id_tx_enabled) return;
    CPP_AT_PRINTF(
        "WARNING: Remote ID is enabled but will be paused while WiFi AP or WiFi Station is enabled: this hardware's "
        "ESP32 has no PSRAM, so it can't run WiFi networking and Remote ID at the same time.\r\n");
}

CPP_AT_CALLBACK(CommsManager::ATRemoteIDCallback) {
    switch (op) {
        case '?':
            // Report the requested settings plus the ESP32's resolved live status bitfield (why RID may not be running).
            // remote_id_status is cached on the low-level ESP32 slave interface (esp32_ll) by ESP32::Update(); the
            // high-level SPICoprocessor `esp32` only inherits the base interface, which does not carry it.
            CPP_AT_CMD_PRINTF("=%d,0x%02X(TRANSPORTS),0x%02X(ESP32_STATUS)",
                              settings_manager.settings.remote_id_rx_enabled,
                              settings_manager.settings.remote_id_transports, esp32_ll.remote_id_status);
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=':
            if (!CPP_AT_HAS_ARG(0)) {
                CPP_AT_ERROR("Requires at least one argument. AT+REMOTE_ID=<enabled>[,<transport_mask>]");
            }
            bool rx_enabled;
            CPP_AT_TRY_ARG2NUM(0, rx_enabled);
            if (rx_enabled && ESP32KnownToLackPSRAM() && WiFiAPOrSTAEnabled()) {
                CPP_AT_ERROR("%s", kRemoteIDNeedsPSRAMError);
            }
            settings_manager.settings.remote_id_rx_enabled = rx_enabled;
            if (CPP_AT_HAS_ARG(1)) {
                CPP_AT_TRY_ARG2NUM(1, settings_manager.settings.remote_id_transports);
            }
            CPP_AT_CMD_PRINTF(": remote_id_rx_enabled: %d, transports: 0x%02X\r\n",
                              settings_manager.settings.remote_id_rx_enabled,
                              settings_manager.settings.remote_id_transports);
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
        "\tAT+REMOTE_ID_TX=<enabled>[,<transport_mask>[,<uas_id>[,<id_type>[,<ua_type>[,<operator_id>]]]]]\r\n"
        "\tTransmit Broadcast Remote ID (drone ID) from this device, for use as a Remote ID test transmitter or as a "
        "combined ADS-B receiver / Remote ID transmitter on a drone. No WiFi AP/STA is required.\r\n"
        "\ttransport_mask bits: 1=BT4 legacy, 2=BT5 Long Range, 4=WiFi beacon (default 7 = all methods).\r\n"
        "\tuas_id = UAS serial / registration, up to %d chars ('-' to clear and use this device's serial).\r\n"
        "\tid_type = [1 SERIAL, 2 CAA_REGISTRATION, 3 UTM_UUID, 4 SESSION_ID].\r\n"
        "\tua_type = [1 AEROPLANE, 2 HELICOPTER_MULTIROTOR, ... 15 OTHER].\r\n"
        "\toperator_id = operator registration, up to %d chars ('-' to clear).\r\n"
        "\tThe transmitted position comes from AT+RX_POSITION, but ONLY from the FIXED and GNSS sources, which\r\n"
        "\tdescribe this device. The aircraft-derived sources (LOWEST, ICAO) carry a received aircraft's position;\r\n"
        "\tbroadcasting those would place a fabricated drone on a real aircraft, so an unknown position is sent\r\n"
        "\tinstead.\r\n"
        "\tAT+REMOTE_ID_TX?\r\n\tQuery transmit settings and the ESP32's live status bitfield.",
        SettingsManager::Settings::kRemoteIDIDMaxLen, SettingsManager::Settings::kRemoteIDIDMaxLen);
}

CPP_AT_CALLBACK(CommsManager::ATRemoteIDTxCallback) {
    switch (op) {
        case '?':
            // Report the requested settings plus the ESP32's resolved live status bitfield (the high byte carries the
            // transmit state; see RemoteIDManager::Status).
            CPP_AT_CMD_PRINTF("=%d,0x%02X(TRANSPORTS),%s(UAS_ID),%d(ID_TYPE),%d(UA_TYPE),%s(OPERATOR_ID),0x%04X(ESP32_STATUS)",
                              settings_manager.settings.remote_id_tx_enabled,
                              settings_manager.settings.remote_id_tx_transports,
                              settings_manager.settings.remote_id_tx_uas_id[0] == '\0'
                                  ? "(device serial)"
                                  : settings_manager.settings.remote_id_tx_uas_id,
                              settings_manager.settings.remote_id_tx_uas_id_type,
                              settings_manager.settings.remote_id_tx_ua_type,
                              settings_manager.settings.remote_id_tx_operator_id[0] == '\0'
                                  ? "(none)"
                                  : settings_manager.settings.remote_id_tx_operator_id,
                              esp32_ll.remote_id_status);
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=': {
            if (!CPP_AT_HAS_ARG(0)) {
                CPP_AT_ERROR("Requires at least one argument. AT+REMOTE_ID_TX=<enabled>[,<transport_mask>]");
            }
            bool tx_enabled;
            CPP_AT_TRY_ARG2NUM(0, tx_enabled);
            if (tx_enabled && ESP32KnownToLackPSRAM() && WiFiAPOrSTAEnabled()) {
                CPP_AT_ERROR("%s", kRemoteIDNeedsPSRAMError);
            }
            settings_manager.settings.remote_id_tx_enabled = tx_enabled;
            if (CPP_AT_HAS_ARG(1)) {
                CPP_AT_TRY_ARG2NUM(1, settings_manager.settings.remote_id_tx_transports);
            }
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
            CPP_AT_CMD_PRINTF(": remote_id_tx_enabled: %d, transports: 0x%02X\r\n",
                              settings_manager.settings.remote_id_tx_enabled,
                              settings_manager.settings.remote_id_tx_transports);
            WarnIfRemoteIDTxPositionSourceNotTransmittable();
            CPP_AT_SUCCESS();
            break;
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

void ATFeedHelpCallback() {
    CPP_AT_PRINTF(
        "\tAT+FEED=<index>,<uri>,<port>,<active>,<protocol>\r\n\tSet details for a "
        "network feed.\r\n\tindex = [0-%d], uri = ip address or URL, feed_port = [0-65535], "
        "active = [0 1], protocol = [BEAST BEAST_RAW].\r\n\t\r\n\tAT+FEED?\r\n\tPrint details for all "
        "feeds.\r\n\t\r\n\tAT+FEED?<index>\r\n\tPrint details for a specific feed.\r\n\tfeed_index = [0-%d]\r\n",
        SettingsManager::Settings::kMaxNumFeeds - 1, SettingsManager::Settings::kMaxNumFeeds - 1);
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
                char receiver_id_str[SettingsManager::Settings::kFeedReceiverIDNumBytes * 2 + 1];
                SettingsManager::ReceiverIDToStr(settings_manager.settings.feed_receiver_ids[index], receiver_id_str);
                CPP_AT_CMD_PRINTF(
                    "=%d(INDEX),%s(URI),%d(PORT),%d(ACTIVE),%s(PROTOCOL),%s(RECEIVER_ID)", index,
                    settings_manager.settings.feed_uris[index], settings_manager.settings.feed_ports[index],
                    settings_manager.settings.feed_is_active[index],
                    SettingsManager::kReportingProtocolStrs[settings_manager.settings.feed_protocols[index]],
                    receiver_id_str);
            } else {
                // Querying info about all feeds.
                for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
                    char receiver_id_str[SettingsManager::Settings::kFeedReceiverIDNumBytes * 2 + 1];
                    SettingsManager::ReceiverIDToStr(settings_manager.settings.feed_receiver_ids[i], receiver_id_str);
                    CPP_AT_CMD_PRINTF(
                        "=%d(INDEX),%s(URI),%d(PORT),%d(ACTIVE),%s(PROTOCOL),%s(RECEIVER_ID)", i,
                        settings_manager.settings.feed_uris[i], settings_manager.settings.feed_ports[i],
                        settings_manager.settings.feed_is_active[i],
                        SettingsManager::kReportingProtocolStrs[settings_manager.settings.feed_protocols[i]],
                        receiver_id_str);
                }
            }
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=':
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
            // Set FEED_URI. A single "-" is a sentinel meaning "clear the URI to empty".
            if (CPP_AT_HAS_ARG(1)) {
                if (args[1] == "-") {
                    settings_manager.settings.feed_uris[index][0] = '\0';
                } else {
                    strncpy(settings_manager.settings.feed_uris[index], args[1].data(),
                            SettingsManager::Settings::kFeedURIMaxNumChars);
                    settings_manager.settings.feed_uris[index][SettingsManager::Settings::kFeedURIMaxNumChars] = '\0';
                }
            }
            // Set FEED_PORT
            if (CPP_AT_HAS_ARG(2)) {
                CPP_AT_TRY_ARG2NUM(2, settings_manager.settings.feed_ports[index]);
            }
            // Set ACTIVE
            if (CPP_AT_HAS_ARG(3)) {
                uint8_t is_active;
                CPP_AT_TRY_ARG2NUM(3, settings_manager.settings.feed_is_active[index]);
            }
            // Set PROTOCOL
            if (CPP_AT_HAS_ARG(4)) {
                SettingsManager::ReportingProtocol feed_protocol;
                for (uint16_t i = 0; i < SettingsManager::ReportingProtocol::kNumProtocols; i++) {
                    if (args[4].compare(SettingsManager::kReportingProtocolStrs[i]) == 0) {
                        feed_protocol = static_cast<SettingsManager::ReportingProtocol>(i);
                    }
                }
                settings_manager.settings.feed_protocols[index] = feed_protocol;
            }
            settings_manager.SyncToCoprocessors();
            CPP_AT_SUCCESS();
            break;
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

                    // Send OK to indicate that we're ready to receive data.
                    CPP_AT_PRINTF("READY\r\n");

                    uint32_t old_esp32_heartbeat_ms = esp32.update_interval_ms;
                    esp32.update_interval_ms = kOTAHeartbeatMs;  // Faster heartbeat during OTA.
                    int32_t buf_len_bytes = ATReadConsole(reinterpret_cast<char*>(buf), len_bytes, kOTAWriteTimeoutMs);
                    esp32.update_interval_ms = old_esp32_heartbeat_ms;  // Restore old heartbeat.

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

CPP_AT_CALLBACK(CommsManager::ATNetworkInfoCallback) {
    switch (op) {
        case '?':
            ObjectDictionary::ESP32NetworkInfo network_info;
            if (!esp32.Read(ObjectDictionary::Address::kAddrESP32NetworkInfo, network_info, sizeof(network_info))) {
                CPP_AT_ERROR("Error while reading network info from ESP32.");
            }

            CPP_AT_PRINTF("Ethernet: %s\r\n", network_info.ethernet_enabled ? "ENABLED" : "DISABLED");
            if (!network_info.ethernet_has_ip) {
                CPP_AT_PRINTF("\tNo IP address assigned.\r\n");
            } else {
                CPP_AT_PRINTF("\tIP Address: %s\r\n", network_info.ethernet_ip);
                CPP_AT_PRINTF("\tSubnet Mask: %s\r\n", network_info.ethernet_netmask);
                CPP_AT_PRINTF("\tGateway: %s\r\n", network_info.ethernet_gateway);
            }

            CPP_AT_PRINTF("WiFi Station: %s\r\n", network_info.wifi_sta_enabled ? "ENABLED" : "DISABLED");
            CPP_AT_PRINTF("\tSSID: %s\r\n", network_info.wifi_sta_ssid);
            if (!network_info.wifi_sta_has_ip) {
                CPP_AT_PRINTF("\tNo IP address assigned.\r\n");
            } else {
                CPP_AT_PRINTF("\tIP Address: %s\r\n", network_info.wifi_sta_ip);
                CPP_AT_PRINTF("\tSubnet Mask: %s\r\n", network_info.wifi_sta_netmask);
                CPP_AT_PRINTF("\tGateway: %s\r\n", network_info.wifi_sta_gateway);
            }

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
    CPP_AT_PRINTF("\"BAUD_RATE\":[%lu,%lu],",
                  (unsigned long)s.baud_rates[SettingsManager::SerialInterface::kCommsUART],
                  (unsigned long)s.baud_rates[SettingsManager::SerialInterface::kGNSSUART]);
    CPP_AT_PRINTF("\"BIAS_TEE_ENABLE\":[%d,%d],", adsbee.BiasTeeIsEnabled(), s.subg_bias_tee_enabled);
    CPP_AT_PRINTF("\"ETHERNET\":[%d],", cns.ethernet_enabled);
    CPP_AT_PRINTF("\"ESP32_ENABLE\":[%d],", esp32.IsEnabled());
    CPP_AT_PRINTF("\"FEED_ENABLE\":[%d],", s.feeds_enabled);
    CPP_AT_PRINTF("\"GNSS\":[%d,\"%s\",%d],", s.gnss_enabled,
                  GNSSModuleTypeToStr(SettingsToGNSSModuleType(s.gnss_receiver_type)), s.gnss_notify);
    CPP_AT_PRINTF("\"HOSTNAME\":[\"%s\"],", JSONEscapeStr(cns.hostname, esc, sizeof(esc)));
    CPP_AT_PRINTF("\"LED_ENABLE\":[%d],", s.led_enabled);
    CPP_AT_PRINTF("\"LOG_LEVEL\":[\"%s\"],", SettingsManager::kConsoleLogLevelStrs[s.log_level]);
    CPP_AT_PRINTF("\"MAVLINK_ID\":[%d,%d],", s.mavlink_system_id, s.mavlink_component_id);
    CPP_AT_PRINTF("\"PROTOCOL_OUT\":[\"%s\",\"%s\"],",
                  SettingsManager::kReportingProtocolStrs[s.reporting_protocols[SettingsManager::kConsole]],
                  SettingsManager::kReportingProtocolStrs[s.reporting_protocols[SettingsManager::kCommsUART]]);
    CPP_AT_PRINTF("\"REMOTE_ID\":[%d,%u,\"0x%02X\"],", s.remote_id_rx_enabled, s.remote_id_transports,
                  esp32_ll.remote_id_status);
    CPP_AT_PRINTF("\"REMOTE_ID_TX\":[%d,%u,\"%s\",", s.remote_id_tx_enabled, s.remote_id_tx_transports,
                  JSONEscapeStr(s.remote_id_tx_uas_id, esc, sizeof(esc)));
    CPP_AT_PRINTF("%d,%d,\"%s\",\"0x%04X\"],", s.remote_id_tx_uas_id_type, s.remote_id_tx_ua_type,
                  JSONEscapeStr(s.remote_id_tx_operator_id, esc, sizeof(esc)), esp32_ll.remote_id_status);
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
    CPP_AT_PRINTF("\"WIFI_STA\":[%d,\"%s\"]", cns.wifi_sta_enabled, JSONEscapeStr(cns.wifi_sta_ssid, esc, sizeof(esc)));
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
            CPP_AT_CMD_PRINTF("=%dmV (%d dBm)\r\n", tl_mv, adsbee.AD8313MilliVoltsTodBm(tl_mv));
            CPP_AT_SILENT_SUCCESS();
            break;
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

/**
 * AT+TL_OFFSET Callback
 * AT+TL_OFFSET=<tl_offset_mv>
 *  tl_offset_mv = Trigger Level Offset, in milliVolts.
 * AT+TL_OFFSET?
 * +TL_OFFSET=
 */
CPP_AT_CALLBACK(CommsManager::ATTLSetCallback) {
    switch (op) {
        case '?': {
            // AT+TL_SET value query.
            int tl_mv = adsbee.GetTLOffsetMilliVolts();
            CPP_AT_CMD_PRINTF("=%dmV (%d dBm)\r\n", tl_mv, adsbee.AD8313MilliVoltsTodBm(tl_mv));
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
            if (CPP_AT_HAS_ARG(0)) {
                CPP_AT_TRY_ARG2NUM(0, cns.wifi_ap_enabled);
                CPP_AT_CMD_PRINTF(": wifi_ap_enabled=%d\r\n", cns.wifi_ap_enabled);
                if (cns.wifi_ap_enabled) WarnIfWiFiPausesRemoteID();
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
            if (CPP_AT_HAS_ARG(0)) {
                CPP_AT_TRY_ARG2NUM(0, cns.wifi_sta_enabled);
                CPP_AT_CMD_PRINTF(": wifi_sta_enabled=%d\r\n", cns.wifi_sta_enabled);
                if (cns.wifi_sta_enabled) WarnIfWiFiPausesRemoteID();
            }
            if (CPP_AT_HAS_ARG(1)) {
                strncpy(cns.wifi_sta_ssid, args[1].data(), SettingsManager::Settings::kWiFiSSIDMaxLen);
                cns.wifi_sta_ssid[SettingsManager::Settings::kWiFiSSIDMaxLen] = '\0';
                CPP_AT_CMD_PRINTF(": sta_ssid=%s\r\n", cns.wifi_sta_ssid);
            }
            if (CPP_AT_HAS_ARG(2)) {
                strncpy(cns.wifi_sta_password, args[2].data(), SettingsManager::Settings::kWiFiPasswordMaxLen);
                cns.wifi_sta_password[SettingsManager::Settings::kWiFiPasswordMaxLen] = '\0';
                char redacted_password[SettingsManager::Settings::kWiFiPasswordMaxLen];
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

const CppAT::ATCommandDef_t at_command_list[] = {
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
    {.command = "BOOT_USB_UF2",
     .min_args = 0,
     .max_args = 1,
     .help_string = "Reboot ADSBee into RP2040 USB Bootloader.\r\n\tDo not use unless you have a USB connection to the "
                    "ADSBee\r\n\t"
                    "AT+BOOT_USB_UF2=1DEADBEE",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATBootloader, comms_manager)},
    {.command = "DEVICE_INFO",
     .min_args = 0,
     .max_args = 5,  // TODO: check this value.
     .help_string = "AT+DEVICE_INFO?\r\n\tQuery device information.",
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
     .max_args = 5,
     .help_callback = ATFeedHelpCallback,
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATFeedCallback, comms_manager)},
    {.command = "FEED_ENABLE",
     .min_args = 0,
     .max_args = 1,
     .help_string = "AT+FEED_ENABLE=<enabled>\r\n\tMaster switch for all outbound network feeds. Set to 0 to disable "
                    "all feeds, or 1 to allow feeds marked active. Does not change per-feed active flags set with "
                    "AT+FEED.\r\n\tAT+FEED_ENABLE?\r\n\tQuery whether outbound feeds are enabled.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATFeedEnableCallback, comms_manager)},
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
         "AT+REMOTE_ID=<enabled>[,<transport_mask>]\r\n\tEnable/disable Broadcast Remote ID (drone) reception on the "
         "ESP32.\r\n\ttransport_mask bits: 1=BT4 legacy, 2=BT5 Long Range, 4=WiFi beacon (default 7).\r\n\tOn "
         "hardware whose ESP32 has no PSRAM (see AT+DEVICE_INFO?), Remote ID only runs when WiFi AP/STA are disabled "
         "and Ethernet is up; with PSRAM it runs alongside WiFi (the WiFi beacon receiver then listens on the WiFi "
         "channel only).\r\n\t"
         "AT+REMOTE_ID?\r\n\tQuery Remote ID settings and the ESP32's live status bitfield.",
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
                    "detector.\r\n\tAT+TL_OFFSET=<tl_offset_mv>"
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
         "Get WiFi access point params.\r\n\tAT+WIFI_AP?\r\n\t+WIFI_AP=<enabled>,<ap_ssid>,<ap_pwd>,<ap_channel>",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATWiFiAPCallback, comms_manager)},
    {.command = "WIFI_STA",
     .min_args = 0,
     .max_args = 3,
     .help_string = "Set WiFi station params.\r\n\tAT+WIFI_STA=<enabled>,<sta_ssid>,<sta_pwd>\r\n\t"
                    "Get WiFi station params.\r\n\tAT+WIFI_STA?\r\n\t+WIFI_STA=<enabled>,<sta_ssid>,<sta_pwd>",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATWiFiSTACallback, comms_manager)},
};
const uint16_t at_command_list_num_commands = sizeof(at_command_list) / sizeof(at_command_list[0]);

bool CommsManager::UpdateAT() {
    static char stdio_at_command_buf[kATCommandBufMaxLen + 1];
    static uint16_t stdio_at_command_buf_len = 0;
    // Check for new AT commands from STDIO. Process up to one line per loop.
    char c = static_cast<char>(getchar_timeout_us(0));
    while (static_cast<int8_t>(c) != PICO_ERROR_TIMEOUT) {
        stdio_at_command_buf[stdio_at_command_buf_len] = c;
        stdio_at_command_buf_len++;
        stdio_at_command_buf[stdio_at_command_buf_len] = '\0';
        if (stdio_at_command_buf_len >= kATCommandBufMaxLen) {
            CONSOLE_ERROR("CommsManager::UpdateAT", "AT command buffer overflow.");
            stdio_at_command_buf_len = 0;
            stdio_at_command_buf[stdio_at_command_buf_len] = '\0';  // clear command buffer
        }
        if (c == '\n') {
            at_parser_.ParseMessage(std::string_view(stdio_at_command_buf));
            stdio_at_command_buf_len = 0;
            stdio_at_command_buf[stdio_at_command_buf_len] = '\0';  // clear command buffer
        }
        c = static_cast<char>(getchar_timeout_us(0));
    }

    if (esp32.IsEnabled()) {
        // Receive incoming network console characters.
        static char esp32_console_rx_buf[kATCommandBufMaxLen + 1];
        static uint16_t esp32_console_rx_buf_len = 0;
        while (esp32_console_rx_queue.Dequeue(c)) {
            esp32_console_rx_buf[esp32_console_rx_buf_len] = c;
            esp32_console_rx_buf_len++;
            esp32_console_rx_buf[esp32_console_rx_buf_len] = '\0';
            if (esp32_console_rx_buf_len >= kATCommandBufMaxLen) {
                CONSOLE_ERROR("CommsManager::UpdateAT", "Network console buffer overflow.");
                esp32_console_rx_buf_len = 0;
                esp32_console_rx_buf[esp32_console_rx_buf_len] = '\0';  // clear command buffer
            }
            if (c == '\n') {
                CONSOLE_INFO("CommsManager::UpdateAT", "Received network console message: %s\r\n",
                             esp32_console_rx_buf);
                at_parser_.ParseMessage(std::string_view(esp32_console_rx_buf));
                esp32_console_rx_buf_len = 0;
                esp32_console_rx_buf[esp32_console_rx_buf_len] = '\0';  // clear command buffer
            }
        }
    }

    return true;
}
