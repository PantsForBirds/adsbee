#include <stdio.h>  // for printing

#include <cstring>   // for strcat
#include <iostream>  // for AT command ingestion

#include "adsbee.hh"
#include "comms.hh"
#include "object_dictionary.hh"
#include "packet_decoder.hh"
#include "settings.hh"
#include "sub_ghz_radio.hh"

/* clang-format off */
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(inc/hw_types.h)
#include DeviceFamily_constructPath(inc/hw_memmap.h)
#include DeviceFamily_constructPath(inc/hw_fcfg1.h)
/* clang-format on */

#ifdef HARDWARE_UNIT_TESTS
#include "hardware_unit_tests.hh"
#endif

// For mapping cpp_at_printf
#include <cstdarg>
// #include "printf.h" // for using custom printf defined in printf.h
#include <cstdio>  // for using regular printf

// This is intended to stop people from accidentally modifying serial number information on their device.
const uint32_t kDeviceInfoProgrammingPassword = 0xDEDBEEF;

/** stdio _write retarget — routes all printf/puts/etc. to the console interface **/
extern "C" int _write(int /*fd*/, const char* buf, int len) {
    comms_manager.iface_printf(SettingsManager::SerialInterface::kConsole, "%.*s", len, buf);
    return len;
}

/** CppAT Printf Override **/
int CppAT::cpp_at_printf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    int ret = comms_manager.iface_vprintf(SettingsManager::SerialInterface::kConsole, format, args);
    va_end(args);

    return ret;
}

/** AT Command Callback Functions **/

CPP_AT_CALLBACK(CommsManager::ATBaudRateCallback) {
    switch (op) {
        case '?':
            // Prints "BAUD_RATE=CONSOLE,<baud>".
            CPP_AT_CMD_PRINTF("=%s,%lu",
                              SettingsManager::kSerialInterfaceStrs[SettingsManager::SerialInterface::kConsole],
                              (unsigned long)GetBaudRate());
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=': {
            if (!(CPP_AT_HAS_ARG(0) && CPP_AT_HAS_ARG(1))) {
                CPP_AT_ERROR("Requires two arguments: AT+BAUD_RATE=CONSOLE,<baud>.");
            }
            if (args[0].compare(
                    SettingsManager::kSerialInterfaceStrs[SettingsManager::SerialInterface::kConsole]) != 0) {
                CPP_AT_ERROR("Invalid interface %s; only CONSOLE is supported.", args[0].data());
            }
            uint32_t baud;
            CPP_AT_TRY_ARG2NUM(1, baud);
            if (!IsAllowedBaudRate(baud)) {
                CPP_AT_ERROR("Baud %lu not in {115200 230400 460800 921600 1000000}.", (unsigned long)baud);
            }
            // All validation is done, so the command cannot fail: acknowledge at the OLD baud rate,
            // then switch. SetBaudRate() drains the TX line (including this OK) before reconfiguring,
            // so the host reads the acknowledgment intact and then reopens its port at the new rate.
            // Note the inversion vs the ADSBee 1090, which reconfigures before printing OK — atomic on
            // the RP2040 UART, but not across TI's UART2 close/reopen.
            CPP_AT_PRINTF("OK\r\n");
            SetBaudRate(baud);
            CPP_AT_SILENT_SUCCESS();
            break;
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
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
            // Get 8-byte unique ID from CC1314R10 FCFG1 IEEE 802.15.4 EUI-64 registers.
            // (The BLE MAC field is unprogrammed on this sub-GHz part and reads 0xFFFFFFFF.)
            uint32_t unique_id_lo = HWREG(FCFG1_BASE + FCFG1_O_MAC_15_4_0);
            uint32_t unique_id_hi = HWREG(FCFG1_BASE + FCFG1_O_MAC_15_4_1);
            CPP_AT_PRINTF("CC1314R10 Unique ID: %02X%02X%02X%02X%02X%02X%02X%02X\r\n", (unique_id_hi >> 24) & 0xFF,
                          (unique_id_hi >> 16) & 0xFF, (unique_id_hi >> 8) & 0xFF, unique_id_hi & 0xFF,
                          (unique_id_lo >> 24) & 0xFF, (unique_id_lo >> 16) & 0xFF, (unique_id_lo >> 8) & 0xFF,
                          unique_id_lo & 0xFF);
            if (object_dictionary.kFirmwareVersionReleaseCandidate == 0) {
                // Indicates a finalized release; no need to print release candidate number.
                CPP_AT_PRINTF("CC1314R10 Firmware Version: %d.%d.%d\r\n", object_dictionary.kFirmwareVersionMajor,
                              object_dictionary.kFirmwareVersionMinor, object_dictionary.kFirmwareVersionPatch);
            } else {
                // Print release candidate number.
                CPP_AT_PRINTF("CC1314R10 Firmware Version: %d.%d.%d-rc%d\r\n", object_dictionary.kFirmwareVersionMajor,
                              object_dictionary.kFirmwareVersionMinor, object_dictionary.kFirmwareVersionPatch,
                              object_dictionary.kFirmwareVersionReleaseCandidate);
            }

            for (uint16_t i = 0; i < SettingsManager::DeviceInfo::kNumOTAKeys; i++) {
                CPP_AT_PRINTF("OTA Key %d: %s\r\n", i, device_info.ota_keys[i]);
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
                    CPP_AT_SUCCESS();
                }
            }
            CPP_AT_ERROR("Verbosity level %s not recognized.", args[0].data());
            break;
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATLREnableCallback) {
    switch (op) {
        case '?':
            CPP_AT_CMD_PRINTF("=%d", adsbee.LR2021IsEnabled());
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=': {
            if (!CPP_AT_HAS_ARG(0)) {
                CPP_AT_ERROR("Requires an argument: AT+LR_ENABLE=<enabled [1,0]>.");
            }
            bool enabled;
            CPP_AT_TRY_ARG2NUM(0, enabled);
            if (!adsbee.SetLR2021Enabled(enabled)) {
                CPP_AT_ERROR("Failed to %s the LR2021 interface.", enabled ? "enable" : "disable");
            }
            CPP_AT_SUCCESS();
            break;
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATR1090GainCallback) {
    switch (op) {
        case '?': {
            uint8_t gain = adsbee.GetR1090Gain();
            if (gain == 0) {
                CPP_AT_CMD_PRINTF("=AUTO");
            } else {
                CPP_AT_CMD_PRINTF("=%u", gain);
            }
            CPP_AT_SILENT_SUCCESS();
            break;
        } case '=': {
            if (!CPP_AT_HAS_ARG(0)) {
                CPP_AT_ERROR("Need to specify a gain step (AUTO or 0=auto, 1-15).");
            }
            if (args[0].compare("AUTO") == 0) {
                settings_manager.settings.r1090_gain = 0;
                adsbee.SetR1090Gain(0);
                CPP_AT_SUCCESS();
            }
            uint16_t gain;
            CPP_AT_TRY_ARG2NUM(0, gain);
            if (gain > 15) {
                CPP_AT_ERROR("Gain step %u out of range (0=auto, 1-15).", gain);
            }
            settings_manager.settings.r1090_gain = static_cast<uint8_t>(gain);
            adsbee.SetR1090Gain(static_cast<uint8_t>(gain));
            CPP_AT_SUCCESS();
            break;
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATR1090PreambleCallback) {
    switch (op) {
        case '?':
            CPP_AT_CMD_PRINTF("=%s", SettingsManager::kR1090PreambleModeStrs[adsbee.GetR1090PreambleMode()]);
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=':
            if (!CPP_AT_HAS_ARG(0)) {
                CPP_AT_ERROR("Need to specify a Mode S preamble mode.");
            }
            for (uint16_t i = 0; i < SettingsManager::kNumR1090PreambleModes; i++) {
                if (args[0].compare(SettingsManager::kR1090PreambleModeStrs[i]) == 0) {
                    SettingsManager::R1090PreambleMode mode = static_cast<SettingsManager::R1090PreambleMode>(i);
                    settings_manager.settings.r1090_preamble_mode = mode;
                    adsbee.SetR1090PreambleMode(mode);
                    CPP_AT_SUCCESS();
                }
            }
            CPP_AT_ERROR("Mode S preamble mode %s not recognized.", args[0].data());
            break;
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATR1090RxBoostCallback) {
    switch (op) {
        case '?': {
            CPP_AT_CMD_PRINTF("=%u", adsbee.GetR1090RxBoost());
            CPP_AT_SILENT_SUCCESS();
            break;
        }
        case '=': {
            if (!CPP_AT_HAS_ARG(0)) {
                CPP_AT_ERROR("Need to specify a boost level (0=off, 1-7, 7=max).");
            }
            uint16_t rx_boost;
            CPP_AT_TRY_ARG2NUM(0, rx_boost);
            if (rx_boost > LR2021::RxBoost::kBoostMax) {
                CPP_AT_ERROR("Boost level %u out of range (0=off, 1-7).", rx_boost);
            }
            settings_manager.settings.r1090_rx_boost = static_cast<uint8_t>(rx_boost);
            adsbee.SetR1090RxBoost(static_cast<uint8_t>(rx_boost));
            CPP_AT_SUCCESS();
            break;
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATRxStatsCallback) {
    // Both branches read/write LR2021 registers over SPI; with the interface disabled the chip is in
    // reset and the bus is released, so WaitUntilReady would stall ~100 ms before failing. Fail fast.
    if (!adsbee.LR2021IsEnabled()) {
        CPP_AT_ERROR("LR2021 interface disabled (AT+LR_ENABLE=0); chip stats unavailable.");
    }
    switch (op) {
        case '?': {
            LR2021::OokRxStatsAdv stats;
            if (!adsbee.lr2021.GetOokRxStatsAdv(&stats)) {
                CPP_AT_ERROR("Failed to read LR2021 Rx stats.");
            }
            CPP_AT_CMD_PRINTF(
                "=pkt_rx=%u,crc_error=%u,len_error=%u,pbl_det=%u,sync_ok=%u,sync_fail=%u,timeout=%u,"
                "fifo_full=%lu,q_ovf=%lu,bitflips_fixed=%lu,uat_rx_restarts=%lu,uat_setlen_errs=%lu,"
                "max_loop_us=%lu,max_lr2021_us=%lu,drain_max_us=%lu,"
                "irq_drains=%lu,irq_skip_busy=%lu,irq_skip_slots=%lu,irq_restarts=%lu,irq_timeouts=%lu,"
                "irq_errors=%lu,irq_assists=%lu,tx_stalls=%lu,tx_drops=%lu,tx_ring_hw=%u,"
                "fifo_ovf=%lu,fifo_resyncs=%lu,rx_rearms=%lu,rx_reconfigs=%lu,cfg_fails=%lu,"
                "dma_timeouts=%lu,report_q_ovf=%lu,"
                "validity_reconfigs=%lu,stale_edges=%lu,uat_len_mismatch=%lu",
                stats.pkt_rx, stats.crc_error, stats.len_error, stats.pbl_det, stats.sync_ok, stats.sync_fail,
                stats.timeout, (unsigned long)adsbee.lr2021_fifo_full_count,
                (unsigned long)packet_decoder.raw_queue_overflow_count,
                (unsigned long)packet_decoder.bitflips_fixed_count, (unsigned long)subg_radio.rx_error_restart_count,
                (unsigned long)subg_radio.set_len_error_count,
                (unsigned long)adsbee.max_loop_us, (unsigned long)adsbee.max_lr2021_us,
                (unsigned long)adsbee.lr2021_drain_max_us, (unsigned long)adsbee.lr2021.irq_drain_count,
                (unsigned long)adsbee.lr2021.irq_drain_skip_busy, (unsigned long)adsbee.lr2021.irq_drain_skip_slots,
                (unsigned long)adsbee.lr2021.irq_drain_restarts, (unsigned long)adsbee.lr2021.irq_chain_timeouts,
                (unsigned long)adsbee.lr2021.irq_chain_errors, (unsigned long)adsbee.lr2021.irq_thread_assist_posts,
                (unsigned long)comms_manager.uart_tx_stall_count, (unsigned long)comms_manager.uart_tx_drop_count,
                comms_manager.uart_tx_high_water_bytes, (unsigned long)adsbee.lr2021.fifo_overflow_count,
                (unsigned long)adsbee.lr2021_fifo_resync_count, (unsigned long)adsbee.lr2021_rx_rearm_count,
                (unsigned long)adsbee.lr2021_rx_reconfig_count, (unsigned long)adsbee.lr2021_config_fail_count,
                (unsigned long)adsbee.lr2021.drain_dma_timeouts,
                (unsigned long)comms_manager.mode_s_report_queue_ovf_count,
                (unsigned long)adsbee.lr2021_validity_reconfig_count, (unsigned long)adsbee.lr2021.irq_stale_edges,
                (unsigned long)subg_radio.uat_len_mismatch_count);
            CPP_AT_SILENT_SUCCESS();
            break;
        }
        case '=': {
            if (!CPP_AT_HAS_ARG(0) || args[0].compare("RESET") != 0) {
                CPP_AT_ERROR("Only AT+RX_STATS=RESET is supported.");
            }
            if (!adsbee.lr2021.ResetRxStats()) {
                CPP_AT_ERROR("Failed to reset LR2021 Rx stats.");
            }
            adsbee.lr2021_fifo_full_count = 0;
            packet_decoder.raw_queue_overflow_count = 0;
            packet_decoder.bitflips_fixed_count = 0;
            subg_radio.rx_error_restart_count = 0;
            subg_radio.set_len_error_count = 0;
            adsbee.max_loop_us = 0;
            adsbee.max_lr2021_us = 0;
            adsbee.lr2021_drain_max_us = 0;
            adsbee.lr2021.irq_drain_count = 0;
            adsbee.lr2021.irq_drain_skip_busy = 0;
            adsbee.lr2021.irq_drain_skip_slots = 0;
            adsbee.lr2021.irq_drain_restarts = 0;
            adsbee.lr2021.irq_chain_timeouts = 0;
            adsbee.lr2021.irq_chain_errors = 0;
            adsbee.lr2021.irq_thread_assist_posts = 0;
            comms_manager.uart_tx_stall_count = 0;
            comms_manager.uart_tx_drop_count = 0;
            comms_manager.uart_tx_high_water_bytes = 0;
            adsbee.lr2021.fifo_overflow_count = 0;
            adsbee.lr2021_fifo_resync_count = 0;
            adsbee.lr2021_rx_rearm_count = 0;
            adsbee.lr2021_rx_reconfig_count = 0;
            adsbee.lr2021_config_fail_count = 0;
            adsbee.lr2021.drain_dma_timeouts = 0;
            comms_manager.mode_s_report_queue_ovf_count = 0;
            adsbee.lr2021_validity_reconfig_count = 0;
            adsbee.lr2021.irq_stale_edges = 0;
            subg_radio.uat_len_mismatch_count = 0;
            CPP_AT_SUCCESS();
            break;
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

CPP_AT_CALLBACK(CommsManager::ATProtocolOutCallback) {
    switch (op) {
        case '?':
            // Print out reporting protocols for CONSOLE and COMMS_UART.
            for (uint16_t iface = 0; iface < SettingsManager::SerialInterface::kNumSerialInterfaces; iface++) {
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
            for (uint16_t iface = 0; iface < SettingsManager::SerialInterface::kNumSerialInterfaces; iface++) {
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
    for (uint16_t iface = 0; iface < SettingsManager::SerialInterface::kNumSerialInterfaces; iface++) {
        CPP_AT_PRINTF("%s ", SettingsManager::kSerialInterfaceStrs[iface]);
    }
    CPP_AT_PRINTF("\r\n\t<protocol> = ");
    for (uint16_t protocol = 0; protocol < SettingsManager::ReportingProtocol::kNumProtocols; protocol++) {
        CPP_AT_PRINTF("\t\t%s ", SettingsManager::kReportingProtocolStrs[protocol]);
    }
    CPP_AT_PRINTF("\r\n\tQuery the reporting protocol used on all interfaces:\r\n");
    CPP_AT_PRINTF("\tAT+PROTOCOL_OUT?\r\n\tPROTOCOL_OUT=<iface>,<protocol>\r\n\t...\r\n");
}

CPP_AT_CALLBACK(CommsManager::ATBootUARTBootloaderCallback) {
    switch (op) {
        case '?':
            CPP_AT_PRINTF(
                "AT+BOOT_UART_BOOTLOADER=1DEADBEE enters the CC1314 ROM UART bootloader (DIO2/DIO3) for "
                "reflashing. Erases the app image; the device stays in the bootloader until reflashed.\r\n");
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=':
            if (!CPP_AT_HAS_ARG(0) || args[0].compare("1DEADBEE") != 0) {
                CPP_AT_ERROR("Must confirm with AT+BOOT_UART_BOOTLOADER=1DEADBEE.");
            }
            adsbee.EnterUARTBootloader();  // Does not return.
            CPP_AT_ERROR("Failed to enter bootloader.");
            break;
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
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
                if (!adsbee.SetRx1090Enabled(all_enabled)) {
                    CPP_AT_ERROR("Failed to %s the 1090 receiver.", all_enabled ? "enable" : "disable");
                }
                if (!adsbee.SetRxSubGHzEnabled(all_enabled)) {
                    CPP_AT_ERROR("Failed to %s the Sub-GHz receiver.", all_enabled ? "enable" : "disable");
                }
            } else {
                if (CPP_AT_HAS_ARG(1)) {
                    bool rx_1090_enabled;
                    CPP_AT_TRY_ARG2NUM(1, rx_1090_enabled);
                    if (!adsbee.SetRx1090Enabled(rx_1090_enabled)) {
                        CPP_AT_ERROR("Failed to %s the 1090 receiver.", rx_1090_enabled ? "enable" : "disable");
                    }
                }
                if (CPP_AT_HAS_ARG(2)) {
                    bool rx_subg_enabled;
                    CPP_AT_TRY_ARG2NUM(2, rx_subg_enabled);
                    if (!adsbee.SetRxSubGHzEnabled(rx_subg_enabled)) {
                        CPP_AT_ERROR("Failed to %s the Sub-GHz receiver.", rx_subg_enabled ? "enable" : "disable");
                    }
                }
            }
            CPP_AT_SUCCESS();
            break;
        case '?':
            CPP_AT_PRINTF("1090 Receiver: %s\r\nSubG Receiver: %s\r\n",
                          adsbee.Rx1090IsEnabled() ? "ENABLED" : "DISABLED",
                          adsbee.RxSubGHzIsEnabled() ? "ENABLED" : "DISABLED");
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
        "<1090_enabled [ENABLED,DISABLED]>\r\n\tSubG Receiver: <subg_enabled [ENABLED,DISABLED]>\r\n\tQuery whether "
        "the "
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
            CPP_AT_SUCCESS();
            break;
        }
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

// Prints all settings as one JSON object on a single line: SETTINGS={"CMD":[...],...}.
// Each value is an array ordered to match the web settings GUI's field list for that
// command (the writable AT= args plus query-only extras like position availability),
// using the same value representations as the individual AT queries (enum strings, 0/1
// booleans). Emitted in multiple printf calls to stay under kPrintfBufferMaxSize.
// Keep in sync with SETTINGS_SCHEMA_1421 in
// software/adsbee_1421_console/adsbee_1421_console.html when adding commands. No
// settings on this product hold user-entered strings, so no JSON escaping is needed.
static void PrintSettingsJSON() {
    const SettingsManager::Settings& s = settings_manager.settings;

    char gain_str[8];
    uint8_t gain = adsbee.GetR1090Gain();
    if (gain == 0) {
        snprintf(gain_str, sizeof(gain_str), "\"AUTO\"");
    } else {
        snprintf(gain_str, sizeof(gain_str), "%u", gain);
    }

    CPP_AT_PRINTF("SETTINGS={");
    CPP_AT_PRINTF("\"BAUD_RATE\":[%lu],", (unsigned long)comms_manager.GetBaudRate());
    CPP_AT_PRINTF("\"LOG_LEVEL\":[\"%s\"],", SettingsManager::kConsoleLogLevelStrs[s.log_level]);
    CPP_AT_PRINTF("\"LR_ENABLE\":[%d],", adsbee.LR2021IsEnabled());
    CPP_AT_PRINTF("\"MAVLINK_ID\":[%d,%d],", s.mavlink_system_id, s.mavlink_component_id);
    CPP_AT_PRINTF("\"PROTOCOL_OUT\":[\"%s\"],",
                  SettingsManager::kReportingProtocolStrs[s.reporting_protocols[SettingsManager::kConsole]]);
    CPP_AT_PRINTF("\"R1090_GAIN\":[%s],", gain_str);
    CPP_AT_PRINTF("\"R1090_PREAMBLE\":[\"%s\"],",
                  SettingsManager::kR1090PreambleModeStrs[adsbee.GetR1090PreambleMode()]);
    CPP_AT_PRINTF("\"R1090_RX_BOOST\":[%u],", adsbee.GetR1090RxBoost());
    CPP_AT_PRINTF("\"RX_ENABLE\":[%d,%d],", adsbee.Rx1090IsEnabled(), adsbee.RxSubGHzIsEnabled());
    CPP_AT_PRINTF("\"RX_POSITION\":[\"%s\",\"%s\",%.6f,%.6f,%d,%d,%.1f,%d,\"%06X\"],",
                  SettingsManager::RxPosition::kPositionSourceStrs[adsbee.rx_position.source],
                  adsbee.rx_position_available ? "OK" : "NOT AVAILABLE", adsbee.rx_position.latitude_deg,
                  adsbee.rx_position.longitude_deg, (int)adsbee.rx_position.gnss_altitude_ft,
                  (int)adsbee.rx_position.baro_altitude_ft, adsbee.rx_position.heading_deg,
                  (int)adsbee.rx_position.speed_kts, (unsigned)adsbee.rx_position.icao_address);
    CPP_AT_PRINTF("\"SUBG_MODE\":[\"%s\"],", SettingsManager::kSubGHzModeStrs[subg_radio.GetMode()]);
    CPP_AT_PRINTF("\"WATCHDOG\":[%lu]", (unsigned long)adsbee.GetWatchdogTimeoutSec());
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
                    // Acknowledge at the old baud rate first: Apply() may switch the console to the
                    // default baud, and the host must read the OK intact (mirrors the AT+BAUD_RATE
                    // ordering).
                    CPP_AT_PRINTF("OK\r\n");
                    settings_manager.Apply();
                    // Persist. ResetToDefaults() only rewrites the RAM struct, so without this a RESET
                    // is undone by the next reboot -- which made it useless as the documented recovery
                    // from a bad persisted blob, the exact case it exists for. The OK is already on the
                    // wire by now, so a failure can only be logged, not returned.
                    if (!settings_manager.Save()) {
                        CONSOLE_ERROR("CommsManager::ATSettingsCallback",
                                      "Defaults applied but could not be written to flash; they will not "
                                      "survive a reboot.");
                    }
                    CPP_AT_SILENT_SUCCESS();
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

CPP_AT_CALLBACK(CommsManager::ATSubGRxModeCallback) {
    switch (op) {
        case '?':
            CPP_AT_CMD_PRINTF("=%s", SettingsManager::kSubGHzModeStrs[subg_radio.GetMode()]);
            CPP_AT_SILENT_SUCCESS();
            break;
        case '=':
            if (!CPP_AT_HAS_ARG(0)) {
                CPP_AT_ERROR("Need to specify a Sub-GHz RX mode.");
            }
            for (uint16_t i = 0; i < SettingsManager::kNumSubGHzRadioModes; i++) {
                if (args[0].compare(SettingsManager::kSubGHzModeStrs[i]) == 0) {
                    SettingsManager::SubGHzRadioMode mode = static_cast<SettingsManager::SubGHzRadioMode>(i);
                    // Apply to the radio first; only mirror into the settings struct once it took, so a failed
                    // restart can't leave RAM settings (and a later AT+SETTINGS=SAVE) disagreeing with the radio.
                    if (!subg_radio.SetMode(mode)) {
                        CPP_AT_ERROR("Failed to restart Sub-GHz RX in mode %s.", args[0].data());
                    }
                    settings_manager.settings.subg_mode = mode;
                    CPP_AT_SUCCESS();
                }
            }
            CPP_AT_ERROR("Sub-GHz RX mode %s not recognized.", args[0].data());
            break;
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
                    // Block for watdog timeout + 1 seconds to try triggering a watchdog event.
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

#ifdef HARDWARE_UNIT_TESTS
// Wide, datasheet-derived per-band frequency bounds (MHz), shared by TX_CW and RX_CW. The board
// baluns are tuned for the operating frequencies (978 / 1090 / 2400), so off-target carriers
// radiate (and receive) weakly.
static constexpr uint16_t kSubGMinMHz = 718;
static constexpr uint16_t kSubGMaxMHz = 1054;
static constexpr uint16_t kLrLfMinMHz = 150;
static constexpr uint16_t kLrLfMaxMHz = 1100;
static constexpr uint16_t kLrHfMinMHz = 1900;
static constexpr uint16_t kLrHfMaxMHz = 2700;

CPP_AT_CALLBACK(CommsManager::ATTxCWCallback) {
    // LR2021 TX power bounds, in whole dBm (DS.LR2021: LF -9.5..22 dBm, HF -19.5..12 dBm).
    static constexpr int32_t kLrLfMinPowerDbm = -9;
    static constexpr int32_t kLrLfMaxPowerDbm = 22;
    static constexpr int32_t kLrHfMinPowerDbm = -19;
    static constexpr int32_t kLrHfMaxPowerDbm = 12;
    static constexpr int8_t kDefaultPowerDbm = 0;

    if (op != '=') {
        CPP_AT_ERROR("Operator '%c' not supported.", op);
    }
    if (!(CPP_AT_HAS_ARG(0) && CPP_AT_HAS_ARG(1))) {
        CPP_AT_ERROR(
            "Requires at least two arguments: AT+TX_CW=<band [SUBG LRLF LRHF]>,<freq_MHz>[,<power_dBm> (LRLF/LRHF "
            "only)].");
    }
    uint16_t freq_mhz = 0;
    CPP_AT_TRY_ARG2NUM(1, freq_mhz);

    // Optional third argument: TX power in dBm (LR2021 bands only). Range is band-specific and validated
    // in the per-band branches below. Defaults to 0 dBm.
    int32_t power_arg = kDefaultPowerDbm;
    bool power_specified = CPP_AT_HAS_ARG(2);
    if (power_specified) {
        CPP_AT_TRY_ARG2NUM(2, power_arg);
    }
    int8_t power_dbm = (int8_t)power_arg;

    // LRLF/LRHF key up the LR2021 over SPI; refuse while the interface is released (AT+LR_ENABLE=0).
    if ((args[0].compare("LRLF") == 0 || args[0].compare("LRHF") == 0) && !adsbee.LR2021IsEnabled()) {
        CPP_AT_ERROR("LR2021 interface disabled (AT+LR_ENABLE=0); LRLF/LRHF unavailable.");
    }

    // Dispatch on band, validating the frequency before keying up the radio.
    enum CwBand { kBandSubG, kBandLrLf, kBandLrHf } band;
    if (args[0].compare("SUBG") == 0) {
        if (freq_mhz < kSubGMinMHz || freq_mhz > kSubGMaxMHz) {
            CPP_AT_ERROR("Frequency %u MHz out of range for SUBG (%u-%u MHz).", freq_mhz, kSubGMinMHz, kSubGMaxMHz);
        }
        // SUBG (CC1314) CW power comes from the SmartRF TX power table, not the dBm argument.
        if (power_specified) {
            CPP_AT_ERROR("Power argument not supported for SUBG; TX power is fixed at +12 dBm by the SmartRF TX power table.");
        }
        band = kBandSubG;
        if (!subg_radio.StartCWTest(freq_mhz)) {
            CPP_AT_ERROR("Failed to start CW on SUBG.");
        }
    } else if (args[0].compare("LRLF") == 0) {
        if (freq_mhz < kLrLfMinMHz || freq_mhz > kLrLfMaxMHz) {
            CPP_AT_ERROR("Frequency %u MHz out of range for LRLF (%u-%u MHz).", freq_mhz, kLrLfMinMHz, kLrLfMaxMHz);
        }
        if (power_arg < kLrLfMinPowerDbm || power_arg > kLrLfMaxPowerDbm) {
            CPP_AT_ERROR("Power %ld dBm out of range for LRLF (%ld to %ld dBm).", (long)power_arg,
                         (long)kLrLfMinPowerDbm, (long)kLrLfMaxPowerDbm);
        }
        band = kBandLrLf;
        if (!adsbee.lr2021.StartCwTone(/*use_hf_path=*/false, (uint32_t)freq_mhz * 1000000u, power_dbm)) {
            CPP_AT_ERROR("Failed to start CW on LRLF.");
        }
    } else if (args[0].compare("LRHF") == 0) {
        if (freq_mhz < kLrHfMinMHz || freq_mhz > kLrHfMaxMHz) {
            CPP_AT_ERROR("Frequency %u MHz out of range for LRHF (%u-%u MHz).", freq_mhz, kLrHfMinMHz, kLrHfMaxMHz);
        }
        if (power_arg < kLrHfMinPowerDbm || power_arg > kLrHfMaxPowerDbm) {
            CPP_AT_ERROR("Power %ld dBm out of range for LRHF (%ld to %ld dBm).", (long)power_arg,
                         (long)kLrHfMinPowerDbm, (long)kLrHfMaxPowerDbm);
        }
        band = kBandLrHf;
        if (!adsbee.lr2021.StartCwTone(/*use_hf_path=*/true, (uint32_t)freq_mhz * 1000000u, power_dbm)) {
            CPP_AT_ERROR("Failed to start CW on LRHF.");
        }
    } else {
        CPP_AT_ERROR("Unknown band '%s' (SUBG, LRLF, LRHF).", args[0].data());
    }

    if (band == kBandSubG) {
        CPP_AT_PRINTF("Transmitting CW on %s at %u MHz. Press any key to stop.\r\n", args[0].data(), freq_mhz);
    } else {
        CPP_AT_PRINTF("Transmitting CW on %s at %u MHz, %d dBm. Press any key to stop.\r\n", args[0].data(), freq_mhz,
                      power_dbm);
    }

    // Block until any key is received, petting the watchdog so we don't reset during transmission.
    char c;
    while (!iface_getc(SettingsManager::SerialInterface::kConsole, c)) {
        adsbee.FeedWatchdog();
    }

    // Stop the carrier and restore normal reception.
    bool restore_ok;
    if (band == kBandSubG) {
        restore_ok = subg_radio.StopCWTest();
    } else {
        restore_ok = adsbee.lr2021.StopCwTone();
        restore_ok &= adsbee.Init();  // Re-apply the 1090 MHz receiver configuration.
    }

    // A keypress like Enter sends "\r\n" but only the first byte broke the loop above. Drain any
    // remaining buffered input (e.g. the trailing '\n') so it isn't parsed as a stray AT command.
    sleep_ms_blocking(10);  // Allow the rest of the key sequence to arrive.
    char drain_c;
    while (iface_getc(SettingsManager::SerialInterface::kConsole, drain_c)) {
    }

    // Report a failure to stop the carrier / restore reception instead of printing success, since the
    // device may be left in an unexpected RF state (e.g. carrier still up or receiver not restarted).
    if (!restore_ok) {
        CPP_AT_ERROR("Failed to stop CW carrier and restore normal reception.");
    }

    CPP_AT_SUCCESS();
}

// Formats an RSSI value in half-dB units (dBm * 2) as e.g. "-93.5" into buf. Both radios report in
// half-dB resolution or better, so this avoids float printf.
static const char* RssiHalfDbToStr(int16_t rssi_x2, char* buf, size_t buf_len) {
    uint16_t v = (uint16_t)(rssi_x2 < 0 ? -rssi_x2 : rssi_x2);
    snprintf(buf, buf_len, "%s%u.%u", rssi_x2 < 0 ? "-" : "", v / 2, (v % 2) * 5);
    return buf;
}

CPP_AT_CALLBACK(CommsManager::ATRxCWCallback) {
    if (op != '=') {
        CPP_AT_ERROR("Operator '%c' not supported.", op);
    }
    if (!(CPP_AT_HAS_ARG(0) && CPP_AT_HAS_ARG(1))) {
        CPP_AT_ERROR("Requires two arguments: AT+RX_CW=<band [SUBG LRLF LRHF]>,<freq_MHz>.");
    }
    uint16_t freq_mhz = 0;
    CPP_AT_TRY_ARG2NUM(1, freq_mhz);

    // LRLF/LRHF retune the LR2021 over SPI; refuse while the interface is released (AT+LR_ENABLE=0).
    if ((args[0].compare("LRLF") == 0 || args[0].compare("LRHF") == 0) && !adsbee.LR2021IsEnabled()) {
        CPP_AT_ERROR("LR2021 interface disabled (AT+LR_ENABLE=0); LRLF/LRHF unavailable.");
    }

    // Dispatch on band, validating the frequency before retuning the receiver.
    enum RxBand { kBandSubG, kBandLrLf, kBandLrHf } band;
    if (args[0].compare("SUBG") == 0) {
        if (freq_mhz < kSubGMinMHz || freq_mhz > kSubGMaxMHz) {
            CPP_AT_ERROR("Frequency %u MHz out of range for SUBG (%u-%u MHz).", freq_mhz, kSubGMinMHz, kSubGMaxMHz);
        }
        band = kBandSubG;
        if (!subg_radio.StartRssiScan(freq_mhz)) {
            CPP_AT_ERROR("Failed to start RSSI scan on SUBG.");
        }
    } else if (args[0].compare("LRLF") == 0) {
        if (freq_mhz < kLrLfMinMHz || freq_mhz > kLrLfMaxMHz) {
            CPP_AT_ERROR("Frequency %u MHz out of range for LRLF (%u-%u MHz).", freq_mhz, kLrLfMinMHz, kLrLfMaxMHz);
        }
        band = kBandLrLf;
        if (!adsbee.lr2021.StartRssiScan(/*use_hf_path=*/false, (uint32_t)freq_mhz * 1000000u)) {
            adsbee.Init();  // Don't leave the receiver half-configured.
            CPP_AT_ERROR("Failed to start RSSI scan on LRLF.");
        }
    } else if (args[0].compare("LRHF") == 0) {
        if (freq_mhz < kLrHfMinMHz || freq_mhz > kLrHfMaxMHz) {
            CPP_AT_ERROR("Frequency %u MHz out of range for LRHF (%u-%u MHz).", freq_mhz, kLrHfMinMHz, kLrHfMaxMHz);
        }
        band = kBandLrHf;
        if (!adsbee.lr2021.StartRssiScan(/*use_hf_path=*/true, (uint32_t)freq_mhz * 1000000u)) {
            adsbee.Init();  // Don't leave the receiver half-configured.
            CPP_AT_ERROR("Failed to start RSSI scan on LRHF.");
        }
    } else {
        CPP_AT_ERROR("Unknown band '%s' (SUBG, LRLF, LRHF).", args[0].data());
    }

    CPP_AT_PRINTF("Measuring RSSI on %s at %u MHz. Press any key to stop.\r\n", args[0].data(), freq_mhz);

    // Sample RSSI until any key is received, tracking in half-dB units (dBm * 2) so both radios
    // share one code path. Pet the watchdog so we don't reset while blocking the main loop.
    int16_t max_rssi_x2 = INT16_MIN;
    int16_t cur_rssi_x2 = INT16_MIN;
    bool have_sample = false;
    bool last_sample_valid = false;
    static constexpr uint32_t kRssiPrintIntervalMs = 1 * kMsPerSec;
    uint32_t last_print_timestamp_ms = get_time_since_boot_ms();
    char c;
    char cur_str[8], max_str[8];
    while (!iface_getc(SettingsManager::SerialInterface::kConsole, c)) {
        adsbee.FeedWatchdog();

        bool sample_valid = false;
        if (band == kBandSubG) {
            int8_t rssi_dbm = subg_radio.ReadRssiDbm();
            if (rssi_dbm != INT8_MIN) {  // INT8_MIN = RF core not actively receiving.
                cur_rssi_x2 = (int16_t)rssi_dbm * 2;
                sample_valid = true;
            }
        } else {
            LR2021::RssiInstRsp rsp;
            if (adsbee.lr2021.GetRssiInst(&rsp)) {
                cur_rssi_x2 = -(int16_t)rsp.rssi;  // 9-bit raw value; power (dBm) = -rssi / 2.
                sample_valid = true;
            }
        }
        last_sample_valid = sample_valid;
        if (sample_valid) {
            have_sample = true;
            if (cur_rssi_x2 > max_rssi_x2) {
                max_rssi_x2 = cur_rssi_x2;
            }
        }

        if (get_time_since_boot_ms() - last_print_timestamp_ms >= kRssiPrintIntervalMs) {
            if (have_sample) {
                // Show "n/a" rather than the last good value when the most recent read failed, so a receiver
                // that stopped reporting is visible instead of appearing frozen at its last reading.
                CPP_AT_PRINTF("RSSI: cur %s dBm, max %s dBm\r\n",
                              last_sample_valid ? RssiHalfDbToStr(cur_rssi_x2, cur_str, sizeof(cur_str)) : "n/a",
                              RssiHalfDbToStr(max_rssi_x2, max_str, sizeof(max_str)));
            } else {
                CPP_AT_PRINTF("RSSI: no valid samples yet.\r\n");
            }
            last_print_timestamp_ms = get_time_since_boot_ms();
        }
    }

    // Restore normal reception: SUBG retunes and restarts packet RX; the LR2021 paths are fully
    // reconfigured for 1090 MHz OOK reception by adsbee.Init() (mirrors the TX_CW restore).
    bool restore_ok = (band == kBandSubG) ? subg_radio.StopRssiScan() : adsbee.Init();

    // A keypress like Enter sends "\r\n" but only the first byte broke the loop above. Drain any
    // remaining buffered input (e.g. the trailing '\n') so it isn't parsed as a stray AT command.
    sleep_ms_blocking(10);  // Allow the rest of the key sequence to arrive.
    char drain_c;
    while (iface_getc(SettingsManager::SerialInterface::kConsole, drain_c)) {
    }

    if (!restore_ok) {
        CPP_AT_ERROR("Failed to restore normal reception.");
    }

    if (band == kBandSubG && subg_radio.rssi_scan_rx_restart_count > 0) {
        // The RX command ended (false sync / RX buffer error, typically under a strong carrier) and had to be
        // re-armed to keep RSSI live. Report it for bench characterization.
        CPP_AT_PRINTF("Rx restarts during scan: %lu\r\n", (unsigned long)subg_radio.rssi_scan_rx_restart_count);
    }

    if (have_sample) {
        CPP_AT_PRINTF("Max RSSI: %s dBm\r\n", RssiHalfDbToStr(max_rssi_x2, max_str, sizeof(max_str)));
    } else {
        CPP_AT_PRINTF("No valid RSSI samples recorded.\r\n");
    }
    CPP_AT_SUCCESS();
}
#endif

const CppAT::ATCommandDef_t at_command_list[] = {
    {.command = "BAUD_RATE",
     .min_args = 0,
     .max_args = 2,
     .help_string = "AT+BAUD_RATE=CONSOLE,<baud [115200 230400 460800 921600 1000000]>\r\n\tChange the console baud "
                    "rate. Prints OK at the old baud rate, then switches; reopen the host port at the new "
                    "rate. Takes effect immediately but persists across reboots only after AT+SETTINGS=SAVE. "
                    "Boot default is 1000000; AT+SETTINGS=RESET restores it.\r\n\t"
                    "AT+BAUD_RATE?\r\n\tQuery the current console baud rate.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATBaudRateCallback, comms_manager)},
    {.command = "BOOT_UART_BOOTLOADER",
     .min_args = 1,
     .max_args = 1,
     .help_string = "AT+BOOT_UART_BOOTLOADER=1DEADBEE\r\n\tErase the app image and enter the ROM UART "
                    "bootloader (DIO2/DIO3) for reflashing.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATBootUARTBootloaderCallback, comms_manager)},
    {.command = "DEVICE_INFO",
     .min_args = 0,
     .max_args = 5,  // TODO: check this value.
     .help_string = "AT+DEVICE_INFO?\r\n\tQuery device information.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATDeviceInfoCallback, comms_manager)},
    {.command = "LOG_LEVEL",
     .min_args = 0,
     .max_args = 1,
     .help_string =
         "AT+LOG_LEVEL=<log_level [SILENT ERRORS WARNINGS INFO]>\r\n\tSet how much stuff gets printed to the "
         "console.\r\n\t",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATLogLevelCallback, comms_manager)},
    {.command = "LR_ENABLE",
     .min_args = 0,
     .max_args = 1,
     .help_string =
         "AT+LR_ENABLE=<enabled [1,0]>\r\n\t1 (default): the CC1314 drives the LR2021 1090MHz radio "
         "(reception per AT+RX_ENABLE).\r\n\t0: release the LR2021 bus to an external host: the radio is "
         "held in reset and the CC1314-side pins are parked hi-Z (NSS pulled up; RESET/SCLK/PICO pulled "
         "down); 1090MHz reception stops. RX_ENABLE / R1090_* settings are remembered and take effect on "
         "re-enable. Takes effect immediately; persist with AT+SETTINGS=SAVE.\r\n\t"
         "AT+LR_ENABLE?\r\n\tLR_ENABLE=<enabled>\r\n\tQuery whether the LR2021 interface is enabled.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATLREnableCallback, comms_manager)},
    {.command = "MAVLINK_ID",
     .min_args = 0,
     .max_args = 2,
     .help_string = "AT+MAVLINK_ID=<system_id>,<component_id>\r\n\tSet the MAVLink system and component IDs.\r\n\t"
                    "AT+MAVLINK_ID?\r\n\tQuery the MAVLink system and component IDs.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATMAVLINKIDCallback, comms_manager)},
    {.command = "PROTOCOL_OUT",
     .min_args = 0,
     .max_args = 2,
     .help_callback = CPP_AT_BIND_MEMBER_HELP_CALLBACK(CommsManager::ATProtocolOutHelpCallback, comms_manager),
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATProtocolOutCallback, comms_manager)},
    {.command = "REBOOT",
     .min_args = 0,
     .max_args = 0,
     .help_string = "REBOOT\r\n\tReboots the device.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATRebootCallback, comms_manager)},
    {.command = "R1090_GAIN",
     .min_args = 0,
     .max_args = 1,
     .help_string = "AT+R1090_GAIN=<gain_step [0=auto, 1-15, 13=max; default AUTO]>\r\n\tSet the 1090MHz LF-frontend AGC "
                    "gain.\r\n\tAT+R1090_GAIN?\r\n\tQuery the current gain.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATR1090GainCallback, comms_manager)},
    {.command = "R1090_PREAMBLE",
     .min_args = 0,
     .max_args = 1,
     .help_string = "AT+R1090_PREAMBLE=<mode [MODE_S_PREAMBLE DF17 MODE_S_SW_CRC]>\r\n\tSet the 1090MHz Mode S "
                    "receiver preamble mode.\r\n\t"
                    "AT+R1090_PREAMBLE?\r\n\tQuery the current preamble mode.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATR1090PreambleCallback, comms_manager)},
    {.command = "R1090_RX_BOOST",
     .min_args = 0,
     .max_args = 1,
     .help_string = "AT+R1090_RX_BOOST=<boost [0=off, 1-7, 7=max]>\r\n\tSet the 1090MHz LF RX path boost "
                    "level.\r\n\tAT+R1090_RX_BOOST?\r\n\tQuery the current boost level.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATR1090RxBoostCallback, comms_manager)},
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
    {.command = "RX_STATS",
     .min_args = 0,
     .max_args = 1,
     .help_string = "AT+RX_STATS?\r\n\tQuery LR2021 OOK Rx stats (pkt_rx, crc_error, len_error, pbl_det, sync_ok, "
                    "sync_fail, timeout) plus health counters (fifo_full, q_ovf, bitflips_fixed, "
                    "uat_rx_restarts, uat_setlen_errs = failed CMD_PROP_SET_LEN commands, "
                    "max_loop_us = longest main loop iteration, max_lr2021_us = longest single "
                    "LR2021 update call, drain_max_us = longest complete async RX FIFO drain, "
                    "irq_* = IRQ-paced FIFO drain chain diagnostics (chains started / edges skipped busy / edges "
                    "skipped no-slot / back-to-back restarts / wedge timeouts / chain errors / thread-assisted "
                    "frame posts), tx_stalls / tx_drops / tx_ring_hw = "
                    "console TX ring waits, dropped writes and peak occupancy in bytes).\r\n\tAT+RX_STATS=RESET\r\n\t"
                    "Reset all Rx stats counters.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATRxStatsCallback, comms_manager)},
    {.command = "SETTINGS",
     .min_args = 0,
     .max_args = 3,
     .help_string = "Load, save, or reset nonvolatile settings. RESET restores defaults and writes them to "
                    "flash.\r\n\tAT+SETTINGS=<op [LOAD SAVE RESET]>\r\n\t"
                    "Display nonvolatile settings.\r\n\tAT+SETTINGS?\r\n\t+SETTINGS=...\r\n\tDump settings in AT "
                    "command format.\r\n\tAT+SETTINGS?DUMP\r\n\t+SETTINGS=...\r\n\tDump settings as a "
                    "single-line JSON object keyed by AT command.\r\n\tAT+SETTINGS?JSON\r\n\tSETTINGS={...}",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATSettingsCallback, comms_manager)},
    {.command = "SUBG_MODE",
     .min_args = 0,
     .max_args = 1,
     .help_string = "AT+SUBG_MODE=<mode [UAT_RX UAT_RX_NO_UPLINK]>\r\n\tSet the Sub-GHz receiver protocol mode. "
                    "UAT_RX receives 978MHz UAT ADS-B and ground uplink messages; UAT_RX_NO_UPLINK receives ADS-B "
                    "only (uplink sync word disabled at the RF core). Takes effect immediately; persist with "
                    "AT+SETTINGS=SAVE.\r\n\tAT+SUBG_MODE?\r\n\tQuery the current mode.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATSubGRxModeCallback, comms_manager)},
#ifdef HARDWARE_UNIT_TESTS
    {.command = "TEST",
     .min_args = 0,
     .max_args = 1,
     .help_string = "Run hardware self-tests.",
     .callback = ATTestCallback},
    {.command = "TX_CW",
     .min_args = 2,
     .max_args = 3,
     .help_string = "AT+TX_CW=<band [SUBG LRLF LRHF]>,<freq_MHz>[,<power_dBm>]\r\n\tTransmit an unmodulated CW "
                    "carrier until a key is pressed.\r\n\tSUBG=CC1314, LRLF/LRHF=LR2021 low/high band. Power "
                    "(default 0 dBm) applies to LR2021 bands only: LRLF -9..22 dBm, LRHF -19..12 dBm.\r\n\tSUBG "
                    "power is fixed at +12 dBm by the SmartRF TX power table; a power argument is an error.",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATTxCWCallback, comms_manager)},
    {.command = "RX_CW",
     .min_args = 2,
     .max_args = 2,
     .help_string = "AT+RX_CW=<band [SUBG LRLF LRHF]>,<freq_MHz>\r\n\tTune the selected receiver and continuously "
                    "measure instantaneous RSSI, printing current/max about once per second. Press any key to stop; "
                    "prints the max RSSI in dBm and restores normal reception.\r\n\tSUBG=CC1314 (718-1054 MHz), "
                    "LRLF/LRHF=LR2021 low/high band (150-1100 / 1900-2700 MHz).",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATRxCWCallback, comms_manager)},
#endif
    {.command = "UPTIME",
     .min_args = 0,
     .max_args = 0,
     .help_string = "AT+UPTIME?\r\n\tQuery the time since boot in seconds.\r\n\tUPTIME=<uptime_sec>",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATUptimeCallback, comms_manager)},
    {.command = "WATCHDOG",
     .min_args = 0,
     .max_args = 1,
     .help_string = "Set the watchdog timeout, in seconds, 0-65535. 0 = watchdog disabled, 65535 = "
                    "18.2hrs.\r\n\tAT+WATCHDOG=<timeout_sec>\r\n\tTest watchdog by blocking for timeout_sec+1 "
                    "seconds.\r\n\tAT+WATCHDOG=TEST",
     .callback = CPP_AT_BIND_MEMBER_CALLBACK(CommsManager::ATWatchdogCallback, comms_manager)},
};
const uint16_t at_command_list_num_commands = sizeof(at_command_list) / sizeof(at_command_list[0]);

bool CommsManager::UpdateAT() {
    static char stdio_at_command_buf[kATCommandBufMaxLen + 1];
    static uint16_t stdio_at_command_buf_len = 0;
    // Check for new AT commands from STDIO. Process up to one line per loop.
    char c;
    bool got_a_symbol = iface_getc(SettingsManager::SerialInterface::kConsole, c);
    while (got_a_symbol) {
        stdio_at_command_buf[stdio_at_command_buf_len] = c;
        stdio_at_command_buf_len++;
        stdio_at_command_buf[stdio_at_command_buf_len] = '\0';
        if (stdio_at_command_buf_len >= kATCommandBufMaxLen) {
            CONSOLE_ERROR("CommsManager::UpdateAT", "AT command buffer overflow.");
            stdio_at_command_buf_len = 0;
            stdio_at_command_buf[stdio_at_command_buf_len] = '\0';  // clear command buffer
        }
        if (c == '\n') {
            // Ignore blank lines (bare Enter / stray CR-LF) instead of handing them to cppAT, which would print an
            // "Unable to find AT prefix" error. Terminals (and the web console) send a bare newline as the
            // "press any key" keystroke that stops AT+RX_CW / AT+TX_CW, and it should otherwise be silent.
            if (strspn(stdio_at_command_buf, " \t\r\n") != stdio_at_command_buf_len) {
                at_parser_.ParseMessage(std::string_view(stdio_at_command_buf));
            }
            stdio_at_command_buf_len = 0;
            stdio_at_command_buf[stdio_at_command_buf_len] = '\0';  // clear command buffer
        }
        got_a_symbol = iface_getc(SettingsManager::SerialInterface::kConsole, c);
    }

    return true;
}