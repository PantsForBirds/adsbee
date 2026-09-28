#include "settings.hh"

#ifdef ON_PICO
#include "adsbee.hh"
#endif
#include "comms.hh"

// A bool loaded from a raw settings blob can hold any byte value, and reading one that isn't 0 or 1 is undefined
// behavior. Inspect the storage byte instead of the bool, and rewrite any nonzero value as true.
static bool SanitizeBool(bool& value, const char* name, int index = -1) {
    uint8_t raw;
    memcpy(&raw, &value, sizeof(raw));
    if (raw <= 1) {
        return false;
    }
    if (index >= 0) {
        CONSOLE_ERROR("SettingsManager::Sanitize", "%s[%d] has non-boolean value %u, resetting to true.", name, index,
                      raw);
    } else {
        CONSOLE_ERROR("SettingsManager::Sanitize", "%s has non-boolean value %u, resetting to true.", name, raw);
    }
    const uint8_t one = 1;
    memcpy(&value, &one, sizeof(one));
    return true;
}

// Forces NUL termination of a fixed-size char array, so %s formatting and strlen() stay inside the field.
static bool SanitizeString(char* str, size_t size, const char* name, int index = -1) {
    if (memchr(str, '\0', size) != nullptr) {
        return false;
    }
    if (index >= 0) {
        CONSOLE_ERROR("SettingsManager::Sanitize", "%s[%d] is not NUL-terminated, truncating.", name, index);
    } else {
        CONSOLE_ERROR("SettingsManager::Sanitize", "%s is not NUL-terminated, truncating.", name);
    }
    str[size - 1] = '\0';
    return true;
}

bool SettingsManager::Sanitize() {
    bool changed = false;

    // CoreNetworkSettings is CRC-protected. If the CRC was valid before sanitizing, keep it valid after any fix-up so
    // the block can still be restored after a failed migration.
    Settings::CoreNetworkSettings& cns = settings.core_network_settings;
    bool cns_was_valid = cns.IsValid();
    bool cns_changed = false;
    cns_changed |= SanitizeBool(cns.esp32_enabled, "esp32_enabled");
    cns_changed |= SanitizeBool(cns.wifi_ap_enabled, "wifi_ap_enabled");
    cns_changed |= SanitizeBool(cns.wifi_sta_enabled, "wifi_sta_enabled");
    cns_changed |= SanitizeBool(cns.ethernet_enabled, "ethernet_enabled");
    cns_changed |= SanitizeString(cns.hostname, sizeof(cns.hostname), "hostname");
    cns_changed |= SanitizeString(cns.wifi_ap_ssid, sizeof(cns.wifi_ap_ssid), "wifi_ap_ssid");
    cns_changed |= SanitizeString(cns.wifi_ap_password, sizeof(cns.wifi_ap_password), "wifi_ap_password");
    cns_changed |= SanitizeString(cns.wifi_sta_ssid, sizeof(cns.wifi_sta_ssid), "wifi_sta_ssid");
    cns_changed |= SanitizeString(cns.wifi_sta_password, sizeof(cns.wifi_sta_password), "wifi_sta_password");
    if (cns_changed) {
        if (cns_was_valid) {
            cns.UpdateCRC32();
        }
        changed = true;
    }

    changed |= SanitizeBool(settings.r1090_rx_enabled, "r1090_rx_enabled");
    changed |= SanitizeBool(settings.r1090_bias_tee_enabled, "r1090_bias_tee_enabled");
    changed |= SanitizeBool(settings.led_enabled, "led_enabled");
    changed |= SanitizeBool(settings.feeds_enabled, "feeds_enabled");
    changed |= SanitizeBool(settings.gnss_enabled, "gnss_enabled");
    changed |= SanitizeBool(settings.gnss_notify, "gnss_notify");
    changed |= SanitizeBool(settings.subg_rx_enabled, "subg_rx_enabled");
    changed |= SanitizeBool(settings.subg_bias_tee_enabled, "subg_bias_tee_enabled");
    changed |= SanitizeBool(settings.remote_id_rx_enabled, "remote_id_rx_enabled");
    changed |= SanitizeBool(settings.remote_id_tx_enabled, "remote_id_tx_enabled");
    changed |= SanitizeString(settings.remote_id_tx_uas_id, sizeof(settings.remote_id_tx_uas_id), "remote_id_tx_uas_id");
    changed |= SanitizeString(settings.remote_id_tx_operator_id, sizeof(settings.remote_id_tx_operator_id),
                              "remote_id_tx_operator_id");
    for (uint16_t i = 0; i < Settings::kMaxNumFeeds; i++) {
        changed |= SanitizeBool(settings.feed_is_active[i], "feed_is_active", i);
        changed |= SanitizeString(settings.feed_uris[i], sizeof(settings.feed_uris[i]), "feed_uris", i);
    }

    // A baud rate of 0 is invalid for the hardware UARTs (the console is a USB CDC port, where 0 is the normal value).
    if (settings.baud_rates[SerialInterface::kCommsUART] == 0) {
        CONSOLE_ERROR("SettingsManager::Sanitize", "Comms UART baud rate is 0, resetting to %lu.",
                      (unsigned long)Settings::kDefaultCommsUARTBaudrate);
        settings.baud_rates[SerialInterface::kCommsUART] = Settings::kDefaultCommsUARTBaudrate;
        changed = true;
    }
    if (settings.baud_rates[SerialInterface::kGNSSUART] == 0) {
        CONSOLE_ERROR("SettingsManager::Sanitize", "GNSS UART baud rate is 0, resetting to %lu.",
                      (unsigned long)Settings::kDefaultGNSSUARTBaudrate);
        settings.baud_rates[SerialInterface::kGNSSUART] = Settings::kDefaultGNSSUARTBaudrate;
        changed = true;
    }

    if (settings.log_level >= LogLevel::kNumLogLevels) {
        CONSOLE_ERROR("SettingsManager::Sanitize", "log_level %u out of range, resetting to kWarnings.",
                      settings.log_level);
        settings.log_level = LogLevel::kWarnings;
        changed = true;
    }

    for (uint16_t i = 0; i < SerialInterface::kNumSerialInterfaces; i++) {
        if (settings.reporting_protocols[i] >= ReportingProtocol::kNumProtocols) {
            CONSOLE_ERROR("SettingsManager::Sanitize", "reporting_protocols[%u]=%u out of range, resetting to kNoReports.",
                          i, settings.reporting_protocols[i]);
            settings.reporting_protocols[i] = ReportingProtocol::kNoReports;
            changed = true;
        }
    }
    for (uint16_t i = 0; i < Settings::kMaxNumFeeds; i++) {
        if (settings.feed_protocols[i] >= ReportingProtocol::kNumProtocols) {
            CONSOLE_ERROR("SettingsManager::Sanitize", "feed_protocols[%u]=%u out of range, resetting to kNoReports.",
                          i, settings.feed_protocols[i]);
            settings.feed_protocols[i] = ReportingProtocol::kNoReports;
            changed = true;
        }
    }

    if (settings.subg_mode >= kNumSubGHzRadioModes) {
        CONSOLE_ERROR("SettingsManager::Sanitize", "subg_mode %u out of range, resetting to kSubGHzRadioModeUATRx.",
                      settings.subg_mode);
        settings.subg_mode = SubGHzRadioMode::kSubGHzRadioModeUATRx;
        changed = true;
    }
    if (settings.subg_enabled != EnableState::kEnableStateExternal &&
        settings.subg_enabled != EnableState::kEnableStateDisabled &&
        settings.subg_enabled != EnableState::kEnableStateEnabled) {
        CONSOLE_ERROR("SettingsManager::Sanitize", "subg_enabled %d out of range, resetting to kEnableStateEnabled.",
                      settings.subg_enabled);
        settings.subg_enabled = EnableState::kEnableStateEnabled;
        changed = true;
    }

    if (settings.gnss_receiver_type != kGNSSReceiverNone && settings.gnss_receiver_type != kGNSSReceiverGeneric &&
        settings.gnss_receiver_type != kGNSSReceiverUBXMIA) {
        CONSOLE_ERROR("SettingsManager::Sanitize", "gnss_receiver_type %u out of range, resetting to kGNSSReceiverNone.",
                      settings.gnss_receiver_type);
        settings.gnss_receiver_type = kGNSSReceiverNone;
        changed = true;
    }
    // GNSS can't run without a receiver type; AT+GNSS refuses to enable it that way, so treat the combination as disabled.
    if (settings.gnss_enabled && settings.gnss_receiver_type == kGNSSReceiverNone) {
        CONSOLE_ERROR("SettingsManager::Sanitize", "gnss_enabled is set with receiver type NONE, disabling GNSS.");
        settings.gnss_enabled = false;
        changed = true;
    }

    if (settings.rx_position.source >= RxPosition::kNumPositionSources) {
        CONSOLE_ERROR("SettingsManager::Sanitize", "rx_position.source %u out of range, resetting to kPositionSourceLowestAircraft.",
                      settings.rx_position.source);
        settings.rx_position.source = RxPosition::kPositionSourceLowestAircraft;
        changed = true;
    }

    return changed;
}

// NOTE: This function needs to be updated separately for ESP32.
void SettingsManager::Print() {
    char print_buf[500];
    uint16_t print_buf_len = 0;

    print_buf_len += snprintf(print_buf + print_buf_len, sizeof(print_buf) - print_buf_len, "Settings Struct\r\n");
    print_buf_len += snprintf(print_buf + print_buf_len, sizeof(print_buf) - print_buf_len, "\t1090 Receiver: %s\r\n",
                              settings.r1090_rx_enabled ? "ENABLED" : "DISABLED");
    print_buf_len += snprintf(print_buf + print_buf_len, sizeof(print_buf) - print_buf_len,
                              "\tSub-GHz Receiver: %s\r\n", settings.subg_rx_enabled ? "ENABLED" : "DISABLED");
    print_buf_len += snprintf(print_buf + print_buf_len, sizeof(print_buf) - print_buf_len,
                              "\tRemote ID Receiver: %s (transports 0x%02X) [EXPERIMENTAL]\r\n",
                              settings.remote_id_rx_enabled ? "ENABLED" : "DISABLED", settings.remote_id_transports);
    print_buf_len += snprintf(print_buf + print_buf_len, sizeof(print_buf) - print_buf_len,
                              "\tRemote ID Transmitter: %s (transports 0x%02X) [EXPERIMENTAL]\r\n",
                              settings.remote_id_tx_enabled ? "ENABLED" : "DISABLED",
                              settings.remote_id_tx_transports);

    print_buf_len += snprintf(print_buf + print_buf_len, sizeof(print_buf) - print_buf_len,
                              "\tTrigger Offset Level: %d milliVolts\r\n", settings.tl_offset_mv);
    print_buf_len += snprintf(print_buf + print_buf_len, sizeof(print_buf) - print_buf_len,
                              "\tBias Tee: \r\n\t\t1090: %s\r\n\t\tSub-GHz: %s\r\n",
                              settings.r1090_bias_tee_enabled ? "ENABLED" : "DISABLED",
                              settings.subg_bias_tee_enabled ? "ENABLED" : "DISABLED");
    print_buf_len += snprintf(print_buf + print_buf_len, sizeof(print_buf) - print_buf_len,
                              "\tWatchdog Timeout: %lu seconds\r\n", settings.watchdog_timeout_sec);
    print_buf_len += snprintf(print_buf + print_buf_len, sizeof(print_buf) - print_buf_len, "\tHardware LEDs: %s\r\n",
                              settings.led_enabled ? "ENABLED" : "DISABLED");
    print_buf_len += snprintf(print_buf + print_buf_len, sizeof(print_buf) - print_buf_len, "\tNetwork Feeds: %s\r\n",
                              settings.feeds_enabled ? "ENABLED" : "DISABLED");
    print_buf_len += snprintf(print_buf + print_buf_len, sizeof(print_buf) - print_buf_len,
                              "\tGNSS: %s (%s), fix notifications: %s\r\n",
                              settings.gnss_enabled ? "ENABLED" : "DISABLED",
                              GNSSReceiverTypeToStr(settings.gnss_receiver_type),
                              settings.gnss_notify ? "ENABLED" : "DISABLED");
    print_buf_len += snprintf(print_buf + print_buf_len, sizeof(print_buf) - print_buf_len, "\tLog Level: %s\r\n",
                              kConsoleLogLevelStrs[settings.log_level]);
    CONSOLE_PRINTF("%s", print_buf);
    print_buf_len = 0;

#ifdef ON_PICO
    // Only print serial reporting protocols on master, where we aren't constrained by log message size.
    print_buf_len +=
        snprintf(print_buf + print_buf_len, sizeof(print_buf) - print_buf_len, "\tReporting Protocols:\r\n");
    CONSOLE_PRINTF("%s", print_buf);
    print_buf_len = 0;
    for (uint16_t i = 0; i < SerialInterface::kGNSSUART; i++) {
        // Only report protocols for CONSOLE and COMMS_UART.
        print_buf_len += snprintf(print_buf + print_buf_len, sizeof(print_buf) - print_buf_len, "\t\t%s: %s\r\n",
                                  kSerialInterfaceStrs[i], kReportingProtocolStrs[settings.reporting_protocols[i]]);
    }
    print_buf_len +=
        snprintf(print_buf + print_buf_len, sizeof(print_buf) - print_buf_len, "\tComms UART Baud Rate: %lu baud\r\n",
                 settings.baud_rates[SettingsManager::SerialInterface::kCommsUART]);
    print_buf_len +=
        snprintf(print_buf + print_buf_len, sizeof(print_buf) - print_buf_len, "\tGNSS UART Baud Rate: %lu baud\r\n",
                 settings.baud_rates[SettingsManager::SerialInterface::kGNSSUART]);
    CONSOLE_PRINTF("%s", print_buf);
    print_buf_len = 0;
#endif

    print_buf_len += snprintf(print_buf + print_buf_len, sizeof(print_buf) - print_buf_len, "\tESP32: %s\r\n",
                              settings.core_network_settings.esp32_enabled ? "ENABLED" : "DISABLED");
    print_buf_len += snprintf(
        print_buf + print_buf_len, sizeof(print_buf) - print_buf_len, "\tSub-GHz Radio: %s\r\n",
        settings.subg_enabled == SettingsManager::EnableState::kEnableStateEnabled
            ? "ENABLED"
            : (settings.subg_enabled == SettingsManager::EnableState::kEnableStateExternal ? "EXTERNAL" : "DISABLED"));
    CONSOLE_PRINTF("%s", print_buf);
    print_buf_len = 0;

#ifndef ON_TI
    // Print WiFi AP settings.
    print_buf_len += snprintf(print_buf + print_buf_len, sizeof(print_buf) - print_buf_len, "\tWiFi AP: %s\r\n",
                              settings.core_network_settings.wifi_ap_enabled ? "ENABLED" : "DISABLED");
    if (settings.core_network_settings.wifi_ap_enabled) {
        // Access Point settings. Don't censor password.
        print_buf_len += snprintf(print_buf + print_buf_len, sizeof(print_buf) - print_buf_len, "\t\tChannel: %d\r\n",
                                  settings.core_network_settings.wifi_ap_channel);
        print_buf_len += snprintf(print_buf + print_buf_len, sizeof(print_buf) - print_buf_len, "\t\tSSID: %s\r\n",
                                  settings.core_network_settings.wifi_ap_ssid);
        print_buf_len += snprintf(print_buf + print_buf_len, sizeof(print_buf) - print_buf_len, "\t\tPassword: %s\r\n",
                                  settings.core_network_settings.wifi_ap_password);
    }
    CONSOLE_PRINTF("%s", print_buf);
    print_buf_len = 0;

    // Print WiFi Station settings.
    print_buf_len += snprintf(print_buf + print_buf_len, sizeof(print_buf) - print_buf_len, "\tWiFi STA: %s\r\n",
                              settings.core_network_settings.wifi_sta_enabled ? "ENABLED" : "DISABLED");
    if (settings.core_network_settings.wifi_sta_enabled) {
        // Station settings. Censor password.
        print_buf_len += snprintf(print_buf + print_buf_len, sizeof(print_buf) - print_buf_len, "\t\tSSID: %s\r\n",
                                  settings.core_network_settings.wifi_sta_ssid);
        char redacted_wifi_sta_password[Settings::kWiFiPasswordMaxLen];
        RedactPassword(settings.core_network_settings.wifi_sta_password, redacted_wifi_sta_password,
                       strnlen(settings.core_network_settings.wifi_sta_password, Settings::kWiFiPasswordMaxLen));
        print_buf_len += snprintf(print_buf + print_buf_len, sizeof(print_buf) - print_buf_len, "\t\tPassword: %s\r\n",
                                  redacted_wifi_sta_password);
    }
    CONSOLE_PRINTF("%s", print_buf);
    print_buf_len = 0;
#endif

#ifdef ON_PICO
    // Only print feed stuff on the master, where we aren't constrained by log message size.
    // Print Ethernet settings.
    print_buf_len += snprintf(print_buf + print_buf_len, sizeof(print_buf) - print_buf_len, "\tEthernet: %s\r\n",
                              settings.core_network_settings.ethernet_enabled ? "ENABLED" : "DISABLED");

    // Print feed settings.
    CONSOLE_PRINTF("\tFeed URIs:\r\n");
    CONSOLE_PRINTF("%s", print_buf);
    print_buf_len = 0;
    for (uint16_t i = 0; i < Settings::kMaxNumFeeds; i++) {
        print_buf_len += snprintf(print_buf + print_buf_len, sizeof(print_buf) - print_buf_len,
                                  "\t\t%d URI:%s Port:%d %s Protocol:%s ID:0x", i, settings.feed_uris[i],
                                  settings.feed_ports[i], settings.feed_is_active[i] ? "ACTIVE" : "INACTIVE",
                                  kReportingProtocolStrs[settings.feed_protocols[i]]);
        for (int16_t feeder_id_byte_index = 0; feeder_id_byte_index < Settings::kFeedReceiverIDNumBytes;
             feeder_id_byte_index++) {
            print_buf_len += snprintf(print_buf + print_buf_len, sizeof(print_buf) - print_buf_len, "%02x",
                                      settings.feed_receiver_ids[i][feeder_id_byte_index]);
        }
        CONSOLE_PRINTF("%s\r\n", print_buf);
        print_buf_len = 0;
    }
#endif
}

void SettingsManager::PrintAT() {
    // AT+BAUD_RATE
    // Note: Baud rate cannot be changed for CONSOLE since it is a virtual COM port. Don't print its baud rate.
    for (uint16_t i = SerialInterface::kCommsUART; i < SerialInterface::kNumSerialInterfaces; i++) {
        CONSOLE_PRINTF("AT+BAUD_RATE=%s,%lu\r\n", kSerialInterfaceStrs[i], settings.baud_rates[i]);
    }

    // AT+BIAS_TEE_ENABLE
    CONSOLE_PRINTF("AT+BIAS_TEE_ENABLE=%d\r\n", settings.r1090_bias_tee_enabled);

    // AT+DEVICE_INFO: Don't store this.

    // AT+ESP32_ENABLE
    CONSOLE_PRINTF("AT+ESP32_ENABLE=%d\r\n", settings.core_network_settings.esp32_enabled);

    // AT+ETHERNET
    CONSOLE_PRINTF("AT+ETHERNET=%d\r\n", settings.core_network_settings.ethernet_enabled);

    // AT+FEED
    for (uint16_t i = 0; i < Settings::kMaxNumFeeds; i++) {
        CONSOLE_PRINTF("AT+FEED=%d,%s,%u,%d,%s\r\n", i, settings.feed_uris[i], settings.feed_ports[i],
                       settings.feed_is_active[i], kReportingProtocolStrs[settings.feed_protocols[i]]);
    }

    // AT+LED_ENABLE
    CONSOLE_PRINTF("AT+LED_ENABLE=%d\r\n", settings.led_enabled);

    // AT+FEED_ENABLE
    CONSOLE_PRINTF("AT+FEED_ENABLE=%d\r\n", settings.feeds_enabled);

    // AT+GNSS
    CONSOLE_PRINTF("AT+GNSS=%d,%s,%d\r\n", settings.gnss_enabled,
                   GNSSReceiverTypeToStr(settings.gnss_receiver_type), settings.gnss_notify);

    // AT+LOG_LEVEL
    CONSOLE_PRINTF("AT+LOG_LEVEL=%s\r\n", kConsoleLogLevelStrs[settings.log_level]);

    // AT+PROTOCOL_OUT
    for (uint16_t i = 0; i < SerialInterface::kGNSSUART; i++) {
        CONSOLE_PRINTF("AT+PROTOCOL_OUT=%s,%s\r\n", kSerialInterfaceStrs[i],
                       kReportingProtocolStrs[settings.reporting_protocols[i]]);
    }

    // AT+RX_ENABLE
    CONSOLE_PRINTF("AT+RX_ENABLE=%d\r\n", settings.r1090_rx_enabled);

    // AT+SUBG_ENABLE
    CONSOLE_PRINTF("AT+SUBG_ENABLE=%s\r\n", SettingsManager::EnableStateToATValueStr(settings.subg_enabled));

    // AT+TL_OFFSET
    CONSOLE_PRINTF("AT+TL_OFFSET=%u\r\n", settings.tl_offset_mv);

    // AT+WATCHDOG
    CONSOLE_PRINTF("AT+WATCHDOG=%lu\r\n", settings.watchdog_timeout_sec);

    // AT+WIFI_AP
    CONSOLE_PRINTF("AT+WIFI_AP=%d,%s,%s,%d\r\n", settings.core_network_settings.wifi_ap_enabled,
                   settings.core_network_settings.wifi_ap_ssid, settings.core_network_settings.wifi_ap_password,
                   settings.core_network_settings.wifi_ap_channel);

    // AT+WIFI_STA
    CONSOLE_PRINTF("AT+WIFI_STA=%d,%s,%s\r\n", settings.core_network_settings.wifi_sta_enabled,
                   settings.core_network_settings.wifi_sta_ssid, settings.core_network_settings.wifi_sta_password);
}
