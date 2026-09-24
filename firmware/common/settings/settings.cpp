#include "settings.hh"

#ifdef ON_PICO
#include "adsbee.hh"
#endif
#include "comms.hh"

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
