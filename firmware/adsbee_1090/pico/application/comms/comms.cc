#include "comms.hh"

#include <cstdarg>  // For debug printf.
#include <cstdio>   // Regular pico/stdio.h doesn't support vprint functions.

#include "pico/stdlib.h"
#include "spi_coprocessor.hh"

static const CommsManager::ReportSink kReportingSinks[] = {SettingsManager::SerialInterface::kConsole,
                                                           SettingsManager::SerialInterface::kCommsUART};
static const uint16_t kNumReportingSinks = sizeof(kReportingSinks) / sizeof(CommsManager::ReportSink);

CommsManager::CommsManager(CommsManagerConfig config_in)
    : config_(config_in), at_parser_(CppAT(at_command_list, at_command_list_num_commands, true)) {}

bool CommsManager::Init() {
    gpio_set_function(config_.comms_uart_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(config_.comms_uart_rx_pin, GPIO_FUNC_UART);
    uart_set_translate_crlf(config_.comms_uart_handle, false);
    uart_init(config_.comms_uart_handle, SettingsManager::Settings::kDefaultCommsUARTBaudrate);

    gpio_set_function(config_.gnss_uart_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(config_.gnss_uart_rx_pin, GPIO_FUNC_UART);
    uart_set_translate_crlf(config_.gnss_uart_handle, false);
    uart_init(config_.gnss_uart_handle, SettingsManager::Settings::kDefaultGNSSUARTBaudrate);

    // Don't mess with ESP32 enable / reset GPIOs here, since they need to be toggled by the programmer. Only initialize
    // them if no programming is required. Don't mess with ESP32 wifi pin until we're ready to try firmware updates.

    stdio_init_all();
    stdio_set_translate_crlf(&stdio_usb, false);

    return true;
}

bool CommsManager::Update() {
    UpdateAT();
    UpdateNetworkConsole();

    uint32_t timestamp_ms = get_time_since_boot_ms();
    if (timestamp_ms - last_raw_report_check_timestamp_ms_ > kRawReportingCheckIntervalMs) {
        last_raw_report_check_timestamp_ms_ = timestamp_ms;  // Proceed with update and record timestamp.

        // Calculate how much buffer space the current packets would need.
        uint16_t required_buffer_len = CompositeArray::CalculateRawPacketsBufferLength(
            &mode_s_packet_reporting_queue, &uat_adsb_packet_reporting_queue, &uat_uplink_packet_reporting_queue);

        // Flush when combined packets from all queues would fill a composite array (maximizes load factor per
        // SPI transaction), or when the max reporting interval has elapsed (guarantees ≤200ms latency).
        bool max_interval_elapsed = (timestamp_ms - last_raw_report_timestamp_ms_) >= kRawReportingMaxIntervalMs;

        if (mode_s_packet_reporting_queue.Length() == 0 && uat_adsb_packet_reporting_queue.Length() == 0 &&
            uat_uplink_packet_reporting_queue.Length() == 0) {
            // No packets: still drive the locally decoded protocols, which send the heartbeats.
            uint8_t no_packets_buf[sizeof(CompositeArray::RawPackets::Header)];
            CompositeArray::RawPackets no_packets = CompositeArray::PackRawPacketsBuffer(
                no_packets_buf, sizeof(no_packets_buf), nullptr, nullptr, nullptr);
            UpdateReporting(kReportingSinks, settings_manager.settings.reporting_protocols, kNumReportingSinks,
                            &no_packets);
        } else if (required_buffer_len >= CompositeArray::RawPackets::kMaxLenBytes || max_interval_elapsed) {
            ForceFlushRawPackets();
        }
    }

    return true;
}

bool CommsManager::ForceFlushRawPackets() {
    for (uint16_t i = 0; i < kMaxForceFlushIterations; i++) {
        if (mode_s_packet_reporting_queue.Length() == 0 && uat_adsb_packet_reporting_queue.Length() == 0 &&
            uat_uplink_packet_reporting_queue.Length() == 0) {
            break;
        }
        last_raw_report_timestamp_ms_ = get_time_since_boot_ms();
        uint8_t packets_to_report_buf[CompositeArray::RawPackets::kMaxLenBytes] = {0};
        CompositeArray::RawPackets packets_to_report = CompositeArray::PackRawPacketsBuffer(
            packets_to_report_buf, sizeof(packets_to_report_buf), &mode_s_packet_reporting_queue,
            &uat_adsb_packet_reporting_queue, &uat_uplink_packet_reporting_queue);
        if (esp32.IsEnabled() && packets_to_report.IsValid()) {
            esp32.Write(ObjectDictionary::kAddrCompositeArrayRawPackets, packets_to_report_buf, true,
                        packets_to_report.len_bytes);
        }
        UpdateReporting(kReportingSinks, settings_manager.settings.reporting_protocols, kNumReportingSinks,
                        &packets_to_report);
    }
    if (mode_s_packet_reporting_queue.Length() > 0 || uat_adsb_packet_reporting_queue.Length() > 0 ||
        uat_uplink_packet_reporting_queue.Length() > 0) {
        CONSOLE_ERROR("CommsManager::ForceFlushRawPackets", "Reporting queues not fully drained after %d iterations.",
                      kMaxForceFlushIterations);
    }
    return true;
}

bool CommsManager::UpdateNetworkConsole(bool force) {
    static bool recursion_alert = false;
    if (recursion_alert) {
        return false;
    }
    recursion_alert = true;
    if (esp32.IsEnabled()) {
        // Limit the max reporting rate.
        if (!force && esp32_console_tx_queue.Length() < kNetworkConsoleReportingIntervalOverrideNumChars &&
            (get_time_since_boot_ms() - last_esp32_console_tx_timestamp_ms_) < kNetworkConsoleMinReportingIntervalMs) {
            // If the queue is too long, or if enough time has passed since the last TX, send the characters.
            recursion_alert = false;
            return true;  // Don't send anything if we don't need to.
        }

        // Send outgoing network console characters.
        char esp32_console_tx_buf[SPICoprocessorPacket::SCWritePacket::kDataMaxLenBytes];
        char c = '\0';
        while (esp32_console_tx_queue.Length() > 0) {
            uint16_t message_len = 0;
            for (; message_len < SPICoprocessorPacket::SCWritePacket::kDataMaxLenBytes &&
                   esp32_console_tx_queue.Dequeue(c);
                 message_len++) {
                esp32_console_tx_buf[message_len] = c;
            }
            if (message_len > 0) {
                if (!esp32.Write(ObjectDictionary::kAddrConsole, esp32_console_tx_buf, true, message_len)) {
                    break;
                } else {
                    last_esp32_console_tx_timestamp_ms_ = get_time_since_boot_ms();
                }
            }
        }
    }
    recursion_alert = false;
    return true;
}

int CommsManager::console_printf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    int res = iface_vprintf(SettingsManager::SerialInterface::kConsole, format, args);
    va_end(args);
    return res;
}

int CommsManager::console_level_printf(SettingsManager::LogLevel level, const char* format, ...) {
    if (settings_manager.settings.log_level < level) return 0;
    va_list args;
    va_start(args, format);
    int res = iface_vprintf(SettingsManager::SerialInterface::kConsole, format, args);
    va_end(args);
    return res;
}

int CommsManager::iface_printf(SettingsManager::SerialInterface iface, const char* format, ...) {
    va_list args;
    va_start(args, format);
    int res = iface_vprintf(iface, format, args);
    va_end(args);
    return res;
}

int CommsManager::iface_vprintf(SettingsManager::SerialInterface iface, const char* format, va_list args) {
    char buf[kPrintfBufferMaxSize];

    // Formatted print to buffer.
    int res = vsnprintf(buf, kPrintfBufferMaxSize, format, args);
    if (res <= 0) {
        return res;  // vsnprintf failed.
    }
    // Send buffer to interface, then manually push messages (otherwise they only pop out when the buffer gets full).
    if (iface_puts(iface, buf) && comms_manager.UpdateNetworkConsole()) {
        return res;  // Return number of characters written.
    }

    return -1;  // puts failed.
}

bool CommsManager::iface_putc(SettingsManager::SerialInterface iface, char c) {
    switch (iface) {
        case SettingsManager::kCommsUART:
            uart_putc_raw(config_.comms_uart_handle, c);
            return true;  // Function is void so we won't know if it succeeds.
            break;
        case SettingsManager::kGNSSUART:
            uart_putc_raw(config_.gnss_uart_handle, c);
            return true;  // Function is void so we won't know if it succeeds.
            break;
        case SettingsManager::kConsole:
            return putchar(c) >= 0 && (!esp32.IsEnabled() || network_console_putc(c) >= 0);
            break;
        case SettingsManager::kNumSerialInterfaces:
        default:
            CONSOLE_WARNING("CommsManager::iface_putc", "Unrecognized iface %d.", iface);
            return false;
    }
    return false;  // Should never get here.
}

bool CommsManager::iface_getc(SettingsManager::SerialInterface iface, char& c) {
    switch (iface) {
        case SettingsManager::kCommsUART:
            if (uart_is_readable_within_us(config_.comms_uart_handle, config_.uart_timeout_us)) {
                c = uart_getc(config_.comms_uart_handle);
                return true;
            }
            return false;  // No chars to read.
            break;
        case SettingsManager::kGNSSUART:
            if (uart_is_readable_within_us(config_.gnss_uart_handle, config_.uart_timeout_us)) {
                c = uart_getc(config_.gnss_uart_handle);
                return true;
            }
            return false;  // No chars to read.
            break;
        case SettingsManager::kConsole: {
            int ret = getchar_timeout_us(config_.uart_timeout_us);
            if (ret >= 0) {
                c = (char)ret;
                return true;
            }
            return false;  // Failed to read character.
            break;
        }
        case SettingsManager::kNumSerialInterfaces:
        default:
            CONSOLE_WARNING("CommsManager::iface_getc", "Unrecognized iface %d.", iface);
            return false;  // Didn't match an interface.
            break;
    }
    return false;  // Should never get here.
}

bool CommsManager::iface_puts(SettingsManager::SerialInterface iface, const char* buf) {
    switch (iface) {
        case SettingsManager::kCommsUART:
            uart_puts(config_.comms_uart_handle, buf);
            return true;  // Function is void so we won't know if it succeeds.
            break;
        case SettingsManager::kGNSSUART:
            uart_puts(config_.gnss_uart_handle, buf);
            return true;  // Function is void so we won't know if it succeeds.
            break;
        case SettingsManager::kConsole:
            // Note: Using fputs instead of standard puts, since puts adds a line feed.
            return fputs(buf, stdout) >= 0 && (!esp32.IsEnabled() || network_console_puts(buf) >= 0);
            break;
        case SettingsManager::kNumSerialInterfaces:
        default:
            CONSOLE_WARNING("CommsManager::iface_puts", "Unrecognized iface %d.", iface);
            return false;  // Didn't match an interface.
            break;
    }
    return false;  // Should never get here.
}

bool CommsManager::network_console_putc(char c) {
    static bool recursion_alert = false;
    if (recursion_alert) {
        return false;  // Don't get into infinite loops in case UpdateAT or Enqueue() create error messages that would
                       // in turn create more network_console_putc calls.
    }
    recursion_alert = true;
    if (!comms_manager.esp32_console_tx_queue.Enqueue(c)) {
        // Try flushing before dropping. No UpdateAT() here: inside an AT callback (e.g. AT+OTA=WRITE) it would parse
        // binary payload as commands.
        comms_manager.UpdateNetworkConsole();
        if (comms_manager.esp32_console_tx_queue.Enqueue(c)) {
            recursion_alert = false;
            return true;  // Crisis averted! Phew.
        }
        // Flush failed, clear the buffer.
        comms_manager.esp32_console_tx_queue.Clear();
        recursion_alert = false;
        CONSOLE_ERROR("CommsManager::network_console_putc", "Overflowed buffer for outgoing network console chars.");
        return false;
    }
    recursion_alert = false;
    return true;
}
bool CommsManager::network_console_puts(const char* buf, uint16_t len) {
    for (uint16_t i = 0; i < strlen(buf) && i < len; i++) {
        if (!network_console_putc(buf[i])) {
            return false;
        }
    }
    return true;
}