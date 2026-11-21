#include "comms.hh"

#include <cstdarg>  // For debug printf.
#include <cstdio>   // Regular pico/stdio.h doesn't support vprint functions.

#include "cpu_utils.hh"
#include "esp32.hh"
#include "hardware/irq.h"
#include "pico/printf.h"  // For vfctprintf.
#include "pico/stdlib.h"
#include "spi_coprocessor.hh"

static const CommsManager::ReportSink kReportingSinks[] = {SettingsManager::SerialInterface::kConsole,
                                                           SettingsManager::SerialInterface::kCommsUART};
static const uint16_t kNumReportingSinks = sizeof(kReportingSinks) / sizeof(CommsManager::ReportSink);

// The serial sinks with their protocols from the settings. Serial interfaces have no capabilities check: beast_caps 0.
static uint16_t SerialReportSinks(CommsManager::ReportSinkInfo (&sinks)[kNumReportingSinks]) {
    for (uint16_t i = 0; i < kNumReportingSinks; i++) {
        sinks[i] = {kReportingSinks[i], settings_manager.settings.reporting_protocols[kReportingSinks[i]]};
    }
    return kNumReportingSinks;
}

CommsManager::CommsManager(CommsManagerConfig config_in)
    : config_(config_in), at_parser_(CppAT(at_command_list, at_command_list_num_commands, true)) {}

bool CommsManager::Init() {
    gpio_set_function(config_.comms_uart_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(config_.comms_uart_rx_pin, GPIO_FUNC_UART);
    uart_set_translate_crlf(config_.comms_uart_handle, false);
    uart_init(config_.comms_uart_handle, SettingsManager::Settings::kDefaultCommsUARTBaudrate);
    // TX interrupt when the FIFO drops to 4 bytes, so each one refills most of it. Lowest priority: output can wait.
    hw_write_masked(&uart_get_hw(config_.comms_uart_handle)->ifls, 0 << UART_UARTIFLS_TXIFLSEL_LSB,
                    UART_UARTIFLS_TXIFLSEL_BITS);
    irq_set_exclusive_handler(UART_IRQ_NUM(config_.comms_uart_handle), OnCommsUARTTx);
    irq_set_priority(UART_IRQ_NUM(config_.comms_uart_handle), PICO_LOWEST_IRQ_PRIORITY);
    irq_set_enabled(UART_IRQ_NUM(config_.comms_uart_handle), true);

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
        CPUMonitor::WorkScope work;

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
            ReportSinkInfo sinks[kNumReportingSinks];
            uint16_t num_sinks = SerialReportSinks(sinks);
            UpdateReporting(sinks, num_sinks,
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
        ReportSinkInfo sinks[kNumReportingSinks];
        uint16_t num_sinks = SerialReportSinks(sinks);
        UpdateReporting(sinks, num_sinks,
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
    va_list args_again;
    va_copy(args_again, args);

    CPUMonitor::WorkScope work;
    // Formatted print to buffer.
    int res = vsnprintf(buf, kPrintfBufferMaxSize, format, args);
    if (res >= kPrintfBufferMaxSize) {
        // Too long for buf: format it again with the SDK's streaming printf, writing out buf each time it fills.
        struct Sink {
            SettingsManager::SerialInterface iface;
            char* buf;
            uint16_t len;
            bool ok;
        } sink = {iface, buf, 0, true};
        vfctprintf(
            [](char c, void* arg) {
                Sink* s = static_cast<Sink*>(arg);
                s->buf[s->len++] = c;
                if (s->len == kPrintfBufferMaxSize) {
                    s->ok = comms_manager.iface_write(s->iface, s->buf, s->len) && s->ok;
                    s->len = 0;
                }
            },
            &sink, format, args_again);
        va_end(args_again);
        sink.ok = iface_write(iface, buf, sink.len) && sink.ok;
        return sink.ok && comms_manager.UpdateNetworkConsole() ? res : -1;
    }
    va_end(args_again);
    if (res <= 0) {
        return res;  // vsnprintf failed.
    }
    // Send buffer to interface, then manually push messages (otherwise they only pop out when the buffer gets full).
    if (iface_puts(iface, buf) && comms_manager.UpdateNetworkConsole()) {
        return res;  // Return number of characters written.
    }

    return -1;  // puts failed.
}

// t-1261 LAB: PL011 internal loopback + Beast parser on RX (never for merge).
volatile uint32_t xu_bytes = 0, xu_ok = 0, xu_other_df = 0, xu_bad_crc = 0, xu_framing = 0, xu_other_type = 0, xu_oe = 0,
                  xu_short = 0, xu_t0_ms = 0;
static uint8_t xu_state = 0, xu_type = 0, xu_buf[24], xu_len = 0, xu_need = 0;
static bool xu_esc = false;
static uint32_t XUCRC(const uint8_t* b, int n) {
    uint32_t crc = 0;
    for (int i = 0; i < n - 3; i++) {
        crc ^= (uint32_t)b[i] << 16;
        for (int k = 0; k < 8; k++) crc = (crc & 0x800000) ? ((crc << 1) ^ 0xFFF409) : (crc << 1);
    }
    crc &= 0xFFFFFF;
    return crc ^ (((uint32_t)b[n - 3] << 16) | ((uint32_t)b[n - 2] << 8) | b[n - 1]);
}
static void XUFinish() {
    int n = xu_type == 0x33 ? 14 : 7;
    const uint8_t* m = xu_buf + 7;
    uint8_t df = m[0] >> 3;
    uint32_t r = XUCRC(m, n);
    if (df == 17 || df == 18 || df == 11) {
        if (r == 0) xu_ok++; else xu_bad_crc++;
    } else {
        xu_other_df++;
    }
}
static void XUByte(uint8_t c) {
    xu_bytes++;
    switch (xu_state) {
        case 0:  // Expect 0x1A.
            if (c == 0x1A) xu_state = 1; else { xu_framing++; xu_state = 3; }
            break;
        case 1:  // Type.
            if (c == 0x32 || c == 0x33) {
                xu_type = c; xu_len = 0; xu_need = 7 + (c == 0x33 ? 14 : 7); xu_esc = false; xu_state = 2;
            } else { xu_other_type++; xu_state = 3; }
            break;
        case 2:  // Payload with escapes.
            if (xu_esc) {
                xu_esc = false;
                if (c != 0x1A) { xu_short++; xu_type = 0; if (c == 0x32 || c == 0x33) { xu_type = c; xu_len = 0; xu_need = 7 + (c == 0x33 ? 14 : 7); xu_state = 2; } else { xu_other_type++; xu_state = 3; } break; }
            } else if (c == 0x1A) { xu_esc = true; break; }
            xu_buf[xu_len++] = c;
            if (xu_len == xu_need) { XUFinish(); xu_state = 0; }
            break;
        case 3:  // Resync: wait for 0x1A followed by a Mode S type.
            if (c == 0x1A) xu_state = 1;
            break;
    }
}
static void XUDrainRx(uart_inst_t* u) {
    while (uart_is_readable(u)) {
        uint32_t dr = uart_get_hw(u)->dr;
        if (dr & UART_UARTDR_OE_BITS) xu_oe++;
        XUByte(dr & 0xFF);
    }
}
void XUEnable(bool on) {
    uart_inst_t* u = comms_manager.LabUART();
    uint32_t irq = save_and_disable_interrupts();
    if (on) {
        hw_set_bits(&uart_get_hw(u)->cr, UART_UARTCR_LBE_BITS);
        while (uart_is_readable(u)) (void)uart_get_hw(u)->dr;
        xu_bytes = xu_ok = xu_other_df = xu_bad_crc = xu_framing = xu_other_type = xu_oe = xu_short = 0;
        xu_state = 3; xu_t0_ms = get_time_since_boot_ms();
        hw_set_bits(&uart_get_hw(u)->imsc, UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS);
    } else {
        hw_clear_bits(&uart_get_hw(u)->imsc, UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS);
        hw_clear_bits(&uart_get_hw(u)->cr, UART_UARTCR_LBE_BITS);
    }
    restore_interrupts(irq);
}

void CommsManager::OnCommsUARTTx() {
    XUDrainRx(comms_manager.config_.comms_uart_handle);
    comms_manager.FillCommsUARTTxFIFO();
}

void CommsManager::FillCommsUARTTxFIFO() {
    uart_hw_t* uart_hw = uart_get_hw(config_.comms_uart_handle);
    uint32_t interrupts = save_and_disable_interrupts();  // Keeps the TX interrupt out when called from core 0 code.
    char c;
    while (uart_is_writable(config_.comms_uart_handle) && comms_uart_tx_queue_.Dequeue(c)) {
        uart_hw->dr = c;
    }
    // The interrupt fires when the FIFO level falls through the threshold. Bytes left over mean the FIFO is full, so it
    // will fire again.
    if (comms_uart_tx_queue_.IsEmpty()) {
        hw_clear_bits(&uart_hw->imsc, UART_UARTIMSC_TXIM_BITS);
    } else {
        hw_set_bits(&uart_hw->imsc, UART_UARTIMSC_TXIM_BITS);
    }
    restore_interrupts(interrupts);
}

bool CommsManager::iface_putc(SettingsManager::SerialInterface iface, char c) {
    switch (iface) {
        case SettingsManager::kCommsUART:
            return iface_write(iface, &c, 1);
            break;
        case SettingsManager::kGNSSUART:
            uart_putc_raw(config_.gnss_uart_handle, c);
            return true;  // Function is void so we won't know if it succeeds.
            break;
        case SettingsManager::kConsole:
            return putchar(c) >= 0 && (!MirrorToNetworkConsole() || network_console_putc(c) >= 0);
            break;
        case SettingsManager::kNumSerialInterfaces:
        default:
            CONSOLE_WARNING("CommsManager::iface_putc", "Unrecognized iface %d.", iface);
            return false;
    }
    return false;  // Should never get here.
}

bool CommsManager::iface_write(SettingsManager::SerialInterface iface, const char* buf, uint16_t len, bool wait) {
    if (iface == SettingsManager::kCommsUART) {
        if (!wait && len > comms_uart_tx_queue_.MaxNumElements() - comms_uart_tx_queue_.Length()) {
            return false;  // No room for the whole buffer.
        }
        for (uint16_t i = 0; i < len; i++) {
            if (comms_uart_tx_queue_.Enqueue(buf[i])) {
                continue;
            }
            // Queue full: make sure it is draining, then wait for space unless the UART has stalled.
            uint32_t wait_start_ms = get_time_since_boot_ms();
            do {
                FillCommsUARTTxFIFO();
                if (get_time_since_boot_ms() - wait_start_ms > kCommsUARTTxStallTimeoutMs) {
                    return false;
                }
            } while (!comms_uart_tx_queue_.Enqueue(buf[i]));
        }
        FillCommsUARTTxFIFO();  // Starts the drain if the UART was idle.
        return true;
    }
    if (iface != SettingsManager::kConsole) {
        for (uint16_t i = 0; i < len; i++) {
            if (!iface_putc(iface, buf[i])) {
                return false;
            }
        }
        return true;
    }
    stdio_put_string(buf, len, false, false);
    if (MirrorToNetworkConsole()) {
        for (uint16_t i = 0; i < len; i++) {
            network_console_putc(buf[i]);
        }
    }
    return true;
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
            return iface_write(iface, buf, strlen(buf));
            break;
        case SettingsManager::kGNSSUART:
            uart_puts(config_.gnss_uart_handle, buf);
            return true;  // Function is void so we won't know if it succeeds.
            break;
        case SettingsManager::kConsole:
            // Note: Using fputs instead of standard puts, since puts adds a line feed.
            return fputs(buf, stdout) >= 0 && (!MirrorToNetworkConsole() || network_console_puts(buf) >= 0);
            break;
        case SettingsManager::kNumSerialInterfaces:
        default:
            CONSOLE_WARNING("CommsManager::iface_puts", "Unrecognized iface %d.", iface);
            return false;  // Didn't match an interface.
            break;
    }
    return false;  // Should never get here.
}

bool CommsManager::MirrorToNetworkConsole() { return esp32.IsEnabled() && esp32_ll.NetworkConsoleHasConsumers(); }

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