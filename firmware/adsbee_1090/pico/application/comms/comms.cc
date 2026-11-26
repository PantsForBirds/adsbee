#include "comms.hh"

#include <cstdarg>  // For debug printf.
#include <cstdio>   // Regular pico/stdio.h doesn't support vprint functions.

#include "cpu_utils.hh"
#include "esp32.hh"
#include "adsbee.hh"  // t-1360 LAB: GetPPSEdges.
#include "gnss_receiver.hh"
#include "hardware/irq.h"
#include "pico/printf.h"  // For vfctprintf.
#include "pico/stdlib.h"
#include "spi_coprocessor.hh"

extern GNSSReceiver* gnss;  // The selected GNSS receiver (gnss_interface.hh).

static const CommsManager::ReportSink kReportingSinks[] = {SettingsManager::SerialInterface::kConsole,
                                                           SettingsManager::SerialInterface::kCommsUART,
                                                           SettingsManager::SerialInterface::kGNSSUART};
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

    InitGNSSUART();

    for (UARTTx& uart : uart_tx_) {
        // TX interrupt when the FIFO drops to 4 bytes, so each one refills most of it.
        hw_write_masked(&uart_get_hw(uart.handle)->ifls, 0 << UART_UARTIFLS_TXIFLSEL_LSB, UART_UARTIFLS_TXIFLSEL_BITS);
        irq_set_exclusive_handler(UART_IRQ_NUM(uart.handle), OnUARTIRQ);
        irq_set_enabled(UART_IRQ_NUM(uart.handle), true);
    }
    // Lowest priority: output can wait. The GNSS UART keeps the default, since its interrupt also reads GNSS input.
    irq_set_priority(UART_IRQ_NUM(config_.comms_uart_handle), PICO_LOWEST_IRQ_PRIORITY);

    // Don't mess with ESP32 enable / reset GPIOs here, since they need to be toggled by the programmer. Only initialize
    // them if no programming is required. Don't mess with ESP32 wifi pin until we're ready to try firmware updates.

    stdio_init_all();
    stdio_set_translate_crlf(&stdio_usb, false);

    return true;
}

void CommsManager::InitGNSSUART() {
    gpio_set_function(config_.gnss_uart_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(config_.gnss_uart_rx_pin, GPIO_FUNC_UART);
    uart_set_translate_crlf(config_.gnss_uart_handle, false);
    uart_init(config_.gnss_uart_handle, settings_manager.settings.baud_rates[SettingsManager::kGNSSUART]);
}

static void XBUpdate();
bool CommsManager::Update() {
    XBUpdate();
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


// t-1341 LAB (never for merge): GNSS UART internal loopback at a test baud. DMA sends a burst shaped like a MAX-M10
// epoch every period; the GNSS receiver (GENERIC) reads it through its real RX interrupt. A 1 kHz-ish timer alarm at
// the GNSS UART's interrupt priority on core 0 measures how long core 0 can't take that interrupt.
#include "hardware/dma.h"
#include "hardware/timer.h"
#include "hardware/structs/timer.h"
extern volatile uint32_t lab_rx_bytes, lab_poll_bytes, lab_rx_oe, lab_rx_fe, lab_rx_be, lab_rx_pe, lab_ris_oe,
    lab_rx_irqs, lab_rx_max, lab_rx_hist[33];
extern volatile uint32_t lab_sent_total, lab_sent_unknown, lab_sent_malformed, lab_sent_gst;
static uint8_t xb_buf[2048];
static uint16_t xb_len = 0, xb_sentences = 0;
static int xb_dma = -1, xb_alarm = -1;
static uint32_t xb_baud = 0, xb_actual = 0, xb_period_ms = 0, xb_last_ms = 0, xb_bursts = 0, xb_skipped = 0,
                xb_t0_ms = 0, xb_relbe = 0;
static uint32_t xb_gga0 = 0, xb_rmc0 = 0, xb_ck0 = 0, xb_ovf0 = 0, xb_parsed0 = 0;
static volatile uint32_t xb_target = 0, xb_lcg = 12345, xb_lat_max = 0, xb_lat_n = 0, xb_lat_hist[10] = {0};
static const uint32_t kXBLatEdges[9] = {2, 5, 10, 20, 50, 100, 200, 500, 1000};
static void XBAlarmIRQ() {
    uint32_t now = timer_hw->timerawl;
    timer_hw->intr = 1u << xb_alarm;
    uint32_t lat = now - xb_target;
    if (lat > xb_lat_max) xb_lat_max = lat;
    int b = 0;
    while (b < 9 && lat >= kXBLatEdges[b]) b++;
    xb_lat_hist[b] = xb_lat_hist[b] + 1;
    xb_lat_n = xb_lat_n + 1;
    xb_lcg = xb_lcg * 1103515245u + 12345u;
    xb_target = timer_hw->timerawl + 500 + ((xb_lcg >> 16) % 1000);  // 0.5-1.5 ms, not phase-locked to loops
    timer_hw->alarm[xb_alarm] = xb_target;
}
static int XBAdd(int pos, const char* body) {
    uint8_t cs = 0;
    for (const char* p = body; *p; p++) cs ^= (uint8_t)*p;
    int n = snprintf((char*)xb_buf + pos, sizeof(xb_buf) - pos, "$%s*%02X\r\n", body, cs);
    xb_sentences++;
    return pos + n;
}
static void XBBuild(int want) {
    int pos = 0;
    xb_sentences = 0;
    pos = XBAdd(pos, "GNGGA,123519.00,3746.12345,N,12225.12345,W,1,32,0.6,10.2,M,-30.1,M,,");
    pos = XBAdd(pos, "GNRMC,123519.00,A,3746.12345,N,12225.12345,W,0.012,,251126,,,A,V");
    pos = XBAdd(pos, "GNGST,123519.00,1.2,0.8,0.6,45.0,0.9,0.7,1.6");
    char body[96];
    int k = 0;
    while (pos < want) {
        snprintf(body, sizeof(body), "GPGSV,9,%d,36,%02d,45,123,38,%02d,12,045,30,%02d,67,300,41,%02d,05,210,22,1",
                 (k % 9) + 1, (k * 4 + 1) % 99, (k * 4 + 2) % 99, (k * 4 + 3) % 99, (k * 4 + 4) % 99);
        pos = XBAdd(pos, body);
        k++;
    }
    xb_len = pos;
}
void XBStart(uint32_t baud, int bytes, uint32_t period_ms) {
    uart_inst_t* u = uart0;
    if (xb_alarm < 0) {
        xb_alarm = hardware_alarm_claim_unused(true);
        uint irq = TIMER_IRQ_0 + xb_alarm;
        irq_set_exclusive_handler(irq, XBAlarmIRQ);
        irq_set_priority(irq, PICO_DEFAULT_IRQ_PRIORITY);  // The GNSS UART's priority.
        hw_set_bits(&timer_hw->inte, 1u << xb_alarm);
        irq_set_enabled(irq, true);
        xb_target = timer_hw->timerawl + 1000;
        timer_hw->alarm[xb_alarm] = xb_target;
    }
    if (xb_dma < 0) xb_dma = dma_claim_unused_channel(true);
    if (baud == 0) {  // Hold: no new bursts, counters kept.
        xb_period_ms = 0;
        return;
    }
    while (dma_channel_is_busy(xb_dma)) tight_loop_contents();
    uart_tx_wait_blocking(u);
    XBBuild(bytes > (int)sizeof(xb_buf) ? sizeof(xb_buf) : bytes);
    xb_baud = baud;
    xb_actual = uart_set_baudrate(u, baud);
    hw_set_bits(&uart_get_hw(u)->cr, UART_UARTCR_LBE_BITS);
    sleep_ms(5);
    uint32_t irqs = save_and_disable_interrupts();
    while (uart_is_readable(u)) (void)uart_get_hw(u)->dr;
    lab_rx_bytes = lab_poll_bytes = lab_rx_oe = lab_rx_fe = lab_rx_be = lab_rx_pe = lab_ris_oe = lab_rx_irqs =
        lab_rx_max = 0;
    for (int i = 0; i < 33; i++) lab_rx_hist[i] = 0;
    lab_sent_total = lab_sent_unknown = lab_sent_malformed = lab_sent_gst = 0;
    xb_lat_max = xb_lat_n = 0;
    for (int i = 0; i < 10; i++) xb_lat_hist[i] = 0;
    restore_interrupts(irqs);
    if (gnss) {
        xb_gga0 = gnss->lab_gga();
        xb_rmc0 = gnss->lab_rmc();
        xb_ck0 = gnss->lab_cksum();
        xb_ovf0 = gnss->lab_ring_overflow();
        xb_parsed0 = gnss->lab_parsed_bytes();
    }
    xb_bursts = xb_skipped = xb_relbe = 0;
    xb_t0_ms = get_time_since_boot_ms();
    xb_last_ms = xb_t0_ms;
    xb_period_ms = period_ms;
}
// t-1360 LAB: one RMC per PPS edge (AT+UPTIME=XP,<baud>,<utc0>,<delay_ms>): delay_ms after each edge the loopback
// sends GGA + RMC with second utc0 + (edges since start), so the receiver labels the lab PPS edges like a real module.
static uint32_t xp_delay_us = 0, xp_n0 = 0, xp_last_n = 0, xp_sent = 0;
static uint64_t xp_utc0 = 0, xp_first_mlat = 0;
static bool xp_on = false;
static void XPBuild(uint64_t utc) {
    int64_t days = utc / 86400, sod = utc % 86400;
    // civil_from_days (H. Hinnant)
    int64_t z = days + 719468, era = z / 146097, doe = z - era * 146097;
    int64_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365, y = yoe + era * 400;
    int64_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100), mp = (5 * doy + 2) / 153;
    int64_t d = doy - (153 * mp + 2) / 5 + 1, m = mp < 10 ? mp + 3 : mp - 9;
    if (m <= 2) y++;
    char t[16], body[128];
    snprintf(t, sizeof(t), "%02d%02d%02d.00", (int)(sod / 3600), (int)(sod / 60 % 60), (int)(sod % 60));
    int pos = 0;
    xb_sentences = 0;
    snprintf(body, sizeof(body), "GNGGA,%s,3746.12345,N,12225.12345,W,1,12,0.8,10.2,M,-30.1,M,,", t);
    pos = XBAdd(pos, body);
    snprintf(body, sizeof(body), "GNRMC,%s,A,3746.12345,N,12225.12345,W,0.012,,%02d%02d%02d,,,A,V", t, (int)d, (int)m,
             (int)(y % 100));
    pos = XBAdd(pos, body);
    xb_len = pos;
}
void XPStart(uint32_t baud, uint64_t utc0, uint32_t delay_ms) {
    XBStart(baud, 200, 0);  // Loopback on at baud, no periodic bursts.
    PPSClock::Edge latest, previous;
    xp_n0 = xp_last_n = adsbee.GetPPSEdges(latest, previous);
    xp_utc0 = utc0;
    xp_first_mlat = 0;
    xp_delay_us = delay_ms * 1000;
    xp_sent = 0;
    xp_on = baud != 0;
}
int XPPrint(char* out, int n) {
    return snprintf(out, n, "XP on=%d sent=%lu utc0=%llu delay_us=%lu seq=%u step=%d", xp_on, (unsigned long)xp_sent,
                    (unsigned long long)xp_utc0, (unsigned long)xp_delay_us, GNSSReceiver::pps_time_seq(),
                    GNSSReceiver::pps_time_step());
}
static void XPUpdate() {
    if (!xp_on || xb_dma < 0 || dma_channel_is_busy(xb_dma)) return;
    PPSClock::Edge latest, previous;
    uint32_t num = adsbee.GetPPSEdges(latest, previous);
    if (num == xp_last_n || time_us_32() - latest.time_us < xp_delay_us) return;
    xp_last_n = num;
    // The second by elapsed time from the first edge (48 MHz counts, rounded): a skipped edge keeps the labels right.
    if (xp_first_mlat == 0) xp_first_mlat = latest.mlat_48mhz_counts;
    XPBuild(xp_utc0 + 1 + (latest.mlat_48mhz_counts - xp_first_mlat + 24'000'000) / 48'000'000);
    dma_channel_config c = dma_channel_get_default_config(xb_dma);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, uart_get_dreq(uart0, true));
    dma_channel_configure(xb_dma, &c, &uart_get_hw(uart0)->dr, xb_buf, xb_len, true);
    xp_sent++;
}

static void XBUpdate() {
    if (xb_dma < 0 || xb_baud == 0) return;
    XPUpdate();
    if (!(uart_get_hw(uart0)->cr & UART_UARTCR_LBE_BITS)) {
        hw_set_bits(&uart_get_hw(uart0)->cr, UART_UARTCR_LBE_BITS);
        xb_relbe++;
    }
    if (xb_period_ms == 0) return;
    uint32_t now = get_time_since_boot_ms();
    if (now - xb_last_ms < xb_period_ms) return;
    xb_last_ms += xb_period_ms;
    if (dma_channel_is_busy(xb_dma)) {
        xb_skipped++;
        return;
    }
    dma_channel_config c = dma_channel_get_default_config(xb_dma);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, uart_get_dreq(uart0, true));
    dma_channel_configure(xb_dma, &c, &uart_get_hw(uart0)->dr, xb_buf, xb_len, true);
    xb_bursts++;
}
int XBPrint(char* out, int n) {
    uint32_t remaining = xb_dma >= 0 ? dma_hw->ch[xb_dma].transfer_count : 0;
    uint32_t sent = xb_bursts * xb_len - (dma_channel_is_busy(xb_dma) ? remaining : 0);
    uint32_t got = lab_rx_bytes + lab_poll_bytes;
    int k = snprintf(out, n,
                     "XB baud=%lu actual=%lu len=%u sent_per_burst=%u secs=%lu bursts=%lu skipped=%lu sent=%lu rx=%lu "
                     "poll=%lu lost=%ld oe=%lu ris_oe=%lu fe=%lu be=%lu pe=%lu irqs=%lu rxmax=%lu ovf=%lu gga=%lu rmc=%lu "
                     "gst=%lu ck=%lu unk=%lu mal=%lu sent_total=%lu parsed=%lu relbe=%lu lat_max=%lu lat_n=%lu lat=",
                     (unsigned long)xb_baud, (unsigned long)xb_actual, xb_len, xb_sentences,
                     (unsigned long)((get_time_since_boot_ms() - xb_t0_ms) / 1000), (unsigned long)xb_bursts,
                     (unsigned long)xb_skipped, (unsigned long)sent, (unsigned long)lab_rx_bytes,
                     (unsigned long)lab_poll_bytes, (long)(sent - got), (unsigned long)lab_rx_oe,
                     (unsigned long)lab_ris_oe, (unsigned long)lab_rx_fe, (unsigned long)lab_rx_be,
                     (unsigned long)lab_rx_pe, (unsigned long)lab_rx_irqs, (unsigned long)lab_rx_max,
                     (unsigned long)(gnss ? gnss->lab_ring_overflow() - xb_ovf0 : 0),
                     (unsigned long)(gnss ? gnss->lab_gga() - xb_gga0 : 0),
                     (unsigned long)(gnss ? gnss->lab_rmc() - xb_rmc0 : 0), (unsigned long)lab_sent_gst,
                     (unsigned long)(gnss ? gnss->lab_cksum() - xb_ck0 : 0), (unsigned long)lab_sent_unknown,
                     (unsigned long)lab_sent_malformed, (unsigned long)lab_sent_total,
                     (unsigned long)(gnss ? gnss->lab_parsed_bytes() - xb_parsed0 : 0), (unsigned long)xb_relbe,
                     (unsigned long)xb_lat_max, (unsigned long)xb_lat_n);
    for (int i = 0; i < 10 && k < n; i++) k += snprintf(out + k, n - k, "%lu%s", (unsigned long)xb_lat_hist[i], i < 9 ? "/" : "");
    if (k < n) k += snprintf(out + k, n - k, " hist=");
    for (int i = 0; i < 33 && k < n; i++)
        if (lab_rx_hist[i]) k += snprintf(out + k, n - k, "%d:%lu,", i, (unsigned long)lab_rx_hist[i]);
    return k;
}

CommsManager::UARTTx* CommsManager::GetUARTTx(SettingsManager::SerialInterface iface) {
    switch (iface) {
        case SettingsManager::kCommsUART:
            return &uart_tx_[0];
        case SettingsManager::kGNSSUART:
            return &uart_tx_[1];
        default:
            return nullptr;
    }
}

// Shared by both UART interrupts. The GNSS UART's also carries GNSS receiver input.
void CommsManager::OnUARTIRQ() {
    uint irq_num = __get_current_exception() - VTABLE_FIRST_IRQ;
    for (UARTTx& uart : comms_manager.uart_tx_) {
        if (UART_IRQ_NUM(uart.handle) == irq_num) {
            comms_manager.FillUARTTxFIFO(uart);
        }
    }
    if (irq_num == UART_IRQ_NUM(comms_manager.config_.gnss_uart_handle)) {
        GNSSReceiver::RxIRQHandler();
    }
}

void CommsManager::ApplyGNSSUARTBaudRate() {
    if (gnss != nullptr) {
        gnss->ApplyBaudRateSetting();
    } else {
        uart_set_baudrate(config_.gnss_uart_handle, settings_manager.settings.baud_rates[SettingsManager::kGNSSUART]);
    }
}

bool CommsManager::FlushUARTTx(SettingsManager::SerialInterface iface, uint32_t timeout_ms) {
    UARTTx* uart = GetUARTTx(iface);
    if (uart == nullptr) return false;
    uint32_t start_ms = get_time_since_boot_ms();
    while (!uart->queue.IsEmpty()) {
        FillUARTTxFIFO(*uart);
        if (get_time_since_boot_ms() - start_ms > timeout_ms) return false;
    }
    uart_tx_wait_blocking(uart->handle);  // At most a FIFO (32 bytes) on the wire.
    return true;
}

void CommsManager::FillUARTTxFIFO(UARTTx& uart) {
    uart_hw_t* uart_hw = uart_get_hw(uart.handle);
    uint32_t interrupts = save_and_disable_interrupts();  // Keeps the TX interrupt out when called from core 0 code.
    char c;
    while (uart_is_writable(uart.handle) && uart.queue.Dequeue(c)) {
        uart_hw->dr = c;
    }
    // The interrupt fires when the FIFO level falls through the threshold. Bytes left over mean the FIFO is full, so it
    // will fire again.
    if (uart.queue.IsEmpty()) {
        hw_clear_bits(&uart_hw->imsc, UART_UARTIMSC_TXIM_BITS);
    } else {
        hw_set_bits(&uart_hw->imsc, UART_UARTIMSC_TXIM_BITS);
    }
    restore_interrupts(interrupts);
}

bool CommsManager::iface_putc(SettingsManager::SerialInterface iface, char c) {
    switch (iface) {
        case SettingsManager::kCommsUART:
        case SettingsManager::kGNSSUART:
            return iface_write(iface, &c, 1);
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
    UARTTx* uart = GetUARTTx(iface);
    if (uart != nullptr) {
        if (!wait && len > uart->queue.MaxNumElements() - uart->queue.Length()) {
            return false;  // No room for the whole buffer.
        }
        for (uint16_t i = 0; i < len; i++) {
            if (uart->queue.Enqueue(buf[i])) {
                continue;
            }
            // Queue full: make sure it is draining, then wait for space unless the UART has stalled.
            uint32_t wait_start_ms = get_time_since_boot_ms();
            do {
                FillUARTTxFIFO(*uart);
                if (get_time_since_boot_ms() - wait_start_ms > kUARTTxStallTimeoutMs) {
                    return false;
                }
            } while (!uart->queue.Enqueue(buf[i]));
        }
        FillUARTTxFIFO(*uart);  // Starts the drain if the UART was idle.
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
        case SettingsManager::kGNSSUART:
            return iface_write(iface, buf, strlen(buf));
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