#include "gnss_receiver.hh"

#include <cstdio>

#include "comms.hh"             // For comms_manager (UART I/O + baud) and CONSOLE_* logging.
#include "hal.hh"               // For get_time_since_boot_ms().

void GNSSReceiver::ClaimUart() {
    // Bring the GNSS UART up to a known state. uart0 is shared with the ESP32 flasher (GPIO 16/17),
    // which calls uart_deinit(uart0) when it finishes; claim the GNSS pins (GPIO 0/1) and
    // (re)initialize the peripheral here so a prior ESP32 flash cannot leave the GNSS link dead.
    //
    // Fully reset the peripheral before re-init. On a cold boot (no ESP32 flash) uart0 has already
    // been brought up by CommsManager::Init() -- and, until we re-baud below, at a rate that doesn't
    // match the module, so its RX FIFO has been filling with framing-error garbage. uart_init() on an
    // already-live uart0 does not reliably clear that latched state, which corrupts the first probe.
    // uart_deinit() first mirrors the clean-peripheral state the ESP32 flasher's DeInit() hands us on
    // the post-flash path (which is why GNSS bring-up is reliable there but flaky on cold boot).
    uart_deinit(config_.uart_handle);
    gpio_set_function(config_.uart_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(config_.uart_rx_pin, GPIO_FUNC_UART);
    uart_set_translate_crlf(config_.uart_handle, false);
    uart_init(config_.uart_handle, GetDefaultBaudrate());

    // Drain any bytes left in the RX FIFO from before the re-init so the first UBX-MON-VER probe
    // reads a clean stream (mirrors the flasher's EnterBootloader() flush).
    while (uart_is_readable(config_.uart_handle)) {
        (void)uart_getc(config_.uart_handle);
    }
}

bool GNSSReceiver::Init() {
    // NONE means that no receiver is connected. Do not claim the UART, toggle the power pin, or
    // otherwise touch the GNSS interface, even if the caller requested that it be enabled.
    if (module_type_ == BSP::kGNSSModuleNone) {
        healthy_ = false;
        active_ = false;
        return false;
    }

    active_ = true;

    SetEnable(false);

    // ClaimUart() already brings the hardware up at GetDefaultBaudrate() so we can talk to the
    // receiver. Do not also route through comms_manager.SetBaudRate() here: that call persists its
    // argument into settings_manager.settings.baud_rates[kGNSSUART], which would silently overwrite
    // the user-configured/persisted baud rate with the receiver's hardcoded probing default on every
    // boot.
    ClaimUart();
    EnableRxInterrupt();

    // Do not send anything here. Update() starts a generic receiver's passive probe immediately and
    // applies the non-blocking power-on delay only to vendor-specific initialization commands.
    healthy_ = false;
    power_on_timestamp_ms_ = get_time_since_boot_ms();
    last_init_attempt_timestamp_ms_ = power_on_timestamp_ms_;
    unhealthy_retry_count_ = 0;
    initializing_ = true;
    notify_observed_valid_ = false;
    notify_last_emitted_valid_ = false;
    notify_pending_ = false;
    notify_has_emitted_ = false;
    notify_last_timestamp_ms_ = 0;

     SetEnable(true);

    return true;
}

bool GNSSReceiver::Update() {
    if (suspended_) return true;

    if (power_enable_pending_) {
        uint32_t now_ms = get_time_since_boot_ms();
        if (now_ms - uart_ready_timestamp_ms_ < kGenericPrePowerDelayMs) return true;
        power_enable_pending_ = false;
        SetEnable(true);
    }

    // Continue parsing standard NMEA while active even if an optional vendor-specific probe or
    // configuration step failed. A valid NMEA fix is still usable in that situation.
    if (!active_) return true;

    // PPS pulses are normally wide relative to the main-loop period, so polling avoids installing a
    // second RP2040 GPIO callback (the ADS-B demodulator already owns the bank IRQ callback).
    if (config_.pps_pin != UINT16_MAX) {
        bool pps_level = gpio_get(config_.pps_pin);
        if (pps_level && !pps_last_level_) pps_count_++;
        pps_last_level_ = pps_level;
    }

    uint32_t now_ms = get_time_since_boot_ms();
    // SendInitCommands() may parse NMEA received during its probe, so establish the timestamp
    // before initialization as well as before the normal UART drain below.
    parser_.SetTimestampMs(now_ms);
    if (initializing_) {
        uint32_t power_on_delay_ms =
            module_type_ == BSP::kGNSSModuleGeneric ? 0 : kVendorPowerOnDelayMs;
        if (now_ms - power_on_timestamp_ms_ < power_on_delay_ms) return true;
        initializing_ = false;

        // This is the first communication with the receiver after power was asserted.
        switch (module_type_) {
            case BSP::kGNSSModuleGeneric:
                // Passively probe the generic receiver for standard NMEA-0183 messages.
                healthy_ = SendInitCommands();
                break;
            case BSP::kGNSSModuleUbloxMAXM10:
                // Configure the u-blox module and enable the NMEA messages used by the parser.
                // SendInitCommands() also performs the bounded UBX liveness check.
                healthy_ = SendInitCommands();
                break;
            case BSP::kGNSSModuleNone:
            default:
                // NONE returned before touching hardware. Keep this defensive branch for invalid or
                // newly-added types that do not yet have an explicit initialization policy.
                healthy_ = false;
                active_ = false;
                return false;
        }

        if (!healthy_) {
            CONSOLE_WARNING("GNSSReceiver::Update",
                            "GNSS module did not respond; continuing to listen for NMEA messages.");
        } else {
            CONSOLE_INFO("GNSSReceiver::Update", "GNSS module configured at %lu baud.",
                         static_cast<unsigned long>(GetDefaultBaudrate()));
        }

        // Initialization/probing can consume measurable time. Refresh the clock used for fix
        // freshness and notification rate limiting before continuing this update.
        now_ms = get_time_since_boot_ms();
        last_init_attempt_timestamp_ms_ = now_ms;
    } else if (active_ && !healthy_ && unhealthy_retry_count_ < kMaxUnhealthyRetries &&
               now_ms - last_init_attempt_timestamp_ms_ >= kUnhealthyRetryIntervalMs) {
        // The initial probe (in the initializing_ block above) only gets one early window right
        // after power-on. A module that's still finishing its cold start past that window would
        // otherwise be permanently marked unhealthy for the rest of this boot even once it starts
        // answering. Retry periodically, bounded by kMaxUnhealthyRetries, so it can still be
        // detected and configured (e.g. NMEA sentence selection, AssistNow) once it wakes up.
        unhealthy_retry_count_++;
        healthy_ = SendInitCommands();
        if (healthy_) {
            CONSOLE_INFO("GNSSReceiver::Update",
                         "GNSS module responded on retry %lu/%lu; configured at %lu baud.",
                         static_cast<unsigned long>(unhealthy_retry_count_),
                         static_cast<unsigned long>(kMaxUnhealthyRetries),
                         static_cast<unsigned long>(GetDefaultBaudrate()));
        } else {
            CONSOLE_WARNING("GNSSReceiver::Update", "GNSS module still not responding (retry %lu/%lu).",
                            static_cast<unsigned long>(unhealthy_retry_count_),
                            static_cast<unsigned long>(kMaxUnhealthyRetries));
        }

        // SendInitCommands() may parse NMEA/consume measurable time; refresh the clock as above.
        now_ms = get_time_since_boot_ms();
        last_init_attempt_timestamp_ms_ = now_ms;
    }

    // Stamp the parser with the current time so applied fixes carry a freshness timestamp.
    parser_.SetTimestampMs(now_ms);

    // Normally the IRQ has already moved every byte into the software ring. Poll the hardware FIFO
    // as a safety net in case an IRQ was missed, masked, or left pending incorrectly.
    PollUartIntoRxBuffer();

    // Drain all currently available bytes from the GNSS UART.
    char c;
    while (ReadBufferedByte(c)) {
        NMEAParser::SentenceType result = parser_.IngestByte(c);
        // TEMPORARY: also feed the byte to the receiver-specific binary sniffer (UBX for ublox).
        DebugIngestByte(c);
        // TEMPORARY debug instrumentation (remove once root cause found).
        debug_total_rx_bytes_++;
        if (result != NMEAParser::kSentenceNone) {
            debug_last_sentence_ = result;
            if (result == NMEAParser::kSentenceGGA) debug_gga_count_++;
            if (result == NMEAParser::kSentenceRMC) debug_rmc_count_++;
            if (result == NMEAParser::kSentenceChecksumFail) debug_cksum_fail_count_++;
        }
    }

    // TEMPORARY periodic debug print (remove once root cause found). Uses ungated CONSOLE_PRINTF so
    // it shows regardless of the configured log level.
    if (now_ms - debug_last_print_timestamp_ms_ >= kDebugPrintIntervalMs) {
        debug_last_print_timestamp_ms_ = now_ms;
        const GNSSFix& f = parser_.fix();
        CONSOLE_INFO(
            "GNSSReceiver::Update",
            "GNSS dbg: healthy=%d rx_bytes=%lu rx_buffered=%u rx_overflow=%lu rx_irqs=%lu rx_polled=%lu "
            "last_sentence=%d gga=%lu rmc=%lu cksum_fail=%lu | fix: valid=%d q=%u sats=%u lat=%.5f lon=%.5f "
            "age_ms=%lu hasfix=%d\r\n",
            healthy_, static_cast<unsigned long>(debug_total_rx_bytes_),
            static_cast<unsigned int>(static_cast<uint16_t>(rx_head_ - rx_tail_)),
            static_cast<unsigned long>(rx_overflow_count_), static_cast<unsigned long>(rx_irq_count_),
            static_cast<unsigned long>(rx_polled_byte_count_), static_cast<int>(debug_last_sentence_),
            static_cast<unsigned long>(debug_gga_count_), static_cast<unsigned long>(debug_rmc_count_),
            static_cast<unsigned long>(debug_cksum_fail_count_), f.valid, f.fix_quality, f.num_satellites,
            f.latitude_deg, f.longitude_deg,
            static_cast<unsigned long>(now_ms - f.last_update_timestamp_ms), HasValidFix());
    }

    // TEMPORARY: periodically dump module RF/antenna status (ublox UBX-MON-RF) to diagnose 0-sats.
    // Throttled harder than the counter print because polling UBX briefly competes with the NMEA drain.
    if (healthy_ && now_ms - debug_last_rf_dump_timestamp_ms_ >= kDebugRfDumpIntervalMs) {
        debug_last_rf_dump_timestamp_ms_ = now_ms;
        DebugDumpModuleStatus();
    }

    bool current_fix_valid = HasValidFix();
    if (current_fix_valid != notify_observed_valid_) {
        notify_observed_valid_ = current_fix_valid;
        notify_pending_ = !notify_has_emitted_ || current_fix_valid != notify_last_emitted_valid_;
    }
    if (settings_manager.settings.gnss_notify && notify_pending_ &&
        (!notify_has_emitted_ || now_ms - notify_last_timestamp_ms_ >= kFixNotifyMinIntervalMs)) {
        const GNSSFix& f = parser_.fix();
        char utc_time[9] = "--:--:--";
        if (f.utc_time_valid) {
            snprintf(utc_time, sizeof(utc_time), "%02u:%02u:%02u", f.utc_hour, f.utc_minute, f.utc_second);
        }
        CONSOLE_PRINTF("GNSS_FIX=%d,%.6f,%.6f,%ld,%.1f,%ld,%u,%s,%lu\r\n", current_fix_valid,
                       f.latitude_deg, f.longitude_deg, static_cast<long>(f.altitude_ft), f.heading_deg,
                       static_cast<long>(f.speed_kts), static_cast<unsigned int>(f.num_satellites), utc_time,
                       static_cast<unsigned long>(pps_count()));
        notify_last_emitted_valid_ = current_fix_valid;
        notify_has_emitted_ = true;
        notify_pending_ = false;
        notify_last_timestamp_ms_ = now_ms;
    }
    return true;
}

void GNSSReceiver::SuspendForUartHandover() {
    // Nothing to park if the module never came up: its pins aren't driving UART0 with real traffic,
    // and skipping avoids stalling the flash path on boards with no GNSS.
    if (!active_ || suspended_) return;
    suspended_ = true;
    // Stop UART0 RX IRQ activity and discard queued GNSS bytes before the ESP32 flasher re-routes
    // the same peripheral. The flasher uses polling and must have sole ownership of UART0.
    DisableRxInterrupt();
    // De-mux the GNSS pins from uart0 so the module can no longer drive UART0 RX. On RP2040 a
    // GPIO's function select determines which peripheral input can see it; leaving GPIO 1 on
    // GPIO_FUNC_UART would keep feeding the module's byte stream into the shared UART0 RX FIFO and
    // corrupt the ESP-ROM bootloader handshake once the flasher claims GPIO 16/17. Park the pins
    // as plain SIO inputs. The module stays powered (enable pin untouched) so its BBR stays warm.
    gpio_set_function(config_.uart_rx_pin, GPIO_FUNC_SIO);
    gpio_set_function(config_.uart_tx_pin, GPIO_FUNC_SIO);
    gpio_set_dir(config_.uart_rx_pin, GPIO_IN);
    gpio_set_dir(config_.uart_tx_pin, GPIO_IN);
}

void GNSSReceiver::ResumeAfterUartHandover() {
    if (!suspended_) return;  // Not handed over.
    // Re-claim the GNSS pins and re-init uart0 (the flasher's DeInit() called uart_deinit(uart0)).
    // See the comment in Init(): ClaimUart() already sets the hardware baud, so don't also route
    // through comms_manager.SetBaudRate() here and clobber the persisted setting.
    ClaimUart();
    EnableRxInterrupt();
    // The module kept running throughout, so no re-power/boot delay is needed. Re-assert only the
    // runtime message-output config (cheap, non-destructive) so NMEA output resumes.
    ResendRuntimeConfig();
    suspended_ = false;
}

void GNSSReceiver::SetEnable(bool enabled) {
    if (!enabled) {
        power_enable_pending_ = false;
        DisableRxInterrupt();
    }
    pps_count_ = 0;
    active_ = enabled;
    if (!enabled) initializing_ = false;
    if (config_.enable_pin == UINT16_MAX) return;  // Not connected.
    gpio_init(config_.enable_pin);
    gpio_set_dir(config_.enable_pin, GPIO_OUT);
    // Active low: driving the pin low turns on the high-side power switch (module powered).
    gpio_put(config_.enable_pin, !enabled);
}

void GNSSReceiver::EnableRxInterrupt() {
    const uint irq_num = config_.uart_handle == uart0 ? UART0_IRQ : UART1_IRQ;
    const uint irq_index = config_.uart_handle == uart0 ? 0 : 1;

    irq_set_enabled(irq_num, false);
    uart_set_irq_enables(config_.uart_handle, false, false);
    rx_head_ = 0;
    rx_tail_ = 0;
    rx_overflow_count_ = 0;
    rx_irq_count_ = 0;
    rx_polled_byte_count_ = 0;
    rx_irq_owner_ = this;

    if (!rx_irq_handler_installed_[irq_index]) {
        irq_set_exclusive_handler(irq_num, RxIRQHandler);
        rx_irq_handler_installed_[irq_index] = true;
    }
    uart_set_irq_enables(config_.uart_handle, true, false);
    irq_set_enabled(irq_num, true);
}

void GNSSReceiver::DisableRxInterrupt() {
    if (rx_irq_owner_ != this) return;
    const uint irq_num = config_.uart_handle == uart0 ? UART0_IRQ : UART1_IRQ;
    uart_set_irq_enables(config_.uart_handle, false, false);
    irq_set_enabled(irq_num, false);
    rx_irq_owner_ = nullptr;
    rx_head_ = 0;
    rx_tail_ = 0;
}

bool GNSSReceiver::ReadBufferedByte(char& c) {
    uint16_t tail = rx_tail_;
    if (tail == rx_head_) return false;
    c = rx_buffer_[tail & kRxBufferMask];
    rx_tail_ = static_cast<uint16_t>(tail + 1);
    return true;
}

void GNSSReceiver::PushRxByte(char c) {
    uint16_t head = rx_head_;
    if (static_cast<uint16_t>(head - rx_tail_) >= kRxBufferSize) {
        rx_overflow_count_ = rx_overflow_count_ + 1;
        return;  // Preserve queued data and drop only the new byte on overflow.
    }
    rx_buffer_[head & kRxBufferMask] = c;
    rx_head_ = static_cast<uint16_t>(head + 1);
}

void GNSSReceiver::PollUartIntoRxBuffer() {
    if (rx_irq_owner_ != this) return;
    const uint irq_num = config_.uart_handle == uart0 ? UART0_IRQ : UART1_IRQ;

    // The ISR and this fallback both produce into the ring. Mask the IRQ during the short FIFO
    // drain so their head-index updates cannot interleave.
    irq_set_enabled(irq_num, false);
    while (uart_is_readable(config_.uart_handle)) {
        PushRxByte(static_cast<char>(uart_getc(config_.uart_handle)));
        rx_polled_byte_count_++;
    }
    irq_set_enabled(irq_num, true);
}

void GNSSReceiver::RxIRQHandler() {
    GNSSReceiver* receiver = rx_irq_owner_;
    if (receiver == nullptr) return;
    receiver->rx_irq_count_ = receiver->rx_irq_count_ + 1;

    while (uart_is_readable(receiver->config_.uart_handle)) {
        receiver->PushRxByte(static_cast<char>(uart_getc(receiver->config_.uart_handle)));
    }
}

bool GNSSReceiver::HasValidFix() const {
    if (!active_) return false;
    const GNSSFix& f = parser_.fix();
    if (!f.valid) return false;
    return (get_time_since_boot_ms() - f.last_update_timestamp_ms) < kFixStaleTimeoutMs;
}
