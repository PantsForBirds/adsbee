#include "comms.hh"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hal.hh"

/* clang-format off */
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(inc/hw_memmap.h)
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)
#include DeviceFamily_constructPath(driverlib/uart.h)
#include <ti/drivers/dpl/HwiP.h>
/* clang-format on */

static const CommsManager::ReportSink kReportingSinks[] = {SettingsManager::SerialInterface::kConsole};
static const uint16_t kNumReportingSinks = sizeof(kReportingSinks) / sizeof(CommsManager::ReportSink);

// Margin added to every baud-derived TX wait so tiny shortfalls don't get a zero-length budget.
static const uint32_t kTxWaitMarginMs = 5;

CommsManager::CommsManager(CommsManagerConfig config)
    : config_(config), at_parser_(CppAT(at_command_list, at_command_list_num_commands, true)) {}

void CommsManager::uart_write_callback(UART2_Handle handle, void* buf, size_t count, void* userArg,
                                       int_fast16_t status) {
    // HWI context. Retire the bytes the driver actually consumed (count can be short if the write was cancelled),
    // then chain the next contiguous ring segment straight from here so the wire never idles between segments. The
    // UART2CC26X2 driver clears its writeInUse flag before invoking this callback, so a nested UART2_write is accepted.
    CommsManager* self = static_cast<CommsManager*>(userArg);
    self->uart_tx_head_ = static_cast<uint16_t>((self->uart_tx_head_ + count) & (kUartTxRingBytes - 1));
    self->uart_tx_in_progress_ = false;
    if (status == UART2_STATUS_SUCCESS) {
        self->KickTx();
    }
    // On cancel/error, leave the ring alone: SetBaudRate() resets it after close, and Update()'s safety re-kick covers
    // anything else.
}

void CommsManager::KickTx() {
    uint16_t head = uart_tx_head_;
    uint16_t tail = uart_tx_tail_;
    if (head == tail) {
        uart_tx_in_progress_ = false;
        return;  // Nothing queued.
    }
    // Longest contiguous run starting at head (stop at the end of the ring; the callback chains the wrapped part).
    size_t count = tail > head ? static_cast<size_t>(tail - head) : static_cast<size_t>(kUartTxRingBytes - head);
    uart_tx_in_progress_ = true;
    // The driver internally restarts DMA in <=1024 B pieces until the whole count is out, so no chunking here.
    int_fast16_t status = UART2_write(uart_handle_, &uart_tx_ring_[head], count, nullptr);
    if (status != UART2_STATUS_SUCCESS) {
        uart_tx_in_progress_ = false;  // Update() / the next iface_write will retry.
    }
}

bool CommsManager::WaitForTxRingSpace(uint16_t num_bytes) {
    if (TxRingFreeBytes() >= num_bytes) {
        return true;
    }
    uart_tx_stall_count++;
    // Budget: time for the shortfall to clock out at the current baud rate, doubled, plus margin. Bounded so a wedged
    // UART degrades to dropped output rather than a frozen main loop.
    uint32_t shortfall = num_bytes - TxRingFreeBytes();
    uint32_t deadline_ms = get_time_since_boot_ms() + 2 * TxBytesToMs(shortfall) + kTxWaitMarginMs;
    while (TxRingFreeBytes() < num_bytes && get_time_since_boot_ms() < deadline_ms) {
        // Safety net: if a write callback was ever lost, restart the drain instead of timing out.
        if (!uart_tx_in_progress_ && uart_tx_head_ != uart_tx_tail_) {
            uintptr_t key = HwiP_disable();
            if (!uart_tx_in_progress_) {
                KickTx();
            }
            HwiP_restore(key);
        }
    }
    return TxRingFreeBytes() >= num_bytes;
}

bool CommsManager::OpenUART(uint32_t baud) {
    UART2_Params uart_params;
    UART2_Params_init(&uart_params);
    uart_params.baudRate = baud;
    uart_params.writeMode = UART2_Mode_CALLBACK;
    uart_params.writeCallback = uart_write_callback;
    uart_params.userArg = this;
    uart_handle_ = UART2_open(config_.uart_index, &uart_params);
    if (uart_handle_ == NULL) {
        return false;
    }
    UART2_rxEnable(uart_handle_);
    return true;
}

bool CommsManager::Init() {
    if (!OpenUART(config_.uart_baud_rate)) {
        // Never spin here: with no console there is no way to report the failure, and a silent hang is
        // indistinguishable from a dead board (and unrecoverable, since the app is what the recovery
        // tooling talks to). Reset instead -- the resulting reboot loop is visible to the programmer jig.
        SysCtrlSystemReset();
    }
    return true;
}

bool CommsManager::SetBaudRate(uint32_t baud) {
    if (!IsAllowedBaudRate(baud)) {
        return false;
    }
    if (baud == config_.uart_baud_rate) {
        return true;
    }

    // Let any queued response (e.g. the "OK" for AT+BAUD_RATE) reach the host intact at the old baud
    // before the line reconfigures.
    DrainConsoleTx();

    // The driver requires an unfinished asynchronous write to be cancelled before UART2_close().
    if (uart_tx_in_progress_) {
        UART2_writeCancel(uart_handle_);
    }
    UART2_close(uart_handle_);
    // Whatever didn't make it out is gone with the old baud rate; start the ring clean. No callback can fire after
    // close, so plain assignments are safe here.
    uart_tx_head_ = 0;
    uart_tx_tail_ = 0;
    uart_tx_in_progress_ = false;
    config_.uart_baud_rate = baud;
    if (!OpenUART(baud)) {
        // Deterministic params make reopen failure effectively impossible, but fall back to the
        // boot default rather than hanging the console.
        config_.uart_baud_rate = SettingsManager::Settings::kDefaultUARTBaudRate;
        if (!OpenUART(config_.uart_baud_rate)) {
            // Reset rather than spin. This path is reachable at boot whenever a non-default console baud is
            // persisted (SettingsManager::Apply() -> SetBaudRate()), so a hang here bricks the console at
            // every rate, permanently, with no way in.
            SysCtrlSystemReset();
        }
    }
    // Discard any garbage clocked in at the mismatched rate (e.g. a trailing newline from the host).
    UART2_flushRx(uart_handle_);

    // Mirror the live value so settings queries display it; AT+SETTINGS=SAVE persists it and
    // SettingsManager::Apply() re-applies it at boot.
    settings_manager.settings.baud_rates[SettingsManager::SerialInterface::kConsole] = config_.uart_baud_rate;
    return baud == config_.uart_baud_rate;
}

bool CommsManager::DrainConsoleTx(uint32_t timeout_margin_ms) {
    // Each stage gets its own budget rather than sharing one deadline: the stages wait on unrelated
    // events, and a slow first stage must not starve the second. The hardware stage is the one that
    // matters for STANDBY (it gates the UART's power constraint), so it is exactly the one that must
    // not be skipped after a long DMA wait.

    // Software side: wait for the TX ring to empty and the last CALLBACK-mode write to complete. Budget
    // is what the queued bytes need at the current baud rate (doubled) plus the caller's margin.
    uint32_t deadline_ms =
        get_time_since_boot_ms() + 2 * TxBytesToMs(TxRingUsedBytes() + kPrintfBufferMaxSize) + timeout_margin_ms;
    while ((uart_tx_in_progress_ || uart_tx_head_ != uart_tx_tail_) && get_time_since_boot_ms() < deadline_ms) {
        // Safety net: restart the drain if a write callback was ever lost.
        if (!uart_tx_in_progress_) {
            uintptr_t key = HwiP_disable();
            if (!uart_tx_in_progress_) {
                KickTx();
            }
            HwiP_restore(key);
        }
    }
    // Hardware side: the write callback fires on DMA completion, not when the bytes have left the
    // wire — up to a FIFO's worth can still be in flight. UARTBusy() stays set until the last stop
    // bit is shifted out.
    deadline_ms = get_time_since_boot_ms() + timeout_margin_ms;
    while (UARTBusy(UART0_BASE) && get_time_since_boot_ms() < deadline_ms) {
    }
    return !uart_tx_in_progress_ && uart_tx_head_ == uart_tx_tail_ && !UARTBusy(UART0_BASE);
}

bool CommsManager::Suspend() {
    // Release the PowerCC26XX_DISALLOW_STANDBY constraint that UART2_rxEnable() holds while RX is on,
    // so the MCU can reach STANDBY. TX is left intact so console logging still flushes before sleep.
    UART2_rxDisable(uart_handle_);
    return true;
}

bool CommsManager::Resume() {
    // Re-arm console UART reception after wake.
    UART2_rxEnable(uart_handle_);
    return true;
}

bool CommsManager::Update() {
    // Safety net for the TX ring: if a kick failed (UART2_write rejected) or a callback was lost, restart the drain.
    // In normal operation the write callback chains segments itself and this branch is never taken.
    if (!uart_tx_in_progress_ && uart_tx_head_ != uart_tx_tail_) {
        uintptr_t key = HwiP_disable();
        if (!uart_tx_in_progress_) {
            KickTx();
        }
        HwiP_restore(key);
    }

    UpdateAT();

    uint32_t timestamp_ms = get_time_since_boot_ms();
    if (timestamp_ms - last_raw_report_check_timestamp_ms_ > kRawReportingCheckIntervalMs) {
        last_raw_report_check_timestamp_ms_ = timestamp_ms;  // Proceed with update and record timestamp.

        // Calculate how much buffer space the current packets would need.
        uint16_t required_buffer_len = CompositeArray::CalculateRawPacketsBufferLength(
            &mode_s_packet_reporting_queue, &uat_adsb_packet_reporting_queue, &uat_uplink_packet_reporting_queue);

        // Only forward packets if buffer would be full or if max reporting interval has elapsed.
        bool buffer_would_be_full = required_buffer_len >= CompositeArray::RawPackets::kMaxLenBytes;
        bool max_interval_elapsed = (timestamp_ms - last_raw_report_timestamp_ms_) >= kRawReportingMaxIntervalMs;

        if (buffer_would_be_full || max_interval_elapsed) {
            // Update the last report timestamp now that we're actually sending packets.
            last_raw_report_timestamp_ms_ = timestamp_ms;

            // Don't deplete the packet queues until we are ready to report!
            uint8_t packets_to_report_buf[CompositeArray::RawPackets::kMaxLenBytes] = {0};
            CompositeArray::RawPackets packets_to_report = CompositeArray::PackRawPacketsBuffer(
                packets_to_report_buf, sizeof(packets_to_report_buf), &mode_s_packet_reporting_queue,
                &uat_adsb_packet_reporting_queue, &uat_uplink_packet_reporting_queue);

            // Interfaces to send reports on.
            UpdateReporting(kReportingSinks, settings_manager.settings.reporting_protocols, kNumReportingSinks,
                            &packets_to_report);
        }
    }
    return true;
}

int CommsManager::console_printf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    int res = console_vprintf(format, args);
    va_end(args);
    return res;
}

int CommsManager::console_level_printf(SettingsManager::LogLevel level, const char* format, ...) {
    if (settings_manager.settings.log_level < level) return 0;
    va_list args;
    va_start(args, format);
    int res = console_vprintf(format, args);
    va_end(args);
    return res;
}

// int CommsManager::console_printf(const char* fmt, ...) {
//     char string[255];
//     va_list args;
//     va_start(args, fmt);
//     uint16_t len = vsprintf(string, fmt, args);
//     va_end(args);
//     uint16_t len_written = UART2_write(uart_handle_, string, len);
//     return len_written;
// }

int CommsManager::console_vprintf(const char* fmt, va_list args) {
    char buf[kPrintfBufferMaxSize];
    int len = vsnprintf(buf, kPrintfBufferMaxSize, fmt, args);
    if (len <= 0) return len;
    // Queued into the TX ring and sent in the background; callers that need the text on the wire (reboot, sleep,
    // baud change) call DrainConsoleTx().
    return iface_puts(SettingsManager::SerialInterface::kConsole, buf) ? len : -1;
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
    if (iface_puts(iface, buf)) {
        return res;  // Return number of characters written.
    }

    return -1;  // puts failed.
}

bool CommsManager::iface_write(SettingsManager::SerialInterface iface, const void* buf, size_t len, bool blocking) {
    switch (iface) {
        case SettingsManager::kConsole: {
            (void)blocking;  // Data is copied into the ring; completion is never awaited here (see header).
            if (uart_handle_ == nullptr) {
                return false;  // Console not open yet (e.g. a static-init error print).
            }
            if (len == 0) {
                return true;
            }
            const uint8_t* p = static_cast<const uint8_t*>(buf);
            size_t remaining = len;
            while (remaining > 0) {
                // A write is normally queued whole (all-or-nothing, so a dropped write never leaves a partial frame
                // on the wire). Only a write larger than the ring itself -- not reachable with the 2 kB composite
                // array cap -- is fed through in ring-sized pieces.
                uint16_t chunk = remaining < static_cast<size_t>(kUartTxRingBytes - 1)
                                     ? static_cast<uint16_t>(remaining)
                                     : static_cast<uint16_t>(kUartTxRingBytes - 1);
                if (!WaitForTxRingSpace(chunk)) {
                    uart_tx_drop_count++;
                    return false;  // Link oversubscribed; drop rather than stall the receiver.
                }

                // Copy outside the critical section: [tail, tail + chunk) is exclusively the producer's -- the
                // consumer only ever reads up to the published tail, and free space can only grow underneath us.
                uint16_t tail = uart_tx_tail_;
                uint16_t first = static_cast<uint16_t>(kUartTxRingBytes - tail);
                if (first > chunk) {
                    first = chunk;
                }
                memcpy(&uart_tx_ring_[tail], p, first);
                if (chunk > first) {
                    memcpy(&uart_tx_ring_[0], p + first, chunk - first);  // Wrapped remainder.
                }

                // Publish and, if the UART is idle, start it. Both under HWI-disable so the write callback (which
                // reads tail and may clear in_progress) can't interleave with the publish/kick decision.
                uintptr_t key = HwiP_disable();
                uart_tx_tail_ = static_cast<uint16_t>((tail + chunk) & (kUartTxRingBytes - 1));
                uint16_t used = TxRingUsedBytes();
                if (used > uart_tx_high_water_bytes) {
                    uart_tx_high_water_bytes = used;
                }
                if (!uart_tx_in_progress_) {
                    KickTx();
                }
                HwiP_restore(key);

                p += chunk;
                remaining -= chunk;
            }
            return true;
        }
        case SettingsManager::kNumSerialInterfaces:
        default:
            CONSOLE_WARNING("CommsManager::iface_write", "Unrecognized iface %d.", iface);
            return false;
    }
    return false;
}

bool CommsManager::iface_putc(SettingsManager::SerialInterface iface, char c, bool blocking) {
    return iface_write(iface, &c, 1, blocking);
}

bool CommsManager::iface_getc(SettingsManager::SerialInterface iface, char& c) {
    switch (iface) {
        // case SettingsManager::kCommsUART:
        //     if (uart_is_readable_within_us(config_.comms_uart_handle, config_.uart_timeout_us)) {
        //         c = uart_getc(config_.comms_uart_handle);
        //         return true;
        //     }
        //     return false;  // No chars to read.
        //     break;
        // case SettingsManager::kGNSSUART:
        //     if (uart_is_readable_within_us(config_.gnss_uart_handle, config_.uart_timeout_us)) {
        //         c = uart_getc(config_.gnss_uart_handle);
        //         return true;
        //     }
        //     return false;  // No chars to read.
        //     break;
        case SettingsManager::kConsole: {
            if (UART2_getRxCount(uart_handle_) == 0) {
                return false;  // No chars to read.
            }
            size_t bytes_read;
            int_fast16_t status = UART2_read(uart_handle_, &c, 1, &bytes_read);
            if (status == UART2_STATUS_SUCCESS && bytes_read == 1) {
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

bool CommsManager::iface_puts(SettingsManager::SerialInterface iface, const char* buf, bool blocking) {
    return iface_write(iface, buf, strlen(buf), blocking);
}