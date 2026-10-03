#include "comms.hh"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "beast_utils.hh"
#include "gdl90_utils.hh"
#include "hal.hh"
#include "raw_utils.hh"

/* clang-format off */
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(inc/hw_memmap.h)
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)
#include DeviceFamily_constructPath(driverlib/uart.h)
#include DeviceFamily_constructPath(driverlib/cpu.h)
#include <ti/drivers/GPIO.h>
#include <ti/drivers/dpl/HwiP.h>
/* clang-format on */

static const CommsManager::ReportSink kReportingSinks[] = {SettingsManager::SerialInterface::kConsole};
static const uint16_t kNumReportingSinks = sizeof(kReportingSinks) / sizeof(CommsManager::ReportSink);

// Worst-case console bytes one ReportQueuedRawPackets() chunk can produce. RAW is the largest encoding of every
// packet type, so the RAW sizes also cover Beast and GDL90 uplink pass-through.
static_assert(BeastReporter::kModeSBeastFrameMaxLenBytes <= kRawModeSFrameMaxNumChars &&
                  BeastReporter::kUATADSBBeastFrameMaxLenBytes <= kRawUATADSBFrameMaxNumChars &&
                  BeastReporter::kUATUplinkBeastFrameMaxLenBytes <= kRawUATUplinkFrameMaxNumChars &&
                  GDL90Reporter::kGDL90MessageMaxLenBytes <= kRawUATUplinkFrameMaxNumChars,
              "RAW must be the largest raw packet encoding.");
static constexpr uint16_t kRawReportChunkMaxTxBytes =
    CommsManager::kRawReportMaxModeSPacketsPerUpdate * kRawModeSFrameMaxNumChars +
    CommsManager::kRawReportMaxUATADSBPacketsPerUpdate * kRawUATADSBFrameMaxNumChars +
    CommsManager::kRawReportMaxUATUplinkPacketsPerUpdate * kRawUATUplinkFrameMaxNumChars;
static_assert(kRawReportChunkMaxTxBytes < CommsManager::kUartTxRingBytes, "A report chunk must fit in the TX ring.");

// Margin added to every baud-derived TX wait so tiny shortfalls don't get a zero-length budget.
static const uint32_t kTxWaitMarginMs = 5;
// Caps on the baud-derived waits, so low console rates don't stall reception or trip the watchdog. Output that doesn't
// fit is dropped.
static const uint32_t kTxRingSpaceWaitMaxMs = 250;
static const uint32_t kTxDrainWaitMaxMs = 2000;

// A host holding console RX low through a SYNC sleep gets "UU" on wake, like a break. 5 ms is far longer than any low
// stretch in console traffic. CPUdelay() runs 4 cycles per loop at 48 MHz.
static const uint32_t kWakeTriggerLowUs = 5000;
static const uint32_t kWakeTriggerPollUs = 100;

// True if the console RX line stays low for timeout_us.
static bool ConsoleRxHeldLow(uint32_t timeout_us) {
    for (uint32_t waited_us = 0;; waited_us += kWakeTriggerPollUs) {
        if (GPIO_read(bsp.kSubGUARTRXPin) != 0) return false;
        if (waited_us >= timeout_us) return true;
        CPUdelay(kWakeTriggerPollUs * 48 / 4);
    }
}

// Break detection. The UART2 driver never enables the break interrupt, so poll the raw interrupt status, which latches
// a break regardless. The driver never clears this bit itself.
static inline bool ConsoleBreakSeen() { return (HWREG(UART0_BASE + UART_O_RIS) & UART_INT_BE) != 0; }
static inline void ClearConsoleBreak() { HWREG(UART0_BASE + UART_O_ICR) = UART_INT_BE; }

CommsManager::CommsManager(CommsManagerConfig config)
    : config_(config), at_parser_(CppAT(at_command_list, at_command_list_num_commands, true)) {}

void CommsManager::uart_write_callback(UART2_Handle handle, void* buf, size_t count, void* userArg,
                                       int_fast16_t status) {
    // HWI context. Retire the bytes the driver actually consumed (count can be short if the write was canceled),
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
    uint32_t wait_ms = 2 * TxBytesToMs(shortfall) + kTxWaitMarginMs;
    uint32_t deadline_ms =
        get_time_since_boot_ms() + (wait_ms < kTxRingSpaceWaitMaxMs ? wait_ms : kTxRingSpaceWaitMaxMs);
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
        // tooling talks to). So the firmware resets, and the resulting reboot loop is visible to the ADSBee 1421
        // Programmer.
        SysCtrlSystemReset();
    }
    return true;
}

void CommsManager::AnnounceConsoleRate() {
    rate_announced_ = true;
    iface_write(SettingsManager::SerialInterface::kConsole, ConsoleAutobaud::kAnswer, ConsoleAutobaud::kAnswerLen);
}

void CommsManager::DropQueuedConsoleTx() {
    // The write callback retires what already went out, synchronously.
    if (uart_tx_in_progress_) UART2_writeCancel(uart_handle_);
    uintptr_t key = HwiP_disable();
    // Restart at the ring's start so the "UU" that follows goes out in one write, unsplit.
    if (!uart_tx_in_progress_) uart_tx_head_ = 0;
    uart_tx_tail_ = uart_tx_head_;
    HwiP_restore(key);
}

void CommsManager::AnswerConsoleBreak() {
    ClearConsoleBreak();
    DropQueuedConsoleTx();
    AnnounceConsoleRate();
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

    // The driver requires an unfinished asynchronous write to be canceled before UART2_close().
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
    ClearConsoleBreak();  // Bytes at the old rate can read as a break at a much higher one.
    AnnounceConsoleRate();

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
    uint32_t wait_ms = 2 * TxBytesToMs(TxRingUsedBytes() + kPrintfBufferMaxSize);
    uint32_t deadline_ms =
        get_time_since_boot_ms() + (wait_ms < kTxDrainWaitMaxMs ? wait_ms : kTxDrainWaitMaxMs) + timeout_margin_ms;
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
    // A host holding RX low gets "UU" on wake (Resume()). It can't read queued output yet, so drop it.
    wake_trigger_ = ConsoleRxHeldLow(kWakeTriggerLowUs);
    if (wake_trigger_) DropQueuedConsoleTx();
    return true;
}

bool CommsManager::Resume() {
    // Re-arm console UART reception after wake.
    UART2_rxEnable(uart_handle_);
    if (wake_trigger_ || ConsoleRxHeldLow(kWakeTriggerLowUs)) AnswerConsoleBreak();
    wake_trigger_ = false;
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

    if (ConsoleBreakSeen()) AnswerConsoleBreak();
    UpdateAT();
    ReportQueuedRawPackets();

    uint32_t timestamp_ms = get_time_since_boot_ms();
    if (timestamp_ms - last_raw_report_check_timestamp_ms_ > kRawReportingCheckIntervalMs) {
        last_raw_report_check_timestamp_ms_ = timestamp_ms;
        // Raw packets went out in ReportQueuedRawPackets(); this tick only drives the aircraft dictionary protocols.
        uint8_t no_packets_buf[sizeof(CompositeArray::RawPackets::Header)];
        CompositeArray::RawPackets no_packets = CompositeArray::PackRawPacketsBuffer(
            no_packets_buf, sizeof(no_packets_buf), nullptr, nullptr, nullptr);
        UpdateReporting(kReportingSinks, settings_manager.settings.reporting_protocols, kNumReportingSinks,
                        &no_packets);
    }
    return true;
}

bool CommsManager::ReportsRawPackets() const {
    for (uint16_t i = 0; i < kNumReportingSinks; i++) {
        switch (settings_manager.settings.reporting_protocols[kReportingSinks[i]]) {
            case SettingsManager::kRaw:
            case SettingsManager::kBeast:
            case SettingsManager::kBeastNoUAT:
            case SettingsManager::kBeastNoUATUplink:
            case SettingsManager::kGDL90:  // Passes UAT uplinks through.
                return true;
            default:
                break;
        }
    }
    return false;
}

void CommsManager::ReportQueuedRawPackets() {
    if (mode_s_packet_reporting_queue.Length() == 0 && uat_adsb_packet_reporting_queue.Length() == 0 &&
        uat_uplink_packet_reporting_queue.Length() == 0) {
        return;
    }
    if (ReportsRawPackets() && TxRingFreeBytes() < kRawReportChunkMaxTxBytes) {
        // The link is behind. Leave the packets queued rather than wait for ring space inside iface_write: the Mode S
        // queue overwrites its oldest entry when full (report_q_ovf) and the UAT queues refuse new entries
        // (uat_report_q_ovf), so an oversubscribed link costs reports but never stalls the receiver.
        return;
    }

    // Format at most one chunk per main loop iteration. The rest stays queued for the following iterations.
    alignas(4) uint8_t chunk_buf[kRawReportChunkBufBytes];
    CompositeArray::RawPackets chunk = CompositeArray::PackRawPacketsBuffer(
        chunk_buf, sizeof(chunk_buf), &mode_s_packet_reporting_queue, &uat_adsb_packet_reporting_queue,
        &uat_uplink_packet_reporting_queue, nullptr,
        CompositeArray::PackLimits(kRawReportMaxModeSPacketsPerUpdate, kRawReportMaxUATADSBPacketsPerUpdate,
                                   kRawReportMaxUATUplinkPacketsPerUpdate));
    UpdateReporting(kReportingSinks, settings_manager.settings.reporting_protocols, kNumReportingSinks, &chunk,
                    false);
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
            while (UART2_getRxCount(uart_handle_) != 0) {
                size_t bytes_read;
                int_fast16_t status = UART2_read(uart_handle_, &c, 1, &bytes_read);
                if (status != UART2_STATUS_SUCCESS || bytes_read != 1) {
                    return false;  // Failed to read character.
                }
                // A break's NUL: answer the break now if Update() hasn't yet.
                if (c == '\0' && ConsoleBreakSeen()) AnswerConsoleBreak();
                if (!ConsoleAutobaud::IgnoredConsoleByte(c)) return true;
            }
            return false;  // No chars to read.
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