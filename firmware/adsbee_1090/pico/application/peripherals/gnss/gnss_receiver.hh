#pragma once

#include "bsp.hh"
#include "hardware/irq.h"
#include "hardware/uart.h"  // For uart_inst_t and UART bring-up in Init().
#include "nmea_parser.hh"
#include "pps_clock.hh"
#include "pico/stdlib.h"  // For gpio_* and tight_loop_contents() used by subclasses/impl.
#include "settings.hh"

/**
 * Abstract base class for a GNSS receiver that emits NMEA over UART.
 *
 * Owns the generic NMEA parser and the UART read loop, and manages the (optional) active-low
 * power-enable pin. Receiver-specific behavior (default baud rate, vendor configuration commands,
 * boot delay) is provided by subclasses (e.g. UbloxMAXM10).
 *
 * Received UART bytes are captured by an interrupt into a software ring buffer and parsed from the
 * main loop. UART ownership is released while the shared peripheral is used to flash the ESP32.
 */
class GNSSReceiver {
   public:
    // Reads the UART RX FIFO into the receiver's ring. Called by CommsManager's UART interrupt handler.
    static void RxIRQHandler();

    using GNSSFix = NMEAParser::GNSSFix;

    // A fix is considered current only if it was updated within this window.
    static constexpr uint32_t kFixStaleTimeoutMs = 5000;
    static constexpr uint32_t kVendorPowerOnDelayMs = 1000;
    static constexpr uint32_t kGenericPrePowerDelayMs = 100;
    static constexpr uint32_t kFixNotifyMinIntervalMs = 1000;
    // If the vendor-specific init/liveness probe fails (e.g. the module is still warming up past
    // ProbeLiveness()'s initial window), retry it periodically from Update() rather than leaving
    // healthy_ permanently false until the next reboot. Bounded by kMaxUnhealthyRetries so a
    // genuinely absent module doesn't stall the main loop indefinitely -- each retry costs up to
    // SendInitCommands()'s own worst-case blocking time (see UbloxMAXM10::kProbeRounds /
    // kProbeTimeoutMs).
    static constexpr uint32_t kUnhealthyRetryIntervalMs = 5000;
    static constexpr uint32_t kMaxUnhealthyRetries = 5;
    // Longest wait for queued TX bytes to leave before a baud change (a UBX command is a few ms; protocol output queued
    // on the GNSS UART can be more, and is cut off rather than waited for).
    static constexpr uint32_t kUartTxFlushTimeoutMs = 250;
    static constexpr uint16_t kRxBufferSize = 2048;
    static constexpr uint16_t kRxBufferMask = kRxBufferSize - 1;
    static_assert((kRxBufferSize & kRxBufferMask) == 0, "GNSS RX buffer size must be a power of two.");

    struct GNSSReceiverConfig {
        uart_inst_t* uart_handle = uart0;
        uint16_t uart_tx_pin = bsp.gnss_uart_tx_pin;  // GPIO 0
        uint16_t uart_rx_pin = bsp.gnss_uart_rx_pin;  // GPIO 1
        // Active-low high-side power switch for the module's VCC/VCC_IO. UINT16_MAX = not connected.
        uint16_t enable_pin = bsp.gnss_enable_pin;
        uint16_t pps_pin = bsp.gnss_pps_pin;
    };

    GNSSReceiver(GNSSReceiverConfig config_in, BSP::GNSSModuleType module_type)
        : config_(config_in), module_type_(module_type) {}
    virtual ~GNSSReceiver() = default;

    /**
     * Power on the module and schedule receiver initialization. Generic receivers begin listening
     * immediately; vendor-specific receivers wait one second non-blockingly before Update() sends
     * their first command.
     * @retval True if initialization was scheduled; false for NONE.
     */
    virtual bool Init();

    /**
     * Drain available bytes from the GNSS UART and feed them to the NMEA parser. Call from the
     * main loop. No-op while suspended for a UART handover (see SuspendForUartHandover()).
     * @retval True on success.
     */
    bool Update();

    /**
     * Release the GNSS pins so the shared uart0 peripheral can be re-routed to the ESP32 flasher
     * pins (GPIO 16/17) without the GNSS module's continuous NMEA/UBX stream corrupting the
     * ESP-ROM bootloader handshake. De-muxes GPIO 0/1 off uart0 (back to SIO input) so they can
     * no longer feed UART0 RX. The module is intentionally left powered so its BBR (ephemeris /
     * almanac / RTC) stays warm and it hot-starts once the pins are re-claimed. Idempotent; safe
     * to call when the module is absent/unhealthy (no-op). Update() no-ops until Resume.
     */
    void SuspendForUartHandover();

    /**
     * Re-claim GPIO 0/1 for uart0 and re-initialize the peripheral (the ESP32 flasher's DeInit()
     * calls uart_deinit(uart0)) after an ESP32 flash, then re-assert the runtime NMEA message
     * output configuration and resume Update(). Counterpart to SuspendForUartHandover().
     */
    void ResumeAfterUartHandover();

    /**
     * Enable or disable module power via the active-low enable pin. No-op if no enable pin.
     */
    void SetEnable(bool enabled);

    /**
     * @retval The most recently merged GNSS fix.
     */
    const GNSSFix& fix() const { return parser_.fix(); }

    /**
     * @retval True if the module is healthy AND has a valid, non-stale fix. This is the single
     *         signal the application uses to decide whether GNSS position is usable.
     */
    bool HasValidFix() const;

    /**
     * Altitude error estimate from the receiver's GST sentence, if one arrived recently with a valid fix.
     * @param[out] std_dev_cm Standard deviation (1 sigma) of the altitude error, in centimeters.
     * @retval True if std_dev_cm was written.
     */
    bool GetAltitudeStdDevCm(int32_t& std_dev_cm) const;

    /**
     * @retval True if the module responded during Init() (i.e. it is present and configured).
     */
    bool IsHealthy() const { return healthy_; }

    /** @retval True while the GNSS interface is powered/enabled and should consume UART data. */
    bool IsActive() const { return active_; }

    /** @retval Number of PPS rising edges observed since GNSS was last enabled or disabled. */
    uint32_t pps_count() const { return pps_count_; }

    // GNSS time on the MLAT counter, from the PPS edges (see PPSClock).
    const PPSClock& pps_clock() const { return pps_clock_; }

    /**
     * Converts a 48 MHz MLAT count (a Mode S, UAT or Remote ID timestamp) to UTC.
     * @param[out] unix_seconds UTC seconds since 1970 without leap seconds (POSIX).
     * @param[out] ns Nanoseconds into that second.
     * @retval False without a PPS fit, or if the count is more than PPSClock::kHoldoverSeconds from the last PPS.
     */
    bool MLATCountsToUTC(uint64_t mlat_48mhz_counts, uint64_t& unix_seconds, uint32_t& ns) const {
        return pps_clock_.ToUTC(mlat_48mhz_counts, unix_seconds, ns);
    }

    /**
     * Programs the GNSS UART for the stored GNSS baud setting (settings.baud_rates[kGNSSUART]); CommsManager calls it
     * whenever that setting is set. The default runs the UART at GetLinkBaudrate(). A receiver that sets its module's
     * baud rate (UbloxMAXM10) overrides it to command the module to the setting too.
     * No-op while the UART is handed over to the ESP32 flasher.
     */
    virtual void ApplyBaudRateSetting();

    /** @retval The baud rate the GNSS UART runs at for this receiver (shared by protocol output on that UART). */
    uint32_t link_baudrate() const { return GetLinkBaudrate(); }

   protected:
    /**
     * @retval The baud rate our UART runs at for this receiver: the GNSS baud setting, or for UbloxMAXM10 the rate its
     *         module answered at (the setting unless switching to it failed).
     */
    virtual uint32_t GetLinkBaudrate() const = 0;

    /** @retval The GNSS baud setting (settings.baud_rates[kGNSSUART]). */
    static uint32_t BaudRateSetting() { return settings_manager.settings.baud_rates[SettingsManager::kGNSSUART]; }

    /**
     * Changes our UART's baud rate: waits for queued TX bytes to leave at the old rate, reprograms the UART, then drops
     * whatever was received so far (bytes from before the change are framed at the wrong rate). For receivers that
     * renegotiate the link (UbloxMAXM10). The caller keeps GetLinkBaudrate() in step.
     */
    void SetUartBaudrate(uint32_t baudrate);

    /**
     * Send receiver-specific configuration (e.g. UBX-CFG-VALSET for u-blox). Called by Init()
     * after the boot delay.
     * @retval True if the module acknowledged configuration (also used as the liveness check).
     */
    virtual bool SendInitCommands() = 0;

    /**
     * Re-assert only the runtime message-output configuration (the sentences we consume), without
     * the full init/config pass. Called by ResumeAfterUartHandover() after an ESP32 flash so the
     * module keeps emitting the NMEA we need, cheaply and without disturbing stored config.
     * Default no-op; receivers that need it (e.g. UbloxMAXM10) override.
     */
    virtual void ResendRuntimeConfig() {}

    /**
     * Route the GNSS UART pins (GPIO 0/1) to uart0 and (re)initialize the peripheral to the
     * receiver's link baud (GetLinkBaudrate()). Interrupt reception is enabled separately after final baud setup.
     */
    void ClaimUart();

    /** Start/stop interrupt-driven UART reception and reset the software RX ring buffer. */
    void EnableRxInterrupt();
    void DisableRxInterrupt();

    // Labels the latest PPS edge with the RMC sentence just parsed.
    void LabelPPSEdge();

    /** Drop every byte received so far, in the hardware FIFO and the software ring. */
    void FlushRx();

    /** Pop one byte captured by the UART RX interrupt. */
    bool ReadBufferedByte(char& c);

    /** Recover any hardware-FIFO bytes not serviced by the UART interrupt. */
    void PollUartIntoRxBuffer();

    /** Add one UART byte to the software ring, dropping it and counting an overflow if full. */
    void PushRxByte(char c);

    // TEMPORARY debug hooks (remove with the rest of the GNSS debug instrumentation).
    // DebugIngestByte: fed every received UART byte so a concrete receiver can passively sniff its
    // binary protocol (e.g. ublox UBX) from the same stream the NMEA parser consumes.
    // DebugDumpModuleStatus: prints the latest sniffed module diagnostics. Defaults no-op.
    virtual void DebugIngestByte(char c) { (void)c; }
    virtual void DebugDumpModuleStatus() {}

    GNSSReceiverConfig config_;
    BSP::GNSSModuleType module_type_ = BSP::kGNSSModuleNone;
    NMEAParser parser_;
    bool healthy_ = false;
    // Kept separate from healthy_: a receiver may emit usable generic NMEA even if its optional
    // manufacturer-specific probe or configuration transaction fails.
    bool active_ = false;
    // True while the GNSS pins are released for an ESP32 flash (see SuspendForUartHandover()).
    // Update() no-ops while suspended so it doesn't touch the (re-routed) uart0.
    bool suspended_ = false;
    bool power_enable_pending_ = false;
    uint32_t uart_ready_timestamp_ms_ = 0;
    bool initializing_ = false;
    uint32_t power_on_timestamp_ms_ = 0;
    // Timestamp of the last SendInitCommands() attempt (initial or retry) and how many retries
    // (beyond the initial attempt) have been made since Init(). Reset in Init().
    uint32_t last_init_attempt_timestamp_ms_ = 0;
    uint32_t unhealthy_retry_count_ = 0;
    bool notify_observed_valid_ = false;
    bool notify_last_emitted_valid_ = false;
    bool notify_pending_ = false;
    bool notify_has_emitted_ = false;
    uint32_t notify_last_timestamp_ms_ = 0;
    bool pps_last_level_ = false;
    // Shared by all dynamically selected receiver instances and reset on every SetEnable() call.
    // Updated from the main loop, so no atomic access is required.
    static inline uint32_t pps_count_ = 0;
    PPSClock pps_clock_;

    // Single-producer (UART ISR), single-consumer (main loop) ring buffer. Monotonic 16-bit indices
    // make all 2048 slots usable; unsigned subtraction remains valid across index wrap.
    char rx_buffer_[kRxBufferSize] = {0};
    volatile uint16_t rx_head_ = 0;
    volatile uint16_t rx_tail_ = 0;
    volatile uint32_t rx_overflow_count_ = 0;
    volatile uint32_t rx_irq_count_ = 0;
    uint32_t rx_polled_byte_count_ = 0;
    static inline GNSSReceiver* rx_irq_owner_ = nullptr;

    // TEMPORARY debug instrumentation for diagnosing "GNSS NOT AVAILABLE". Remove once root cause
    // is found. Tracks whether bytes are arriving and what the parser is producing.
    static constexpr uint32_t kDebugPrintIntervalMs = 2000;
    static constexpr uint32_t kDebugRfDumpIntervalMs = 10000;  // MON-RF poll competes with NMEA drain; throttle harder.
    uint32_t debug_total_rx_bytes_ = 0;
    uint32_t debug_gga_count_ = 0;
    uint32_t debug_rmc_count_ = 0;
    uint32_t debug_cksum_fail_count_ = 0;
    NMEAParser::SentenceType debug_last_sentence_ = NMEAParser::kSentenceNone;
    uint32_t debug_last_print_timestamp_ms_ = 0;
    uint32_t debug_last_rf_dump_timestamp_ms_ = 0;
};
