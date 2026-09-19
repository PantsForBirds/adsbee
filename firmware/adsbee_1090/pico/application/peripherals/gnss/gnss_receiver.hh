#pragma once

#include "bsp.hh"
#include "hardware/irq.h"
#include "hardware/uart.h"  // For uart_inst_t and UART bring-up in Init().
#include "nmea_parser.hh"
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
    // SendInitCommands()'s own worst-case blocking time (see UbloxMAXM10::kProbeAttempts /
    // kProbeTimeoutMs).
    static constexpr uint32_t kUnhealthyRetryIntervalMs = 5000;
    static constexpr uint32_t kMaxUnhealthyRetries = 5;
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
     * @retval True if the module responded during Init() (i.e. it is present and configured).
     */
    bool IsHealthy() const { return healthy_; }

    /** @retval True while the GNSS interface is powered/enabled and should consume UART data. */
    bool IsActive() const { return active_; }

    /** @retval Number of PPS rising edges observed since GNSS was last enabled or disabled. */
    uint32_t pps_count() const { return pps_count_; }

   protected:
    /**
     * @retval Receiver-specific default UART baud rate (e.g. factory default for the module).
     */
    virtual uint32_t GetDefaultBaudrate() const = 0;

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
     * receiver's default baud. Interrupt reception is enabled separately after final baud setup.
     */
    void ClaimUart();

    /** Start/stop interrupt-driven UART reception and reset the software RX ring buffer. */
    void EnableRxInterrupt();
    void DisableRxInterrupt();

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

    // Single-producer (UART ISR), single-consumer (main loop) ring buffer. Monotonic 16-bit indices
    // make all 2048 slots usable; unsigned subtraction remains valid across index wrap.
    char rx_buffer_[kRxBufferSize] = {0};
    volatile uint16_t rx_head_ = 0;
    volatile uint16_t rx_tail_ = 0;
    volatile uint32_t rx_overflow_count_ = 0;
    volatile uint32_t rx_irq_count_ = 0;
    uint32_t rx_polled_byte_count_ = 0;
    static inline GNSSReceiver* rx_irq_owner_ = nullptr;
    static inline bool rx_irq_handler_installed_[2] = {false, false};
    static void RxIRQHandler();

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
