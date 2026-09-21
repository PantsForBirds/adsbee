#pragma once

#include "aircraft_dictionary_config.hh"
#include "composite_array.hh"
#include "cpp_at.hh"
#include "data_structures.hh"  // For PFBQueue.
#include "hardware/uart.h"
#include "mode_s_packet.hh"
#include "settings.hh"
#include "uat_packet.hh"

class CommsManager {
   public:
    // Each queue is sized to hold as many packets as would fill a composite array for that type alone.
    // This ensures the 200ms-timer flush always sends a nearly-full array, and the combined byte-count
    // check (queue_filled) fires early in mixed-traffic scenarios as the types sum toward kMaxLenBytes.
    static constexpr uint16_t kModeSPacketReportingQueueDepth =
        (CompositeArray::RawPackets::kMaxLenBytes - sizeof(CompositeArray::RawPackets::Header)) /
        sizeof(RawModeSPacket);  // 124 packets = 3968 B payload
    static constexpr uint16_t kUATADSBPacketReportingQueueDepth =
        (CompositeArray::RawPackets::kMaxLenBytes - sizeof(CompositeArray::RawPackets::Header)) /
        sizeof(RawUATADSBPacket);  // 62 packets = 3968 B payload
    static constexpr uint16_t kUATUplinkPacketReportingQueueDepth =
        (CompositeArray::RawPackets::kMaxLenBytes - sizeof(CompositeArray::RawPackets::Header)) /
        sizeof(RawUATUplinkPacket);  // 7 packets = 3976 B payload

    static constexpr uint16_t kATCommandBufMaxLen = 1200;
    static constexpr uint16_t kNetworkConsoleBufMaxLen = 4000;  // Must match ObjectDictionary::kNetworkConsoleMessageMaxLenBytes.
    static constexpr uint16_t kNetworkConsoleReportingIntervalOverrideNumChars =
        kNetworkConsoleBufMaxLen * 3 /
        4;  // Drain the network console queue immediately if it has more than this many characters.
    static constexpr uint32_t kNetworkConsoleMinReportingIntervalMs =
        50;  // Report messages to nextwork console at minimum rate of 20Hz.
    static constexpr uint16_t kPrintfBufferMaxSize = 500;

    static constexpr uint32_t kOTAWriteTimeoutMs = 5000;  // ms until OTA write command exits if all bytes not received.
    static constexpr uint16_t kMaxForceFlushIterations = 3;

    struct CommsManagerConfig {
        uart_inst_t* comms_uart_handle = uart1;
        uint16_t comms_uart_tx_pin = 4;
        uint16_t comms_uart_rx_pin = 5;
        uart_inst_t* gnss_uart_handle = uart0;
        uint16_t gnss_uart_tx_pin = 0;
        uint16_t gnss_uart_rx_pin = 1;
        uint16_t uart_timeout_us = 0;  // Time to wait for a character if there isn't one alredy available.
    };

    CommsManager(CommsManagerConfig config_in);

    /**
     * Initialize the CommsManager. Sets up UARTs and other necessary peripherals.
     * @retval True if initialization succeeded, false otherwise.
     */
    bool Init();

    /**
     * Update the CommsManager. Runs all the update subroutines required for normal operation.
     * @retval True if update succeeded, false otherwise.
     */
    bool Update();

    /**
     * Immediately drain all reporting queues to the ESP32, looping until empty or kMaxForceFlushIterations
     * is reached. Called from IngestAndForwardPackets when a reporting queue is full.
     * @retval True always; logs an error if queues are not fully drained.
     */
    bool ForceFlushRawPackets();

    /**
     * Update incoming and outgoing buffers for the ESP32 network console. Called as part of Update(), or can be called
     * separately if desired (e.g. while printing to the console without forwarding incoming data to the AT command
     * parser).
     * @retval True if update succeeded, false otherwise.
     */
    bool UpdateNetworkConsole();

    CPP_AT_CALLBACK(ATBaudRateCallback);
    CPP_AT_CALLBACK(ATBiasTeeEnableCallback);
    CPP_AT_CALLBACK(ATDeviceInfoCallback);
    CPP_AT_CALLBACK(ATESP32EnableCallback);
    CPP_AT_CALLBACK(ATESP32FlashCallback);
    CPP_AT_CALLBACK(ATESP32RebootInfoCallback);
#ifdef HARDWARE_UNIT_TESTS
    CPP_AT_CALLBACK(ATESP32TriggerAbortCallback);
#endif
    CPP_AT_CALLBACK(ATEthernetCallback);
    CPP_AT_CALLBACK(ATFeedCallback);
    CPP_AT_CALLBACK(ATRemoteIDCallback);
    CPP_AT_CALLBACK(ATRemoteIDTxCallback);
    CPP_AT_CALLBACK(ATGNSSCallback);
    CPP_AT_CALLBACK(ATGNSSFixCallback);
    CPP_AT_CALLBACK(ATHostnameCallback);
    CPP_AT_CALLBACK(ATLEDBlinkCallback);
    CPP_AT_CALLBACK(ATLEDEnableCallback);
    CPP_AT_CALLBACK(ATOTACallback);
    CPP_AT_HELP_CALLBACK(ATOTAHelpCallback);
    CPP_AT_CALLBACK(ATLogLevelCallback);
    CPP_AT_CALLBACK(ATMAVLINKIDCallback);
    CPP_AT_CALLBACK(ATNetworkInfoCallback);
    CPP_AT_CALLBACK(ATProtocolOutCallback);
    CPP_AT_HELP_CALLBACK(ATProtocolOutHelpCallback);
    CPP_AT_CALLBACK(ATRebootCallback);
    CPP_AT_CALLBACK(ATRxEnableCallback);
    CPP_AT_HELP_CALLBACK(ATRxEnableHelpCallback);
    CPP_AT_CALLBACK(ATRxPositionCallback);
    CPP_AT_CALLBACK(ATSettingsCallback);
    CPP_AT_CALLBACK(ATSubGEnableCallback);
    CPP_AT_CALLBACK(ATSubGFlashCallback);
    CPP_AT_CALLBACK(ATTLReadCallback);
    CPP_AT_CALLBACK(ATTLSetCallback);
    CPP_AT_CALLBACK(ATUptimeCallback);
    CPP_AT_CALLBACK(ATWatchdogCallback);
    CPP_AT_CALLBACK(ATWiFiAPCallback);
    CPP_AT_CALLBACK(ATWiFiSTACallback);
    CPP_AT_CALLBACK(ATBootloader);

    /**
     * Read characters from stdio and the network console into a buffer. Stops when max_len chars
     * have been read, when a newline is received (if terminate_on_newline is true), or when
     * timeout_ms elapses.
     * @param buf Buffer to write into. Not null-terminated by this function.
     * @param max_len Maximum number of characters to read.
     * @param timeout_ms Timeout duration in milliseconds.
     * @param terminate_on_newline If true, stop reading when \r or \n is received.
     * @return Number of characters read, or -1 on timeout.
     */
    int32_t ATReadConsole(char* buf, uint16_t max_len, uint32_t timeout_ms, bool terminate_on_newline = false);

    int console_printf(const char* format, ...);
    int console_level_printf(SettingsManager::LogLevel level, const char* format, ...);
    int iface_printf(SettingsManager::SerialInterface iface, const char* format, ...);
    int iface_vprintf(SettingsManager::SerialInterface iface, const char* format, va_list args);
    bool iface_putc(SettingsManager::SerialInterface iface, char c);
    bool iface_getc(SettingsManager::SerialInterface iface, char& c);
    bool iface_puts(SettingsManager::SerialInterface iface, const char* buf);

    bool network_console_putc(char c);
    bool network_console_puts(const char* buf, uint16_t len = UINT16_MAX);

    inline void ResetConsoleTxTimestamp() { last_esp32_console_tx_timestamp_ms_ = 0; }

#include "comms_reporting.hh"

    inline bool SendBuf(ReportSink sink, const char* buf, uint16_t buf_len, uint16_t num_msgs = 1) {
        for (uint16_t i = 0; i < buf_len; i++) {
            if (!iface_putc(static_cast<SettingsManager::SerialInterface>(sink), buf[i])) {
                return false;
            }
        }
        return true;
    }

    /**
     * Sets the baudrate for a serial interface.
     * @param[in] iface SerialInterface to set baudrate for.
     * @param[in] baudrate Baudrate to set.
     * @retval True if the baudrate could be set, false if the interface specified does not support a baudrate.
     */
    inline bool SetBaudRate(SettingsManager::SerialInterface iface, uint32_t baudrate) {
        switch (iface) {
            case SettingsManager::kCommsUART:
                // Program the hardware, but persist the requested baudrate (not the
                // hardware-rounded value uart_set_baudrate() returns) so settings round-trip
                // exactly.
                uart_set_baudrate(config_.comms_uart_handle, baudrate);
                settings_manager.settings.baud_rates[SettingsManager::SerialInterface::kCommsUART] = baudrate;
                return true;
                break;
            case SettingsManager::kGNSSUART:
                // Program the hardware, but persist the requested baudrate (not the
                // hardware-rounded value uart_set_baudrate() returns) so settings round-trip
                // exactly.
                uart_set_baudrate(config_.gnss_uart_handle, baudrate);
                settings_manager.settings.baud_rates[SettingsManager::SerialInterface::kGNSSUART] = baudrate;
                return true;
                break;
            default:
                return false;  // Other interfaces don't have a baudrate.
        }
        return false;  // Should never get here.
    }

    /**
     * Returns the currently set baudrate for a serial interface.
     * @param[in] iface SerialInterface to get the baudrate for.
     * @param[out] baudrate Reference to uint32_t to fill with retrieved value.
     * @retval True if baudrate retrieval succeeded, false if iface does not support a baudrate.
     */
    inline bool GetBaudRate(SettingsManager::SerialInterface iface, uint32_t& baudrate) {
        switch (iface) {
            case SettingsManager::kCommsUART:
                // Save the actual set value as comms_uart_baudrate_.
                baudrate = settings_manager.settings.baud_rates[SettingsManager::SerialInterface::kCommsUART];
                return true;
                break;
            case SettingsManager::kGNSSUART:
                // Save the actual set value as gnss_uart_baudrate_.
                baudrate = settings_manager.settings.baud_rates[SettingsManager::SerialInterface::kGNSSUART];
                return true;
                break;
            default:
                return false;  // Other interfaces don't have a baudrate.
        }
        return false;  // Should never get here.
    }

    /**
     * Specify the reporting protocol for a given serial interface.
     * @param[in] iface SerialInterface to set reporting protocol on.
     * @param[in] protocol Reporting protocol to set on iface.
     * @retval True if succeeded, false otherwise.
     */
    inline bool SetReportingProtocol(SettingsManager::SerialInterface iface,
                                     SettingsManager::ReportingProtocol protocol) {
        settings_manager.settings.reporting_protocols[iface] = protocol;
        return true;
    }

    /**
     * Get the reporting protocol for a given serial interface.
     * @param[in] iface SerialInterface to get the reporting protocol from.
     * @param[out] protocol reference to ReportingProtocol to fill with result.
     * @retval True if reportig protocol could be retrieved, false otherwise.
     */
    inline bool GetReportingProtocol(SettingsManager::SerialInterface iface,
                                     SettingsManager::ReportingProtocol& protocol) {
        protocol = settings_manager.settings.reporting_protocols[iface];
        return true;
    }

    // Queue for storing transponder packets before they get reported.
    PFBQueue<RawModeSPacket> mode_s_packet_reporting_queue = PFBQueue<RawModeSPacket>(
        {.buf_len_num_elements = kModeSPacketReportingQueueDepth, .buffer = mode_s_packet_reporting_queue_buffer_});
    PFBQueue<RawUATADSBPacket> uat_adsb_packet_reporting_queue = PFBQueue<RawUATADSBPacket>(
        {.buf_len_num_elements = kUATADSBPacketReportingQueueDepth, .buffer = uat_adsb_packet_reporting_queue_buffer_});
    PFBQueue<RawUATUplinkPacket> uat_uplink_packet_reporting_queue =
        PFBQueue<RawUATUplinkPacket>({.buf_len_num_elements = kUATUplinkPacketReportingQueueDepth,
                                      .buffer = uat_uplink_packet_reporting_queue_buffer_});

    // Queues for incoming / outgoing network characters.
    PFBQueue<char> esp32_console_rx_queue =
        PFBQueue<char>({.buf_len_num_elements = kNetworkConsoleBufMaxLen, .buffer = esp32_console_rx_queue_buffer_});
    PFBQueue<char> esp32_console_tx_queue =
        PFBQueue<char>({.buf_len_num_elements = kNetworkConsoleBufMaxLen, .buffer = esp32_console_tx_queue_buffer_});

   private:
    // AT Functions
    bool UpdateAT();

    CommsManagerConfig config_;

    // Console Settings
    CppAT at_parser_;

    // Queues for incoming / outgoing network console characters.
    char esp32_console_rx_queue_buffer_[kNetworkConsoleBufMaxLen];
    char esp32_console_tx_queue_buffer_[kNetworkConsoleBufMaxLen];
    uint32_t last_esp32_console_tx_timestamp_ms_ = 0;  // Timestamp of last network console TX.

    // Queue for holding new transponder packets before they get reported.
    RawModeSPacket mode_s_packet_reporting_queue_buffer_[kModeSPacketReportingQueueDepth];
    RawUATADSBPacket uat_adsb_packet_reporting_queue_buffer_[kUATADSBPacketReportingQueueDepth];
    RawUATUplinkPacket uat_uplink_packet_reporting_queue_buffer_[kUATUplinkPacketReportingQueueDepth];

    // Reporting protocol timestamps
    // NOTE: Raw reporting interval used for RAW and BEAST protocols as well as internal functions.
    uint32_t last_raw_report_check_timestamp_ms_ =
        0;  // Timestamp of last time we checked whether we need to report packets.
    uint32_t last_raw_report_timestamp_ms_ = 0;
    // Single shared timer for all locally-decoded dictionary report protocols (CSBee, MAVLINK, GDL90).
    // All protocols share a UID snapshot and therefore share a reporting interval.
    uint32_t last_locally_decoded_report_timestamp_ms_ = 0;

    // Shared UID snapshot for all periodic dictionary reporting protocols.
    // One array avoids duplicating 800 bytes per protocol. UpdateReporting populates it once
    // per round; each protocol walks it independently via its own index below.
    uint32_t report_uids_[kMaxReportUIDs];
    uint16_t report_uids_count_ = 0;

    // Per-protocol progress indices into report_uids_[].
    uint16_t csbee_report_uid_index_ = 0;
    uint16_t mavlink1_report_uid_index_ = 0;
    uint16_t mavlink2_report_uid_index_ = 0;
    uint16_t gdl90_report_uid_index_ = 0;

    // Per-protocol "round active" flags. Set by UpdateReporting when a new round starts;
    // cleared by each Report* function when it finishes sending its footer/delimiter.
    bool csbee_round_active_ = false;
    bool mavlink1_round_active_ = false;
    bool mavlink2_round_active_ = false;
    bool gdl90_round_active_ = false;

    // Overrun error gates — each fires once per overrunning round to prevent log spam.
    bool csbee_overrun_reported_ = false;
    bool mavlink1_overrun_reported_ = false;
    bool mavlink2_overrun_reported_ = false;
    bool gdl90_overrun_reported_ = false;

    uint16_t aircraftjson_report_uid_index_ = 0;
    bool aircraftjson_round_active_ = false;
    bool aircraftjson_overrun_reported_ = false;
};

extern CommsManager comms_manager;
extern const CppAT::ATCommandDef_t at_command_list[];  // Initialized in comms_at.cc
extern const uint16_t at_command_list_num_commands;

#define TEXT_COLOR_RED              "\033[31m"
#define TEXT_COLOR_GREEN            "\033[32m"
#define TEXT_COLOR_YELLOW           "\033[33m" /* orange on some systems */
#define TEXT_COLOR_BLUE             "\033[34m"
#define TEXT_COLOR_MAGENTA          "\033[35m"
#define TEXT_COLOR_CYAN             "\033[36m"
#define TEXT_COLOR_LIGHT_GRAY       "\033[37m"
#define TEXT_COLOR_DARK_GRAY        "\033[90m"
#define TEXT_COLOR_BRIGHT_RED       "\033[91m"
#define TEXT_COLOR_BRIGHT_GREEN     "\033[92m"
#define TEXT_COLOR_BRIGHT_YELLOW    "\033[93m"
#define TEXT_COLOR_BRIGHT_BLUE      "\033[94m"
#define TEXT_COLOR_BRIGHT_MAGENTA   "\033[95m"
#define TEXT_COLOR_BRIGHT_CYAN      "\033[96m"
#define TEXT_COLOR_WHITE            "\033[97m"

#define TEXT_COLOR_RESET            "\033[0m"

#define CONSOLE_PRINTF(format, ...) comms_manager.console_printf(format __VA_OPT__(, ) __VA_ARGS__);
#define CONSOLE_INFO(tag, format, ...)                                                                         \
    comms_manager.console_level_printf(SettingsManager::LogLevel::kInfo,                                       \
                                       tag ": " TEXT_COLOR_GREEN format TEXT_COLOR_RESET "\r\n" __VA_OPT__(, ) \
                                           __VA_ARGS__);
#define CONSOLE_WARNING(tag, format, ...)                                                                       \
    comms_manager.console_level_printf(SettingsManager::LogLevel::kWarnings,                                    \
                                       tag ": " TEXT_COLOR_YELLOW format TEXT_COLOR_RESET "\r\n" __VA_OPT__(, ) \
                                           __VA_ARGS__);
#define CONSOLE_ERROR(tag, format, ...)                                        \
    comms_manager.console_level_printf(SettingsManager::LogLevel::kErrors, tag \
                                       ": " TEXT_COLOR_RED format TEXT_COLOR_RESET "\r\n" __VA_OPT__(, ) __VA_ARGS__);
