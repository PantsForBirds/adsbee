#pragma once

#ifdef ON_PICO
#define ON_COPRO_MASTER
#elif defined(ON_ESP32) || defined(ON_TI)
#define ON_COPRO_SLAVE
#endif

#include "composite_array.hh"
#include "data_structures.hh"
#include "hal.hh"
#include "mode_s_packet.hh"
#include "settings.hh"
#include "stdint.h"
#include "uat_packet.hh"
#ifdef ON_ESP32
#include "adsbee_server.hh"
#include "esp_mac.h"   // For retrieving Base MAC address.
#include "esp_wifi.h"  // For retrieving WiFi Station MAC address.
#endif
#ifdef ON_TI
#endif

class ObjectDictionary {
   public:
    static const uint8_t kFirmwareVersionMajor;
    static const uint8_t kFirmwareVersionMinor;
    static const uint8_t kFirmwareVersionPatch;
    static const uint8_t kFirmwareVersionReleaseCandidate;
    static const uint32_t kFirmwareVersion;

    static constexpr uint16_t kMACAddrLenBytes = 6;

    static constexpr uint16_t kNetworkConsoleMessageMaxLenBytes =
        4000;  // Maximum size of SCWritePacket (actually is a bit smaller due to headers).
    static constexpr uint16_t kLogMessageMaxNumChars = 500;
    static constexpr uint16_t kLogMessageTagMaxNumChars = 20;
    static constexpr uint16_t kLogMessageQueueDepth = 10;

#ifdef ON_COPRO_SLAVE
    static constexpr uint16_t kSCCommandRequestQueueDepth = 10;
    static constexpr uint16_t kNetworkConsoleRxQueueDepth = kNetworkConsoleMessageMaxLenBytes * 4;
    static constexpr uint16_t kNetworkConsoleMutexTimeoutMs = 5;  // Max wait for network_console_rx_queue_mutex in SPI task context.
#endif
#ifdef ON_TI
    static constexpr uint16_t kDecodedUATADSBPacketQueueDepth = 10;
    static constexpr uint16_t kDecodedUATUplinkPacketQueueDepth = 2;
#endif

    enum Address : uint8_t {
        kAddrInvalid = 0,             // Default value.
        kAddrFirmwareVersion = 0x01,  // Firmware version as a uint32_t.
        kAddrScratch = 0x02,          // Used for testing SPI communications.
        kAddrSettingsData = 0x03,     // Used to transfer settings information.
        kAddrRawModeSPacket = 0x04,   // Used to forward raw packets from RP2040 to ESP32.
        kAddrDecodedModeSPacket = 0x05,
        kAddrRawModeSPacketArray = 0x06,
        kAddrDecodedModeSPacketArray = 0x07,
        kAddrAircraftDictionaryMetrics = 0x08,  // For forwarding dictionary metrics from RP2040 to ESP32.
        kAddrESP32DeviceInfo = 0x09,            // ESP32 MAC addresses.
        kAddrConsole = 0x0A,                    // Pipe for console characters.
        kAddrESP32NetworkInfo = 0x0B,           // Network information for ESP32.
        kAddrDeviceStatus = 0x0C,  // Struct containing number of pending log messages and current timestamp.
        kAddrLogMessages = 0x0D,   // Used to retrieve log messages from ESP32 and CC1312.
        kAddrRollQueue = 0x0E,     // Used to roll various queues on coprocessor slaves to confirm they have been read.
        kAddrSCCommandRequests = 0x0F,         // Used by slave to request commands from master.
        kAddrCompositeArrayRawPackets = 0x10,  // Single endpoint for reading / writing raw ADSB and UAT packets.
        kAddrESP32RebootInfo = 0x11,           // ESP32 last reset reason and optional core dump summary.
        kAddrESP32TriggerAbort = 0x12,         // Debug only: trigger abort() on the ESP32 to test core dump.
        kAddrLEDBlink = 0x13,  // Force-blink the slave's LED: uint32_t duration_ms, bypasses led_enabled.
        kNumAddrs
    };

    // Maximum duration for a forced LED blink via kAddrLEDBlink (shared with the AT+LED_BLINK master-side check).
    static constexpr uint32_t kLEDBlinkMaxDurationMs = 60'000;

    // Commands are written from Master to Slave.
    enum SCCommand : uint8_t {
        kCmdInvalid = 0x00,
        kCmdWriteToSlave = 0x01,            // No response expected.
        kCmdWriteToSlaveRequireAck = 0x02,  // Expects a response to continue to the next block.
        kCmdReadFromSlave = 0x03,
        // kCmdWriteToMaster = 0x04,            // No response expected.
        // kCmdWriteToMasterRequireAck = 0x05,  // Expects a response to continue to the next block.
        // kCmdReadFromMaster = 0x06,
        kCmdDataBlock = 0x07,
        kCmdAck = 0x08
    };

    // Queues are read by peeking, then a separate command to "roll" the queue by discarding the elements that were
    // read. This way we don't lose items if a queue is read from but the data gets corrupted.
    // QueueID is the enum value used to determine which queue to roll.
    enum QueueID : uint8_t {
        kQueueIDInvalid = 0,        // Default value.
        kQueueIDLogMessages,        // Queue for log messages.
        kQueueIDSCCommandRequests,  // Queue for SCCommand requests from slave to master.
        kQueueIDConsole,            // Queue for network console messages.
        kNumQueueIDs
    };

    struct __attribute__((__packed__)) RollQueueRequest {
        QueueID queue_id = kQueueIDInvalid;  // Queue to roll.
        uint16_t num_items = 0;              // Number of items in the queue to discard.
    };

    /**
     * Struct used by the slave to request a command to be performed by the master.
     * Can't use packed attribute because of std::function member (not Plain Old Data aka. POD type).
     */
    struct SCCommandRequest {
        SCCommand command = SCCommand::kCmdInvalid;
        ObjectDictionary::Address addr = ObjectDictionary::Address::kAddrInvalid;
        uint16_t offset = 0;
        uint16_t len = 0;  // Length of the data to read/write in the requested command.
    };

    struct SCCommandRequestWithCallback {
        SCCommandRequest request;
        std::function<void()> complete_callback = nullptr;
    };

    struct __attribute__((packed)) RP2040DeviceStatus {
        uint32_t timestamp_ms = 0;
        int8_t temperature_deg_c = 0;
        uint8_t core_0_usage_percent = 0;
        uint8_t core_1_usage_percent = 0;
        SettingsManager::RxPosition rx_position;  // Current receiver position.
        bool rx_position_available = false;
        bool gnss_enabled = false;
        bool gnss_fix_valid = false;
        float gnss_latitude_deg = 0.0f;
        float gnss_longitude_deg = 0.0f;
        bool gnss_utc_time_valid = false;
        uint8_t gnss_utc_hour = 0;
        uint8_t gnss_utc_minute = 0;
        uint8_t gnss_utc_second = 0;
        uint16_t gnss_utc_millisecond = 0;
        // 1090MHz receiver noise floor estimate (RSSI sampled between packets and low-pass filtered). The trigger
        // level sits at noise_floor_mv + TL offset.
        uint16_t noise_floor_mv = 0;
        int16_t noise_floor_dbm = 0;
    };

    struct __attribute__((__packed__)) ESP32DeviceStatus {
        uint32_t timestamp_ms = 0;
        int8_t temperature_deg_c = 0;
        uint8_t core_0_usage_percent = 0;
        uint8_t core_1_usage_percent = 0;
        uint32_t heap_free_bytes = 0;
        uint32_t heap_largest_free_block_bytes = 0;

        uint16_t num_queued_log_messages = 0;
        uint32_t queued_log_messages_packed_size_bytes = 0;

        uint16_t num_queued_sc_command_requests = 0;  // Number of SCCommand requests queued for the master.
        uint32_t num_queued_network_console_rx_chars =
            0;  // Number incoming of characters waiting to be read by the RP2040 from the ESP32's network console.

        // Length in bytes (including the CompositeArray header) of Remote ID packets the ESP32 has received over
        // BLE/WiFi and has queued for the RP2040 to pull from kAddrCompositeArrayRawPackets. Header-only (== 8) means
        // nothing pending. See the ESP32->RP2040 forwarding path in peripherals/esp32/esp32.cc.
        uint16_t pending_raw_packets_len_bytes = 0;
        // Bitfield reporting the live Remote ID receiver AND transmitter state (see RemoteIDManager::Status). Lets the
        // RP2040 explain, e.g., "Remote ID requested but blocked because WiFi is enabled on a non-PSRAM build" in
        // AT+REMOTE_ID? / AT+REMOTE_ID_TX?.
        uint16_t remote_id_status = 0;
    };

    /**
     * Struct used to retrieve device information from the ESP32.
     */
    struct __attribute__((__packed__)) ESP32DeviceInfo {
        uint8_t base_mac[kMACAddrLenBytes];
        uint8_t wifi_station_mac[kMACAddrLenBytes];
        uint8_t wifi_ap_mac[kMACAddrLenBytes];
        uint8_t bluetooth_mac[kMACAddrLenBytes];
        uint8_t ethernet_mac[kMACAddrLenBytes];
    };

    static constexpr uint16_t kResetReasonStrMaxLen = 32;

    struct __attribute__((__packed__)) ESP32RebootInfo {
        uint8_t reset_reason;                    // esp_reset_reason_t cast to uint8_t
        char reset_reason_str[kResetReasonStrMaxLen + 1];
        bool core_dump_to_flash_enabled;         // True if built with CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    };

    /**
     * Struct used to retrieve network information from the ESP32.
     */
    struct __attribute__((__packed__)) ESP32NetworkInfo {
        bool ethernet_enabled = false;
        bool ethernet_has_ip = false;
        char ethernet_ip[SettingsManager::Settings::kIPAddrStrLen + 1] = {0};
        char ethernet_netmask[SettingsManager::Settings::kIPAddrStrLen + 1] = {0};
        char ethernet_gateway[SettingsManager::Settings::kIPAddrStrLen + 1] = {0};

        bool wifi_sta_enabled = false;
        char wifi_sta_ssid[SettingsManager::Settings::kWiFiSSIDMaxLen + 1] = {0};
        bool wifi_sta_has_ip = false;
        char wifi_sta_ip[SettingsManager::Settings::kIPAddrStrLen + 1] = {0};
        char wifi_sta_netmask[SettingsManager::Settings::kIPAddrStrLen + 1] = {0};
        char wifi_sta_gateway[SettingsManager::Settings::kIPAddrStrLen + 1] = {0};

        bool wifi_ap_enabled = false;
        bool wifi_ap_num_clients = 0;
        char wifi_ap_client_ips[SettingsManager::Settings::kWiFiMaxNumClients]
                               [SettingsManager::Settings::kIPAddrStrLen + 1];
        char wifi_ap_client_macs[SettingsManager::Settings::kWiFiMaxNumClients]
                                [SettingsManager::Settings::kMACAddrStrLen + 1];

        ESP32NetworkInfo() {
            for (uint16_t i = 0; i < SettingsManager::Settings::kWiFiMaxNumClients; i++) {
                memset(wifi_ap_client_ips[i], 0x0, SettingsManager::Settings::kIPAddrStrLen + 1);
                memset(wifi_ap_client_macs[i], 0x0, SettingsManager::Settings::kMACAddrNumBytes);
            }
        }
    };

    struct __attribute__((__packed__)) SubGHzDeviceStatus {
        uint32_t timestamp_ms = 0;  // Timestamp in milliseconds since boot.
        uint8_t temperature_deg_c = 0;
        uint8_t cpu_usage_percent = 0;

        uint16_t num_queued_log_messages = 0;
        uint32_t queued_log_messages_packed_size_bytes = 0;

        uint16_t num_queued_sc_command_requests = 0;  // Number of SCCommand requests queued for the master.

        uint16_t pending_raw_packets_len_bytes = 0;  // Number of bytes of raw packets waiting to be read by the master.

        // Counters for number of packets received since the last time the device info struct was read.
        uint16_t num_raw_uat_adsb_packets = 0;
        uint16_t num_valid_uat_adsb_packets = 0;
        uint16_t num_raw_uat_uplink_packets = 0;
        uint16_t num_valid_uat_uplink_packets = 0;
    };

    /**
     * Combination device status struct that allows the RP2040 device status and Sub-GHz device status to be written to
     * the ESP32 simultaneously.
     */
    struct __attribute__((__packed__)) CompositeDeviceStatus {
        RP2040DeviceStatus rp2040;
        SubGHzDeviceStatus subg;
    };

    /**
     * Struct used to pass debug messages between devices.
     */
    struct __attribute__((__packed__)) LogMessage {
        static const uint16_t kHeaderSize = sizeof(SettingsManager::LogLevel) + sizeof(uint16_t);
        SettingsManager::LogLevel log_level;
        uint16_t num_chars;
        char message[kLogMessageMaxNumChars + 1] = {'\0'};
    };

#ifdef ON_COPRO_SLAVE
    /**
     * Setter for writing data to the address space.
     * @param[in] addr Address to write to.
     * @param[in] buf Buffer to read from.
     * @param[in] buf_len Number of Bytes to write.
     * @param[in] offset Byte offset from beginning of object. Used for partial reads.
     * @retval Returns true if successfully wrote, false if address was invalid or something else borked.
     */
    bool SetBytes(Address addr, uint8_t* buf, uint16_t buf_len, uint16_t offset = 0);

    /**
     * Getter for reading data from the address space.
     * @param[in] addr Address to read from.
     * @param[out] buf Buffer to write to.
     * @param[in] buf_len Number of Bytes to read.
     * @param[in] offset Byte offset from beginning of object. Used for partial reads.
     * @retval Returns true if successfully read, false if address was invalid or something else borked.
     */
    bool GetBytes(Address addr, uint8_t* buf, uint16_t buf_len, uint16_t offset = 0);

    /**
     * Request that a command be performed by the master. Non-blocking.
     * @param[in] request Command request to send to the master.
     * @retval True if the request was successfully queued, false if the queue is full.
     * @note The master will execute the command and call the callback function when it is done
     */
    bool RequestSCCommand(const SCCommandRequestWithCallback& request);

    bool RequestSCCommandBlocking(const SCCommandRequestWithCallback& request);

    /**
     * Helper functions for requests with no callback.
     */
    bool RequestSCCommand(const SCCommandRequest& request) {
        return RequestSCCommand(SCCommandRequestWithCallback{
            .request = request,
            .complete_callback = nullptr,
        });
    }
    bool RequestSCCommandBlocking(const SCCommandRequest& request) {
        return RequestSCCommandBlocking(SCCommandRequestWithCallback{
            .request = request,
            .complete_callback = nullptr,
        });
    }

    /**
     * Updates the device status struct with the latest information. Called when the object dictionary is read from at
     * kAddrDeviceStatus, or when other functions want to refresh the device status before borrowing it for other uses.
     */
    void UpdateDeviceStatus();

#endif

    /**
     * Builds a buffer of log messages with empty space removed.
     * @param[out] buf Buffer to write the log messages to.
     * @param[in] buf_len Length of the buffer.
     * @param[in] log_message_queue Queue of log messages to pack.
     * @param[in] num_messages Number of log messages in the array.
     * @retval Number of bytes written to the buffer.
     */
    uint16_t PackLogMessages(uint8_t* buf, uint16_t buf_len, PFBQueue<ObjectDictionary::LogMessage>& log_message_queue,
                             uint16_t num_messages, uint16_t offset = 0);

    /**
     * Unpacks a buffer of log messages into an array.
     * @param[in] buf Buffer to read the log messages from.
     * @param[in] buf_len Length of the buffer.
     * @param[out] log_message_queue Queue to store the unpacked log messages.
     * @param[in] max_num_messages Maximum number of log messages to unpack.
     * @retval Number of log messages unpacked.
     */
    uint16_t UnpackLogMessages(uint8_t* buf, uint16_t buf_len,
                               PFBQueue<ObjectDictionary::LogMessage>& log_message_queue, uint16_t max_num_messages);

    PFBQueue<LogMessage> log_message_queue = PFBQueue<LogMessage>({
        .buf_len_num_elements = kLogMessageQueueDepth,
        .buffer = log_message_queue_buffer_,
        .overwrite_when_full = true  // Some of you may die, but that is a sacrifice I am willing to make.
    });

#ifdef ON_COPRO_SLAVE
    // Stuff on ESP32 and CC1312.
    PFBQueue<ObjectDictionary::SCCommandRequestWithCallback> sc_command_request_queue =
        PFBQueue<ObjectDictionary::SCCommandRequestWithCallback>({
            .buf_len_num_elements = ObjectDictionary::kSCCommandRequestQueueDepth,
            .buffer = sc_command_request_queue_buffer_,
            .overwrite_when_full = false  // We don't want to overwrite command requests, since they could be important.
        });
#endif

#ifdef ON_ESP32
    PFBQueue<char> network_console_rx_queue = PFBQueue<char>(PFBQueue<char>::PFBQueueConfig{
        .buf_len_num_elements = kNetworkConsoleRxQueueDepth,
        .buffer = network_console_message_buffer_,
        .overwrite_when_full =
            false  // We don't want to overwrite network console messages, since they could be importatnt.
    });
    SemaphoreHandle_t network_console_rx_queue_mutex = xSemaphoreCreateMutex();
    CompositeDeviceStatus composite_device_status = {};
    ESP32DeviceStatus device_status = {};
#elif defined(ON_TI)
    PFBQueue<RawUATADSBPacket> raw_uat_adsb_packet_queue =
        PFBQueue<RawUATADSBPacket>({.buf_len_num_elements = kDecodedUATADSBPacketQueueDepth,
                                    .buffer = raw_uat_adsb_packet_buffer_,
                                    .overwrite_when_full = true});
    PFBQueue<RawUATUplinkPacket> raw_uat_uplink_packet_queue =
        PFBQueue<RawUATUplinkPacket>({.buf_len_num_elements = kDecodedUATUplinkPacketQueueDepth,
                                      .buffer = raw_uat_uplink_packet_buffer_,
                                      .overwrite_when_full = true});

    struct SubGHzRadioMetrics {
        uint16_t num_raw_uat_adsb_packets = 0;
        uint16_t num_valid_uat_adsb_packets = 0;
        uint16_t num_raw_uat_uplink_packets = 0;
        uint16_t num_valid_uat_uplink_packets = 0;
    };
    SubGHzRadioMetrics metrics = {0};
    SubGHzDeviceStatus device_status = {};
#endif

   private:
    uint32_t scratch_ = 0x0;  // Scratch register used for testing.

    // On Pico, this is a queue of log messages gathered from other devices. On other devices, this is a queue of log
    // messages waiting to be slurped up by the RP2040.
    LogMessage log_message_queue_buffer_[kLogMessageQueueDepth] = {};

#ifdef ON_COPRO_SLAVE
    // Private stuff on ESP32 and CC1312.
    ObjectDictionary::SCCommandRequestWithCallback
        sc_command_request_queue_buffer_[ObjectDictionary::kSCCommandRequestQueueDepth] = {};
#endif

#ifdef ON_ESP32
    // ESP32
    // char* network_console_message_buffer_ = (char*)heap_caps_malloc(kNetworkConsoleRxQueueDepth, 0);
    char network_console_message_buffer_[kNetworkConsoleRxQueueDepth] = {};
#elif defined(ON_TI)
    // CC1312
    RawUATADSBPacket raw_uat_adsb_packet_buffer_[kDecodedUATADSBPacketQueueDepth] = {};
    RawUATUplinkPacket raw_uat_uplink_packet_buffer_[kDecodedUATUplinkPacketQueueDepth] = {};
#endif
};

extern ObjectDictionary object_dictionary;
