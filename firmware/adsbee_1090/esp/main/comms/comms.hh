#pragma once

#include "data_structures.hh"
#include "driver/gpio.h"
#include "esp_eth.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
// #include "gdl90/gdl90_utils.hh"
#include "aircraft_dictionary_config.hh"
#include "composite_array.hh"
#include "lwip/sockets.h"  // For port definition.
#include "mode_s_packet.hh"
#include "nvs_flash.h"
#include "object_dictionary.hh"
#include "settings.hh"

class CommsManager {
   public:
    // Packet queue sizes used to stage packets for reporting.
    static const uint16_t kMaxNumModeSPackets = 100;
    static const uint16_t kMaxNumUATADSBPackets = 20;
    static const uint16_t kMaxNumUATUplinkPackets = 6;

    // Reporting via the IP task is done by forwarding CompositeArray::RawPackets buffers from the ADSBeeServer task.
    // These buffers are put into a queue, which has its element size and number of elements set here.
    static const uint16_t kReportingCompositeArrayQueueNumElements = 5;

    static const uint16_t kWiFiMessageQueueLen = 8;
    // Reconnect intervals must be long enough that we register an IP lost event before trying the reconnect, otherwise
    // we get stuck in limbo where we may attempt a reconnect but the new IP address is never looked for (not controlled
    // by our own flags, but by internal LwIP stuff).
    static const uint32_t kWiFiSTAReconnectIntervalMs = 10e3;
    static const uint32_t kEthernetReconnectIntervalMs = 10e3;
    static const uint32_t kWiFiSTATaskUpdateIntervalMs = 100;
    static const uint32_t kWiFiSTATaskUpdateIntervalTicks = kWiFiSTATaskUpdateIntervalMs / portTICK_PERIOD_MS;

    static const uint16_t kConsolePrintBufMaxLen = 512;

    struct CommsManagerConfig {
        int32_t aux_spi_clk_rate_hz = 40e6;  // 40 MHz (this could go up to 80MHz).
        spi_host_device_t aux_spi_handle = SPI3_HOST;
        gpio_num_t aux_spi_mosi_pin = GPIO_NUM_14;
        gpio_num_t aux_spi_miso_pin = GPIO_NUM_13;
        gpio_num_t aux_spi_clk_pin = GPIO_NUM_17;
        gpio_num_t aux_spi_cs_pin = GPIO_NUM_18;
        gpio_num_t aux_io_b_pin = GPIO_NUM_48;  // W5500 RST
        gpio_num_t aux_io_c_pin = GPIO_NUM_47;  // W5500 INT
    };

    struct NetworkMessage {
        static const uint16_t kMaxLenBytes = 600;  // Needs to be big enough for UAT uplink packets.
        in_port_t port = 0;
        uint16_t len = 0;
        uint8_t data[kMaxLenBytes];

        NetworkMessage() { memset(data, 0x0, kMaxLenBytes); }
    };

    CommsManager(CommsManagerConfig config_in) : config_(config_in) {
        wifi_clients_list_mutex_ = xSemaphoreCreateMutex();
        // wifi_ap_message_queue_ (kWiFiMessageQueueLen * ~604 B ≈ 4.8 KB internal SRAM) is only used by the WiFi-AP
        // UDP-broadcast path, so it is created lazily in WiFiInit() when the AP is actually enabled — an Ethernet-only
        // (no-AP) unit never pays for it. See WiFiInit() / WiFiAccessPointSendMessageToAllStations().
        wifi_ap_message_queue_ = nullptr;
        ip_wan_reporting_composite_array_queue_ =
            xQueueCreate(kReportingCompositeArrayQueueNumElements, CompositeArray::RawPackets::kMaxLenBytes);
    }

    ~CommsManager() {
        vSemaphoreDelete(wifi_clients_list_mutex_);
        if (wifi_ap_message_queue_ != nullptr) vQueueDelete(wifi_ap_message_queue_);
        vQueueDelete(ip_wan_reporting_composite_array_queue_);
    }

    /**
     * Helper function that converts a MAC buffer to a uint64_t that's easier to pass around and compare.
     * @param[in] mac_buf_in Buffer to read the MAC address value from.
     */
    static uint64_t MACToUint64(uint8_t* mac_buf_in) {
        uint64_t mac_out = 0;
        for (uint16_t i = 0; i < SettingsManager::Settings::kMACAddrNumBytes; i++) {
            mac_out |= mac_buf_in[i] << ((5 - i) * 8);
        }
        return mac_out;
    }

    struct NetworkClient {
        esp_ip4_addr_t ip;
        uint64_t mac;
        bool active = false;  // "Active" flag allows reuse of open slots in the list when a client disconnects.

        /**
         * Set the MAC address from a buffer, assuming the buffer is a 6 Byte MAC address MSB first.
         * @param[in] mac_buf_in Buffer to read the MAC address value from.
         */
        void SetMAC(uint8_t* mac_buf_in) { mac = MACToUint64(mac_buf_in); }

        /**
         * Get the MAC address by writing it out to a buffer, assuming the buffer is a 6 Byte MAC address MSB first.
         */
        void GetMAC(uint8_t* mac_buf_out) {
            for (uint16_t i = 0; i < SettingsManager::Settings::kMACAddrNumBytes; i++) {
                mac_buf_out[i] = mac >> ((5 - i) * 8);
            }
        }
    };

    /**
     * Initialize prerequisites for WiFi and Ethernet.
     */
    bool Init();

    /**
     * Connect to an external network via Ethernet. Used during ethernet restarts to acquire new IP address. For some
     * reason ethernet requires the DHCP client service to be stopped and restarted in order to recover with an IP
     * address, so this function provides a convenient function that does that.
     * @retval True if successfully connected, false otherwise.
     */
    bool ConnectToEthernet();

    /**
     * Returns whether the ESP32 is connected to an external network via Ethernet.
     * @retval True if connected and assigned IP address, false otherwise.
     */
    inline bool EthernetHasIP() { return ethernet_has_ip_; }

    /**
     * Initialize the Ethernet peripheral (WIZNet W5500).
     */
    bool EthernetInit();

    /**
     * De-initialize the Ethernet peripheral (WIZNet W5500).
     */
    bool EthernetDeInit();

    /**
     * Handle Ethernet hardware level events.
     */
    void EthernetEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

    /**
     * Get the current status of ESP32 network interfaces as an ESP32NetworkInfo struct.
     */
    ObjectDictionary::ESP32NetworkInfo GetNetworkInfo();

    inline uint16_t GetNumWiFiClients() { return num_wifi_clients_; }

    /**
     * Returns whether the ESP32 is connected to an external network via either Ethernet or WiFi.
     */
    inline bool HasIP() { return wifi_sta_has_ip_ || ethernet_has_ip_; }

    /**
     * Handler for IP events associated with ethernet and WiFi. Public so that pass through functions can access it.
     */
    void IPEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

    bool LogMessageToCoprocessor(SettingsManager::LogLevel log_level, const char* tag, const char* format, ...);

    /**
     * Initialize the WiFi peripheral (access point and station). WiFiDeInit() and WiFiInit() should be called every
     * time network settings are chagned.
     */
    bool WiFiInit();

    /**
     * De-initialize the WiFi peripheral (access point and station).
     */
    bool WiFiDeInit();

    /**
     * Returns whether the WiFi access point is currently hosting any clients. Used to avoid sending packets to the WiFi
     * AP queue when nobody is listening.
     */
    bool WiFiAccessPointHasClients() { return num_wifi_clients_ > 0; }

    /**
     * Send a raw UDP message to all statiosn that are connected to the ESP32 while operating in access point mode.
     */
    bool WiFiAccessPointSendMessageToAllStations(NetworkMessage& message);

    /**
     * Send messages to stations connected to the ESP32 access point.
     */
    void WiFiAccessPointTask(void* pvParameters);

    /**
     * Returns whether the ESP32 is connected to an external WiFi network as a station.
     * @retval True if connected and assigned IP address, false otherwise.
     */
    inline bool WiFiStationHasIP() { return wifi_sta_has_ip_; }

    // Public so that pass-through functions can access it.
    void WiFiEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

    /**
     * Send messages to feeds that are being fed via an internet or LAN connection.
     */
    void IPWANTask(void* pvParameters);

    /**
     * Sends a composite array of raw packets to feeds via the external IP network that the ESP32 is a station on. It's
     * recommended to only call this function if HasIP() returns true, otherwise it will throw a warning.
     * @param[in] raw_packets_buf Buffer containing the CompositeArray::RawPackets to send. Assumed to be of size
     * CompositeArray::RawPackets::kMaxLenBytes.
     * @retval True if packet was successfully sent, false otherwise.
     */
    bool IPWANSendRawPacketCompositeArray(uint8_t* raw_packets_buf);

    // Network hostname.
    char hostname[SettingsManager::Settings::kHostnameMaxLen + 1] = {0};

    // Ethernet public variables.
    bool ethernet_enabled = false;
    char ethernet_ip[SettingsManager::Settings::kIPAddrStrLen + 1] = {
        0};  // IP address of the ESP32 Ethernet interface.
    char ethernet_netmask[SettingsManager::Settings::kIPAddrStrLen + 1] = {
        0};  // Netmask of the ESP32 Ethernet interface.
    char ethernet_gateway[SettingsManager::Settings::kIPAddrStrLen + 1] = {
        0};  // Gateway of the ESP32 Ethernet interface.

    // WiFi AP public variables.
    bool wifi_ap_enabled = true;
    uint8_t wifi_ap_channel = 1;
    char wifi_ap_ssid[SettingsManager::Settings::kWiFiSSIDMaxLen + 1];          // Add space for null terminator.
    char wifi_ap_password[SettingsManager::Settings::kWiFiPasswordMaxLen + 1];  // Add space for null terminator.

    // WiFi STA public variables.
    bool wifi_sta_enabled = false;
    char wifi_sta_ssid[SettingsManager::Settings::kWiFiSSIDMaxLen + 1];          // Add space for null terminator.
    char wifi_sta_password[SettingsManager::Settings::kWiFiPasswordMaxLen + 1];  // Add space for null terminator.
    char wifi_sta_ip[SettingsManager::Settings::kIPAddrStrLen + 1] = {0};       // IP address of the ESP32 WiFi station.
    char wifi_sta_netmask[SettingsManager::Settings::kIPAddrStrLen + 1] = {0};  // Netmask of the ESP32 WiFi station.
    char wifi_sta_gateway[SettingsManager::Settings::kIPAddrStrLen + 1] = {0};  // Gateway of the ESP32 WiFi station.

    // Feed statistics (messages per second).
    uint16_t feed_mps[SettingsManager::Settings::kMaxNumFeeds] = {0};

    PFBQueue<RawModeSPacket> raw_mode_s_packet_reporting_queue = PFBQueue<RawModeSPacket>(
        {.buf_len_num_elements = kMaxNumModeSPackets, .buffer = raw_mode_s_packet_reporting_queue_buffer_});
    PFBQueue<RawUATADSBPacket> raw_uat_adsb_packet_reporting_queue = PFBQueue<RawUATADSBPacket>(
        {.buf_len_num_elements = kMaxNumUATADSBPackets, .buffer = raw_uat_adsb_packet_reporting_queue_buffer_});
    PFBQueue<RawUATUplinkPacket> raw_uat_uplink_packet_reporting_queue = PFBQueue<RawUATUplinkPacket>(
        {.buf_len_num_elements = kMaxNumUATUplinkPackets, .buffer = raw_uat_uplink_packet_reporting_queue_buffer_});

#include "comms_reporting.hh"

    /**
     * Sends a buffer to a given feed.
     * @param[in] iface Feed index.
     * @param[in] buf Buffer to send.
     * @param[in] buf_len Number of bytes to send.
     * @retval True if bytes were sent successfully, false otherwise.
     */
    bool SendBuf(uint16_t iface, const char* buf, uint16_t buf_len, uint16_t num_msgs = 1);

   private:
    bool ConnectFeedSocket(uint16_t feed_index);
    void CloseFeedSocket(uint16_t feed_index);

    /**
     * Initializes the IP event handler that is common to both Ethernet and WiFi events. Automatically called by
     * WiFiInit() and EthernetInit().
     * @retval True if successfully initialized, false otherwise.
     */
    bool IPInit();

    /**
     * Updates the feed metrics values and prints a cute lil message.
     */
    void UpdateFeedMetrics();

    /**
     * Adds a WiFi client to the WiFi client list. Takes both an IP and a MAC address because this is when the IP
     * address is assigned, and the MAC address will be needed for removing the client later.
     * @param[in] client_ip IP address of the new client.
     * @param[in] client_mac MAC address of the new client.
     */
    void WiFiAddClient(esp_ip4_addr_t client_ip, uint8_t* client_mac);

    /**
     * Removes a WiFi client from the WiFi client list. Takes a MAC address since disconnection events don't come with
     * an IP.
     * @param[in] mac_buf_in 6-Byte buffer containing the MAC address of the client that disconnected.
     */
    void WiFiRemoveClient(uint8_t* mac_buf_in);

    CommsManagerConfig config_;

    bool ethernet_was_initialized_ = false;
    bool wifi_was_initialized_ = false;
    bool ip_event_handler_was_initialized_ = false;

    // Ethernet private variables.
    esp_eth_handle_t ethernet_handle_;
    esp_netif_t* ethernet_netif_ = nullptr;
    bool ethernet_connected_ = false;
    bool ethernet_has_ip_ = false;
    uint32_t ethernet_link_up_timestamp_ms_ = 0;  // This will loop every 49.7 days or so.

    // WiFi AP private variables.
    esp_netif_t* wifi_ap_netif_ = nullptr;
    NetworkClient wifi_clients_list_[SettingsManager::Settings::kWiFiMaxNumClients] = {0, 0, 0};
    uint16_t num_wifi_clients_ = 0;
    SemaphoreHandle_t wifi_clients_list_mutex_;
    QueueHandle_t wifi_ap_message_queue_;

    // WiFi STA private variables.
    esp_netif_t* wifi_sta_netif_ = nullptr;
    QueueHandle_t ip_wan_reporting_composite_array_queue_;
    uint8_t composite_array_scratch_buf_[CompositeArray::RawPackets::kMaxLenBytes] = {};
    bool composite_array_scratch_has_data_ = false;
    TaskHandle_t wifi_ap_task_handle = nullptr;
    TaskHandle_t ip_wan_task_handle = nullptr;
    bool wifi_sta_connected_ = false;
    bool wifi_sta_has_ip_ = false;
    uint32_t wifi_sta_connected_timestamp_ms_ = 0;  // This will loop every 49.7 days or so.

    uint16_t feed_mps_counter_[SettingsManager::Settings::kMaxNumFeeds] = {0};
    uint32_t feed_mps_last_update_timestamp_ms_ = 0;
    int feed_sock_[SettingsManager::Settings::kMaxNumFeeds] = {0};
    bool feed_sock_is_connected_[SettingsManager::Settings::kMaxNumFeeds] = {false};
    uint32_t feed_sock_last_connect_timestamp_ms_[SettingsManager::Settings::kMaxNumFeeds] = {0};

    // Queue for raw packets staged for reporting via comms interfaces.
    RawModeSPacket raw_mode_s_packet_reporting_queue_buffer_[kMaxNumModeSPackets];
    RawUATADSBPacket raw_uat_adsb_packet_reporting_queue_buffer_[kMaxNumUATADSBPackets];
    RawUATUplinkPacket raw_uat_uplink_packet_reporting_queue_buffer_[kMaxNumUATUplinkPackets];

    // Reporting protocol timestamps
    // NOTE: Raw reporting interval used for RAW and BEAST protocols as well as internal functions.
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

#define CONSOLE_ERROR(tag, ...) \
    ESP_LOGE(tag, __VA_ARGS__); \
    comms_manager.LogMessageToCoprocessor(SettingsManager::LogLevel::kErrors, tag, __VA_ARGS__);
#define CONSOLE_WARNING(tag, ...) \
    ESP_LOGW(tag, __VA_ARGS__);   \
    comms_manager.LogMessageToCoprocessor(SettingsManager::LogLevel::kWarnings, tag, __VA_ARGS__);
#define CONSOLE_INFO(tag, ...)  \
    ESP_LOGI(tag, __VA_ARGS__); \
    comms_manager.LogMessageToCoprocessor(SettingsManager::LogLevel::kInfo, tag, __VA_ARGS__);
#define CONSOLE_PRINTF(...)                                                                                \
    do {                                                                                                   \
        char _console_buf[CommsManager::kConsolePrintBufMaxLen];                                           \
        snprintf(_console_buf, sizeof(_console_buf), __VA_ARGS__);                                         \
        printf("%s", _console_buf);                                                                        \
        comms_manager.LogMessageToCoprocessor(SettingsManager::LogLevel::kSilent, "", "%s", _console_buf); \
    } while (0)
