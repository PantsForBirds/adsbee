#pragma once

#include "aircraft_dictionary.hh"
#include "auto_update_manager.hh"
#include "broadcast_ring.hh"
#include "composite_array.hh"
#include "data_structures.hh"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "idle_session_tracker.hh"
#include "settings.hh"
#include "uat_uplink_utils.hh"
#include "websocket_server.hh"

class ADSBeeServer {
   public:
    // Depth of queue for incoming packets from RP2040. The RP2040 sends at most ~62 Mode S packets per composite array
    // and Update() drains the queue whenever it is over half full, so 150 leaves >2 arrays of headroom; deeper queues
    // just eat static RAM that the heap needs.
    static const uint16_t kMaxNumModeSPackets = 150;
    // The in-queues hold two composite arrays of UAT packets from the RP2040, or UnpackRawPacketsBufferToQueues() drops
    // the rest of an array ("Queue full, dropped ... UAT ADS-B").
    static constexpr uint16_t kMaxNumUATADSBPacketsPerCompositeArray =
        (CompositeArray::RawPackets::kMaxLenBytes - sizeof(CompositeArray::RawPackets::Header)) /
        sizeof(RawUATADSBPacket);
    static const uint16_t kMaxNumUATADSBPackets = 2 * kMaxNumUATADSBPacketsPerCompositeArray;
    // A single composite array from the RP2040 can carry this many UAT uplink packets. The in-queue must hold at least
    // that many, or UnpackRawPacketsBufferToQueues() drops the rest of the array ("UAT Uplink queue full, cannot
    // enqueue packet 2 / 3"). The RP2040 flushes every 200 ms, the same as the Update() drain interval, so leave room
    // for two arrays.
    static constexpr uint16_t kMaxNumUATUplinkPacketsPerCompositeArray =
        (CompositeArray::RawPackets::kMaxLenBytes - sizeof(CompositeArray::RawPackets::Header)) /
        sizeof(RawUATUplinkPacket);
    static const uint16_t kMaxNumUATUplinkPackets =
        2 * kMaxNumUATUplinkPacketsPerCompositeArray;  // Depth of queue for incoming UAT uplink packets from RP2040.
    // (The Remote ID -> RP2040 out-queue lives in RemoteIDManager, allocated lazily only when Remote ID runs, so this
    // build pays no internal SRAM for it when Remote ID is disabled. See RemoteIDManager::GetOutQueue().)
    // Interval of the aircraft dictionary update and metrics message while no RP2040 metrics arrive to trigger them.
    static const uint32_t kAircraftDictionaryUpdateFallbackIntervalMs = 1500;
    static const uint32_t kRawPacketProcessingIntervalMs = 200;
    static const uint32_t kGDL90ReportingIntervalMs = 1000;
    static const uint32_t kAircraftJSONReportingIntervalMs = 1000;
    // Update() pass time spent on /aircraft frames, so a large report doesn't hold off draining the packet in-queues.
    static const uint32_t kAircraftJSONPassBudgetUs = 20000;
    // RP2040 metrics older than this are stale and are dropped from the metrics message.
    static const uint32_t kRP2040MetricsStaleTimeoutMs = 3000;

    // UAT uplinks kept for /uplink clients that are not ready to receive. Each update queues the uplinks of at most one
    // composite array and then sends them, so the ring holds one array.
    static constexpr uint16_t kUplinkRingDepth = 8;
    static_assert(kUplinkRingDepth >= kMaxNumUATUplinkPacketsPerCompositeArray);
    static_assert(kUATUplinkMessageNumBytes <= WebSocketServer::kNonBlockingMessageMaxLen);
    static constexpr uint32_t kUplinkDropWarningIntervalMs = 5000;

    // Web server sessions. Every WebSocket client holds one for as long as it is connected. The rest are for HTTP
    // requests: enough for the connections a browser opens in parallel while it loads a page.
    static constexpr uint16_t kNumWebSocketEndpoints = 4;  // /console, /metrics, /aircraft, /uplink.
    static constexpr uint16_t kMaxNumClientsPerWebSocket = 4;
    static constexpr uint16_t kMaxNumHTTPSessions = 6;
    static constexpr uint16_t kMaxNumWebServerSessions =
        kNumWebSocketEndpoints * kMaxNumClientsPerWebSocket + kMaxNumHTTPSessions;
    static constexpr uint16_t kWebServerBacklog = 5;  // Connections waiting for a session.
    // lwIP must hold every web server session next to all feeds, or it runs out before the web server's own limit and
    // the web server never closes a session to make room.
    // Sockets: the web server uses 3 itself, GDL90 uses 1.
    static_assert(kMaxNumWebServerSessions + 3 + SettingsManager::Settings::kMaxNumFeeds + 1 <= CONFIG_LWIP_MAX_SOCKETS);
    static_assert(kMaxNumWebServerSessions + kWebServerBacklog + SettingsManager::Settings::kMaxNumFeeds <=
                  CONFIG_LWIP_MAX_ACTIVE_TCP);
    // An HTTP session unused for this long is closed.
    static constexpr uint32_t kHTTPSessionIdleTimeoutMs = 5000;
    static constexpr uint32_t kHTTPSessionSweepIntervalMs = 1000;
    // Each close is one message in the web server's control queue, which drops messages when it is full.
    static constexpr uint16_t kMaxNumHTTPSessionsClosedPerSweep = 4;
    static_assert(kMaxNumHTTPSessionsClosedPerSweep < CONFIG_LWIP_UDP_RECVMBOX_SIZE);

    static const uint16_t kNetworkConsoleQueueLen = 10;
    // RP2040 console output waiting to be broadcast to /console clients. Sized for one full console write from the
    // RP2040 (CommsManager::kNetworkConsoleBufMaxLen); anything that doesn't fit is dropped.
    static const uint16_t kNetworkConsoleTxBufLenBytes = 4096;
    static const uint16_t kNetworkConsoleTxChunkLenBytes = 512;  // Max bytes per /console frame when draining.

    /**
     * Constructor.
     */
    ADSBeeServer() {
        rp2040_aircraft_dictionary_metrics_queue = xQueueCreate(1, sizeof(AircraftDictionary::Metrics));
        network_console_tx_buf_ = xStreamBufferCreate(kNetworkConsoleTxBufLenBytes, 1);
        raw_packets_buf_ = (uint8_t*)heap_caps_malloc(CompositeArray::RawPackets::kMaxLenBytes, MALLOC_CAP_8BIT);
    };

    /**
     * Destructor.
     */
    ~ADSBeeServer() {
        delete uplink_ring_;
        vQueueDelete(rp2040_aircraft_dictionary_metrics_queue);
        if (raw_packets_buf_) {
            heap_caps_free(raw_packets_buf_);
        }
    }

    bool Init();
    bool Update();

    /**
     * Queues console output from the RP2040 for broadcast to /console clients. Called from the SPI receive task, so it
     * must never block: websocket sends can block for seconds on a client that stops reading, and a blocked SPI task
     * stops all RP2040 <-> ESP32 traffic until the RP2040 power-cycles the ESP32. Update() does the broadcast.
     * @param[in] buf Console bytes.
     * @param[in] buf_len Number of bytes.
     * @retval Number of bytes queued. Bytes that don't fit are dropped.
     */
    size_t QueueNetworkConsoleMessage(const uint8_t* buf, size_t buf_len) {
        auto_update_manager.OnConsoleOutput(buf, buf_len);
        if (!network_console_tx_buf_) return 0;
        return xStreamBufferSend(network_console_tx_buf_, buf, buf_len, 0);
    }

    /**
     * Task that runs continuously to receive SPI messages.
     */
    void SPIReceiveTask();

    /**
     * TCP server that accepts incoming connections (used for network control via port 3333).
     */
    void TCPServerTask(void* pvParameters);

    // Written by the SPI task (core 1) and read by the main task (core 0): one writer and one reader, so unlocked.
    PFBQueue<RawModeSPacket> raw_mode_s_packet_in_queue =
        PFBQueue<RawModeSPacket>({.buf_len_num_elements = kMaxNumModeSPackets,
                                  .buffer = raw_mode_s_packet_in_queue_buffer_});
    PFBQueue<RawUATADSBPacket> raw_uat_adsb_packet_in_queue =
        PFBQueue<RawUATADSBPacket>({.buf_len_num_elements = kMaxNumUATADSBPackets,
                                    .buffer = raw_uat_adsb_packet_in_queue_buffer_});
    PFBQueue<RawUATUplinkPacket> raw_uat_uplink_packet_in_queue =
        PFBQueue<RawUATUplinkPacket>({.buf_len_num_elements = kMaxNumUATUplinkPackets,
                                      .buffer = raw_uat_uplink_packet_in_queue_buffer_});

    AircraftDictionary aircraft_dictionary;

    httpd_handle_t server = nullptr;
    // Used on the web server task only.
    IdleSessionTracker<kMaxNumWebServerSessions> web_server_sessions;
    WebSocketServer network_console;
    WebSocketServer network_metrics;
    WebSocketServer network_aircraft;
    WebSocketServer network_uplink;

    QueueHandle_t rp2040_aircraft_dictionary_metrics_queue = nullptr;
    AircraftDictionary::Metrics rp2040_aircraft_dictionary_metrics;

   private:
    struct WSClientInfo {
        bool in_use = false;
        int client_fd = 0;
        uint32_t last_message_timestamp_ms = 0;
    };

    /**
     * Send a GDL90 report of all aircraft positions to connected WiFi clients.
     * @retval True if successful, false on error.
     */
    bool ReportGDL90();

    /**
     * Sends a GDL90 uplink packet to connected WiFi clients. Does not provide traffic reports.
     * @param[in] uplink_packet Decoded UAT uplink packet to send.
     * @retval True if successful, false on error.
     */
    bool ReportGDL90UplinkDataMessage(const DecodedUATUplinkPacket& uplink_packet);

    /**
     * Broadcasts metrics message to all connected network metrics websocket clients.
     */
    void SendNetworkMetricsMessage();
    // Sends one metrics message unless it was cut short.
    void BroadcastNetworkMetrics(const char* message);

    /**
     * Broadcasts the next frames of the aircraft JSON report to all connected /aircraft websocket clients, until the
     * report is done or the pass budget is used up.
     */
    void SendAircraftJSONMessages();

    /**
     * Queues a UAT uplink for /uplink clients. Does nothing while no client is connected.
     */
    void QueueUplinkMessage(const DecodedUATUplinkPacket& uplink_packet);

    /**
     * Sends queued UAT uplinks to the /uplink clients that can take them without waiting, and holds the queue only
     * while clients are connected.
     */
    void UpdateUplinkBroadcast();

    void SetOwnshipPosition(float latitude_deg, float longitude_deg);

    /**
     * Sets up the TCPServerTask as well as the WebSocket handlers and HTTP server.
     */
    bool TCPServerInit();

    bool spi_receive_task_should_exit_ = false;

    // Queue for raw packets from RP2040.
    RawModeSPacket raw_mode_s_packet_in_queue_buffer_[kMaxNumModeSPackets];
    RawUATADSBPacket raw_uat_adsb_packet_in_queue_buffer_[kMaxNumUATADSBPackets];
    RawUATUplinkPacket raw_uat_uplink_packet_in_queue_buffer_[kMaxNumUATUplinkPackets];

    uint8_t* raw_packets_buf_ = nullptr;
    StreamBufferHandle_t network_console_tx_buf_ = nullptr;

    typedef BroadcastRing<kUATUplinkMessageNumBytes, kUplinkRingDepth, WebSocketServer::kMaxNumClients> UplinkRing;
    UplinkRing* uplink_ring_ = nullptr;  // Allocated while /uplink has clients.
    uint32_t last_uplink_drop_warning_timestamp_ms_ = 0;

    uint32_t last_raw_packet_process_timestamp_ms_ = 0;
    uint32_t last_aircraft_dictionary_update_timestamp_ms_ = 0;
    uint32_t last_gdl90_report_timestamp_ms_ = 0;
    uint32_t last_aircraft_json_report_timestamp_ms_ = 0;
    // Aircraft JSON report in progress: the aircraft to send and the next one.
    uint32_t aircraft_json_uids_[kAircraftDictionaryMaxNumAircraft];
    uint16_t aircraft_json_num_uids_ = 0;
    uint16_t aircraft_json_next_uid_ = 0;
    uint32_t last_rp2040_metrics_timestamp_ms_ = 0;
    uint32_t last_http_session_sweep_timestamp_ms_ = 0;
};

extern ADSBeeServer adsbee_server;