#pragma once

#include "aircraft_dictionary.hh"
#include "composite_array.hh"
#include "data_structures.hh"
#include "esp_http_server.h"
#include "websocket_server.hh"

class ADSBeeServer {
   public:
    // Depth of queue for incoming packets from RP2040. The RP2040 sends at most ~62 Mode S packets per composite array
    // and Update() drains the queue whenever it is over half full, so 150 leaves >2 arrays of headroom; deeper queues
    // just eat static RAM that the heap needs.
    static const uint16_t kMaxNumModeSPackets = 150;
    static const uint16_t kMaxNumUATADSBPackets = 20;   // Depth of queue for incoming UAT ADS-B packets from RP2040.
    static const uint16_t kMaxNumUATUplinkPackets = 2;  // Depth of queue for incoming UAT uplink packets from RP2040.
    // (The Remote ID -> RP2040 out-queue lives in RemoteIDManager, allocated lazily only when Remote ID runs, so this
    // build pays no internal SRAM for it when Remote ID is disabled. See RemoteIDManager::GetOutQueue().)
    static const uint32_t kAircraftDictionaryUpdateIntervalMs = 1000;
    static const uint32_t kRawPacketProcessingIntervalMs = 200;
    static const uint32_t kGDL90ReportingIntervalMs = 1000;
    static const uint32_t kAircraftJSONReportingIntervalMs = 1000;

    static const uint16_t kNetworkConsoleQueueLen = 10;

    /**
     * Constructor.
     */
    ADSBeeServer() {
        rp2040_aircraft_dictionary_metrics_queue = xQueueCreate(1, sizeof(AircraftDictionary::Metrics));
        raw_packets_buf_ = (uint8_t*)heap_caps_malloc(CompositeArray::RawPackets::kMaxLenBytes, MALLOC_CAP_8BIT);
    };

    /**
     * Destructor.
     */
    ~ADSBeeServer() {
        vQueueDelete(rp2040_aircraft_dictionary_metrics_queue);
        if (raw_packets_buf_) {
            heap_caps_free(raw_packets_buf_);
        }
    }

    bool Init();
    bool Update();

    /**
     * Task that runs continuously to receive SPI messages.
     */
    void SPIReceiveTask();

    /**
     * TCP server that accepts incoming connections (used for network control via port 3333).
     */
    void TCPServerTask(void* pvParameters);

    // Written by Core 1 (SPI task) and read by Core 0 (main task) — must be thread-safe.
    // Define PFB_QUEUE_NO_THREAD_SAFETY to disable mutex locking (e.g. to diagnose deadlocks).
    PFBQueue<RawModeSPacket> raw_mode_s_packet_in_queue =
        PFBQueue<RawModeSPacket>({.buf_len_num_elements = kMaxNumModeSPackets,
                                  .buffer = raw_mode_s_packet_in_queue_buffer_,
#ifndef PFB_QUEUE_NO_THREAD_SAFETY
                                  .is_thread_safe = true
#endif
        });
    PFBQueue<RawUATADSBPacket> raw_uat_adsb_packet_in_queue =
        PFBQueue<RawUATADSBPacket>({.buf_len_num_elements = kMaxNumUATADSBPackets,
                                    .buffer = raw_uat_adsb_packet_in_queue_buffer_,
#ifndef PFB_QUEUE_NO_THREAD_SAFETY
                                    .is_thread_safe = true
#endif
        });
    PFBQueue<RawUATUplinkPacket> raw_uat_uplink_packet_in_queue =
        PFBQueue<RawUATUplinkPacket>({.buf_len_num_elements = kMaxNumUATUplinkPackets,
                                      .buffer = raw_uat_uplink_packet_in_queue_buffer_,
#ifndef PFB_QUEUE_NO_THREAD_SAFETY
                                      .is_thread_safe = true
#endif
        });

    AircraftDictionary aircraft_dictionary;

    httpd_handle_t server = nullptr;
    WebSocketServer network_console;
    WebSocketServer network_metrics;
    WebSocketServer network_aircraft;

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

    /**
     * Broadcasts one JSON object per tracked aircraft to all connected /aircraft websocket clients.
     */
    void SendAircraftJSONMessages();

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

    uint32_t last_raw_packet_process_timestamp_ms_ = 0;
    uint32_t last_aircraft_dictionary_update_timestamp_ms_ = 0;
    uint32_t last_gdl90_report_timestamp_ms_ = 0;
    uint32_t last_aircraft_json_report_timestamp_ms_ = 0;
};

extern ADSBeeServer adsbee_server;