#include "adsbee_server.hh"

#include <sys/time.h>

#include <algorithm>
#include <cmath>
#include <ctime>
#include <new>

#include "esp_heap_caps.h"
#include "lwip/sockets.h"

#include "comms.hh"
#include "aircraftjson_utils.hh"
#include "auto_update_manager.hh"
#include "swarm_manager.hh"
#include "feed_ram_budget.hh"
#include "network_time.hh"
#include "gdl90/gdl90_utils.hh"
#include "hardware_capabilities.hh"
#include "heap_diagnostics.hh"
#include "json_utils.hh"
#include "pico.hh"
#include "ble/ble_output.hh"
#include "flarm_nmea/flarm_nmea.hh"
#include "remote_id/remote_id_manager.hh"
#include "settings.hh"
#include "spi_coprocessor.hh"
#include "task_priorities.hh"
#include "unit_conversions.hh"

// #define VERBOSE_DEBUG

static const uint16_t kGDL90Port = 4000;

// True if GDL90 goes to WiFi access point clients.
static bool GDL90ToWiFi() {
    return comms_manager.WiFiAccessPointIsRunning() && comms_manager.WiFiAccessPointHasClients();
}

/**
 * Sends framed GDL90 messages to WiFi access point clients and the Bluetooth client.
 * @param[in] droppable True for FIS-B uplinks, which Bluetooth refuses first when its queue fills.
 * @retval False if the WiFi send failed. A full Bluetooth queue shows in its drop count.
 */
static bool SendGDL90(CommsManager::NetworkMessage& message, bool droppable = false) {
    bool ret = true;
    if (GDL90ToWiFi()) ret = comms_manager.WiFiAccessPointSendMessageToAllStations(message);
    if (ble_output.StreamsMode(SettingsManager::kBLEOutputGDL90)) ble_output.Send(message.data, message.len, droppable);
    return ret;
}

static const uint16_t kNetworkConsoleWelcomeMessageMaxLen = 1000;
static const uint16_t kNetworkMetricsMessageMaxLen = 2048;
static_assert(kNetworkMetricsMessageMaxLen <= WebSocketServer::kNonBlockingMessageMaxLen);

/* obsolete */
static const uint16_t kNetworkControlPort = 3333;  // NOTE: This must match the port number used in index.html!

// TCP Server socket options
static const int kTCPServerSockOptReuseAddr = true;  // Enable TCP server to reuse socket addresses.
static const int kTCPServerSockOptKeepAlive = true;  // Enable TCP socket keepalive for TCP server.
// Number of seconds socket is idle before sending keepalive probes.
static const int kTCPServerSockOptKeepIdleTimeoutSec = 5;
// Number of seconds betwen keepalive packets.
static const int kTCPServerSockOptKeepAliveIntervalSec = 5;
// Number of failed keepalive probes before closing socket.
static const int kTCPServerSockOptMaxFailedKeepAliveCount = 3;
// Number of seconds to wait before giving up on selecting a TCP socket.
static const int kTCPServerSockSelectTimeoutSec = 1;

/**
 * Prints the settings received from the RP2040 with the WiFi passwords masked. SettingsManager::Print() shows the AP
 * password in clear (it also answers AT+SETTINGS? on the RP2040), and on the ESP32 its output goes to the console
 * log, which reaches the RP2040 console and the web UI's /console websocket. Prints a heap copy so the live settings
 * are never modified.
 */
static void PrintSettingsWithoutSecrets() {
    SettingsManager* copy = new (std::nothrow) SettingsManager(settings_manager);
    if (copy == nullptr) {
        CONSOLE_ERROR("ADSBeeServer::Init", "Not enough memory to print the settings.");
        return;
    }
    SettingsManager::Settings::CoreNetworkSettings& cns = copy->settings.core_network_settings;
    // Same mask for every non-empty password so its length isn't revealed either; an empty password stays empty.
    static const char kMaskedPassword[] = "********";
    if (cns.wifi_ap_password[0] != '\0') strcpy(cns.wifi_ap_password, kMaskedPassword);
    if (cns.wifi_sta_password[0] != '\0') strcpy(cns.wifi_sta_password, kMaskedPassword);
    copy->Print();
    delete copy;
}
/* end obsolete */

// Embedded files from the web folder.
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");
extern const uint8_t style_css_start[] asm("_binary_style_css_start");
extern const uint8_t style_css_end[] asm("_binary_style_css_end");
extern const uint8_t adsbee_js_start[] asm("_binary_adsbee_js_start");
extern const uint8_t adsbee_js_end[] asm("_binary_adsbee_js_end");
extern const uint8_t settings_js_start[] asm("_binary_settings_js_start");
extern const uint8_t settings_js_end[] asm("_binary_settings_js_end");
extern const uint8_t ota_updater_js_start[] asm("_binary_ota_updater_js_start");
extern const uint8_t ota_updater_js_end[] asm("_binary_ota_updater_js_end");
extern const uint8_t firmware_release_js_start[] asm("_binary_firmware_release_js_start");
extern const uint8_t firmware_release_js_end[] asm("_binary_firmware_release_js_end");
extern const uint8_t fisb_js_start[] asm("_binary_fisb_js_start");
extern const uint8_t fisb_js_end[] asm("_binary_fisb_js_end");
extern const uint8_t fisb_stations_js_start[] asm("_binary_fisb_stations_js_start");
extern const uint8_t fisb_stations_js_end[] asm("_binary_fisb_stations_js_end");
extern const uint8_t favicon_png_start[] asm("_binary_favicon_png_start");
extern const uint8_t favicon_png_end[] asm("_binary_favicon_png_end");

extern GDL90Reporter gdl90;

/** "Pass-Through" functions used to access member functions in callbacks. **/

void esp_spi_receive_task(void* pvParameters) {
    adsbee_server.SPIReceiveTask();  // Only returns during DeInit.
    vTaskDelete(NULL);               // Delete this task.
}

void tcp_server_task(void* pvParameters) { adsbee_server.TCPServerTask(pvParameters); }
// esp_err_t console_ws_handler(httpd_req_t *req) { return adsbee_server.NetworkConsoleWebSocketHandler(req); }
static bool web_server_fd_is_websocket(httpd_handle_t hd, int sockfd) {
    return httpd_ws_get_fd_info(hd, sockfd) == HTTPD_WS_CLIENT_WEBSOCKET;
}

/**
 * Asks the web server to close the HTTP session unused for the longest time, if that is at least idle_timeout_ms.
 * WebSockets and the session except_fd are never closed. The request is queued and can get lost; the session stays
 * tracked until it is closed, and restarting its timeout makes a later call ask again.
 * @retval True if a session was found.
 */
static bool web_server_close_idle_session(httpd_handle_t hd, uint32_t idle_timeout_ms, int except_fd = -1) {
    uint32_t timestamp_ms = get_time_since_boot_ms();
    int fd = adsbee_server.web_server_sessions.NextIdle(timestamp_ms, idle_timeout_ms, [hd, except_fd](int fd) {
        return fd == except_fd || web_server_fd_is_websocket(hd, fd);
    });
    if (fd < 0) {
        return false;
    }
    adsbee_server.web_server_sessions.Touch(fd, timestamp_ms);
    httpd_sess_trigger_close(hd, fd);
    return true;
}

static bool web_server_sessions_are_full() {
    return adsbee_server.web_server_sessions.NumSessions() >= ADSBeeServer::kMaxNumWebServerSessions;
}

/**
 * Runs on the web server task for every new session. Keeps one session free for the next client.
 */
esp_err_t web_server_open_fd(httpd_handle_t hd, int sockfd) {
    adsbee_server.web_server_sessions.Open(sockfd, get_time_since_boot_ms());
    // The web server ignores a request to close a session that has no use count, which is the case until its first
    // HTTP request.
    httpd_sess_update_lru_counter(hd, sockfd);
    if (web_server_sessions_are_full()) {
        web_server_close_idle_session(hd, 0, sockfd);
    }
    return ESP_OK;
}

/**
 * Runs on the web server task. Closes HTTP sessions that a client keeps open without sending a request, and makes
 * room again if a request to close a session got lost while every session was in use.
 */
void web_server_close_idle_sessions(void* arg) {
    httpd_handle_t hd = adsbee_server.server;
    uint16_t num_closed = 0;
    while (num_closed < ADSBeeServer::kMaxNumHTTPSessionsClosedPerSweep &&
           web_server_close_idle_session(hd, ADSBeeServer::kHTTPSessionIdleTimeoutMs)) {
        num_closed++;
    }
    if (num_closed == 0 && web_server_sessions_are_full()) {
        web_server_close_idle_session(hd, 0);
    }
}

void ws_close_fd(httpd_handle_t hd, int sockfd) {
    adsbee_server.web_server_sessions.Close(sockfd);
    adsbee_server.network_console.RemoveClient(sockfd);
    adsbee_server.network_metrics.RemoveClient(sockfd);
    adsbee_server.network_aircraft.RemoveClient(sockfd);
    adsbee_server.network_uplink.RemoveClient(sockfd);
    // SO_LINGER with l_linger=0 sends RST instead of FIN, bypassing TCP TIME_WAIT.
    // Without this, graceful FIN/FIN-ACK teardown (which always happens on Ethernet) accumulates
    // TIME_WAIT entries in LWIP's PCB table (2 * MSL per close), exhausting the pool and blocking
    // new connections. WiFi avoids this because link events cause RST, not FIN.
    struct linger linger_opt = {.l_onoff = 1, .l_linger = 0};
    setsockopt(sockfd, SOL_SOCKET, SO_LINGER, &linger_opt, sizeof(linger_opt));
    close(sockfd);
}
/** End "Pass-Through" functions. **/

bool ADSBeeServer::Init() {
    if (!pico.Init()) {
        CONSOLE_ERROR("ADSBeeServer::Init", "SPI Coprocessor initialization failed.");
        return false;
    }

    HeapDiagnostics::Mark("pico");
    // Initialize SPI receive task before requesting settings so that we can tend to messages from the RP2040 and stop
    // it from freaking out.
    spi_receive_task_should_exit_ = false;
    // SPI receive task is high priority but we can't starve the WiFi task, which is pinned to core 0 in the sdkconfig.
    xTaskCreatePinnedToCoreWithCaps(esp_spi_receive_task, "spi_receive_task", kSPIReceiveTaskStackSizeBytes, NULL,
                                    kSPIReceiveTaskPriority, NULL, kSPIReceiveTaskCore, MALLOC_CAP_INTERNAL);

    // Initialize prerequisites for Ethernet and WiFi. Needs to be done before settings are applied.
    comms_manager.Init();
    HeapDiagnostics::Mark("comms");

    SemaphoreHandle_t settings_read_semaphore = xSemaphoreCreateBinary();
    if (settings_read_semaphore == NULL) {
        CONSOLE_ERROR("ADSBeeServer::Init", "Failed to create settings read semaphore.");
        return false;
    }

    object_dictionary.RequestSCCommand(ObjectDictionary::SCCommandRequestWithCallback{
        .request =
            ObjectDictionary::SCCommandRequest{.command = ObjectDictionary::SCCommand::kCmdWriteToSlaveRequireAck,
                                               .addr = ObjectDictionary::Address::kAddrSettingsData,
                                               .offset = 0,
                                               .len = sizeof(SettingsManager::Settings)},
        .complete_callback =
            [settings_read_semaphore]() {
                CONSOLE_INFO("ADSBeeServer::Init", "Settings data read from Pico.");
                xSemaphoreGive(settings_read_semaphore);
            },
    });  // Require ack.

    // Wait for the callback to complete. If the RP2040 runs firmware with a different Settings layout, it rejects this
    // request (length mismatch) and it never completes. Continuing without settings isn't safe (defaults would bring up
    // a default WiFi AP and start the default feeds), so keep waiting, but report why instead of hanging silently. Log
    // messages still reach the RP2040 console over SPI while we wait.
    static constexpr uint32_t kSettingsWaitReportIntervalMs = 10000;
    uint32_t settings_wait_s = 0;
    while (xSemaphoreTake(settings_read_semaphore, pdMS_TO_TICKS(kSettingsWaitReportIntervalMs)) != pdTRUE) {
        settings_wait_s += kSettingsWaitReportIntervalMs / 1000;
        CONSOLE_ERROR("ADSBeeServer::Init",
                      "No settings from the RP2040 after %lu s. This ESP32 firmware expects %u-byte version %lu "
                      "settings; if the RP2040 firmware differs, reflash the ESP32 (AT+ESP32_FLASH or a reboot).",
                      (unsigned long)settings_wait_s, (unsigned)sizeof(SettingsManager::Settings),
                      (unsigned long)kSettingsVersion);
    }
    vSemaphoreDelete(settings_read_semaphore);
    PrintSettingsWithoutSecrets();
    settings_manager.Apply();
    HeapDiagnostics::Mark("settings");

    return TCPServerInit();
}

bool ADSBeeServer::Update() {
    bool ret = true;
    // Do NOT call pico.Update() from here since that's already taken care of by the SPIReceiveTask.
    // Update the LED here so it has better time resolution than it would in the SPI task, which blocks frequently.
    pico.UpdateLED();

    uint32_t timestamp_ms = get_time_since_boot_ms();

    auto_update_manager.Update();
    swarm_manager.Update();
    network_time.Update();

    // Ingest any Broadcast Remote ID advertisements received on the BLE/WiFi radios into the local aircraft dictionary
    // (and forward rate-limited copies up to the RP2040), then service the WiFi channel hopper. Done on this task so all
    // aircraft-dictionary mutation stays single-threaded.
    remote_id_manager.ServiceIngestQueue();
    remote_id_manager.Update();
    ble_output.Update();

    // Prune aircraft dictionary. Need to do this up front so that we don't end up with a negative timestamp delta
    // caused by packets being ingested more recently than the timestamp we take at the beginning of this function.
    // The RP2040 sends its metrics once per second on its own clock. Run this block when they arrive, so that every set
    // of RP2040 metrics goes out in exactly one metrics message, and on a timer while none arrive.
    bool received_rp2040_metrics =
        xQueueReceive(rp2040_aircraft_dictionary_metrics_queue, &rp2040_aircraft_dictionary_metrics, 0) == pdTRUE;
    if (received_rp2040_metrics) {
        last_rp2040_metrics_timestamp_ms_ = timestamp_ms;
        // The RP2040 sees every frame; the ESP32 only those forwarded to it.
        const AircraftDictionary::Metrics& m = rp2040_aircraft_dictionary_metrics;
        swarm_manager.AddFrames(m.valid_squitter_frames, m.valid_extended_squitter_frames, m.valid_uat_adsb_frames,
                                m.valid_uat_uplink_frames);
    }
    if (received_rp2040_metrics ||
        timestamp_ms - last_aircraft_dictionary_update_timestamp_ms_ > kAircraftDictionaryUpdateFallbackIntervalMs) {
        if (object_dictionary.composite_device_status.rp2040.rx_position_available) {
            aircraft_dictionary.SetReferencePosition(
                object_dictionary.composite_device_status.rp2040.rx_position.latitude_deg,
                object_dictionary.composite_device_status.rp2040.rx_position.longitude_deg);
            aircraft_dictionary.SetReferenceAltitudeFt(
                object_dictionary.composite_device_status.rp2040.rx_position.FilterAltitudeFt());
        } else {
            aircraft_dictionary.ClearReferencePosition();
        }

        aircraft_dictionary.Update(timestamp_ms);
        last_aircraft_dictionary_update_timestamp_ms_ = timestamp_ms;
        uint32_t heap_free_kb = heap_caps_get_free_size(MALLOC_CAP_8BIT) / 1024;
        uint32_t heap_total_kb = heap_caps_get_total_size(MALLOC_CAP_8BIT) / 1024;
        uint16_t ws_used = network_console.GetNumClients() + network_metrics.GetNumClients() +
                           network_aircraft.GetNumClients() + network_uplink.GetNumClients();
        uint16_t ws_total = network_console.GetNumClientsAllowed() + network_metrics.GetNumClientsAllowed() +
                            network_aircraft.GetNumClientsAllowed() + network_uplink.GetNumClientsAllowed();
        CONSOLE_INFO("ADSBeeServer::Update", "\t %d clients, %d aircraft, heap: %luK free / %luK total, ws: %d/%d",
                     comms_manager.GetNumWiFiClients(), aircraft_dictionary.GetNumAircraft(), heap_free_kb,
                     heap_total_kb, ws_used, ws_total);

        object_dictionary.UpdateDeviceStatus();  // Get newest values for ESP32 status.
        // RP2040 and SubG status are updated by the RP2040 writing to the object dictionary periodically. Nothing to do
        // here.
        SendNetworkMetricsMessage();
        last_network_metrics_timestamp_ms_ = timestamp_ms;
    }

    if (timestamp_ms - last_http_session_sweep_timestamp_ms_ > kHTTPSessionSweepIntervalMs) {
        last_http_session_sweep_timestamp_ms_ = timestamp_ms;
        if (server != nullptr) {
            httpd_queue_work(server, web_server_close_idle_sessions, nullptr);
        }
    }

    // Run raw packet ingestion and reporting if queues are >50% full or every 200ms.
    bool at_least_one_queue_half_full =
        (raw_mode_s_packet_in_queue.Length() > raw_mode_s_packet_in_queue.MaxNumElements() / 2) ||
        (raw_uat_adsb_packet_in_queue.Length() > raw_uat_adsb_packet_in_queue.MaxNumElements() / 2) ||
        (raw_uat_uplink_packet_in_queue.Length() > raw_uat_uplink_packet_in_queue.MaxNumElements() / 2);

    if (timestamp_ms - last_raw_packet_process_timestamp_ms_ > kRawPacketProcessingIntervalMs ||
        at_least_one_queue_half_full) {
        last_raw_packet_process_timestamp_ms_ = timestamp_ms;

        // Check that buffer was allocated successfully.
        if (!raw_packets_buf_) {
            CONSOLE_ERROR("ADSBeeServer::Update", "Raw packets buffer not allocated. Cannot process packets.");
            return false;
        }

        // Assemble a CompositeArray of transponder packets to report.
        // NOTE: Rate metering of raw packets is done by upstream RP2040, which only forward packets on the raw
        // packet reporting time interval.
        CompositeArray::RawPackets raw_packets = CompositeArray::PackRawPacketsBuffer(
            raw_packets_buf_, CompositeArray::RawPackets::kMaxLenBytes, &(adsbee_server.raw_mode_s_packet_in_queue),
            &(adsbee_server.raw_uat_adsb_packet_in_queue), &(adsbee_server.raw_uat_uplink_packet_in_queue));

        if (!raw_packets.IsValid()) {
            if (raw_packets.len_bytes > sizeof(CompositeArray::RawPackets::Header)) {
                CONSOLE_ERROR("ADSBeeServer::Update",
                              "Invalid CompositeArray of transponder packets (len_bytes = %u). Dropping packets.",
                              raw_packets.len_bytes);
            }
            // Else it's OK to have no packets.
        } else if (raw_packets.len_bytes > sizeof(CompositeArray::RawPackets::Header)) {
            // Report raw packets then add them to the aircraft dictionary. Report first to allow network queues to be
            // processed during ingestion, which takes a while.

            // Forward packets to WAN.
            comms_manager.IPWANSendRawPacketCompositeArray(raw_packets_buf_);

            // Mode S Packets
            for (uint16_t i = 0; i < raw_packets.header->num_mode_s_packets; i++) {
                RawModeSPacket& raw_mode_s_packet = raw_packets.mode_s_packets[i];
                DecodedModeSPacket decoded_packet = DecodedModeSPacket(raw_mode_s_packet);

                // Assume all packets being sent are valid
                /*if (!decoded_packet.is_valid ) {
                    CONSOLE_ERROR("ADSBeeServer::Update", "Received invalid Mode S packet.");
                    continue;
                }
                */
#ifdef VERBOSE_DEBUG
                if (raw_mode_s_packet.buffer_len_bytes == RawModeSPacket::kExtendedSquitterPacketLenBytes) {
                    CONSOLE_INFO("ADSBeeServer::Update", "New message: 0x%08lx|%08lx|%08lx|%04lx RSSI=%ddBm MLAT=%llu",
                                 raw_mode_s_packet.buffer[0], raw_mode_s_packet.buffer[1], raw_mode_s_packet.buffer[2],
                                 (raw_mode_s_packet.buffer[3]) >> (4 * kBitsPerNibble), raw_mode_s_packet.sigs_dbm,
                                 raw_mode_s_packet.mlat_48mhz_64bit_counts);
                } else {
                    CONSOLE_INFO("ADSBeeServer::Update", "New message: 0x%08lx|%06lx RSSI=%ddBm MLAT=%llu",
                                 raw_mode_s_packet.buffer[0], (raw_mode_s_packet.buffer[1]) >> (2 * kBitsPerNibble),
                                 raw_mode_s_packet.sigs_dbm, raw_mode_s_packet.mlat_48mhz_64bit_counts);
                }
                CONSOLE_INFO("ADSBeeServer::Update", "\tdf=%d icao_address=0x%06lx", decoded_packet.downlink_format,
                             decoded_packet.icao_address);
#endif

                if (!aircraft_dictionary.IngestDecodedModeSPacket(decoded_packet)) {
                    // NOTE: Pushing to a queue here will only forward valid packets!
                    CONSOLE_ERROR("ADSBeeServer::Update",
                                  "Failed to ingest Mode S packet into aircraft dictionary: DF=%d ICAO=0x%06lx.",
                                  decoded_packet.downlink_format, decoded_packet.icao_address);
                }
            }

            // UAT ADSB Packets
            for (uint16_t i = 0; i < raw_packets.header->num_uat_adsb_packets; i++) {
                RawUATADSBPacket& raw_uat_adsb_packet = raw_packets.uat_adsb_packets[i];
                DecodedUATADSBPacket decoded_packet = DecodedUATADSBPacket(raw_uat_adsb_packet);
                if (!decoded_packet.is_valid) {
                    CONSOLE_ERROR("ADSBeeServer::Update", "Received invalid UAT ADSB packet.");
                    continue;
                }
                if (!aircraft_dictionary.IngestDecodedUATADSBPacket(decoded_packet)) {
                    CONSOLE_ERROR("ADSBeeServer::Update",
                                  "Failed to ingest decoded UAT ADSB packet into aircraft dictionary.");
                }
            }

            // UAT Uplink Packets
            // We don't currently digest uplink packets, they just get forwarded. The raw_packets_buf composite packet
            // array takes care of forwarding to relevant feeds. We also forward them over GDL90 to connected WiFi
            // clients here (separate from the traffic reporting that occurs on regular intervals), and to /uplink
            // WebSocket clients.
            for (uint16_t i = 0; i < raw_packets.header->num_uat_uplink_packets; i++) {
                RawUATUplinkPacket& raw_uat_uplink_packet = raw_packets.uat_uplink_packets[i];
                DecodedUATUplinkPacket decoded_packet = DecodedUATUplinkPacket(raw_uat_uplink_packet);
                if (!decoded_packet.is_valid) {
                    CONSOLE_ERROR("ADSBeeServer::Update", "Received invalid UAT Uplink packet.");
                    continue;
                }
                ReportGDL90UplinkDataMessage(decoded_packet);
                QueueUplinkMessage(decoded_packet);
            }
        }
    }

    // Broadcast aircraft locations over GDL90 to WiFi access point clients and the Bluetooth client.
    if (timestamp_ms - last_gdl90_report_timestamp_ms_ > kGDL90ReportingIntervalMs) {
        last_gdl90_report_timestamp_ms_ = timestamp_ms;
        if ((GDL90ToWiFi() || ble_output.StreamsMode(SettingsManager::kBLEOutputGDL90)) && !ReportGDL90()) {
            CONSOLE_ERROR("ADSBeeServer::Update", "Encountered error while reporting GDL90.");
            ret = false;
        }
        if (ble_output.StreamsMode(SettingsManager::kBLEOutputFLARMNMEA)) ReportFLARMNMEA();
    }

    // Broadcast aircraft JSON to connected /aircraft WebSocket clients. A report starts once the last one is done.
    // Reports start between metrics messages (or on their own while none are sent).
    const uint32_t ms_since_metrics = timestamp_ms - last_network_metrics_timestamp_ms_;
    const bool between_metrics = (ms_since_metrics >= kAircraftJSONMetricsGapMs &&
                                  ms_since_metrics < kAircraftJSONReportingIntervalMs - kAircraftJSONMetricsGapMs) ||
                                 ms_since_metrics >= 2 * kAircraftJSONReportingIntervalMs;
    if (aircraft_json_next_uid_ >= aircraft_json_num_uids_ && between_metrics &&
        timestamp_ms - last_aircraft_json_report_timestamp_ms_ >=
            kAircraftJSONReportingIntervalMs - kAircraftJSONMetricsGapMs) {
        last_aircraft_json_report_timestamp_ms_ = timestamp_ms;
        aircraft_json_num_uids_ =
            network_aircraft.GetNumClients() > 0
                ? ListAircraftJSONLiveMapUIDs(aircraft_dictionary, timestamp_ms, aircraft_json_uids_,
                                              kAircraftDictionaryMaxNumAircraft,
                                              aircraft_dictionary.GetReportFilter(settings_manager.settings))
                : 0;
        aircraft_json_next_uid_ = 0;
    }
    SendAircraftJSONMessages();

    // Prune inactive WebSocket clients and other housekeeping.
    // Broadcast RP2040 console output queued by the SPI receive task.
    if (network_console_tx_buf_) {
        char console_chunk[kNetworkConsoleTxChunkLenBytes];
        size_t len;
        while ((len = xStreamBufferReceive(network_console_tx_buf_, console_chunk, sizeof(console_chunk), 0)) > 0) {
            network_console.BroadcastMessage(console_chunk, len);
        }
    }
    network_console.Update();
    network_metrics.Update();
    network_aircraft.Update();
    network_uplink.Update();
    UpdateUplinkBroadcast();

    return ret;
}

void ADSBeeServer::SPIReceiveTask() {
    CONSOLE_INFO("ADSBeeServer::SPIReceiveTask", "Started SPI receive task.");

    while (!spi_receive_task_should_exit_) {
        // Wait for a transaction to complete. Allow this task to block if no SPI transaction is received by using
        // max delay.
        pico.Update();
        if (pico_ll.SPIClaimNextTransaction()) {
            pico_ll.SPIReleaseNextTransaction();
        } else {
            CONSOLE_ERROR("ADSBeeServer::SPIReceiveTask", "Failed to bump SPI context mutex for lower priority tasks.");
        }
    }

    CONSOLE_INFO("esp_spi_receive_task", "Received exit signal, ending SPI receive task.");
}

bool ADSBeeServer::ReportGDL90() {
    ble_output.BeginReport();
    CommsManager::NetworkMessage message;
    message.port = kGDL90Port;

    // Build the ownship report up front from the receiver position pushed up by the RP2040. Its validity also
    // drives the heartbeat's GPS Position Valid flag, which some EFBs require before they will show ownship.
    const ObjectDictionary::RP2040DeviceStatus& rp2040_status = object_dictionary.composite_device_status.rp2040;
    GDL90Reporter::GDL90TargetReportData ownship_data;
    bool have_position = GDL90Reporter::BuildOwnshipReportData(ownship_data, rp2040_status.rx_position,
                                                               rp2040_status.rx_position_available);

    // Heartbeat status flags are sticky members of the shared GDL90Reporter, so set them on every report.
    gdl90.gnss_position_valid = have_position;
    gdl90.utc_timing_is_valid = rp2040_status.gnss_utc_time_valid;
    gdl90.maintenance_required = false;

    // Heartbeat Message
    message.len = gdl90.WriteGDL90HeartbeatMessage(
        message.data, CommsManager::NetworkMessage::kMaxLenBytes, get_time_since_boot_ms() / 1000,
        aircraft_dictionary.metrics.valid_squitter_frames + aircraft_dictionary.metrics.valid_extended_squitter_frames +
            aircraft_dictionary.metrics.valid_uat_adsb_frames,  // ADS-B message count includes UAT ADS-B messages.
        aircraft_dictionary.metrics.valid_uat_uplink_frames);
    SendGDL90(message);
    message.len = 0;

    // Ownship Report
    message.len = gdl90.WriteGDL90TargetReportMessage(message.data, CommsManager::NetworkMessage::kMaxLenBytes,
                                                      ownship_data, true);
    SendGDL90(message);
    message.len = 0;

    // Traffic Reports
    const bool to_wifi = GDL90ToWiFi();
    const bool to_ble = ble_output.StreamsMode(SettingsManager::kBLEOutputGDL90);
    uint32_t timestamp_ms = get_time_since_boot_ms();
    AircraftReportFilter report_filter = aircraft_dictionary.GetReportFilter(settings_manager.settings);
    // Writes the traffic report of one dictionary entry; 0 if it isn't reported.
    auto write_traffic = [&](auto& itr, uint8_t* buf, uint16_t buf_len, bool log) -> uint16_t {
        if (!aircraft_dictionary.IsPreferredReportForAddress(itr.first, timestamp_ms)) {
            return 0;  // Another entry (e.g. the aircraft's own ADS-B) is reported for this ICAO address.
        }
        if (std::visit([&report_filter](const auto& a) { return report_filter.Excludes(a); }, itr.second)) {
            return 0;  // AT+FILTER_DISTANCE_NM, AT+FILTER_ALTITUDE_FT.
        }
        if (ModeSAircraft* mode_s_aircraft = get_if<ModeSAircraft>(&(itr.second)); mode_s_aircraft) {
            if (!mode_s_aircraft->HasBitFlag(ModeSAircraft::kBitFlagPositionValid) ||
                mode_s_aircraft->icao_address == ownship_data.participant_address) {
                // Don't report aircraft without a valid position, and ignore ownship position when bootstrapping off a
                // fixed ICAO.
                return 0;
            }
            if (log) {
                printf("\t#A %s (0x%06lX): %.5f %.5f %ld\r\n", mode_s_aircraft->callsign,
                       mode_s_aircraft->icao_address, mode_s_aircraft->latitude_deg, mode_s_aircraft->longitude_deg,
                       mode_s_aircraft->baro_altitude_ft);
            }
            return gdl90.WriteGDL90TargetReportMessage(buf, buf_len, *mode_s_aircraft, false);
        }
        if (UATAircraft* uat_aircraft = get_if<UATAircraft>(&(itr.second)); uat_aircraft) {
            if (!uat_aircraft->HasBitFlag(UATAircraft::kBitFlagPositionValid) ||
                uat_aircraft->icao_address == ownship_data.participant_address) {
                return 0;
            }
            if (log) {
                printf("\t#U %s (0x%06lX): %.5f %.5f %ld\r\n", uat_aircraft->callsign, uat_aircraft->icao_address,
                       uat_aircraft->latitude_deg, uat_aircraft->longitude_deg, uat_aircraft->baro_altitude_ft);
            }
            return gdl90.WriteGDL90TargetReportMessage(buf, buf_len, *uat_aircraft, false);
        }
        if (RemoteIDAircraft* remote_id_aircraft = get_if<RemoteIDAircraft>(&(itr.second)); remote_id_aircraft) {
            if (!remote_id_aircraft->HasBitFlag(RemoteIDAircraft::kBitFlagPositionValid)) {
                return 0;  // Don't report drones without a valid position.
            }
            return gdl90.WriteGDL90TargetReportMessage(buf, buf_len, *remote_id_aircraft, false);
        }
        if (log) {
            CONSOLE_WARNING("CommsManager::ReportGDL90", "Unknown aircraft type in dictionary for UID 0x%lx.",
                            itr.first);
        }
        return 0;
    };

    // WiFi gets the aircraft in dictionary order, packed into datagrams. Bluetooth gets one message per aircraft,
    // starting at ble_output.TrafficStart() and wrapping around (second loop), so a full queue skips other aircraft
    // next time.
    uint8_t aircraft_msg_buf[CommsManager::NetworkMessage::kMaxLenBytes];
    const uint16_t ble_start = ble_output.TrafficStart();
    uint16_t num_reported = 0;
    for (auto& itr : aircraft_dictionary.dict) {
        const uint16_t aircraft_msg_buf_len = write_traffic(itr, aircraft_msg_buf, sizeof(aircraft_msg_buf), true);
        if (aircraft_msg_buf_len == 0) continue;
        if (to_ble && num_reported >= ble_start) {
            ble_output.SendTraffic(aircraft_msg_buf, aircraft_msg_buf_len, num_reported);
        }
        num_reported++;
        if (!to_wifi) continue;
        if (message.len + aircraft_msg_buf_len <= CommsManager::NetworkMessage::kMaxLenBytes) {
            // We can tack this aircraft message onto the existing message to save space in the queue.
            memcpy(message.data + message.len, aircraft_msg_buf, aircraft_msg_buf_len);
            message.len += aircraft_msg_buf_len;
        } else {
            // Send the existing message and start a new one.
            if (!comms_manager.WiFiAccessPointSendMessageToAllStations(message)) {
                CONSOLE_ERROR("ADSBeeServer::ReportGDL90", "Failed to send info about aircraft %u to all clients.",
                              num_reported);
            }
            memcpy(message.data, aircraft_msg_buf, aircraft_msg_buf_len);
            message.len = aircraft_msg_buf_len;
        }
    }
    if (to_ble && ble_start > 0) {
        uint16_t index = 0;
        for (auto& itr : aircraft_dictionary.dict) {
            if (index >= ble_start) break;
            const uint16_t aircraft_msg_buf_len = write_traffic(itr, aircraft_msg_buf, sizeof(aircraft_msg_buf), false);
            if (aircraft_msg_buf_len == 0) continue;
            ble_output.SendTraffic(aircraft_msg_buf, aircraft_msg_buf_len, index);
            index++;
        }
    }
    if (to_ble) ble_output.EndTraffic(num_reported);

    // Send any remaining message data.
    if (to_wifi && message.len > 0) {
        if (!comms_manager.WiFiAccessPointSendMessageToAllStations(message)) {
            CONSOLE_ERROR("ADSBeeServer::ReportGDL90", "Failed to send final aircraft message to all clients.");
        }
    }

    return true;
}

namespace {
// UTC for GPRMC / GPGGA: GNSS time of day, else the ESP32 clock once NTP has set it. The date comes from the clock.
FLARMNMEAReporter::UTCTime FLARMNMEAUTCTime(const ObjectDictionary::RP2040DeviceStatus& rp2040_status) {
    FLARMNMEAReporter::UTCTime utc;
    struct timeval tv;
    struct tm tm_utc;
    // Before NTP the clock counts from 1970.
    const bool clock_set =
        gettimeofday(&tv, nullptr) == 0 && gmtime_r(&tv.tv_sec, &tm_utc) != nullptr && tm_utc.tm_year + 1900 >= 2025;
    if (clock_set) {
        utc.date_valid = true;
        utc.year = static_cast<uint16_t>(tm_utc.tm_year + 1900);
        utc.month = static_cast<uint8_t>(tm_utc.tm_mon + 1);
        utc.day = static_cast<uint8_t>(tm_utc.tm_mday);
    }
    if (rp2040_status.gnss_utc_time_valid) {
        utc.time_valid = true;
        utc.hour = rp2040_status.gnss_utc_hour % 24;
        utc.minute = rp2040_status.gnss_utc_minute % 60;
        utc.second = rp2040_status.gnss_utc_second % 60;
    } else if (clock_set) {
        utc.time_valid = true;
        utc.hour = static_cast<uint8_t>(tm_utc.tm_hour);
        utc.minute = static_cast<uint8_t>(tm_utc.tm_min);
        utc.second = static_cast<uint8_t>(tm_utc.tm_sec);
    }
    return utc;
}
}  // namespace

void ADSBeeServer::ReportFLARMNMEA() {
    ble_output.BeginReport();
    const ObjectDictionary::RP2040DeviceStatus& rp2040_status = object_dictionary.composite_device_status.rp2040;
    FLARMNMEAReporter::Ownship own;
    const bool have_ownship =
        FLARMNMEAReporter::OwnshipFromRxPosition(own, rp2040_status.rx_position, rp2040_status.rx_position_available);
    // Our own ADS-B, when ownship comes from it.
    const uint32_t ownship_icao =
        have_ownship && rp2040_status.rx_position.source == SettingsManager::RxPosition::kPositionSourceAircraftMatchingICAO
            ? rp2040_status.rx_position.icao_address
            : UINT32_MAX;

    // Reported aircraft, nearest first. Without ownship they are only counted for PFLAU.
    struct TrafficRef {
        uint32_t distance_m;
        const AircraftDictionary::AircraftEntry* entry;
    };
    TrafficRef* traffic = have_ownship ? new (std::nothrow) TrafficRef[AircraftDictionary::kMaxNumAircraft] : nullptr;
    uint16_t num_traffic = 0;
    uint16_t num_received = 0;
    const uint32_t timestamp_ms = get_time_since_boot_ms();
    AircraftReportFilter report_filter = aircraft_dictionary.GetReportFilter(settings_manager.settings);
    FLARMNMEAReporter::Target target;
    for (auto& itr : aircraft_dictionary.dict) {
        if (!aircraft_dictionary.IsPreferredReportForAddress(itr.first, timestamp_ms)) continue;
        if (std::visit([&report_filter](const auto& a) { return report_filter.Excludes(a); }, itr.second)) continue;
        if (!std::visit([&target](const auto& a) { return FLARMNMEAReporter::TargetFromAircraft(target, a); },
                        itr.second)) {
            continue;  // No position.
        }
        if (Aircraft::UIDToAircraftType(itr.first) != Aircraft::kAircraftTypeRemoteID && target.id == ownship_icao) {
            continue;
        }
        num_received++;
        int32_t north_m, east_m, vertical_m;
        if (traffic == nullptr || num_traffic >= AircraftDictionary::kMaxNumAircraft ||
            !FLARMNMEAReporter::Relative(own, target, north_m, east_m, vertical_m)) {
            continue;
        }
        traffic[num_traffic++] = {
            .distance_m = static_cast<uint32_t>(sqrt(static_cast<double>(north_m) * north_m +
                                                     static_cast<double>(east_m) * east_m)),
            .entry = &itr.second};
    }
    if (traffic != nullptr) {
        std::sort(traffic, traffic + num_traffic,
                  [](const TrafficRef& a, const TrafficRef& b) { return a.distance_m < b.distance_m; });
    }

    char sentence[FLARMNMEAReporter::kMaxSentenceLenBytes + 1];
    uint16_t len;
    if (have_ownship) {
        const FLARMNMEAReporter::UTCTime utc = FLARMNMEAUTCTime(rp2040_status);
        if ((len = FLARMNMEAReporter::WriteGPRMC(sentence, sizeof(sentence), own, utc)) > 0) {
            ble_output.Send(reinterpret_cast<uint8_t*>(sentence), len);
        }
        if ((len = FLARMNMEAReporter::WritePGRMZ(sentence, sizeof(sentence), own)) > 0) {
            ble_output.Send(reinterpret_cast<uint8_t*>(sentence), len);
        }
        if ((len = FLARMNMEAReporter::WriteGPGGA(sentence, sizeof(sentence), own, utc)) > 0) {
            ble_output.Send(reinterpret_cast<uint8_t*>(sentence), len);
        }
    }
    // The heartbeat goes before traffic so a full queue never drops it.
    const uint8_t gps = !have_ownship ? 0 : (own.airborne ? 2 : 1);
    len = FLARMNMEAReporter::WritePFLAU(sentence, sizeof(sentence), num_received, gps);
    ble_output.Send(reinterpret_cast<uint8_t*>(sentence), len);

    for (uint16_t i = 0; i < num_traffic; i++) {
        std::visit([&target](const auto& a) { FLARMNMEAReporter::TargetFromAircraft(target, a); },
                   *traffic[i].entry);
        len = FLARMNMEAReporter::WritePFLAA(sentence, sizeof(sentence), own, target);
        if (len > 0 && !ble_output.Send(reinterpret_cast<uint8_t*>(sentence), len)) break;  // Queue full.
    }
    delete[] traffic;
}

bool ADSBeeServer::ReportGDL90UplinkDataMessage(const DecodedUATUplinkPacket& uplink_packet) {
    if (!comms_manager.WiFiAccessPointIsRunning() && !ble_output.StreamsMode(SettingsManager::kBLEOutputGDL90)) {
        return true;  // Nothing to do.
    }

    CommsManager::NetworkMessage message;
    message.port = kGDL90Port;

    message.len = gdl90.WriteGDL90UplinkDataMessage(
        message.data, CommsManager::NetworkMessage::kMaxLenBytes, uplink_packet.decoded_payload,
        DecodedUATUplinkPacket::kDecodedPayloadNumBytes,
        GDL90Reporter::MLAT48MHz64BitCountsToUATTORTicks(uplink_packet.raw.mlat_48mhz_64bit_counts));
    if (message.len == 0) {
        CONSOLE_ERROR("ADSBeeServer::ReportGDL90UplinkPacket", "Failed to write GDL90 UAT uplink packet message.");
        return false;
    }

    if (!SendGDL90(message, /*droppable=*/true)) {
        CONSOLE_ERROR("ADSBeeServer::ReportGDL90UplinkPacket", "Failed to send UAT uplink packet to all clients.");
        return false;
    }

    return true;
}

void ADSBeeServer::QueueUplinkMessage(const DecodedUATUplinkPacket& uplink_packet) {
    if (!uplink_ring_) {
        return;
    }
    uint8_t message[kUATUplinkMessageNumBytes];
    // The packet's own timestamp counts radio timer ticks, so report the time of this message.
    BuildUATUplinkMessage(message, uplink_packet.decoded_payload, uplink_packet.raw.sigs_dbm,
                          get_time_since_boot_ms());
    uplink_ring_->Push(message);
}

void ADSBeeServer::UpdateUplinkBroadcast() {
    int client_fds[WebSocketServer::kMaxNumClients];
    uint16_t num_clients = network_uplink.GetClientFds(client_fds);
    if (num_clients == 0) {
        delete uplink_ring_;
        uplink_ring_ = nullptr;
        return;
    }
    if (!uplink_ring_) {
        uplink_ring_ = new (std::nothrow) UplinkRing();
        if (!uplink_ring_) {
            return;  // Try again on the next update.
        }
    }
    uplink_ring_->SetClients(client_fds, num_clients);

    for (uint16_t i = 0; i < num_clients; i++) {
        // Only send to a client that takes the frame right away, so a slow client can't stall this task.
        const uint8_t* message;
        while ((message = uplink_ring_->Peek(client_fds[i])) != nullptr &&
               network_uplink.ClientIsWritable(client_fds[i])) {
            // A failed send drops this message for the client.
            network_uplink.SendMessageToClient(client_fds[i], (const char*)message, kUATUplinkMessageNumBytes);
            uplink_ring_->Pop(client_fds[i]);
        }
    }

    uint32_t timestamp_ms = get_time_since_boot_ms();
    if (uplink_ring_->num_dropped > 0 &&
        timestamp_ms - last_uplink_drop_warning_timestamp_ms_ > kUplinkDropWarningIntervalMs) {
        last_uplink_drop_warning_timestamp_ms_ = timestamp_ms;
        CONSOLE_WARNING("ADSBeeServer::UpdateUplinkBroadcast", "Dropped %lu UAT uplinks for slow /uplink clients.",
                        (unsigned long)uplink_ring_->num_dropped);
        uplink_ring_->num_dropped = 0;
    }
}

// void ADSBeeServer::TCPServerTask(void *pvParameters) {
//     int addr_family = AF_INET;
//     int ip_protocol = 0;
//     struct sockaddr_in dest_addr;

//     dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
//     dest_addr.sin_family = AF_INET;
//     dest_addr.sin_port = htons(kNetworkControlPort);
//     ip_protocol = IPPROTO_IP;

//     int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
//     if (listen_sock < 0) {
//         CONSOLE_ERROR("ADSBeeServer::TCPServerTask", "Unable to create socket: errno %d", errno);
//         vTaskDelete(NULL);
//         return;
//     }

//     int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
//     if (err != 0) {
//         CONSOLE_ERROR("ADSBeeServer::TCPServerTask", "Socket unable to bind: errno %d", errno);
//         close(listen_sock);
//         vTaskDelete(NULL);
//         return;
//     }

//     err = listen(listen_sock, 1);
//     if (err != 0) {
//         CONSOLE_ERROR("ADSBeeServer::TCPServerTask", "Error occurred during listen: errno %d", errno);
//         close(listen_sock);
//         vTaskDelete(NULL);
//         return;
//     }

//     while (1) {
//         CONSOLE_INFO("ADSBeeServer::TCPServerTask", "Socket listening");

//         struct sockaddr_storage source_addr;
//         socklen_t addr_len = sizeof(source_addr);
//         int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
//         if (sock < 0) {
//             CONSOLE_ERROR("ADSBeeServer::TCPServerTask", "Unable to accept connection: errno %d", errno);
//             break;
//         }

//         // Display IPv4 address.
//         char addr_str[128];
//         inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
//         CONSOLE_INFO("ADSBeeServer::TCPServerTask", "Socket accepted ip address: %s", addr_str);

//         // Handle received data
//         while (1) {
//             // Data is available to read
//             uint8_t rx_buffer[128];
//             int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);

//             if (len < 0) {
//                 if (errno == EAGAIN || errno == EWOULDBLOCK) {
//                     // No data available right now, try again later
//                     continue;
//                 }
//                 // Some other error occurred
//                 CONSOLE_ERROR("ADSBeeServer::TCPServerTask", "Error occurred during receiving: errno %d", errno);
//                 break;
//             } else if (len == 0) {
//                 CONSOLE_INFO("ADSBeeServer::TCPServerTask", "Connection closed by peer");
//                 break;
//             }

//             // Process received data
//             rx_buffer[len] = 0;  // Null-terminate
//             CONSOLE_INFO("ADSBeeServer::TCPServerTask", "Received %d bytes: %s", len, rx_buffer);
//         }

//         shutdown(sock, 0);
//         close(sock);
//     }
// }

/**
 * Sends an HTTP response and asks the client to close the connection, which frees its session for the next client.
 * @param[in] content_type Content type, or nullptr for the default (text/html).
 */
static esp_err_t send_response(httpd_req_t* req, const char* content_type, const char* buf, ssize_t buf_len) {
    if (content_type != nullptr) {
        httpd_resp_set_type(req, content_type);
    }
    httpd_resp_set_hdr(req, "Connection", "close");
    esp_err_t ret = httpd_resp_send(req, buf, buf_len);
    // The idle timeout starts when the response is sent, however long sending took.
    adsbee_server.web_server_sessions.Touch(httpd_req_to_sockfd(req), get_time_since_boot_ms());
    return ret;
}

static esp_err_t send_file(httpd_req_t* req, const char* content_type, const uint8_t* file_start,
                           const uint8_t* file_end) {
    send_response(req, content_type, (const char*)file_start, file_end - file_start);
    return ESP_OK;
}

static esp_err_t root_handler(httpd_req_t* req) { return send_file(req, nullptr, index_html_start, index_html_end); }

static esp_err_t css_handler(httpd_req_t* req) { return send_file(req, "text/css", style_css_start, style_css_end); }

static esp_err_t adsbee_js_handler(httpd_req_t* req) {
    return send_file(req, "application/javascript", adsbee_js_start, adsbee_js_end);
}

static esp_err_t settings_js_handler(httpd_req_t* req) {
    return send_file(req, "application/javascript", settings_js_start, settings_js_end);
}

static esp_err_t ota_updater_js_handler(httpd_req_t* req) {
    return send_file(req, "application/javascript", ota_updater_js_start, ota_updater_js_end);
}

static esp_err_t firmware_release_js_handler(httpd_req_t* req) {
    return send_file(req, "application/javascript", firmware_release_js_start, firmware_release_js_end);
}

static esp_err_t fisb_js_handler(httpd_req_t* req) {
    return send_file(req, "application/javascript", fisb_js_start, fisb_js_end);
}

static esp_err_t fisb_stations_js_handler(httpd_req_t* req) {
    return send_file(req, "application/javascript", fisb_stations_js_start, fisb_stations_js_end);
}

static void json_escape(char* out, size_t out_size, const char* in) {
    size_t o = 0;
    for (size_t i = 0; in[i] && o + 2 < out_size; i++) {
        char c = in[i];
        if (c == '"' || c == '\\') { out[o++] = '\\'; out[o++] = c; }
        else if (c == '\n')        { out[o++] = '\\'; out[o++] = 'n'; }
        else if (c == '\r')        { out[o++] = '\\'; out[o++] = 'r'; }
        else if (c == '\t')        { out[o++] = '\\'; out[o++] = 't'; }
        else                       { out[o++] = c; }
    }
    out[o] = '\0';
}

static esp_err_t feed_api_get_handler(httpd_req_t* req) {
    char query[32];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing query");
        return ESP_OK;
    }
    char index_str[8];
    if (httpd_query_key_value(query, "index", index_str, sizeof(index_str)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing index");
        return ESP_OK;
    }
    int index = atoi(index_str);
    if (index < 0 || index >= (int)SettingsManager::Settings::kMaxNumFeeds) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Index out of range");
        return ESP_OK;
    }
    char escaped_uri[SettingsManager::Settings::kFeedURIMaxNumChars * 2 + 1];
    json_escape(escaped_uri, sizeof(escaped_uri), settings_manager.settings.feed_uris[index]);
    char receiver_id[SettingsManager::Settings::kFeedReceiverIDNumBytes * 2 + 1];
    SettingsManager::ReceiverIDToStr(settings_manager.settings.feed_receiver_ids[index], receiver_id);
    // The shared MQTT broker, without its password.
    const SettingsManager::Settings::MQTTBroker& mqtt = settings_manager.settings.mqtt_broker;
    char escaped_host[sizeof(mqtt.host) * 2];
    json_escape(escaped_host, sizeof(escaped_host), mqtt.host);
    char escaped_username[sizeof(mqtt.username) * 2];
    json_escape(escaped_username, sizeof(escaped_username), mqtt.username);
    char escaped_prefix[sizeof(mqtt.topic_prefix) * 2];
    json_escape(escaped_prefix, sizeof(escaped_prefix), mqtt.topic_prefix);
    char escaped_path[sizeof(mqtt.websocket_path) * 2];
    json_escape(escaped_path, sizeof(escaped_path), mqtt.websocket_path);
    char json[1152];
    snprintf(json, sizeof(json),
             "{\"index\":%d,\"uri\":\"%s\",\"port\":%d,\"active\":%d,\"protocol\":\"%s\",\"receiver_id\":\"%s\","
             "\"transport\":\"%s\",\"mqtt\":{\"host\":\"%s\",\"port\":%u,\"username\":\"%s\",\"has_password\":%s,"
             "\"tls\":%s,\"qos\":%u,\"topic_prefix\":\"%s\",\"transport\":\"%s\",\"path\":\"%s\"}}",
             index, escaped_uri, settings_manager.settings.feed_ports[index],
             (int)settings_manager.settings.feed_is_active[index],
             SettingsManager::kReportingProtocolStrs[settings_manager.settings.feed_protocols[index]], receiver_id,
             SettingsManager::kFeedTransportStrs[settings_manager.settings.feed_transports[index]], escaped_host,
             mqtt.port, escaped_username, mqtt.password[0] ? "true" : "false", mqtt.tls ? "true" : "false", mqtt.qos,
             escaped_prefix, SettingsManager::kMQTTTransportStrs[mqtt.transport], escaped_path);
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    send_response(req, "application/json", json, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t favicon_handler(httpd_req_t* req) {
    httpd_resp_set_hdr(req, "Cache-Control", "max-age=2592000, public");  // Cache for 30 days

    esp_err_t res =
        send_response(req, "image/png", (const char*)favicon_png_start, favicon_png_end - favicon_png_start);
    if (res != ESP_OK) {
        ESP_LOGE("FAVICON", "Failed to send favicon");
        return res;
    }
    return ESP_OK;
}

void NetworkConsolePostConnectCallback(WebSocketServer* ws_server, int client_fd) {
    char welcome_message[kNetworkConsoleWelcomeMessageMaxLen];
    if (object_dictionary.kFirmwareVersionReleaseCandidate == 0) {
        snprintf(welcome_message, kNetworkConsoleWelcomeMessageMaxLen,
                 "\r\n █████  ██████  ███████ ██████  ███████ ███████      ██  ██████   █████   ██████  "
                 "\r\n██   ██ ██   ██ ██      ██   ██ ██      ██          ███ ██  ████ ██   ██ ██  ████ "
                 "\r\n███████ ██   ██ ███████ ██████  █████   █████        ██ ██ ██ ██  ██████ ██ ██ ██ "
                 "\r\n██   ██ ██   ██      ██ ██   ██ ██      ██           ██ ████  ██      ██ ████  ██ "
                 "\r\n██   ██ ██████  ███████ ██████  ███████ ███████      ██  ██████   █████   ██████  "
                 "\r\n\r\nFirmware Version: %d.%d.%d\r\nAP SSID: %s\r\n",
                 object_dictionary.kFirmwareVersionMajor, object_dictionary.kFirmwareVersionMinor,
                 object_dictionary.kFirmwareVersionPatch, settings_manager.settings.core_network_settings.wifi_ap_ssid);
    } else {
        snprintf(welcome_message, kNetworkConsoleWelcomeMessageMaxLen,
                 "\r\n █████  ██████  ███████ ██████  ███████ ███████      ██  ██████   █████   ██████  "
                 "\r\n██   ██ ██   ██ ██      ██   ██ ██      ██          ███ ██  ████ ██   ██ ██  ████ "
                 "\r\n███████ ██   ██ ███████ ██████  █████   █████        ██ ██ ██ ██  ██████ ██ ██ ██ "
                 "\r\n██   ██ ██   ██      ██ ██   ██ ██      ██           ██ ████  ██      ██ ████  ██ "
                 "\r\n██   ██ ██████  ███████ ██████  ███████ ███████      ██  ██████   █████   ██████  "
                 "\r\n\r\nFirmware Version: %d.%d.%d-rc%d\r\nAP SSID: %s\r\n",
                 object_dictionary.kFirmwareVersionMajor, object_dictionary.kFirmwareVersionMinor,
                 object_dictionary.kFirmwareVersionPatch, object_dictionary.kFirmwareVersionReleaseCandidate,
                 settings_manager.settings.core_network_settings.wifi_ap_ssid);
    }
    ws_server->SendMessage(client_fd, welcome_message);
}

void NetworkConsoleMessageReceivedCallback(WebSocketServer* ws_server, int client_fd, httpd_ws_frame_t& ws_pkt) {
    // Forward the network console message to the RP2040.
    char* message = (char*)ws_pkt.payload;
    uint16_t message_len = (uint16_t)ws_pkt.len;
    auto_update_manager.OnConsoleInput(message, message_len);
    xSemaphoreTake(object_dictionary.network_console_rx_queue_mutex, portMAX_DELAY);
    if (object_dictionary.network_console_rx_queue.MaxNumElements() -
            object_dictionary.network_console_rx_queue.Length() <
        message_len) {
        CONSOLE_ERROR("NetworkConsoleMessageReceivedCallback",
                      "Network console tx queue is full, dropping message of length %d.", message_len);
        xSemaphoreGive(object_dictionary.network_console_rx_queue_mutex);
        return;
    }
    for (uint16_t i = 0; i < ws_pkt.len; i++) {
        if (!object_dictionary.network_console_rx_queue.Enqueue(message[i])) {
            CONSOLE_ERROR("NetworkConsoleMessageReceivedCallback",
                          "Failed to push character %d of network console message to tx queue.", i + 1);
            xSemaphoreGive(object_dictionary.network_console_rx_queue_mutex);
            return;
        }
    }
    xSemaphoreGive(object_dictionary.network_console_rx_queue_mutex);
}

void ADSBeeServer::SendNetworkMetricsMessage() {
    // ESP32 can't see number of attempted demodulations or raw packets, so steal that from RP2040 metrics
    // dictionary.
    AircraftDictionary::Metrics combined_metrics = aircraft_dictionary.metrics;
    // The RP2040 sends its metrics every second. If they stop arriving (e.g. the SPI link is down), report zeros: the
    // last snapshot is no longer current.
    if (get_time_since_boot_ms() - adsbee_server.last_rp2040_metrics_timestamp_ms_ > kRP2040MetricsStaleTimeoutMs) {
        adsbee_server.rp2040_aircraft_dictionary_metrics = AircraftDictionary::Metrics();
    }
    // Steal demods_1090.
    combined_metrics.demods_1090 = adsbee_server.rp2040_aircraft_dictionary_metrics.demods_1090;
    for (uint16_t i = 0; i < AircraftDictionary::kMaxNumSources; i++) {
        combined_metrics.demods_1090_by_source[i] +=
            adsbee_server.rp2040_aircraft_dictionary_metrics.demods_1090_by_source[i];
    }
    // Steal raw_squitter_frames.
    combined_metrics.raw_squitter_frames = adsbee_server.rp2040_aircraft_dictionary_metrics.raw_squitter_frames;
    for (uint16_t i = 0; i < AircraftDictionary::kMaxNumSources; i++) {
        combined_metrics.raw_squitter_frames_by_source[i] +=
            adsbee_server.rp2040_aircraft_dictionary_metrics.raw_squitter_frames_by_source[i];
    }
    // Steal raw_extended_squitter_frames.
    combined_metrics.raw_extended_squitter_frames =
        adsbee_server.rp2040_aircraft_dictionary_metrics.raw_extended_squitter_frames;
    for (uint16_t i = 0; i < AircraftDictionary::kMaxNumSources; i++) {
        combined_metrics.raw_extended_squitter_frames_by_source[i] +=
            adsbee_server.rp2040_aircraft_dictionary_metrics.raw_extended_squitter_frames_by_source[i];
    }
    // Steal UAT metrics.
    combined_metrics.raw_uat_adsb_frames = adsbee_server.rp2040_aircraft_dictionary_metrics.raw_uat_adsb_frames;
    combined_metrics.valid_uat_adsb_frames = adsbee_server.rp2040_aircraft_dictionary_metrics.valid_uat_adsb_frames;
    combined_metrics.raw_uat_uplink_frames = adsbee_server.rp2040_aircraft_dictionary_metrics.raw_uat_uplink_frames;
    combined_metrics.valid_uat_uplink_frames = adsbee_server.rp2040_aircraft_dictionary_metrics.valid_uat_uplink_frames;
    // Bit error corrections happen on the RP2040.
    for (uint16_t i = 0; i < AircraftDictionary::kNumCorrectionKinds; i++) {
        combined_metrics.corrected_frames[i] = adsbee_server.rp2040_aircraft_dictionary_metrics.corrected_frames[i];
    }

    // Broadcast dictionary metrics over the metrics Websocket.
    char metrics_message[kNetworkMetricsMessageMaxLen];
    snprintf(metrics_message, kNetworkMetricsMessageMaxLen, "{ \"aircraft_dictionary_metrics\": ");
    combined_metrics.ToJSON(metrics_message + strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
                            kNetworkMetricsMessageMaxLen - strnlen(metrics_message, kNetworkMetricsMessageMaxLen));
    snprintf(metrics_message + strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
             kNetworkMetricsMessageMaxLen - strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
             ", \"server_metrics\": { ");
    // ADSBee Server Metrics
    ArrayToJSON(metrics_message + strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
                kNetworkMetricsMessageMaxLen - strnlen(metrics_message, kNetworkMetricsMessageMaxLen), "feed_uri",
                settings_manager.settings.feed_uris, "\"%s\"", true);
    ArrayToJSON(metrics_message + strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
                kNetworkMetricsMessageMaxLen - strnlen(metrics_message, kNetworkMetricsMessageMaxLen), "feed_mps",
                comms_manager.feed_mps, "%u", true);
    // Transport of each feed and the RAM the active feeds take out of this unit's budget.
    snprintf(metrics_message + strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
             kNetworkMetricsMessageMaxLen - strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
             "\"feed_transport\": [");
    for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
        snprintf(metrics_message + strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
                 kNetworkMetricsMessageMaxLen - strnlen(metrics_message, kNetworkMetricsMessageMaxLen), "%s\"%s\"",
                 i ? "," : "", SettingsManager::kFeedTransportStrs[settings_manager.settings.feed_transports[i]]);
    }
    snprintf(metrics_message + strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
             kNetworkMetricsMessageMaxLen - strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
             "], \"feed_ram_bytes\": %lu, \"feed_ram_budget_bytes\": %lu",
             (unsigned long)FeedRAMBudget::UsedBytes(settings_manager.settings),
             (unsigned long)FeedRAMBudget::BudgetBytes(settings_manager.settings, HardwareCapabilities::HasPSRAM()));
    snprintf(metrics_message + strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
             kNetworkMetricsMessageMaxLen - strnlen(metrics_message, kNetworkMetricsMessageMaxLen), "}}\n");
    BroadcastNetworkMetrics(metrics_message);

    // Device status goes in a second message, so neither can outgrow the buffer (ten 63-character feed URIs take
    // about 700 bytes). Consumers read whichever top-level keys a message has.
    snprintf(metrics_message, kNetworkMetricsMessageMaxLen, "{ \"device_status\": { ");
    // Device Status
    const ObjectDictionary::RP2040DeviceStatus& rp2040_status = object_dictionary.composite_device_status.rp2040;
    char gnss_utc_time[9] = "--:--:--";
    if (rp2040_status.gnss_utc_time_valid) {
        snprintf(gnss_utc_time, sizeof(gnss_utc_time), "%02u:%02u:%02u",
                 static_cast<unsigned>(rp2040_status.gnss_utc_hour % 24),
                 static_cast<unsigned>(rp2040_status.gnss_utc_minute % 60),
                 static_cast<unsigned>(rp2040_status.gnss_utc_second % 61));
    }
    snprintf(metrics_message + strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
             kNetworkMetricsMessageMaxLen - strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
             "\"rp2040\": { \"uptime_ms\": %lu, \"core_0_usage_percent\": %u, "
             "\"core_1_usage_percent\": %u, \"temperature_deg_c\": %d, \"noise_floor_dbm\": %d, "
             "\"noise_floor_mv\": %u }",
             rp2040_status.timestamp_ms, rp2040_status.core_0_usage_percent, rp2040_status.core_1_usage_percent,
             rp2040_status.temperature_deg_c, rp2040_status.noise_floor_dbm, rp2040_status.noise_floor_mv);
    snprintf(metrics_message + strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
             kNetworkMetricsMessageMaxLen - strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
             ", \"gnss\": { \"enabled\": %s, \"fix_valid\": %s, \"latitude_deg\": %.6f, "
             "\"longitude_deg\": %.6f, \"altitude_ft\": %ld, \"heading_deg\": %.1f, \"speed_kts\": %ld, "
             "\"num_satellites\": %u, \"utc_time\": \"%s\" }",
             rp2040_status.gnss_enabled ? "true" : "false", rp2040_status.gnss_fix_valid ? "true" : "false",
             rp2040_status.gnss_latitude_deg, rp2040_status.gnss_longitude_deg,
             static_cast<long>(rp2040_status.gnss_altitude_ft), rp2040_status.gnss_heading_deg,
             static_cast<long>(rp2040_status.gnss_speed_kts), static_cast<unsigned int>(rp2040_status.gnss_num_satellites),
             gnss_utc_time);
    snprintf(metrics_message + strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
             kNetworkMetricsMessageMaxLen - strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
             ", \"subg\": { \"uptime_ms\": %lu, \"user_core_usage_percent\": %u, "
             "\"temperature_deg_c\": %u }",
             object_dictionary.composite_device_status.subg.timestamp_ms,
             object_dictionary.composite_device_status.subg.cpu_usage_percent,
             object_dictionary.composite_device_status.subg.temperature_deg_c);
    snprintf(metrics_message + strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
             kNetworkMetricsMessageMaxLen - strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
             ", \"esp32\": { \"uptime_ms\": %lu, \"core_0_usage_percent\": %u, "
             "\"core_1_usage_percent\": %u, \"temperature_deg_c\": %d, \"heap_free_bytes\": %lu, "
             "\"heap_largest_free_block_bytes\": %lu }",
             object_dictionary.device_status.timestamp_ms, object_dictionary.device_status.core_0_usage_percent,
             object_dictionary.device_status.core_1_usage_percent, object_dictionary.device_status.temperature_deg_c,
             object_dictionary.device_status.heap_free_bytes,
             object_dictionary.device_status.heap_largest_free_block_bytes);
    AutoUpdateManager::Status auto_update;
    auto_update_manager.GetStatus(auto_update);
    snprintf(metrics_message + strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
             kNetworkMetricsMessageMaxLen - strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
             ", \"auto_update\": { \"enabled\": %s, \"state\": \"%s\", \"version\": \"%s\", \"progress\": %u }",
             auto_update.enabled ? "true" : "false", auto_update.state, auto_update.version,
             auto_update.progress_percent);
    SwarmManager::Status swarm;
    swarm_manager.GetStatus(swarm);
    char last_report_s_ago[12] = "null";
    if (swarm.last_report_s_ago != UINT32_MAX) {
        snprintf(last_report_s_ago, sizeof(last_report_s_ago), "%lu", (unsigned long)swarm.last_report_s_ago);
    }
    snprintf(metrics_message + strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
             kNetworkMetricsMessageMaxLen - strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
             ", \"swarm\": { \"enabled\": %s, \"result\": \"%s\", \"http_status\": %d, \"last_report_s_ago\": %s, "
             "\"next_report_s\": %lu, \"seq\": %lu, \"body_bytes\": %u }",
             swarm.enabled ? "true" : "false", swarm.result, swarm.http_status, last_report_s_ago,
             (unsigned long)swarm.next_report_s, (unsigned long)swarm.seq, swarm.body_len);
    NetworkTime::Status sntp;
    network_time.GetStatus(sntp);
    snprintf(metrics_message + strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
             kNetworkMetricsMessageMaxLen - strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
             ", \"sntp\": { \"enabled\": %s, \"server\": \"%s\", \"synced\": %s, \"last_sync_age_s\": %lu }",
             sntp.enabled ? "true" : "false", sntp.server, sntp.synced ? "true" : "false",
             static_cast<unsigned long>(sntp.last_sync_age_s));
    // Receiver ID of each feed, for the feed cards.
    snprintf(metrics_message + strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
             kNetworkMetricsMessageMaxLen - strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
             " }, \"feed_receiver_id\": [");
    for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
        char receiver_id[SettingsManager::Settings::kFeedReceiverIDNumBytes * 2 + 1];
        SettingsManager::ReceiverIDToStr(settings_manager.settings.feed_receiver_ids[i], receiver_id);
        snprintf(metrics_message + strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
                 kNetworkMetricsMessageMaxLen - strnlen(metrics_message, kNetworkMetricsMessageMaxLen), "%s\"%s\"",
                 i ? "," : "", receiver_id);
    }
    snprintf(metrics_message + strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
             kNetworkMetricsMessageMaxLen - strnlen(metrics_message, kNetworkMetricsMessageMaxLen), "]}\n");

    BroadcastNetworkMetrics(metrics_message);
}

void ADSBeeServer::BroadcastNetworkMetrics(const char* message) {
    size_t len = strnlen(message, kNetworkMetricsMessageMaxLen);
    if (len >= kNetworkMetricsMessageMaxLen - 1) {
        // snprintf cut the message short: it would not be valid JSON.
        CONSOLE_WARNING("ADSBeeServer::BroadcastNetworkMetrics", "Metrics message longer than %d bytes, not sent.",
                        kNetworkMetricsMessageMaxLen - 1);
        return;
    }
    network_metrics.BroadcastMessage(message, len);
}

bool ADSBeeServer::TCPServerInit() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = kHTTPServerStackSizeBytes;
    // config.task_caps = MALLOC_CAP_IRAM_8BIT;
    config.max_open_sockets = kMaxNumWebServerSessions;
    config.backlog_conn = kWebServerBacklog;
    // 12 URIs are registered below (8 HTTP + 4 websocket); the default limit is 8 and a
    // failed registration aborts boot via ESP_ERROR_CHECK.
    config.max_uri_handlers = 16;
    config.open_fn = web_server_open_fd;
    config.close_fn = ws_close_fd;
    // The web server's own purge closes its least recently used session, which can be a WebSocket in the middle of a
    // firmware update. web_server_open_fd() makes room instead.
    config.lru_purge_enable = false;
    config.keep_alive_enable = true;
    config.keep_alive_idle = 5;      // Seconds idle before sending first keepalive probe.
    config.keep_alive_interval = 5;  // Seconds between keepalive probes.
    config.keep_alive_count = 3;     // Close connection after 3 failed probes (~20s dead detection).

    esp_err_t ret = httpd_start(&server, &config);
    HeapDiagnostics::Mark("httpd");
    if (ret != ESP_OK) {
        CONSOLE_ERROR("ADSBeeServer::TCPServerInit", "Failed to start HTTP server: %s, remaining stack %u Bytes.",
                      esp_err_to_name(ret), uxTaskGetStackHighWaterMark(NULL));
        return false;
    }

    // Root URI handler (HTML)
    httpd_uri_t root = {.uri = "/",
                        .method = HTTP_GET,
                        .handler = root_handler,
                        .user_ctx = NULL,
                        .is_websocket = false,
                        .handle_ws_control_frames = false,
                        .supported_subprotocol = nullptr};
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &root));

    // CSS URI handler
    httpd_uri_t css = {.uri = "/style.css",
                       .method = HTTP_GET,
                       .handler = css_handler,
                       .user_ctx = NULL,
                       .is_websocket = false,
                       .handle_ws_control_frames = false,
                       .supported_subprotocol = nullptr};
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &css));

    // JavaScript library URI handler
    httpd_uri_t adsbee_js = {.uri = "/adsbee.js",
                             .method = HTTP_GET,
                             .handler = adsbee_js_handler,
                             .user_ctx = NULL,
                             .is_websocket = false,
                             .handle_ws_control_frames = false,
                             .supported_subprotocol = nullptr};
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &adsbee_js));

    // Settings GUI JavaScript URI handler
    httpd_uri_t settings_js = {.uri = "/settings.js",
                               .method = HTTP_GET,
                               .handler = settings_js_handler,
                               .user_ctx = NULL,
                               .is_websocket = false,
                               .handle_ws_control_frames = false,
                               .supported_subprotocol = nullptr};
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &settings_js));

    // Firmware update (OTA) JavaScript URI handler
    httpd_uri_t ota_updater_js = {.uri = "/ota_updater.js",
                                  .method = HTTP_GET,
                                  .handler = ota_updater_js_handler,
                                  .user_ctx = NULL,
                                  .is_websocket = false,
                                  .handle_ws_control_frames = false,
                                  .supported_subprotocol = nullptr};
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &ota_updater_js));

    // Firmware release check JavaScript URI handler
    httpd_uri_t firmware_release_js = {.uri = "/firmware_release.js",
                                       .method = HTTP_GET,
                                       .handler = firmware_release_js_handler,
                                       .user_ctx = NULL,
                                       .is_websocket = false,
                                       .handle_ws_control_frames = false,
                                       .supported_subprotocol = nullptr};
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &firmware_release_js));

    // FIS-B weather decoder and station table JavaScript URI handlers
    httpd_uri_t fisb_js = {.uri = "/fisb.js",
                           .method = HTTP_GET,
                           .handler = fisb_js_handler,
                           .user_ctx = NULL,
                           .is_websocket = false,
                           .handle_ws_control_frames = false,
                           .supported_subprotocol = nullptr};
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &fisb_js));
    httpd_uri_t fisb_stations_js = {.uri = "/fisb_stations.js",
                                    .method = HTTP_GET,
                                    .handler = fisb_stations_js_handler,
                                    .user_ctx = NULL,
                                    .is_websocket = false,
                                    .handle_ws_control_frames = false,
                                    .supported_subprotocol = nullptr};
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &fisb_stations_js));

    // Favicon URI handler
    httpd_uri_t favicon = {.uri = "/favicon.png",
                           .method = HTTP_GET,
                           .handler = favicon_handler,
                           .user_ctx = NULL,
                           .is_websocket = false,
                           .handle_ws_control_frames = false,
                           .supported_subprotocol = nullptr};
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &favicon));

    // Feed settings API handler
    httpd_uri_t feed_api = {.uri = "/api/feed",
                            .method = HTTP_GET,
                            .handler = feed_api_get_handler,
                            .user_ctx = NULL,
                            .is_websocket = false,
                            .handle_ws_control_frames = false,
                            .supported_subprotocol = nullptr};
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &feed_api));

    network_console = WebSocketServer({.label = "Network Console",
                                       .server = server,
                                       .uri = "/console",
                                       .num_clients_allowed = kMaxNumClientsPerWebSocket,
                                       .send_as_binary = true,      // Network console messages can contain binary data.
                                       .inactivity_timeout_ms = 0,  // Disabled; TCP keepalives handle dead connections.
                                       .post_connect_callback = NetworkConsolePostConnectCallback,
                                       .message_received_callback = NetworkConsoleMessageReceivedCallback});
    if (!network_console.Init()) {
        CONSOLE_ERROR("ADSBeeServer::TCPServerInit", "Failed to start Network Console WebSocket server.");
        return false;
    }
    network_metrics = WebSocketServer({.label = "Network Metrics",
                                       .server = server,
                                       .uri = "/metrics",
                                       .num_clients_allowed = kMaxNumClientsPerWebSocket,
                                       .send_as_binary = false,                 // Network metrics are always ASCII.
                                       .inactivity_timeout_ms = 90 * 1000,  // 90s — 1.5× JS heartbeat interval.
                                       .post_connect_callback = nullptr,
                                       .message_received_callback = nullptr});
    if (!network_metrics.Init()) {
        CONSOLE_ERROR("ADSBeeServer::TCPServerInit", "Failed to start Network Metrics WebSocket server.");
        return false;
    }
    network_aircraft = WebSocketServer({.label = "Network Aircraft",
                                        .server = server,
                                        .uri = "/aircraft",
                                        .num_clients_allowed = kMaxNumClientsPerWebSocket,
                                        .send_as_binary = false,
                                        .inactivity_timeout_ms = 90 * 1000,
                                        .post_connect_callback = nullptr,
                                        .message_received_callback = nullptr});
    if (!network_aircraft.Init()) {
        CONSOLE_ERROR("ADSBeeServer::TCPServerInit", "Failed to start Network Aircraft WebSocket server.");
        return false;
    }
    network_uplink = WebSocketServer({.label = "Network Uplink",
                                      .server = server,
                                      .uri = "/uplink",
                                      .num_clients_allowed = kMaxNumClientsPerWebSocket,
                                      .send_as_binary = true,
                                      .inactivity_timeout_ms = 0,  // Disabled; TCP keepalives handle dead connections.
                                      .post_connect_callback = nullptr,
                                      .message_received_callback = nullptr});
    if (!network_uplink.Init()) {
        CONSOLE_ERROR("ADSBeeServer::TCPServerInit", "Failed to start Network Uplink WebSocket server.");
        return false;
    }
    HeapDiagnostics::Mark("ws");

    if (server == nullptr) {
        CONSOLE_ERROR("ADSBeeServer::TCPServerInit", "HTTP server instance is null after start.");
        return false;
    }
    CONSOLE_INFO("ADSBeeServer::TCPServerInit", "HTTP server started successfully, remaining stack %u Bytes.",
                 uxTaskGetStackHighWaterMark(NULL));

    return true;
}

void ADSBeeServer::SendAircraftJSONMessages() {
    // Batch aircraft into JSON arrays of up to one websocket message each. One frame per aircraft (two socket writes
    // each) cost ~100 TCP segments per second per client at a busy site; on the W5500 Ethernet board the network tasks
    // doing that outrank the SPI receive task, which then missed RP2040 handshakes until the RP2040 reset the board.
    // Frames of up to kAircraftJSONBatchMaxLen always fit a writable socket whole.
    if (aircraft_json_next_uid_ >= aircraft_json_num_uids_) {
        return;
    }
    if (network_aircraft.GetNumClients() == 0) {
        aircraft_json_next_uid_ = aircraft_json_num_uids_;  // Nobody left to send the rest to.
        return;
    }
    char batch_buf[kAircraftJSONBatchMaxLen];
    uint64_t start_us = get_time_since_boot_us();
    while (get_time_since_boot_us() - start_us < kAircraftJSONPassBudgetUs) {
        uint16_t len =
            WriteAircraftJSONLiveMapArray(aircraft_dictionary, get_time_since_boot_ms(), aircraft_json_uids_,
                                          aircraft_json_num_uids_, aircraft_json_next_uid_, batch_buf, sizeof(batch_buf));
        if (len == 0) {
            break;
        }
        network_aircraft.BroadcastMessage(batch_buf, len);
    }
}
