#include "adsbee_server.hh"

#include "esp_heap_caps.h"
#include "lwip/sockets.h"

#include "comms.hh"
#include "aircraftjson_utils.hh"
#include "gdl90/gdl90_utils.hh"
#include "json_utils.hh"
#include "pico.hh"
#include "remote_id/remote_id_manager.hh"
#include "settings.hh"
#include "spi_coprocessor.hh"
#include "task_priorities.hh"
#include "unit_conversions.hh"

// #define VERBOSE_DEBUG

static const uint16_t kGDL90Port = 4000;

static const uint16_t kNetworkConsoleWelcomeMessageMaxLen = 1000;
static const uint16_t kNetworkMetricsMessageMaxLen = 2048;

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
void ws_close_fd(httpd_handle_t hd, int sockfd) {
    adsbee_server.network_console.RemoveClient(sockfd);
    adsbee_server.network_metrics.RemoveClient(sockfd);
    adsbee_server.network_aircraft.RemoveClient(sockfd);
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

    // Initialize SPI receive task before requesting settings so that we can tend to messages from the RP2040 and stop
    // it from freaking out.
    spi_receive_task_should_exit_ = false;
    // SPI receive task is high priority but we can't starve the WiFi task, which is pinned to core 0 in the sdkconfig.
    xTaskCreatePinnedToCoreWithCaps(esp_spi_receive_task, "spi_receive_task", kSPIReceiveTaskStackSizeBytes, NULL,
                                    kSPIReceiveTaskPriority, NULL, kSPIReceiveTaskCore, MALLOC_CAP_INTERNAL);

    // Initialize prerequisites for Ethernet and WiFi. Needs to be done before settings are applied.
    comms_manager.Init();

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

    // Wait for the callback to complete
    xSemaphoreTake(settings_read_semaphore, portMAX_DELAY);
    vSemaphoreDelete(settings_read_semaphore);
    settings_manager.Print();
    settings_manager.Apply();

    return TCPServerInit();
}

bool ADSBeeServer::Update() {
    bool ret = true;
    // Do NOT call pico.Update() from here since that's already taken care of by the SPIReceiveTask.
    // Update the LED here so it has better time resolution than it would in the SPI task, which blocks frequently.
    pico.UpdateLED();

    uint32_t timestamp_ms = get_time_since_boot_ms();

    // Ingest any Broadcast Remote ID advertisements received on the BLE/WiFi radios into the local aircraft dictionary
    // (and forward rate-limited copies up to the RP2040), then service the WiFi channel hopper. Done on this task so all
    // aircraft-dictionary mutation stays single-threaded.
    remote_id_manager.ServiceIngestQueue();
    remote_id_manager.Update();

    // Prune aircraft dictionary. Need to do this up front so that we don't end up with a negative timestamp delta
    // caused by packets being ingested more recently than the timestamp we take at the beginning of this function.
    if (timestamp_ms - last_aircraft_dictionary_update_timestamp_ms_ > kAircraftDictionaryUpdateIntervalMs) {
        aircraft_dictionary.SetReferencePosition(
            object_dictionary.composite_device_status.rp2040.rx_position.latitude_deg,
            object_dictionary.composite_device_status.rp2040.rx_position.longitude_deg);

        aircraft_dictionary.Update(timestamp_ms);
        last_aircraft_dictionary_update_timestamp_ms_ = timestamp_ms;
        uint32_t heap_free_kb = heap_caps_get_free_size(MALLOC_CAP_8BIT) / 1024;
        uint32_t heap_total_kb = heap_caps_get_total_size(MALLOC_CAP_8BIT) / 1024;
        uint16_t ws_used = network_console.GetNumClients() + network_metrics.GetNumClients() +
                           network_aircraft.GetNumClients();
        uint16_t ws_total = network_console.GetNumClientsAllowed() + network_metrics.GetNumClientsAllowed() +
                            network_aircraft.GetNumClientsAllowed();
        CONSOLE_INFO("ADSBeeServer::Update", "\t %d clients, %d aircraft, heap: %luK free / %luK total, ws: %d/%d",
                     comms_manager.GetNumWiFiClients(), aircraft_dictionary.GetNumAircraft(), heap_free_kb,
                     heap_total_kb, ws_used, ws_total);

        object_dictionary.UpdateDeviceStatus();  // Get newest values for ESP32 status.
        // RP2040 and SubG status are updated by the RP2040 writing to the object dictionary periodically. Nothing to do
        // here.
        SendNetworkMetricsMessage();
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
                                  "Failed to ingest decoded Mode S packet into aircraft dictionary.");
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
            // clients here (separate from the traffic reporting that occurs on regular intervals).
            for (uint16_t i = 0; i < raw_packets.header->num_uat_uplink_packets; i++) {
                RawUATUplinkPacket& raw_uat_uplink_packet = raw_packets.uat_uplink_packets[i];
                DecodedUATUplinkPacket decoded_packet = DecodedUATUplinkPacket(raw_uat_uplink_packet);
                if (!decoded_packet.is_valid) {
                    CONSOLE_ERROR("ADSBeeServer::Update", "Received invalid UAT Uplink packet.");
                    continue;
                }
                ReportGDL90UplinkDataMessage(decoded_packet);
            }
        }
    }

    // Broadcast aircraft locations to connected WiFi clients over GDL90.
    if (timestamp_ms - last_gdl90_report_timestamp_ms_ > kGDL90ReportingIntervalMs) {
        last_gdl90_report_timestamp_ms_ = timestamp_ms;
        if (comms_manager.WiFiAccessPointHasClients() && !ReportGDL90()) {
            CONSOLE_ERROR("ADSBeeServer::Update", "Encountered error while reporting GDL90.");
            ret = false;
        }
    }

    // Broadcast aircraft JSON to connected /aircraft WebSocket clients.
    if (timestamp_ms - last_aircraft_json_report_timestamp_ms_ > kAircraftJSONReportingIntervalMs) {
        last_aircraft_json_report_timestamp_ms_ = timestamp_ms;
        if (network_aircraft.GetNumClients() > 0) {
            SendAircraftJSONMessages();
        }
    }

    // Prune inactive WebSocket clients and other housekeeping.
    network_console.Update();
    network_metrics.Update();
    network_aircraft.Update();

    // Check to see whether the RP2040 sent over new metrics.
    xQueueReceive(rp2040_aircraft_dictionary_metrics_queue, &rp2040_aircraft_dictionary_metrics, 0);

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
    if (!settings_manager.settings.core_network_settings.wifi_ap_enabled) {
        return true;  // Nothing to do.
    }

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
    comms_manager.WiFiAccessPointSendMessageToAllStations(message);
    message.len = 0;

    // Ownship Report
    message.len = gdl90.WriteGDL90TargetReportMessage(message.data, CommsManager::NetworkMessage::kMaxLenBytes,
                                                      ownship_data, true);
    comms_manager.WiFiAccessPointSendMessageToAllStations(message);
    message.len = 0;

    // Traffic Reports
    int16_t aircraft_index = -1;  // Just used for error reporting.
    uint8_t aircraft_msg_buf[CommsManager::NetworkMessage::kMaxLenBytes];
    uint16_t aircraft_msg_buf_len = 0;
    for (auto& itr : aircraft_dictionary.dict) {
        aircraft_index++;

        if (ModeSAircraft* mode_s_aircraft = get_if<ModeSAircraft>(&(itr.second)); mode_s_aircraft) {
            if (!mode_s_aircraft->HasBitFlag(ModeSAircraft::kBitFlagPositionValid) ||
                mode_s_aircraft->icao_address == ownship_data.participant_address) {
                // Don't report aircraft without a valid position, and ignore ownship position when bootstrapping off a
                // fixed ICAO.
                continue;
            }
            printf("\t#A %s (0x%06lX): %.5f %.5f %ld\r\n", mode_s_aircraft->callsign, mode_s_aircraft->icao_address,
                   mode_s_aircraft->latitude_deg, mode_s_aircraft->longitude_deg, mode_s_aircraft->baro_altitude_ft);
            aircraft_msg_buf_len = gdl90.WriteGDL90TargetReportMessage(aircraft_msg_buf, sizeof(aircraft_msg_buf),
                                                                       *mode_s_aircraft, false);
        } else if (UATAircraft* uat_aircraft = get_if<UATAircraft>(&(itr.second)); uat_aircraft) {
            if (!uat_aircraft->HasBitFlag(UATAircraft::kBitFlagPositionValid) ||
                uat_aircraft->icao_address == ownship_data.participant_address) {
                // Don't report aircraft without a valid position, and ignore ownship position when bootstrapping off a
                // fixed ICAO.
                continue;
            }
            printf("\t#U %s (0x%06lX): %.5f %.5f %ld\r\n", uat_aircraft->callsign, uat_aircraft->icao_address,
                   uat_aircraft->latitude_deg, uat_aircraft->longitude_deg, uat_aircraft->baro_altitude_ft);
            aircraft_msg_buf_len =
                gdl90.WriteGDL90TargetReportMessage(aircraft_msg_buf, sizeof(aircraft_msg_buf), *uat_aircraft, false);
        } else if (RemoteIDAircraft* remote_id_aircraft = get_if<RemoteIDAircraft>(&(itr.second)); remote_id_aircraft) {
            if (!remote_id_aircraft->HasBitFlag(RemoteIDAircraft::kBitFlagPositionValid)) {
                // Don't report drones without a valid position.
                continue;
            }
            aircraft_msg_buf_len = gdl90.WriteGDL90TargetReportMessage(aircraft_msg_buf, sizeof(aircraft_msg_buf),
                                                                       *remote_id_aircraft, false);
        } else {
            CONSOLE_WARNING("CommsManager::ReportGDL90", "Unknown aircraft type in dictionary for UID 0x%lx.",
                            itr.first);
            continue;
        }

        if (message.len + aircraft_msg_buf_len <= CommsManager::NetworkMessage::kMaxLenBytes) {
            // We can tack this aircraft message onto the existing message to save space in the queue.
            memcpy(message.data + message.len, aircraft_msg_buf, aircraft_msg_buf_len);
            message.len += aircraft_msg_buf_len;
        } else {
            // Send the existing message and start a new one.
            if (!comms_manager.WiFiAccessPointSendMessageToAllStations(message)) {
                CONSOLE_ERROR("ADSBeeServer::ReportGDL90", "Failed to send info about aircraft %d to all clients.",
                              aircraft_index);
            }
            memcpy(message.data, aircraft_msg_buf, aircraft_msg_buf_len);
            message.len = aircraft_msg_buf_len;
        }
    }

    // Send any remaining message data.
    if (message.len > 0) {
        if (!comms_manager.WiFiAccessPointSendMessageToAllStations(message)) {
            CONSOLE_ERROR("ADSBeeServer::ReportGDL90", "Failed to send final aircraft message to all clients.");
        }
    }

    return true;
}

bool ADSBeeServer::ReportGDL90UplinkDataMessage(const DecodedUATUplinkPacket& uplink_packet) {
    if (!settings_manager.settings.core_network_settings.wifi_ap_enabled) {
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

    if (!comms_manager.WiFiAccessPointSendMessageToAllStations(message)) {
        CONSOLE_ERROR("ADSBeeServer::ReportGDL90UplinkPacket", "Failed to send UAT uplink packet to all clients.");
        return false;
    }

    return true;
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

static esp_err_t root_handler(httpd_req_t* req) {
    httpd_resp_send(req, (const char*)index_html_start, index_html_end - index_html_start);
    return ESP_OK;
}

static esp_err_t css_handler(httpd_req_t* req) {
    // Set the CSS content type
    httpd_resp_set_type(req, "text/css");

    // Send the CSS file
    httpd_resp_send(req, (const char*)style_css_start, style_css_end - style_css_start);

    return ESP_OK;
}

static esp_err_t adsbee_js_handler(httpd_req_t* req) {
    httpd_resp_set_type(req, "application/javascript");
    httpd_resp_send(req, (const char*)adsbee_js_start, adsbee_js_end - adsbee_js_start - 1);
    return ESP_OK;
}

static esp_err_t settings_js_handler(httpd_req_t* req) {
    httpd_resp_set_type(req, "application/javascript");
    // EMBED_TXTFILES null-terminates the embedded file; don't send the terminator.
    httpd_resp_send(req, (const char*)settings_js_start, settings_js_end - settings_js_start - 1);
    return ESP_OK;
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
    char json[512];
    snprintf(json, sizeof(json), "{\"index\":%d,\"uri\":\"%s\",\"port\":%d,\"active\":%d,\"protocol\":\"%s\"}", index,
             escaped_uri, settings_manager.settings.feed_ports[index],
             (int)settings_manager.settings.feed_is_active[index],
             SettingsManager::kReportingProtocolStrs[settings_manager.settings.feed_protocols[index]]);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t favicon_handler(httpd_req_t* req) {
    httpd_resp_set_type(req, "image/png");
    httpd_resp_set_hdr(req, "Cache-Control", "max-age=2592000, public");  // Cache for 30 days

    esp_err_t res = httpd_resp_send(req, (const char*)favicon_png_start, favicon_png_end - favicon_png_start);
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
                comms_manager.feed_mps, "%u", false);  // Mo trailing comma.
    snprintf(metrics_message + strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
             kNetworkMetricsMessageMaxLen - strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
             "}, \"device_status\": { ");
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
             "\"longitude_deg\": %.6f, \"utc_time\": \"%s\" }",
             rp2040_status.gnss_enabled ? "true" : "false", rp2040_status.gnss_fix_valid ? "true" : "false",
             rp2040_status.gnss_latitude_deg, rp2040_status.gnss_longitude_deg, gnss_utc_time);
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
    snprintf(metrics_message + strnlen(metrics_message, kNetworkMetricsMessageMaxLen),
             kNetworkMetricsMessageMaxLen - strnlen(metrics_message, kNetworkMetricsMessageMaxLen), " }}\n");

    network_metrics.BroadcastMessage(metrics_message, strnlen(metrics_message, kNetworkMetricsMessageMaxLen));
}

bool ADSBeeServer::TCPServerInit() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = kHTTPServerStackSizeBytes;
    // config.task_caps = MALLOC_CAP_IRAM_8BIT;
    config.max_open_sockets = 16;  // Must be <= CONFIG_LWIP_MAX_SOCKETS - 3 (currently 20 - 3 = 17).
    // 9 URIs are registered below (6 HTTP + 3 websocket); the default limit is 8 and a
    // failed registration aborts boot via ESP_ERROR_CHECK.
    config.max_uri_handlers = 16;
    config.close_fn = ws_close_fd;
    config.lru_purge_enable =
        true;  // Allow purging of the least recently used connections when max clients is reached.
    config.keep_alive_enable = true;
    config.keep_alive_idle = 5;      // Seconds idle before sending first keepalive probe.
    config.keep_alive_interval = 5;  // Seconds between keepalive probes.
    config.keep_alive_count = 3;     // Close connection after 3 failed probes (~20s dead detection).

    esp_err_t ret = httpd_start(&server, &config);
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
                                       .num_clients_allowed = 4,
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
                                       .num_clients_allowed = 4,
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
                                        .num_clients_allowed = 4,
                                        .send_as_binary = false,
                                        .inactivity_timeout_ms = 90 * 1000,
                                        .post_connect_callback = nullptr,
                                        .message_received_callback = nullptr});
    if (!network_aircraft.Init()) {
        CONSOLE_ERROR("ADSBeeServer::TCPServerInit", "Failed to start Network Aircraft WebSocket server.");
        return false;
    }

    if (server == nullptr) {
        CONSOLE_ERROR("ADSBeeServer::TCPServerInit", "HTTP server instance is null after start.");
        return false;
    }
    CONSOLE_INFO("ADSBeeServer::TCPServerInit", "HTTP server started successfully, remaining stack %u Bytes.",
                 uxTaskGetStackHighWaterMark(NULL));

    return true;
}

void ADSBeeServer::SendAircraftJSONMessages() {
    char json_buf[kAircraftJSONMessageStrMaxLen];
    for (auto& itr : aircraft_dictionary.dict) {
        int16_t len = -1;
        if (ModeSAircraft* ac = get_if<ModeSAircraft>(&itr.second); ac) {
            len = WriteAircraftJSONModeSAircraftStr(json_buf, *ac);
        } else if (UATAircraft* ac = get_if<UATAircraft>(&itr.second); ac) {
            len = WriteAircraftJSONUATAircraftStr(json_buf, *ac);
        } else if (RemoteIDAircraft* ac = get_if<RemoteIDAircraft>(&itr.second); ac) {
            len = WriteAircraftJSONRemoteIDAircraftStr(json_buf, *ac);
        }
        if (len > 0) {
            network_aircraft.BroadcastMessage(json_buf, len);
        }
    }
}
