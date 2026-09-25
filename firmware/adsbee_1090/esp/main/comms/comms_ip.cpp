#include "beast/beast_utils.hh"  // For beast reporting.
#include "comms.hh"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "hardware_capabilities.hh"  // Internal-RAM heap guards (PSRAM must not mask an internal shortage).
#include "esp_mac.h"
#include "hal.hh"
#include "lwip/dns.h"
#include "lwip/err.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "mdns.h"
#include "task_priorities.hh"

static const uint32_t kTCPSocketReconnectIntervalMs = 10000;

// Heap threshold for back-pressure. If free heap drops below this, safe_send will block.
// Set high enough to leave room for WebSocket allocations (~2KB) and other system needs.
static const uint32_t kHeapBackPressureThresholdBytes = 20480;
// DMA-capable memory threshold. WiFi TX buffers require internal DMA memory which is more constrained. When WiFi is
// enabled we must reserve enough DMA-capable SRAM for esp_wifi's TX path; but on an Ethernet-only unit esp_wifi_init()
// is never called, so those buffers are never allocated and reserving 16 KB for them is a false back-pressure trip
// (it starved the WAN queue on the PSRAM-less S3 running Bluetooth Remote ID). Use the lower Ethernet-only threshold
// when both WiFi AP and STA are disabled.
static const uint32_t kDMABackPressureThresholdBytes = 16384;
static const uint32_t kDMABackPressureThresholdEthernetOnlyBytes = 10240;
static const uint32_t kHeapBackPressureCheckIntervalMs = 50;
static const uint32_t kHeapBackPressureTimeoutMs = 500;

// #define ENABLE_TCP_SOCKET_RATE_LIMITING
#ifdef ENABLE_TCP_SOCKET_RATE_LIMITING
static const uint32_t kSendBufRateLimitBytesPerSecond = 100000;  // 100 kB/s
static const uint32_t kSendBufRateLimitIntervalMs = 10;
static const uint32_t kSendBufRateLimitBytesPerInterval =
    kSendBufRateLimitBytesPerSecond / (1000 / kSendBufRateLimitIntervalMs);
static const uint32_t kRateLimitDelayDurationMs = 1;
#endif

static const uint32_t kTCPKeepAliveEnable = 1;
static const uint32_t kTCPKeepAliveIdleSecondsBeforeStartingProbe = 120;
static const uint32_t kTCPKeepAliveIntervalSecondsBetweenProbes = 30;
static const uint32_t kTCPKeepAliveMaxFailedProbesBeforeDisconnect = 3;
static const uint32_t kTCPReuseAddrEnable =
    1;  // Allow reuse of local addresses and sockets that are in the TIME_WAIT state.

/** "Pass-Through" functions used to access member functions in callbacks. **/
void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    comms_manager.IPEventHandler(arg, event_base, event_id, event_data);
}
void ip_wan_task(void* pvParameters) { comms_manager.IPWANTask(pvParameters); }
/** End "Pass-Through" functions. **/

bool IsNotIPAddress(const char* uri) {
    // Check if the URI contains any letters
    for (const char* p = uri; *p != '\0'; p++) {
        if (isalpha(*p)) {
            return true;
        }
    }
    return false;
}

bool ResolveURIToIP(const char* url, char* ip) {
    struct addrinfo hints;
    struct addrinfo* res;
    struct in_addr addr;
    int err;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;

    err = getaddrinfo(url, NULL, &hints, &res);
    if (err != 0 || res == NULL) {
        CONSOLE_ERROR("ResolveURLToIP", "DNS lookup failed for %s: %d", url, err);
        freeaddrinfo(res);
        return false;
    }

    addr.s_addr = ((struct sockaddr_in*)res->ai_addr)->sin_addr.s_addr;
    inet_ntop(AF_INET, &addr, ip, 16);
    CONSOLE_INFO("ResolveURLToIP", "DNS lookup succeeded. IP=%s", ip);

    freeaddrinfo(res);
    return true;
}

esp_err_t safe_send(int sock, const void* data, size_t total_len) {
    // 0. Back-pressure: block if heap or DMA memory is low to prevent OOM crashes. The DMA reserve only needs to cover
    // esp_wifi's TX buffers, which don't exist on an Ethernet-only unit (esp_wifi_init() is never called), so drop the
    // DMA threshold in that case to avoid a false back-pressure trip that starves the WAN queue.
    const uint32_t dma_backpressure_threshold_bytes =
        (comms_manager.wifi_ap_enabled || comms_manager.wifi_sta_enabled) ? kDMABackPressureThresholdBytes
                                                                          : kDMABackPressureThresholdEthernetOnlyBytes;
    uint32_t backpressure_start_ms = get_time_since_boot_ms();
    while (HardwareCapabilities::GetInternalFreeBytes() < kHeapBackPressureThresholdBytes ||
           heap_caps_get_free_size(MALLOC_CAP_DMA) < dma_backpressure_threshold_bytes) {
        if (get_time_since_boot_ms() - backpressure_start_ms > kHeapBackPressureTimeoutMs) {
            CONSOLE_WARNING("safe_send", "Heap back-pressure timeout after %lu ms (heap=%u, dma=%u)",
                            kHeapBackPressureTimeoutMs, HardwareCapabilities::GetInternalFreeBytes(),
                            heap_caps_get_free_size(MALLOC_CAP_DMA));
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(kHeapBackPressureCheckIntervalMs));
    }

    // 1. Set socket to non-blocking mode
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    size_t sent_total = 0;
    while (sent_total < total_len) {
        fd_set write_set;
        FD_ZERO(&write_set);
        FD_SET(sock, &write_set);

        // 2. Set a timeout for select()
        struct timeval timeout = {.tv_sec = 1,  // 1 second timeout
                                  .tv_usec = 0};

        // Wait until the socket is ready to accept data
        int res = select(sock + 1, NULL, &write_set, NULL, &timeout);

        if (res < 0) {
            CONSOLE_ERROR("safe_send", "Select error: %d", errno);
            return ESP_FAIL;
        } else if (res == 0) {
            CONSOLE_WARNING("safe_send", "Send timeout - Network congested");
            return ESP_ERR_TIMEOUT;
        }

        // 3. Socket is ready, try to send the remaining chunk
        int sent_now = send(sock, static_cast<const uint8_t*>(data) + sent_total, total_len - sent_total, 0);

        if (sent_now > 0) {
            sent_total += sent_now;
            // CONSOLE_INFO("safe_send", "Sent %d bytes (%zu/%zu)", sent_now, sent_total, total_len);
        } else if (sent_now < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // This shouldn't really happen after select() says it's ready,
                // but it's good practice to handle it.
                continue;
            } else {
                CONSOLE_ERROR("safe_send", "Socket error during send: %d", errno);
                return ESP_FAIL;
            }
        }
    }

    return ESP_OK;
}

bool CommsManager::IPInit() {
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &ip_event_handler, NULL));
    ip_event_handler_was_initialized_ = true;

    // IP WAN task stack size reduced since raw_packets_buf is now allocated on heap instead of stack.
    xTaskCreate(ip_wan_task, "ip_wan_task", 2 * 4096, &ip_wan_task_handle, kIPWANTaskPriority, NULL);

    // Initialize mDNS service.
    esp_err_t err = mdns_init();
    if (err) {
        CONSOLE_ERROR("CommsManager::IPInit", "MDNS Init failed: %d\n", err);
        return false;
    }

    // Set hostname.
    err = mdns_hostname_set(settings_manager.settings.core_network_settings.hostname);
    if (err) {
        CONSOLE_ERROR("CommsManager::IPInit", "Failed setting MDNS hostname to %s: %d\n",
                      settings_manager.settings.core_network_settings.hostname, err);
        return false;
    }
    // Set default instance.
    mdns_instance_name_set("ADSBee 1090");

    CONSOLE_INFO("CommsManager::IPInit", "MDNS initialized with hostname %s.\n",
                 settings_manager.settings.core_network_settings.hostname);
    return true;
}

void CommsManager::IPEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    switch (event_id) {
        case IP_EVENT_AP_STAIPASSIGNED: {
            // A new station has connected to the ADSBee's softAP network.
            ip_event_ap_staipassigned_t* event = (ip_event_ap_staipassigned_t*)event_data;

            char client_mac_str[SettingsManager::Settings::kMACAddrStrLen + 1] = {0};
            char client_ip_str[SettingsManager::Settings::kIPAddrStrLen + 1] = {0};
            snprintf(client_mac_str, SettingsManager::Settings::kMACAddrStrLen, MACSTR, MAC2STR(event->mac));
            client_mac_str[SettingsManager::Settings::kMACAddrStrLen] = '\0';
            snprintf(client_ip_str, SettingsManager::Settings::kIPAddrStrLen, IPSTR, IP2STR(&event->ip));
            client_ip_str[SettingsManager::Settings::kIPAddrStrLen] = '\0';

            CONSOLE_INFO("CommsManager::IPEventHandler",
                         "WiFi Access Point assigned IP address to client. IP: %s, MAC: %s", client_ip_str,
                         client_mac_str);

            WiFiAddClient(event->ip, event->mac);
            break;
        }
        case IP_EVENT_STA_GOT_IP: {
            // The ADSBee has connected to an external network as a WiFi station.
            ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
            const esp_netif_ip_info_t* ip_info = &event->ip_info;

            wifi_sta_has_ip_ = true;
            snprintf(wifi_sta_ip, SettingsManager::Settings::kIPAddrStrLen, IPSTR, IP2STR(&ip_info->ip));
            wifi_sta_ip[SettingsManager::Settings::kIPAddrStrLen] = '\0';
            snprintf(wifi_sta_netmask, SettingsManager::Settings::kIPAddrStrLen, IPSTR, IP2STR(&ip_info->netmask));
            wifi_sta_netmask[SettingsManager::Settings::kIPAddrStrLen] = '\0';
            snprintf(wifi_sta_gateway, SettingsManager::Settings::kIPAddrStrLen, IPSTR, IP2STR(&ip_info->gw));
            wifi_sta_gateway[SettingsManager::Settings::kIPAddrStrLen] = '\0';

            CONSOLE_INFO("CommsManager::IPEventHandler",
                         "WiFi Station got IP Address. IP: %s, Netmask: %s, Gateway: %s", wifi_sta_ip, wifi_sta_netmask,
                         wifi_sta_gateway);
            break;
        }
        case IP_EVENT_ETH_GOT_IP: {
            // The ADSBee's Ethernet interface has connected to an external network.
            ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
            const esp_netif_ip_info_t* ip_info = &event->ip_info;

            ethernet_has_ip_ = true;
            snprintf(ethernet_ip, SettingsManager::Settings::kIPAddrStrLen, IPSTR, IP2STR(&ip_info->ip));
            ethernet_ip[SettingsManager::Settings::kIPAddrStrLen] = '\0';
            snprintf(ethernet_netmask, SettingsManager::Settings::kIPAddrStrLen, IPSTR, IP2STR(&ip_info->netmask));
            ethernet_netmask[SettingsManager::Settings::kIPAddrStrLen] = '\0';
            snprintf(ethernet_gateway, SettingsManager::Settings::kIPAddrStrLen, IPSTR, IP2STR(&ip_info->gw));
            ethernet_gateway[SettingsManager::Settings::kIPAddrStrLen] = '\0';

            CONSOLE_INFO("CommsManager::IPEventHandler", "Ethernet got IP address. IP: %s, Netmask: %s, Gateway: %s",
                         ethernet_ip, ethernet_netmask, ethernet_gateway);
            break;
        }
        case IP_EVENT_ETH_LOST_IP: {
            // The ADSBee's Ethernet interface has disconnected from an external network.
            ethernet_has_ip_ = false;
            CONSOLE_INFO("CommsManager::IPEventHandler", "Ethernet lost IP address.");
            break;
        }
    }
}

void CommsManager::IPWANTask(void* pvParameters) {
    CONSOLE_INFO("CommsManager::IPWANTask", "IP WAN Task started.");

    uint8_t* raw_packets_buf = (uint8_t*)heap_caps_malloc(CompositeArray::RawPackets::kMaxLenBytes, MALLOC_CAP_8BIT);
    if (raw_packets_buf == nullptr) {
        CONSOLE_ERROR("CommsManager::IPWANTask", "Failed to allocate raw packets buffer on heap. Task exiting.");
        vTaskDelete(NULL);
        return;
    }

    while (true) {
        // Don't try establishing socket connections until the ESP32 has been assigned an IP address.
        while (!HasIP()) {
            vTaskDelay(1);  // Delay for 1 tick.
        }

        UpdateFeedMetrics();

        uint16_t num_active_report_sinks = 0;
        ReportSink active_report_sinks[SettingsManager::Settings::kMaxNumFeeds] = {0};
        SettingsManager::ReportingProtocol active_reporting_protocols[SettingsManager::Settings::kMaxNumFeeds] = {
            SettingsManager::ReportingProtocol::kNoReports};

        // Maintain socket connections and build list of acive report sinks.
        for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
            // Iterate through feeds, open/close and send message as required. The feeds_enabled master switch gates
            // all outbound feed connections: when it's off no feed sockets are opened, regardless of the per-feed
            // feed_is_active[] flags.
            if (!settings_manager.settings.feeds_enabled || !settings_manager.settings.feed_is_active[i]) {
                // Feed is not active, ensure socket is closed.
                if (feed_sock_is_connected_[i]) {
                    // Need to close the socket connection.
                    CloseFeedSocket(i);
                }
                continue;
            } else {
                // Feed is active, ensure socket is open and add to active sinks if connected.
                if (!feed_sock_is_connected_[i] && !ConnectFeedSocket(i)) {
                    // Need to open the socket connection, but failed to do so.
                    continue;  // Failed to connect, try again later.
                }
                // Socket is connected, add to active sinks.
                active_report_sinks[num_active_report_sinks] = i;
                active_reporting_protocols[num_active_report_sinks] = settings_manager.settings.feed_protocols[i];
                num_active_report_sinks++;
            }
        }

        // Gather packet(s) to send.
        /**
         * Architecture note: We send packets in the queue as a buffer, since the buffer in the other task gets
         * deallocated. Here, we can unpack the buffer into a CompositeArray object with pointers because we know it
         * won't go out of scope till we are done with it.
         */
        if (xQueueReceive(ip_wan_reporting_composite_array_queue_, raw_packets_buf, kWiFiSTATaskUpdateIntervalTicks) !=
            pdTRUE) {
            // No packets available to send, wait and try again.
            continue;
        }

        CompositeArray::RawPackets reporting_composite_array;
        if (!CompositeArray::UnpackRawPacketsBuffer(reporting_composite_array, raw_packets_buf,
                                                    CompositeArray::RawPackets::kMaxLenBytes)) {
            CONSOLE_ERROR("CommsManager::IPWANTask", "Failed to unpack CompositeArray from buffer.");
            continue;
        }

        // Update feeds with raw and digested reports.
        if (!UpdateReporting(active_report_sinks, active_reporting_protocols, num_active_report_sinks,
                             &reporting_composite_array)) {
            CONSOLE_ERROR("CommsManager::IPWANTask", "Error during UpdateReporting for feeds.");
        }
    }

    // Close all sockets while exiting.
    for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
        CloseFeedSocket(i);
    }

    // Free heap buffer.
    heap_caps_free(raw_packets_buf);

    CONSOLE_INFO("CommsManager::IPWANTask", "IP WAN Task exiting.");
}

void CommsManager::CloseFeedSocket(uint16_t feed_index) {
    // Need to close the socket connection.
    close(feed_sock_[feed_index]);
    feed_sock_is_connected_[feed_index] = false;
    CONSOLE_INFO("CommsManager::IPWANTask", "Closed socket for feed %d.", feed_index);
}

bool CommsManager::ConnectFeedSocket(uint16_t feed_index) {
    // Meter reconnect attempt interval.
    uint32_t timestamp_ms = get_time_since_boot_ms();
    if (timestamp_ms - feed_sock_last_connect_timestamp_ms_[feed_index] <= kTCPSocketReconnectIntervalMs) {
        return false;
    }
    feed_sock_last_connect_timestamp_ms_[feed_index] = timestamp_ms;

    // Create socket.
    // IPv4, TCP
    feed_sock_[feed_index] = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (feed_sock_[feed_index] <= 0) {
        CONSOLE_ERROR("CommsManager::IPWANTask", "Unable to create socket for feed %d: errno %d (%s)", feed_index,
                      errno, strerror(errno));
        CloseFeedSocket(feed_index);
        return false;
    }
    CONSOLE_INFO("CommsManager::IPWANTask", "Socket for feed %d created, connecting to %s:%d", feed_index,
                 settings_manager.settings.feed_uris[feed_index], settings_manager.settings.feed_ports[feed_index]);

    // Enable TCP keepalive
    setsockopt(feed_sock_[feed_index], SOL_SOCKET, SO_KEEPALIVE, &kTCPKeepAliveEnable, sizeof(kTCPKeepAliveEnable));
    setsockopt(feed_sock_[feed_index], IPPROTO_TCP, TCP_KEEPIDLE, &kTCPKeepAliveIdleSecondsBeforeStartingProbe,
               sizeof(kTCPKeepAliveIdleSecondsBeforeStartingProbe));
    setsockopt(feed_sock_[feed_index], IPPROTO_TCP, TCP_KEEPINTVL, &kTCPKeepAliveIntervalSecondsBetweenProbes,
               sizeof(kTCPKeepAliveIntervalSecondsBetweenProbes));
    setsockopt(feed_sock_[feed_index], IPPROTO_TCP, TCP_KEEPCNT, &kTCPKeepAliveMaxFailedProbesBeforeDisconnect,
               sizeof(kTCPKeepAliveMaxFailedProbesBeforeDisconnect));
    // Allow reuse of local addresses.
    setsockopt(feed_sock_[feed_index], SOL_SOCKET, SO_REUSEADDR, &kTCPReuseAddrEnable, sizeof(kTCPReuseAddrEnable));

    struct sockaddr_in dest_addr;
    // If the URI contains letters, resolve it to an IP address
    if (IsNotIPAddress(settings_manager.settings.feed_uris[feed_index])) {
        // Is not an IP address, try DNS resolution.
        char resolved_ip[16];
        if (!ResolveURIToIP(settings_manager.settings.feed_uris[feed_index], resolved_ip)) {
            CONSOLE_ERROR("CommsManager::IPWANTask", "Failed to resolve URL %s for feed %d",
                          settings_manager.settings.feed_uris[feed_index], feed_index);
            CloseFeedSocket(feed_index);
            return false;
        }
        inet_pton(AF_INET, resolved_ip, &dest_addr.sin_addr);
    } else {
        // Is an IP address, use it directly.
        inet_pton(AF_INET, settings_manager.settings.feed_uris[feed_index], &dest_addr.sin_addr);
    }

    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(settings_manager.settings.feed_ports[feed_index]);

    int err = connect(feed_sock_[feed_index], (struct sockaddr*)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        CONSOLE_ERROR("CommsManager::IPWANTask", "Socket unable to connect to URI %s:%d for feed %d: errno %d (%s)",
                      settings_manager.settings.feed_uris[feed_index], settings_manager.settings.feed_ports[feed_index],
                      feed_index, errno, strerror(errno));
        CloseFeedSocket(feed_index);
        return false;
    }
    CONSOLE_INFO("CommsManager::IPWANTask", "Successfully connected to %s",
                 settings_manager.settings.feed_uris[feed_index]);
    feed_sock_is_connected_[feed_index] = true;

    // Perform beginning-of-connection actions here.
    switch (settings_manager.settings.feed_protocols[feed_index]) {
        case SettingsManager::ReportingProtocol::kBeast:
        case SettingsManager::ReportingProtocol::kBeastNoUAT:
        case SettingsManager::ReportingProtocol::kBeastNoUATUplink: {
            uint8_t beast_message_buf[2 * SettingsManager::Settings::kFeedReceiverIDNumBytes +
                                      BeastReporter::kModeSBeastFrameMaxLenBytes];
            uint16_t beast_message_len_bytes = BeastReporter::BuildFeedStartFrame(
                beast_message_buf, settings_manager.settings.feed_receiver_ids[feed_index]);
            int err = safe_send(feed_sock_[feed_index], beast_message_buf, beast_message_len_bytes);
            if (err < 0) {
                CONSOLE_ERROR("CommsManager::IPWANTask",
                              "Error occurred while sending %d Byte Beast start of feed message to feed %d "
                              "with URI %s "
                              "on port %d: "
                              "errno %d.",
                              beast_message_len_bytes, feed_index, settings_manager.settings.feed_uris[feed_index],
                              settings_manager.settings.feed_ports[feed_index], errno);
                // Mark socket as disconnected and try reconnecting in next reporting interval.
                CloseFeedSocket(feed_index);
                return false;
            }
            break;
        }
        default:
            // No start of connections actions required for other protocols.
            break;
    }
    return true;
}

bool CommsManager::SendBuf(uint16_t iface, const char* buf, uint16_t buf_len, uint16_t num_msgs) {
    if (iface >= SettingsManager::Settings::kMaxNumFeeds) {
        CONSOLE_ERROR("CommsManager::SendBuf", "Invalid feed index %d.", iface);
        return false;
    }
    if (!feed_sock_is_connected_[iface]) {
        CONSOLE_ERROR("CommsManager::SendBuf", "Can't send to feed %d, socket not connected.", iface);
        return false;
    }

#ifdef ENABLE_TCP_SOCKET_RATE_LIMITING
    uint32_t timestamp_ms = get_time_since_boot_ms();
    if (timestamp_ms - last_send_buf_counter_reset_timestamp_ms > kSendBufRateLimitIntervalMs) {
        // Reset the counter every 10ms.
        last_send_buf_counter_reset_timestamp_ms = timestamp_ms;
        send_buf_counter_bytes = 0;
    }
    if (send_buf_counter_bytes + buf_len > kSendBufRateLimitBytesPerInterval) {
        vTaskDelay(
            pdTICKS_TO_MS(kRateLimitDelayDurationMs));  // Delay to yield to other tasks and allow buffers to clear.
    }
    send_buf_counter_bytes += buf_len;
#endif

    int err = safe_send(feed_sock_[iface], buf, buf_len);

    if (err < 0) {
        CONSOLE_ERROR("CommsManager::SendBuf",
                      "Error occurred during sending %d byte message to feed %d with URI %s "
                      "on port %d: "
                      "errno %d (%s).",
                      buf_len, iface, settings_manager.settings.feed_uris[iface],
                      settings_manager.settings.feed_ports[iface], errno, strerror(errno));
        CloseFeedSocket(iface);
        return false;
    } else {
        // CONSOLE_INFO("CommsManager::IPWANTask", "Message sent to feed %d.", i);
        feed_mps_counter_[iface] += num_msgs;  // Log that a message was sent in statistics.
    }
    return true;
}

void CommsManager::UpdateFeedMetrics() {
    // Update feed statistics once per second and print them. Put this before the queue receive so that it runs even
    // if no packets are received.
    static const uint16_t kStatsMessageMaxLen = 500;
    uint32_t timestamp_ms = get_time_since_boot_ms();
    if (timestamp_ms - feed_mps_last_update_timestamp_ms_ > kMsPerSec) {
        for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
            feed_mps[i] = feed_mps_counter_[i];
            feed_mps_counter_[i] = 0;
        }
        feed_mps_last_update_timestamp_ms_ = timestamp_ms;

        char feeds_metrics_message[kStatsMessageMaxLen] = {'\0'};

        for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
            char single_feed_metrics_message[kStatsMessageMaxLen / SettingsManager::Settings::kMaxNumFeeds] = {'\0'};
            snprintf(single_feed_metrics_message, kStatsMessageMaxLen / SettingsManager::Settings::kMaxNumFeeds,
                     "%d:[%d] ", i, feed_mps[i]);
            strcat(feeds_metrics_message, single_feed_metrics_message);
        }
        CONSOLE_INFO("CommsManager::IPWANTask", "Feed msgs/s: %s", feeds_metrics_message);
    }
}
