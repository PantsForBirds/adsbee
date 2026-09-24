#include <string.h>

#include <functional>  // for std::bind

#include "cc.h"  // For endiannness swapping.
#include "comms.hh"
#include "esp_event.h"
#include "esp_mac.h"
#include "hal.hh"
#include "lwip/dns.h"
#include "lwip/err.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "nvs_flash.h"
#include "task_priorities.hh"
#include "utils/task_utils.hh"  // For delayed reconnect callbacks.

static const uint16_t kWiFiNumRetries = 3;
static const uint16_t kWiFiRetryWaitTimeMs = 100;
static const uint16_t kWiFiStaMaxNumReconnectAttempts = 5;
static const uint16_t kWiFiScanDefaultListSize = 20;
static const uint16_t kWiFiAPMessageQueueTimeout = 100;     // ms
static const uint16_t kWiFiSTAMessageQueueTimeoutMs = 100;  // ms

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

/** "Pass-Through" functions used to access member functions in callbacks. **/
void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    comms_manager.WiFiEventHandler(arg, event_base, event_id, event_data);
}

void wifi_access_point_task(void* pvParameters) { comms_manager.WiFiAccessPointTask(pvParameters); }
inline void connect_to_wifi(void* arg = nullptr) {
    if (esp_wifi_connect() != ESP_OK) {
        CONSOLE_ERROR("connect_to_wifi", "Failed to connect to WiFi.");
    }
}
/** End "Pass-Through" functions. **/

void CommsManager::WiFiEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    switch (event_id) {
        case WIFI_EVENT_AP_STACONNECTED: {
            // A new station has connected to the ADSBee's softAP network.
            wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*)event_data;
            CONSOLE_INFO("CommsManager::WiFiEventHandler", "Station " MACSTR " joined, AID=%d", MAC2STR(event->mac),
                         event->aid);
            break;
        }
        case WIFI_EVENT_AP_STADISCONNECTED: {
            // A station has disconnected from the ADSBee's softAP network.
            wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*)event_data;
            CONSOLE_INFO("CommsManager::WiFiEventHandler", "Station " MACSTR " left, AID=%d", MAC2STR(event->mac),
                         event->aid);
            WiFiRemoveClient(event->mac);
            break;
        }
        case WIFI_EVENT_STA_START: {
            // The ADSBee is attempting to connect to an external network.
            char redacted_password[SettingsManager::Settings::kWiFiPasswordMaxLen];
            SettingsManager::RedactPassword(wifi_sta_password, redacted_password,
                                            SettingsManager::Settings::kWiFiPasswordMaxLen);
            CONSOLE_INFO("CommsManager::WiFiInit", "WiFi Station started. SSID:%s password:%s", wifi_sta_ssid,
                         redacted_password);
            ESP_ERROR_CHECK(esp_wifi_connect());
            // Note: wifi_sta_has_ip_ will get filled in by the IP event handler if an IP is issued.
            break;
        }
        case WIFI_EVENT_STA_DISCONNECTED: {
            // The ADSBee has disconnected from an external network.
            wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*)event_data;
            CONSOLE_ERROR("CommsManager::WiFiEventHandler",
                          "Disconnected from (or failed to connect to) ap SSID:%s password:%s - Disconnect reason : %d",
                          wifi_sta_ssid, wifi_sta_password, event->reason);
            wifi_sta_connected_ = false;
            wifi_sta_has_ip_ = false;
            if (wifi_sta_enabled) {
                ScheduleDelayedFunctionCall(kWiFiSTAReconnectIntervalMs, &connect_to_wifi);
            }
            break;
        }
        case WIFI_EVENT_STA_CONNECTED:
            char redacted_password[SettingsManager::Settings::kWiFiPasswordMaxLen];
            SettingsManager::RedactPassword(wifi_sta_password, redacted_password,
                                            SettingsManager::Settings::kWiFiPasswordMaxLen);
            CONSOLE_INFO("CommsManager::WiFiInit", "Connected to ap SSID:%s password:%s", wifi_sta_ssid,
                         redacted_password);
            wifi_sta_connected_ = true;
            wifi_sta_connected_timestamp_ms_ = get_time_since_boot_ms();
            break;
    }
}

void CommsManager::WiFiAccessPointTask(void* pvParameters) {
    NetworkMessage message;

    // Create socket.
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        CONSOLE_ERROR("CommsManager::WiFiAccessPointTask", "Unable to create socket: errno %d", errno);
        return;
    }

    // Set timeout
    struct timeval timeout;
    timeout.tv_sec = 10;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

    while (true) {
        if (xQueueReceive(wifi_ap_message_queue_, &message, portMAX_DELAY) == pdTRUE) {
            struct sockaddr_in dest_addr;
            dest_addr.sin_family = AF_INET;
            dest_addr.sin_port = htons(message.port);

            xSemaphoreTake(wifi_clients_list_mutex_, portMAX_DELAY);
            for (int i = 0; i < SettingsManager::Settings::kWiFiMaxNumClients; i++) {
                if (wifi_clients_list_[i].active) {
                    dest_addr.sin_addr.s_addr = wifi_clients_list_[i].ip.addr;
                    int ret = 0;
                    uint16_t num_tries;
                    for (num_tries = 0; num_tries < kWiFiNumRetries; num_tries++) {
                        ret =
                            sendto(sock, message.data, message.len, 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
                        // ENOMEM (errno=12) resolution: https://github.com/espressif/esp-idf/issues/390
                        // Increased the number of UDP control blocks (LWIP_MAX_UDP_PCBS) in SDK menuconfig
                        // from 16 to 96. Changed TCP/IP stack size from 3072 to 12288.
                        if (ret >= 0 || errno == ENOMEM) {
                            // Packet successfully sent, or we got ENOMEM. No retry required.
                            break;  // Drop immediately on ENOMEM — retrying against an exhausted pool wastes time.
                        } else {
                            // Non-ENOMEM error. Wait and retry.
                            vTaskDelay(kWiFiRetryWaitTimeMs / portTICK_PERIOD_MS);
                        }
                    }

                    if (ret < 0) {
                        // CONSOLE_ERROR("CommsManager::WiFiAccessPointTask", "Error occurred during sending:
                        // errno %d.", errno);
                        CONSOLE_ERROR("CommsManager::WiFiAccessPointTask",
                                      "Error occurred during sending: errno %d (%s). Tried %d times.", errno,
                                      strerror(errno), num_tries);
                    }
                }
            }
            xSemaphoreGive(wifi_clients_list_mutex_);

            // CONSOLE_INFO("CommsManager::WiFiUDPServerTask", "Message sent to %d clients.",
            // num_wifi_clients_);
        }
    }
    shutdown(sock, 0);
    close(sock);
}

bool CommsManager::WiFiInit() {
    esp_netif_t* wifi_ap_netif_ = esp_netif_create_default_wifi_ap();
    assert(wifi_ap_netif_);
    esp_netif_t* wifi_sta_netif_ = esp_netif_create_default_wifi_sta();
    assert(wifi_sta_netif_);

    ESP_ERROR_CHECK(esp_netif_set_hostname(wifi_sta_netif_, hostname));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    if (!ip_event_handler_was_initialized_) {
        IPInit();
    }

    wifi_mode_t wifi_mode;
    if (wifi_ap_enabled && wifi_sta_enabled) {
        wifi_mode = WIFI_MODE_APSTA;
    } else if (wifi_ap_enabled) {
        wifi_mode = WIFI_MODE_AP;
    } else {
        wifi_mode = WIFI_MODE_STA;
    }
    ESP_ERROR_CHECK(esp_wifi_set_mode(wifi_mode));

    wifi_was_initialized_ = true;

    if (wifi_ap_enabled) {
        // Access Point Configuration
        wifi_config_t wifi_config_ap = {};

        strncpy((char*)(wifi_config_ap.ap.ssid), wifi_ap_ssid, SettingsManager::Settings::kWiFiSSIDMaxLen + 1);
        strncpy((char*)(wifi_config_ap.ap.password), wifi_ap_password,
                SettingsManager::Settings::kWiFiPasswordMaxLen + 1);
        wifi_config_ap.ap.channel = wifi_ap_channel;
        wifi_config_ap.ap.ssid_len = (uint8_t)strnlen(wifi_ap_ssid, SettingsManager::Settings::kWiFiSSIDMaxLen);
        if (strnlen(wifi_ap_password, SettingsManager::Settings::kWiFiPasswordMaxLen) == 0) {
            wifi_config_ap.ap.authmode = WIFI_AUTH_OPEN;
        } else {
            wifi_config_ap.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
        }
        wifi_config_ap.ap.max_connection = SettingsManager::Settings::kWiFiMaxNumClients;

        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config_ap));
    }

    if (wifi_sta_enabled) {
        // Station Configuration
        wifi_config_t wifi_config_sta = {};

        strncpy((char*)(wifi_config_sta.sta.ssid), wifi_sta_ssid, SettingsManager::Settings::kWiFiSSIDMaxLen + 1);
        strncpy((char*)(wifi_config_sta.sta.password), wifi_sta_password,
                SettingsManager::Settings::kWiFiPasswordMaxLen + 1);

        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config_sta));
    }

    if (!wifi_ap_enabled && !wifi_sta_enabled) {
        ESP_ERROR_CHECK(esp_wifi_stop());
        CONSOLE_INFO("CommsManager::WiFiInit", "WiFi disabled.");
        return true;
    }

    ESP_ERROR_CHECK(esp_wifi_start());

    if (wifi_ap_enabled) {
        CONSOLE_INFO("CommsManager::WiFiInit", "WiFi AP started. SSID:%s password:%s", wifi_ap_ssid, wifi_ap_password);
        // Lazily create the AP broadcast queue now that the AP is actually enabled (see the CommsManager constructor).
        if (wifi_ap_message_queue_ == nullptr) {
            wifi_ap_message_queue_ = xQueueCreate(kWiFiMessageQueueLen, sizeof(NetworkMessage));
        }
        xTaskCreate(wifi_access_point_task, "wifi_ap_task", 2 * 4096, &wifi_ap_task_handle, kWiFiAPTaskPriority, NULL);
    }
    if (wifi_sta_enabled) {
        char redacted_password[SettingsManager::Settings::kWiFiPasswordMaxLen];
        SettingsManager::RedactPassword(wifi_sta_password, redacted_password,
                                        SettingsManager::Settings::kWiFiPasswordMaxLen);
        CONSOLE_INFO("CommsManager::WiFiInit", "WiFi Station started. SSID:%s password:%s", wifi_sta_ssid,
                     redacted_password);
    }

    return true;
}

bool CommsManager::WiFiDeInit() {
    if (!wifi_was_initialized_) return true;  // Don't try de-initializing if it was never initialized.

    // The de-init functions are not yet supported by ESP IDF, so the best bet is to just restart.
    esp_restart();  // Software reset.
    return false;   // abort didn't work
}

bool CommsManager::IPWANSendRawPacketCompositeArray(uint8_t* raw_packets_buf) {
    if (!comms_manager.HasIP()) {
        // CONSOLE_WARNING(
        //     "CommsManager::IPWANSendRawPacketCompositeArray",
        //     "Can't push to WAN raw packet composite array queue if WiFi station is not running and Ethernet is "
        //     "disconnected.");
        return false;  // Task not started yet, queue not created yet. Pushing to queue would cause an abort.
    }

    UBaseType_t slots_used = uxQueueMessagesWaiting(ip_wan_reporting_composite_array_queue_);
    bool queue_half_full = (slots_used * 2 >= kReportingCompositeArrayQueueNumElements);

    // Opportunistically flush the scratch buffer when the queue has dropped below half capacity.
    if (composite_array_scratch_has_data_ && !queue_half_full) {
        if (xQueueSend(ip_wan_reporting_composite_array_queue_, composite_array_scratch_buf_,
                       pdMS_TO_TICKS(kWiFiSTAMessageQueueTimeoutMs)) == pdTRUE) {
            composite_array_scratch_has_data_ = false;
            slots_used++;
            queue_half_full = (slots_used * 2 >= kReportingCompositeArrayQueueNumElements);
        }
    }

    // When the queue is at least half full, merge incoming packets into the scratch buffer instead
    // of pushing a new slot, to reduce the rate at which the queue fills.
    if (queue_half_full) {
        if (composite_array_scratch_has_data_) {
            if (CompositeArray::MergeRawPacketsBuffers(composite_array_scratch_buf_, raw_packets_buf)) {
                return true;  // Merged into scratch; scratch will be flushed when queue has room.
            }
            // Scratch is too full to absorb the merge; push it and start fresh with incoming.
            int flush_err = xQueueSend(ip_wan_reporting_composite_array_queue_, composite_array_scratch_buf_,
                                       pdMS_TO_TICKS(kWiFiSTAMessageQueueTimeoutMs));
            if (flush_err == errQUEUE_FULL) {
                UBaseType_t slots_waiting = uxQueueMessagesWaiting(ip_wan_reporting_composite_array_queue_);
                auto* scratch_hdr = reinterpret_cast<CompositeArray::RawPackets::Header*>(composite_array_scratch_buf_);
                CONSOLE_WARNING("CommsManager::IPWANSendRawPacketCompositeArray",
                                "Overflowed WAN raw packet composite array queue (%u/%u slots used) while flushing "
                                "scratch buffer. Dropping: %u ModeS, %u UAT-ADS-B, %u UAT-Uplink.",
                                (unsigned)slots_waiting, kReportingCompositeArrayQueueNumElements,
                                scratch_hdr->num_mode_s_packets, scratch_hdr->num_uat_adsb_packets,
                                scratch_hdr->num_uat_uplink_packets);
                xQueueReset(ip_wan_reporting_composite_array_queue_);
            }
            composite_array_scratch_has_data_ = false;
        }
        // Save incoming into scratch buffer (either it was empty, or we just flushed it).
        memcpy(composite_array_scratch_buf_, raw_packets_buf, CompositeArray::RawPackets::kMaxLenBytes);
        composite_array_scratch_has_data_ = true;
        return true;
    }

    // Normal path: queue is below half capacity, push incoming directly.
    int err = xQueueSend(ip_wan_reporting_composite_array_queue_, raw_packets_buf,
                         pdMS_TO_TICKS(kWiFiSTAMessageQueueTimeoutMs));
    if (err == errQUEUE_FULL) {
        UBaseType_t slots_waiting = uxQueueMessagesWaiting(ip_wan_reporting_composite_array_queue_);
        auto* incoming_hdr = reinterpret_cast<CompositeArray::RawPackets::Header*>(raw_packets_buf);
        CONSOLE_WARNING("CommsManager::IPWANSendRawPacketCompositeArray",
                        "Overflowed WAN raw packet composite array queue (%u/%u slots used). "
                        "Rejected: %u ModeS, %u UAT-ADS-B, %u UAT-Uplink.",
                        (unsigned)slots_waiting, kReportingCompositeArrayQueueNumElements,
                        incoming_hdr->num_mode_s_packets, incoming_hdr->num_uat_adsb_packets,
                        incoming_hdr->num_uat_uplink_packets);
        xQueueReset(ip_wan_reporting_composite_array_queue_);
        return false;
    } else if (err != pdTRUE) {
        CONSOLE_WARNING("CommsManager::IPWANSendRawPacketCompositeArray",
                        "Pushing transponder packet to WAN queue resulted in error code %d.", err);
        return false;
    }
    return true;
}

bool CommsManager::WiFiAccessPointSendMessageToAllStations(NetworkMessage& message) {
    if (!wifi_ap_enabled || wifi_ap_message_queue_ == nullptr) {
        // The queue is created lazily in WiFiInit() only when the AP is enabled; on an Ethernet-only unit it is null.
        CONSOLE_WARNING("CommsManager::WiFiAccessPointSendMessageToAllStations",
                        "Can't push to WiFi AP message queue if AP is not running.");
        return false;  // Task not started yet, pushing to queue could create an overflow.
    }
    // If the queue is full, wait up to 10ms for it to have space.
    int err = xQueueSend(wifi_ap_message_queue_, &message, pdMS_TO_TICKS(kWiFiAPMessageQueueTimeout));
    if (err == errQUEUE_FULL) {
        CONSOLE_WARNING("CommsManager::WiFiAccessPointSendMessageToAllStations", "Overflowed WiFi AP message queue.");
        xQueueReset(wifi_ap_message_queue_);
        return false;
    } else if (err != pdTRUE) {
        CONSOLE_WARNING("CommsManager::WiFiAccessPointSendMessageToAllStations",
                        "Pushing message to WiFi AP message queue resulted in error code %d.", err);
        return false;
    }
    return true;
}