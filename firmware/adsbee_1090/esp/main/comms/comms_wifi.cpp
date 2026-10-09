#include <string.h>

#include <functional>  // for std::bind

#include "cc.h"  // For endiannness swapping.
#include "comms.hh"
#include "esp_event.h"
#include "esp_mac.h"
#include "hal.hh"
#include "heap_diagnostics.hh"
#include "lwip/dns.h"
#include "lwip/err.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "nvs_flash.h"
#include "task_priorities.hh"
#include "utils/task_utils.hh"  // For delayed reconnect callbacks.
#include "comms/wifi_network_selector.hh"

static const uint16_t kWiFiNumRetries = 3;
static const uint16_t kWiFiRetryWaitTimeMs = 100;
static const uint16_t kWiFiStaMaxNumReconnectAttempts = 5;
static const uint16_t kWiFiScanDefaultListSize = 20;
static const uint16_t kWiFiAPMessageQueueTimeout = 100;     // ms
static const uint16_t kWiFiSTAMessageQueueTimeoutMs = 100;  // ms
static const uint32_t kWANQueueOverflowLogIntervalMs = 10000;

// Counts packets dropped by WAN queue overflows; logs at most once per kWANQueueOverflowLogIntervalMs.
static void LogWANQueueOverflow(const uint8_t* raw_packets_buf) {
    static uint32_t num_overflows = 0, dropped_mode_s = 0, dropped_uat_adsb = 0, dropped_uat_uplink = 0;
    static uint32_t last_log_timestamp_ms = 0;
    static bool logged_once = false;
    auto* header = reinterpret_cast<const CompositeArray::RawPackets::Header*>(raw_packets_buf);
    num_overflows++;
    dropped_mode_s += header->num_mode_s_packets;
    dropped_uat_adsb += header->num_uat_adsb_packets;
    dropped_uat_uplink += header->num_uat_uplink_packets;
    uint32_t timestamp_ms = get_time_since_boot_ms();
    if (logged_once && timestamp_ms - last_log_timestamp_ms < kWANQueueOverflowLogIntervalMs) {
        return;
    }
    CONSOLE_WARNING("CommsManager::IPWANSendRawPacketCompositeArray",
                    "WAN raw packet queue overflowed %lu times in %lu ms, dropped at least %lu ModeS, %lu UAT-ADS-B, %lu "
                    "UAT-Uplink.",
                    num_overflows, logged_once ? timestamp_ms - last_log_timestamp_ms : 0, dropped_mode_s,
                    dropped_uat_adsb, dropped_uat_uplink);
    logged_once = true;
    last_log_timestamp_ms = timestamp_ms;
    num_overflows = dropped_mode_s = dropped_uat_adsb = dropped_uat_uplink = 0;
}

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// WPA2 needs 8-63 characters; an empty password means an open network. Stations also accept WEP keys (5 or 13).
static bool WiFiPasswordIsValid(const char* password, bool is_station) {
    size_t len = strnlen(password, SettingsManager::Settings::kWiFiPasswordMaxLen + 1);
    return len == 0 || (len >= 8 && len <= SettingsManager::Settings::kWiFiPasswordMaxLen) ||
           (is_station && (len == 5 || len == 13));
}

// Logs a failed WiFi driver call and returns false instead of aborting, so a bad stored setting can't boot loop the
// ESP32.
static bool WiFiCheck(esp_err_t err, const char* what) {
    if (err != ESP_OK) {
        CONSOLE_ERROR("CommsManager::WiFiInit", "%s failed: %s.", what, esp_err_to_name(err));
        return false;
    }
    return true;
}

/** "Pass-Through" functions used to access member functions in callbacks. **/
void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    comms_manager.WiFiEventHandler(arg, event_base, event_id, event_data);
}

void wifi_access_point_task(void* pvParameters) { comms_manager.WiFiAccessPointTask(pvParameters); }

// Station state is only touched on the default event loop task, so timer callbacks post an event instead of calling
// WiFiStationJoin() from the esp_timer task.
ESP_EVENT_DEFINE_BASE(ADSBEE_WIFI_EVENT);
enum ADSBeeWiFiEvent : int32_t { kADSBeeWiFiEventJoin = 0 };
inline void join_wifi(void* arg = nullptr) {
    if (esp_event_post(ADSBEE_WIFI_EVENT, kADSBeeWiFiEventJoin, nullptr, 0, 0) != ESP_OK) {
        CONSOLE_ERROR("join_wifi", "Failed to post a WiFi join event.");
    }
}
/** End "Pass-Through" functions. **/

void CommsManager::WiFiEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == ADSBEE_WIFI_EVENT) {
        if (event_id == kADSBeeWiFiEventJoin) WiFiStationJoin();
        return;
    }
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
            // Never log WiFi passwords: the console log reaches the web UI.
            WiFiStationJoin();
            // Note: wifi_sta_has_ip_ will get filled in by the IP event handler if an IP is issued.
            break;
        }
        case WIFI_EVENT_STA_DISCONNECTED: {
            // The ADSBee has disconnected from an external network.
            wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*)event_data;
            CONSOLE_ERROR("CommsManager::WiFiEventHandler",
                          "Disconnected from (or failed to connect to) ap SSID:%s - Disconnect reason : %d",
                          WiFiStationSSID(wifi_sta_network_index_), event->reason);
            // Without an IP the attempt failed (e.g. wrong password): try other stored networks first next time.
            if (!wifi_sta_has_ip_) wifi_sta_failed_mask_ |= 1u << wifi_sta_network_index_;
            wifi_sta_connected_ = false;
            wifi_sta_has_ip_ = false;
            if (wifi_sta_enabled) {
                ScheduleDelayedFunctionCall(kWiFiSTAReconnectIntervalMs, &join_wifi);
            }
            break;
        }
        case WIFI_EVENT_STA_CONNECTED:
            CONSOLE_INFO("CommsManager::WiFiInit", "Connected to ap SSID:%s", WiFiStationSSID(wifi_sta_network_index_));
            wifi_sta_connected_ = true;
            wifi_sta_connected_timestamp_ms_ = get_time_since_boot_ms();
            break;
        case WIFI_EVENT_SCAN_DONE:
            // Second half of WiFiStationJoin(): the scan it started has finished.
            if (wifi_sta_scan_in_progress_) {
                wifi_sta_scan_in_progress_ = false;
                WiFiStationConnect(WiFiStationPickNetwork());
            }
            break;
    }
}

const char* CommsManager::WiFiStationSSID(int index) {
    if (index == 0) return wifi_sta_ssid;
    if (index > 0 && index < SettingsManager::Settings::kWiFiSTAMaxNumNetworks) {
        return wifi_sta_extra_networks[index - 1].ssid;
    }
    return "";
}

const char* CommsManager::WiFiStationPassword(int index) {
    return index == 0 ? wifi_sta_password : wifi_sta_extra_networks[index - 1].password;
}

uint16_t CommsManager::WiFiStationUsableSSIDs(const char* ssids[]) {
    uint16_t num_usable = 0;
    for (int i = 0; i < SettingsManager::Settings::kWiFiSTAMaxNumNetworks; i++) {
        bool usable = WiFiStationSSID(i)[0] != '\0' && WiFiPasswordIsValid(WiFiStationPassword(i), true);
        ssids[i] = usable ? WiFiStationSSID(i) : "";
        num_usable += usable;
    }
    return num_usable;
}

int CommsManager::WiFiStationPickNetwork() {
    const char* ssids[SettingsManager::Settings::kWiFiSTAMaxNumNetworks];
    WiFiStationUsableSSIDs(ssids);

    // Static to keep ~2 kB off the event loop task's stack.
    static wifi_ap_record_t records[kWiFiScanDefaultListSize];
    static WiFiNetworkSelector::ScanResult results[kWiFiScanDefaultListSize];
    uint16_t num_records = kWiFiScanDefaultListSize;
    if (esp_wifi_scan_get_ap_records(&num_records, records) != ESP_OK) num_records = 0;
    CONSOLE_INFO("CommsManager::WiFiStationPickNetwork", "WiFi scan done, %u access points seen.", num_records);
    for (uint16_t i = 0; i < num_records; i++) {
        results[i] = {.ssid = (const char*)records[i].ssid, .rssi_dbm = records[i].rssi};
    }

    return WiFiNetworkSelector::Pick(ssids, SettingsManager::Settings::kWiFiSTAMaxNumNetworks, results, num_records,
                                     wifi_sta_failed_mask_, wifi_sta_network_index_);
}

void CommsManager::WiFiStationConnect(int index) {
    if (index == WiFiNetworkSelector::kNone) return;
    wifi_sta_network_index_ = index;
    wifi_config_t wifi_config_sta = {};
    strncpy((char*)(wifi_config_sta.sta.ssid), WiFiStationSSID(index), sizeof(wifi_config_sta.sta.ssid));
    strncpy((char*)(wifi_config_sta.sta.password), WiFiStationPassword(index), sizeof(wifi_config_sta.sta.password));
    // Never log WiFi passwords: the console log reaches the web UI.
    CONSOLE_INFO("CommsManager::WiFiStationConnect", "Joining SSID:%s", WiFiStationSSID(index));
    if (!WiFiCheck(esp_wifi_set_config(WIFI_IF_STA, &wifi_config_sta), "WiFi station config") ||
        !WiFiCheck(esp_wifi_connect(), "WiFi station connect")) {
        ScheduleDelayedFunctionCall(kWiFiSTAReconnectIntervalMs, &join_wifi);
    }
}

// Runs on the event loop task (STA_START, or a delayed join_wifi() after a disconnect). With several stored networks
// it only starts a non-blocking scan; WIFI_EVENT_SCAN_DONE then picks a network and connects.
void CommsManager::WiFiStationJoin() {
    if (wifi_sta_scan_in_progress_ || wifi_sta_connected_) return;  // A scan is pending, or a stale delayed join.
    const char* ssids[SettingsManager::Settings::kWiFiSTAMaxNumNetworks];
    if (WiFiStationUsableSSIDs(ssids) <= 1) {
        // One stored network: join it directly, which also finds hidden SSIDs.
        WiFiStationConnect(WiFiNetworkSelector::Next(ssids, SettingsManager::Settings::kWiFiSTAMaxNumNetworks, -1));
        return;
    }
    esp_err_t err = esp_wifi_scan_start(nullptr, false);
    if (err == ESP_OK) {
        wifi_sta_scan_in_progress_ = true;
        return;
    }
    // E.g. the Remote ID sniffer holds the radio: skip the scan and try the stored networks in turn.
    CONSOLE_WARNING("CommsManager::WiFiStationJoin", "WiFi scan failed (%s), trying the next stored network.",
                    esp_err_to_name(err));
    WiFiStationConnect(WiFiNetworkSelector::Next(ssids, SettingsManager::Settings::kWiFiSTAMaxNumNetworks,
                                                 wifi_sta_network_index_, wifi_sta_failed_mask_));
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
    // Locals so a rejected interface doesn't change the settings comparison in SettingsManager::Apply().
    bool ap_enabled = wifi_ap_enabled, sta_enabled = wifi_sta_enabled;
    if (ap_enabled && !WiFiPasswordIsValid(wifi_ap_password, false)) {
        CONSOLE_ERROR("CommsManager::WiFiInit", "WiFi AP disabled: password must be 8-63 characters.");
        ap_enabled = false;
    }
    if (sta_enabled) {
        const char* ssids[SettingsManager::Settings::kWiFiSTAMaxNumNetworks];
        WiFiStationUsableSSIDs(ssids);
        for (int i = 0; i < SettingsManager::Settings::kWiFiSTAMaxNumNetworks; i++) {
            if (WiFiStationSSID(i)[0] != '\0' && ssids[i][0] == '\0') {
                CONSOLE_ERROR("CommsManager::WiFiInit",
                              "Skipping WiFi network %d (%s): password must be 8-63 characters or a 5 or 13 character "
                              "WEP key.",
                              i + 1, WiFiStationSSID(i));
            }
        }
        if (WiFiStationUsableSSIDs(ssids) == 0) {
            CONSOLE_ERROR("CommsManager::WiFiInit", "WiFi station disabled: no usable network stored.");
            sta_enabled = false;
        }
    }

    esp_netif_t* wifi_ap_netif_ = esp_netif_create_default_wifi_ap();
    assert(wifi_ap_netif_);
    esp_netif_t* wifi_sta_netif_ = esp_netif_create_default_wifi_sta();
    assert(wifi_sta_netif_);

    ESP_ERROR_CHECK(esp_netif_set_hostname(wifi_sta_netif_, hostname));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    HeapDiagnostics::Mark("netifs");
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    HeapDiagnostics::Mark("wifi_init");

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(ADSBEE_WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    if (!ip_event_handler_was_initialized_) {
        IPInit();
    }

    wifi_was_initialized_ = true;

    if (ap_enabled) {
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

        // The mode must include the AP before its config can be set.
        ap_enabled = WiFiCheck(esp_wifi_set_mode(sta_enabled ? WIFI_MODE_APSTA : WIFI_MODE_AP), "Setting WiFi mode") &&
                     WiFiCheck(esp_wifi_set_config(WIFI_IF_AP, &wifi_config_ap), "WiFi AP config");
    }

    if (sta_enabled) {
        // The station config is set per network in WiFiStationConnect().
        sta_enabled = WiFiCheck(esp_wifi_set_mode(ap_enabled ? WIFI_MODE_APSTA : WIFI_MODE_STA), "Setting WiFi mode");
    }

    if (!ap_enabled && !sta_enabled) {
        esp_wifi_stop();
        CONSOLE_INFO("CommsManager::WiFiInit", "WiFi disabled.");
        return true;
    }

    // Drop an interface whose config was rejected above.
    if (!WiFiCheck(esp_wifi_set_mode(ap_enabled && sta_enabled ? WIFI_MODE_APSTA
                                     : ap_enabled              ? WIFI_MODE_AP
                                                               : WIFI_MODE_STA),
                   "Setting WiFi mode") ||
        !WiFiCheck(esp_wifi_start(), "Starting WiFi")) {
        esp_wifi_stop();
        return false;
    }
    HeapDiagnostics::Mark("wifi_start");

    wifi_ap_running_ = ap_enabled;
    if (ap_enabled) {
        CONSOLE_INFO("CommsManager::WiFiInit", "WiFi AP started. SSID:%s", wifi_ap_ssid);
        // Lazily create the AP broadcast queue now that the AP is actually enabled (see the CommsManager constructor).
        if (wifi_ap_message_queue_ == nullptr) {
            wifi_ap_message_queue_ = xQueueCreate(kWiFiMessageQueueLen, sizeof(NetworkMessage));
        }
        xTaskCreate(wifi_access_point_task, "wifi_ap_task", kWiFiAPTaskStackSizeBytes, &wifi_ap_task_handle, kWiFiAPTaskPriority, NULL);
    }
    if (sta_enabled) {
        CONSOLE_INFO("CommsManager::WiFiInit", "WiFi Station started.");
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
                LogWANQueueOverflow(composite_array_scratch_buf_);
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
        LogWANQueueOverflow(raw_packets_buf);
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