#include "comms.hh"

#include "spi_coprocessor.hh"

bool CommsManager::Init() {
    // Initialize Non Volatile Storage Flash, used by WiFi library.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (esp_netif_init() != ESP_OK) {
        CONSOLE_ERROR("CommsManager::Init", "Failed to initialize esp_netif.");
        return false;
    }
    if (esp_event_loop_create_default() != ESP_OK) {
        CONSOLE_ERROR("CommsManager::Init", "Failed to create default event loop.");
        return false;
    }
    return true;
}

ObjectDictionary::ESP32NetworkInfo CommsManager::GetNetworkInfo() {
    ObjectDictionary::ESP32NetworkInfo network_info;

    // Ethernet network info.
    network_info.ethernet_enabled = ethernet_enabled;
    network_info.ethernet_has_ip = ethernet_has_ip_;
    memcpy(network_info.ethernet_ip, ethernet_ip, SettingsManager::Settings::kIPAddrStrLen + 1);
    memcpy(network_info.ethernet_netmask, ethernet_netmask, SettingsManager::Settings::kIPAddrStrLen + 1);
    memcpy(network_info.ethernet_gateway, ethernet_gateway, SettingsManager::Settings::kIPAddrStrLen + 1);

    // WiFi station network info.
    network_info.wifi_sta_enabled = wifi_sta_enabled;
    memcpy(network_info.wifi_sta_ssid, WiFiStationSSID(wifi_sta_network_index_), SettingsManager::Settings::kWiFiSSIDMaxLen + 1);
    network_info.wifi_sta_has_ip = wifi_sta_has_ip_;
    memcpy(network_info.wifi_sta_ip, wifi_sta_ip, SettingsManager::Settings::kIPAddrStrLen + 1);
    memcpy(network_info.wifi_sta_netmask, wifi_sta_netmask, SettingsManager::Settings::kIPAddrStrLen + 1);
    memcpy(network_info.wifi_sta_gateway, wifi_sta_gateway, SettingsManager::Settings::kIPAddrStrLen + 1);

    // WiFi access point network info.
    network_info.wifi_ap_enabled = wifi_ap_enabled;
    network_info.wifi_ap_num_clients = num_wifi_clients_;
    for (uint16_t i = 0; i < SettingsManager::Settings::kWiFiMaxNumClients; i++) {
        // Turn client IP address into a string.
        esp_ip4_addr_t ip = wifi_clients_list_[i].ip;
        snprintf(network_info.wifi_ap_client_ips[i], SettingsManager::Settings::kIPAddrStrLen, IPSTR, IP2STR(&ip));
        network_info.wifi_ap_client_ips[i][SettingsManager::Settings::kIPAddrStrLen] = '\0';

        // Turn client MAC address into a string.
        uint8_t mac[SettingsManager::Settings::kMACAddrNumBytes];
        wifi_clients_list_[i].GetMAC(mac);
        snprintf(network_info.wifi_ap_client_macs[i], SettingsManager::Settings::kMACAddrStrLen, MACSTR, MAC2STR(mac));
        network_info.wifi_ap_client_macs[i][SettingsManager::Settings::kMACAddrStrLen] = '\0';
    }

    return network_info;
}

bool CommsManager::LogMessageToCoprocessor(SettingsManager::LogLevel log_level, const char* tag, const char* format,
                                           ...) {
    if (log_level > settings_manager.settings.log_level) {
        return true;  // Skip logging messages that aren't necessary.
    }
    va_list args;
    va_start(args, format);
    bool ret = pico.LogMessage(log_level, tag, format, args);
    va_end(args);
    return ret;
}

void CommsManager::WiFiAddClient(esp_ip4_addr_t client_ip, uint8_t* client_mac) {
    uint64_t mac_uint64 = MACToUint64(client_mac);
    int16_t new_client_index = -1;
    xSemaphoreTake(wifi_clients_list_mutex_, portMAX_DELAY);
    // Check for an existing active entry with the same MAC (e.g. DHCP lease renewal fires
    // IP_EVENT_AP_STAIPASSIGNED again without a preceding disconnect event). Update the IP
    // in place rather than adding a duplicate entry that would cause every message to be
    // sent multiple times.
    for (int i = 0; i < SettingsManager::Settings::kWiFiMaxNumClients; i++) {
        if (wifi_clients_list_[i].active && wifi_clients_list_[i].mac == mac_uint64) {
            wifi_clients_list_[i].ip = client_ip;
            new_client_index = i;
            break;
        }
    }
    if (new_client_index < 0) {
        for (int i = 0; i < SettingsManager::Settings::kWiFiMaxNumClients; i++) {
            if (!wifi_clients_list_[i].active) {
                wifi_clients_list_[i].ip = client_ip;
                wifi_clients_list_[i].SetMAC(client_mac);
                wifi_clients_list_[i].active = true;
                num_wifi_clients_++;
                new_client_index = i;
                break;
            }
        }
    }
    xSemaphoreGive(wifi_clients_list_mutex_);
    if (new_client_index < 0) {
        CONSOLE_ERROR("CommsManager::WiFiAddClient", "No space to add new client to list.");
    }
}

void CommsManager::WiFiRemoveClient(uint8_t* mac_buf_in) {
    uint64_t client_mac = MACToUint64(mac_buf_in);
    xSemaphoreTake(wifi_clients_list_mutex_, portMAX_DELAY);
    for (int i = 0; i < SettingsManager::Settings::kWiFiMaxNumClients; i++) {
        if (wifi_clients_list_[i].active && (wifi_clients_list_[i].mac == client_mac)) {
            wifi_clients_list_[i].active = false;
            num_wifi_clients_--;
            break;
        }
    }
    xSemaphoreGive(wifi_clients_list_mutex_);
}