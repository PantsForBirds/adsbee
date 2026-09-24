#include "settings.hh"

#include "comms.hh"
#include "remote_id/remote_id_manager.hh"

bool SettingsManager::Apply() {
    bool ethernet_restart_required = false;
    bool wifi_sta_restart_required = false;
    bool wifi_ap_restart_required = false;

    // Check if the hostname has changed.
    if (strcmp(comms_manager.hostname, settings.core_network_settings.hostname) != 0) {
        // Restart all network interfaces if the hostname has changed.
        ethernet_restart_required = true;
        wifi_sta_restart_required = true;
        wifi_ap_restart_required = true;
    }

    // Apply the hostname.
    strncpy(comms_manager.hostname, settings.core_network_settings.hostname, Settings::kHostnameMaxLen);
    comms_manager.hostname[Settings::kHostnameMaxLen] = '\0';

    // Check if Ethernet settings have changed.
    if (comms_manager.ethernet_enabled != settings.core_network_settings.ethernet_enabled) {
        ethernet_restart_required = true;
    }

    // Apply the Ethernet settings.
    comms_manager.ethernet_enabled = settings.core_network_settings.ethernet_enabled;

    // Check if the AP settings have changed.
    if (comms_manager.wifi_ap_enabled != settings.core_network_settings.wifi_ap_enabled ||
        (comms_manager.wifi_ap_enabled &&
         (comms_manager.wifi_ap_channel != settings.core_network_settings.wifi_ap_channel ||
          strcmp(comms_manager.wifi_ap_ssid, settings.core_network_settings.wifi_ap_ssid) != 0 ||
          strcmp(comms_manager.wifi_ap_password, settings.core_network_settings.wifi_ap_password) != 0))) {
        wifi_ap_restart_required = true;
    }
    // Apply the AP settings.
    comms_manager.wifi_ap_enabled = settings.core_network_settings.wifi_ap_enabled;
    comms_manager.wifi_ap_channel = settings.core_network_settings.wifi_ap_channel;
    strncpy(comms_manager.wifi_ap_ssid, settings.core_network_settings.wifi_ap_ssid,
            SettingsManager::Settings::kWiFiSSIDMaxLen + 1);
    strncpy(comms_manager.wifi_ap_password, settings.core_network_settings.wifi_ap_password,
            SettingsManager::Settings::kWiFiPasswordMaxLen + 1);

    // Check if the Station settings have changed.
    if (comms_manager.wifi_sta_enabled != settings.core_network_settings.wifi_sta_enabled ||
        (comms_manager.wifi_sta_enabled &&
         (strcmp(comms_manager.wifi_sta_ssid, settings.core_network_settings.wifi_sta_ssid) != 0 ||
          strcmp(comms_manager.wifi_sta_password, settings.core_network_settings.wifi_sta_password) != 0))) {
        wifi_sta_restart_required = true;
    }

    // Apply the Station settings.
    comms_manager.wifi_sta_enabled = settings.core_network_settings.wifi_sta_enabled;
    strncpy(comms_manager.wifi_sta_ssid, settings.core_network_settings.wifi_sta_ssid,
            SettingsManager::Settings::kWiFiSSIDMaxLen + 1);
    strncpy(comms_manager.wifi_sta_password, settings.core_network_settings.wifi_sta_password,
            SettingsManager::Settings::kWiFiPasswordMaxLen + 1);

    // Restart network interfaces if necessary.
    if (ethernet_restart_required) {
        if (!comms_manager.EthernetDeInit()) {
            CONSOLE_ERROR("SettingsManager::Apply", "Failed to deinitialize Ethernet.");
            return false;
        }
        if (settings.core_network_settings.ethernet_enabled && !comms_manager.EthernetInit()) {
            CONSOLE_ERROR("SettingsManager::Apply", "Failed to initialize Ethernet.");
            return false;
        }
    }

    if (wifi_ap_restart_required || wifi_sta_restart_required) {
        if (!comms_manager.WiFiDeInit()) {
            CONSOLE_ERROR("SettingsManager::Apply", "Failed to deinitialize WiFi.");
            return false;
        }
        if ((settings.core_network_settings.wifi_ap_enabled || settings.core_network_settings.wifi_sta_enabled) &&
            !comms_manager.WiFiInit()) {
            CONSOLE_ERROR("SettingsManager::Apply", "Failed to initialize WiFi.");
            return false;
        }
    }

    // (Re)configure Broadcast Remote ID reception now that the WiFi/Ethernet state is settled. RemoteIDManager decides
    // which transports can actually run given the build (PSRAM vs not), the WiFi/Ethernet state, and free heap.
    remote_id_manager.Apply();

    return true;
}