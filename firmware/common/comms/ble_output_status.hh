#pragma once

#include <cstdint>

// Live state of the Bluetooth LE output (AT+BLE_OUTPUT) on the ESP32, reported to the RP2040 in
// ESP32DeviceStatus::ble_output_status.
enum BLEOutputStatus : uint8_t {
    kBLEOutputStatusOff = 0,          // Turned off in the settings.
    kBLEOutputStatusNotInBuild,       // Firmware built without Bluetooth.
    kBLEOutputStatusBlockedByWiFi,    // No PSRAM: needs WiFi access point and station off.
    kBLEOutputStatusBlockedByRAM,     // Not enough free RAM; retried every few seconds.
    kBLEOutputStatusAdvertising,      // Waiting for a client.
    kBLEOutputStatusConnected,        // Client connected, not subscribed yet.
    kBLEOutputStatusStreaming,        // Client subscribed to the notifications.
    kBLEOutputStatusBlockedByRemoteID,  // No PSRAM: needs Remote ID over Bluetooth off.
    kBLEOutputStatusNumValues
};

inline const char* BLEOutputStatusStr(uint8_t status) {
    static const char* const kNames[kBLEOutputStatusNumValues] = {
        "OFF",       "NOT_IN_BUILD", "BLOCKED_BY_WIFI", "BLOCKED_BY_RAM", "ADVERTISING",
        "CONNECTED", "STREAMING",    "BLOCKED_BY_REMOTE_ID"};
    return status < kBLEOutputStatusNumValues ? kNames[status] : "UNKNOWN";
}

/**
 * Whether the network and Remote ID settings leave room for the Bluetooth output. Without PSRAM the internal RAM holds
 * neither WiFi access point or station nor Remote ID over Bluetooth next to it (a connected client would push both
 * under the heap reserve). Remote ID keeps the radio then.
 * @param[in] remote_id_uses_bluetooth Remote ID receive or transmit is on with a Bluetooth transport.
 * @retval kBLEOutputStatusAdvertising if it may run, else the blocking status.
 */
inline BLEOutputStatus BLEOutputAllowed(bool has_psram, bool wifi_ap_enabled, bool wifi_sta_enabled,
                                        bool remote_id_uses_bluetooth) {
    if (has_psram) return kBLEOutputStatusAdvertising;
    if (wifi_ap_enabled || wifi_sta_enabled) return kBLEOutputStatusBlockedByWiFi;
    if (remote_id_uses_bluetooth) return kBLEOutputStatusBlockedByRemoteID;
    return kBLEOutputStatusAdvertising;
}
