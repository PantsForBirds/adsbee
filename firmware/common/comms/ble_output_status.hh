#pragma once

#include <cstdint>

// Live state of the Bluetooth LE output (AT+BLE_OUTPUT) on the ESP32, reported to the RP2040 in ESP32DeviceStatus::ble_output_status.
enum BLEOutputStatus : uint8_t {
    kBLEOutputStatusOff = 0,          // Turned off in the settings.
    kBLEOutputStatusNotInBuild,       // Firmware built without Bluetooth.
    kBLEOutputStatusBlockedByWiFi,    // No PSRAM: needs WiFi access point and station off.
    kBLEOutputStatusBlockedByRAM,     // Not enough free RAM; retried every few seconds.
    kBLEOutputStatusAdvertising,      // Waiting for a client.
    kBLEOutputStatusConnected,        // Client connected, not subscribed yet.
    kBLEOutputStatusStreaming,        // Client subscribed to the GDL90 notifications.
    kBLEOutputStatusNumValues
};

inline const char* BLEOutputStatusStr(uint8_t status) {
    static const char* const kNames[kBLEOutputStatusNumValues] = {
        "OFF", "NOT_IN_BUILD", "BLOCKED_BY_WIFI", "BLOCKED_BY_RAM", "ADVERTISING", "CONNECTED", "STREAMING"};
    return status < kBLEOutputStatusNumValues ? kNames[status] : "UNKNOWN";
}
