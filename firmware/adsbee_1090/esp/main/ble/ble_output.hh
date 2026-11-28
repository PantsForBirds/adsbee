#pragma once

#include <atomic>
#include <cstdint>

#include "ble_output_buffers.hh"
#include "ble_output_status.hh"
#include "remote_id_guard.hh"
#include "settings.hh"

/**
 * Bluetooth LE output (AT+BLE_OUTPUT, docs/guides/bluetooth-output.md): GDL90 or FLARM NMEA to one client over the
 * Nordic UART Service and the HM-10 service (FFE0/FFE1), on the NimBLE host shared with Remote ID (BLEHost). Without
 * PSRAM it runs only while BLEOutputAllowed() and the Remote ID heap reserve allow.
 */
class BLEOutput {
   public:
    // Bytes queued for notifications. A 1 Hz report of 100 aircraft is about 3 kB; 200 is the ESP32 dictionary size.
    // Without PSRAM the RAM is too tight for that, and a larger report goes out in turns (ReportRotation).
    static constexpr uint16_t kStreamCapacityBytes = 6 * 1024;
    static constexpr uint16_t kStreamCapacityBytesNoPSRAM = 3 * 1024;
    // Notifications are sent only while this many NimBLE buffers stay free. The host queues notifications the
    // controller can't take yet in the same buffers that HCI commands and Remote ID advertising data need.
    static constexpr int kMinFreeNimBLEBuffers = 4;
    static constexpr uint32_t kAdvRetryIntervalMs = 1000;
    static constexpr uint32_t kMinHeapFreeBytesToStart = 60 * 1024;  // With PSRAM.
    static constexpr uint32_t kBlockedRetryIntervalMs = 10 * 1000;

    /**
     * Registers the GATT services with NimBLE. Called by BLEHost each time the host starts.
     */
    static bool RegisterGATTServices();

    /**
     * Requests a reconcile with the current settings and network state, done in the next Update().
     */
    void Apply() { apply_pending_ = true; }  // Any task.

    /**
     * Main task: reconciles with the settings, watches the heap and sends queued data.
     */
    void Update();

    /**
     * True while a client is subscribed to the notifications.
     */
    bool HasSubscriber() const;

    /**
     * True while a client is subscribed and the output runs in this mode.
     */
    bool StreamsMode(SettingsManager::BLEOutputMode mode) const { return running_mode_ == mode && HasSubscriber(); }

    /**
     * Starts a 1 Hz report: drops what the last report couldn't send yet, so the heartbeat leads and traffic stays
     * current when the link is slower than the report.
     */
    void BeginReport();

    /**
     * Queues framed GDL90 messages or NMEA sentences for the subscribed client.
     * @param[in] droppable True for FIS-B uplinks, which are refused before traffic is.
     * @retval False if the client isn't subscribed or the queue is full.
     */
    bool Send(const uint8_t* buf, uint16_t len, bool droppable = false);

    /**
     * Traffic of a 1 Hz report, one framed message per aircraft. Send indices TrafficStart()..n-1, then
     * 0..TrafficStart()-1, then call EndTraffic(n).
     */
    uint16_t TrafficStart() const { return rotation_.Start(); }
    void SendTraffic(const uint8_t* buf, uint16_t len, uint16_t index);
    void EndTraffic(uint16_t num_aircraft);

    /**
     * The FLARM NMEA report's traffic list, allocated with the output's queue: null unless the output runs in FLARM
     * NMEA mode.
     */
    FLARMTrafficList* FLARMTraffic() const;

    BLEOutputStatus GetStatus() const { return status_; }

   private:
    void Reconcile();
    bool Start(SettingsManager::BLEOutputMode mode);
    void Stop();

    bool running() const { return running_mode_ != SettingsManager::kBLEOutputOff; }

    std::atomic<bool> apply_pending_ = true;
    SettingsManager::BLEOutputMode running_mode_ = SettingsManager::kBLEOutputOff;  // Off while stopped.
    BLEOutputStatus status_ = kBLEOutputStatusOff;
    RemoteIDGuard::HeapWatch heap_watch_;
    ReportRotation rotation_;
    uint32_t last_heap_watch_ms_ = 0;
    uint32_t last_blocked_retry_ms_ = 0;
    uint32_t last_adv_retry_ms_ = 0;
    bool alloc_failure_logged_ = false;
};

extern BLEOutput ble_output;
