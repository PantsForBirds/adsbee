#pragma once

#include <atomic>
#include <cstdint>

#include "ble_output_status.hh"
#include "remote_id_guard.hh"

/**
 * Bluetooth LE output (AT+BLE_OUTPUT): a GATT server with the two generic serial profiles EFB apps read, the Nordic
 * UART Service (NUS) and the HM-10 / TI CC254x service (FFE0/FFE1). The client subscribes to the NUS TX or the FFE1
 * characteristic; writes to either service are accepted and ignored.
 *
 * Mode GDL90 streams the GDL90 the WiFi access point broadcasts on UDP port 4000 (heartbeat, ownship, traffic at 1 Hz,
 * FIS-B uplinks as they arrive), framed as on UDP (0x7E flags, byte stuffing, CRC). Each notification carries whole
 * messages unless one message alone exceeds the payload size.
 *
 * One client at a time. The server shares the NimBLE host with Remote ID (BLEHost) and runs under the same network
 * and RAM rules as Remote ID over Bluetooth (RemoteIDGuard::AllowedTransports and the heap reserve).
 */
class BLEOutput {
   public:
    // Bytes queued for notifications. A 1 Hz report of 100 aircraft is about 3 kB.
    static constexpr uint16_t kStreamCapacityBytes = 6 * 1024;
    // Notifications are sent only while this many NimBLE buffers stay free. The host queues notifications the
    // controller can't take yet in the same buffers that HCI commands and Remote ID advertising data need.
    static constexpr int kMinFreeNimBLEBuffers = 4;
    static constexpr uint32_t kAdvRetryIntervalMs = 1000;
    static constexpr uint32_t kMinHeapFreeBytesToStart = 60 * 1024;
    static constexpr uint32_t kBLEHeapCostBytes = 48 * 1024;
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
     * Starts a 1 Hz report: drops what the last report couldn't send yet, so the heartbeat leads and traffic stays
     * current when the link is slower than the report.
     */
    void BeginReport();

    /**
     * Queues framed GDL90 messages for the subscribed client.
     * @param[in] droppable True for FIS-B uplinks, which are refused before traffic is.
     * @retval False if the client isn't subscribed or the queue is full.
     */
    bool Send(const uint8_t* buf, uint16_t len, bool droppable = false);

    BLEOutputStatus GetStatus() const { return status_; }

   private:
    void Reconcile();
    bool Start();
    void Stop();

    std::atomic<bool> apply_pending_ = true;
    bool running_ = false;
    BLEOutputStatus status_ = kBLEOutputStatusOff;
    RemoteIDGuard::HeapWatch heap_watch_;
    uint32_t last_heap_watch_ms_ = 0;
    uint32_t last_blocked_retry_ms_ = 0;
    uint32_t last_adv_retry_ms_ = 0;
};

extern BLEOutput ble_output;
