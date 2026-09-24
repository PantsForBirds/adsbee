#pragma once

#include <cstdint>

#include "data_structures.hh"  // PFBQueue
#include "remote_id_packet.hh"
#include "remote_id_tx.hh"
#include "settings.hh"

/**
 * RemoteIDManager orchestrates Broadcast Remote ID (ASTM F3411) reception on the ESP32-S3.
 *
 * It decides which transports can actually run given (a) the user's settings, (b) the hardware build (PSRAM lets Remote
 * ID coexist with WiFi AP/STA; without PSRAM, RAM only allows Remote ID when WiFi is off and Ethernet carries IP), and
 * (c) live heap headroom. It brings up the BLE (NimBLE) observer and, on capable configurations, the WiFi promiscuous
 * sniffer; both feed a single de-duplication / rate-limiting stage before packets are ingested locally and forwarded to
 * the RP2040.
 *
 * Wiring:
 *   - Received advertisements are parsed to raw ODID bytes and handed to OnRawRemoteIDPacket() (from the NimBLE host
 *     task or the WiFi promiscuous callback).
 *   - OnRawRemoteIDPacket() always ingests into the ESP32's local AircraftDictionary (drives network output) and, after
 *     rate-limiting, enqueues a copy into the out-queue (GetOutQueue()) for the RP2040 to pull.
 *
 * Buffers: the ingest and out queues are allocated lazily (heap) only once a transport actually starts, so a build
 * where Remote ID is disabled or Bluetooth isn't compiled in pays zero internal SRAM for them. They are not freed at
 * runtime: the out-queue is read from the SPI task (ObjectDictionary::GetBytes) while the main task services it, so
 * freeing them on disable would race those consumers; the RAM stays claimed only after Remote ID is enabled at least
 * once (rare), which is an acceptable trade for avoiding a use-after-free.
 *   - Apply() is called from SettingsManager::Apply() (esp/main/settings.cpp) after the WiFi/Ethernet interfaces are
 *     (re)configured, so it sees the resolved network state.
 *   - GetStatus() feeds ObjectDictionary::ESP32DeviceStatus::remote_id_status so the RP2040 can explain the live state.
 */
class RemoteIDManager {
   public:
    // Per-transport heap guards: Remote ID is a best-effort add-on and must never starve the network stack (whose
    // back-pressure trips around 20 KB, see comms_ip.cpp::safe_send). BLE is the priority transport; the WiFi sniffer is
    // best-effort and only starts if enough internal SRAM remains after the (heavier) BLE stack is up. Tune against the
    // 1 Hz heap_free_bytes telemetry.
    static constexpr uint32_t kMinHeapFreeBytesForBLE = 70 * 1024;
    static constexpr uint32_t kMinHeapFreeBytesForWiFiSniffer = 55 * 1024;

    // Packet queue depths (lazily heap-allocated; see class comment). Kept small: after per-source rate limiting the
    // sustained rate is ~1 Hz/drone, and both queues are drained every main-loop / SPI iteration.
    static constexpr uint16_t kIngestQueueDepth = 8;  // BLE + WiFi producers -> main task.
    static constexpr uint16_t kOutQueueDepth = 8;     // main task -> RP2040 (SPI) pull.

    // Per-source rate limiting for SPI forwarding to the RP2040. Local dictionary ingest is not rate limited.
    static constexpr uint32_t kLocationForwardIntervalMs = 1000;  // Location/Vector: at most 1 Hz per source.
    static constexpr uint32_t kStaticForwardIntervalMs = 10000;   // Static messages: at most every 10 s per source.
    static constexpr uint16_t kDedupTableNumEntries = 24;         // Tracked simultaneous transmitters for rate limiting.

    // WiFi sniffer channel hop set (US: 1/6/11 carry virtually all WiFi Remote ID beacons).
    static constexpr uint8_t kSnifferChannels[3] = {1, 6, 11};
    static constexpr uint32_t kSnifferChannelDwellMs = 450;  // >= 4 beacon intervals per channel.

    // Transmit tick period. ASTM F3411 requires the dynamic Location message at >= 1 Hz; refreshing the advertised
    // content every 500 ms keeps Location above that even on the BT4 legacy path, which can only carry one message per
    // advertisement and therefore interleaves Location with the static identity messages.
    static constexpr uint32_t kTxTickIntervalMs = 500;

    // Heap guard for the transmitter. Transmitting needs far less RAM than receiving (no packet queues, no promiscuous
    // RX buffers), but the BLE/WiFi stacks it brings up are the same, so it still must not starve the network stack.
    static constexpr uint32_t kMinHeapFreeBytesForTx = 60 * 1024;

    // Live status bitfield, mirrored into ObjectDictionary::ESP32DeviceStatus::remote_id_status. Low byte is receive
    // state, high byte is transmit state.
    enum Status : uint16_t {
        kStatusBLEActive = 1 << 0,          // NimBLE observer is scanning.
        kStatusBLECodedPHYActive = 1 << 1,  // Coded PHY (BT5 Long Range) extended scan is active.
        kStatusWiFiSnifferActive = 1 << 2,  // WiFi promiscuous sniffer is running.
        kStatusBlockedByWiFi = 1 << 3,      // Requested but blocked: WiFi AP/STA up on a non-PSRAM build.
        kStatusBlockedByRAM = 1 << 4,       // Requested but blocked: insufficient free heap.
        kStatusNotInBuild = 1 << 5,         // Requested but Bluetooth is not compiled into this firmware.

        kStatusTxBLELegacyActive = 1 << 8,   // Advertising Remote ID on BT4 legacy (1M PHY).
        kStatusTxBLECodedActive = 1 << 9,    // Advertising Remote ID on BT5 Long Range (Coded PHY).
        kStatusTxWiFiBeaconActive = 1 << 10, // Injecting Remote ID WiFi beacon frames.
        kStatusTxNoPosition = 1 << 11,       // Transmitting, but rx_position is unavailable (Location sent as unknown).
        kStatusTxBlocked = 1 << 12,          // Transmit requested but could not start (RAM, radio conflict, or build).
        // Transmitting, but the configured rx_position source is one we may never broadcast as our own (an
        // aircraft-derived position, or none at all) -- see SettingsManager::RxPosition::MayBeTransmittedAsOwnPosition.
        // Always accompanied by kStatusTxNoPosition; this bit says the fix is a setting to change, not a wait.
        kStatusTxPositionSourceNotAllowed = 1 << 13,
    };

    RemoteIDManager() = default;

    /**
     * (Re)configures Remote ID reception from the current settings and network state. Idempotent: brings transports up
     * or down to match. Called from SettingsManager::Apply() and safe to call repeatedly.
     */
    void Apply();

    /**
     * Per-iteration work: services the WiFi channel hopper (when the sniffer is active) and any deferred bookkeeping.
     * Called from ADSBeeServer::Update().
     */
    void Update();

    /**
     * Ingests one raw Remote ID advertisement (already stripped to ODID bytes + transport metadata). Called from the
     * NimBLE host task and the WiFi promiscuous callback. Thread-safe with respect to Update()/Apply() via the queue and
     * the fact that dictionary ingest happens on the main task (see below).
     */
    void OnRawRemoteIDPacket(const RawRemoteIDPacket& packet);

    /**
     * Drains queued advertisements on the main task: ingests each into the local AircraftDictionary and forwards
     * rate-limited copies to the RP2040. Called from ADSBeeServer::Update() so all dictionary mutation stays on one
     * task. Returns the number of packets processed.
     */
    uint16_t ServiceIngestQueue();

    uint16_t GetStatus() const { return status_; }

    /**
     * Returns the queue of rate-limited Remote ID packets waiting to be pulled by the RP2040 over SPI, or nullptr if no
     * transport has started (queues not allocated). ObjectDictionary's ESP32 read/status paths pass this straight to
     * CompositeArray::PackRawPacketsBuffer / CalculateRawPacketsBufferLength, which already treat nullptr as "empty".
     */
    PFBQueue<RawRemoteIDPacket>* GetOutQueue() { return out_queue_; }

    /**
     * The single Remote ID transmitter (message content builder) shared by all transmit transports, so they advance one
     * common message schedule. Used by the BLE advertising callbacks, which run outside a member context.
     */
    RemoteIDTransmitter& GetTransmitter() { return tx_; }

   private:
    // Decides the target transport set from settings + build + network + heap, then reconciles the running transports.
    void Reconcile();

    // Same, for the transmit transports (BLE advertising, WiFi beacon injection).
    void ReconcileTx();

    // Rebuilds and re-publishes the transmitted Remote ID content at kTxTickIntervalMs. Called from Update().
    void ServiceTxTick();

    // Returns true if this build/config allows Remote ID to coexist with active WiFi AP/STA (i.e. has PSRAM).
    static bool CanCoexistWithWiFi();

    // Rate-limit decision for forwarding a packet to the RP2040. Updates the dedup table. Returns true to forward.
    bool ShouldForwardToRP2040(const RawRemoteIDPacket& packet);

    // Lazily heap-allocates the ingest + out packet queues on first transport start. Returns true if both exist.
    // Never freed at runtime (see class comment).
    bool EnsureBuffers();

    // BLE (NimBLE) control — implemented in remote_id_ble.cpp.
    bool BLEStart(bool enable_coded_phy);
    void BLEStop();

    // BLE transmit (broadcaster) control — implemented in remote_id_ble.cpp. Shares the NimBLE host with the observer
    // above, so scanning and advertising can run at the same time.
    bool BLETxStart(bool enable_legacy, bool enable_coded_phy);
    void BLETxStop();
    void BLETxServiceTick();  // Refreshes the advertised ODID content; call at the transmit cadence.
    static bool BluetoothIsSupported();  // False when Bluetooth isn't compiled into this firmware.

    // WiFi promiscuous sniffer control — implemented in remote_id_wifi_sniffer.cpp.
    bool WiFiSnifferStart();
    void WiFiSnifferStop();
    void WiFiSnifferServiceHopper();

    // WiFi beacon transmit control — implemented in remote_id_wifi_sniffer.cpp (which owns the shared WiFi radio).
    bool WiFiTxStart();
    void WiFiTxStop();
    void WiFiTxServiceTick(RemoteIDTransmitter& transmitter);

    struct DedupEntry {
        bool in_use = false;
        uint8_t mac[6] = {0};
        uint32_t last_location_forward_ms = 0;
        uint32_t last_static_forward_ms = 0;
    };
    DedupEntry* FindOrAllocDedupEntry(const uint8_t mac[6]);

    uint16_t status_ = 0;
    bool ble_running_ = false;
    bool ble_coded_running_ = false;
    bool wifi_sniffer_running_ = false;

    // Transmit state.
    bool ble_tx_legacy_running_ = false;
    bool ble_tx_coded_running_ = false;
    bool wifi_tx_running_ = false;
    uint32_t last_tx_tick_ms_ = 0;
    // Builds the transmitted Open Drone ID message content. One instance shared by every transmit transport so they
    // advance a single message schedule and message-counter set.
    RemoteIDTransmitter tx_;

    uint8_t sniffer_channel_index_ = 0;
    uint32_t last_channel_hop_ms_ = 0;

    // Lazily heap-allocated (see EnsureBuffers). null until the first transport starts.
    PFBQueue<RawRemoteIDPacket>* ingest_queue_ = nullptr;  // BLE/WiFi producers -> main task.
    PFBQueue<RawRemoteIDPacket>* out_queue_ = nullptr;     // main task -> RP2040 (SPI) pull.

    DedupEntry dedup_table_[kDedupTableNumEntries];
};

// Global instance (defined in remote_id_manager.cpp), analogous to comms_manager / object_dictionary.
extern RemoteIDManager remote_id_manager;
