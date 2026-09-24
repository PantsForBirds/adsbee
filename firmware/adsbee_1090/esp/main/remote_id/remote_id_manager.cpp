#include "remote_id_manager.hh"

#include <cstring>
#include <new>  // std::nothrow

#include "comms.hh"                 // comms_manager (WiFi/Ethernet state) + logging.
#include "esp_heap_caps.h"          // heap_caps_get_free_size.
#include "hal.hh"                   // get_time_since_boot_ms.
#include "object_dictionary.hh"     // object_dictionary.device_status.remote_id_status.
#include "sdkconfig.h"              // CONFIG_SPIRAM / CONFIG_BT_* feature flags.
#include "server/adsbee_server.hh"  // adsbee_server: local aircraft dictionary.

RemoteIDManager remote_id_manager;

bool RemoteIDManager::EnsureBuffers() {
    // Allocate the hand-off queues on first use so a Remote-ID-disabled / no-Bluetooth build pays zero internal SRAM.
    // PFBQueue with buffer=nullptr self-allocates its backing store. Never freed (see class comment). is_thread_safe is
    // required: the ingest queue has two producer tasks (NimBLE host + WiFi promiscuous callback), and the out queue is
    // produced on the main task and consumed on the SPI task.
    if (ingest_queue_ == nullptr) {
        ingest_queue_ = new (std::nothrow) PFBQueue<RawRemoteIDPacket>(
            {.buf_len_num_elements = kIngestQueueDepth,
             .buffer = nullptr,
             .overwrite_when_full = true,  // Drop oldest under a burst rather than block the radio callback.
             .is_thread_safe = true});
    }
    if (out_queue_ == nullptr) {
        out_queue_ = new (std::nothrow) PFBQueue<RawRemoteIDPacket>({.buf_len_num_elements = kOutQueueDepth,
                                                                     .buffer = nullptr,
                                                                     .overwrite_when_full = false,
                                                                     .is_thread_safe = true});
    }
    return ingest_queue_ != nullptr && out_queue_ != nullptr;
}

bool RemoteIDManager::CanCoexistWithWiFi() {
#ifdef CONFIG_SPIRAM
    return true;  // PSRAM build: NimBLE host heap + LWIP/WiFi buffers live in PSRAM, so BLE can coexist with WiFi AP/STA.
#else
    return false;  // Internal-RAM-only build: not enough SRAM to run WiFi AP/STA and the BLE stack simultaneously.
#endif
}

void RemoteIDManager::OnRawRemoteIDPacket(const RawRemoteIDPacket& packet) {
    // Runs in the NimBLE host task or the WiFi promiscuous callback. Keep it minimal: just enqueue. The queue exists
    // whenever a transport is running (EnsureBuffers is called before any transport starts).
    if (ingest_queue_ != nullptr) {
        ingest_queue_->Enqueue(packet);
    }
}

RemoteIDManager::DedupEntry* RemoteIDManager::FindOrAllocDedupEntry(const uint8_t mac[6]) {
    DedupEntry* free_slot = nullptr;
    uint32_t oldest_ms = UINT32_MAX;
    DedupEntry* oldest_slot = &dedup_table_[0];
    for (auto& entry : dedup_table_) {
        if (entry.in_use && memcmp(entry.mac, mac, 6) == 0) {
            return &entry;
        }
        if (!entry.in_use && free_slot == nullptr) {
            free_slot = &entry;
        }
        uint32_t recent = entry.last_location_forward_ms > entry.last_static_forward_ms
                              ? entry.last_location_forward_ms
                              : entry.last_static_forward_ms;
        if (recent < oldest_ms) {
            oldest_ms = recent;
            oldest_slot = &entry;
        }
    }
    // Not found: reuse a free slot, else evict the least-recently-forwarded transmitter.
    DedupEntry* slot = free_slot ? free_slot : oldest_slot;
    slot->in_use = true;
    memcpy(slot->mac, mac, 6);
    slot->last_location_forward_ms = 0;
    slot->last_static_forward_ms = 0;
    return slot;
}

bool RemoteIDManager::ShouldForwardToRP2040(const RawRemoteIDPacket& packet) {
    // Determine whether this advertisement carries dynamic (Location/Vector) data, which we forward at up to 1 Hz, vs
    // static identity data, which we forward at most every 10 s. We only need a coarse classification here; the RP2040
    // decodes the full content. A message pack always contains a Location, so treat packs as dynamic. For a single
    // message, the message type is the high nibble of the first ODID byte: type 1 == Location/Vector.
    bool is_dynamic = packet.GetIsMessagePack() ||
                      (packet.payload_len_bytes > 0 && (packet.payload[0] >> 4) == 0x1 /* ODID_MESSAGETYPE_LOCATION */);

    uint32_t now_ms = get_time_since_boot_ms();
    DedupEntry* entry = FindOrAllocDedupEntry(packet.source_mac);
    if (is_dynamic) {
        if (now_ms - entry->last_location_forward_ms < kLocationForwardIntervalMs) return false;
        entry->last_location_forward_ms = now_ms;
        return true;
    }
    if (now_ms - entry->last_static_forward_ms < kStaticForwardIntervalMs) return false;
    entry->last_static_forward_ms = now_ms;
    return true;
}

uint16_t RemoteIDManager::ServiceIngestQueue() {
    // Runs on the main task (ADSBeeServer::Update), so it is the only mutator of the aircraft dictionary and the RP2040
    // out-queue from this path.
    if (ingest_queue_ == nullptr) return 0;  // No transport has started; nothing allocated.
    uint16_t num_processed = 0;
    RawRemoteIDPacket packet;
    while (ingest_queue_->Dequeue(packet)) {
        num_processed++;
        // Always ingest locally (cheap) — this drives the ESP32's network/web output.
        adsbee_server.aircraft_dictionary.IngestRawRemoteIDPacket(packet);
        // Forward a rate-limited copy up to the RP2040 for serial reporting.
        if (out_queue_ != nullptr && ShouldForwardToRP2040(packet)) {
            out_queue_->Enqueue(packet);
        }
    }
    return num_processed;
}

void RemoteIDManager::Reconcile() {
    status_ = 0;

    if (!settings_manager.settings.remote_id_rx_enabled) {
        BLEStop();
        WiFiSnifferStop();
        return;
    }

    const uint8_t transports = settings_manager.settings.remote_id_transports;
    const bool want_ble = transports & (SettingsManager::kRemoteIDTransportBLE4 | SettingsManager::kRemoteIDTransportBLE5Long);
    const bool want_coded = transports & SettingsManager::kRemoteIDTransportBLE5Long;
    const bool want_wifi = transports & SettingsManager::kRemoteIDTransportWiFiBeacon;

    const bool wifi_ap_sta_up =
        comms_manager.wifi_ap_enabled || comms_manager.wifi_sta_enabled;

    // On non-PSRAM builds, Remote ID may only run when WiFi AP/STA are off (and Ethernet is carrying IP).
    if (wifi_ap_sta_up && !CanCoexistWithWiFi()) {
        status_ |= kStatusBlockedByWiFi;
        // Still surface whether Bluetooth is even compiled in, so this early-return doesn't mask kStatusNotInBuild: a
        // user seeing only "blocked_by_wifi" can't otherwise tell they also need the Bluetooth-enabled firmware.
        if (want_ble && !BluetoothIsSupported()) status_ |= kStatusNotInBuild;
        BLEStop();
        WiFiSnifferStop();
        return;
    }

    // At least one transport is wanted and allowed here. Allocate the packet queues on first use (a Remote-ID-disabled
    // build returns above and never reaches this, paying zero internal SRAM for them).
    if (!EnsureBuffers()) {
        status_ |= kStatusBlockedByRAM;
        return;
    }

    // --- BLE (priority transport) ---
    // BLE is the dominant Remote ID transport, so it gets first claim on the shared radio and internal RAM. It time-
    // shares the radio with the WiFi sniffer below via software coexistence (CONFIG_ESP_COEX_SW_COEXIST_ENABLE); the BLE
    // scan runs at a reduced duty cycle (see remote_id_ble.cpp) so coex has idle slots to schedule WiFi RX.
    if (want_ble) {
        if (!BluetoothIsSupported()) {
            // Bluetooth isn't compiled into this firmware at all: the user needs a different image.
            status_ |= kStatusNotInBuild;
        } else if (ble_running_ || heap_caps_get_free_size(MALLOC_CAP_8BIT) >= kMinHeapFreeBytesForBLE) {
            if (BLEStart(want_coded)) {
                status_ |= kStatusBLEActive;
                if (ble_coded_running_) status_ |= kStatusBLECodedPHYActive;
            } else {
                // Bluetooth IS in this build, so a failure here is a runtime one (e.g. nimble_port_init() ran out of
                // resources); BLEStart() logs the underlying error. Report it as a RAM/resource block rather than
                // telling the user to reflash firmware they already have.
                status_ |= kStatusBlockedByRAM;
            }
        } else {
            status_ |= kStatusBlockedByRAM;
        }
    } else {
        BLEStop();
    }

    // --- WiFi beacon sniffer (best-effort) ---
    // Only meaningful when WiFi AP/STA is not otherwise in use (channel-hopping sniffer). On a non-PSRAM board the
    // internal SRAM cannot hold BOTH the BLE controller/host and the WiFi driver alongside the network stack (ethernet +
    // httpd + feeds) — running both drops free heap/DMA low enough to starve safe_send and drop packets — so the sniffer
    // is mutually exclusive with BLE there: it runs only when BLE is not active (BLE is the priority transport). A PSRAM
    // board (CanCoexistWithWiFi) has the headroom to run both. When allowed, it still passes a heap guard as a backstop.
    const bool ble_is_active = (status_ & kStatusBLEActive) != 0;
    const bool wifi_sniffer_allowed = want_wifi && !wifi_ap_sta_up && (CanCoexistWithWiFi() || !ble_is_active);
    if (wifi_sniffer_allowed) {
        if (wifi_sniffer_running_ ||
            heap_caps_get_free_size(MALLOC_CAP_8BIT) >= kMinHeapFreeBytesForWiFiSniffer) {
            if (WiFiSnifferStart()) {
                status_ |= kStatusWiFiSnifferActive;
            }
        } else {
            status_ |= kStatusBlockedByRAM;  // Not enough RAM for the WiFi sniffer; BLE stays up.
        }
    } else {
        WiFiSnifferStop();
    }
}

void RemoteIDManager::ReconcileTx() {
    if (!settings_manager.settings.remote_id_tx_enabled) {
        BLETxStop();
        WiFiTxStop();
        return;
    }

    const uint8_t transports = settings_manager.settings.remote_id_tx_transports;
    const bool want_ble_legacy = transports & SettingsManager::kRemoteIDTransportBLE4;
    const bool want_ble_coded = transports & SettingsManager::kRemoteIDTransportBLE5Long;
    const bool want_wifi = transports & SettingsManager::kRemoteIDTransportWiFiBeacon;

    // Transmitting does not need WiFi AP/STA (a Remote ID transmitter has no use for them), but if the user has them
    // enabled on a board that can't coexist, the radios are already committed and Remote ID must stay off.
    const bool wifi_ap_sta_up = comms_manager.wifi_ap_enabled || comms_manager.wifi_sta_enabled;
    if (wifi_ap_sta_up && !CanCoexistWithWiFi()) {
        status_ |= kStatusBlockedByWiFi | kStatusTxBlocked;
        BLETxStop();
        WiFiTxStop();
        return;
    }

    // --- BLE transmit ---
    if (want_ble_legacy || want_ble_coded) {
        if (!BluetoothIsSupported()) {
            status_ |= kStatusNotInBuild | kStatusTxBlocked;
        } else if (ble_tx_legacy_running_ || ble_tx_coded_running_ ||
                   heap_caps_get_free_size(MALLOC_CAP_8BIT) >= kMinHeapFreeBytesForTx) {
            if (BLETxStart(want_ble_legacy, want_ble_coded)) {
                if (ble_tx_legacy_running_) status_ |= kStatusTxBLELegacyActive;
                if (ble_tx_coded_running_) status_ |= kStatusTxBLECodedActive;
            } else {
                status_ |= kStatusTxBlocked;
            }
        } else {
            status_ |= kStatusBlockedByRAM | kStatusTxBlocked;
        }
    } else {
        BLETxStop();
    }

    // --- WiFi beacon transmit ---
    // Only when WiFi AP/STA isn't using the radio. The transmitter parks the radio on its beacon channel; the RX
    // sniffer (if also running) stops hopping and listens there (see WiFiSnifferServiceHopper).
    if (want_wifi && !wifi_ap_sta_up) {
        if (wifi_tx_running_ || heap_caps_get_free_size(MALLOC_CAP_8BIT) >= kMinHeapFreeBytesForTx) {
            if (WiFiTxStart()) {
                status_ |= kStatusTxWiFiBeaconActive;
            } else {
                status_ |= kStatusTxBlocked;
            }
        } else {
            status_ |= kStatusBlockedByRAM | kStatusTxBlocked;
        }
    } else {
        WiFiTxStop();
    }
}

void RemoteIDManager::ServiceTxTick() {
    if (!ble_tx_legacy_running_ && !ble_tx_coded_running_ && !wifi_tx_running_) return;

    uint32_t now_ms = get_time_since_boot_ms();
    if (now_ms - last_tx_tick_ms_ < kTxTickIntervalMs) return;
    last_tx_tick_ms_ = now_ms;

    // Recomputed below from the current position availability and source.
    status_ &= ~(kStatusTxNoPosition | kStatusTxPositionSourceNotAllowed);

    // BLE refreshes its own advertising payload (it owns the RemoteIDTransmitter that tracks the message schedule).
    BLETxServiceTick();

    if (wifi_tx_running_) {
        // The WiFi beacon carries a full message pack built from the same refreshed ODID data.
        if (!tx_.RefreshFromDeviceState()) {
            status_ |= kStatusTxNoPosition;
            if (!tx_.PositionSourceAllowed()) status_ |= kStatusTxPositionSourceNotAllowed;
        }
        WiFiTxServiceTick(tx_);
    }
}

void RemoteIDManager::Apply() {
    Reconcile();
    ReconcileTx();
    // Publish the resolved status so the RP2040 can surface it via AT+REMOTE_ID? / AT+REMOTE_ID_TX?.
    object_dictionary.device_status.remote_id_status = status_;
}

void RemoteIDManager::Update() {
    if (wifi_sniffer_running_) {
        WiFiSnifferServiceHopper();
    }
    ServiceTxTick();
    object_dictionary.device_status.remote_id_status = status_;
}
