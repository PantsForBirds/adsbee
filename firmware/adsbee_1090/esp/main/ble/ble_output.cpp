#include "ble_output.hh"

#include "object_dictionary.hh"
#include "sdkconfig.h"

BLEOutput ble_output;

#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BT_NIMBLE_ENABLED) && defined(CONFIG_BT_NIMBLE_ROLE_PERIPHERAL)

#include <atomic>
#include <cstdio>
#include <cstring>
#include <new>

#include "ble_host.hh"
#include "comms.hh"  // comms_manager (network state) + logging.
#include "esp_mac.h"
#include "gdl90_ble_stream.hh"
#include "hal.hh"  // get_time_since_boot_ms.
#include "hardware_capabilities.hh"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_store.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "settings.hh"

namespace {

// Nordic UART Service, 6E40000x-B5A3-F393-E0A9-E50E24DCCA9E (NimBLE stores 128-bit UUIDs little endian).
#define NUS_UUID128(x) \
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, x, 0x00, 0x40, 0x6E)
const ble_uuid128_t kNUSServiceUUID = NUS_UUID128(0x01);
const ble_uuid128_t kNUSRxUUID = NUS_UUID128(0x02);  // Client writes; ignored.
const ble_uuid128_t kNUSTxUUID = NUS_UUID128(0x03);  // GDL90 notifications.
// HM-10 / TI CC254x serial service: one characteristic for both directions.
const ble_uuid16_t kHM10ServiceUUID = BLE_UUID16_INIT(0xFFE0);
const ble_uuid16_t kHM10DataUUID = BLE_UUID16_INIT(0xFFE1);

// Extended advertising instances 0 and 1 belong to Remote ID transmit.
constexpr uint8_t kAdvInstance = 2;
// 152.5 ms: one of the intervals Apple's accessory guidelines list for fast discovery.
constexpr uint32_t kAdvIntervalUnits = 244;  // Units of 0.625 ms.
// ATT notification header: opcode and attribute handle.
constexpr uint16_t kNotifyHeaderBytes = 3;
constexpr uint16_t kMaxNotifyPayloadBytes = CONFIG_BT_NIMBLE_ATT_PREFERRED_MTU - kNotifyHeaderBytes;

uint16_t s_nus_tx_val_handle = 0;
uint16_t s_hm10_val_handle = 0;

std::atomic<bool> s_want = false;  // Advertise and serve.
std::atomic<bool> s_advertising = false;
std::atomic<uint16_t> s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
// Notifications the client turned on. A bonded client gets its old subscriptions back on reconnecting, so it may be
// subscribed to both characteristics; each gets the stream.
std::atomic<bool> s_nus_subscribed = false;
std::atomic<bool> s_hm10_subscribed = false;

bool Subscribed() { return s_nus_subscribed || s_hm10_subscribed; }

void ClearSubscriptions() {
    s_nus_subscribed = false;
    s_hm10_subscribed = false;
}
std::atomic<uint16_t> s_mtu = BLE_ATT_MTU_DFLT;
std::atomic<bool> s_clear_pending = false;  // Drop queued data (the client left or unsubscribed).

// Used on the main task only. Allocated while the server runs.
GDL90BLEStream* s_stream = nullptr;

int AccessCallback(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt* ctxt, void* arg) {
    // Writes are accepted and dropped. Nothing is readable.
    return ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR ? 0 : BLE_ATT_ERR_READ_NOT_PERMITTED;
}

const struct ble_gatt_chr_def kNUSCharacteristics[] = {
    {
        .uuid = &kNUSRxUUID.u,
        .access_cb = AccessCallback,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
    },
    {
        .uuid = &kNUSTxUUID.u,
        .access_cb = AccessCallback,
        .flags = BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &s_nus_tx_val_handle,
    },
    {},
};

const struct ble_gatt_chr_def kHM10Characteristics[] = {
    {
        .uuid = &kHM10DataUUID.u,
        .access_cb = AccessCallback,
        .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
        .val_handle = &s_hm10_val_handle,
    },
    {},
};

const struct ble_gatt_svc_def kServices[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &kNUSServiceUUID.u,
        .characteristics = kNUSCharacteristics,
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &kHM10ServiceUUID.u,
        .characteristics = kHM10Characteristics,
    },
    {},
};

// "ADSBee 1090 " plus the last two Bluetooth MAC bytes, so units can be told apart.
void DeviceName(char* name, size_t len) {
    uint8_t mac[6] = {};
    esp_read_mac(mac, ESP_MAC_BT);
    snprintf(name, len, "ADSBee 1090 %02X%02X", mac[4], mac[5]);
}

// Main task: sends queued bytes until the stream is empty or the NimBLE buffers run low.
void Pump() {
    if (s_stream == nullptr) return;
    if (s_clear_pending.exchange(false)) s_stream->Clear();
    const uint16_t conn_handle = s_conn_handle;
    uint16_t handles[2];
    uint8_t num_handles = 0;
    if (s_nus_subscribed) handles[num_handles++] = s_nus_tx_val_handle;
    if (s_hm10_subscribed) handles[num_handles++] = s_hm10_val_handle;
    if (num_handles == 0 || conn_handle == BLE_HS_CONN_HANDLE_NONE) return;
    uint16_t payload_len = s_mtu - kNotifyHeaderBytes;
    if (payload_len > kMaxNotifyPayloadBytes) payload_len = kMaxNotifyPayloadBytes;
    uint8_t chunk[kMaxNotifyPayloadBytes];
    while (os_msys_num_free() >= BLEOutput::kMinFreeNimBLEBuffers + num_handles) {
        const uint16_t len = s_stream->Peek(chunk, payload_len);
        if (len == 0) return;
        for (uint8_t i = 0; i < num_handles; i++) {
            struct os_mbuf* om = ble_hs_mbuf_from_flat(chunk, len);
            // Consumes the mbuf whether it succeeds or fails.
            if (om == nullptr || ble_gatts_notify_custom(conn_handle, handles[i], om) != 0) {
                if (i == 0) return;  // Nothing sent: retried on the next Update().
            }
        }
        s_stream->Consume(len);
    }
}

int GapEventHandler(struct ble_gap_event* event, void* arg);

bool StartAdvertising() {
    if (!BLEHost::IsSynced() || s_advertising || s_conn_handle != BLE_HS_CONN_HANDLE_NONE) return s_advertising;

    if (!ble_gap_ext_adv_active(kAdvInstance)) {
        ble_gap_ext_adv_remove(kAdvInstance);  // Left over from before a host reset.
    }
    struct ble_gap_ext_adv_params params;
    memset(&params, 0, sizeof(params));
    params.connectable = 1;
    params.scannable = 1;
    params.legacy_pdu = 1;
    params.itvl_min = kAdvIntervalUnits;
    params.itvl_max = kAdvIntervalUnits;
    params.own_addr_type = BLE_OWN_ADDR_PUBLIC;
    params.primary_phy = BLE_HCI_LE_PHY_1M;
    params.secondary_phy = BLE_HCI_LE_PHY_1M;
    params.sid = kAdvInstance;
    params.tx_power = 127;  // Controller maximum.
    int rc = ble_gap_ext_adv_configure(kAdvInstance, &params, nullptr, GapEventHandler, nullptr);
    if (rc != 0) {
        CONSOLE_ERROR("ble_output", "ble_gap_ext_adv_configure failed, rc=%d.", rc);
        return false;
    }

    // Advertising data: flags and the service UUID. Scan response: the name.
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids16 = &kHM10ServiceUUID;
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;
    fields.uuids128 = &kNUSServiceUUID;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
    uint8_t buf[BLE_HS_ADV_MAX_SZ];
    uint8_t len = 0;
    rc = ble_hs_adv_set_fields(&fields, buf, &len, sizeof(buf));
    struct os_mbuf* om = rc == 0 ? os_msys_get_pkthdr(len, 0) : nullptr;
    if (om != nullptr && os_mbuf_append(om, buf, len) != 0) {
        os_mbuf_free_chain(om);
        om = nullptr;
    }
    // ble_gap_ext_adv_set_data consumes the mbuf whether it succeeds or fails.
    if (om == nullptr || ble_gap_ext_adv_set_data(kAdvInstance, om) != 0) {
        CONSOLE_ERROR("ble_output", "Failed to set advertising data.");
        return false;
    }

    char name[24];
    DeviceName(name, sizeof(name));
    memset(&fields, 0, sizeof(fields));
    fields.name = reinterpret_cast<const uint8_t*>(name);
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;
    len = 0;
    rc = ble_hs_adv_set_fields(&fields, buf, &len, sizeof(buf));
    om = rc == 0 ? os_msys_get_pkthdr(len, 0) : nullptr;
    if (om != nullptr && os_mbuf_append(om, buf, len) != 0) {
        os_mbuf_free_chain(om);
        om = nullptr;
    }
    if (om == nullptr || ble_gap_ext_adv_rsp_set_data(kAdvInstance, om) != 0) {
        CONSOLE_ERROR("ble_output", "Failed to set scan response data.");
        return false;
    }

    rc = ble_gap_ext_adv_start(kAdvInstance, /*duration=*/0, /*max_events=*/0);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        CONSOLE_ERROR("ble_output", "ble_gap_ext_adv_start failed, rc=%d.", rc);
        return false;
    }
    s_advertising = true;
    return true;
}

void StopAdvertising() {
    if (!BLEHost::IsSynced()) {
        s_advertising = false;
        return;
    }
    ble_gap_ext_adv_stop(kAdvInstance);
    ble_gap_ext_adv_remove(kAdvInstance);
    s_advertising = false;
}

int GapEventHandler(struct ble_gap_event* event, void* arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                s_conn_handle = event->connect.conn_handle;
                s_mtu = BLE_ATT_MTU_DFLT;
                s_advertising = false;  // A connection ends a legacy connectable set.
                CONSOLE_INFO("ble_output", "Client connected.");
            } else {
                s_advertising = false;
                if (s_want) StartAdvertising();
            }
            return 0;
        case BLE_GAP_EVENT_DISCONNECT:
            CONSOLE_INFO("ble_output", "Client disconnected, reason=0x%x.", event->disconnect.reason);
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ClearSubscriptions();
            s_clear_pending = true;
            if (s_want) StartAdvertising();  // Or Update() retries.
            return 0;
        case BLE_GAP_EVENT_ADV_COMPLETE:
            s_advertising = false;
            if (s_want && s_conn_handle == BLE_HS_CONN_HANDLE_NONE) StartAdvertising();
            return 0;
        case BLE_GAP_EVENT_SUBSCRIBE:
            if (event->subscribe.attr_handle == s_nus_tx_val_handle ||
                event->subscribe.attr_handle == s_hm10_val_handle) {
                const bool on = event->subscribe.cur_notify;
                (event->subscribe.attr_handle == s_nus_tx_val_handle ? s_nus_subscribed : s_hm10_subscribed) = on;
                if (!Subscribed()) s_clear_pending = true;
            }
            return 0;
        case BLE_GAP_EVENT_REPEAT_PAIRING: {
            // The client lost its bond with us: forget ours and let it pair again.
            struct ble_gap_conn_desc desc;
            if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
                ble_store_util_delete_peer(&desc.peer_id_addr);
            }
            return BLE_GAP_REPEAT_PAIRING_RETRY;
        }
        case BLE_GAP_EVENT_MTU:
            s_mtu = event->mtu.value;
            return 0;
        default:
            return 0;
    }
}

void OnHostSync() {
    s_advertising = false;
    if (s_want) StartAdvertising();
}

void OnHostReset() {
    s_advertising = false;
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    ClearSubscriptions();
}

}  // namespace

bool BLEOutput::RegisterGATTServices() {
    ble_svc_gap_init();
    ble_svc_gatt_init();
    char name[24];
    DeviceName(name, sizeof(name));
    ble_svc_gap_device_name_set(name);
    int rc = ble_gatts_count_cfg(kServices);
    if (rc == 0) rc = ble_gatts_add_svcs(kServices);
    if (rc != 0) {
        CONSOLE_ERROR("ble_output", "Failed to register the GATT services, rc=%d.", rc);
        return false;
    }
    return true;
}

bool BLEOutput::HasSubscriber() const { return running_ && Subscribed(); }

void BLEOutput::BeginReport() {
    if (!HasSubscriber()) return;
    if (s_clear_pending.exchange(false)) s_stream->Clear();
    s_stream->DropUnsent();
}

bool BLEOutput::Send(const uint8_t* buf, uint16_t len, bool droppable) {
    if (!HasSubscriber()) return false;
    if (s_clear_pending.exchange(false)) s_stream->Clear();
    return s_stream->Push(buf, len, droppable);
}

void BLEOutput::SendTraffic(const uint8_t* buf, uint16_t len, uint16_t index) {
    if (!Send(buf, len)) rotation_.Refused(index);
}

void BLEOutput::EndTraffic(uint16_t num_aircraft) { rotation_.End(num_aircraft); }

bool BLEOutput::Start(SettingsManager::BLEOutputMode mode) {
    if (running_) return true;
    if (s_stream == nullptr) {
        s_stream = new (std::nothrow) GDL90BLEStream(
            HardwareCapabilities::HasPSRAM() ? kStreamCapacityBytes : kStreamCapacityBytesNoPSRAM,
            mode == SettingsManager::kBLEOutputFLARMNMEA ? GDL90BLEStream::kFramingLines
                                                        : GDL90BLEStream::kFramingGDL90);
        if (s_stream == nullptr || !s_stream->Valid()) {
            delete s_stream;
            s_stream = nullptr;
            return false;
        }
    }
    BLEHost::SetListener(BLEHost::kUserBLEOutput, {.on_sync = OnHostSync, .on_reset = OnHostReset});
    if (!BLEHost::Acquire(BLEHost::kUserBLEOutput)) return false;
    s_want = true;
    running_ = true;
    running_mode_ = mode;
    StartAdvertising();  // Or OnHostSync() does, once the host syncs.
    return true;
}

void BLEOutput::Stop() {
    if (!running_) return;
    s_want = false;
    StopAdvertising();
    const uint16_t conn_handle = s_conn_handle;
    if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    ClearSubscriptions();
    s_clear_pending = false;
    BLEHost::Release(BLEHost::kUserBLEOutput);
    if (!BLEHost::IsInitialized()) OnHostReset();
    delete s_stream;
    s_stream = nullptr;
    running_ = false;
    running_mode_ = SettingsManager::kBLEOutputOff;
}

void BLEOutput::Reconcile() {
    const SettingsManager::BLEOutputMode mode = settings_manager.settings.ble_output_mode;
    if (mode != SettingsManager::kBLEOutputGDL90 && mode != SettingsManager::kBLEOutputFLARMNMEA) {
        Stop();
        status_ = kBLEOutputStatusOff;
        return;
    }
    // A new mode drops the client: its app reads the other format.
    if (running_ && running_mode_ != mode) Stop();
    const SettingsManager::Settings& settings = settings_manager.settings;
    const bool remote_id_uses_bluetooth =
        (settings.remote_id_rx_enabled && (settings.remote_id_transports & RemoteIDGuard::kTransportsBLE)) ||
        (settings.remote_id_tx_enabled && (settings.remote_id_tx_transports & RemoteIDGuard::kTransportsBLE));
    const BLEOutputStatus allowed = BLEOutputAllowed(HardwareCapabilities::HasPSRAM(), comms_manager.wifi_ap_enabled,
                                                     comms_manager.wifi_sta_enabled, remote_id_uses_bluetooth);
    if (allowed != kBLEOutputStatusAdvertising) {
        Stop();
        status_ = allowed;
        return;
    }
    if (heap_watch_.IsPaused()) {
        Stop();
        status_ = kBLEOutputStatusBlockedByRAM;
        return;
    }
    if (!running_) {
        const uint32_t start_bytes =
            BLEHost::IsInitialized()
                ? RemoteIDGuard::kReserveFreeBytes
                : (HardwareCapabilities::HasPSRAM() ? kMinHeapFreeBytesToStart
                                                    : RemoteIDGuard::kReserveFreeBytes + kBLEHeapCostBytes);
        if (HardwareCapabilities::GetInternalFreeBytes() < start_bytes || !Start(mode)) {
            status_ = kBLEOutputStatusBlockedByRAM;
            return;
        }
    }
    status_ = kBLEOutputStatusAdvertising;
}

void BLEOutput::Update() {
    if (apply_pending_.exchange(false)) {
        heap_watch_.Reset();
        Reconcile();
    }

    const uint32_t now_ms = get_time_since_boot_ms();
    if (now_ms - last_heap_watch_ms_ >= 1000) {
        last_heap_watch_ms_ = now_ms;
        const uint32_t free_bytes = HardwareCapabilities::GetInternalFreeBytes();
        const uint32_t largest_block_bytes =
            heap_caps_get_largest_free_block(HardwareCapabilities::kInternalHeapCaps);
        const bool was_paused = heap_watch_.IsPaused();
        if (heap_watch_.Update(free_bytes, largest_block_bytes, now_ms, running_) != was_paused) {
            if (heap_watch_.IsPaused()) {
                CONSOLE_WARNING("ble_output", "Pausing Bluetooth output: %lu Bytes free. Retrying in %lu s.",
                                (unsigned long)free_bytes, (unsigned long)(heap_watch_.HoldOffMs() / 1000));
            }
            Reconcile();
        } else if (status_ == kBLEOutputStatusBlockedByRAM &&
                   now_ms - last_blocked_retry_ms_ >= kBlockedRetryIntervalMs) {
            last_blocked_retry_ms_ = now_ms;
            Reconcile();
        }
    }

    if (running_) {
        if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            status_ = Subscribed() ? kBLEOutputStatusStreaming : kBLEOutputStatusConnected;
        } else {
            status_ = kBLEOutputStatusAdvertising;
            // Restart advertising that a failed call or a host reset left off.
            if (s_want && !s_advertising && BLEHost::IsSynced() && now_ms - last_adv_retry_ms_ >= kAdvRetryIntervalMs) {
                last_adv_retry_ms_ = now_ms;
                StartAdvertising();
            }
        }
        Pump();
    }
    object_dictionary.device_status.ble_output_status = status_;
}

#else  // Built without the Bluetooth peripheral role.

bool BLEOutput::RegisterGATTServices() { return true; }
bool BLEOutput::HasSubscriber() const { return false; }
void BLEOutput::BeginReport() {}
void BLEOutput::SendTraffic(const uint8_t*, uint16_t, uint16_t) {}
void BLEOutput::EndTraffic(uint16_t) {}
bool BLEOutput::Send(const uint8_t*, uint16_t, bool) { return false; }
bool BLEOutput::Start(SettingsManager::BLEOutputMode) { return false; }
void BLEOutput::Stop() {}
void BLEOutput::Reconcile() {
    status_ = settings_manager.settings.ble_output_mode != SettingsManager::kBLEOutputOff ? kBLEOutputStatusNotInBuild
                                                                                       : kBLEOutputStatusOff;
}
void BLEOutput::Update() {
    if (apply_pending_.exchange(false)) {
        Reconcile();
    }
    object_dictionary.device_status.ble_output_status = status_;
}

#endif
