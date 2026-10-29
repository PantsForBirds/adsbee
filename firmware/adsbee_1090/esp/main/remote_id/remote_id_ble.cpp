// NimBLE Bluetooth LE transport for Broadcast Remote ID (ASTM F3411) — both receive and transmit.
//
// Receive (observer): scans (passively) on both the 1M PHY (BT4 legacy advertising) and, when requested and supported,
// the Coded PHY (BT5 Long Range) using BLE 5 extended discovery. Advertisements are filtered for the ASTM Remote ID
// Service Data (16-bit UUID 0xFFFA, application code 0x0D); the enclosed Open Drone ID message / message pack bytes are
// handed to RemoteIDManager::OnRawRemoteIDPacket().
//
// Transmit (broadcaster): advertises this device's own Remote ID using BLE 5 extended advertising instances — a legacy
// (BT4, 1M PHY) instance carrying one 25-byte ODID message per advertisement, and/or a Coded PHY (BT5 Long Range)
// instance carrying a full message pack. Both share the single NimBLE host initialized here, and scanning and
// advertising run concurrently under it.
//
// The whole file compiles to no-ops when Bluetooth/NimBLE is not built into the firmware (e.g. a WiFi-only image), so
// RemoteIDManager reports kStatusNotInBuild instead of failing to link.

#include "remote_id_manager.hh"
#include "sdkconfig.h"

#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BT_NIMBLE_ENABLED)

#include <atomic>
#include <cstring>

#include "ble/ble_host.hh"
#include "ble_adv_watchdog.hh"
#include "ble_scan_watchdog.hh"
#include "comms.hh"  // Logging.
#include "hal.hh"    // get_time_since_boot_ms.
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "os/os_mbuf.h"
#include "remote_id_tx.hh"

namespace {

// ASTM Remote ID Bluetooth framing constants.
constexpr uint16_t kRemoteIDServiceUUID16 = 0xFFFA;
constexpr uint8_t kRemoteIDAppCode = 0x0D;  // "ODID" AD application code.
// Within an AD element's data: [UUID lo][UUID hi][app code][message counter][ODID message/pack ...].
constexpr uint8_t kODIDPayloadOffsetInSvcData = 4;

constexpr uint8_t kAdvTypeServiceData16 = 0x16;  // BLE_HS_ADV_TYPE_SVC_DATA_UUID16.

bool s_ble_acquired = false;  // Remote ID holds the shared host (BLEHost).
bool s_scanning = false;
bool s_want_coded = false;
bool s_scan_want = false;  // Receive scanning requested (may be pending until the host syncs).
// Advertising reports received and scan starts, counted on the host task for the scan watchdog on the main task.
std::atomic<uint32_t> s_num_scan_reports = 0;
std::atomic<uint32_t> s_num_scan_starts = 0;
uint32_t s_watched_scan_starts = 0;
uint32_t s_last_scan_watchdog_ms = 0;
BLEScanWatchdog s_scan_watchdog;

// --- Transmit state ---------------------------------------------------------------------------------------------
// Extended advertising instance IDs. Instance 0 carries BT4 legacy advertisements (one 25-byte ODID message each, the
// most any 31-byte legacy payload can hold); instance 1 carries a BT5 Long Range (Coded PHY) message pack.
constexpr uint8_t kAdvInstanceLegacy = 0;
constexpr uint8_t kAdvInstanceCoded = 1;

// ASTM F3411 requires the dynamic Location message at >= 1 Hz. Advertising every 500 ms with Location in every other
// legacy slot keeps Location at ~1 Hz there, while the Coded PHY pack carries Location in every advertisement.
constexpr uint32_t kAdvIntervalMs = 500;
// NimBLE advertising intervals are in units of 0.625 ms.
constexpr uint32_t kAdvIntervalUnits = (kAdvIntervalMs * 1000) / 625;

bool s_tx_legacy_configured = false;
bool s_tx_coded_configured = false;
bool s_tx_legacy_want = false;     // Requested by settings (applied once the host syncs).
bool s_tx_coded_want = false;
// Set from ble_gap_ext_adv_start's result, so the status reports what is really on the air.
bool s_tx_legacy_advertising = false;
bool s_tx_coded_advertising = false;
// Advertising events the controller reported sent, per instance, counted on the host task for the advertising
// watchdogs on the main task.
constexpr uint8_t kNumAdvInstances = 2;
std::atomic<uint32_t> s_num_adv_events[kNumAdvInstances] = {};
BLEAdvWatchdog s_adv_watchdogs[kNumAdvInstances];
bool s_adv_watched[kNumAdvInstances] = {};
uint32_t s_last_adv_watchdog_ms = 0;

int StartAdvInstance(uint8_t instance) {
    return ble_gap_ext_adv_start(instance, /*duration=*/0, /*max_events=*/BLEAdvWatchdog::kEventsPerRun);
}

// An advertising set ends after BLEAdvWatchdog::kEventsPerRun events: count them and start it again at once.
int AdvEventHandler(struct ble_gap_event* event, void* arg) {
    if (event->type != BLE_GAP_EVENT_ADV_COMPLETE) return 0;
    const uint8_t instance = event->adv_complete.instance;
    if (instance >= kNumAdvInstances) return 0;
    s_num_adv_events[instance] += event->adv_complete.num_ext_adv_events;
    const bool wanted = instance == kAdvInstanceLegacy ? s_tx_legacy_want : s_tx_coded_want;
    if (wanted) StartAdvInstance(instance);  // A refusal is retried by the next payload update.
    return 0;
}

// Wraps ODID payload bytes in the ASTM Remote ID BLE Service Data AD structure and pushes them into an advertising
// instance. Layout: [len][0x16][UUID lo][UUID hi][app code][msg counter][ODID bytes...].
bool SetAdvODIDPayload(uint8_t instance, const uint8_t* odid, uint16_t odid_len,
                       RawRemoteIDPacket::Transport transport) {
    if (odid_len == 0) return false;

    // Total AD bytes = 1 length byte + 1 AD type byte + kODIDPayloadOffsetInSvcData framing bytes + the ODID payload.
    // A legacy (BT4) advertisement is capped at BLE_HS_ADV_MAX_SZ (31) bytes, which is exactly why ASTM sizes a single
    // Open Drone ID message at 25 bytes: 2 + 4 + 25 == 31. Extended (Coded PHY) advertisements carry a whole message
    // pack, so the buffer must be sized for that, NOT for BLE_HS_ADV_MAX_SZ.
    const uint16_t ad_len = 2 + kODIDPayloadOffsetInSvcData + odid_len;
    uint8_t ad[2 + kODIDPayloadOffsetInSvcData + RemoteIDTransmitter::kMaxPackLenBytes];
    if (ad_len > sizeof(ad)) return false;
    // Legacy PDUs cannot carry more than 31 bytes of advertising data; reject rather than let the controller truncate.
    if (instance == kAdvInstanceLegacy && ad_len > BLE_HS_ADV_MAX_SZ) return false;

    // The service data element's length byte counts everything after itself.
    uint16_t element_payload_len = 1 /*AD type*/ + kODIDPayloadOffsetInSvcData + odid_len;
    if (element_payload_len > 255) return false;

    uint16_t i = 0;
    ad[i++] = static_cast<uint8_t>(element_payload_len);
    ad[i++] = kAdvTypeServiceData16;
    ad[i++] = static_cast<uint8_t>(kRemoteIDServiceUUID16 & 0xFF);
    ad[i++] = static_cast<uint8_t>(kRemoteIDServiceUUID16 >> 8);
    ad[i++] = kRemoteIDAppCode;
    ad[i++] = remote_id_manager.GetTransmitter().NextMessageCounter(transport);
    memcpy(&ad[i], odid, odid_len);
    i += odid_len;

    struct os_mbuf* om = os_msys_get_pkthdr(i, 0);
    if (om == nullptr) return false;
    if (os_mbuf_append(om, ad, i) != 0) {
        os_mbuf_free_chain(om);
        return false;
    }
    // ble_gap_ext_adv_set_data consumes the mbuf (frees it) whether it succeeds or fails.
    int rc = ble_gap_ext_adv_set_data(instance, om);
    if (rc != 0) {
        CONSOLE_ERROR("remote_id_ble", "ble_gap_ext_adv_set_data(%u) failed, rc=%d.", instance, rc);
        return false;
    }
    return true;
}

// Configures one non-connectable, non-scannable advertising instance. `legacy` selects BT4 legacy PDUs on the 1M PHY;
// otherwise extended PDUs on the Coded PHY (BT5 Long Range).
bool ConfigureAdvInstance(uint8_t instance, bool legacy) {
    struct ble_gap_ext_adv_params params;
    memset(&params, 0, sizeof(params));
    params.connectable = 0;
    params.scannable = 0;
    params.directed = 0;
    params.legacy_pdu = legacy ? 1 : 0;
    params.itvl_min = kAdvIntervalUnits;
    params.itvl_max = kAdvIntervalUnits;
    params.own_addr_type = BLE_OWN_ADDR_PUBLIC;
    params.primary_phy = legacy ? BLE_HCI_LE_PHY_1M : BLE_HCI_LE_PHY_CODED;
    params.secondary_phy = legacy ? BLE_HCI_LE_PHY_1M : BLE_HCI_LE_PHY_CODED;
    params.sid = instance;
    params.tx_power = 127;  // Let the controller pick its maximum.

    int8_t selected_tx_power = 0;
    int rc = ble_gap_ext_adv_configure(instance, &params, &selected_tx_power, AdvEventHandler, nullptr);
    if (rc != 0) {
        CONSOLE_ERROR("remote_id_ble", "ble_gap_ext_adv_configure(%u, legacy=%d) failed, rc=%d.", instance,
                      (int)legacy, rc);
        return false;
    }
    return true;
}

// Loads new ODID data into an instance and starts it if it isn't running. Data changes while advertising (one HCI
// command per tick), so the scan and the other set aren't disturbed. Returns whether it is advertising afterwards.
bool ReloadAdvInstance(uint8_t instance, const uint8_t* odid, uint16_t odid_len,
                       RawRemoteIDPacket::Transport transport) {
    if (!SetAdvODIDPayload(instance, odid, odid_len, transport)) return false;
    if (ble_gap_ext_adv_active(instance)) return true;
    int rc = StartAdvInstance(instance);
    if (rc == BLE_HS_EALREADY) return true;  // Started again by AdvEventHandler meanwhile.
    if (rc != 0) {
        CONSOLE_ERROR("remote_id_ble", "ble_gap_ext_adv_start(%u) failed, rc=%d.", instance, rc);
        return false;
    }
    return true;
}

// Refreshes the advertised payload for whichever transmit instances are running. Called on the transmit tick.
void ServiceAdvertisingPayloads() {
    if (!BLEHost::IsSynced()) return;

    RemoteIDTransmitter& transmitter = remote_id_manager.GetTransmitter();
    uint8_t buf[RemoteIDTransmitter::kMaxPackLenBytes];

    if (s_tx_legacy_configured) {
        // BT4 legacy advertising fits exactly one 25-byte ODID message; cycle through the message types.
        uint16_t len = transmitter.BuildNextSingleMessage(buf);
        s_tx_legacy_advertising =
            len > 0 && ReloadAdvInstance(kAdvInstanceLegacy, buf, len, RawRemoteIDPacket::kTransportBT4Legacy);
    }

    if (s_tx_coded_configured) {
        // BT5 Long Range extended advertising carries the whole message pack in one advertisement.
        uint16_t len = transmitter.BuildMessagePack(buf);
        s_tx_coded_advertising =
            len > 0 && ReloadAdvInstance(kAdvInstanceCoded, buf, len, RawRemoteIDPacket::kTransportBT5LongRange);
    }
}

// Brings the requested advertising instances up. Safe to call repeatedly; only (re)configures what changed.
void StartRequestedAdvertising() {
    if (!BLEHost::IsSynced()) return;

    if (s_tx_legacy_want && !s_tx_legacy_configured) {
        s_tx_legacy_configured = ConfigureAdvInstance(kAdvInstanceLegacy, /*legacy=*/true);
    } else if (!s_tx_legacy_want && s_tx_legacy_configured) {
        ble_gap_ext_adv_stop(kAdvInstanceLegacy);
        ble_gap_ext_adv_remove(kAdvInstanceLegacy);
        s_tx_legacy_configured = false;
        s_tx_legacy_advertising = false;
    }

    if (s_tx_coded_want && !s_tx_coded_configured) {
        s_tx_coded_configured = ConfigureAdvInstance(kAdvInstanceCoded, /*legacy=*/false);
    } else if (!s_tx_coded_want && s_tx_coded_configured) {
        ble_gap_ext_adv_stop(kAdvInstanceCoded);
        ble_gap_ext_adv_remove(kAdvInstanceCoded);
        s_tx_coded_configured = false;
        s_tx_coded_advertising = false;
    }

    // Load an initial payload (and start the instances) immediately so transmission begins without waiting a tick.
    remote_id_manager.GetTransmitter().RefreshFromDeviceState();
    ServiceAdvertisingPayloads();
}

// Extract the ODID payload from an advertisement's AD structures and hand it to the manager. Returns true if a Remote ID
// service data element was found and forwarded.
bool ParseAndForward(const uint8_t* adv_data, uint8_t adv_len, const uint8_t* mac, int8_t rssi,
                     RawRemoteIDPacket::Transport transport) {
    uint8_t i = 0;
    while (i + 1 < adv_len) {
        uint8_t elem_len = adv_data[i];
        if (elem_len == 0) break;
        if (i + 1 + elem_len > adv_len) break;  // Malformed length.
        uint8_t elem_type = adv_data[i + 1];
        const uint8_t* elem_data = &adv_data[i + 2];
        uint8_t elem_data_len = elem_len - 1;

        if (elem_type == kAdvTypeServiceData16 && elem_data_len >= kODIDPayloadOffsetInSvcData) {
            uint16_t uuid = (uint16_t)elem_data[0] | ((uint16_t)elem_data[1] << 8);
            if (uuid == kRemoteIDServiceUUID16 && elem_data[2] == kRemoteIDAppCode) {
                uint8_t odid_len = elem_data_len - kODIDPayloadOffsetInSvcData;
                if (odid_len > 0 && odid_len <= RawRemoteIDPacket::kMaxPayloadLenBytes) {
                    RawRemoteIDPacket packet;
                    packet.received_timestamp_ms = get_time_since_boot_ms();
                    memcpy(packet.source_mac, mac, 6);
                    packet.rssi_dbm = rssi;
                    packet.transport = transport;
                    packet.payload_len_bytes = odid_len;
                    memcpy(packet.payload, &elem_data[kODIDPayloadOffsetInSvcData], odid_len);
                    remote_id_manager.OnRawRemoteIDPacket(packet);
                    return true;
                }
            }
        }
        i += 1 + elem_len;
    }
    return false;
}

int GapEventHandler(struct ble_gap_event* event, void* arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_EXT_DISC: {
            s_num_scan_reports++;
            const struct ble_gap_ext_disc_desc* d = &event->ext_disc;
            // Coded PHY (secondary) indicates a BT5 Long Range advertisement.
            RawRemoteIDPacket::Transport transport = (d->sec_phy == BLE_HCI_LE_PHY_CODED)
                                                         ? RawRemoteIDPacket::kTransportBT5LongRange
                                                         : RawRemoteIDPacket::kTransportBT4Legacy;
            ParseAndForward(d->data, d->length_data, d->addr.val, d->rssi, transport);
            return 0;
        }
        default:
            return 0;
    }
}

void StartExtDiscovery() {
    // Passive scan, no duplicate filtering (Location updates repeat with the same AD structure; the manager de-dups).
    // window < itvl leaves an ~20% idle slice each interval so that, under WiFi/BLE software coexistence, the coex
    // arbiter has BLE-idle time to schedule the WiFi promiscuous sniffer's RX (an unassociated WiFi is otherwise treated
    // as IDLE and starved by BLE). At 80% duty with a 100 ms interval, BLE still catches the ~1 Hz Remote ID adverts
    // easily, so this costs BLE-only capture nothing meaningful. This keeps BLE the priority transport, WiFi best-effort.
    struct ble_gap_ext_disc_params uncoded = {};
    uncoded.itvl = 160;    // 100 ms scan interval (units of 0.625 ms).
    uncoded.window = 128;  // 80 ms scan window -> ~80% duty, ~20% left for the WiFi sniffer under coexistence.
    uncoded.passive = 1;

    struct ble_gap_ext_disc_params coded = uncoded;

    uint8_t own_addr_type = BLE_OWN_ADDR_PUBLIC;
    int rc = ble_gap_ext_disc(own_addr_type, /*duration=*/0, /*period=*/0, /*filter_duplicates=*/0,
                              /*filter_policy=*/BLE_HCI_SCAN_FILT_NO_WL, /*limited=*/0, &uncoded,
                              s_want_coded ? &coded : nullptr, GapEventHandler, nullptr);
    if (rc != 0) {
        CONSOLE_ERROR("remote_id_ble", "ble_gap_ext_disc failed, rc=%d.", rc);
        s_scanning = false;
    } else {
        s_scanning = true;
        s_num_scan_starts++;
    }
}

void OnHostReset() {
    s_scanning = false;
    s_tx_legacy_advertising = false;
    s_tx_coded_advertising = false;
}

void OnHostSync() {
    // Controller/host are in sync; safe to start scanning and/or advertising. Either may have been requested before the
    // host finished syncing, so both are (re)applied here. After a host reset the controller has lost its scan and
    // advertising sets (NimBLE still marks the sets configured), so everything is set up again.
    if (!s_ble_acquired) return;
    s_scanning = false;
    s_tx_legacy_configured = false;
    s_tx_coded_configured = false;
    s_tx_legacy_advertising = false;
    s_tx_coded_advertising = false;
    if (s_scan_want) StartExtDiscovery();
    StartRequestedAdvertising();
}

// Brings the shared host up for the receive (scan) and transmit (advertise) paths. Returns false only if the stack
// failed to initialize.
bool EnsureHostInitialized() {
    if (s_ble_acquired) return true;
    BLEHost::SetListener(BLEHost::kUserRemoteID, {.on_sync = OnHostSync, .on_reset = OnHostReset});
    s_ble_acquired = BLEHost::Acquire(BLEHost::kUserRemoteID);
    // The host may have synced already for another user: OnHostSync() won't run for us then.
    return s_ble_acquired;
}

}  // namespace

bool RemoteIDManager::BLEStart(bool enable_coded_phy) {
    s_want_coded = enable_coded_phy;
    s_scan_want = true;

    if (ble_running_) {
        // Already scanning: if the coded-PHY request changed, restart discovery with the new parameters.
        if (s_scanning && ble_coded_running_ != enable_coded_phy) {
            ble_gap_disc_cancel();
            StartExtDiscovery();
        }
        ble_coded_running_ = enable_coded_phy;
        return true;
    }

    if (!EnsureHostInitialized()) return false;
    // If the host has already synced (e.g. transmit brought it up first, or scanning was toggled off and on), start
    // discovery now; otherwise OnHostSync() will start it. Sync can run first in the host task, so skip if it did.
    if (BLEHost::IsSynced() && !s_scanning) StartExtDiscovery();

    ble_running_ = true;
    ble_coded_running_ = enable_coded_phy;
    return true;
}

void RemoteIDManager::BLEStop() {
    s_scan_want = false;
    if (!ble_running_) return;
    if (s_scanning) {
        ble_gap_disc_cancel();
        s_scanning = false;
    }
    // The host stays up for the transmit path; BLEReleaseIfIdle() frees it once nothing uses it.
    ble_running_ = false;
    ble_coded_running_ = false;
}

bool RemoteIDManager::BLETxStart(bool enable_legacy, bool enable_coded_phy) {
    if (!enable_legacy && !enable_coded_phy) {
        BLETxStop();
        return false;
    }
    s_tx_legacy_want = enable_legacy;
    s_tx_coded_want = enable_coded_phy;

    if (!EnsureHostInitialized()) return false;
    // If the host is already synced, (re)configure the instances now; otherwise OnHostSync() will.
    StartRequestedAdvertising();

    ble_tx_legacy_running_ = enable_legacy;
    ble_tx_coded_running_ = enable_coded_phy;
    return true;
}

void RemoteIDManager::BLETxStop() {
    s_tx_legacy_want = false;
    s_tx_coded_want = false;
    if (BLEHost::IsSynced()) {
        StartRequestedAdvertising();  // Tears down whichever instances are configured.
    }
    ble_tx_legacy_running_ = false;
    ble_tx_coded_running_ = false;
}

void RemoteIDManager::BLETxUpdateStatus() {
    status_ &= ~(kStatusTxBLELegacyActive | kStatusTxBLECodedActive | kStatusTxBlocked);
    if (tx_blocked_by_policy_) status_ |= kStatusTxBlocked;
    if (s_tx_legacy_advertising) status_ |= kStatusTxBLELegacyActive;
    if (s_tx_coded_advertising) status_ |= kStatusTxBLECodedActive;
    // Until the host syncs, nothing is on the air yet and nothing has failed either.
    if (BLEHost::IsSynced() && ((s_tx_legacy_want && !s_tx_legacy_advertising) ||
                            (s_tx_coded_want && !s_tx_coded_advertising))) {
        status_ |= kStatusTxBlocked;
    }
}

void RemoteIDManager::BLETxServiceTick() {
    if (s_tx_legacy_configured || s_tx_coded_configured) {
        // Refresh the ODID content (position moves, message counters advance) and push it into the running instances.
        if (!tx_.RefreshFromDeviceState()) {
            status_ |= kStatusTxNoPosition;
            if (!tx_.PositionSourceAllowed()) status_ |= kStatusTxPositionSourceNotAllowed;
        }
        ServiceAdvertisingPayloads();
    }
    BLETxUpdateStatus();
}

void RemoteIDManager::BLEServiceScanWatchdog() {
    uint32_t now_ms = get_time_since_boot_ms();
    if (now_ms - s_last_scan_watchdog_ms < 1000) return;
    s_last_scan_watchdog_ms = now_ms;
    if (!ble_running_ || !s_scanning || !BLEHost::IsSynced()) return;

    const uint32_t num_reports = s_num_scan_reports;
    const uint32_t num_starts = s_num_scan_starts;
    if (num_starts != s_watched_scan_starts) {
        s_watched_scan_starts = num_starts;
        s_scan_watchdog.Restart(num_reports, now_ms);
        return;
    }
    if (!s_scan_watchdog.Update(num_reports, now_ms)) return;
    // Resets the controller and syncs again; OnHostSync() restarts scanning and advertising.
    CONSOLE_INFO("remote_id_ble", "Bluetooth scan stalled, resetting the controller.");
    BLEHost::ScheduleWatchdogReset();
    s_scan_watchdog.Restart(num_reports, now_ms);
}

void RemoteIDManager::BLEServiceAdvWatchdog() {
    uint32_t now_ms = get_time_since_boot_ms();
    if (now_ms - s_last_adv_watchdog_ms < 1000) return;
    s_last_adv_watchdog_ms = now_ms;
    const bool synced = BLEHost::IsSynced();
    const bool advertising[kNumAdvInstances] = {synced && s_tx_legacy_want && s_tx_legacy_configured,
                                                synced && s_tx_coded_want && s_tx_coded_configured};
    for (uint8_t instance = 0; instance < kNumAdvInstances; instance++) {
        const uint32_t num_events = s_num_adv_events[instance];
        if (!advertising[instance] || !s_adv_watched[instance]) {
            // Not advertising, or just (re)configured: the quiet time starts now.
            s_adv_watched[instance] = advertising[instance];
            s_adv_watchdogs[instance].Restart(num_events, now_ms);
            continue;
        }
        if (!s_adv_watchdogs[instance].Update(num_events, now_ms)) continue;
        // Resets the controller and syncs again; OnHostSync() restarts scanning and advertising.
        CONSOLE_INFO("remote_id_ble", "Bluetooth advertising stalled (set %u), resetting the controller.", instance);
        BLEHost::ScheduleWatchdogReset();
        for (bool& watched : s_adv_watched) watched = false;
        return;
    }
}

void RemoteIDManager::BLEReleaseIfIdle() {
    if (!s_ble_acquired || ble_running_ || ble_tx_legacy_running_ || ble_tx_coded_running_) return;
    // Frees the host and controller, which hold most of the Remote ID RAM, unless the GDL90 server still uses them.
    BLEHost::Release(BLEHost::kUserRemoteID);
    s_ble_acquired = false;
    s_scanning = false;
    s_tx_legacy_configured = false;
    s_tx_coded_configured = false;
    s_tx_legacy_advertising = false;
    s_tx_coded_advertising = false;
}

bool RemoteIDManager::BluetoothIsSupported() { return true; }

#else  // Bluetooth not compiled in: no-op stubs so RemoteIDManager links and reports kStatusNotInBuild.

bool RemoteIDManager::BLEStart(bool) { return false; }
void RemoteIDManager::BLEStop() {}
bool RemoteIDManager::BLETxStart(bool, bool) { return false; }
void RemoteIDManager::BLETxStop() {}
void RemoteIDManager::BLEReleaseIfIdle() {}
void RemoteIDManager::BLETxServiceTick() {}
void RemoteIDManager::BLETxUpdateStatus() {}
void RemoteIDManager::BLEServiceScanWatchdog() {}
void RemoteIDManager::BLEServiceAdvWatchdog() {}
bool RemoteIDManager::BluetoothIsSupported() { return false; }

#endif  // CONFIG_BT_ENABLED && CONFIG_BT_NIMBLE_ENABLED
