#include "ble_host.hh"

#include "sdkconfig.h"

#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BT_NIMBLE_ENABLED)

#include <atomic>

#include "comms.hh"  // Logging.
#include "ble_output.hh"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

extern "C" void ble_store_config_init(void);  // NimBLE declares it in no header.

namespace {

constexpr uint8_t kNumUsers = 2;

bool s_initialized = false;
std::atomic<bool> s_synced = false;
uint8_t s_users = 0;
BLEHost::Listener s_listeners[kNumUsers];
std::atomic<bool> s_watchdog_reset = false;  // The next host reset is a watchdog's, already logged.

uint8_t UserIndex(BLEHost::User user) { return user == BLEHost::kUserRemoteID ? 0 : 1; }

void OnHostSync() {
    s_synced = true;
    for (const BLEHost::Listener& listener : s_listeners) {
        if (listener.on_sync) listener.on_sync();
    }
}

void OnHostReset(int reason) {
    s_synced = false;
    if (!s_watchdog_reset.exchange(false)) {
        CONSOLE_WARNING("ble_host", "Bluetooth host reset, reason=%d.", reason);
    }
    for (const BLEHost::Listener& listener : s_listeners) {
        if (listener.on_reset) listener.on_reset();
    }
}

void HostTask(void* param) {
    nimble_port_run();  // Runs until nimble_port_stop().
    nimble_port_freertos_deinit();
}

}  // namespace

namespace BLEHost {

bool Acquire(User user) {
    if (!s_initialized) {
        esp_err_t err = nimble_port_init();
        if (err != ESP_OK) {
            CONSOLE_ERROR("ble_host", "nimble_port_init failed: %d.", err);
            return false;
        }
        ble_hs_cfg.sync_cb = OnHostSync;
        ble_hs_cfg.reset_cb = OnHostReset;
        // Just Works pairing with bonds kept in NVS: some apps (AvareX) connect only to a bonded device. No
        // characteristic requires encryption, so clients that never pair work too.
        ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
        ble_hs_cfg.sm_bonding = 1;
        ble_hs_cfg.sm_sc = 1;
        ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
        ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
        ble_hs_cfg.store_status_cb = ble_store_util_status_rr;  // Full bond store: forget the oldest peer.
        ble_store_config_init();
        // Services must be in place before the host task starts and syncs.
        if (!BLEOutput::RegisterGATTServices()) {
            nimble_port_deinit();
            return false;
        }
        nimble_port_freertos_init(HostTask);
        s_initialized = true;
    }
    s_users |= user;
    return true;
}

void Release(User user) {
    s_users &= ~user;
    if (!s_initialized || s_users != 0) return;
    // Stops the host task and frees the host and controller.
    int rc = nimble_port_stop();
    if (rc != 0) {
        CONSOLE_ERROR("ble_host", "nimble_port_stop failed, rc=%d.", rc);
        return;
    }
    esp_err_t err = nimble_port_deinit();
    if (err != ESP_OK) {
        CONSOLE_ERROR("ble_host", "nimble_port_deinit failed: %d.", err);
    }
    s_initialized = false;
    s_synced = false;
}

bool IsInitialized() { return s_initialized; }

bool IsSynced() { return s_initialized && s_synced && ble_hs_synced(); }

void SetListener(User user, const Listener& listener) { s_listeners[UserIndex(user)] = listener; }

void ScheduleWatchdogReset() {
    s_watchdog_reset = true;
    ble_hs_sched_reset(BLE_HS_ECONTROLLER);
}

}  // namespace BLEHost

#else  // Bluetooth not compiled in.

namespace BLEHost {
bool Acquire(User) { return false; }
void Release(User) {}
bool IsInitialized() { return false; }
bool IsSynced() { return false; }
void SetListener(User, const Listener&) {}
void ScheduleWatchdogReset() {}
}  // namespace BLEHost

#endif  // CONFIG_BT_ENABLED && CONFIG_BT_NIMBLE_ENABLED
