#pragma once

#include <cstdint>

/**
 * The one NimBLE Bluetooth host, shared by Remote ID (scan and advertising) and the Bluetooth output GATT server
 * (BLEOutput). The host comes up when the first user acquires it and is freed when the last one releases it.
 *
 * The output's GATT services are registered every time the host starts, since NimBLE only takes services before the
 * host syncs. They are unreachable until the output starts its connectable advertising.
 */
namespace BLEHost {

enum User : uint8_t {
    kUserRemoteID = 1 << 0,
    kUserBLEOutput = 1 << 1,
};

// Called on the NimBLE host task after the host (re)syncs with the controller, and when it resets. After a reset the
// controller has lost every scan, advertising set and connection; users set them up again on the next sync.
struct Listener {
    void (*on_sync)() = nullptr;
    void (*on_reset)() = nullptr;
};

/**
 * Brings the host up if it isn't, and marks `user` as using it.
 * @retval False if the stack failed to initialize.
 */
bool Acquire(User user);

/**
 * Marks `user` done. Frees the host and controller once no user is left.
 */
void Release(User user);

bool IsInitialized();

// True once the host has synced with the controller, so GAP calls are legal.
bool IsSynced();

/**
 * Registers the callbacks of one user. Call before Acquire().
 */
void SetListener(User user, const Listener& listener);

/**
 * Resets the controller and syncs again (all users' on_reset, then on_sync). Used by the stall watchdogs. The reset
 * isn't logged as unexpected.
 */
void ScheduleWatchdogReset();

}  // namespace BLEHost
