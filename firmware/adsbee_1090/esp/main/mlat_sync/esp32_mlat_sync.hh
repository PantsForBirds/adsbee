#pragma once

#include <cstdint>

#include "driver/mcpwm_cap.h"
#include "esp32_clock_math.hh"
#include "freertos/FreeRTOS.h"
#include "mlat_time_sync.hh"

/**
 * Puts Remote ID receive times on the RP2040's 48 MHz MLAT time base (the one Mode S packets use).
 *
 * The RP2040 pulses the shared SYNC line about once a second and then writes the pulse's MLAT count over SPI
 * (ObjectDictionary::SyncPulse). An MCPWM capture channel latches each rising edge on GPIO38 in hardware; pairing the
 * edge with the MLAT count gives a sync point against the system timer. GPIO38 is an input without pulls: SYNC is also
 * the CC1312 bootloader backdoor, so the ESP32 must never drive or pull it.
 *
 * Receive times: WiFi beacons use rx_ctrl.timestamp (see esp32_clock_math::WiFiRxClock). NimBLE gives no receive
 * time, so BLE advertisements are stamped when the GAP callback runs, about 1 ms after the air time.
 */
class ESP32MLATSync {
   public:
    static constexpr int kSyncPinGPIO = 38;
    // A fit this long without a new sync point is dropped (pulses stopped).
    static constexpr uint32_t kSyncPointMaxAgeUs = 30'000'000;

    bool Init();

    // Pairs the RP2040's MLAT count for its last pulse with the last captured edge. SPI task.
    void HandleSyncPulse(uint64_t mlat_48mhz_counts, uint32_t pulse_num);

    // ObjectDictionary::MLATSyncFlag bits for the device status.
    uint8_t GetMLATSyncFlags();

    // System timer (16 MHz, the clock behind esp_timer), read with sub-microsecond resolution. Any task or ISR.
    static uint64_t SystimerNow();

    // 48 MHz MLAT counts of a system timer time, or 0 if not synced. Any task.
    uint64_t SystimerToMLAT48MHzCounts(uint64_t systimer_ticks);
    uint64_t NowToMLAT48MHzCounts() { return SystimerToMLAT48MHzCounts(SystimerNow()); }

    /**
     * 48 MHz MLAT counts of a received WiFi frame, or 0 if not synced. Call from the promiscuous callback for every
     * frame (it also keeps the MAC clock offset), in order.
     */
    uint64_t WiFiRxToMLAT48MHzCounts(uint32_t rx_timestamp_us);

    // Called from the capture interrupts.
    void HandleEdgeCapture(uint32_t capture);
    void HandleSoftCapture(uint32_t capture_now);

   private:
    portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
    mcpwm_cap_timer_handle_t capture_timer_ = nullptr;
    mcpwm_cap_channel_handle_t edge_channel_ = nullptr;
    mcpwm_cap_channel_handle_t soft_channel_ = nullptr;  // Latches the capture timer next to a system timer read.

    // Written by the capture interrupts.
    uint32_t pending_edge_capture_ = 0;
    uint64_t pending_edge_systimer_now_ = 0;
    bool edge_pending_ = false;
    uint64_t edge_systimer_ = 0;
    uint16_t edge_count_ = 0;

    // Under lock_.
    MLATTimeSync time_sync_ = MLATTimeSync(MLATTimeSync::kMLATTicksPerESP32SystimerTick, 2);
    uint64_t last_sync_point_systimer_ = 0;

    SyncPulsePairing sync_pulse_pairing_;  // SPI task only.

    // WiFi task only.
    esp32_clock_math::WiFiRxClock wifi_rx_clock_;
};

extern ESP32MLATSync esp32_mlat_sync;
