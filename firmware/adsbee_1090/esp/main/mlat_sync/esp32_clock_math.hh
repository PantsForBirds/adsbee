#pragma once

#include <cstdint>

/**
 * Clock conversions for the ESP32-S3 MLAT time sync, kept free of ESP-IDF so host tests can run them. The ESP32's local
 * time base is the 16 MHz system timer behind esp_timer (64 bits, 1 us = 16 ticks).
 */
namespace esp32_clock_math {

static constexpr uint32_t kSystimerTicksPerUs = 16;
static constexpr uint32_t kAPBTicksPerSystimerTick = 5;  // MCPWM capture timer: 80 MHz APB clock, same crystal.

/**
 * System timer time of a captured edge. The capture timer has no readable counter, so a software capture
 * (capture_now) and the system timer (systimer_now) are read together after the edge.
 * @param[in] capture Capture timer value latched by the edge.
 * @param[in] capture_now Capture timer value latched together with systimer_now, less than 53 s after the edge.
 * @param[in] systimer_now System timer value.
 */
inline uint64_t CaptureToSystimerTicks(uint32_t capture, uint32_t capture_now, uint64_t systimer_now) {
    uint32_t apb_ticks_since_edge = capture_now - capture;
    return systimer_now - (apb_ticks_since_edge + kAPBTicksPerSystimerTick / 2) / kAPBTicksPerSystimerTick;
}

/**
 * Air time in microseconds of a non-HT (802.11b or 802.11a/g) frame's PSDU, from rx_ctrl.rate (wifi_phy_rate_t) and
 * rx_ctrl.sig_len, or 0 for a rate this doesn't know.
 */
inline uint32_t LegacyPSDUAirTimeUs(uint8_t rate, uint16_t len_bytes) {
    // Twice the rate in Mbps per wifi_phy_rate_t code, 0 for none.
    static constexpr uint8_t kHalfMbps[16] = {2, 4, 11, 22, 0, 4, 11, 22, 96, 48, 24, 12, 108, 72, 36, 18};
    if (rate >= sizeof(kHalfMbps) || kHalfMbps[rate] == 0) return 0;
    uint32_t bits = len_bytes * 8u;
    if (rate < 8) return (bits * 2 + kHalfMbps[rate] - 1) / kHalfMbps[rate];  // DSSS/CCK.
    uint32_t bits_per_symbol = kHalfMbps[rate] * 2;                            // OFDM: 4 us symbols.
    return 4 * ((16 + bits + 6 + bits_per_symbol - 1) / bits_per_symbol);
}

/**
 * Maps rx_ctrl.timestamp of received WiFi frames (32-bit microseconds on the WiFi MAC clock) to esp_timer time. Both
 * count the same crystal, but the MAC clock has its own offset, which the WiFi driver moves when it gates the MAC
 * clock. The receive callback runs after the whole frame, so (callback time - air time - frame time) is the offset plus
 * a processing latency; its smallest value over recent frames is the offset plus the shortest latency. Frame times
 * come out late by that shortest latency, a constant. The air time matters: a 300 byte beacon at 1 Mbps takes 2.4 ms.
 */
class WiFiRxClock {
   public:
    // The offset estimate can rise once per window, by more than kMinRiseUs: the shortest latency in a window of
    // frames may stay a little above the true one.
    static constexpr uint16_t kWindowNumFrames = 256;
    static constexpr uint32_t kMinRiseUs = 10;
    // A frame this much later than the current estimate means the MAC clock stepped back: start over.
    static constexpr uint32_t kMaxLatencyUs = 100'000;

    /**
     * Feed every received frame, in order.
     * @param[in] rx_timestamp_us The frame's rx_ctrl.timestamp.
     * @param[in] now_us esp_timer_get_time() in the receive callback.
     * @param[in] air_time_us LegacyPSDUAirTimeUs() of the frame, or 0 if unknown (HT): then the frame is converted but
     * doesn't move the offset.
     * @retval The frame's esp_timer time, in microseconds.
     */
    int64_t ToEspTimerUs(uint32_t rx_timestamp_us, int64_t now_us, uint32_t air_time_us) {
        if (air_time_us > 0) {
            UpdateOffset(static_cast<uint32_t>(now_us - air_time_us) - rx_timestamp_us);
        }
        if (!valid_) return now_us;
        // rx_timestamp_us + offset_us_, unwrapped next to now_us.
        uint32_t ago_us = static_cast<uint32_t>(now_us) - rx_timestamp_us - offset_us_;
        return now_us - static_cast<int32_t>(ago_us);
    }

   private:
    // delta_us: callback time minus air time minus frame time.
    void UpdateOffset(uint32_t delta_us) {
        int32_t latency_us = static_cast<int32_t>(delta_us - offset_us_);
        if (!valid_ || latency_us > static_cast<int32_t>(kMaxLatencyUs)) {
            valid_ = true;  // First frame, or the MAC clock stepped back: start over.
            window_count_ = 0;
            offset_us_ = delta_us;
        } else if (latency_us < 0) {
            offset_us_ = delta_us;  // A faster callback, or the MAC clock stepped forward.
        }
        // The smallest delta of each window can raise the offset, so the estimate also follows a small step back.
        if (window_count_ == 0 || static_cast<int32_t>(delta_us - window_min_delta_us_) < 0) {
            window_min_delta_us_ = delta_us;
        }
        if (++window_count_ >= kWindowNumFrames) {
            if (static_cast<int32_t>(window_min_delta_us_ - offset_us_) > static_cast<int32_t>(kMinRiseUs)) {
                offset_us_ = window_min_delta_us_;
            }
            window_count_ = 0;
        }
    }

    bool valid_ = false;
    uint32_t offset_us_ = 0;  // esp_timer minus MAC clock, plus the shortest callback latency.
    uint32_t window_min_delta_us_ = 0;
    uint16_t window_count_ = 0;
};

}  // namespace esp32_clock_math
