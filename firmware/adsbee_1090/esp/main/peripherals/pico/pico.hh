#pragma once

#include "bsp.hh"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "settings.hh"
#include "spi_coprocessor.hh"

class Pico : public SPICoprocessorMasterInterface {
   public:
    static constexpr uint32_t kNetworkLEDBlinkDurationMs = 1;
    static constexpr uint32_t kNetworkLEDBlinkDurationTicks = kNetworkLEDBlinkDurationMs / portTICK_PERIOD_MS;
    // Since the transaction timeout value is used by threads waiting to use the SPI peripheral, and the SPI update task
    // blocks the copro_spi_mutex_ until it receives a transfer from the master, this timeout needs to be set to the
    // maximum delay between unsolicited packets from the master (heartbeat) to avoid threads giving up while the SPI
    // update task is hogging the mutex.
    // How long to wait once a transaction is started before timing out.
    static constexpr uint16_t kSPITransactionTimeoutMs = 200;
    static constexpr uint16_t kSPITransactionTimeoutTicks = kSPITransactionTimeoutMs / portTICK_PERIOD_MS;
    static constexpr uint16_t kSPIMutexTimeoutMs =
        1200;  // How long to wait for the transaction mutex before timing out.
    static constexpr uint16_t kSPIMutexTimeoutTicks = kSPIMutexTimeoutMs / portTICK_PERIOD_MS;

    struct PicoConfig {
        spi_host_device_t spi_handle = bsp.copro_spi_handle;
        gpio_num_t spi_mosi_pin = bsp.copro_spi_mosi_pin;
        gpio_num_t spi_miso_pin = bsp.copro_spi_miso_pin;
        gpio_num_t spi_clk_pin = bsp.copro_spi_clk_pin;
        gpio_num_t spi_cs_pin = bsp.copro_spi_cs_pin;
        gpio_num_t spi_handshake_pin = bsp.copro_spi_handshake_pin;

        gpio_num_t network_led_pin = bsp.network_led_pin;
    };

    Pico(PicoConfig config_in);
    ~Pico();

    bool Init();
    bool DeInit();

    /**
     * Helper function used by callbacks to set the handshake pin high or low on the ESP32.
     * Located in IRAM for performance improvements when called from ISR.
     */
    inline void IRAM_ATTR SetSPIHandshakePinLevel(bool level) {
        // Only set the handshake pin HI when we know we want to solicit a response and not block + wait.
        // Handshake pin can always be set LO.
        gpio_set_level(config_.spi_handshake_pin, level && use_handshake_pin_);
    }

    inline void SPIUseHandshakePin(bool level) { use_handshake_pin_ = level; }

    inline bool SPIBeginTransaction() {
        if (xSemaphoreTake(spi_mutex_, kSPIMutexTimeoutTicks) != pdTRUE) {
            CONSOLE_ERROR("Pico::SPIBeginTransaction", "Failed to acquire coprocessor SPI mutex after waiting %d ms.",
                          kSPIMutexTimeoutMs);
            return false;
        }
        last_bytes_transacted_ = 0;  // Reset the last bytes transacted counter.
        return true;
    }
    inline void SPIEndTransaction() {
        xSemaphoreGive(spi_mutex_);
        if (last_bytes_transacted_ > 0) {
            BlinkNetworkLED();  // Blink the network LED to indicate a successful transaction.
        }
    }
    inline bool SPIClaimNextTransaction() {
        if (xSemaphoreTake(spi_next_transaction_mutex_, kSPIMutexTimeoutTicks) != pdTRUE) {
            CONSOLE_ERROR("Pico::SPIClaimNextTransaction", "Failed to take SPI context mutex after waiting for %d ms.",
                          kSPIMutexTimeoutMs);
            return false;
        }
        return true;
    }
    inline void SPIReleaseNextTransaction() { xSemaphoreGive(spi_next_transaction_mutex_); }
    int SPIWriteReadBlocking(uint8_t *tx_buf, uint8_t *rx_buf,
                             uint16_t len_bytes = SPICoprocessorPacket::kSPITransactionMaxLenBytes,
                             bool end_transaction = true);

    /**
     * Function called from the task spawned during Init().
     */
    void SPIReceiveTask();

    /**
     * Turns on the network LED for a specified number of milliseconds. Relies on the UpdateNetowrkLED() function to
     * turn the lED off.
     * @param[in] blink_duration_ms Number of milliseconds that the LED should stay on for.
     */
    inline void BlinkNetworkLED(uint32_t blink_duration_ms = kNetworkLEDBlinkDurationMs) {
        // Disable hardware LED via LED_ENABLE setting.
        if (!settings_manager.settings.led_enabled) {
            return;
        }
        BlinkNetworkLEDInternal(blink_duration_ms, false);
    }

    /**
     * Forced blink for test fixtures (AT+LED_BLINK): always lights the LED, bypassing the LED_ENABLE setting.
     * @param[in] blink_duration_ms Number of milliseconds that the LED should stay on for.
     */
    inline void ForceBlinkNetworkLED(uint32_t blink_duration_ms) { BlinkNetworkLEDInternal(blink_duration_ms, true); }

    /**
     * Turns off the network LED if necessary.
     */
    inline void UpdateLED() {
        if (!network_led_on) return;
        // Force the LED off (vs waiting for blink timer) once hardware LEDs are disabled, so a
        // synced LED_ENABLE=0 is honored as soon as it arrives from the master. Forced blinks are exempt since they
        // intentionally bypass LED_ENABLE.
        bool blink_expired =
            xTaskGetTickCount() - network_led_turn_on_timestamp_ticks_ > network_led_blink_duration_ticks_;
        if (blink_expired || (!settings_manager.settings.led_enabled && !network_led_force_)) {
            gpio_set_level(config_.network_led_pin, 0);
            network_led_on = false;
            network_led_force_ = false;
            network_led_blink_duration_ticks_ = kNetworkLEDBlinkDurationTicks;
        }
    }

   private:
    /**
     * Shared implementation for normal and forced network LED blinks. Called from the SPI task (activity blinks) and
     * the SPI receive task (forced blinks via SetBytes) while UpdateLED() runs in the server task; worst case a race
     * costs one tick of blink duration, so no mutex is used.
     * @param[in] blink_duration_ms Number of milliseconds that the LED should stay on for.
     * @param[in] force True to bypass the LED_ENABLE setting until this blink expires.
     */
    inline void BlinkNetworkLEDInternal(uint32_t blink_duration_ms, bool force) {
        TickType_t now = xTaskGetTickCount();
        TickType_t duration_ticks = pdMS_TO_TICKS(blink_duration_ms);
        if (duration_ticks == 0) {
            duration_ticks = 1;
        }
        // Later off-deadline wins: don't let a short activity blink cut an in-progress longer blink short.
        if (!network_led_on ||
            (int32_t)((now + duration_ticks) -
                      (network_led_turn_on_timestamp_ticks_ + network_led_blink_duration_ticks_)) > 0) {
            network_led_turn_on_timestamp_ticks_ = now;
            network_led_blink_duration_ticks_ = duration_ticks;
        }
        if (force) {
            network_led_force_ = true;
        }
        gpio_set_level(config_.network_led_pin, 1);
        network_led_on = true;
    }

    PicoConfig config_;            // Configuration for the RP2040 SPI coprocessor master interface.
    SemaphoreHandle_t spi_mutex_;  // Low level mutex used to guard the SPI peripheral (don't let multiple
                                   // threads queue packets at the same time).
    SemaphoreHandle_t spi_next_transaction_mutex_;  // High level mutex used to claim the next transaction interval.

    // SPI peripheral needs to operate on special buffers that are 32-bit word aligned and in DMA accessible memory.
    uint8_t *spi_rx_buf_ = nullptr;
    uint8_t *spi_tx_buf_ = nullptr;

    bool spi_receive_task_should_exit_ = false;  // Flag used to tell SPI receive task to exit.
    TickType_t network_led_turn_on_timestamp_ticks_ = 0;
    TickType_t network_led_blink_duration_ticks_ = kNetworkLEDBlinkDurationTicks;
    bool network_led_on = false;
    bool network_led_force_ = false;  // True while a forced blink (bypassing LED_ENABLE) is active.
    bool use_handshake_pin_ =
        false;  // Allow handshake pin toggle to be skipped if waiting for a mesage and not writing to master.
    int last_bytes_transacted_ = 0;  // Used to determine whether the last transaction was successful.
};

extern Pico pico_ll;  // Global instance of the RP2040 SPI coprocessor master interface.