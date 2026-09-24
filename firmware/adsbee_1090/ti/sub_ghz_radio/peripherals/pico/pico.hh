#pragma once

#include "bsp.hh"
#include "settings.hh"
#include "spi_coprocessor.hh"
#include "spi_coprocessor_packet.hh"

extern "C" {
#include "ti/drivers/GPIO.h"
#include "ti/drivers/SPI.h"
}

// // Include TI's memory utilities
// #include "ti/drivers/dma/UDMACC26XX.h"

class Pico : public SPICoprocessorMasterInterface {
   public:
    static constexpr uint32_t kSubGLEDBlinkDurationMs = 1;

    static const uint16_t kSPITransactionMaxLenBytes =
        SPICoprocessorPacket::kSPITransactionMaxLenBytes;  // Normally set to
                                                           // SPICoprocessorPacket::kSPITransactionMaxLenBytes, but can
                                                           // be set to a lower value for testing purposes.

    enum SPITransactionError : int { kSPITransactionErrorReturnedFalse = -1, kSPITransactionErrorLengthIncorrect = -2 };

    struct PicoConfig {
        uint16_t spi_handshake_pin = bsp.kSubGIRQPin;  // Pin used to signal the ESP32 that a transaction is ready.

        uint16_t subg_led_pin = bsp.kSubGLEDPin;  // Pin used to blink the network LED on successful transactions.
    };

    Pico(PicoConfig config_in);
    ~Pico();

    bool Init();
    bool DeInit();

    /**
     * Helper function used by callbacks to set the handshake pin high or low.
     */
    inline void SetSPIHandshakePinLevel(bool level) {
        // Only set the handshake pin HI when we know we want to solicit a response and not block + wait.
        // Handshake pin is active LO.

        GPIO_write(config_.spi_handshake_pin, !level);
    }

    inline void SPIUseHandshakePin(bool level) { use_handshake_pin_ = level; }
    inline bool SPIIsUsingHandshakePin() const { return use_handshake_pin_; }

    bool SPIBeginTransaction();
    void SPIEndTransaction();

    bool SPIWriteNonBlocking(uint8_t *tx_buf, uint16_t len_bytes = SPICoprocessorPacket::kSPITransactionMaxLenBytes);

    bool SPIProcessTransaction();
    void SPIResetTransaction();

#ifndef ON_TI
    /**
     * Function called from the task spawned during Init().
     */
    void SPIReceiveTask();
#endif  // ON_TI

    /**
     * Turns on the network LED for a specified number of milliseconds. Relies on the UpdateNetworkLED() function to
     * turn the LED off.
     * @param[in] blink_duration_ms Number of milliseconds that the LED should stay on for.
     */
    inline void BlinkSubGLED(uint32_t blink_duration_ms = kSubGLEDBlinkDurationMs) {
        // Disable hardware LED via LED_ENABLE setting.
        if (!settings_manager.settings.led_enabled) {
            return;
        }
        BlinkSubGLEDInternal(blink_duration_ms, false);
    }

    /**
     * Forced blink for test fixtures (AT+LED_BLINK): always lights the LED, bypassing the LED_ENABLE setting.
     * @param[in] blink_duration_ms Number of milliseconds that the LED should stay on for.
     */
    inline void ForceBlinkSubGLED(uint32_t blink_duration_ms) { BlinkSubGLEDInternal(blink_duration_ms, true); }

    /**
     * Turns off the network LED if necessary.
     */
    inline void UpdateLED() {
        if (!subg_led_on) return;
        // Force the LED off (vs waiting for blink timer) once hardware LEDs are disabled, so a synced LED_ENABLE=0 is
        // honored as soon as it arrives from the master. Forced blinks are exempt since they intentionally bypass
        // LED_ENABLE.
        bool blink_expired =
            get_time_since_boot_ms() - subg_led_turn_on_timestamp_ms_ > subg_led_blink_duration_ms_;
        if (blink_expired || (!settings_manager.settings.led_enabled && !subg_led_force_)) {
            GPIO_write(config_.subg_led_pin, 0);
            subg_led_on = false;
            subg_led_force_ = false;
            subg_led_blink_duration_ms_ = kSubGLEDBlinkDurationMs;
        }
    }

   private:
    /**
     * Shared implementation for normal and forced sub-GHz LED blinks.
     * @param[in] blink_duration_ms Number of milliseconds that the LED should stay on for.
     * @param[in] force True to bypass the LED_ENABLE setting until this blink expires.
     */
    inline void BlinkSubGLEDInternal(uint32_t blink_duration_ms, bool force) {
        uint32_t now = get_time_since_boot_ms();
        // Later off-deadline wins: don't let a short packet-decode blink cut an in-progress longer blink short.
        if (!subg_led_on ||
            (int32_t)((now + blink_duration_ms) - (subg_led_turn_on_timestamp_ms_ + subg_led_blink_duration_ms_)) > 0) {
            subg_led_turn_on_timestamp_ms_ = now;
            subg_led_blink_duration_ms_ = blink_duration_ms;
        }
        if (force) {
            subg_led_force_ = true;
        }
        GPIO_write(config_.subg_led_pin, 1);
        subg_led_on = true;
    }

    PicoConfig config_;  // Configuration for the RP2040 SPI coprocessor master interface.
                         // SemaphoreHandle_t spi_mutex_;                  // Low level mutex used to guard the SPI
                         // peripheral (don't let multiple
                         //                                                // threads queue packets at the same time).
                         // SemaphoreHandle_t spi_next_transaction_mutex_; // High level mutex used to claim the next
                         // transaction interval.

    volatile uint8_t spi_rx_buf_[kSPITransactionMaxLenBytes];
    volatile uint8_t spi_tx_buf_[kSPITransactionMaxLenBytes];
    volatile SPI_Transaction spi_transaction_ = {
        // SPI transaction used to send and receive data from the SPI peripheral.
        .count = 0,
        .txBuf = (void *)spi_tx_buf_,
        .rxBuf = (void *)spi_rx_buf_,
        .arg = nullptr};

    SPI_Params spi_params_;
    SPI_Handle spi_handle_ =
        nullptr;  // Handle to the SPI peripheral used by the RP2040 SPI coprocessor master interface.
    // static SPI_Transaction spi_transaction_; // SPI transaction used to send and receive data from the SPI
    // peripheral.
    uint16_t spi_transaction_len_bytes_ = 0;  // Length of the pending SPI transaction in bytes. Allows the callback
                                              // function to figure out how long the transaction was supposed to be.

    bool spi_receive_task_should_exit_ = false;  // Flag used to tell SPI receive task to exit.
    uint32_t subg_led_turn_on_timestamp_ms_ = 0;
    uint32_t subg_led_blink_duration_ms_ = kSubGLEDBlinkDurationMs;
    bool subg_led_on = false;
    bool subg_led_force_ = false;  // True while a forced blink (bypassing LED_ENABLE) is active.
    bool use_handshake_pin_ =
        false;  // Allow handshake pin toggle to be skipped if waiting for a mesage and not writing to master.
    int last_bytes_transacted_ = 0;  // Used to determine whether the last transaction was successful.
};

extern Pico pico_ll;  // Global instance of the RP2040 SPI coprocessor master interface.