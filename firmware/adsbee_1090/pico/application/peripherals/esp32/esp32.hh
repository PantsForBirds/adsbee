#pragma once

#include "bsp.hh"
#include "hardware/spi.h"
#include "spi_coprocessor.hh"

// Uncomment this to forward log messages overs SPI from the ESP32.
// #define PULL_ESP32_LOG_MESSAGES

class ESP32 : public SPICoprocessorSlaveInterface {
   public:
    static const uint32_t kBootupDelayMs = 500;  // Delay after enabling the ESP32 before starting comms.
    // How long we wait to start a transaction after the last one is completed. Can be overridden if the handshake line
    // goes high after kSPIHandshakeLockoutUs.
    static constexpr uint32_t kSPIPostTransmitLockoutUs = 2000;
    // Number of dummy bytes clocked when recovering from an unexpected handshake-high condition. A bare CS pulse
    // (no SCLK edges) is insufficient to complete the ESP32's pending spi_slave_transmit; clocking real bytes
    // with CS held low gives the DMA hardware enough edges to declare the transaction done and lower the handshake.
    static constexpr uint16_t kSPIHandshakeHighRecoveryLenBytes = 4;

    struct ESP32Config {
        uint16_t enable_pin = bsp.esp32_enable_pin;
        spi_inst_t* spi_handle = bsp.copro_spi_handle;
        uint16_t spi_cs_pin = bsp.esp32_spi_cs_pin;
        uint16_t spi_handshake_pin = bsp.esp32_spi_handshake_pin;
        gpio_drive_strength spi_drive_strength = bsp.copro_spi_drive_strength;
        bool spi_pullup = bsp.copro_spi_pullup;
        bool spi_pulldown = bsp.copro_spi_pulldown;
    };

    ESP32(ESP32Config config_in);

    bool Init();
    bool DeInit();

    /**
     * Updates the ESP32 peripheral, including querying device status, processing any pending SCCommand requests, and
     * pulling logs (if enabled).
     * @retval True if successful, false if there was an error.
     */
    bool Update();

    inline bool IsEnabled() { return bsp.has_esp32 && enabled_; }
    inline void SetEnable(bool enabled) {
        gpio_put(config_.enable_pin, enabled);
        enabled_ = enabled;
    }

    inline bool SPIBeginTransaction() {
        while (get_time_since_boot_us() - spi_last_transmit_timestamp_us_ < spi_handshake_lockout_us_) {
            // Wait for the lockout period to expire before checking the handshake pin.
            // This handshake lockout interval is too short to check for a handshake timeout during.
        }
        while (get_time_since_boot_us() - spi_last_transmit_timestamp_us_ < kSPIPostTransmitLockoutUs) {
            // Wait for the lockout period to expire before starting a new transaction.
            if (expecting_handshake_ && SPIGetHandshakePinLevel()) {
                // If we are expecting a handshake and the pin is high, we can proceed with the transaction.
                break;
            }
        }
        gpio_put(config_.spi_cs_pin, 0);
        return true;
    }
    inline void SPIEndTransaction() {
        expecting_handshake_ = false;  // Reset the handshake expectation after a transaction.
        gpio_put(config_.spi_cs_pin, 1);
        spi_last_transmit_timestamp_us_ = get_time_since_boot_us();
    }
    bool SPIGetHandshakePinLevel();

    int SPIWriteReadBlocking(uint8_t* tx_buf, uint8_t* rx_buf,
                             uint16_t len_bytes = SPICoprocessorPacket::kSPITransactionMaxLenBytes,
                             bool end_transaction = true);

    /**
     * True while something on the network reads the console: a /console client is connected, or console input came
     * from the network recently (an AT command or OTA session expects its reply). Also true until the first status
     * read, so nothing is lost while the ESP32 boots.
     */
    bool NetworkConsoleHasConsumers() const {
        return num_network_console_clients_ != 0 ||
               get_time_since_boot_ms() - last_network_console_rx_timestamp_ms_ < kNetworkConsoleReplyWindowMs;
    }

    // Console output keeps going to the network this long after the last console input from the network.
    static constexpr uint32_t kNetworkConsoleReplyWindowMs = 10'000;
    int16_t num_network_console_clients_ = -1;  // -1 until the first ESP32 status read.
    uint32_t last_network_console_rx_timestamp_ms_ = 0;

   private:
    ESP32Config config_;
    bool enabled_ = false;
};

extern ESP32 esp32_ll;