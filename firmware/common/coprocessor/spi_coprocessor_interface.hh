#pragma once

#include "object_dictionary.hh"
#include "spi_coprocessor_packet.hh"

#ifdef ON_PICO
#include <functional>  // For std::function.

#include "bsp.hh"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#elif defined(ON_ESP32)
#include "bsp.hh"
#include "data_structures.hh"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#elif defined(ON_TI)
#include "ti/drivers/SPI.h"
#endif

class SPICoprocessorInterface {
   public:
    static constexpr uint16_t kErrorMessageMaxLen = 500;
    enum ReturnCode : int {
        kOk = 0,  // All good.
        kErrorGeneric = -1,
        kErrorTimeout = -2,
        kErrorHandshakeHigh = -3  // Used on the Master to indicate that a transaction was aborted because the slave was
                                  // tryingto talk at the same time as the master.
    };

    /**
     * Convert a ReturnCode to a human-readable string.
     * @param[in] code The ReturnCode to convert.
     * @retval String representation of the ReturnCode.
     */
    static const char* ReturnCodeToString(ReturnCode code) {
        switch (code) {
            case kOk:
                return "OK";
            case kErrorGeneric:
                return "Generic Error";
            case kErrorTimeout:
                return "Timeout Error";
            case kErrorHandshakeHigh:
                return "Handshake High Error";
            default:
                return "Unknown Error";
        }
    }

    /**
     * Initialize the SPI coprocessor interface.
     * @retval True if initialization was successful, false otherwise.
     */
    virtual bool Init() = 0;

    /**
     * Deinitialize the SPI coprocessor interface.
     * @retval True if deinitialization was successful, false otherwise.
     */
    virtual bool DeInit() = 0;

#ifndef ON_TI
    /**
     * On Pico and ESP32, we can block while writing and ensure that we return the bytes that were read.
     */
    virtual int SPIWriteReadBlocking(uint8_t* tx_buf, uint8_t* rx_buf,
                                     uint16_t len_bytes = SPICoprocessorPacket::kSPITransactionMaxLenBytes,
                                     bool end_transaction = true) = 0;
#else
    /**
     * On CC1312, we can only write and then let the callback complete function read the result. We don't know the
     * number of bytes transmitted until the callback, so all we can return is a bool.
     * @param[in] tx_buf Bytes to transmit.
     * @param[in] len_bytes Number of bytes to transmit.
     * @retval True if transfer was queued successfully, false otherwise.
     */
    virtual bool SPIWriteNonBlocking(uint8_t* tx_buf,
                                     uint16_t len_bytes = SPICoprocessorPacket::kSPITransactionMaxLenBytes) = 0;
#endif
};

/**
 * Abstract interface for interacting with a SPI coprocessor master. For instance, this is used to interact with the
 * RP2040 (coprocessor master) on the ESP32 (coprocessor slave).
 */
class SPICoprocessorMasterInterface : public SPICoprocessorInterface {
   public:
    /**
     * Determine whether the handshake pin will be used during the next SPI transaction to solicit a response.
     * @param[in] level True to set pin high once SPI peripheral is loaded up, false to set pin low.
     */
    virtual inline void SPIUseHandshakePin(bool level) = 0;

    virtual inline void UpdateLED() = 0;

    virtual bool SPIBeginTransaction() = 0;
    virtual void SPIEndTransaction() = 0;
};

class SPICoprocessorSlaveInterface : public SPICoprocessorInterface {
   public:
    // Wait this long after a transmission is complete before allowing the HANDSHAKE line to override the minimum
    // transmit interval timeout. This ensures that we don't double-transmit to the slave before it has a chance to
    // lower the HANDSHAKE line following a transaction.
    static constexpr uint32_t kDefaultSPIHandshakeLockoutUs = 10;
    // How long a blocking wait for a handshake can last.
    static constexpr uint32_t kSPIHandshakeTimeoutMs = 100;
    // How long to loop in Update() for after initializing the device in order to allow it to query for settings data.
    static constexpr uint32_t kBootupDelayMs = 500;

    virtual bool Update() = 0;

    /**
     * Checks the level of the HANDSHAKE pin used to initiate communication from the ESP32 to RP2040.
     * NOTE: There is some hysteresis! The ESP32 can request a transfer as soon as kSPIPostTransmitLockoutUs is up, but
     * this function won't unblock and confidently state that the HANDSHAKE pin is not asserted unless
     * kSPIMinTransmitIntervalUs has elapsed.
     * @param[in] blocking If true, wait until the pin is readable before reading it (e.g. it's been long enough since
     * the end of the last transaction that the ESP32 has been able to assert or de-assert the HANDSHAKE pin as
     * necessary). If false, return false if kSPIPostTransmitLockoutUs has not elapsed, otherwise return the HANDSHAKE
     * pin state.
     */
    virtual bool SPIGetHandshakePinLevel() = 0;

    /**
     * Blocks on waiting for the handshake pin to go high, until a timeout is reached.
     * @retval True if handshake line went high before timeout, false otherwise.
     */
    inline bool SPIWaitForHandshake() {
        uint32_t wait_begin_timestamp_ms = get_time_since_boot_ms();
        expecting_handshake_ =
            true;  // Set this so that we know we are expecting the handshake line to go high.
                   // Make sure the ESP32 has time to lower the handshake pin after the last transaction.
        while (get_time_since_boot_us() - spi_last_transmit_timestamp_us_ < spi_handshake_lockout_us_) {
            // Wait for the lockout period to expire before checking the handshake pin.
            // This handshake lockout interval is too short to check for a handshake timeout during.
        }
        while (get_time_since_boot_ms() - wait_begin_timestamp_ms < kSPIHandshakeTimeoutMs) {
            // Exit early if handshake goes high. Otherwise check for timeout.
            if (SPIGetHandshakePinLevel()) {
                // Allowed to exit blocking early if ESP32 asserts the HANDSHAKE pin.
                return true;
            }
        }
        expecting_handshake_ = false;  // Reset this so that we don't think we are expecting a handshake.
        return false;                  // Timed out waiting for the handshake pin to go high.
    }

    virtual bool IsEnabled() = 0;
    virtual void SetEnable(bool enabled) = 0;

    virtual bool SPIBeginTransaction() = 0;
    virtual void SPIEndTransaction() = 0;

    uint32_t GetLastUpdateTimestampMs() const { return last_update_timestamp_ms_; }

    uint16_t num_queued_log_messages = 0;                // Number of log messages queued to be read from the slave.
    uint16_t queued_log_messages_packed_size_bytes = 0;  // Size of the pending log messages in bytes.
    uint16_t num_queued_sc_command_requests = 0;         // Number of SCCommand requests queued on the slave.
    // Cached ESP32 Remote ID receiver status bitfield (see RemoteIDManager::Status). Only meaningful for the ESP32
    // coprocessor; left 0 for others. Surfaced to the user via AT+REMOTE_ID?.
    uint16_t remote_id_status = 0;

   protected:
    // Use this flag to indicate whether we are expecting the handshake line to go high. If it is high during a
    // transaction when we aren't expecting it, that means that we are stomping on the slave! Not good.
    bool expecting_handshake_ = false;
    uint64_t spi_last_transmit_timestamp_us_ = 0;  // Timestamp of the end of the last SPI transaction.

    uint32_t spi_handshake_lockout_us_ = kDefaultSPIHandshakeLockoutUs;  // How long to wait after a transaction before
    // allowing the handshake pin to be asserted.

    uint32_t last_update_timestamp_ms_ = 0;  // Timestamp of the last device status update.
};