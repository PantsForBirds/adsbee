#ifndef SPI_COPROCESSOR_HH_
#define SPI_COPROCESSOR_HH_

#include <cstdarg>  // For va_list.

#ifndef ON_TI
#include "aircraft_dictionary.hh"
#include "mode_s_packet.hh"
#endif
#include "comms.hh"
#include "hal.hh"
#include "macros.hh"
#include "object_dictionary.hh"
#include "settings.hh"
#include "spi_coprocessor_interface.hh"
#include "spi_coprocessor_packet.hh"

class SPICoprocessor : public SPICoprocessorInterface {
   public:
    static constexpr uint16_t kSPITransactionQueueLenTransactions = 3;

    // Max num retries per block in a multi-transfer transaction.
    static constexpr uint16_t kSPITransactionMaxNumRetries = 3;
    static const uint16_t kTagStrMaxLen = 32;
    static const uint16_t kMaxNumSCCommandRequestsPerUpdate = 5;
    static const uint16_t kDefaultUpdateIntervalMs = 100;  // 10Hz updates by default.

    struct SPICoprocessorConfig {
#ifdef ON_COPRO_MASTER
        SPICoprocessorSlaveInterface& interface;  // Reference to the slave interface.
        char tag_str[kTagStrMaxLen] = {0};        // Tag to prepend to console messages.
        bool pull_log_messages = true;            // Whether to pull log messages from the slave.
#elif defined(ON_COPRO_SLAVE)
        SPICoprocessorMasterInterface& interface;  // Reference to the master interface.
#else
        SPICoprocessorInterface& interface;  // Reference to the interface. Not used on other platforms.
#endif
    };

    /**
     * Constructor
     */
    SPICoprocessor(SPICoprocessorConfig config_in)
        : config_(config_in) {

          };

    bool Init();
    bool DeInit();

#ifdef ON_COPRO_MASTER
    inline bool IsEnabled() { return config_.interface.IsEnabled(); }
    inline void SetEnable(bool enabled) { config_.interface.SetEnable(enabled); }

    // Set when a boot-time firmware update of this coprocessor failed, so it runs mismatched firmware (or none) and was
    // disabled. It stays disabled for the rest of this boot, but the disable is not persisted to settings: the next
    // boot retries the update. Cleared by a successful manual reflash (e.g. AT+ESP32_FLASH).
    bool firmware_update_failed = false;

    /**
     * Gets the timestamp of the last successful device status query from the ESP32.
     * @retval Timestamp in milliseconds since boot.
     */
    inline uint32_t GetLastHeartbeatTimestampMs() { return last_update_timestamp_ms_; }

    /**
     * Top level function that translates a write to an object (with associated address) into SPI transaction(s).
     * Included in the header file since the template function implementation needs to be visible to any file that
     * utilizes it.
     */
    template <typename T>
    bool Write(ObjectDictionary::Address addr, T& object, bool require_ack = false, uint16_t len_bytes = 0) {
        if (len_bytes == 0) {
            len_bytes = sizeof(object);
        }
        if (len_bytes < SPICoprocessorPacket::SCWritePacket::kDataMaxLenBytes) {
            // Single write. Write the full object at once, no offset, require ack if necessary.
            bool ret = PartialWrite(addr, (uint8_t*)&object, len_bytes, 0, require_ack);
            return ret;
        }
        // Multi write.
        int16_t bytes_remaining = len_bytes;
        while (bytes_remaining > 0) {
            if (!PartialWrite(addr,                                                                         // addr
                              (uint8_t*)(&object),                                                          // object
                              MIN(SPICoprocessorPacket::SCWritePacket::kDataMaxLenBytes, bytes_remaining),  // len
                              len_bytes - bytes_remaining,                                                  // offset
                              require_ack)  // require_ack
            ) {
                CONSOLE_ERROR("SPICoprocessor::Write",
                              "Multi-transfer %d Byte write of object at address 0x%x failed with %d Bytes remaining.",
                              len_bytes, addr, bytes_remaining);
                return false;
            }
            bytes_remaining -= SPICoprocessorPacket::SCWritePacket::kDataMaxLenBytes;
        }
        // No bytes remaining.
        return true;
    }

    /**
     * Top level function that translates a read from an object (with associated address) into SPI transaction(s).
     * Included in the header file since the template function implementation needs to be visible to any file that
     * utilizes it.
     */
    template <typename T>
    bool Read(ObjectDictionary::Address addr, T& object, uint16_t len_bytes = 0) {
        if (len_bytes == 0) {
            len_bytes = sizeof(object);
        }
        if (len_bytes < SPICoprocessorPacket::SCResponsePacket::kDataMaxLenBytes) {
            // Single read.
            bool ret = PartialRead(addr, (uint8_t*)&object, len_bytes);
            return ret;
        }
        // Multi-read.
        // Write and read are separate transactions.
        uint16_t max_chunk_size_bytes = SPICoprocessorPacket::SCResponsePacket::kDataMaxLenBytes;

        int16_t bytes_remaining = len_bytes;
        while (bytes_remaining > 0) {
            if (!PartialRead(addr,                                        // address
                             (uint8_t*)(&object),                         // object
                             MIN(max_chunk_size_bytes, bytes_remaining),  // len
                             len_bytes - bytes_remaining)                 // offset
            ) {
                CONSOLE_ERROR("SPICoprocessor::Read",
                              "Multi-transfer %d Byte read of object at address 0x%x failed with %d Bytes remaining.",
                              len_bytes, addr, bytes_remaining);
                return false;
            }
            bytes_remaining -=
                max_chunk_size_bytes;  // Overshoot on purpose on the last chunk. Bytes remaining will go negative.
        }
        // No bytes remaining.
        return true;
    }

#ifdef HARDWARE_UNIT_TESTS
    bool TestSPIHandshakeDeadlock();
    bool TestSPIPersistentDesync();
#endif
#endif  // ON_COPRO_MASTER (public methods)
    /**
     * Updates the SPI coprocessor interface. On the master, this will read from the slave and process any requests the
     * slave has queued. On the slave, this will process incoming transactions from the master and reply if necessary
     * (the slave does not initiate any unsolicited transactions).
     */
    bool Update();

    // Interval for device status updates.
    uint32_t update_interval_ms = kDefaultUpdateIntervalMs;

#ifdef ON_COPRO_SLAVE

    void UpdateLED() { config_.interface.UpdateLED(); }

    /**
     * Log a message to the coprocessor. Not available on RP2040 since it's the master (other stuff logs to it).
     * @param[in] log_level Log level of the message.
     * @param[in] tag Tag to prepend to the message.
     * @param[in] format Format string for the message.
     * @param[in] ... Variable arguments for the format string.
     */
    bool LogMessage(SettingsManager::LogLevel log_level, const char* tag, const char* format, std::va_list args);

    /**
     * Send an SCResponse packet with a single byte ACK payload.
     * @param[in] success True if sending an ACK, false if sending a NACK.
     * @retval True if ACK was transmitted successfully, false if something went wrong.
     */
    bool SPISendAck(bool success);
#endif

   protected:
#ifdef ON_COPRO_MASTER
    /**
     * Executes a single SCCommand request received from the coprocessor.
     * @param[in] request The SCCommandRequest to execute.
     * @retval True if the command was executed successfully, false otherwise.
     */
    bool ExecuteSCCommandRequest(const ObjectDictionary::SCCommandRequest& request);

    bool PartialWrite(ObjectDictionary::Address addr, uint8_t* object_buf, uint16_t len, uint16_t offset = 0,
                      bool require_ack = false);

    bool PartialRead(ObjectDictionary::Address addr, uint8_t* object_buf, uint16_t len, uint16_t offset = 0);

    /**
     * Blocks until an ACK is received or a timeout is reached.
     * @retval True if received an ACK, false if received NACK or timed out.
     */
    bool SPIWaitForAck();

    uint32_t last_update_timestamp_ms_ = 0;  // Timestamp of the last device status update.
#endif                                       // ON_COPRO_MASTER

#ifndef ON_TI
    /**
     * Low level HAL for SPI Write Read call. Transmits the contents of tx_buf and receives into rx_buf.
     * Both buffers MUST be at least SPICoprocessorPacket::kSPITransactionMaxLenBytes Bytes long.
     * @param[in] tx_buf Buffer with data to transmit.
     * @param[in] rx_buf Buffer to fill with data that is received.
     * @param[in] len_bytes Number of bytes to transmit. Only has an effect when this function is being called on the
     * master.
     * @param[in] end_transaction Whether to de-assert chip select at the end of the transaction. Only has an effect
     * when this function is being called on the master.
     * @retval Number of bytes that were written and read, or a negative value if something broke.
     */
    int SPIWriteReadBlocking(uint8_t* tx_buf, uint8_t* rx_buf,
                             uint16_t len_bytes = SPICoprocessorPacket::kSPITransactionMaxLenBytes,
                             bool end_transaction = true) {
        return config_.interface.SPIWriteReadBlocking(tx_buf, rx_buf, len_bytes, end_transaction);
    }

    // Write / Read aliases for SPIWriteReadBlocking.
    inline int SPIWriteBlocking(uint8_t* tx_buf, uint16_t len_bytes = SPICoprocessorPacket::kSPITransactionMaxLenBytes,
                                bool end_transaction = true) {
        return SPIWriteReadBlocking(tx_buf, nullptr, len_bytes, end_transaction);
    }
    inline int SPIReadBlocking(uint8_t* rx_buf, uint16_t len_bytes = SPICoprocessorPacket::kSPITransactionMaxLenBytes,
                               bool end_transaction = true) {
        return SPIWriteReadBlocking(nullptr, rx_buf, len_bytes, end_transaction);
    }
#else
    // On the CC1312, we can only queue up a write and let the callback handle the bytes that were read.
    bool SPIWriteNonBlocking(uint8_t* tx_buf, uint16_t len_bytes = SPICoprocessorPacket::kSPITransactionMaxLenBytes) {
        return config_.interface.SPIWriteNonBlocking(tx_buf, len_bytes);
    }
#endif  // ON_TI

    SPICoprocessorConfig config_;
};

#ifdef ON_COPRO_MASTER
extern SPICoprocessor esp32;
extern SPICoprocessor cc1312;
#elif defined(ON_COPRO_SLAVE)
extern SPICoprocessor pico;
#endif

#endif /* SPI_COPROCESSOR_HH_ */