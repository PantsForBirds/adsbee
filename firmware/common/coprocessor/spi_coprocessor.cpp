#include "spi_coprocessor.hh"

#include "buffer_utils.hh"

#ifdef ON_PICO
#include "hal.hh"
#elif defined(ON_coprocessor)
#include "adsbee_server.hh"

#endif

bool SPICoprocessor::Init() {
    if (!config_.interface.Init()) {
        CONSOLE_ERROR("SPICoprocessor::Init", "Failed to initialize SPI coprocessor interface.");
        return false;  // Initialization failed.
    }
#ifdef ON_COPRO_MASTER
    // Loop for a period of time to allow the device to query for settings data.
    uint32_t bootup_start_timestamp_ms = get_time_since_boot_ms();
    while (get_time_since_boot_ms() - bootup_start_timestamp_ms < SPICoprocessorSlaveInterface::kBootupDelayMs) {
        // Loop here to service the coprocessor's query for settings information when it starts up.
        Update();
    }
#endif
    return true;
}

bool SPICoprocessor::DeInit() {
    if (!config_.interface.DeInit()) {
        CONSOLE_ERROR("SPICoprocessor::DeInit", "Failed to deinitialize SPI coprocessor.");
        return false;  // Deinitialization failed.
    }

    // Don't deinit pins or SPI peripheral since some are shared by other peripherals.
    return true;
}

bool SPICoprocessor::Update() {
    bool ret = true;
#ifdef ON_COPRO_MASTER
    if (!IsEnabled()) {
        return false;  // Nothing to do.
    }

    uint32_t timestamp_ms = get_time_since_boot_ms();

    if (timestamp_ms - last_update_timestamp_ms_ <= update_interval_ms) {
        return true;  // No need to update yet.
    }
    // Rely on the slave interface to query device status and process SCCommand requests, since behavior varies by
    // device.
    if (!config_.interface.Update()) {
        CONSOLE_ERROR("SPICoprocessor::Update", "Failed to update SPI coprocessor interface.");
        return false;  // Update failed.
    }

    // Process pending SCCommand requests from the slave interface.
    if (config_.interface.num_queued_sc_command_requests > 0) {
        uint16_t num_requests_processed = 0;
        while (num_requests_processed < config_.interface.num_queued_sc_command_requests &&
               num_requests_processed < kMaxNumSCCommandRequestsPerUpdate) {
            // Read SCCommand request from coprocessor.
            ObjectDictionary::SCCommandRequest sc_command_request;
            if (!Read(ObjectDictionary::Address::kAddrSCCommandRequests, sc_command_request)) {
                CONSOLE_ERROR("SPICoprocessor::Update", "Unable to read SCCommand request %d/%d from coprocessor.",
                              num_requests_processed + 1, config_.interface.num_queued_sc_command_requests);
                return false;
            }
            // Execute the request.
            if (!ExecuteSCCommandRequest(sc_command_request)) {
                CONSOLE_ERROR("SPICoprocessor::Update", "Failed to execute SCCommand request %d/%d from coprocessor.",
                              num_requests_processed + 1, config_.interface.num_queued_sc_command_requests);
                return false;
            }

            // Roll the requests queue.
            ObjectDictionary::RollQueueRequest roll_request = {
                .queue_id = ObjectDictionary::QueueID::kQueueIDSCCommandRequests,
                .num_items = 1,
            };
            if (!Write(ObjectDictionary::Address::kAddrRollQueue, roll_request, true)) {
                // Require the roll request to be acknowledged.
                CONSOLE_ERROR("SPICoprocessor::Update", "Unable to roll SCCommand request queue on coprocessor.");
                return false;
            }
            num_requests_processed++;
        }
    }

    // Pull pending log messages from the slave interface.
    if (config_.pull_log_messages && config_.interface.num_queued_log_messages > 0) {
        // Read log messages from coprocessor.
        uint8_t log_messages_buffer[ObjectDictionary::kLogMessageMaxNumChars * ObjectDictionary::kLogMessageQueueDepth];
        if (Read(ObjectDictionary::Address::kAddrLogMessages, log_messages_buffer,
                 config_.interface.queued_log_messages_packed_size_bytes)) {
            // Acknowledge the log messages to clear the queue.

            uint16_t num_messages_pulled = object_dictionary.UnpackLogMessages(
                log_messages_buffer, sizeof(log_messages_buffer), object_dictionary.log_message_queue,
                config_.interface.num_queued_log_messages);

            ObjectDictionary::RollQueueRequest roll_request = {
                .queue_id = ObjectDictionary::QueueID::kQueueIDLogMessages, .num_items = num_messages_pulled};
            if (!Write(ObjectDictionary::Address::kAddrRollQueue, roll_request,
                       true)) {  // Require the roll request to be acknowledged.
                CONSOLE_ERROR("SPICoprocessor::Update", "Failed to roll log messages queue after reading %d messages.",
                              num_messages_pulled);
                return false;
            }

            ObjectDictionary::LogMessage log_message;
            while (object_dictionary.log_message_queue.Dequeue(log_message)) {
                if (strnlen(log_message.message, sizeof(log_message.message)) == 0) {
                    continue;  // Skip empty messages.
                }
                switch (log_message.log_level) {
                    case SettingsManager::LogLevel::kInfo:
                        CONSOLE_INFO("CoProcessor", "%s >> %s", config_.tag_str, log_message.message);
                        break;
                    case SettingsManager::LogLevel::kWarnings:
                        CONSOLE_WARNING("CoProcessor", "%s >> %s", config_.tag_str, log_message.message);
                        break;
                    case SettingsManager::LogLevel::kErrors:
                        CONSOLE_ERROR("CoProcessor", "%s >> %s", config_.tag_str, log_message.message);
                        break;
                    default:
                        CONSOLE_PRINTF("CoProcessor %s >> %s", config_.tag_str, log_message.message);
                        break;
                }
            }
        } else {
            CONSOLE_ERROR("main", "Unable to read log messages from %s.", config_.tag_str);
        }
    }
    last_update_timestamp_ms_ = timestamp_ms;
#elif defined(ON_COPRO_SLAVE) && !defined(ON_TI)
    static uint8_t rx_buf[SPICoprocessorPacket::kSPITransactionMaxLenBytes];
    memset(rx_buf, 0, SPICoprocessorPacket::kSPITransactionMaxLenBytes);

    if (!config_.interface.SPIBeginTransaction()) {
        CONSOLE_ERROR("SPICoprocessor::Update", "Failed to begin SPI transaction.");
        return false;
    }

    config_.interface.SPIUseHandshakePin(false);  // Don't solicit a transfer.
    int16_t bytes_read = SPIReadBlocking(rx_buf);
    if (bytes_read < 0) {
        bool ret = false;
        if (bytes_read != kErrorTimeout) {
            // Non-timeout errors are worth complaining about.
            CONSOLE_ERROR("SPICoprocessor::Update", "SPI read received non-timeout error code %d (%s).", bytes_read,
                          ReturnCodeToString(static_cast<ReturnCode>(bytes_read)));
        } else {
            // Timeout errors are OK and expected.
            ret = true;
        }
        config_.interface.SPIEndTransaction();
        return ret;
    }

    uint8_t cmd = rx_buf[0];
    switch (cmd) {
        case ObjectDictionary::SCCommand::kCmdWriteToSlave:
        case ObjectDictionary::SCCommand::kCmdWriteToSlaveRequireAck: {
            static SPICoprocessorPacket::SCWritePacket write_packet;
            write_packet.ConstructFromBuffer(rx_buf, bytes_read);
            if (!write_packet.IsValid()) {
                CONSOLE_ERROR("SPICoprocessor::Update",
                              "Received unsolicited write to slave with bad checksum, packet length %d Bytes.",
                              bytes_read);
                config_.interface.SPIEndTransaction();
                return false;
            }
            ret =
                object_dictionary.SetBytes(write_packet.addr, write_packet.data, write_packet.len, write_packet.offset);
            bool ack = true;
            if (!ret) {
                CONSOLE_ERROR("SPICoprocessor::Update",
                              "Failed to write data for %d Byte write to slave at address 0x%x with offset %d Bytes.",
                              write_packet.len, write_packet.addr, write_packet.offset);
                ack = false;
            }
            if (cmd == ObjectDictionary::SCCommand::kCmdWriteToSlaveRequireAck) {
                SPISendAck(ack);
            }
            break;
        }
        case ObjectDictionary::SCCommand::kCmdReadFromSlave: {
            static SPICoprocessorPacket::SCReadRequestPacket read_request_packet;
            read_request_packet.ConstructFromBuffer(rx_buf, bytes_read);
            if (!read_request_packet.IsValid()) {
                CONSOLE_ERROR("SPICoprocessor::Update",
                              "Received unsolicited read from slave with bad checksum, packet length %d Bytes.",
                              bytes_read);
                config_.interface.SPIEndTransaction();
                return false;
            }

            static SPICoprocessorPacket::SCResponsePacket response_packet;
            response_packet.cmd = ObjectDictionary::SCCommand::kCmdDataBlock;
            ret = object_dictionary.GetBytes(read_request_packet.addr, response_packet.data, read_request_packet.len,
                                             read_request_packet.offset);
            if (!ret) {
                CONSOLE_ERROR("SPICoprocessor::Update",
                              "Failed to retrieve data for %d Byte read from slave at address 0x%x",
                              read_request_packet.len, read_request_packet.addr);
                config_.interface.SPIEndTransaction();
                return false;
            }
            response_packet.data_len_bytes = read_request_packet.len;  // Assume the correct number of bytes were read.
            response_packet.PopulateCRC();
            config_.interface.SPIUseHandshakePin(true);  // Solicit a read from the RP2040.
            SPIWriteBlocking(response_packet.GetBuf(), response_packet.GetBufLenBytes());
            break;
        }
        default:
            CONSOLE_ERROR("SPICoprocessor::Update",
                          "Received unsolicited packet from RP2040 with unsupported cmd=%d, packet length %d Bytes.",
                          cmd, bytes_read);
            config_.interface.SPIEndTransaction();
            return false;
    }

    config_.interface.SPIEndTransaction();
#elif defined(ON_TI)
    // TI platform doesn't have an RTOS, so we can't block without stalling the processor. Utilize SPI callbacks for
    // primary SPI driver updates. Just update the LED here.
    config_.interface.UpdateLED();
#endif
    return ret;
}

#ifdef ON_COPRO_SLAVE
bool SPICoprocessor::LogMessage(SettingsManager::LogLevel log_level, const char* tag, const char* format,
                                va_list args) {
    // Make the scratch LogMessage static so that we don't need to allocate it all the time.
    // Allocating a LogMessage buffer on the stack can cause overflows in some limited resource event handlers.
    static ObjectDictionary::LogMessage log_message;
    log_message.log_level = log_level;
    log_message.num_chars = 0;
    log_message.message[0] = '\0';  // Initialize to empty string.

    if (strnlen(tag, ObjectDictionary::kLogMessageTagMaxNumChars) > 0) {
        log_message.num_chars += snprintf(log_message.message, ObjectDictionary::kLogMessageMaxNumChars, "[%s] ", tag);
    }

    log_message.num_chars += vsnprintf(log_message.message + log_message.num_chars,
                                       ObjectDictionary::kLogMessageMaxNumChars - log_message.num_chars, format, args);

    return object_dictionary.log_message_queue.Enqueue(log_message);
}
#endif

#ifdef ON_COPRO_MASTER

bool SPICoprocessor::ExecuteSCCommandRequest(const ObjectDictionary::SCCommandRequest& request) {
    bool write_requires_ack = false;
    switch (request.command) {
        case ObjectDictionary::SCCommand::kCmdWriteToSlaveRequireAck:
            write_requires_ack = true;
            [[fallthrough]];
        case ObjectDictionary::SCCommand::kCmdWriteToSlave: {
            if (request.len == 0) {
                CONSOLE_WARNING("SPICoprocessor::ExecuteSCCommandRequest",
                                "Skipping write request to address 0x%x with zero length.", request.addr);
                return true;
            }
            switch (request.addr) {
                /** These are the addresses that the coprocessor can request a write to. **/
                case ObjectDictionary::Address::kAddrSettingsData: {
                    if (request.offset != 0) {
                        CONSOLE_ERROR("SPICoprocessor::ExecuteSCCommandRequest",
                                      "Settings data write with non-zero offset (%d) not supported.", request.offset);
                        return false;
                    }
                    // Write settings data to coprocessor.
                    if (request.len != sizeof(settings_manager.settings)) {
                        CONSOLE_ERROR("SPICoprocessor::ExecuteSCCommandRequest",
                                      "Settings data write with invalid length (%d). Expected %d. The coprocessor's "
                                      "firmware uses a different Settings layout; reflash it to match this firmware.",
                                      request.len, sizeof(settings_manager.settings));
                        return false;
                    }
                    if (!Write(request.addr, settings_manager.settings, write_requires_ack)) {
                        CONSOLE_ERROR("SPICoprocessor::ExecuteSCCommandRequest",
                                      "Unable to write settings data to coprocessor.");
                        return false;
                    }
                    break;  // Successfully wrote settings data to coprocessor.
                }
                default:
                    CONSOLE_ERROR("SPICoprocessor::ExecuteSCCommandRequest",
                                  "No implementation defined for writing to address 0x%x on slave.", request.addr);
                    return false;
            }
            break;
        }

        case ObjectDictionary::SCCommand::kCmdReadFromSlave: {
            if (request.len == 0) {
                CONSOLE_WARNING("SPICoprocessor::ExecuteSCCommandRequest",
                                "Skipping read request to address 0x%x with zero length.", request.addr);
                return true;
            }
            switch (request.addr) {
                /**  These are the addresses the coprocessor can request a read from. **/
                default:
                    CONSOLE_ERROR("SPICoprocessor::ExecuteSCCommandRequest",
                                  "No implementation defined for reading from address 0x%x on slave.", request.addr);
                    return false;
            }
            break;
        }
        default:
            CONSOLE_ERROR("SPICoprocessor::ExecuteSCCommandRequest",
                          "Unsupported SCCommand received from coprocessor: %d.", request.command);
            return false;
    }
    return true;
}

bool SPICoprocessor::PartialWrite(ObjectDictionary::Address addr, uint8_t* object_buf, uint16_t len, uint16_t offset,
                                  bool require_ack) {
    // Guard against re-entrant calls: CONSOLE_WARNING() inside this function can trigger UpdateNetworkConsole() ->
    // esp32.Write() -> PartialWrite(), which would overwrite the shared static write_packet mid-retry.
    static bool spi_write_in_progress = false;
    static SPICoprocessorPacket::SCWritePacket write_packet;

    if (spi_write_in_progress) {
        return false;
    }
    spi_write_in_progress = true;

    write_packet.cmd = require_ack ? ObjectDictionary::SCCommand::kCmdWriteToSlaveRequireAck
                                   : ObjectDictionary::SCCommand::kCmdWriteToSlave;

    write_packet.addr = addr;
    memcpy(write_packet.data, object_buf + offset, len);
    write_packet.len = len;
    write_packet.offset = offset;
    write_packet.PopulateCRC();

    int num_attempts = 0;
    char error_message[kErrorMessageMaxLen + 1] = "No error.";
    error_message[kErrorMessageMaxLen] = '\0';
    bool ret = true;
    while (num_attempts < kSPITransactionMaxNumRetries) {
        // Don't end the transaction yet to allow recovery of packets from kErrorHandshakeHigh.
        int bytes_written = SPIWriteBlocking(write_packet.GetBuf(), write_packet.GetBufLenBytes(), true);

        if (bytes_written < 0) {
            snprintf(error_message, kErrorMessageMaxLen, "Error code %d (%s) while writing object over SPI.",
                     bytes_written, ReturnCodeToString(static_cast<ReturnCode>(bytes_written)));

            goto PARTIAL_WRITE_FAILED;
        }
        if (require_ack && !SPIWaitForAck()) {
            snprintf(error_message, kErrorMessageMaxLen, "Timed out or received bad ack after writing to coprocessor.");
            goto PARTIAL_WRITE_FAILED;
        }
        // Completed successfully!
        ret = true;
        break;
    PARTIAL_WRITE_FAILED:
        CONSOLE_WARNING("SPICoprocessor::PartialWrite", "[%s] %s", config_.tag_str, error_message);
        num_attempts++;
        ret = false;
        continue;
    }
    if (!ret) {
        CONSOLE_ERROR("SPICoprocessor::PartialWrite", "[%s] Failed after %d tries: %s", config_.tag_str, num_attempts,
                      error_message);
    }
    spi_write_in_progress = false;
    return ret;
}

bool SPICoprocessor::PartialRead(ObjectDictionary::Address addr, uint8_t* object_buf, uint16_t len, uint16_t offset) {
    static bool spi_read_in_progress = false;
    static SPICoprocessorPacket::SCReadRequestPacket read_request_packet;

    if (spi_read_in_progress) {
        return false;
    }
    spi_read_in_progress = true;

    read_request_packet.cmd = ObjectDictionary::SCCommand::kCmdReadFromSlave;

    read_request_packet.addr = addr;
    read_request_packet.offset = offset;
    read_request_packet.len = len;
    read_request_packet.PopulateCRC();

    static uint8_t rx_buf[SPICoprocessorPacket::kSPITransactionMaxLenBytes];

    uint16_t read_request_bytes = read_request_packet.GetBufLenBytes();

    int num_attempts = 0;
    char error_message[kErrorMessageMaxLen + 1] = "No error.";
    error_message[kErrorMessageMaxLen] = '\0';
    bool ret = true;
    static SPICoprocessorPacket::SCResponsePacket response_packet;
    while (num_attempts < kSPITransactionMaxNumRetries) {
        // On the master, reading from the slave is two transactions: The read request is sent, then we wait on the
        // handshake line to read the reply.

        // Don't end transaction in order to allow recovery from kErrorHandshakeHigh error.
        int bytes_written = SPIWriteBlocking(read_request_packet.GetBuf(), read_request_bytes, true);
        int bytes_read = 0;
        if (bytes_written < 0) {
            snprintf(error_message, kErrorMessageMaxLen, "Error code %d (%s) while writing read request over SPI.",
                     bytes_written, ReturnCodeToString(static_cast<ReturnCode>(bytes_written)));
            goto PARTIAL_READ_FAILED;
        }
        if (!config_.interface.SPIWaitForHandshake()) {
            snprintf(error_message, kErrorMessageMaxLen,
                     "Timed out while waiting for handshake after sending read request.");
            goto PARTIAL_READ_FAILED;
        }

        response_packet.data_len_bytes =
            len;  // We need to set this manually since we are using the default constructor.
        bytes_read = SPIReadBlocking(response_packet.GetBuf(),
                                     SPICoprocessorPacket::SCResponsePacket::GetBufLenForPayloadLenBytes(len));
        if (bytes_read < 0) {
            snprintf(error_message, kErrorMessageMaxLen, "Error code %d (%s) while reading read response over SPI.",
                     bytes_read, ReturnCodeToString(static_cast<ReturnCode>(bytes_written)));
            goto PARTIAL_READ_FAILED;
        }

        if (!response_packet.IsValid()) {
            snprintf(error_message, kErrorMessageMaxLen,
                     "Received response packet of length %d Bytes with an invalid CRC.",
                     response_packet.GetBufLenBytes());
            goto PARTIAL_READ_FAILED;
        }
        if (response_packet.cmd != ObjectDictionary::SCCommand::kCmdDataBlock) {
            snprintf(error_message, kErrorMessageMaxLen,
                     "Received invalid response with cmd=0x%x to requested read at address 0x%x of length %d with "
                     "offset %d Bytes.",
                     response_packet.cmd, read_request_packet.addr, read_request_packet.len,
                     read_request_packet.offset);
            goto PARTIAL_READ_FAILED;
        }
        if (response_packet.data_len_bytes != len) {
            snprintf(error_message, kErrorMessageMaxLen,
                     "Received incorrect number of Bytes while reading object at address 0x%x with offset %d "
                     "Bytes. Requested %d Bytes but received %d.",
                     addr, offset, read_request_packet.len, response_packet.data_len_bytes);
            goto PARTIAL_READ_FAILED;
        }
        // Completed successfully!
        ret = true;
        memcpy(object_buf + offset, response_packet.data, response_packet.data_len_bytes);
        break;
    PARTIAL_READ_FAILED:
        CONSOLE_WARNING("SPICoprocessor::PartialRead", "[%s] %s", config_.tag_str, error_message);
        num_attempts++;
        ret = false;
        continue;
    }

    if (!ret) {
        CONSOLE_ERROR("SPICoprocessor::PartialRead", "[%s] Failed after %d tries: %s", config_.tag_str, num_attempts,
                      error_message);
    }
    spi_read_in_progress = false;
    return ret;
}

bool SPICoprocessor::SPIWaitForAck() {
    static bool ack_wait_in_progress = false;
    static SPICoprocessorPacket::SCAckPacket ack_packet;

    if (ack_wait_in_progress) {
        return false;
    }
    ack_wait_in_progress = true;

    if (!config_.interface.SPIWaitForHandshake()) {
        CONSOLE_ERROR("SPICoprocessor::SPIWaitForAck", "[%s] Timed out while waiting for ack: never received handshake.",
                      config_.tag_str);
        ack_wait_in_progress = false;
        return false;
    }

    // We need to call SPIReadBlocking without ending the transaction in case we want to recover a
    // kErrorHandshakeHigh error.
    int bytes_read =
        SPIReadBlocking(ack_packet.GetBuf(), SPICoprocessorPacket::SCAckPacket::kBufLenBytes, true);

    if (bytes_read < 0) {
        CONSOLE_ERROR("SPICoprocessor::SPIWaitForAck", "[%s] SPI read failed with code %d (%s).", config_.tag_str,
                      bytes_read, ReturnCodeToString(static_cast<ReturnCode>(bytes_read)));
        ack_wait_in_progress = false;
        return false;
    }
    if (ack_packet.cmd != ObjectDictionary::SCCommand::kCmdAck) {
        CONSOLE_ERROR("SPICoprocessor::SPIWaitForAck",
                      "[%s] Received a message that was not an ack (cmd=0x%x, expected 0x%x).", config_.tag_str,
                      ack_packet.cmd, ObjectDictionary::SCCommand::kCmdAck);
        ack_wait_in_progress = false;
        return false;
    }
    if (!ack_packet.IsValid()) {
        CONSOLE_ERROR("SPICoprocessor::SPIWaitForAck", "[%s] Received a response packet with an invalid CRC.",
                      config_.tag_str);
        ack_wait_in_progress = false;
        return false;
    }
    ack_wait_in_progress = false;
    return ack_packet.ack;
}

#endif

#ifdef ON_COPRO_SLAVE
bool SPICoprocessor::SPISendAck(bool success) {
    static SPICoprocessorPacket::SCAckPacket ack_packet;
    ack_packet.ack = success;
    ack_packet.PopulateCRC();
    config_.interface.SPIUseHandshakePin(true);  // Solicit a transfer to send the ack.
#ifndef ON_TI
    return SPIWriteBlocking(ack_packet.GetBuf(), SPICoprocessorPacket::SCAckPacket::kBufLenBytes) > 0;
#else
    return SPIWriteNonBlocking(ack_packet.GetBuf(), SPICoprocessorPacket::SCAckPacket::kBufLenBytes) > 0;
#endif  // ON_TI
}
#endif  // ON_COPRO_SLAVE