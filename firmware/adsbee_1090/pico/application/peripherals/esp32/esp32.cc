#include "esp32.hh"

#include "adsbee.hh"  // Get access to the Sub-GHz radio for its status.
#include "composite_array.hh"  // For pulling Remote ID packets forwarded up from the ESP32.
#include "cpu_utils.hh"
#include "gnss_interface.hh"
#include "hal.hh"

extern CPUMonitor core_0_monitor;
extern CPUMonitor core_1_monitor;

ESP32::ESP32(ESP32Config config_in) : config_(config_in) {}

bool ESP32::Init() {
    // ESP32 enable pin.
    gpio_init(config_.enable_pin);
    gpio_set_dir(config_.enable_pin, GPIO_OUT);
    SetEnable(true);
    // ESP32 chip select pin.
    gpio_init(config_.spi_cs_pin);
    gpio_set_dir(config_.spi_cs_pin, GPIO_OUT);
    gpio_put(config_.spi_cs_pin, 0);
    // ESP32 handshake pin.
    gpio_init(config_.spi_handshake_pin);
    gpio_set_dir(config_.spi_handshake_pin, GPIO_IN);
    gpio_set_pulls(config_.spi_handshake_pin, true,
                   false);  // Handshake pin is pulled up to not enter bootloader on startup.
    // Handshake line being pulled up results in false positive handshakes during startup, but this only happens during
    // bootup.

    // Require SPI pins to be initialized before this function is called, since they get shared.
    gpio_set_drive_strength(config_.spi_cs_pin, config_.spi_drive_strength);
    gpio_set_pulls(config_.spi_cs_pin, config_.spi_pullup, config_.spi_pulldown);  // CS pin pulls.

    // Delay for kBootupDelayMs to avoid a bunch of failed transactions while the ESP32 wakes up.
    uint32_t boot_start_timestamp_ms = get_time_since_boot_ms();
    while (get_time_since_boot_ms() - boot_start_timestamp_ms < kBootupDelayMs) {
    }

    return true;
}

bool ESP32::DeInit() {
    // ESP32 enable pin.
    SetEnable(false);
    return true;
}

bool ESP32::Update() {
    // IsEnabled() check happens at the higher level Update() function in SPICoprocessor, so don't repeat it here.

    // Query ESP32's device status.
    ObjectDictionary::ESP32DeviceStatus device_status;
    if (esp32.Read(ObjectDictionary::Address::kAddrDeviceStatus, device_status)) {
        // We only update the device_status vars exposed publicly here. Other reads of device_status are for
        // internal use only.
        num_queued_log_messages = device_status.num_queued_log_messages;
        queued_log_messages_packed_size_bytes = device_status.queued_log_messages_packed_size_bytes;
        num_queued_sc_command_requests = device_status.num_queued_sc_command_requests;
        remote_id_status = device_status.remote_id_status;
    } else {
        CONSOLE_ERROR("ESP32::Update", "Unable to read ESP32 status.");
        return false;
    }

    static const uint16_t kMaxNumConsoleReadsPerUpdate = 5;
    for (uint16_t console_read_num = 0;
         console_read_num < kMaxNumConsoleReadsPerUpdate && device_status.num_queued_network_console_rx_chars > 0;
         console_read_num++) {
        // Read console message from ESP32.
        char buf[ObjectDictionary::kNetworkConsoleMessageMaxLenBytes] = {0};

        // Read as many bytes as we can without overflowing the RX queue or exceeding our buffer size or max SPI
        // transaction size.
        uint16_t bytes_to_read =
            MIN(comms_manager.esp32_console_rx_queue.MaxNumElements() - comms_manager.esp32_console_rx_queue.Length(),
                MIN(device_status.num_queued_network_console_rx_chars, sizeof(buf)));
        if (bytes_to_read == 0) {
            // Nasty failure mode: If you request a write with buf length zero, the Read() function defaults to the
            // size of the object passed in, so you could be reading for a quite a while! Catch it here.
            break;  // No more console messages to read.
        }

        if (!esp32.Read(ObjectDictionary::Address::kAddrConsole, buf, bytes_to_read)) {
            CONSOLE_ERROR("ESP32::ExecuteSCCommandRequest", "Unable to read console message from ESP32.");
            return false;
        }
        for (uint16_t i = 0; i < bytes_to_read; i++) {
            char c = (char)buf[i];
            if (!comms_manager.esp32_console_rx_queue.Enqueue(c)) {
                CONSOLE_ERROR("ObjectDictionary::SetBytes", "ESP32 overflowed RP2040's network console queue.");
                comms_manager.esp32_console_rx_queue.Clear();
                return false;
            }
        }
        ObjectDictionary::RollQueueRequest roll_request = {
            .queue_id = ObjectDictionary::QueueID::kQueueIDConsole,
            .num_items = (uint16_t)bytes_to_read,
        };
        if (!esp32.Write(ObjectDictionary::Address::kAddrRollQueue, roll_request, true)) {
            // Require the roll request to be acknowledged.
            CONSOLE_ERROR("ESP32::Update", "Unable to roll console queue after reading by %d bytes on ESP32.",
                          bytes_to_read);
            return false;
        }
        if (!esp32.Read(ObjectDictionary::Address::kAddrDeviceStatus, device_status)) {
            CONSOLE_ERROR("ESP32::Update", "Failed to re-read ESP32 device status on console read %d/%d.",
                          console_read_num + 1, kMaxNumConsoleReadsPerUpdate);
            return false;
        }
        // Successfully read console message from ESP32.
    }

    // Pull any Broadcast Remote ID packets the ESP32 received over BLE/WiFi and enqueue them for the main loop to decode
    // into the aircraft dictionary (which drives serial reporting). Mirrors the CC1312 UAT pull in CC1312::Update().
    if (device_status.pending_raw_packets_len_bytes > sizeof(CompositeArray::RawPackets::Header)) {
        uint8_t read_buf[CompositeArray::RawPackets::kMaxLenBytes] = {0};
        if (!esp32.Read(ObjectDictionary::Address::kAddrCompositeArrayRawPackets, read_buf,
                        device_status.pending_raw_packets_len_bytes)) {
            CONSOLE_ERROR("ESP32::Update", "Unable to read Remote ID raw packet array from ESP32.");
            return false;
        }
        if (!CompositeArray::UnpackRawPacketsBufferToQueues(read_buf, sizeof(read_buf), nullptr, nullptr, nullptr,
                                                            &adsbee.raw_remote_id_packet_queue)) {
            CONSOLE_ERROR("ESP32::Update", "Failed to unpack Remote ID packets from ESP32.");
            return false;
        }
    }

    // Send the RP2040's status to the ESP32.
    const NMEAParser::GNSSFix& gnss_fix = gnss->fix();
    ObjectDictionary::CompositeDeviceStatus composite_status = {
        .rp2040 =
            {
                .timestamp_ms = get_time_since_boot_ms(),
                .temperature_deg_c = (int8_t)(core_0_monitor.ReadTemperatureDegC()),
                .core_0_usage_percent = core_0_monitor.GetUsagePercent(),
                .core_1_usage_percent = core_1_monitor.GetUsagePercent(),
                .rx_position = adsbee.rx_position,
                .rx_position_available = adsbee.rx_position_available,
                .gnss_enabled = gnss->IsActive(),
                .gnss_fix_valid = gnss->HasValidFix(),
                .gnss_latitude_deg = gnss_fix.latitude_deg,
                .gnss_longitude_deg = gnss_fix.longitude_deg,
                .gnss_utc_time_valid = gnss_fix.utc_time_valid,
                .gnss_utc_hour = gnss_fix.utc_hour,
                .gnss_utc_minute = gnss_fix.utc_minute,
                .gnss_utc_second = gnss_fix.utc_second,
                .gnss_utc_millisecond = gnss_fix.utc_millisecond,
                .noise_floor_mv = static_cast<uint16_t>(adsbee.GetNoiseFloorMilliVolts()),
                .noise_floor_dbm = static_cast<int16_t>(adsbee.GetNoiseFloordBm()),
            },
        .subg = adsbee.subg_radio_ll.device_status,
    };
    if (!esp32.Write(ObjectDictionary::Address::kAddrDeviceStatus, composite_status, true)) {
        CONSOLE_ERROR("ESP32::Update", "Unable to write RP2040 and SubGHz radio device status to ESP32.");
        return false;
    }

    last_update_timestamp_ms_ = get_time_since_boot_ms();
    return true;
}

bool ESP32::SPIGetHandshakePinLevel() { return gpio_get(config_.spi_handshake_pin); }

int ESP32::SPIWriteReadBlocking(uint8_t* tx_buf, uint8_t* rx_buf, uint16_t len_bytes, bool end_transaction) {
    int bytes_written = 0;
#ifdef ON_PICO
    SPIBeginTransaction();  // Begin SPI transaction, which sets the CS pin low.
    // Blocking check of handshake line. If we're expecting a handshake, it's OK for the line to be high. Otherwise, we
    // need to bail out to not stomp on the ESP32's incoming message.
    if (SPIGetHandshakePinLevel() && !expecting_handshake_) {
        // Clock dummy bytes so the ESP32's pending spi_slave_transmit sees real SCLK edges and completes,
        // lowering the handshake. A bare CS pulse without clock edges is not guaranteed to unblock DMA.
        uint8_t dummy[kSPIHandshakeHighRecoveryLenBytes];
        spi_read_blocking(config_.spi_handle, 0x0, dummy, sizeof(dummy));
        SPIEndTransaction();
        return ReturnCode::kErrorHandshakeHigh;
    }
    // Pico SDK doesn't have nice nullptr behavior for tx_buf and rx_buf, so we have to do this.
    if (tx_buf == nullptr) {
        // Transmit 0's when reading.
        bytes_written = spi_read_blocking(config_.spi_handle, 0x0, rx_buf, len_bytes);
    } else if (rx_buf == nullptr) {
        bytes_written = spi_write_blocking(config_.spi_handle, tx_buf, len_bytes);
    } else {
        bytes_written = spi_write_read_blocking(config_.spi_handle, tx_buf, rx_buf, len_bytes);
    }

    if (end_transaction) {
        SPIEndTransaction();
        // Only the last transfer chunk of the transaction is used to record the last transmission timestamp. This stops
        // transactions from getting too long as earlier chunks reset the lockout timer for later chungs, e.g. if we
        // only read one byte we don't want to wait for the timeout before conducting the rest of the transaction.
    }
    if (bytes_written < 0) {
        CONSOLE_ERROR("ESP32::SPIWriteReadBlocking", "SPI write read call returned error code 0x%x.", bytes_written);
    }
#elif defined(ON_ESP32)
    bytes_written = config_.interface.SPIWriteReadBlocking(tx_buf, rx_buf, len_bytes, end_transaction);
#endif
    return bytes_written;
}
