#include "hardware_unit_tests.hh"

#include "adsbee.hh"
#include "eeprom.hh"
#include "main.hh"  // For ISRS_ON_CORE1.
#include "pico/multicore.h"
#include "spi_coprocessor.hh"

UTEST_STATE();

// Inter-core FIFO protocol: core 0 pushes function and argument; core 1 replies kCore1TestFunctionDone. Nothing else
// uses the FIFO after core 1 launches.
static constexpr uint32_t kCore1TestFunctionDone = 0xD0D0D0D0;

void RunPendingTestFunctionOnCore1() {
    if (!multicore_fifo_rvalid()) {
        return;
    }
    void (*function)(void*) = reinterpret_cast<void (*)(void*)>(multicore_fifo_pop_blocking());
    // Core 0 pushes the argument right after the function pointer.
    void* arg = reinterpret_cast<void*>(multicore_fifo_pop_blocking());
    function(arg);
    multicore_fifo_push_blocking(kCore1TestFunctionDone);
}

bool RunOnISRCore(void (*function)(void*), void* arg, uint32_t timeout_ms) {
#ifdef ISRS_ON_CORE1
    uint64_t timeout_us = static_cast<uint64_t>(timeout_ms) * kUsPerMs;
    multicore_fifo_drain();  // Drop a reply left over from an earlier call that timed out.
    if (!multicore_fifo_push_timeout_us(reinterpret_cast<uintptr_t>(function), timeout_us) ||
        !multicore_fifo_push_timeout_us(reinterpret_cast<uintptr_t>(arg), timeout_us)) {
        return false;
    }
    uint32_t reply;
    return multicore_fifo_pop_timeout_us(timeout_us, &reply) && reply == kCore1TestFunctionDone;
#else
    (void)timeout_ms;
    function(arg);
    return true;
#endif
}

static constexpr uint32_t kTestWatchdogDeadlineMs = 60'000;
static constexpr uint32_t kTestWatchdogFeedIntervalMs = 1000;
static uint64_t test_watchdog_deadline_us = 0;

static bool FeedWatchdogUntilTestDeadline(repeating_timer_t* timer) {
    if (time_us_64() < test_watchdog_deadline_us) {
        adsbee.PokeWatchdog();
    }
    return true;  // Keep the timer running; AT+TEST cancels it.
}

CPP_AT_CALLBACK(ATTestCallback) {
    if (op == '=') {
        if (CPP_AT_HAS_ARG(0)) {
#ifdef HARDWARE_UNIT_TESTS
            if (args[0].compare("DUMP_EEPROM") == 0) {
                eeprom.Dump();
                CPP_AT_SILENT_SUCCESS();
            } else if (args[0].compare("CLEAR_EEPROM") == 0) {
                uint16_t eeprom_size_bytes = eeprom.GetSizeBytes();
                uint8_t buf[eeprom_size_bytes];
                memset(buf, 0xFF, eeprom_size_bytes);
                eeprom.WriteBuf(0x0, buf, eeprom_size_bytes);
                CPP_AT_SUCCESS();
            } else if (args[0].compare("SPI_HANDSHAKE_DEADLOCK") == 0) {
                CPP_AT_PRINTF("Simulating SPI handshake deadlock. Expect kErrorHandshakeHigh then recovery...\r\n");
                if (esp32.TestSPIHandshakeDeadlock()) {
                    CPP_AT_SUCCESS();
                } else {
                    CPP_AT_ERROR("SPI handshake deadlock recovery failed.");
                }
            } else if (args[0].compare("SPI_PERSISTENT_DESYNC") == 0) {
                CPP_AT_PRINTF(
                    "Running persistent SPI desync test. Stresses rapid recovery + UpdateNetworkConsole "
                    "interaction. If deadlock is reproduced the system will stop responding — power cycle to "
                    "recover.\r\n");
                if (esp32.TestSPIPersistentDesync()) {
                    CPP_AT_SUCCESS();
                } else {
                    CPP_AT_ERROR("SPI persistent desync test detected a deadlock.");
                }
            }
#endif
        }
    }

    int argc = 0;
    const char* argv[1];
    // Tests outlast the maximum watchdog timeout (8.3 s). Feed it from a timer until a deadline so a hung test still
    // reboots the board.
    repeating_timer_t watchdog_feeder;
    test_watchdog_deadline_us = time_us_64() + kTestWatchdogDeadlineMs * kUsPerMs;
    add_repeating_timer_ms(kTestWatchdogFeedIntervalMs, FeedWatchdogUntilTestDeadline, nullptr, &watchdog_feeder);
    // utest_main returns the number of failed tests.
    int ret = utest_main(argc, argv);
    cancel_repeating_timer(&watchdog_feeder);
    adsbee.PokeWatchdog();
    if (ret == 0) {
        CPP_AT_SUCCESS();
    } else {
        CPP_AT_ERROR("%d hardware unit test(s) failed.", ret);
    }
}

CPP_AT_CALLBACK(ATIngestModeSCallback) {
    if (op == '=') {
        if (!CPP_AT_HAS_ARG(0)) {
            CPP_AT_ERROR(
                "Usage: AT+INGEST_MODE_S=<hex_data>,<sigs_dbm>,<sigq_db>,<mlat_counts>\r\n\t"
                "hex_data: 14 hex chars (squitter) or 28 hex chars (extended squitter).\r\n\t"
                "sigs_dbm, sigq_db, mlat_counts are optional.");
        }
        int32_t sigs_dbm_arg = INT16_MIN;
        int32_t sigq_db_arg = INT16_MIN;
        uint32_t mlat_counts_arg = 0;
        if (CPP_AT_HAS_ARG(1)) CPP_AT_TRY_ARG2NUM(1, sigs_dbm_arg);
        if (CPP_AT_HAS_ARG(2)) CPP_AT_TRY_ARG2NUM(2, sigq_db_arg);
        if (CPP_AT_HAS_ARG(3)) CPP_AT_TRY_ARG2NUM(3, mlat_counts_arg);

        std::string hex_str(args[0]);
        RawModeSPacket packet(hex_str.data(), -1, static_cast<int16_t>(sigs_dbm_arg), static_cast<int16_t>(sigq_db_arg),
                              static_cast<uint64_t>(mlat_counts_arg));

        if (packet.buffer_len_bytes != RawModeSPacket::kSquitterPacketLenBytes &&
            packet.buffer_len_bytes != RawModeSPacket::kExtendedSquitterPacketLenBytes) {
            CPP_AT_ERROR("Invalid Mode S packet: got %d bytes, expected %d (squitter) or %d (extended squitter).",
                         packet.buffer_len_bytes, RawModeSPacket::kSquitterPacketLenBytes,
                         RawModeSPacket::kExtendedSquitterPacketLenBytes);
        }
        if (!adsbee.raw_mode_s_packet_queue.Enqueue(packet)) {
            CPP_AT_ERROR("Failed to enqueue Mode S packet: queue full.");
        }
        CPP_AT_SUCCESS();
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}

#ifdef ON_COPRO_MASTER
#ifdef HARDWARE_UNIT_TESTS

bool SPICoprocessor::TestSPIPersistentDesync() {
    uint32_t saved_interval = update_interval_ms;
    update_interval_ms = 0;
    last_update_timestamp_ms_ = 0;
    if (!Update()) {
        CONSOLE_ERROR("TestSPIPersistentDesync", "ABORT: Baseline Update() failed before test began.");
        update_interval_ms = saved_interval;
        return false;
    }
    CONSOLE_INFO("TestSPIPersistentDesync", "Baseline OK. Starting 200-iteration stress loop...");

    static const int kNumStressIterations = 200;
    static const int kDesyncEveryN = 20;
    static uint8_t fake_composite[512];
    memset(fake_composite, 0, sizeof(fake_composite));
    static const uint8_t kScratchPayload[] = {0xDE, 0xAD, 0xBE, 0xEF};
    uint32_t success_count = 0, fail_count = 0;

    for (int i = 0; i < kNumStressIterations; i++) {
        const char* flood = "TestSPIPersistentDesync: simulated console error message\r\n";
        for (const char* p = flood; *p; p++) {
            comms_manager.esp32_console_tx_queue.Enqueue(*p);
        }
        comms_manager.UpdateNetworkConsole();
        Write(ObjectDictionary::Address::kAddrCompositeArrayRawPackets, fake_composite,
              /*require_ack=*/true, (uint16_t)sizeof(fake_composite));
        if (i % kDesyncEveryN == 0) {
            SPICoprocessorPacket::SCReadRequestPacket req;
            req.cmd = ObjectDictionary::SCCommand::kCmdReadFromSlave;
            req.addr = ObjectDictionary::Address::kAddrDeviceStatus;
            req.offset = 0;
            req.len = sizeof(ObjectDictionary::ESP32DeviceStatus);
            req.PopulateCRC();
            SPIWriteBlocking(req.GetBuf(), req.GetBufLenBytes(), /*end_transaction=*/true);
            uint32_t wait_start = get_time_since_boot_ms();
            while (get_time_since_boot_ms() - wait_start <
                   SPICoprocessorSlaveInterface::kSPIHandshakeTimeoutMs / 2) {}
            PartialWrite(ObjectDictionary::Address::kAddrScratch,
                         const_cast<uint8_t*>(kScratchPayload), sizeof(kScratchPayload), 0,
                         /*require_ack=*/false);
        }
        last_update_timestamp_ms_ = 0;
        if (Update()) {
            success_count++;
        } else {
            fail_count++;
        }
        if ((i + 1) % 20 == 0) {
            CONSOLE_INFO("TestSPIPersistentDesync", "Iter %d/%d: %lu successes, %lu failures.",
                         i + 1, kNumStressIterations, success_count, fail_count);
        }
    }

    update_interval_ms = saved_interval;
    CONSOLE_INFO("TestSPIPersistentDesync",
                 "Stress loop complete (%lu successes, %lu failures). Checking for deadlock...",
                 success_count, fail_count);
    static const int kMaxRecoveryAttempts = 30;
    bool recovered = false;
    for (int attempt = 0; attempt < kMaxRecoveryAttempts; attempt++) {
        last_update_timestamp_ms_ = 0;
        if (Update()) {
            recovered = true;
            CONSOLE_INFO("TestSPIPersistentDesync",
                         "System functional after stress loop (recovered on attempt %d/%d).",
                         attempt + 1, kMaxRecoveryAttempts);
            break;
        }
        uint32_t delay_start = get_time_since_boot_ms();
        while (get_time_since_boot_ms() - delay_start < 100) {}
    }

    uint32_t diag_start_ms = get_time_since_boot_ms();
    uint32_t high_count = 0, sample_count = 0;
    while (get_time_since_boot_ms() - diag_start_ms < 1000) {
        if (config_.interface.SPIGetHandshakePinLevel()) high_count++;
        sample_count++;
        uint32_t sample_delay = get_time_since_boot_ms();
        while (get_time_since_boot_ms() - sample_delay < 10) {}
    }
    uint32_t handshake_high_pct = sample_count > 0 ? (high_count * 100 / sample_count) : 0;
    CONSOLE_INFO("TestSPIPersistentDesync", "Handshake HIGH %lu%% of 1s diagnostic window (%lu/%lu samples).",
                 handshake_high_pct, high_count, sample_count);
    if (handshake_high_pct > 90) {
        CONSOLE_ERROR("TestSPIPersistentDesync",
                      "DEADLOCK CONFIRMED: handshake stuck HIGH — ESP32 spi_slave_transmit is blocked.");
    }
    if (!recovered) {
        CONSOLE_ERROR("TestSPIPersistentDesync",
                      "DEADLOCK REPRODUCED: SPI communication lost after %d stress iterations. "
                      "System left in deadlocked state for observation. Power cycle to recover.",
                      kNumStressIterations);
    } else {
        CONSOLE_INFO("TestSPIPersistentDesync",
                     "No deadlock after %d iterations (%lu successes, %lu failures). "
                     "Handshake stuck HIGH %lu%% of diagnostic window.",
                     kNumStressIterations, success_count, fail_count, handshake_high_pct);
    }
    return recovered;
}

bool SPICoprocessor::TestSPIHandshakeDeadlock() {
    SPICoprocessorPacket::SCReadRequestPacket request;
    request.cmd = ObjectDictionary::SCCommand::kCmdReadFromSlave;
    request.addr = ObjectDictionary::Address::kAddrDeviceStatus;
    request.offset = 0;
    request.len = sizeof(ObjectDictionary::ESP32DeviceStatus);
    request.PopulateCRC();
    int bytes = SPIWriteBlocking(request.GetBuf(), request.GetBufLenBytes(), /*end_transaction=*/true);
    if (bytes < 0) {
        CONSOLE_ERROR("TestSPIHandshakeDeadlock", "Failed to send read request (code %d).", bytes);
        return false;
    }
    uint32_t delay_start_ms = get_time_since_boot_ms();
    while (get_time_since_boot_ms() - delay_start_ms <
           SPICoprocessorSlaveInterface::kSPIHandshakeTimeoutMs + 10) {}
    CONSOLE_INFO("TestSPIHandshakeDeadlock",
                 "Handshake should be HIGH. Attempting write to trigger recovery...");
    uint8_t dummy[4] = {0};
    bool recovered = PartialWrite(ObjectDictionary::Address::kAddrScratch, dummy, sizeof(dummy));
    if (recovered) {
        CONSOLE_INFO("TestSPIHandshakeDeadlock", "Recovery succeeded — deadlock fix is working.");
    } else {
        CONSOLE_ERROR("TestSPIHandshakeDeadlock", "Recovery failed — system may be deadlocked.");
    }
    return recovered;
}

#endif  // HARDWARE_UNIT_TESTS
#endif  // ON_COPRO_MASTER

CPP_AT_CALLBACK(ATIngestUATCallback) {
    if (op == '=') {
        if (!CPP_AT_HAS_ARG(0)) {
            CPP_AT_ERROR(
                "Usage: AT+INGEST_UAT=<hex_data|UPLINK>,<sigs_dbm>,<sigq_bits>,<mlat_counts>\r\n\t"
                "hex_data: %d hex chars (short ADS-B) or %d (long ADS-B).\r\n\t"
                "UPLINK: send UPLINK as hex_data, then send %d hex chars followed by newline.\r\n\t"
                "sigs_dbm, sigq_bits, mlat_counts are optional.",
                RawUATADSBPacket::kShortADSBMessageNumBytes * 2, RawUATADSBPacket::kLongADSBMessageNumBytes * 2,
                RawUATUplinkPacket::kUplinkMessageNumBytes * 2);
        }
        int32_t sigs_dbm_arg = INT16_MIN;
        int32_t sigq_bits_arg = INT16_MIN;
        uint32_t mlat_counts_arg = 0;
        if (CPP_AT_HAS_ARG(1)) CPP_AT_TRY_ARG2NUM(1, sigs_dbm_arg);
        if (CPP_AT_HAS_ARG(2)) CPP_AT_TRY_ARG2NUM(2, sigq_bits_arg);
        if (CPP_AT_HAS_ARG(3)) CPP_AT_TRY_ARG2NUM(3, mlat_counts_arg);

        const int16_t sigs_dbm = static_cast<int16_t>(sigs_dbm_arg);
        const int16_t sigq_bits = static_cast<int16_t>(sigq_bits_arg);
        const uint64_t mlat_counts = static_cast<uint64_t>(mlat_counts_arg);

        if (args[0].compare("UPLINK") == 0) {
            // Uplink hex strings are too long for a CppAT argument. Signal readiness and read the next line.
            static constexpr uint16_t kUplinkHexMaxLen =
                RawUATUplinkPacket::kUplinkMessageNumBytes * 2 + 2;  // Add space for \r\n characters.
            static constexpr uint32_t kUplinkReadTimeoutMs = 5000;
            char hex_buf[kUplinkHexMaxLen + 1];
            CPP_AT_PRINTF("READY\r\n");

            int32_t hex_buf_len = comms_manager.ATReadConsole(hex_buf, kUplinkHexMaxLen, kUplinkReadTimeoutMs,
                                                              /*terminate_on_newline=*/true);
            if (hex_buf_len < 0) {
                CPP_AT_ERROR("Timed out after %ums waiting for uplink hex data.", kUplinkReadTimeoutMs);
            }
            hex_buf[hex_buf_len] = '\0';

            RawUATUplinkPacket packet(hex_buf, sigs_dbm, sigq_bits, mlat_counts);
            if (packet.encoded_message_len_bytes != RawUATUplinkPacket::kUplinkMessageNumBytes) {
                CPP_AT_ERROR("Failed to construct UAT uplink packet (%d bytes parsed, expected %d).",
                             packet.encoded_message_len_bytes, RawUATUplinkPacket::kUplinkMessageNumBytes);
            }
            if (!adsbee.raw_uat_uplink_packet_queue.Enqueue(packet)) {
                CPP_AT_ERROR("Failed to enqueue UAT uplink packet: queue full.");
            }
        } else {
            uint16_t data_bytes = static_cast<uint16_t>(args[0].size() / 2);
            std::string hex_str(args[0]);

            if (data_bytes == RawUATADSBPacket::kShortADSBMessageNumBytes ||
                data_bytes == RawUATADSBPacket::kLongADSBMessageNumBytes) {
                RawUATADSBPacket packet(hex_str.c_str(), sigs_dbm, sigq_bits, mlat_counts);
                if (packet.buffer_len_bytes != data_bytes) {
                    CPP_AT_ERROR("Failed to construct UAT ADS-B packet from hex data (%d bytes parsed, expected %d).",
                                 packet.buffer_len_bytes, data_bytes);
                }
                if (!adsbee.raw_uat_adsb_packet_queue.Enqueue(packet)) {
                    CPP_AT_ERROR("Failed to enqueue UAT ADS-B packet: queue full.");
                }
            } else {
                CPP_AT_ERROR(
                    "Invalid UAT ADS-B packet length %d bytes. Expected %d (short) or %d (long). Use UPLINK for "
                    "uplink packets.",
                    data_bytes, RawUATADSBPacket::kShortADSBMessageNumBytes,
                    RawUATADSBPacket::kLongADSBMessageNumBytes);
            }
        }
        CPP_AT_SUCCESS();
    }
    CPP_AT_ERROR("Operator '%c' not supported.", op);
}