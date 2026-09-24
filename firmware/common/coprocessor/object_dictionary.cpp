#include "object_dictionary.hh"

#include "hal.hh"  // for timestamping
#ifdef ON_ESP32
#include "adsbee_server.hh"
#include "cpu_utils.hh"
#include "device_info.hh"
#include "esp_system.h"
#include "pico.hh"               // pico_ll.ForceBlinkNetworkLED() for kAddrLEDBlink.
#include "remote_id_manager.hh"  // remote_id_manager.GetOutQueue() for the ESP32 -> RP2040 Remote ID pull.
#include "task_utils.hh"
#ifdef CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
#include "esp_core_dump.h"
#include "esp_err.h"
#endif
#elif defined(ON_TI)
#include "cpu_utils.hh"
#include "pico.hh"  // pico_ll.BlinkSubGLED() for kAddrLEDBlink.
#endif

#include "comms.hh"

const uint8_t ObjectDictionary::kFirmwareVersionMajor = 0;
const uint8_t ObjectDictionary::kFirmwareVersionMinor = 9;
const uint8_t ObjectDictionary::kFirmwareVersionPatch = 1;
// NOTE: Indicate a final release with RC = 0.
const uint8_t ObjectDictionary::kFirmwareVersionReleaseCandidate = 2;

const uint32_t ObjectDictionary::kFirmwareVersion = (kFirmwareVersionMajor << 24) | (kFirmwareVersionMinor << 16) |
                                                    (kFirmwareVersionPatch << 8) | kFirmwareVersionReleaseCandidate;

#ifdef ON_TI
extern CPUMonitor user_core_monitor;
#elif defined(ON_ESP32)
extern CPUMonitor cpu_monitor;
#endif

#ifdef ON_COPRO_SLAVE
/**
 * Returns true if a write or read of buf_len bytes at offset fits inside an object of object_len bytes, otherwise logs
 * an error naming the object and returns false. Guards every memcpy into or out of a fixed size object so that a peer
 * running a different struct layout (or a corrupted transaction) can't overrun the object.
 */
static bool CheckObjectBounds(const char* tag, const char* object_name, uint16_t offset, uint16_t buf_len,
                              size_t object_len) {
    if ((size_t)offset + buf_len > object_len) {
        CONSOLE_ERROR(tag, "%d Bytes at offset %d exceeds the %d Byte %s object.", buf_len, offset, (int)object_len,
                      object_name);
        return false;
    }
    return true;
}

bool ObjectDictionary::SetBytes(Address addr, uint8_t* buf, uint16_t buf_len, uint16_t offset) {
    switch (addr) {
        case kAddrScratch:
            // Warning: printing here will cause a timeout and tests will fail.
            // CONSOLE_INFO("ObjectDictionary::SetBytes", "Setting %d settings Bytes at offset %d.", buf_len,
            // offset);
            if (!CheckObjectBounds("ObjectDictionary::SetBytes", "scratch", offset, buf_len, sizeof(scratch_))) {
                return false;
            }
            memcpy((uint8_t*)&scratch_ + offset, buf, buf_len);
            break;
        case kAddrSettingsData:
            // Warning: printing here will cause a timeout and tests will fail.
            // CONSOLE_INFO("ObjectDictionary::SetBytes", "Setting %d settings Bytes at offset %d.", buf_len,
            // offset);
            if (!CheckObjectBounds("ObjectDictionary::SetBytes", "settings", offset, buf_len,
                                   sizeof(SettingsManager::Settings))) {
                return false;
            }
            memcpy((uint8_t*)&(settings_manager.settings) + offset, buf, buf_len);
            if (offset + buf_len == sizeof(SettingsManager::Settings)) {
                settings_manager.Apply();
                CONSOLE_INFO("SPICoprocessor::SetBytes", "Wrote last chunk of settings data. Applying new values.");
            }
            break;
        case kAddrRollQueue: {
            // Only full writes are allowed for this command: a roll request is a single small struct, so a partial or
            // offset write can only come from a malformed transaction and must not execute a roll.
            if (buf_len != sizeof(RollQueueRequest) || offset != 0) {
                CONSOLE_ERROR("ObjectDictionary::SetBytes",
                              "Buffer length %d and offset %d for writing RollQueueRequest must be exactly %d and 0.",
                              buf_len, offset, (int)sizeof(RollQueueRequest));
                return false;
            }
            RollQueueRequest roll_request;
            memcpy(&roll_request, buf, sizeof(RollQueueRequest));
            switch (roll_request.queue_id) {
                case kQueueIDLogMessages:
                    // Roll the log message queue.
                    log_message_queue.Discard(roll_request.num_items);
                    break;
                case kQueueIDSCCommandRequests:
                    for (uint16_t i = 0; i < roll_request.num_items; i++) {
                        // Dequeue requests one by one so that we can call their callbacks.
                        SCCommandRequestWithCallback request_with_callback;
                        if (!sc_command_request_queue.Dequeue(request_with_callback)) {
                            CONSOLE_ERROR("ObjectDictionary::SetBytes",
                                          "Failed to pop SCCommand request from queue during roll.");
                            return false;
                        }
                        if (request_with_callback.complete_callback) {
                            request_with_callback.complete_callback();  // Call the callback if it exists.
                        }
                    }
                    break;
                case kQueueIDConsole:
#ifdef ON_ESP32
                    if (xSemaphoreTake(object_dictionary.network_console_rx_queue_mutex,
                                       pdMS_TO_TICKS(kNetworkConsoleMutexTimeoutMs)) != pdTRUE) {
                        CONSOLE_ERROR("ObjectDictionary::SetBytes",
                                      "Timed out waiting for network console RX queue mutex; skipping queue roll.");
                        return false;
                    }
                    if (!network_console_rx_queue.Discard(roll_request.num_items)) {
                        CONSOLE_ERROR("ObjectDictionary::SetBytes",
                                      "Failed to discard %d chars from the network console TX queue.",
                                      roll_request.num_items);
                        xSemaphoreGive(object_dictionary.network_console_rx_queue_mutex);
                        return false;
                    }
                    xSemaphoreGive(object_dictionary.network_console_rx_queue_mutex);
#elif defined(ON_PICO)
                    if (!network_console_rx_queue.Discard(roll_request.num_items)) {
                        CONSOLE_ERROR("ObjectDictionary::SetBytes",
                                      "Failed to discard %d chars from the network console TX queue.",
                                      roll_request.num_items);

                        return false;
                    }
#else
                    CONSOLE_ERROR("ObjectDictionary::SetBytes",
                                  "Received roll queue request for network console on unsupported device.");
                    return false;
#endif
                    break;
                default:
                    CONSOLE_ERROR("ObjectDictionary::SetBytes",
                                  "Received roll queue request for nonexistent queue ID %d.", roll_request.queue_id);
                    return false;
            }
            break;
        }
#ifdef ON_ESP32
        case kAddrConsole: {
            // Don't print here to avoid print of print doom loop explosion.
            // CONSOLE_INFO("ObjectDictionary::SetBytes", "Forwarding %d byte message to network console.", buf_len);
            adsbee_server.network_console.BroadcastMessage(reinterpret_cast<const char*>(buf), buf_len);
            break;
        }
#ifdef CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
        case kAddrESP32TriggerAbort: {
            abort();
            break;
        }
#endif
        case kAddrAircraftDictionaryMetrics: {
            // Only whole-struct writes are meaningful here. A length mismatch means the RP2040 and ESP32 were built
            // with different AircraftDictionary::Metrics layouts; reject it loudly instead of overrunning the stack.
            if (offset != 0 || buf_len != sizeof(AircraftDictionary::Metrics)) {
                CONSOLE_ERROR("ObjectDictionary::SetBytes",
                              "AircraftDictionary::Metrics write of %d Bytes at offset %d does not match the %d Byte "
                              "struct; RP2040 and ESP32 firmware are out of sync.",
                              buf_len, offset, (int)sizeof(AircraftDictionary::Metrics));
                return false;
            }
            AircraftDictionary::Metrics rp2040_metrics;
            memcpy(&rp2040_metrics, buf, sizeof(AircraftDictionary::Metrics));
            xQueueSend(adsbee_server.rp2040_aircraft_dictionary_metrics_queue, &rp2040_metrics, 0);
            break;
        }
        case kAddrCompositeArrayRawPackets: {
            if (offset != 0) {
                CONSOLE_ERROR("ObjectDictionary::SetBytes",
                              "Offset %d for writing CompositeArray::RawPackets not supported, must be 0.", offset);
                return false;
            }
            CompositeArray::UnpackRawPacketsBufferToQueues(buf, buf_len, &(adsbee_server.raw_mode_s_packet_in_queue),
                                                           &(adsbee_server.raw_uat_adsb_packet_in_queue),
                                                           &(adsbee_server.raw_uat_uplink_packet_in_queue));
            break;
        }
        case kAddrDeviceStatus: {
            if (buf_len != sizeof(CompositeDeviceStatus) || offset != 0) {
                CONSOLE_ERROR(
                    "ObjectDictionary::SetBytes",
                    "Buffer length %d and offset %d for writing CompositeDeviceStatus must be exactly %d and 0.",
                    buf_len, offset, sizeof(CompositeDeviceStatus));
                return false;
            }
            memcpy(&composite_device_status, buf, sizeof(CompositeDeviceStatus));
            break;
        }
        case kAddrLEDBlink: {
            if (buf_len != sizeof(uint32_t) || offset != 0) {
                CONSOLE_ERROR("ObjectDictionary::SetBytes",
                              "Buffer length %d and offset %d for writing LED blink duration must be exactly %d and 0.",
                              buf_len, offset, sizeof(uint32_t));
                return false;
            }
            uint32_t duration_ms;
            memcpy(&duration_ms, buf, sizeof(duration_ms));
            pico_ll.ForceBlinkNetworkLED(duration_ms);
            break;
        }
#elif defined(ON_TI)
        case kAddrLEDBlink: {
            if (buf_len != sizeof(uint32_t) || offset != 0) {
                CONSOLE_ERROR("ObjectDictionary::SetBytes",
                              "Buffer length %d and offset %d for writing LED blink duration must be exactly %d and 0.",
                              buf_len, offset, sizeof(uint32_t));
                return false;
            }
            uint32_t duration_ms;
            memcpy(&duration_ms, buf, sizeof(duration_ms));
            pico_ll.ForceBlinkSubGLED(duration_ms);
            break;
        }
#endif
        default:
            CONSOLE_ERROR("SPICoprocessor::SetBytes", "No behavior implemented for writing to address 0x%x.", addr);
            return false;
    }
    return true;
}

bool ObjectDictionary::GetBytes(Address addr, uint8_t* buf, uint16_t buf_len, uint16_t offset) {
    switch (addr) {
        case kAddrFirmwareVersion:
            if (!CheckObjectBounds("ObjectDictionary::GetBytes", "firmware version", offset, buf_len,
                                   sizeof(kFirmwareVersion))) {
                return false;
            }
            memcpy(buf, (uint8_t*)(&kFirmwareVersion) + offset, buf_len);
            break;
        case kAddrScratch:
            // Warning: printing here will cause a timeout and tests will fail.
            // CONSOLE_INFO("ObjectDictionary::GetBytes", "Getting %d scratch Bytes at offset %d.", buf_len,
            // offset);
            if (!CheckObjectBounds("ObjectDictionary::GetBytes", "scratch", offset, buf_len, sizeof(scratch_))) {
                return false;
            }
            memcpy(buf, (uint8_t*)(&scratch_) + offset, buf_len);
            break;
        case kAddrSettingsData:
            // Warning: printing here will cause a timeout and tests will fail.
            // CONSOLE_INFO("ObjectDictionary::GetBytes", "Getting %d settings Bytes at offset %d.",
            // buf_len, offset);
            if (!CheckObjectBounds("ObjectDictionary::GetBytes", "settings", offset, buf_len,
                                   sizeof(SettingsManager::Settings))) {
                return false;
            }
            memcpy(buf, (uint8_t*)&(settings_manager.settings) + offset, buf_len);
            break;
        case kAddrDeviceStatus: {
            if (!CheckObjectBounds("ObjectDictionary::GetBytes", "device status", offset, buf_len,
                                   sizeof(device_status))) {
                return false;
            }
            UpdateDeviceStatus();
            memcpy(buf, (uint8_t*)&device_status + offset, buf_len);
            break;
        }
        case kAddrLogMessages: {
            // RP2040 reading log messages from the ESP32. Offset-aware FIFO read (see PackLogMessages and the
            // kAddrConsole handler): a log burst whose packed size exceeds one SPI transaction is read in chunks with an
            // increasing byte offset, so we forward `offset` to serve the correct window of the packed stream.
            PackLogMessages(buf, buf_len, log_message_queue, kLogMessageQueueDepth, offset);
            break;
        }
        case kAddrSCCommandRequests: {
            if (offset != 0) {
                CONSOLE_ERROR("ObjectDictionary::GetBytes",
                              "Offset %d for reading SCCommandRequest not supported, must be 0.", offset);
                return false;
            }
            if (buf_len != sizeof(SCCommandRequest)) {
                CONSOLE_ERROR("ObjectDictionary::GetBytes",
                              "Buffer length %d for reading SCCommandRequest must be exactly %d.", buf_len,
                              sizeof(SCCommandRequest));
                return false;
            }
            SCCommandRequestWithCallback request_with_callback;
            if (!sc_command_request_queue.Peek(request_with_callback)) {
                CONSOLE_ERROR("ObjectDictionary::GetBytes", "No SCCommand requests available to read.");
                return false;
            }
            SCCommandRequest& request = request_with_callback.request;
            memcpy(buf, &request, sizeof(SCCommandRequest));
            break;
        }
#ifdef ON_ESP32
        case kAddrESP32DeviceInfo: {
            // NOTE: offset is a byte offset, so it must be applied to a byte pointer (not to the struct pointer, which
            // would step by whole structs and read off the end of the stack).
            if (!CheckObjectBounds("ObjectDictionary::GetBytes", "ESP32 device info", offset, buf_len,
                                   sizeof(ESP32DeviceInfo))) {
                return false;
            }
            ESP32DeviceInfo esp32_device_info = GetESP32DeviceInfo();
            memcpy(buf, (uint8_t*)&esp32_device_info + offset, buf_len);
            break;
        }
        case kAddrESP32NetworkInfo: {
            if (!CheckObjectBounds("ObjectDictionary::GetBytes", "ESP32 network info", offset, buf_len,
                                   sizeof(ESP32NetworkInfo))) {
                return false;
            }
            ESP32NetworkInfo network_info = comms_manager.GetNetworkInfo();
            memcpy(buf, (uint8_t*)&network_info + offset, buf_len);
            break;
        }
        case kAddrESP32RebootInfo: {
            ESP32RebootInfo reboot_info = {};
            esp_reset_reason_t reason = esp_reset_reason();
            reboot_info.reset_reason = static_cast<uint8_t>(reason);
            const char* reason_str;
            switch (reason) {
                case ESP_RST_POWERON:
                    reason_str = "POWERON";
                    break;
                case ESP_RST_EXT:
                    reason_str = "EXT_RESET";
                    break;
                case ESP_RST_SW:
                    reason_str = "SOFTWARE";
                    break;
                case ESP_RST_PANIC:
                    reason_str = "PANIC";
                    break;
                case ESP_RST_INT_WDT:
                    reason_str = "INT_WATCHDOG";
                    break;
                case ESP_RST_TASK_WDT:
                    reason_str = "TASK_WATCHDOG";
                    break;
                case ESP_RST_WDT:
                    reason_str = "WATCHDOG";
                    break;
                case ESP_RST_DEEPSLEEP:
                    reason_str = "DEEP_SLEEP_WAKEUP";
                    break;
                case ESP_RST_BROWNOUT:
                    reason_str = "BROWNOUT";
                    break;
                case ESP_RST_SDIO:
                    reason_str = "SDIO";
                    break;
                default:
                    reason_str = "UNKNOWN";
                    break;
            }
            strncpy(reboot_info.reset_reason_str, reason_str, kResetReasonStrMaxLen);
            reboot_info.reset_reason_str[kResetReasonStrMaxLen] = '\0';
#ifdef CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
            reboot_info.core_dump_to_flash_enabled = true;
            // Defer flash reads to esp_timer task — calling them here (SPI slave task context)
            // deadlocks because esp_core_dump_* disables the CPU cache while the SPI task is live.
            ScheduleDelayedFunctionCall(0, [](void*) {
                esp_err_t check_err = esp_core_dump_image_check();
                if (check_err == ESP_OK) {
                    esp_core_dump_summary_t summary;
                    esp_err_t summary_err = esp_core_dump_get_summary(&summary);
                    if (summary_err == ESP_OK) {
                        static const char* const kExcCauseStr[] = {"IllegalInstruction",
                                                                   "Syscall",
                                                                   "InstructionFetchError",
                                                                   "LoadStoreError",
                                                                   "Level1Interrupt",
                                                                   "Alloca",
                                                                   "IntegerDivideByZero",
                                                                   "PCValue",
                                                                   "Privileged",
                                                                   "LoadStoreAlignment",
                                                                   "res",
                                                                   "res",
                                                                   "InstrPDAddrError",
                                                                   "LoadStorePIFDataError",
                                                                   "InstrPIFAddrError",
                                                                   "LoadStorePIFAddrError",
                                                                   "InstTLBMiss",
                                                                   "InstTLBMultiHit",
                                                                   "InstFetchPrivilege",
                                                                   "res",
                                                                   "InstrFetchProhibited",
                                                                   "res",
                                                                   "res",
                                                                   "res",
                                                                   "LoadStoreTLBMiss",
                                                                   "LoadStoreTLBMultihit",
                                                                   "LoadStorePrivilege",
                                                                   "res",
                                                                   "LoadProhibited",
                                                                   "StoreProhibited",
                                                                   "res",
                                                                   "res",
                                                                   "Cp0Dis",
                                                                   "Cp1Dis",
                                                                   "Cp2Dis",
                                                                   "Cp3Dis",
                                                                   "Cp4Dis",
                                                                   "Cp5Dis",
                                                                   "Cp6Dis",
                                                                   "Cp7Dis"};
                        uint32_t cause = summary.ex_info.exc_cause;
                        const char* cause_str =
                            (cause < sizeof(kExcCauseStr) / sizeof(kExcCauseStr[0])) ? kExcCauseStr[cause] : "Unknown";
                        CONSOLE_ERROR("ESP32RebootInfo",
                                      "Core Dump Task=%.16s PC=0x%08lx ExcCause=%lu (%s) ExcVAddr=0x%08lx",
                                      summary.exc_task, (unsigned long)summary.exc_pc, (unsigned long)cause, cause_str,
                                      (unsigned long)summary.ex_info.exc_vaddr);
                        char panic_reason[200] = {};
                        if (esp_core_dump_get_panic_reason(panic_reason, sizeof(panic_reason)) == ESP_OK) {
                            CONSOLE_ERROR("ESP32RebootInfo", "Panic: %s", panic_reason);
                        }
                        CONSOLE_ERROR("ESP32RebootInfo", "BT depth=%lu corrupted=%d",
                                      (unsigned long)summary.exc_bt_info.depth, (int)summary.exc_bt_info.corrupted);
                        for (uint32_t i = 0; i < summary.exc_bt_info.depth; i++) {
                            CONSOLE_ERROR("ESP32RebootInfo", "  BT[%lu] 0x%08lx", (unsigned long)i,
                                          (unsigned long)summary.exc_bt_info.bt[i]);
                        }
                    } else {
                        CONSOLE_ERROR("ESP32RebootInfo", "get_summary failed: %s", esp_err_to_name(summary_err));
                    }
                } else {
                    CONSOLE_ERROR("ESP32RebootInfo", "image_check failed: %s", esp_err_to_name(check_err));
                }
            });
#endif
            memcpy(buf, reinterpret_cast<uint8_t*>(&reboot_info) + offset, buf_len);
            break;
        }
        case kAddrConsole: {
            // Offset-aware FIFO read. SPICoprocessor::Read splits reads larger than one SPI transaction into multiple
            // PartialReads with an increasing byte offset (e.g. a 4000-Byte console read becomes chunk 1 [offset 0] +
            // chunk 2 [offset ~2045] when kSPITransactionMaxLenBytes is 2048). This handler MUST honor `offset` and peek
            // the queue starting at `offset`, otherwise a multi-chunk read re-serves the head of the queue and corrupts
            // the tail of the reassembled buffer (this silently broke network-console OTA when the transaction size was
            // reduced 4096->2048). The queue is only rolled after the full Read completes, and concurrent producers only
            // append at the tail, so head-relative indices [offset, offset + buf_len) are stable across chunks.
            if (xSemaphoreTake(object_dictionary.network_console_rx_queue_mutex,
                               pdMS_TO_TICKS(kNetworkConsoleMutexTimeoutMs)) != pdTRUE) {
                CONSOLE_ERROR("ObjectDictionary::GetBytes",
                              "Timed out waiting for network console RX queue mutex; skipping console read.");
                return false;
            }
            if (network_console_rx_queue.Length() < offset + buf_len) {
                CONSOLE_ERROR(
                    "ObjectDictionary::GetBytes",
                    "Network console read of %d Bytes at offset %d exceeds RX queue length %d.", buf_len, offset,
                    network_console_rx_queue.Length());
                xSemaphoreGive(object_dictionary.network_console_rx_queue_mutex);
                return false;
            }
            for (uint16_t i = 0; i < buf_len; i++) {
                char ch;
                if (!network_console_rx_queue.Peek(ch, offset + i)) {
                    CONSOLE_ERROR("ObjectDictionary::GetBytes",
                                  "Failed to peek character %d from network console RX queue.", offset + i);
                    xSemaphoreGive(object_dictionary.network_console_rx_queue_mutex);
                    return false;
                }
                buf[i] = static_cast<uint8_t>(ch);
            }
            xSemaphoreGive(object_dictionary.network_console_rx_queue_mutex);
            break;
        }
        case kAddrCompositeArrayRawPackets: {
            // ESP32 -> RP2040 direction: pack the Remote ID packets the ESP32 has received over BLE/WiFi so the RP2040
            // can ingest them into its own AircraftDictionary (which drives serial reporting). Mode S and UAT flow the
            // other direction (RP2040 -> ESP32), so only the Remote ID queue is packed here.
            if (offset != 0) {
                CONSOLE_ERROR("ObjectDictionary::GetBytes",
                              "Offset %d for reading CompositeArrayRawPackets not supported, must be 0.", offset);
                return false;
            }
            if (buf_len < sizeof(CompositeArray::RawPackets::Header)) {
                CONSOLE_ERROR("ObjectDictionary::GetBytes",
                              "Buffer length %d for reading CompositeArrayRawPackets must be at least %d.", buf_len,
                              sizeof(CompositeArray::RawPackets::Header));
                return false;
            }

            CompositeArray::RawPackets raw_packets = CompositeArray::PackRawPacketsBuffer(
                buf, buf_len, nullptr, nullptr, nullptr, remote_id_manager.GetOutQueue());
            if (raw_packets.IsValid() == false) {
                CONSOLE_ERROR("ObjectDictionary::GetBytes",
                              "Failed to pack CompositeArray::RawPackets into buffer for reading.");
                return false;
            }

            break;
        }
#elif defined(ON_TI)
        case kAddrCompositeArrayRawPackets: {
            if (offset != 0) {
                CONSOLE_ERROR("ObjectDictionary::GetBytes",
                              "Offset %d for reading CompositeArrayRawPackets not supported, must be 0.", offset);
                return false;
            }
            if (buf_len < sizeof(CompositeArray::RawPackets::Header)) {
                CONSOLE_ERROR("ObjectDictionary::GetBytes",
                              "Buffer length %d for reading CompositeArrayRawPackets must be at least %d.", buf_len,
                              sizeof(CompositeArray::RawPackets::Header));
                return false;
            }

            CompositeArray::RawPackets raw_packets = CompositeArray::PackRawPacketsBuffer(
                buf, buf_len, nullptr, &raw_uat_adsb_packet_queue, &raw_uat_uplink_packet_queue);
            if (raw_packets.IsValid() == false) {
                CONSOLE_ERROR("ObjectDictionary::GetBytes",
                              "Failed to pack CompositeArray::RawPackets into buffer for reading.");
                return false;
            }

            break;
        }
#endif
        default:
            CONSOLE_ERROR("SPICoprocessor::SetBytes", "No behavior implemented for reading from address 0x%x.", addr);
            return false;
    }
    return true;
}

bool ObjectDictionary::RequestSCCommand(const SCCommandRequestWithCallback& request_with_callback) {
    if (sc_command_request_queue.Enqueue(request_with_callback)) {
        return true;
    } else {
        CONSOLE_ERROR("ObjectDictionary::RequestSCCommand", "Failed to push SCCommandRequest to queue, queue is full.");
        return false;
    }
}

bool ObjectDictionary::RequestSCCommandBlocking(const SCCommandRequestWithCallback& request_with_callback) {
#ifdef ON_ESP32
    SemaphoreHandle_t command_complete_semaphore = xSemaphoreCreateBinary();
    if (command_complete_semaphore == NULL) {
        CONSOLE_ERROR("ADSBeeServer::Init", "Failed to create settings read semaphore.");
        return false;
    }
#endif

    bool ret = object_dictionary.RequestSCCommand(ObjectDictionary::SCCommandRequestWithCallback {
        .request = request_with_callback.request,
        .complete_callback =
#ifdef ON_ESP32
            [command_complete_semaphore, request_with_callback]() {
#else
            [request_with_callback]() {
#endif
                if (request_with_callback.complete_callback) {
                    request_with_callback.complete_callback();  // Call the existing callback if it exists.
                }
#ifdef ON_ESP32
                xSemaphoreGive(command_complete_semaphore);
#endif
            },
    });  // Require ack.

#ifdef ON_ESP32
    // Wait for the callback to complete
    xSemaphoreTake(command_complete_semaphore, portMAX_DELAY);
    vSemaphoreDelete(command_complete_semaphore);
#endif
    return ret;
}
#endif

#ifdef ON_COPRO_SLAVE
/**
 * Updates the device status struct with the latest information. Called when the object dictionary is read from at
 * kAddrDeviceStatus, or when other functions want to refresh the device status before borrowing it for other uses.
 */
void ObjectDictionary::UpdateDeviceStatus() {
    uint16_t num_log_messages = log_message_queue.Length();
#ifdef ON_ESP32
    uint16_t num_network_console_rx_chars = 0;
    if (xSemaphoreTake(network_console_rx_queue_mutex, pdMS_TO_TICKS(kNetworkConsoleMutexTimeoutMs)) == pdTRUE) {
        num_network_console_rx_chars = network_console_rx_queue.Length();
        xSemaphoreGive(network_console_rx_queue_mutex);
    }

    // We only read fast stuff here. Slow things like core usage calculations and temperature reads are done in
    // device_status_update_task.
    device_status.timestamp_ms = get_time_since_boot_ms();
    device_status.num_queued_log_messages = num_log_messages;
    device_status.queued_log_messages_packed_size_bytes =
        static_cast<uint32_t>(num_log_messages * LogMessage::kHeaderSize);
    device_status.num_queued_sc_command_requests = sc_command_request_queue.Length();
    device_status.num_queued_network_console_rx_chars = num_network_console_rx_chars;
    // Length of Remote ID packets queued for the RP2040 to pull (header-only == nothing pending). The RP2040 polls this
    // in ESP32::Update() and issues a kAddrCompositeArrayRawPackets read when it exceeds the header size. Mirrors the
    // CC1312's pending_raw_packets_len_bytes pull mechanism (see the ON_TI branch below).
    device_status.pending_raw_packets_len_bytes = CompositeArray::CalculateRawPacketsBufferLength(
        nullptr, nullptr, nullptr, remote_id_manager.GetOutQueue());
#elif defined(ON_TI)
    device_status = {
        .timestamp_ms = get_time_since_boot_ms(),
        .temperature_deg_c = static_cast<uint8_t>(CPUMonitor::ReadTemperatureDegC()),
        .cpu_usage_percent = user_core_monitor.GetUsagePercent(),
        .num_queued_log_messages = num_log_messages,
        .queued_log_messages_packed_size_bytes = static_cast<uint32_t>(num_log_messages * LogMessage::kHeaderSize),
        .num_queued_sc_command_requests = sc_command_request_queue.Length(),
        .pending_raw_packets_len_bytes =
            static_cast<uint16_t>(raw_uat_adsb_packet_queue.Length() * sizeof(RawUATADSBPacket) +
                                  raw_uat_uplink_packet_queue.Length() * sizeof(RawUATUplinkPacket) +
                                  sizeof(CompositeArray::RawPackets::Header)),
        .num_raw_uat_adsb_packets = metrics.num_raw_uat_adsb_packets,
        .num_valid_uat_adsb_packets = metrics.num_valid_uat_adsb_packets,
        .num_raw_uat_uplink_packets = metrics.num_raw_uat_uplink_packets,
        .num_valid_uat_uplink_packets = metrics.num_valid_uat_uplink_packets};
    // Reset metrics after reading.
    metrics = {0};
#endif
    for (uint16_t i = 0; i < log_message_queue.Length(); i++) {
        LogMessage log_message;
        if (log_message_queue.Peek(log_message, i)) {
            device_status.queued_log_messages_packed_size_bytes += log_message.num_chars + 1;  // +1 for null terminator
        }
    }
}
#endif

uint16_t ObjectDictionary::PackLogMessages(uint8_t* buf, uint16_t buf_len,
                                           PFBQueue<ObjectDictionary::LogMessage>& log_message_queue,
                                           uint16_t num_messages, uint16_t offset) {
    // Offset-aware FIFO pack: fill buf with the byte window [offset, offset + buf_len) of the concatenated packed
    // log-message stream, where each message packs to (kHeaderSize + num_chars + 1) Bytes. This is required so that
    // SPICoprocessor::Read's multi-transaction (chunked) reads reassemble correctly: when the packed stream exceeds one
    // SPI transaction it is read in chunks with increasing byte offset, and a chunk boundary can fall in the middle of a
    // message. We copy the intersection of each message's stream range with the requested window. With offset == 0 and a
    // buf_len large enough to hold whole messages (the common single-transaction case) this is byte-identical to a
    // simple head-packed stream. See the matching offset-aware kAddrConsole handler; any FIFO-backed object read this
    // way MUST honor offset or reject non-zero offset, or a future transaction-size change silently corrupts it.
    const uint32_t window_start = offset;
    const uint32_t window_end = static_cast<uint32_t>(offset) + buf_len;
    uint32_t stream_pos = 0;     // Byte position of the current message within the full packed stream.
    uint16_t bytes_written = 0;  // Furthest byte written into buf, relative to window_start.
    for (uint16_t i = 0; i < num_messages; i++) {
        LogMessage log_message;
        if (!log_message_queue.Peek(log_message, i)) {
            break;  // No more messages to pack.
        }
        const uint16_t packed_size = LogMessage::kHeaderSize + log_message.num_chars + 1;  // +1 for null terminator.
        const uint32_t msg_start = stream_pos;
        const uint32_t msg_end = stream_pos + packed_size;
        // Intersect this message's stream range [msg_start, msg_end) with the requested window.
        const uint32_t copy_start = msg_start > window_start ? msg_start : window_start;
        const uint32_t copy_end = msg_end < window_end ? msg_end : window_end;
        if (copy_start < copy_end) {
            if (msg_start >= window_start && msg_end <= window_end) {
                // Whole message fits in the window: copy directly (common case, offset 0).
                uint8_t* dst = buf + (msg_start - window_start);
                memcpy(dst, &log_message, packed_size - 1);
                dst[packed_size - 1] = '\0';  // Null terminate the message.
            } else {
                // Message straddles a chunk boundary: materialize its packed bytes and copy the overlapping slice.
                uint8_t packed[LogMessage::kHeaderSize + kLogMessageMaxNumChars + 1];
                memcpy(packed, &log_message, packed_size - 1);
                packed[packed_size - 1] = '\0';  // Null terminate the message.
                memcpy(buf + (copy_start - window_start), packed + (copy_start - msg_start), copy_end - copy_start);
            }
            bytes_written = static_cast<uint16_t>(copy_end - window_start);
        }
        stream_pos = msg_end;
        if (stream_pos >= window_end) {
            break;  // Filled the requested window.
        }
        // Don't pop log messages here, wait for the roll request to do that.
    }
    return bytes_written;
}

uint16_t ObjectDictionary::UnpackLogMessages(uint8_t* buf, uint16_t buf_len,
                                             PFBQueue<ObjectDictionary::LogMessage>& log_message_queue,
                                             uint16_t max_num_messages) {
    uint16_t bytes_read = 0;
    uint16_t num_messages = 0;

    while (bytes_read < buf_len && num_messages < max_num_messages) {
        LogMessage log_message;
        if (buf_len - bytes_read < LogMessage::kHeaderSize) {
            break;  // Not enough data for header.
        }
        // Cast to a Byte array to avoid warnings about memcpy not writing the full LogMessage object.
        memcpy((uint8_t*)(&log_message), buf + bytes_read, LogMessage::kHeaderSize);

        if (log_message.num_chars > kLogMessageMaxNumChars) {
            CONSOLE_ERROR("ObjectDictionary::UnpackLogMessages", "Invalid log message length: %d",
                          log_message.num_chars);
            break;  // Invalid length.
        }

        if (buf_len - bytes_read < LogMessage::kHeaderSize + log_message.num_chars + 1) {
            break;  // Not enough data for the full message.
        }

        memcpy(log_message.message, buf + bytes_read + LogMessage::kHeaderSize, log_message.num_chars);
        log_message.message[log_message.num_chars] = '\0';  // Null terminate the message.

        log_message_queue.Enqueue(log_message);

        bytes_read += LogMessage::kHeaderSize + log_message.num_chars + 1;  // Move past header and message.
        num_messages++;
    }

    return num_messages;
}
