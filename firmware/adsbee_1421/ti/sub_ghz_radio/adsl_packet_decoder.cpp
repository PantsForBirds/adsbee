#include "adsl_packet_decoder.hh"

#include <cstring>

#include <ti/drivers/dpl/HwiP.h>

#include "buffer_utils.hh"
#include "comms.hh"
#include "led.hh"

void ADSLPacketDecoder::EnqueueChips(const uint8_t *chips, uint16_t num_chip_bytes, int8_t rssi_dbm,
                                     uint32_t timestamp, uint8_t channel) {
    Chips entry;
    entry.num_chip_bytes = num_chip_bytes;
    memcpy(entry.chips, chips, num_chip_bytes);
    entry.channel = channel;
    entry.rssi_dbm = rssi_dbm;
    entry.timestamp = timestamp;
    // Called from the RF callback; Update() masks interrupts around its queue calls.
    chips_queue_.Enqueue(entry);
    num_raw_packets++;
}

bool ADSLPacketDecoder::Update() {
    while (true) {
        Chips entry;
        uintptr_t key = HwiP_disable();
        bool got_entry = chips_queue_.Dequeue(entry);
        HwiP_restore(key);
        if (!got_entry) break;

        RawADSLPacket packet;
        bool valid = packet.FromChips(entry.chips, entry.num_chip_bytes);
        if (valid) {
            packet.sigs_dbm = entry.rssi_dbm;
            packet.channel = entry.channel;
            packet.mlat_48mhz_64bit_counts = entry.timestamp;
            num_valid_packets++;
            leds.FlashLED(bsp.kSubGLEDPin, 50);
            if (!adsl_packet_out_queue.Enqueue(packet)) {
                CONSOLE_ERROR("ADSLPacketDecoder::Update", "ADS-L output queue overflowed.");
            }
        }
        if (settings_manager.settings.log_level >= SettingsManager::LogLevel::kInfo) {
            if (valid) {
                char hex[2 * RawADSLPacket::kMaxPacketLenBytes + 1];
                ByteBufferToHexString(hex, packet.buffer, packet.buffer_len_bytes);
                CONSOLE_INFO("ADSLPacketDecoder::Update", "~[%dFIXD     ] len=%d buf=%s rssi=%d ch=%d ts=%lu",
                             packet.sigq_bits, packet.buffer_len_bytes, hex, packet.sigs_dbm, packet.channel,
                             entry.timestamp);
            } else {
                CONSOLE_INFO("ADSLPacketDecoder::Update", "~[     INVLD] chips=%d rssi=%d ts=%lu", entry.num_chip_bytes,
                             entry.rssi_dbm, entry.timestamp);
            }
        }
    }
    return true;
}
