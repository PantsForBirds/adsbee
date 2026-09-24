#include "uat_packet_decoder.hh"

#include <ti/drivers/dpl/HwiP.h>

#include "buffer_utils.hh"
#include "comms.hh"
#include "led.hh"

// The raw UAT queues are filled from the RF-core ISR callback (SubGHzRadio::HandlePacketRx) and
// PFBQueue is not ISR-safe, so consumer-side dequeues mask interrupts around the queue operation.
// On this single-core M4F that is sufficient: the ISR producer can never observe (or interleave
// with) a half-updated head/tail. Do NOT use PFBQueue's is_thread_safe flag here -- its mutex
// backend is a no-op stub on this platform and a blocking mutex would deadlock in ISR context.

bool UATPacketDecoder::Update() {
    // Decode incoming UAT ADS-B packets and push valid ones to the output queue.
    while (true) {
        RawUATADSBPacket packet;
        uintptr_t key = HwiP_disable();
        bool got_packet = raw_uat_adsb_packet_queue.Dequeue(packet);
        HwiP_restore(key);
        if (!got_packet) {
            break;
        }
        uint16_t packet_len_bytes = packet.buffer_len_bytes;
        DecodedUATADSBPacket decoded_packet = DecodedUATADSBPacket(packet);

        if (decoded_packet.is_valid) {
            leds.FlashLED(bsp.kSubGLEDPin, 50);
            if (!decoded_uat_adsb_packet_out_queue.Enqueue(decoded_packet)) {
                CONSOLE_ERROR("UATPacketDecoder::Update", "UAT ADS-B decoded output queue overflowed.");
            }
        }

        // Per-packet prints, gated on log level up front: the hex string formatting is real work that
        // would otherwise run (and CONSOLE_INFO would evaluate its arguments) for every packet even
        // with the output suppressed below kInfo.
        if (settings_manager.settings.log_level >= SettingsManager::LogLevel::kInfo) {
            char raw_packet_buffer[2 * RawUATADSBPacket::kADSBMessageMaxSizeBytes + 1];
            ByteBufferToHexString(raw_packet_buffer, packet.buffer, packet_len_bytes);
            if (decoded_packet.is_valid) {
                CONSOLE_INFO(
                    "UATPacketDecoder::Update", "-[%dFIXD     ] mdb_tc=%d icao=0x%06x len=%d buf=%s rssi=%d ts=%lu",
                    decoded_packet.raw.sigq_bits, decoded_packet.header.mdb_type_code,
                    decoded_packet.header.icao_address, packet_len_bytes, raw_packet_buffer, packet.sigs_dbm,
                    packet.mlat_48mhz_64bit_counts);
            } else {
                CONSOLE_INFO("UATPacketDecoder::Update", "-[     INVLD] len=%d buf=%s rssi=%d ts=%lu", packet_len_bytes,
                             raw_packet_buffer, packet.sigs_dbm, packet.mlat_48mhz_64bit_counts);
            }
        }
    }

    // Decode incoming UAT uplink packets and push valid ones to the output queue.
    while (true) {
        RawUATUplinkPacket packet;
        uintptr_t key = HwiP_disable();
        bool got_packet = raw_uat_uplink_packet_queue.Dequeue(packet);
        HwiP_restore(key);
        if (!got_packet) {
            break;
        }
        DecodedUATUplinkPacket decoded_packet = DecodedUATUplinkPacket(packet);

        if (decoded_packet.is_valid) {
            leds.FlashLED(bsp.kSubGLEDPin, 50);
            if (!decoded_uat_uplink_packet_out_queue.Enqueue(decoded_packet)) {
                CONSOLE_ERROR("UATPacketDecoder::Update", "UAT uplink decoded output queue overflowed.");
            }
        }

        // Per-packet prints, gated on log level up front (mirrors the ADS-B path above).
        if (settings_manager.settings.log_level >= SettingsManager::LogLevel::kInfo) {
            if (decoded_packet.is_valid) {
                CONSOLE_INFO("UATPacketDecoder::Update", "+[%02dFIXD     ] len=%d rssi=%d ts=%lu",
                             decoded_packet.raw.sigq_bits, RawUATUplinkPacket::kUplinkMessageNumBytes,
                             packet.sigs_dbm, packet.mlat_48mhz_64bit_counts);
            } else {
                CONSOLE_INFO("UATPacketDecoder::Update", "+[     INVLD] len=%d rssi=%d ts=%lu",
                             RawUATUplinkPacket::kUplinkMessageNumBytes, packet.sigs_dbm,
                             packet.mlat_48mhz_64bit_counts);
            }
        }
    }
    return true;
}
