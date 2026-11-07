#include "uat_packet_decoder.hh"

#include <ti/drivers/dpl/HwiP.h>

#include "adsl_packet.hh"
#include "buffer_utils.hh"
#include "comms.hh"
#include "led.hh"

// The raw UAT queues are filled from the RF-core ISR callback (SubGHzRadio::HandlePacketRx) and
// PFBQueue is not ISR-safe, so consumer-side dequeues mask interrupts around the queue operation.
// On this single-core M4F that is sufficient: the ISR producer can never observe (or interleave
// with) a half-updated head/tail. Do NOT use PFBQueue's is_thread_safe flag here -- its mutex
// backend is a no-op stub on this platform and a blocking mutex would deadlock in ISR context.

bool UATPacketDecoder::Update() {
    DecodeADSBPackets();
    DecodeUplinkPacket();

    uint32_t num_dropped = num_raw_uat_uplink_packets_dropped;
    if (num_dropped != num_raw_uat_uplink_packets_dropped_logged_) {
        CONSOLE_WARNING("UATPacketDecoder::Update", "Dropped %lu UAT uplinks: decoder queue full.",
                        num_dropped - num_raw_uat_uplink_packets_dropped_logged_);
        num_raw_uat_uplink_packets_dropped_logged_ = num_dropped;
    }
    return true;
}

void UATPacketDecoder::DecodeADSBPackets() {
    // Decode incoming UAT ADS-B packets and push valid ones to the output queue.
    while (true) {
        RawUATADSBPacket packet;
        uintptr_t key = HwiP_disable();
        bool got_packet = raw_uat_adsb_packet_queue.Dequeue(packet);
        HwiP_restore(key);
        if (!got_packet) {
            break;
        }
        if (packet.protocol == RawUATADSBPacket::kProtocolADSL) {
            DecodeADSLPacket(packet);
            continue;
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
}

void UATPacketDecoder::DecodeADSLPacket(const RawUATADSBPacket& packet) {
    RawADSLPacket adsl_packet = RawADSLPacket(packet);
    uint16_t num_erasures = adsl_packet.sigq_bits;
    bool valid = adsl_packet.RepairErasures();
    if (valid) {
        leds.FlashLED(bsp.kSubGLEDPin, 50);
        // The decoded queue carries ADS-L packets in raw (raw.protocol); nothing else of the UAT decode is used.
        DecodedUATADSBPacket decoded_packet;
        decoded_packet.raw = adsl_packet;
        decoded_packet.is_valid = true;
        if (!decoded_uat_adsb_packet_out_queue.Enqueue(decoded_packet)) {
            CONSOLE_ERROR("UATPacketDecoder::Update", "UAT ADS-B decoded output queue overflowed.");
        }
    }
    if (settings_manager.settings.log_level >= SettingsManager::LogLevel::kInfo) {
        if (valid) {
            char hex[2 * RawADSLPacket::kMaxPacketLenBytes + 1];
            ByteBufferToHexString(hex, adsl_packet.buffer, adsl_packet.buffer_len_bytes);
            CONSOLE_INFO("UATPacketDecoder::Update", "~[%dFIXD     ] len=%d buf=%s rssi=%d ch=%d ts=%lu",
                         adsl_packet.sigq_bits, adsl_packet.buffer_len_bytes, hex, adsl_packet.sigs_dbm,
                         adsl_packet.channel, (uint32_t)adsl_packet.mlat_48mhz_64bit_counts);
        } else {
            CONSOLE_INFO("UATPacketDecoder::Update", "~[     INVLD] len=%d erasures=%d rssi=%d ts=%lu",
                         packet.buffer_len_bytes, num_erasures, packet.sigs_dbm,
                         (uint32_t)packet.mlat_48mhz_64bit_counts);
        }
    }
}

void UATPacketDecoder::DecodeUplinkPacket() {
    // Decode one incoming UAT uplink packet and push it to the output queue if valid.
    RawUATUplinkPacket packet;
    uintptr_t key = HwiP_disable();
    bool got_packet = raw_uat_uplink_packet_queue.Dequeue(packet);
    HwiP_restore(key);
    if (!got_packet) {
        return;
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
