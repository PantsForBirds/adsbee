#include "uat_packet_decoder.hh"

#include <ti/drivers/dpl/HwiP.h>

#include "adsl_packet.hh"
#include "buffer_utils.hh"
#include "comms.hh"
#include "object_dictionary.hh"
#include "pico.hh"

// The raw UAT queues are written by the RF callback and the object dictionary queues are read by the SPI callback (both
// software interrupts). The ADS-B queues overwrite when full, which is not interrupt safe, so mask interrupts around
// every queue call made here. Don't use PFBQueue's is_thread_safe flag: its lock is not usable in interrupt context.
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
    while (true) {
        RawUATADSBPacket packet;
        uintptr_t key = HwiP_disable();
        bool got_packet = raw_uat_adsb_packet_queue.Dequeue(packet);
        HwiP_restore(key);
        if (!got_packet) {
            break;
        }
        pico_ll.ProcessPendingTransaction();  // Don't keep the RP2040 waiting while a burst is decoded.
        if (packet.protocol == RawUATADSBPacket::kProtocolADSL) {
            DecodeADSLPacket(packet);
            continue;
        }
        uint16_t packet_len_bytes = packet.buffer_len_bytes;
        // Decode the packet and enqueue the result.
        DecodedUATADSBPacket decoded_packet = DecodedUATADSBPacket(packet);

        char raw_packet_buffer[2 * packet_len_bytes + 1];
        ByteBufferToHexString(raw_packet_buffer, packet.buffer, packet_len_bytes);

        if (decoded_packet.is_valid) {
            pico_ll.BlinkSubGLED();
            object_dictionary.metrics.num_valid_uat_adsb_packets++;
            CONSOLE_INFO(
                "UATPacketDecoder::Update", "-[%dFIXD     ] mdb_tc=%d icao=0x%06x len=%d buf=%s rssi=%d ts=%lu",
                decoded_packet.raw.sigq_bits, decoded_packet.header.mdb_type_code, decoded_packet.header.icao_address,
                packet_len_bytes, raw_packet_buffer, packet.sigs_dbm, packet.mlat_48mhz_64bit_counts);
            key = HwiP_disable();
            bool enqueued = object_dictionary.raw_uat_adsb_packet_queue.Enqueue(decoded_packet.raw);
            HwiP_restore(key);
            if (!enqueued) {
                CONSOLE_ERROR("UATPacketDecoder::Update", "Failed to enqueue decoded UAT ADS-B packet.");
            }
        } else {
            CONSOLE_INFO("UATPacketDecoder::Update", "-[     INVLD] len=%d buf=%s rssi=%d ts=%lu", packet_len_bytes,
                         raw_packet_buffer, packet.sigs_dbm, packet.mlat_48mhz_64bit_counts);
        }
    }
}

void UATPacketDecoder::DecodeADSLPacket(const RawUATADSBPacket& packet) {
    RawADSLPacket adsl_packet = RawADSLPacket(packet);
    uint16_t num_erasures = adsl_packet.sigq_bits;
    if (!adsl_packet.RepairErasures()) {
        CONSOLE_INFO("UATPacketDecoder::Update", "~[     INVLD] len=%d erasures=%d rssi=%d ts=%lu", packet.buffer_len_bytes,
                     num_erasures, packet.sigs_dbm, (uint32_t)packet.mlat_48mhz_64bit_counts);
        return;
    }
    pico_ll.BlinkSubGLED();
    object_dictionary.metrics.num_valid_uat_adsb_packets++;
    char hex[2 * RawADSLPacket::kMaxPacketLenBytes + 1];
    ByteBufferToHexString(hex, adsl_packet.buffer, adsl_packet.buffer_len_bytes);
    CONSOLE_INFO("UATPacketDecoder::Update", "~[%dFIXD     ] len=%d buf=%s rssi=%d ch=%d ts=%lu", adsl_packet.sigq_bits,
                 adsl_packet.buffer_len_bytes, hex, adsl_packet.sigs_dbm, adsl_packet.channel,
                 (uint32_t)adsl_packet.mlat_48mhz_64bit_counts);
    uintptr_t key = HwiP_disable();
    bool enqueued = object_dictionary.raw_uat_adsb_packet_queue.Enqueue(adsl_packet);
    HwiP_restore(key);
    if (!enqueued) {
        CONSOLE_ERROR("UATPacketDecoder::Update", "Failed to enqueue decoded ADS-L packet.");
    }
}

void UATPacketDecoder::DecodeUplinkPacket() {
    RawUATUplinkPacket packet;
    uintptr_t key = HwiP_disable();
    bool got_packet = raw_uat_uplink_packet_queue.Dequeue(packet);
    HwiP_restore(key);
    if (!got_packet) {
        return;
    }
    DecodedUATUplinkPacket decoded_packet = DecodedUATUplinkPacket(packet);

    if (decoded_packet.is_valid) {
        pico_ll.BlinkSubGLED();
        object_dictionary.metrics.num_valid_uat_uplink_packets++;
        CONSOLE_INFO("UATPacketDecoder::Update", "+[%02dFIXD     ] len=%d rssi=%d ts=%lu", decoded_packet.raw.sigq_bits,
                     RawUATUplinkPacket::kUplinkMessageNumBytes, packet.sigs_dbm, packet.mlat_48mhz_64bit_counts);
        key = HwiP_disable();
        bool enqueued = object_dictionary.raw_uat_uplink_packet_queue.Enqueue(decoded_packet.raw);
        HwiP_restore(key);
        if (!enqueued) {
            CONSOLE_ERROR("UATPacketDecoder::Update", "Failed to enqueue decoded UAT Uplink packet.");
        }
    } else {
        CONSOLE_INFO("UATPacketDecoder::Update", "+[     INVLD] len=%d rssi=%d ts=%lu",
                     RawUATUplinkPacket::kUplinkMessageNumBytes, packet.sigs_dbm, packet.mlat_48mhz_64bit_counts);
    }
}
