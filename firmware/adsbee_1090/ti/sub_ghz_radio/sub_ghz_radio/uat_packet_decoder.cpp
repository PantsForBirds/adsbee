#include "uat_packet_decoder.hh"

#include <ti/drivers/dpl/HwiP.h>

#include "buffer_utils.hh"
#include "comms.hh"
#include "object_dictionary.hh"
#include "pico.hh"

// The raw UAT queues are filled from the RF callback (software interrupt context), and PFBQueue is not interrupt-safe,
// so mask interrupts around each consumer-side queue operation. Don't use PFBQueue's is_thread_safe flag: its blocking
// mutex would deadlock in interrupt context.
bool UATPacketDecoder::Update() {
    // Process incoming UAT ADS-B packets.
    while (true) {
        RawUATADSBPacket packet;
        uintptr_t key = HwiP_disable();
        bool got_packet = raw_uat_adsb_packet_queue.Dequeue(packet);
        HwiP_restore(key);
        if (!got_packet) {
            break;
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
            if (!object_dictionary.raw_uat_adsb_packet_queue.Enqueue(decoded_packet.raw)) {
                CONSOLE_ERROR("UATPacketDecoder::Update", "Failed to enqueue decoded UAT ADS-B packet.");
            }
        } else {
            CONSOLE_INFO("UATPacketDecoder::Update", "-[     INVLD] len=%d buf=%s rssi=%d ts=%lu", packet_len_bytes,
                         raw_packet_buffer, packet.sigs_dbm, packet.mlat_48mhz_64bit_counts);
        }
    }

    while (true) {
        RawUATUplinkPacket packet;
        uintptr_t key = HwiP_disable();
        bool got_packet = raw_uat_uplink_packet_queue.Dequeue(packet);
        HwiP_restore(key);
        if (!got_packet) {
            break;
        }
        // Decode the packet and enqueue the result.
        DecodedUATUplinkPacket decoded_packet = DecodedUATUplinkPacket(packet);

        if (decoded_packet.is_valid) {
            pico_ll.BlinkSubGLED();
            object_dictionary.metrics.num_valid_uat_uplink_packets++;
            CONSOLE_INFO("UATPacketDecoder::Update", "+[%02dFIXD     ] len=%d rssi=%d ts=%lu",
                         decoded_packet.raw.sigq_bits, RawUATUplinkPacket::kUplinkMessageNumBytes, packet.sigs_dbm,
                         packet.mlat_48mhz_64bit_counts);
            if (!object_dictionary.raw_uat_uplink_packet_queue.Enqueue(decoded_packet.raw)) {
                CONSOLE_ERROR("UATPacketDecoder::Update", "Failed to enqueue decoded UAT Uplink packet.");
            }
        } else {
            CONSOLE_INFO("UATPacketDecoder::Update", "+[     INVLD] len=%d rssi=%d ts=%lu",
                         RawUATUplinkPacket::kUplinkMessageNumBytes, packet.sigs_dbm, packet.mlat_48mhz_64bit_counts);
        }
    }
    return true;
}
