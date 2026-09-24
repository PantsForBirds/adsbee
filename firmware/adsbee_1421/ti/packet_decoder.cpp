#include "packet_decoder.hh"

#include "adsbee.hh"
#include "bsp.hh"
#include "buffer_utils.hh"
#include "comms.hh"
#include "crc.hh"
#include "led.hh"

PacketDecoder::PacketDecoder()
    : raw_mode_s_packet_queue({
          .buf_len_num_elements = kRawModeSPacketQueueDepth,
          .buffer = raw_mode_s_packet_queue_buffer_,
          .overwrite_when_full = true,
          .is_thread_safe = false,
      }),
      decoded_mode_s_packet_out_queue({
          .buf_len_num_elements = kDecodedModeSPacketQueueDepth,
          .buffer = decoded_mode_s_packet_out_queue_buffer_,
          .overwrite_when_full = true,
          .is_thread_safe = false,
      }) {}

bool PacketDecoder::Update() {
    // In DF17 sync mode the detector consumes the first 4 DF bits (1000), so every reconstructed
    // frame decodes as DF17 or DF16 -- and only DF17 is wanted (DF16/ACAS remains receivable in the
    // preamble modes). Latch the mode once per pass to skip work on everything else below.
    const bool df17_mode = LR2021::IsOokDF17PreambleMode(adsbee.GetR1090PreambleMode());
    RawModeSPacket raw_packet;
    while (raw_mode_s_packet_queue.Dequeue(raw_packet)) {
        // The LR2021 always captures 112-bit (extended squitter length) frames, so a 56-bit squitter
        // (DF < 16: DF 0/4/5/11) arrives with 56 bits of noise appended. Truncate to squitter length
        // before decoding so the software CRC covers the real frame. DF17 sync mode reconstructs
        // frames with DF >= 16 by construction, so the check is skipped there.
        if (!df17_mode && raw_packet.buffer_len_bytes == RawModeSPacket::kExtendedSquitterPacketLenBytes &&
            (raw_packet.buffer[0] >> 27) < 16) {
            raw_packet = RawModeSPacket(raw_packet.buffer, RawModeSPacket::kSquitterPacketNumWords32,
                                        raw_packet.source, raw_packet.sigs_dbm, raw_packet.sigq_db,
                                        raw_packet.mlat_48mhz_64bit_counts);
        }
        DecodedModeSPacket decoded_packet(raw_packet);

        // Print every received packet (valid or not) with its raw contents, validity, and CRC residual.
        // The residual is calculated_checksum XOR received_parity: 0 for a valid DF17 frame, and the
        // ICAO address for address-parity frames (DF 0/4/5/16/20/21). Computed here from mode_s_packet's
        // public API (CalculateCRC24 + the raw buffer) so the shared module stays untouched. Gated on
        // log level up front: the residual recompute and buffer formatting are real per-packet work,
        // and CONSOLE_INFO would evaluate its arguments (then discard the output) even below kInfo.
        if (settings_manager.settings.log_level >= SettingsManager::LogLevel::kInfo) {
            const uint16_t len_bits = raw_packet.buffer_len_bytes * 8;  // 56 (squitter) or 112 (extended)
            const uint32_t crc_residual =
                decoded_packet.CalculateCRC24(len_bits) ^ Get24BitsFromWordBuffer(len_bits - 24, raw_packet.buffer);
            char print_buf[29];  // "%08lX%08lX%08lX%04lX" = 28 chars + null
            raw_packet.PrintBuffer(print_buf, sizeof(print_buf));
            CONSOLE_INFO("PacketDecoder::Update", "DF=%2u ICAO=%06lX valid=%u res=%06lX %s",
                         decoded_packet.downlink_format, (unsigned long)decoded_packet.icao_address,
                         decoded_packet.is_valid ? 1u : 0u, (unsigned long)crc_residual, print_buf);
        }

        if (df17_mode && decoded_packet.downlink_format != 17) {
            // DF17 mode: anything else (DF16-labeled noise triggers, or real DF16/ACAS) is dropped
            // before address-parity forwarding, bit correction, and the dictionary lookup they cost.
            // The debug print above still shows these frames at kInfo for bench visibility.
            continue;
        }

        if (decoded_packet.is_valid || decoded_packet.is_address_parity) {
            // Address-parity frames (DF 0/4/5/16/20/21) carry ICAO ^ CRC in the parity field and can't
            // be validated standalone; the aircraft dictionary promotes them to valid downstream iff
            // the recovered ICAO matches an aircraft it already tracks.
            decoded_mode_s_packet_out_queue.Enqueue(decoded_packet);
            // leds.FlashLED(bsp.k1090LEDPin, 10);
        } else if (decoded_packet.raw.buffer_len_bytes == RawModeSPacket::kExtendedSquitterPacketLenBytes) {
            // Extended squitter with a failed CRC: attempt single-bit error correction. The syndrome
            // maps to a unique flip position for any single-bit error; a successful flip re-decodes to
            // a frame whose CRC checks out. Capped at one bit to bound false corrections.
            uint8_t raw_buffer[RawModeSPacket::kExtendedSquitterPacketLenBytes];
            WordBufferToByteBuffer(decoded_packet.raw.buffer, raw_buffer, sizeof(raw_buffer));
            int16_t bit_flip_index = crc24_find_single_bit_error(crc24_syndrome(raw_buffer, sizeof(raw_buffer)),
                                                                 RawModeSPacket::kExtendedSquitterPacketLenBits);
            if (bit_flip_index > 0) {
                flip_bit(decoded_packet.raw.buffer, bit_flip_index);
                DecodedModeSPacket corrected_packet(decoded_packet.raw);
                if (corrected_packet.is_valid) {
                    bitflips_fixed_count++;
                    decoded_mode_s_packet_out_queue.Enqueue(corrected_packet);
                }
            }
        }
    }
    return true;
}
