#include "packet_decoder.hh"

#include "buffer_utils.hh"
#include "comms.hh"
#include "settings.hh"

// Host tests build DecodeOne() without the SDK, so Update() and its includes are left out there.
#ifndef ON_HOST
#include "adsbee.hh"
#include "bsp.hh"
#include "cycle_counter.hh"
#include "led.hh"
#endif

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

#ifndef ON_HOST
bool PacketDecoder::Update() {
    // In DF17 sync mode the detector consumes the first 4 DF bits (1000), so every reconstructed
    // frame decodes as DF17 or DF16 -- and only DF17 is wanted (DF16/ACAS remains receivable in the
    // preamble modes). Latch the mode once per pass to skip work on everything else below.
    const bool df17_mode = LR2021::IsOokDF17PreambleMode(adsbee.GetR1090PreambleMode());
    RawModeSPacket raw_packet;
    while (raw_mode_s_packet_queue.Dequeue(raw_packet)) {
        const uint32_t start_cycles = CycleCounter::Now();
        DecodeOne(raw_packet, df17_mode);
        const uint32_t cycles = CycleCounter::Since(start_cycles);
        if (cycles > decode_max_cycles) decode_max_cycles = cycles;
    }
    return true;
}
#endif

void PacketDecoder::DecodeOne(RawModeSPacket& raw_packet, bool df17_mode) {
    // The LR2021 always captures 112 bits, so cut squitters (DF < 16) to 56 bits before the CRC check. DF17 mode
    // only produces DF >= 16 frames.
    if (!df17_mode && raw_packet.buffer_len_bytes == RawModeSPacket::kExtendedSquitterPacketLenBytes &&
        (raw_packet.buffer[0] >> 27) < 16) {
        raw_packet = RawModeSPacket(raw_packet.buffer, RawModeSPacket::kSquitterPacketNumWords32,
                                    raw_packet.source, raw_packet.sigs_dbm, raw_packet.sigq_db,
                                    raw_packet.mlat_48mhz_64bit_counts);
    }
    DecodedModeSPacket decoded_packet(raw_packet);

    // Log every packet with its CRC residual: 0 for a valid DF17 frame, the ICAO address for address-parity frames.
    // Checked against the log level first, since CONSOLE_INFO evaluates its arguments even when filtered out.
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
        // DF17 mode: drop everything else (noise triggers or real DF16) before the costlier steps below.
        return;
    }

    if (decoded_packet.is_valid || decoded_packet.is_address_parity) {
        // Address-parity frames (DF 0/4/5/16/20/21) carry ICAO ^ CRC in the parity field; the aircraft dictionary
        // accepts them only if that ICAO matches a tracked aircraft.
        decoded_mode_s_packet_out_queue.Enqueue(decoded_packet);
        // leds.FlashLED(bsp.k1090LEDPin, 10);
    } else if (decoded_packet.CorrectSingleBitError() >= 0) {
        // Single-bit error correction, shared with the ADSBee 1090: only DF 17/18, never in the DF field
        // (DO-260B 2.2.4.3.4.7.3.a).
        bitflips_fixed_count++;
        decoded_mode_s_packet_out_queue.Enqueue(decoded_packet);
    }
}
