#include "mode_s_packet_decoder.hh"

#include <cstring>  // For memcmp, memcpy.

#include "comms.hh"

// Uncomment the line below to allow duplicate packets (e.g. for testing).
// #define DISABLE_DUPLICATE_FILTER

bool ModeSPacketDecoder::UpdateLogLoop() {
    uint16_t num_messages = debug_message_out_queue.Length();
    for (uint16_t i = 0; i < num_messages; i++) {
        DebugMessage message;
        debug_message_out_queue.Dequeue(message);
        switch (message.log_level) {
            case SettingsManager::LogLevel::kInfo:
                CONSOLE_INFO("ModeSPacketDecoder::DecoderLoop", "%s", message.message);
                break;
            case SettingsManager::LogLevel::kWarnings:
                CONSOLE_WARNING("ModeSPacketDecoder::DecoderLoop", "%s", message.message);
                break;
            case SettingsManager::LogLevel::kErrors:
                CONSOLE_ERROR("ModeSPacketDecoder::DecoderLoop", "%s", message.message);
                break;
            default:
                break;  // Don't do anything when logs are silent.
        }
    }
    uint16_t bit_flip_index;
    while (decoded_mode_s_packet_bit_flip_locations_out_queue.Dequeue(bit_flip_index)) {
        CONSOLE_INFO("ModeSPacketDecoder::DecoderLoop", "Corrected single bit error at bit index %d.", bit_flip_index);
    }
    return true;
}

bool ModeSPacketDecoder::UpdateDecoderLoop() {
    uint16_t num_packets_to_process = raw_mode_s_packet_in_queue.Length();
    if (num_packets_to_process == 0) {
        return true;  // Nothing to do.
    }

    // Per-packet debug messages are only ever printed at log level kInfo. Skip formatting them entirely otherwise: this
    // loop runs for every demodulation attempt (most of which are noise), so the formatting cost is significant.
    debug_enabled_ = settings_manager.settings.log_level >= SettingsManager::LogLevel::kInfo;

    for (uint16_t i = 0; i < num_packets_to_process; i++) {
        RawModeSPacket raw_packet;
        if (!raw_mode_s_packet_in_queue.Dequeue(raw_packet)) {
            debug_message_out_queue.Enqueue(DebugMessage{
                .message = "Failed to pop raw packet from input queue.",
                .log_level = SettingsManager::LogLevel::kErrors,
            });
            return false;
        }

        // Short formats (DF 0-15) are always 56 bits, but the receiver may hand them over as 112 bits (e.g. a pulse
        // right after the message). Trim them back to 56 bits.
        RawModeSPacket framed_packet = raw_packet;
        if (framed_packet.buffer_len_bytes == RawModeSPacket::kExtendedSquitterPacketLenBytes &&
            (framed_packet.buffer[0] >> (kBytesPerWord * kBitsPerByte - DecodedModeSPacket::kDFNumBits)) <
                DecodedModeSPacket::kDownlinkFormatLongRangeAirToAirSurveillance) {
            framed_packet.buffer_len_bytes = RawModeSPacket::kSquitterPacketLenBytes;
            framed_packet.buffer[1] &= 0xFFFFFF00;
            framed_packet.buffer[2] = 0;
            framed_packet.buffer[3] = 0;
        }

        DecodedModeSPacket decoded_packet = DecodedModeSPacket(framed_packet);
        const char* status_str = nullptr;
        if (decoded_packet.is_valid) {
            PushPacketIfNotDuplicate(decoded_packet);
            status_str = "VALID     ";
        } else if (decoded_packet.is_address_parity) {
            // Forward for validation against ICAO addresses in the aircraft dictionary.
            PushPacketIfNotDuplicate(decoded_packet);
            status_str = "APFWD     ";
        } else if (config_.enable_1090_error_correction && decoded_packet.IsSingleBitCorrectable()) {
            // Checksum correction is enabled and the packet is a DF=17/18 extended squitter.
            int16_t bit_flip_index = decoded_packet.CorrectSingleBitError();
            if (bit_flip_index >= 0) {
                // Corrected a single bit error.
                decoded_mode_s_packet_bit_flip_locations_out_queue.Enqueue(bit_flip_index);
                PushPacketIfNotDuplicate(decoded_packet);
                status_str = "1FIXD     ";
            } else {
                // Checksum correction failed.
                status_str = "     NOFIX";
            }
        } else {
            // Invalid and not worth correcting.
            status_str = "     INVLD";
        }

        if (debug_enabled_) {
            DebugMessage debug_message = DebugMessage{
                .log_level = SettingsManager::LogLevel::kInfo,
            };
            int message_len = snprintf(debug_message.message, DebugMessage::kMessageMaxLen,
                                       "src=%d [%s] df=%02d icao=0x%06x ts=%llu ", decoded_packet.raw.source, status_str,
                                       decoded_packet.downlink_format, (unsigned)decoded_packet.icao_address,
                                       (unsigned long long)decoded_packet.raw.GetTimestampMs());
            // snprintf returns the length the full string would have had, so clamp on truncation. message[] is
            // kMessageMaxLen + 1 long, so message + kMessageMaxLen always points at the terminating null.
            if (message_len < 0) {
                message_len = 0;
            } else if (message_len >= DebugMessage::kMessageMaxLen) {
                message_len = DebugMessage::kMessageMaxLen;
            }
            // Append a print of the packet contents as received (before any bit flip correction).
            raw_packet.PrintBuffer(debug_message.message + message_len, DebugMessage::kMessageMaxLen - message_len);
            debug_message_out_queue.Enqueue(debug_message);
        }
    }

    return true;
}

bool ModeSPacketDecoder::PushPacketIfNotDuplicate(const DecodedModeSPacket& decoded_packet) {
    const RawModeSPacket& raw = decoded_packet.raw;
    int16_t packet_source = raw.source;

#ifndef DISABLE_DUPLICATE_FILTER
    // Check if we have already seen this exact packet from another source (got caught by multiple state machines
    // simultaneously). Compare only the packet's bits: the last word read from the demodulator may hold trailing bits.
    constexpr uint16_t kBitsPerWord = kBytesPerWord * kBitsPerByte;
    uint16_t num_words = (raw.buffer_len_bytes + kBytesPerWord - 1) / kBytesPerWord;
    uint16_t last_word_num_bits = raw.buffer_len_bytes * kBitsPerByte - (num_words - 1) * kBitsPerWord;
    uint32_t last_word_mask = num_words > 0 ? UINT32_MAX << (kBitsPerWord - last_word_num_bits) : 0;
    for (uint16_t i = 0; i < kMaxNumSources; i++) {
        const LastPacket& last = last_packet_[i];
        if (last.buffer_len_bytes != raw.buffer_len_bytes) {
            continue;
        }
        uint64_t delta_counts = raw.mlat_48mhz_64bit_counts >= last.mlat_48mhz_64bit_counts
                                    ? raw.mlat_48mhz_64bit_counts - last.mlat_48mhz_64bit_counts
                                    : last.mlat_48mhz_64bit_counts - raw.mlat_48mhz_64bit_counts;
        if (delta_counts >= kDuplicatePacketWindow48MHzCounts) {
            continue;
        }
        if (num_words == 0 || memcmp(last.buffer, raw.buffer, (num_words - 1) * kBytesPerWord) != 0 ||
            ((last.buffer[num_words - 1] ^ raw.buffer[num_words - 1]) & last_word_mask) != 0) {
            continue;
        }
        // Already seen this exact packet within the duplicate window.
        if (debug_enabled_) {
            DebugMessage debug_message = DebugMessage{
                .log_level = SettingsManager::LogLevel::kInfo,
            };
            snprintf(debug_message.message, DebugMessage::kMessageMaxLen,
                     "ModeSPacketDecoder::PushPacketIfNotDuplicate: Skipped duplicate packet with icao=0x%x src=%d "
                     "(first seen from src=%d).",
                     (unsigned)decoded_packet.icao_address, packet_source, i);
            debug_message_out_queue.Enqueue(debug_message);
        }
        return false;
    }
#endif  // DISABLE_DUPLICATE_FILTER

    if (!decoded_mode_s_packet_out_queue.Enqueue(decoded_packet)) {
        decoded_mode_s_packet_out_queue_overflowed_ = true;
    }

    if (packet_source >= 0 && packet_source < kMaxNumSources) {
        // Only update the packet cache if the source is valid.
        LastPacket& last = last_packet_[packet_source];
        memcpy(last.buffer, raw.buffer, sizeof(last.buffer));
        last.buffer_len_bytes = raw.buffer_len_bytes;
        last.mlat_48mhz_64bit_counts = raw.mlat_48mhz_64bit_counts;
    }

    return true;
}
