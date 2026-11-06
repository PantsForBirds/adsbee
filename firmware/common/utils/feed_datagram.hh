#pragma once

#include <cstdint>
#include <cstring>

#include "settings.hh"

// Splits a batch of reporting frames into UDP datagrams that hold whole frames only.
namespace FeedDatagram {

// Fits a datagram into the IPv6 minimum MTU (1280 bytes) after the IPv6 and UDP headers, so it is never fragmented.
static constexpr uint16_t kMaxPayloadBytes = 1232;

/**
 * Length of the frame that starts at buf[0], or len if the protocol has no frame boundaries this function knows.
 */
inline uint16_t FrameLen(SettingsManager::ReportingProtocol protocol, const uint8_t* buf, uint16_t len) {
    switch (protocol) {
        case SettingsManager::kBeast:
        case SettingsManager::kBeastNoUAT:
        case SettingsManager::kBeastNoUATUplink:
            // Frames start with 0x1A and a type byte; 0x1A in a frame is sent twice.
            for (uint16_t i = 1; i + 1 < len; i++) {
                if (buf[i] != 0x1A) continue;
                if (buf[i + 1] != 0x1A) return i;
                i++;
            }
            return len;
        case SettingsManager::kGDL90:
        case SettingsManager::kGDL90NoUATUplink: {
            // Frames start and end with a 0x7E flag byte.
            const void* end = len > 1 ? memchr(buf + 1, 0x7E, len - 1) : nullptr;
            return end ? static_cast<const uint8_t*>(end) - buf + 1 : len;
        }
        case SettingsManager::kMAVLINK1:
        case SettingsManager::kMAVLINK2: {
            // Version 1: 6 header bytes, payload, 2 checksum bytes. Version 2: 10 header bytes, payload, 2 checksum
            // bytes and a 13 byte signature if the first incompatibility flag is set.
            uint16_t frame_len = len;
            if (len >= 2 && buf[0] == 0xFE) {
                frame_len = buf[1] + 8;
            } else if (len >= 3 && buf[0] == 0xFD) {
                frame_len = buf[1] + 12 + ((buf[2] & 0x01) ? 13 : 0);
            }
            return frame_len < len ? frame_len : len;
        }
        case SettingsManager::kASTERIX: {
            // Data blocks start with a category byte and a 2 byte big-endian length that includes those 3 bytes.
            uint16_t block_len = len >= 3 ? (buf[1] << 8) | buf[2] : len;
            return block_len >= 3 && block_len < len ? block_len : len;
        }
        case SettingsManager::kRaw:
        case SettingsManager::kCSBee:
        case SettingsManager::kAircraftJSON:
        case SettingsManager::kAircraftJSONUATUplink: {
            // One line per frame.
            const void* newline = memchr(buf, '\n', len);
            return newline ? static_cast<const uint8_t*>(newline) - buf + 1 : len;
        }
        default:
            return len;
    }
}

/**
 * Length of the next datagram to send from buf: as many whole frames as fit in max_len, or one frame that is longer.
 */
inline uint16_t NextLen(SettingsManager::ReportingProtocol protocol, const uint8_t* buf, uint16_t len,
                        uint16_t max_len = kMaxPayloadBytes) {
    if (len <= max_len) {
        return len;
    }
    uint16_t datagram_len = FrameLen(protocol, buf, len);
    while (datagram_len < len) {
        uint16_t frame_len = FrameLen(protocol, buf + datagram_len, len - datagram_len);
        if (datagram_len + frame_len > max_len) {
            break;
        }
        datagram_len += frame_len;
    }
    return datagram_len;
}

}  // namespace FeedDatagram
