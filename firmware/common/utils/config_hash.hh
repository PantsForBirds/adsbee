#pragma once

#include <cstdint>
#include <cstring>

#include "crc.hh"

/**
 * Running CRC32 (IEEE 802.3, same as zlib) for AT+CONFIG_HASH. Values are added in one fixed form on every target:
 * integers little-endian, booleans as 0 or 1, strings zero-padded to their buffer size.
 */
class ConfigHasher {
   public:
    void AddBytes(const uint8_t* buf, uint32_t len_bytes) {
        value_ = crc32_ieee_802_3(buf, len_bytes, Reflect(value_ ^ 0xFFFFFFFF));
    }

    void AddU8(uint8_t value) { AddBytes(&value, 1); }
    void AddBool(bool value) { AddU8(value ? 1 : 0); }
    void AddU16(uint16_t value) {
        uint8_t buf[2] = {static_cast<uint8_t>(value), static_cast<uint8_t>(value >> 8)};
        AddBytes(buf, sizeof(buf));
    }
    void AddU32(uint32_t value) {
        uint8_t buf[4] = {static_cast<uint8_t>(value), static_cast<uint8_t>(value >> 8),
                          static_cast<uint8_t>(value >> 16), static_cast<uint8_t>(value >> 24)};
        AddBytes(buf, sizeof(buf));
    }
    void AddI32(int32_t value) { AddU32(static_cast<uint32_t>(value)); }
    void AddFloat(float value) {
        uint32_t bits;
        memcpy(&bits, &value, sizeof(bits));
        AddU32(bits);
    }

    /**
     * Adds a string as buf_len_bytes Bytes: the characters before the first NUL, then zeros.
     */
    void AddString(const char* str, uint16_t buf_len_bytes) {
        uint16_t len = strnlen(str, buf_len_bytes);
        AddBytes(reinterpret_cast<const uint8_t*>(str), len);
        for (uint16_t i = len; i < buf_len_bytes; i++) {
            AddU8(0);
        }
    }

    uint32_t Value() const { return value_; }

    /**
     * Hash of several hashes, in the order given.
     */
    static uint32_t Combine(const uint32_t* hashes, uint16_t num_hashes) {
        ConfigHasher hasher;
        for (uint16_t i = 0; i < num_hashes; i++) {
            hasher.AddU32(hashes[i]);
        }
        return hasher.Value();
    }

    /**
     * Adds a receiver position. Coordinates count only for a fixed position and the ICAO address only when it selects
     * the aircraft to follow: the other sources overwrite them while running.
     * @param[in] include_values False to add the position source alone.
     */
    template <typename RxPosition>
    void AddRxPosition(const RxPosition& position, bool include_values) {
        AddU8(position.source);
        if (!include_values) {
            return;
        }
        if (position.source == RxPosition::kPositionSourceFixed) {
            AddU32(position.latitude_awb);
            AddU32(position.longitude_awb);
            AddI32(position.gnss_altitude_ft);
            AddI32(position.baro_altitude_ft);
            AddFloat(position.heading_deg);
            AddI32(position.speed_kts);
        } else if (position.source == RxPosition::kPositionSourceAircraftMatchingICAO) {
            AddU32(position.icao_address);
        }
    }

   private:
    static uint32_t Reflect(uint32_t value) {
        uint32_t reflected = 0;
        for (uint16_t i = 0; i < 32; i++) {
            reflected = (reflected << 1) | ((value >> i) & 1);
        }
        return reflected;
    }

    uint32_t value_ = 0;  // CRC32 of the Bytes added so far.
};
