// Symbols emitted by scripts/hex_to_c.py into the generated firmware_image.c: the baked-in
// adsbee_1421 firmware image as bootloader-ready segments (4-byte aligned, main flash first,
// CCFG last) with build-time CRC32s matching the ROM bootloader's CRC32 command.

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t addr;
    uint32_t len;
    uint32_t crc32;  // IEEE 802.3 (zlib) CRC32 of data, as the ROM CRC32 command computes.
    const uint8_t* data;
} FirmwareSegment;

extern const FirmwareSegment kFirmwareSegments[];  // Main-flash segments first, CCFG last.
extern const unsigned kFirmwareNumSegments;
extern const char kFirmwareVersionStr[];  // e.g. "0.3.5" or "0.3.5-rc1"; status prints only.

#ifdef __cplusplus
}
#endif
