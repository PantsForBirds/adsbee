#include "flasher.hh"

#include "firmware_image.hh"
#include "status.hh"

static const uint32_t kCcfgBase = 0x50000000;
static const uint32_t kFlashSectorSize = 0x800;  // CC1314R10 main-bank sector: 2 KB.
// Top of the main bank is persistent storage owned by the application, never by the image:
//   0x000FC000  8 KB  Settings
//   0x000FE000  8 KB  Device Info (part code, OTA keys)
// (see ti/settings/settings.cpp). Nothing regenerates Device Info, so the programmer must never
// erase or program at or above this address. This is why the flash below uses per-sector erases
// of only the image-covered sectors instead of COMMAND_BANK_ERASE, which wipes the whole bank.
// Boundary the BAKED IMAGE must never reach. Unrelated to EraseSettingsRegion() below, which erases
// part of this region on purpose: what must never be clobbered by a *flash* is the persistent data,
// and what must never be clobbered by *anything* is Device Info.
static const uint32_t kProtectedFlashStart = 0x000FC000;

// Settings region, erasable as an explicit recovery action (EraseSettingsRegion).
static const uint32_t kSettingsFlashStart = 0x000FC000;
// Device Info (part code, OTA keys). Nothing regenerates this -- never erase at or above it.
static const uint32_t kDeviceInfoFlashStart = 0x000FE000;
static_assert(kSettingsFlashStart < kDeviceInfoFlashStart, "Settings region must sit below Device Info.");

static inline uint32_t AlignDown(uint32_t addr) { return addr & ~(kFlashSectorSize - 1); }
static inline uint32_t AlignUp(uint32_t addr) { return (addr + kFlashSectorSize - 1) & ~(kFlashSectorSize - 1); }

bool BakedImageMatches(Cc13x4Bootloader& bl) {
    for (unsigned i = 0; i < kFirmwareNumSegments; i++) {
        const FirmwareSegment& seg = kFirmwareSegments[i];
        uint32_t device_crc;
        if (!bl.Crc32(seg.addr, seg.len, &device_crc)) return false;
        if (device_crc != seg.crc32) {
            CdcPrintf("CRC mismatch @ 0x%08lX: device 0x%08lX, baked 0x%08lX\r\n",
                      (unsigned long)seg.addr, (unsigned long)device_crc,
                      (unsigned long)seg.crc32);
            return false;
        }
    }
    return true;
}

// Erases exactly the main-bank sectors the baked image covers, leaving the Settings and Device
// Info sectors (and any other untouched sector) intact. Must complete for ALL main segments before
// anything is programmed: two segments can share a sector, and erasing it after the first was
// written would wipe that data. Segments arrive address-sorted from hex_to_c.py.
static FlashResult EraseImageSectors(Cc13x4Bootloader& bl) {
    // Refuse up front (before any erase) if the image reaches into the protected region. hex_to_c.py
    // rejects such an image at build time; this is the last line of defence.
    uint32_t first_sector = UINT32_MAX, last_sector = 0, num_sectors = 0;
    for (unsigned i = 0; i < kFirmwareNumSegments; i++) {
        const FirmwareSegment& seg = kFirmwareSegments[i];
        if (seg.addr >= kCcfgBase) continue;  // CCFG lives in its own bank; erased separately.
        if (seg.addr + seg.len > kProtectedFlashStart) {
            CdcPrintf("Refusing to flash: segment 0x%08lX (%lu bytes) overlaps the reserved "
                      "settings / device-info region at 0x%08lX.\r\n",
                      (unsigned long)seg.addr, (unsigned long)seg.len,
                      (unsigned long)kProtectedFlashStart);
            return FlashResult::kEraseFailed;
        }
        uint32_t start = AlignDown(seg.addr);
        uint32_t end = AlignUp(seg.addr + seg.len);
        if (start < first_sector) first_sector = start;
        if (end > last_sector) last_sector = end;
        num_sectors += (end - start) / kFlashSectorSize;  // Upper bound; shared sectors counted twice.
    }
    if (num_sectors == 0) return FlashResult::kOk;

    CdcPrintf("Erasing image sectors 0x%08lX..0x%08lX (device info / settings preserved)...\r\n",
              (unsigned long)first_sector, (unsigned long)last_sector);
    uint32_t erased = 0;
    bool have_last_erased = false;
    uint32_t last_erased = 0;
    for (unsigned i = 0; i < kFirmwareNumSegments; i++) {
        const FirmwareSegment& seg = kFirmwareSegments[i];
        if (seg.addr >= kCcfgBase) continue;
        uint32_t end = AlignUp(seg.addr + seg.len);
        for (uint32_t sector = AlignDown(seg.addr); sector < end; sector += kFlashSectorSize) {
            if (have_last_erased && sector <= last_erased) continue;  // Shared with the previous segment.
            if (!bl.SectorErase(sector)) {
                CdcPrintf("Sector erase failed @ 0x%08lX: %s\r\n", (unsigned long)sector, bl.LastError());
                return FlashResult::kEraseFailed;
            }
            last_erased = sector;
            have_last_erased = true;
            erased++;
            if (erased % 64 == 0) {
                CdcPrintf("  %lu sectors erased (through 0x%08lX)\r\n", (unsigned long)erased,
                          (unsigned long)(sector + kFlashSectorSize));
            }
        }
    }
    CdcPrintf("  %lu sectors erased.\r\n", (unsigned long)erased);
    return FlashResult::kOk;
}

FlashResult FlashBakedImage(Cc13x4Bootloader& bl) {
    uint32_t chip_id;
    if (bl.GetChipId(&chip_id)) {
        CdcPrintf("Chip ID: 0x%08lX\r\n", (unsigned long)chip_id);
    }

    // Per-sector erase of the image footprint only -- deliberately NOT a bank erase, which would
    // destroy the Device Info (OTA keys) and Settings sectors at the top of the bank.
    FlashResult erase_result = EraseImageSectors(bl);
    if (erase_result != FlashResult::kOk) return erase_result;
    // Erasing CCFG restores TI factory defaults (bootloader enabled), so an interrupted flash
    // is always recoverable over UART. The CCFG segment is programmed last for the same reason.
    CdcPrintf("Erasing CCFG sector...\r\n");
    if (!bl.SectorErase(kCcfgBase)) return FlashResult::kEraseFailed;

    for (unsigned i = 0; i < kFirmwareNumSegments; i++) {
        const FirmwareSegment& seg = kFirmwareSegments[i];
        CdcPrintf("Programming 0x%08lX (%lu bytes)\r\n", (unsigned long)seg.addr,
                  (unsigned long)seg.len);
        if (!bl.Download(seg.addr, seg.len)) return FlashResult::kProgramFailed;
        uint32_t sent = 0;
        uint32_t next_report = 0;
        while (sent < seg.len) {
            uint32_t chunk = seg.len - sent;
            if (chunk > Cc13x4Bootloader::kMaxDataChunk) chunk = Cc13x4Bootloader::kMaxDataChunk;
            if (!bl.SendData(seg.data + sent, chunk)) return FlashResult::kProgramFailed;
            sent += chunk;
            if (sent >= next_report) {
                CdcPrintf("  %lu/%lu bytes (%lu%%)\r\n", (unsigned long)sent,
                          (unsigned long)seg.len, (unsigned long)(100 * sent / seg.len));
                next_report += 65536;
            }
        }
    }

    StatusSet(Status::kVerifying);
    CdcPrintf("Verifying...\r\n");
    if (!BakedImageMatches(bl)) return FlashResult::kVerifyFailed;
    CdcPrintf("CRC32 verified, flash complete.\r\n");
    return FlashResult::kOk;
}

FlashResult EraseSettingsRegion(Cc13x4Bootloader& bl) {
    CdcPrintf("Erasing settings sectors 0x%08lX..0x%08lX (device info / OTA keys preserved)...\r\n",
              (unsigned long)kSettingsFlashStart, (unsigned long)(kDeviceInfoFlashStart - 1));
    for (uint32_t sector = kSettingsFlashStart; sector < kDeviceInfoFlashStart; sector += kFlashSectorSize) {
        // Belt and braces: Cc13x4Bootloader::SectorErase() takes any address, and erasing even one
        // sector at or above kDeviceInfoFlashStart would destroy the OTA keys irrecoverably.
        if (sector >= kDeviceInfoFlashStart) {
            CdcPrintf("Refusing to erase 0x%08lX: at or above the device info region.\r\n",
                      (unsigned long)sector);
            return FlashResult::kEraseFailed;
        }
        if (!bl.SectorErase(sector)) {
            CdcPrintf("Settings sector erase failed @ 0x%08lX: %s\r\n", (unsigned long)sector, bl.LastError());
            return FlashResult::kEraseFailed;
        }
    }
    CdcPrintf("Settings erased; the device will boot with factory defaults.\r\n");
    return FlashResult::kOk;
}
