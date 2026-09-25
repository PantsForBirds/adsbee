# Getting Started in WSL2
## Installation
1. Install idfx following [these instructions](https://github.com/abobija/idfx).
2. Install ESP-IDF in WSL2 following [this script](https://gist.github.com/abobija/2f11d1b2c7cb079bec4df6e2348d969f).

## Helpful commands
* `idf.py set-target esp32s3`
* `idf.py menuconfig`
* `idf.py build`
* `idfx flash COM2`
* `idfx monitor COM2`

## Killing a rogue OpenOCD app (in case debugger won't let you bind to a port because it's already in use)

### Kill with PID:
`lsof -n -i | grep ":6666"` (or whatever the port is).
`kill <pid>` with pid set to the process number for OpenOCD.

### Kill with Process Name:
`pkill openocd`
## One image for both ESP32-S3 modules (PSRAM / no PSRAM)
The ESP32 firmware embedded in `combined.uf2` runs on both ESP32-S3 modules used on ADSBee 1090 hardware:

| Module | Flash | PSRAM | Hardware |
|--------|-------|-------|----------|
| ESP32-S3-MINI-1U-N8   | 8 MB | none | original ADSBee 1090U |
| ESP32-S3-MINI-1U-N4R2 | 4 MB | 2 MB quad | later hardware revisions |

How it works:
* **Flash:** the image is built for 4 MB (`CONFIG_ESPTOOLPY_FLASHSIZE_4MB`) with `partitions_remote_id.csv`, whose
  nvs / phy_init / factory offsets (0x9000 / 0xF000 / 0x10000) are the same as every earlier release, so NVS survives
  an update. The 3 MB factory app ends at 0x310000. On the 8 MB module the upper 4 MB is unused.
* **PSRAM:** `CONFIG_SPIRAM` (quad, 80 MHz) is enabled with `CONFIG_SPIRAM_IGNORE_NOTFOUND`, so a module without PSRAM
  boots normally and the PSRAM probe fails without an error. Only options that fall back to internal RAM at runtime are used:
  * `SPIRAM_USE_MALLOC`: `malloc()` of blocks over `SPIRAM_MALLOC_ALWAYSINTERNAL` (16 KB) prefers PSRAM when it is present.
  * `SPIRAM_TRY_ALLOCATE_WIFI_LWIP`: WiFi/LWIP buffers prefer PSRAM and fall back to internal RAM. With
    `IGNORE_NOTFOUND`, IDF keeps the no-PSRAM WiFi/LWIP buffer counts, so the N8 behaves as before.
  * `SPIRAM_MALLOC_RESERVE_INTERNAL`: reserves internal DMA-capable RAM only if PSRAM was actually initialized.
  * `BT_NIMBLE_MEM_ALLOC_MODE_DEFAULT` (plain malloc). Do **not** use `..._EXTERNAL`: it allocates from PSRAM only and
    asserts when there is none.
  * Do **not** enable `SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY` / `..._NOINIT_...`: they cannot work without PSRAM (and
    Kconfig makes them mutually exclusive with `IGNORE_NOTFOUND`).
* **Runtime gating:** `HardwareCapabilities::Detect()` (`main/hardware_capabilities.*`) runs first in `app_main()` and
  checks `esp_psram_is_initialized()` / `heap_caps_get_total_size(MALLOC_CAP_SPIRAM)`. The result is published to the
  RP2040 in `ObjectDictionary::ESP32DeviceStatus::hardware_capabilities` (plus `psram_total_kb` / `psram_free_kb`).
  With PSRAM, Remote ID (BLE and the WiFi beacon sniffer) can run alongside WiFi AP/STA. Without PSRAM it only runs with
  WiFi AP/STA off, as before. The RP2040 rejects `AT+REMOTE_ID=1` / `AT+REMOTE_ID_TX=1` with a clear error while WiFi
  is enabled on hardware without PSRAM. `AT+DEVICE_INFO?` shows `ESP32 PSRAM: ...`.
* **Heap guards** (Remote ID start thresholds, `safe_send` back-pressure, websocket admission, `heap_free_bytes`
  telemetry) measure **internal** RAM (`HardwareCapabilities::kInternalHeapCaps`), so the ~2 MB of PSRAM does not hide an
  internal-RAM shortage. Without PSRAM this is the same number as before.

The former `sdkconfig.psram` overlay and `partitions_remote_id_4mb.csv` have been folded into `sdkconfig` /
`partitions_remote_id.csv` and removed. `sdkconfig.debug` still layers on top (coredump partition at 0x3C0000).
