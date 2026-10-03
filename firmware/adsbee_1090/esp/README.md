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
The ESP32 firmware in `combined.uf2` runs on both modules used on ADSBee 1090 hardware:

| Module | Flash | PSRAM | Hardware |
|--------|-------|-------|----------|
| ESP32-S3-MINI-1U-N8   | 8 MB | none | original ADSBee 1090U |
| ESP32-S3-MINI-1U-N4R2 | 4 MB | 2 MB quad | later hardware revisions |

* **Flash:** built for 4 MB with `partitions_remote_id.csv`. The nvs / phy_init / factory offsets (0x9000 / 0xF000 /
  0x10000) never change, so settings survive updates.
* **PSRAM:** enabled with `CONFIG_SPIRAM_IGNORE_NOTFOUND`, so modules without PSRAM boot normally. Only use options
  that fall back to internal RAM. Never enable `BT_NIMBLE_MEM_ALLOC_MODE_EXTERNAL` or
  `SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY` / `..._NOINIT_...`: they crash without PSRAM.
* **Remote ID:** with PSRAM it runs alongside WiFi. Without PSRAM it needs WiFi off, and `AT+REMOTE_ID=1` /
  `AT+REMOTE_ID_TX=1` are rejected while WiFi is on. `AT+DEVICE_INFO?` shows `ESP32 PSRAM: ...`.
* **Code:** gate RAM-hungry features on `HardwareCapabilities` (`main/hardware_capabilities.*`). Heap guards measure
  internal RAM only.
* `sdkconfig.debug` layers on top (coredump partition at 0x3C0000).
