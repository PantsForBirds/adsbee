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
* **Remote ID:** with PSRAM it runs alongside WiFi and Ethernet. Without PSRAM it can't run while WiFi is on, and
  only its Bluetooth transports run while Ethernet is on (Bluetooth plus the WiFi beacon receiver leave too little RAM
  for the web server).
  `AT+REMOTE_ID` / `AT+REMOTE_ID_TX` reject other transports, and saved ones pause at boot. `AT+DEVICE_INFO?` shows
  `ESP32 PSRAM: ...` and the ESP32 heap.
* **Remote ID AT:** transports are names joined with `+` (`BT4`, `BT5_LR`, `WIFI_BEACON`), or `ALL` / `NONE`, e.g.
  `AT+REMOTE_ID=1,BT4+BT5_LR` (numbers are refused). Queries print names and the ESP32 status as words
  (`RemoteIDGuard::kStatusNames`), e.g.
  `REMOTE_ID=1,BT4+BT5_LR(TRANSPORTS),BT_SCANNING+BT5_LR_SCANNING(ESP32_STATUS),BT4+BT5_LR(ALLOWED_TRANSPORTS)`.
  `AT+SETTINGS?JSON` keeps the numbers.
* **Heap reserve:** Remote ID pauses while ESP32 free heap is below `RemoteIDGuard::kReserveFreeBytes`
  (`firmware/common/coprocessor/remote_id_guard.hh`) and retries after a growing hold-off.
* **Bluetooth RAM:** NimBLE is an observer + broadcaster with trimmed pools (8 HCI event buffers, 1 ACL buffer, 3
  MSYS_2 blocks, no periodic advertising/sync, 3 controller activities (enough for receive and both transmit sets), 3.5 KB host
  stack). Bluetooth costs about
  31 KB in the controller and 13 KB in the host while running and frees it all when stopped. Measure before raising
  any of these.
* **Bluetooth controller in flash:** `CONFIG_BT_CTRL_RUN_IN_FLASH_ONLY` links the controller from flash instead of
  IRAM. That pays for advertising (`CONFIG_BT_CTRL_BLE_ADV`, needed for Remote ID transmit, ~4.5 KB in IRAM) and
  still leaves ~8.5 KB more heap on every unit, at ~120 KB of flash. Bluetooth pauses while the flash cache is off
  (flash writes), which Remote ID tolerates.
* **Web server RAM:** each viewer pins the data queued on its sockets until the browser acknowledges it. lwIP's send
  buffer is 4 KB per socket with quarter-MSS oversize, Live Map frames are 1 KB, a Live Map report starts between
  two metrics messages so both bursts don't queue at once, and 3 HTTP sessions serve a page load in turn. Measure
  the heap with 3 viewers before raising any of these.
* **Code:** gate RAM-hungry features on `HardwareCapabilities` (`main/hardware_capabilities.*`). Heap guards measure
  internal RAM only.
* `sdkconfig.debug` layers on top (coredump partition at 0x3C0000).
