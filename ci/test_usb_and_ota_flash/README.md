# test_usb_and_ota_flash

CI hardware test suite for ADSBee: flashes firmware over USB (BOOTSEL/UF2) and
over the air (OTA), then verifies the device comes back healthy and running
the expected build.

## Architecture (from AGENTS.md)

- USB connects to the RP2040 (main processor).
- `combined.uf2` embeds all three processor binaries (RP2040 + ESP32 + CC1312).
  Flashing: putting the RP2040 in BOOTSEL mode mounts it as an `RPI-RP2` USB
  mass-storage drive; copying `combined.uf2` onto it triggers an automatic
  reboot.
- OTA (`.ota` file) updates the RP2040 via the ESP32's WebSocket → SPI path.
- Health is checked via the ESP32's `/metrics` WebSocket over WiFi, with an
  optional firmware-version check over the `/console` WebSocket.

## Files

| File | Role |
|---|---|
| `test_ota.py` | Entry point. Orchestrates the full flash → verify → OTA → verify → confirm-boot sequence. |
| `reboot_to_bootloader.py` | Puts the RP2040 into BOOTSEL mode, via USB serial or the console WebSocket. |
| `ota_upload.py` | Speaks the `AT+OTA=...` protocol over the console WebSocket to erase/write/verify/boot an `.ota` image. Mirrors the `FirmwareUploader`/`ADSBeeAT` logic in `adsbee.js` so CI can drive it without a browser. |
| `check_device.py` | Polls `/metrics` to confirm both the ESP32 and RP2040 are alive (and the RP2040's reported uptime is actively advancing, not just stale/cached), plus an optional `AT+DEVICE_INFO?` version check. |
| `requirements.txt` | Python deps (`websockets`). |

`test_ota.py` imports the other three as siblings via `sys.path`, so this
directory is meant to be invoked as a unit, not split apart.

## Test steps

1. Trigger BOOTSEL, wait for the `RPI-RP2` USB drive, flash `combined.uf2`.
2. Verify base firmware health via `/metrics`.
3. Upload the candidate `.ota` via the WebSocket AT protocol `[full test / --ota-only]`.
4. Verify post-OTA firmware health `[full test / --ota-only]`.
5. Confirm the device actually booted the new partition (the active/complementary
   partition index must flip) `[full test / --ota-only]` — catches a no-op OTA
   that a same-build version check alone can't.

## Usage

```bash
# Full test: USB flash, then OTA, with health checks at each stage.
python3 test_ota.py --uf2 combined.uf2 --ota-fw adsbee_1090.ota

# USB flash + health check only; skip OTA steps.
python3 test_ota.py --usb-only --uf2 combined.uf2

# OTA upload + health check only; device must already be running base firmware.
python3 test_ota.py --ota-only --ota-fw adsbee_1090.ota
```

Run `python3 test_ota.py --help` for the full flag list (`--port`/`--host` for
transport selection, `--expected-version` to assert the reported firmware
version, timeouts, etc.).

## CI integration

Invoked from the `hardware_test` job in
[`.github/workflows/firmware.yml`](../../.github/workflows/firmware.yml) on a
self-hosted runner with real ADSBee hardware attached. The workflow derives
`--expected-version` from `firmware/common/coprocessor/object_dictionary.cpp`
so the version check always matches the build under test.

**Currently disabled.** While the HIL fixture is down the job is gated on the
`HIL_ENABLED` repository variable and marked `continue-on-error`, so it is
skipped and cannot fail a CI build. Re-enable it by setting `HIL_ENABLED=true`
(Settings → Secrets and variables → Actions → Variables) and removing the
temporary lines in the `hardware_test` job.
