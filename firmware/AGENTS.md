# ADSBee Firmware — Agent Guide

## Products

This repo hosts firmware for two products, sharing the code in `firmware/common/` and
`firmware/modules/`:

- **`adsbee_1090/`** — ADSBee 1090 (RP2040 + ESP32-S3 + CC1312). Documented in this file.
- **`adsbee_1421/`** — ADSBee m1421 (CC1314R10 + LR2021), plus its RP2040 flashing jig. See
  [`adsbee_1421/AGENTS.md`](adsbee_1421/AGENTS.md).

Build either through the dispatcher at `firmware/build.sh`:

```bash
bash firmware/build.sh adsbee_1090 [args...]   # forwards to adsbee_1090/build.sh
bash firmware/build.sh adsbee_1421 [args...]   # forwards to adsbee_1421/build.sh
```

Each product has its own firmware/settings versions; the version-management rules below apply
**per product**, and a `firmware/common/` change requires a version bump in **both** products
(enforced by `scripts/check_version_sync.sh`). CI builds a product only when its own files,
`firmware/common/`, or `firmware/modules/` changed.

## Project Summary

ADSBee 1090 is an ADS-B/UAT aviation transponder receiver with a 3-processor heterogeneous firmware:

- **RP2040** — main processor (RF decoding of 1090 MHz Mode-S, system orchestration, dual-core ARM Cortex-M0+)
- **ESP32-S3** — WiFi/network processor (TCP/IP stack, HTTP/WebSocket server, output feeds)
- **CC1312** — Sub-GHz coprocessor (978 MHz UAT reception, TI SimpleLink)

The ESP32 and CC1312 binaries are embedded inside the RP2040 UF2. Flashing one file (`combined.uf2`) updates all three processors.

---

## Build System

**Working directory**: `firmware/adsbee_1090/`

```bash
bash build.sh [-d] [target]
```

| Flag/Target | Meaning |
|-------------|---------|
| (none) | Release build of all targets |
| `-d` | Debug build |
| `all` | ESP32 + CC1312 + RP2040 (default) |
| `esp` | ESP32-S3 only |
| `ti` | CC1312 only |
| `pico` | RP2040 only (requires ESP32 + CC1312 built first) |
| `test` | Host unit tests (no hardware needed) |
| `build_and_flash` | Build all targets, then reflash an attached device over USB |
| `flash` | Reflash using the already-built `combined.uf2`; runs no build steps |
| `clean` | Remove all build directories |

**Requires Docker.** Three images are used:
- `espressif/idf:v5.5.2` — ESP32-S3 (ESP-IDF)
- `coolnamesalltaken/pico-docker:latest` — RP2040 (Pico SDK + host tests)
- `coolnamesalltaken/ti-lpf2:latest` — CC1312 (TI SimpleLink SDK)

**Build order matters**: ESP32 must build before RP2040 (RP2040 embeds the other binaries).

**Primary output**: `pico/build/Release/application/combined.uf2`

---

## Directory Structure

```
firmware/
├── adsbee_1090/
│   ├── build.sh                   # Master build script
│   ├── compose.yml                # Docker Compose for all build images
│   ├── Developers_Guide.md        # Human-readable dev docs (pitfalls, troubleshooting)
│   ├── esp/                       # ESP32-S3 — ESP-IDF project
│   │   ├── main/
│   │   │   ├── app_main.cpp       # Entry point: initializes all subsystems
│   │   │   ├── server/            # ADSBeeServer, WebSocketServer
│   │   │   ├── comms/             # Protocol output handlers (ESP32 side)
│   │   │   └── peripherals/       # GPIO, SPI, UART drivers
│   │   └── sdkconfig              # ESP-IDF config (target: esp32s3)
│   ├── pico/                      # RP2040 — CMake + Pico SDK
│   │   ├── application/
│   │   │   ├── main.cc            # Entry point
│   │   │   ├── adsbee.cc/hh       # Core RF decoding logic (~47 KB)
│   │   │   ├── peripherals/       # RF frontend, EEPROM, SPI to ESP32, UART to CC1312
│   │   │   └── comms/             # AT command interface
│   │   ├── bootloader/            # UART reflash bootloader
│   │   └── host_test/             # GoogleTest unit tests (run on host)
│   └── ti/sub_ghz_radio/          # CC1312 — TI SimpleLink SDK
│       └── main.cpp               # Entry point
├── common/                        # Shared code (RP2040 + ESP32 both include this)
│   ├── adsb/                      # Mode-S/UAT packet decoding, aircraft dictionary
│   │   └── nasa_cpr/              # Compact Position Reporting algorithm
│   ├── coprocessor/               # SPI inter-processor protocol
│   │   ├── object_dictionary.hh/cpp   # Firmware version, command addresses
│   │   ├── composite_array.hh/cpp     # Packet batching for SPI transport
│   │   └── spi_coprocessor.hh/cpp     # SPI master/slave implementation
│   ├── comms/                     # Output protocol implementations
│   │   ├── gdl90/                 # GDL 90 (aviation standard)
│   │   ├── beast/                 # BEAST protocol
│   │   ├── mavlink/               # MAVLink
│   │   └── json/                  # JSON
│   ├── settings/
│   │   └── settings.hh/cpp        # Shared Settings struct + kSettingsVersion
│   ├── utils/                     # PFBQueue (thread-safe circular buffer), CRC, FEC, geo, HAL
│   └── firmware_update/           # OTA update logic
└── modules/                       # Git submodules: googletest, cppAT
```

---

## Critical: Version Management

Two version values control whether RP2040 reflashes the coprocessors on boot. (adsbee_1421 has
its own independent pair under `adsbee_1421/ti/` — see
[`adsbee_1421/AGENTS.md`](adsbee_1421/AGENTS.md); the rules below are the adsbee_1090 side.)

### Firmware version — `common/coprocessor/object_dictionary.cpp`
```cpp
const uint8_t ObjectDictionary::kFirmwareVersionMajor = 0;
const uint8_t ObjectDictionary::kFirmwareVersionMinor = 9;
const uint8_t ObjectDictionary::kFirmwareVersionPatch = 0;
const uint8_t ObjectDictionary::kFirmwareVersionReleaseCandidate = 19;  // 0 = release
```

### Settings version — `common/settings/settings.hh`
```cpp
static constexpr uint32_t kSettingsVersion = N;
```

### Rules
1. **Any change to ESP32 or CC1312 code, or to shared `common/` code** → increment firmware version (RC for dev builds, patch for releases). A `common/` change also requires an adsbee_1421 version bump.
2. **Any change to the `Settings` struct** → increment `kSettingsVersion` AND firmware version; commit both together
3. If firmware version is unchanged, RP2040 skips reflashing the coprocessors — symptom: old behavior persists after flashing new `combined.uf2`

### Automated enforcement
These rules are checked automatically by `scripts/check_version_sync.sh` (covers both products):
- **`build.sh`** (both products') runs the check locally before every build, but only **warns** —
  a failed check never blocks a local build.
- **Local git hook** — the enforcing gate; catches it before you even commit. A native `pre-commit` hook (no external tooling) is installed by the dev setup script:
  ```
  bash firmware/scripts/setup_dev.sh
  ```
  (bypass a single commit with `git commit --no-verify`)

---

## Inter-Processor Communication

| Link | Protocol | Key files |
|------|----------|-----------|
| RP2040 ↔ ESP32 | SPI (custom binary) | `common/coprocessor/spi_coprocessor.*`, `composite_array.hh` |
| RP2040 ↔ CC1312 | SPI | `pico/application/peripherals/` |

On boot, RP2040 reads the firmware version from each coprocessor and reflashes if it differs.

---

## Testing

Run from `firmware/adsbee_1090/`:

```bash
bash build.sh test                   # Build and run the full host test suite (no hardware needed)
bash build.sh test AircraftJSON      # Build and run only tests whose ctest name matches "AircraftJSON"
bash build.sh test CSBee             # Run only CSBee tests
```

The optional second argument is a regex passed to `ctest -R`. Test names follow the pattern
`host_test.<SuiteName>.<TestName>` (e.g. `host_test.AircraftJSON.ModeSAircraftAllFields`).

- **Host unit tests** (no hardware): `bash build.sh test` — runs GoogleTest suite via Docker
- **Hardware integration tests**: `esp/main/target_test/` and `pico/application/target_test/` — must run on device
- CI builds the firmware artifact; on-device testing is manual

---

## Flashing

For an ADSBee 1090/1090U attached over USB, `cd firmware/adsbee_1090 && ./build.sh build_and_flash`
does all of this automatically: it builds every target, finds the device, reboots it into the
bootloader with `AT+BOOT_USB_UF2` (no button press), copies the `.uf2`, and then verifies the RP2040
and ESP32 firmware versions. Pass a CDC node (`./build.sh build_and_flash /dev/cu.usbmodem21201`) to
choose between multiple attached devices.

`./build.sh flash` is the same minus every build step: it pushes the `combined.uf2` already on disk,
which is what you want when re-flashing after a failed copy or flashing several boards from one
build. It warns if that image is older than the source tree, because the post-flash version check
reads the expected version from `object_dictionary.cpp` source and a stale image will fail it.

By hand, or to recover a device that will not enumerate:

1. Hold **BOOTSEL** on RP2040 while connecting USB → RP2040 mounts as a USB drive
2. Copy `combined.uf2` to the drive → device reboots automatically
3. On boot, RP2040 checks coprocessor firmware versions and reflashes ESP32/CC1312 if they differ
4. To force ESP32 reflash: increment firmware version in `object_dictionary.cpp`, rebuild, reflash

---

## Common Pitfalls

| Symptom | Cause | Fix |
|---------|-------|-----|
| `Settings write requested with len X, sending Y instead` in logs | Firmware version skew between processors | Increment firmware version, rebuild, reflash |
| ESP32 keeps old behavior after flashing | Firmware version unchanged | Increment firmware version |
| RP2040 build fails (missing binary) | ESP32 or CC1312 not built yet | Run `bash build.sh all` or build in order |
| Settings reset on every boot | `kSettingsVersion` mismatch | Ensure both processors run the same firmware |
