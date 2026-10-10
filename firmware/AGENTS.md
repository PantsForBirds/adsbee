# ADSBee Firmware — Agent Guide

## Products

This repo hosts firmware for two products, sharing the code in `firmware/common/` and
`firmware/modules/`:

- **`adsbee_1090/`** — ADSBee 1090 (RP2040 + ESP32-S3 + CC1312). Documented in this file.
- **`adsbee_1421/`** — ADSBee m1421 (CC1314R10 + LR2021), plus the ADSBee 1421 Programmer (RP2040) that flashes it. See
  [`adsbee_1421/AGENTS.md`](adsbee_1421/AGENTS.md).

Build either through the dispatcher at `firmware/build.sh`:

```bash
bash firmware/build.sh adsbee_1090 [args...]   # forwards to adsbee_1090/build.sh
bash firmware/build.sh adsbee_1421 [args...]   # forwards to adsbee_1421/build.sh
```

Each product has its own firmware and settings versions, and the rules below apply per product. A
`firmware/common/` change needs a version bump in **both** products, except
`common/coprocessor/object_dictionary.cpp` (adsbee_1090's version constants). CI builds a product only when its own
files, `firmware/common/` or `firmware/modules/` changed.

## Comments and docs

- Use as few words as possible. Say what the code does and why; history, rejected alternatives, bench
  measurements and task ids go in the commit message or PR description.
- One-line comments by default. Write a short paragraph only for non-obvious hardware behavior or a rule
  callers must follow.
- Don't restate what the code or names already say.
- Spell out abbreviations on first use; avoid jargon a new contributor wouldn't know.
- READMEs are for users: short sections, bullets and the rules they need to operate the device. Design
  notes belong in code comments.
- Use American spelling. Avoid "it's not X, it's Y" phrasing.

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
- `espressif/idf:release-v5.4@sha256:…` — ESP32-S3 (ESP-IDF 5.4.4, pinned by digest in `compose.yml`;
  see "ESP-IDF and component versions" in `firmware/README.md`)
- `coolnamesalltaken/pico-docker:latest` — RP2040 (Pico SDK + host tests)
- `coolnamesalltaken/ti-lpf2:latest` — CC1312 (TI SimpleLink SDK)

**Build order matters**: ESP32 must build before RP2040 (RP2040 embeds the other binaries).

**Primary output**: `pico/build/Release/application/combined.uf2`, plus version-stamped copies
`adsbee_1090-<version>.uf2` and `adsbee_1090-<version>.ota` (the release assets)

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
1. **Any change to ESP32 or CC1312 code, or to shared `common/` code** → the firmware version must be **not yet released** (an RC for dev builds). Keep the branch's version if it is already the next unreleased RC; if it matches a release tag, move to the next unreleased RC. A `common/` change applies to adsbee_1421 too, except `common/coprocessor/object_dictionary.cpp`.
2. **Any change to the `Settings` struct** → increment `kSettingsVersion` and follow rule 1; commit both together.
3. If the firmware version is unchanged, RP2040 skips reflashing the coprocessors, so old behavior persists after flashing a new `combined.uf2`. To test under an unchanged version, force a reflash or bump the RC locally.
4. Never go below the latest release. Every RC sorts below its release: `0.9.1-rc9` < `0.9.1` < `0.9.2-rc1`.

Release tags are `<product>-M.m.p-rcN` or `<product>-M.m.p` (`kFirmwareVersionReleaseCandidate = 0`), e.g.
`adsbee_1090-0.9.1-rc3`, `adsbee_1421-0.3.10`. Pushing one runs `.github/workflows/release.yml`, which builds that
product and drafts a GitHub release (see [Releases](README.md#releases)).

### Automated enforcement
`scripts/check_version_sync.sh` checks these rules for both products. If a product's watched paths or
`kSettingsVersion` changed, it fails when the new version matches a release tag or is below the latest release. It
reads local tags, so run `git fetch --tags`; with no tags it only requires a version different from the base.
- Watched paths: adsbee_1090: `adsbee_1090/esp/`, `adsbee_1090/ti/`, `common/`. adsbee_1421: `adsbee_1421/` and
  `common/` except `common/coprocessor/object_dictionary.cpp`.
- `*.md` changes need no bump; any other file in a watched path does.
- The CI `version_sync_check` job runs the check and its tests (`scripts/test_check_version_sync.sh`).
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

### HIL testing with a signal source

Before driving an ADSBee from a Bee Wiggler, Pluto wiggler or any other SDR:

1. **Turn its feeds off first**, so synthetic aircraft never reach an aggregator.
   - Check: `AT+FEED_ENABLE?` must print `=0` (and `AT+FEED?` shows each slot).
   - Turn off: `AT+FEED_ENABLE=0` then `AT+SETTINGS=SAVE`, so a reboot or watchdog reset keeps them
     off. Firmware older than settings v15 has no master switch: set every slot inactive with
     `AT+FEED=<index>,,,0`.
   - Check again after every flash or settings reset; the defaults turn feeds back on.
   - An m1421 has no network of its own: stop whatever host process forwards its output.
   - Restore: record `AT+FEED_ENABLE?` and `AT+FEED?` before the test, put them back (plus
     `AT+SETTINGS=SAVE`) once the transmitter is off, and say so in your report.
2. **Use the reserved synthetic ICAO range `0xADF000`-`0xADF0FF`** for every frame you transmit.
3. **Never exceed 0 dBm into a receiver input.**

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
| ESP32 aborts at boot on one hardware revision only | An sdkconfig option that needs PSRAM or > 4 MB flash | One ESP32 image runs on the ESP32-S3-MINI-1U-N8 (8 MB flash, no PSRAM) and -N4R2 (4 MB flash, 2 MB PSRAM). Gate RAM-hungry features on `HardwareCapabilities::HasPSRAM()`; see `adsbee_1090/esp/README.md` |
