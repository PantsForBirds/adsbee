# ads-bee Firmware

This directory hosts firmware for two products, which share the code in `firmware/common/` and
`firmware/modules/`:

- **`adsbee_1090/`** — ADSBee 1090 (RP2040 + ESP32-S3 + CC1312)
- **`adsbee_1421/`** — ADSBee m1421 (CC1314R10 + LR2021) and the ADSBee 1421 Programmer (RP2040-Zero) that flashes it

Build either through the dispatcher:

```bash
bash firmware/build.sh adsbee_1090 [target]
bash firmware/build.sh adsbee_1421 [target]
```

## Prerequisites

### Docker

Install Docker via the official apt repository — **do not use the snap package**, as it has socket permission issues.

### Git Submodules

```bash
git submodule update --init --recursive
```

> **Windows note:** Cloning on Windows may produce files with CR+LF line endings, which break Docker builds. Run `git config --global core.autocrlf false` before cloning.

### Developer Setup

Bootstrap a fresh clone (submodules + git hooks) with:

```bash
bash firmware/scripts/setup_dev.sh
```

This initializes submodules and installs a native git pre-commit hook (no extra tooling) that enforces the [version-management rule](AGENTS.md) on every commit. It does not install the build toolchain (Docker/ESP-IDF/TI SDK).

---

## Build System Overview

The build system uses Docker Compose with three pre-built containers — no local image build is required. Images are pulled automatically on first run.

| Service | Image | Builds |
|---------|-------|--------|
| `pico-docker` | `coolnamesalltaken/pico-docker:latest` | RP2040 firmware, host tests, 1421 programmer |
| `esp-idf` | `espressif/idf:v5.5.2` | ESP32-S3 firmware |
| `ti-lpf2` | `coolnamesalltaken/ti-lpf2:latest` | TI CC1312 firmware, CC1314 (adsbee_1421) firmware |

Each product has its own compose file (`firmware/adsbee_1090/compose.yml`,
`firmware/adsbee_1421/compose.yml`). The build scripts handle the required build order
(for adsbee_1090: ESP32 → CC1312 → RP2040) automatically.

---

## Building ADSBee 1090 Firmware

Run from the repo root:

```bash
bash firmware/build.sh adsbee_1090 [options] [target]
# equivalent: bash firmware/adsbee_1090/build.sh [options] [target]
```

### Targets

| Target | Description |
|--------|-------------|
| `all` (default) | Build all three firmware targets in the correct order |
| `esp` | ESP32-S3 only |
| `ti` | TI CC1312 only |
| `pico` | RP2040 only (requires ESP32 and CC1312 builds to exist first) |
| `test [filter]` | Build and run host unit tests; `filter` is an optional ctest `-R` regex (e.g. `AircraftJSON`) |
| `build_and_flash [port]` | Build all three targets, then reflash an attached ADSBee 1090/1090U over USB |
| `flash [port]` | Reflash using the `combined.uf2` already on disk; runs no build steps, warns if it is stale |
| `clean` | Delete all build output directories |

### Options

| Flag | Description |
|------|-------------|
| `-d` | Build in Debug mode (default is Release) |

### Examples

```bash
bash firmware/adsbee_1090/build.sh            # full release build
bash firmware/adsbee_1090/build.sh -d         # full debug build
bash firmware/adsbee_1090/build.sh esp        # ESP32 only
bash firmware/adsbee_1090/build.sh test       # run all host tests
bash firmware/adsbee_1090/build.sh test AircraftJSON  # run filtered tests
```

### Build Output

| Target | Output |
|--------|--------|
| RP2040 (Release) | `firmware/adsbee_1090/pico/build/Release/application/combined.uf2` |
| RP2040 (Debug) | `firmware/adsbee_1090/pico/build/Debug/application/combined.uf2` |
| ESP32 (Release) | `firmware/adsbee_1090/esp/build/Release/adsbee_esp.bin` |
| CC1312 | `firmware/adsbee_1090/ti/sub_ghz_radio/build/sub_ghz_radio.bin` |

The `combined.uf2` embeds all three binaries. The build also writes version-stamped copies next to it,
`adsbee_1090-<version>.uf2` and `adsbee_1090-<version>.ota`, which CI publishes. See [Developers_Guide.md](adsbee_1090/Developers_Guide.md) for flashing instructions.

---

## Building ADSBee 1421 Firmware

Run from the repo root:

```bash
bash firmware/build.sh adsbee_1421 [options] [target]
# equivalent: bash firmware/adsbee_1421/build.sh [options] [target]
```

### Targets

| Target | Description |
|--------|-------------|
| `ti` (default) | CC1314R10 application |
| `programmer` | ADSBee 1421 Programmer, an RP2040-Zero flasher and pass-through (build `ti` first: it bakes in the hex) |
| `build_and_flash` | Build `ti` + `programmer`, then reflash an attached m1421 through the Programmer |
| `flash` | Reflash with the Programmer uf2 already on disk, without building; warns if it is stale |
| `clean [target]` | Delete the target's build directory |

`-d` selects a Debug build, as for adsbee_1090; `./build.sh -d build_and_flash` flashes the m1421 with the
Debug image.

### Debug builds and RF test commands

Commands that make a device transmit on demand for bench testing are compiled into **Debug builds only**. Release
builds (which is what CI and the published releases produce) don't contain them: `AT+HELP` doesn't list them and
sending one returns an unknown-command error.

| Product | Debug-only commands |
|---------|---------------------|
| ADSBee 1421 / m1421 (CC1314) | `AT+TX_CW`: unmodulated CW carrier on the CC1314 (SUBG) or LR2021 (LRLF/LRHF) until a key is pressed |
| ADSBee 1090 family (RP2040) | None. The RP2040 has no RF test transmit commands. |

`AT+REMOTE_ID_TX` (Remote ID broadcast) is an operational feature and is present in all builds. `AT+RX_CW` only
receives and is present in all builds.

`AT+DEVICE_INFO?` reports which kind of build is running, as `CC1314R10 Firmware Build: Debug|Release` on the
1421 and `RP2040 Firmware Build: Debug|Release` on the 1090. Older firmware omits the line. In CMake, Debug
builds define `ADSBEE_DEBUG_BUILD`.

Only use `AT+TX_CW` for conducted measurements into a spectrum analyzer through an attenuator, or into a dummy
load. Don't connect an antenna: the LR2021 bands cover licensed and aviation spectrum (including 978, 1030 and
1090 MHz). See [scripts/cw_sweep/README.md](scripts/cw_sweep/README.md).

### Build Output

| Target | Output |
|--------|--------|
| CC1314 (Release) | `firmware/adsbee_1421/ti/build/Release/adsbee_1421.hex` (+ `.elf`, `.map`, version-stamped copies) |
| Programmer | `firmware/adsbee_1421/programmer/build/Release/adsbee_1421_programmer.uf2` (+ `.elf`, version-stamped `-fw<version>` copies) |

See [adsbee_1421/AGENTS.md](adsbee_1421/AGENTS.md) for JTAG flashing, debugging and SYNC sleep, and
[adsbee_1421/README.md](adsbee_1421/README.md#reflashing-over-uart-the-sync-bootloader-backdoor) for
reflashing over UART without a debugger.

---

## Releases

Releases are drafted by CI from release tags (`.github/workflows/release.yml`):

1. Merge the version bump to `main`, then tag the merge commit and push the tag:
   ```bash
   git tag adsbee_1421-0.3.11-rc4 origin/main
   git push origin adsbee_1421-0.3.11-rc4
   ```
   Tags are `<product>-M.m.p-rcN` for release candidates and `<product>-M.m.p` for stable releases
   (see [AGENTS.md](AGENTS.md#rules)).
2. The workflow fails if the tag doesn't match the source's firmware version, then builds and tests that
   product and creates a **draft** release:

   | Product | Release name | Assets |
   |---|---|---|
   | adsbee_1090 | `ADSBee 1090 Firmware 0.9.1 Release Candidate 4` | `adsbee_1090-<version>.ota`, `adsbee_1090-<version>.uf2` |
   | adsbee_1421 | `ADSBee 1421 Firmware Version 0.3.11 Release Candidate 3` | `adsbee_1421-<version>.hex`, `adsbee_1421-<version>.elf`, `adsbee_1421_programmer-fw<version>.uf2` |

   Stable releases drop "Release Candidate N". The body has release-notes placeholders and a CI block
   (run link, commit, asset SHA-256s). The 1421 ELF keeps debug info for decoding crash addresses.
   ADSBee 1090 releases up to 0.9.1-rc6 name their assets `adsbee_1090.ota` and `combined.uf2`.
3. Write the release notes in the draft on GitHub and publish it.

Re-running the workflow (or re-pushing the tag) on a draft replaces its assets and CI block and keeps
your notes. A published release is never modified; the run fails.

---

## Interactive Shell

To open a shell inside a container for debugging or manual builds:

```bash
cd firmware/adsbee_1090
docker compose run --rm pico-docker bash   # RP2040 / test container
docker compose run --rm esp-idf bash       # ESP32 container
docker compose run --rm ti-lpf2 bash       # CC1312 container
```

---

## VS Code Integration

1. Install the **Dev Containers** VS Code extension.
2. Start the target container: `cd firmware/adsbee_1090 && docker compose run --rm pico-docker bash`
3. In VS Code, open the Remote Explorer panel, right-click the running container, and select **Attach to Container**.
4. Inside the attached VS Code, install the **Cortex-Debug** and **C/C++** extensions.
5. Use **Open Folder** to navigate to `/firmware/adsbee_1090` to pick up the `.vscode/launch.json` debug configuration.

---

## Removing Docker Images

```bash
docker image rm coolnamesalltaken/pico-docker:latest
docker image rm espressif/idf:v5.5.2
docker image rm coolnamesalltaken/ti-lpf2:latest
```
