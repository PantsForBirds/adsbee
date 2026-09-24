# ads-bee Firmware

This directory hosts firmware for two products, which share the code in `firmware/common/` and
`firmware/modules/`:

- **`adsbee_1090/`** — ADSBee 1090 (RP2040 + ESP32-S3 + CC1312)
- **`adsbee_1421/`** — ADSBee m1421 (CC1314R10 + LR2021) and its RP2040 flashing jig

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

The `combined.uf2` embeds all three binaries. See [Developers_Guide.md](adsbee_1090/Developers_Guide.md) for flashing instructions.

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
| `programmer` | RP2040-Zero flash/passthrough jig (requires `ti` built first — it bakes in the hex) |
| `build_and_flash` | Build `ti` + `programmer`, then reflash an attached m1421 through the jig |
| `flash` | Reflash using the jig uf2 already on disk; runs no build steps, warns if it or its baked-in hex is stale |
| `clean [target]` | Delete the target's build directory |

The `-d` flag selects a Debug build, as for adsbee_1090.

### Build Output

| Target | Output |
|--------|--------|
| CC1314 (Release) | `firmware/adsbee_1421/ti/build/Release/adsbee_1421.hex` (+ `.elf`, `.map`, version-stamped copies) |
| Programmer | `firmware/adsbee_1421/programmer/build/Release/adsbee_1421_programmer.uf2` (+ `.elf`, version-stamped `-fw<version>` copies) |

See [adsbee_1421/AGENTS.md](adsbee_1421/AGENTS.md) for flashing, debugging, and the SYNC
low-power sleep contract.

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
