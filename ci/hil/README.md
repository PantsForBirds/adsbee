# adsbee-hil: hardware-in-the-loop bench toolkit

`adsbee_hil` is a Python package and `adsbee-hil` command for benches with many ADSBee receivers
and test transmitters. It can:

- **Find** devices by USB serial number. All RP2040-based ADSBees share VID:PID `2e8a:000a` (app)
  / `2e8a:0003` (BOOTSEL), so `/dev/ttyACMn` is ambiguous. BOOTSEL drives are matched by USB port.
- **Lock** each device (`flock`, keyed by serial) so CI jobs, runners and people can share a bench.
- **Talk** to each model's AT console.
- **Flash** each model.
- **RF-test** receivers: a transmitter plays a known pattern and the receiver must report it.

Needs Python ≥ 3.9 and `pyserial`; `pyadi-iio` and `numpy` only for `pluto`. Device selection and
locking use only the standard library, so `ci/test_usb_and_ota_flash` imports them without installing.

## Install

```sh
python3 -m venv ~/hil-venv            # --system-site-packages to reuse a system pyserial
~/hil-venv/bin/pip install -e ci/hil  # extras: [pluto], [ota], [test]
export PATH=~/hil-venv/bin:$PATH
```

## Quick start

```sh
adsbee-hil discover                    # what's attached; works without a bench file
cp ci/hil/bench.example.toml ~/.config/adsbee-hil/bench.toml   # fill in the serials
adsbee-hil list                        # the inventory and each device's state
adsbee-hil info --all                  # identity and firmware versions, all boards in parallel
adsbee-hil at -d 1421-a "AT+RX_STATS?"
adsbee-hil flash -m adsbee_1421 adsbee_1421-0.3.11-rc1.hex   # every 1421 on the bench, in parallel
```

Example `discover` output from a bench with a 1090U and an ADSBee 1421 Programmer:

```
1-1.3      2e8a:000a app     serial=E000000000000001 id=1090u   model=adsbee_1090u  'Pants for Birds ADSBee 1090'
           console /dev/serial/by-id/usb-Pants_for_Birds_ADSBee_1090_E000000000000001-if00
1-1.4      2e8a:000a app     serial=E000000000000002 id=1421    model=adsbee_1421   'Pants for Birds ADSBee 1421 Programmer'
           console /dev/serial/by-id/usb-Pants_for_Birds_ADSBee_1421_Programmer_E000000000000002-if00
```

All RP2040 ADSBee 1090 variants (1090, 1090U, m1090, GS3M, Winglet) report the same USB strings, so
without a bench file a 1090U shows as `model=?`. Pass `--model adsbee_1090u`, or `discover --probe`
to query `AT+DEVICE_INFO?`.

## Bench file

TOML ([`bench.example.toml`](bench.example.toml)), looked up in order: `--config`,
`$ADSBEE_HIL_CONFIG`, `~/.config/adsbee-hil/bench.toml`.

- `[bench]`: `lock_dir` (shared by all bench users) and `busy_processes`, regexes of command lines
  (e.g. `bin/Runner.Worker` for a running GitHub Actions job). `flash` refuses to run while one
  matches, unless given `--force`.
- `[flashers.<model>]`: `command` for external flashers (see the 1421 notes).
- `[[receivers]]`: `id`, `model`, `usb_serial`; optional `usb_port` (checked when set; finds the
  BOOTSEL drive), `tags`, and model keys such as `host` (1090U OTA) or `flasher_command`.
- `[[transmitters]]`: `id`, `type`, `usb_serial` or type-specific keys, and `receivers` it reaches
  (default: all).

Serial numbers go in the bench file, never in code.

## Commands

Device selection, for every per-device command:

- `-d ID|SERIAL`: one device (repeatable).
- `-m MODEL`: every receiver of that model.
- `-t TAG`: every receiver with that tag.
- `-a`: every receiver.

Selected devices run concurrently (`-j` sets how many), output lines are prefixed with `[id]`, and
the exit status is 1 if any device failed.

| Command | What it does |
|---|---|
| `discover [--json] [--probe]` | Lists RP2040-family USB devices (port, mode, serial, product, console, bench id) and configured devices that are absent. |
| `list` | Bench inventory with device states, known models and transmitter types. |
| `port` | Prints the console path (`/dev/serial/by-id/...`). |
| `at CMD [-T S]` | Sends one AT command. Streaming commands (`AT+RX_CW`) stop after `-T`. OTA keys are redacted unless `--show-secrets`. Transmit commands (`AT+TX_CW`, `AT+REMOTE_ID_TX`) need `--allow-tx`. |
| `info [--json]` | Parsed `AT+DEVICE_INFO?`, without keys. |
| `rx-stats [--reset]` | Receive counters where supported (1421: `AT+RX_STATS?`). |
| `capture -s SEC [--print]` | Counts frames in RAW reporting mode, then restores the previous protocol (not saved). |
| `reboot` | `AT+REBOOT`, then waits for the console. |
| `flash IMAGE [--force] [--host H]` | Model-specific (see below). |
| `tx info\|play\|run\|stop TX` | Drives a transmitter (see below). |
| `loopback [--pattern P] [--min-fraction F] [--fail-on-skip]` | RF loopback test; skipped (exit 0) with no transmitter. |
| `remote-id CASE -d A -d B` | Remote ID release test on two ADSBee 1090s (see [below](#remote-id-release-test)). |
| `locks` | Lock files and their holders. |
| `lock DEVICE -- CMD...` | Runs a command while holding a device's lock. |

## Models

### `adsbee_1090u`

Console: the RP2040's USB serial port (baud rate ignored).

`flash adsbee_1090-<version>.uf2` (or a local build's `combined.uf2`):

1. Sends `AT+BOOT_USB_UF2=1DEADBEE`.
2. Waits for `2e8a:0003` on the same USB port and mounts its RPI-RP2 drive (desktop automounter,
   else `udisksctl mount`).
3. Copies the image, waits for the app, prints `AT+DEVICE_INFO?`.

The RP2040 then updates the ESP32 and CC1312 from the image. `flash file.ota --host NAME`
uploads over WiFi with `ci/test_usb_and_ota_flash/ota_upload.py` (needs `websockets`). The full CI
flash → OTA → partition-flip test is `ci/test_usb_and_ota_flash/test_ota.py --serial`.

### `adsbee_1421`, behind the ADSBee 1421 Programmer

The m1421 module (CC1314R10 + LR2021) has no USB port. The USB device and `usb_serial` belong to the
ADSBee 1421 Programmer (`firmware/adsbee_1421/programmer`), a USB-to-UART bridge whose control lines
drive the module:

- **RTS asserted: module awake. RTS deasserted: module sleeps, and the ROM bootloader is armed at
  reset. DTR assert: reset pulse.** The driver keeps DTR and RTS asserted after closing the port
  (clears HUPCL). Tools that don't (plain pyserial, miniterm, screen) leave the module asleep until
  the next open, which resets it.
- **Baud rate is virtual:** the Programmer follows the console's rate. The driver opens at
  1 000 000 baud, falling back to 921600, 460800, 230400 and 115200 for Programmer 0.3.11-rc3 and
  earlier. It refuses `AT+BAUD_RATE=CONSOLE,...`.
- **No bare `AT`:** the driver probes with `AT+UPTIME?`.
- **`AT+RX_CW` runs until it gets a key;** the driver stops it after `-T`. `AT+TX_CW` needs
  `--allow-tx`.
- **Module firmware (`.hex`)** is flashed by an external flasher (not in this repository) over the
  CC13x4 ROM serial bootloader:

  ```toml
  [flashers.adsbee_1421]
  command = ["python3", "/path/to/flasher.py", "{image}", "-p", "{port}", "-b", "{baud}", "--erase", "sector"]
  ```

  Or per receiver (`flasher_command`), or `$ADSBEE_HIL_FLASHER_ADSBEE_1421` (shell-quoted).
  Placeholders: `{image}`, `{port}` (Programmer console), `{baud}` (1000000) and `{verbose}`.
  **The flasher must erase only the sectors the image covers;** a full erase wipes settings and
  the device-info / OTA-key sector. Afterwards the driver reopens the console (resetting the
  module) and prints `AT+DEVICE_INFO?`.
- **Reflashes last only until the Programmer re-enumerates.** At power-up the Programmer reflashes
  the module if it differs from the image baked into the Programmer's firmware.
- **Programmer images (`.uf2`):** `flash programmer.uf2` opens the port at 233495534 baud
  (`0xDEADBEE`) to reboot into BOOTSEL, copies the image to that port's RPI-RP2 drive, waits for
  the module reflash and prints `AT+DEVICE_INFO?`. Older Programmer images ignore the magic baud:
  the command prints `HUMAN NEEDED` and waits (5 min default) for someone to hold BOOT while
  replugging.

### Adding a model

Subclass `adsbee_hil.Receiver`: set `model`, console behavior (`console_baud`, `keep_lines`, or
override `console()`) and `version_key`; implement `flash()` and, if supported, `rx_counters()`.
Built-in models are in `receivers.BUILTIN_MODELS`. Out-of-tree drivers register an entry point:

```toml
[project.entry-points."adsbee_hil.receivers"]
adsbee_future = "my_package.drivers:AdsbeeFuture"
```

## Scaling to many receivers

- **One bench file per host,** one row per receiver. Tag CI boards (`tags = ["ci"]`) and select
  with `-t ci`.
- **Every operation takes the device's lock.** Different devices run in parallel; the same device
  queues (`--lock-timeout`, default 600 s). The kernel releases locks when the holder exits. The
  lock directory must be shared by every user and runner on the host.
- **Several CI runners on one host:** give each its own label and tag of boards, or share boards
  and rely on the locks.
- **USB layout:** set `usb_port` for each board so moved boards are reported and BOOTSEL drives
  can be found. Keep ADSBee 1421 Programmers and receivers on powered hubs.
- **Timing:** a 1421 `.hex` flash takes about 15 s; different boards flash concurrently.

## Transmitters and RF loopback

A `TestPattern` sets the band (`1090`, `978` or `dual`), packet count (−1 = until stopped), rate,
and optionally output power, attenuation and explicit hex messages. Named patterns are in
`transmitters.PATTERNS`; `--band`, `--count`, `--rate`, `--power-dbm`, `--atten-db` and `--message`
override them.

- **`bee_wiggler`**: the wiggler family over its USB AT console (`usb_serial`): Bee Wiggler (RP2040),
  Pluto Wiggler (ADALM-Pluto) and Bee Waggler (ZynqSDR 7020). The driver picks the command set from
  `AT+DEVICE_INFO?`:

  | Firmware | Commands | End of a transmission |
  |---|---|---|
  | Bee Wiggler; Pluto Wiggler / Bee Waggler 1.x | `AT+MODE_S_ATTEN`, `AT+SUBG_ATTEN`, `AT+MODE_S_TABLE_TX`, `AT+MODE_S_TX`, `AT+UAT_ADSB_TABLE_TX`, `AT+DUAL_TABLE_TX` | `OK`; any character stops an unbounded one |
  | Pluto Wiggler / Bee Waggler 2.x | `AT+TX=<port>,MODE_S\|UAT,…` jobs with the level in dBm, `AT+HOLD` (dual); several explicit messages: `AT+SCHED` entries (finite, ≤ 256 packets) or an `AT+FILE` packet file with a period | `+DONE:<id>,sent=,late=,dropped=,reason=` per job; `AT+STOP` ends unbounded ones |

  On 2.x, `atten_db` keeps its meaning: the level is `power_dbm` (default +5 dBm, the 1.x reference) minus
  the Bee-quantized attenuation, so a bench file gives the same level on every firmware. On a Bee Waggler
  every job goes out on both ports (`AT+TX=BOTH`, each at its own calibration), as 1.x mirrored TX1 to TX2,
  so the cabling stays as it was; a Pluto Wiggler has port 1. `mode_s_port` and `uat_port` (1, 2 or BOTH)
  change that. `dual` is time-shared unless the rate is 0 or `simultaneous = true`; then a Bee Waggler checks
  `AT+SIMULTANEOUS?` (pair available, levels under the plan's maximum) and opts in to the fixed-LO plan for
  that run (about 25 dB less maximum level, Mode S on port 1, UAT on port 2). An ADSBee 1090U fed more than
  about -45 dBm in that plan misses frames unless its `AT+TL_OFFSET` is raised (1200 mV worked, DESIGN-2
  §5.3) or you attenuate more; the driver warns. Explicit messages may be Mode S (14/28 hex digits) or, on
  2.x, UAT payloads (`packets.unique_uat_set()`); `dual` with messages needs both kinds. A looped `sched`
  or a packet file without a period wraps back to back and loses a packet per loop, so the driver plays
  `sched` once and gives files a period. Each job's sent/late/dropped counts end up in the loopback line.
  `adsbee-hil tx stop` sends `AT+STOP=ALL` and `AT+SIMULTANEOUS=OFF` to a 2.x unit (it ignores a bare
  character).
- **`pluto`**: ADALM-Pluto (or compatible) SDR at `uri`, not a Pluto Wiggler, playing a 1090 MHz
  waveform of distinct DF17 messages (`packets.unique_df17_set()`). **Not yet tested on hardware;**
  see the TODO in `transmitters.py`.
- **New transmitter:** subclass `Transmitter` and register it under `adsbee_hil.transmitters`.

```sh
adsbee-hil tx info wiggler-a
adsbee-hil tx play wiggler-a --pattern mode_s_table_100 --atten-db 60
adsbee-hil loopback -t ci --pattern df17_unique_10
```

`loopback`, per receiver:

1. Picks the first transmitter that reaches it.
2. Switches the console to RAW and transmits the pattern.
3. Collects reported frames and restores the reporting protocol.
4. Passes if at least `--min-fraction` (default 0.5) of sent packets were reported.

With explicit messages only exact matches count, so nearby aircraft can't pass a test. With an
instrument's built-in table every frame on the band counts, so use a cabled or shielded setup. With
no transmitter the result is `SKIP` (exit 0, or 1 with `--fail-on-skip`).

**Before transmitting into an ADSBee:**

- Turn its feeds off so synthetic aircraft never reach an aggregator: `AT+FEED_ENABLE=0` then
  `AT+SETTINGS=SAVE` (older firmware: `AT+FEED=<index>,,,0` for every slot). Check with
  `AT+FEED_ENABLE?` (`=0`) and `AT+FEED?` before the first transmission and after every flash or
  settings reset. For an m1421, stop the host process that forwards its output.
- Restore the recorded `AT+FEED_ENABLE` and `AT+FEED` values (and save) when the test is over.
- Transmit only ICAO addresses `0xBEE000`-`0xBEE0FF`. ICAO Annex 10 Vol III (2nd ed., through
  Amendment 93), Appendix to Chapter 9, Table 9-1 allocates this block to no state, so a stray frame
  won't land on a real aircraft's track.
- Before an RF run, confirm no aircraft from that block is on a public aggregator; after the run, a
  hit means investigate:

  ```sh
  curl -s "https://api.adsb.lol/v2/hex/$(python3 -c "print(','.join(f'{0xBEE000 + i:06x}' for i in range(256)))")" \
      | grep -o '"total": *[0-9]*'  # expect "total": 0
  ```
- Never exceed 0 dBm into a receiver input.

**Keep receivers on attenuated coax or behind attenuators.** Full transmitter output can damage a
receiver's front end, and radiating on 1090 or 978 MHz outside a shielded setup is illegal in most
places.

## Remote ID release test

`adsbee-hil remote-id` runs one case of the ADSBee 1090 Remote ID release test
([`firmware/RELEASE_TESTING.md`](../../firmware/RELEASE_TESTING.md)) on two units in Bluetooth range
of each other. The units hear each other, so the host needs no Bluetooth adapter.

```sh
adsbee-hil remote-id tx-both -d 1090u-a -d 1090u-b --minutes 15 --json rid.jsonl
adsbee-hil remote-id rxtx -d user@host-a:1090u-a -d user@host-b:1090u-b \
    --remote-hil '~/hil-venv/bin/adsbee-hil'          # units on other hosts, over ssh
adsbee-hil remote-id guards -d 1090u-a -d 1090u-b
```

| Case | Unit A | Unit B |
|---|---|---|
| `rxtx` | RX + TX BT4+BT5_LR | RX + TX BT4+BT5_LR |
| `tx-bt4` / `tx-bt5lr` / `tx-both` | TX only: BT4 / BT5_LR / both | RX only |
| `rx-only` | RX + TX | RX only |
| `off` | off | off (decode and heap baseline) |
| `guards` | each `-d` unit: refuses the transports its RAM can't run next to WiFi or Ethernet (read-only) | |

Each unit's side holds its lock for the whole case, records what it finds, sets `AT+LOG_LEVEL=INFO`,
`AT+REMOTE_ID=` and `AT+REMOTE_ID_TX=` (bench UAS IDs `ADSBEEBENCHA`/`B`, ID type serial, no
position) **without saving**, and puts everything back at the end (checked with `AT+CONFIG_HASH?`).
It refuses to transmit from a unit whose `AT+RX_POSITION` source is FIXED or GNSS, and stops if the
status lacks `TX_NO_POSITION`. Units without PSRAM can't run Remote ID next to the WiFi AP:
`--wifi-ap-off` turns the AP off for the case (not saved).

Every minute it prints, per unit:

- **Remote ID frames/s** from the peer. With `host` in the bench file (or `--host-a/--host-b`)
  from `/metrics` (`pip install -e 'ci/hil[ota]'` for `websockets`). Over USB only, from the peer's
  `AIRCRAFT_JSON` contact: its RSSI changes when frames arrive, and the contact stays 60 s after the
  last one, so prefer a networked receiver.
- **ESP32 heap** (min/median) from the ESP32's 1 s heartbeat line (or `/metrics`).
- **Watchdog resets** ("Bluetooth scan stalled", "Bluetooth advertising stalled"), Bluetooth host
  resets, Remote ID errors and heap-watch pauses, from the console.
- **ESP32 reboots** (`/metrics` uptime, or heartbeat gaps of 10 s or more) and Mode S/UAT frames/s.

The verdict passes when every receiving unit heard its peer in every minute, neither unit's ESP32
or RP2040 rebooted, the status words appeared without a save, and the settings were restored. Exit
status 0 = pass. `--json` appends every record for later comparison (e.g. heap vs the previous
release).

## Tests

```sh
pip install -e 'ci/hil[test]'
pytest ci/hil                                       # no hardware needed
ADSBEE_HIL_CONFIG=~/bench.toml pytest ci/hil -m hil # against the bench (read-only + loopback)
```

Unit tests use a fake sysfs tree and fake AT devices on pseudo-terminals. Hardware tests are skipped
without a bench file; they check each receiver's identity, that uptime advances without resets, and
run the loopback. Zero received packets without a transmitter is never a failure.

## CI

`hardware_test` in `.github/workflows/firmware.yml` runs `test_ota.py --serial $HIL_1090U_SERIAL`
when the repository variable `HIL_1090U_SERIAL` is set (else `-p /dev/ttyACM0`). It holds the
board's lock, so `adsbee-hil` users on the same host wait for CI.
