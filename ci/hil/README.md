# adsbee-hil: hardware-in-the-loop bench toolkit

`adsbee_hil` is a small Python package plus an `adsbee-hil` command for benches with many ADSBee
receivers of different models, and later with test transmitters. It does five things:

- **Find** devices by USB serial number. Every RP2040-based ADSBee enumerates as `2e8a:000a`
  (app) or `2e8a:0003` (BOOTSEL), so VID:PID and `/dev/ttyACMn` are ambiguous. The BOOTSEL drive
  is matched by USB port, never by taking "the first RPI-RP2 mount".
- **Lock** each device (`flock`, keyed by serial), so parallel CI jobs, several runners and
  humans can share one bench.
- **Talk** to each model's AT console, with that model's quirks handled.
- **Flash** each model its own way.
- **RF-test** receivers. A transmitter plays a known pattern and the receiver has to report it.

It needs Python ≥ 3.9 and `pyserial`. `pyadi-iio` and `numpy` are optional (Pluto only). The
device-selection and locking modules use only the standard library, so
`ci/test_usb_and_ota_flash` can import them without installing anything.

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

Example `discover` output from a bench with a 1090U and a 1421 jig:

```
1-1.3      2e8a:000a app     serial=E000000000000001 id=1090u   model=adsbee_1090u  'Raspberry Pi Pico'
           console /dev/serial/by-id/usb-Raspberry_Pi_Pico_E000000000000001-if00
1-1.4      2e8a:000a app     serial=E000000000000002 id=1421    model=adsbee_1421   'ADSBee ADSBee 1421 Programmer'
           console /dev/serial/by-id/usb-ADSBee_ADSBee_1421_Programmer_E000000000000002-if00
```

The 1090U uses the stock Pico USB strings, so without a bench file it shows as `model=?`. Pass
`--model adsbee_1090u` to use it ad hoc, or `discover --probe` to ask it for `AT+DEVICE_INFO?`.

## Bench file

The bench file is TOML ([`bench.example.toml`](bench.example.toml)). It is looked up in this
order: `--config`, then `$ADSBEE_HIL_CONFIG`, then `~/.config/adsbee-hil/bench.toml`. It contains:

- `[bench]`: `lock_dir` (shared by every user of the bench), and `busy_processes`, a list of
  regexes. While a process whose command line matches one of them is running, `flash` refuses
  to run unless given `--force`. For example, `bin/Runner.Worker` means "a GitHub Actions job
  is running".
- `[flashers.<model>]`: `command`, for external flashers that aren't part of this repository.
  See the 1421 notes below.
- `[[receivers]]`: `id`, `model`, `usb_serial`, and optionally `usb_port` (checked when set;
  used to find the BOOTSEL drive while the board is off the bus), `tags` and model-specific
  keys such as `host` (1090U OTA) or `flasher_command`.
- `[[transmitters]]`: `id`, `type`, `usb_serial` or type-specific connection keys, and
  `receivers` (which receivers it reaches; the default is all of them).

Library code never hardcodes serial numbers. They belong in the bench file.

## Commands

Selecting devices works the same way for every per-device command:

- `-d ID|SERIAL` picks one device and can be repeated.
- `-m MODEL` picks every receiver of that model.
- `-t TAG` picks every receiver with that tag.
- `-a` picks every receiver.

The selected devices are handled concurrently (`-j` sets how many). Output lines are prefixed
with `[id]`. The exit status is 1 if any device failed.

| Command | What it does |
|---|---|
| `discover [--json] [--probe]` | Lists RP2040-family USB devices: port path, mode (app/bootsel), serial, product, console, bench id, and configured devices that are absent. |
| `list` | Bench inventory with each device's state, and the known models and transmitter types. |
| `port` | Prints the console path (`/dev/serial/by-id/...`). |
| `at CMD [-T S]` | Sends one AT command. Streaming commands (`AT+RX_CW`) are stopped when `-T` expires. OTA keys are redacted unless `--show-secrets` is given. Commands that transmit (`AT+TX_CW`, `AT+REMOTE_ID_TX`) need `--allow-tx`. |
| `info [--json]` | Parsed `AT+DEVICE_INFO?`, without keys. |
| `rx-stats [--reset]` | Receive counters, for models that have them (1421: `AT+RX_STATS?`). |
| `capture -s SEC [--print]` | Switches the console to RAW reporting, counts the frames reported, then restores the previous protocol. The setting isn't saved. |
| `reboot` | `AT+REBOOT`, then waits for the console. |
| `flash IMAGE [--force] [--host H]` | Model-specific (see below). |
| `tx info\|play\|run\|stop TX` | Drives a transmitter (see below). |
| `loopback [--pattern P] [--min-fraction F] [--fail-on-skip]` | RF loopback test. It is skipped (exit 0) when no transmitter is available. |
| `locks` | Lock files and who holds them. |
| `lock DEVICE -- CMD...` | Runs any command while holding a device's lock, for scripts that don't use this package. |

## Models

### `adsbee_1090u`

The console is the RP2040's USB CDC port (the baud rate is ignored).

`flash combined.uf2` goes through these steps:

1. It sends `AT+BOOT_USB_UF2=1DEADBEE`.
2. It waits for the `2e8a:0003` device on the same USB port and mounts that port's RPI-RP2
   drive. It uses the desktop automounter if there is one and falls back to
   `udisksctl mount`.
3. It copies the image and waits for the app to come back, then prints `AT+DEVICE_INFO?`.

The RP2040 then updates the ESP32 and CC1312 from the images embedded in `combined.uf2`.
`flash file.ota --host NAME` uploads over WiFi with `ci/test_usb_and_ota_flash/ota_upload.py`,
which needs a repository checkout and `websockets`. The full CI flash → OTA → partition-flip test
is still `ci/test_usb_and_ota_flash/test_ota.py`, which now accepts `--serial`.

### `adsbee_1421`, behind the ADSBee 1421 Programmer jig

The m1421 module (CC1314R10 + LR2021) has no USB port of its own. The USB device, and the
`usb_serial` in the bench file, belong to the programmer jig (`firmware/adsbee_1421/programmer`),
a USB↔UART bridge whose modem-control lines drive the module:

- **RTS asserted → SYNC low (awake). RTS deasserted → SYNC high:** the application sleeps, and
  the ROM bootloader backdoor is armed at reset. **DTR assert edge → reset pulse.** The driver
  clears HUPCL so DTR and RTS stay asserted after it closes the port. A tool that closes the port
  with HUPCL set (plain pyserial, miniterm, screen) puts the module to sleep until the next open.
  That next open resets it, which is harmless.
- **The baud rate matters:** the jig copies the host's line coding onto the UART. The driver opens
  at 1 000 000 baud (the factory default) and falls back to the firmware's whitelist (921600,
  460800, 230400, 115200). It refuses `AT+BAUD_RATE=CONSOLE,...`, which would desync the bridge.
- **There is no bare `AT`:** the parser rejects it. The driver probes with `AT+UPTIME?`.
- **`AT+RX_CW` runs until it gets a key.** The driver stops it after `-T`, so the next command
  isn't swallowed. `AT+TX_CW` transmits and is refused unless `--allow-tx` is given.
- **Flashing module firmware (`.hex`):** the driver uses the CC13x4 ROM serial bootloader
  through the jig, but runs an **external flasher** configured in the bench file, because that
  flasher isn't part of this repository:

  ```toml
  [flashers.adsbee_1421]
  command = ["python3", "/path/to/flasher.py", "{image}", "-p", "{port}", "-b", "{baud}", "--erase", "sector"]
  ```

  You can also set it per receiver (`flasher_command`) or with
  `$ADSBEE_HIL_FLASHER_ADSBEE_1421`, a shell-quoted string. The placeholders are `{image}`,
  `{port}` (the jig's console), `{baud}` (1000000, so the jig stays transparent after the
  flash) and `{verbose}`. **The flasher must erase only the sectors the image covers.** A full
  bank erase also wipes the settings sector and the device-info / OTA-key sector. After the
  flasher exits, the driver reopens the console (which wakes and resets the module) and
  prints `AT+DEVICE_INFO?`.
- **Reflashes don't stick until the image is baked into the jig.** At every power-up, the jig
  CRC-checks the module against the image baked into its own firmware and reflashes the module
  if they differ. A `.hex` flashed as above therefore lasts until the jig next re-enumerates.
- **Jig images (`.uf2`) need a human.** The jig has no software reboot into BOOTSEL.
  `flash programmer.uf2` prints `HUMAN NEEDED` and waits (5 min by default) for someone to
  hold BOOT on the jig while replugging it. It then copies the image to the RPI-RP2 drive on
  that USB port, waits for the jig to reflash the module, and prints `AT+DEVICE_INFO?`.

### Adding a model

Subclass `adsbee_hil.Receiver`. Set `model`, the console behaviour (`console_baud`,
`keep_lines`, or override `console()`) and `version_key`, and implement `flash()` and, if the
model has them, `rx_counters()`. Built-in models are listed in `receivers.BUILTIN_MODELS`.
Out-of-tree drivers register through an entry point:

```toml
[project.entry-points."adsbee_hil.receivers"]
adsbee_future = "my_package.drivers:AdsbeeFuture"
```

## Scaling to many receivers

- **One bench file per host.** Each row is a receiver, keyed by USB serial. Give CI's boards a
  tag (`tags = ["ci"]`) and select them with `-t ci`. `-m adsbee_1421` selects every 1421.
- **Every operation takes the device's lock.** Jobs on different devices run in parallel, and
  jobs on the same device queue up (`--lock-timeout`, 600 s by default). flock locks are
  released by the kernel when the holder exits, so a crashed job can't leave a stale lock.
  The lock directory must be shared by every user and runner on the host.
- **Multiple CI runners on one host:** give each runner its own label and its own tag of boards,
  or let them share boards and rely on the locks.
- **USB layout:** pin `usb_port` for each board. The tool then reports a board that moved
  ports instead of quietly using it, and can find a board's BOOTSEL drive while its app serial
  is off the bus. Keep jigs and receivers on powered hubs. `discover` shows the port paths.
- **Timing:** a 1421 `.hex` flash takes about 15 s. Flashes of different boards run
  concurrently.

## Transmitters and RF loopback

A `TestPattern` sets the band (`1090`, `978` or `dual`), the number of packets (or −1 to
transmit until stopped), the rate, and optionally the output power, the step attenuation and
explicit hex messages. Named patterns are in `transmitters.PATTERNS`, and `--band`, `--count`,
`--rate`, `--power-dbm`, `--atten-db` and `--message` override them.

- **`bee_wiggler`** is PantsForBirds' Mode S / UAT packet generator. It is driven over its USB
  AT console (`usb_serial` in the bench file) with its documented commands: `AT+MODE_S_ATTEN`,
  `AT+SUBG_ATTEN`, `AT+MODE_S_TABLE_TX`, `AT+MODE_S_TX`, `AT+UAT_ADSB_TABLE_TX` and
  `AT+DUAL_TABLE_TX`. A finite pattern ends with `OK`, and any character stops an unbounded
  one. The wiggler's firmware isn't in this repository.
- **`pluto`** is an ADALM Pluto(+) SDR (`uri` in the bench file) that plays a generated 1090 MHz
  PPM waveform. The waveform comes from `packets.unique_df17_set()` by default: N distinct
  DF17 identification messages with consecutive ICAO addresses. **It hasn't been tested on
  hardware yet.** Output-level calibration, UAT, and exact packet counts (cyclic playback
  makes the count approximate) are still to do; see the TODO in `transmitters.py`.
- **Adding a transmitter:** subclass `Transmitter` and register it under
  `adsbee_hil.transmitters`.

```sh
adsbee-hil tx info wiggler-a
adsbee-hil tx play wiggler-a --pattern mode_s_table_100 --atten-db 60
adsbee-hil loopback -t ci --pattern df17_unique_10
```

`loopback` does the following for each selected receiver:

1. It picks the first transmitter that reaches it.
2. It switches the receiver's console to RAW and transmits the pattern.
3. It collects the reported frames and restores the console's reporting protocol.
4. It passes if at least `--min-fraction` (default 0.5) of the sent packets were reported.

With explicit messages, only frames identical to a sent message count, so background aircraft
can't make a test pass. With an instrument's built-in table, every frame on that band counts, so
use a cabled or shielded setup. With no transmitter, the result is `SKIP` (exit 0, or 1 with
`--fail-on-skip`).

Keep receivers on attenuated coax, or behind attenuators at a distance. A transmitter's full
output into a receiver's front end can damage it, and radiating on 1090 or 978 MHz outside a
shielded setup is not legal in most places.

## Tests

```sh
pip install -e 'ci/hil[test]'
pytest ci/hil                                       # no hardware needed
ADSBEE_HIL_CONFIG=~/bench.toml pytest ci/hil -m hil # against the bench (read-only + loopback)
```

The unit tests use a fake sysfs tree and fake AT devices on ptys. They cover discovery, config
parsing, locking across threads and processes, the CLI, and 1421 console handling (probe
command, refusal of transmit and baud-rate commands, redaction), plus the external-flasher
invocation, the wiggler command sequences, Mode S encoding and loopback scoring. The hardware
tests are skipped unless a bench file is given. They check each attached receiver's identity and
that its uptime advances with no reset between commands. They also run the loopback (skipped
while no transmitter is configured). Few aircraft fly at night, so nothing here treats zero
received packets as a failure without a transmitter.

## CI

The `hardware_test` job in `.github/workflows/firmware.yml` runs `test_ota.py --serial
$HIL_1090U_SERIAL` when the repository variable `HIL_1090U_SERIAL` is set. With the variable
unset, it keeps the old `-p /dev/ttyACM0` behaviour. `test_ota.py` locks the board for the run
through this package, so `adsbee-hil` users on the same host wait for CI instead of interrupting
it.
