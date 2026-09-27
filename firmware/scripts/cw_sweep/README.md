# CW Sweep Script

Steps an ADSBee 1421 / m1421 CW carrier across a frequency range over the AT console, holding each frequency so
a spectrum analyzer (e.g. in max-hold) can capture the fundamental and its harmonics. It's meant for mapping
notch, balun and filter response on the LR2021 LRHF (2.4 GHz) output, and also works for LRLF and SUBG.

## Safety: conducted measurements only

This tool keys up an unmodulated carrier of up to +22 dBm (LRLF) or +12 dBm (LRHF). The LRLF range covers
978 MHz (UAT), 960-1215 MHz (DME, 1030/1090 MHz SSR and ADS-B), and LRHF covers licensed spectrum outside the
2400-2483.5 MHz ISM band.

- Connect the RF output to the analyzer through a suitable attenuator, or terminate it in a dummy load.
- **Never connect an antenna.** A CW carrier on an aviation frequency from an ADS-B product is exactly the kind of
  interference the band exists to avoid.
- If the script is killed while the carrier is on, the device keeps transmitting until it gets a keystroke on the
  console or is power-cycled. See [Stopping the carrier](#stopping-the-carrier).

## Requirements

- An ADSBee 1421 / m1421 running a **Debug build** of firmware 0.3.11-rc6 or later. `AT+TX_CW` is compiled into
  Debug builds only; Release firmware (including CI builds and published releases) rejects it. Build and flash one
  with:

  ```sh
  cd firmware/adsbee_1421
  ./build.sh -d build_and_flash
  ```

  The script reads `CC1314R10 Firmware Build:` from `AT+DEVICE_INFO?` and stops with an error before transmitting
  anything if the device reports a Release build.
- Python 3.9+ and [pyserial](https://pypi.org/project/pyserial/) (or [Poetry](https://python-poetry.org/)).

## Setup

```sh
poetry install
```

## Usage

```sh
poetry run cw-sweep [--port DEV] [--baud N] [--band LRHF] [--start 1900] [--stop 2700]
                    [--step 0.1] [--power 0] [--dwell 1.0] [--manual] [--csv FILE] [-v]
```

With no `--port`, the script uses the only `/dev/cu.usbmodem*`, `/dev/ttyACM*` or `/dev/ttyUSB*` present. With no
`--baud`, it tries each console baud rate the firmware allows, 1 Mbaud first.

**Examples:**

```sh
# Full LRHF range, 100 kHz steps, 1 s per step, 0 dBm (default). About 3.3 hours.
poetry run cw-sweep --port /dev/cu.usbmodem21301

# Zoom in around the 2.4 GHz operating band at +10 dBm, 2 s per step, log to CSV.
poetry run cw-sweep --start 2380 --stop 2420 --step 0.1 --power 10 --dwell 2 --csv sweep.csv

# Manual mode: press Enter to advance to the next frequency (take a screenshot first).
poetry run cw-sweep --start 2400 --stop 2500 --step 10 --manual
```

## Options

| Option | Description |
|--------|-------------|
| `--band` | `LRHF` (LR2021 high band, 1900-2700 MHz, default), `LRLF` (LR2021 low band, 150-1100 MHz) or `SUBG` (CC1314, 718-1054 MHz, whole MHz only) |
| `--start`, `--stop` | Sweep range in MHz, inclusive. Defaults to the whole band. |
| `--step` | Step in MHz (default 0.1). The firmware rounds to 1 kHz. |
| `--power` | TX power in dBm for LRLF (-9 to 22) and LRHF (-19 to 12). Default 0. SUBG is fixed at +12 dBm. |
| `--dwell` | Seconds to hold each carrier (default 1.0). |
| `--manual` | Wait for Enter instead of `--dwell` before moving on. |
| `--csv` | Append one row per step (`utc_time, band, freq_mhz, power_dbm, on_s`). |
| `-v` | Echo the AT traffic. |

## Stopping the carrier

Ctrl-C stops the carrier and restores normal reception before the script exits. The firmware itself has no
on-time limit: `AT+TX_CW` transmits until any byte arrives on the console. If the script dies without stopping
it, send a newline from any serial terminal (or press Enter in the ADSBee 1421 console) or power-cycle the
device.
