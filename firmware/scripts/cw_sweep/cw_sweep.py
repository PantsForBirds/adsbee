#!/usr/bin/env python3
"""
cw_sweep.py — Step an ADSBee 1421 CW carrier across a frequency range.

Drives AT+TX_CW over the console UART, dwelling at each frequency so a spectrum analyzer
(e.g. in max-hold) can capture the fundamental and its harmonics. Intended for mapping
notch / balun / filter response on the LR2021 LRHF (2.4 GHz) output, but works for LRLF
and SUBG too.

Requires adsbee_1421 firmware >= 0.3.11rc2 (AT+TX_CW accepts fractional MHz).

Usage:
  python cw_sweep.py [--port DEV] [--baud N] [--band LRHF] [--start 1900] [--stop 2700]
                     [--step 0.1] [--power 0] [--dwell 1.0] [--manual] [--csv FILE]

Examples:
  # Full LRHF range, 100 kHz steps, 1 s per step, 0 dBm (default).
  python cw_sweep.py --port /dev/cu.usbmodem21301

  # Zoom in around the 2.4 GHz operating band at +10 dBm, 2 s per step, log to CSV.
  python cw_sweep.py --start 2380 --stop 2420 --step 0.1 --power 10 --dwell 2 --csv sweep.csv

  # Manual mode: press Enter to advance to the next frequency (take a screenshot first).
  python cw_sweep.py --start 2400 --stop 2500 --step 10 --manual

Ctrl-C at any point stops the carrier and restores normal reception before exiting.
"""

from __future__ import annotations

import argparse
import csv
import glob
import sys
import time
from datetime import datetime

try:
    import serial
except ImportError:  # pragma: no cover
    sys.exit("pyserial is required: pip install pyserial   (or: poetry install && poetry run cw-sweep)")

# Console baud whitelist from firmware/adsbee_1421/ti/comms/comms.hh (kAllowedBaudRates). Factory default is 1 M.
ALLOWED_BAUDS = (1000000, 115200, 230400, 460800, 921600)

# Per-band bounds (MHz) mirroring comms_at.cpp. LR2021 bands accept fractional MHz; SUBG is whole MHz only.
BANDS = {
    "SUBG": (718, 1054),
    "LRLF": (150, 1100),
    "LRHF": (1900, 2700),
}

START_TIMEOUT_S = 5.0  # Time to wait for "Transmitting CW" after sending AT+TX_CW.
STOP_TIMEOUT_S = 15.0  # Time to wait for OK/ERROR after the stop keystroke (firmware re-inits the LR2021).


class SweepError(Exception):
    pass


def find_port() -> str:
    candidates = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        raise SweepError("No serial port found; pass --port.")
    raise SweepError(f"Multiple serial ports found, pass --port: {', '.join(candidates)}")


def open_port(port: str, baud: int) -> serial.Serial:
    # Opening the port normally asserts DTR, which resets the 1421 behind some adapters. Configure the
    # port with DTR deasserted before opening so the device keeps running.
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.05
    ser.write_timeout = 1.0
    ser.dtr = False
    ser.rts = False
    ser.open()
    ser.reset_input_buffer()
    return ser


def read_until(ser: serial.Serial, needles: tuple[str, ...], timeout_s: float, echo: bool = False) -> tuple[str | None, str]:
    """Read lines until one contains any needle. Returns (matched_needle_or_None, everything_read)."""
    deadline = time.monotonic() + timeout_s
    buf = b""
    seen = ""
    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode("utf-8", errors="replace").strip("\r")
            seen += text + "\n"
            if echo and text.strip():
                print(f"    < {text}")
            for n in needles:
                if n in text:
                    return n, seen
    # Check any partial trailing line too (prompt-style output without newline).
    tail = buf.decode("utf-8", errors="replace")
    seen += tail
    for n in needles:
        if n in tail:
            return n, seen
    return None, seen


def probe_baud(port: str, requested: int | None, verbose: bool) -> serial.Serial:
    """Open the port at the requested baud, or walk the firmware whitelist until the device answers."""
    bauds = (requested,) if requested else ALLOWED_BAUDS
    for baud in bauds:
        ser = open_port(port, baud)
        # Flush any half-typed junk on the device side, then ask a harmless query. Queries print no OK, so we
        # just look for any legible response.
        ser.write(b"\r\n")
        time.sleep(0.05)
        ser.reset_input_buffer()
        ser.write(b"AT+UPTIME?\r\n")
        matched, seen = read_until(ser, ("UPTIME", "ERROR"), 1.0)
        if matched:
            if verbose:
                print(f"Connected to {port} at {baud} baud.")
            return ser
        ser.close()
        if verbose:
            print(f"No response at {baud} baud.")
    raise SweepError(f"Device did not answer on {port} at {'requested baud' if requested else 'any whitelisted baud'}.")


def start_cw(ser: serial.Serial, band: str, freq_mhz: float, power_dbm: int | None, verbose: bool) -> str:
    ser.reset_input_buffer()
    freq_str = f"{freq_mhz:.3f}".rstrip("0").rstrip(".")
    cmd = f"AT+TX_CW={band},{freq_str}"
    if power_dbm is not None:
        cmd += f",{power_dbm}"
    if verbose:
        print(f"    > {cmd}")
    ser.write((cmd + "\r\n").encode())
    matched, seen = read_until(ser, ("Transmitting CW", "ERROR"), START_TIMEOUT_S, echo=verbose)
    if matched != "Transmitting CW":
        raise SweepError(f"{cmd} failed:\n{seen.strip()}")
    return cmd


def stop_cw(ser: serial.Serial, verbose: bool) -> None:
    # Any single byte stops the carrier; the firmware drains the rest of the keystroke itself.
    ser.write(b"\n")
    matched, seen = read_until(ser, ("OK", "ERROR"), STOP_TIMEOUT_S, echo=verbose)
    if matched != "OK":
        raise SweepError(f"Stopping CW carrier failed:\n{seen.strip()}")


def frange(start: float, stop: float, step: float):
    # Integer kHz stepping avoids float accumulation error over thousands of steps.
    start_khz, stop_khz, step_khz = (round(v * 1000) for v in (start, stop, step))
    if step_khz <= 0:
        raise SweepError("--step must be positive.")
    f = start_khz
    while f <= stop_khz:
        yield f / 1000.0
        f += step_khz


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", help="Serial port (default: the single /dev/cu.usbmodem* or /dev/ttyACM* present)")
    p.add_argument("--baud", type=int, help="Console baud (default: probe the firmware whitelist, 1000000 first)")
    p.add_argument("--band", choices=sorted(BANDS), default="LRHF")
    p.add_argument("--start", type=float, help="Start frequency, MHz (default: band minimum)")
    p.add_argument("--stop", type=float, help="Stop frequency, MHz inclusive (default: band maximum)")
    p.add_argument("--step", type=float, default=0.1, help="Step, MHz (default 0.1 = 100 kHz)")
    p.add_argument("--power", type=int, default=0, help="TX power dBm for LRLF/LRHF (default 0); ignored for SUBG")
    p.add_argument("--dwell", type=float, default=1.0, help="Seconds to hold each carrier (default 1.0)")
    p.add_argument("--manual", action="store_true", help="Wait for Enter instead of --dwell before advancing")
    p.add_argument("--csv", help="Append one row per step (utc_time, band, freq_mhz, power_dbm, on_s) to this file")
    p.add_argument("-v", "--verbose", action="store_true", help="Echo AT traffic")
    args = p.parse_args(argv)

    band_min, band_max = BANDS[args.band]
    start = args.start if args.start is not None else band_min
    stop = args.stop if args.stop is not None else band_max
    if not (band_min <= start <= stop <= band_max):
        p.error(f"{args.band} range is {band_min}-{band_max} MHz; got start={start} stop={stop}")
    if args.band == "SUBG" and any(round(v * 1000) % 1000 for v in (start, stop, args.step)):
        p.error("SUBG only supports whole-MHz frequencies and steps")
    power = None if args.band == "SUBG" else args.power

    freqs = list(frange(start, stop, args.step))
    est_s = len(freqs) * (args.dwell + 1.5)  # Allow ~1.5 s/step for the AT round trip and LR2021 re-init.
    est = f"~{est_s / 60:.0f} min" if est_s >= 120 else f"~{est_s:.0f} s"
    print(f"Sweeping {args.band} {start:g} -> {stop:g} MHz in {args.step:g} MHz steps: {len(freqs)} points"
          + (", manual advance" if args.manual else f", {args.dwell:g} s dwell ({est} total)")
          + (f", {power:+d} dBm" if power is not None else ", +12 dBm (SmartRF table)"), flush=True)

    try:
        port = args.port or find_port()
        ser = probe_baud(port, args.baud, True)
    except (SweepError, serial.SerialException) as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1

    csv_file = None
    writer = None
    if args.csv:
        csv_file = open(args.csv, "a", newline="")
        writer = csv.writer(csv_file)
        if csv_file.tell() == 0:
            writer.writerow(["utc_time", "band", "freq_mhz", "power_dbm", "on_s"])

    carrier_on = False
    rc = 0
    try:
        for i, f in enumerate(freqs, 1):
            print(f"[{i}/{len(freqs)}] {f:.3f} MHz", end="", flush=True)
            start_cw(ser, args.band, f, power, args.verbose)
            carrier_on = True
            t0 = time.monotonic()
            if args.manual:
                input("  -- carrier on; press Enter for next ")
            else:
                time.sleep(args.dwell)
            on_s = time.monotonic() - t0
            stop_cw(ser, args.verbose)
            carrier_on = False
            print(f"  ok ({on_s:.1f} s)")
            if writer:
                writer.writerow([datetime.utcnow().isoformat(timespec="milliseconds"), args.band, f"{f:.3f}",
                                 "" if power is None else power, f"{on_s:.2f}"])
                csv_file.flush()
        print("Sweep complete.")
    except KeyboardInterrupt:
        print("\nInterrupted.")
        rc = 130
    except SweepError as e:
        print(f"\nERROR: {e}", file=sys.stderr)
        rc = 1
    finally:
        if carrier_on:
            print("Stopping carrier...")
            try:
                stop_cw(ser, args.verbose)
            except SweepError as e:
                print(f"WARNING: {e}\nDevice may still be transmitting; power-cycle or send any key on the console.",
                      file=sys.stderr)
                rc = rc or 1
        ser.close()
        if csv_file:
            csv_file.close()
    return rc


if __name__ == "__main__":
    sys.exit(main())
