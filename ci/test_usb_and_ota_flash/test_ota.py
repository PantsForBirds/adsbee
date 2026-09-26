#!/usr/bin/env python3
"""
CI-driven OTA update tester for ADSBee.

Architecture (from AGENTS.md):
  USB connects to the RP2040 (main processor).
  combined.uf2 embeds all three processor binaries (RP2040 + ESP32 + CC1312).
  Flashing: RP2040 in BOOTSEL mode mounts as RPI-RP2 USB mass storage;
            copying combined.uf2 triggers an automatic reboot.
  OTA (.ota file) updates the RP2040 via the ESP32 WebSocket → SPI path.
  Health is checked via the ESP32's /metrics WebSocket over WiFi.

Steps:
  1. Trigger BOOTSEL, wait for RPI-RP2 USB drive, flash combined.uf2.
  2. Verify base firmware health via /metrics WebSocket.
  3. Upload candidate .ota via WebSocket AT protocol   [full test / --ota-only].
  4. Verify post-OTA firmware health                   [full test / --ota-only].
  5. Confirm the device booted the new partition       [full test / --ota-only].

Usage (full test):
  python3 test_ota.py --uf2 combined.uf2 --ota-fw adsbee_1090.ota

Usage (USB flash + health check only):
  python3 test_ota.py --usb-only --uf2 combined.uf2

Usage (OTA upload + health check only, device already running base firmware):
  python3 test_ota.py --ota-only --ota-fw adsbee_1090.ota

On a bench with several boards, select the device with --serial <USB serial> (see ci/hil).
"""

import argparse
import asyncio
import os
import shutil
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent
sys.path.insert(0, str(SCRIPT_DIR))

import check_device
import ota_upload
import reboot_to_bootloader

# Device selection and locking are shared with the bench toolkit in ci/hil (stdlib only here).
sys.path.insert(0, str(SCRIPT_DIR.parent / "hil"))
from adsbee_hil import lock as hil_lock  # noqa: E402
from adsbee_hil import usb as hil_usb  # noqa: E402

RED    = '\033[0;31m'
GREEN  = '\033[0;32m'
YELLOW = '\033[1;33m'
RESET  = '\033[0m'


def log(msg: str)  -> None: print(f"[{time.strftime('%H:%M:%S')}] {msg}")
def ok(msg: str)   -> None: log(f"{GREEN}PASS{RESET}  {msg}")
def fail(msg: str) -> None: log(f"{RED}FAIL{RESET}  {msg}")
def info(msg: str) -> None: log(f"{YELLOW}----{RESET}  {msg}")


def find_rpi_mount() -> Path | None:
    user = os.environ.get("USER", "")
    for candidate in [
        Path("/media/RPI-RP2"),
        Path(f"/media/{user}/RPI-RP2"),
        Path(f"/run/media/{user}/RPI-RP2"),
    ]:
        if candidate.is_dir():
            return candidate
    return None


def usb_port_of_tty(port: str) -> str | None:
    """USB port path (e.g. "1-1.3") of the device that owns a tty such as /dev/ttyACM0 or a by-id link."""
    name = os.path.basename(os.path.realpath(port))
    for d in hil_usb.usb_devices():
        if name in d.ttys:
            return d.port_path
    return None


def wait_for_rpi_mount(timeout: int = 30) -> Path | None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        mp = find_rpi_mount()
        if mp:
            return mp
        time.sleep(1)
    return None


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--uf2", metavar="FILE",
                        help="combined.uf2 to flash (required unless --ota-only)")
    parser.add_argument("-o", "--ota-fw", metavar="FILE",
                        help="candidate .ota for OTA test")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--usb-only", action="store_true",
                      help="flash + verify base firmware only; skip OTA steps")
    mode.add_argument("--ota-only", action="store_true",
                      help="OTA upload + verify only; skip UF2 flash (device must already be running)")
    parser.add_argument("-p", "--port", metavar="DEV",
                        help="USB serial device (e.g. /dev/ttyACM0) — no WiFi needed for BOOTSEL")
    parser.add_argument("-s", "--serial", metavar="USB_SERIAL",
                        help="select the device by USB serial number (implies --port); unambiguous on "
                             "a bench with several RP2040 boards. The RPI-RP2 drive is then matched "
                             "by USB port and the device is locked for the run (see ci/hil)")
    parser.add_argument("--lock-timeout", type=float, default=1800.0, metavar="SEC",
                        help="with --serial: max wait for another job using the device (default: 1800)")
    parser.add_argument("-H", "--host", metavar="NAME", default="cibee.local",
                        help="device hostname or IP (default: cibee.local)")
    parser.add_argument("--boot-wait", type=int, default=5, metavar="SEC",
                        help="wait after UF2 flash in seconds (default: 5)")
    parser.add_argument("--ota-wait", type=int, default=5, metavar="SEC",
                        help="wait after OTA reboot in seconds (default: 5)")
    parser.add_argument("--health-timeout", type=float, default=30.0, metavar="SEC",
                        help="metrics wait timeout in seconds (default: 30)")
    parser.add_argument("--expected-version", metavar="VER",
                        help="expected RP2040 firmware version (e.g. 0.9.0-rc19); "
                             "if set, health checks also verify AT+DEVICE_INFO? reports it")
    args = parser.parse_args()

    if not args.ota_only and not args.uf2:
        parser.error("--uf2 is required (or pass --ota-only to skip UF2 flash)")
    if not args.usb_only and not args.ota_fw:
        parser.error("--ota-fw is required (or pass --usb-only to skip OTA steps)")

    uf2: Path | None = None
    if args.uf2:
        uf2 = Path(args.uf2)
        if not uf2.is_file():
            sys.exit(f"Error: UF2 file not found: {uf2}")

    ota: Path | None = None
    if args.ota_fw:
        ota = Path(args.ota_fw)
        if not ota.is_file():
            sys.exit(f"Error: OTA file not found: {ota}")

    host = args.host

    # With --serial, resolve the console and USB port from the serial number, and hold the
    # device's lock (shared with ci/hil's adsbee-hil) so nothing else drives it meanwhile.
    usb_port: str | None = None
    if args.serial:
        dev = hil_usb.find_by_serial(args.serial)
        if not dev or not dev.console:
            sys.exit(f"Error: no USB serial console for serial {args.serial} "
                     f"(attached: {[d.serial for d in hil_usb.adsbee_candidates()]})")
        args.port = dev.console
        usb_port = dev.port_path
        info(f"Device {args.serial}: {args.port} on USB port {usb_port}")
        hil_lock.DeviceLock(args.serial, timeout=args.lock_timeout, purpose="test_ota.py").acquire()
    elif args.port:
        usb_port = usb_port_of_tty(args.port)

    durations: dict[str, float] = {}
    overall_t0 = time.monotonic()

    if not args.ota_only:
        # ── Step 1: Flash base firmware ───────────────────────────────────────
        log("━━━ Step 1: Flash base firmware via UF2 ━━━")
        step_t0 = time.monotonic()

        if args.port:
            info(f"Triggering BOOTSEL mode via serial ({args.port})...")
            reboot_to_bootloader.reboot_via_serial(args.port)
        else:
            info(f"Triggering BOOTSEL mode via WebSocket ({host})...")
            asyncio.run(reboot_to_bootloader.reboot_via_websocket(host))

        time.sleep(2)

        if usb_port:
            # Only the drive on this device's USB port: another board on the bench may be in
            # BOOTSEL too, and copying there would flash the wrong device.
            info(f"Waiting for the RPI-RP2 drive on USB port {usb_port} (up to 30s)...")
            try:
                rpi_mount = Path(hil_usb.mount_rp2_drive(usb_port, timeout=30))
            except RuntimeError as exc:
                fail(f"RPI-RP2 drive did not appear: {exc}")
                sys.exit(1)
        else:
            info("Waiting for RPI-RP2 USB drive (up to 30s)...")
            rpi_mount = wait_for_rpi_mount(30)
            if not rpi_mount:
                fail("RPI-RP2 drive did not appear. Is the device in BOOTSEL mode?")
                sys.exit(1)
        info(f"Found RPI-RP2 at {rpi_mount}")

        info(f"Copying {uf2} → {rpi_mount} ...")
        try:
            shutil.copy(str(uf2), str(rpi_mount))
            os.sync()
            ok("UF2 flash succeeded (device will reboot automatically)")
        except Exception as exc:
            fail(f"UF2 copy failed: {exc}")
            sys.exit(1)

        info(f"Waiting {args.boot_wait}s for device to boot (RP2040 will reflash ESP32/CC1312 if versions differ)...")
        time.sleep(args.boot_wait)
        durations["1. UF2 flash"] = time.monotonic() - step_t0

        # ── Step 2: Verify base firmware ──────────────────────────────────────
        log(f"━━━ Step 2: Verify base firmware health ({host}) ━━━")
        step_t0 = time.monotonic()
        if not asyncio.run(check_device.check_health(host, args.health_timeout, args.expected_version)):
            fail("Base firmware not responding")
            sys.exit(1)
        ok("Base firmware healthy")
        durations["2. Base FW health"] = time.monotonic() - step_t0

        if args.usb_only:
            total = time.monotonic() - overall_t0
            print()
            log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
            log(" Test summary  (--usb-only)")
            log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
            log(f"  1. UF2 flash:       {GREEN}PASS{RESET}   {durations['1. UF2 flash']:6.1f}s")
            log(f"  2. Base FW health:  {GREEN}PASS{RESET}   {durations['2. Base FW health']:6.1f}s")
            log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
            log(f"  Total:              {total:6.1f}s")
            log(f"{GREEN}ALL PASSED{RESET}")
            return

    # ── Step 3: OTA upload ────────────────────────────────────────────────────
    log(f"━━━ Step 3: OTA upload ({ota} → {host}) ━━━")
    ota_ok = False
    target_partition: int | None = None
    step_t0 = time.monotonic()
    try:
        target_partition = asyncio.run(ota_upload.upload(host, str(ota)))
        ok(f"OTA upload succeeded (wrote partition {target_partition})")
        ota_ok = True
    except Exception as exc:
        fail(f"OTA upload failed: {exc}")
    durations["3. OTA upload"] = time.monotonic() - step_t0

    # ── Step 4: Verify post-OTA firmware ─────────────────────────────────────
    log(f"━━━ Step 4: Verify post-OTA firmware health ({host}) ━━━")
    step_t0 = time.monotonic()
    info(f"Waiting {args.ota_wait}s for device to reboot after OTA...")
    time.sleep(args.ota_wait)
    post_ok = asyncio.run(check_device.check_health(host, args.health_timeout, args.expected_version))
    if post_ok:
        ok("Post-OTA firmware healthy")
    else:
        fail("Post-OTA firmware not responding")
    durations["4. Post-OTA health"] = time.monotonic() - step_t0

    # ── Step 5: Confirm the device booted the freshly-written partition ──────
    # GET_PARTITION reports the *complementary* (inactive) partition. After a
    # successful OTA boot the active partition flips, so it must now differ from
    # the partition we wrote to. Equal value ⇒ the device fell back to the old
    # image (a no-op OTA), which a same-build version check alone cannot catch.
    boot_ok = False
    if ota_ok and target_partition is not None:
        log(f"━━━ Step 5: Confirm partition flip ({host}) ━━━")
        step_t0 = time.monotonic()
        try:
            new_complementary = asyncio.run(ota_upload.get_partition(host))
            if new_complementary != target_partition:
                ok(f"Partition flipped — now running {target_partition} "
                   f"(complementary is {new_complementary})")
                boot_ok = True
            else:
                fail(f"Partition did NOT flip (still {new_complementary}); "
                     f"device booted the old image")
        except Exception as exc:
            fail(f"Could not read partition after OTA: {exc}")
        durations["5. Partition flip"] = time.monotonic() - step_t0

    # ── Summary ───────────────────────────────────────────────────────────────
    fails = (0 if ota_ok else 1) + (0 if post_ok else 1)
    if ota_ok:
        fails += (0 if boot_ok else 1)

    total = time.monotonic() - overall_t0

    print()
    log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    log(f" Test summary  {'(--ota-only)' if args.ota_only else ''}")
    log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    if not args.ota_only:
        log(f"  1. UF2 flash:        {GREEN}PASS{RESET}   {durations['1. UF2 flash']:6.1f}s")
        log(f"  2. Base FW health:   {GREEN}PASS{RESET}   {durations['2. Base FW health']:6.1f}s")
    log(f"  3. OTA upload:       {GREEN if ota_ok  else RED}{'PASS' if ota_ok  else 'FAIL'}{RESET}   {durations['3. OTA upload']:6.1f}s")
    log(f"  4. Post-OTA health:  {GREEN if post_ok else RED}{'PASS' if post_ok else 'FAIL'}{RESET}   {durations['4. Post-OTA health']:6.1f}s")
    if "5. Partition flip" in durations:
        log(f"  5. Partition flip:   {GREEN if boot_ok else RED}{'PASS' if boot_ok else 'FAIL'}{RESET}   {durations['5. Partition flip']:6.1f}s")
    log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    log(f"  Total:               {total:6.1f}s")

    if fails == 0:
        log(f"{GREEN}ALL PASSED{RESET}")
    else:
        log(f"{RED}{fails} STEP(S) FAILED{RESET}")
        sys.exit(1)


if __name__ == "__main__":
    main()
