"""Receiver drivers: one class per ADSBee model, selected by the ``model`` key in the bench file.

A driver knows how to reach its model's AT console (with that model's quirks), query identity
and version, reboot, flash, capture reported packets and read receive counters. Other packages
can add models through the ``adsbee_hil.receivers`` entry-point group (see ``registry.py``).

All public methods take the device's lock (reentrant), so they are safe to call from parallel
jobs sharing a bench.
"""

import contextlib
import os
import re
import shutil
import struct
import subprocess
import time
from typing import Dict, Iterator, List, Optional

from . import usb
from .config import Bench, ReceiverConfig
from .console import AtConsole, AtError, LineReader, redact
from .lock import DeviceLock
from .packets import Frame, parse_raw_frames

UF2_MAGIC = (0x0A324655, 0x9E5D5157)
UF2_FAMILY_RP2040 = 0xE48BFF56
UF2_FLAG_FAMILY = 0x2000
BOOT_USB_UF2_CMD = "AT+BOOT_USB_UF2=1DEADBEE"


class HilError(RuntimeError):
    pass


def check_uf2(path: str, family: int = UF2_FAMILY_RP2040) -> None:
    with open(path, "rb") as f:
        hdr = f.read(32)
    if len(hdr) < 32:
        raise HilError(f"{path} is not a UF2 file (too short)")
    m0, m1, flags, _addr, _size, _blk, _nblk, fam = struct.unpack("<8I", hdr)
    if (m0, m1) != UF2_MAGIC:
        raise HilError(f"{path} is not a UF2 file")
    if flags & UF2_FLAG_FAMILY and fam != family:
        raise HilError(f"{path}: UF2 family 0x{fam:08x}, expected 0x{family:08x}")


def _ancestors() -> set:
    pids, pid = set(), os.getpid()
    while pid > 1 and pid not in pids:
        pids.add(pid)
        try:
            with open(f"/proc/{pid}/stat") as f:
                pid = int(f.read().rsplit(")", 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            break
    return pids


def busy_process(patterns: List[str]) -> Optional[str]:
    """First pattern found in a running process's command line, or None.

    Like ``pgrep -f``, but ignores this process and its ancestors (a shell whose command line
    merely mentions the pattern shouldn't count).
    """
    if not patterns:
        return None
    mine = _ancestors()
    for d in os.listdir("/proc"):
        if not d.isdigit() or int(d) in mine:
            continue
        try:
            with open(f"/proc/{d}/cmdline", "rb") as f:
                cmdline = f.read().replace(b"\0", b" ").decode(errors="replace")
        except OSError:
            continue
        for pat in patterns:
            if re.search(pat, cmdline):
                return pat
    return None


def parse_key_values(text: str) -> Dict[str, str]:
    """'Key: value' lines (AT+DEVICE_INFO?) to a dict, with secrets dropped."""
    out = {}
    for line in text.splitlines():
        m = re.match(r"^\s*([A-Za-z0-9][A-Za-z0-9 ./()-]*?):\s*(.*?)\s*$", line)
        if m and not m.group(1).startswith("OTA Key"):
            out[m.group(1)] = m.group(2)
    return out


def parse_counters(text: str) -> Dict[str, int]:
    """'k=v,k=v' counters (e.g. AT+RX_STATS?) to a dict of ints."""
    return {k: int(v) for k, v in re.findall(r"([a-z][a-z0-9_]*)=(-?\d+)", text)}


class Receiver:
    model = "generic"
    description = ""
    console_baud = 115200
    keep_lines = False
    version_key: Optional[str] = None  # DEVICE_INFO key holding the main firmware version.

    def __init__(self, cfg: ReceiverConfig, bench: Optional[Bench] = None, lock_timeout: float = 600.0):
        self.cfg = cfg
        self.bench = bench or Bench()
        self.lock_timeout = lock_timeout
        self._last_port_path = cfg.usb_port

    # -- identity / location ------------------------------------------------------------------

    @property
    def id(self) -> str:
        return self.cfg.id

    def __repr__(self) -> str:
        return f"<{type(self).__name__} {self.id} serial={self.cfg.usb_serial}>"

    def lock(self, purpose: str = "") -> DeviceLock:
        return DeviceLock(self.cfg.usb_serial, self.bench.lock_dir, self.lock_timeout, purpose, self.id)

    def usb(self) -> Optional[usb.UsbDevice]:
        d = usb.find_by_serial(self.cfg.usb_serial)
        if d:
            if self.cfg.usb_port and d.port_path != self.cfg.usb_port:
                raise HilError(f"{self.id}: serial {self.cfg.usb_serial} is on USB port {d.port_path}, "
                               f"but the bench file says {self.cfg.usb_port}; recable or fix the bench file")
            self._last_port_path = d.port_path
        return d

    def port_path(self) -> Optional[str]:
        d = self.usb()
        return d.port_path if d else self._last_port_path

    def state(self) -> str:
        """'app', 'bootsel' or 'absent'."""
        d = self.usb()
        if d and d.ttys:
            return "app"
        if self._last_port_path:
            other = usb.find_by_port(self._last_port_path)
            if other and other.is_bootsel:
                return "bootsel"
        return "absent" if not d else d.mode

    def console_path(self) -> str:
        d = self.usb()
        if not d or not d.console:
            raise HilError(f"{self.id}: no serial console for USB serial {self.cfg.usb_serial} "
                           f"(state: {self.state()})")
        return d.console

    # -- AT console ---------------------------------------------------------------------------

    @contextlib.contextmanager
    def console(self) -> Iterator[AtConsole]:
        with self.lock("console"):
            c = AtConsole(self.console_path(), self.console_baud, self.keep_lines).open()
            try:
                yield c
            finally:
                c.close()

    def check_command(self, cmd: str, allow_tx: bool = False) -> None:
        """Refuses commands that make a receiver transmit, unless allow_tx."""
        if not allow_tx and re.match(r"^\s*AT\+(TX_CW|REMOTE_ID_TX)\b", cmd, re.IGNORECASE):
            raise HilError(f"{self.id}: '{cmd}' transmits; pass allow_tx/--allow-tx if you mean it")

    def at(self, cmd: str, timeout: float = 3.0, allow_tx: bool = False) -> str:
        self.check_command(cmd, allow_tx)
        with self.console() as c:
            return self._transact(c, cmd, timeout)

    def _transact(self, c: AtConsole, cmd: str, timeout: float) -> str:
        out = c.transact(cmd, timeout)
        if "Press any key" in out:
            out += c.interrupt()  # Streaming commands (RX_CW) run until a byte arrives.
        return out

    def at_retry(self, cmd: str, total_timeout: float, timeout: float = 3.0) -> str:
        """at(), retried until the console answers or total_timeout expires (e.g. after a flash)."""
        deadline = time.monotonic() + total_timeout
        err: Exception = HilError("no attempt")
        while time.monotonic() < deadline:
            try:
                out = self.at(cmd, timeout)
                if out.strip():
                    return out
            except (AtError, HilError, OSError) as e:
                err = e
            time.sleep(2)
        raise HilError(f"{self.id}: no answer to {cmd} within {total_timeout:.0f} s ({err})")

    def device_info(self) -> Dict[str, str]:
        return parse_key_values(self.at("AT+DEVICE_INFO?"))

    def firmware_version(self) -> Optional[str]:
        return self.device_info().get(self.version_key) if self.version_key else None

    def uptime(self) -> Optional[int]:
        m = re.search(r"UPTIME=(\d+)", self.at("AT+UPTIME?"))
        return int(m.group(1)) if m else None

    def reboot(self) -> None:
        with self.lock("reboot"):
            with self.console() as c:
                c.write(b"AT+REBOOT\r\n")
                time.sleep(0.2)
            time.sleep(2)
            self.at_retry("AT+UPTIME?", 30)

    def rx_counters(self) -> Dict[str, int]:
        """Receive counters, if the model has them (empty dict otherwise)."""
        return {}

    # -- packet capture -----------------------------------------------------------------------

    @contextlib.contextmanager
    def capture(self, protocol: str = "RAW") -> Iterator["Capture"]:
        """Switches the console's reporting protocol to RAW and collects frames until exit.

        The previous CONSOLE protocol is restored afterwards (settings aren't saved to flash).
        """
        with self.console() as c:
            before = self._transact(c, "AT+PROTOCOL_OUT?", 3.0)
            m = re.search(r"CONSOLE,([A-Z0-9_]+)", before)
            previous = m.group(1) if m else None
            out = self._transact(c, f"AT+PROTOCOL_OUT=CONSOLE,{protocol}", 3.0)
            if "ERROR" in out:
                raise HilError(f"{self.id}: could not select {protocol} output: {out.strip()}")
            cap = Capture(LineReader(c).start())
            try:
                yield cap
            finally:
                cap.stop()
                if previous:
                    c.transact(f"AT+PROTOCOL_OUT=CONSOLE,{previous}", 3.0)

    # -- flashing -----------------------------------------------------------------------------

    def check_not_busy(self, force: bool) -> None:
        patterns = self.cfg.options.get("busy_processes", self.bench.busy_processes)
        hit = busy_process(patterns)
        if hit and not force:
            raise HilError(f"{self.id}: a process matching '{hit}' is running (e.g. a CI job owns this "
                           "bench); refusing to flash. Use --force to override.")

    def flash(self, image: str, force: bool = False, **kw) -> Dict[str, str]:
        raise HilError(f"{self.model}: flashing is not implemented")

    def _copy_to_bootsel(self, image: str, port_path: str, bootsel_timeout: float) -> str:
        if not usb.wait_for(lambda: (d := usb.find_by_port(port_path)) and d.is_bootsel, bootsel_timeout, 0.5):
            raise HilError(f"{self.id}: no RP2040 BOOTSEL device on USB port {port_path} "
                           f"within {bootsel_timeout:.0f} s")
        mp = usb.mount_rp2_drive(port_path)
        print(f"[{self.id}] copying {os.path.basename(image)} -> {mp} (USB port {port_path})", flush=True)
        shutil.copy(image, mp)
        os.sync()
        return mp


class Capture:
    def __init__(self, reader: LineReader):
        self._reader = reader
        self.lines: List[str] = reader.lines

    def stop(self) -> None:
        if self._reader:
            self.lines = self._reader.stop()
            self._reader = None

    @property
    def frames(self) -> List[Frame]:
        return parse_raw_frames(self.lines)


class Adsbee1090U(Receiver):
    """ADSBee 1090U: RP2040 + ESP32 + CC1312. USB CDC console on the RP2040 (baud ignored).

    Flashing a combined.uf2 reboots the RP2040 into BOOTSEL (AT+BOOT_USB_UF2), copies the image to
    the RPI-RP2 drive on the same USB port, and waits for the app; the RP2040 then updates the
    ESP32 and CC1312 from the images embedded in combined.uf2. OTA (.ota over the ESP32 WebSocket)
    goes through ci/test_usb_and_ota_flash/ota_upload.py.
    """

    model = "adsbee_1090u"
    description = "ADSBee 1090U (RP2040 console over USB CDC)"
    version_key = "RP2040 Firmware Version"

    def flash(self, image: str, force: bool = False, host: Optional[str] = None,
              boot_timeout: float = 90.0, **kw) -> Dict[str, str]:
        image = os.path.abspath(image)
        if not os.path.isfile(image):
            raise HilError(f"{image} not found")
        self.check_not_busy(force)
        if image.endswith(".ota"):
            return self._flash_ota(image, host or self.cfg.options.get("host"))
        check_uf2(image)
        with self.lock("flash"):
            port_path = self.port_path()
            if self.state() == "app":
                with self.console() as c:
                    print(f"[{self.id}] {BOOT_USB_UF2_CMD} via {c.port} (USB port {port_path})", flush=True)
                    c.write((BOOT_USB_UF2_CMD + "\r\n").encode())
                    time.sleep(0.2)
            if not port_path:
                raise HilError(f"{self.id}: not on USB, and no usb_port in the bench file to find its BOOTSEL drive")
            self._copy_to_bootsel(image, port_path, 20)
            if not usb.wait_for(lambda: self.state() == "app", boot_timeout, 1.0):
                raise HilError(f"{self.id}: did not come back in application mode within {boot_timeout:.0f} s")
            return parse_key_values(self.at_retry("AT+DEVICE_INFO?", 60))

    def _flash_ota(self, image: str, host: Optional[str]) -> Dict[str, str]:
        if not host:
            raise HilError(f"{self.id}: OTA needs the device's host name (--host, or 'host' in the bench file)")
        import asyncio
        import sys

        ota_dir = os.path.join(os.path.dirname(__file__), "..", "..", "test_usb_and_ota_flash")
        if not os.path.isfile(os.path.join(ota_dir, "ota_upload.py")):
            raise HilError("OTA needs ci/test_usb_and_ota_flash/ota_upload.py (run from a repo checkout)")
        sys.path.insert(0, os.path.abspath(ota_dir))
        import ota_upload  # noqa: E402

        with self.lock("ota"):
            partition = asyncio.run(ota_upload.upload(host, image))
            print(f"[{self.id}] OTA wrote partition {partition}; waiting for reboot", flush=True)
            time.sleep(5)
            return parse_key_values(self.at_retry("AT+DEVICE_INFO?", 60))


class Adsbee1421(Receiver):
    """ADSBee 1421 (TI CC1314R10 + Semtech LR2021) behind an "ADSBee 1421 Programmer" jig.

    The module has no USB. The USB device (and ``usb_serial``) is the jig's RP2040
    (firmware/adsbee_1421/programmer), a USB<->UART bridge whose modem-control lines drive the
    module: RTS asserted -> SYNC low (awake), RTS deasserted -> SYNC high (host-controlled sleep /
    ROM bootloader backdoor armed), DTR assert edge -> reset pulse. Hence:

    * HUPCL is cleared so DTR/RTS stay asserted after close (else every close sleeps the module).
    * The jig copies the host's line coding to the UART, so the host must open at the console's
      live baud: 1 000 000 at factory default, else one of the firmware's whitelisted rates.
      Never send AT+BAUD_RATE=CONSOLE,... through the jig.
    * The AT parser rejects a bare "AT"; AT+UPTIME? is the probe.

    Module firmware (.hex) goes through the CC13x4 ROM serial bootloader using an external flasher
    command from the bench file (``[flashers.adsbee_1421] command``), since the flasher is not
    part of this repository. It must erase only the sectors the image covers: a full bank erase
    also wipes the settings and device-info (OTA keys) sectors. The jig reflashes the module with
    its own baked image at every power-up if they differ, so a .hex flashed this way lasts until
    the jig next re-enumerates. Jig images (.uf2) need a human to hold BOOT while replugging.
    """

    model = "adsbee_1421"
    description = "ADSBee 1421 via the programmer jig (UART console bridged over USB CDC)"
    console_baud = 1_000_000
    console_bauds = [1_000_000, 921_600, 460_800, 230_400, 115_200]
    keep_lines = True
    version_key = "CC1314R10 Firmware Version"

    @contextlib.contextmanager
    def console(self) -> Iterator[AtConsole]:
        with self.lock("console"):
            port = self.console_path()
            last_err = None
            for i, baud in enumerate(self._baud_order()):
                c = AtConsole(port, baud, keep_lines=True).open()
                ok = False
                try:
                    # The first open after the jig enumerated (or after a tool closed the port with
                    # HUPCL set) resets the module; give it a few tries to boot.
                    for _ in range(3 if i == 0 else 1):
                        c.write(b"\r\n")
                        time.sleep(0.05)
                        if "UPTIME=" in c.transact("AT+UPTIME?", 1.0, 0.3):
                            ok = True
                            break
                        if i == 0:
                            time.sleep(1.0)
                except OSError as e:
                    last_err = e
                if not ok:
                    c.close()
                    continue
                self.console_baud = baud
                try:
                    yield c
                finally:
                    c.close()
                return
            raise AtError(f"{self.id}: console did not answer at any of {self.console_bauds} baud"
                          f"{f' ({last_err})' if last_err else ''}. Tap BOOTSEL on the jig or replug it.")

    def _baud_order(self) -> List[int]:
        return [self.console_baud] + [b for b in self.console_bauds if b != self.console_baud]

    def check_command(self, cmd: str, allow_tx: bool = False) -> None:
        super().check_command(cmd, allow_tx)
        if re.match(r"^\s*AT\+BAUD_RATE\s*=\s*CONSOLE", cmd, re.IGNORECASE):
            raise HilError(f"{self.id}: changing the console baud through the jig desyncs the bridge")

    def rx_counters(self) -> Dict[str, int]:
        return parse_counters(self.at("AT+RX_STATS?"))

    def reset_rx_counters(self) -> None:
        self.at("AT+RX_STATS=RESET")

    def flash(self, image: str, force: bool = False, human_timeout: float = 300.0, verbose: bool = False,
              **kw) -> Dict[str, str]:
        image = os.path.abspath(image)
        if not os.path.isfile(image):
            raise HilError(f"{image} not found")
        self.check_not_busy(force)
        if image.endswith(".uf2"):
            return self._flash_jig(image, human_timeout)
        if not image.endswith(".hex"):
            raise HilError("1421 images are .hex (module firmware) or .uf2 (programmer jig image)")
        template = self.bench.flasher_command(self.model, self.cfg)
        if not template:
            raise HilError(
                f"{self.id}: no flasher configured for {self.model}. Set [flashers.{self.model}] command in "
                f"the bench file or ADSBEE_HIL_FLASHER_{self.model.upper()}; see ci/hil/README.md")
        with self.lock("flash"):
            port = self.console_path()
            fields = {"image": image, "port": port, "baud": str(self.console_bauds[0]),
                      "verbose": "-v" if verbose else ""}
            cmd = [a.format(**fields) for a in template]
            cmd = [a for a in cmd if a != ""]
            print(f"[{self.id}] + {' '.join(cmd)}", flush=True)
            r = subprocess.run(cmd)
            if r.returncode != 0:
                raise HilError(f"{self.id}: flasher exited {r.returncode}")
            # The flasher's close dropped RTS (module asleep); our next open wakes and resets it.
            self.console_baud = self.console_bauds[0]
            time.sleep(1.0)
            return parse_key_values(self.at_retry("AT+DEVICE_INFO?", 30))

    def _flash_jig(self, image: str, human_timeout: float) -> Dict[str, str]:
        check_uf2(image)
        with self.lock("flash-jig"):
            port_path = self.port_path()
            if not port_path:
                raise HilError(f"{self.id}: jig not on USB and no usb_port in the bench file")
            d = usb.find_by_port(port_path)
            if not (d and d.is_bootsel):
                print(f"[{self.id}] HUMAN NEEDED: the jig has no software reboot to BOOTSEL. Hold BOOT on "
                      f"the jig's RP2040 while replugging its USB (port {port_path}). Waiting "
                      f"{human_timeout:.0f} s ...", flush=True)
            self._copy_to_bootsel(image, port_path, human_timeout)
            # The jig then CRC-checks the module against its baked image and reflashes it (~1 min).
            if not usb.wait_for(lambda: self.state() == "app", 60, 1.0):
                raise HilError(f"{self.id}: jig did not come back in application mode")
            time.sleep(5)
            return parse_key_values(self.at_retry("AT+DEVICE_INFO?", 120))


BUILTIN_MODELS = {cls.model: cls for cls in (Adsbee1090U, Adsbee1421)}

__all__ = ["Receiver", "Adsbee1090U", "Adsbee1421", "HilError", "BUILTIN_MODELS", "check_uf2",
           "parse_key_values", "parse_counters", "redact", "Capture"]
