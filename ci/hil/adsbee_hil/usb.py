"""USB discovery through sysfs: which ADSBee-family devices are attached, and where.

Everything here reads plain files under /sys and /dev, so it needs no third-party packages and can
be pointed at a fake tree in tests (``sysfs_root`` / ``dev_root``).

Why not VID:PID? Every RP2040-based ADSBee device (1090U, the 1421 programmer jig, a Pico) shows
up as 2e8a:000a in application mode and 2e8a:0003 in BOOTSEL, so VID:PID can't tell two boards
apart, and ``/dev/ttyACMn`` numbering depends on enumeration order. Devices are addressed by USB
serial number, and the BOOTSEL mass-storage drive by the USB *port path* (``1-1.3``), which stays
the same when the board re-enumerates as a different USB device on the same physical port.
"""

import glob
import os
import subprocess
import time
from dataclasses import dataclass, field
from typing import Iterator, List, Optional

RP2040_VID = "2e8a"
RP2040_APP_PID = "000a"
RP2040_BOOTSEL_PID = "0003"
RPI_RP2_INFO_FILE = "INFO_UF2.TXT"

# Roots used when a function isn't given one explicitly. Tests point these at a fake tree.
SYSFS_ROOT = "/sys"
DEV_ROOT = "/dev"

# USB product strings that identify a model without talking to it. The 1090U enumerates with the
# stock Pico SDK strings ("Raspberry Pi" / "Pico"), so it is only identifiable by serial (bench
# config) or by asking it (`discover --probe`).
PRODUCT_HINTS = {
    "ADSBee 1421 Programmer": "adsbee_1421",
}


@dataclass
class UsbDevice:
    port_path: str  # sysfs name of the USB device, e.g. "1-1.3" (bus 1, hub port 1, port 3).
    vid: str
    pid: str
    serial: str
    manufacturer: str
    product: str
    ttys: List[str] = field(default_factory=list)  # e.g. ["ttyACM0"]
    by_id: List[str] = field(default_factory=list)  # /dev/serial/by-id links for this serial.

    @property
    def vidpid(self) -> str:
        return f"{self.vid}:{self.pid}"

    @property
    def is_rp2040(self) -> bool:
        return self.vid == RP2040_VID

    @property
    def is_bootsel(self) -> bool:
        return self.vid == RP2040_VID and self.pid == RP2040_BOOTSEL_PID

    @property
    def mode(self) -> str:
        if self.is_bootsel:
            return "bootsel"
        return "app" if self.ttys else "other"

    @property
    def model_hint(self) -> Optional[str]:
        return PRODUCT_HINTS.get(self.product)

    @property
    def console(self) -> Optional[str]:
        """Stable path of the first CDC interface: the by-id link if udev made one, else /dev/ttyX."""
        for link in self.by_id:
            if link.endswith("-if00"):
                return link
        if self.by_id:
            return self.by_id[0]
        return f"/dev/{self.ttys[0]}" if self.ttys else None


def _read(path: str) -> str:
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return ""


def usb_devices(sysfs_root: Optional[str] = None, dev_root: Optional[str] = None) -> Iterator[UsbDevice]:
    """Yields every USB device (not interface) in sysfs, with its tty nodes and by-id links."""
    sysfs_root = sysfs_root or SYSFS_ROOT
    dev_root = dev_root or DEV_ROOT
    by_id_dir = os.path.join(dev_root, "serial", "by-id")
    for d in sorted(glob.glob(os.path.join(sysfs_root, "bus", "usb", "devices", "*"))):
        name = os.path.basename(d)
        # Interfaces are "1-1.3:1.0"; root hubs are "usb1". Real devices have idVendor.
        if ":" in name or not os.path.exists(os.path.join(d, "idVendor")):
            continue
        # CDC ACM: <intf>/tty/ttyACMn. USB-serial converters: <intf>/ttyUSBn.
        ttys = sorted({os.path.basename(t) for t in glob.glob(os.path.join(d, f"{name}:*", "tty", "tty*"))}
                      | {os.path.basename(t) for t in glob.glob(os.path.join(d, f"{name}:*", "ttyUSB*"))})
        serial = _read(os.path.join(d, "serial"))
        by_id = sorted(glob.glob(os.path.join(by_id_dir, f"*_{glob.escape(serial)}-if*"))) if serial else []
        yield UsbDevice(
            port_path=name,
            vid=_read(os.path.join(d, "idVendor")),
            pid=_read(os.path.join(d, "idProduct")),
            serial=serial,
            manufacturer=_read(os.path.join(d, "manufacturer")),
            product=_read(os.path.join(d, "product")),
            ttys=ttys,
            by_id=by_id,
        )


def adsbee_candidates(sysfs_root: Optional[str] = None, dev_root: Optional[str] = None) -> List[UsbDevice]:
    """RP2040-family devices (app or BOOTSEL): every ADSBee receiver, jig and wiggler is one."""
    return [d for d in usb_devices(sysfs_root, dev_root) if d.is_rp2040]


def find_by_serial(serial: str, sysfs_root: Optional[str] = None,
                   dev_root: Optional[str] = None) -> Optional[UsbDevice]:
    for d in usb_devices(sysfs_root, dev_root):
        if d.serial == serial:
            return d
    return None


def find_by_port(port_path: str, sysfs_root: Optional[str] = None,
                 dev_root: Optional[str] = None) -> Optional[UsbDevice]:
    for d in usb_devices(sysfs_root, dev_root):
        if d.port_path == port_path:
            return d
    return None


def block_partition(port_path: str, sysfs_root: Optional[str] = None) -> Optional[str]:
    """/dev node of the mass-storage partition exposed by the USB device at port_path (RPI-RP2)."""
    base = os.path.join(sysfs_root or SYSFS_ROOT, "bus", "usb", "devices", port_path)
    for blk in sorted(glob.glob(os.path.join(base, f"{port_path}:*", "host*", "target*", "*", "block", "*"))):
        name = os.path.basename(blk)
        parts = sorted(glob.glob(os.path.join(blk, f"{name}*[0-9]")))
        return "/dev/" + (os.path.basename(parts[0]) if parts else name)
    return None


def mountpoint_of(dev: str, mounts_file: str = "/proc/mounts") -> Optional[str]:
    try:
        with open(mounts_file) as f:
            for line in f:
                fields = line.split()
                if len(fields) >= 2 and fields[0] == dev:
                    return fields[1].replace("\\040", " ")
    except OSError:
        pass
    return None


def wait_for(pred, timeout: float, interval: float = 0.25):
    """Polls pred() until it returns something truthy (returned) or timeout expires (None)."""
    deadline = time.monotonic() + timeout
    while True:
        v = pred()
        if v:
            return v
        if time.monotonic() >= deadline:
            return None
        time.sleep(interval)


def mount_rp2_drive(port_path: str, timeout: float = 20.0) -> str:
    """Returns the mount point of the RPI-RP2 drive on USB port port_path.

    Waits for the block device, then for the desktop automounter, then tries ``udisksctl mount``.
    Matching by USB port (not by volume label) matters on a bench with several RP2040 boards: two
    RPI-RP2 drives can be mounted at once, and the first one found may be the wrong board.
    """
    dev = wait_for(lambda: block_partition(port_path), timeout)
    if not dev:
        raise RuntimeError(f"no mass-storage device under USB port {port_path} within {timeout:.0f} s")
    mp = wait_for(lambda: mountpoint_of(dev), 10)
    if not mp:
        r = subprocess.run(["udisksctl", "mount", "--no-user-interaction", "-b", dev],
                           capture_output=True, text=True)
        mp = mountpoint_of(dev)
        if not mp:
            raise RuntimeError(f"could not mount {dev} (udisksctl: {(r.stderr or r.stdout).strip()}); "
                               "mounting needs an automounter, a polkit rule or root")
    if not os.path.exists(os.path.join(mp, RPI_RP2_INFO_FILE)):
        raise RuntimeError(f"{mp} ({dev}) has no {RPI_RP2_INFO_FILE}; not an RPI-RP2 drive")
    return mp
