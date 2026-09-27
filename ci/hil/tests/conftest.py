import os
import pty
import sys
import threading
import tty

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from adsbee_hil import usb  # noqa: E402


class FakeSysfs:
    """A fake /sys + /dev tree with USB devices, tty nodes, by-id links and block devices."""

    def __init__(self, root):
        self.sys = os.path.join(root, "sys")
        self.dev = os.path.join(root, "dev")
        os.makedirs(os.path.join(self.sys, "bus", "usb", "devices"))
        os.makedirs(os.path.join(self.dev, "serial", "by-id"))

    def _dir(self, port):
        return os.path.join(self.sys, "bus", "usb", "devices", port)

    def add(self, port, vid, pid, serial="", manufacturer="", product="", tty_name=None, by_id=None,
            tty_target=None, block=None):
        d = self._dir(port)
        os.makedirs(d)
        for k, v in (("idVendor", vid), ("idProduct", pid), ("serial", serial),
                     ("manufacturer", manufacturer), ("product", product)):
            if v != "" or k in ("idVendor", "idProduct"):
                with open(os.path.join(d, k), "w") as f:
                    f.write(v + "\n")
        os.makedirs(os.path.join(d, f"{port}:1.0"))
        if tty_name:
            os.makedirs(os.path.join(d, f"{port}:1.0", "tty", tty_name))
            if by_id:
                link = os.path.join(self.dev, "serial", "by-id", by_id)
                os.symlink(tty_target or os.path.join(self.dev, tty_name), link)
        if block:
            b = os.path.join(d, f"{port}:1.0", "host0", "target0:0:0", "0:0:0:0", "block", block)
            os.makedirs(os.path.join(b, block + "1"))
        return d

    def remove(self, port):
        import shutil

        shutil.rmtree(self._dir(port))


@pytest.fixture
def fake_sysfs(tmp_path, monkeypatch):
    fs = FakeSysfs(str(tmp_path))
    monkeypatch.setattr(usb, "SYSFS_ROOT", fs.sys)
    monkeypatch.setattr(usb, "DEV_ROOT", fs.dev)
    return fs


@pytest.fixture(autouse=True)
def lock_dir(tmp_path, monkeypatch):
    d = str(tmp_path / "locks")
    monkeypatch.setenv("ADSBEE_HIL_LOCK_DIR", d)
    monkeypatch.delenv("ADSBEE_HIL_CONFIG", raising=False)
    return d


class FakeAtDevice:
    """Answers AT commands on a pty, like an ADSBee console."""

    def __init__(self, replies):
        self.replies = replies  # command -> reply text (without trailing OK)
        self.received = []
        self.master, slave = pty.openpty()
        tty.setraw(slave)
        self.path = os.ttyname(slave)
        self._slave = slave
        self._stop = False
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def _run(self):
        buf = b""
        while not self._stop:
            try:
                data = os.read(self.master, 1024)
            except OSError:
                return
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                cmd = line.decode(errors="replace").strip()
                if not cmd:
                    continue
                self.received.append(cmd)
                reply = self.replies.get(cmd)
                if reply is None:
                    text = "ERROR: unknown command\r\n"
                else:
                    text = reply + "\r\nOK\r\n"
                os.write(self.master, text.encode())

    def close(self):
        self._stop = True
        os.close(self.master)
        os.close(self._slave)


@pytest.fixture
def fake_at():
    devs = []

    def make(replies):
        d = FakeAtDevice(replies)
        devs.append(d)
        return d
    yield make
    for d in devs:
        d.close()
