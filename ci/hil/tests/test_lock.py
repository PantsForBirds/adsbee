import subprocess
import sys
import threading
import time

import pytest

from adsbee_hil.lock import DeviceLock, LockTimeout, is_locked, read_holder

HOLD = """
import sys, time
sys.path.insert(0, {root!r})
from adsbee_hil.lock import DeviceLock
with DeviceLock("SER1", {d!r}, purpose="holder"):
    print("locked", flush=True)
    time.sleep(30)
"""


def test_acquire_release(lock_dir):
    lk = DeviceLock("SER/1", lock_dir, timeout=1, purpose="test")
    with lk:
        assert is_locked(lk.path)
        assert "test" in read_holder(lk.path)
        assert lk.path.endswith("SER_1.lock")
    assert not is_locked(lk.path)


def test_reentrant_same_thread(lock_dir):
    a = DeviceLock("S", lock_dir, timeout=1)
    with a:
        with DeviceLock("S", lock_dir, timeout=1):
            pass
        assert is_locked(a.path)  # inner release doesn't drop the outer hold
    assert not is_locked(a.path)


def test_other_thread_waits(lock_dir):
    got = []

    def other():
        try:
            with DeviceLock("S", lock_dir, timeout=0.3):
                got.append("acquired")
        except LockTimeout:
            got.append("timeout")

    with DeviceLock("S", lock_dir, timeout=1):
        t = threading.Thread(target=other)
        t.start()
        t.join()
    assert got == ["timeout"]
    t = threading.Thread(target=other)
    t.start()
    t.join()
    assert got == ["timeout", "acquired"]


def test_other_process(lock_dir):
    import os

    root = os.path.join(os.path.dirname(__file__), "..")
    p = subprocess.Popen([sys.executable, "-c", HOLD.format(root=root, d=lock_dir)], stdout=subprocess.PIPE,
                         text=True)
    try:
        assert p.stdout.readline().strip() == "locked"
        t0 = time.monotonic()
        with pytest.raises(LockTimeout, match="holder"):
            DeviceLock("SER1", lock_dir, timeout=0.5).acquire()
        assert time.monotonic() - t0 >= 0.5
        with DeviceLock("OTHER", lock_dir, timeout=0.5):  # different device: not blocked
            pass
    finally:
        p.kill()
        p.wait()
    # The kernel drops the lock with the process: no stale locks.
    with DeviceLock("SER1", lock_dir, timeout=2):
        pass
