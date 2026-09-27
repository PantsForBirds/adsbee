"""Per-device locks so several jobs (CI runners, humans, parallel workers) can share one bench.

A lock is an ``flock`` on ``<lock_dir>/<key>.lock``, where the key is the device's USB serial (or
another stable identity for non-USB instruments). flock locks are released by the kernel when the
holder exits or crashes, so there are no stale locks to clean up. The lock file holds a line
describing the current holder, for ``adsbee-hil locks``.

Everything that talks to a device must hold its lock: the tools in this package take it
automatically; external scripts can use ``adsbee-hil lock <device> -- <command>``.
"""

import errno
import fcntl
import json
import os
import re
import socket
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

DEFAULT_LOCK_DIR = "/tmp/adsbee-hil-locks"


class LockTimeout(RuntimeError):
    pass


def lock_dir(configured: Optional[str] = None) -> str:
    return os.environ.get("ADSBEE_HIL_LOCK_DIR") or configured or DEFAULT_LOCK_DIR


def _safe_key(key: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]", "_", key)


class DeviceLock:
    """Exclusive lock on one device, waiting up to ``timeout`` seconds for it.

    Reentrant within one thread (a flash that runs AT queries takes the same lock again); other
    threads and processes wait. flock conflicts between separate open() calls even in the same
    process, so two threads of one process can't both hold a device either.
    """

    _held: Dict[Tuple[str, int], List[int]] = {}  # (path, thread id) -> [fd, depth]
    _held_mutex = threading.Lock()

    def __init__(self, key: str, directory: Optional[str] = None, timeout: float = 600.0,
                 purpose: str = "", label: Optional[str] = None):
        self.key = key
        self.label = label or key  # For messages, e.g. the bench id.
        self.directory = lock_dir(directory)
        self.path = os.path.join(self.directory, _safe_key(key) + ".lock")
        self.timeout = timeout
        self.purpose = purpose or " ".join(os.path.basename(a) for a in sys.argv[:3])

    def _slot(self) -> Tuple[str, int]:
        return (self.path, threading.get_ident())

    def acquire(self) -> "DeviceLock":
        with DeviceLock._held_mutex:
            held = DeviceLock._held.get(self._slot())
            if held:
                held[1] += 1
                return self
        os.makedirs(self.directory, exist_ok=True)
        try:
            os.chmod(self.directory, 0o1777)  # shared between users (CI runner, humans)
        except OSError:
            pass
        fd = os.open(self.path, os.O_RDWR | os.O_CREAT, 0o666)
        deadline = time.monotonic() + self.timeout
        announced = False
        while True:
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except OSError as e:
                if e.errno not in (errno.EAGAIN, errno.EACCES):
                    os.close(fd)
                    raise
            if time.monotonic() >= deadline:
                holder = read_holder(self.path)
                os.close(fd)
                raise LockTimeout(f"{self.label}: still locked after {self.timeout:.0f} s by {holder or 'unknown'}")
            if not announced:
                print(f"[{self.label}] waiting for lock held by {read_holder(self.path) or 'unknown'} ...",
                      file=sys.stderr, flush=True)
                announced = True
            time.sleep(0.5)
        info = {"pid": os.getpid(), "host": socket.gethostname(), "since": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "purpose": self.purpose}
        os.ftruncate(fd, 0)
        os.pwrite(fd, (json.dumps(info) + "\n").encode(), 0)
        with DeviceLock._held_mutex:
            DeviceLock._held[self._slot()] = [fd, 1]
        return self

    def release(self) -> None:
        with DeviceLock._held_mutex:
            held = DeviceLock._held.get(self._slot())
            if not held:
                return
            held[1] -= 1
            if held[1] > 0:
                return
            del DeviceLock._held[self._slot()]
        fd = held[0]
        try:
            os.ftruncate(fd, 0)
        except OSError:
            pass
        fcntl.flock(fd, fcntl.LOCK_UN)
        os.close(fd)

    def __enter__(self) -> "DeviceLock":
        return self.acquire()

    def __exit__(self, *exc) -> None:
        self.release()


def read_holder(path: str) -> Optional[str]:
    try:
        with open(path) as f:
            text = f.read().strip()
    except OSError:
        return None
    if not text:
        return None
    try:
        info = json.loads(text)
        return f"pid {info.get('pid')} on {info.get('host')} since {info.get('since')} ({info.get('purpose')})"
    except ValueError:
        return text


def is_locked(path: str) -> bool:
    try:
        fd = os.open(path, os.O_RDONLY)
    except OSError:
        return False
    try:
        fcntl.flock(fd, fcntl.LOCK_SH | fcntl.LOCK_NB)
        fcntl.flock(fd, fcntl.LOCK_UN)
        return False
    except OSError:
        return True
    finally:
        os.close(fd)
