"""AT console transport over a USB CDC serial port (pyserial)."""

import re
import termios
import threading
import time
from typing import Callable, List, Optional

# AT+DEVICE_INFO? prints per-device OTA keys. Keep them out of CI logs and tickets.
_SECRET_LINE = re.compile(r"^(\s*OTA Key \d+:).*$", re.MULTILINE)


def redact(text: str) -> str:
    return _SECRET_LINE.sub(r"\1 <redacted>", text)


class AtError(RuntimeError):
    pass


def _done(text: str) -> bool:
    # Commands end with a line "OK" or an "ERROR..." line. Streaming commands (RX_CW) print
    # "Press any key" first and keep going until a byte arrives, so they are never "done".
    if "Press any key" in text:
        return False
    return bool(re.search(r"(^|\n)\s*OK\s*($|\r|\n)", text)) or "ERROR" in text


class AtConsole:
    """One open AT console.

    ``keep_lines`` clears HUPCL so DTR/RTS stay asserted after close. Some fixtures wire the
    modem-control lines to the target (the ADSBee 1421 programmer jig maps RTS to the module's
    SYNC pin and a DTR edge to a reset), and dropping them on close would reset or sleep it.
    """

    def __init__(self, port: str, baud: int = 115200, keep_lines: bool = False):
        self.port = port
        self.baud = baud
        self.keep_lines = keep_lines
        self._s = None

    def open(self) -> "AtConsole":
        import serial  # pyserial

        s = serial.Serial()
        s.port = self.port
        s.baudrate = self.baud
        s.timeout = 0.05
        # Linux raises DTR and RTS on open anyway; asking for the same state avoids a toggle later.
        s.dtr = True
        s.rts = True
        s.open()
        if self.keep_lines:
            attrs = termios.tcgetattr(s.fd)
            attrs[2] &= ~termios.HUPCL
            termios.tcsetattr(s.fd, termios.TCSANOW, attrs)
        self._s = s
        return self

    def close(self) -> None:
        if self._s is not None:
            self._s.close()
            self._s = None

    def __enter__(self) -> "AtConsole":
        return self.open() if self._s is None else self

    def __exit__(self, *exc) -> None:
        self.close()

    def write(self, data: bytes) -> None:
        self._s.write(data)
        self._s.flush()

    def read(self, n: int = 4096) -> bytes:
        return self._s.read(n)

    def drain(self, seconds: float = 0.1) -> bytes:
        out = b""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            out += self._s.read(4096)
        return out

    def transact(self, cmd: str, timeout: float = 3.0, quiet: float = 0.5) -> str:
        """Sends one command; returns output up to OK/ERROR, or what arrived before a timeout.

        Returns early after ``quiet`` seconds of silence once some output arrived, for commands
        that don't end with OK.
        """
        self._s.reset_input_buffer()
        self.write((cmd.strip() + "\r\n").encode())
        out = b""
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            d = self._s.read(4096)
            if d:
                out += d
                last = time.monotonic()
                if _done(out.decode(errors="replace")):
                    time.sleep(0.05)
                    out += self._s.read(4096)
                    break
            elif last is not None and time.monotonic() - last > quiet and \
                    "Press any key" not in out.decode(errors="replace"):
                break
        return out.decode(errors="replace")

    def interrupt(self, timeout: float = 3.0) -> str:
        """Stops a streaming command (anything that waits for a key) and reads the tail."""
        self.write(b"x")
        out = b""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            out += self._s.read(4096)
            if _done(out.decode(errors="replace").split("Press any key", 1)[-1]):
                break
        return out.decode(errors="replace")


class LineReader:
    """Collects console lines on a background thread (e.g. packet reports) until stopped."""

    def __init__(self, console: AtConsole, on_line: Optional[Callable[[str], None]] = None):
        self.console = console
        self.lines: List[str] = []
        self._on_line = on_line
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _run(self) -> None:
        buf = b""
        while not self._stop.is_set():
            buf += self.console.read(4096)
            *complete, buf = buf.split(b"\n")
            for raw in complete:
                line = raw.decode(errors="replace").strip()
                if line:
                    self.lines.append(line)
                    if self._on_line:
                        self._on_line(line)

    def start(self) -> "LineReader":
        self._thread.start()
        return self

    def stop(self) -> List[str]:
        self._stop.set()
        self._thread.join(timeout=2)
        return self.lines
