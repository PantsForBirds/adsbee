"""Test transmitters: instruments that put known packets on the air (or on a cable) for receivers.

A transmitter plays a ``TestPattern``: a band, a number of packets (or -1 for "until stopped"),
a rate, and an output level. ``transmit()`` plays a finite pattern to completion; ``start()`` /
``stop()`` run a continuous one around some other activity.

Implementations here:

* ``bee_wiggler``: PantsForBirds' Mode S / UAT packet generator, driven over its USB AT console.
  Only the documented AT commands are used; its firmware is not part of this repository.
* ``pluto``: an ADALM Pluto(+) SDR playing a generated 1090 MHz PPM waveform (pyadi-iio).
  UNTESTED on hardware so far; UAT is not implemented.

Other types can be added through the ``adsbee_hil.transmitters`` entry-point group.

Cabling note: keep receivers on attenuated coax (or at a safe distance behind attenuators). A
wiggler's or Pluto's full output into a receiver's front end can damage it, and radiating on
1090/978 MHz outside a shielded setup is not allowed in most places.
"""

import contextlib
import math
import re
import time
from dataclasses import dataclass, field
from typing import Dict, Iterator, List, Optional

from . import usb
from .config import Bench, TransmitterConfig
from .console import AtConsole
from .lock import DeviceLock
from .packets import unique_df17_set


class TransmitterError(RuntimeError):
    pass


@dataclass
class TestPattern:
    """What to transmit.

    band: "1090" (Mode S), "978" (UAT ADS-B) or "dual" (both, where supported).
    count: packets to send per band; -1 = until stop().
    rate: packets per second per band (0 = as fast as the instrument can).
    power_dbm: requested output power, if the instrument supports it.
    atten_db: instrument step-attenuator setting, if it has one.
    messages: explicit hex messages (cycled). Empty = the instrument's built-in table, or
        ``unique_df17_set(unique)`` for instruments without one.
    unique: number of distinct messages for generated patterns.
    """

    __test__ = False  # Not a pytest test class.

    band: str = "1090"
    count: int = 100
    rate: float = 10.0
    power_dbm: Optional[float] = None
    atten_db: Optional[float] = None
    messages: List[str] = field(default_factory=list)
    unique: int = 10

    def duration(self) -> float:
        if self.count < 0:
            return math.inf
        return self.count / self.rate if self.rate > 0 else 0.0

    def resolved_messages(self) -> List[str]:
        return self.messages or unique_df17_set(self.unique)


PATTERNS: Dict[str, TestPattern] = {
    # Short, gentle bursts for a quick loopback check.
    "mode_s_table_100": TestPattern(band="1090", count=100, rate=20),
    "uat_table_100": TestPattern(band="978", count=100, rate=20),
    "dual_10s": TestPattern(band="dual", count=100, rate=10),
    "df17_unique_10": TestPattern(band="1090", count=100, rate=20, unique=10,
                                  messages=unique_df17_set(10)),
}


class Transmitter:
    type = "generic"

    def __init__(self, cfg: TransmitterConfig, bench: Optional[Bench] = None, lock_timeout: float = 600.0):
        self.cfg = cfg
        self.bench = bench or Bench()
        self.lock_timeout = lock_timeout
        self._active: Optional[TestPattern] = None

    @property
    def id(self) -> str:
        return self.cfg.id

    def lock(self, purpose: str = "") -> DeviceLock:
        return DeviceLock(self.cfg.usb_serial or f"tx-{self.cfg.id}", self.bench.lock_dir, self.lock_timeout,
                          purpose, self.id)

    def available(self) -> bool:
        """True if the instrument is attached and reachable (cheap, no side effects)."""
        raise NotImplementedError

    def info(self) -> Dict[str, str]:
        return {}

    def start(self, pattern: TestPattern) -> None:
        raise NotImplementedError

    def wait(self, timeout: Optional[float] = None) -> None:
        """Blocks until a finite pattern has been sent."""
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    def transmit(self, pattern: TestPattern) -> None:
        if pattern.count < 0:
            raise TransmitterError("transmit() needs a finite pattern; use start()/stop()")
        self.start(pattern)
        try:
            self.wait(pattern.duration() * 1.5 + 10)
        finally:
            if self._active is not None:
                self.stop()

    @contextlib.contextmanager
    def running(self, pattern: TestPattern) -> Iterator["Transmitter"]:
        self.start(pattern)
        try:
            yield self
        finally:
            self.stop()

    def close(self) -> None:
        pass


def _num(v: Optional[float]) -> str:
    return str(int(v)) if v is not None and float(v).is_integer() else str(v)


class BeeWiggler(Transmitter):
    """bee_wiggler over its USB AT console.

    Commands used (arguments as listed by the wiggler's AT+HELP):
      AT+DEVICE_INFO?
      AT+MODE_S_ATTEN=<dB>                   1090 MHz step attenuator
      AT+SUBG_ATTEN=<dB>                     978 MHz step attenuator
      AT+MODE_S_TABLE_TX=<n>,<rate>,<dBm>    built-in Mode S table; n=-1 until a char arrives
      AT+MODE_S_TX=<hex>,<repeats>,<rate>      repeats: 0 = once, -1 = until a char arrives
      AT+UAT_ADSB_TABLE_TX=<n>,<rate>        built-in UAT ADS-B table
      AT+DUAL_TABLE_TX=<seconds>,<mode_s_rate>,<uat_rate>,ADSB,<dBm>
    A running transmission ends with OK; any received character stops an unbounded one.

    Bench file keys: usb_serial (required), mode_s_atten_db / subg_atten_db (applied on start
    when the pattern doesn't set atten_db).
    """

    type = "bee_wiggler"
    baud = 115200

    def __init__(self, cfg: TransmitterConfig, bench: Optional[Bench] = None, lock_timeout: float = 600.0):
        super().__init__(cfg, bench, lock_timeout)
        if not cfg.usb_serial:
            raise TransmitterError(f"{cfg.id}: bee_wiggler needs usb_serial in the bench file")
        self._console: Optional[AtConsole] = None
        self._lock: Optional[DeviceLock] = None
        self._out = ""

    def _port(self) -> Optional[str]:
        d = usb.find_by_serial(self.cfg.usb_serial)
        return d.console if d else None

    def available(self) -> bool:
        return self._port() is not None

    def _open(self) -> AtConsole:
        port = self._port()
        if not port:
            raise TransmitterError(f"{self.id}: wiggler {self.cfg.usb_serial} is not attached")
        return AtConsole(port, self.baud).open()

    def at(self, cmd: str, timeout: float = 3.0) -> str:
        with self.lock("at"):
            with self._open() as c:
                return c.transact(cmd, timeout)

    def info(self) -> Dict[str, str]:
        from .receivers import parse_key_values

        return parse_key_values(self.at("AT+DEVICE_INFO?"))

    def commands_for(self, p: TestPattern) -> List[str]:
        """AT commands that play pattern p (the last one transmits)."""
        cmds = []
        atten = p.atten_db
        if p.band in ("1090", "dual"):
            a = atten if atten is not None else self.cfg.options.get("mode_s_atten_db")
            if a is not None:
                cmds.append(f"AT+MODE_S_ATTEN={_num(a)}")
        if p.band in ("978", "dual"):
            a = atten if atten is not None else self.cfg.options.get("subg_atten_db")
            if a is not None:
                cmds.append(f"AT+SUBG_ATTEN={_num(a)}")
        rate = _num(p.rate)
        power = [] if p.power_dbm is None else [_num(p.power_dbm)]
        if p.band == "1090":
            if p.messages:
                if len(p.messages) != 1:
                    raise TransmitterError("bee_wiggler plays one explicit Mode S message per command; "
                                           "use its table or a single message")
                if p.power_dbm is not None:
                    raise TransmitterError("single-message Mode S takes no power argument; use atten_db")
                repeats = -1 if p.count < 0 else max(p.count - 1, 0)
                cmds.append(f"AT+MODE_S_TX={p.messages[0]},{repeats},{rate}")
            else:
                cmds.append(",".join([f"AT+MODE_S_TABLE_TX={p.count}", rate] + power))
        elif p.band == "978":
            if p.messages:
                raise TransmitterError("explicit UAT messages are not supported yet")
            if p.power_dbm is not None:
                raise TransmitterError("bee_wiggler's UAT commands take no power argument; use atten_db")
            cmds.append(f"AT+UAT_ADSB_TABLE_TX={p.count},{rate}")
        elif p.band == "dual":
            if p.messages:
                raise TransmitterError("dual-band patterns use the wiggler's tables")
            seconds = -1 if p.count < 0 else max(1, math.ceil(p.duration()))
            cmds.append(",".join([f"AT+DUAL_TABLE_TX={seconds}", rate, rate, "ADSB"] + power))
        else:
            raise TransmitterError(f"unknown band {p.band!r}")
        return cmds

    def start(self, pattern: TestPattern) -> None:
        if self._active is not None:
            raise TransmitterError(f"{self.id}: already transmitting")
        cmds = self.commands_for(pattern)
        self._lock = self.lock("tx").acquire()
        try:
            self._console = self._open()
            for cmd in cmds[:-1]:
                out = self._console.transact(cmd, 3.0)
                if "ERROR" in out or "OK" not in out:
                    raise TransmitterError(f"{self.id}: {cmd} -> {out.strip()!r}")
            self._console.write((cmds[-1] + "\r\n").encode())
            self._out = ""
            self._active = pattern
        except Exception:
            self._cleanup()
            raise

    def wait(self, timeout: Optional[float] = None) -> None:
        if self._active is None:
            return
        deadline = time.monotonic() + (timeout if timeout is not None else self._active.duration() * 1.5 + 10)
        while time.monotonic() < deadline:
            self._out += self._console.read().decode(errors="replace")
            if re.search(r"(^|\n)\s*OK\s*(\r|\n|$)", self._out) or "ERROR" in self._out:
                break
        else:
            raise TransmitterError(f"{self.id}: transmission did not finish within the timeout")
        out, self._active = self._out, None
        self._cleanup()
        if "ERROR" in out:
            raise TransmitterError(f"{self.id}: {out.strip()!r}")

    def stop(self) -> None:
        if self._active is None:
            return
        try:
            self._console.write(b"x")  # Any character ends the transmission.
            self.wait(5)
        finally:
            self._active = None
            self._cleanup()

    def _cleanup(self) -> None:
        if self._console:
            self._console.close()
            self._console = None
        if self._lock:
            self._lock.release()
            self._lock = None


class PlutoSdr(Transmitter):
    """ADALM Pluto(+) SDR playing a generated 1090 MHz Mode S waveform. UNTESTED on hardware.

    Bench file keys: uri (e.g. "ip:192.168.2.1" or "usb:1.2.5"), tx_gain_db (<= 0, default -40),
    sample_rate (default 4 MS/s, i.e. 8 samples per 1 us bit), center_hz (default 1090e6).
    Needs ``pip install pyadi-iio numpy`` (the "pluto" extra).

    TODO(pluto):
      * verify output level/spectrum on a spectrum analyzer and calibrate tx_gain_db -> dBm;
      * UAT (978 MHz, 1.041667 Mbit/s CPFSK) waveform generation;
      * precise packet counts: cyclic playback loops the buffer, so the count is approximate
        (stop() after duration); a non-cyclic push of exactly N buffers would be exact.
    """

    type = "pluto"

    def __init__(self, cfg: TransmitterConfig, bench: Optional[Bench] = None, lock_timeout: float = 600.0):
        super().__init__(cfg, bench, lock_timeout)
        self.uri = cfg.options.get("uri", "ip:192.168.2.1")
        self.sample_rate = int(cfg.options.get("sample_rate", 4_000_000))
        self.center_hz = int(cfg.options.get("center_hz", 1_090_000_000))
        self.tx_gain_db = float(cfg.options.get("tx_gain_db", -40))
        self._sdr = None
        self._lock: Optional[DeviceLock] = None
        self._t_end = 0.0

    def available(self) -> bool:
        try:
            import adi  # noqa: F401
        except ImportError:
            return False
        try:
            import iio

            iio.Context(self.uri)
            return True
        except Exception:
            return False

    def start(self, pattern: TestPattern) -> None:
        if pattern.band != "1090":
            raise TransmitterError("pluto: only band 1090 is implemented (TODO: UAT)")
        try:
            import adi
            import numpy as np
        except ImportError as e:
            raise TransmitterError("pluto needs pyadi-iio and numpy (pip install 'adsbee-hil[pluto]')") from e
        rate = pattern.rate if pattern.rate > 0 else 1000.0
        iq = mode_s_iq(pattern.resolved_messages(), self.sample_rate, 1.0 / rate)
        self._lock = self.lock("tx").acquire()
        try:
            sdr = adi.Pluto(self.uri)
            sdr.sample_rate = self.sample_rate
            sdr.tx_lo = self.center_hz
            sdr.tx_rf_bandwidth = self.sample_rate
            sdr.tx_hardwaregain_chan0 = min(self.tx_gain_db, 0.0)
            sdr.tx_cyclic_buffer = True
            sdr.tx((np.array(iq) * (2 ** 14)).astype(np.complex64))
            self._sdr = sdr
        except Exception:
            self._lock.release()
            self._lock = None
            raise
        self._active = pattern
        self._t_end = time.monotonic() + pattern.duration()

    def wait(self, timeout: Optional[float] = None) -> None:
        if self._active is None:
            return
        remaining = self._t_end - time.monotonic()
        if timeout is not None and remaining > timeout:
            raise TransmitterError(f"{self.id}: pattern needs {remaining:.0f} s, more than the timeout")
        if remaining > 0:
            time.sleep(remaining)
        self.stop()

    def stop(self) -> None:
        if self._sdr is not None:
            try:
                self._sdr.tx_destroy_buffer()
            finally:
                self._sdr = None
        self._active = None
        if self._lock:
            self._lock.release()
            self._lock = None


def mode_s_iq(messages: List[str], sample_rate: int, spacing_s: float) -> List[complex]:
    """Baseband IQ (amplitude 0/1) for Mode S messages, one every spacing_s seconds.

    Each message is the 8 us preamble (pulses at 0, 1, 3.5 and 4.5 us) followed by PPM data
    bits: a pulse in the first half of each 1 us bit for 1, in the second half for 0.
    """
    half = sample_rate / 2_000_000  # samples per 0.5 us
    if half < 1 or not float(half).is_integer():
        raise ValueError("sample_rate must be a multiple of 2 MS/s")
    half = int(half)
    slot = int(round(spacing_s * sample_rate))
    out: List[complex] = []
    for msg in messages:
        bits = bin(int(msg, 16))[2:].zfill(len(msg) * 4)
        chips = [0] * 16
        for p in (0, 2, 7, 9):  # preamble pulses in 0.5 us chips
            chips[p] = 1
        for b in bits:
            chips += [1, 0] if b == "1" else [0, 1]
        burst = [complex(c) for c in chips for _ in range(half)]
        if len(burst) > slot:
            raise ValueError("rate too high: messages overlap")
        out += burst + [0j] * (slot - len(burst))
    return out


BUILTIN_TYPES = {cls.type: cls for cls in (BeeWiggler, PlutoSdr)}
