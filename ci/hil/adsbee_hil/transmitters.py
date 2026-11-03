"""Test transmitters: instruments that put known packets on the air (or on a cable) for receivers.

``transmit()`` plays a finite ``TestPattern`` to completion; ``start()`` / ``stop()`` run a
continuous one. Types: ``bee_wiggler`` (Bee Wiggler, Pluto Wiggler or Bee Waggler over their USB AT console)
and ``pluto`` (ADALM-Pluto SDR, untested, no UAT); others register through the ``adsbee_hil.transmitters`` entry-point group.

Keep receivers on attenuated coax or behind attenuators: full output can damage a receiver, and
radiating on 1090/978 MHz outside a shielded setup is illegal in most places.
"""

import contextlib
import math
import re
import time
from dataclasses import dataclass, field
from typing import Dict, Iterator, List, Optional, Tuple

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


#: Command sets of the wiggler family (AT+DEVICE_INFO? tells them apart).
CMD_SET_1 = "1"  # Bee Wiggler (RP2040) and pwiggd 1.x (Pluto Wiggler, Bee Waggler): MODE_S_TX, DUAL_TABLE_TX, ...
CMD_SET_2 = "2"  # pwiggd 2.x: AT+TX jobs, AT+LEVEL, AT+STOP (projects/adsbee/firmware/pluto_wiggler/AT.md)

#: Reference level (dBm at 0 dB attenuation) a pwiggd unit uses for the Bee Wiggler commands: MODE_S_PWR and
#: PW_UAT_PWR boot at +5 dBm and this tool never changes them. On 2.x the same bench numbers give the same level:
#: ``power_dbm`` (or this) minus the Bee-quantized attenuation, in dBm at the SMA.
PWIGGD_REF_DBM = 5.0

_FW_VERSION = re.compile(r"Firmware Version:\s*(\d+)\.(\d+)\.(\d+)")
_PRODUCT_2X = re.compile(r"^\s*Product:\s*(Bee Waggler|Pluto Wiggler)\b", re.MULTILINE)


def command_set(device_info: str) -> str:
    """CMD_SET_2 for a pwiggd 2.x unit (a ``Product:`` line, or no RP2040 line and firmware >= 2), else CMD_SET_1."""
    if _PRODUCT_2X.search(device_info):
        return CMD_SET_2
    m = _FW_VERSION.search(device_info)
    if m and "RP2040" not in device_info and int(m.group(1)) >= 2:
        return CMD_SET_2
    return CMD_SET_1


def bee_quantize_atten(db: float) -> float:
    """The Bee Wiggler's attenuation as programmed: 4 PE4312 steps of 0.5 dB, 0-126 dB (pwiggd 1.x does the same)."""
    remaining = min(max(float(db), 0.0), 126.0)
    total = 0.0
    for left in (4, 3, 2, 1):
        step = int(min(max(remaining / left, 0.0), 31.5) / 0.5 + 0.5) * 0.5
        total += min(int(step / 0.5 + 0.5), 63) * 0.5
        remaining -= step
    return total


class BeeWiggler(Transmitter):
    """The wiggler family over its USB AT console: Bee Wiggler (RP2040), Pluto Wiggler (ADALM-Pluto) and Bee
    Waggler (ZynqSDR 7020). The command set follows the firmware.

    Bee Wiggler and pwiggd 1.x (command set 1):
      AT+MODE_S_ATTEN=<dB>                   Mode S (1090 MHz) step attenuator
      AT+SUBG_ATTEN=<dB>                     UAT (978 MHz) step attenuator
      AT+MODE_S_TABLE_TX=<n>,<rate>,<dBm>    built-in Mode S table; n=-1 until a char arrives
      AT+MODE_S_TX=<hex>,<repeats>,<rate>      repeats: 0 = once, -1 = until a char arrives
      AT+UAT_ADSB_TABLE_TX=<n>,<rate>        built-in UAT ADS-B table
      AT+DUAL_TABLE_TX=<seconds>,<mode_s_rate>,<uat_rate>,ADSB,<dBm>
    A running transmission ends with OK; any received character stops an unbounded one.

    pwiggd 2.x (command set 2, AT.md): every transmission is a job, ``AT+TX=<port>,<proto>,…`` answers
    ``+TX:job=<id>`` at once and ends with ``+DONE:<id>,…``; ``AT+STOP`` ends unbounded ones. Levels are absolute
    dBm at the SMA: ``power_dbm`` (default PWIGGD_REF_DBM) minus the Bee-quantized attenuation, so a bench file
    gives the same level as on 1.x. Several explicit Mode S messages play as an ``AT+SCHED`` file. ``dual`` runs
    Mode S on port 1 and UAT on port 2 (port 1 on a Pluto Wiggler), started together with ``AT+HOLD``, time-shared
    unless the pattern needs overlap (rate 0) or ``simultaneous = true``: then a Bee Waggler opts in to the fixed-LO
    plan (``AT+SIMULTANEOUS=MODE_S,UAT,ACCEPT_COSTS``, about 25 dB less maximum level) for that run only.

    Bench file keys: usb_serial (required), mode_s_atten_db / subg_atten_db (applied when the pattern doesn't set
    atten_db); 2.x only: mode_s_port (default 1), uat_port (default 2 on a two-port unit), simultaneous (default
    false), command_set ("auto", "1" or "2"; default auto).
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
        self._cmd_set: Optional[str] = None
        self._ports = 1
        self._jobs: Dict[str, Optional[str]] = {}  # 2.x: job id -> its +DONE line (None while running)
        self._buf = ""
        self._simultaneous = False

    def _device(self) -> Optional[usb.UsbDevice]:
        return usb.find_by_serial(self.cfg.usb_serial)

    def _port(self) -> Optional[str]:
        d = self._device()
        if d is None:
            return None
        if not d.is_rp2040 and len(d.by_id) + len(d.ttys) > 1:
            # A pwiggd unit's AT console is its LAST CDC-ACM port (the first is the stock Pluto console).
            return d.by_id[-1] if d.by_id else f"/dev/{d.ttys[-1]}"
        return d.console

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

    # ── command-set selection ───────────────────────────────────────────────

    def _detect(self, console: AtConsole) -> str:
        """Command set of the attached unit, cached. An RP2040 (Bee Wiggler) needs no query."""
        forced = str(self.cfg.options.get("command_set", "auto"))
        if forced in (CMD_SET_1, CMD_SET_2):
            self._cmd_set = forced
        if self._cmd_set is None:
            d = self._device()
            if d is not None and d.is_rp2040:
                self._cmd_set = CMD_SET_1
            else:
                text = console.transact("AT+DEVICE_INFO?", 3.0)
                self._cmd_set = command_set(text)
                m = re.search(r"^\s*Ports:\s*(\d+)", text, re.MULTILINE)
                self._ports = int(m.group(1)) if m else 1
        return self._cmd_set

    # ── command set 1 ───────────────────────────────────────────────────────

    def commands_for(self, p: TestPattern) -> List[str]:
        """Command set 1 (Bee Wiggler, pwiggd 1.x): AT commands that play pattern p (the last one transmits)."""
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
                    raise TransmitterError("Bee Wiggler plays one explicit Mode S message per command; "
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
                raise TransmitterError("Bee Wiggler UAT commands take no power argument; use atten_db")
            cmds.append(f"AT+UAT_ADSB_TABLE_TX={p.count},{rate}")
        elif p.band == "dual":
            if p.messages:
                raise TransmitterError("dual-band patterns use the Bee Wiggler's tables")
            seconds = -1 if p.count < 0 else max(1, math.ceil(p.duration()))
            cmds.append(",".join([f"AT+DUAL_TABLE_TX={seconds}", rate, rate, "ADSB"] + power))
        else:
            raise TransmitterError(f"unknown band {p.band!r}")
        return cmds

    # ── command set 2 (pwiggd 2.x) ──────────────────────────────────────────

    def _level_v2(self, p: TestPattern, atten_key: str) -> List[str]:
        """The ``,<dBm>`` argument for a 2.x TX command, or nothing (the unit's AT+LEVEL default)."""
        atten = p.atten_db if p.atten_db is not None else self.cfg.options.get(atten_key)
        if atten is None and p.power_dbm is None:
            return []
        ref = PWIGGD_REF_DBM if p.power_dbm is None else float(p.power_dbm)
        return [_num(ref - bee_quantize_atten(atten or 0.0))]

    def _count_v2(self, n: int) -> str:
        return "-1" if n < 0 else str(n)

    def v2_ports(self) -> Tuple[int, int]:
        """(Mode S port, UAT port) on a 2.x unit."""
        ms = int(self.cfg.options.get("mode_s_port", 1))
        uat = int(self.cfg.options.get("uat_port", 2 if self._ports >= 2 else 1))
        return ms, uat

    def commands_for_v2(self, p: TestPattern, ports: Optional[int] = None) -> List[str]:
        """Command set 2 (pwiggd 2.x): AT commands that play pattern p. Each AT+TX line starts a job."""
        if ports is not None:
            self._ports = ports
        ms_port, uat_port = self.v2_ports()
        rate = _num(p.rate)
        cmds: List[str] = []
        if p.band == "1090":
            level = self._level_v2(p, "mode_s_atten_db")
            msgs = p.messages
            if len(msgs) > 1:
                # Several messages, cycled: an in-memory schedule played `loops` times.
                if p.rate <= 0:
                    raise TransmitterError("several explicit messages need a rate > 0")
                if len(msgs) > 256:
                    raise TransmitterError("AT+SCHED holds at most 256 messages")
                dbm = level[0] if level else _num(PWIGGD_REF_DBM)
                cmds.append("AT+SCHED=CLEAR")
                for i, m in enumerate(msgs):
                    cmds.append(f"AT+SCHED=ADD,{ms_port},MODE_S,{round(i * 1e6 / p.rate)},{dbm},{m}")
                loops = -1 if p.count < 0 else max(1, math.ceil(p.count / len(msgs)))
                cmds.append(f"AT+TX={ms_port},FILE,sched,{loops}")
            else:
                payload = msgs[0] if msgs else "TABLE"
                cmds.append(",".join([f"AT+TX={ms_port}", "MODE_S", payload, self._count_v2(p.count), rate] + level))
        elif p.band == "978":
            if p.messages:
                raise TransmitterError("explicit UAT messages are not supported yet")
            cmds.append(",".join([f"AT+TX={uat_port}", "UAT", "TABLE", self._count_v2(p.count), rate]
                                 + self._level_v2(p, "subg_atten_db")))
        elif p.band == "dual":
            if p.messages:
                raise TransmitterError("dual-band patterns use the wiggler's tables")
            want_simul = bool(self.cfg.options.get("simultaneous", False)) or p.rate <= 0
            if want_simul and self._ports >= 2:
                cmds.append("AT+SIMULTANEOUS=MODE_S,UAT,ACCEPT_COSTS")
            cmds.append("AT+HOLD=1")
            cmds.append(",".join([f"AT+TX={ms_port}", "MODE_S", "TABLE", self._count_v2(p.count), rate]
                                 + self._level_v2(p, "mode_s_atten_db")))
            cmds.append(",".join([f"AT+TX={uat_port}", "UAT", "TABLE", self._count_v2(p.count), rate]
                                 + self._level_v2(p, "subg_atten_db")))
            cmds.append("AT+HOLD=0")
        else:
            raise TransmitterError(f"unknown band {p.band!r}")
        return cmds

    def _note(self, line: str) -> bool:
        """Records an unsolicited line; True if it was one (+DONE, +CONTROL:lost)."""
        m = re.match(r"\+DONE:(\d+),", line)
        if m:
            self._jobs[m.group(1)] = line
            return True
        return line.startswith("+CONTROL:lost")

    def _lines(self, deadline: float) -> Iterator[str]:
        while time.monotonic() < deadline:
            while "\n" in self._buf:
                line, self._buf = self._buf.split("\n", 1)
                line = line.strip()
                if line:
                    yield line
            self._buf += self._console.read().decode(errors="replace")

    def _cmd_v2(self, cmd: str, timeout: float = 5.0) -> List[str]:
        """One 2.x command on the open console: its reply lines, unsolicited lines set aside."""
        self._console.write((cmd + "\r\n").encode())
        out: List[str] = []
        for line in self._lines(time.monotonic() + timeout):
            if self._note(line):
                continue
            if line == "OK":
                m = next((re.match(r"\+TX:job=(\d+)", x) for x in out if x.startswith("+TX:job=")), None)
                if m:
                    self._jobs.setdefault(m.group(1), None)
                return out
            if line.startswith("ERROR"):
                raise TransmitterError(f"{self.id}: {cmd} -> {line}")
            out.append(line)
        raise TransmitterError(f"{self.id}: {cmd}: no OK within {timeout:g} s")

    def _start_v2(self, pattern: TestPattern) -> None:
        self._jobs, self._buf, self._simultaneous = {}, "", False
        cmds = self.commands_for_v2(pattern)
        self._cmd_v2("AT+CONTROL=TAKE")  # The bench lock makes this tool the unit's only user.
        self._cmd_v2("AT+STOP=ALL")  # A job left running by a crashed run.
        try:
            for cmd in cmds:
                self._cmd_v2(cmd, 10.0 if cmd.startswith("AT+SIMULTANEOUS") else 5.0)
                if cmd.startswith("AT+SIMULTANEOUS="):
                    self._simultaneous = True
        except Exception:
            if "AT+HOLD=1" in cmds:
                with contextlib.suppress(Exception):
                    self._cmd_v2("AT+STOP=ALL")
                    self._cmd_v2("AT+HOLD=0")
            self._leave_simultaneous()
            raise

    def _wait_v2(self, deadline: float) -> None:
        pending = [j for j, d in self._jobs.items() if d is None]
        for line in self._lines(deadline):
            self._note(line)
            pending = [j for j, d in self._jobs.items() if d is None]
            if not pending:
                break
        if pending:
            raise TransmitterError(f"{self.id}: transmission did not finish within the timeout")
        self._out = "\n".join(d for d in self._jobs.values() if d)
        bad = [d for d in self._jobs.values() if d and not re.search(r"reason=(done|stopped)\b", d)]
        if bad:
            raise TransmitterError(f"{self.id}: {'; '.join(bad)}")

    def _leave_simultaneous(self) -> None:
        if self._simultaneous and self._console is not None:
            self._simultaneous = False
            with contextlib.suppress(Exception):
                self._cmd_v2("AT+SIMULTANEOUS=OFF", 10.0)

    # ── common ──────────────────────────────────────────────────────────────

    def start(self, pattern: TestPattern) -> None:
        if self._active is not None:
            raise TransmitterError(f"{self.id}: already transmitting")
        self._lock = self.lock("tx").acquire()
        try:
            self._console = self._open()
            if self._detect(self._console) == CMD_SET_2:
                self._start_v2(pattern)
            else:
                cmds = self.commands_for(pattern)
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
        if self._cmd_set == CMD_SET_2:
            try:
                self._wait_v2(deadline)
            finally:
                self._active = None
                self._leave_simultaneous()
                self._cleanup()
            return
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
            if self._cmd_set == CMD_SET_2:
                self._cmd_v2("AT+STOP=ALL")  # Ends after the packet on air; each job then sends +DONE.
            else:
                self._console.write(b"x")  # Any character ends the transmission.
            self.wait(5)
        finally:
            self._active = None
            self._leave_simultaneous()
            self._cleanup()

    def _cleanup(self) -> None:
        if self._console:
            self._console.close()
            self._console = None
        if self._lock:
            self._lock.release()
            self._lock = None


class PlutoSdr(Transmitter):
    """ADALM-Pluto (or compatible) SDR playing a generated 1090 MHz Mode S waveform. UNTESTED on hardware.

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
