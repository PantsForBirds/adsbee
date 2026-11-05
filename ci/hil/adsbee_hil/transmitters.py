"""Test transmitters: instruments that put known packets on the air (or on a cable) for receivers.

``transmit()`` plays a finite ``TestPattern`` to completion; ``start()`` / ``stop()`` run a
continuous one. Types: ``bee_wiggler`` (Bee Wiggler, Pluto Wiggler or Bee Waggler over their USB AT console)
and ``pluto`` (ADALM-Pluto SDR, untested, no UAT); others register through the ``adsbee_hil.transmitters`` entry-point group.

Keep receivers on attenuated coax or behind attenuators: full output can damage a receiver, and
radiating on 1090/978 MHz outside a shielded setup is illegal in most places.
"""

import contextlib
import hashlib
import json
import logging
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

log = logging.getLogger(__name__)

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

#: An ADSBee 1090U fed Mode S above about this level in the simultaneous plan misses frames unless its
#: AT+TL_OFFSET is raised (the plan's LO leaves its noise floor low; DESIGN-2 §5.3, t-1014).
SIMULTANEOUS_TL_DBM = -45.0

SCHED_MAX = 256  # AT+SCHED entries
FILE_CHUNK = 4096  # bytes per AT+FILE=DATA (8192 hex digits)
_MODE_S_HEX = (14, 28)
_UAT_HEX = (36, 68, 864, 60, 96, 1104)  # payload only (the unit adds the FEC) or payload + FEC

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


def split_messages(messages: List[str]) -> Tuple[List[str], List[str]]:
    """(Mode S, UAT) messages, told apart by length."""
    ms = [m for m in messages if len(m) in _MODE_S_HEX]
    uat = [m for m in messages if len(m) in _UAT_HEX]
    bad = [m for m in messages if m not in ms and m not in uat]
    if bad:
        raise TransmitterError(f"not a Mode S (14/28 hex digits) or UAT (36/68/864, or with FEC 60/96/1104) "
                               f"message: {bad[0]}")
    return ms, uat


def packet_file(proto: str, messages: List[str], count: int, rate: float,
                dbm: Optional[str]) -> Tuple[str, int]:
    """A waggler-packets/1 JSONL file (DESIGN-2 §6) of ``messages`` cycled at ``rate``, and its loop count.

    The header's period is the cycle's length, so loops stay evenly spaced: without one a loop starts right after
    the last packet and a receiver loses a packet per loop (DESIGN-2 §6.4). Without ``dbm`` packets play at the
    port's AT+LEVEL.
    """
    n = len(messages)
    if count < 0:
        packets, loops = messages, -1
    elif count % n == 0:
        packets, loops = messages, count // n
    else:
        packets, loops = [messages[i % n] for i in range(count)], 1
    lines = [json.dumps({"format": "waggler-packets/1", "time": "relative", "crc": "check",
                         "period": round(len(packets) / rate, 9)})]
    for i, m in enumerate(packets):
        pkt = {"t": round(i / rate, 9), "proto": proto, "hex": m}
        if dbm is not None:
            pkt["dBm"] = float(dbm)
        lines.append(json.dumps(pkt))
    return "\n".join(lines) + "\n", loops


def file_upload_commands(name: str, text: str) -> List[str]:
    """AT+FILE=BEGIN / DATA / END lines that upload ``text`` as file ``name`` (the same name replaces it)."""
    data = text.encode()
    cmds = [f"AT+FILE=BEGIN,{name},{len(data)},{hashlib.sha256(data).hexdigest()},jsonl"]
    for off in range(0, len(data), FILE_CHUNK):
        cmds.append(f"AT+FILE=DATA,{off},{data[off:off + FILE_CHUNK].hex().upper()}")
    cmds.append("AT+FILE=END")
    return cmds


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
    ``+TX:job=<id>`` at once and ends with ``+DONE:<id>,sent=,late=,dropped=,reason=``; ``AT+STOP`` ends unbounded
    ones. Levels are absolute dBm at the SMA: ``power_dbm`` (default PWIGGD_REF_DBM) minus the Bee-quantized
    attenuation, so a bench file gives the same level as on 1.x. Several explicit messages of one protocol play as
    ``AT+SCHED`` entries (finite, up to 256 packets) or else as an uploaded packet file with a period (deleted
    after). On a two-port Bee Waggler every job goes out on both ports (``AT+TX=BOTH``, each port at its own
    calibration), as 1.x mirrored TX1 to TX2; a Pluto Wiggler has port 1. ``dual`` starts a Mode S and a UAT job
    together with ``AT+HOLD``, time-shared unless the pattern needs overlap (rate 0) or ``simultaneous = true``:
    then a Bee Waggler checks ``AT+SIMULTANEOUS?`` and opts in to the fixed-LO plan
    (``AT+SIMULTANEOUS=MODE_S,UAT,ACCEPT_COSTS``, about 25 dB less maximum level, Mode S on port 1 and UAT on
    port 2) for that run only. After a 2.x run ``last_result`` holds each job's counts and ``warnings`` any
    ``+TX:warning`` lines and level notes.

    Bench file keys: usb_serial (required), mode_s_atten_db / subg_atten_db (applied when the pattern doesn't set
    atten_db); 2.x only: mode_s_port / uat_port (1, 2 or "BOTH"; default BOTH on a two-port unit), simultaneous (default
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
        self._job_cmd: Dict[str, str] = {}
        self._buf = ""
        self._simultaneous = False
        self._files: List[str] = []  # 2.x: packet files uploaded for this run
        self._levels: Dict[str, Optional[float]] = {}  # 2.x: dBm per protocol in this run (None = AT+LEVEL)
        self.simultaneous_costs: List[str] = []
        self.warnings: List[str] = []
        self.last_result: Optional[Dict] = None

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

    def v2_ports(self) -> Tuple[str, str]:
        """(Mode S port, UAT port) on a 2.x unit: "1", "2" or "BOTH" (a two-port unit's default, 1.x's mirror)."""
        default = "BOTH" if self._ports >= 2 else "1"
        return (str(self.cfg.options.get("mode_s_port", default)).upper(),
                str(self.cfg.options.get("uat_port", default)).upper())

    def _band_v2(self, proto: str, port: str, msgs: List[str], p: TestPattern, level: List[str], name: str,
                 default_level, allow_sched: bool) -> List[str]:
        """Commands for one protocol's job: its table, one message repeated, AT+SCHED entries or a packet file."""
        self._levels[proto] = float(level[0]) if level else None
        if len(msgs) <= 1:
            payload = msgs[0] if msgs else "TABLE"
            return [",".join([f"AT+TX={port}", proto, payload, self._count_v2(p.count), _num(p.rate)] + level)]
        if p.rate <= 0:
            raise TransmitterError("several explicit messages need a rate > 0")
        if allow_sched and 0 <= p.count <= SCHED_MAX:
            # Finite: every packet is an entry, played once (a looped sched wraps back to back). No validator, so
            # rates above the packet files' 7,800 Mode S/s per lane work.
            dbm = level[0] if level else (_num(default_level(port, proto)) if default_level else None)
            if dbm is None:
                raise TransmitterError("AT+SCHED entries need a level: set atten_db or power_dbm")
            cmds = ["AT+SCHED=CLEAR"]
            for i in range(p.count):
                cmds.append(f"AT+SCHED=ADD,{port},{proto},{round(i * 1e6 / p.rate)},{dbm},{msgs[i % len(msgs)]}")
            return cmds + [f"AT+TX={port},FILE,sched,1"]
        text, loops = packet_file(proto, msgs, p.count, p.rate, level[0] if level else None)
        self._files.append(name)
        return file_upload_commands(name, text) + [f"AT+TX={port},FILE,{name},{loops}"]

    def commands_for_v2(self, p: TestPattern, ports: Optional[int] = None, default_level=None) -> List[str]:
        """Command set 2 (pwiggd 2.x): AT commands that play pattern p. Each AT+TX line starts a job.

        ``default_level(port, proto)`` gives the unit's AT+LEVEL when AT+SCHED entries need a level the pattern
        and bench file don't set.
        """
        if ports is not None:
            self._ports = ports
        self._files, self._levels = [], {}
        ms_port, uat_port = self.v2_ports()
        ms_msgs, uat_msgs = split_messages(p.messages)
        cmds: List[str] = []
        if p.band == "1090":
            if uat_msgs:
                raise TransmitterError("band 1090 takes Mode S messages only")
            cmds += self._band_v2("MODE_S", ms_port, ms_msgs, p, self._level_v2(p, "mode_s_atten_db"), "hil_mode_s",
                                  default_level, True)
        elif p.band == "978":
            if ms_msgs:
                raise TransmitterError("band 978 takes UAT messages only")
            cmds += self._band_v2("UAT", uat_port, uat_msgs, p, self._level_v2(p, "subg_atten_db"), "hil_uat",
                                  default_level, True)
        elif p.band == "dual":
            if p.messages and not (ms_msgs and uat_msgs):
                raise TransmitterError("dual with explicit messages needs Mode S and UAT messages")
            want_simul = bool(self.cfg.options.get("simultaneous", False)) or p.rate <= 0
            if want_simul and self._ports >= 2:
                cmds.append("AT+SIMULTANEOUS=MODE_S,UAT,ACCEPT_COSTS")
                ms_port, uat_port = "1", "2"  # the fixed-LO plan: Mode S on port 1, UAT on port 2
            # Two jobs, so no AT+SCHED (one in-memory file): several messages go in packet files.
            jobs = (self._band_v2("MODE_S", ms_port, ms_msgs, p, self._level_v2(p, "mode_s_atten_db"), "hil_mode_s",
                                  default_level, False)
                    + self._band_v2("UAT", uat_port, uat_msgs, p, self._level_v2(p, "subg_atten_db"), "hil_uat",
                                    default_level, False))
            uploads = [c for c in jobs if c.startswith("AT+FILE=")]
            cmds += uploads + ["AT+HOLD=1"] + [c for c in jobs if not c.startswith("AT+FILE=")] + ["AT+HOLD=0"]
        else:
            raise TransmitterError(f"unknown band {p.band!r}")
        return cmds

    def _note(self, line: str) -> bool:
        """Records an unsolicited line; True if it was one (+DONE, +CONTROL:lost)."""
        m = re.match(r"\+DONE:(\d+),", line)
        if m:
            self._jobs[m.group(1)] = line
            return True
        if line.startswith("+CONTROL:lost"):
            self._warn(f"{self.id}: another session took control")
            return True
        return False

    def _warn(self, msg: str) -> None:
        self.warnings.append(msg)
        log.warning("%s", msg)

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
                    self._job_cmd[m.group(1)] = cmd
                for x in out:
                    if x.startswith("+TX:warning="):  # e.g. a looped file without a period
                        self._warn(f"{self.id}: {x}")
                return out
            if line.startswith("ERROR"):
                raise TransmitterError(f"{self.id}: {cmd} -> {line}")
            out.append(line)
        raise TransmitterError(f"{self.id}: {cmd}: no OK within {timeout:g} s")

    def _default_level(self, port: str, proto: str) -> float:
        """The unit's AT+LEVEL for a port (port 1 for BOTH) and protocol."""
        want = "1" if port == "BOTH" else port
        for line in self._cmd_v2("AT+LEVEL?"):
            f = line.split("=", 1)[-1].split(",")
            if line.startswith("+LEVEL=") and len(f) >= 3 and f[0] == want and f[1] == proto:
                return float(f[2])
        raise TransmitterError(f"{self.id}: AT+LEVEL? has no level for port {want} {proto}")

    def _check_simultaneous(self) -> None:
        """Before opting in: the unit can do Mode S + UAT, and the levels fit the plan. Keeps the cost lines."""
        out = self._cmd_v2("AT+SIMULTANEOUS?")
        # Prefix match: the pair line goes on with the method and a status note.
        pair = next((x for x in out if x.startswith("+SIMULTANEOUS:pair=MODE_S+UAT,")), None)
        if pair is None or not pair.startswith("+SIMULTANEOUS:pair=MODE_S+UAT,available,"):
            raise TransmitterError(f"{self.id}: no simultaneous Mode S + UAT on this unit ({pair or 'no pair line'})")
        self.simultaneous_costs = [x.split("=", 1)[1] for x in out if x.startswith("+SIMULTANEOUS:cost=")]
        for c in self.simultaneous_costs:
            log.info("%s: simultaneous cost: %s", self.id, c)
        for proto, key in (("MODE_S", "max_mode_s_dbm"), ("UAT", "max_uat_dbm")):
            line = next((x for x in out if x.startswith(f"+SIMULTANEOUS:{key}=")), None)
            level = self._levels.get(proto)
            if line and level is not None and level > float(line.split("=", 1)[1]) + 1e-9:
                raise TransmitterError(f"{self.id}: {proto} at {level:g} dBm is above the simultaneous plan's "
                                       f"maximum ({line.split('=', 1)[1]} dBm); attenuate more")
        level = self._levels.get("MODE_S")
        if level is not None and level > SIMULTANEOUS_TL_DBM:
            self._warn(f"{self.id}: simultaneous Mode S at {level:g} dBm: an ADSBee fed above about "
                       f"{SIMULTANEOUS_TL_DBM:g} dBm misses frames unless its AT+TL_OFFSET is raised (or attenuate "
                       f"more)")

    def _start_v2(self, pattern: TestPattern) -> None:
        self._jobs, self._job_cmd, self._buf, self._simultaneous = {}, {}, "", False
        self.simultaneous_costs, self.warnings, self.last_result = [], [], None
        self._cmd_v2("AT+CONTROL=TAKE")  # The bench lock makes this tool the unit's only user.
        self._cmd_v2("AT+STOP=ALL")  # A job left running by a crashed run.
        cmds = self.commands_for_v2(pattern, default_level=self._default_level)
        try:
            for cmd in cmds:
                if cmd.startswith("AT+SIMULTANEOUS="):
                    self._check_simultaneous()
                out = self._cmd_v2(cmd, 10.0 if cmd.startswith(("AT+SIMULTANEOUS", "AT+FILE=END")) else 5.0)
                if cmd.startswith("AT+SIMULTANEOUS="):
                    self._simultaneous = True
                if cmd == "AT+FILE=END" and not (out and ",valid," in out[0]):
                    raise TransmitterError(f"{self.id}: packet file refused: {'; '.join(out[:4])}")
        except Exception:
            if "AT+HOLD=1" in cmds:
                with contextlib.suppress(Exception):
                    self._cmd_v2("AT+STOP=ALL")
                    self._cmd_v2("AT+HOLD=0")
            self._finish_v2()
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
        jobs = []
        for job, line in self._jobs.items():
            f = dict(kv.split("=", 1) for kv in line.split(",")[1:] if "=" in kv)
            jobs.append({"job": int(job), "command": self._job_cmd.get(job, ""), "reason": f.get("reason", ""),
                         **{k: int(f.get(k, 0)) for k in ("sent", "late", "dropped")}})
        self.last_result = {k: sum(j[k] for j in jobs) for k in ("sent", "late", "dropped")}
        self.last_result["jobs"] = jobs
        for j in jobs:  # per job since pwiggd 2.0 counts each late / dropped packet to its own job
            if j["late"] or j["dropped"]:
                self._warn(f"{self.id}: job {j['job']} ({j['command']}): {j['late']} late, {j['dropped']} dropped "
                           f"of {j['sent']} sent")
        bad = [d for d in self._jobs.values() if d and not re.search(r"reason=(done|stopped)\b", d)]
        if bad:
            raise TransmitterError(f"{self.id}: {'; '.join(bad)}")

    def _finish_v2(self) -> None:
        """After a 2.x run: back to the time-shared plan, uploaded files deleted."""
        if self._console is None:
            return
        if self._simultaneous:
            self._simultaneous = False
            with contextlib.suppress(Exception):
                self._cmd_v2("AT+SIMULTANEOUS=OFF", 10.0)
        for name in self._files:
            with contextlib.suppress(Exception):
                self._cmd_v2(f"AT+FILE=DELETE,{name}")
        self._files = []

    def stop_all(self) -> str:
        """Emergency stop for a transmission left running by another process. 2.x ignores a bare character."""
        with self.lock("at"):
            self._console = self._open()
            try:
                if self._detect(self._console) != CMD_SET_2:
                    return self._console.transact("x", 3.0)  # Any character ends a 1.x transmission.
                self._buf, self._jobs = "", {}
                out = []
                for cmd in ("AT+CONTROL=TAKE", "AT+STOP=ALL", "AT+SIMULTANEOUS=OFF"):
                    out += [cmd] + self._cmd_v2(cmd, 10.0)
                for line in self._lines(time.monotonic() + 0.5):  # +DONE lines of the stopped jobs
                    self._note(line)
                return "\n".join(out + [d for d in self._jobs.values() if d])
            finally:
                self._console.close()
                self._console = None

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
                self._finish_v2()
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
            if self._cmd_set == CMD_SET_2:
                self._finish_v2()
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
