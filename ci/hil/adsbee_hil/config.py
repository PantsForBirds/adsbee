"""Bench inventory: which receivers and transmitters are attached to this host, and how.

The inventory is a TOML file (see ``bench.example.toml``). It is looked up in this order:
``--config``, ``$ADSBEE_HIL_CONFIG``, ``~/.config/adsbee-hil/bench.toml``. Most commands also work
without one: devices can be addressed by ``--serial`` plus ``--model``.

Library code never hardcodes a bench's serial numbers; they live only in the bench file.
"""

import os
import shlex
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

try:
    import tomllib  # Python 3.11+
except ModuleNotFoundError:  # pragma: no cover
    import tomli as tomllib  # type: ignore

DEFAULT_PATHS = [os.path.expanduser("~/.config/adsbee-hil/bench.toml")]


class ConfigError(ValueError):
    pass


@dataclass
class ReceiverConfig:
    id: str
    model: str
    usb_serial: str
    usb_port: Optional[str] = None  # Expected sysfs USB port path, e.g. "1-1.3". Checked if given.
    tags: List[str] = field(default_factory=list)
    options: Dict[str, Any] = field(default_factory=dict)  # Model-specific extras.


@dataclass
class TransmitterConfig:
    id: str
    type: str
    usb_serial: Optional[str] = None
    tags: List[str] = field(default_factory=list)
    # Receivers this transmitter is cabled/radiating to. Empty = all receivers on the bench.
    receivers: List[str] = field(default_factory=list)
    options: Dict[str, Any] = field(default_factory=dict)  # Type-specific: uri, attenuation, ...


@dataclass
class Bench:
    name: str = "unnamed"
    path: Optional[str] = None
    lock_dir: Optional[str] = None
    # Refuse to flash while a process whose command line matches one of these regexes runs (like
    # pgrep -f), e.g. a CI job that owns a receiver. Overridable per command with --force.
    busy_processes: List[str] = field(default_factory=list)
    # External tools, e.g. flashers["adsbee_1421"] = {"command": [...]}.
    flashers: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    receivers: List[ReceiverConfig] = field(default_factory=list)
    transmitters: List[TransmitterConfig] = field(default_factory=list)

    def receiver(self, key: str) -> Optional[ReceiverConfig]:
        for r in self.receivers:
            if key in (r.id, r.usb_serial):
                return r
        return None

    def transmitter(self, key: str) -> Optional[TransmitterConfig]:
        for t in self.transmitters:
            if key in (t.id, t.usb_serial):
                return t
        return None

    def flasher_command(self, model: str, receiver: Optional[ReceiverConfig] = None) -> Optional[List[str]]:
        """Command template for a model's external flasher: receiver option > env var > [flashers]."""
        if receiver and receiver.options.get("flasher_command"):
            return _as_argv(receiver.options["flasher_command"])
        env = os.environ.get(f"ADSBEE_HIL_FLASHER_{model.upper()}")
        if env:
            return shlex.split(env)
        cmd = self.flashers.get(model, {}).get("command")
        return _as_argv(cmd) if cmd else None


def _as_argv(cmd) -> List[str]:
    if isinstance(cmd, str):
        return shlex.split(cmd)
    if isinstance(cmd, list) and all(isinstance(c, str) for c in cmd):
        return list(cmd)
    raise ConfigError(f"flasher command must be a string or a list of strings, not {cmd!r}")


def _take(d: Dict[str, Any], where: str, key: str, typ, required: bool = False, default=None):
    if key not in d:
        if required:
            raise ConfigError(f"{where}: missing required key '{key}'")
        return default
    v = d.pop(key)
    if not isinstance(v, typ):
        raise ConfigError(f"{where}: '{key}' must be {getattr(typ, '__name__', typ)}, not {type(v).__name__}")
    return v


def parse(data: Dict[str, Any], path: Optional[str] = None) -> Bench:
    data = dict(data)
    bench_tbl = dict(data.pop("bench", {}))
    bench = Bench(
        path=path,
        name=_take(bench_tbl, "[bench]", "name", str, default="unnamed"),
        lock_dir=_take(bench_tbl, "[bench]", "lock_dir", str),
        busy_processes=_take(bench_tbl, "[bench]", "busy_processes", list, default=[]),
    )
    if bench_tbl:
        raise ConfigError(f"[bench]: unknown keys {sorted(bench_tbl)}")
    bench.flashers = data.pop("flashers", {})
    for model, tbl in bench.flashers.items():
        if "command" in tbl:
            _as_argv(tbl["command"])

    ids, serials = set(), set()

    def claim(where, ident, serial):
        if ident in ids:
            raise ConfigError(f"{where}: duplicate id '{ident}'")
        ids.add(ident)
        if serial:
            if serial in serials:
                raise ConfigError(f"{where}: duplicate usb_serial '{serial}'")
            serials.add(serial)

    for i, raw in enumerate(data.pop("receivers", [])):
        r = dict(raw)
        where = f"receivers[{i}]"
        rc = ReceiverConfig(
            id=_take(r, where, "id", str, required=True),
            model=_take(r, where, "model", str, required=True),
            usb_serial=_take(r, where, "usb_serial", str, required=True),
            usb_port=_take(r, where, "usb_port", str),
            tags=_take(r, where, "tags", list, default=[]),
        )
        rc.options = r  # Whatever is left is model-specific.
        claim(where, rc.id, rc.usb_serial)
        bench.receivers.append(rc)

    for i, raw in enumerate(data.pop("transmitters", [])):
        t = dict(raw)
        where = f"transmitters[{i}]"
        tc = TransmitterConfig(
            id=_take(t, where, "id", str, required=True),
            type=_take(t, where, "type", str, required=True),
            usb_serial=_take(t, where, "usb_serial", str),
            tags=_take(t, where, "tags", list, default=[]),
            receivers=_take(t, where, "receivers", list, default=[]),
        )
        tc.options = t
        claim(where, tc.id, tc.usb_serial)
        bench.transmitters.append(tc)

    rx_ids = {r.id for r in bench.receivers}
    for t in bench.transmitters:
        unknown = [r for r in t.receivers if r not in rx_ids]
        if unknown:
            raise ConfigError(f"transmitter '{t.id}': unknown receivers {unknown}")
    if data:
        raise ConfigError(f"unknown top-level keys {sorted(data)}")
    return bench


def load(path: Optional[str] = None, required: bool = False) -> Bench:
    """Loads the bench file. With no file found, returns an empty Bench (unless required)."""
    candidates = [path] if path else [os.environ.get("ADSBEE_HIL_CONFIG")] + DEFAULT_PATHS
    for p in candidates:
        if p and os.path.isfile(p):
            with open(p, "rb") as f:
                try:
                    return parse(tomllib.load(f), path=p)
                except tomllib.TOMLDecodeError as e:
                    raise ConfigError(f"{p}: {e}") from e
    if path or required:
        raise ConfigError(f"bench config not found: {path or 'set --config or ADSBEE_HIL_CONFIG'}")
    return Bench()
