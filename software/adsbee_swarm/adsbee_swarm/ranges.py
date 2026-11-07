"""
Range maps: 12 height bands x 36 bearing sectors, one max distance per bin, one map per protocol.

On the wire (docs/swarm_api.md) a map is a 54-byte bitmap of present bins plus one byte per present
bin, in bin order, each `ceil(distance / unit_m)`. Stored the same way: `pack` writes the bitmap then
the values, so a sparse hour costs 54 bytes plus one per heard bin. Everything above this module
works with `Map`: {bin index: distance in meters}.
"""
from __future__ import annotations

import base64
import binascii
import json
import math
from pathlib import Path

_REGISTRY = json.loads((Path(__file__).parent / 'protocols.json').read_text(encoding='utf-8'))

SECTORS = _REGISTRY['sectors']
BANDS = _REGISTRY['bands']
BINS = SECTORS * BANDS
BITMAP_BYTES = BINS // 8
SECTOR_DEG = 360 // SECTORS
DEFAULT_UNIT_M = 2000

Map = dict[int, int]


class Protocol:
    """One entry of protocols.json."""

    def __init__(self, entry: dict):
        self.name: str = entry['name']
        self.reported: bool = entry['reported']
        self.unit_m: int | None = entry['unit_m']       # the one a report must use (None: any)
        self.basis: str = entry['basis']                # what the bands measure
        self.floors_ft: list[int] = entry['floors_ft']
        self.top_ft: int = entry['top_ft']              # the top band's ceiling, for the horizon limit
        self.aircraft: bool = entry['aircraft']         # part of the `aircraft` union
        self.stats: tuple[str, ...] = tuple(entry['stats'])
        self.label, self.symbol, self.icon, self.tip = entry['label'], entry['symbol'], entry['icon'], entry['tip']


#: Every protocol a stored map may have, in picker order. `aircraft` is also an old report's `range`
#: (Mode S and UAT mixed).
PROTOCOLS = {entry['name']: Protocol(entry) for entry in _REGISTRY['protocols']}
LEGACY = 'aircraft'
#: What a bee may send in `ranges`, `listen_s` and `frames` (also the report reply's `protocols`).
REPORTED = tuple(name for name, p in PROTOCOLS.items() if p.reported)
#: Stored rows each user-facing protocol reads: `aircraft` is the max over every aircraft map.
SOURCES = {name: (tuple(n for n, q in PROTOCOLS.items() if q.aircraft) if name == LEGACY else (name,))
           for name in PROTOCOLS}
#: Reported protocols counted in `frames`: those without legacy stats columns.
FRAMES = tuple(name for name in REPORTED if not PROTOCOLS[name].stats)
#: Aircraft band floors in feet (baro, else geometric altitude); the last band has no ceiling.
BAND_FLOORS_FT = PROTOCOLS[LEGACY].floors_ft
#: Remote ID band floors in feet above the receiver.
REMOTE_ID_FLOORS_FT = PROTOCOLS['remote_id'].floors_ft


class RangeError(ValueError):
    """A map that does not decode; the message says why."""


def bin_index(band: int, sector: int) -> int:
    return band * SECTORS + sector


FT_M = 0.3048


def horizon_limit_m(band: int, rx_alt_m: float | None, protocol: str = LEGACY) -> float:
    """The farthest a position in `band` can plausibly be (position guards):
    1.15 * 4.12 km * (sqrt(h_rx) + sqrt(h_band_top)) + 20 km, heights in meters, h_rx floored at 0."""
    table = PROTOCOLS[protocol]
    top_ft = table.floors_ft[band + 1] if band + 1 < BANDS else table.top_ft
    return (1.15 * 4.12 * (math.sqrt(max(rx_alt_m or 0.0, 0.0)) + math.sqrt(top_ft * FT_M)) + 20) * 1000


def within_horizon(values: Map, rx_alt_m: float | None, unit_m: int, protocol: str = LEGACY) -> tuple[Map, int]:
    """(the bins inside their band's horizon limit, how many were not). A value is rounded up to
    `unit_m` on the wire, so a bin up to one unit past the limit still counts."""
    kept = {i: m for i, m in values.items() if m <= horizon_limit_m(i // SECTORS, rx_alt_m, protocol) + unit_m}
    return kept, len(values) - len(kept)


def band_edges_ft(protocol: str = LEGACY) -> list[list[int | None]]:
    floors = PROTOCOLS[protocol].floors_ft
    return [[floor, floors[i + 1] if i + 1 < BANDS else None] for i, floor in enumerate(floors)]


def _bits(bitmap: bytes) -> list[int]:
    return [i for i in range(BINS) if bitmap[i >> 3] >> (i & 7) & 1]


def decode(changed_b64: str, max_b64: str, unit_m: int) -> Map:
    """The wire form to {bin: meters}. Bit i of the bitmap is byte i // 8, bit i % 8 (LSB first)."""
    try:
        bitmap = base64.b64decode(changed_b64, validate=True)
        values = base64.b64decode(max_b64, validate=True)
    except (binascii.Error, ValueError, TypeError) as exc:
        raise RangeError('changed and max must be base64') from exc
    if len(bitmap) != BITMAP_BYTES:
        raise RangeError(f'changed must be {BITMAP_BYTES} bytes')
    present = _bits(bitmap)
    if len(values) != len(present):
        raise RangeError(f'max has {len(values)} bytes for {len(present)} bins')
    if 0 in values:
        raise RangeError('max values are 1..255')
    return {index: value * unit_m for index, value in zip(present, values)}


def encode(ranges: Map, unit_m: int = DEFAULT_UNIT_M) -> tuple[str, str]:
    """{bin: meters} to the wire form (the simulator and tests)."""
    bitmap, values = _pack_parts(ranges, unit_m)
    return base64.b64encode(bitmap).decode(), base64.b64encode(values).decode()


def _pack_parts(ranges: Map, unit_m: int) -> tuple[bytes, bytes]:
    bitmap = bytearray(BITMAP_BYTES)
    values = bytearray()
    for index in sorted(ranges):
        if not 0 <= index < BINS:
            raise RangeError(f'bin {index} out of range')
        bitmap[index >> 3] |= 1 << (index & 7)
        values.append(min(255, max(1, math.ceil(ranges[index] / unit_m))))
    return bytes(bitmap), bytes(values)


def pack(ranges: Map, unit_m: int) -> bytes:
    """Stored form: bitmap then values, in `unit_m` steps (rounded up, as the device does)."""
    bitmap, values = _pack_parts(ranges, unit_m)
    return bitmap + values


def unpack(blob: bytes | memoryview | None, unit_m: int) -> Map:
    if not blob:
        return {}
    blob = bytes(blob)
    present = _bits(blob[:BITMAP_BYTES])
    return {index: value * unit_m for index, value in zip(present, blob[BITMAP_BYTES:])}


def merge(into: Map, other: Map) -> Map:
    """Max per bin, in place."""
    for index, meters in other.items():
        if meters > into.get(index, 0):
            into[index] = meters
    return into


def matrix(ranges: Map) -> list[list[int | None]]:
    """bands x sectors, meters or None: what a 3D view draws."""
    return [[ranges.get(bin_index(band, sector)) for sector in range(SECTORS)] for band in range(BANDS)]


def sector_max(ranges: Map) -> list[int | None]:
    """Per sector, the max over every band: the 2D outline."""
    out: list[int | None] = [None] * SECTORS
    for index, meters in ranges.items():
        sector = index % SECTORS
        if out[sector] is None or meters > out[sector]:
            out[sector] = meters
    return out
