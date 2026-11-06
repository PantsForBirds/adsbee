"""
The device report, v1 (docs/swarm_api.md): validated into a `Report`, then stored by `ingest`.

Retries (the device keeps seq and period.start_uptime_s until a report is acknowledged):
same seq, start and len_s is a duplicate and is not stored; same seq and start with a longer len_s
is the earlier report widened, so it replaces that report's stats and max-merges its range.

Range maps: `ranges` holds one per protocol (`ranges.REPORTED`); older firmware sends one `range`
for every aircraft, stored as protocol `aircraft`. `listen_s` (seconds each receiver was on) and
`frames` (valid frames of protocols without legacy stats) are optional: without `listen_s` the
listen time is unknown. A protocol this server doesn't know is ignored and counted, never a 400, so
newer firmware keeps reporting to an older server.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from datetime import datetime, timedelta

from django.db import transaction
from django.utils import timezone

from . import conf, ranges
from .models import PEAK_STATS, REPORTED_STATS, STATS, WITHHELD, Device, Sample, SampleRange

UNIQUE_ID = re.compile(r'^[0-9A-F]{16}$')
MAC = re.compile(r'^[0-9A-F]{2}(:[0-9A-F]{2}){5}$')
VERSION = re.compile(r'^[0-9A-Za-z._+-]{0,64}$')
SOURCE = re.compile(r'^[a-z0-9_]{0,16}$')
MAX_COUNT = 2 ** 40
MAX_LISTEN_S = 7 * 86400
#: Most protocols one of `ranges`, `listen_s` or `frames` may hold, known or not.
MAX_PROTOCOLS = 16
PROTOCOL_NAME = re.compile(r'^[a-z0-9_]{1,16}$')


class ReportError(ValueError):
    """400 bad_request; the message is the detail."""


@dataclass
class Report:
    product: str
    unique_id: str
    mac_address: str
    versions: dict
    auto_update: bool
    uptime_s: int
    seq: int
    start_uptime_s: int
    len_s: int
    rx_position: dict | None
    stats: dict
    #: {protocol: (unit_m, {bin: meters})}, bins past the horizon already left out.
    maps: dict[str, tuple[int, ranges.Map]] = field(default_factory=dict)
    #: Why there is no range: '', 'no_position' or 'implausible' (the bee's, or ours).
    range_withheld: str = ''
    #: {protocol: seconds listened, > 0}, or None when the report has no `listen_s` (unknown).
    listen: dict[str, int] | None = None
    #: {protocol: valid frames} for `ranges.FRAMES` protocols.
    frames: dict[str, int] = field(default_factory=dict)
    #: Protocol names this server doesn't know, sorted.
    ignored: list[str] = field(default_factory=list)


def _int(data: dict, key: str, low: int = 0, high: int = MAX_COUNT) -> int:
    value = data.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
        raise ReportError(f'{key} must be an integer {low}..{high}')
    return value


def _number(data: dict, key: str, low: float, high: float) -> float:
    value = data.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) \
            or not low <= value <= high:
        raise ReportError(f'rx_position.{key} must be a number {low}..{high}')
    return float(value)


def parse(data) -> Report:
    if not isinstance(data, dict):
        raise ReportError('body must be a JSON object')
    if data.get('v') != 1:
        raise ReportError('v must be 1')
    product = data.get('product')
    if product not in conf.products():
        raise ReportError('unknown product')
    unique_id = str(data.get('unique_id') or '').strip().upper()
    if not UNIQUE_ID.match(unique_id):
        raise ReportError('unique_id must be 16 hex digits')
    mac = str(data.get('mac_address') or '').strip().upper().replace('-', ':')
    if mac and not MAC.match(mac):
        raise ReportError('mac_address must look like DC:54:75:D2:6C:24')
    versions = data.get('versions') or {}
    if (not isinstance(versions, dict) or len(versions) > 8
            or not all(isinstance(k, str) and len(k) <= 16 and isinstance(v, str) and VERSION.match(v)
                       for k, v in versions.items())):
        raise ReportError('versions must map component to version string')
    auto_update = data.get('auto_update', False)
    if not isinstance(auto_update, bool):
        raise ReportError('auto_update must be true or false')
    period = data.get('period')
    if not isinstance(period, dict):
        raise ReportError('period must be an object')
    stats = data.get('stats') or {}
    if not isinstance(stats, dict):
        raise ReportError('stats must be an object')
    report = Report(
        product=product, unique_id=unique_id, mac_address=mac, versions=versions, auto_update=auto_update,
        uptime_s=_int(data, 'uptime_s'), seq=_int(data, 'seq'),
        start_uptime_s=_int(period, 'start_uptime_s'), len_s=_int(period, 'len_s', 1, 7 * 86400),
        rx_position=_position(data.get('rx_position')),
        stats={key: _int(stats, key) if key in stats else 0 for key in REPORTED_STATS})
    withheld = data.get('range_withheld') or ''
    if withheld not in WITHHELD:
        raise ReportError('range_withheld is no_position or implausible')
    report.range_withheld = withheld
    report.stats['range_dropped'] = _int(data, 'range_dropped') if 'range_dropped' in data else 0
    report.stats['bins_ignored'] = 0
    ignored: set[str] = set()
    maps = _maps(data, ignored)
    listen = _counts(data, 'listen_s', MAX_LISTEN_S, ignored)
    if listen is not None:
        # Seconds past the period are rounding or a retry's sum: never more than the period.
        report.listen = {name: min(value, report.len_s) for name, value in listen.items() if value > 0}
    report.frames = {name: value for name, value in (_counts(data, 'frames', MAX_COUNT, ignored) or {}).items()
                     if name in ranges.FRAMES}
    report.ignored = sorted(ignored)
    report.stats['protocols_ignored'] = len(ignored)
    if maps and report.rx_position is None:
        report.range_withheld = report.range_withheld or 'no_position'
    elif maps:
        # Defense for firmware without the guards: a bin past its band's horizon is not real.
        for protocol, (unit, values) in maps.items():
            kept, ignored = ranges.within_horizon(values, report.rx_position['alt_m'], unit, protocol)
            report.stats['bins_ignored'] += ignored
            if kept:
                report.maps[protocol] = (unit, kept)
    return report


def _protocols(data: dict, key: str, ignored: set[str]) -> dict | None:
    """`data[key]` ({protocol: value}) with the protocols this server knows; the others go to
    `ignored`. None when the key is absent."""
    value = data.get(key)
    if value is None:
        return None
    if not isinstance(value, dict):
        raise ReportError(f'{key} must be an object')
    if len(value) > MAX_PROTOCOLS:
        raise ReportError(f'{key} has more than {MAX_PROTOCOLS} protocols')
    known = {}
    for name, item in value.items():
        if not PROTOCOL_NAME.match(name):
            raise ReportError(f'{key} protocol names are 1-16 of a-z, 0-9 and _')
        if name in ranges.REPORTED:
            known[name] = item
        else:
            ignored.add(name)
    return known


def _counts(data: dict, key: str, high: int, ignored: set[str]) -> dict[str, int] | None:
    """`listen_s` or `frames`: {protocol: integer 0..high}."""
    known = _protocols(data, key, ignored)
    if known is None:
        return None
    try:
        return {name: _int(known, name, 0, high) for name in known}
    except ReportError as exc:
        raise ReportError(f'{key}.{exc}') from exc


def _maps(data: dict, ignored: set[str]) -> dict[str, tuple[int, ranges.Map]]:
    """`ranges` ({protocol: map}), else `range` as protocol `aircraft`."""
    legacy = data.get('range')
    per_protocol = _protocols(data, 'ranges', ignored)
    if per_protocol is not None:
        if legacy is not None:
            raise ReportError('send range or ranges, not both')
        return {protocol: _range(value, protocol) for protocol, value in per_protocol.items()}
    if legacy is not None:
        return {ranges.LEGACY: _range(legacy, ranges.LEGACY)}
    return {}


def _position(value) -> dict | None:
    if value is None:
        return None
    if not isinstance(value, dict):
        raise ReportError('rx_position must be an object or null')
    source = str(value.get('source') or '')[:16]
    if not SOURCE.match(source):
        raise ReportError('rx_position.source must be a short lowercase word')
    alt = value.get('alt_m')
    position = {'lat': _number(value, 'lat', -90, 90), 'lon': _number(value, 'lon', -180, 180),
                'alt_m': None if alt is None else _number(value, 'alt_m', -1000, 100000), 'source': source}
    # 0,0 is an unset position, not a bee in the Gulf of Guinea.
    return None if position['lat'] == 0 and position['lon'] == 0 else position


def _range(value, protocol: str) -> tuple[int, ranges.Map]:
    name = 'range' if protocol == ranges.LEGACY else f'ranges.{protocol}'
    if not isinstance(value, dict):
        raise ReportError(f'{name} must be an object')
    if value.get('sectors') != ranges.SECTORS or value.get('bands') != ranges.BANDS:
        raise ReportError(f'{name} must be {ranges.SECTORS} sectors x {ranges.BANDS} bands')
    try:
        unit = _int(value, 'unit_m', 1, 1_000_000)
    except ReportError as exc:
        raise ReportError(f'{name}.{exc}') from exc
    required = ranges.PROTOCOLS[protocol].unit_m
    if required is not None and unit != required:
        raise ReportError(f'{name}.unit_m must be {required}')
    try:
        return unit, ranges.decode(value.get('changed'), value.get('max'), unit)
    except ranges.RangeError as exc:
        raise ReportError(f'{name}: {exc}') from exc


def hour_of(moment: datetime) -> datetime:
    return moment.replace(minute=0, second=0, microsecond=0)


def day_of(moment: datetime) -> datetime:
    return moment.replace(hour=0, minute=0, second=0, microsecond=0)


def _replace(current: int | None, old: int, new: int) -> int:
    """A sum with the widened report's value taken back out first."""
    return max(0, (current or 0) - old) + new


def _apply(sample: Sample, report: Report, replaced: dict | None) -> None:
    """Add a report to an hourly or daily row. `replaced`: the last report (Device.last_report)
    when this one widens it: its stats, listen time and frames are taken back out first, so a
    widened retry counts once (its range is a superset: max-merge)."""
    old = (replaced or {}).get('stats') or {}
    for name in STATS:
        if name in PEAK_STATS:
            setattr(sample, name, max(getattr(sample, name), report.stats[name]))
        else:
            setattr(sample, name, _replace(getattr(sample, name), old.get(name, 0), report.stats[name]))
    if replaced is None:
        sample.reports += 1
    if WITHHELD.index(report.range_withheld) > WITHHELD.index(sample.range_withheld or ''):
        sample.range_withheld = report.range_withheld
    sample.save()
    old_listen = (replaced or {}).get('listen_s') or {}
    old_frames = (replaced or {}).get('frames') or {}
    listen = report.listen or {}
    for protocol in dict.fromkeys([*report.maps, *listen, *old_listen, *report.frames, *old_frames]):
        row = SampleRange.objects.select_for_update().filter(sample=sample, protocol=protocol).first()
        if protocol in report.maps:
            unit, values = report.maps[protocol]
            if row is None:
                row = SampleRange(sample=sample, protocol=protocol, unit_m=unit)
                merged = dict(values)
            else:
                merged = ranges.merge(row.ranges(), values)
                # Coarsen to the largest unit either came in.
                row.unit_m = max(row.unit_m, unit)
            row.set_ranges(merged)
        elif row is None:
            # Listened (or counted frames) and heard nothing.
            row = SampleRange(sample=sample, protocol=protocol, unit_m=ranges.PROTOCOLS[protocol].unit_m)
            row.set_ranges({})
        if protocol in listen or protocol in old_listen:
            row.listen_s = _replace(row.listen_s, old_listen.get(protocol, 0), listen.get(protocol, 0))
        if protocol in report.frames or protocol in old_frames:
            row.frames = _replace(row.frames, old_frames.get(protocol, 0), report.frames.get(protocol, 0))
        row.save()


def ingest(device: Device, report: Report, now: datetime | None = None) -> bool:
    """Store a report on `device` (locked by the caller). Returns False for a duplicate."""
    now = now or timezone.now()
    key = [report.seq, report.start_uptime_s, report.len_s]
    last = device.last_report or {}
    previous = last.get('key') or [None, None, None]
    widened = previous[:2] == key[:2] and previous[2] is not None and report.len_s > previous[2]
    if previous == key or (previous[:2] == key[:2] and not widened):
        return False

    if widened:
        sample = Sample.objects.select_for_update().filter(
            device=device, span_s=Sample.HOUR, start=last['start']).first()
    else:
        sample = None
    if sample is None:
        sample, _ = Sample.objects.select_for_update().get_or_create(
            device=device, span_s=Sample.HOUR, start=hour_of(now))
        replaced = None
    else:
        replaced = last
    _apply(sample, report, replaced)
    # The bee's day, kept beside its hours from the first report: long windows read days.
    daily, _ = Sample.objects.select_for_update().get_or_create(
        device=device, span_s=Sample.DAY, start=day_of(sample.start))
    _apply(daily, report, replaced)

    device.mac_address = report.mac_address or device.mac_address
    device.versions = report.versions
    device.auto_update = report.auto_update
    device.uptime_s = report.uptime_s
    device.last_report_at = now
    device.range_withheld = report.range_withheld
    device.range_dropped = report.stats['range_dropped']
    device.bins_ignored = report.stats['bins_ignored']
    device.protocols_ignored = len(report.ignored)
    device.ignored_protocols = ','.join(report.ignored)[:200]
    if report.rx_position is not None:
        device.rx_lat, device.rx_lon = report.rx_position['lat'], report.rx_position['lon']
        device.rx_alt_m, device.rx_source = report.rx_position['alt_m'], report.rx_position['source']
    device.last_report = {'key': key, 'start': sample.start.isoformat(), 'stats': report.stats,
                          'listen_s': report.listen or {}, 'frames': report.frames}
    device.save()
    return True


def store(device: Device, report: Report, now: datetime | None = None) -> bool:
    """Ingest under a row lock on the device. Returns False for a duplicate."""
    with transaction.atomic():
        device = Device.objects.select_for_update().get(pk=device.pk)
        return ingest(device, report, now)


def window(since: datetime | None, until: datetime | None, default_days: int = 7) -> tuple[datetime, datetime]:
    until = until or timezone.now()
    return since or until - timedelta(days=default_days), until
