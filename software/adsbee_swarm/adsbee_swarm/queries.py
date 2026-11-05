"""The read side: range maps and stats over a time window, for one bee or an account's swarm."""
from __future__ import annotations

from collections import defaultdict
from datetime import datetime, timedelta, timezone as tz


from . import ranges
from .models import PEAK_STATS, STATS, Device, Sample, SampleRange


#: Windows longer than this read daily rows (kept from the first report); shorter ones hourly.
DAILY_AFTER = timedelta(days=3)


def _samples(devices, since: datetime, until: datetime, span: int | None = None):
    """Hourly rows starting inside the window, or daily rows whose day overlaps it."""
    span = span or (Sample.DAY if until - since > DAILY_AFTER else Sample.HOUR)
    rows = Sample.objects.filter(device__in=devices, span_s=span, start__lt=until)
    rows = rows.filter(start__gte=since) if span == Sample.HOUR else rows.filter(start__gt=since - timedelta(days=1))
    return rows.order_by('start')


def _range_rows(devices, since: datetime, until: datetime):
    return SampleRange.objects.filter(sample__in=_samples(devices, since, until).values('pk'))


def device_range(device: Device, since: datetime, until: datetime, protocol: str = ranges.LEGACY) -> ranges.Map:
    """Max per bin over the window. `aircraft`: over Mode S, UAT and old firmware's mixed maps."""
    merged: ranges.Map = {}
    for row in _range_rows([device], since, until).filter(protocol__in=ranges.SOURCES[protocol]).only(
            'range_map', 'unit_m'):
        ranges.merge(merged, row.ranges())
    return merged


def protocols_with_data(devices, since: datetime, until: datetime) -> list[str]:
    """Which `ranges.SOURCES` keys have a map in the window, in `PROTOCOLS` order."""
    stored = set(_range_rows(devices, since, until).values_list('protocol', flat=True).distinct())
    return [name for name in ranges.PROTOCOLS if stored & set(ranges.SOURCES[name])]


def range_payload(device: Device, merged: ranges.Map) -> dict:
    return {'id': device.pk, 'label': device.label, 'unique_id': device.unique_id,
            'position': device.position, 'check_position': device.check_position, 'max_m': ranges.matrix(merged),
            'sector_max_m': ranges.sector_max(merged), 'bins': len(merged),
            'max_range_m': max(merged.values(), default=None)}


def swarm_range(devices, since: datetime, until: datetime, protocol: str = ranges.LEGACY) -> dict:
    """Per bee its bands x sectors maxima for `protocol`; `swarm` is the max per bin over all of
    them. `protocols`: the ones with data for these bees in the window."""
    devices = list(devices)
    per_device = []
    swarm: ranges.Map = {}
    for device in devices:
        merged = device_range(device, since, until, protocol)
        ranges.merge(swarm, merged)
        per_device.append(range_payload(device, merged))
    return {'since': since.isoformat(), 'until': until.isoformat(), 'protocol': protocol,
            'protocols': protocols_with_data(devices, since, until),
            'sectors': ranges.SECTORS, 'bands': ranges.BANDS, 'sector_deg': ranges.SECTOR_DEG,
            'band_basis': ranges.PROTOCOLS[protocol].basis,
            'band_edges_ft': ranges.band_edges_ft(protocol), 'devices': per_device,
            'swarm': {'max_m': ranges.matrix(swarm), 'sector_max_m': ranges.sector_max(swarm)}}


def stats_series(devices, since: datetime, until: datetime, step: str = 'hour') -> list[dict]:
    """Counts summed over bees per step; peaks (aircraft_max, aircraft_unique) are summed too
    across bees (each bee hears its own sky), maxed within a bee's step. `bees`: how many of them
    reported in that step (a dip may be a quiet bee, not a quiet sky). Steps without a report are
    left out."""
    width = 3600 if step == 'hour' else 86400
    per_bee: dict = defaultdict(lambda: defaultdict(int))
    span = Sample.HOUR if step == 'hour' else Sample.DAY
    for sample in _samples(devices, since, until, span).values('device_id', 'start', *STATS):
        epoch = int(sample['start'].timestamp()) // width * width
        slot = per_bee[(epoch, sample['device_id'])]
        for name in STATS:
            slot[name] = max(slot[name], sample[name]) if name in PEAK_STATS else slot[name] + sample[name]
    totals: dict = defaultdict(lambda: {'bees': 0, **dict.fromkeys(STATS, 0)})
    for (epoch, _device), slot in per_bee.items():
        totals[epoch]['bees'] += 1
        for name in STATS:
            totals[epoch][name] += slot[name]
    return [{'t': datetime.fromtimestamp(epoch, tz.utc).isoformat(), **totals[epoch]} for epoch in sorted(totals)]
