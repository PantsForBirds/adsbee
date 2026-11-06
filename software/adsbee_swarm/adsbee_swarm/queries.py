"""The read side: range maps and stats over a time window, for one bee or an account's swarm."""
from __future__ import annotations

from collections import defaultdict
from datetime import datetime, timedelta, timezone as tz


from django.db.models import Max, Sum

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
    """Which `ranges.SOURCES` keys have a map with bins in the window, in `PROTOCOLS` order."""
    stored = set(_range_rows(devices, since, until).filter(bins__gt=0).values_list('protocol', flat=True).distinct())
    return [name for name in ranges.PROTOCOLS if stored & set(ranges.SOURCES[name])]


def reception(devices, since: datetime, until: datetime) -> dict[int, dict[str, dict]]:
    """{device id: {stored protocol: {'listen_s': seconds or None (not reported), 'bins': most bins
    in a row, 'frames': count or None}}} over the window."""
    out: dict[int, dict[str, dict]] = defaultdict(dict)
    rows = _range_rows(devices, since, until).values('sample__device_id', 'protocol').annotate(
        listen_s=Sum('listen_s'), bins=Max('bins'), frames=Sum('frames'))
    for row in rows:
        out[row['sample__device_id']][row['protocol']] = {k: row[k] for k in ('listen_s', 'bins', 'frames')}
    return out


def reception_of(stored: dict[str, dict], protocol: str) -> dict:
    """One bee's reception of a user-facing protocol (`aircraft`: over its sources).
    `listened`: True, False (it reports listen time, none for this), None (unknown: older firmware)."""
    rows = [stored[name] for name in ranges.SOURCES[protocol] if name in stored]
    listen = [r['listen_s'] for r in rows if r['listen_s'] is not None]
    frames = [r['frames'] for r in rows if r['frames'] is not None]
    reports_listen = any(r['listen_s'] is not None for r in stored.values())
    # A union (`aircraft`) listened as long as its longest source: receivers run side by side.
    return {'listen_s': max(listen) if listen else None,
            'listened': bool(sum(listen)) if listen else (False if reports_listen else None),
            'heard': any(r['bins'] for r in rows), 'frames': sum(frames) if frames else None}


def listening(devices, since: datetime, until: datetime, received: dict | None = None) -> dict[str, dict]:
    """Per user-facing protocol: how many of these bees listened to it and how many heard
    something. Bees on older firmware (no listen time) count as listened when they heard it."""
    received = reception(devices, since, until) if received is None else received
    out = {}
    for name in ranges.PROTOCOLS:
        seen = [reception_of(received.get(d.pk, {}), name) for d in devices]
        out[name] = {'listened': sum(1 for s in seen if s['listened'] or (s['listened'] is None and s['heard'])),
                     'heard': sum(1 for s in seen if s['heard'])}
    return out


def range_payload(device: Device, merged: ranges.Map, reception_: dict | None = None) -> dict:
    reception_ = reception_ or {'listen_s': None, 'listened': None, 'heard': bool(merged), 'frames': None}
    return {'id': device.pk, 'label': device.label, 'unique_id': device.unique_id,
            'position': device.position, 'check_position': device.check_position,
            'check_protocols': device.check_protocols, 'max_m': ranges.matrix(merged),
            'sector_max_m': ranges.sector_max(merged), 'bins': len(merged),
            'max_range_m': max(merged.values(), default=None), **reception_}


def swarm_range(devices, since: datetime, until: datetime, protocol: str = ranges.LEGACY) -> dict:
    """Per bee its bands x sectors maxima for `protocol`; `swarm` is the max per bin over all of
    them. A bee that reports listen time and didn't listen to `protocol` is left out; older
    firmware (no listen time) is always in. `protocols`: the ones with data for these bees in the
    window; `listening`: per protocol, how many bees listened and how many heard."""
    devices = list(devices)
    received = reception(devices, since, until)
    per_device = []
    swarm: ranges.Map = {}
    for device in devices:
        state = reception_of(received.get(device.pk, {}), protocol)
        if state['listened'] is False and not state['heard']:
            continue
        merged = device_range(device, since, until, protocol)
        ranges.merge(swarm, merged)
        per_device.append(range_payload(device, merged, state))
    return {'since': since.isoformat(), 'until': until.isoformat(), 'protocol': protocol,
            'protocols': protocols_with_data(devices, since, until),
            'listening': listening(devices, since, until, received),
            'sectors': ranges.SECTORS, 'bands': ranges.BANDS, 'sector_deg': ranges.SECTOR_DEG,
            'band_basis': ranges.PROTOCOLS[protocol].basis,
            'band_edges_ft': ranges.band_edges_ft(protocol), 'devices': per_device,
            'swarm': {'max_m': ranges.matrix(swarm), 'sector_max_m': ranges.sector_max(swarm)}}


def stats_series(devices, since: datetime, until: datetime, step: str = 'hour') -> list[dict]:
    """Counts summed over bees per step; peaks (aircraft_max, aircraft_unique) are summed too
    across bees (each bee hears its own sky), maxed within a bee's step. `bees`: how many of them
    reported in that step (a dip may be a quiet bee, not a quiet sky). `frames`: per protocol
    without legacy stats, when any were reported. Steps without a report are left out."""
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
    frames: dict = defaultdict(lambda: defaultdict(int))
    for row in SampleRange.objects.filter(sample__in=_samples(devices, since, until, span).values('pk'),
                                          frames__isnull=False).values('sample__start', 'protocol', 'frames'):
        frames[int(row['sample__start'].timestamp()) // width * width][row['protocol']] += row['frames']
    return [{'t': datetime.fromtimestamp(epoch, tz.utc).isoformat(), **totals[epoch],
             **({'frames': dict(frames[epoch])} if epoch in frames else {})} for epoch in sorted(totals)]
