"""
Each bee keeps a daily row beside its hourly ones from the first report (protocol.ingest), so
long windows read days. Retention (run daily: `manage.py swarm_retention`):

1. Every day whose hourly rows are about to go (older than HOURLY_RETENTION_DAYS, whole days)
   has its daily row rebuilt from those hours first: counts summed, peaks and range maxed, listen
   time and frames summed (per protocol; null only when every hour was null).
   A day the ingest path already got right is left as it is; one it did not (rows stored
   before daily rows existed, a crash between the two writes) is fixed and counted.
2. Those hourly rows are deleted.
3. Daily rows older than DAILY_RETENTION_DAYS (if set) are deleted.

`--backfill` rebuilds every day that still has hourly rows, deleting nothing.
"""
from __future__ import annotations

from collections import defaultdict
from datetime import datetime, timedelta

from django.db import transaction
from django.utils import timezone

from . import conf, ranges
from .models import PEAK_STATS, STATS, WITHHELD, Sample, SampleRange

FIELDS = ('reports', *STATS)
#: Per-protocol sums on SampleRange.
COUNTED = ('listen_s', 'frames')


def _day(moment: datetime) -> datetime:
    return moment.replace(hour=0, minute=0, second=0, microsecond=0)


def rebuild_day(device_id: int, day: datetime, hours: list[Sample]) -> bool:
    """The daily row as its hours add up. True when it was missing or different."""
    totals = dict.fromkeys(FIELDS, 0)
    merged: dict[str, ranges.Map] = {}
    units: dict[str, int] = {}
    sums: dict[str, dict[str, int | None]] = {}
    withheld = max((s.range_withheld or '' for s in hours), key=WITHHELD.index, default='')
    for sample in hours:
        totals['reports'] += sample.reports
        for name in STATS:
            value = getattr(sample, name)
            totals[name] = max(totals[name], value) if name in PEAK_STATS else totals[name] + value
        for row in sample.range_rows.all():
            ranges.merge(merged.setdefault(row.protocol, {}), row.ranges())
            units[row.protocol] = max(units.get(row.protocol, 0), row.unit_m)
            totals_row = sums.setdefault(row.protocol, dict.fromkeys(COUNTED))
            for name in COUNTED:
                if getattr(row, name) is not None:
                    totals_row[name] = (totals_row[name] or 0) + getattr(row, name)
    # What the daily rows should hold: each protocol in its coarsest unit, rounded up.
    expected = {protocol: SampleRange(protocol=protocol, unit_m=units[protocol], **sums[protocol])
                for protocol in merged}
    for protocol, row in expected.items():
        row.set_ranges(merged[protocol])
    with transaction.atomic():
        daily = Sample.objects.select_for_update().filter(device_id=device_id, start=day, span_s=Sample.DAY).first()
        stored = {row.protocol: row for row in daily.range_rows.all()} if daily is not None else {}
        same_ranges = stored.keys() == expected.keys() and all(
            stored[p].ranges() == expected[p].ranges() and stored[p].bins == expected[p].bins
            and all(getattr(stored[p], n) == getattr(expected[p], n) for n in COUNTED) for p in expected)
        if daily is not None and same_ranges and daily.range_withheld == withheld \
                and all(getattr(daily, f) == totals[f] for f in FIELDS):
            return False
        if daily is None:
            daily = Sample(device_id=device_id, start=day, span_s=Sample.DAY)
        for name in FIELDS:
            setattr(daily, name, totals[name])
        daily.range_withheld = withheld
        daily.save()
        daily.range_rows.exclude(protocol__in=list(expected)).delete()
        for protocol, row in expected.items():
            SampleRange.objects.update_or_create(sample=daily, protocol=protocol, defaults={
                'unit_m': row.unit_m, 'range_map': row.range_map, 'bins': row.bins,
                **{name: getattr(row, name) for name in COUNTED}})
        return True


def _hours_by_day(rows) -> dict:
    groups: dict = defaultdict(list)
    for sample in rows.order_by('device_id', 'start').prefetch_related('range_rows').iterator(chunk_size=2000):
        groups[(sample.device_id, _day(sample.start))].append(sample)
    return groups


def roll_up(now: datetime | None = None, *, dry_run: bool = False, backfill: bool = False) -> dict:
    now = now or timezone.now()
    hourly = Sample.objects.filter(span_s=Sample.HOUR)
    if not backfill:
        hourly = hourly.filter(start__lt=_day(now - timedelta(days=conf.get('HOURLY_RETENTION_DAYS'))))
    groups = _hours_by_day(hourly)
    days = conf.get('DAILY_RETENTION_DAYS')
    expired = Sample.objects.filter(span_s=Sample.DAY, start__lt=_day(now - timedelta(days=days))) if days else None
    result = {'hours': sum(len(g) for g in groups.values()), 'days': len(groups), 'fixed_days': 0,
              'expired_days': 0 if backfill or expired is None else expired.count()}
    if dry_run:
        return result
    for (device_id, day), samples in groups.items():
        result['fixed_days'] += rebuild_day(device_id, day, samples)
    if backfill:
        return result
    hourly.delete()
    if expired is not None:
        expired.delete()
    return result
