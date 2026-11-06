"""
Old firmware keeps ingesting exactly as before listen_s, frames and the protocol registry.
`golden/v1_bodies.json`: bodies from the 1090's own SwarmReporter::BuildBody (Mode S only, all
three protocols, UAT only, no position, nothing heard, withheld as implausible). `v1_stored.json`:
what the server stored and answered for them before those changes. Regenerate (only on purpose)
with SWARM_GOLDEN_WRITE=1.
"""
import copy
import json
import os
from datetime import datetime, timedelta, timezone as tz
from pathlib import Path

import pytest

from adsbee_swarm.models import Device, Sample

from .conftest import post_report

pytestmark = pytest.mark.django_db

GOLDEN = Path(__file__).parent / 'golden'
SAMPLE_FIELDS = ('span_s', 'reports', 'modes_valid', 'es_valid', 'uat_adsb', 'uat_uplink', 'aircraft_max',
                 'aircraft_unique', 'range_dropped', 'bins_ignored', 'range_withheld')
DEVICE_FIELDS = ('product', 'versions', 'auto_update', 'uptime_s', 'rx_lat', 'rx_lon', 'rx_alt_m', 'rx_source',
                 'range_withheld', 'range_dropped', 'bins_ignored')


def _bodies() -> list[dict]:
    bodies = json.loads((GOLDEN / 'v1_bodies.json').read_text())
    for i, body in enumerate(bodies):
        body['unique_id'] = f'E6614C311B5B{i:04X}'
    legacy = copy.deepcopy(bodies[0])            # older firmware: one mixed `range`
    legacy['unique_id'] = 'E6614C311B5BAAAA'
    legacy['range'] = legacy.pop('ranges')['mode_s']
    widened = copy.deepcopy(bodies[1])           # a failed report retried with a longer period
    widened['period']['len_s'] += 3600
    widened['stats']['modes_valid'] += 1000
    return [*bodies, legacy, widened]


def _state() -> list[dict]:
    out = []
    for device in Device.objects.order_by('unique_id'):
        samples = []
        for sample in Sample.objects.filter(device=device).order_by('span_s'):
            rows = {r.protocol: {'unit_m': r.unit_m, 'map': bytes(r.range_map).hex()}
                    for r in sample.range_rows.order_by('protocol')}
            samples.append({**{f: getattr(sample, f) for f in SAMPLE_FIELDS}, 'ranges': rows})
        out.append({'unique_id': device.unique_id, **{f: getattr(device, f) for f in DEVICE_FIELDS},
                    'check_position': device.check_position, 'samples': samples})
    return out


def test_v1_bodies_ingest_as_before(client, owner, monkeypatch):
    # One fixed hour so hourly and daily rows don't depend on when the test runs.
    now = datetime(2026, 11, 5, 10, 30, tzinfo=tz.utc)
    monkeypatch.setattr('django.utils.timezone.now', lambda: now)
    replies = []
    for body in _bodies():
        response = post_report(client, owner.token, body)
        reply = response.json()
        reply.pop('protocols', None)             # new in the reply; checked in test_protocols
        replies.append([response.status_code, reply])
        now += timedelta(minutes=1)
    got = {'replies': replies, 'stored': _state()}
    path = GOLDEN / 'v1_stored.json'
    if os.environ.get('SWARM_GOLDEN_WRITE'):
        path.write_text(json.dumps(got, indent=1, sort_keys=True) + '\n')
    assert got == json.loads(path.read_text())
