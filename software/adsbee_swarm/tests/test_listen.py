"""
The protocol registry (protocols.json), unknown protocols ignored and counted, listen time
(`listen_s`: listened, heard nothing), `frames`, and the reply's `protocols` (docs/swarm_api.md).
"""
import json
import random
import re
from datetime import timedelta
from io import StringIO
from pathlib import Path

import pytest
import yaml
from django.core.management import call_command
from django.db import connection
from django.db.migrations.executor import MigrationExecutor
from django.utils import timezone

from adsbee_swarm import protocol, queries, ranges, retention
from adsbee_swarm.models import Device, Sample, SampleRange
from adsbee_swarm.simulator import Bee, _wire

from .conftest import post_report

pytestmark = pytest.mark.django_db

ROOT = Path(__file__).parents[1]
MODE_S = {ranges.bin_index(8, 2): 300000}
DRONES = {ranges.bin_index(0, 2): 1200}


def body(bee, maps=None, listen=None, frames=None, **stats) -> dict:
    """A newer firmware report: `maps` {protocol: (values, unit)}, optional listen_s and frames."""
    out = bee.report()
    out.pop('range')
    maps = {'mode_s': (MODE_S, 2000)} if maps is None else maps
    out['ranges'] = {name: _wire(values, unit) for name, (values, unit) in maps.items()}
    if listen is not None:
        out['listen_s'] = listen
    if frames is not None:
        out['frames'] = frames
    out['stats'].update(stats)
    return out


def rows(span=Sample.HOUR) -> dict:
    return {r.protocol: (r.bins, r.listen_s, r.frames) for r in SampleRange.objects.filter(sample__span_s=span)}


def api(client, owner, path, **params):
    response = client.get(f'/api/swarm/v1/{path}', params, HTTP_AUTHORIZATION=f'Bearer {owner.api_token}')
    assert response.status_code == 200, response.content
    return response.json()


# ─── Registry ────────────────────────────────────────────────────────────────

def test_registry_drives_the_tables():
    assert ranges.REPORTED == ('mode_s', 'uat', 'remote_id')
    assert list(ranges.PROTOCOLS) == ['mode_s', 'uat', 'remote_id', 'aircraft']
    assert ranges.SOURCES == {'mode_s': ('mode_s',), 'uat': ('uat',), 'remote_id': ('remote_id',),
                              'aircraft': ('mode_s', 'uat', 'aircraft')}
    assert ranges.FRAMES == ('remote_id',)
    assert ranges.PROTOCOLS['remote_id'].unit_m == 50 and ranges.PROTOCOLS['aircraft'].unit_m is None
    for p in ranges.PROTOCOLS.values():
        assert len(p.floors_ft) == ranges.BANDS and p.floors_ft == sorted(p.floors_ft) and p.top_ft > p.floors_ft[-1]
        assert p.basis in ('altitude', 'height_above_rx') and p.label and p.symbol and p.icon and p.tip
    from adsbee_swarm.views import PROTOCOL_TABS
    assert [key for key, _symbol, _tip in PROTOCOL_TABS] == ['aircraft', 'mode_s', 'uat', 'remote_id']


def _band_tokens(cell: str) -> list[int]:
    def ft(text):
        return int(float(text[:-1]) * 1000) if text.endswith('k') else int(text)
    return [ft(low) for low, _high in re.findall(r'\[([\d.]+k?),([\d.]+k?|∞)\)', cell)]


def test_docs_protocol_table_matches_the_registry():
    docs = (ROOT / 'docs' / 'swarm_api.md').read_text()
    table = docs.split('| Protocol | Positions | `unit_m` | Bands (ft) |')[1].split('\n\n')[0]
    cells = {}
    for line in table.strip().splitlines()[1:]:
        name, _positions, unit, bands = [c.strip() for c in line.strip('|').split('|')]
        cells[name] = (unit, bands)
    registry = json.loads((ROOT / 'adsbee_swarm' / 'protocols.json').read_text())
    reported = [p for p in registry['protocols'] if p['reported']]
    assert [n for n in cells if n.startswith('`') and not n.startswith('`range`')] == \
        [f"`{p['name']}`" for p in reported], 'one row per reported protocol, in registry order'
    for p in reported:
        unit, bands = cells[f"`{p['name']}`"]
        assert unit == str(p['unit_m']), p['name']
        same = re.search(r'as `(\w+)`', bands)
        floors = _band_tokens(cells[f'`{same.group(1)}`'][1]) if same else _band_tokens(bands)
        assert floors == p['floors_ft'], p['name']


def test_openapi_protocol_lists_match_the_registry():
    spec = yaml.safe_load((ROOT / 'adsbee_swarm' / 'openapi.yaml').read_text())
    assert spec['components']['parameters']['protocol']['schema']['enum'] == \
        sorted(ranges.PROTOCOLS, key=lambda n: n != ranges.LEGACY)
    report = spec['components']['schemas']['Report']['properties']
    assert list(report['ranges']['properties']) == list(ranges.REPORTED)


# ─── Unknown protocols ───────────────────────────────────────────────────────

def test_unknown_protocols_are_ignored_and_counted(client, owner, bee):
    report = body(bee, {'mode_s': (MODE_S, 2000), 'ads_l': (MODE_S, 2000)},
                  listen={'mode_s': 3600, 'ads_l': 3600}, frames={'fanet': 12, 'remote_id': 3})
    assert post_report(client, owner.token, report).status_code == 200
    assert rows() == {'mode_s': (1, 3600, None), 'remote_id': (0, None, 3)}
    device = Device.objects.get()
    assert device.protocols_ignored == 2 and device.ignored_protocols == 'ads_l,fanet' and device.check_protocols
    assert not device.check_position
    assert Sample.objects.get(span_s=Sample.HOUR).protocols_ignored == 2
    listed = api(client, owner, 'devices')[0]
    assert listed['check_protocols'] and listed['ignored_protocols'] == ['ads_l', 'fanet']
    assert api(client, owner, 'stats', step='hour')['series'][0]['protocols_ignored'] == 2
    # The next report without them clears the warning.
    assert post_report(client, owner.token, body(bee)).status_code == 200
    assert not Device.objects.get().check_protocols


def test_ui_warns_about_unknown_protocols(client, owner, bee):
    post_report(client, owner.token, body(bee, {'mode_s': (MODE_S, 2000), 'ads_l': (MODE_S, 2000)}))
    client.login(username='alice', password='pw')
    device = Device.objects.get()
    for page in ('/', f'/devices/{device.pk}/'):
        assert "doesn't know (ads_l)" in client.get(page).content.decode(), page


# ─── Listen time ─────────────────────────────────────────────────────────────

def test_listened_heard_nothing_is_stored(client, owner, bee):
    report = body(bee, listen={'mode_s': 3600, 'uat': 3600, 'remote_id': 0})
    assert post_report(client, owner.token, report).status_code == 200
    assert rows() == rows(Sample.DAY) == {'mode_s': (1, 3600, None), 'uat': (0, 3600, None)}
    assert Sample.objects.get(span_s=Sample.HOUR).range_maps() == {'mode_s': MODE_S, 'uat': {}}
    since, until = timezone.now() - timedelta(days=1), timezone.now() + timedelta(hours=1)
    assert queries.protocols_with_data(Device.objects.all(), since, until) == ['mode_s', 'aircraft']


def test_listen_time_is_clamped_to_the_period(client, owner, bee):
    report = body(bee, listen={'mode_s': 9000})
    assert post_report(client, owner.token, report).status_code == 200
    assert rows()['mode_s'][1] == 3600


def test_old_reports_leave_listen_time_null(client, owner, bee):
    assert post_report(client, owner.token, body(bee, {'mode_s': (MODE_S, 2000), 'remote_id': (DRONES, 50)})).status_code == 200
    assert rows() == {'mode_s': (1, None, None), 'remote_id': (1, None, None)}
    payload = api(client, owner, 'range', protocol='uat')
    assert payload['devices'][0]['listened'] is None, 'unknown, so still shown'
    assert payload['listening']['uat'] == {'listened': 0, 'heard': 0}
    assert payload['listening']['mode_s'] == {'listened': 1, 'heard': 1}, 'older firmware: heard counts as listened'


def test_hours_and_days_sum_listen_time_and_frames(client, owner, bee):
    for _ in range(2):
        assert post_report(client, owner.token, body(bee, listen={'mode_s': 1800, 'remote_id': 1800},
                                                    frames={'remote_id': 40, 'mode_s': 5})).status_code == 200
    assert rows() == rows(Sample.DAY) == {'mode_s': (1, 3600, None), 'remote_id': (0, 3600, 80)}
    series = api(client, owner, 'stats', step='hour')['series']
    assert series[0]['frames'] == {'remote_id': 80}


def test_a_widened_retry_replaces_listen_time_and_frames(client, owner, bee):
    first = body(bee, listen={'mode_s': 3600, 'uat': 3600}, frames={'remote_id': 10})
    assert post_report(client, owner.token, first).status_code == 200
    widened = json.loads(json.dumps(first))
    widened['period']['len_s'] = 7200
    widened['listen_s'] = {'mode_s': 7200}
    widened['frames'] = {'remote_id': 25}
    assert post_report(client, owner.token, widened).status_code == 200
    assert rows() == rows(Sample.DAY) == {'mode_s': (1, 7200, None), 'uat': (0, 0, None), 'remote_id': (0, None, 25)}
    assert Sample.objects.get(span_s=Sample.HOUR).reports == 1


def test_retention_keeps_listen_time_and_frames(owner):
    device = Device.objects.create(account=owner.account, product='adsbee_1090', unique_id='E6614C311B5B6F2A')
    bee = Bee.make(0)
    start = (timezone.now() - timedelta(days=120)).replace(hour=1, minute=0, second=0, microsecond=0)
    reports = [bee.report(protocols=True, listen=('mode_s', 'uat', 'remote_id'), quiet=('uat',)),
               bee.report(protocols=True), bee.report(protocols=True, listen=('mode_s',))]
    reports[0]['frames'] = {'remote_id': 7}
    for hour, report in enumerate(reports):
        protocol.ingest(device, protocol.parse(report), start + timedelta(hours=hour))
    assert retention.roll_up(backfill=True)['fixed_days'] == 0, 'ingest and retention agree'
    daily = {r.protocol: (r.listen_s, r.frames) for r in SampleRange.objects.filter(sample__span_s=Sample.DAY)}
    # Rebuilt from the hours, a day keeps the same sums.
    SampleRange.objects.filter(sample__span_s=Sample.DAY).delete()
    call_command('swarm_retention', stdout=StringIO())
    assert {r.protocol: (r.listen_s, r.frames) for r in SampleRange.objects.filter(sample__span_s=Sample.DAY)} == daily
    assert daily['mode_s'] == (7200, None) and daily['uat'] == (3600, None) and daily['remote_id'] == (3600, 7)
    assert not Sample.objects.filter(span_s=Sample.HOUR).exists()


# ─── Reply and read side ─────────────────────────────────────────────────────

def test_reply_lists_the_protocols_this_server_takes(client, owner, bee):
    reply = post_report(client, owner.token, bee.report()).json()
    assert reply['protocols'] == ['mode_s', 'uat', 'remote_id']


def test_swarm_view_shows_only_bees_that_listened(client, owner):
    europe, us, old = Bee.make(0), Bee.make(1, seed=4), Bee.make(2, seed=5)
    reports = [europe.report(rng=random.Random(1), protocols=True, listen=('mode_s', 'uat'), quiet=('uat',)),
               us.report(rng=random.Random(2), protocols=True, listen=('mode_s', 'uat', 'remote_id')),
               old.report(rng=random.Random(3), protocols=True)]
    assert all(name in r['ranges'] for r in reports[1:] for name in ('uat', 'remote_id')), 'seeds hear UAT and drones'
    for report in reports:
        assert post_report(client, owner.token, report).status_code == 200
    ids = {d.unique_id: d.pk for d in Device.objects.all()}
    drones = api(client, owner, 'range', protocol='remote_id')
    assert {d['id'] for d in drones['devices']} == {ids[us.unique_id], ids[old.unique_id]}, 'Europe had RID off'
    assert drones['listening']['remote_id'] == {'listened': 2, 'heard': 2}
    uat = api(client, owner, 'range', protocol='uat')
    by_id = {d['id']: d for d in uat['devices']}
    quiet = by_id[ids[europe.unique_id]]
    assert quiet['listened'] is True and quiet['heard'] is False and quiet['listen_s'] == 3600 and quiet['bins'] == 0
    assert by_id[ids[old.unique_id]]['listened'] is None
    assert uat['listening']['uat'] == {'listened': 3, 'heard': 2}
    union = {d['id']: d for d in api(client, owner, 'range')['devices']}
    assert union[ids[us.unique_id]]['listen_s'] == 3600, 'Mode S and UAT side by side: not 7200'


def test_ui_picker_and_bee_page_say_listened_heard_nothing(client, owner):
    call_command('swarm_simulate', 'alice', '--bees', '2', '--hours', '3', '--listen', stdout=StringIO())
    client.login(username='alice', password='pw')
    page = client.get('/?w=24h&p=uat').content.decode()
    assert 'Aircraft on 978 MHz (UAT ADS-B): 2 listened, 1 heard' in page and '1/2' in page
    quiet = Device.objects.order_by('pk')[1]
    bee_page = client.get(f'/devices/{quiet.pk}/?w=24h&p=uat').content.decode()
    assert 'listened 3.0 h' in bee_page and 'heard nothing' in bee_page


# ─── Migration ───────────────────────────────────────────────────────────────

@pytest.mark.django_db(transaction=True)
def test_migration_counts_bins_and_leaves_listen_time_null():
    executor = MigrationExecutor(connection)
    executor.migrate([('adsbee_swarm', '0005_enroll_tokens')])
    apps = executor.loader.project_state([('adsbee_swarm', '0005_enroll_tokens')]).apps
    User = apps.get_model('auth', 'User')
    account = apps.get_model('adsbee_swarm', 'Account').objects.create(user=User.objects.create(username='old'))
    device = apps.get_model('adsbee_swarm', 'Device').objects.create(account=account, product='adsbee_1090',
                                                                      unique_id='0000000000000009')
    sample = apps.get_model('adsbee_swarm', 'Sample').objects.create(device=device, start=timezone.now(), span_s=3600)
    apps.get_model('adsbee_swarm', 'SampleRange').objects.create(
        sample=sample, protocol='mode_s', unit_m=2000, range_map=ranges.pack({4: 6000, 5: 2000, 300: 10000}, 2000))
    executor = MigrationExecutor(connection)
    executor.loader.build_graph()
    executor.migrate(executor.loader.graph.leaf_nodes())
    row = SampleRange.objects.get()
    assert (row.bins, row.listen_s, row.frames) == (3, None, None)
    assert row.ranges() == {4: 6000, 5: 2000, 300: 10000}


def _migration_0006():
    import importlib
    return importlib.import_module('adsbee_swarm.migrations.0006_listen_time')


@pytest.mark.skipif(connection.vendor != 'postgresql', reason='the set-based path is PostgreSQL only')
def test_bins_sql_matches_python_on_the_same_rows():
    """0006's table rewrite (bit_count from PG 14, the bit-string fallback before) gives each row the bins the
    Python path gives: maps from the packer, random bytes, short, empty and all-ones bitmaps."""
    from types import SimpleNamespace

    from django.contrib.auth import get_user_model
    from adsbee_swarm.models import Account

    mig = _migration_0006()
    account = Account.objects.create(user=get_user_model().objects.create(username='bins'))
    device = Device.objects.create(account=account, product='adsbee_1090', unique_id='00000000000000B1')
    sample = Sample.objects.create(device=device, start=timezone.now(), span_s=3600)
    rng = random.Random(7)
    maps = [b'', bytes(10), b'\xff' * 54, b'\xff' * 80, bytes(54) + b'\xff' * 20,
            ranges.pack({0: 2000, 431: 4000}, 2000), ranges.pack({b: 2000 for b in range(0, 432, 3)}, 2000)]
    maps += [rng.randbytes(rng.randrange(0, 120)) for _ in range(200)]
    for i, blob in enumerate(maps):
        SampleRange.objects.create(sample=sample, protocol=f'p{i}', unit_m=2000, range_map=blob)

    def bins():
        return dict(SampleRange.objects.filter(sample=sample).values_list('protocol', 'bins'))

    mig.count_bins_python(SampleRange)
    expected = bins()
    assert expected['p2'] == 432 and expected['p3'] == 432 and expected['p4'] == 0 and expected['p0'] == 0
    assert expected['p5'] == 2
    table = SampleRange._meta.db_table
    for version in (connection.pg_version, 130000):     # this server, and a PG 13 one (the fallback)
        SampleRange.objects.update(bins=0)
        sql = mig.count_bins_sql(SimpleNamespace(pg_version=version, ops=connection.ops), table,
                                 SampleRange._meta.get_field('bins').db_type(connection))
        assert ('bit_count' in sql) == (version >= 140000) and 'TYPE smallint USING' in sql
        if version >= 140000 and connection.pg_version < 140000:
            continue
        with connection.cursor() as cursor:
            cursor.execute('SET CONSTRAINTS ALL IMMEDIATE')   # this test's own inserts; the migration has none
            cursor.execute(sql)
        assert bins() == expected, version
