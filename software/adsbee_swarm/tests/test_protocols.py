"""Per-protocol range maps: `ranges` (mode_s, uat, remote_id) beside old firmware's `range`."""
from datetime import timedelta
from io import StringIO

import pytest
from django.core.management import call_command
from django.db import connection
from django.db.migrations.executor import MigrationExecutor
from django.utils import timezone

from adsbee_swarm import protocol, queries, ranges, retention
from adsbee_swarm.models import Device, Sample, SampleRange
from adsbee_swarm.simulator import Bee, _wire

from .conftest import post_report, save_sample

pytestmark = pytest.mark.django_db

MODE_S = {ranges.bin_index(8, 2): 300000, ranges.bin_index(0, 2): 20000}
UAT = {ranges.bin_index(8, 2): 150000, ranges.bin_index(3, 5): 90000}
DRONES = {ranges.bin_index(0, 2): 1200, ranges.bin_index(11, 30): 12750}


def per_protocol(bee, mode_s=MODE_S, uat=UAT, remote_id=DRONES) -> dict:
    body = bee.report()
    body.pop('range')
    body['ranges'] = {name: _wire(values, unit) for name, values, unit in
                      (('mode_s', mode_s, 2000), ('uat', uat, 2000), ('remote_id', remote_id, 50)) if values}
    return body


def stored(span=Sample.HOUR) -> dict:
    return Sample.objects.get(span_s=span).range_maps()


def api_range(client, owner, **params):
    response = client.get('/api/swarm/v1/range', params, HTTP_AUTHORIZATION=f'Bearer {owner.api_token}')
    return response.status_code, response.json()


# ─── Reports ─────────────────────────────────────────────────────────────────

def test_old_firmware_range_is_stored_as_aircraft(client, owner, bee):
    body = bee.report()
    assert post_report(client, owner.token, body).status_code == 200
    expected = ranges.decode(body['range']['changed'], body['range']['max'], 2000)
    assert stored() == stored(Sample.DAY) == {'aircraft': expected}


def test_new_reports_store_one_map_per_protocol_hourly_and_daily(client, owner, bee):
    assert post_report(client, owner.token, per_protocol(bee)).status_code == 200
    assert stored() == stored(Sample.DAY) == {'mode_s': MODE_S, 'uat': UAT, 'remote_id': DRONES}
    assert SampleRange.objects.get(protocol='remote_id', sample__span_s=Sample.HOUR).unit_m == 50


def test_empty_ranges_is_a_report_with_no_bins(client, owner, bee):
    body = bee.report()
    body.pop('range')
    body['ranges'] = {}
    assert post_report(client, owner.token, body).status_code == 200
    sample = Sample.objects.get(span_s=Sample.HOUR)
    assert sample.range_maps() == {} and sample.reports == 1 and sample.range_withheld == ''


def test_a_protocol_with_no_bins_stores_nothing(client, owner, bee):
    body = per_protocol(bee, uat={}, remote_id={})
    body['ranges']['uat'] = _wire({}, 2000)
    assert post_report(client, owner.token, body).status_code == 200
    assert stored() == {'mode_s': MODE_S}


def test_without_a_position_ranges_are_dropped(client, owner, bee):
    body = per_protocol(bee)
    body['rx_position'] = None
    assert post_report(client, owner.token, body).status_code == 200
    assert stored() == {} and Sample.objects.get(span_s=Sample.HOUR).range_withheld == 'no_position'


def test_a_widened_retry_max_merges_each_protocol(client, owner, bee):
    body = per_protocol(bee, uat={})
    assert post_report(client, owner.token, body).status_code == 200
    wider = per_protocol(bee, mode_s={ranges.bin_index(8, 2): 100000, 7: 6000}, remote_id={})
    wider.update(seq=body['seq'], period={**body['period'], 'len_s': body['period']['len_s'] + 600})
    assert post_report(client, owner.token, wider).status_code == 200
    maps = stored()
    assert maps['mode_s'] == {**MODE_S, 7: 6000} and maps['uat'] == UAT and maps['remote_id'] == DRONES
    assert Sample.objects.get(span_s=Sample.HOUR).reports == 1


def _mutate(body, how):
    how(body)
    return body


@pytest.mark.parametrize('change, detail', [
    (lambda b: b.update(range=b['ranges']['mode_s']), 'range or ranges, not both'),
    (lambda b: b.update(ranges=[1]), 'ranges must be an object'),
    (lambda b: b['ranges'].update({'ADS-C': b['ranges']['uat']}), 'protocol names are 1-16 of a-z'),
    (lambda b: b['ranges'].update({f'p{i}': {} for i in range(14)}), 'more than 16 protocols'),
    (lambda b: b.update(listen_s=[1]), 'listen_s must be an object'),
    (lambda b: b.update(listen_s={'mode_s': -1}), 'listen_s.mode_s must be an integer 0..604800'),
    (lambda b: b.update(frames={'remote_id': 'x'}), 'frames.remote_id must be an integer'),
    (lambda b: b['ranges']['mode_s'].update(unit_m=500), 'ranges.mode_s.unit_m must be 2000'),
    (lambda b: b['ranges']['remote_id'].update(unit_m=2000), 'ranges.remote_id.unit_m must be 50'),
    (lambda b: b['ranges']['uat'].update(unit_m='x'), 'ranges.uat.unit_m must be an integer'),
    (lambda b: b['ranges']['uat'].update(bands=10), 'ranges.uat must be 36 sectors x 12 bands'),
    (lambda b: b['ranges']['remote_id'].update(max='!!'), 'ranges.remote_id: changed and max must be base64'),
    (lambda b: b['ranges'].update(uat='x'), 'ranges.uat must be an object'),
])
def test_malformed_ranges_are_400(client, owner, bee, change, detail):
    response = post_report(client, owner.token, _mutate(per_protocol(bee), change))
    assert response.status_code == 400 and detail in response.json()['detail']
    assert not Sample.objects.exists()


def test_old_range_unit_is_still_free(client, owner, bee):
    body = bee.report()
    body['range'] = _wire({7: 1500}, 500)
    assert post_report(client, owner.token, body).status_code == 200
    assert stored() == {'aircraft': {7: 1500}}


# ─── Horizon ─────────────────────────────────────────────────────────────────

def test_horizon_limit_uses_each_protocols_band_tops():
    def limit(top_ft):
        return (1.15 * 4.12 * (10 + (top_ft * 0.3048) ** .5) + 20) * 1000
    assert ranges.horizon_limit_m(0, 100, 'mode_s') == pytest.approx(limit(1000))
    assert ranges.horizon_limit_m(11, 100, 'uat') == pytest.approx(limit(60000))
    assert ranges.horizon_limit_m(0, 100, 'remote_id') == pytest.approx(limit(100))
    assert ranges.horizon_limit_m(10, 100, 'remote_id') == pytest.approx(limit(5000))
    assert ranges.horizon_limit_m(11, 100, 'remote_id') == pytest.approx(limit(10000))
    assert ranges.horizon_limit_m(0, 100) == ranges.horizon_limit_m(0, 100, 'aircraft')


def test_bins_past_the_horizon_are_ignored_per_protocol(client, owner, bee):
    bee.alt_m = 0
    # 0 m antenna, band 0 (top 1000 ft): about 66 km for aircraft.
    body = per_protocol(bee, mode_s={0: 60000, 1: 120000}, uat={2: 200000, 400: 200000},
                        remote_id={ranges.bin_index(0, 3): 12750})
    assert post_report(client, owner.token, body).status_code == 200
    maps = stored()
    assert maps == {'mode_s': {0: 60000}, 'uat': {400: 200000}, 'remote_id': {ranges.bin_index(0, 3): 12750}}
    device = Device.objects.get()
    assert device.bins_ignored == 2 and device.check_position
    # Remote ID band 0 (top 100 ft) at a 0 m antenna: about 46 km, so a clamped 12.75 km drone stays.
    kept, ignored = ranges.within_horizon({0: 12750, 1: 50000}, 0, 50, 'remote_id')
    assert kept == {0: 12750} and ignored == 1


# ─── Retention ───────────────────────────────────────────────────────────────

@pytest.fixture
def device(owner):
    return Device.objects.create(account=owner.account, product='adsbee_1090', unique_id='E6614C311B5B6F2A')


def _hour(device, start, maps: dict, **stats):
    sample = save_sample(Sample(device=device, start=start, reports=1, **stats))
    for name, (values, unit) in maps.items():
        row = SampleRange(sample=sample, protocol=name, unit_m=unit)
        row.set_ranges(values)
        row.save()
    return sample


def test_retention_rolls_up_each_protocol_then_deletes_hours(device):
    day = (timezone.now() - timedelta(days=120)).replace(hour=0, minute=0, second=0, microsecond=0)
    _hour(device, day + timedelta(hours=1), {'aircraft': ({0: 10000}, 2000), 'remote_id': ({5: 400}, 50)})
    _hour(device, day + timedelta(hours=2), {'mode_s': ({0: 20000}, 2000), 'remote_id': ({5: 900, 6: 50}, 50)})
    call_command('swarm_retention', stdout=StringIO())
    daily = Sample.objects.get(span_s=Sample.DAY)
    assert daily.range_maps() == {'aircraft': {0: 10000}, 'mode_s': {0: 20000}, 'remote_id': {5: 900, 6: 50}}
    assert not Sample.objects.filter(span_s=Sample.HOUR).exists()
    assert not SampleRange.objects.filter(sample__span_s=Sample.HOUR).exists(), "range rows go with their hours"


def test_a_day_missing_or_extra_protocol_is_fixed(device):
    day = (timezone.now() - timedelta(days=3)).replace(hour=0, minute=0, second=0, microsecond=0)
    hour = _hour(device, day + timedelta(hours=4), {'uat': ({1: 4000}, 2000), 'remote_id': ({2: 100}, 50)})
    wrong = _hour(device, day, {'uat': ({1: 4000}, 2000), 'mode_s': ({9: 9000}, 2000)})
    Sample.objects.filter(pk=wrong.pk).update(span_s=Sample.DAY)
    assert retention.roll_up(backfill=True)['fixed_days'] == 1
    assert Sample.objects.get(span_s=Sample.DAY).range_maps() == {'uat': {1: 4000}, 'remote_id': {2: 100}}
    assert retention.rebuild_day(device.pk, day, [hour]) is False, "now it adds up"


def test_ingest_and_retention_agree_on_new_reports(device):
    bee = Bee.make(0)
    start = timezone.now().replace(hour=2, minute=0, second=0, microsecond=0)
    for hour in range(3):
        protocol.ingest(device, protocol.parse(bee.report(protocols=True)), start + timedelta(hours=hour))
    protocol.ingest(device, protocol.parse(bee.report()), start + timedelta(hours=3))
    assert retention.roll_up(backfill=True)['fixed_days'] == 0


# ─── User API ────────────────────────────────────────────────────────────────

def test_api_protocol_filter_and_protocol_list(client, owner, bee):
    assert post_report(client, owner.token, per_protocol(bee)).status_code == 200
    status, everything = api_range(client, owner)
    assert status == 200 and everything['protocol'] == 'aircraft'
    assert everything['protocols'] == ['mode_s', 'uat', 'remote_id', 'aircraft']
    grid = everything['swarm']['max_m']
    assert grid[8][2] == 300000 and grid[3][5] == 90000 and grid[0][2] == 20000, "max over Mode S and UAT"
    assert grid[11][30] is None, "Remote ID never mixes into aircraft"
    _, uat = api_range(client, owner, protocol='uat')
    assert uat['swarm']['max_m'][8][2] == 150000 and uat['swarm']['max_m'][0][2] is None
    _, drones = api_range(client, owner, protocol='remote_id')
    assert drones['band_basis'] == 'height_above_rx' and everything['band_basis'] == 'altitude'
    assert drones['band_edges_ft'][0] == [0, 100] and drones['band_edges_ft'][-1] == [5000, None]
    assert drones['devices'][0]['max_m'][0][2] == 1200 and drones['devices'][0]['max_range_m'] == 12750
    status, refused = api_range(client, owner, protocol='adsc')
    assert status == 400 and 'protocol is mode_s, uat, remote_id, aircraft' in refused['detail']


def test_api_mixed_history(client, owner, bee):
    now = timezone.now()
    device = Device.objects.create(account=owner.account, product=bee.product, unique_id=bee.unique_id)
    old = bee.report()
    old['range'] = _wire({ranges.bin_index(8, 2): 400000, 3: 4000}, 2000)
    protocol.store(device, protocol.parse(old), now - timedelta(hours=5))
    protocol.store(device, protocol.parse(per_protocol(bee, remote_id={})), now - timedelta(hours=1))
    _, everything = api_range(client, owner)
    assert everything['protocols'] == ['mode_s', 'uat', 'aircraft']
    assert everything['swarm']['max_m'][8][2] == 400000 and everything['swarm']['max_m'][0][3] == 4000
    _, mode_s = api_range(client, owner, protocol='mode_s')
    assert mode_s['swarm']['max_m'][8][2] == 300000, "old firmware's map is only under aircraft"
    _, recent = api_range(client, owner, since=(now - timedelta(hours=3)).isoformat())
    assert recent['swarm']['max_m'][8][2] == 300000
    _, drones = api_range(client, owner, protocol='remote_id')
    assert drones['swarm']['sector_max_m'] == [None] * 36


def test_api_lists_only_the_old_protocol_for_old_firmware(client, owner, bee):
    post_report(client, owner.token, bee.report())
    _, payload = api_range(client, owner)
    assert payload['protocols'] == ['aircraft']
    _, mode_s = api_range(client, owner, protocol='mode_s')
    assert mode_s['protocols'] == ['aircraft'] and mode_s['devices'][0]['bins'] == 0


def test_api_protocols_follow_the_chosen_bees(client, owner, other, bee):
    post_report(client, owner.token, bee.report())
    second = Bee.make(1, seed=7)
    post_report(client, owner.token, per_protocol(second, uat={}, remote_id={}))
    first = Device.objects.get(unique_id=bee.unique_id)
    _, payload = api_range(client, owner, device=first.pk)
    assert payload['protocols'] == ['aircraft']
    post_report(client, other.token, per_protocol(Bee.make(2)))
    _, mine = api_range(client, owner)
    assert mine['protocols'] == ['mode_s', 'aircraft'], "another account's Remote ID does not show"


# ─── Reference UI ────────────────────────────────────────────────────────────

def test_ui_protocol_picker(client, owner):
    call_command('swarm_simulate', 'alice', '--bees', '2', '--hours', '6', stdout=StringIO())
    client.login(username='alice', password='pw')
    page = client.get('/?w=24h').content.decode()
    for key in ('aircraft', 'mode_s', 'uat', 'remote_id'):
        assert f'p={key}' in page
    device = Device.objects.first()
    drones = client.get(f'/devices/{device.pk}/?w=24h&p=remote_id').content.decode()
    assert '↑ 0–100 ft' in drones and '↑ ≥5000 ft' in drones and 'Range by height' in drones
    assert 'href="?w=7d&amp;p=remote_id"' in drones, "window links keep the protocol"


def test_ui_hides_the_picker_for_old_firmware(client, owner):
    call_command('swarm_simulate', 'alice', '--bees', '1', '--hours', '3', '--legacy', stdout=StringIO())
    client.login(username='alice', password='pw')
    page = client.get('/?w=24h').content.decode()
    assert 'p=mode_s' not in page and 'class="protocols"' not in page


# ─── Migration ───────────────────────────────────────────────────────────────

@pytest.mark.django_db(transaction=True)
def test_migration_moves_maps_to_aircraft_rows_and_back():
    executor = MigrationExecutor(connection)
    executor.migrate([('adsbee_swarm', '0002_position_guards')])
    apps = executor.loader.project_state([('adsbee_swarm', '0002_position_guards')]).apps
    User = apps.get_model('auth', 'User')
    Account = apps.get_model('adsbee_swarm', 'Account')
    OldDevice = apps.get_model('adsbee_swarm', 'Device')
    OldSample = apps.get_model('adsbee_swarm', 'Sample')
    account = Account.objects.create(user=User.objects.create(username='old'))
    device = OldDevice.objects.create(account=account, product='adsbee_1090', unique_id='0000000000000009')
    now = timezone.now().replace(minute=0, second=0, microsecond=0)
    for span in (Sample.HOUR, Sample.DAY):
        OldSample.objects.create(device=device, start=now, span_s=span, unit_m=500, range_map=ranges.pack({4: 1500}, 500))
    OldSample.objects.create(device=device, start=now - timedelta(hours=1), span_s=Sample.HOUR)

    executor = MigrationExecutor(connection)
    executor.loader.build_graph()
    executor.migrate([('adsbee_swarm', '0004_remove_sample_range_map')])
    # The models as of 0004 (later migrations add columns).
    apps = executor.loader.project_state([('adsbee_swarm', '0004_remove_sample_range_map')]).apps
    Range4, Sample4 = apps.get_model('adsbee_swarm', 'SampleRange'), apps.get_model('adsbee_swarm', 'Sample')
    rows = Range4.objects.order_by('sample__span_s')
    assert [(r.protocol, r.unit_m, ranges.unpack(r.range_map, r.unit_m), r.sample.span_s) for r in rows] == [
        ('aircraft', 500, {4: 1500}, Sample.HOUR), ('aircraft', 500, {4: 1500}, Sample.DAY)]

    sample = Sample4.objects.get(span_s=Sample.HOUR, start=now)
    Range4.objects.create(sample=sample, protocol='mode_s', unit_m=2000, range_map=ranges.pack({4: 6000, 5: 2000}, 2000))
    executor = MigrationExecutor(connection)
    executor.loader.build_graph()
    executor.migrate([('adsbee_swarm', '0002_position_guards')])
    apps = executor.loader.project_state([('adsbee_swarm', '0002_position_guards')]).apps
    back = apps.get_model('adsbee_swarm', 'Sample').objects.get(span_s=Sample.HOUR, start=now)
    assert back.unit_m == 2000 and ranges.unpack(back.range_map, back.unit_m) == {4: 6000, 5: 2000}

    executor = MigrationExecutor(connection)
    executor.loader.build_graph()
    executor.migrate(executor.loader.graph.leaf_nodes())
