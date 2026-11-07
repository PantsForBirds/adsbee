"""
One Mode S count: all valid 56- and 112-bit frames. Newer firmware sends `frames.mode_s`; older
firmware `modes_valid` and `es_valid`, stored as before. Every read shows the total.
"""
import json
from datetime import timedelta
from pathlib import Path

import pytest
import yaml
from django.contrib.auth import get_user_model
from django.utils import timezone

from adsbee_swarm import protocol, queries, ranges, retention
from adsbee_swarm.models import Device, Sample, SampleRange
from adsbee_swarm.simulator import Bee

from .conftest import post_report

pytestmark = pytest.mark.django_db

ROOT = Path(__file__).parents[1]


@pytest.fixture
def device(owner):
    return Device.objects.create(account=owner.account, product='adsbee_1090', unique_id='E6614C311B5B6F2A')


def old(bee, modes=1200, es=500):
    """Firmware before the total: per-protocol maps, Mode S as modes_valid + es_valid."""
    body = bee.report(protocols=True)
    body['stats'].update(modes_valid=modes, es_valid=es)
    return body


def new(bee, mode_s=1700):
    """Firmware with the total: frames.mode_s, no modes_valid / es_valid."""
    body = bee.report(protocols=True, listen=('mode_s', 'uat', 'remote_id'), mode_s_frames=True)
    body['frames']['mode_s'] = mode_s
    return body


def stats(client, owner, **params):
    response = client.get('/api/swarm/v1/stats', params, HTTP_AUTHORIZATION=f'Bearer {owner.api_token}')
    assert response.status_code == 200, response.content
    return response.json()['series']


def test_the_registry_says_mode_s_sends_frames_and_keeps_its_old_columns():
    registry = json.loads((ROOT / 'adsbee_swarm' / 'protocols.json').read_text())
    entry = next(p for p in registry['protocols'] if p['name'] == 'mode_s')
    assert entry['stats'] == [] and entry['legacy_stats'] == ['modes_valid', 'es_valid']
    assert next(p for p in registry['protocols'] if p['name'] == 'uat')['stats'] == ['uat_adsb', 'uat_uplink']
    # The firmware's registry test cuts each entry at its first '}': entries stay flat.
    for p in registry['protocols']:
        assert not any(isinstance(v, dict) for v in p.values()), p['name']


def test_the_simulator_sends_the_total_as_newer_firmware():
    body = new(Bee.make(0))
    assert body['frames'] == {'mode_s': 1700} and 'modes_valid' not in body['stats'] and 'es_valid' not in body['stats']
    assert body['stats']['uat_adsb'] >= 0, 'UAT keeps its stats keys'


def test_a_new_report_stores_frames_mode_s(client, owner, bee):
    assert post_report(client, owner.token, new(bee, 1700)).status_code == 200
    hour = Sample.objects.get(span_s=Sample.HOUR)
    assert (hour.modes_valid, hour.es_valid) == (0, 0)
    assert SampleRange.objects.get(sample=hour, protocol='mode_s').frames == 1700
    series = stats(client, owner)
    assert series[0]['mode_s'] == 1700 and series[0]['frames']['mode_s'] == 1700


def test_an_old_report_is_stored_as_before_and_totals_both_columns(client, owner, bee):
    assert post_report(client, owner.token, old(bee, 1200, 500)).status_code == 200
    hour = Sample.objects.get(span_s=Sample.HOUR)
    assert (hour.modes_valid, hour.es_valid) == (1200, 500)
    assert SampleRange.objects.get(sample=hour, protocol='mode_s').frames is None
    row = stats(client, owner)[0]
    assert (row['mode_s'], row['modes_valid'], row['es_valid']) == (1700, 1200, 500) and 'frames' not in row


def test_a_report_with_both_counts_mode_s_once(client, owner, bee):
    body = new(bee, 1700)
    body['stats'].update(modes_valid=1200, es_valid=500)
    assert post_report(client, owner.token, body).status_code == 200
    assert stats(client, owner)[0]['mode_s'] == 1700


def test_a_bee_updated_mid_hour_and_old_and_new_bees_add_up_in_the_hour_and_day(client, owner):
    updated, legacy, current = Bee.make(0, seed=1), Bee.make(1, seed=1), Bee.make(2, seed=1)
    # One hour: the updated bee reports twice (old firmware, then new), the others once each.
    for body in (old(updated, 100, 50), new(updated, 30), old(legacy, 1000, 400), new(current, 2000)):
        assert post_report(client, owner.token, body).status_code == 200
    hourly = stats(client, owner, step='hour')
    assert len(hourly) == 1 and hourly[0]['bees'] == 3
    assert hourly[0]['mode_s'] == 150 + 30 + 1400 + 2000
    assert (hourly[0]['modes_valid'], hourly[0]['es_valid'], hourly[0]['frames']['mode_s']) == (1100, 450, 2030)
    daily = stats(client, owner, step='day')
    assert [row['mode_s'] for row in daily] == [3580], 'the daily rows add up the same'


def test_a_day_rolled_up_from_old_and_new_hours_keeps_the_total(device):
    bee = Bee.make(0)
    day = (timezone.now() - timedelta(days=120)).replace(hour=0, minute=0, second=0, microsecond=0)
    hours = [old(bee, 100, 50), old(bee, 200, 60), new(bee, 400), new(bee, 500)]
    for hour, body in enumerate(hours):
        protocol.ingest(device, protocol.parse(body), day + timedelta(hours=hour))
    assert retention.roll_up(backfill=True)['fixed_days'] == 0, 'ingest and retention agree'
    SampleRange.objects.filter(sample__span_s=Sample.DAY).delete()
    Sample.objects.filter(span_s=Sample.DAY).delete()
    retention.roll_up()
    assert not Sample.objects.filter(span_s=Sample.HOUR).exists()
    daily = Sample.objects.get(span_s=Sample.DAY)
    assert (daily.modes_valid, daily.es_valid) == (300, 110), 'old columns kept'
    assert SampleRange.objects.get(sample=daily, protocol='mode_s').frames == 900
    series = queries.stats_series([device], day, day + timedelta(days=1), 'day')
    assert [row['mode_s'] for row in series] == [1310]


def test_a_widened_retry_replaces_the_total(client, owner, bee):
    first = new(bee, 1000)
    assert post_report(client, owner.token, first).status_code == 200
    widened = json.loads(json.dumps(first))
    widened['period']['len_s'] = 7200
    widened['frames']['mode_s'] = 2500
    assert post_report(client, owner.token, widened).status_code == 200
    assert stats(client, owner)[0]['mode_s'] == 2500
    assert SampleRange.objects.get(sample__span_s=Sample.DAY, protocol='mode_s').frames == 2500


def test_the_range_reply_gives_each_bee_the_total(client, owner):
    legacy, current = Bee.make(0, seed=1), Bee.make(1, seed=1)
    for body in (old(legacy, 1000, 400), new(current, 2000)):
        assert post_report(client, owner.token, body).status_code == 200
    response = client.get('/api/swarm/v1/range', {'protocol': 'mode_s'},
                          HTTP_AUTHORIZATION=f'Bearer {owner.api_token}')
    frames = {d['unique_id']: d['frames'] for d in response.json()['devices']}
    assert frames == {legacy.unique_id: 1400, current.unique_id: 2000}
    everything = client.get('/api/swarm/v1/range', HTTP_AUTHORIZATION=f'Bearer {owner.api_token}').json()
    assert {d['frames'] for d in everything['devices']} == {None}, '`aircraft` has no frame count of its own'


def test_the_reference_ui_and_admin_show_one_mode_s_count(client, owner, bee):
    assert post_report(client, owner.token, old(bee, 1200, 500)).status_code == 200
    assert post_report(client, owner.token, new(bee, 300)).status_code == 200
    device = Device.objects.get()
    client.login(username='alice', password='pw')
    page = client.get(f'/devices/{device.pk}/').content.decode()
    assert 'Mode S decoded (56- and 112-bit)' in page and 'Extended squitters' not in page
    assert ': 2,000</title>' in page, 'the bar holds the total'
    get_user_model().objects.create_superuser('root', 'r@example.com', 'pw')
    client.login(username='root', password='pw')
    admin = client.get('/admin/adsbee_swarm/sample/').content.decode()
    assert 'column-mode_s' in admin and 'column-es_valid' not in admin
    assert admin.count('<td class="field-mode_s">2000</td>') == 2, 'the hour and the day'


def test_the_docs_and_openapi_describe_the_total():
    spec = yaml.safe_load((ROOT / 'adsbee_swarm' / 'openapi.yaml').read_text())
    row = spec['components']['schemas']['Stats']['properties']['series']['items']['properties']
    assert 'mode_s' in row and 'modes_valid' in row and 'es_valid' in row
    report = spec['components']['schemas']['Report']['properties']
    assert 'mode_s' in report['frames']['properties']
    docs = (ROOT / 'docs' / 'swarm_api.md').read_text()
    assert '"frames": {"mode_s":' in docs and 'modes_valid + es_valid' in docs
