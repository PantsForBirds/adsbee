from datetime import timedelta

import pytest
from django.utils import timezone

from adsbee_swarm import queries, ranges
from adsbee_swarm.models import Device, Sample

from .conftest import save_sample

pytestmark = pytest.mark.django_db


def _sample(device, start, values, span=Sample.HOUR, protocol='aircraft', **stats):
    return save_sample(Sample(device=device, start=start, span_s=span, reports=1, **stats), values, protocol)


@pytest.fixture
def device(owner):
    return Device.objects.create(account=owner.account, product='adsbee_1090', unique_id='E6614C311B5B6F2A',
                                 rx_lat=37.0, rx_lon=-122.0)


def test_the_range_is_the_max_per_bin_inside_the_window(device):
    now = timezone.now().replace(minute=0, second=0, microsecond=0)
    _sample(device, now - timedelta(hours=1), {0: 10000, 5: 50000})
    _sample(device, now - timedelta(hours=2), {0: 30000, 6: 20000})
    _sample(device, now - timedelta(days=10), {0: 400000, 7: 1})        # outside
    merged = queries.device_range(device, now - timedelta(days=1), now)
    assert merged == {0: 30000, 5: 50000, 6: 20000}


def test_daily_rows_count_when_their_day_overlaps(device):
    now = timezone.now()
    day = (now - timedelta(days=400)).replace(hour=0, minute=0, second=0, microsecond=0)
    _sample(device, day, {3: 80000}, span=Sample.DAY)
    assert queries.device_range(device, day - timedelta(days=2), day + timedelta(days=2)) == {3: 80000}
    assert queries.device_range(device, day + timedelta(days=1), day + timedelta(days=5)) == {}


def test_the_payload_is_ready_for_a_3d_view(device, other):
    now = timezone.now()
    _sample(device, now - timedelta(hours=1), {ranges.bin_index(4, 9): 120000})
    second = Device.objects.create(account=device.account, product='adsbee_1090', unique_id='0000000000000001')
    _sample(second, now - timedelta(hours=1), {ranges.bin_index(4, 9): 150000, ranges.bin_index(0, 0): 4000})
    payload = queries.swarm_range([device, second], now - timedelta(days=1), now)
    assert payload['band_edges_ft'][0] == [0, 1000] and payload['band_edges_ft'][-1] == [40000, None]
    first = payload['devices'][0]
    assert first['max_m'][4][9] == 120000 and first['sector_max_m'][9] == 120000
    assert first['position'] == {'lat': 37.0, 'lon': -122.0, 'alt_m': None, 'source': ''}
    assert payload['swarm']['max_m'][4][9] == 150000 and payload['swarm']['max_m'][0][0] == 4000


def test_stats_sum_counts_and_max_peaks_per_bee(device):
    hour = timezone.now().replace(minute=0, second=0, microsecond=0) - timedelta(hours=2)
    _sample(device, hour, {}, es_valid=10, aircraft_max=7)
    second = Device.objects.create(account=device.account, product='adsbee_1090', unique_id='0000000000000001')
    _sample(second, hour, {}, es_valid=5, aircraft_max=3)
    series = queries.stats_series([device, second], hour - timedelta(hours=1), hour + timedelta(hours=1))
    assert len(series) == 1 and series[0]['es_valid'] == 15 and series[0]['aircraft_max'] == 10
    assert series[0]['bees'] == 2
    _sample(device, hour.replace(hour=0), {}, span=Sample.DAY, es_valid=10)
    daily = queries.stats_series([device], hour - timedelta(days=1), hour + timedelta(hours=1), 'day')
    assert sum(row['es_valid'] for row in daily) == 10, "step=day reads the daily rows"


def test_stats_count_the_bees_that_reported_and_leave_out_silent_steps(device):
    hour = timezone.now().replace(minute=0, second=0, microsecond=0) - timedelta(hours=5)
    second = Device.objects.create(account=device.account, product='adsbee_1090', unique_id='0000000000000002')
    _sample(device, hour, {}, es_valid=4)
    _sample(second, hour, {}, es_valid=6)
    _sample(device, hour + timedelta(hours=3), {}, es_valid=1)
    series = queries.stats_series([device, second], hour - timedelta(hours=1), hour + timedelta(hours=4))
    assert [(row['t'], row['bees'], row['es_valid']) for row in series] == [
        (hour.isoformat(), 2, 10), ((hour + timedelta(hours=3)).isoformat(), 1, 1)], "no rows for the silent hours"
