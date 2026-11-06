from datetime import timedelta

import pytest
from django.test import override_settings
from django.utils import timezone

from adsbee_swarm import protocol, ranges, services
from adsbee_swarm.models import Device, Sample

from .conftest import post_report

pytestmark = pytest.mark.django_db


def test_a_report_is_stored_and_answered(client, owner, bee, rng):
    body = bee.report(rng=rng)
    response = post_report(client, owner.token, body)
    assert response.status_code == 200, response.content
    assert response.json() == {'next_report_s': 3600, 'firmware': None, 'protocols': ['mode_s', 'uat', 'remote_id']}
    device = Device.objects.get(account=owner.account, unique_id=bee.unique_id)
    assert device.versions == {'rp2040': '0.9.1', 'esp32': '0.9.1'} and device.version == '0.9.1'
    assert (device.rx_lat, device.rx_source) == (bee.lat, 'fixed') and device.auto_update is True
    sample = Sample.objects.get(device=device, span_s=Sample.HOUR)
    assert Sample.objects.get(device=device, span_s=Sample.DAY).ranges() == sample.ranges()
    assert sample.es_valid == body['stats']['es_valid'] and sample.reports == 1
    expected = ranges.decode(body['range']['changed'], body['range']['max'], 2000)
    assert sample.ranges() == expected and expected


def test_values_decode_to_meters_in_the_reports_unit(client, owner, bee):
    body = bee.report()
    body['range'].update(unit_m=500, **dict(zip(('changed', 'max'), ranges.encode({7: 1500, 400: 2000}, 500))))
    assert post_report(client, owner.token, body).status_code == 200
    assert Sample.objects.get(span_s=Sample.HOUR).ranges() == {7: 1500, 400: 2000}


def test_without_a_position_the_range_is_dropped_and_stats_kept(client, owner, bee):
    body = bee.report()
    body['rx_position'] = None
    assert post_report(client, owner.token, body).status_code == 200
    sample = Sample.objects.get(span_s=Sample.HOUR)
    assert sample.range_maps() == {} and sample.modes_valid == body['stats']['modes_valid']


def test_two_reports_in_an_hour_sum_counts_and_max_peaks_and_range(client, owner, bee):
    first, second = bee.report(), bee.report()
    first['stats'].update(es_valid=10, aircraft_max=30)
    second['stats'].update(es_valid=5, aircraft_max=20)
    for body in (first, second):
        assert post_report(client, owner.token, body).status_code == 200
    sample = Sample.objects.get(span_s=Sample.HOUR)
    assert (sample.es_valid, sample.aircraft_max, sample.reports) == (15, 30, 2)
    both = ranges.merge(ranges.decode(first['range']['changed'], first['range']['max'], 2000),
                        ranges.decode(second['range']['changed'], second['range']['max'], 2000))
    assert sample.ranges() == both


def test_a_repeated_report_is_not_counted_twice(client, owner, bee):
    body = bee.report()
    assert post_report(client, owner.token, body).status_code == 200
    assert post_report(client, owner.token, body).status_code == 200
    assert Sample.objects.get(span_s=Sample.HOUR).reports == 1
    assert Sample.objects.get(span_s=Sample.HOUR).es_valid == body['stats']['es_valid']


def test_a_widened_retry_replaces_the_stats_it_covers(client, owner, bee):
    """Same seq and start, longer len_s: a superset of the stored report."""
    body = bee.report()
    body['stats']['es_valid'] = 100
    assert post_report(client, owner.token, body).status_code == 200
    widened = {**body, 'period': {**body['period'], 'len_s': body['period']['len_s'] + 600},
               'stats': {**body['stats'], 'es_valid': 130}}
    widened['range'] = dict(body['range'], **dict(zip(('changed', 'max'), ranges.encode({405: 300000}, 2000))))
    assert post_report(client, owner.token, widened).status_code == 200
    sample = Sample.objects.get(span_s=Sample.HOUR)
    assert (sample.es_valid, sample.reports) == (130, 1)
    assert sample.ranges()[405] == 300000
    # The same seq and start with a SHORTER period is a stale copy: ignored.
    shorter = {**widened, 'period': body['period'], 'stats': {**body['stats'], 'es_valid': 1}}
    assert post_report(client, owner.token, shorter).status_code == 200
    assert Sample.objects.get(span_s=Sample.HOUR).es_valid == 130


def test_a_widened_retry_lands_on_its_own_hour_even_later():
    from adsbee_swarm.simulator import Bee
    from adsbee_swarm import services as s

    account, _ = s.create_account('carol')
    bee = Bee.make(1)
    device = Device.objects.create(account=account, product=bee.product, unique_id=bee.unique_id)
    body = bee.report()
    first_hour = timezone.now().replace(minute=50) - timedelta(hours=1)
    protocol.ingest(device, protocol.parse(body), first_hour)
    widened = {**body, 'period': {**body['period'], 'len_s': 4000}}
    protocol.ingest(device, protocol.parse(widened), first_hour + timedelta(minutes=20))
    assert Sample.objects.filter(device=device, span_s=Sample.HOUR).count() == 1


@pytest.mark.parametrize('header', ['', 'Bearer ', 'Bearer nope', 'Token x'])
def test_a_bad_token_is_401(client, owner, bee, header):
    response = client.post('/api/swarm/v1/report', data=bee.report(), content_type='application/json',
                           **({'HTTP_AUTHORIZATION': header} if header else {}))
    assert response.status_code == 401 and response.json() == {'error': 'bad_token'}


def test_an_api_token_is_not_an_enroll_token(client, owner, bee):
    assert post_report(client, owner.api_token, bee.report()).status_code == 401


def test_a_disabled_account_or_one_without_swarm_is_401(client, owner, bee):
    services.revoke(owner.account, 'swarm')
    assert post_report(client, owner.token, bee.report()).status_code == 401
    services.grant(owner.account, 'swarm')
    services.set_disabled(owner.account)
    assert post_report(client, owner.token, bee.report()).status_code == 401


def test_a_rotated_token_stops_the_old_one(client, owner, bee):
    new = services.rotate_enroll_token(owner.account)
    assert post_report(client, owner.token, bee.report()).status_code == 401
    assert post_report(client, new, bee.report()).status_code == 200


def test_a_removed_device_is_410_until_the_token_is_rotated(client, owner, bee):
    assert post_report(client, owner.token, bee.report()).status_code == 200
    services.remove_device(Device.objects.get())
    response = post_report(client, owner.token, bee.report())
    assert response.status_code == 410 and response.json() == {'error': 'removed'}
    new = services.rotate_enroll_token(owner.account)
    assert post_report(client, new, bee.report()).status_code == 200
    assert Device.objects.get().removed_at is None


def test_a_restored_device_reports_again(client, owner, bee):
    assert post_report(client, owner.token, bee.report()).status_code == 200
    device = Device.objects.get()
    services.remove_device(device)
    services.restore_device(device)
    assert post_report(client, owner.token, bee.report()).status_code == 200


def verify_by_key(request, report, account):
    return report.get('ota_keys') == ['good']


@override_settings(ADSBEE_SWARM_DEVICE_VERIFIER='tests.test_report.verify_by_key')
def test_the_device_verifier_decides_403(client, owner, bee):
    body = bee.report()
    response = post_report(client, owner.token, {**body, 'ota_keys': ['bad']})
    assert response.status_code == 403 and response.json() == {'error': 'bad_key'}
    assert not Device.objects.exists(), "nothing is stored for a device that failed the check"
    assert post_report(client, owner.token, {**body, 'ota_keys': ['good']}).status_code == 200


@override_settings(ADSBEE_SWARM_RATE_LIMITS={'report_ip': '100/60', 'report_device': '2/3600', 'user_api': '9/60'})
def test_a_bee_reporting_too_often_is_429_with_retry_after(client, owner, bee):
    for _ in range(2):
        assert post_report(client, owner.token, bee.report()).status_code == 200
    response = post_report(client, owner.token, bee.report())
    assert response.status_code == 429 and int(response['Retry-After']) > 0


@override_settings(ADSBEE_SWARM_RATE_LIMITS={'report_ip': '1/60', 'report_device': '9/3600', 'user_api': '9/60'})
def test_one_address_is_limited_before_the_token_is_read(client, owner, bee):
    assert post_report(client, owner.token, bee.report()).status_code == 200
    assert post_report(client, 'garbage', bee.report()).status_code == 429


@pytest.mark.parametrize('change, detail', [
    (lambda b: b.update(v=2), 'v must be 1'),
    (lambda b: b.update(product='toaster'), 'unknown product'),
    (lambda b: b.update(unique_id='XYZ'), 'unique_id'),
    (lambda b: b.update(seq=-1), 'seq'),
    (lambda b: b.update(seq=True), 'seq'),
    (lambda b: b['period'].pop('len_s'), 'len_s'),
    (lambda b: b['rx_position'].update(lat=91), 'lat'),
    (lambda b: b['range'].update(sectors=72), '36 sectors'),
    (lambda b: b['range'].update(max='AA=='), 'max has'),
    (lambda b: b['stats'].update(es_valid='many'), 'es_valid'),
    (lambda b: b.update(versions={'rp2040': 1}), 'versions'),
])
def test_bad_reports_are_400_with_the_reason(client, owner, bee, change, detail):
    body = bee.report()
    change(body)
    response = post_report(client, owner.token, body)
    assert response.status_code == 400 and detail in response.json()['detail']


def test_not_json_and_not_post(client, owner):
    response = client.post('/api/swarm/v1/report', data='nope', content_type='application/json',
                           HTTP_AUTHORIZATION=f'Bearer {owner.token}')
    assert response.status_code == 400
    assert client.get('/api/swarm/v1/report').status_code == 405


def test_the_same_bee_on_two_accounts_is_two_devices(client, owner, other, bee):
    assert post_report(client, owner.token, bee.report()).status_code == 200
    assert post_report(client, other.token, bee.report()).status_code == 200
    assert Device.objects.filter(unique_id=bee.unique_id).count() == 2


# ─── Position guards ─────────────────────────────────────────────────────────

def test_the_horizon_limit_per_band():
    # 1.15 * 4.12 km * (sqrt(h_rx) + sqrt(h_top)) + 20 km; the top band uses 60k ft.
    assert ranges.horizon_limit_m(0, 0) == pytest.approx((1.15 * 4.12 * (1000 * 0.3048) ** 0.5 + 20) * 1000)
    assert ranges.horizon_limit_m(11, 100) == pytest.approx((1.15 * 4.12 * (10 + (60000 * 0.3048) ** 0.5) + 20) * 1000)
    assert ranges.horizon_limit_m(3, -50) == ranges.horizon_limit_m(3, None) == ranges.horizon_limit_m(3, 0)


def test_bins_past_the_horizon_are_ignored_and_counted(client, owner, bee):
    body = bee.report()
    body['rx_position']['alt_m'] = 0
    low_limit = ranges.horizon_limit_m(0, 0)
    values = {0: 20000, 1: int(low_limit) + 1000, 2: int(low_limit) + 5000, 400: 300000}
    body['range'].update(zip(('changed', 'max'), ranges.encode(values, 2000)))
    assert post_report(client, owner.token, body).status_code == 200
    sample = Sample.objects.get(span_s=Sample.HOUR)
    assert set(sample.ranges()) == {0, 1, 400}, "one unit of rounding is allowed; 5 km past is not"
    assert sample.bins_ignored == 1
    device = Device.objects.get()
    assert device.bins_ignored == 1 and device.check_position


@pytest.mark.parametrize('withheld', ['no_position', 'implausible'])
def test_a_withheld_range_is_stored_and_flagged(client, owner, bee, withheld):
    body = bee.report()
    body.pop('range')
    body.pop('rx_position')
    body.update(range_withheld=withheld, range_dropped=14)
    assert post_report(client, owner.token, body).status_code == 200
    sample = Sample.objects.get(span_s=Sample.HOUR)
    assert (sample.range_withheld, sample.range_dropped, sample.range_maps()) == (withheld, 14, {})
    assert Sample.objects.get(span_s=Sample.DAY).range_withheld == withheld
    device = Device.objects.get()
    assert (device.range_withheld, device.range_dropped, device.check_position) == (withheld, 14, True)
    body = bee.report()
    assert post_report(client, owner.token, body).status_code == 200
    device.refresh_from_db()
    assert not device.check_position, "a clean report clears the warning"
    assert Sample.objects.get(span_s=Sample.HOUR).range_withheld == withheld, "the hour keeps the most severe"


def test_implausible_outranks_no_position_in_a_sample(client, owner, bee):
    for withheld in ('implausible', 'no_position'):
        body = bee.report()
        body.pop('range')
        body['range_withheld'] = withheld
        assert post_report(client, owner.token, body).status_code == 200
    assert Sample.objects.get(span_s=Sample.HOUR).range_withheld == 'implausible'


def test_a_position_at_0_0_is_no_position(client, owner, bee):
    body = bee.report()
    body['rx_position'].update(lat=0, lon=0)
    assert post_report(client, owner.token, body).status_code == 200
    sample = Sample.objects.get(span_s=Sample.HOUR)
    assert sample.range_maps() == {} and sample.range_withheld == 'no_position'
    assert Device.objects.get().rx_lat is None


@pytest.mark.parametrize('field, value', [('range_withheld', 'gps_sad'), ('range_dropped', -1), ('range_dropped', 'x')])
def test_bad_guard_fields_are_400(client, owner, bee, field, value):
    body = bee.report()
    body[field] = value
    assert post_report(client, owner.token, body).status_code == 400


def test_the_api_and_the_page_say_check_position(client, owner, bee):
    body = bee.report()
    body.pop('range')
    body.update(range_withheld='implausible', range_dropped=9)
    post_report(client, owner.token, body)
    device = Device.objects.get()
    api = client.get(f'/api/swarm/v1/devices/{device.pk}', HTTP_AUTHORIZATION=f'Bearer {owner.api_token}').json()
    assert (api['range_withheld'], api['range_dropped'], api['check_position']) == ('implausible', 9, True)
    client.login(username='alice', password='pw')
    assert '⚠' in client.get('/').content.decode()
    assert 'too many aircraft looked farther' in client.get(f'/devices/{device.pk}/').content.decode()


def unavailable(request, report, account):
    from adsbee_swarm import VerifierUnavailable

    raise VerifierUnavailable('no keys loaded')


def unavailable_soon(request, report, account):
    from adsbee_swarm import VerifierUnavailable

    raise VerifierUnavailable(retry_after=120)


@override_settings(ADSBEE_SWARM_DEVICE_VERIFIER='tests.test_report.unavailable')
def test_a_verifier_that_cannot_decide_answers_503_with_retry_after(client, owner, bee):
    response = post_report(client, owner.token, bee.report())
    assert response.status_code == 503 and response.json() == {'error': 'unavailable', 'detail': 'no keys loaded'}
    assert response['Retry-After'] == '3600'
    assert not Device.objects.exists(), "nothing is stored while undecided"


@override_settings(ADSBEE_SWARM_DEVICE_VERIFIER='tests.test_report.unavailable_soon')
def test_the_verifier_may_name_its_own_retry_after(client, owner, bee):
    response = post_report(client, owner.token, bee.report())
    assert response.status_code == 503 and response['Retry-After'] == '120'
