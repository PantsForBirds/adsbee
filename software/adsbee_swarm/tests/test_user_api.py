import json
import re
from datetime import timedelta
from pathlib import Path

import pytest
import yaml
from django.test import Client
from django.utils import timezone

from adsbee_swarm import services
from adsbee_swarm.models import Device, Sample

from .conftest import post_report

pytestmark = pytest.mark.django_db
API = '/api/swarm/v1'


def api(client, method, path, owner=None, body=None, **params):
    headers = {'HTTP_AUTHORIZATION': f'Bearer {owner.api_token}'} if owner else {}
    if body is not None:
        return getattr(client, method)(API + path, data=json.dumps(body), content_type='application/json', **headers)
    return getattr(client, method)(API + path, params or None, **headers)


@pytest.fixture
def mine(client, owner, bee):
    post_report(client, owner.token, bee.report())
    return Device.objects.get(account=owner.account)


@pytest.fixture
def theirs(other):
    return Device.objects.create(account=other.account, product='adsbee_1090', unique_id='00000000000000FF',
                                 label='secret')


def test_an_account_never_sees_another_accounts_bees(client, owner, other, mine, theirs):
    listed = api(client, 'get', '/devices', owner).json()
    assert [d['id'] for d in listed] == [mine.pk]
    for method, path, body in [('get', f'/devices/{theirs.pk}', None), ('patch', f'/devices/{theirs.pk}', {'label': 'x'}),
                               ('delete', f'/devices/{theirs.pk}', None), ('post', f'/devices/{theirs.pk}/restore', {})]:
        response = api(client, method, path, owner, body)
        assert response.status_code == 404 and response.json() == {'error': 'not_found'}
    assert api(client, 'get', '/range', owner, device=theirs.pk).status_code == 404
    assert api(client, 'get', '/stats', owner, device=theirs.pk).json()['series'] == []
    theirs.refresh_from_db()
    assert theirs.label == 'secret' and theirs.removed_at is None
    plan = services.create_rollout(other.account, {'product': 'adsbee_1090', 'tag': '1.0.0'})
    assert api(client, 'get', f'/rollouts/{plan.pk}', owner).status_code == 404
    assert api(client, 'post', f'/rollouts/{plan.pk}', owner, {'action': 'cancel'}).status_code == 404
    assert api(client, 'get', '/rollouts', owner).json() == []


def test_auth_is_required_and_tokens_are_checked(client, owner):
    assert client.get(API + '/devices').status_code == 401
    assert client.get(API + '/devices', HTTP_AUTHORIZATION='Bearer nope').status_code == 401
    assert client.get(API + '/devices', HTTP_AUTHORIZATION=f'Bearer {owner.token}').status_code == 401, \
        "an enroll token is not an API token"
    services.set_disabled(owner.account)
    assert api(client, 'get', '/devices', owner).status_code == 401


def test_the_swarm_privilege_is_needed(client, owner):
    services.revoke(owner.account, 'swarm')
    response = api(client, 'get', '/devices', owner)
    assert response.status_code == 403 and response.json()['error'] == 'no_privilege'


def test_a_session_works_and_needs_csrf_for_changes(owner, mine):
    browser = Client(enforce_csrf_checks=True)
    assert browser.login(username='alice', password='pw')
    assert browser.get(API + '/devices').status_code == 200
    response = browser.patch(f'{API}/devices/{mine.pk}', data='{"label": "x"}', content_type='application/json')
    assert response.status_code == 403 and response.json()['error'] == 'csrf'
    browser.get('/')                                    # sets the CSRF cookie
    token = browser.cookies['csrftoken'].value
    response = browser.patch(f'{API}/devices/{mine.pk}', data='{"label": "x"}', content_type='application/json',
                             HTTP_X_CSRFTOKEN=token)
    assert response.status_code == 200 and response.json()['label'] == 'x'


def test_label_locate_remove_restore(client, owner, mine):
    body = api(client, 'patch', f'/devices/{mine.pk}', owner,
               {'label': 'Roof', 'note': 'north mast', 'location': {'lat': 37.5, 'lon': -122.2, 'alt_m': 12}}).json()
    assert body['label'] == 'Roof' and body['position'] == {'lat': 37.5, 'lon': -122.2, 'alt_m': 12.0, 'source': 'owner'}
    body = api(client, 'patch', f'/devices/{mine.pk}', owner, {'location': None}).json()
    assert body['position']['source'] == 'fixed', "back to the reported position"
    assert api(client, 'patch', f'/devices/{mine.pk}', owner, {'location': {'lat': 99, 'lon': 0}}).status_code == 400
    assert api(client, 'delete', f'/devices/{mine.pk}', owner).json()['removed'] is True
    assert api(client, 'get', '/devices', owner).json() == []
    assert len(api(client, 'get', '/devices', owner, removed=1).json()) == 1
    assert api(client, 'post', f'/devices/{mine.pk}/restore', owner, {}).json()['removed'] is False


def test_range_and_stats(client, owner, mine, bee):
    payload = api(client, 'get', '/range', owner).json()
    assert len(payload['devices']) == 1 and len(payload['devices'][0]['max_m']) == 12
    assert payload['devices'][0]['bins'] == Sample.objects.get(span_s=Sample.HOUR).ranges().__len__()
    one = api(client, 'get', '/range', owner, device=mine.pk).json()
    assert one['devices'][0]['id'] == mine.pk
    since = (timezone.now() - timedelta(hours=3)).isoformat()
    stats = api(client, 'get', '/stats', owner, since=since, step='hour').json()
    assert stats['series'][0]['es_valid'] == Sample.objects.get(span_s=Sample.HOUR).es_valid
    assert stats['series'][0]['bees'] == 1
    assert api(client, 'get', '/stats', owner, step='week').status_code == 400
    years = {'since': '2020-01-01T00:00:00Z', 'until': '2026-01-01T00:00:00Z'}
    assert api(client, 'get', '/stats', owner, step='day', **years).status_code == 200, "daily stats: any window"
    assert api(client, 'get', '/stats', owner, step='hour', **years).status_code == 400
    assert api(client, 'get', '/range', owner, since='yesterday').status_code == 400
    assert api(client, 'get', '/range', owner, since='2020-01-01T00:00:00Z', until='2026-01-01T00:00:00Z').status_code == 400


def test_rollouts_crud(client, owner, mine):
    created = api(client, 'post', '/rollouts', owner, {'product': 'adsbee_1090', 'tag': '0.9.2',
                                                       'test_devices': [mine.pk]})
    assert created.status_code == 201 and created.json()['state'] == 'test'
    pk = created.json()['id']
    assert created.json()['progress'] == {'targeted': 1, 'on_tag': 0, 'auto_update': 1}
    assert api(client, 'get', f'/devices/{mine.pk}', owner).json()['firmware_target']['tag'] == '0.9.2'
    for action, state in [('promote', 'all'), ('pause', 'paused'), ('resume', 'all'), ('cancel', 'cancelled')]:
        assert api(client, 'post', f'/rollouts/{pk}', owner, {'action': action}).json()['state'] == state
    assert api(client, 'post', f'/rollouts/{pk}', owner, {'action': 'resume'}).status_code == 400
    assert api(client, 'post', '/rollouts', owner, {'product': 'adsbee_1090', 'tag': ''}).status_code == 400
    assert [r['id'] for r in api(client, 'get', '/rollouts', owner).json()] == [pk]


def test_enroll_token_rotation_and_api_tokens(client, owner, bee):
    body = api(client, 'post', '/account/enroll-token', owner, {}).json()
    assert re.fullmatch(r'[0-9a-f]{32}', body['token'])
    assert body['at_command'] == f"AT+SWARM=1,http://testserver,{body['token']}"
    assert post_report(client, owner.token, bee.report()).status_code == 401
    assert post_report(client, body['token'], bee.report()).status_code == 200
    made = api(client, 'post', '/account/api-tokens', owner, {'name': 'grafana'}).json()
    assert made['token'].startswith('swarm_')
    listed = api(client, 'get', '/account/api-tokens', owner).json()
    assert {t['name'] for t in listed} == {'test', 'grafana'} and 'token' not in listed[0]
    assert api(client, 'delete', f"/account/api-tokens/{made['id']}", owner).status_code == 200
    assert client.get(API + '/devices', HTTP_AUTHORIZATION=f"Bearer {made['token']}").status_code == 401
    account = api(client, 'get', '/account', owner).json()
    assert account['privileges'] == ['swarm'] and account['enroll_token_prefix'] == body['token'][:6]


def test_methods_are_checked(client, owner):
    assert api(client, 'put', '/devices', owner, {}).status_code == 405


def test_the_openapi_spec_names_every_route():
    spec = yaml.safe_load((Path(__file__).parents[1] / 'adsbee_swarm' / 'openapi.yaml').read_text())
    assert spec['openapi'].startswith('3.')
    from adsbee_swarm import api_urls

    routes = {'/' + re.sub(r'<int:(\w+)>', '{id}', str(p.pattern)) for p in api_urls.urlpatterns}
    assert routes == set(spec['paths'])
