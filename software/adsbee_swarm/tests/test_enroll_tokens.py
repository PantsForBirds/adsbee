"""Enroll tokens: shown again to the signed-in owner; a new one keeps the bees; cut off stops all."""
import json
import logging
from datetime import timedelta

import pytest
from cryptography.fernet import Fernet
from django.contrib.auth import get_user_model
from django.db import connection
from django.db.migrations.executor import MigrationExecutor
from django.utils import timezone

from adsbee_swarm import sealed, services
from adsbee_swarm.models import Device, EnrollToken, token_hash
from adsbee_swarm.simulator import Bee

from .conftest import post_report

pytestmark = pytest.mark.django_db
API = '/api/swarm/v1'


@pytest.fixture
def known(client, owner, bee):
    """A bee on the account, reporting with the first token."""
    assert post_report(client, owner.token, bee.report()).status_code == 200
    return Device.objects.get()


@pytest.fixture
def session(client, owner):
    client.login(username='alice', password='pw')
    return client


def reveal(client, **headers):
    return client.post(API + '/account/enroll-token/reveal', data='{}', content_type='application/json', **headers)


# ─── Show again ──────────────────────────────────────────────────────────────

def test_the_current_token_is_kept_encrypted_and_shown_to_its_owner(owner):
    row = owner.account.enroll_token
    assert row.sealed and owner.token not in row.sealed and row.token_hash == token_hash(owner.token)
    assert services.reveal_enroll_token(owner.account) == owner.token


def test_reveal_needs_a_session_not_an_api_token(client, owner):
    response = reveal(client, HTTP_AUTHORIZATION=f'Bearer {owner.api_token}')
    assert response.status_code == 403 and response.json()['error'] == 'session_only'
    assert reveal(client).status_code == 401
    assert reveal(client, HTTP_AUTHORIZATION=f'Bearer {owner.token}').status_code == 401, 'not an enroll token either'


def test_reveal_in_a_session(session, owner):
    response = reveal(session)
    assert response.status_code == 200 and response['Cache-Control'] == 'no-store'
    assert response.json() == {'token': owner.token, 'at_command': f'AT+SWARM=1,http://testserver,{owner.token}'}
    assert session.get(API + '/account/enroll-token/reveal').status_code == 405


def test_reveal_needs_the_csrf_token(client, owner):
    strict = client.__class__(enforce_csrf_checks=True)
    strict.login(username='alice', password='pw')
    assert reveal(strict).status_code == 403


def test_a_changed_key_hides_the_token_but_it_keeps_working(client, owner, bee, settings):
    settings.ADSBEE_SWARM_TOKEN_KEY = Fernet.generate_key().decode()
    assert services.reveal_enroll_token(owner.account) is None
    assert post_report(client, owner.token, bee.report()).status_code == 200
    client.login(username='alice', password='pw')
    assert reveal(client).json() == {'error': 'unavailable', 'detail': 'make a new token to see it here'}
    new = services.rotate_enroll_token(owner.account)
    assert services.reveal_enroll_token(owner.account) == new


def test_a_key_list_reads_tokens_sealed_with_an_older_key(owner, settings):
    settings.ADSBEE_SWARM_TOKEN_KEY = [Fernet.generate_key().decode(), sealed.derived_key(settings.SECRET_KEY)]
    assert services.reveal_enroll_token(owner.account) == owner.token


def test_a_token_made_before_tokens_were_kept_is_not_shown(owner):
    EnrollToken.objects.update(sealed='')
    assert services.reveal_enroll_token(owner.account) is None
    assert services.account_by_enroll_token(owner.token) == owner.account


def test_reveals_are_audited_without_the_token(owner, settings, caplog):
    seen = []
    settings.ADSBEE_SWARM_AUDIT = 'tests.test_enroll_tokens.record'
    record.seen = seen
    with caplog.at_level(logging.DEBUG):
        services.reveal_enroll_token(owner.account)
        new = services.rotate_enroll_token(owner.account)
        cut = services.rotate_enroll_token(owner.account, cut_off=True)
    assert [event for _account, event, _request, _data in seen] == [
        'enroll_token_revealed', 'enroll_token_new', 'enroll_token_cut_off']
    text = caplog.text + repr(seen)
    assert 'enroll_token_revealed' in caplog.text
    assert not any(t in text or t[:6] in text for t in (owner.token, new, cut))


def record(account, event, request=None, **data):
    record.seen.append((account.pk, event, request, data))


def test_the_django_admin_never_shows_a_token(client, owner):
    get_user_model().objects.create_superuser('root', 'r@example.com', 'pw')
    client.login(username='root', password='pw')
    page = client.get(f'/admin/adsbee_swarm/account/{owner.account.pk}/change/').content.decode()
    row = owner.account.enroll_token
    assert row.prefix in page and owner.token not in page and row.sealed not in page and row.token_hash not in page


# ─── A new token keeps the bees ──────────────────────────────────────────────

def test_a_new_token_keeps_known_bees_and_turns_away_new_ones(client, owner, known, bee):
    new = services.rotate_enroll_token(owner.account)
    assert post_report(client, owner.token, bee.report()).status_code == 200, 'a known bee keeps its old token'
    assert Device.objects.get().old_token
    stranger = Bee.make(1, seed=9)
    response = post_report(client, owner.token, stranger.report())
    assert response.status_code == 401 and response.json() == {'error': 'bad_token'}
    assert Device.objects.count() == 1
    assert post_report(client, new, stranger.report()).status_code == 200
    assert post_report(client, new, bee.report()).status_code == 200
    assert not Device.objects.get(pk=known.pk).old_token


def test_a_bee_first_seen_after_the_token_was_replaced_is_not_admitted_by_it(client, owner, bee):
    first = owner.token
    services.rotate_enroll_token(owner.account)
    newer = services.rotate_enroll_token(owner.account)
    assert post_report(client, newer, bee.report()).status_code == 200
    assert post_report(client, first, bee.report()).status_code == 401, 'the bee came after `first` was retired'


def test_every_old_token_keeps_working_for_the_bees_it_had(client, owner, known, bee):
    second = services.rotate_enroll_token(owner.account)
    services.rotate_enroll_token(owner.account)
    assert post_report(client, owner.token, bee.report()).status_code == 200
    assert post_report(client, second, bee.report()).status_code == 200


def test_cut_off_stops_every_old_token_for_everyone(client, owner, known, bee):
    second = services.rotate_enroll_token(owner.account)
    third = services.rotate_enroll_token(owner.account, cut_off=True)
    for old in (owner.token, second):
        response = post_report(client, old, bee.report())
        assert response.status_code == 401 and response.json() == {'error': 'bad_token'}
    assert post_report(client, third, bee.report()).status_code == 200
    fourth = services.rotate_enroll_token(owner.account)
    assert post_report(client, owner.token, bee.report()).status_code == 401, 'cut off stays cut off'
    assert post_report(client, third, bee.report()).status_code == 200
    assert services.reveal_enroll_token(owner.account) == fourth


def test_only_the_current_token_is_kept_encrypted(owner):
    services.rotate_enroll_token(owner.account)
    rows = list(EnrollToken.objects.order_by('created_at', 'pk'))
    assert [bool(r.sealed) for r in rows] == [False, True] and [r.current for r in rows] == [False, True]


def test_another_accounts_token_does_not_admit_my_bee(client, owner, other, known, bee):
    services.rotate_enroll_token(other.account)
    assert post_report(client, other.token, bee.report()).status_code == 401
    assert Device.objects.filter(account=other.account).count() == 0


# ─── Removed bees ────────────────────────────────────────────────────────────

def test_a_removed_bee_is_not_readmitted_by_an_old_token(client, owner, known, bee):
    services.remove_device(known)
    new = services.rotate_enroll_token(owner.account)
    assert post_report(client, owner.token, bee.report()).status_code == 410
    assert post_report(client, new, bee.report()).status_code == 200, 'a token made after the removal'
    assert Device.objects.get().removed_at is None


def test_a_token_made_before_the_removal_does_not_readmit(client, owner, known, bee):
    new = services.rotate_enroll_token(owner.account)
    Device.objects.update(removed_at=timezone.now() + timedelta(seconds=1))
    assert post_report(client, new, bee.report()).status_code == 410
    services.restore_device(Device.objects.get())
    assert post_report(client, owner.token, bee.report()).status_code == 200, 'restored by its owner'


# ─── API ─────────────────────────────────────────────────────────────────────

def test_the_api_makes_a_new_token_or_cuts_off(client, owner, known, bee):
    headers = {'HTTP_AUTHORIZATION': f'Bearer {owner.api_token}'}
    made = client.post(API + '/account/enroll-token', data='{}', content_type='application/json', **headers)
    assert made.status_code == 200 and made.json()['cut_off'] is False and made['Cache-Control'] == 'no-store'
    assert post_report(client, owner.token, bee.report()).status_code == 200
    device = client.get(API + '/devices', **headers).json()[0]
    assert device['old_token'] is True
    account = client.get(API + '/account', **headers).json()
    assert account['enroll_token_prefix'] == made.json()['token'][:6] and account['enroll_token_revealable']
    cut = client.post(API + '/account/enroll-token', data=json.dumps({'cut_off': True}),
                      content_type='application/json', **headers)
    assert cut.json()['cut_off'] is True
    for old in (owner.token, made.json()['token']):
        assert post_report(client, old, bee.report()).status_code == 401
    bad = client.post(API + '/account/enroll-token', data=json.dumps({'cut_off': 'yes'}),
                      content_type='application/json', **headers)
    assert bad.status_code == 400


# ─── The self-hosted page ────────────────────────────────────────────────────

def test_the_account_page_reveals_makes_and_cuts_off(session, owner, known, bee):
    page = session.get('/account/').content.decode()
    assert owner.token not in page and owner.token[:6] in page and 'value="reveal"' in page
    shown = session.post('/account/action/', {'action': 'reveal'})
    assert f'AT+SWARM=1,http://testserver,{owner.token}' in shown.content.decode()
    assert shown['Cache-Control'] == 'no-store'
    made = session.post('/account/action/', {'action': 'rotate'}).content.decode()
    new = owner.account.enroll_tokens.get(retired_at=None)
    assert f'AT+SWARM=1,http://testserver,' in made and 'bees already here keep' in made
    assert post_report(session, owner.token, bee.report()).status_code == 200
    listed = session.get('/').content.decode()
    assert 'Old token' in listed
    session.post('/account/action/', {'action': 'cut_off'})
    assert post_report(session, owner.token, bee.report()).status_code == 401
    assert owner.account.enroll_tokens.filter(cut_off=True).count() == 2 and new.pk


def test_the_account_page_says_when_the_token_cannot_be_shown(session, owner):
    EnrollToken.objects.update(sealed='')
    page = session.post('/account/action/', {'action': 'reveal'}).content.decode()
    assert 'Make a new token to see it here' in page and 'AT+SWARM=1' not in page


# ─── Migration ───────────────────────────────────────────────────────────────

@pytest.mark.django_db(transaction=True)
def test_the_migration_keeps_an_existing_hash_as_the_current_token():
    executor = MigrationExecutor(connection)
    executor.migrate([('adsbee_swarm', '0004_remove_sample_range_map')])
    apps = executor.loader.project_state([('adsbee_swarm', '0004_remove_sample_range_map')]).apps
    user = apps.get_model('auth', 'User').objects.create(username='old')
    rotated = timezone.now() - timedelta(days=30)
    token = 'ab' * 16
    apps.get_model('adsbee_swarm', 'Account').objects.create(
        user=user, privileges=['swarm'], enroll_token_hash=token_hash(token), enroll_token_prefix=token[:6],
        enroll_token_rotated_at=rotated)
    apps.get_model('adsbee_swarm', 'Account').objects.create(
        user=apps.get_model('auth', 'User').objects.create(username='none'), privileges=['swarm'])
    executor = MigrationExecutor(connection)
    executor.loader.build_graph()
    executor.migrate(executor.loader.graph.leaf_nodes())
    row = EnrollToken.objects.get()
    assert (row.account.user.username, row.prefix, row.created_at, row.sealed, row.current) == (
        'old', token[:6], rotated, '', True)
    assert services.account_by_enroll_token(token) == row.account
    assert services.reveal_enroll_token(row.account) is None
