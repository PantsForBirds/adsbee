from io import StringIO

import pytest
from django.contrib.auth import get_user_model
from django.core.management import CommandError, call_command

from adsbee_swarm import protocol, services
from adsbee_swarm.models import Account, EnrollToken, token_hash
from adsbee_swarm.simulator import Bee

pytestmark = pytest.mark.django_db


def test_create_account_without_a_password_is_invite_only():
    account, token = services.create_account('dana', email='d@example.com', privileges=['swarm', 'wiggler'])
    assert not account.user.has_usable_password()
    assert account.privileges == ['swarm', 'wiggler'] and account.has('wiggler')
    assert account.enroll_token.token_hash == token_hash(token)
    assert token not in str(Account.objects.values()) + str(EnrollToken.objects.values())


def test_privileges_are_checked_and_extensible(settings):
    account, _ = services.create_account('erin')
    with pytest.raises(services.SwarmError):
        services.grant(account, 'teleport')
    settings.ADSBEE_SWARM_PRIVILEGES = {'swarm': 'Swarm', 'teleport': 'Teleport'}
    assert services.grant(account, 'teleport').has('teleport')


def test_disable_blocks_login_and_tokens():
    account, token = services.create_account('fred', password='pw')
    services.set_disabled(account)
    assert not get_user_model().objects.get(username='fred').is_active
    assert services.account_by_enroll_token(token) is None
    services.set_disabled(account, False)
    assert services.account_by_enroll_token(token) == account


def test_the_account_command():
    out = StringIO()
    call_command('swarm_account', 'create', 'gus', '--privilege', 'swarm', stdout=out)
    assert 'enroll token:' in out.getvalue()
    call_command('swarm_account', 'grant', 'gus', '--privilege', 'wiggler', stdout=StringIO())
    assert Account.objects.get().privileges == ['swarm', 'wiggler']
    call_command('swarm_account', 'disable', 'gus', stdout=StringIO())
    assert Account.objects.get().disabled
    with pytest.raises(CommandError):
        call_command('swarm_account', 'grant', 'nobody', stdout=StringIO())


def test_simulated_reports_parse_and_look_like_a_receiver():
    bee = Bee.make(2)
    report = protocol.parse(bee.report())
    assert report.rx_position['alt_m'] == bee.alt_m and report.seq == 1
    (unit, values), = report.maps.values()
    high = max(m for i, m in values.items() if i >= 11 * 36)
    low = max((m for i, m in values.items() if i < 36), default=0)
    assert high > low and high <= 400_000


@pytest.mark.django_db(transaction=True)
def test_simulate_runs_outside_a_test_transaction():
    """The command takes row locks; on Postgres they need its own transaction, not the test's."""
    services.create_account('hal', password='pw')
    call_command('swarm_simulate', 'hal', '--bees', '1', '--hours', '2', stdout=StringIO())
    from adsbee_swarm.models import Sample

    assert Sample.objects.count() >= 1
