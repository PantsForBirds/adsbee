"""
What a host site calls to run accounts (staff pages, invites) and what the APIs and pages call to
change things. Nothing here checks who is asking: callers do.
"""
from __future__ import annotations

import logging
import re
import secrets

from django.contrib.auth import get_user_model
from django.db import transaction
from django.utils import timezone

from . import conf, sealed
from .models import Account, ApiToken, Device, EnrollToken, Rollout, token_hash

log = logging.getLogger('adsbee_swarm.audit')


class SwarmError(ValueError):
    """Refused as asked; the message is the sentence to show."""


# ─── Accounts ────────────────────────────────────────────────────────────────

def _check_privileges(privileges) -> list[str]:
    known = conf.get('PRIVILEGES')
    unknown = [p for p in privileges if p not in known]
    if unknown:
        raise SwarmError(f"Unknown privilege: {', '.join(unknown)}")
    return sorted(set(privileges))


@transaction.atomic
def create_account(username: str, *, email: str = '', password: str | None = None,
                   privileges=None) -> tuple[Account, str]:
    """A user and its account. Returns (account, enroll token): the token is shown once.
    No password: the host site sends an invite (set_unusable_password)."""
    User = get_user_model()
    user = User.objects.create_user(username=username, email=email, password=password)
    if password is None:
        user.set_unusable_password()
        user.save()
    account = Account.objects.create(
        user=user, privileges=_check_privileges(conf.get('DEFAULT_PRIVILEGES') if privileges is None else privileges))
    return account, _new_enroll_token(account)


def account_for(user) -> Account | None:
    if user is None or not user.is_authenticated:
        return None
    return Account.objects.filter(user=user).first()


def set_privileges(account: Account, privileges) -> Account:
    account.privileges = _check_privileges(privileges)
    account.save(update_fields=['privileges'])
    return account


def grant(account: Account, privilege: str) -> Account:
    return set_privileges(account, [*account.privileges, privilege])


def revoke(account: Account, privilege: str) -> Account:
    return set_privileges(account, [p for p in account.privileges if p != privilege])


def set_disabled(account: Account, disabled: bool = True) -> Account:
    """A disabled account's login, API tokens and enroll token stop working; nothing is deleted."""
    account.disabled = disabled
    account.save(update_fields=['disabled'])
    if disabled:
        account.user.is_active = False
    else:
        account.user.is_active = True
    account.user.save(update_fields=['is_active'])
    return account


# ─── Tokens ──────────────────────────────────────────────────────────────────

def audit(account: Account, event: str, request=None, **data) -> None:
    """Logs `event` (never a token) and calls the AUDIT hook, if set."""
    log.info('%s account=%s %s', event, account.pk, ' '.join(f'{k}={v}' for k, v in data.items()))
    hook = conf.hook('AUDIT')
    if hook:
        hook(account, event, request, **data)


def _new_enroll_token(account: Account, cut_off: bool = False) -> str:
    """Retires the current token (and, with `cut_off`, stops every older one) and makes a new one."""
    token = secrets.token_hex(16)
    now = timezone.now()
    with transaction.atomic():
        Account.objects.select_for_update().filter(pk=account.pk).first()
        account.enroll_tokens.filter(retired_at__isnull=True).update(retired_at=now, sealed='')
        if cut_off:
            account.enroll_tokens.update(cut_off=True)
        EnrollToken.objects.create(account=account, token_hash=token_hash(token), prefix=token[:6],
                                   sealed=sealed.seal(token), created_at=now)
    return token


def rotate_enroll_token(account: Account, *, cut_off: bool = False, request=None) -> str:
    """
    A new enroll token, for new bees. 32 hex digits: safe inside AT+SWARM.
    The old ones keep working for the bees already on the account, unless `cut_off`: then every
    old token stops working and each bee needs the new line.
    """
    token = _new_enroll_token(account, cut_off)
    audit(account, 'enroll_token_cut_off' if cut_off else 'enroll_token_new', request)
    return token


def reveal_enroll_token(account: Account, *, request=None) -> str | None:
    """
    The current enroll token, for its signed-in owner only (never through an API token). None
    when it can't be read: made before tokens were kept, or the key changed. Make a new one.
    """
    current = account.enroll_token
    token = sealed.unseal(current.sealed) if current else None
    if token is not None and token_hash(token) != current.token_hash:
        token = None
    if token is not None:
        audit(account, 'enroll_token_revealed', request)
    return token


def enroll_token_by(token: str) -> EnrollToken | None:
    """The enroll token row, if it works at all: not cut off, the account not disabled."""
    if not token or len(token) > 128:
        return None
    row = EnrollToken.objects.select_related('account__user').filter(token_hash=token_hash(token)).first()
    if row is None or row.cut_off or row.account.disabled:
        return None
    return row


def account_by_enroll_token(token: str) -> Account | None:
    """The account of a working enroll token (current or old: see `admits`)."""
    row = enroll_token_by(token)
    return row.account if row else None


def admits(row: EnrollToken, device: Device | None) -> bool:
    """
    The current token admits any bee; an old one only a bee the account had before it was
    retired.
    """
    if row.retired_at is None:
        return True
    return device is not None and device.first_seen < row.retired_at


def create_api_token(account: Account, name: str = '') -> tuple[ApiToken, str]:
    token = 'swarm_' + secrets.token_urlsafe(30)
    row = ApiToken.objects.create(account=account, name=name[:64], token_hash=token_hash(token), prefix=token[6:12])
    return row, token


def account_by_api_token(token: str) -> Account | None:
    if not token or len(token) > 128:
        return None
    row = ApiToken.objects.select_related('account__user').filter(token_hash=token_hash(token)).first()
    if row is None or row.account.disabled:
        return None
    ApiToken.objects.filter(pk=row.pk).update(last_used_at=timezone.now())
    return row.account


# ─── Devices ─────────────────────────────────────────────────────────────────

def edit_device(device: Device, data: dict) -> Device:
    """label, note, location ({lat, lon, alt_m} or None to use the reported position)."""
    fields = []
    for key, limit in (('label', 120), ('note', 500)):
        if key in data:
            value = data[key]
            if not isinstance(value, str) or len(value) > limit:
                raise SwarmError(f'{key} must be text up to {limit} characters')
            setattr(device, key, value.strip())
            fields.append(key)
    if 'location' in data:
        location = data['location']
        if location is None:
            device.lat = device.lon = device.alt_m = None
        else:
            device.lat, device.lon, device.alt_m = _location(location)
        fields += ['lat', 'lon', 'alt_m']
    if fields:
        device.save(update_fields=fields)
    return device


def _location(value) -> tuple[float, float, float | None]:
    try:
        lat, lon = float(value['lat']), float(value['lon'])
        alt = value.get('alt_m')
        alt = None if alt in (None, '') else float(alt)
    except (TypeError, KeyError, ValueError) as exc:
        raise SwarmError('location needs numbers lat and lon (alt_m optional)') from exc
    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
        raise SwarmError('location is off the map')
    return lat, lon, alt


def remove_device(device: Device) -> Device:
    """Off the account: its reports answer 410 until it is restored, or it reports with an enroll
    token made after its removal."""
    device.removed_at = timezone.now()
    device.save(update_fields=['removed_at'])
    return device


def restore_device(device: Device) -> Device:
    device.removed_at = None
    device.save(update_fields=['removed_at'])
    return device


def is_turned_away(device: Device, row: EnrollToken | None = None) -> bool:
    """Removed, and reporting with a token (default: the current one) made before its removal.
    A newer token restores it."""
    if device.removed_at is None:
        return False
    row = row or device.account.enroll_token
    if row is not None and row.created_at > device.removed_at:
        restore_device(device)
        return False
    return True


# ─── Rollouts ────────────────────────────────────────────────────────────────

TAG = re.compile(r'^[0-9A-Za-z._+-]{1,64}$')


def _owned(account: Account, ids) -> list[Device]:
    ids = list(ids or [])
    if not all(isinstance(i, int) for i in ids):
        raise SwarmError('devices are given by id')
    found = list(Device.objects.filter(account=account, pk__in=ids, removed_at__isnull=True))
    if len(found) != len(set(ids)):
        raise SwarmError('Not every device is on this account')
    return found


@transaction.atomic
def create_rollout(account: Account, data: dict) -> Rollout:
    """product, tag, channel?, scope (all|label|devices), label?, devices?, test_devices?, note?
    With test_devices the rollout starts in stage `test`; `promote` widens it to the scope."""
    product = data.get('product')
    if product not in conf.products():
        raise SwarmError('Unknown product')
    tag = str(data.get('tag') or '').strip()
    if not TAG.match(tag):
        raise SwarmError('tag must be a version tag like 0.9.2')
    validator = conf.hook('TAG_VALIDATOR')
    problem = validator(product, tag) if validator else None
    if problem:
        raise SwarmError(problem)
    channel = str(data.get('channel') or ('rc' if '-rc' in tag.lower() else 'stable'))[:16]
    scope = data.get('scope', Rollout.SCOPE_ALL)
    if scope not in dict(Rollout.SCOPES):
        raise SwarmError('scope is all, label or devices')
    label = str(data.get('label') or '').strip()
    if scope == Rollout.SCOPE_LABEL and not label:
        raise SwarmError('Give the label')
    devices = _owned(account, data.get('devices'))
    if scope == Rollout.SCOPE_DEVICES and not devices:
        raise SwarmError('Choose the bees')
    tests = _owned(account, data.get('test_devices'))
    rollout = Rollout.objects.create(
        account=account, product=product, tag=tag, channel=channel, scope=scope, label=label[:120],
        stage=Rollout.STAGE_TEST if tests else Rollout.STAGE_ALL, note=str(data.get('note') or '')[:200])
    rollout.devices.set(devices)
    rollout.test_devices.set(tests)
    return rollout


def rollout_action(rollout: Rollout, action: str) -> Rollout:
    """promote (test bees -> every bee in scope), pause, resume, cancel."""
    if rollout.cancelled_at:
        raise SwarmError('This rollout was cancelled')
    if action == 'promote':
        rollout.stage, rollout.promoted_at = Rollout.STAGE_ALL, timezone.now()
    elif action == 'pause':
        rollout.paused = True
    elif action == 'resume':
        rollout.paused = False
    elif action == 'cancel':
        rollout.cancelled_at = timezone.now()
    else:
        raise SwarmError('action is promote, pause, resume or cancel')
    rollout.save()
    return rollout
