"""
The user API under /api/swarm/v1/ (openapi.yaml). JSON in and out; errors {"error", "detail"?}.

Auth: a signed-in session (unsafe methods need the CSRF token), or `Authorization: Bearer <API
token>` (services.create_api_token). Either way the caller is one account, and every lookup is
filtered by it: another account's bee is a 404, never a 403.
"""
from __future__ import annotations

import json
from datetime import datetime
from functools import wraps

from django.http import Http404, JsonResponse
from django.middleware.csrf import CsrfViewMiddleware
from django.shortcuts import get_object_or_404
from django.utils import timezone
from django.utils.dateparse import parse_datetime
from django.views.decorators.csrf import csrf_exempt

from . import conf, queries, ranges, rollout as rollouts, services
from .api_device import error, limited
from .models import ApiToken, Device, Rollout

SAFE = ('GET', 'HEAD', 'OPTIONS')
MAX_BODY = 64 * 1024
MAX_WINDOW_DAYS = 400


def _csrf_refused(request):
    return CsrfViewMiddleware(lambda r: None).process_view(request, None, (), {})


def user_api(*methods, privilege: str = 'swarm', session_only: bool = False):
    """Resolves request.account (session or API token), checks the method, CSRF and privilege.
    `session_only`: an API token gets 403 session_only."""
    def wrapper(func):
        @wraps(func)
        def inner(request, *args, **kwargs):
            if request.method not in methods:
                return error(405, 'method_not_allowed')
            header = request.headers.get('Authorization', '')
            if header.startswith('Bearer '):
                account = services.account_by_api_token(header[7:].strip())
                if account is None:
                    return error(401, 'bad_token')
                if session_only:
                    return error(403, 'session_only')
            else:
                account = services.account_for(getattr(request, 'user', None))
                if account is None or account.disabled:
                    return error(401, 'not_signed_in')
                if request.method not in SAFE and _csrf_refused(request) is not None:
                    return error(403, 'csrf')
            if not account.has(privilege):
                return error(403, 'no_privilege', privilege)
            refused = limited('user_api', f'account:{account.pk}')
            if refused:
                return refused
            request.account = account
            try:
                return func(request, *args, **kwargs)
            except services.SwarmError as exc:
                return error(400, 'bad_request', str(exc))
            except Http404:
                return error(404, 'not_found')
        return csrf_exempt(inner)
    return wrapper


def _body(request) -> dict:
    if len(request.body) > MAX_BODY:
        raise services.SwarmError('body too large')
    try:
        data = json.loads(request.body or b'{}')
    except (ValueError, UnicodeDecodeError) as exc:
        raise services.SwarmError('body must be JSON') from exc
    if not isinstance(data, dict):
        raise services.SwarmError('body must be a JSON object')
    return data


def _json(data, status: int = 200):
    return JsonResponse(data, status=status, safe=not isinstance(data, list))


def _when(request, name: str) -> datetime | None:
    value = request.GET.get(name)
    if not value:
        return None
    moment = parse_datetime(value)
    if moment is None:
        raise services.SwarmError(f'{name} must be an ISO 8601 time')
    return moment if timezone.is_aware(moment) else timezone.make_aware(moment, timezone.utc)


def _window(request) -> tuple[datetime, datetime]:
    until = _when(request, 'until') or timezone.now()
    since = _when(request, 'since') or until - timezone.timedelta(days=7)
    if since >= until:
        raise services.SwarmError('since must be before until')
    if (until - since).days > MAX_WINDOW_DAYS:
        raise services.SwarmError(f'at most {MAX_WINDOW_DAYS} days at a time')
    return since, until


def _protocol(request) -> str:
    protocol = request.GET.get('protocol') or ranges.LEGACY
    if protocol not in ranges.PROTOCOLS:
        raise services.SwarmError(f"protocol is {', '.join(ranges.PROTOCOLS)}")
    return protocol


def device_json(device: Device) -> dict:
    return {'id': device.pk, 'product': device.product, 'unique_id': device.unique_id,
            'mac_address': device.mac_address, 'label': device.label, 'note': device.note,
            'location': ({'lat': device.lat, 'lon': device.lon, 'alt_m': device.alt_m}
                         if device.lat is not None else None),
            'rx_position': ({'lat': device.rx_lat, 'lon': device.rx_lon, 'alt_m': device.rx_alt_m,
                             'source': device.rx_source} if device.rx_lat is not None else None),
            'position': device.position, 'versions': device.versions, 'version': device.version,
            'auto_update': device.auto_update, 'uptime_s': device.uptime_s,
            'first_seen': device.first_seen.isoformat(),
            'last_report_at': device.last_report_at.isoformat() if device.last_report_at else None,
            'removed': device.removed, 'firmware_target': rollouts.target(device),
            'range_withheld': device.range_withheld, 'range_dropped': device.range_dropped,
            'bins_ignored': device.bins_ignored, 'check_position': device.check_position,
            'old_token': device.old_token}


def rollout_json(rollout: Rollout) -> dict:
    return {'id': rollout.pk, 'product': rollout.product, 'tag': rollout.tag, 'channel': rollout.channel,
            'scope': rollout.scope, 'label': rollout.label,
            'devices': list(rollout.devices.values_list('pk', flat=True)),
            'test_devices': list(rollout.test_devices.values_list('pk', flat=True)),
            'stage': rollout.stage, 'state': rollout.state, 'paused': rollout.paused,
            'cancelled_at': rollout.cancelled_at.isoformat() if rollout.cancelled_at else None,
            'note': rollout.note, 'created_at': rollout.created_at.isoformat(),
            'progress': rollouts.progress(rollout)}


def _devices(request):
    devices = request.account.devices.select_related('enroll_token')
    if request.GET.get('removed') != '1':
        devices = devices.filter(removed_at__isnull=True)
    return devices


# ─── Account ─────────────────────────────────────────────────────────────────

@user_api('GET')
def account(request):
    a = request.account
    current = a.enroll_token
    return _json({'username': a.user.get_username(), 'privileges': a.privileges,
                  'enroll_token_prefix': current.prefix if current else '',
                  'enroll_token_rotated_at': current.created_at.isoformat() if current else None,
                  'enroll_token_revealable': bool(current and current.sealed),
                  'report_url': request.build_absolute_uri('/').rstrip('/'),
                  'products': {key: value['label'] for key, value in conf.products().items()}})


def _secret(data: dict):
    response = _json(data)
    response['Cache-Control'] = 'no-store'
    return response


@user_api('POST')
def rotate_enroll_token(request):
    """A new token; old ones keep working for bees already on the account. {"cut_off": true}:
    every old token stops working."""
    cut_off = _body(request).get('cut_off', False)
    if not isinstance(cut_off, bool):
        raise services.SwarmError('cut_off is true or false')
    token = services.rotate_enroll_token(request.account, cut_off=cut_off, request=request)
    base = request.build_absolute_uri('/').rstrip('/')
    return _secret({'token': token, 'at_command': f'AT+SWARM=1,{base},{token}', 'cut_off': cut_off})


@user_api('POST', session_only=True)
def reveal_enroll_token(request):
    """The current token again: a signed-in session only. 404 unavailable: make a new one."""
    token = services.reveal_enroll_token(request.account, request=request)
    if token is None:
        return error(404, 'unavailable', 'make a new token to see it here')
    base = request.build_absolute_uri('/').rstrip('/')
    return _secret({'token': token, 'at_command': f'AT+SWARM=1,{base},{token}'})


@user_api('GET', 'POST')
def api_tokens(request):
    if request.method == 'POST':
        row, token = services.create_api_token(request.account, str(_body(request).get('name') or ''))
        return _json({'id': row.pk, 'name': row.name, 'token': token}, status=201)
    return _json([{'id': t.pk, 'name': t.name, 'prefix': t.prefix, 'created_at': t.created_at.isoformat(),
                   'last_used_at': t.last_used_at.isoformat() if t.last_used_at else None}
                  for t in request.account.api_tokens.order_by('-created_at')])


@user_api('DELETE')
def api_token(request, pk: int):
    get_object_or_404(ApiToken, pk=pk, account=request.account).delete()
    return _json({'deleted': pk})


# ─── Devices ─────────────────────────────────────────────────────────────────

@user_api('GET')
def devices(request):
    return _json([device_json(d) for d in _devices(request)])


@user_api('GET', 'PATCH', 'DELETE')
def device(request, pk: int):
    found = get_object_or_404(Device, pk=pk, account=request.account)
    if request.method == 'PATCH':
        services.edit_device(found, _body(request))
    elif request.method == 'DELETE':
        services.remove_device(found)
    return _json(device_json(found))


@user_api('POST')
def device_restore(request, pk: int):
    return _json(device_json(services.restore_device(get_object_or_404(Device, pk=pk, account=request.account))))


@user_api('GET')
def range_map(request):
    """?device=<id> (repeatable; none: the whole swarm), since, until, protocol."""
    since, until = _window(request)
    protocol = _protocol(request)
    chosen = _devices(request)
    ids = request.GET.getlist('device')
    if ids:
        try:
            chosen = chosen.filter(pk__in=[int(i) for i in ids])
        except ValueError as exc:
            raise services.SwarmError('device is an id') from exc
        if chosen.count() != len(set(ids)):
            return error(404, 'not_found')
    return _json(queries.swarm_range(chosen, since, until, protocol))


@user_api('GET')
def stats(request):
    """?device=<id> (repeatable), since, until, step=hour|day."""
    since, until = _window(request)
    step = request.GET.get('step', 'hour')
    if step not in ('hour', 'day'):
        raise services.SwarmError('step is hour or day')
    chosen = _devices(request)
    ids = request.GET.getlist('device')
    if ids:
        try:
            chosen = chosen.filter(pk__in=[int(i) for i in ids])
        except ValueError as exc:
            raise services.SwarmError('device is an id') from exc
    return _json({'since': since.isoformat(), 'until': until.isoformat(), 'step': step,
                  'series': queries.stats_series(list(chosen), since, until, step)})


# ─── Rollouts ────────────────────────────────────────────────────────────────

@user_api('GET', 'POST')
def rollout_list(request):
    if request.method == 'POST':
        return _json(rollout_json(services.create_rollout(request.account, _body(request))), status=201)
    return _json([rollout_json(r) for r in request.account.rollouts.all()])


@user_api('GET', 'POST')
def rollout_detail(request, pk: int):
    """POST {"action": "promote" | "pause" | "resume" | "cancel"}."""
    found = get_object_or_404(Rollout, pk=pk, account=request.account)
    if request.method == 'POST':
        services.rollout_action(found, str(_body(request).get('action') or ''))
    return _json(rollout_json(found))
