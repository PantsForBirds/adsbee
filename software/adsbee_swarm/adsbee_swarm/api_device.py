"""
POST /api/swarm/v1/report: what a bee sends every period (docs/swarm_api.md, "Device report").

401 bad_token (unknown, cut off, or an old token from a bee that wasn't on the account when it
was replaced), 403 bad_key (the DEVICE_VERIFIER said no), 503 unavailable (it could not decide;
Retry-After), 410 removed, 429 rate_limited (Retry-After), 400 bad_request. Errors are {"error": code, "detail"?: text}.
"""
from __future__ import annotations

import json
import logging

from django.db import IntegrityError
from django.http import JsonResponse
from django.views.decorators.csrf import csrf_exempt
from django.views.decorators.http import require_POST

from . import VerifierUnavailable, conf, protocol, ratelimit, rollout, services
from .models import Device

log = logging.getLogger('adsbee_swarm.report')


def error(status: int, code: str, detail: str = '', retry_after: int | None = None) -> JsonResponse:
    body = {'error': code, **({'detail': detail} if detail else {})}
    response = JsonResponse(body, status=status)
    if retry_after is not None:
        response['Retry-After'] = str(retry_after)
    return response


def client_ip(request) -> str:
    header = conf.get('CLIENT_IP_HEADER')
    if header:
        forwarded = request.headers.get(header, '')
        if forwarded:
            return forwarded.split(',')[0].strip()
    return request.META.get('REMOTE_ADDR', '')


def limited(name: str, ident: str) -> JsonResponse | None:
    wait = ratelimit.hit(name, ident)
    return error(429, 'rate_limited', retry_after=wait) if wait else None


@csrf_exempt
@require_POST
def report(request):
    refused = limited('report_ip', client_ip(request))
    if refused:
        return refused
    header = request.headers.get('Authorization', '')
    token = services.enroll_token_by(header[7:].strip()) if header.startswith('Bearer ') else None
    account = token.account if token else None
    if account is None or not account.has('swarm'):
        return error(401, 'bad_token')
    if len(request.body) > conf.get('MAX_REPORT_BYTES'):
        return error(400, 'bad_request', 'body too large')
    try:
        data = json.loads(request.body or b'null')
        parsed = protocol.parse(data)
    except (ValueError, UnicodeDecodeError) as exc:
        return error(400, 'bad_request', str(exc)[:200])

    known = Device.objects.filter(account=account, product=parsed.product, unique_id=parsed.unique_id).first()
    if not services.admits(token, known):
        return error(401, 'bad_token')

    verifier = conf.hook('DEVICE_VERIFIER')
    try:
        genuine = verifier is None or verifier(request, data, account)
    except VerifierUnavailable as exc:
        log.warning('verifier unavailable: %s', exc.detail)
        return error(503, 'unavailable', exc.detail,
                     retry_after=exc.retry_after or conf.get('VERIFIER_RETRY_AFTER_S'))
    if not genuine:
        log.info('bad_key account=%s device=%s:%s', account.pk, parsed.product, parsed.unique_id)
        return error(403, 'bad_key')

    device = known
    if device is None:
        try:
            device, _ = Device.objects.get_or_create(account=account, product=parsed.product,
                                                     unique_id=parsed.unique_id)
        except IntegrityError:      # the same bee's report racing itself
            device = Device.objects.get(account=account, product=parsed.product, unique_id=parsed.unique_id)
    if services.is_turned_away(device, token):
        return error(410, 'removed')
    refused = limited('report_device', str(device.pk))
    if refused:
        return refused
    protocol.store(device, parsed)
    Device.objects.filter(pk=device.pk).exclude(enroll_token=token).update(enroll_token=token)
    device.refresh_from_db()
    return JsonResponse({'next_report_s': conf.get('REPORT_PERIOD_S'), 'firmware': rollout.target(device)})
