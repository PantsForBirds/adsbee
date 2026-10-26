"""
The reference UI: plain server-rendered pages so a self-hosted server is usable as it comes.
Host sites with their own look include only `adsbee_swarm.api_urls` and build on the API.
"""
from __future__ import annotations

import math
from datetime import timedelta
from functools import wraps

from django.contrib import messages
from django.contrib.auth.decorators import login_required
from django.http import HttpResponseForbidden
from django.shortcuts import get_object_or_404, redirect, render
from django.urls import reverse
from django.utils import timezone
from django.views.decorators.http import require_POST

from . import conf, queries, ranges, rollout as rollouts, services
from .api_user import device_json
from .models import Device, Rollout

WINDOWS = {'24h': timedelta(hours=24), '7d': timedelta(days=7), '30d': timedelta(days=30), '1y': timedelta(days=365)}
#: The protocol picker: (key, symbol, tooltip).
PROTOCOL_TABS = [('aircraft', '✈', 'All aircraft: Mode S, UAT and older firmware'), ('mode_s', 'S', 'Mode S (1090 MHz)'),
                 ('uat', 'UAT', 'UAT (978 MHz)'), ('remote_id', 'RID', 'Remote ID drones (2.4 GHz), height above the bee')]


def swarm_page(func):
    @login_required
    @wraps(func)
    def inner(request, *args, **kwargs):
        account = services.account_for(request.user)
        if account is None or not account.has('swarm'):
            return HttpResponseForbidden('This login has no swarm.')
        request.account = account
        return func(request, *args, **kwargs)
    return inner


def _window(request) -> tuple[str, timezone.datetime, timezone.datetime]:
    key = request.GET.get('w') if request.GET.get('w') in WINDOWS else '7d'
    until = timezone.now()
    return key, until - WINDOWS[key], until


def _protocol(request) -> str:
    protocol = request.GET.get('p')
    return protocol if protocol in ranges.PROTOCOLS else ranges.LEGACY


def _picker(request, protocol: str, have: list[str]) -> dict:
    """Template context for the window and protocol links; tabs without data are hidden."""
    return {'protocol': protocol,
            'tabs': [{'key': k, 'symbol': s, 'title': t} for k, s, t in PROTOCOL_TABS if k in have or k == protocol]}


@swarm_page
def device_list(request):
    devices = list(request.account.devices.filter(removed_at__isnull=True).select_related('enroll_token'))
    key, since, until = _window(request)
    protocol = _protocol(request)
    payload = queries.swarm_range(devices, since, until, protocol)
    reach = {d['id']: d['max_range_m'] for d in payload['devices']}
    rows = [{**device_json(d), 'max_range_m': reach.get(d.pk), 'max_range_km': (reach.get(d.pk) or 0) / 1000,
             'last_dt': d.last_report_at} for d in devices]
    removed = request.account.devices.filter(removed_at__isnull=False)
    return render(request, 'adsbee_swarm/devices.html', {
        'rows': rows, 'removed': removed, 'window': key, 'windows': WINDOWS,
        **_picker(request, protocol, payload['protocols']),
        'markers': [{'id': r['id'], 'url': reverse('adsbee_swarm:device', args=[r['id']]),
                     'label': r['label'] or r['unique_id'], 'lat': r['position']['lat'],
                     'lon': r['position']['lon'], 'range_m': r['max_range_m']} for r in rows if r['position']],
        'stale_before': timezone.now() - timedelta(hours=3)})


@swarm_page
def device_detail(request, pk: int):
    device = get_object_or_404(Device, pk=pk, account=request.account)
    if request.method == 'POST':
        action = request.POST.get('action')
        try:
            if action == 'save':
                location = None
                if request.POST.get('lat') or request.POST.get('lon'):
                    location = {'lat': request.POST.get('lat'), 'lon': request.POST.get('lon'),
                                'alt_m': request.POST.get('alt_m')}
                services.edit_device(device, {'label': request.POST.get('label', ''),
                                              'note': request.POST.get('note', ''), 'location': location})
            elif action == 'remove':
                services.remove_device(device)
                return redirect('adsbee_swarm:devices')
            elif action == 'restore':
                services.restore_device(device)
        except services.SwarmError as exc:
            messages.error(request, str(exc))
        return redirect('adsbee_swarm:device', pk=device.pk)
    key, since, until = _window(request)
    protocol = _protocol(request)
    merged = queries.device_range(device, since, until, protocol)
    scale = max(merged.values(), default=0)
    plots = [{'title': _band_title(band, protocol), 'svg': polar_svg(_band(merged, band), scale)}
             for band in range(ranges.BANDS)]
    return render(request, 'adsbee_swarm/device.html', {
        'device': device, 'info': device_json(device), 'window': key, 'windows': WINDOWS,
        **_picker(request, protocol, queries.protocols_with_data([device], since, until)),
        'height_above_rx': ranges.PROTOCOLS[protocol].basis == 'height_above_rx',
        'overall': polar_svg(ranges.sector_max(merged), scale, size=320), 'plots': plots,
        'max_km': _km(scale) if scale else None,
        'charts': _charts(queries.stats_series([device], since, until, 'hour' if key in ('24h', '7d') else 'day')),
        'target': rollouts.target(device)})


def _band(merged: ranges.Map, band: int) -> list[int | None]:
    return [merged.get(ranges.bin_index(band, sector)) for sector in range(ranges.SECTORS)]


def _band_title(band: int, protocol: str = ranges.LEGACY) -> str:
    low, high = ranges.band_edges_ft(protocol)[band]
    if ranges.PROTOCOLS[protocol].basis == 'height_above_rx':
        return f"↑ {low}–{high} ft" if high else f"↑ ≥{low} ft"
    return f"{low // 1000}–{high // 1000}k ft" if high else f"≥{low // 1000}k ft"


def _km(meters: float) -> str:
    """Whole km, or one decimal under 10 km (Remote ID)."""
    return f'{meters / 1000:.0f}' if meters >= 10000 else f'{meters / 1000:.1f}'


def polar_svg(values: list[int | None], scale: int, size: int = 140) -> str:
    """36 wedges, north up, radius = range / scale; rings every 50 km (or a quarter of the scale)."""
    half = size / 2
    radius = half - 4
    parts = [f'<svg viewBox="0 0 {size} {size}" width="{size}" height="{size}" class="polar" role="img">',
             f'<circle cx="{half}" cy="{half}" r="{radius}" class="ring"/>']
    if scale:
        step = 50000 if scale > 100000 else max(scale / 4, 1)
        ring = step
        while ring < scale:
            parts.append(f'<circle cx="{half}" cy="{half}" r="{radius * ring / scale:.1f}" class="ring">'
                         f'<title>{_km(ring)} km</title></circle>')
            ring += step
    parts.append(f'<line x1="{half}" y1="2" x2="{half}" y2="{size - 2}" class="axis"/>'
                 f'<line x1="2" y1="{half}" x2="{size - 2}" y2="{half}" class="axis"/>')
    for sector, meters in enumerate(values):
        if not meters or not scale:
            continue
        r = radius * meters / scale
        a0, a1 = math.radians(sector * ranges.SECTOR_DEG), math.radians((sector + 1) * ranges.SECTOR_DEG)
        x0, y0 = half + r * math.sin(a0), half - r * math.cos(a0)
        x1, y1 = half + r * math.sin(a1), half - r * math.cos(a1)
        parts.append(f'<path d="M{half},{half} L{x0:.1f},{y0:.1f} A{r:.1f},{r:.1f} 0 0 1 {x1:.1f},{y1:.1f} Z" '
                     f'class="wedge"><title>{sector * 10}–{sector * 10 + 10}°: {_km(meters)} km</title></path>')
    parts.append(f'<text x="{half}" y="11" class="north">N</text></svg>')
    return ''.join(parts)


CHARTS = [('es_valid', 'ES', 'Extended squitters decoded'), ('modes_valid', 'S', 'Mode S decoded'),
          ('uat_adsb', 'UAT', 'UAT ADS-B decoded'), ('aircraft_max', '✈', 'Most aircraft at once')]


def _charts(series: list[dict]) -> list[dict]:
    first = series[0]['t'][:16].replace('T', ' ') if series else ''
    last = series[-1]['t'][:16].replace('T', ' ') if series else ''
    return [{'label': label, 'title': title, 'first': first, 'last': last,
             'svg': bars_svg([(row['t'][:16].replace('T', ' '), row[key]) for row in series])}
            for key, label, title in CHARTS]


def bars_svg(points: list[tuple[str, int]], width: int = 540, height: int = 70) -> str:
    """One bar per step, scaled to the peak; each bar's tooltip says when and how many."""
    top = max((value for _t, value in points), default=0)
    parts = [f'<svg viewBox="0 0 {width} {height}" class="bars" preserveAspectRatio="none" role="img">']
    if top and points:
        step = width / len(points)
        for i, (when, value) in enumerate(points):
            h = (height - 2) * value / top
            parts.append(f'<rect x="{i * step:.2f}" y="{height - h:.2f}" width="{max(step - .5, .5):.2f}" '
                         f'height="{h:.2f}"><title>{when}: {value:,}</title></rect>')
    parts.append('</svg>')
    return ''.join(parts)


@swarm_page
def rollout_page(request):
    account = request.account
    if request.method == 'POST':
        try:
            if request.POST.get('action') == 'create':
                services.create_rollout(account, {
                    'product': request.POST.get('product'), 'tag': request.POST.get('tag'),
                    'scope': request.POST.get('scope'), 'label': request.POST.get('label'),
                    'devices': [int(i) for i in request.POST.getlist('devices')],
                    'test_devices': [int(i) for i in request.POST.getlist('test_devices')],
                    'note': request.POST.get('note', '')})
            else:
                services.rollout_action(get_object_or_404(Rollout, pk=request.POST.get('rollout'), account=account),
                                        request.POST.get('action', ''))
        except (services.SwarmError, ValueError) as exc:
            messages.error(request, str(exc))
        return redirect('adsbee_swarm:rollouts')
    return render(request, 'adsbee_swarm/rollouts.html', {
        'rollouts': [{'r': r, 'progress': rollouts.progress(r)} for r in account.rollouts.all()[:50]],
        'devices': account.devices.filter(removed_at__isnull=True),
        'labels': sorted({d.label for d in account.devices.all() if d.label}),
        'products': conf.products(), 'scopes': Rollout.SCOPES})


def _account_context(request, **extra) -> dict:
    account = request.account
    return {'account': account, 'current': account.enroll_token,
            'base': request.build_absolute_uri('/').rstrip('/'),
            'tokens': account.api_tokens.order_by('-created_at'),
            'old_token_bees': account.devices.filter(removed_at__isnull=True, enroll_token__retired_at__isnull=False,
                                                     enroll_token__cut_off=False).count(), **extra}


def _no_store(response):
    response['Cache-Control'] = 'no-store'
    return response


@swarm_page
def account_page(request):
    return render(request, 'adsbee_swarm/account.html', _account_context(request))


@swarm_page
@require_POST
def account_action(request):
    action = request.POST.get('action')
    context = {}
    if action in ('rotate', 'cut_off'):
        context['enroll_token'] = services.rotate_enroll_token(request.account, cut_off=action == 'cut_off',
                                                               request=request)
        context['made'] = action
    elif action == 'reveal':
        context['enroll_token'] = services.reveal_enroll_token(request.account, request=request)
        context['unavailable'] = context['enroll_token'] is None
    elif action == 'api_token':
        context['api_token'] = services.create_api_token(request.account, request.POST.get('name', ''))[1]
    elif action == 'delete_token':
        request.account.api_tokens.filter(pk=request.POST.get('token')).delete()
        return redirect('adsbee_swarm:account')
    return _no_store(render(request, 'adsbee_swarm/account.html', _account_context(request, **context)))
