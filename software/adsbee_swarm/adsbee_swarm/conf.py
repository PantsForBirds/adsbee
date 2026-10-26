"""
Settings, each overridable in the host project's settings as ADSBEE_SWARM_<NAME>.

    ADSBEE_SWARM_DEVICE_VERIFIER = 'myapp.keys.verify'   # adsbee.aero: the OTA-key check
"""
from __future__ import annotations

from django.conf import settings
from django.utils.module_loading import import_string

DEFAULTS = {
    #: Products a report may name, and the `versions` key that holds the firmware version.
    'PRODUCTS': {
        'adsbee_1090': {'label': 'ADSBee 1090', 'version_key': 'rp2040'},
        'adsbee_1421': {'label': 'ADSBee 1421', 'version_key': 'cc1314'},
    },
    #: Privilege flags an account may hold. `swarm` opens the swarm pages and APIs.
    'PRIVILEGES': {'swarm': 'Swarm', 'wiggler': 'Pluto wiggler'},
    #: Privileges a new account gets unless told otherwise.
    'DEFAULT_PRIVILEGES': ['swarm'],
    #: Dotted path to `verify(request, report: dict, account) -> bool`; None accepts every device.
    #: False answers 403 bad_key; raising adsbee_swarm.VerifierUnavailable answers 503.
    'DEVICE_VERIFIER': None,
    #: Retry-After (seconds) when the verifier raises VerifierUnavailable without its own.
    'VERIFIER_RETRY_AFTER_S': 3600,
    #: Fernet key (or a list: the first encrypts, any decrypts) that keeps the current enroll token
    #: readable for its owner. None: derived from SECRET_KEY (changing that hides tokens until the
    #: next new one; they keep working).
    'TOKEN_KEY': None,
    #: Dotted path to `audit(account, event, request | None, **data)`, called on enroll_token_new,
    #: enroll_token_cut_off, enroll_token_revealed. Never given a token. Events are also logged
    #: to `adsbee_swarm.audit`.
    'AUDIT': None,
    #: Dotted path to `validate_tag(product, tag) -> str | None` (an error), for rollout targets.
    'TAG_VALIDATOR': None,
    #: Seconds between reports, told to every device.
    'REPORT_PERIOD_S': 3600,
    #: Hourly rows older than this many days are deleted (`manage.py swarm_retention`); daily rows stay.
    'HOURLY_RETENTION_DAYS': 90,
    #: Daily rows older than this are deleted. None keeps them.
    'DAILY_RETENTION_DAYS': None,
    #: "<count>/<seconds>" in the default cache. Per client IP before the token is read, then per
    #: device, per account (user API).
    'RATE_LIMITS': {'report_ip': '120/60', 'report_device': '12/3600', 'user_api': '600/60'},
    #: Request bodies larger than this are refused.
    'MAX_REPORT_BYTES': 16 * 1024,
    #: Header that carries the client IP behind a proxy (e.g. 'X-Forwarded-For'); None: REMOTE_ADDR.
    'CLIENT_IP_HEADER': None,
}


def get(name: str):
    return getattr(settings, f'ADSBEE_SWARM_{name}', DEFAULTS[name])


def hook(name: str):
    path = get(name)
    return import_string(path) if path else None


def products() -> dict:
    return get('PRODUCTS')
