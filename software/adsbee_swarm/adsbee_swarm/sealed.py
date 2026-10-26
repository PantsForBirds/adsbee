"""
The current enroll token, encrypted at rest so its owner can see it again (Fernet).

Key: setting ADSBEE_SWARM_TOKEN_KEY (a Fernet key, or a list of them: the first encrypts, any
decrypts), else one derived from SECRET_KEY with HKDF. A changed key only makes old tokens
unreadable: they keep working, and a new token can be read again.
"""
from __future__ import annotations

import base64

from cryptography.fernet import Fernet, InvalidToken, MultiFernet
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.kdf.hkdf import HKDF
from django.conf import settings

from . import conf

#: HKDF context: a key derived for this and nothing else.
CONTEXT = b'adsbee_swarm enroll token v1'


def derived_key(secret: str) -> bytes:
    key = HKDF(algorithm=hashes.SHA256(), length=32, salt=None, info=CONTEXT).derive(secret.encode())
    return base64.urlsafe_b64encode(key)


def _fernet() -> MultiFernet:
    keys = conf.get('TOKEN_KEY')
    if not keys:
        keys = [derived_key(settings.SECRET_KEY)]
    elif isinstance(keys, (str, bytes)):
        keys = [keys]
    return MultiFernet([Fernet(k) for k in keys])


def seal(token: str) -> str:
    return _fernet().encrypt(token.encode()).decode()


def unseal(sealed: str) -> str | None:
    """The token, or None: nothing kept, or the key changed."""
    if not sealed:
        return None
    try:
        return _fernet().decrypt(sealed.encode()).decode()
    except (InvalidToken, ValueError):
        return None
