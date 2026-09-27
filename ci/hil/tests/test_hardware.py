"""Bench tests against real hardware. Skipped unless ADSBEE_HIL_CONFIG names a bench file.

    ADSBEE_HIL_CONFIG=~/bench.toml pytest -m hil ci/hil/tests/test_hardware.py [-k 1421]

Read-only except for the loopback test, which switches each receiver's console to RAW output for
a few seconds (restored afterwards) and makes the configured transmitters transmit.
"""

import os

import pytest

from adsbee_hil import config, registry
from adsbee_hil.loopback import rf_loopback
from adsbee_hil.transmitters import PATTERNS

pytestmark = pytest.mark.hil

CONFIG = os.environ.get("ADSBEE_HIL_CONFIG")


def _receivers():
    if not CONFIG:
        return []
    bench = config.load(CONFIG, required=True)
    return [registry.make_receiver(r, bench) for r in bench.receivers]


@pytest.fixture(params=_receivers() or [None], ids=lambda r: r.id if r else "no-bench")
def receiver(request):
    if request.param is None:
        pytest.skip("set ADSBEE_HIL_CONFIG to a bench file to run hardware tests")
    if request.param.state() != "app":
        pytest.skip(f"{request.param.id} is {request.param.state()}")
    return request.param


def test_identity(receiver):
    info = receiver.device_info()
    assert receiver.version_key in info, info
    assert not any(k.startswith("OTA Key") for k in info)


def test_uptime_advances(receiver):
    import time

    a = receiver.uptime()
    time.sleep(2)
    b = receiver.uptime()
    assert a is not None and b is not None and b > a  # no reset between commands


@pytest.mark.parametrize("pattern", ["mode_s_table_100", "df17_unique_10"])
def test_rf_loopback(receiver, pattern):
    bench = receiver.bench
    txs = registry.transmitters_for(bench, receiver.id)
    res = rf_loopback(receiver, txs[0] if txs else None, PATTERNS[pattern])
    if res.status == "skip":
        pytest.skip(res.detail)
    assert res.status == "pass", res.summary()
