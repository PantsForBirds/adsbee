import contextlib

import pytest

from adsbee_hil.config import ReceiverConfig, TransmitterConfig
from adsbee_hil.loopback import rf_loopback
from adsbee_hil.receivers import Receiver
from adsbee_hil.transmitters import BeeWiggler, TestPattern, Transmitter, TransmitterError


def wiggler(**opts):
    return BeeWiggler(TransmitterConfig("w", "bee_wiggler", usb_serial="W1", options=opts))


def test_wiggler_commands():
    w = wiggler(mode_s_atten_db=60)
    assert w.commands_for(TestPattern(band="1090", count=100, rate=20)) == [
        "AT+MODE_S_ATTEN=60", "AT+MODE_S_TABLE_TX=100,20"]
    assert w.commands_for(TestPattern(band="1090", count=-1, rate=5, power_dbm=-10, atten_db=30)) == [
        "AT+MODE_S_ATTEN=30", "AT+MODE_S_TABLE_TX=-1,5,-10"]
    assert w.commands_for(TestPattern(band="1090", count=10, rate=2, messages=["8D00"])) == [
        "AT+MODE_S_ATTEN=60", "AT+MODE_S_TX=8D00,9,2"]
    assert w.commands_for(TestPattern(band="978", count=50, rate=10)) == ["AT+UAT_ADSB_TABLE_TX=50,10"]
    assert w.commands_for(TestPattern(band="dual", count=25, rate=10)) == [
        "AT+MODE_S_ATTEN=60", "AT+DUAL_TABLE_TX=3,10,10,ADSB"]
    assert w.commands_for(TestPattern(band="dual", count=-1, rate=10))[-1] == "AT+DUAL_TABLE_TX=-1,10,10,ADSB"
    with pytest.raises(TransmitterError):
        w.commands_for(TestPattern(band="978", messages=["00"]))
    with pytest.raises(TransmitterError):
        w.commands_for(TestPattern(band="1090", messages=["8D00", "8D01"]))


def test_wiggler_needs_serial():
    with pytest.raises(TransmitterError):
        BeeWiggler(TransmitterConfig("w", "bee_wiggler"))


def test_wiggler_plays_on_fake_console(fake_sysfs, fake_at):
    dev = fake_at({"AT+MODE_S_ATTEN=60": "", "AT+MODE_S_TABLE_TX=3,100": "sent 3"})
    fake_sysfs.add("1-1.6", "2e8a", "000a", "W1", tty_name="ttyACM5", by_id="usb-x_W1-if00",
                   tty_target=dev.path)
    w = wiggler(mode_s_atten_db=60)
    assert w.available()
    w.transmit(TestPattern(band="1090", count=3, rate=100))
    assert dev.received == ["AT+MODE_S_ATTEN=60", "AT+MODE_S_TABLE_TX=3,100"]


class FakeRx(Receiver):
    model = "fake"

    def __init__(self, lines):
        super().__init__(ReceiverConfig("rx", "fake", "R1"))
        self.lines = lines

    @contextlib.contextmanager
    def capture(self, protocol="RAW"):
        class Cap:
            pass
        cap = Cap()
        cap.frames = []
        yield cap
        from adsbee_hil.packets import parse_raw_frames
        cap.frames = parse_raw_frames(self.lines)


class FakeTx(Transmitter):
    def __init__(self, attached=True):
        super().__init__(TransmitterConfig("tx", "fake"))
        self.attached = attached
        self.played = []

    def available(self):
        return self.attached

    def start(self, pattern):
        self.played.append(pattern)
        self._active = pattern

    def wait(self, timeout=None):
        self._active = None

    def stop(self):
        self._active = None


def test_loopback_skips_without_transmitter():
    rx = FakeRx([])
    assert rf_loopback(rx, None, TestPattern()).status == "skip"
    assert rf_loopback(rx, FakeTx(attached=False), TestPattern()).status == "skip"


def test_loopback_explicit_messages_match_exactly():
    from adsbee_hil.packets import unique_df17_set

    msgs = unique_df17_set(4)
    lines = [f"#MDS*{m};(1,-60,0,0)" for m in msgs * 3]  # 12 of the sent frames...
    lines += ["#MDS*8D4840D6202CC371C32CE0576098;(1,-80,0,0)"] * 50  # ...plus background traffic
    p = TestPattern(band="1090", count=20, rate=100, messages=msgs)
    tx = FakeTx()
    res = rf_loopback(FakeRx(lines), tx, p, min_fraction=0.5, settle_s=0)
    assert tx.played == [p]
    assert (res.status, res.sent, res.received, res.matched, res.distinct_icao) == ("pass", 20, 62, 12, 5)
    res = rf_loopback(FakeRx(lines), FakeTx(), p, min_fraction=0.9, settle_s=0)
    assert res.status == "fail"  # background traffic doesn't count


def test_loopback_table_pattern_counts_band():
    lines = ["#UAT*-00A1B2C3;(0,0,0)"] * 30 + ["#MDS*8D4840D6202CC371C32CE0576098;(1,0,0,0)"] * 30
    res = rf_loopback(FakeRx(lines), FakeTx(), TestPattern(band="978", count=40, rate=100), settle_s=0)
    assert (res.status, res.received) == ("pass", 30)
    res = rf_loopback(FakeRx(lines), FakeTx(), TestPattern(band="dual", count=40, rate=100), settle_s=0)
    assert (res.sent, res.received, res.status) == (80, 60, "pass")
