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


# ── pwiggd units (Pluto Wiggler, Bee Waggler): firmware 1.x keeps the Bee Wiggler set, 2.x the AT+TX jobs ──

from adsbee_hil.transmitters import CMD_SET_1, CMD_SET_2, bee_quantize_atten, command_set  # noqa: E402

BEE_INFO = ("Device: ADSBee Wiggler\r\nRP2040 Flash Unique ID: E66138935F4C2B2D\r\n"
            "RP2040 Firmware Version: 0.3.1\r\nSub-GHz CC1312 Firmware Version: 0.3.0")
PWIGGD1_INFO = ("Device: ADSBee Wiggler\r\nPluto Unique ID: 0123456789ABCDEF\r\n"
                "Pluto Wiggler Firmware Version: 1.4.0\r\n"
                "Variant: Bee Waggler (ZynqSDR 7020) AD9363, HDL 065c8f18, gateware 1.4.0\r\nFirmware Flavor: debug")
PWIGGD2_INFO = ("Device: ADSBee Wiggler\r\nProduct: Bee Waggler\r\nUnique ID: 0123456789ABCDEF\r\n"
                "Firmware Version: 2.0.0-dev\r\nVariant: Bee Waggler (ZynqSDR 7020) AD9363, HDL x\r\n"
                "Gateware: 1.5.0\r\nPorts: 2\r\nReceivers: none\r\nFirmware Flavor: debug")
PLUTO2_INFO = PWIGGD2_INFO.replace("Product: Bee Waggler", "Product: Pluto Wiggler").replace("Ports: 2", "Ports: 1")


def test_command_set_from_device_info():
    assert command_set(BEE_INFO) == CMD_SET_1
    assert command_set(PWIGGD1_INFO) == CMD_SET_1
    assert command_set(PWIGGD2_INFO) == CMD_SET_2
    assert command_set(PLUTO2_INFO) == CMD_SET_2
    assert command_set("Device: ADSBee Wiggler\r\nFirmware Version: 2.1.0") == CMD_SET_2  # no Product line
    assert command_set("RP2040 Firmware Version: 2.0.0") == CMD_SET_1  # a Bee Wiggler stays on its set


def test_bee_quantize_atten_matches_pe4312_steps():
    assert [bee_quantize_atten(x) for x in (0, 0.2, 60, 60.3, 125.9, 200, -3)] == [0, 0, 60, 60.5, 126, 126, 0]


def test_wiggler_commands_v2_bee_waggler():
    w = wiggler(mode_s_atten_db=60, subg_atten_db=50.3)
    v2 = lambda p: w.commands_for_v2(p, ports=2)  # noqa: E731
    # Same level as 1.x: reference +5 dBm (or power_dbm) minus the Bee-quantized attenuation.
    assert v2(TestPattern(band="1090", count=100, rate=20)) == ["AT+TX=BOTH,MODE_S,TABLE,100,20,-55"]
    assert v2(TestPattern(band="1090", count=-1, rate=5, power_dbm=-10, atten_db=30)) == [
        "AT+TX=BOTH,MODE_S,TABLE,-1,5,-40"]
    assert v2(TestPattern(band="1090", count=10, rate=2, messages=["8D00"])) == ["AT+TX=BOTH,MODE_S,8D00,10,2,-55"]
    assert v2(TestPattern(band="978", count=50, rate=10)) == ["AT+TX=BOTH,UAT,TABLE,50,10,-45.5"]
    # two ports: every job on both (1.x mirrored TX1 to TX2)
    # dual at a finite rate: time-shared, started on the same sample
    assert v2(TestPattern(band="dual", count=25, rate=10)) == [
        "AT+HOLD=1", "AT+TX=BOTH,MODE_S,TABLE,25,10,-55", "AT+TX=BOTH,UAT,TABLE,25,10,-45.5", "AT+HOLD=0"]
    # dual as fast as possible: the overlap needs the fixed-LO plan, opted in for this run
    assert v2(TestPattern(band="dual", count=-1, rate=0)) == [
        "AT+SIMULTANEOUS=MODE_S,UAT,ACCEPT_COSTS", "AT+HOLD=1", "AT+TX=1,MODE_S,TABLE,-1,0,-55",
        "AT+TX=2,UAT,TABLE,-1,0,-45.5", "AT+HOLD=0"]  # the fixed-LO plan: Mode S on 1, UAT on 2
    # several explicit messages: an AT+SCHED file, looped
    msgs = ["8D00", "8D01", "8D02"]
    assert v2(TestPattern(band="1090", count=7, rate=100, messages=msgs)) == [
        "AT+SCHED=CLEAR", "AT+SCHED=ADD,BOTH,MODE_S,0,-55,8D00", "AT+SCHED=ADD,BOTH,MODE_S,10000,-55,8D01",
        "AT+SCHED=ADD,BOTH,MODE_S,20000,-55,8D02", "AT+TX=BOTH,FILE,sched,3"]
    with pytest.raises(TransmitterError):
        v2(TestPattern(band="1090", count=7, rate=0, messages=msgs))
    with pytest.raises(TransmitterError):
        v2(TestPattern(band="978", messages=["00"]))


def test_wiggler_commands_v2_pluto_wiggler_and_options():
    w = wiggler()  # no attenuation in the bench file: the unit's AT+LEVEL default
    assert w.commands_for_v2(TestPattern(band="978", count=5, rate=1), ports=1) == ["AT+TX=1,UAT,TABLE,5,1"]
    # one port: no simultaneous plan, both protocols time-shared on port 1
    assert w.commands_for_v2(TestPattern(band="dual", count=5, rate=0), ports=1) == [
        "AT+HOLD=1", "AT+TX=1,MODE_S,TABLE,5,0", "AT+TX=1,UAT,TABLE,5,0", "AT+HOLD=0"]
    w = wiggler(simultaneous=True, uat_port=1)
    assert w.commands_for_v2(TestPattern(band="dual", count=5, rate=10), ports=2)[0].startswith("AT+SIMULTANEOUS=")
    assert w.commands_for_v2(TestPattern(band="978", count=5, rate=1), ports=2) == ["AT+TX=1,UAT,TABLE,5,1"]


class _Jobs:
    """2.x replies: each AT+TX gets a job id, its +DONE comes right after OK (unsolicited)."""

    def __init__(self):
        self.n = 0

    def tx(self, cmd):
        self.n += 1
        return f"+TX:job={self.n}\r\nOK\r\n+DONE:{self.n},sent=3,late=0,dropped=0,reason=done\r\n"

    def tx_unbounded(self, cmd):
        self.n += 1
        return f"+TX:job={self.n}\r\nOK\r\n"

    def stop(self, cmd):
        out = "".join(f"+DONE:{i},sent=9,late=0,dropped=0,reason=stopped\r\n" for i in range(1, self.n + 1))
        self.n = 0
        return out + "OK\r\n"


def _pwiggd_unit(fake_sysfs, fake_at, replies, info=PWIGGD2_INFO):
    dev = fake_at({"AT+DEVICE_INFO?": info, **replies})
    # pwiggd gadget: ADI VID/PID, not an RP2040, so the driver asks for DEVICE_INFO
    fake_sysfs.add("1-1.7", "0456", "b673", "W1", tty_name="ttyACM6", by_id="usb-x_W1-if02", tty_target=dev.path)
    return dev


def test_wiggler_v2_plays_and_waits_for_done(fake_sysfs, fake_at):
    jobs = _Jobs()
    dev = _pwiggd_unit(fake_sysfs, fake_at, {"AT+CONTROL=TAKE": "+CONTROL=USB,1", "AT+STOP=ALL": "",
                                             "AT+TX=BOTH,MODE_S,TABLE,3,100,-55": jobs.tx})
    w = wiggler(mode_s_atten_db=60)
    w.transmit(TestPattern(band="1090", count=3, rate=100))
    assert dev.received == ["AT+DEVICE_INFO?", "AT+CONTROL=TAKE", "AT+STOP=ALL", "AT+TX=BOTH,MODE_S,TABLE,3,100,-55"]
    assert "reason=done" in w._out


def test_wiggler_v2_dual_unbounded_stops_and_leaves_simultaneous(fake_sysfs, fake_at):
    jobs = _Jobs()
    dev = _pwiggd_unit(fake_sysfs, fake_at, {
        "AT+CONTROL=TAKE": "+CONTROL=USB,1", "AT+STOP=ALL": jobs.stop,
        "AT+SIMULTANEOUS=MODE_S,UAT,ACCEPT_COSTS": "", "AT+SIMULTANEOUS=OFF": "",
        "AT+HOLD=1": "", "AT+HOLD=0": "",
        "AT+TX=1,MODE_S,TABLE,-1,0,-55": jobs.tx_unbounded, "AT+TX=2,UAT,TABLE,-1,0,-55": jobs.tx_unbounded})
    w = wiggler(mode_s_atten_db=60, subg_atten_db=60)
    with w.running(TestPattern(band="dual", count=-1, rate=0)):
        pass
    assert dev.received[-3:] == ["AT+HOLD=0", "AT+STOP=ALL", "AT+SIMULTANEOUS=OFF"]
    assert dev.received[3] == "AT+SIMULTANEOUS=MODE_S,UAT,ACCEPT_COSTS"


def test_wiggler_v2_job_error_raises(fake_sysfs, fake_at):
    _pwiggd_unit(fake_sysfs, fake_at, {
        "AT+CONTROL=TAKE": "+CONTROL=USB,1", "AT+STOP=ALL": "",
        "AT+TX=BOTH,UAT,TABLE,3,10": lambda c: "+TX:job=4\r\nOK\r\n+DONE:4,sent=1,late=0,dropped=0,reason=conflict\r\n"})
    with pytest.raises(TransmitterError, match="reason=conflict"):
        wiggler().transmit(TestPattern(band="978", count=3, rate=10))


def test_wiggler_v2_refused_command_raises(fake_sysfs, fake_at):
    _pwiggd_unit(fake_sysfs, fake_at, {"AT+CONTROL=TAKE": "+CONTROL=USB,1", "AT+STOP=ALL": "",
                                       "AT+TX=BOTH,MODE_S,TABLE,3,10": lambda c: "ERROR unlicensed\r\n"})
    with pytest.raises(TransmitterError, match="unlicensed"):
        wiggler().transmit(TestPattern(band="1090", count=3, rate=10))


def test_wiggler_pwiggd_1x_keeps_bee_commands(fake_sysfs, fake_at):
    dev = _pwiggd_unit(fake_sysfs, fake_at, {"AT+MODE_S_ATTEN=60": "", "AT+MODE_S_TABLE_TX=3,100": "sent 3"},
                       info=PWIGGD1_INFO)
    wiggler(mode_s_atten_db=60).transmit(TestPattern(band="1090", count=3, rate=100))
    assert dev.received == ["AT+DEVICE_INFO?", "AT+MODE_S_ATTEN=60", "AT+MODE_S_TABLE_TX=3,100"]
