import contextlib
import hashlib
import json

import pytest

from adsbee_hil.config import ReceiverConfig, TransmitterConfig
from adsbee_hil.loopback import rf_loopback
from adsbee_hil.packets import unique_df17_set, unique_uat_set
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

from adsbee_hil.transmitters import (CMD_SET_1, CMD_SET_2, FILE_CHUNK, bee_quantize_atten, command_set,  # noqa: E402
                                     file_upload_commands, packet_file)

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


MS = unique_df17_set(4)
UAT = unique_uat_set(2)


def test_wiggler_commands_v2_bee_waggler():
    w = wiggler(mode_s_atten_db=60, subg_atten_db=50.3)
    v2 = lambda p: w.commands_for_v2(p, ports=2)  # noqa: E731
    # Same level as 1.x: reference +5 dBm (or power_dbm) minus the Bee-quantized attenuation.
    assert v2(TestPattern(band="1090", count=100, rate=20)) == ["AT+TX=BOTH,MODE_S,TABLE,100,20,-55"]
    assert v2(TestPattern(band="1090", count=-1, rate=5, power_dbm=-10, atten_db=30)) == [
        "AT+TX=BOTH,MODE_S,TABLE,-1,5,-40"]
    assert v2(TestPattern(band="1090", count=10, rate=2, messages=[MS[0]])) == [f"AT+TX=BOTH,MODE_S,{MS[0]},10,2,-55"]
    assert v2(TestPattern(band="978", count=50, rate=10)) == ["AT+TX=BOTH,UAT,TABLE,50,10,-45.5"]
    # two ports: every job on both (1.x mirrored TX1 to TX2)
    # dual at a finite rate: time-shared, started on the same sample
    assert v2(TestPattern(band="dual", count=25, rate=10)) == [
        "AT+HOLD=1", "AT+TX=BOTH,MODE_S,TABLE,25,10,-55", "AT+TX=BOTH,UAT,TABLE,25,10,-45.5", "AT+HOLD=0"]
    # dual as fast as possible: the overlap needs the fixed-LO plan, opted in for this run
    assert v2(TestPattern(band="dual", count=-1, rate=0)) == [
        "AT+SIMULTANEOUS=MODE_S,UAT,ACCEPT_COSTS", "AT+HOLD=1", "AT+TX=1,MODE_S,TABLE,-1,0,-55",
        "AT+TX=2,UAT,TABLE,-1,0,-45.5", "AT+HOLD=0"]  # the fixed-LO plan: Mode S on 1, UAT on 2
    # several explicit messages, finite: every packet an AT+SCHED entry, played once (a looped sched wraps
    # back to back)
    msgs = [MS[0], MS[1], MS[2]]
    cmds = v2(TestPattern(band="1090", count=4, rate=100, messages=msgs))
    assert cmds == ["AT+SCHED=CLEAR", f"AT+SCHED=ADD,BOTH,MODE_S,0,-55,{MS[0]}",
                    f"AT+SCHED=ADD,BOTH,MODE_S,10000,-55,{MS[1]}", f"AT+SCHED=ADD,BOTH,MODE_S,20000,-55,{MS[2]}",
                    f"AT+SCHED=ADD,BOTH,MODE_S,30000,-55,{MS[0]}", "AT+TX=BOTH,FILE,sched,1"]  # mirrored like 1.x
    # unbounded (or over 256): a packet file whose period keeps the loops evenly spaced
    cmds = v2(TestPattern(band="1090", count=-1, rate=100, messages=msgs))
    assert cmds[0].startswith("AT+FILE=BEGIN,hil_mode_s,") and cmds[0].endswith(",jsonl")
    assert cmds[-2:] == ["AT+FILE=END", "AT+TX=BOTH,FILE,hil_mode_s,-1"]
    assert v2(TestPattern(band="1090", count=600, rate=100, messages=msgs))[-1] == "AT+TX=BOTH,FILE,hil_mode_s,200"
    with pytest.raises(TransmitterError):
        v2(TestPattern(band="1090", count=7, rate=0, messages=msgs))
    with pytest.raises(TransmitterError, match="not a Mode S"):
        v2(TestPattern(band="978", messages=["00"]))
    with pytest.raises(TransmitterError, match="Mode S messages only"):
        v2(TestPattern(band="1090", messages=[UAT[0]]))
    # explicit UAT messages (2.x): payload only, the unit adds the FEC
    assert v2(TestPattern(band="978", count=5, rate=10, messages=[UAT[0]])) == [
        f"AT+TX=BOTH,UAT,{UAT[0]},5,10,-45.5"]
    # dual with explicit messages: a Mode S and a UAT job under HOLD; several messages go in files (one sched)
    cmds = v2(TestPattern(band="dual", count=-1, rate=10, messages=msgs + [UAT[0]]))
    assert cmds[cmds.index("AT+HOLD=1"):] == [
        "AT+HOLD=1", "AT+TX=BOTH,FILE,hil_mode_s,-1", f"AT+TX=BOTH,UAT,{UAT[0]},-1,10,-45.5", "AT+HOLD=0"]
    assert all(c.startswith("AT+FILE=") for c in cmds[:cmds.index("AT+HOLD=1")])
    with pytest.raises(TransmitterError, match="needs Mode S and UAT"):
        v2(TestPattern(band="dual", messages=msgs))


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
    replies.setdefault("AT+DEVICE_INFO?", info)
    dev = fake_at(replies)
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
    dev.replies["AT+SIMULTANEOUS?"] = SIMUL_Q
    w = wiggler(mode_s_atten_db=60, subg_atten_db=60)
    with w.running(TestPattern(band="dual", count=-1, rate=0)):
        pass
    assert dev.received[-3:] == ["AT+HOLD=0", "AT+STOP=ALL", "AT+SIMULTANEOUS=OFF"]
    assert dev.received[3:5] == ["AT+SIMULTANEOUS?", "AT+SIMULTANEOUS=MODE_S,UAT,ACCEPT_COSTS"]
    assert any(c.startswith("spurs: about -39.0 dBc") for c in w.simultaneous_costs)
    assert w.warnings == []  # -55 dBm: below the level where a 1090U needs its TL_OFFSET raised
    assert w.last_result == {"sent": 18, "late": 0, "dropped": 0, "jobs": [
        {"job": 1, "command": "AT+TX=1,MODE_S,TABLE,-1,0,-55", "reason": "stopped", "sent": 9, "late": 0, "dropped": 0},
        {"job": 2, "command": "AT+TX=2,UAT,TABLE,-1,0,-55", "reason": "stopped", "sent": 9, "late": 0, "dropped": 0}]}


# AT+SIMULTANEOUS? on a Bee Waggler in the time-shared plan (pwiggd 2.0 cmd_rf.c / plan.c formats)
SIMUL_Q = "\r\n".join([
    "+SIMULTANEOUS:plan=off",
    "+SIMULTANEOUS:pair=MODE_S+UAT,available,image,measured on hardware",
    "+SIMULTANEOUS:pair=MODE_AC+UAT,available,image,measured on hardware",
    "+SIMULTANEOUS:pair=MODE_S+MODE_S,time-shared",
    "+SIMULTANEOUS:max_mode_s_dbm=-18.6",
    "+SIMULTANEOUS:max_uat_dbm=-15.2",
    "+SIMULTANEOUS:max=Mode S 1090 MHz,1,-18.6,-24.6",
    "+SIMULTANEOUS:cost=maximum level -18.6 dBm Mode S, -15.2 dBm UAT on this unit, about 25 dB below normal",
    "+SIMULTANEOUS:cost=spurs: about -39.0 dBc 56.0 MHz beyond the other protocol (922.0 and 1146.0 MHz)",
    "+SIMULTANEOUS:cost=switching takes about 0.2 s in and 0.7 s out; running jobs are stopped"])


def _simul_unit(fake_sysfs, fake_at, simul_q, jobs):
    return _pwiggd_unit(fake_sysfs, fake_at, {
        "AT+CONTROL=TAKE": "+CONTROL=USB,1", "AT+STOP=ALL": jobs.stop, "AT+SIMULTANEOUS?": simul_q,
        "AT+SIMULTANEOUS=MODE_S,UAT,ACCEPT_COSTS": "", "AT+SIMULTANEOUS=OFF": "", "AT+HOLD=1": "", "AT+HOLD=0": "",
        "AT+TX=1,MODE_S,TABLE,5,10,-40": jobs.tx, "AT+TX=2,UAT,TABLE,5,10,-40": jobs.tx,
        "AT+TX=1,MODE_S,TABLE,5,10,-5": jobs.tx})


def test_wiggler_v2_simultaneous_checks_pair_levels_and_warns_on_tl(fake_sysfs, fake_at):
    jobs = _Jobs()
    dev = _simul_unit(fake_sysfs, fake_at, SIMUL_Q, jobs)
    w = wiggler(mode_s_atten_db=45, subg_atten_db=45, simultaneous=True)
    w.transmit(TestPattern(band="dual", count=5, rate=10))
    assert dev.received[-1] == "AT+SIMULTANEOUS=OFF"
    assert len(w.warnings) == 1 and "AT+TL_OFFSET" in w.warnings[0]  # -40 dBm into a 1090U (t-1014)
    # above the plan's maximum: refused before anything transmits, and the plan is never entered
    with pytest.raises(TransmitterError, match="above the simultaneous plan's maximum"):
        wiggler(mode_s_atten_db=10, subg_atten_db=45, simultaneous=True).transmit(
            TestPattern(band="dual", count=5, rate=10))
    assert "AT+SIMULTANEOUS=MODE_S,UAT,ACCEPT_COSTS" not in dev.received[-3:]


def test_wiggler_v2_simultaneous_pair_not_on_this_unit(fake_sysfs, fake_at):
    jobs = _Jobs()
    q = SIMUL_Q.replace("MODE_S+UAT,available,", "MODE_S+UAT,not on this unit,")
    dev = _simul_unit(fake_sysfs, fake_at, q, jobs)
    with pytest.raises(TransmitterError, match="not on this unit"):
        wiggler(mode_s_atten_db=45, subg_atten_db=45, simultaneous=True).transmit(
            TestPattern(band="dual", count=5, rate=10))
    assert "AT+SIMULTANEOUS=MODE_S,UAT,ACCEPT_COSTS" not in dev.received


def test_packet_file_period_and_upload():
    text, loops = packet_file("MODE_S", MS[:3], 6, 1000, "-60")
    lines = [json.loads(x) for x in text.splitlines()]
    assert loops == 2 and lines[0]["period"] == 0.003 and lines[0]["format"] == "waggler-packets/1"
    assert [x["t"] for x in lines[1:]] == [0, 0.001, 0.002] and lines[1]["dBm"] == -60.0
    text, loops = packet_file("UAT", UAT, 5, 10, None)  # not a whole number of cycles: unrolled, once
    lines = [json.loads(x) for x in text.splitlines()]
    assert loops == 1 and len(lines) == 6 and "dBm" not in lines[1] and lines[0]["period"] == 0.5
    big = "x" * (FILE_CHUNK * 2 + 10)
    cmds = file_upload_commands("f", big)
    assert cmds[0] == f"AT+FILE=BEGIN,f,{len(big)},{hashlib.sha256(big.encode()).hexdigest()},jsonl"
    assert [c.split(",")[1] for c in cmds[1:-1]] == ["0", str(FILE_CHUNK), str(2 * FILE_CHUNK)]
    assert len(cmds[1].split(",")[2]) == 2 * FILE_CHUNK and cmds[-1] == "AT+FILE=END"


class _Replies(dict):
    """Fake replies with prefix keys ("AT+FILE=DATA,*")."""

    def get(self, cmd, default=None):
        if cmd in self:
            return self[cmd]
        return next((v for k, v in self.items() if k.endswith("*") and cmd.startswith(k[:-1])), default)


def test_wiggler_v2_packet_file_run_counts_and_cleanup(fake_sysfs, fake_at):
    def tx(cmd):
        return ("+TX:job=7\r\nOK\r\n+DONE:7,sent=400,late=2,dropped=0,reason=done\r\n")
    dev = _pwiggd_unit(fake_sysfs, fake_at, _Replies({
        "AT+CONTROL=TAKE": "+CONTROL=USB,1", "AT+STOP=ALL": "",
        "AT+FILE=BEGIN,*": "+FILE=receiving,0,1", "AT+FILE=DATA,*": "",
        "AT+FILE=END": "+FILE=hil_mode_s,valid,4,0.004000,MODE_S,1,0,0",
        "AT+TX=BOTH,FILE,hil_mode_s,100": tx, "AT+FILE=DELETE,hil_mode_s": ""}))
    w = wiggler(mode_s_atten_db=60)
    w.transmit(TestPattern(band="1090", count=400, rate=1000, messages=MS))
    assert dev.received[-2:] == ["AT+TX=BOTH,FILE,hil_mode_s,100", "AT+FILE=DELETE,hil_mode_s"]
    assert w.last_result["sent"] == 400 and w.last_result["late"] == 2
    assert w.warnings == ["w: job 7 (AT+TX=BOTH,FILE,hil_mode_s,100): 2 late, 0 dropped of 400 sent"]
    # a refused file never plays, and is deleted (its report is kept otherwise)
    dev.replies["AT+FILE=END"] = "+FILE=hil_mode_s,invalid,4,0.004000,MODE_S,1,1,0\r\n+FILE:line=2,rate,too fast"
    with pytest.raises(TransmitterError, match="packet file refused.*invalid"):
        w.transmit(TestPattern(band="1090", count=400, rate=1000, messages=MS))
    assert dev.received[-2:] == ["AT+FILE=END", "AT+FILE=DELETE,hil_mode_s"]


def test_wiggler_v2_sched_without_level_uses_unit_level_and_keeps_tx_warning(fake_sysfs, fake_at):
    def tx(cmd):
        return "+TX:job=1\r\n+TX:warning=example\r\nOK\r\n+DONE:1,sent=2,late=0,dropped=0,reason=done\r\n"
    dev = _pwiggd_unit(fake_sysfs, fake_at, _Replies({
        "AT+CONTROL=TAKE": "+CONTROL=USB,1", "AT+STOP=ALL": "", "AT+SCHED=*": "",
        "AT+LEVEL?": "+LEVEL=1,MODE_S,-60.0,-102.8,7.0\r\n+LEVEL=1,UAT,-65.5,-102.8,7.0\r\n"
                     "+LEVEL=2,MODE_S,-70.0,-102.8,7.0",
        "AT+TX=BOTH,FILE,sched,1": tx}))
    w = wiggler()
    w.transmit(TestPattern(band="1090", count=2, rate=100, messages=MS[:2]))
    assert f"AT+SCHED=ADD,BOTH,MODE_S,10000,-60,{MS[1]}" in dev.received
    assert w.warnings == ["w: +TX:warning=example"]


def test_wiggler_v2_stop_all(fake_sysfs, fake_at):
    dev = _pwiggd_unit(fake_sysfs, fake_at, {
        "AT+CONTROL=TAKE": "+CONTROL=USB,1", "AT+SIMULTANEOUS=OFF": "",
        "AT+STOP=ALL": lambda c: "OK\r\n+DONE:3,sent=50,late=0,dropped=0,reason=stopped\r\n"})
    out = wiggler().stop_all()
    assert dev.received[1:] == ["AT+CONTROL=TAKE", "AT+STOP=ALL", "AT+SIMULTANEOUS=OFF"]
    assert "+DONE:3,sent=50" in out


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
