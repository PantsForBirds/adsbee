import json
import os

import pytest

from adsbee_hil import cli
from adsbee_hil.lock import DeviceLock

INFO_1421 = ("Part Code: 010260002D-TEST\r\nCC1314R10 Unique ID: 0011223344556677\r\n"
             "CC1314R10 Firmware Version: 0.3.11-rc1\r\nOTA Key 0: supersecret\r\n")
INFO_1090 = "Part Code: 010250002D-TEST\r\nRP2040 Firmware Version: 0.9.1-rc3\r\nOTA Key 0: hush\r\n"


@pytest.fixture
def bench(tmp_path, fake_sysfs, fake_at):
    rx1421 = fake_at({"AT+UPTIME?": "+UPTIME=42", "AT+DEVICE_INFO?": INFO_1421,
                      "AT+RX_STATS?": "+RX_STATS=pkt_rx=5,crc_error=1,pbl_det=9"})
    rx1090 = fake_at({"AT+UPTIME?": "+UPTIME=7", "AT+DEVICE_INFO?": INFO_1090})
    fake_sysfs.add("1-1.4", "2e8a", "000a", "J1421", "ADSBee", "ADSBee 1421 Programmer", "ttyACM1",
                   "usb-ADSBee_ADSBee_1421_Programmer_J1421-if00", tty_target=rx1421.path)
    fake_sysfs.add("1-1.3", "2e8a", "000a", "P1090", "Raspberry Pi", "Pico", "ttyACM0",
                   "usb-Raspberry_Pi_Pico_P1090-if00", tty_target=rx1090.path)
    cfg = tmp_path / "bench.toml"
    cfg.write_text("""
[bench]
name = "test"
[[receivers]]
id = "a1421"
model = "adsbee_1421"
usb_serial = "J1421"
usb_port = "1-1.4"
[[receivers]]
id = "a1090"
model = "adsbee_1090u"
usb_serial = "P1090"
tags = ["ci"]
[[receivers]]
id = "gone"
model = "adsbee_1090u"
usb_serial = "MISSING"
""")
    return {"cfg": str(cfg), "1421": rx1421, "1090": rx1090}


def run(capsys, *argv):
    rc = cli.main(list(argv))
    o = capsys.readouterr()
    return rc, o.out, o.err


def test_discover_without_config(fake_sysfs, capsys):
    fake_sysfs.add("1-1.4", "2e8a", "000a", "J1421", "ADSBee", "ADSBee 1421 Programmer", "ttyACM1")
    fake_sysfs.add("1-1.5", "2e8a", "0003", "BOOTROM1", "Raspberry Pi", "RP2 Boot")
    rc, out, _ = run(capsys, "discover")
    assert rc == 0
    assert "J1421" in out and "adsbee_1421?" in out and "bootsel" in out
    rc, out, _ = run(capsys, "discover", "--json")
    data = json.loads(out)
    assert {d["serial"]: d["mode"] for d in data["devices"]} == {"J1421": "app", "BOOTROM1": "bootsel"}


def test_discover_with_bench(bench, capsys):
    rc, out, _ = run(capsys, "-c", bench["cfg"], "discover", "--json")
    data = json.loads(out)
    assert {d["serial"]: d["bench_id"] for d in data["devices"]} == {"J1421": "a1421", "P1090": "a1090"}
    assert data["missing"] == ["gone"]


def test_at_and_redaction(bench, capsys):
    rc, out, _ = run(capsys, "-c", bench["cfg"], "at", "-d", "a1421", "AT+DEVICE_INFO?")
    assert rc == 0
    assert "[a1421] CC1314R10 Firmware Version: 0.3.11-rc1" in out
    assert "supersecret" not in out and "OTA Key 0: <redacted>" in out
    # 1421 console: probe with AT+UPTIME? (never a bare AT) before the command.
    assert bench["1421"].received[-2:] == ["AT+UPTIME?", "AT+DEVICE_INFO?"]
    assert "AT" not in bench["1421"].received


def test_info_parallel_all(bench, capsys):
    rc, out, _ = run(capsys, "-c", bench["cfg"], "info", "-d", "a1421", "-d", "a1090", "--json")
    assert rc == 0
    data = json.loads(out[out.index("{"):])
    assert data["a1421"]["CC1314R10 Firmware Version"] == "0.3.11-rc1"
    assert data["a1090"]["RP2040 Firmware Version"] == "0.9.1-rc3"
    assert "OTA Key 0" not in data["a1090"]
    rc, out, _ = run(capsys, "-c", bench["cfg"], "info", "--all")
    assert rc == 1  # "gone" is absent
    assert "[gone] ERROR" in out and "[a1090] RP2040 Firmware Version: 0.9.1-rc3" in out


def test_select_by_model_and_tag(bench, capsys):
    rc, out, _ = run(capsys, "-c", bench["cfg"], "port", "--tag", "ci")
    assert rc == 0 and out.strip().endswith("usb-Raspberry_Pi_Pico_P1090-if00")
    rc, out, _ = run(capsys, "-c", bench["cfg"], "port", "--model", "adsbee_1421")
    assert out.strip().endswith("ADSBee_1421_Programmer_J1421-if00")


def test_ad_hoc_serial_without_bench(bench, capsys):
    # The 1421 jig is identifiable by its product string; a bare Pico needs --model.
    rc, out, _ = run(capsys, "at", "-d", "J1421", "AT+UPTIME?")
    assert rc == 0 and "UPTIME=42" in out
    rc, out, err = run(capsys, "at", "-d", "P1090", "AT+UPTIME?")
    assert rc == 2 and "--model" in err
    rc, out, _ = run(capsys, "at", "-d", "P1090", "-m", "adsbee_1090u", "AT+UPTIME?")
    assert rc == 0 and "UPTIME=7" in out


def test_refuses_transmit_and_baud_change(bench, capsys):
    rc, out, _ = run(capsys, "-c", bench["cfg"], "at", "-d", "a1421", "AT+TX_CW=LRLF,1090")
    assert rc == 1 and "transmits" in out
    rc, out, _ = run(capsys, "-c", bench["cfg"], "at", "-d", "a1421", "AT+BAUD_RATE=CONSOLE,115200")
    assert rc == 1 and "desyncs" in out
    assert not any("TX_CW" in c or "BAUD" in c for c in bench["1421"].received)


def test_rx_stats(bench, capsys):
    rc, out, _ = run(capsys, "-c", bench["cfg"], "rx-stats", "-d", "a1421", "-d", "a1090")
    assert rc == 0
    assert "[a1421] pkt_rx=5, crc_error=1, pbl_det=9" in out
    assert "[a1090] adsbee_1090u has no receive counters" in out


def test_usb_port_mismatch_is_reported(bench, capsys, tmp_path):
    cfg = open(bench["cfg"]).read().replace('usb_port = "1-1.4"', 'usb_port = "1-1.9"')
    p = tmp_path / "moved.toml"
    p.write_text(cfg)
    rc, out, _ = run(capsys, "-c", str(p), "at", "-d", "a1421", "AT+UPTIME?")
    assert rc == 1 and "recable or fix the bench file" in out


def test_busy_device_times_out(bench, capsys, lock_dir):
    import threading

    held = threading.Event()
    release = threading.Event()

    def hold():
        with DeviceLock("J1421", lock_dir, purpose="other job"):
            held.set()
            release.wait(5)
    t = threading.Thread(target=hold)
    t.start()
    held.wait()
    try:
        rc, out, err = run(capsys, "-c", bench["cfg"], "--lock-timeout", "0.3", "at", "-d", "a1421", "AT+UPTIME?")
        assert rc == 1 and "still locked" in out and "other job" in out
        rc, out, _ = run(capsys, "-c", bench["cfg"], "locks")
        assert "J1421" in out and "LOCKED" in out
    finally:
        release.set()
        t.join()


def test_flash_1421_without_flasher_explains(bench, capsys, tmp_path, monkeypatch):
    monkeypatch.delenv("ADSBEE_HIL_FLASHER_ADSBEE_1421", raising=False)
    img = tmp_path / "fw.hex"
    img.write_text(":00000001FF\n")
    rc, out, _ = run(capsys, "-c", bench["cfg"], "flash", "-d", "a1421", str(img))
    assert rc == 1 and "no flasher configured" in out


def test_flash_1421_runs_external_flasher(bench, capsys, tmp_path, monkeypatch):
    img = tmp_path / "fw.hex"
    img.write_text(":00000001FF\n")
    log = tmp_path / "flasher.log"
    monkeypatch.setenv("ADSBEE_HIL_FLASHER_ADSBEE_1421", f"sh -c 'echo \"$@\" > {log}' flasher {{image}} "
                                                         "-p {port} -b {baud} {verbose}")
    rc, out, _ = run(capsys, "-c", bench["cfg"], "flash", "-d", "a1421", str(img))
    assert rc == 0, out
    args = log.read_text().split()
    assert args[0] == str(img) and args[1] == "-p" and args[2].endswith("J1421-if00") and args[4] == "1000000"
    assert "[a1421] CC1314R10 Firmware Version: 0.3.11-rc1" in out


def test_flash_refuses_when_busy(bench, capsys, tmp_path):
    import subprocess

    busy = subprocess.Popen(["sleep", "37.5"])
    cfg = open(bench["cfg"]).read().replace('name = "test"', 'name = "test"\nbusy_processes = ["^sleep 37\\\\.5"]')
    p = tmp_path / "busy.toml"
    p.write_text(cfg)
    img = tmp_path / "fw.hex"
    img.write_text(":00000001FF\n")
    try:
        rc, out, _ = run(capsys, "-c", str(p), "flash", "-d", "a1421", str(img))
        assert rc == 1 and "refusing to flash" in out
    finally:
        busy.kill()
        busy.wait()
    from adsbee_hil.receivers import _ancestors, busy_process
    assert busy_process(["^sleep 37\\.5"]) is None
    # Our own process and its parents never count, even when their command lines match.
    assert {os.getpid(), os.getppid()} <= _ancestors()


def test_lock_command(bench, capsys, tmp_path):
    marker = tmp_path / "ran"
    rc, _, _ = run(capsys, "-c", bench["cfg"], "lock", "a1421", "--", "touch", str(marker))
    assert rc == 0 and marker.exists()


def test_loopback_skips_cleanly(bench, capsys):
    rc, out, _ = run(capsys, "-c", bench["cfg"], "loopback", "-d", "a1421")
    assert rc == 0 and "SKIP" in out
    rc, out, _ = run(capsys, "-c", bench["cfg"], "loopback", "-d", "a1421", "--fail-on-skip")
    assert rc == 1
