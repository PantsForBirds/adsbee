import struct

import pytest

from adsbee_hil import config, receivers, registry, usb

INFO = "CC1314R10 Firmware Version: 0.3.11-rc3\r\n"


@pytest.fixture
def jig(tmp_path, fake_sysfs, monkeypatch):
    fake_sysfs.add("1-1.4", "2e8a", "000a", "J1421", "Pants for Birds", "ADSBee 1421 Programmer", "ttyACM1",
                   "usb-Pants_for_Birds_ADSBee_1421_Programmer_J1421-if00")
    cfg = tmp_path / "bench.toml"
    cfg.write_text('[[receivers]]\nid = "a1421"\nmodel = "adsbee_1421"\nusb_serial = "J1421"\nusb_port = "1-1.4"\n')
    bench = config.load(str(cfg))
    rx = registry.make_receiver(bench.receivers[0], bench)
    uf2 = tmp_path / "programmer.uf2"
    uf2.write_bytes(struct.pack("<8I", *receivers.UF2_MAGIC, 0x2000, 0, 256, 0, 1, receivers.UF2_FAMILY_RP2040))
    drive = tmp_path / "RPI-RP2"
    drive.mkdir()
    monkeypatch.setattr(usb, "mount_rp2_drive", lambda port_path: str(drive))
    monkeypatch.setattr(receivers, "JIG_BOOTSEL_WAIT_S", 0.3)
    monkeypatch.setattr(receivers.time, "sleep", lambda s: None)
    monkeypatch.setattr(rx, "at_retry", lambda cmd, total: INFO)
    return {"rx": rx, "uf2": str(uf2), "drive": drive, "fs": fake_sysfs}


def to_bootsel(fs):
    fs.remove("1-1.4")
    fs.add("1-1.4", "2e8a", "0003", "BOOTROM", "Raspberry Pi", "RP2 Boot", block="sdb")


def test_magic_baud_reboots_jig_without_a_human(jig, monkeypatch, capsys):
    opened = []

    def magic(port):
        opened.append(port)
        to_bootsel(jig["fs"])

    monkeypatch.setattr(receivers, "open_at_magic_baud", magic)
    states = iter(["app"])
    monkeypatch.setattr(jig["rx"], "state", lambda: next(states, "app"))
    info = jig["rx"].flash(jig["uf2"], human_timeout=0.5)
    assert opened == [usb.DEV_ROOT + "/serial/by-id/usb-Pants_for_Birds_ADSBee_1421_Programmer_J1421-if00"]
    assert (jig["drive"] / "programmer.uf2").exists()
    assert info["CC1314R10 Firmware Version"] == "0.3.11-rc3"
    out = capsys.readouterr().out
    assert f"at {0xDEADBEE} baud" in out and "HUMAN NEEDED" not in out


def test_old_jig_image_falls_back_to_a_human(jig, monkeypatch, capsys):
    monkeypatch.setattr(receivers, "open_at_magic_baud", lambda port: None)  # Old image: stays in the app.
    with pytest.raises(receivers.HilError, match="no RP2040 BOOTSEL device"):
        jig["rx"].flash(jig["uf2"], human_timeout=0.2)
    assert "HUMAN NEEDED" in capsys.readouterr().out
    assert not (jig["drive"] / "programmer.uf2").exists()


def test_open_at_magic_baud_uses_the_1090_magic_baud(monkeypatch):
    import serial

    seen = []

    class FakeSerial:
        def __init__(self, port, baud):
            seen.append((port, baud))

        def close(self):
            raise serial.SerialException("device disconnected")  # The Programmer drops off the bus.

    monkeypatch.setattr(serial, "Serial", FakeSerial)
    receivers.open_at_magic_baud("/dev/ttyACM9")
    assert seen == [("/dev/ttyACM9", 233495534)]
