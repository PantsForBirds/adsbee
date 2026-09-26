from adsbee_hil import usb


def populate(fs):
    fs.add("1-1.3", "2e8a", "000a", "E000000000000001", "Pants for Birds", "ADSBee 1090", "ttyACM0",
           "usb-Pants_for_Birds_ADSBee_1090_E000000000000001-if00")
    fs.add("1-1.4", "2e8a", "000a", "E000000000000002", "Pants for Birds", "ADSBee 1421 Programmer", "ttyACM1",
           "usb-Pants_for_Birds_ADSBee_1421_Programmer_E000000000000002-if00")
    fs.add("1-1.5", "2e8a", "0003", "B00000000001", "Raspberry Pi", "RP2 Boot", block="sda")
    fs.add("1-1.1", "0424", "ec00", product="Some hub")


def test_enumeration(fake_sysfs):
    populate(fake_sysfs)
    devs = {d.port_path: d for d in usb.usb_devices()}
    assert set(devs) == {"1-1.1", "1-1.3", "1-1.4", "1-1.5"}
    pico = devs["1-1.3"]
    assert pico.vidpid == "2e8a:000a" and pico.mode == "app" and pico.ttys == ["ttyACM0"]
    assert pico.console.endswith("by-id/usb-Pants_for_Birds_ADSBee_1090_E000000000000001-if00")
    assert pico.model_hint is None
    assert devs["1-1.4"].model_hint == "adsbee_1421"
    assert devs["1-1.5"].mode == "bootsel" and devs["1-1.5"].console is None
    assert [d.port_path for d in usb.adsbee_candidates()] == ["1-1.3", "1-1.4", "1-1.5"]


def test_find(fake_sysfs):
    populate(fake_sysfs)
    assert usb.find_by_serial("E000000000000002").port_path == "1-1.4"
    assert usb.find_by_serial("nope") is None
    assert usb.find_by_port("1-1.5").is_bootsel


def test_console_without_by_id(fake_sysfs):
    fake_sysfs.add("1-1.2", "2e8a", "000a", "X1", tty_name="ttyACM7")
    d = usb.find_by_serial("X1")
    assert d.console.endswith("/ttyACM7")


def test_block_partition_and_mount(fake_sysfs, tmp_path):
    populate(fake_sysfs)
    assert usb.block_partition("1-1.5") == "/dev/sda1"
    assert usb.block_partition("1-1.3") is None
    mounts = tmp_path / "mounts"
    mounts.write_text("/dev/sdb1 /media/x/OTHER vfat rw 0 0\n/dev/sda1 /media/ci/RPI\\040RP2 vfat rw 0 0\n")
    assert usb.mountpoint_of("/dev/sda1", str(mounts)) == "/media/ci/RPI RP2"
    assert usb.mountpoint_of("/dev/sdc1", str(mounts)) is None


def test_wait_for():
    calls = []
    assert usb.wait_for(lambda: calls.append(1) or len(calls) >= 3, 5, 0.01) is True
    assert usb.wait_for(lambda: None, 0.05, 0.01) is None
