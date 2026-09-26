import os

import pytest

from adsbee_hil import config

HERE = os.path.dirname(__file__)
EXAMPLE = os.path.join(HERE, "..", "bench.example.toml")


def test_example_parses():
    b = config.load(EXAMPLE)
    assert b.name == "example-bench"
    assert [r.id for r in b.receivers] == ["1090u-a", "1421-a"]
    assert b.receiver("E000000000000002").model == "adsbee_1421"
    assert b.receiver("1090u-a").usb_port == "1-1.3"
    assert b.receiver("1090u-a").tags == ["ci"]
    assert "{image}" in b.flasher_command("adsbee_1421")
    assert b.flasher_command("adsbee_1090u") is None


def test_no_config_is_empty_bench(tmp_path, monkeypatch):
    monkeypatch.setattr(config, "DEFAULT_PATHS", [str(tmp_path / "missing.toml")])
    b = config.load()
    assert b.receivers == [] and b.path is None
    with pytest.raises(config.ConfigError):
        config.load(str(tmp_path / "missing.toml"))


def test_env_config(tmp_path, monkeypatch):
    p = tmp_path / "b.toml"
    p.write_text('[bench]\nname = "env"\n')
    monkeypatch.setenv("ADSBEE_HIL_CONFIG", str(p))
    assert config.load().name == "env"


@pytest.mark.parametrize("data, msg", [
    ({"receivers": [{"id": "a", "model": "m"}]}, "usb_serial"),
    ({"receivers": [{"id": "a", "model": "m", "usb_serial": "S"},
                    {"id": "a", "model": "m", "usb_serial": "T"}]}, "duplicate id"),
    ({"receivers": [{"id": "a", "model": "m", "usb_serial": "S"},
                    {"id": "b", "model": "m", "usb_serial": "S"}]}, "duplicate usb_serial"),
    ({"receivers": [{"id": "a", "model": "m", "usb_serial": 5}]}, "must be str"),
    ({"bench": {"nmae": "typo"}}, "unknown keys"),
    ({"recievers": []}, "unknown top-level"),
    ({"transmitters": [{"id": "t", "type": "pluto", "receivers": ["ghost"]}]}, "unknown receivers"),
    ({"flashers": {"adsbee_1421": {"command": 5}}}, "flasher command"),
])
def test_invalid(data, msg):
    with pytest.raises(config.ConfigError, match=msg):
        config.parse(data)


def test_model_options_and_flasher_precedence(monkeypatch):
    b = config.parse({
        "flashers": {"adsbee_1421": {"command": "bench-flasher {image}"}},
        "receivers": [
            {"id": "a", "model": "adsbee_1421", "usb_serial": "S1", "flasher_command": ["own", "{image}"]},
            {"id": "b", "model": "adsbee_1421", "usb_serial": "S2", "extra": 3},
        ],
        "transmitters": [{"id": "w", "type": "bee_wiggler", "usb_serial": "S3", "mode_s_atten_db": 40}],
    })
    a, rb = b.receivers
    assert rb.options == {"extra": 3}
    assert b.transmitter("w").options == {"mode_s_atten_db": 40}
    assert b.flasher_command("adsbee_1421", a) == ["own", "{image}"]
    assert b.flasher_command("adsbee_1421", rb) == ["bench-flasher", "{image}"]
    monkeypatch.setenv("ADSBEE_HIL_FLASHER_ADSBEE_1421", "env-flasher '{image}'")
    assert b.flasher_command("adsbee_1421", rb) == ["env-flasher", "{image}"]
    assert b.flasher_command("adsbee_1421", a) == ["own", "{image}"]
