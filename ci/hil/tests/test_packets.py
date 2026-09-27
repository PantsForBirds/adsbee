from adsbee_hil import packets
from adsbee_hil.transmitters import mode_s_iq


def test_crc_known_message():
    assert packets.mode_s_crc(bytes.fromhex("8D4840D6202CC371C32CE0576098")) == 0
    assert packets.mode_s_crc(bytes.fromhex("8D4840D6202CC371C32CE0576099")) != 0


def test_df17_identification_matches_reference():
    assert packets.df17_identification(0x4840D6, "KLM1023") == "8D4840D6202CC371C32CE0576098"


def test_unique_set():
    msgs = packets.unique_df17_set(25, base_icao=0xABC000)
    assert len(set(msgs)) == 25
    assert [packets.icao_of(m) for m in msgs] == list(range(0xABC000, 0xABC000 + 25))
    assert all(packets.mode_s_crc(bytes.fromhex(m)) == 0 for m in msgs)


def test_parse_raw_frames():
    lines = [
        "#MDS*8D4840D6202CC371C32CE0576098;(1,-75,12,0000000000001234)",
        "AT+SOMETHING noise",
        "#UAT*-00A1B2C3D4E5;(0,-90,0000000000000001)",
        "#UAT*+0123;(0,-90,0000000000000002)",
    ]
    frames = packets.parse_raw_frames(lines)
    assert [f.kind for f in frames] == ["mode_s", "uat_adsb", "uat_uplink"]
    assert frames[0].icao == 0x4840D6 and frames[0].meta.startswith("1,-75")
    assert frames[1].icao == 0xA1B2C3
    assert frames[2].icao is None


def test_mode_s_iq_pulse_positions():
    msg = "8D4840D6202CC371C32CE0576098"
    iq = mode_s_iq([msg], sample_rate=2_000_000, spacing_s=0.001)  # 1 sample per 0.5 us chip
    assert len(iq) == 2000
    chips = [int(abs(s)) for s in iq[:16 + 224]]
    assert chips[:16] == [1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0]
    bits = "".join("1" if chips[16 + 2 * i] else "0" for i in range(112))
    assert int(bits, 2) == int(msg, 16)
    assert all(chips[16 + 2 * i] != chips[17 + 2 * i] for i in range(112))
