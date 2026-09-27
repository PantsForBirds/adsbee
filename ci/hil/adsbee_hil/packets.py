"""Test-packet helpers: Mode S CRC, DF17 builders, and parsing of receiver RAW output.

Used to generate known test patterns for transmitters that take raw messages (Pluto, or a
wiggler's single-message mode) and to match what receivers report against what was sent.
"""

import re
from dataclasses import dataclass
from typing import Iterable, List, Optional

MODE_S_GENERATOR = 0xFFF409  # CRC-24 generator polynomial (x^24 + ... + 1, top bit implied).

_CALLSIGN_CHARS = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ##### ###############0123456789######"


def mode_s_crc(data: bytes) -> int:
    """CRC-24 over data (bitwise, MSB first). For a whole DF17 message, 0 means valid."""
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1000000 | MODE_S_GENERATOR
    return crc & 0xFFFFFF


def df17_identification(icao: int, callsign: str, category: int = 0, ca: int = 5) -> str:
    """112-bit DF17 aircraft identification message (TC 4) as uppercase hex."""
    cs = callsign.upper().ljust(8)[:8]
    me = (4 << 51) | ((category & 0x7) << 48)
    for i, ch in enumerate(cs):
        idx = _CALLSIGN_CHARS.find(ch)
        if idx < 0 or ch == "#":
            raise ValueError(f"callsign character {ch!r} is not encodable")
        me |= idx << (42 - 6 * i)
    head = bytes([(17 << 3) | (ca & 0x7)]) + (icao & 0xFFFFFF).to_bytes(3, "big") + me.to_bytes(7, "big")
    return (head + mode_s_crc(head).to_bytes(3, "big")).hex().upper()


def unique_df17_set(count: int, base_icao: int = 0xADB000, prefix: str = "HIL") -> List[str]:
    """``count`` distinct DF17 messages with consecutive ICAO addresses from base_icao."""
    return [df17_identification(base_icao + i, f"{prefix}{i:0{8 - len(prefix)}d}"[:8]) for i in range(count)]


def icao_of(hex_msg: str) -> Optional[int]:
    """ICAO address of a DF11/17/18 Mode S or a UAT ADS-B message (bytes 1..3 in both)."""
    if len(hex_msg) < 8:
        return None
    return int(hex_msg[2:8], 16)


@dataclass
class Frame:
    kind: str  # "mode_s", "uat_adsb", "uat_uplink"
    hex: str
    meta: str = ""

    @property
    def icao(self) -> Optional[int]:
        return icao_of(self.hex) if self.kind != "uat_uplink" else None


# RAW reporting protocol lines (firmware/common/comms/raw/raw_utils.cpp):
#   #MDS*<hex>;(<meta>)   Mode S
#   #UAT*-<hex>;(<meta>)  UAT ADS-B
#   #UAT*+<hex>;(<meta>)  UAT uplink
_RAW = re.compile(r"#(MDS|UAT)\*([-+]?)([0-9A-Fa-f]+);(\([^)]*\))?")


def parse_raw_frames(lines: Iterable[str]) -> List[Frame]:
    frames = []
    for line in lines:
        for m in _RAW.finditer(line):
            kind = "mode_s" if m.group(1) == "MDS" else ("uat_uplink" if m.group(2) == "+" else "uat_adsb")
            frames.append(Frame(kind, m.group(3).upper(), (m.group(4) or "").strip("()")))
    return frames
