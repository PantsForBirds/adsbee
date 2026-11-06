#!/usr/bin/env python3
"""readsbee feed protocol v3.2: reference encoder/decoder.

Spec: FEED-PROTOCOL.md. Golden vectors: vectors.json.
  python3 readsbee_proto.py           check against vectors.json -> "vectors ok"
  python3 readsbee_proto.py --write   regenerate vectors.json

Wire rules (all frames): 0x1a <type> [<msg type letter>] <body>. Every 0x1a in <body> is sent
twice. The type byte and the msg type letter are never escaped (and never 0x1a).
Integers are big-endian.

v3 adds the capabilities check (0xE5, FEED-PROTOCOL.md section 7): the feeder sends a query
'Q' right after 0xE4, a readsbee server answers 'A' on the same socket. Every byte after the
type is >= 0x80 or an ASCII letter, so the frame never contains 0x1a and is never escaped.

v3.1 adds capability bit 0 (signal level as signed int8 dBm on 0x31/0x32/0x33/0xEC) and the
confirm frame 'C' (feeder -> server): the feeder sends it once, after the answer, right before
the first frame that uses the agreed bits. A decoder switches the signal byte's meaning there.

v3.2: bit 0 decides the signal byte of every frame type, 0xED and 0xEF included. Without it (or
before 'C') all signal bytes are Beast power bytes (0 = no level); with it, int8 dBm (0x80 = no
level). Decoder(sig_dbm=True) starts in dBm mode (used by vectors that show one frame as it
looks after 'C').
"""
from __future__ import annotations

import json
import math
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path

ESC = 0x1A

# ---- frame types (byte after 0x1a) ---------------------------------------------------------
T_MODE_AC = 0x31
T_MODE_S_SHORT = 0x32
T_MODE_S_LONG = 0x33
T_INGEST_ID = 0xE3       # readsb: 8-byte receiver id
T_UUID = 0xE4            # readsb: 36 ASCII chars
T_UAT = 0xEC             # ADSBee/readsb: UAT
T_ADSL = 0xED            # v2: ADS-L
T_RID = 0xEF             # v2: Remote ID (ODID)
T_CAPS = 0xE5            # v3: capabilities query / answer
T_PING = 0x50            # readsb -> feeder: '1a P' + 3 bytes (escaped), only from a server

# ---- capabilities check (v3) ---------------------------------------------------------------
CAPS_QUERY, CAPS_ANSWER, CAPS_CONFIRM = "Q", "A", "C"
CAPS_LETTERS = (CAPS_QUERY, CAPS_ANSWER, CAPS_CONFIRM)
CAPS_VERSION = 1
# v3.2 order (admin): the signal-level capability first, then the protocols.
CAP_SIG_DBM = 1 << 0            # signal byte of every frame type is int8 dBm (else Beast power byte)
CAP_ADSL = 1 << 1               # 0xED ADS-L
CAP_RID = 1 << 2                # 0xEF Remote ID
CAP_UAT_MODES_CLOCK = 1 << 3    # 0xEC with upper-case S L U
CAP_NAMES = {CAP_SIG_DBM: "sig_dbm", CAP_ADSL: "adsl_ed", CAP_RID: "rid_ef", CAP_UAT_MODES_CLOCK: "uat_upper"}
CAP_KNOWN = CAP_ADSL | CAP_RID | CAP_UAT_MODES_CLOCK | CAP_SIG_DBM
CAPS_LEN = 6                    # 1a E5 letter ver f0 f1
CAPS_ANSWER_TIMEOUT_S = 5.0     # feeder: wait this long for 'A', sending plain Beast meanwhile

MODE_LEN = {T_MODE_AC: 2, T_MODE_S_SHORT: 7, T_MODE_S_LONG: 14}

# Msg type letters. Lower case: timestamp NOT on the Mode S clock (unusable for MLAT).
# Upper case: timestamp on this connection's Mode S clock, 0 = not synced yet.
UAT_LEN = {"s": 30, "l": 48, "u": 552}                     # payload bytes, same for S/L/U
ADSL_CH = {"a": "868.2", "b": "868.4", "x": None}         # A/B/X: same, Mode S clock
RID_TR = {"b": "bt4_legacy", "c": "bt5_coded", "e": "bt5_ext_1m", "w": "wifi_beacon", "n": "wifi_nan"}

ADSL_MIN_L, ADSL_MAX_L = 1, 31      # on-air length byte (firmware: 12..31 today)
ODID_MSG = 25
ODID_PACK_MAX = 9
REP_UNKNOWN = 0xFF
DBM_NONE = -128                     # 0xED/0xEF signal: int8 dBm, -128 (0x80) = no level


def escape(body: bytes) -> bytes:
    return body.replace(b"\x1a", b"\x1a\x1a")


def _letter(ch: str, table: dict, modes_clock: bool) -> int:
    if ch.lower() not in table:
        raise ValueError(f"bad msg type {ch!r}")
    return ord(ch.upper() if modes_clock else ch.lower())


def _hdr(ts: int, rssi: int) -> bytes:
    """ts6 + Beast power byte (Mode S, UAT)."""
    return ts.to_bytes(6, "big") + bytes([rssi])


def _hdr_dbm(ts: int, dbm: int) -> bytes:
    """ts6 + signed int8 dBm (ADS-L, Remote ID), -128 = no level."""
    return ts.to_bytes(6, "big") + struct.pack("b", max(-128, min(127, dbm)))


# ---- signal level: Beast power byte <-> dBm (v3.1) ----------------------------------------
# Firmware table (firmware/common/comms/beast/generate_beast_tables.py -> beast_tables.hh):
# dBm clamped to -100..-40, dBFS = 0.75 * (dBm + 40), byte = round(255 * 10^(dBFS / 20)).
FW_MIN_DBM, FW_MAX_DBM, FW_DBFS_PER_DB = -100, -40, 45 / 60
FW_POWER_TABLE = [round(10 ** (((d - FW_MAX_DBM) * FW_DBFS_PER_DB) / 10 / 2 + math.log10(255)))
                  for d in range(FW_MIN_DBM, FW_MAX_DBM + 1)]
SIG_DBM_MIN, SIG_DBM_MAX = -127, 127   # -128 (0x80) = no level


def dbm_to_power(dbm):
    """What ADSBee firmware writes today (WriteRSSIdBmAsBeastPowerLevel). None = no level -> 0
    (Beast convention; today's firmware sends 1 for no level, see FEED-PROTOCOL.md 2.1)."""
    if dbm is None:
        return 0
    return FW_POWER_TABLE[min(max(dbm - FW_MIN_DBM, 0), len(FW_POWER_TABLE) - 1)]


def power_to_dbm(b: int):
    """Inverse of the firmware mapping (float dBm), None for 0. Only meaningful for ADSBee feeds:
    255 means "-40 dBm or stronger", 1 means "-100 dBm or weaker"."""
    if b == 0:
        return None
    return FW_MAX_DBM + 20 * math.log10(b / 255) / FW_DBFS_PER_DB


def dbm_byte(dbm) -> int:
    """v3.1 signal byte: signed int8 dBm, clamped to -127..+127, None -> 0x80."""
    if dbm is None:
        return 0x80
    return max(SIG_DBM_MIN, min(SIG_DBM_MAX, round(dbm))) & 0xFF


def dbm_from_byte(b: int):
    v = struct.unpack("b", bytes([b]))[0]
    return None if v == DBM_NONE else v


def signal_level_from_dbm(dbm) -> float:
    """readsbee: internal signalLevel (power, 0..1, as readsb derives from the power byte) for a
    dBm byte, on the firmware's scale, unrounded; capped at 1.0 (= -40 dBm and stronger)."""
    if dbm is None:
        return 0.0
    return min(1.0, 10 ** (FW_DBFS_PER_DB * (dbm - FW_MAX_DBM) / 10))


def _sig(rssi, dbm) -> int:
    if (rssi is None) == (dbm is None) and not (rssi is None and dbm == "none"):
        raise ValueError("give rssi (power byte) or dbm, not both")
    return rssi if rssi is not None else dbm_byte(None if dbm == "none" else dbm)


# ---- encoders ------------------------------------------------------------------------------
def enc_mode_s(msg: bytes, ts: int, rssi: int = None, dbm=None) -> bytes:
    """rssi = Beast power byte (default); dbm = v3.1 int8 dBm ("none" = no level)."""
    t = {2: T_MODE_AC, 7: T_MODE_S_SHORT, 14: T_MODE_S_LONG}[len(msg)]
    return bytes([ESC, t]) + escape(_hdr(ts, _sig(rssi, dbm)) + msg)


def enc_uuid(uuid: str) -> bytes:
    return bytes([ESC, T_UUID]) + uuid.encode()


def enc_uat(kind: str, ts: int, rssi, payload: bytes, modes_clock: bool, dbm=None) -> bytes:
    """kind s/l/u. modes_clock=True sends S/L/U (after bit 3 was agreed). rssi = power byte, or
    rssi=None and dbm=... after bit 0 was agreed."""
    rssi = _sig(rssi, dbm)
    k = _letter(kind, UAT_LEN, modes_clock)
    if len(payload) != UAT_LEN[chr(k).lower()]:
        raise ValueError("bad UAT length")
    return bytes([ESC, T_UAT, k]) + escape(_hdr(ts, rssi) + payload)


def _sig_any(dbm, rssi) -> bytes:
    """v3.2 signal byte for 0xED/0xEF: rssi = power byte (bit 0 not agreed), else int8 dBm."""
    if rssi is not None:
        if dbm is not None:
            raise ValueError("give rssi (power byte) or dbm, not both")
        return bytes([rssi])
    return struct.pack("b", max(-128, min(127, dbm)))


def enc_adsl(onair: bytes, ts: int, dbm, channel: str, repaired: int = REP_UNKNOWN,
             modes_clock: bool = True, rssi: int = None) -> bytes:
    """onair = bytes after the sync word: length byte L, then L bytes (header .. CRC-24).
    channel 'a' 868.2, 'b' 868.4, 'x' unknown."""
    if not (ADSL_MIN_L <= onair[0] <= ADSL_MAX_L) or len(onair) != 1 + onair[0]:
        raise ValueError("bad ADS-L on-air buffer")
    k = _letter(channel, ADSL_CH, modes_clock)
    return bytes([ESC, T_ADSL, k]) + escape(ts.to_bytes(6, "big") + _sig_any(dbm, rssi) + bytes([repaired]) + onair)


def enc_rid(odid: bytes, ts: int, dbm, transport: str, addr: bytes,
            modes_clock: bool = False, rssi: int = None) -> bytes:
    """odid = one 25-byte ASTM F3411 message, or a message pack (first byte 0xF?, size 25,
    count 1..9, then count x 25). addr = 6-byte source MAC, display order."""
    if len(addr) != 6 or _odid_len(odid) != len(odid):
        raise ValueError("bad Remote ID frame")
    k = _letter(transport, RID_TR, modes_clock)
    return bytes([ESC, T_RID, k]) + escape(ts.to_bytes(6, "big") + _sig_any(dbm, rssi) + addr + odid)


def enc_caps(letter: str, bits: int, version: int = CAPS_VERSION) -> bytes:
    """Capabilities query ('Q', feeder -> server: what the feeder can send) or answer ('A',
    server -> feeder: what the server accepts). 14 feature bits in two bytes, 7 bits each,
    high bit always set; version byte 0x80 | version."""
    if letter not in CAPS_LETTERS or not 1 <= version <= 0x7F or not 0 <= bits < 1 << 14:
        raise ValueError("bad caps frame")
    return bytes([ESC, T_CAPS, ord(letter), 0x80 | version, 0x80 | (bits & 0x7F), 0x80 | (bits >> 7)])


def dec_caps_body(body: bytes):
    """letter + 3 bytes -> (letter, version, bits) or None if malformed."""
    if len(body) != 4 or chr(body[0]) not in CAPS_LETTERS or any(b < 0x80 for b in body[1:]):
        return None
    ver = body[1] & 0x7F
    if ver == 0:
        return None
    return chr(body[0]), ver, (body[2] & 0x7F) | (body[3] & 0x7F) << 7


def cap_names(bits: int) -> list[str]:
    return [n for b, n in CAP_NAMES.items() if bits & b]


def negotiate(offered: int, answer_bits: int) -> int:
    """What the feeder sends after an answer: offered AND accepted, known bits only."""
    return offered & answer_bits & CAP_KNOWN


class AnswerScanner:
    """Feeder side: scan what a server sends back on the feed socket for '1a E5 A ...'.
    This is the logic the ESP32 needs (a few bytes of state). Everything else is skipped:
    readsb pings '1a P xx xx xx' (escaped), Beast heartbeats, our own query echoed back,
    unknown frames, garbage. feed() returns (version, bits) once, then None."""

    def __init__(self):
        self.st = 0              # 0 find 1a, 1 after 1a, 2.. collecting caps bytes
        self.got = bytearray()
        self.answer = None
        self.skipped = 0

    def feed(self, data: bytes):
        for x in data:
            if self.answer is not None:
                self.skipped += 1
                continue
            if self.st == 0:
                if x == ESC:
                    self.st = 1
                else:
                    self.skipped += 1
            elif self.st == 1:
                if x == T_CAPS:
                    self.st, self.got = 2, bytearray()
                elif x == ESC:
                    self.st = 0            # escaped 1a inside some other frame
                    self.skipped += 2
                else:
                    self.st = 0            # some other frame: skip to the next 1a
                    self.skipped += 2
            else:
                if x == ESC:               # caps bytes are never 1a: a new frame starts here
                    self.skipped += 2 + len(self.got)
                    self.st = 1
                    continue
                self.got.append(x)
                if len(self.got) == 4:
                    r = dec_caps_body(bytes(self.got))
                    self.st = 0
                    if r and r[0] == CAPS_ANSWER:
                        self.answer = (r[1], r[2])
                        return self.answer
                    self.skipped += 6      # malformed, or a 'Q' (echo): ignore
        return None


def _odid_len(head: bytes):
    """Length of an ODID message or pack from its first 3 bytes. None = need more, False = bad."""
    if not head:
        return None
    if head[0] >> 4 != 0xF:
        return ODID_MSG
    if len(head) < 3:
        return None
    size, count = head[1], head[2]
    if size != ODID_MSG or not 1 <= count <= ODID_PACK_MAX:
        return False
    return 3 + count * ODID_MSG


# ---- decoder -------------------------------------------------------------------------------
@dataclass
class Frame:
    type: int
    letter: str
    body: bytes                      # unescaped body (after the type / letter byte)
    raw: bytes                       # bytes as on the wire
    fields: dict = field(default_factory=dict)


class Decoder:
    """Incremental stream decoder: feed(bytes) -> list[Frame].
    Unknown type or msg type letter: skip to the next frame start (0x1a followed by anything
    but 0x1a), the readsbee rule. Skipped bytes are counted in .skipped, frames in .unknown."""

    def __init__(self, sig_dbm: bool = False):
        self.buf = bytearray()
        self.skipped = 0
        self.unknown = 0
        self.sig_dbm = sig_dbm       # v3.1/3.2: set by a 'C' frame with bit 0, for the rest of the stream

    def _take(self, i: int, n: int):
        """n unescaped bytes from buf[i:] -> (bytes, next_i); None = incomplete; False = a
        lone 0x1a (frame start) inside."""
        out = bytearray()
        b = self.buf
        while len(out) < n:
            if i >= len(b):
                return None
            x = b[i]
            i += 1
            if x == ESC:
                if i >= len(b):
                    return None
                if b[i] != ESC:
                    return False
                i += 1
            out.append(x)
        return bytes(out), i

    def _skip_frame(self, i: int):
        """Index of the next frame start after buf[i] (= 0x1a of this frame), None if unknown yet."""
        b = self.buf
        j = i + 2
        while True:
            j = b.find(ESC, j)
            if j < 0 or j + 1 >= len(b):
                return None
            if b[j + 1] == ESC:
                j += 2
                continue
            return j

    def _body(self, t: int, p: int):
        """Parse one frame body starting at buf[p] (after the type byte).
        Returns (letter, body, next) / None (incomplete) / False (lone 0x1a) / 'unknown'."""
        b = self.buf
        if t in MODE_LEN:
            r = self._take(p, 7 + MODE_LEN[t])
            return r and ("", r[0], r[1])
        if t == T_UUID:
            return ("", bytes(b[p:p + 36]), p + 36) if len(b) >= p + 36 else None
        if t == T_INGEST_ID:
            r = self._take(p, 8)
            return r and ("", r[0], r[1])
        if t == T_CAPS:
            if len(b) < p + 4:
                return None
            body = bytes(b[p:p + 4])
            if ESC in body or dec_caps_body(body) is None:
                return "unknown"
            return (chr(body[0]), body[1:], p + 4)
        if t not in (T_UAT, T_ADSL, T_RID):
            return "unknown"
        if p >= len(b):
            return None
        k = chr(b[p])
        p += 1
        if t == T_UAT:
            if k.lower() not in UAT_LEN:
                return "unknown"
            r = self._take(p, 7 + UAT_LEN[k.lower()])
            return r and (k, r[0], r[1])
        if t == T_ADSL:
            if k.lower() not in ADSL_CH:
                return "unknown"
            r = self._take(p, 7 + 1 + 1)              # ts rssi rep L
            if not r:
                return r
            L = r[0][-1]
            if not ADSL_MIN_L <= L <= ADSL_MAX_L:
                return "unknown"
            r2 = self._take(r[1], L)
            return r2 and (k, r[0] + r2[0], r2[1])
        # Remote ID
        if k.lower() not in RID_TR:
            return "unknown"
        r = self._take(p, 7 + 6 + 1)                  # ts rssi addr6 odid[0]
        if not r:
            return r
        if r[0][-1] >> 4 == 0xF:
            r2 = self._take(r[1], 2)
            if not r2:
                return r2
            n = _odid_len(r[0][-1:] + r2[0])
            if n is False:
                return "unknown"
            r3 = self._take(r2[1], n - 3)
            return r3 and (k, r[0] + r2[0] + r3[0], r3[1])
        r2 = self._take(r[1], ODID_MSG - 1)
        return r2 and (k, r[0] + r2[0], r2[1])

    def feed(self, data: bytes) -> list[Frame]:
        self.buf += data
        out: list[Frame] = []
        b = self.buf
        i = 0
        while True:
            j = b.find(ESC, i)
            if j < 0:
                self.skipped += len(b) - i
                i = len(b)
                break
            self.skipped += j - i
            i = j
            if i + 1 >= len(b):
                break
            t = b[i + 1]
            if t == ESC:                    # stray escaped pair: skip it
                self.skipped += 2
                i += 2
                continue
            res = self._body(t, i + 2)
            if res is None:
                break                       # incomplete, wait for more
            if res is False:
                self.skipped += 1           # lone 0x1a inside: resync there
                i += 1
                continue
            if res == "unknown":
                nxt = self._skip_frame(i)
                if nxt is None:
                    break
                self.unknown += 1
                self.skipped += nxt - i
                i = nxt
                continue
            letter, body, nxt = res
            f = Frame(t, letter, body, bytes(b[i:nxt]))
            f.fields = parse_fields(t, letter, body, self.sig_dbm)
            if t == T_CAPS and letter == CAPS_CONFIRM:
                self.sig_dbm = bool(f.fields["bits"] & CAP_SIG_DBM)
            out.append(f)
            i = nxt
        del b[:i]
        return out


def parse_fields(t: int, k: str, body: bytes, sig_dbm: bool = False) -> dict:
    if t == T_UUID:
        return {"kind": "uuid", "uuid": body.decode(errors="replace")}
    if t == T_INGEST_ID:
        return {"kind": "ingest_id", "id": body.hex()}
    if t == T_CAPS:
        _, ver, bits = dec_caps_body(k.encode() + body)
        kind = {CAPS_QUERY: "caps_query", CAPS_ANSWER: "caps_answer", CAPS_CONFIRM: "caps_confirm"}[k]
        return {"kind": kind, "version": ver, "bits": bits,
                "features": cap_names(bits)}
    ts, rssi, p = int.from_bytes(body[:6], "big"), body[6], body[7:]
    common = {"ts": ts, "rssi": rssi}
    if sig_dbm:                      # v3.2: every frame type follows bit 0, 0xED/0xEF included
        common = {"ts": ts, "dbm": dbm_from_byte(rssi)}
    if t in MODE_LEN:
        return {"kind": {0x31: "mode_ac", 0x32: "mode_s_short", 0x33: "mode_s_long"}[t], **common, "msg": p.hex()}
    clock = ("modes" if ts else "unsynced") if k.isupper() else "not_mlat"
    if t == T_UAT:
        return {"kind": "uat_" + k.lower(), "clock": clock, **common, "msg": p.hex()}
    if t == T_ADSL:
        return {"kind": "adsl", "clock": clock, **common, "channel": ADSL_CH[k.lower()],
                "repaired": None if p[0] == REP_UNKNOWN else p[0], "onair": p[1:].hex()}
    odid = p[6:]
    return {"kind": "rid", "clock": clock, **common, "transport": RID_TR[k.lower()], "addr": p[:6].hex(":"),
            "odid": odid.hex(), "odid_msgs": odid_types(odid)}


def odid_types(odid: bytes) -> list[int]:
    """ASTM F3411 message types (high nibble) in a message or pack."""
    if odid[0] >> 4 == 0xF:
        return [odid[3 + i * ODID_MSG] >> 4 for i in range(odid[2])]
    return [odid[0] >> 4]


def decode_all(data: bytes, sig_dbm: bool = False) -> tuple[list[Frame], Decoder]:
    d = Decoder(sig_dbm)
    return d.feed(data), d


# ---- golden vectors ------------------------------------------------------------------------
def _odid_msg(mtype: int, fill: int) -> bytes:
    return bytes([mtype << 4 | 0x2]) + bytes([fill]) * 24


def build_vectors() -> list[dict]:
    ts = 0x01_23_45_1A_67_89                     # contains 0x1a on purpose
    adsl_body = bytes.fromhex("00" "02" "1a" "3c4d5e6f" "1a1a" "00112233445566778899aabb" "0102" "a1b2c3")
    adsl = bytes([len(adsl_body)]) + adsl_body    # L = 26 = 0x1a: the length byte itself is escaped
    pack = bytes([0xF2, 25, 3]) + _odid_msg(0, 0x1A) + _odid_msg(1, 0x11) + _odid_msg(4, 0x44)
    mac = bytes.fromhex("c0ffee1a0001")
    v = []

    def add(name, wire, note="", mode=None):
        frames, d = decode_all(wire, mode == "dbm")
        e = {"name": name, "note": note, "hex": wire.hex(),
             "decoded": [dict(type=f.type, **f.fields) for f in frames],
             "skipped": d.skipped, "unknown": d.unknown}
        if mode:
            e["signal_mode"] = mode      # v3.2: decode as after 'C' with bit 0
        v.append(e)

    add("uuid", enc_uuid("0123abcd-0123-4567-89ab-0123456789ab"))
    add("mode_s_long", enc_mode_s(bytes.fromhex("8d4840d6202cc371c32ce0576098"), ts, 0x1A),
        "DF17, ts and rssi contain 0x1a")
    add("uat_s_old", enc_uat("s", 0x00_00_00_00_1A_1A, 200, bytes(range(0x10, 0x10 + 30)), modes_clock=False),
        "BEAST feed / old firmware: ts not on the Mode S clock")
    add("uat_S_modes", enc_uat("s", ts, 0x1A, bytes(range(0x10, 0x10 + 30)), modes_clock=True),
        "READSBEE feed: ts on the Mode S clock")
    add("uat_L_unsynced", enc_uat("l", 0, 150, bytes([0x1A]) * 48, modes_clock=True),
        "READSBEE feed, sub-GHz clock not synced yet: ts 0")
    add("uat_U_uplink", enc_uat("u", ts, 90, bytes((i * 7) & 0xFF for i in range(552)), modes_clock=True),
        "uplink, 552 bytes")
    add("adsl_868_4", enc_adsl(adsl, ts, -87, "b", repaired=2), "L = 0x1a escaped, 2 erasures repaired, -87 dBm", mode="dbm")
    add("adsl_868_2_unsynced", enc_adsl(adsl, 0, -108, "a", repaired=0), "ts 0 = not synced, -108 dBm = 0x94", mode="dbm")
    add("adsl_channel_unknown", enc_adsl(adsl, ts, DBM_NONE, "x", repaired=REP_UNKNOWN),
        "no level: 0x80", mode="dbm")
    add("rid_bt4_single", enc_rid(_odid_msg(1, 0x1A), 0, -72, "b", mac),
        "Location message, BT4 legacy, no timestamp", mode="dbm")
    add("rid_wifi_beacon_pack", enc_rid(pack, 0, -60, "w", bytes.fromhex("6a1a2b3c4d5e")),
        "message pack: Basic ID, Location, System", mode="dbm")
    add("rid_bt5_coded", enc_rid(_odid_msg(0, 0x22), 0, -95, "c", mac), mode="dbm")
    tail = enc_mode_s(bytes.fromhex("5d4840d6abcdef"), 2, 2)
    add("unknown_type", bytes([ESC, 0xF5, ord("q")]) + escape(b"\x1a\x33" + bytes(20)) + tail,
        "unknown type: skipped to the next frame start, the Mode S frame after it decodes")
    add("unknown_uat_letter", bytes([ESC, T_UAT, ord("z")]) + escape(bytes(7) + b"\x1a\x1a\x33" + bytes(9)) + tail,
        "known type, unknown letter: same skip")
    mixed = b"".join(bytes.fromhex(x["hex"]) for x in v) + enc_mode_s(bytes(7), 1, 1)
    add("mixed_stream", mixed, "all of the above back to back, then a short Mode S frame")

    # ---- v3: capabilities check, feeder -> server (appended; the v2 vectors above are unchanged)
    q = enc_caps(CAPS_QUERY, CAP_ADSL | CAP_RID | CAP_UAT_MODES_CLOCK)
    add("caps_query", q, "feeder -> server, right after 0xE4: can send ADS-L, Remote ID, upper-case UAT")
    add("caps_query_none", enc_caps(CAPS_QUERY, 0), "feeder that has nothing extra to send (still allowed)")
    add("feed_start_v3", enc_uuid("0bee0003-8172-d18c-0bee-00038172d18c") + q
        + enc_mode_s(bytes.fromhex("8d4840d6202cc371c32ce0576098"), ts, 0x1A),
        "start of every v3 TCP Beast feed: 0xE4, 0xE5 Q, then today's Beast")
    add("caps_bad_byte", bytes([ESC, T_CAPS, ord("Q"), 0x81, 0x07, 0x80]) + tail,
        "feature byte without the high bit: malformed, skipped like an unknown frame")

    # ---- v3: server -> feeder, what AnswerScanner must find (or not)
    def scan(name, wire, note=""):
        sc = AnswerScanner()
        a = sc.feed(wire)
        v.append({"name": name, "note": note, "direction": "server_to_feeder", "hex": wire.hex(),
                  "answer": None if a is None else {"version": a[0], "bits": a[1], "features": cap_names(a[1])}})

    ans = enc_caps(CAPS_ANSWER, CAP_ADSL | CAP_RID | CAP_UAT_MODES_CLOCK)
    ping_esc = bytes([ESC, T_PING, 0x00, 0x1A, 0x1A, 0x05])     # readsb ping 0x001a05, escaped
    scan("answer_all", ans, "readsbee accepts ADS-L, Remote ID, upper-case UAT")
    scan("answer_none", enc_caps(CAPS_ANSWER, 0), "readsbee that accepts nothing extra: plain Beast")
    scan("answer_rid_only", enc_caps(CAPS_ANSWER, CAP_RID), "policy: Remote ID only")
    scan("answer_after_pings", ping_esc + bytes([ESC, ord("1")]) + bytes(9) + ans,
         "readsb-style ping (escaped 1a) and a Beast heartbeat before the answer: skipped")
    scan("answer_future", enc_caps(CAPS_ANSWER, CAP_RID | 1 << 9, version=2),
         "newer server: version 2, an unknown bit 9; the feeder uses known bits only")
    scan("echo_of_query", q, "our own query echoed back by a reflector: not an answer")
    scan("answer_malformed", bytes([ESC, T_CAPS, ord("A"), 0x81, 0x07, 0x80]),
         "feature byte without the high bit: ignored")
    scan("answer_cut_by_frame", bytes([ESC, T_CAPS, ord("A"), 0x81]) + ans,
         "a 1a inside the caps bytes starts a new frame: the second, complete answer counts")
    scan("stock_readsb_silence", ping_esc * 3, "stock readsb: pings only, no answer -> timeout -> plain Beast")

    # ---- v3.1: signal level in dBm (bit 0) and the confirm frame
    all31 = CAP_ADSL | CAP_RID | CAP_UAT_MODES_CLOCK | CAP_SIG_DBM
    conf = enc_caps(CAPS_CONFIRM, all31)
    add("caps_query_v31", enc_caps(CAPS_QUERY, all31), "v3.1 feeder: can also send dBm levels (bit 0)")
    scan("answer_v31_all", enc_caps(CAPS_ANSWER, all31), "readsbee v3.1 accepts bits 0-3")
    scan("answer_v3_server", enc_caps(CAPS_ANSWER, CAP_ADSL | CAP_RID | CAP_UAT_MODES_CLOCK),
         "v3 server without bit 0: the feeder keeps the power byte")
    add("caps_confirm_dbm", conf, "feeder -> server after the answer: bits 0-3 in effect from the next frame")
    ds, dl = bytes.fromhex("5d4840d6abcdef"), bytes.fromhex("8d4840d6202cc371c32ce0576098")
    uat_s, uat_u = bytes(range(0x10, 0x10 + 30)), bytes((i * 7) & 0xFF for i in range(552))
    # same packets, both modes. Power-mode vectors stand alone; dBm-mode vectors start with the
    # confirm frame so the decoder knows the signal byte is dBm.
    for name, dbm, note in (("m87", -87, "-87 dBm: power byte 4, dBm byte a9"),
                            ("m105", -105, "-105 dBm: power byte 1 (floor), dBm 97"),
                            ("m20", -20, "-20 dBm: power byte 255 (saturated), dBm ec"),
                            ("none", None, "no level: power 0, dBm 80")):
        dv = "none" if dbm is None else dbm
        add(f"mode_s_short_power_{name}", enc_mode_s(ds, 0x1234, dbm_to_power(dbm)), note)
        add(f"mode_s_short_dbm_{name}", conf + enc_mode_s(ds, 0x1234, dbm=dv), note)
    add("mode_s_long_power_m60", enc_mode_s(dl, ts, dbm_to_power(-60)), "-60 dBm: power byte 45")
    add("mode_s_long_dbm_m60", conf + enc_mode_s(dl, ts, dbm=-60), "-60 dBm: dBm byte c4")
    add("mode_ac_power_m70", enc_mode_s(bytes.fromhex("1234"), 7, dbm_to_power(-70)),
        "0x31 (ADSBee doesn't send it; same rule if it ever does)")
    add("mode_ac_dbm_m70", conf + enc_mode_s(bytes.fromhex("1234"), 7, dbm=-70))
    add("mode_s_dbm_plus26_escaped", conf + enc_mode_s(ds, 0x1234, dbm=26),
        "+26 dBm = 0x1a: escaped like any body byte (impossible in practice)")
    add("uat_s_power_m95", enc_uat("s", 0x55, dbm_to_power(-95), uat_s, modes_clock=False), "-95 dBm: power 2")
    add("uat_s_dbm_m95", conf + enc_uat("s", 0x55, None, uat_s, modes_clock=False, dbm=-95), "-95 dBm: dBm a1")
    add("uat_S_dbm_m95", conf + enc_uat("s", ts, None, uat_s, modes_clock=True, dbm=-95),
        "bits 0 and 3: upper case and dBm together")
    add("uat_U_power_m70", enc_uat("u", ts, dbm_to_power(-70), uat_u, modes_clock=True), "uplink, power byte")
    add("uat_U_dbm_m70", conf + enc_uat("u", ts, None, uat_u, modes_clock=True, dbm=-70), "uplink, dBm")
    add("v31_feed_start", enc_uuid("0bee0003-8172-d18c-0bee-00038172d18c") + enc_caps(CAPS_QUERY, all31)
        + enc_mode_s(ds, 1, dbm_to_power(-87)) + conf + enc_mode_s(ds, 2, dbm=-87)
        + enc_rid(_odid_msg(1, 0x33), 0, -70, "b", mac),
        "E4, query, a frame with the power byte (sent before the answer arrived), confirm, then dBm")

    # ---- v3.2: 0xED / 0xEF follow bit 0 too. Without it: Beast power byte (0 = no level).
    v3caps = CAP_ADSL | CAP_RID | CAP_UAT_MODES_CLOCK
    conf3 = enc_caps(CAPS_CONFIRM, v3caps)
    add("adsl_868_4_power", enc_adsl(adsl, ts, None, "b", repaired=2, rssi=dbm_to_power(-87)),
        "bit 0 not agreed: -87 dBm as power byte 4 (cf. adsl_868_4, dBm a9)")
    add("adsl_868_2_power_floor", enc_adsl(adsl, 0, None, "a", repaired=0, rssi=dbm_to_power(-108)),
        "-108 dBm reads as power byte 1: the -100 dBm floor (cf. adsl_868_2_unsynced, dBm 94)")
    add("adsl_power_none", enc_adsl(adsl, ts, None, "x", repaired=REP_UNKNOWN, rssi=dbm_to_power(None)),
        "no level in power-byte mode: 0 (dBm mode: 80)")
    add("rid_bt4_single_power", enc_rid(_odid_msg(1, 0x1A), 0, None, "b", mac, rssi=dbm_to_power(-72)),
        "bit 0 not agreed: -72 dBm as power byte 16 (cf. rid_bt4_single)")
    add("rid_bt4_power_saturated", enc_rid(_odid_msg(1, 0x1A), 0, None, "b", mac, rssi=dbm_to_power(-34)),
        "-34 dBm (strongest Remote ID in the t1069 capture) reads 255: the -40 dBm ceiling")
    add("rid_wifi_beacon_pack_power", enc_rid(pack, 0, None, "w", bytes.fromhex("6a1a2b3c4d5e"),
                                              rssi=dbm_to_power(-60)), "pack, -60 dBm as power byte 45")
    add("rid_bt4_dbm_after_confirm", conf + enc_rid(_odid_msg(1, 0x1A), 0, -72, "b", mac),
        "bits 0-3 confirmed: Remote ID level in dBm (b8)")
    add("v32_stream_no_dbm", conf3 + enc_mode_s(ds, 3, dbm_to_power(-87))
        + enc_adsl(adsl, ts, None, "a", repaired=1, rssi=dbm_to_power(-87))
        + enc_rid(_odid_msg(1, 0x33), 0, None, "b", mac, rssi=dbm_to_power(-87)),
        "readsbee answered bits 1–3 only: C 8e, then Mode S, ADS-L, Remote ID all with power byte 4")
    add("v32_stream_dbm", conf + enc_mode_s(ds, 3, dbm=-87)
        + enc_adsl(adsl, ts, -87, "a", repaired=1) + enc_rid(_odid_msg(1, 0x33), 0, -87, "b", mac),
        "bits 0-3 confirmed: C 8f, then the same three frames, all with dBm a9")
    return v


def check(path: Path) -> int:
    want = json.loads(path.read_text())
    got = build_vectors()
    bad = 0
    for w, g in zip(want, got):
        if w != g:
            print("MISMATCH", w["name"])
            bad += 1
    if len(want) != len(got):
        bad += 1
    # decoded frames re-assemble to the input, and byte-at-a-time feeding gives the same frames
    for g in got:
        wire = bytes.fromhex(g["hex"])
        if g.get("direction") == "server_to_feeder":
            sc = AnswerScanner()
            slow = None
            for x in wire:
                slow = sc.feed(bytes([x])) or slow
            want_a = g["answer"] and (g["answer"]["version"], g["answer"]["bits"])
            if slow != (want_a or None):
                print("SCANNER STREAMING MISMATCH", g["name"])
                bad += 1
            continue
        frames, d = decode_all(wire, g.get("signal_mode") == "dbm")
        known = b"".join(f.raw for f in frames)
        if g["unknown"] == 0 and known != wire:
            print("RAW MISMATCH", g["name"])
            bad += 1
        d2 = Decoder(g.get("signal_mode") == "dbm")
        slow = [f for x in wire for f in d2.feed(bytes([x]))]
        if [f.raw for f in slow] != [f.raw for f in frames]:
            print("STREAMING MISMATCH", g["name"])
            bad += 1
    print("vectors ok" if not bad else f"{bad} failures")
    return 1 if bad else 0


if __name__ == "__main__":
    vp = Path(__file__).with_name("vectors.json")
    if "--write" in sys.argv:
        vp.write_text(json.dumps(build_vectors(), indent=1) + "\n")
        print("wrote", vp)
    else:
        sys.exit(check(vp))
