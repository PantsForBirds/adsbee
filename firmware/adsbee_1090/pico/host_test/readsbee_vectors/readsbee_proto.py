#!/usr/bin/env python3
"""ADSBee Beast extensions (2026-11-06): reference encoder/decoder.

Spec: FEED-PROTOCOL.md. Golden vectors: vectors/feed_vectors.json in readsbee, vectors.json next
to this file in adsbee's host_test/readsbee_vectors (the same file, byte for byte).
  python3 readsbee_proto.py           check against the vectors -> "vectors ok"
  python3 readsbee_proto.py --write   regenerate them

Wire rules (all frames): 0x1a <type> [<msg type letter>] <body>. Every 0x1a in <body> is sent
twice. The type byte and the msg type letter are never escaped (and never 0x1a).
Integers are big-endian.

Capabilities check (0xE5, FEED-PROTOCOL.md section 7): the feeder sends a query
'Q' right after 0xE4, a readsbee server answers 'A' on the same socket. Every byte after the
type is >= 0x80 or an ASCII letter, so the frame never contains 0x1a and is never escaped.

The feeder sends the agreed frames (query bits AND answer bits) as soon as it has read the
answer: every frame says what it is, so there is no switch point. The signal byte of every frame
type (0x31-0x33, 0xEC, 0xED, 0xEF) is the Beast power byte, 0 = no level. The letter 'C' of 0xE5
is reserved (dBm levels and the confirm frame were removed 2026-11-07).

GPS time sync (0xE6 'P', capability bit 0 since 2026-11-26, FEED-PROTOCOL.md section 12): one frame
per labeled PPS edge, the edge's 12 MHz count (ts6 + sub1/256) and its POSIX second; utc_of() is
the server-side conversion of a Mode S timestamp on the same connection to UTC.
"""
from __future__ import annotations

import json
import math
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
T_ADSL = 0xED            # ADS-L
T_RID = 0xEF             # Remote ID (ODID)
T_CAPS = 0xE5            # capabilities query / answer
T_GPS_TIME = 0xE6        # GPS time sync (PPS edge on the Mode S clock)
T_PING = 0x50            # readsb -> feeder: '1a P' + 3 bytes (escaped), only from a server

# ---- capabilities check -------------------------------------------------------------------
CAPS_QUERY, CAPS_ANSWER = "Q", "A"
CAPS_LETTERS = (CAPS_QUERY, CAPS_ANSWER)
CAPS_RESERVED = "C"             # reserved letter (was the confirm frame): skipped as a 6-byte frame
CAPS_VERSION = 1
CAP_GPS_TIME = 1 << 0           # 0xE6 GPS time sync (2026-11-26; was dBm levels before 2026-11-07, never deployed)
CAP_ADSL = 1 << 1               # 0xED ADS-L
CAP_RID = 1 << 2                # 0xEF Remote ID
CAP_UAT_MODES_CLOCK = 1 << 3    # 0xEC with upper-case S L U
CAP_NAMES = {CAP_GPS_TIME: "gps_time_e6", CAP_ADSL: "adsl_ed", CAP_RID: "rid_ef", CAP_UAT_MODES_CLOCK: "uat_upper"}
CAP_KNOWN = CAP_GPS_TIME | CAP_ADSL | CAP_RID | CAP_UAT_MODES_CLOCK
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

# GPS time sync 0xE6 'P' (section 12): ts6 sub1 utc5 flags1 rate3
GPS_LETTER = "P"
GPS_BODY_LEN = 6 + 1 + 5 + 1 + 3        # after the letter
GPS_FLAG_ALWAYS = 0x80                  # always set: flags is never 0x1a, never escaped
GPS_FLAG_STEP = 0x01                    # fit restarted / re-anchored since the last frame
GPS_RATE_UNKNOWN = -(1 << 23)           # 80 00 00
TICKS_PER_S = 12_000_000
TS_MOD = 1 << 48
GPS_VALID_S = 4                         # a frame is good for 4 s around its edge
GPS_PAIR_TOLERANCE_TICKS = 12           # 1 us


def escape(body: bytes) -> bytes:
    return body.replace(b"\x1a", b"\x1a\x1a")


def _letter(ch: str, table: dict, modes_clock: bool) -> int:
    if ch.lower() not in table:
        raise ValueError(f"bad msg type {ch!r}")
    return ord(ch.upper() if modes_clock else ch.lower())


def _hdr(ts: int, rssi: int) -> bytes:
    """ts6 + Beast power byte (every frame type)."""
    return ts.to_bytes(6, "big") + bytes([rssi])


# ---- signal level: dBm -> Beast power byte ------------------------------------------------
# Firmware table (firmware/common/comms/beast/generate_beast_tables.py -> beast_tables.hh):
# dBm clamped to -100..-40, dBFS = 0.75 * (dBm + 40), byte = round(255 * 10^(dBFS / 20)).
FW_MIN_DBM, FW_MAX_DBM, FW_DBFS_PER_DB = -100, -40, 45 / 60
FW_POWER_TABLE = [round(10 ** (((d - FW_MAX_DBM) * FW_DBFS_PER_DB) / 10 / 2 + math.log10(255)))
                  for d in range(FW_MIN_DBM, FW_MAX_DBM + 1)]


def dbm_to_power(dbm):
    """What ADSBee firmware writes (WriteRSSIdBmAsBeastPowerLevel): 255 at -40 dBm and above,
    1 at -100 dBm and below. None = no level -> 0 (Beast convention)."""
    if dbm is None:
        return 0
    return FW_POWER_TABLE[min(max(dbm - FW_MIN_DBM, 0), len(FW_POWER_TABLE) - 1)]


def power_to_dbm(b: int):
    """Inverse of the firmware mapping (float dBm), None for 0. Only meaningful for ADSBee feeds:
    255 means "-40 dBm or stronger", 1 means "-100 dBm or weaker"."""
    if b == 0:
        return None
    return FW_MAX_DBM + 20 * math.log10(b / 255) / FW_DBFS_PER_DB


# ---- encoders ------------------------------------------------------------------------------
def enc_mode_s(msg: bytes, ts: int, rssi: int) -> bytes:
    """rssi = Beast power byte."""
    t = {2: T_MODE_AC, 7: T_MODE_S_SHORT, 14: T_MODE_S_LONG}[len(msg)]
    return bytes([ESC, t]) + escape(_hdr(ts, rssi) + msg)


def enc_uuid(uuid: str) -> bytes:
    return bytes([ESC, T_UUID]) + uuid.encode()


def enc_uat(kind: str, ts: int, rssi: int, payload: bytes, modes_clock: bool) -> bytes:
    """kind s/l/u. modes_clock=True sends S/L/U (after bit 3 was agreed). rssi = power byte."""
    k = _letter(kind, UAT_LEN, modes_clock)
    if len(payload) != UAT_LEN[chr(k).lower()]:
        raise ValueError("bad UAT length")
    return bytes([ESC, T_UAT, k]) + escape(_hdr(ts, rssi) + payload)


def enc_adsl(onair: bytes, ts: int, rssi: int, channel: str, repaired: int = REP_UNKNOWN,
             modes_clock: bool = True) -> bytes:
    """onair = bytes after the sync word: length byte L, then L bytes (header .. CRC-24).
    channel 'a' 868.2, 'b' 868.4, 'x' unknown."""
    if not (ADSL_MIN_L <= onair[0] <= ADSL_MAX_L) or len(onair) != 1 + onair[0]:
        raise ValueError("bad ADS-L on-air buffer")
    k = _letter(channel, ADSL_CH, modes_clock)
    return bytes([ESC, T_ADSL, k]) + escape(_hdr(ts, rssi) + bytes([repaired]) + onair)


def enc_rid(odid: bytes, ts: int, rssi: int, transport: str, addr: bytes,
            modes_clock: bool = False) -> bytes:
    """odid = one 25-byte ASTM F3411 message, or a message pack (first byte 0xF?, size 25,
    count 1..9, then count x 25). addr = 6-byte source MAC, display order."""
    if len(addr) != 6 or _odid_len(odid) != len(odid):
        raise ValueError("bad Remote ID frame")
    k = _letter(transport, RID_TR, modes_clock)
    return bytes([ESC, T_RID, k]) + escape(_hdr(ts, rssi) + addr + odid)


def enc_caps(letter: str, bits: int, version: int = CAPS_VERSION) -> bytes:
    """Capabilities query ('Q', feeder -> server: what the feeder can send) or answer ('A',
    server -> feeder: what the server accepts). 14 feature bits in two bytes, 7 bits each,
    high bit always set; version byte 0x80 | version."""
    if letter not in CAPS_LETTERS or not 1 <= version <= 0x7F or not 0 <= bits < 1 << 14:
        raise ValueError("bad caps frame")
    return bytes([ESC, T_CAPS, ord(letter), 0x80 | version, 0x80 | (bits & 0x7F), 0x80 | (bits >> 7)])


def enc_gps_time(ts: int, sub: int, utc: int, step: bool, rate_ppb) -> bytes:
    """PPS edge of POSIX second utc at 12 MHz tick ts + sub/256 on this connection's Mode S clock.
    rate_ppb: one GNSS second = 12e6 * (1 + rate * 1e-9) ticks, None = unknown (80 00 00)."""
    rate = GPS_RATE_UNKNOWN if rate_ppb is None else rate_ppb
    if not (0 <= ts < TS_MOD and 0 <= sub < 256 and 0 < utc < 1 << 40 and GPS_RATE_UNKNOWN <= rate < 1 << 23):
        raise ValueError("bad gps time frame")
    head = ts.to_bytes(6, "big") + bytes([sub]) + utc.to_bytes(5, "big")
    return (bytes([ESC, T_GPS_TIME, ord(GPS_LETTER)]) + escape(head)
            + bytes([GPS_FLAG_ALWAYS | (GPS_FLAG_STEP if step else 0)]) + escape(rate.to_bytes(3, "big", signed=True)))


def dec_caps_body(body: bytes):
    """letter + 3 bytes -> (letter, version, bits) or None if malformed. The reserved letter 'C'
    decodes too (readsbee skips it as one 6-byte frame); nothing acts on it."""
    if len(body) != 4 or chr(body[0]) not in CAPS_LETTERS + (CAPS_RESERVED,) or any(b < 0x80 for b in body[1:]):
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

    def __init__(self):
        self.buf = bytearray()
        self.skipped = 0
        self.unknown = 0

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
        if t not in (T_UAT, T_ADSL, T_RID, T_GPS_TIME):
            return "unknown"
        if p >= len(b):
            return None
        k = chr(b[p])
        p += 1
        if t == T_GPS_TIME:
            if k != GPS_LETTER:                       # other letters reserved
                return "unknown"
            r = self._take(p, GPS_BODY_LEN)
            if not r:
                return r
            if not r[0][12] & GPS_FLAG_ALWAYS:
                return "unknown"
            return k, r[0], r[1]
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
            f.fields = parse_fields(t, letter, body)
            out.append(f)
            i = nxt
        del b[:i]
        return out


def parse_fields(t: int, k: str, body: bytes) -> dict:
    if t == T_UUID:
        return {"kind": "uuid", "uuid": body.decode(errors="replace")}
    if t == T_INGEST_ID:
        return {"kind": "ingest_id", "id": body.hex()}
    if t == T_CAPS:
        _, ver, bits = dec_caps_body(k.encode() + body)
        kind = {CAPS_QUERY: "caps_query", CAPS_ANSWER: "caps_answer", CAPS_RESERVED: "caps_reserved"}[k]
        return {"kind": kind, "version": ver, "bits": bits,
                "features": cap_names(bits)}
    if t == T_GPS_TIME:
        rate = int.from_bytes(body[13:16], "big", signed=True)
        return {"kind": "gps_time", "clock": "modes", "ts": int.from_bytes(body[:6], "big"), "sub": body[6],
                "utc": int.from_bytes(body[7:12], "big"), "step": bool(body[12] & GPS_FLAG_STEP),
                "rate_ppb": None if rate == GPS_RATE_UNKNOWN else rate}
    ts, rssi, p = int.from_bytes(body[:6], "big"), body[6], body[7:]
    common = {"ts": ts, "rssi": rssi}
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


# ---- GPS time sync: server side (section 12.3) -------------------------------------------
def _d48(a: int, b: int) -> int:
    """a - b as a signed 48-bit difference (the counter wraps every 2^48 / 12e6 s = 271.5 days)."""
    d = (a - b) % TS_MOD
    return d - TS_MOD if d >= TS_MOD // 2 else d


def pair_rate_ppb(prev: dict, cur: dict):
    """Rate from two consecutive frames, or None if cur has step or the pair is inconsistent: the
    edges must be (utc difference) x 12e6 x (1 + rate) ticks apart within 1 us (a mislabeled second)."""
    if cur["step"] or not 0 < cur["utc"] - prev["utc"] <= GPS_VALID_S:
        return None
    ticks = _d48(cur["ts"], prev["ts"]) + (cur["sub"] - prev["sub"]) / 256
    nominal = (cur["utc"] - prev["utc"]) * TICKS_PER_S
    rate = (ticks / nominal - 1) * 1e9
    if prev["rate_ppb"] is not None and cur["rate_ppb"] is not None:
        if abs(ticks - nominal * (1 + cur["rate_ppb"] * 1e-9)) > GPS_PAIR_TOLERANCE_TICKS:
            return None
    elif abs(rate) > 1e6:                            # more than 1000 ppm off: not a crystal
        return None
    return rate


def utc_of(ts: int, sync: dict, prev: dict = None):
    """UTC (whole POSIX seconds, ns) of a frame stamped ts on the same connection, from the latest
    0xE6 (and the one before it). None if there is no rate yet (rate unknown and no good pair), if
    the two frames disagree (consecutive, no step), or if ts is more than 4 s from the edge."""
    rate = sync["rate_ppb"]
    pair = prev is not None and not sync["step"] and 0 < sync["utc"] - prev["utc"] <= GPS_VALID_S
    if rate is None or (pair and prev["rate_ppb"] is not None):
        r = pair_rate_ppb(prev, sync) if pair else None
        if r is None:
            return None
        rate = r if rate is None else rate
    ticks = _d48(ts, sync["ts"]) - sync["sub"] / 256
    if abs(ticks) > GPS_VALID_S * TICKS_PER_S:
        return None
    s = ticks / (TICKS_PER_S * (1 + rate * 1e-9))
    whole = int(s // 1)
    return sync["utc"] + whole, (s - whole) * 1e9


def decode_all(data: bytes) -> tuple[list[Frame], Decoder]:
    d = Decoder()
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

    def add(name, wire, note="", exchange=None):
        frames, d = decode_all(wire)
        e = {"name": name, "note": note, "hex": wire.hex(),
             "decoded": [dict(type=f.type, **f.fields) for f in frames],
             "skipped": d.skipped, "unknown": d.unknown}
        if exchange:
            # both directions in order: hex = the feeder steps back to back (what the server reads)
            e["exchange"] = [{"from": who, "hex": b.hex()} for who, b in exchange]
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
    # The first set's ADS-L / Remote ID frames (adsl_868_4 .. rid_bt5_coded) carried int8 dBm
    # bytes (a9 94 80 b8 c4 a1) and were removed with dBm (2026-11-07); their power-byte
    # versions are below. mixed_stream keeps their bytes, read as power bytes.
    first = [bytes.fromhex(x["hex"]) for x in v] + [
        enc_adsl(adsl, ts, 0xA9, "b", repaired=2), enc_adsl(adsl, 0, 0x94, "a", repaired=0),
        enc_adsl(adsl, ts, 0x80, "x", repaired=REP_UNKNOWN), enc_rid(_odid_msg(1, 0x1A), 0, 0xB8, "b", mac),
        enc_rid(pack, 0, 0xC4, "w", bytes.fromhex("6a1a2b3c4d5e")), enc_rid(_odid_msg(0, 0x22), 0, 0xA1, "c", mac)]
    tail = enc_mode_s(bytes.fromhex("5d4840d6abcdef"), 2, 2)
    add("unknown_type", bytes([ESC, 0xF5, ord("q")]) + escape(b"\x1a\x33" + bytes(20)) + tail,
        "unknown type: skipped to the next frame start, the Mode S frame after it decodes")
    add("unknown_uat_letter", bytes([ESC, T_UAT, ord("z")]) + escape(bytes(7) + b"\x1a\x1a\x33" + bytes(9)) + tail,
        "known type, unknown letter: same skip")
    mixed = b"".join(first + [bytes.fromhex(x["hex"]) for x in v[-2:]]) + enc_mode_s(bytes(7), 1, 1)
    add("mixed_stream", mixed, "all of the above and the first set's ADS-L / Remote ID frames back to back, "
        "then a short Mode S frame")

    # ---- capabilities check, feeder -> server (appended; the frame vectors above are unchanged)
    q = enc_caps(CAPS_QUERY, CAP_ADSL | CAP_RID | CAP_UAT_MODES_CLOCK)
    add("caps_query", q, "feeder -> server, right after 0xE4: can send ADS-L, Remote ID, upper-case UAT")
    add("caps_query_none", enc_caps(CAPS_QUERY, 0), "feeder that has nothing extra to send (still allowed)")
    add("feed_start_query", enc_uuid("0bee0003-8172-d18c-0bee-00038172d18c") + q
        + enc_mode_s(bytes.fromhex("8d4840d6202cc371c32ce0576098"), ts, 0x1A),
        "start of every TCP Beast feed: 0xE4, 0xE5 Q, then today's Beast")
    add("caps_bad_byte", bytes([ESC, T_CAPS, ord("Q"), 0x81, 0x07, 0x80]) + tail,
        "feature byte without the high bit: malformed, skipped like an unknown frame")

    # ---- server -> feeder, what AnswerScanner must find (or not)
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

    # ---- bit 0 set (readsbee before 2026-11-07 answered 8f for dBm levels; since 2026-11-26 bit 0 is
    # GPS time sync 0xE6 and readsbee answers 8f by default)
    scan("answer_bit0_set", enc_caps(CAPS_ANSWER, CAP_GPS_TIME | CAP_ADSL | CAP_RID | CAP_UAT_MODES_CLOCK),
         "bit 0 set: GPS time sync 0xE6 since 2026-11-26 (readsbee's default answer); the feeder sends 0xE6 "
         "only if it offered bit 0")

    # ---- the signal byte of every frame type is the Beast power byte (0 = no level)
    ds, dl = bytes.fromhex("5d4840d6abcdef"), bytes.fromhex("8d4840d6202cc371c32ce0576098")
    uat_s, uat_u = bytes(range(0x10, 0x10 + 30)), bytes((i * 7) & 0xFF for i in range(552))
    for name, dbm, note in (("m87", -87, "-87 dBm: power byte 4"),
                            ("m105", -105, "-105 dBm: power byte 1 (floor)"),
                            ("m20", -20, "-20 dBm: power byte 255 (saturated)"),
                            ("none", None, "no level: power byte 0")):
        add(f"mode_s_short_power_{name}", enc_mode_s(ds, 0x1234, dbm_to_power(dbm)), note)
    add("mode_s_long_power_m60", enc_mode_s(dl, ts, dbm_to_power(-60)), "-60 dBm: power byte 45")
    add("mode_ac_power_m70", enc_mode_s(bytes.fromhex("1234"), 7, dbm_to_power(-70)),
        "0x31 (ADSBee doesn't send it; same rule if it ever does)")
    add("uat_s_power_m95", enc_uat("s", 0x55, dbm_to_power(-95), uat_s, modes_clock=False), "-95 dBm: power 2")
    add("uat_U_power_m70", enc_uat("u", ts, dbm_to_power(-70), uat_u, modes_clock=True), "uplink, power byte")

    # ---- 0xED / 0xEF: the same power byte, with its -100 dBm floor and -40 dBm ceiling
    add("adsl_868_4_power", enc_adsl(adsl, ts, dbm_to_power(-87), "b", repaired=2),
        "L = 0x1a escaped, 2 erasures repaired, -87 dBm as power byte 4")
    add("adsl_868_2_power_floor", enc_adsl(adsl, 0, dbm_to_power(-108), "a", repaired=0),
        "ts 0 = not synced; -108 dBm reads as power byte 1: the -100 dBm floor")
    add("adsl_power_none", enc_adsl(adsl, ts, dbm_to_power(None), "x", repaired=REP_UNKNOWN),
        "channel unknown, no level: power byte 0")
    add("rid_bt4_single_power", enc_rid(_odid_msg(1, 0x1A), 0, dbm_to_power(-72), "b", mac),
        "Location message, BT4 legacy, no timestamp, -72 dBm as power byte 16")
    add("rid_bt4_power_saturated", enc_rid(_odid_msg(1, 0x1A), 0, dbm_to_power(-34), "b", mac),
        "-34 dBm (strongest Remote ID in the t1069 capture) reads 255: the -40 dBm ceiling")
    add("rid_wifi_beacon_pack_power", enc_rid(pack, 0, dbm_to_power(-60), "w", bytes.fromhex("6a1a2b3c4d5e")),
        "message pack: Basic ID, Location, System; -60 dBm as power byte 45")
    add("rid_bt5_coded_power", enc_rid(_odid_msg(0, 0x22), 0, dbm_to_power(-95), "c", mac),
        "BT5 Coded, -95 dBm as power byte 2")

    # ---- a whole feed: query, answer, then the agreed frames (query bits AND answer bits)
    uuid = enc_uuid("0bee0003-8172-d18c-0bee-00038172d18c")
    q3 = CAP_ADSL | CAP_RID | CAP_UAT_MODES_CLOCK
    for name, ans_bits, note in (
            ("stream_query_answer", q3, "E4 + query 8e, a Mode S frame sent while waiting; answer 8e; then Mode S, "
             "ADS-L, Remote ID and upper-case UAT, all with power byte 4"),
            ("stream_query_answer_rid_only", CAP_RID, "same feed, the server answers 84 (Remote ID only): "
             "no ADS-L, UAT stays lower case")):
        agreed = negotiate(q3, ans_bits)
        before = uuid + enc_caps(CAPS_QUERY, q3) + enc_mode_s(ds, 1, dbm_to_power(-87))
        after = enc_mode_s(ds, 3, dbm_to_power(-87))
        if agreed & CAP_ADSL:
            after += enc_adsl(adsl, ts, dbm_to_power(-87), "a", repaired=1)
        if agreed & CAP_RID:
            after += enc_rid(_odid_msg(1, 0x33), 0, dbm_to_power(-87), "b", mac)
        after += enc_uat("s", ts, dbm_to_power(-87), uat_s, modes_clock=bool(agreed & CAP_UAT_MODES_CLOCK))
        add(name, before + after, note, exchange=[("feeder", before), ("server", enc_caps(CAPS_ANSWER, ans_bits)),
                                                  ("feeder", after)])

    # ---- Remote ID on the Mode S clock (ESP32 synced, adsbee #428): upper-case letter, real ts.
    add("rid_bt4_modes_clock_power", enc_rid(_odid_msg(1, 0x1A), ts, dbm_to_power(-72), "b", mac, modes_clock=True),
        "ESP32 synced: B, ts on the Mode S clock (~1 ms BLE callback time, not MLAT-grade), power byte 16")
    add("rid_wifi_beacon_pack_modes_clock", enc_rid(pack, ts, dbm_to_power(-60), "w", bytes.fromhex("6a1a2b3c4d5e"),
                                                    modes_clock=True),
        "W, ts on the Mode S clock (~2-4 us), pack, power byte 45")
    add("rid_bt4_modes_clock_wrap_power",
        enc_rid(_odid_msg(1, 0x33), 0xFF_FF_FF_FF_FF_FF, dbm_to_power(-70), "b", mac, modes_clock=True)
        + enc_rid(_odid_msg(1, 0x33), 2, dbm_to_power(-70), "b", mac, modes_clock=True),
        "the 48-bit 12 MHz counter wraps: ffffffffffff, then 000000000002; power byte 20")

    # ---- GPS time sync 0xE6 'P' (capability bit 0, section 12; from t-1336 gps_time_proto.py)
    utc = 1795483693                                 # 2026-11-24 01:28:13 UTC
    edge0 = 0x00_12_34_56_78_9B << 8 | 0x80          # in 1/256 ticks
    g = enc_gps_time(edge0 >> 8, edge0 & 0xFF, utc, False, 17300)
    add("gps_time", g,
        "the PPS edge of UTC second 1795483693 is at 12 MHz tick 0x00123456789b + 128/256; the counter runs "
        "17.3 ppm fast")
    add("gps_time_escaped_step", enc_gps_time(0x00_00_1A_1A_00_1A, 0x1A, utc + 1, True, 0x1A1A),
        "ts6, sub and rate contain 0x1a (escaped; flags 0x81 never is). step: the fit restarted or was re-anchored "
        "since the last frame, don't derive a rate across it")
    edge2 = edge0 + round(2 * TICKS_PER_S * (1 + 17300e-9) * 256)   # two seconds after gps_time, same counter
    add("gps_time_rate_unknown", enc_gps_time(edge2 >> 8, edge2 & 0xFF, utc + 2, False, None),
        "a feeder that doesn't measure a rate (80 00 00): the edge of UTC second 1795483695, two seconds after "
        "gps_time on the same counter; the server takes the rate from the two frames")
    q = enc_caps(CAPS_QUERY, CAP_KNOWN)
    m1 = enc_mode_s(ds, 0x00_12_34_50_00_00, 4)
    m2 = enc_mode_s(ds, 0x00_12_34_5A_00_00, 4)
    m3 = enc_mode_s(ds, 0x00_12_34_60_00_00, 4)
    add("stream_query_answer_gps_time", uuid + q + m1 + m2 + g + m3,
        "E4 + query 8f (GNSS with PPS configured), a Mode S frame while waiting; answer 8f (bit 0 agreed); "
        "then Mode S and the sync frame of the last labeled PPS edge",
        exchange=[("feeder", uuid + q + m1), ("server", enc_caps(CAPS_ANSWER, CAP_KNOWN)), ("feeder", m2 + g + m3)])
    return v


def _gps_ok(v: list) -> int:
    """0xE6 checks beyond the vectors (section 12): flags, conversion, pairs. Returns failures."""
    bad = 0

    def ok(cond, what):
        nonlocal bad
        if not cond:
            print("GPS TIME", what)
            bad += 1
    by = {e["name"]: e for e in v}
    sync, step, norate = (by[n]["decoded"][0] for n in ("gps_time", "gps_time_escaped_step", "gps_time_rate_unknown"))
    # reserved flag bits ignored; no 0x80: unknown; another letter: unknown; nothing around it lost
    raw = bytearray(bytes.fromhex(by["gps_time"]["hex"]))
    raw[15] |= 0x7E
    ok(dict(type=T_GPS_TIME, **Decoder().feed(bytes(raw))[0].fields) == sync, "reserved flags")
    tail = enc_mode_s(bytes.fromhex("5d4840d6abcdef"), 1, 4)
    for name, wire in (("no 0x80", bytes(raw[:15]) + b"\x01" + bytes(raw[16:])),
                       ("letter Q", raw[:2] + b"Q" + raw[3:]),
                       ("short", bytes(raw[:10]))):
        d = Decoder()
        fr = d.feed(tail + bytes(wire) + tail)
        ok([f.type for f in fr] == [0x32, 0x32] and d.unknown + (d.skipped > 0) >= 1, "malformed " + name)
    # conversion: at the edge, half a second later at 17.3 ppm (within half a tick), across the wrap, +-4 s
    ok(utc_of(sync["ts"], sync) == (sync["utc"] - 1, 1e9 * (1 - 0.5 / TICKS_PER_S / (1 + 17300e-9))), "at the edge")
    s, ns = utc_of((sync["ts"] + 6_000_104) % TS_MOD, sync)
    ok(s == sync["utc"] and abs(ns - 5e8) < 42, "half a second")
    wrap = dict(sync, ts=TS_MOD - 12_000_000, sub=0)
    s, ns = utc_of(1000, wrap)
    ok(s == wrap["utc"] + 1 and abs(ns - ((12e6 + 1000) / (12e6 * (1 + 17300e-9)) - 1) * 1e9) < 1, "48-bit wrap")
    ok(utc_of((sync["ts"] + 5 * TICKS_PER_S) % TS_MOD, sync) is None, "beyond 4 s")
    ok(utc_of(sync["ts"] - 4 * TICKS_PER_S + 1000, sync) is not None, "4 s before")
    # rate unknown: needs the frame before it; the pair gives 17.3 ppm back
    ok(utc_of(norate["ts"], norate) is None, "rate unknown, one frame")
    r = pair_rate_ppb(sync, norate)
    ok(r is not None and abs(r - 17300) < 0.2, "pair rate")
    s, ns = utc_of((norate["ts"] + 6_000_104) % TS_MOD, norate, sync)
    ok(s == norate["utc"] and abs(ns - 5e8) < 42, "rate from the pair")
    # step: no rate across it; a mislabeled second or an edge 13 ticks off fails the pair check
    ok(pair_rate_ppb(sync, dict(step, utc=sync["utc"] + 1)) is None, "step")
    nxt = dict(sync, utc=sync["utc"] + 1, ts=sync["ts"] + 12_000_208)
    ok(pair_rate_ppb(sync, nxt) is not None and utc_of(nxt["ts"], nxt, sync) is not None, "consecutive pair")
    ok(pair_rate_ppb(sync, dict(nxt, ts=nxt["ts"] + 13)) is None, "pair 13 ticks off")
    ok(utc_of(nxt["ts"], dict(nxt, ts=nxt["ts"] + 13), sync) is None, "conversion refused, pair 13 ticks off")
    ok(pair_rate_ppb(sync, dict(nxt, utc=sync["utc"] + 2)) is None, "mislabeled second")
    ok(utc_of(nxt["ts"], dict(nxt, step=True, ts=nxt["ts"] + 999), sync) is not None, "after step: frame alone")
    ok(max(len(bytes.fromhex(by[n]["hex"])) for n in ("gps_time", "gps_time_escaped_step", "gps_time_rate_unknown"))
       <= 3 + 2 * 15 + 1, "34 bytes max")
    return bad


def _exchange_ok(g: dict) -> bool:
    """query -> answer -> frames: the feeder steps make up hex, the scanner finds the answer, and
    after it the feeder sends only the agreed extensions (before it: none)."""
    steps = g["exchange"]
    if b"".join(bytes.fromhex(s["hex"]) for s in steps if s["from"] == "feeder").hex() != g["hex"]:
        return False
    q = next(f for f in decode_all(bytes.fromhex(steps[0]["hex"]))[0] if f.fields.get("kind") == "caps_query")
    a = AnswerScanner().feed(bytes.fromhex(steps[1]["hex"]))
    agreed = negotiate(q.fields["bits"], a[1])
    for i, s in ((0, steps[0]), (1, steps[2])):
        for f in decode_all(bytes.fromhex(s["hex"]))[0]:
            if f.type == T_ADSL and not (i and agreed & CAP_ADSL) or f.type == T_RID and not (i and agreed & CAP_RID):
                return False
            if f.type == T_UAT and f.letter.isupper() and not (i and agreed & CAP_UAT_MODES_CLOCK):
                return False
            if f.type == T_GPS_TIME and not (i and agreed & CAP_GPS_TIME):
                return False
    return True


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
        frames, d = decode_all(wire)
        known = b"".join(f.raw for f in frames)
        if g["unknown"] == 0 and known != wire:
            print("RAW MISMATCH", g["name"])
            bad += 1
        d2 = Decoder()
        slow = [f for x in wire for f in d2.feed(bytes([x]))]
        if [f.raw for f in slow] != [f.raw for f in frames]:
            print("STREAMING MISMATCH", g["name"])
            bad += 1
        if "exchange" in g and not _exchange_ok(g):
            print("EXCHANGE MISMATCH", g["name"])
            bad += 1
    bad += _gps_ok(got)
    print("vectors ok" if not bad else f"{bad} failures")
    return 1 if bad else 0


if __name__ == "__main__":
    vp = Path(__file__).parent / "vectors" / "feed_vectors.json"  # readsbee repo layout
    if not vp.parent.is_dir():
        vp = Path(__file__).with_name("vectors.json")             # adsbee host_test/readsbee_vectors
    if "--write" in sys.argv:
        vp.write_text(json.dumps(build_vectors(), indent=1) + "\n")
        print("wrote", vp)
    else:
        sys.exit(check(vp))
