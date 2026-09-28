#!/usr/bin/env python3
"""
Checks that an ADSBee 1090 .ota image contains nothing that an older firmware's AT parser would execute if part of the
image were misread as console text during an OTA update.

Why: up to 0.9.1-rc4, the RP2040 receiving an OTA reads each AT+OTA=WRITE payload with a 5 s timeout. Payload bytes
that arrive after the timeout (common on a busy network) go to the normal console line assembler, and CppAT runs any
"AT+<CMD>..." it finds anywhere in a line. The image being sent contains text such as the help line
"AT+BOOT_USB_UF2=1DEADBEE", which reboots the RP2040 into its USB bootloader: a unit reachable only over the network
then needs a power cycle. Newer firmware no longer parses payload bytes (see common/comms/at_console_guard.hh), but
devices still running the old firmware can only be protected by the image itself.

Two checks, both over each partition image exactly as the OTA sender transmits it (20 B header, then the application):

1. Strict: every occurrence of "AT+" followed by a command name that any firmware version knows, ended by one of the
   parser's terminators ("?", " ", "=", "\\r", "\\n") or by a NUL (end of the C string the old parser sees). The
   misread can start at any byte, so the position of the match doesn't matter and no chunking can avoid it. Text
   ending a chunk that is then merged with the sender's next command line only produces names that no longer match,
   so this check also covers chunk boundaries.
2. Emulation: replays 0.9.0-rc19's CommsManager::UpdateAT() + CppAT::ParseMessage() over the stream for each chunk
   size the senders use (web UI and CI tool: 12288 B; others: 4096 B), both as one continuous misread stream and with
   the misread starting fresh at every chunk. Reports every command whose callback would run.

Exit status 1 if either check finds anything. Usage:
    ota_at_scan.py adsbee_1090.ota [--commands-from comms_at.cc] [--verbose]
"""
import argparse
import re
import struct
import sys

# 0.9.0-rc19's command table: name -> (min_args, max_args). CppAT also accepts HELP.
RC19_COMMANDS = {
    "BAUD_RATE": (0, 2), "BIAS_TEE_ENABLE": (0, 2), "BOOT_USB_UF2": (0, 1), "DEVICE_INFO": (0, 5),
    "ETHERNET": (0, 1), "ESP32_ENABLE": (0, 1), "ESP32_FLASH": (0, 0), "ESP32_REBOOT_INFO": (0, 0),
    "ESP32_TRIGGER_ABORT": (0, 0), "FEED": (0, 5), "HOSTNAME": (0, 1), "LOG_LEVEL": (0, 1), "MAVLINK_ID": (0, 2),
    "NETWORK_INFO": (0, 0), "OTA": (0, 4), "PROTOCOL_OUT": (0, 2), "REBOOT": (0, 0), "RX_ENABLE": (0, 3),
    "RX_POSITION": (0, 8), "SETTINGS": (0, 3), "SUBG_ENABLE": (0, 2), "SUBG_FLASH": (0, 0), "INGEST_MODE_S": (1, 4),
    "INGEST_UAT": (1, 4), "TEST": (0, 1), "TL_READ": (0, 0), "TL_OFFSET": (0, 1), "UPTIME": (0, 0),
    "WATCHDOG": (0, 1), "WIFI_AP": (0, 4), "WIFI_STA": (0, 3), "HELP": (0, 0),
}
# Every command name used by any adsbee_1090 release (0.1.0 .. 0.9.1), including the 0.5.x-era "+NAME" table.
ALL_KNOWN_COMMANDS = set(RC19_COMMANDS) | {
    "BAUDRATE", "FLASH_ESP32", "PROTOCOL", "TL_SET", "FEED_ENABLE", "GNSS", "GNSS_FIX", "LED_BLINK", "LED_ENABLE",
    "REMOTE_ID", "REMOTE_ID_TX",
}
SENDER_CHUNK_SIZES = (12288, 4096)
HEADER_LEN = 20
AT_BUF_MAX_LEN = 1200  # CommsManager::kATCommandBufMaxLen
ARG_MAX_LEN = 128  # CPP_AT_ARG_MAX_LEN
MAX_NUM_ARGS = 10  # CPP_AT_MAX_NUM_ARGS
TERMINATORS = b"? =\r\n"


def partitions(ota):
    n = struct.unpack_from("<I", ota, 0)[0]
    for p in range(n):
        off = struct.unpack_from("<I", ota, 4 + 4 * p)[0]
        magic, _ver, app_len, _crc, _status = struct.unpack_from("<5I", ota, off)
        if magic != 0x0AD5BEEE:
            raise ValueError(f"partition {p}: bad magic {magic:#x}")
        yield p, ota[off:off + HEADER_LEN + app_len]


def strict_hits(data, names):
    """Every AT+<known name> ended by a parser terminator or NUL, regardless of where a misread would start."""
    hits = []
    for m in re.finditer(rb"AT\+([A-Z0-9_]+)", data):
        name = m.group(1).decode()
        end = m.end()
        nxt = data[end:end + 1]
        if name in names and (nxt == b"" or nxt in (b"\x00",) or nxt[0] in TERMINATORS):
            hits.append((m.start(), data[m.start():m.start() + 60].split(b"\x00")[0]))
    return hits


def cppat_parse(msg):
    """CppAT::ParseMessage() as of cppAT d115bce. Returns [(name, op, args)] of callbacks that would run."""
    ran = []
    start = msg.find(b"AT+")
    while start != -1:
        start += 3
        ends = [i for i in (msg.find(bytes([c]), start) for c in TERMINATORS) if i != -1]
        cmd_end = min(ends) if ends else len(msg)
        name = msg[start:cmd_end]
        if not name:
            return ran
        name = name.decode("latin1")
        if name not in RC19_COMMANDS:
            return ran
        start += len(name)
        op = ""
        if start < len(msg):
            if msg[start] not in b"\r\n":
                op = chr(msg[start])
            while start < len(msg) and not chr(msg[start]).isalnum() and msg[start] not in b",-":
                start += 1
        line_ends = [i for i in (msg.find(b"\r", start), msg.find(b"\n", start)) if i != -1]
        args_string = msg[start:min(line_ends) if line_ends else len(msg)]
        args = []
        if args_string:
            args = args_string.split(b",")
            if args_string.endswith(b","):
                pass  # Trailing blank argument counts (CppAT keeps it).
        if len(args) > MAX_NUM_ARGS or any(len(a) > ARG_MAX_LEN for a in args):
            return ran
        lo, hi = RC19_COMMANDS[name]
        if not lo <= len(args) <= hi:
            return ran
        ran.append((name, op, [a.decode("latin1") for a in args]))
        start = msg.find(b"AT+", start)
    return ran


class RC19LineAssembler:
    """CommsManager::UpdateAT() network console path in 0.9.0-rc19."""

    def __init__(self):
        self.buf = bytearray()

    def push(self, data):
        ran = []
        for b in data:
            self.buf.append(b)
            if len(self.buf) >= AT_BUF_MAX_LEN:
                self.buf = bytearray()
                continue
            if b == 0x0A:
                msg = bytes(self.buf).split(b"\x00")[0]  # Parsed as a C string.
                ran += cppat_parse(msg)
                self.buf = bytearray()
        return ran


def sent_writes(data, chunk):
    """(file offset, bytes) of each AT+OTA=WRITE payload, as ota_upload.py and adsbee.js send them."""
    yield 0, data[:HEADER_LEN]
    for i in range(HEADER_LEN, len(data), chunk):
        yield i, data[i:i + chunk]


def emulation_hits(data):
    hits = []
    for chunk in SENDER_CHUNK_SIZES:
        continuous = RC19LineAssembler()
        for n, (off, payload) in enumerate(sent_writes(data, chunk)):
            for mode, asm in (("fresh", RC19LineAssembler()), ("continuous", continuous)):
                for ran in asm.push(payload):
                    hits.append((chunk, n, off, mode, ran))
    return hits


def commands_from_source(path):
    src = open(path, encoding="utf-8", errors="replace").read()
    return set(re.findall(r'\.command\s*=\s*"([A-Z0-9_]+)"', src))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ota")
    ap.add_argument("--commands-from", action="append", default=[],
                    help="comms_at.cc to add the current firmware's command names from")
    ap.add_argument("--verbose", action="store_true")
    a = ap.parse_args()
    names = set(ALL_KNOWN_COMMANDS)
    for path in a.commands_from:
        names |= commands_from_source(path)
    ota = open(a.ota, "rb").read()
    bad = 0
    for p, data in partitions(ota):
        s = strict_hits(data, names)
        e = emulation_hits(data)
        at_total = data.count(b"AT+")
        print(f"{a.ota} partition {p}: {len(data)} B, 'AT+' occurrences {at_total}, "
              f"executable (strict) {len(s)}, executed in rc19 emulation {len(e)}")
        for off, ctx in s:
            print(f"  STRICT   offset {off:#09x} (12 KiB write #{(off - HEADER_LEN) // 12288 + 1 if off >= HEADER_LEN else 0}): {ctx!r}")
        for chunk, n, off, mode, ran in e:
            print(f"  EMULATED chunk {chunk} write #{n} ({mode}) at {off:#x}: AT+{ran[0]} op {ran[1]!r} args {ran[2]}")
        if a.verbose:
            for m in re.finditer(rb"AT\+", data):
                print(f"  (info) 'AT+' at {m.start():#09x}: {data[m.start():m.start() + 40]!r}")
        bad += len(s) + len(e)
    if bad:
        print("FAIL: this image contains AT commands that firmware up to 0.9.1-rc4 could execute if an OTA write "
              "timed out. Keep 'AT+' out of strings in flash (see ota_at_scan.py).")
        return 1
    print("OK: no executable AT command in the image.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
