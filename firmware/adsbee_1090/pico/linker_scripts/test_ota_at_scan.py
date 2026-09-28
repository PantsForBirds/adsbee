#!/usr/bin/env python3
"""Tests for ota_at_scan.py. Run: python3 -m unittest test_ota_at_scan (from this directory)."""
import os
import struct
import subprocess
import sys
import tempfile
import unittest
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ota_at_scan as scan  # noqa: E402


def make_ota(app):
    """A two-partition .ota with the same application bytes in both partitions."""
    header = struct.pack("<5I", 0x0AD5BEEE, 0, len(app), zlib.crc32(app), 0xFFFFFFFF)
    part = header + app
    off0 = 12
    off1 = off0 + len(part)
    return struct.pack("<3I", 2, off0, off1) + part + part


FILLER = bytes(range(256)) * 64  # Machine-code-like bytes: control characters, NULs and newlines.


class StrictScan(unittest.TestCase):
    def test_help_text_is_flagged(self):
        data = FILLER + b"Reboot into the USB bootloader.\r\n\tAT+BOOT_USB_UF2=1DEADBEE\x00" + FILLER
        hits = scan.strict_hits(data, scan.ALL_KNOWN_COMMANDS)
        self.assertEqual(len(hits), 1)
        self.assertIn(b"BOOT_USB_UF2", hits[0][1])

    def test_nul_terminated_command_is_flagged(self):
        # CppAT runs "AT+REBOOT" with no operator when the C string ends right after the name.
        self.assertEqual(len(scan.strict_hits(b"\x00AT+REBOOT\x00", scan.ALL_KNOWN_COMMANDS)), 1)

    def test_marker_text_is_clean(self):
        data = FILLER + b"\tAT\x1aBOOT_USB_UF2=1DEADBEE\x00\tAT\x1aWATCHDOG=TEST\x00" + FILLER
        self.assertEqual(scan.strict_hits(data, scan.ALL_KNOWN_COMMANDS), [])

    def test_unknown_names_and_partial_names_are_clean(self):
        data = b"AT+FOO=1\r\nAT+REBOOTX\r\nAT+\r\nAT+ OTA=ERASE\r\n"
        self.assertEqual(scan.strict_hits(data, scan.ALL_KNOWN_COMMANDS), [])

    def test_legacy_names_are_known(self):
        # 0.5.x-era names must be flagged too.
        self.assertEqual(len(scan.strict_hits(b"AT+BAUDRATE=0,1\r\n", scan.ALL_KNOWN_COMMANDS)), 1)


class RC19Emulation(unittest.TestCase):
    def test_help_line_runs_boot_command(self):
        asm = scan.RC19LineAssembler()
        ran = asm.push(b"\x08\xb5\x00\n\tAT+BOOT_USB_UF2=1DEADBEE\x00\x00\x70\x47\n")
        self.assertEqual(ran, [("BOOT_USB_UF2", "=", ["1DEADBEE"])])

    def test_wrong_arg_count_does_not_run(self):
        # WATCHDOG takes at most one argument.
        self.assertEqual(scan.RC19LineAssembler().push(b"AT+WATCHDOG=1,2\n"), [])

    def test_line_without_newline_is_not_parsed(self):
        self.assertEqual(scan.RC19LineAssembler().push(b"AT+REBOOT\r"), [])

    def test_overflow_drops_start_of_line(self):
        # rc19 empties its 1200 B buffer when full: what came before is lost, what comes after is still parsed.
        self.assertEqual(scan.RC19LineAssembler().push(b"AT+REBOOT" + b"x" * 1300 + b"\n"), [])
        self.assertEqual(scan.RC19LineAssembler().push(b"x" * 1300 + b"AT+REBOOT\n"), [("REBOOT", "", [])])

    def test_emulation_covers_every_sent_write(self):
        cmd = b"\n\tAT+WATCHDOG=TEST\x00\n"
        # Place the command in the 3rd 4 KiB write but the 1st 12 KiB write.
        data = b"\x00" * (scan.HEADER_LEN + 2 * 4096 + 100) + cmd + b"\x00" * 5000
        hits = scan.emulation_hits(data)
        self.assertEqual({(h[0], h[1]) for h in hits}, {(12288, 1), (4096, 3)})


class CommandLine(unittest.TestCase):
    def run_scan(self, app):
        with tempfile.NamedTemporaryFile(suffix=".ota", delete=False) as f:
            f.write(make_ota(app))
        try:
            return subprocess.run([sys.executable, scan.__file__, f.name], capture_output=True, text=True)
        finally:
            os.unlink(f.name)

    def test_fails_on_hit(self):
        r = self.run_scan(FILLER + b"\tAT+OTA=ERASE\x00" + FILLER)
        self.assertEqual(r.returncode, 1, r.stdout)
        self.assertIn("FAIL", r.stdout)

    def test_passes_clean_image(self):
        r = self.run_scan(FILLER + b"\tAT\x1aOTA=ERASE\x00" + FILLER)
        self.assertEqual(r.returncode, 0, r.stdout)


if __name__ == "__main__":
    unittest.main()
