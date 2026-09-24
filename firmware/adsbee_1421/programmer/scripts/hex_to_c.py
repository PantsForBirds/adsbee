#!/usr/bin/env python3
"""Bake an adsbee_1421 Intel HEX image into a C source file for the RP2040 programmer jig.

Parses the hex into 4-byte-aligned segments exactly like the host flasher
(software/adsbee_1421_flasher/adsbee_1421_flasher.py load_segments()), computes each
segment's IEEE 802.3 CRC32 (zlib.crc32 -- the same polynomial the CC13x4 ROM bootloader's
CRC32 command uses), and emits an array of FirmwareSegment records plus the firmware
version string parsed from object_dictionary.cpp.

Main-flash segments are emitted first and the CCFG segment last, preserving the
program-CCFG-last ordering that keeps an interrupted flash recoverable.

Must stay compatible with python 3.8 (the pico-docker container) and use only the stdlib.
"""

import argparse
import re
import sys
import zlib

MAIN_FLASH_BASE = 0x00000000
MAIN_FLASH_SIZE = 0x00100000  # CC1314R10: 1 MB main bank
CCFG_BASE = 0x50000000
CCFG_SIZE = 0x800
# Top 16 KB of the main bank is persistent storage owned by the application, never by the image:
# Settings @ 0x000FC000 (8 KB) and Device Info (part code, OTA keys) @ 0x000FE000 (8 KB); see
# firmware/adsbee_1421/ti/settings/settings.cpp. The programmer erases only image-covered sectors
# so these survive a reflash -- which only holds if the image never reaches up here. Refuse to
# bake one that does (the firmware linker script also caps FLASH at this address).
PROTECTED_FLASH_START = 0x000FC000


class HexError(Exception):
    pass


def parse_intel_hex(path):
    """Returns a dict addr -> byte for all data records in the hex file."""
    memory = {}
    base = 0  # From extended segment (type 02, <<4) or extended linear (type 04, <<16) records.
    with open(path, "r") as f:
        for lineno, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            if not line.startswith(":"):
                raise HexError("{}:{}: line does not start with ':'".format(path, lineno))
            try:
                raw = bytes.fromhex(line[1:])
            except ValueError:
                raise HexError("{}:{}: invalid hex characters".format(path, lineno))
            if len(raw) < 5:
                raise HexError("{}:{}: record too short".format(path, lineno))
            count, addr_hi, addr_lo, rectype = raw[0], raw[1], raw[2], raw[3]
            data = raw[4:-1]
            if len(data) != count:
                raise HexError("{}:{}: length mismatch".format(path, lineno))
            if sum(raw) & 0xFF != 0:
                raise HexError("{}:{}: checksum mismatch".format(path, lineno))
            if rectype == 0x00:  # Data
                start = base + ((addr_hi << 8) | addr_lo)
                for i, byte in enumerate(data):
                    memory[start + i] = byte
            elif rectype == 0x01:  # EOF
                break
            elif rectype == 0x02:  # Extended segment address
                if count != 2:
                    raise HexError("{}:{}: bad type-02 record".format(path, lineno))
                base = ((data[0] << 8) | data[1]) << 4
            elif rectype == 0x04:  # Extended linear address
                if count != 2:
                    raise HexError("{}:{}: bad type-04 record".format(path, lineno))
                base = ((data[0] << 8) | data[1]) << 16
            elif rectype in (0x03, 0x05):  # Start addresses: entry point, irrelevant for flashing.
                continue
            else:
                raise HexError(
                    "{}:{}: unsupported record type 0x{:02X}".format(path, lineno, rectype)
                )
    if not memory:
        raise HexError("{}: no data records found".format(path))
    return memory


def memory_to_segments(memory):
    """Coalesces the addr->byte dict into contiguous (start, bytearray) segments."""
    segments = []
    start = None
    data = None
    for addr in sorted(memory):
        if start is not None and addr == start + len(data):
            data.append(memory[addr])
        else:
            if start is not None:
                segments.append((start, data))
            start = addr
            data = bytearray([memory[addr]])
    segments.append((start, data))
    return segments


def align_segments(segments, memory):
    """Pads segments to 4-byte alignment: start down with real preceding bytes (they must
    exist in the image for the start to be misaligned mid-flash), tail up with 0xFF."""
    aligned = []
    for start, data in segments:
        data = bytearray(data)
        pad = start % 4
        if pad:
            start -= pad
            for i in range(pad):
                if start + i not in memory:
                    raise HexError(
                        "Segment start 0x{:08X} misaligned with no preceding bytes".format(
                            start + pad
                        )
                    )
                data.insert(i, memory[start + i])
        if len(data) % 4:
            data += b"\xff" * (4 - len(data) % 4)
        aligned.append((start, bytes(data)))
    return aligned


def classify_segments(segments):
    """Splits into (main, ccfg) lists, erroring on anything outside both regions or inside the
    protected settings / device-info sectors at the top of the main bank."""
    main_segments = []
    ccfg_segments = []
    for start, data in segments:
        end = start + len(data)
        if MAIN_FLASH_BASE <= start and end <= MAIN_FLASH_BASE + MAIN_FLASH_SIZE:
            if end > PROTECTED_FLASH_START:
                raise HexError(
                    "Segment 0x{:08X}..0x{:08X} overlaps the reserved settings / device-info "
                    "region at 0x{:08X}; refusing to bake an image that would clobber OTA keys".format(
                        start, end, PROTECTED_FLASH_START
                    )
                )
            main_segments.append((start, data))
        elif CCFG_BASE <= start and end <= CCFG_BASE + CCFG_SIZE:
            ccfg_segments.append((start, data))
        else:
            raise HexError(
                "Segment 0x{:08X}..0x{:08X} is outside main flash (1 MB @ 0x0) and CCFG "
                "(2 KB @ 0x{:08X})".format(start, end, CCFG_BASE)
            )
    return main_segments, ccfg_segments


def parse_version(object_dictionary_path):
    """Extracts "X.Y.Z" / "X.Y.Z-rcN" from object_dictionary.cpp, matching the regexes the
    adsbee_1421 CMakeLists uses on the same file."""
    with open(object_dictionary_path, "r") as f:
        text = f.read()
    fields = {}
    for name in ("Major", "Minor", "Patch", "ReleaseCandidate"):
        match = re.search(
            r"kFirmwareVersion{}\s*=\s*(\d+)".format(name), text
        )
        if not match:
            raise HexError(
                "{}: could not find kFirmwareVersion{}".format(object_dictionary_path, name)
            )
        fields[name] = int(match.group(1))
    version = "{Major}.{Minor}.{Patch}".format(**fields)
    if fields["ReleaseCandidate"] != 0:
        version += "-rc{}".format(fields["ReleaseCandidate"])
    return version


def emit_c(segments, version, out_path):
    lines = []
    lines.append("// Generated by hex_to_c.py -- do not edit.")
    lines.append('#include "firmware_image.hh"')
    lines.append("")
    for index, (start, data) in enumerate(segments):
        lines.append(
            "// 0x{:08X}..0x{:08X} ({} bytes)".format(start, start + len(data), len(data))
        )
        lines.append("static const uint8_t kSegment{}Data[] = {{".format(index))
        for offset in range(0, len(data), 16):
            chunk = data[offset : offset + 16]
            lines.append("    " + "".join("0x{:02X},".format(b) for b in chunk))
        lines.append("};")
    lines.append("")
    lines.append("const FirmwareSegment kFirmwareSegments[] = {")
    for index, (start, data) in enumerate(segments):
        crc = zlib.crc32(data) & 0xFFFFFFFF
        lines.append(
            "    {{0x{:08X}u, {}u, 0x{:08X}u, kSegment{}Data}},".format(
                start, len(data), crc, index
            )
        )
    lines.append("};")
    lines.append("const unsigned kFirmwareNumSegments = {};".format(len(segments)))
    lines.append('const char kFirmwareVersionStr[] = "{}";'.format(version))
    lines.append("")
    with open(out_path, "w") as f:
        f.write("\n".join(lines))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hex", required=True, help="Intel HEX firmware image")
    parser.add_argument(
        "--object-dictionary", required=True, help="object_dictionary.cpp with version constants"
    )
    parser.add_argument("--out", required=True, help="output C source path")
    args = parser.parse_args()

    try:
        memory = parse_intel_hex(args.hex)
        segments = align_segments(memory_to_segments(memory), memory)
        main_segments, ccfg_segments = classify_segments(segments)
        version = parse_version(args.object_dictionary)
        ordered = main_segments + ccfg_segments  # CCFG programmed last.
        emit_c(ordered, version, args.out)
    except (HexError, OSError) as e:
        print("hex_to_c.py: {}".format(e), file=sys.stderr)
        return 1

    total = sum(len(d) for _, d in ordered)
    print(
        "hex_to_c.py: baked {} ({} bytes, {} main + {} CCFG segments, version {})".format(
            args.hex, total, len(main_segments), len(ccfg_segments), version
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
