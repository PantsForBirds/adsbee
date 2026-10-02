#!/usr/bin/env python3
"""Helpers for .github/workflows/release.yml (draft GitHub releases built from release tags).

Subcommands:
  info <tag>
      Parses a release tag (<product>-M.m.p or <product>-M.m.p-rcN), checks that the version in
      the source matches it, and prints product, version, release name and the asset list as
      key=value lines (for $GITHUB_OUTPUT). For adsbee_1421 it also checks the version the
      ADSBee 1421 Programmer build will bake in (programmer/scripts/hex_to_c.py parse_version).
  verify <tag> <dir>
      Checks the built assets in <dir>: every asset exists and is non-empty, and every .uf2 is a
      well-formed UF2 file. For adsbee_1421 also: the receiver ELF is an ARM ELF with debug info
      whose loaded contents match the .hex byte for byte, and the ADSBee 1421 Programmer ELF
      (adsbee_1421_programmer-fw<version>.elf, checked but not released) has the tag's version
      baked in as kFirmwareVersionStr.
  body <tag> <run_url> <commit> <dir> [<existing_body_file>]
      Prints the release body. Without an existing body: a stub with placeholders for the release
      notes. With one (a re-run on an existing draft): the same body with only the CI block
      (between the ci-assets markers) replaced, so hand-written notes are kept.

Uses only the Python standard library.
"""

import hashlib
import os
import re
import struct
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

PRODUCTS = {
    "adsbee_1090": {
        "display": "ADSBee 1090",
        # Release names follow the most common existing 1090 form (no "Version").
        "name_prefix": "ADSBee 1090 Firmware",
        "object_dictionary": "firmware/common/coprocessor/object_dictionary.cpp",
    },
    "adsbee_1421": {
        "display": "ADSBee 1421",
        "name_prefix": "ADSBee 1421 Firmware Version",
        "object_dictionary": "firmware/adsbee_1421/ti/object_dictionary/object_dictionary.cpp",
    },
}

TAG_RE = re.compile(r"^(adsbee_1090|adsbee_1421)-([0-9]+)\.([0-9]+)\.([0-9]+)(?:-rc([1-9][0-9]*))?$")

CI_START = "<!-- ci-assets:start -->"
CI_END = "<!-- ci-assets:end -->"


class ReleaseError(Exception):
    pass


def parse_tag(tag):
    m = TAG_RE.match(tag)
    if not m:
        raise ReleaseError(
            "Tag '{}' is not a release tag: expected adsbee_1090-M.m.p[-rcN] or "
            "adsbee_1421-M.m.p[-rcN]".format(tag)
        )
    product, major, minor, patch, rc = m.groups()
    version = "{}.{}.{}".format(major, minor, patch) + ("-rc{}".format(rc) if rc else "")
    return product, version, int(rc or 0)


def source_version(object_dictionary_path):
    """Same parse as firmware/adsbee_1421/cmake/fw_version.cmake and the CI workflow."""
    with open(object_dictionary_path, "r") as f:
        text = f.read()
    fields = {}
    for field in ("Major", "Minor", "Patch", "ReleaseCandidate"):
        m = re.search(r"ObjectDictionary::kFirmwareVersion" + field + r"\s*=\s*([0-9]+)[uUlL]*\s*;", text)
        if not m:
            raise ReleaseError("Could not parse kFirmwareVersion{} from {}".format(field, object_dictionary_path))
        fields[field] = int(m.group(1))
    version = "{}.{}.{}".format(fields["Major"], fields["Minor"], fields["Patch"])
    if fields["ReleaseCandidate"]:
        version += "-rc{}".format(fields["ReleaseCandidate"])
    return version


def release_name(product, version):
    m = re.match(r"^([0-9]+\.[0-9]+\.[0-9]+)(?:-rc([0-9]+))?$", version)
    name = "{} {}".format(PRODUCTS[product]["name_prefix"], m.group(1))
    if m.group(2):
        name += " Release Candidate {}".format(m.group(2))
    return name


def asset_names(product, version):
    if product == "adsbee_1090":
        return ["adsbee_1090.ota", "combined.uf2"]
    return [
        "adsbee_1421-{}.hex".format(version),
        "adsbee_1421-{}.elf".format(version),
        "adsbee_1421_programmer-fw{}.uf2".format(version),
    ]


def cmd_info(tag):
    product, version, _ = parse_tag(tag)
    od = os.path.join(REPO_ROOT, PRODUCTS[product]["object_dictionary"])
    src = source_version(od)
    if src != version:
        raise ReleaseError(
            "Tag {} says version {}, but {} is at {}. Tag the commit that carries this version, "
            "or bump the version first.".format(tag, version, PRODUCTS[product]["object_dictionary"], src)
        )
    if product == "adsbee_1421":
        # The Programmer bakes the version string parsed by hex_to_c.py into its image.
        sys.path.insert(0, os.path.join(REPO_ROOT, "firmware/adsbee_1421/programmer/scripts"))
        import hex_to_c  # noqa: E402

        baked = hex_to_c.parse_version(od)
        if baked != version:
            raise ReleaseError(
                "The ADSBee 1421 Programmer would bake firmware version {}, but the tag is {}".format(baked, tag)
            )
    print("product={}".format(product))
    print("version={}".format(version))
    print("name={}".format(release_name(product, version)))
    print("assets={}".format(" ".join(asset_names(product, version))))


# ------------------------------------------------------------------------------------------------
# Minimal ELF32 little-endian reader (enough for the ARM images built here).
# ------------------------------------------------------------------------------------------------


EM_ARM = 40
SHF_ALLOC = 0x2
SHT_NOBITS = 8


class Elf:
    def __init__(self, path):
        if not os.path.isfile(path):
            raise ReleaseError("{} not found".format(path))
        with open(path, "rb") as f:
            self.data = f.read()
        d = self.data
        if d[:4] != b"\x7fELF":
            raise ReleaseError("{} is not an ELF file".format(path))
        if d[4] != 1 or d[5] != 1:
            raise ReleaseError("{} is not a 32-bit little-endian ELF".format(path))
        (self.e_type, self.e_machine, _, _, self.e_phoff, self.e_shoff, _, _, self.e_phentsize,
         self.e_phnum, self.e_shentsize, self.e_shnum, self.e_shstrndx) = struct.unpack_from(
            "<HHIIIIIHHHHHH", d, 16)
        self.sections = []
        for i in range(self.e_shnum):
            fields = struct.unpack_from("<IIIIIIIIII", d, self.e_shoff + i * self.e_shentsize)
            self.sections.append(dict(zip(
                ("name", "type", "flags", "addr", "offset", "size", "link", "info", "addralign", "entsize"),
                fields)))
        shstr = self.sections[self.e_shstrndx]
        for s in self.sections:
            s["name"] = self._cstr(shstr["offset"] + s["name"])

    def _cstr(self, offset):
        end = self.data.index(b"\0", offset)
        return self.data[offset:end].decode("ascii", "replace")

    def section(self, name):
        for s in self.sections:
            if s["name"] == name:
                return s
        return None

    def load_memory(self):
        """addr -> byte for every allocated section with contents, at its load address (LMA),
        the way objcopy -O ihex lays out an image. Padding between sections inside a segment is
        left out, as objcopy does."""
        segments = []
        for i in range(self.e_phnum):
            p_type, p_offset, _, p_paddr, p_filesz, _, _, _ = struct.unpack_from(
                "<IIIIIIII", self.data, self.e_phoff + i * self.e_phentsize)
            if p_type == 1 and p_filesz:
                segments.append((p_offset, p_paddr, p_filesz))
        memory = {}
        for s in self.sections:
            if not s["flags"] & SHF_ALLOC or s["type"] == SHT_NOBITS or s["size"] == 0:
                continue
            for p_offset, p_paddr, p_filesz in segments:
                if p_offset <= s["offset"] and s["offset"] + s["size"] <= p_offset + p_filesz:
                    lma = p_paddr + (s["offset"] - p_offset)
                    break
            else:
                continue  # Not loaded (e.g. a section outside every PT_LOAD segment).
            chunk = self.data[s["offset"]:s["offset"] + s["size"]]
            for j, byte in enumerate(chunk):
                memory[lma + j] = byte
        return memory

    def symbol_bytes(self, symbol):
        symtab = self.section(".symtab")
        if symtab is None:
            raise ReleaseError("ELF has no symbol table")
        strtab = self.sections[symtab["link"]]
        for k in range(symtab["size"] // 16):
            st_name, st_value, st_size, _, _, st_shndx = struct.unpack_from(
                "<IIIBBH", self.data, symtab["offset"] + k * 16)
            if self._cstr(strtab["offset"] + st_name) != symbol:
                continue
            sec = self.sections[st_shndx]
            off = sec["offset"] + (st_value - sec["addr"])
            return self.data[off:off + st_size]
        raise ReleaseError("Symbol {} not found".format(symbol))




UF2_MAGIC_START0 = 0x0A324655
UF2_MAGIC_START1 = 0x9E5D5157
UF2_MAGIC_END = 0x0AB16F30


def check_uf2(path):
    with open(path, "rb") as f:
        data = f.read()
    if not data or len(data) % 512:
        raise ReleaseError("{}: size {} is not a positive multiple of 512".format(path, len(data)))
    blocks = len(data) // 512
    for i in range(blocks):
        block = data[i * 512:(i + 1) * 512]
        m0, m1 = struct.unpack_from("<II", block, 0)
        (num_blocks,) = struct.unpack_from("<I", block, 24)
        (m_end,) = struct.unpack_from("<I", block, 508)
        if (m0, m1, m_end) != (UF2_MAGIC_START0, UF2_MAGIC_START1, UF2_MAGIC_END):
            raise ReleaseError("{}: block {} has bad UF2 magic".format(path, i))
        if num_blocks != blocks:
            raise ReleaseError("{}: block {} says {} blocks, file has {}".format(path, i, num_blocks, blocks))
    print("{}: valid UF2, {} blocks".format(os.path.basename(path), blocks))


def cmd_verify(tag, directory):
    product, version, _ = parse_tag(tag)
    for name in asset_names(product, version):
        path = os.path.join(directory, name)
        if not os.path.isfile(path) or os.path.getsize(path) == 0:
            raise ReleaseError("Asset {} is missing or empty".format(path))
        if name.endswith(".uf2"):
            check_uf2(path)
    if product == "adsbee_1421":
        verify_1421(version, directory)


def verify_1421(version, directory):
    sys.path.insert(0, os.path.join(REPO_ROOT, "firmware/adsbee_1421/programmer/scripts"))
    import hex_to_c  # noqa: E402

    elf_path = os.path.join(directory, "adsbee_1421-{}.elf".format(version))
    hex_path = os.path.join(directory, "adsbee_1421-{}.hex".format(version))
    elf = Elf(elf_path)
    if elf.e_machine != EM_ARM:
        raise ReleaseError("{} is not an ARM ELF (e_machine {})".format(elf_path, elf.e_machine))
    if elf.section(".debug_info") is None:
        raise ReleaseError("{} has no debug info (.debug_info)".format(elf_path))
    elf_memory = elf.load_memory()
    hex_memory = hex_to_c.parse_intel_hex(hex_path)
    if elf_memory != hex_memory:
        only_elf = len(set(elf_memory) - set(hex_memory))
        only_hex = len(set(hex_memory) - set(elf_memory))
        differ = sum(1 for a in set(elf_memory) & set(hex_memory) if elf_memory[a] != hex_memory[a])
        raise ReleaseError(
            "{} and {} differ: {} bytes only in the ELF, {} only in the hex, {} with different "
            "values".format(elf_path, hex_path, only_elf, only_hex, differ))
    print("{}: ARM ELF with debug info; loadable contents match {} ({} bytes)".format(
        os.path.basename(elf_path), os.path.basename(hex_path), len(hex_memory)))

    programmer_elf = os.path.join(directory, "adsbee_1421_programmer-fw{}.elf".format(version))
    baked = Elf(programmer_elf).symbol_bytes("kFirmwareVersionStr").split(b"\0")[0].decode("ascii")
    if baked != version:
        raise ReleaseError(
            "The ADSBee 1421 Programmer image has firmware version {} baked in, expected {}".format(baked, version))
    print("ADSBee 1421 Programmer: baked firmware version {}".format(baked))


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def ci_block(tag, run_url, commit, directory):
    product, version, _ = parse_tag(tag)
    lines = [
        CI_START,
        "**Build**",
        "",
        "Built by [CI run]({}) from commit {}.".format(run_url, commit),
        "",
        "| Asset | SHA-256 |",
        "|---|---|",
    ]
    for name in asset_names(product, version):
        lines.append("| `{}` | `{}` |".format(name, sha256(os.path.join(directory, name))))
    lines.append(CI_END)
    return "\n".join(lines)


STUB = """<!-- Release notes: replace the placeholders below, then publish this draft. -->
* TODO: the changes in this release, one bullet per change with its PR number (#NNN).

**Upgrade notes**
* TODO (or delete this section).

**Testing**
* TODO: what was tested, on which hardware, with which assets.

"""


def cmd_body(tag, run_url, commit, directory, existing=None):
    block = ci_block(tag, run_url, commit, directory)
    if existing is None:
        print(STUB + block)
        return
    with open(existing, "r") as f:
        body = f.read().replace("\r\n", "\n")
    start, end = body.find(CI_START), body.find(CI_END)
    if start != -1 and end > start:
        body = body[:start] + block + body[end + len(CI_END):]
    else:
        body = body.rstrip("\n") + "\n\n" + block
    print(body)


def main(argv):
    try:
        if len(argv) == 3 and argv[1] == "info":
            cmd_info(argv[2])
        elif len(argv) == 4 and argv[1] == "verify":
            cmd_verify(argv[2], argv[3])
        elif len(argv) in (6, 7) and argv[1] == "body":
            cmd_body(*argv[2:])
        else:
            sys.stderr.write(__doc__)
            return 2
    except ReleaseError as e:
        print("::error::{}".format(e))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
