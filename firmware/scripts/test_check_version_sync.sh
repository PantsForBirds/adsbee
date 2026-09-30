#!/bin/bash
# Tests for check_version_sync.sh. Builds a throwaway git repository with the version files of
# both products and a set of release tags, makes one kind of change per case, and checks the
# script's verdict (and, where it matters, its message).
#
# Usage: bash firmware/scripts/test_check_version_sync.sh

set -euo pipefail

# CHECK_VERSION_SYNC overrides the script under test (e.g. to run the cases against an old copy).
script="${CHECK_VERSION_SYNC:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/check_version_sync.sh}"
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

cases_failed=0

od_1090=firmware/common/coprocessor/object_dictionary.cpp
od_1421=firmware/adsbee_1421/ti/object_dictionary/object_dictionary.cpp
settings_1090=firmware/common/settings/settings.hh
settings_1421=firmware/adsbee_1421/ti/settings/settings.hh

# Which release tags make_repo creates. The base commit is always adsbee_1090 0.9.1-rc4 and
# adsbee_1421 0.3.11-rc3.
#   unreleased - older releases only (latest: adsbee_1090-0.9.1-rc3, adsbee_1421-0.3.11-rc2)
#   released   - the older releases plus tags for the base versions themselves
#   none       - no release tags at all
tags=unreleased

# Write a version file for M.m.p-rcN (or M.m.p for a stable release).
# Usage: write_version <file> <version>
write_version() {
    local file="$1" version="$2" rc=0
    [[ "$version" =~ ^([0-9]+)\.([0-9]+)\.([0-9]+)(-rc([0-9]+))?$ ]]
    [ -n "${BASH_REMATCH[5]}" ] && rc="${BASH_REMATCH[5]}"
    printf 'const uint8_t ObjectDictionary::kFirmwareVersionMajor = %s;\n' "${BASH_REMATCH[1]}" > "$file"
    printf 'const uint8_t ObjectDictionary::kFirmwareVersionMinor = %s;\n' "${BASH_REMATCH[2]}" >> "$file"
    printf 'const uint8_t ObjectDictionary::kFirmwareVersionPatch = %s;\n' "${BASH_REMATCH[3]}" >> "$file"
    printf '// Use RC = 0 for a release build.\n' >> "$file"
    printf 'const uint8_t ObjectDictionary::kFirmwareVersionReleaseCandidate = %s;\n' "$rc" >> "$file"
    printf 'const uint32_t ObjectDictionary::kFirmwareVersion = (kFirmwareVersionMajor << 24);\n' >> "$file"
}

# Fresh repository with one commit holding both products' version files and some sources.
make_repo() {
    rm -rf "$tmp/repo"
    mkdir -p "$tmp/repo"
    cd "$tmp/repo"
    git init -q
    git config user.email test@example.com
    git config user.name test
    mkdir -p firmware/common/settings firmware/common/coprocessor \
        firmware/adsbee_1090/esp firmware/adsbee_1090/ti firmware/adsbee_1090/pico \
        firmware/adsbee_1421/ti/settings firmware/adsbee_1421/ti/object_dictionary \
        firmware/adsbee_1421/programmer/scripts
    echo 'static const uint32_t kSettingsVersion = 10;' > "$settings_1090"
    write_version "$od_1090" 0.9.1-rc4
    echo 'static const uint32_t kSettingsVersion = 3;' > "$settings_1421"
    write_version "$od_1421" 0.3.11-rc3
    for f in firmware/adsbee_1421/README.md firmware/adsbee_1421/AGENTS.md \
        firmware/adsbee_1421/programmer/README.md firmware/adsbee_1421/ti/main.cpp \
        firmware/adsbee_1421/programmer/bridge.cc firmware/adsbee_1421/programmer/bridge.hh \
        firmware/adsbee_1421/programmer/scripts/hex_to_c.py firmware/adsbee_1421/ti/CMakeLists.txt \
        firmware/adsbee_1090/esp/README.md firmware/adsbee_1090/esp/main.cpp \
        firmware/adsbee_1090/ti/README.md firmware/adsbee_1090/pico/main.cpp \
        firmware/common/README.md firmware/common/decoder.cc \
        firmware/common/coprocessor/spi_coprocessor.cpp; do
        echo original > "$f"
    done
    git add -A
    git commit -q -m base
    local tag
    if [ "$tags" != none ]; then
        # Includes a tag that doesn't follow the naming scheme (one exists upstream); the
        # script must ignore it.
        for tag in adsbee_1090-0.9.0-rc19 adsbee_1090-0.9.0 adsbee_1090-rc17 adsbee_1090-0.9.1-rc3 \
            adsbee_1421-0.3.10 adsbee_1421-0.3.11-rc1 adsbee_1421-0.3.11-rc2; do
            git tag "$tag"
        done
    fi
    if [ "$tags" = released ]; then
        git tag adsbee_1090-0.9.1-rc4
        git tag adsbee_1421-0.3.11-rc3
    fi
}

# Apply one change to the scratch repository's working tree.
#   <file>                      append a line to the file
#   version:<1090|1421>:<ver>   set that product's firmware version
#   settings:<1090|1421>        bump that product's kSettingsVersion
#   commit                      commit everything, then run the check as HEAD^ -> HEAD
#   tag:<name>                  tag HEAD
apply_change() {
    local change="$1" product file
    case "$change" in
        version:*)
            product="${change#version:}"
            file="od_${product%%:*}"
            write_version "${!file}" "${product#*:}" ;;
        settings:*)
            file="settings_${change#settings:}"
            sed 's/kSettingsVersion = \([0-9]*\);/kSettingsVersion = 99;/' "${!file}" > "${!file}.new"
            mv "${!file}.new" "${!file}" ;;
        commit)
            git add -A
            git commit -q -m change
            sources=(HEAD^ HEAD) ;;
        tag:*) git tag "${change#tag:}" ;;
        *) echo changed >> "$change" ;;
    esac
}

# expect <pass|fail> <description> [--msg <text the output must contain>] <change>...
expect() {
    local want="$1" desc="$2" msg=""
    shift 2
    if [ "${1:-}" = --msg ]; then
        msg="$2"
        shift 2
    fi
    make_repo
    sources=(HEAD WORKTREE)
    local change
    for change in "$@"; do
        apply_change "$change"
    done
    local got=pass
    bash "$script" "${sources[@]}" > "$tmp/out" 2>&1 || got=fail
    if [ "$got" != "$want" ]; then
        echo "FAIL (wanted $want, got $got) [tags: $tags] $desc"
        sed 's/^/     | /' "$tmp/out"
        cases_failed=$((cases_failed + 1))
    elif [ -n "$msg" ] && ! grep -qF -- "$msg" "$tmp/out"; then
        echo "FAIL (output lacks \"$msg\") [tags: $tags] $desc"
        sed 's/^/     | /' "$tmp/out"
        cases_failed=$((cases_failed + 1))
    else
        echo "ok   ($want) [tags: $tags] $desc"
    fi
    cd - > /dev/null
}

# Base versions unreleased: changes can accumulate under the next unreleased RC.
tags=unreleased
expect pass "adsbee_1421: .cpp, version unreleased and equal to base" \
    --msg "0.3.11-rc3 is unchanged from HEAD and unreleased" firmware/adsbee_1421/ti/main.cpp
expect pass "common: .cc, both versions unreleased and equal to base" firmware/common/decoder.cc
expect pass "adsbee_1421: settings change, version unreleased and equal to base" settings:1421
expect pass "adsbee_1421: .cpp with a bump to a higher unreleased version" \
    firmware/adsbee_1421/ti/main.cpp version:1421:0.3.12-rc1
expect fail "adsbee_1421: .cpp with the version set to a released RC" \
    --msg "as adsbee_1421-0.3.11-rc2." firmware/adsbee_1421/ti/main.cpp version:1421:0.3.11-rc2
expect fail "adsbee_1421: version set to a released stable version" \
    --msg "as adsbee_1421-0.3.10." version:1421:0.3.10
expect fail "adsbee_1421: version lower than the latest release (unreleased)" \
    --msg "lower than the latest release" version:1421:0.3.10-rc5
expect fail "adsbee_1090: an RC below the stable release of the same version (rc < stable)" \
    --msg "0.9.0-rc20, lower than the latest release" version:1090:0.9.0-rc20
expect fail "adsbee_1421: released version reused suggests the next free RC" \
    --msg "next unreleased RC, adsbee_1421-0.3.11-rc3" version:1421:0.3.11-rc1
expect pass "adsbee_1421: commit that is part of its own release tag" \
    firmware/adsbee_1421/ti/main.cpp commit tag:adsbee_1421-0.3.11-rc3

# Base versions released: any non-Markdown change under the watched paths needs a new version.
tags=released
expect pass "adsbee_1421: Markdown only" \
    firmware/adsbee_1421/README.md firmware/adsbee_1421/AGENTS.md firmware/adsbee_1421/programmer/README.md
expect fail "adsbee_1421: .cpp" --msg "adsbee_1421-0.3.11-rc3" firmware/adsbee_1421/ti/main.cpp
expect fail "adsbee_1421: .cc" firmware/adsbee_1421/programmer/bridge.cc
expect fail "adsbee_1421: .hh" firmware/adsbee_1421/programmer/bridge.hh
expect fail "adsbee_1421: .py" firmware/adsbee_1421/programmer/scripts/hex_to_c.py
expect fail "adsbee_1421: CMakeLists.txt" firmware/adsbee_1421/ti/CMakeLists.txt
expect fail "adsbee_1421: Markdown plus .hh" firmware/adsbee_1421/README.md firmware/adsbee_1421/programmer/bridge.hh
expect fail "adsbee_1421: settings change" --msg "mismatched Settings struct" settings:1421
expect pass "adsbee_1090: Markdown only" firmware/adsbee_1090/esp/README.md firmware/adsbee_1090/ti/README.md
expect fail "adsbee_1090: esp .cpp" --msg "next unreleased RC, adsbee_1090-0.9.1-rc5" \
    firmware/adsbee_1090/esp/main.cpp
expect pass "adsbee_1090: pico (not watched)" firmware/adsbee_1090/pico/main.cpp
expect pass "common: Markdown only" firmware/common/README.md
expect fail "common: .cc (needs both bumps)" firmware/common/decoder.cc version:1090:0.9.1-rc5
expect pass "common: .cc with both bumps" firmware/common/decoder.cc \
    version:1090:0.9.1-rc5 version:1421:0.3.11-rc4
expect pass "adsbee_1421: .cpp with a version bump" firmware/adsbee_1421/ti/main.cpp version:1421:0.3.11-rc4
expect pass "adsbee_1090: esp .cpp with a 1090-only bump (1090 version file not watched for 1421)" \
    --msg "adsbee_1421: no firmware changes" firmware/adsbee_1090/esp/main.cpp version:1090:0.9.1-rc5
expect pass "adsbee_1090: 1090-only bump, committed (refs)" \
    --msg "adsbee_1421: no firmware changes" firmware/adsbee_1090/esp/main.cpp version:1090:0.9.1-rc5 commit
expect fail "common: other coprocessor file still watched for 1421" \
    --msg "adsbee_1421-0.3.11-rc3" firmware/common/coprocessor/spi_coprocessor.cpp version:1090:0.9.1-rc5
expect fail "adsbee_1421: committed .cpp on top of the release (refs)" \
    firmware/adsbee_1421/ti/main.cpp commit

# No release tags: warn and fall back to comparing against the old source.
tags=none
expect fail "adsbee_1421: .cpp without a bump" --msg "git fetch --tags" firmware/adsbee_1421/ti/main.cpp
expect fail "adsbee_1421: settings change without a bump" --msg "mismatched Settings struct" settings:1421
expect pass "adsbee_1421: .cpp with a version bump" --msg "WARNING: no adsbee_1421-* release tags" \
    firmware/adsbee_1421/ti/main.cpp version:1421:0.3.11-rc4
expect pass "adsbee_1421: Markdown only" firmware/adsbee_1421/README.md

if [ "$cases_failed" -ne 0 ]; then
    echo "$cases_failed case(s) failed"
    exit 1
fi
echo "All cases passed"
