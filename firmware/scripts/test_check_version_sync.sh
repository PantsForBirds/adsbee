#!/bin/bash
# Tests for check_version_sync.sh. Builds a throwaway git repository with the version files of
# both products, makes one kind of change per case, and checks the script's verdict.
#
# Usage: bash firmware/scripts/test_check_version_sync.sh

set -euo pipefail

# CHECK_VERSION_SYNC overrides the script under test (e.g. to run the cases against an old copy).
script="${CHECK_VERSION_SYNC:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/check_version_sync.sh}"
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

cases_failed=0

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
    echo 'static const uint32_t kSettingsVersion = 10;' > firmware/common/settings/settings.hh
    printf 'kFirmwareVersionMajor = 0;\nkFirmwareVersionMinor = 8;\nkFirmwareVersionPatch = 4;\n' \
        > firmware/common/coprocessor/object_dictionary.cpp
    echo 'static const uint32_t kSettingsVersion = 3;' > firmware/adsbee_1421/ti/settings/settings.hh
    printf 'kFirmwareVersionMajor = 0;\nkFirmwareVersionMinor = 3;\nkFirmwareVersionPatch = 11;\n' \
        > firmware/adsbee_1421/ti/object_dictionary/object_dictionary.cpp
    for f in firmware/adsbee_1421/README.md firmware/adsbee_1421/AGENTS.md \
        firmware/adsbee_1421/programmer/README.md firmware/adsbee_1421/ti/main.cpp \
        firmware/adsbee_1421/programmer/bridge.cc firmware/adsbee_1421/programmer/bridge.hh \
        firmware/adsbee_1421/programmer/scripts/hex_to_c.py firmware/adsbee_1421/ti/CMakeLists.txt \
        firmware/adsbee_1090/esp/README.md firmware/adsbee_1090/esp/main.cpp \
        firmware/adsbee_1090/ti/README.md firmware/adsbee_1090/pico/main.cpp \
        firmware/common/README.md firmware/common/decoder.cc; do
        echo original > "$f"
    done
    git add -A
    git commit -q -m base
}

# expect <pass|fail> <description> <file-to-change>...
expect() {
    local want="$1" desc="$2"
    shift 2
    make_repo
    local f
    for f in "$@"; do
        case "$f" in
            # Bump the patch version; appending a line would leave the version unchanged.
            */object_dictionary.cpp)
                sed 's/Patch = \([0-9]*\);/Patch = 99;/' "$f" > "$f.new" && mv "$f.new" "$f" ;;
            *) echo changed >> "$f" ;;
        esac
    done
    local got=pass
    bash "$script" HEAD WORKTREE > "$tmp/out" 2>&1 || got=fail
    if [ "$got" = "$want" ]; then
        echo "ok   ($want) $desc"
    else
        echo "FAIL (wanted $want, got $got) $desc"
        sed 's/^/     | /' "$tmp/out"
        cases_failed=$((cases_failed + 1))
    fi
    cd - > /dev/null
}

expect pass "adsbee_1421: Markdown only" \
    firmware/adsbee_1421/README.md firmware/adsbee_1421/AGENTS.md firmware/adsbee_1421/programmer/README.md
expect fail "adsbee_1421: .cpp" firmware/adsbee_1421/ti/main.cpp
expect fail "adsbee_1421: .cc" firmware/adsbee_1421/programmer/bridge.cc
expect fail "adsbee_1421: .hh" firmware/adsbee_1421/programmer/bridge.hh
expect fail "adsbee_1421: .py" firmware/adsbee_1421/programmer/scripts/hex_to_c.py
expect fail "adsbee_1421: CMakeLists.txt" firmware/adsbee_1421/ti/CMakeLists.txt
expect fail "adsbee_1421: Markdown plus .hh" firmware/adsbee_1421/README.md firmware/adsbee_1421/programmer/bridge.hh
expect pass "adsbee_1090: Markdown only" firmware/adsbee_1090/esp/README.md firmware/adsbee_1090/ti/README.md
expect fail "adsbee_1090: esp .cpp" firmware/adsbee_1090/esp/main.cpp
expect pass "adsbee_1090: pico (not watched)" firmware/adsbee_1090/pico/main.cpp
expect pass "common: Markdown only" firmware/common/README.md
expect fail "common: .cc (needs both bumps)" firmware/common/decoder.cc
expect pass "adsbee_1421: .cpp with a version bump" firmware/adsbee_1421/ti/main.cpp \
    firmware/adsbee_1421/ti/object_dictionary/object_dictionary.cpp

if [ "$cases_failed" -ne 0 ]; then
    echo "$cases_failed case(s) failed"
    exit 1
fi
echo "All cases passed"
