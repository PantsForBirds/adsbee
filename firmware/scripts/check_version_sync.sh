#!/bin/bash
# Enforce the firmware/AGENTS.md rule: a change to a product's Settings struct (kSettingsVersion)
# must be accompanied by a firmware version bump for that product. Otherwise the device won't
# detect a version mismatch and won't reflash coprocessors, leaving them on a stale build with a
# mismatched Settings struct (SPI sync failures, reboots).
#
# Checked per product:
#   adsbee_1090: versions live in firmware/common (settings.hh kSettingsVersion,
#                coprocessor/object_dictionary.cpp kFirmwareVersion*); watched paths are the
#                ESP32/CC1312 coprocessor code and shared common/ code.
#   adsbee_1421: versions live in firmware/adsbee_1421/ti (settings/settings.hh,
#                object_dictionary/object_dictionary.cpp); watched paths are the whole product
#                directory and shared common/ code.
# Note: a firmware/common change therefore requires a version bump in BOTH products.
#
# Compares the coupled version numbers between an "old" and a "new" source and fails if the
# settings version changed without a matching firmware version bump, or if watched firmware
# paths changed without a firmware version bump.
#
# Usage: check_version_sync.sh <old_source> <new_source>
#   Each source is one of:
#     WORKTREE   - the current files on disk (unstaged + staged changes)
#     INDEX      - the staged (about-to-be-committed) content
#     <git-ref>  - any git revision, e.g. HEAD, origin/main, a commit SHA
#
#   Examples:
#     check_version_sync.sh HEAD WORKTREE      # local build: committed vs working tree
#     check_version_sync.sh HEAD INDEX         # pre-commit hook: committed vs staged
#     check_version_sync.sh origin/main HEAD   # CI: base branch vs PR head

set -euo pipefail

old_source="${1:-HEAD}"
new_source="${2:-WORKTREE}"

# Count of products that failed, so one product's problem doesn't hide the other's. The script
# exits non-zero at the end if this is non-zero.
failures=0

root=$(git rev-parse --show-toplevel 2>/dev/null) || {
    echo "WARNING: not inside a git repository, skipping version sync check."
    exit 0
}

# Read a file's contents from a given source. Fails loudly if the source can't be
# read (bad ref, missing file) rather than silently treating it as empty.
read_source() {
    local source="$1" file="$2"
    case "$source" in
        WORKTREE) cat "$root/$file" ;;
        INDEX)    git -C "$root" show ":$file" ;;
        *)        git -C "$root" show "$source:$file" ;;
    esac
}

# True (exit 0) if the file exists at the given source. Used to skip products that don't exist
# yet at the old source (e.g. diffing a migration commit against a pre-migration ref).
exists_at() {
    local source="$1" file="$2"
    case "$source" in
        WORKTREE) test -f "$root/$file" ;;
        INDEX)    git -C "$root" cat-file -e ":$file" 2>/dev/null ;;
        *)        git -C "$root" cat-file -e "$source:$file" 2>/dev/null ;;
    esac
}

# Extract the integer value(s) of a constant from a file at a given source.
# Uses POSIX extended regex (grep -E) so it works on GNU and BSD/macOS grep alike
# (grep -P / \K is not portable and silently no-ops on BSD grep).
# Usage: extract <source> <repo-relative-file> <constant-name-ERE>
extract() {
    local source="$1" file="$2" name="$3"
    read_source "$source" "$file" \
        | grep -oE "${name}[[:space:]]*=[[:space:]]*[0-9]+" \
        | grep -oE '[0-9]+$'
}

# True (exit 0) if any of the given paths differ between old_source and new_source.
paths_changed() {
    local old="$1" new="$2"
    shift 2
    case "$new" in
        WORKTREE) ! git -C "$root" diff --quiet "$old" -- "$@" ;;
        INDEX)    ! git -C "$root" diff --quiet --cached "$old" -- "$@" ;;
        *)        ! git -C "$root" diff --quiet "$old" "$new" -- "$@" ;;
    esac
}

# Check one product's settings/firmware version coupling and its watched paths.
# Usage: check_product <name> <settings-file> <version-file> <watched-path>...
check_product() {
    local product="$1" settings_file="$2" firmware_version_file="$3"
    shift 3
    local watched_paths=("$@")

    echo "--- $product ---"

    # A product that doesn't exist at the old source is new (e.g. this check running across the
    # commit that introduced it); there is nothing to compare against yet.
    if ! exists_at "$old_source" "$firmware_version_file" || ! exists_at "$old_source" "$settings_file"; then
        echo "$product does not exist at $old_source (new product), skipping."
        return 0
    fi

    # Assign in two steps (NOT `local x=$(...)`) so `set -e` + `pipefail` abort loudly on a read
    # failure (bad ref, missing file, or a constant that grep can't find) instead of masking it
    # as "unchanged" and passing silently.
    local settings_old settings_new firmware_old firmware_new
    settings_old=$(extract "$old_source" "$settings_file" kSettingsVersion)
    settings_new=$(extract "$new_source" "$settings_file" kSettingsVersion)
    firmware_old=$(extract "$old_source" "$firmware_version_file" 'kFirmwareVersion[A-Za-z]+')
    firmware_new=$(extract "$new_source" "$firmware_version_file" 'kFirmwareVersion[A-Za-z]+')

    if [ "$settings_old" != "$settings_new" ] && [ "$firmware_old" = "$firmware_new" ]; then
        echo "ERROR: $product kSettingsVersion changed but the firmware version in"
        echo "       $firmware_version_file was not bumped."
        echo "Fix: bump kFirmwareVersionPatch (or kFirmwareVersionReleaseCandidate for dev builds),"
        echo "     then commit it together with the settings change."
        failures=$((failures + 1))
        return 0
    fi

    # Broader guard. Firmware is only reflashed onto coprocessors / targets when the reported
    # firmware version differs, so any change to the watched firmware paths that is NOT paired
    # with a version bump silently leaves devices running a stale build. Harmless edits
    # (comments, formatting) also trip it, but a bump is cheap.
    if [ "$firmware_old" = "$firmware_new" ] && paths_changed "$old_source" "$new_source" "${watched_paths[@]}"; then
        echo "ERROR: $product firmware paths (${watched_paths[*]}) changed but the version in"
        echo "       $firmware_version_file was NOT bumped."
        echo "       Firmware is only reflashed on a version mismatch, so devices will keep"
        echo "       running STALE firmware after reflashing. Bump kFirmwareVersionPatch (or"
        echo "       kFirmwareVersionReleaseCandidate for dev builds) to force the reflash."
        failures=$((failures + 1))
        return 0
    fi

    echo "$product version sync OK."
}

echo "=== Checking Settings/firmware version sync ($old_source -> $new_source) ==="

check_product adsbee_1090 \
    "firmware/common/settings/settings.hh" \
    "firmware/common/coprocessor/object_dictionary.cpp" \
    "firmware/adsbee_1090/esp" \
    "firmware/adsbee_1090/ti" \
    "firmware/common"

check_product adsbee_1421 \
    "firmware/adsbee_1421/ti/settings/settings.hh" \
    "firmware/adsbee_1421/ti/object_dictionary/object_dictionary.cpp" \
    "firmware/adsbee_1421" \
    "firmware/common"

if [ "$failures" -ne 0 ]; then
    echo "=== Version sync check FAILED for $failures product(s) (see above) ==="
    exit 1
fi

echo "=== Version sync check passed ==="
