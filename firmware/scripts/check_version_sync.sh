#!/bin/bash
# Fails if a product's firmware changed but its firmware version is already released (firmware/AGENTS.md).
# Devices reflash coprocessors only when the version differs, so a reused version leaves them on a stale
# build, or with a mismatched Settings struct (SPI sync failures, reboots).
#
# Per product:
#   adsbee_1090: versions in firmware/common (settings.hh, coprocessor/object_dictionary.cpp); watches
#                the ESP32/CC1312 code and firmware/common.
#   adsbee_1421: versions in firmware/adsbee_1421/ti (settings/settings.hh,
#                object_dictionary/object_dictionary.cpp); watches firmware/adsbee_1421 and
#                firmware/common, except common/coprocessor/object_dictionary.cpp (1090 only).
#
# If watched paths or kSettingsVersion changed, the NEW version (<product>-M.m.p[-rcN], as release tags
# are named) must not be released (no such local tag; run `git fetch --tags`) and must not be lower than
# the latest release. Each RC sorts below its stable release (0.3.11-rc9 < 0.3.11 < 0.3.12-rc1). An
# unreleased version may be reused across changes. With no release tags, it warns and requires the
# version to differ from the old source's. Markdown files (*.md) are exempt; nothing else is (*.txt and
# images can feed a build).
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

# Products that failed; checked at the end so one failure doesn't hide another.
failures=0

root=$(git rev-parse --show-toplevel 2>/dev/null) || {
    echo "WARNING: not inside a git repository, skipping version sync check."
    exit 0
}

# Read a file from a source. Fails on a bad ref or missing file.
read_source() {
    local source="$1" file="$2"
    case "$source" in
        WORKTREE) cat "$root/$file" ;;
        INDEX)    git -C "$root" show ":$file" ;;
        *)        git -C "$root" show "$source:$file" ;;
    esac
}

# True if the file exists at the source (skips products new since the old source).
exists_at() {
    local source="$1" file="$2"
    case "$source" in
        WORKTREE) test -f "$root/$file" ;;
        INDEX)    git -C "$root" cat-file -e ":$file" 2>/dev/null ;;
        *)        git -C "$root" cat-file -e "$source:$file" 2>/dev/null ;;
    esac
}

# Extract the integer value(s) of a constant from a file at a given source.
# grep -E for GNU and BSD/macOS portability.
# Usage: extract <source> <repo-relative-file> <constant-name-ERE>
extract() {
    local source="$1" file="$2" name="$3"
    read_source "$source" "$file" \
        | grep -oE "${name}[[:space:]]*=[[:space:]]*[0-9]+" \
        | grep -oE '[0-9]+$'
}

# Documentation that never feeds a build. See the header before adding anything.
doc_only_excludes=(':(exclude,glob,icase)**/*.md')

# True if any of the paths differ between the sources, ignoring doc_only_excludes.
paths_changed() {
    local old="$1" new="$2"
    shift 2
    case "$new" in
        WORKTREE) ! git -C "$root" diff --quiet "$old" -- "$@" "${doc_only_excludes[@]}" ;;
        INDEX)    ! git -C "$root" diff --quiet --cached "$old" -- "$@" "${doc_only_excludes[@]}" ;;
        *)        ! git -C "$root" diff --quiet "$old" "$new" -- "$@" "${doc_only_excludes[@]}" ;;
    esac
}

# Print the firmware version as "M.m.p-rcN", or "M.m.p" when kFirmwareVersionReleaseCandidate is 0.
# Usage: version_string <source> <version-file>
version_string() {
    local source="$1" file="$2" major minor patch rc
    major=$(extract "$source" "$file" kFirmwareVersionMajor)
    minor=$(extract "$source" "$file" kFirmwareVersionMinor)
    patch=$(extract "$source" "$file" kFirmwareVersionPatch)
    rc=$(extract "$source" "$file" kFirmwareVersionReleaseCandidate)
    if [ "$rc" -eq 0 ]; then
        echo "$major.$minor.$patch"
    else
        echo "$major.$minor.$patch-rc$rc"
    fi
}

# Print a sort key "major minor patch rc"; stable releases get rc=999999 to sort after their RCs.
version_key() {
    local version="$1" rc=999999
    if [[ "$version" =~ ^([0-9]+)\.([0-9]+)\.([0-9]+)(-rc([0-9]+))?$ ]]; then
        [ -n "${BASH_REMATCH[5]}" ] && rc="${BASH_REMATCH[5]}"
        echo "${BASH_REMATCH[1]} ${BASH_REMATCH[2]} ${BASH_REMATCH[3]} $rc"
    else
        echo "ERROR: can't parse version '$version'" >&2
        return 1
    fi
}

# True (exit 0) if version $1 is lower than version $2.
version_lt() {
    local a b i
    read -ra a <<< "$(version_key "$1")"
    read -ra b <<< "$(version_key "$2")"
    for i in 0 1 2 3; do
        if [ "$((10#${a[i]}))" -lt "$((10#${b[i]}))" ]; then return 0; fi
        if [ "$((10#${a[i]}))" -gt "$((10#${b[i]}))" ]; then return 1; fi
    done
    return 1
}

# Print a product's released versions (local tags <product>-M.m.p[-rcN]), lowest first.
released_versions() {
    local product="$1" tag version
    git -C "$root" tag -l "$product-*" | while read -r tag; do
        version="${tag#"$product"-}"
        if [[ "$version" =~ ^[0-9]+\.[0-9]+\.[0-9]+(-rc[0-9]+)?$ ]]; then
            echo "$(version_key "$version") $version"
        fi
    done | sort -n -k1,1 -k2,2 -k3,3 -k4,4 | awk '{print $5}'
}

# Print the next RC after a released version (rc1 of the next patch after a stable release).
next_rc() {
    local version="$1"
    [[ "$version" =~ ^([0-9]+)\.([0-9]+)\.([0-9]+)(-rc([0-9]+))?$ ]]
    if [ -n "${BASH_REMATCH[5]}" ]; then
        echo "${BASH_REMATCH[1]}.${BASH_REMATCH[2]}.${BASH_REMATCH[3]}-rc$((BASH_REMATCH[5] + 1))"
    else
        echo "${BASH_REMATCH[1]}.${BASH_REMATCH[2]}.$((BASH_REMATCH[3] + 1))-rc1"
    fi
}

# True if the new source is a revision contained in the tag (e.g. CI re-running a released commit).
new_source_in_tag() {
    local tag="$1"
    case "$new_source" in
        WORKTREE|INDEX) return 1 ;;
        *) git -C "$root" merge-base --is-ancestor "$new_source" "$tag" 2>/dev/null ;;
    esac
}

# Usage: check_product <name> <settings-file> <version-file> <watched-path>...
#   A watched path may be an exclude pathspec (":(exclude)<path>").
check_product() {
    local product="$1" settings_file="$2" firmware_version_file="$3"
    shift 3
    local watched_paths=("$@")

    echo "--- $product ---"

    if ! exists_at "$old_source" "$firmware_version_file" || ! exists_at "$old_source" "$settings_file"; then
        echo "$product does not exist at $old_source (new product), skipping."
        return 0
    fi

    # Declare and assign separately: `local x=$(...)` would hide read failures from `set -e`.
    local settings_old settings_new version_old version_new
    settings_old=$(extract "$old_source" "$settings_file" kSettingsVersion)
    settings_new=$(extract "$new_source" "$settings_file" kSettingsVersion)
    version_old=$(version_string "$old_source" "$firmware_version_file")
    version_new=$(version_string "$new_source" "$firmware_version_file")

    local settings_changed=false code_changed=false
    [ "$settings_old" != "$settings_new" ] && settings_changed=true
    paths_changed "$old_source" "$new_source" "${watched_paths[@]}" && code_changed=true

    if ! $settings_changed && ! $code_changed; then
        echo "$product: no firmware changes (excluding *.md); version $version_new."
        echo "$product version sync OK."
        return 0
    fi

    local what="firmware paths (${watched_paths[*]}, excluding *.md)"
    $settings_changed && what="kSettingsVersion ($settings_old -> $settings_new)"
    $settings_changed && $code_changed && what="kSettingsVersion ($settings_old -> $settings_new) and firmware paths"

    local released
    released=$(released_versions "$product")

    if [ -z "$released" ]; then
        # No tags: compare against the old source instead.
        echo "WARNING: no $product-* release tags found in this repository. Run \`git fetch --tags\`"
        echo "         so this check can see which versions are released. Falling back to comparing"
        echo "         the firmware version against $old_source."
        if [ "$version_old" = "$version_new" ]; then
            echo "ERROR: $product $what changed but the firmware version in"
            echo "       $firmware_version_file is still $version_new, the same as at $old_source."
            if $settings_changed; then
                echo "       A Settings struct change needs a firmware version the devices haven't seen,"
                echo "       or they won't reflash and will run with a mismatched Settings struct."
            fi
            echo "Fix: fetch the release tags, or bump kFirmwareVersionReleaseCandidate (dev builds) or"
            echo "     kFirmwareVersionPatch (releases) and commit it together with the change."
            failures=$((failures + 1))
            return 0
        fi
        echo "$product firmware version changed ($version_old -> $version_new)."
        echo "$product version sync OK."
        return 0
    fi

    local latest suggestion
    latest=$(tail -n 1 <<< "$released")
    suggestion=$(next_rc "$latest")

    if grep -qxF "$version_new" <<< "$released"; then
        local tag="$product-$version_new"
        if new_source_in_tag "$tag"; then
            echo "$product: $new_source is part of release $tag."
            echo "$product version sync OK."
            return 0
        fi
        echo "ERROR: $product $what changed, but the firmware version in"
        echo "       $firmware_version_file is $version_new, which is already released"
        echo "       as $tag. Devices that have $tag won't reflash onto this build"
        if $settings_changed; then
            echo "       and will run with a mismatched Settings struct."
        else
            echo "       and will keep running the released one."
        fi
        echo "Fix: set the version to the next unreleased RC, $product-$suggestion (latest release:"
        echo "     $product-$latest), and commit it together with the change."
        failures=$((failures + 1))
        return 0
    fi

    if version_lt "$version_new" "$latest"; then
        echo "ERROR: $product $what changed, and the firmware version in"
        echo "       $firmware_version_file is $version_new, lower than the latest release"
        echo "       $product-$latest. Releasing it would downgrade devices or conflict with"
        echo "       existing releases."
        echo "Fix: set the version to the next unreleased RC, $product-$suggestion, and commit it"
        echo "     together with the change."
        failures=$((failures + 1))
        return 0
    fi

    if [ "$version_old" = "$version_new" ]; then
        echo "$product: $what changed; version $version_new is unchanged from $old_source and unreleased"
        echo "(latest release: $product-$latest). This change ships in the $version_new release."
    else
        echo "$product: $what changed; version $version_old -> $version_new, unreleased"
        echo "(latest release: $product-$latest)."
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
    "firmware/common" \
    ":(exclude)firmware/common/coprocessor/object_dictionary.cpp"

if [ "$failures" -ne 0 ]; then
    echo "=== Version sync check FAILED for $failures product(s) (see above) ==="
    exit 1
fi

echo "=== Version sync check passed ==="
