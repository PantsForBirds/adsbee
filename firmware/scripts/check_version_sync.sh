#!/bin/bash
# Enforce the firmware/AGENTS.md release rule: firmware that changed must never be released under
# a version that is already released. Devices reflash coprocessors only when the reported
# firmware version differs, so two different builds sharing one released version leave devices
# on a stale build (and, after a Settings struct change, on a mismatched Settings struct: SPI
# sync failures, reboots).
#
# Checked per product:
#   adsbee_1090: versions live in firmware/common (settings.hh kSettingsVersion,
#                coprocessor/object_dictionary.cpp kFirmwareVersion*); watched paths are the
#                ESP32/CC1312 coprocessor code and shared common/ code.
#   adsbee_1421: versions live in firmware/adsbee_1421/ti (settings/settings.hh,
#                object_dictionary/object_dictionary.cpp); watched paths are the whole product
#                directory and shared common/ code.
# Note: a firmware/common change therefore concerns BOTH products.
#
# The firmware version is written the way release tags are named: <product>-M.m.p-rcN for a
# release candidate and <product>-M.m.p for a stable release (kFirmwareVersionReleaseCandidate
# = 0), e.g. adsbee_1421-0.3.11-rc2 or adsbee_1090-0.9.0. A version counts as released when a
# local git tag with that name exists. Tags are read from the local repository only (no network
# access), so run `git fetch --tags` first.
#
# If the watched paths or kSettingsVersion changed between the old and the new source, the check
# fails when the NEW firmware version:
#   - is already released (a tag <product>-<version> exists), or
#   - is lower than the highest released version of that product.
# Versions compare by major, minor and patch, then release candidate, with every RC of a
# version ordered below its stable release (0.3.11-rc9 < 0.3.11 < 0.3.12-rc1).
# An unreleased version passes whether or not it equals the old source's version, so several
# changes can accumulate under the next unreleased RC before it is released. Tags that don't
# follow the naming scheme are ignored.
#
# If the product has no release tags at all (e.g. a clone without tags), the check prints a
# WARNING and falls back to the base comparison: watched-path or settings changes fail unless the
# firmware version differs from the old source's.
#
# Documentation-only changes are exempt from the watched-paths check: Markdown files (*.md, any
# case) under the watched paths never feed a build, so editing a README or AGENTS.md doesn't
# require a bump. Nothing else is exempt. In particular *.txt is not (CMakeLists.txt, test
# input data), and neither are images (the ESP32 web server embeds its favicon.png).
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

# Pathspecs excluded from the watched-paths check: documentation that never feeds a build.
# See the header comment before adding anything here.
doc_only_excludes=(':(exclude,glob,icase)**/*.md')

# True (exit 0) if any of the given paths differ between old_source and new_source, ignoring
# documentation-only files (doc_only_excludes).
paths_changed() {
    local old="$1" new="$2"
    shift 2
    case "$new" in
        WORKTREE) ! git -C "$root" diff --quiet "$old" -- "$@" "${doc_only_excludes[@]}" ;;
        INDEX)    ! git -C "$root" diff --quiet --cached "$old" -- "$@" "${doc_only_excludes[@]}" ;;
        *)        ! git -C "$root" diff --quiet "$old" "$new" -- "$@" "${doc_only_excludes[@]}" ;;
    esac
}

# Print a product's firmware version at a source, formatted like a release tag suffix:
# "M.m.p-rcN", or "M.m.p" when kFirmwareVersionReleaseCandidate is 0 (a stable release).
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

# Print a sortable key "major minor patch rc" for a version string, with a stable release's rc
# replaced by a number above any RC so that every M.m.p-rcN sorts below M.m.p.
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

# Print the released versions of a product (the local tags named <product>-M.m.p[-rcN], with
# the product prefix removed), lowest first. Tags that don't follow the scheme are ignored.
released_versions() {
    local product="$1" tag version
    git -C "$root" tag -l "$product-*" | while read -r tag; do
        version="${tag#"$product"-}"
        if [[ "$version" =~ ^[0-9]+\.[0-9]+\.[0-9]+(-rc[0-9]+)?$ ]]; then
            echo "$(version_key "$version") $version"
        fi
    done | sort -n -k1,1 -k2,2 -k3,3 -k4,4 | awk '{print $5}'
}

# Print the version that follows a released version: the next RC of the same version, or the
# first RC of the next patch after a stable release.
next_rc() {
    local version="$1"
    [[ "$version" =~ ^([0-9]+)\.([0-9]+)\.([0-9]+)(-rc([0-9]+))?$ ]]
    if [ -n "${BASH_REMATCH[5]}" ]; then
        echo "${BASH_REMATCH[1]}.${BASH_REMATCH[2]}.${BASH_REMATCH[3]}-rc$((BASH_REMATCH[5] + 1))"
    else
        echo "${BASH_REMATCH[1]}.${BASH_REMATCH[2]}.$((BASH_REMATCH[3] + 1))-rc1"
    fi
}

# True (exit 0) if the new source is a git revision contained in the given tag, i.e. the code
# being checked is part of that release (for example CI re-running on a commit after it was
# released). WORKTREE and INDEX never are.
new_source_in_tag() {
    local tag="$1"
    case "$new_source" in
        WORKTREE|INDEX) return 1 ;;
        *) git -C "$root" merge-base --is-ancestor "$new_source" "$tag" 2>/dev/null ;;
    esac
}

# Check one product: if its watched paths or settings version changed, its new firmware version
# must be unreleased and not lower than its highest release.
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
        # Without tags there is no way to tell what is released. Fall back to comparing against
        # the old source so the check never passes just because the tags are missing.
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
    "firmware/common"

if [ "$failures" -ne 0 ]; then
    echo "=== Version sync check FAILED for $failures product(s) (see above) ==="
    exit 1
fi

echo "=== Version sync check passed ==="
