#!/bin/bash
# ADSBee 1421 firmware build script.
# Builds an application inside the Docker container that owns its toolchain (defined in
# compose.yml) -- the toolchains only exist inside those containers.

set -euo pipefail

# Always run from the directory containing compose.yml so `docker compose` finds it.
cd "$(dirname "$0")"

# find_rpi_drive / list_serial_nodes / wait_for_rpi_drive / copy_uf2_and_confirm.
source ../scripts/uf2_flash_lib.sh

usage() {
    cat <<'EOF'
Usage: ./build.sh [-d] [clean] [app|build_and_flash|flash]
  app         ti (default, CC1314R10 application via ti-lpf2)
              programmer (RP2040-Zero flash/passthrough jig via pico-docker;
                          requires ti to be built first)
  build_and_flash
              build ti + programmer, then reflash an attached ADSBee m1421 via its
              programmer jig: prompts you to put the jig in bootloader mode, copies
              the fresh uf2 onto the RPI-RP2 drive, and watches the jig's console
              while it automatically flashes the m1421.
  flash       same, but using the programmer uf2 that is already built -- no build
              steps run at all. Warns first if that uf2 is older than the source tree,
              or older than the ti hex baked into it.
  (default)   build in Release
  -d          build in Debug instead of Release
  clean       remove <app>/build and exit
EOF
}

APP="ti"
CONFIG="Release"
DO_CLEAN=0
DO_FLASH=0
DO_BUILD_AND_FLASH=0
APP_EXPLICIT=0

for arg in "$@"; do
    case "$arg" in
        -d) CONFIG="Debug" ;;
        clean) DO_CLEAN=1 ;;
        flash) DO_FLASH=1 ;;
        build_and_flash) DO_BUILD_AND_FLASH=1 ;;
        ti|programmer)
            APP="$arg"
            APP_EXPLICIT=1
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $arg" >&2
            usage >&2
            exit 1
            ;;
    esac
done

if [ "$DO_FLASH" -eq 1 ] && [ "$DO_BUILD_AND_FLASH" -eq 1 ]; then
    echo "'flash' and 'build_and_flash' are alternatives; pass only one." >&2
    exit 1
fi

if { [ "$DO_FLASH" -eq 1 ] || [ "$DO_BUILD_AND_FLASH" -eq 1 ]; } &&
   { [ "$DO_CLEAN" -eq 1 ] || [ "$APP_EXPLICIT" -eq 1 ]; }; then
    echo "The flash commands drive the whole sequence themselves; they cannot be combined with" >&2
    echo "'clean' or an app argument." >&2
    exit 1
fi

if [ "$DO_CLEAN" -eq 1 ]; then
    echo "Removing ${APP}/build ..."
    rm -rf "${APP}/build"
    echo "Clean complete."
    exit 0
fi

# Configure + build one app inside its container and list the artifacts it produced.
build_app() {
    local app="$1"
    local service artifact extra_cmake_args=""
    local -a artifact_exts
    case "$app" in
        ti)
            service="ti-lpf2"
            artifact="adsbee_1421"
            artifact_exts=(elf hex map)
            ;;
        programmer)
            service="pico-docker"
            artifact="adsbee_1421_programmer"
            artifact_exts=(uf2 elf)
            # picotool is not installed in the image, and the Pico SDK builds it from source to make
            # .uf2 files. Cache it outside build/ so `clean` does not force another download+build.
            extra_cmake_args="-DPICOTOOL_FETCH_FROM_GIT_PATH=/firmware/adsbee_1421/.picotool"
            ;;
    esac

    echo "Building ${app} (${CONFIG}) in container ${service} ..."

    # Keep the toolchain container running so incremental rebuilds reuse the CMake cache.
    docker compose up -d "${service}"

    # Configure + build inside the container. The CMake ADSBEE_*_DIR defaults resolve to
    # /firmware/common and /firmware/modules via the compose.yml volume mounts (and to
    # firmware/common and firmware/modules on the host), so no overrides are needed.
    docker compose exec "${service}" bash -c "
        set -e
        cd /firmware/adsbee_1421/${app}
        mkdir -p build/${CONFIG}
        cd build/${CONFIG}
        cmake -DCMAKE_BUILD_TYPE=${CONFIG} \
              -DCMAKE_C_COMPILER=/usr/bin/arm-none-eabi-gcc \
              -DCMAKE_CXX_COMPILER=/usr/bin/arm-none-eabi-g++ ${extra_cmake_args} ../..
        cmake --build . --config ${CONFIG} --target all -j \$(nproc)
    "

    echo ""
    echo "Build complete. Artifacts:"
    local out="${app}/build/${CONFIG}"
    local ext f
    # Both the stable names (adsbee_1421.hex) and CMake's version-stamped copies
    # (adsbee_1421-0.3.6.hex) are produced, so glob to list them all.
    for ext in "${artifact_exts[@]}"; do
        for f in "${out}/${artifact}"*".${ext}"; do
            if [ -f "$f" ]; then echo "  $f"; fi
        done
    done
}

# Path to the jig image the flash commands push, and to the CC1314 hex baked into it at CMake
# configure time (see programmer/README.md) -- a rebuilt ti with a stale programmer would flash old
# m1421 firmware even though both files exist.
programmer_uf2_path() { echo "programmer/build/${CONFIG}/adsbee_1421_programmer.uf2"; }
ti_hex_path() { echo "ti/build/${CONFIG}/adsbee_1421.hex"; }

flash_m1421() {
    local uf2
    uf2="$(programmer_uf2_path)"
    if [ ! -f "$uf2" ]; then
        echo "ERROR: ${uf2} not found." >&2
        echo "       Run './build.sh build_and_flash' to build it and flash in one step." >&2
        exit 1
    fi

    echo ""
    echo "=== Reflash an attached ADSBee m1421 via its programmer jig ==="
    echo "Put the ADSBee 1421 programmer (RP2040-Zero) into its UF2 bootloader:"
    echo "  hold BOOT while plugging it in, or hold BOOT and tap its RESET button."
    echo "Waiting up to 120 s for the RPI-RP2 drive to appear (Ctrl-C to abort) ..."

    local drive="" i
    if ! drive="$(wait_for_rpi_drive 120)"; then
        echo "ERROR: RPI-RP2 drive never appeared. Is the jig in bootloader mode?" >&2
        exit 1
    fi

    # Snapshot serial nodes NOW: the jig is in the bootloader, so its CDC node is absent. Whatever
    # node appears after the copy is the rebooted jig -- this works whether the jig was freshly
    # plugged in or was already attached before entering the bootloader.
    local nodes_before
    nodes_before="$(list_serial_nodes)"

    if ! copy_uf2_and_confirm "$uf2" "$drive"; then
        echo "ERROR: RPI-RP2 drive is still mounted -- the uf2 was not accepted." >&2
        echo "       Eject the drive, re-enter the bootloader, and try again." >&2
        exit 1
    fi
    echo "uf2 accepted; programmer rebooting."
    echo "The jig now checks the attached m1421 and flashes it automatically if the firmware differs."

    # Best-effort monitor: find the jig's CDC port (the node that newly appeared vs the snapshot)
    # and watch the flash transcript. Read-only: DTR edges in pass-through mode pulse the target's
    # reset line, so never write or toggle the line after opening.
    local port="" nodes_after node
    for i in $(seq 1 10); do
        sleep 1
        nodes_after="$(list_serial_nodes)"
        port="$(comm -13 <(sort <<<"$nodes_before") <(sort <<<"$nodes_after") | head -n 1)"
        if [ -n "$port" ]; then
            break
        fi
    done
    if [ -z "$port" ]; then
        echo ""
        echo "Could not identify the jig's serial port; monitor skipped."
        echo "Watch the jig's LED instead: cyan = CRC check, magenta blink = flashing the m1421,"
        echo "blue blink = verifying, green = done (pass-through), red = error."
        echo "Or open its CDC port ('ADSBee 1421 Programmer') with any terminal at 1000000 baud."
        return 0
    fi

    echo "Monitoring ${port} (up to 180 s) ..."
    # 1000000 baud matches the jig's console/pass-through rate, so the line-coding event our open
    # generates is harmless even if the jig has already reached pass-through.
    stty -f "$port" 1000000 raw -echo 2>/dev/null || stty -F "$port" 1000000 raw -echo 2>/dev/null || true

    local result="" line deadline
    deadline=$(( $(date +%s) + 180 ))
    exec 3<"$port"
    while [ $(date +%s) -lt "$deadline" ]; do
        if IFS= read -r -t 5 line <&3; then
            printf '%s\n' "$line"
            case "$line" in
                *"Console up at"*)      result="ok"; break ;;
                *"Firmware up to date"*) result="current" ;;  # "Console up" follows shortly; keep reading.
                *"Flash failed"*)       result="fail"; break ;;
            esac
        elif [ "$result" = "current" ]; then
            break  # Up-to-date and gone quiet: good enough.
        fi
    done
    exec 3<&-

    echo ""
    case "$result" in
        ok)
            echo "=== m1421 flash complete; jig is in pass-through (green LED). ==="
            ;;
        current)
            echo "=== m1421 firmware already up to date. ==="
            ;;
        fail)
            echo "ERROR: the jig reported a flash failure (red LED). Check the m1421 connection." >&2
            exit 1
            ;;
        *)
            echo "Monitor timed out without a definitive result. Check the jig's LED:" >&2
            echo "  magenta blink = still flashing, green = done, red = error, yellow blink = no m1421 found." >&2
            exit 1
            ;;
    esac
}

# Verify the Settings/firmware version sync rule (see firmware/scripts/check_version_sync.sh).
# Compares the committed state (HEAD) against the working tree. Advisory only: a failure warns
# and the build continues. The pre-commit hook installed by scripts/setup_dev.sh is the hard gate.
# `set -e` does not apply to a command in an `if` condition, so this cannot abort the build.
if ! "$(pwd)/../scripts/check_version_sync.sh" HEAD WORKTREE; then
    echo ""
    echo "WARNING: version sync check failed (see above). Continuing with the build anyway."
    echo "         Firmware is only reflashed on a version mismatch, so a device flashed with"
    echo "         this build may keep running stale firmware."
    echo ""
fi

if [ "$DO_BUILD_AND_FLASH" -eq 1 ]; then
    # Fresh CC1314 firmware, then a programmer image with it baked in, then flash the jig.
    build_app ti
    build_app programmer
    flash_m1421
    exit 0
fi

if [ "$DO_FLASH" -eq 1 ]; then
    # Flash-only: no container builds at all. Two things can make the jig image stale -- edited
    # sources, or a ti hex rebuilt after the programmer baked its copy in -- so check both.
    warn_if_artifact_stale "$(programmer_uf2_path)" "./build.sh build_and_flash" \
        ../common ../modules ti programmer
    # Both artifacts live under build/, which warn_if_artifact_stale deliberately skips, so compare
    # them directly: a ti hex newer than the jig image means the baked-in copy is out of date.
    if [ -f "$(ti_hex_path)" ] && [ -f "$(programmer_uf2_path)" ] &&
       [ "$(ti_hex_path)" -nt "$(programmer_uf2_path)" ]; then
        echo ""
        echo "WARNING: $(ti_hex_path) is newer than $(programmer_uf2_path)."
        echo "         The jig image bakes in the ti hex at configure time, so this would flash the"
        echo "         OLDER m1421 firmware. Run './build.sh build_and_flash' to rebuild both."
        echo ""
    fi
    flash_m1421
    exit 0
fi

build_app "$APP"
