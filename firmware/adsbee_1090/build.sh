#!/bin/bash
# ADSBee firmware build script.
# Builds all three firmware targets (ESP32, TI CC1312, RP2040 Pico) using Docker containers.
# Usage: ./build.sh [-d] [target] [test_filter|port]
#   targets: all (default), esp, ti, pico, test, build_and_flash, flash, clean
#   -d: build in Debug mode instead of Release
#   test_filter: optional regex passed to ctest -R when target is "test" (e.g. "AircraftJSON")
#   port: optional CDC node to flash, for "build_and_flash" / "flash" (e.g. /dev/cu.usbmodem21201)

set -e

script_dir="$(cd "$(dirname "$0")" && pwd)"
cd "$script_dir"

# find_rpi_drive / list_serial_nodes / wait_for_rpi_drive / copy_uf2_and_confirm.
source ../scripts/uf2_flash_lib.sh

# Number of parallel build jobs.
jobs=$(nproc 2>/dev/null || echo 4)
required_esp_idf_version="v5.5.2"

debug=false
if [ "$1" = "-d" ]; then
    debug=true
    shift
fi
test_filter="${2:-}"
flash_port="${2:-}"

# Verify the Settings/firmware version sync rule (see firmware/scripts/check_version_sync.sh).
# Compares the committed state (HEAD) against the working tree. Advisory only: a failure warns
# and the build continues. The pre-commit hook installed by scripts/setup_dev.sh is the hard gate.
# `set -e` does not apply to a command in an `if` condition, so this cannot abort the build.
check_version_sync() {
    if ! "$script_dir/../scripts/check_version_sync.sh" HEAD WORKTREE; then
        echo ""
        echo "WARNING: version sync check failed (see above). Continuing with the build anyway."
        echo "         Coprocessors are only reflashed on a version mismatch, so a device flashed"
        echo "         with this build may keep running stale coprocessor firmware."
        echo ""
    fi
}

check_esp_idf_version() {
    echo "=== Checking ESP-IDF version (required: $required_esp_idf_version) ==="
    local idf_version
    idf_version=$(docker compose run --rm esp-idf bash -c "idf.py --version")
    echo "ESP-IDF reported by container: $idf_version"
    if [[ "$idf_version" != *"$required_esp_idf_version"* ]]; then
        echo "ERROR: ESP-IDF version mismatch. Expected $required_esp_idf_version."
        echo "Update compose.yml to pin the esp-idf image tag to $required_esp_idf_version."
        exit 1
    fi
}

build_esp() {
    check_esp_idf_version
    if [ "$debug" = true ]; then
        echo "=== Building ESP32-S3 firmware (Debug) ==="
        # sdkconfig_debug is auto-generated on first run by layering sdkconfig.debug on top of sdkconfig.
        # Delete esp/sdkconfig_debug to force regeneration (e.g. after base sdkconfig changes).
        docker compose run --rm esp-idf bash -c "
            cd /firmware/adsbee_1090/esp &&
            idf.py -B build/Debug -D CMAKE_BUILD_TYPE=Debug -D SDKCONFIG=\"\$(pwd)/sdkconfig_debug\" -D \"SDKCONFIG_DEFAULTS=\$(pwd)/sdkconfig;\$(pwd)/sdkconfig.debug\" build
        "
        echo "=== ESP32-S3 build complete (Debug): esp/build/Debug/adsbee_esp.bin ==="
    else
        echo "=== Building ESP32-S3 firmware ==="
        docker compose run --rm esp-idf bash -c "
            cd /firmware/adsbee_1090/esp &&
            idf.py -B build/Release build
        "
        echo "=== ESP32-S3 build complete: esp/build/Release/adsbee_esp.bin ==="
    fi
}

build_ti() {
    local build_type=$( [ "$debug" = true ] && echo "Debug" || echo "Release" )
    echo "=== Building TI CC1312 firmware ($build_type) ==="
    docker compose run --rm ti-lpf2 bash -c "
        cd /firmware/adsbee_1090/ti/sub_ghz_radio &&
        mkdir -p build && cd build &&
        cmake -DCMAKE_BUILD_TYPE=$build_type \
              -DCMAKE_C_COMPILER=/usr/bin/arm-none-eabi-gcc \
              -DCMAKE_CXX_COMPILER=/usr/bin/arm-none-eabi-g++ .. &&
        cmake --build . --config $build_type --target all -j $jobs
    "
    echo "=== TI CC1312 build complete: ti/sub_ghz_radio/build/sub_ghz_radio.bin ==="
}

build_pico() {
    local build_type=$( [ "$debug" = true ] && echo "Debug" || echo "Release" )
    echo "=== Building RP2040 Pico firmware ($build_type) ==="
    # Check that ESP32 and TI firmware exist.
    local esp_build_dir=$( [ "$debug" = true ] && echo "esp/build/Debug" || echo "esp/build/Release" )
    if [ ! -f $esp_build_dir/adsbee_esp.bin ]; then
        echo "ERROR: $esp_build_dir/adsbee_esp.bin not found. Run ESP32 build first."
        exit 1
    fi
    if [ ! -f ti/sub_ghz_radio/build/sub_ghz_radio.bin ]; then
        echo "ERROR: ti/sub_ghz_radio/build/sub_ghz_radio.bin not found. Run TI build first."
        exit 1
    fi
    docker compose run --rm pico-docker bash -c "
        cd /firmware/adsbee_1090/pico &&
        mkdir -p build/$build_type && cd build/$build_type &&
        cmake -DCMAKE_BUILD_TYPE=$build_type \
              -DCMAKE_C_COMPILER=/usr/bin/arm-none-eabi-gcc \
              -DCMAKE_CXX_COMPILER=/usr/bin/arm-none-eabi-g++ ../.. &&
        cmake --build . --config $build_type --target all -j $jobs
    "
    echo "=== RP2040 Pico build complete ==="
    echo "  Firmware: pico/build/$build_type/application/combined.uf2"
    echo "  OTA:      pico/build/$build_type/application/adsbee_1090.ota"
}

build_test() {
    local filter="${1:-}"
    local ctest_filter_opt=""
    if [ -n "$filter" ]; then
        ctest_filter_opt="-R $filter"
        echo "=== Building and running host tests (filter: $filter) ==="
    else
        echo "=== Building and running host tests ==="
    fi
    docker compose run --rm pico-docker bash -c "
        cd /firmware/modules/googletest &&
        mkdir -p build && cd build &&
        cmake -DBUILD_SHARED_LIBS=ON .. &&
        make -j $jobs &&
        cd /firmware/adsbee_1090/pico &&
        mkdir -p build/Test && cd build/Test &&
        cmake -DCMAKE_BUILD_TYPE=Test \
              -DCMAKE_C_COMPILER=/usr/bin/gcc \
              -DCMAKE_CXX_COMPILER=/usr/bin/g++ ../.. &&
        make -j $jobs &&
        ctest --verbose $ctest_filter_opt
    "
    echo "=== Host tests complete ==="
}

# Firmware version as AT+DEVICE_INFO? prints it, e.g. "0.10.0-rc3" (or "0.10.0" for a release).
# Same fields firmware/scripts/check_version_sync.sh watches.
expected_firmware_version() {
    local f="$script_dir/../common/coprocessor/object_dictionary.cpp"
    local field major minor patch rc
    for field in Major Minor Patch ReleaseCandidate; do
        local value
        value="$(sed -n "s/.*kFirmwareVersion${field} *= *\([0-9][0-9]*\).*/\1/p" "$f" | head -n 1)"
        case "$field" in
            Major) major="$value" ;;
            Minor) minor="$value" ;;
            Patch) patch="$value" ;;
            ReleaseCandidate) rc="$value" ;;
        esac
    done
    if [ -z "$major" ] || [ -z "$minor" ] || [ -z "$patch" ] || [ -z "$rc" ]; then
        echo "ERROR: could not parse the firmware version from ${f}." >&2
        return 1
    fi
    if [ "$rc" = "0" ]; then
        echo "${major}.${minor}.${patch}"
    else
        echo "${major}.${minor}.${patch}-rc${rc}"
    fi
}

# True if a CDC node exists and can actually be opened. stty opens with O_NONBLOCK, so this
# answers instantly instead of blocking in open() -- which matters on macOS, where opening
# /dev/cu.usbmodemX blocks for as long as another program holds the matching /dev/tty.usbmodemX
# (a VS Code serial monitor, a Web Serial page, screen, ...).
port_is_available() {
    local port="$1"
    [ -e "$port" ] || return 1
    stty -f "$port" -a >/dev/null 2>&1 || stty -F "$port" -a >/dev/null 2>&1
}

# Puts an already-open CDC node into raw mode. Without this the default ICRNL turns the CR of
# every CRLF the device sends into a second newline, so every line arrives doubled.
#
# No speed operand is ever passed here, and none may be added: the 1090 builds with
# PICO_STDIO_USB_RESET_MAGIC_BAUD_RATE=0xDEADBEE (pico/CMakeLists.txt), so setting a baud rate
# can reboot the device into the bootloader. Call this only while something holds the port open,
# otherwise the settings are dropped when stty closes its own descriptor.
set_raw_mode() {
    local port="$1"
    stty -f "$port" raw -echo 2>/dev/null || stty -F "$port" raw -echo 2>/dev/null || true
}

# Sends one AT command to a CDC node and prints whatever the device replies within <timeout_s>.
# Returns non-zero if the port could not be opened or had to be killed.
#
# The exchange runs in a background subshell with a hard kill, so a port that blocks in open()
# can never wedge the build.
at_query() {
    local port="$1" cmd="$2" timeout="${3:-2}"
    local out pid i finished=0 status=0
    port_is_available "$port" || return 1
    out="$(mktemp)"
    (
        exec 3<>"$port" 2>/dev/null || exit 1
        set_raw_mode "$port"
        printf '%s\r\n' "$cmd" >&3 || true  # A rebooting device can drop the port mid-write.
        local line deadline
        deadline=$(( $(date +%s) + timeout ))
        while [ "$(date +%s)" -lt "$deadline" ]; do
            if IFS= read -r -t 1 line <&3; then
                printf '%s\n' "${line%$'\r'}"  # AT replies are CRLF-terminated.
            elif [ ! -e "$port" ]; then
                break  # Device re-enumerated (e.g. it just rebooted into the bootloader).
            fi
        done
        exit 0
    ) >"$out" 2>/dev/null &
    pid=$!
    for i in $(seq 1 $((timeout + 2))); do
        if ! kill -0 "$pid" 2>/dev/null; then
            finished=1
            break
        fi
        sleep 1
    done
    if [ "$finished" -eq 1 ]; then
        wait "$pid" 2>/dev/null || status=$?
    else
        kill "$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
        status=1
    fi
    cat "$out"
    rm -f "$out"
    return "$status"
}

# Best-effort USB product string for a CDC node; prints nothing if it can't be determined.
# Used only to leave an ADSBee 1421 programmer jig alone -- probing one would pulse the attached
# m1421's reset line. Any platform where this comes up empty just falls through to probing.
usb_product_for_node() {
    local port="$1"
    case "$(uname -s)" in
        Darwin)
            # In the USB device tree the closest preceding "USB Product Name" owns the callout node.
            ioreg -r -c IOUSBHostDevice -l -d 12 2>/dev/null | awk -v want="$port" '
                /"USB Product Name"/     { p = $0; sub(/^[^=]*= /, "", p); gsub(/"/, "", p) }
                /"IOCalloutDevice"/      { d = $0; sub(/^[^=]*= /, "", d); gsub(/"/, "", d)
                                           if (d == want) { print p; exit } }' || true
            ;;
        Linux)
            local product="/sys/class/tty/$(basename "$port")/device/../product"
            if [ -r "$product" ]; then cat "$product"; fi
            ;;
    esac
}

# Prints the CDC nodes that answer AT+DEVICE_INFO? like an ADSBee 1090, one per line.
find_1090_nodes() {
    local node prod reply version
    for node in $(list_serial_nodes); do
        prod="$(usb_product_for_node "$node")"
        case "$prod" in
            *1421*)
                echo "  ${node}: ${prod} -- skipping." >&2
                continue
                ;;
        esac
        if ! port_is_available "$node"; then
            echo "  ${node}: in use by another program -- skipping." >&2
            echo "           (close any serial monitor holding it, e.g. VS Code or a Web Serial page)" >&2
            continue
        fi
        if ! reply="$(at_query "$node" "AT+DEVICE_INFO?" 2)"; then
            echo "  ${node}: no reply -- skipping." >&2
            continue
        fi
        # "RP2040 Flash Unique ID" is the reliable marker: the part code is blank on the
        # non-EEPROM boards (1090U, m1090), and no 1421 prints this line.
        if printf '%s\n' "$reply" | grep -q "RP2040 Flash Unique ID"; then
            version="$(printf '%s\n' "$reply" | sed -n 's/.*RP2040 Firmware Version: *\([0-9.rc-]*\).*/\1/p' | head -n 1)"
            echo "  ${node}: ADSBee 1090 running ${version:-unknown}." >&2
            echo "$node"
        else
            echo "  ${node}: no ADSBee 1090 response -- skipping." >&2
        fi
    done
}

# Path to the RP2040 image the flash commands push. Config-dependent, so both flash_1090 and the
# staleness check ask for it rather than each spelling out the Debug/Release branch.
pico_uf2_path() {
    local build_type=$( [ "$debug" = true ] && echo "Debug" || echo "Release" )
    echo "pico/build/${build_type}/application/combined.uf2"
}

flash_1090() {
    local explicit_port="${1:-}"
    local uf2
    uf2="$(pico_uf2_path)"
    if [ ! -f "$uf2" ]; then
        echo "ERROR: ${uf2} not found." >&2
        echo "       Run './build.sh build_and_flash' to build it and flash in one step." >&2
        exit 1
    fi

    echo ""
    echo "=== Flash an attached ADSBee 1090 / 1090U ==="

    # Step A: find the target. A device already sitting in the bootloader short-circuits everything.
    local port="" drive=""
    if drive="$(find_rpi_drive)"; then
        echo "A device is already in the UF2 bootloader at ${drive}."
    elif [ -n "$explicit_port" ]; then
        if [ ! -e "$explicit_port" ]; then
            echo "ERROR: ${explicit_port} does not exist." >&2
            exit 1
        fi
        port="$explicit_port"
    else
        echo "Looking for an attached ADSBee 1090 ..."
        local node matches=()
        while IFS= read -r node; do
            if [ -n "$node" ]; then matches+=("$node"); fi
        done < <(find_1090_nodes)

        if [ "${#matches[@]}" -gt 1 ]; then
            echo "" >&2
            echo "ERROR: more than one ADSBee 1090 is attached:" >&2
            printf '  %s\n' "${matches[@]}" >&2
            echo "Re-run with the one you want, e.g. ./build.sh flash ${matches[0]}" >&2
            exit 1
        elif [ "${#matches[@]}" -eq 1 ]; then
            port="${matches[0]}"
        else
            # Nothing answered. Fall back to a manually bootloaded device.
            echo ""
            echo "No running ADSBee 1090 found. Put one into its UF2 bootloader by hand:"
            echo "  hold BOOTSEL while plugging in the USB cable."
            echo "Waiting up to 120 s for the RPI-RP2 drive to appear (Ctrl-C to abort) ..."
            if ! drive="$(wait_for_rpi_drive 120)"; then
                echo "ERROR: RPI-RP2 drive never appeared." >&2
                exit 1
            fi
        fi
    fi

    # Step B: reboot into the bootloader. No button press needed -- the firmware exposes
    # AT+BOOT_USB_UF2, which calls rom_reset_usb_boot() (see ci/test_usb_and_ota_flash/).
    if [ -z "$drive" ]; then
        echo "Rebooting ${port} into the RP2040 USB bootloader ..."
        # The device reboots the instant it parses this, so any reply is incidental; at_query is
        # used purely because it bounds the write against a port that blocks in open().
        if ! at_query "$port" "AT+BOOT_USB_UF2=1DEADBEE" 2 >/dev/null; then
            echo "ERROR: could not write to ${port}." >&2
            echo "       Close any serial monitor holding the port and try again." >&2
            exit 1
        fi
        if ! drive="$(wait_for_rpi_drive 60)"; then
            echo "ERROR: RPI-RP2 drive never appeared after AT+BOOT_USB_UF2." >&2
            echo "       Hold BOOTSEL while re-plugging the device and try again." >&2
            exit 1
        fi
    fi

    # Snapshot serial nodes now, while the device is in the bootloader and its CDC node is absent:
    # whatever appears after the copy is the rebooted device.
    local nodes_before
    nodes_before="$(list_serial_nodes)"

    # Step C: copy.
    if ! copy_uf2_and_confirm "$uf2" "$drive"; then
        echo "ERROR: RPI-RP2 drive is still mounted -- the uf2 was not accepted." >&2
        echo "       Eject the drive, re-enter the bootloader, and try again." >&2
        exit 1
    fi
    echo "uf2 accepted; ADSBee rebooting."
    echo "The RP2040 now reflashes the ESP32 and CC1312 if their firmware versions differ."

    # Step D: monitor, then verify. Wait for the device's CDC node to come back.
    local new_port="" nodes_after i
    for i in $(seq 1 60); do
        sleep 1
        if [ -n "$port" ] && [ -e "$port" ]; then
            new_port="$port"
            break
        fi
        nodes_after="$(list_serial_nodes)"
        new_port="$(comm -13 <(sort <<<"$nodes_before") <(sort <<<"$nodes_after") | grep -v '^$' | head -n 1 || true)"
        if [ -n "$new_port" ]; then
            break
        fi
    done
    if [ -z "$new_port" ]; then
        echo ""
        echo "Could not identify the ADSBee's serial port; skipping verification."
        echo "The RP2040 is running the new firmware. Open its console to confirm the"
        echo "ESP32 and CC1312 updates finished."
        return 0
    fi

    echo ""
    local result=""
    if ! port_is_available "$new_port"; then
        echo "${new_port} is in use by another program; skipping the console monitor."
    else
        echo "Monitoring ${new_port} (up to 180 s) ..."
        # A background `cat` owns the port and pipes it through a fifo, so an open() that blocks
        # can never wedge the build -- the deadline below applies regardless. The fifo is opened
        # read-write so opening it never blocks and reads never hit EOF; the read timeout is what
        # ends the loop. Read-only: nothing is written to the device here.
        #
        # Note that CONSOLE_INFO lines never reach the console at the default log level
        # (kWarnings), so only unconditional prints and CONSOLE_ERROR lines are matched; the
        # authoritative check is the AT+DEVICE_INFO? query below. Log macros wrap the message
        # text in ANSI colour, so only message substrings are matched, never "tag: message" spans.
        local fifo cat_pid line deadline quiet=0
        fifo="$(mktemp -u)"
        mkfifo "$fifo"
        exec 3<>"$fifo"
        cat "$new_port" > "$fifo" 2>/dev/null &
        cat_pid=$!
        sleep 1  # Let cat open the port so the raw-mode settings below stick.
        set_raw_mode "$new_port"
        deadline=$(( $(date +%s) + 180 ))
        while [ "$(date +%s)" -lt "$deadline" ]; do
            if IFS= read -r -t 5 line <&3; then
                quiet=0
                printf '%s\n' "${line%$'\r'}"  # Console lines are CRLF-terminated.
                case "$line" in
                    *"Error while flashing ESP32"*)                   result="fail"; break ;;
                    *"Application is not up to date after flashing"*) result="fail"; break ;;
                esac
            else
                # Nothing for 15 s: either it booted before we attached, or the flashing is done.
                quiet=$((quiet + 1))
                if [ "$quiet" -ge 3 ]; then break; fi
            fi
        done
        exec 3>&-
        kill "$cat_pid" 2>/dev/null || true
        wait "$cat_pid" 2>/dev/null || true
        rm -f "$fifo"
    fi

    if [ "$result" = "fail" ]; then
        echo "" >&2
        echo "ERROR: the device reported a coprocessor flash failure (see the console output above)." >&2
        exit 1
    fi

    local expected reply rp_ver="" esp_ver=""
    expected="$(expected_firmware_version)"
    echo ""
    if ! port_is_available "$new_port"; then
        # The uf2 was accepted, so the flash itself succeeded -- only the check is unavailable.
        echo "${new_port} is in use by another program, so the firmware version could not be checked."
        echo "Close the program holding it and run 'AT+DEVICE_INFO?' to confirm ${expected}."
        echo ""
        echo "=== Flash complete (unverified) ==="
        return 0
    fi
    echo "Verifying with AT+DEVICE_INFO? (expecting ${expected}) ..."
    for i in 1 2 3; do
        reply="$(at_query "$new_port" "AT+DEVICE_INFO?" 3 || true)"
        rp_ver="$(printf '%s\n' "$reply" | sed -n 's/.*RP2040 Firmware Version: *\([0-9.rc-]*\).*/\1/p' | head -n 1)"
        esp_ver="$(printf '%s\n' "$reply" | sed -n 's/.*ESP32 Firmware Version: *\([0-9.rc-]*\).*/\1/p' | head -n 1)"
        if [ -n "$rp_ver" ]; then
            break
        fi
        sleep 2
    done

    if [ -z "$rp_ver" ]; then
        echo "ERROR: no response to AT+DEVICE_INFO? on ${new_port}." >&2
        exit 1
    fi
    if [ "$rp_ver" != "$expected" ]; then
        echo "ERROR: RP2040 is running ${rp_ver}, expected ${expected}." >&2
        # ${expected} comes from object_dictionary.cpp (source), not from the image just flashed, so
        # the usual cause here is flashing a uf2 that predates the current tree.
        echo "       The expected version is read from the source tree, so this usually means the" >&2
        echo "       uf2 that was flashed is stale. Run './build.sh build_and_flash' to rebuild." >&2
        exit 1
    fi
    echo "  RP2040: ${rp_ver}"
    if [ -z "$esp_ver" ]; then
        echo "  ESP32:  not enabled on this board."
    elif [ "$esp_ver" != "$expected" ]; then
        echo "ERROR: ESP32 is running ${esp_ver}, expected ${expected}." >&2
        echo "       Coprocessors are only reflashed on a version mismatch; check the console output above." >&2
        echo "       If the uf2 was stale, './build.sh build_and_flash' rebuilds and reflashes." >&2
        exit 1
    else
        echo "  ESP32:  ${esp_ver}"
    fi

    echo ""
    echo "=== Flash complete: ADSBee 1090 running ${expected} ==="
}

# Rejects a bad second positional for the flash commands: it may only be an existing CDC node.
validate_flash_port() {
    local cmd="$1"
    if [ -n "$flash_port" ] && [ "${flash_port#/dev/}" = "$flash_port" ]; then
        echo "ERROR: '$flash_port' is not a serial port. '$cmd' takes only an optional CDC node," >&2
        echo "       e.g. ./build.sh $cmd /dev/cu.usbmodem21201" >&2
        exit 1
    fi
    if [ -n "$flash_port" ] && [ ! -e "$flash_port" ]; then
        echo "ERROR: $flash_port does not exist. Attached CDC nodes:" >&2
        list_serial_nodes >&2
        exit 1
    fi
}

clean_builds() {
    echo "=== Cleaning build directories ==="
    rm -rf esp/build
    rm -rf esp/sdkconfig_debug
    rm -rf ti/sub_ghz_radio/build
    rm -rf pico/build
    echo "=== Clean complete ==="
}

target="${1:-all}"

if [ "$target" != "clean" ]; then
    check_version_sync
fi

case "$target" in
    esp)
        build_esp
        ;;
    ti)
        build_ti
        ;;
    pico)
        build_pico
        ;;
    test)
        build_test "$test_filter"
        ;;
    build_and_flash)
        # Validate the port before the builds so a typo does not cost a full rebuild.
        validate_flash_port "$target"
        build_esp
        build_ti
        build_pico
        flash_1090 "$flash_port"
        ;;
    flash)
        validate_flash_port "$target"
        # Flash-only: warn if the image no longer matches the tree, then push it anyway. Scoped to
        # the source trees -- ti/setup holds a ~2 GB vendored SDK and must never be scanned.
        warn_if_artifact_stale "$(pico_uf2_path)" "./build.sh build_and_flash" \
            ../common ../modules pico/application esp/main ti/sub_ghz_radio
        flash_1090 "$flash_port"
        ;;
    clean)
        clean_builds
        ;;
    all)
        build_type=$( [ "$debug" = true ] && echo "Debug" || echo "Release" )
        build_esp
        build_ti
        build_pico
        echo ""
        echo "=== Full build complete! ==="
        echo "  Output: firmware/pico/build/$build_type/application/combined.uf2"
        ;;
    *)
        echo "Usage: $0 [-d] [esp|ti|pico|test|build_and_flash|flash|clean|all]"
        echo "  -d    - Build in Debug mode instead of Release"
        echo "  all   - Build all firmware targets (default)"
        echo "  esp   - Build ESP32-S3 firmware only"
        echo "  ti    - Build TI CC1312 firmware only"
        echo "  pico  - Build RP2040 Pico firmware only (requires esp + ti first)"
        echo "  test [filter] - Build and run host unit tests; optional filter is a ctest -R regex (e.g. \"AircraftJSON\")"
        echo "  build_and_flash [port] - Build all targets, then reflash an attached ADSBee 1090/1090U"
        echo "                  over USB. Finds the device itself and reboots it into the UF2 bootloader"
        echo "                  via AT+BOOT_USB_UF2 (no BOOTSEL press), copies combined.uf2, then verifies"
        echo "                  the RP2040 and ESP32 firmware versions. Pass a CDC node (e.g."
        echo "                  /dev/cu.usbmodem21201) to pick between multiple attached devices."
        echo "  flash [port]  - Reflash using the combined.uf2 that is already built, skipping every"
        echo "                  build step. Same device handling as build_and_flash; warns first if the"
        echo "                  image is older than the source tree."
        echo "  clean - Remove all build directories"
        exit 1
        ;;
esac
