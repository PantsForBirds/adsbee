set(COMMON_DIR "../../../common")
set(MODULES_DIR "../../../modules")

idf_component_register(
    SRC_DIRS
        "."
        "${MODULES_DIR}/opendroneid-core-c/libopendroneid"  # Broadcast Remote ID decoder (opendroneid.c).
        "${COMMON_DIR}/adsb"
        "${COMMON_DIR}/adsb/nasa_cpr"
        "${COMMON_DIR}/adsb/nasa_cpr/airborne"
        "${COMMON_DIR}/adsb/nasa_cpr/coarse"
        "${COMMON_DIR}/adsb/nasa_cpr/surface"
        "${COMMON_DIR}/adsb/nasa_cpr/intent"
        # "${COMMON_DIR}/comms" # Currently no src files in here.
        "${COMMON_DIR}/coprocessor"
        "${COMMON_DIR}/utils"
        "${COMMON_DIR}/utils/crc"
        "${COMMON_DIR}/utils/fec"
        "${COMMON_DIR}/utils/geo"
        "${COMMON_DIR}/comms"
        "${COMMON_DIR}/comms/beast"
        "${COMMON_DIR}/comms/gdl90"
        "${COMMON_DIR}/comms/json"
        "${COMMON_DIR}/comms/mavlink"
        "${COMMON_DIR}/comms/raw"
        "${COMMON_DIR}/settings"
        "comms"
        "peripherals/pico"
        "remote_id"
        "server"
        "target_test"
    INCLUDE_DIRS
        "."
        "${COMMON_DIR}"
        "${MODULES_DIR}/opendroneid-core-c/libopendroneid"  # opendroneid.h for Remote ID decode.
        "${COMMON_DIR}/adsb"
        "${COMMON_DIR}/adsb/nasa_cpr"
        "${COMMON_DIR}/comms"
        "${COMMON_DIR}/comms/aircraftjson"
        "${COMMON_DIR}/comms/beast"
        "${COMMON_DIR}/comms/csbee"
        "${COMMON_DIR}/comms/gdl90"
        "${COMMON_DIR}/comms/json"
        "${COMMON_DIR}/comms/mavlink"
        "${COMMON_DIR}/comms/raw"
        "${COMMON_DIR}/coprocessor"
        "${COMMON_DIR}/utils"
        "${COMMON_DIR}/utils/crc"
        "${COMMON_DIR}/utils/fec"
        "${COMMON_DIR}/utils/geo"
        "${COMMON_DIR}/settings"
        "comms"
        "peripherals/pico"
        "remote_id"
        "server"
        "target_test"
    EMBED_TXTFILES
        "server/web/index.html"
        "server/web/style.css"
        "server/web/adsbee.js"
        "server/web/settings.js"
        "server/web/favicon.png"
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-packed-bitfield-compat)

# Drop the Open Drone ID library's printf/debug helpers (we only use the decode path). Saves flash and avoids stdio.
target_compile_definitions(${COMPONENT_LIB} PRIVATE ODID_DISABLE_PRINTF)

message("PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")