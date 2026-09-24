# Vendored Open Drone ID codec, needed by aircraft_dictionary.cpp for Remote ID decode.
set(OPENDRONEID_DIR "${ADSBEE_MODULES_DIR}/opendroneid-core-c/libopendroneid")

target_sources(${PROJECT_NAME} PRIVATE
    ${OPENDRONEID_DIR}/opendroneid.c
    ${ADSBEE_COMMON_DIR}/adsb/mode_s_packet.cpp
    ${ADSBEE_COMMON_DIR}/adsb/uat_packet.cpp
    ${ADSBEE_COMMON_DIR}/adsb/aircraft_dictionary.cpp
    ${ADSBEE_COMMON_DIR}/adsb/decode_utils.cpp
    ${ADSBEE_COMMON_DIR}/adsb/mode_s_packet_decoder.cc

    ${ADSBEE_COMMON_DIR}/adsb/nasa_cpr/airborne/cpr_int.cpp
    ${ADSBEE_COMMON_DIR}/adsb/nasa_cpr/airborne/cpr.cpp
    ${ADSBEE_COMMON_DIR}/adsb/nasa_cpr/airborne/nl_int.cpp
    ${ADSBEE_COMMON_DIR}/adsb/nasa_cpr/nasa_cpr_airborne.cpp
    ${ADSBEE_COMMON_DIR}/adsb/nasa_cpr/nasa_cpr_surface.cpp
    ${ADSBEE_COMMON_DIR}/adsb/nasa_cpr/surface/cpr_int.cpp
    ${ADSBEE_COMMON_DIR}/adsb/nasa_cpr/surface/cpr.cpp
    ${ADSBEE_COMMON_DIR}/adsb/nasa_cpr/surface/nl_int.cpp

    ${ADSBEE_COMMON_DIR}/coprocessor/composite_array.cpp
    ${ADSBEE_COMMON_DIR}/coprocessor/composite_array.hh
)
target_include_directories(${PROJECT_NAME} PRIVATE
    .
    ${ADSBEE_COMMON_DIR}/adsb
    ${ADSBEE_COMMON_DIR}/adsb/nasa_cpr
    ${OPENDRONEID_DIR}
)

