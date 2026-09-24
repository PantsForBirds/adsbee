

target_sources(${PROJECT_NAME} PRIVATE
    led.cpp
    ${ADSBEE_COMMON_DIR}/utils/crc/crc.cpp

    ${ADSBEE_COMMON_DIR}/utils/fec/fec.cpp
    ${ADSBEE_COMMON_DIR}/utils/fec/decode_rs_char.c
    ${ADSBEE_COMMON_DIR}/utils/fec/encode_rs_char.c
    ${ADSBEE_COMMON_DIR}/utils/fec/init_rs_char.c

    ${ADSBEE_COMMON_DIR}/utils/buffer_utils.cpp
    ${ADSBEE_COMMON_DIR}/utils/data_structures.cpp
    ${ADSBEE_COMMON_DIR}/utils/geo/geo_utils.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
    .
    ${ADSBEE_COMMON_DIR}/utils
    ${ADSBEE_COMMON_DIR}/utils/crc
    ${ADSBEE_COMMON_DIR}/utils/fec
    ${ADSBEE_COMMON_DIR}/utils/geo
)
