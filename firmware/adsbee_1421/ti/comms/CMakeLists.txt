target_sources(${PROJECT_NAME} PRIVATE
    "comms.cpp"
    "at/comms_at.cpp"
    ${ADSBEE_COMMON_DIR}/comms/comms_reporting.cpp
    ${ADSBEE_COMMON_DIR}/comms/beast/beast_utils.cpp
    ${ADSBEE_COMMON_DIR}/comms/gdl90/gdl90_utils.cpp
    ${ADSBEE_COMMON_DIR}/comms/mavlink/mavlink_utils.cpp
    ${ADSBEE_COMMON_DIR}/comms/raw/raw_utils.cpp
    ${ADSBEE_COMMON_DIR}/coprocessor/composite_array.cpp

    ${ADSBEE_MODULES_DIR}/cppAT/src/cpp_at.cc
)
target_include_directories(${PROJECT_NAME} PRIVATE
    .
    at
    ${ADSBEE_COMMON_DIR}/comms
    ${ADSBEE_COMMON_DIR}/comms/aircraftjson
    ${ADSBEE_COMMON_DIR}/comms/beast
    ${ADSBEE_COMMON_DIR}/comms/csbee
    ${ADSBEE_COMMON_DIR}/comms/gdl90
    ${ADSBEE_COMMON_DIR}/comms/json
    ${ADSBEE_COMMON_DIR}/comms/raw
    ${ADSBEE_COMMON_DIR}/coprocessor

    ${ADSBEE_MODULES_DIR}/cppAT/src
)
# MAVLink is third-party generated code; use SYSTEM to suppress warnings from its headers.
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE
    ${ADSBEE_COMMON_DIR}/comms/mavlink
)
