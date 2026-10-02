# Host unit tests for SDK-independent ADSBee 1421 (CC1314 application) logic, run with googletest like the
# ADSBee 1090 host tests (firmware/adsbee_1090/pico/host_test). Standalone (not part of the TI build). Build googletest
# (firmware/modules/googletest) first, as CI does:
#   cd firmware/modules/googletest && mkdir build && cd build && cmake -DBUILD_SHARED_LIBS=ON .. && make
# Then:
#   cmake -S firmware/adsbee_1421/ti/host_test -B /tmp/adsbee_1421_host_test
#   cmake --build /tmp/adsbee_1421_host_test && ctest --test-dir /tmp/adsbee_1421_host_test --output-on-failure
cmake_minimum_required(VERSION 3.13)
project(adsbee_1421_host_test CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Debug")

set(ADSBEE_1421_TI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(ADSBEE_COMMON_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../common")
set(ADSBEE_MODULES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules")

find_package(Threads REQUIRED)

add_compile_definitions(ON_HOST)
add_compile_options(
    -Wall                # Include all warnings.
    -Wextra
    -Werror
    -ftrapv # Trap on signed integer overflow (undefined behavior).
)

add_executable(host_test main.cc)

# Test: Pull in google test library
add_library(libgtest SHARED IMPORTED)
set_target_properties(libgtest PROPERTIES IMPORTED_LOCATION ${ADSBEE_MODULES_DIR}/googletest/build/lib/libgtest.so)
target_link_libraries(host_test PRIVATE libgtest Threads::Threads)

target_sources(host_test PRIVATE
    ${ADSBEE_1421_TI_DIR}/packet_decoder.cpp  # DecodeOne() only: Update() needs the SDK (left out ON_HOST).
    ${ADSBEE_1421_TI_DIR}/settings/settings_strs.cpp
    ${ADSBEE_COMMON_DIR}/adsb/decode_utils.cpp
    ${ADSBEE_COMMON_DIR}/adsb/mode_s_packet.cpp
    ${ADSBEE_COMMON_DIR}/utils/buffer_utils.cpp
    ${ADSBEE_COMMON_DIR}/utils/crc/crc.cpp  # crc32_ieee_802_3 for Settings::IsValid(), and the Mode S CRC.
    test_lr2021_ook_adsb.cc
    test_lr2021_status_tracker.cc
    test_packet_decoder.cc
    test_settings.cc
)

# mode_s_packet.cpp prints uint32_t with %lX, which is right on the 32-bit targets (where uint32_t is long) and a format
# warning on 64-bit hosts. The ADSBee 1090 host tests build it with -Wno-format for the same reason.
set_source_files_properties(${ADSBEE_COMMON_DIR}/adsb/mode_s_packet.cpp PROPERTIES COMPILE_OPTIONS -Wno-format)

target_include_directories(host_test PRIVATE
    ${ADSBEE_MODULES_DIR}/googletest/googletest/include
    ${ADSBEE_1421_TI_DIR}
    ${ADSBEE_1421_TI_DIR}/lr2021
    ${ADSBEE_1421_TI_DIR}/settings
    ${ADSBEE_COMMON_DIR}/adsb
    ${ADSBEE_COMMON_DIR}/utils
    ${ADSBEE_COMMON_DIR}/utils/crc
    .  # comms.hh: console logging for host tests
)

# Make test runnable with "make test".
enable_testing()

include(GoogleTest)
gtest_discover_tests(host_test
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    PROPERTIES
        TIMEOUT 30
        DISCOVERY_TIMEOUT 10
)
