# Host unit tests for the ADSBee 1421 Programmer's SDK-independent logic, run with googletest like the ADSBee 1090
# host tests (firmware/adsbee_1090/pico/host_test). Standalone (not part of the pico build). Build googletest
# (firmware/modules/googletest) first, as CI does:
#   cd firmware/modules/googletest && mkdir build && cd build && cmake -DBUILD_SHARED_LIBS=ON .. && make
# Then:
#   cmake -S firmware/adsbee_1421/programmer/host_test -B /tmp/adsbee_1421_programmer_host_test
#   cmake --build /tmp/adsbee_1421_programmer_host_test
#   ctest --test-dir /tmp/adsbee_1421_programmer_host_test --output-on-failure
cmake_minimum_required(VERSION 3.13)
project(adsbee_1421_programmer_host_test CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Debug")

set(ADSBEE_MODULES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules")

find_package(Threads REQUIRED)

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
    ../autobaud.cc
    ../rate_watch.cc
    ../modem_lines.cc
    test_autobaud.cc
    test_console_baud.cc
    test_rate_watch.cc
    test_cdc_text.cc
    test_host_line_coding.cc
    test_modem_lines.cc
)

# console_baud.hh and console_autobaud.hh, shared with the module firmware.
target_include_directories(host_test PRIVATE
    ${ADSBEE_MODULES_DIR}/googletest/googletest/include
    ..
    ../../ti/comms
)

# The Programmer's reboot-to-BOOTSEL baud must match the ADSBee 1090's PICO_STDIO_USB_RESET_MAGIC_BAUD_RATE.
set(ADSBEE_1090_PICO_CMAKE "${CMAKE_CURRENT_LIST_DIR}/../../../adsbee_1090/pico/CMakeLists.txt")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${ADSBEE_1090_PICO_CMAKE}")
file(READ "${ADSBEE_1090_PICO_CMAKE}" ADSBEE_1090_PICO_CMAKE_TEXT)
string(REGEX MATCH "PICO_STDIO_USB_RESET_MAGIC_BAUD_RATE=([0-9A-Fa-fXx]+)" _ "${ADSBEE_1090_PICO_CMAKE_TEXT}")
if(NOT CMAKE_MATCH_1)
    message(FATAL_ERROR "PICO_STDIO_USB_RESET_MAGIC_BAUD_RATE not found in ${ADSBEE_1090_PICO_CMAKE}")
endif()
target_compile_definitions(host_test PRIVATE ADSBEE_1090_RESET_MAGIC_BAUD_RATE=${CMAKE_MATCH_1})

# Make test runnable with "make test".
enable_testing()

include(GoogleTest)
gtest_discover_tests(host_test
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    PROPERTIES
        TIMEOUT 30
        DISCOVERY_TIMEOUT 10
)
