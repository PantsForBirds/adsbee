# Host unit tests for the jig's SDK-independent logic. Standalone (not part of the pico build):
#   cmake -S firmware/adsbee_1421/programmer/host_test -B /tmp/jig_host_test
#   cmake --build /tmp/jig_host_test && ctest --test-dir /tmp/jig_host_test --output-on-failure
cmake_minimum_required(VERSION 3.13)
project(adsbee_1421_programmer_host_test CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

add_executable(modem_lines_test modem_lines_test.cc ../modem_lines.cc)
target_include_directories(modem_lines_test PRIVATE ..)
target_compile_options(modem_lines_test PRIVATE -Wall -Wextra -Werror)
add_test(NAME modem_lines_test COMMAND modem_lines_test)

# The jig's reboot-to-BOOTSEL baud must match the ADSBee 1090's PICO_STDIO_USB_RESET_MAGIC_BAUD_RATE.
set(ADSBEE_1090_PICO_CMAKE "${CMAKE_CURRENT_LIST_DIR}/../../../adsbee_1090/pico/CMakeLists.txt")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${ADSBEE_1090_PICO_CMAKE}")
file(READ "${ADSBEE_1090_PICO_CMAKE}" ADSBEE_1090_PICO_CMAKE_TEXT)
string(REGEX MATCH "PICO_STDIO_USB_RESET_MAGIC_BAUD_RATE=([0-9A-Fa-fXx]+)" _ "${ADSBEE_1090_PICO_CMAKE_TEXT}")
if(NOT CMAKE_MATCH_1)
    message(FATAL_ERROR "PICO_STDIO_USB_RESET_MAGIC_BAUD_RATE not found in ${ADSBEE_1090_PICO_CMAKE}")
endif()

add_executable(host_line_coding_test host_line_coding_test.cc)
target_include_directories(host_line_coding_test PRIVATE ..)
target_compile_definitions(host_line_coding_test PRIVATE ADSBEE_1090_RESET_MAGIC_BAUD_RATE=${CMAKE_MATCH_1})
target_compile_options(host_line_coding_test PRIVATE -Wall -Wextra -Werror)
add_test(NAME host_line_coding_test COMMAND host_line_coding_test)
