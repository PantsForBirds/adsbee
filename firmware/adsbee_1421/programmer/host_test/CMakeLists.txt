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
