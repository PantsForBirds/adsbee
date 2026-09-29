# Host unit tests for SDK-independent ADSBee 1421 (CC1314 application) logic. Standalone (not part of the
# TI build):
#   cmake -S firmware/adsbee_1421/ti/host_test -B /tmp/adsbee_1421_host_test
#   cmake --build /tmp/adsbee_1421_host_test && ctest --test-dir /tmp/adsbee_1421_host_test --output-on-failure
cmake_minimum_required(VERSION 3.13)
project(adsbee_1421_host_test CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

add_executable(lr2021_ook_adsb_test lr2021_ook_adsb_test.cc)
target_include_directories(lr2021_ook_adsb_test PRIVATE ../lr2021)
target_compile_options(lr2021_ook_adsb_test PRIVATE -Wall -Wextra -Werror)
add_test(NAME lr2021_ook_adsb_test COMMAND lr2021_ook_adsb_test)
