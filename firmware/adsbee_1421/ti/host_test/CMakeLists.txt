# Host unit tests for hardware-independent ADSBee 1421 (CC1314) application logic. Standalone (not
# part of the firmware build):
#   cmake -S firmware/adsbee_1421/ti/host_test -B /tmp/adsbee_1421_host_test
#   cmake --build /tmp/adsbee_1421_host_test && ctest --test-dir /tmp/adsbee_1421_host_test --output-on-failure
# Pass -DADSBEE_1421_CCFG_BIN=<file> (the .ccfg section of a built image, from
# arm-none-eabi-objcopy -O binary -j .ccfg adsbee_1421.elf <file>) to also check that image's CCFG.
cmake_minimum_required(VERSION 3.13)
project(adsbee_1421_host_test CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

add_executable(ccfg_bootloader_test ccfg_bootloader_test.cc)
target_include_directories(ccfg_bootloader_test PRIVATE ../settings)
target_compile_options(ccfg_bootloader_test PRIVATE -Wall -Wextra -Werror)
add_test(NAME ccfg_bootloader_test COMMAND ccfg_bootloader_test)
if(ADSBEE_1421_CCFG_BIN)
    add_test(NAME ccfg_bootloader_built_image_test COMMAND ccfg_bootloader_test ${ADSBEE_1421_CCFG_BIN})
endif()
