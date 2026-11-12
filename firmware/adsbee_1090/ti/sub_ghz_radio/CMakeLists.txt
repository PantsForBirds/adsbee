# Based on this Makefile: https://github.com/TexasInstruments/simplelink-prop_rf-examples/blob/main/examples/nortos/CC1312R1_LAUNCHXL/prop_rf/rfPacketRx/gcc/makefile

cmake_minimum_required(VERSION 3.16)
set(TI_MIN_CMAKE_VERSION 3.16)

message(STATUS "Current source directory: ${CMAKE_CURRENT_SOURCE_DIR}")
# Set the SimpleLink SDK path
set(SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR "$ENV{TI_LPF2_SDK_PATH}" CACHE PATH "ti-lpf2 SDK install dir.")
set(SYSCFG_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/syscfg" CACHE PATH "dir where all the sysconfig output goes.")
set(CMAKE_COMMON_INSTALL_DIR ${SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR}/source/cmake/scripts/cmake-common CACHE PATH "cmake-common scripts dir from the LPF2 SDK.")
# set(CMAKE_TOOLCHAIN_FILE ${SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR}/source/cmake/toolchains/gcc.cmake)
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/custom_toolchain/gcc.cmake")
message(STATUS "SimpleLink SDK path: ${SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR}")

# Find ARM GCC compiler directory in /usr/local/
file(GLOB GCC_ARM_PATHS "/usr/local/gcc-arm-none-eabi-*")
if(GCC_ARM_PATHS)
    list(GET GCC_ARM_PATHS 0 GCC_ARMCOMPILER)
    message(STATUS "Found ARM GCC compiler: ${GCC_ARMCOMPILER}")
else()
    message(FATAL_ERROR "ARM GCC compiler not found in /usr/local/")
endif()
# Extract the GCC ARM compiler version from the path
string(REGEX MATCH "gcc-arm-none-eabi-([0-9]+\\.[0-9]+\\.[0-9]+)" _ ${GCC_ARMCOMPILER})
set(GCC_ARM_VERSION ${CMAKE_MATCH_1})
message(STATUS "ARM GCC compiler version: ${GCC_ARM_VERSION}")

# Compile definitions need to be defined before the target!
add_compile_definitions(DeviceFamily_CC13X2)
add_compile_definitions(ON_TI)

project(sub_ghz_radio C CXX ASM)

add_compile_options(-Wno-packed-bitfield-compat) # Ignore the packed bitfield compatibility warning from GCC 4.4, since we never used GCC that old.

# Create executable
add_executable(${PROJECT_NAME}
    main.cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES
    OUTPUT_NAME "${PROJECT_NAME}.elf"
)

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    ${SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR}/source
    ${SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR}/kernel/nortos
    ${SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR}/kernel/nortos/posix

    $(GCC_ARMCOMPILER)/arm-none-eabi/include/newlib-nano
    $(GCC_ARMCOMPILER)/arm-none-eabi/include
   
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/syscfg
    ${CMAKE_CURRENT_SOURCE_DIR}/smartrf
)

target_link_directories(${PROJECT_NAME} PRIVATE
    CMakeCommon::gcc_m4f # Use TI gcc toolchain for ARM M4 with hardware floating point.

    ${SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR}/source
    ${SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR}/kernel/nortos
    ${CMAKE_CURRENT_SOURCE_DIR}/syscfg
    $(GCC_ARMCOMPILER)/arm-none-eabi/lib/thumb/v7e-m+fp/hard
)

# Libraries to link
target_link_libraries(${PROJECT_NAME}
    PRIVATE
    stdc++
    gcc # GCC runtime library
    c # C standard library
    m # Math library
    # nosys
    CMakeCommon::gcc_m4f # Use TI gcc toolchain for ARM M4 with hardware floating point.

    ## Manually bring in the libraries from syscfg/ti_utils_build_linker.cmd.genlibs here!
    # Note: need to change "ccs" to "gcc" in the paths.
    ${SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR}/source/ti/drivers/rf/lib/gcc/m4f/rf_multiMode_cc13x2.a
    ${SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR}/source/ti/drivers/lib/gcc/m4f/drivers_cc13x2.a
    # ${SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR}/source/ti/devices/cc13x2_cc26x2/driverlib/bin/gcc/driverlib.lib
    ${SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR}/source/ti/grlib/lib/gcc/m4f/grlib.a

    # I added this.
    ${SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR}/kernel/nortos/lib/gcc/m4f/nortos_cc13x2.a
    # Added this since the driverlib.lib from the genlibs file did not include HWiP_destruct and HWiP_construct.
    ${SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR}/build/gcc/source/ti/devices/cc13x2_cc26x2/lib/gcc/m4f/driverlib/driverlib.lib
)

set(TI_LINKER_COMMAND_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cc13x2_cc26x2_nortos.lds")

target_sources(${PROJECT_NAME} PRIVATE
    ${SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR}/kernel/nortos/startup/startup_cc13x2_cc26x2_gcc.c # Get the resetISR and stuff.
    syscalls_stub.c
    fast_mem.c
    ${SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR}/kernel/nortos/NoRTOS.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/rfPacketRx.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/RFQueue.cpp
    ${SYSCFG_OUTPUT_DIR}/ti_devices_config.c
    ${SYSCFG_OUTPUT_DIR}/ti_drivers_config.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/smartrf/smartrf_settings.c

    settings.cpp
)

# Do NOT add the SDK itself as a subdirectory or else it will assume you are building the SDK and take over a bunch of important variables like CMAKE_COMMON_INSTALL_DIR.
add_subdirectory(${CMAKE_SOURCE_DIR}/../../../common ${CMAKE_BINARY_DIR}/adsbee_common)
add_subdirectory(comms)
add_subdirectory(peripherals)
add_subdirectory(sub_ghz_radio)

# Compiler flags
target_compile_options(${PROJECT_NAME} PRIVATE
    -mcpu=cortex-m4
    -march=armv7e-m
    -mthumb
    $<$<COMPILE_LANGUAGE:C>:-std=gnu17> # Suppress POSIX files from GCC to allow TI POSIX library to work (old versions of GCC included them).
    $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++23> # Suppress POSIX files from GCC to allow TI POSIX library to work (old versions of GCC included them).
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
    -ffunction-sections
    -fdata-sections
    -g
    -gstrict-dwarf
    -Wall
    # -O0
)

# Linker flags
target_link_options(${PROJECT_NAME} PRIVATE
    -T${TI_LINKER_COMMAND_FILE}
    -Wl,-Map=${PROJECT_NAME}.map
    -eresetISR
    -march=armv7e-m+fp
    -mthumb
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
    -nostartfiles
    -static
    -Wl,--gc-sections
    -specs=nano.specs
    # -specs=nosys.specs
)

# Custom command to create binary file
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary 
    --gap-fill 0xff 
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.elf ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMAND ${CMAKE_COMMAND} -DBIN=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin -P ${CMAKE_CURRENT_SOURCE_DIR}/check_ccfg.cmake
    COMMENT "Creating binary file"
)

