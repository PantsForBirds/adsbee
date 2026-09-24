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

# Default to an optimized build. custom_toolchain/gcc.cmake only adds -O3 under $<CONFIG:Release>
# (and -O0 under $<CONFIG:Debug>), so a configure that omits -DCMAKE_BUILD_TYPE would otherwise
# silently produce an unoptimized image (no -O flag at all) -- several times slower on the RS
# decode / reporting hot paths. build.sh and CI always pass the type; this guards hand/IDE configures.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type (Release|Debug)" FORCE)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

project(adsbee_1421 C CXX ASM)

# Derive firmware version from the single source of truth in object_dictionary.cpp (shared module,
# also used by the programmer build to stamp its artifacts with the baked firmware version).
set(FW_VERSION_OBJECT_DICTIONARY "${CMAKE_CURRENT_SOURCE_DIR}/object_dictionary/object_dictionary.cpp")
include("${CMAKE_CURRENT_SOURCE_DIR}/../cmake/fw_version.cmake")
set(ARTIFACT_BASE "${PROJECT_NAME}-${FW_VERSION}")
message(STATUS "Artifact base: ${ARTIFACT_BASE}")

# Compile definitions need to be defined before the target!
add_compile_definitions(DeviceFamily_CC13X4)
add_compile_definitions(ON_TI)
add_compile_definitions(ON_EMBEDDED_DEVICE)
add_compile_definitions(HARDWARE_UNIT_TESTS)
add_compile_definitions(AIRCRAFT_DICTIONARY_MAX_NUM_AIRCRAFT=400)

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
)

target_link_directories(${PROJECT_NAME} PRIVATE
    CMakeCommon::gcc_m33f # Use TI gcc toolchain for ARM M4 with hardware floating point.

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
    CMakeCommon::gcc_m33f # Use TI gcc toolchain for ARM M33 with hardware floating point.

    ## Manually bring in the libraries from syscfg/ti_utils_build_linker.cmd.genlibs here!
    # Note: need to change "ccs" to "gcc" in the paths.
    ${SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR}/source/ti/drivers/rf/lib/gcc/m33f/rf_multiMode_cc13x4.a
    ${SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR}/source/ti/drivers/lib/gcc/m33f/drivers_cc13x4.a
    # ${SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR}/source/ti/devices/cc13x4_cc26x4/driverlib/bin/gcc/driverlib.lib
    # ${SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR}/source/ti/grlib/lib/gcc/m33f/grlib.a

    # I added this.
    ${SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR}/kernel/nortos/lib/gcc/m33f/nortos_cc13x4.a
    # Added this since the driverlib.lib from the genlibs file did not include HWiP_destruct and HWiP_construct.
    ${SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR}/build/gcc/source/ti/devices/cc13x4_cc26x4/lib/gcc/m33f/driverlib/driverlib.lib
)

set(TI_LINKER_COMMAND_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cc13x4_cc26x4_nortos.lds")

target_sources(${PROJECT_NAME} PRIVATE
    ${SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR}/kernel/nortos/startup/startup_cc13x4_cc26x4_gcc.c # Get the resetISR and stuff.
    syscalls_stub.c
    ${SIMPLELINK_CC13XX_CC26XX_SDK_INSTALL_DIR}/kernel/nortos/NoRTOS.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/rfPacketRx.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/RFQueue.cpp
    ${SYSCFG_OUTPUT_DIR}/ti_devices_config.c
    ${SYSCFG_OUTPUT_DIR}/ti_drivers_config.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/smartrf/smartrf_settings.c

    adsbee.cpp
    packet_decoder.cpp
    # settings.cpp
)

# This will help us reference files from the common directory.
set(ADSBEE_COMMON_DIR "${CMAKE_SOURCE_DIR}/../../common" CACHE PATH "Path to adsbee common directory")
target_include_directories(${PROJECT_NAME} PRIVATE
    ${ADSBEE_COMMON_DIR} # This is a bit of a hack so that utils things can still be referenced from utils/whatever
)

set(ADSBEE_MODULES_DIR "${CMAKE_SOURCE_DIR}/../../modules" CACHE PATH "Path to adsbee modules directory")

add_subdirectory(${CMAKE_SOURCE_DIR}/lr2021)
add_subdirectory(${CMAKE_SOURCE_DIR}/object_dictionary)
add_subdirectory(${CMAKE_SOURCE_DIR}/target_test)
add_subdirectory(${CMAKE_SOURCE_DIR}/comms)
add_subdirectory(${CMAKE_SOURCE_DIR}/utils)
add_subdirectory(${CMAKE_SOURCE_DIR}/settings)
add_subdirectory(${CMAKE_SOURCE_DIR}/adsb)
add_subdirectory(${CMAKE_SOURCE_DIR}/sub_ghz_radio)

# Do NOT add the SDK itself as a subdirectory or else it will assume you are building the SDK and take over a bunch of important variables like CMAKE_COMMON_INSTALL_DIR.

# Don't add this subdirectory anymore since we don't want to use its built-in CMakeLists.txt.
# add_subdirectory(${CMAKE_SOURCE_DIR}/../../common ${CMAKE_BINARY_DIR}/adsbee_common)

# Compiler flags
target_compile_options(${PROJECT_NAME} PRIVATE
    -mcpu=cortex-m33
    -mthumb
    $<$<COMPILE_LANGUAGE:C>:-std=gnu17> # Suppress POSIX files from GCC to allow TI POSIX library to work (old versions of GCC included them).
    $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++23> # Suppress POSIX files from GCC to allow TI POSIX library to work (old versions of GCC included them).
    -mfloat-abi=hard
    -mfpu=fpv5-sp-d16
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
    -mcpu=cortex-m33
    -mthumb
    -mfloat-abi=hard
    -mfpu=fpv5-sp-d16
    -nostartfiles
    -static
    -Wl,--gc-sections
    # -specs=nano.specs  # Removed: nano printf lacks %lld support, crashing utest timing output.
    # -specs=nosys.specs
)

option(GENERATE_HEX "Run objcopy to produce a .hex file after build" ON)
if(GENERATE_HEX)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND arm-none-eabi-objcopy -O ihex
            $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex
        COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex ${CMAKE_CURRENT_BINARY_DIR}/${ARTIFACT_BASE}.hex
        COMMENT "Creating hex file and version-stamped copy (${ARTIFACT_BASE}.hex)"
    )
endif()

# Version-stamped elf/map copies for release/CI. The unversioned names above stay
# stable so debug/flash tooling (ti.code-workspace, AGENTS.md) keeps working.
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_CURRENT_BINARY_DIR}/${ARTIFACT_BASE}.elf
    COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map ${CMAKE_CURRENT_BINARY_DIR}/${ARTIFACT_BASE}.map
    COMMENT "Creating version-stamped elf/map copies (${ARTIFACT_BASE})"
)

