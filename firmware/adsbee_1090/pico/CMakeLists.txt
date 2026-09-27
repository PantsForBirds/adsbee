cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 23)

# Directories
set(LINKER_SCRIPTS_DIR "${CMAKE_SOURCE_DIR}/linker_scripts")
set(ADSBEE_COMMON_DIR "${CMAKE_SOURCE_DIR}/../../common") # Used for code shared between ESP32 and RP2040.
set(ADSBEE_MODULES_DIR "${CMAKE_SOURCE_DIR}/../../modules")

set(BOOTLOADER_LINKER_SCRIPT "${LINKER_SCRIPTS_DIR}/bootloader.ld")
set(APPLICATION_LINKER_SCRIPT "${LINKER_SCRIPTS_DIR}/application.ld")
set(APP0_LINKER_SCRIPT "${LINKER_SCRIPTS_DIR}/app0.ld")
set(APP1_LINKER_SCRIPT "${LINKER_SCRIPTS_DIR}/app1.ld")
set(COMBINED_LINKER_SCRIPT "${LINKER_SCRIPTS_DIR}/combined.ld")

set(BOOTLOADER_SECTION_NAME boot3)
set(APPLICATION_HEADER_SECTION_NAME hdr0)
set(APPLICATION_SECTION_NAME app0)

find_package (Python3 REQUIRED COMPONENTS Interpreter)

add_compile_options(-Wno-packed-bitfield-compat) # Ignore the packed bitfield compatibility warning from GCC 4.4, since we never used GCC that old.

# Function that converts a .bin file into a .S (assembly) file, allowing the .bin to be linked within a linker script
# via its section name. Currently only used to create boot3.S in the bootloader folder.
function(bin2asm BIN_FILE SECTION_NAME ASM_FILE)
    add_custom_target(${BOOTLOADER_SECTION_NAME} DEPENDS bootloader)
    add_custom_command(
        # OUTPUT bootloader 
        TARGET bootloader POST_BUILD
        DEPENDS ${BIN_FILE}
        COMMAND ${Python3_EXECUTABLE} ${LINKER_SCRIPTS_DIR}/util_asm.py 
                ${BIN_FILE}
                ${ASM_FILE}
                ${SECTION_NAME}
        # COMMENT "Converting ${BIN_FILE} to ${ASM_FILE} with section name ${SECTION_NAME}"
    )
endfunction()

if (NOT CMAKE_BUILD_TYPE STREQUAL "Test")
    # Flag indicating that build is for the embedded target. Used to toggle target_source calls.
    set(ON_EMBEDDED_DEVICE 1)
endif()

if (ON_EMBEDDED_DEVICE)

    ## Building for target.

    # Pull in SDK (must be before project)
    include(pico_sdk_import.cmake)
    if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
        message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
    endif()
    project(adsbee C CXX ASM)

    # Differentiate common code between Pico and ESP32. For code highlighting, add this to the "defines" property of
    # "configurations" in c_cpp_properties.json.
    add_compile_definitions(ON_PICO)
    add_compile_definitions(
        # Can't pass in expression in parentheses or else it's a "functional" macro that isn't supported.
        PICO_FLASH_SIZE_BYTES=16777216 # 16MB = 16*1024*1024 Bytes of flash.
        # Change magic baud rate for resetting USB to 1234 baud, since 1200 baud is used by some programs like
        # pymavlink as a Linux kernel workaround during device connection.
        PICO_STDIO_USB_RESET_MAGIC_BAUD_RATE=0XDEADBEE
        # Override the default "Raspberry Pi" / "Pico" USB strings so hosts can identify the device. VID/PID are left
        # at the Raspberry Pi CDC defaults so stock host drivers and tooling still bind. The SDK truncates each USB
        # string to USBD_DESC_STR_MAX - 1 = 19 characters.
        USBD_MANUFACTURER="Pants for Birds"
        USBD_PRODUCT="ADSBee 1090"
    )

    # Initialize the SDK
    pico_sdk_init()

    add_subdirectory(bootloader)
    add_subdirectory(application)

    # Note: "combined" build lives in the application subdirectory, since all the include directories and source paths
    # copied from the "application" build are referneced to that directory.

else()
    ## Compiling for testing on host.

    project(adsbee C CXX ASM) # This is duplicated since the target firmware CMakeLists needs to create the project
    # after the pico SDK is pulled in.

    set(CMAKE_BUILD_TYPE "Debug") # Now that we've identified we're running tests, revert to an actual build config.
    add_compile_definitions(ON_HOST)
    add_compile_options(
        -Wall                # Include all warnings.
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        -Wno-packed-bitfield-compat # Ignore the packed bitfield compatibility warning from GCC 4.4, since we never used GCC that old.
    )

    # Create the executable and bring in testing source files.
    add_subdirectory(host_test)
endif()

enable_testing() # Allow subdirectories to create tests.