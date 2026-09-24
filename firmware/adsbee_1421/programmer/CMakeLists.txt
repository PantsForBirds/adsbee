cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 23)

# Waveshare RP2040-Zero. Its only LED is an addressable WS2812 on GP16 (PICO_DEFAULT_WS2812_PIN);
# there is no plain PICO_DEFAULT_LED_PIN, hence the PIO driver below. Overridable from the command
# line for anyone building this jig on a different RP2040 board.
set(PICO_BOARD waveshare_rp2040_zero CACHE STRING "Board type")

# PICO_SDK_PATH comes from the environment inside the pico-docker container (/usr/local/pico-sdk).
include(pico_sdk_import.cmake)

project(adsbee_1421_programmer C CXX ASM)

pico_sdk_init()

# ------------------------------------------------------------------------------------------------
# Baked-in CC1314 firmware image: the adsbee_1421 Release hex is converted into C segment arrays
# at build time. Both applications live under the same /firmware/adsbee_1421 mount inside the
# pico-docker container, so the relative default resolves there as well as on the host.
# ------------------------------------------------------------------------------------------------
set(ADSBEE_1421_HEX "${CMAKE_CURRENT_LIST_DIR}/../ti/build/Release/adsbee_1421.hex"
    CACHE FILEPATH "CC1314 firmware image (Intel HEX) to bake into the programmer")
set(ADSBEE_1421_OBJECT_DICTIONARY
    "${CMAKE_CURRENT_LIST_DIR}/../ti/object_dictionary/object_dictionary.cpp")

if(NOT EXISTS ${ADSBEE_1421_HEX})
    message(FATAL_ERROR
        "Baked firmware image not found: ${ADSBEE_1421_HEX}\n"
        "Build the CC1314 firmware first: ./build.sh ti")
endif()

set(FIRMWARE_IMAGE_C ${CMAKE_CURRENT_BINARY_DIR}/generated/firmware_image.c)
add_custom_command(
    OUTPUT ${FIRMWARE_IMAGE_C}
    COMMAND python3 ${CMAKE_CURRENT_LIST_DIR}/scripts/hex_to_c.py
            --hex ${ADSBEE_1421_HEX}
            --object-dictionary ${ADSBEE_1421_OBJECT_DICTIONARY}
            --out ${FIRMWARE_IMAGE_C}
    DEPENDS ${ADSBEE_1421_HEX} ${ADSBEE_1421_OBJECT_DICTIONARY}
            ${CMAKE_CURRENT_LIST_DIR}/scripts/hex_to_c.py
    COMMENT "Baking ${ADSBEE_1421_HEX} into firmware_image.c")

add_executable(adsbee_1421_programmer
    main.cpp
    target_uart.cc
    target_ctl.cc
    cc13x4_bootloader.cc
    flasher.cc
    at_client.cc
    bridge.cc
    status.cc
    bootsel.cc
    usb_descriptors.c
    ${FIRMWARE_IMAGE_C}
)

pico_generate_pio_header(adsbee_1421_programmer ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

# tusb_config.h lives at the project root; TinyUSB picks it up from the include path.
target_include_directories(adsbee_1421_programmer PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(adsbee_1421_programmer
    pico_stdlib

    hardware_uart    # UART0 to the CC1314 console / ROM bootloader
    hardware_irq     # UART RX interrupt
    hardware_pio     # WS2812 status LED
    hardware_structs # QSPI pads (BOOTSEL read)
    hardware_sync    # save_and_disable_interrupts() around the BOOTSEL read

    tinyusb_device   # CDC owned directly (line coding/state callbacks); no stdio_usb
    pico_unique_id   # USB serial number string
)

# Produces adsbee_1421_programmer.uf2 for drag-and-drop flashing over BOOT.
pico_add_extra_outputs(adsbee_1421_programmer)

# ------------------------------------------------------------------------------------------------
# Version-stamped artifact copies. The programmer has no version of its own -- its identity is the
# CC1314 firmware baked into it -- so the stamp is the baked firmware version, with a "fw" prefix
# (adsbee_1421_programmer-fw0.3.9.uf2) to mark it as such. Matches the names CI already publishes
# (.github/workflows/firmware.yml). The unversioned names stay stable for flashing/debug tooling;
# the stamped names are additional copies, mirroring the ti build's convention.
# ------------------------------------------------------------------------------------------------
set(FW_VERSION_OBJECT_DICTIONARY "${ADSBEE_1421_OBJECT_DICTIONARY}")
include("${CMAKE_CURRENT_LIST_DIR}/../cmake/fw_version.cmake")
set(PROGRAMMER_ARTIFACT_BASE "adsbee_1421_programmer-fw${FW_VERSION}")
add_custom_command(TARGET adsbee_1421_programmer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_BINARY_DIR}/adsbee_1421_programmer.uf2
        ${CMAKE_CURRENT_BINARY_DIR}/${PROGRAMMER_ARTIFACT_BASE}.uf2
    COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:adsbee_1421_programmer>
        ${CMAKE_CURRENT_BINARY_DIR}/${PROGRAMMER_ARTIFACT_BASE}.elf
    COMMENT "Creating version-stamped uf2/elf copies (${PROGRAMMER_ARTIFACT_BASE})"
)
