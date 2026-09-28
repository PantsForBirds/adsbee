if (ON_EMBEDDED_DEVICE)
    ## Building for target.
    add_executable(application main.cc)

    add_subdirectory(${ADSBEE_COMMON_DIR} ${CMAKE_BINARY_DIR}/application/adsbee_common)

    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        # set(PICO_DEOPTIMIZED_DEBUG 1)
        add_compile_definitions(HARDWARE_UNIT_TESTS)
        # Reported as "RP2040 Firmware Build: Debug" by AT+DEVICE_INFO?.
        add_compile_definitions(ADSBEE_DEBUG_BUILD)
    endif()

    # Create C header file with the name <pio program>.pio.h
    pico_generate_pio_header(application  
        ${CMAKE_CURRENT_LIST_DIR}/pio/capture.pio       
    )

    # Firmware: Pull in Pico library
    target_link_libraries(application 
        pico_stdlib
        pico_float # for math functions
        pico_rand # for generating random numbers
        pico_multicore
        hardware_pio
        hardware_pwm
        hardware_adc
        hardware_i2c
        hardware_spi
        hardware_exception
        hardware_watchdog

        hardware_dma
        hardware_flash
        hardware_structs
        hardware_sync # for flashing stuff
        hardware_resets
        cmsis_core # for bootloader regs
    )

    # Firmware: Create map/bin/hex file etc
    # pico_add_extra_outputs(application)
    # pico_add_hex_output(application) # temporary workwaround for this bug: https://github.com/raspberrypi/pico-sdk/issues/1829

    # Set up debug USB
    pico_enable_stdio_usb(application 1) # use USB for standard printing
    pico_enable_stdio_uart(application 0) # disable STDIO UART

    ## ESP32 Serial Flasher
    # Set parameters for serial flasher.
    # NOTE: These are mirrored in the top level `pico/.vscode/c_cpp_properties.json` for intellisense.
    add_compile_definitions(
        MD5_ENABLED 
        SERIAL_FLASHER_INTERFACE_UART 
        SERIAL_FLASHER_DEBUG_TRACE=0 
        SERIAL_FLASHER_WRITE_BLOCK_RETRIES=3
    )

    ## Embed ESP32 binaries into the app.
    # In ESP-IDF this can also be done using EMBED_FILES option of idf_component_register.
    # Here an external tool is used to make file embedding similar with other ports.
    include(${CMAKE_CURRENT_LIST_DIR}/cmake_scripts/bin2array.cmake)
    # Create a file dependency that copies over the most recently built ESP32 firmware binary to the bin directory used by
    # esp32_flasher. Re-runs cmake if the file gets changed.
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        set(ESP_BUILD_SUBDIR "build/Debug")
    else()
        set(ESP_BUILD_SUBDIR "build/Release")
    endif()
    configure_file(${CMAKE_SOURCE_DIR}/../esp/${ESP_BUILD_SUBDIR}/bootloader/bootloader.bin
        ${CMAKE_CURRENT_LIST_DIR}/peripherals/esp32/bin/bootloader.bin
        COPYONLY
    )
    configure_file(${CMAKE_SOURCE_DIR}/../esp/${ESP_BUILD_SUBDIR}/partition_table/partition-table.bin
        ${CMAKE_CURRENT_LIST_DIR}/peripherals/esp32/bin/partition-table.bin
        COPYONLY
    )
    configure_file(${CMAKE_SOURCE_DIR}/../esp/${ESP_BUILD_SUBDIR}/adsbee_esp.bin
        ${CMAKE_CURRENT_LIST_DIR}/peripherals/esp32/bin/adsbee_esp.bin
        COPYONLY
    )
    # For some reason putting binaries.c in the build folder never works out. Put it in the esp32_flasher bin directory
    # instead. NOTE: create_resources is a function defined in bin2array.cmake.
    create_resources(${CMAKE_CURRENT_LIST_DIR}/peripherals/esp32/bin
        ${CMAKE_CURRENT_LIST_DIR}/peripherals/esp32/bin/binaries.c
    )
    set_property(SOURCE ${CMAKE_CURRENT_LIST_DIR}/peripherals/esp32/bin/binaries.c PROPERTY GENERATED 1)
    target_sources(application PRIVATE ${CMAKE_CURRENT_LIST_DIR}/peripherals/esp32/bin/binaries.c)

    ## Embed TI CC1312 binary into the app.
    configure_file(${CMAKE_SOURCE_DIR}/../ti/sub_ghz_radio/build/sub_ghz_radio.bin
        ${CMAKE_CURRENT_LIST_DIR}/peripherals/cc1312/bin/sub_ghz_radio.bin
        COPYONLY
    )
    # Embed files in the cc1312 bin directory into an array of bytes in peripherals/cc1312/bin/binaries.c.
    create_resources(${CMAKE_CURRENT_LIST_DIR}/peripherals/cc1312/bin
        ${CMAKE_CURRENT_LIST_DIR}/peripherals/cc1312/bin/binaries.c
    )
    set_property(SOURCE ${CMAKE_CURRENT_LIST_DIR}/peripherals/cc1312/bin/binaries.c PROPERTY GENERATED 1)
    target_sources(application PRIVATE ${CMAKE_CURRENT_LIST_DIR}/peripherals/cc1312/bin/binaries.c)
    
    ## Build for embedded target
    MESSAGE("CppAT settings: ${ADSBEE_MODULES_DIR}/cppAT/settings")
    target_include_directories(application PRIVATE
        # ${ADSBEE_MODULES_DIR}/cppAT/settings # No longer need this since it's being overridden.
        ${ADSBEE_MODULES_DIR}/cppAT/src
        .
    )
    target_sources(application PRIVATE
        ${ADSBEE_MODULES_DIR}/cppAT/src/cpp_at.cc
        main.cc # Don't include main for testing.
        adsbee.cc
        settings.cc
    )

    pico_set_linker_script(application ${APPLICATION_LINKER_SCRIPT})

    # Generated outputs are .elf and .bin. Manually generate .UF2 output to take into account checksum.
    # pico_add_bin_output(application)

    # Create an executable for the application0 flash slot.
    add_executable(app0 $<TARGET_PROPERTY:application,SOURCES>)
    target_include_directories(app0 PRIVATE $<TARGET_PROPERTY:application,INCLUDE_DIRECTORIES>)
    target_link_libraries(app0 $<TARGET_PROPERTY:application,LINK_LIBRARIES>)
    pico_enable_stdio_usb(app0 1) # use USB for standard printing
    pico_enable_stdio_uart(app0 0) # disable STDIO UART
    pico_set_linker_script(app0 ${APP0_LINKER_SCRIPT})
    pico_add_bin_output(app0)

    # Create an executable for the application1 flash slot.
    add_executable(app1 $<TARGET_PROPERTY:application,SOURCES>)
    target_include_directories(app1 PRIVATE $<TARGET_PROPERTY:application,INCLUDE_DIRECTORIES>)
    target_link_libraries(app1 $<TARGET_PROPERTY:application,LINK_LIBRARIES>)
    pico_enable_stdio_usb(app1 1) # use USB for standard printing
    pico_enable_stdio_uart(app1 0) # disable STDIO UART
    pico_set_linker_script(app1 ${APP1_LINKER_SCRIPT})
    pico_add_bin_output(app1)

    
    # Create a header binary and assembly file for application.bin with section name hdr0.
    # Keeping the checksum separate allows us to do a combined build utilizing the sources for application.bin while
    # also loading the checksum onto the target so that the bootloader can examine it.
    # Make this execute after app1 so that app0.bin and app1.bin are available.
    add_custom_target(app_crcs
            COMMAND ${Python3_EXECUTABLE} ${LINKER_SCRIPTS_DIR}/util_crc.py
                    ${CMAKE_CURRENT_BINARY_DIR}/app0.bin
                    ${CMAKE_CURRENT_BINARY_DIR}/app1.bin
                    --header ${APPLICATION_HEADER_SECTION_NAME}
                    --ota ${CMAKE_CURRENT_BINARY_DIR}/adsbee_1090.ota
            DEPENDS app0 app1
            COMMENT "Calculating and appending CRC32 to application binary and UF2"
    )

    ## Combined build
    # Build a library with items that need to be linked for the combined build.
    add_library(combined_library OBJECT)
    # Ignore .S files during configuration since they are created after the build stage.
    set_source_files_properties(
        ${CMAKE_BINARY_DIR}/bootloader/${BOOTLOADER_SECTION_NAME}.S
        ${CMAKE_BINARY_DIR}/application/${APPLICATION_HEADER_SECTION_NAME}.S
        PROPERTIES
            GENERATED TRUE
            DEPENDS bootloader
            DEPENDS application
            # Also build app0 and app1 to allow processing them into an OTA file.
            DEPENDS app_crcs
    )
    # These source dependencies ensure that the combined target gets rebuilt if the bootloader or app get changed.
    target_sources(combined_library PRIVATE 
        ${CMAKE_BINARY_DIR}/bootloader/${BOOTLOADER_SECTION_NAME}.S
        ${CMAKE_BINARY_DIR}/application/${APPLICATION_HEADER_SECTION_NAME}.S
    )
    # Make sure the boot3.S and hdr0.S files get created before we try building the library!
    add_dependencies(combined_library
        bootloader
        application
        app0
        app1
        app_crcs
    )

    # Create a new executable called "combined" that uses the combined_library library and copies all sources from application.
    add_executable(combined
        $<TARGET_OBJECTS:combined_library>
        $<TARGET_PROPERTY:application,SOURCES>
    )
    target_include_directories(combined PRIVATE $<TARGET_PROPERTY:application,INCLUDE_DIRECTORIES>)
    target_link_libraries(combined $<TARGET_PROPERTY:application,LINK_LIBRARIES>)
    target_link_options(combined PUBLIC "-Wl,-Map=combined.map")
    target_compile_options(combined PRIVATE -fstack-usage) # Adds stack usage files in Debug/application/CMakeFiles/combined.dir/... as .su files.
    add_dependencies(combined
        bootloader
        application
        combined_library
    )
    pico_enable_stdio_usb(combined 1) # use USB for standard printing
    pico_enable_stdio_uart(combined 0) # disable STDIO UART
    pico_set_linker_script(combined ${COMBINED_LINKER_SCRIPT})
    pico_add_uf2_output(combined)

else()
    # Build for testing on host.
    foreach(_test_tgt IN ITEMS host_test host_test_fpu)
        if (TARGET ${_test_tgt})
            target_include_directories(${_test_tgt} PRIVATE

            )
            target_sources(${_test_tgt} PRIVATE

            )
        endif()
    endforeach()
endif()

add_subdirectory(comms)
add_subdirectory(peripherals)
add_subdirectory(target_test)