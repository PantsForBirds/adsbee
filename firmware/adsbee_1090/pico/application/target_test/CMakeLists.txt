if(ON_EMBEDDED_DEVICE)
    # Build for embedded target. AT+TEST only exists in Debug builds, so Release leaves the tests out.
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_sources(application PRIVATE
            hardware_unit_tests.cc
            test_aircraft_dictionary.cc
            test_cc1312.cc
            test_clocks.cc
            test_eeprom.cc
            test_flash.cc
            test_spi_coprocessor.cc
        )
    endif()
    target_include_directories(application PRIVATE
        .
    )
else()
    # Build for testing on host.
    target_sources(host_test PRIVATE

    )
    target_include_directories(host_test PRIVATE
        
    )
endif()