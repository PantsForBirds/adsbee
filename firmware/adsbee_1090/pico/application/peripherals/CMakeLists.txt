if(NOT ON_EMBEDDED_DEVICE)
    # Build for testing on host.
    foreach(_test_tgt IN ITEMS host_test host_test_fpu)
        if (TARGET ${_test_tgt})
            target_sources(${_test_tgt} PRIVATE

            )
            target_include_directories(${_test_tgt} PRIVATE

            )
        endif()
    endforeach()
else()
    # Build for embedded target
    add_subdirectory(esp32)
    target_sources(application PRIVATE
        eeprom.cc
        # si4362/si4362.cc
        cc1312/cc1312.cc
        gnss/nmea_parser.cc
        gnss/gnss_receiver.cc
        gnss/generic_gnss_receiver.cc
        gnss/ublox_max_m10.cc
    )
    target_include_directories(application PRIVATE
        .
        # si4362
        cc1312
        gnss
    )
endif()
