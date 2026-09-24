
# Vendored Open Drone ID (Broadcast Remote ID) decoder. Compiled into the RP2040 (application) and host_test builds,
# which maintain an AircraftDictionary that ingests Remote ID. The CC1312 (sub_ghz_radio) does not decode Remote ID, so
# it is intentionally omitted there. ODID_DISABLE_PRINTF strips the library's stdio-based debug helpers.
set(OPENDRONEID_DIR "${ADSBEE_MODULES_DIR}/opendroneid-core-c/libopendroneid")

if (TARGET application)
    add_subdirectory(nasa_cpr)
    target_sources(application PRIVATE
        mode_s_packet.cpp
        uat_packet.cpp
        aircraft_dictionary.cpp
        decode_utils.cpp
        mode_s_packet_decoder.cc
        ${OPENDRONEID_DIR}/opendroneid.c
    )
    target_include_directories(application PRIVATE
        .
        ${OPENDRONEID_DIR}
    )
    target_compile_definitions(application PRIVATE ODID_DISABLE_PRINTF)
elseif (TARGET sub_ghz_radio)
    add_subdirectory(nasa_cpr)
    target_sources(sub_ghz_radio PRIVATE
        # adsb/mode_s_packet.cpp
        uat_packet.cpp
        # adsb/aircraft_dictionary.cpp
        decode_utils.cpp
    )
    target_include_directories(sub_ghz_radio PRIVATE
        .
    )
elseif (TARGET host_test OR TARGET host_test_fpu)
    add_subdirectory(nasa_cpr)
    foreach(_test_tgt IN ITEMS host_test host_test_fpu)
        if (TARGET ${_test_tgt})
            target_sources(${_test_tgt} PRIVATE
                mode_s_packet.cpp
                uat_packet.cpp
                aircraft_dictionary.cpp
                decode_utils.cpp
                mode_s_packet_decoder.cc
                ${OPENDRONEID_DIR}/opendroneid.c
            )
            target_include_directories(${_test_tgt} PRIVATE
                .
                ${OPENDRONEID_DIR}
            )
            target_compile_definitions(${_test_tgt} PRIVATE ODID_DISABLE_PRINTF)
        endif()
    endforeach()
endif()
