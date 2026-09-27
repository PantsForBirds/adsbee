foreach(_test_tgt IN ITEMS host_test host_test_fpu)
    if (TARGET ${_test_tgt})
        target_sources(${_test_tgt} PRIVATE
            test_aircraft_dictionary_uat.cc
            test_uat_adsb_packet.cc
            test_uat_fec.cc
            test_uat_downlink.cc
            test_uat_uplink.cc
            test_uat_rs_decoder.cc
            test_uat_state_vector.cc

            uat_test_data.c
        )
    endif()
endforeach()
