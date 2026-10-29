
if (TARGET application)
    target_sources(application PRIVATE
        comms_reporting.cpp
        asterix/asterix_utils.cpp
        beast/beast_utils.cpp
        gdl90/gdl90_utils.cpp
        mavlink/mavlink_utils.cpp
        raw/raw_utils.cpp
    )
    target_include_directories(application PRIVATE
        .
        aircraftjson
        asterix
        beast
        csbee
        gdl90
        json
        mavlink
        raw
        uat_uplink
    )
elseif (TARGET sub_ghz_radio)
    target_include_directories(sub_ghz_radio PRIVATE
        .
        json # Not actually used, but added to avoid compile errors.
    )
elseif (TARGET host_test OR TARGET host_test_fpu)
    foreach(_test_tgt IN ITEMS host_test host_test_fpu)
        if (TARGET ${_test_tgt})
            target_sources(${_test_tgt} PRIVATE
                asterix/asterix_utils.cpp
                beast/beast_utils.cpp
                flarm_nmea/flarm_nmea.cpp
                gdl90/gdl90_utils.cpp
                mavlink/mavlink_utils.cpp
                raw/raw_utils.cpp
            )
            target_include_directories(${_test_tgt} PRIVATE
                aircraftjson
                asterix
                beast
                csbee
                flarm_nmea
                gdl90
                json
                mavlink
                raw
                uat_uplink
            )
        endif()
    endforeach()
endif()
