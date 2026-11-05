target_sources(${PROJECT_NAME} PRIVATE
    smartrf/smartrf_settings.c
    sub_ghz_radio.cpp
    uat_packet_decoder.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
    .
)
