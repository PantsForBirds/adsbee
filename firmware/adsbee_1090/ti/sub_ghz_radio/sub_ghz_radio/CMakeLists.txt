target_sources(sub_ghz_radio PRIVATE
    smartrf/smartrf_settings.c
    sub_ghz_radio.cpp
    uat_packet_decoder.cpp
)
target_include_directories(sub_ghz_radio PRIVATE
    .
)
