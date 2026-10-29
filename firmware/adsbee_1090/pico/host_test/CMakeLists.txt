add_executable(host_test main.cc)
add_executable(host_test_fpu main.cc)

find_package(Threads REQUIRED)

set(CMAKE_BUILD_TYPE "Debug") # Now that we've identified we're running tests, revert to an actual build config.

add_compile_options(
    -Wall                # Include all warnings.
    -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
    -Wno-unused-function # we have some for the docs that aren't called
    -Wno-maybe-uninitialized
    -ftrapv # Trap on signed integer overflow (undefined behavior).
)

# Test: Pull in google test library
add_library(libgtest SHARED IMPORTED)
set_target_properties(libgtest PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/../../modules/googletest/build/lib/libgtest.so)
target_link_libraries(host_test PRIVATE libgtest Threads::Threads)
target_link_libraries(host_test_fpu PRIVATE libgtest Threads::Threads)
target_compile_definitions(host_test_fpu PRIVATE HAS_FPU)
# Dictionary size per processor: RP2040 (no FPU) 400, ESP32 (FPU) 200.
target_compile_definitions(host_test PRIVATE AIRCRAFT_DICTIONARY_MAX_NUM_AIRCRAFT=400)
target_compile_definitions(host_test PRIVATE SWARM_RECORDING)
target_compile_definitions(host_test_fpu PRIVATE SWARM_RECORDING)
# The second test binary also runs the geo math with the multiply the RP2040 (Cortex-M0+) uses.
target_compile_definitions(host_test_fpu PRIVATE GEO_UTILS_SPLIT_MULTIPLY)
target_compile_definitions(host_test PRIVATE SETTINGS_BLOBS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/settings_blobs")
target_compile_definitions(host_test_fpu PRIVATE SETTINGS_BLOBS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/settings_blobs")
# MAVLink send helpers need the firmware's comms_manager; tests only exercise message packing.
target_compile_definitions(host_test PRIVATE MAVLINK_NO_CONVENIENCE_FUNCTIONS)
target_compile_definitions(host_test_fpu PRIVATE MAVLINK_NO_CONVENIENCE_FUNCTIONS)

add_subdirectory(${ADSBEE_COMMON_DIR} ${CMAKE_BINARY_DIR}/adsbee_common)
add_subdirectory(${CMAKE_SOURCE_DIR}/bootloader ${CMAKE_BINARY_DIR}/bootloader)
add_subdirectory(${CMAKE_SOURCE_DIR}/application ${CMAKE_BINARY_DIR}/application)

foreach(_test_tgt IN ITEMS host_test host_test_fpu)
    target_sources(${_test_tgt} PRIVATE
        ${ADSBEE_MODULES_DIR}/cppAT/src/cpp_at.cc
        ${ADSBEE_MODULES_DIR}/cppAT/test/test_cpp_at.cc
        hal.cc
        main.cc
        settings.cc
        # test_ads_b_decoder.cc
        test_adsl_packet.cc
        test_adsl_aircraft.cc
        test_aircraft_dictionary_contact_modes.cc
        test_aircraft_dictionary_mode_s.cc
        test_aircraft_dictionary_air_ground.cc
        test_aircraft_dictionary_allocator.cc
        test_ble_adv_watchdog.cc
        test_ble_scan_watchdog.cc
        test_broadcast_ring.cc
        test_buffer_utils.cc
        # test_adsbee.cc
        test_composite_array.cc
        test_esp32_firmware_check.cc
        test_cpu_utils.cc
        test_crc.cc
        test_data_structures.cc
        test_decode_utils.cc
        test_fixedmath.cc
        test_geo_utils.cc
        test_idle_session_tracker.cc
        test_nasa_cpr.cc
        test_mode_s_packet_decoder.cc
        test_mode_s_single_bit_correction.cc
        test_mode_s_bit_error_correction.cc
        test_mode_s_single_bit_filter.cc
        test_nmea_parser.cc
        ${CMAKE_SOURCE_DIR}/application/peripherals/gnss/nmea_parser.cc
        test_platform.cc
        test_reporting_aircraftjson.cc
        test_reporting_asterix.cc
        test_reporting_beast.cc
        test_reporting_csbee.cc
        test_reporting_gdl90.cc
        test_reporting_mavlink.cc
        test_reporting_raw.cc
        test_remote_id_guard.cc
        test_settings.cc
        test_settings_hash.cc
        test_settings_release_blobs.cc
        test_spi_coprocessor.cc
        test_spi_link_gate.cc
        test_at_console_guard.cc
        test_auto_update.cc
        test_wifi_network_selector.cc
        test_feed_datagram.cc
        test_swarm.cc
        test_feed_send_buffer.cc
        test_feed_receiver_id.cc
        test_log_message.cc
        test_squitter_packets.cc
        test_transponder_packet.cc
        test_uat_uplink_utils.cc
        test_websocket_frame_writer.cc
        test_unit_conversions.cc
        test_pfb_queue_thread_safety.cc
        test_mode_s_soft_demod.cc
        ${CMAKE_SOURCE_DIR}/application/mode_s_soft_demod.cc
        test_r1090_rssi.cc
        test_tl_learner.cc
        ${CMAKE_SOURCE_DIR}/application/tl_learner.cc
    )

    target_include_directories(${_test_tgt} PRIVATE
        ${ADSBEE_COMMON_DIR}
        ${ADSBEE_MODULES_DIR}/googletest
        ${ADSBEE_MODULES_DIR}/cppAT/settings # use default settings for testing
        ${ADSBEE_MODULES_DIR}/cppAT/src
        ${CMAKE_SOURCE_DIR}/application/peripherals/gnss
        ${CMAKE_SOURCE_DIR}/../esp/main/remote_id # ble_adv_watchdog.hh, ble_scan_watchdog.hh
        .
        mocks
    )
endforeach()

add_subdirectory(uat)

# Make test runnable with "make test".
enable_testing()

include(GoogleTest)
foreach(_test_tgt IN ITEMS host_test host_test_fpu)
    gtest_discover_tests(${_test_tgt}
        TEST_PREFIX "${_test_tgt}."
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        PROPERTIES
            TIMEOUT 30
            DISCOVERY_TIMEOUT 10
    )
endforeach()