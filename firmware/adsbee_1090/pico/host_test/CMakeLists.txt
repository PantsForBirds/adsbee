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
        test_aircraft_dictionary_mode_s.cc
        test_aircraft_dictionary_allocator.cc
        test_buffer_utils.cc
        # test_adsbee.cc
        test_composite_array.cc
        test_crc.cc
        test_data_structures.cc
        test_decode_utils.cc
        test_fixedmath.cc
        test_geo_utils.cc
        test_nasa_cpr.cc
        test_mode_s_packet_decoder.cc
        test_nmea_parser.cc
        ${CMAKE_SOURCE_DIR}/application/peripherals/gnss/nmea_parser.cc
        test_platform.cc
        test_reporting_aircraftjson.cc
        test_reporting_beast.cc
        test_reporting_csbee.cc
        test_reporting_gdl90.cc
        test_reporting_raw.cc
        test_settings.cc
        test_spi_coprocessor.cc
        test_squitter_packets.cc
        test_transponder_packet.cc
        test_unit_conversions.cc
        test_pfb_queue_thread_safety.cc
    )

    target_include_directories(${_test_tgt} PRIVATE
        ${ADSBEE_COMMON_DIR}
        ${ADSBEE_MODULES_DIR}/googletest
        ${ADSBEE_MODULES_DIR}/cppAT/settings # use default settings for testing
        ${ADSBEE_MODULES_DIR}/cppAT/src
        ${CMAKE_SOURCE_DIR}/application/peripherals/gnss
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