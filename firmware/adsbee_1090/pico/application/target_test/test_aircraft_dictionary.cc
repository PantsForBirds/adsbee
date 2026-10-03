#include <algorithm>

#include "adsbee.hh"
#include "aircraft_dictionary.hh"
#include "hardware_unit_tests.hh"
#include "mode_s_packet.hh"

// Times ingesting an odd packet that completes a CPR pair, which runs the position decode.
uint64_t TimeDictionaryPacketIngestUs() {
    DecodedModeSPacket odd_packet = DecodedModeSPacket((const char*)"8D48C22D60AB00DEABC5DB78FCD6");  // odd
    odd_packet.raw.mlat_48mhz_64bit_counts = 1'000 * 48'000;
    DecodedModeSPacket even_packet = DecodedModeSPacket((const char*)"8D48C22D60AB0452BFAD19A695E0");  // even
    even_packet.raw.mlat_48mhz_64bit_counts = 2'000 * 48'000;
    uint32_t icao = odd_packet.icao_address;

    // Clear the dictionary for a fresh start.
    adsbee.aircraft_dictionary.RemoveAircraft(icao);

    // Ingest the odd packet to initialize the aircraft.
    adsbee.aircraft_dictionary.IngestDecodedModeSPacket(odd_packet);
    adsbee.aircraft_dictionary.IngestDecodedModeSPacket(even_packet);
    ModeSAircraft* aircraft = adsbee.aircraft_dictionary.GetAircraftPtr<ModeSAircraft>(icao);

    // Increment the timestamp of the odd packet and ingest it again. Time how long it takes the CPR filter to run.
    odd_packet.raw.mlat_48mhz_64bit_counts += 3'000 * 48'000;
    uint64_t start_timestamp_us = get_time_since_boot_us();
    adsbee.aircraft_dictionary.IngestDecodedModeSPacket(odd_packet);
    uint64_t end_timestamp_us = get_time_since_boot_us();

    return end_timestamp_us - start_timestamp_us;
}

// Median of several timed ingests, so an interrupt during one sample doesn't fail the test.
uint64_t MedianDictionaryPacketIngestUs() {
    static const uint16_t kNumSamples = 11;
    uint64_t samples_us[kNumSamples];
    for (uint16_t i = 0; i < kNumSamples; i++) {
        samples_us[i] = TimeDictionaryPacketIngestUs();
    }
    std::sort(samples_us, samples_us + kNumSamples);
    uint64_t median_us = samples_us[kNumSamples / 2];
    printf(
        "Aircraft dictionary took %llu us (median of %u, min %llu, max %llu) to ingest a packet with the CPR filter "
        "%s.\n",
        median_us, kNumSamples, samples_us[0], samples_us[kNumSamples - 1],
        (adsbee.aircraft_dictionary.CPRPositionFilterIsEnabled() ? "enabled" : "disabled"));
    return median_us;
}

UTEST(Dictionary, TestCPRFilterTiming) {
    bool original_cpr_filter_setting = adsbee.aircraft_dictionary.CPRPositionFilterIsEnabled();
    adsbee.aircraft_dictionary.SetCPRPositionFilterEnabled(false);
    EXPECT_LE(MedianDictionaryPacketIngestUs(), 100);

    adsbee.aircraft_dictionary.SetCPRPositionFilterEnabled(true);
    EXPECT_LE(MedianDictionaryPacketIngestUs(), 100);

    adsbee.aircraft_dictionary.SetCPRPositionFilterEnabled(original_cpr_filter_setting);
}