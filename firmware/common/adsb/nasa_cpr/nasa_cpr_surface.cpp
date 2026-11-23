#include "awb_utils.h"
#include "comms.hh"
#include "nasa_cpr.hh"
#include "surface/cpr.hh"

static constexpr uint32_t kQuarterAWB32Resolution = 1073741824;  // 2^32 / 4
static constexpr uint32_t kEighthAWB32Resolution = 536870912;    // 2^32 / 8

bool NASACPRDecoder::DecodeSurfaceLocalCPR(const DecodedPosition& reference_position, const CPRMessage& message,
                                           DecodedPosition& decoded_position) {
    // For some reason local_dec doesn't use the struct message format field, so we need to manually tell it whether the
    // incoming packet is odd or even.
    nasa_cpr::surface::recovered_position result =
        nasa_cpr::surface::local_dec(message.odd, reference_position.lat_awb, reference_position.lon_awb,
                                     {.format = message.odd, .yz = message.lat_cpr, .xz = message.lon_cpr});
    decoded_position.lat_awb = result.lat_awb;
    decoded_position.lon_awb = result.lon_awb;
    return result.valid;
};

bool NASACPRDecoder::DecodeSurfaceGlobalCPR(const CPRMessage& even_message, const CPRMessage& odd_message,
                                            uint32_t ref_lat_awb32, uint32_t ref_lon_awb32,
                                            DecodedPosition& decoded_position) {
    if (even_message.odd == odd_message.odd) {
        CONSOLE_ERROR("NASACPRDecoder::DecodeSurfaceGlobalCPR", "Can't perform CPR decode with two %s messages.",
                      even_message.odd ? "odd" : "even");
        return false;  // Both messages must be one odd and one even.
    }

    nasa_cpr::surface::recovered_position result = nasa_cpr::surface::global_dec(
        odd_message.received_timestamp_ms > even_message.received_timestamp_ms,
        {.format = even_message.odd, .yz = even_message.lat_cpr, .xz = even_message.lon_cpr},
        {.format = odd_message.odd, .yz = odd_message.lat_cpr, .xz = odd_message.lon_cpr}, ref_lat_awb32);

    // Select the correct longitude zone. Result longitude must be no more than +/-(kQuarterAWB32Resolution / 2) away
    // from reference longitude.
    uint32_t lon_base_awb32 = ref_lon_awb32 - (ref_lon_awb32 % kQuarterAWB32Resolution);
    uint32_t translated_lon_awb32 = result.lon_awb + lon_base_awb32;
    if (__diffabs__(translated_lon_awb32, ref_lon_awb32) > (kQuarterAWB32Resolution / 2)) {
        translated_lon_awb32 +=
            (translated_lon_awb32 < ref_lon_awb32) ? kQuarterAWB32Resolution : -kQuarterAWB32Resolution;
    }

    decoded_position.lat_awb = result.lat_awb;
    decoded_position.lon_awb = translated_lon_awb32;
#ifdef VERBOSE_NASA_CPR_DEBUG
    if (!result.valid) {
        CONSOLE_WARNING("NASACPRDecoder::DecodeSurfaceGlobalCPR",
                        "Can't decode CPR packets from different latitude zones.");
    }
#endif                    /* VERBOSE_NASA_CPR_DEBUG */
    return result.valid;  // False if aircraft crossed between latitude zones between messages.
}