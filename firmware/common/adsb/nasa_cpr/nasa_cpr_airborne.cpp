#include "airborne/cpr.hh"
#include "comms.hh"
#include "nasa_cpr.hh"

bool NASACPRDecoder::DecodeAirborneLocalCPR(const DecodedPosition& reference_position, const CPRMessage& message,
                                            DecodedPosition& decoded_position) {
    // For some reason local_dec doesn't use the struct message format field, so we need to manually tell it whether the
    // incoming packet is odd or even.
    nasa_cpr::airborne::recovered_position result =
        nasa_cpr::airborne::local_dec(message.odd, reference_position.lat_awb, reference_position.lon_awb,
                                      {.format = message.odd, .yz = message.lat_cpr, .xz = message.lon_cpr});
    decoded_position.lat_awb = result.lat_awb;
    decoded_position.lon_awb = result.lon_awb;
    return result.valid;
};

bool NASACPRDecoder::DecodeAirborneGlobalCPR(const CPRMessage& even_message, const CPRMessage& odd_message,
                                             DecodedPosition& decoded_position) {
    if (even_message.odd == odd_message.odd) {
        CONSOLE_ERROR("NASACPRDecoder::DecodeAirborneGlobalCPR", "Can't perform CPR decode with two %s messages.",
                      even_message.odd ? "odd" : "even");
        return false;  // Both messages must be one odd and one even.
    }

    nasa_cpr::airborne::recovered_position result = nasa_cpr::airborne::global_dec(
        odd_message.received_timestamp_ms > even_message.received_timestamp_ms,
        {.format = even_message.odd, .yz = even_message.lat_cpr, .xz = even_message.lon_cpr},
        {.format = odd_message.odd, .yz = odd_message.lat_cpr, .xz = odd_message.lon_cpr});
    decoded_position.lat_awb = result.lat_awb;
    decoded_position.lon_awb = result.lon_awb;
#ifdef VERBOSE_NASA_CPR_DEBUG
    if (!result.valid) {
        CONSOLE_WARNING("NASACPRDecoder::DecodeAirborneGlobalCPR",
                        "Can't decode CPR packets from different latitude zones.");
    }
#endif                    /* VERBOSE_NASA_CPR_DEBUG */
    return result.valid;  // False if aircraft crossed between latitude zones between messages.
}