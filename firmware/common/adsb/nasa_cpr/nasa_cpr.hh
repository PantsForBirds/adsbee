#pragma once
#include "stdint.h"

// #define VERBOSE_NASA_CPR_DEBUG

class NASACPRDecoder {
   public:
    // Position in Angular Weighted Binary (see geo_utils.hh).
    struct DecodedPosition {
        uint32_t lat_awb;
        uint32_t lon_awb;
    };

    struct CPRMessage {
        bool odd;
        uint32_t lat_cpr;
        uint32_t lon_cpr;
        uint32_t received_timestamp_ms = 0;
    };

    /**
     * Decodes a CPR location for an airborne position using an existing reference position.
     * @param[in] reference_position The reference position to use for decoding. The "valid" property is ignored.
     * @param[in] message The CPR message to decode.
     * @param[out] decoded_position The decoded location.
     * @return True if the decode succeeded (recovered_position is valid), false otherwise.
     */
    static bool DecodeAirborneLocalCPR(const DecodedPosition& reference_position, const CPRMessage& message,
                                       DecodedPosition& decoded_position);

    /**
     * Decodes a CPR location for an airborne position using two CPR messages. The decoded location is returned based on
     * the most recent message, sorted by received_timestamp_ms.
     * @param[in] even_message The last even CPR message.
     * @param[in] odd_message The last odd CPR message.
     * @param[out] decoded_position The decoded location.
     * @return True if the decode succeeded (recovered_position is valid), false otherwise.
     */
    static bool DecodeAirborneGlobalCPR(const CPRMessage& even_message, const CPRMessage& odd_message,
                                        DecodedPosition& decoded_position);

    /**
     * Decodes a CPR location for a surface position using an existing reference position. Note that an ownship position
     * is not required because the reference position can be used.
     * @param[in] reference_position The reference position to use for decoding. The "valid" property is ignored.
     * @param[in] message The CPR message to decode.
     * @param[out] decoded_position The decoded location.
     * @return True if the decode succeeded (recovered_position is valid), false otherwise.
     */
    static bool DecodeSurfaceLocalCPR(const DecodedPosition& reference_position, const CPRMessage& message,
                                      DecodedPosition& decoded_position);

    /**
     * Decodes a CPR location for a surface position using two CPR messages. The decoded location is returned based
     * on the most recent message, sorted by received_timestamp_ms. Requires an ownship latitude.
     * @param[in] even_message The last even CPR message.
     * @param[in] odd_message The last odd CPR message.
     * @param[in] ref_lat_awb32 The ownship latitude in 32-bit AWB format.
     * @param[in] ref_lon_awb32 The ownship longitude in 32-bit AWB format.
     * @param[out] decoded_position The decoded location.
     * @return True if the decode succeeded (recovered_position is valid), false otherwise.
     */
    static bool DecodeSurfaceGlobalCPR(const CPRMessage& even_message, const CPRMessage& odd_message,
                                       uint32_t ref_lat_awb32, uint32_t ref_lon_awb32,
                                       DecodedPosition& decoded_position);
};
