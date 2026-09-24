#pragma once
extern "C" {
#include "rs.h"
}
#include "unit_conversions.hh"  // For CeilBitsToBytes

/**
 * This file contains wrappers for the FEC encode / decode functions that make them convenient to use when decoding UAT
 * messages. Constants for message block lengths and Reed-Solomon parameters are defined here.
 */

class UATReedSolomon {
   public:
    UATReedSolomon();

    /**
     * Attempt decoding of a short ADS-B message.
     * Buffer will only be modified if decode is successful.
     * @param message_buf Buffer with the encoded message to decode. If decode is successful, buffer will be modified in
     * place to contain the decoded payload. If decode is unsuccessful, buffer will not be modified.
     * @return Number of bits corrected. 0 if message had no errors, positive number if errors were corrected, -1 if
     * message was invalid and not correctable.
     */
    int DecodeShortADSBMessage(uint8_t message_buf[]);

    /**
     * Attempt decoding of a long ADS-B message.
     * Buffer will only be modified if decode is successful.
     * @param message_buf Buffer with the encoded message to decode. If decode is successful, buffer will be modified in
     * place to contain the decoded payload. If decode is unsuccessful, buffer will not be modified
     * @return Number of bits corrected. 0 if message had no errors, positive number if errors were corrected, -1 if
     * message was invalid and not correctable.
     */
    int DecodeLongADSBMessage(uint8_t message_buf[]);

    /**
     * Attempt decoding of a UAT uplink message. Corrects the interleaved encoded message IN PLACE as a side effect: on
     * success, encoded_message_buf holds the corrected codeword (payload and parity), so downstream consumers can
     * de-interleave it directly (DeInterleaveUplinkMessage) without a further FEC pass. On failure, blocks decoded
     * before the failing block may already have been corrected in place; callers are expected to discard the message.
     * @param[out] decoded_payload_buf Buffer to store the decoded (de-interleaved, parity-stripped) payload in. May be
     * partially written on failure.
     * @param[in,out] encoded_message_buf Buffer with the encoded message to decode. Corrected in place.
     * @return Number of bytes corrected. 0 if message had no errors, positive number if errors were corrected, -1 if
     * message was invalid and not correctable.
     */
    int DecodeUplinkMessage(uint8_t decoded_payload_buf[], uint8_t encoded_message_buf[]);

    /**
     * Transform an already corrected (but interleaved) raw UAT uplink payload into a de-interleaved payload that can be
     * used directly. Does not apply FEC, just de-interleaves and tosses out the parity bytes. Static since it needs no
     * Reed-Solomon state.
     *
     * The de-interleave is performed directly into deinterleaved_buf (no intermediate copy), so the two buffers MUST
     * NOT overlap. In practice they never do -- the output is a 432-byte payload buffer and the input is a separate
     * 552-byte encoded message buffer -- but callers must not pass aliasing pointers.
     * @param[out] deinterleaved_buf Buffer to store the de-interleaved payload in. Must not overlap encoded_message_buf.
     * @param[in] encoded_message_buf Buffer with the encoded message to de-interleave. Must be a valid message with FEC
     * corrections pre-applied.
     */
    static void DeInterleaveUplinkMessage(uint8_t deinterleaved_buf[], const uint8_t encoded_message_buf[]);

    /**
     * Test functions used to turning test messages (provided without FEC headers) into encoded messages similar to what
     * would be found on the air.
     */
    bool EncodeShortADSBMessage(uint8_t message_buf[]);
    bool EncodeLongADSBMessage(uint8_t message_buf[]);
    bool EncodeUplinkMessage(uint8_t encoded_message_buf[], uint8_t decoded_payload_buf[]);

    void* rs_adsb_short = nullptr;
    void* rs_adsb_long = nullptr;
    void* rs_uplink = nullptr;
};

/**
 * WARNING: The Reed-Solomon decoder below needs to be initialized by an external function before it can be used.
 */
extern UATReedSolomon uat_rs;