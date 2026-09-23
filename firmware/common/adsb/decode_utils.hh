#ifndef DECODE_UTILS_HH_
#define DECODE_UTILS_HH_

#include <cstdint>

const uint16_t kCPRNz = 15;                           // number of latitude zones between equator and a pole
const float kCPRdLatEven = 360.0f / (4 * kCPRNz);     // size of latitude zone for even message
const float kCPRdLatOdd = 360.0f / (4 * kCPRNz - 1);  // size of latitude zone for odd message
const uint32_t kCPRLatLonMaxCount = (2 << 16) - 1;    // 2^17

// Altitudes are encoded as 25N - 1000, where N is a positive integer, so the minimum allowed altitude is -1000ft.
// We can flag an error by setting the output altitude lower than this point.
enum kAltitudeDecodeError : int32_t {
    kAltitudeDecodeErrorInvalid = -1001,  // Used to check if any of the below error codes apply.
    kAltitudeDecodeErrorGillhamDecodeError = -2000,
    kAltitudeDecodeErrorNotAvailableOrInvalid = -2001,
};

/**
 * Converts a numeric value into an ASCII character for use in decoding callsigns.
 * @param[in] value 6-bit value to lookup.
 * @retval Character that matches the input value. If the input value is invalid or unused, returns '#'.
 */
char LookupModeSCallsignChar(uint8_t value);

/**
 * Converts a numeric value into an ASCII character for use in decoding UAT callsigns.
 * @param[in] value Value to lookup.
 * @retval Character that matches the input value. If the input value is invalid or unused, returns '#'.
 */
char LookupUATCallsignChar(uint8_t value);

/**
 * Converts a 16-bit gray code to binary. Credit to Kevin Stewart
 * (https://www.ccsinfo.com/forum/viewtopic.php?p=140960#140960).
 * @param[in] num 16-bit Gray-coded value.
 * @retval 16-bit binary-coded value.
 */
inline uint16_t GrayToBinary(uint16_t num) {
    uint16_t temp = num ^ (num >> 8);
    temp ^= (temp >> 4);
    temp ^= (temp >> 2);
    temp ^= (temp >> 1);
    return temp;
}

/**
 * Converts a Gillham code (modified gray code used by encoding altimeters) to an altitude value in feet.
 * Credit to Kevin Stewart (https://www.ccsinfo.com/forum/viewtopic.php?p=140960#140960).
 * @param[in] gillham_value Gillham encoded altitude value, in the format (MSB to LSB):
 *                              D1 D2 D4 A1 A2 A4 B1 B2 B4 C1 C2 C4.
 * @retval Signed altitude in feet.
 */
int32_t GillhamToAltitudeFt(uint16_t gillham_value);

/**
 * Rearranges the Altitude Code (AC) field of a Mode C (Surveillance Altitude Reply) packet into the gillham encoded
 * altitude value required by GillhamToAltitudeFt.
 * @param[in] altitude_code Mode C packet AC field, in the format (MSB to LSB):
 *                              C1 A1 C2 A2 C4 A4 M(X) B1 Q(D1) B2 D2 B4 D4
 * @retval Gillham encoded altitude value, in the format (MSB to LSB):
 *                              D1 D2 D4 A1 A2 A4 B1 B2 B4 C1 C2 C4.
 */
uint16_t AltitudeCodeToGillham(uint16_t altitude_code);

/**
 * Converts a Mode C (Surveillance Altitude Reply) AC field into an altitude in ft. Uses GillhamToAltitudeFt and
 * AltitudeCodeToGillham for regular Mode C replies, but also handles cases where the altitude reply is in metric or
 * 25ft increment units.
 * @param[in] altitude_code Mode C packet AC field, in the format (MSB to LSB):
 *                              C1 A1 C2 A2 C4 A4 M(X) B1 Q(D1) B2 D2 B4 D4
 * @retval Pressure altitude in feet.
 */
int32_t AltitudeCodeToAltitudeFt(uint16_t altitude_code);

/**
 * Converts a Mode A (Surveillance Identity Reply) ID field into a squawk code.
 * @param[in] identity_code Mode C packet ID field, in the format (MSB to LSB):
 *                              C1 A1 C2 A2 C4 A4 M(X) B1 Q(D1) B2 D2 B4 D4
 * @retval Squawk code as a decimal value whose digits are the four squawk digits (e.g. squawk 7700 returns 7700).
 */
uint16_t IdentityCodeToSquawk(uint16_t identity_code);

/**
 * Wrap southern hemisphere latitudes resulting from CPR decode. Transforms latitude from [270, 360] to [-90, 90].
 * @param[in] latitude Latitude in degrees.
 * @retval Wrapped latitude.
 */
inline float WrapCPRDecodeLatitude(float latitude) { return latitude >= 270.0f ? latitude - 360.0f : latitude; }

/**
 * Wrap longitude floating point value to conventional values. Transforms longitude from [0, 360] to [-180, 180].
 * @param[in] longitude Longitude in degrees.
 * @retval Wrapped longitude.
 */
inline float WrapCPRDecodeLongitude(float longitude) { return longitude >= 180.0f ? longitude - 360.0f : longitude; }

#endif /* DECODE_UTILS_HH_ */