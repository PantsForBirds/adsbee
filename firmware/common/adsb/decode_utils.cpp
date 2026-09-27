#include "decode_utils.hh"

#include <cmath>

#include "unit_conversions.hh"

const uint16_t kModeSCallsignCharArrayLen = 64;
const char kModeSCallsignCharArray[] = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ##### ###############0123456789######";

char LookupModeSCallsignChar(uint8_t value) {
    if (value >= kModeSCallsignCharArrayLen) {
        return '#';
    }
    return kModeSCallsignCharArray[value];
}

const uint16_t kUATCallsignCharArrayLen = 40;
const char kUATCallsignCharArray[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ ###";

char LookupUATCallsignChar(uint8_t value) {
    if (value >= kUATCallsignCharArrayLen) {
        return '#';
    } else if (value >= kUATCallsignCharArrayLen - 3) {
        return '\0';  // Send reserved chars as end of string character.
    }
    return kUATCallsignCharArray[value];
}

int32_t GillhamToAltitudeFt(uint16_t gillham_value)
// Data must be in following order (MSB to LSB)
// D1 D2 D4 A1 A2 A4 B1 B2 B4 C1 C2 C4
{
    int32_t result;
    int16_t five_hundreds;
    int16_t one_hundreds;

    // Convert Gillham value using gray code to binary conversion algorithm.
    // Get rid of Hundreds (lower 3 bits).
    five_hundreds = gillham_value >> 3;

    // Strip off Five Hundreds leaving lower 3 bits.
    one_hundreds = gillham_value & 0x07;

    five_hundreds = GrayToBinary(five_hundreds);
    one_hundreds = GrayToBinary(one_hundreds);

    // Check for invalid codes.
    if (one_hundreds == 5 || one_hundreds == 6 || one_hundreds == 0) {
        return kAltitudeDecodeErrorGillhamDecodeError;
    }

    // Remove 7s from one_hundreds.
    if (one_hundreds == 7) one_hundreds = 5;

    // Correct order of one_hundreds.
    if (five_hundreds % 2) one_hundreds = 6 - one_hundreds;

    // Convert to feet and apply altitude datum offset.
    result = (int32_t)((five_hundreds * 500) + (one_hundreds * 100)) - 1300;

    return result;
}

uint16_t AltitudeCodeToGillham(uint16_t altitude_code) {
    uint8_t d1 = (altitude_code & (0b1 << 4)) >> 4;
    uint8_t d2 = (altitude_code & (0b1 << 2)) >> 2;
    uint8_t d4 = altitude_code & 0b1;

    uint8_t a1 = (altitude_code & (0b1 << 11)) >> 11;
    uint8_t a2 = (altitude_code & (0b1 << 9)) >> 9;
    uint8_t a4 = (altitude_code & (0b1 << 7)) >> 7;

    uint8_t b1 = (altitude_code & (0b1 << 5)) >> 5;
    uint8_t b2 = (altitude_code & (0b1 << 3)) >> 3;
    uint8_t b4 = (altitude_code & (0b1 << 1)) >> 1;

    uint8_t c1 = (altitude_code & (0b1 << 12)) >> 12;
    uint8_t c2 = (altitude_code & (0b1 << 10)) >> 10;
    uint8_t c4 = (altitude_code & (0b1 << 8)) >> 8;

    return (d1 << 11) | (d2 << 10) | (d4 << 9) | (a1 << 8) | (a2 << 7) | (a4 << 6) | (b1 << 5) | (b2 << 4) | (b4 << 3) |
           (c1 << 2) | (c2 << 1) | c4;
}

int32_t AltitudeCodeToAltitudeFt(uint16_t altitude_code) {
    // Altitude Code Format
    //                                 M        Q
    // +----+----+----+----+----+----+---+----+---+----+----+----+----+
    // | C1 | A1 | C2 | A2 | C4 | A4 | 0 | B1 | 0 | B2 | D2 | B4 | D4 |
    // +----+----+----+----+----+----+---+----+---+----+----+----+----+

    if (altitude_code == 0b0) {
        // All bits are 0: Altitude information not available or invalid.
        return kAltitudeDecodeErrorNotAvailableOrInvalid;
    }
    bool m = (altitude_code >> 6) & 0b1;
    bool q = (altitude_code >> 4) & 0b1;
    if (m) {
        // M=1: Altitude in meters.
        // Remove the M bit to get altitude in meters.
        int16_t altitude_meters = ((altitude_code & (0b111111 << 7)) >> 1) | (altitude_code & 0b111111);
        // Return altitude in feet.
        return MetersToFeet(altitude_meters);
    } else if (q) {
        // M=0, Q=1: Altitude in 25ft increments.
        int16_t left_blob = altitude_code & (0b111111 << 7);
        int16_t mid_bit = altitude_code & (0b1 << 5);
        int16_t right_blob = altitude_code & 0b1111;
        int16_t altitude_feet_increments = (left_blob >> 2) | (mid_bit >> 1) | right_blob;
        return 25 * altitude_feet_increments - 1000;
    } else {
        // M=0, Q=0: Regular Mode C reply.
        return GillhamToAltitudeFt(AltitudeCodeToGillham(altitude_code));
    }
}

uint16_t IdentityCodeToSquawk(uint16_t identity_code) {
    uint8_t d1 = (identity_code & (0b1 << 4)) >> 4;
    uint8_t d2 = (identity_code & (0b1 << 2)) >> 2;
    uint8_t d4 = identity_code & 0b1;

    uint8_t a1 = (identity_code & (0b1 << 11)) >> 11;
    uint8_t a2 = (identity_code & (0b1 << 9)) >> 9;
    uint8_t a4 = (identity_code & (0b1 << 7)) >> 7;

    uint8_t b1 = (identity_code & (0b1 << 5)) >> 5;
    uint8_t b2 = (identity_code & (0b1 << 3)) >> 3;
    uint8_t b4 = (identity_code & (0b1 << 1)) >> 1;

    uint8_t c1 = (identity_code & (0b1 << 12)) >> 12;
    uint8_t c2 = (identity_code & (0b1 << 10)) >> 10;
    uint8_t c4 = (identity_code & (0b1 << 8)) >> 8;

    uint16_t a = (a4 << 2) | (a2 << 1) | a1;
    uint16_t b = (b4 << 2) | (b2 << 1) | b1;
    uint16_t c = (c4 << 2) | (c2 << 1) | c1;
    uint16_t d = (d4 << 2) | (d2 << 1) | d1;
    return a * 1000 + b * 100 + c * 10 + d;
}