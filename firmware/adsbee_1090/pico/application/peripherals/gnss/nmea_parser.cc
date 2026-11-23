#include "nmea_parser.hh"

#include <cstdlib>  // For strtol, strtof, atoi.
#include <cstring>  // For strlen, strncmp.

namespace {

// Convert a single hex character to its nibble value, or -1 if not a hex digit.
int8_t HexCharToNibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

// Parse an NMEA latitude/longitude field in ddmm.mmmm / dddmm.mmmm format plus a hemisphere
// character ('N'/'S'/'E'/'W') into signed decimal degrees. Returns false if either field is empty.
bool ParseLatLon(const char* coord, const char* hemi, float& out_deg) {
    if (coord == nullptr || coord[0] == '\0' || hemi == nullptr || hemi[0] == '\0') {
        return false;
    }
    // Degrees are all digits before the decimal point except the last two (which are minutes).
    const char* dot = strchr(coord, '.');
    uint16_t int_digits = dot ? static_cast<uint16_t>(dot - coord) : static_cast<uint16_t>(strlen(coord));
    if (int_digits < 3) return false;  // Need at least dd + mm.
    uint16_t deg_digits = int_digits - 2;

    char deg_str[4] = {0};
    if (deg_digits >= sizeof(deg_str)) return false;
    memcpy(deg_str, coord, deg_digits);

    float degrees = static_cast<float>(atoi(deg_str));
    float minutes = strtof(coord + deg_digits, nullptr);
    float value = degrees + minutes / 60.0f;

    if (hemi[0] == 'S' || hemi[0] == 'W') value = -value;
    out_deg = value;
    return true;
}

constexpr float kMetersToFeet = 3.28084f;

// Parse a decimal field (e.g. "-32.57") into hundredths, rounded half away from zero, without floating point.
// Returns false if the field is empty or not a number.
bool ParseHundredths(const char* field, int32_t& out_hundredths) {
    if (field == nullptr || field[0] == '\0') return false;
    bool negative = field[0] == '-';
    const char* p = (field[0] == '-' || field[0] == '+') ? field + 1 : field;
    int32_t value = 0;
    bool has_digit = false;
    for (; *p >= '0' && *p <= '9'; p++) {
        if (value > INT32_MAX / 1000) return false;  // Leaves room for the fraction digits below.
        value = value * 10 + (*p - '0');
        has_digit = true;
    }
    int32_t fraction = 0;  // Thousandths, for rounding.
    if (*p == '.') {
        p++;
        for (int32_t scale = 100; *p >= '0' && *p <= '9'; p++, scale /= 10) {
            fraction += (*p - '0') * scale;
            has_digit = true;
        }
    }
    if (!has_digit || *p != '\0') return false;
    value = value * 100 + (fraction + 5) / 10;
    out_hundredths = negative ? -value : value;
    return true;
}

bool ParseUTCTime(const char* field, NMEAParser::GNSSFix& fix) {
    if (field == nullptr || strlen(field) < 6) return false;
    for (uint8_t i = 0; i < 6; i++) {
        if (field[i] < '0' || field[i] > '9') return false;
    }

    uint8_t hour = static_cast<uint8_t>((field[0] - '0') * 10 + field[1] - '0');
    uint8_t minute = static_cast<uint8_t>((field[2] - '0') * 10 + field[3] - '0');
    uint8_t second = static_cast<uint8_t>((field[4] - '0') * 10 + field[5] - '0');
    if (hour > 23 || minute > 59 || second > 60) return false;

    uint16_t millisecond = 0;
    if (field[6] == '.') {
        uint16_t scale = 100;
        for (uint8_t i = 7; field[i] != '\0' && scale > 0; i++, scale /= 10) {
            if (field[i] < '0' || field[i] > '9') break;
            millisecond += static_cast<uint16_t>(field[i] - '0') * scale;
        }
    }

    fix.utc_hour = hour;
    fix.utc_minute = minute;
    fix.utc_second = second;
    fix.utc_millisecond = millisecond;
    fix.utc_time_valid = true;
    return true;
}

// Parse an RMC date (ddmmyy) into days since 1970-01-01. From PR #451 (Mark Paluch).
bool ParseUTCDate(const char* field, uint32_t& days_since_epoch) {
    if (strlen(field) != 6) return false;
    for (uint8_t i = 0; i < 6; i++) {
        if (field[i] < '0' || field[i] > '9') return false;
    }
    uint8_t day = (field[0] - '0') * 10 + field[1] - '0';
    uint8_t month = (field[2] - '0') * 10 + field[3] - '0';
    uint16_t year = (field[4] - '0') * 10 + field[5] - '0';
    // NMEA's two-digit year is interpreted within the GNSS era, 1980-2079.
    year += year >= 80 ? 1900 : 2000;
    auto leap_year = [](uint16_t y) { return y % 4 == 0 && (y % 100 != 0 || y % 400 == 0); };
    static constexpr uint8_t month_days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (month < 1 || month > 12 || day < 1 || day > month_days[month - 1] + (month == 2 && leap_year(year))) {
        return false;
    }
    days_since_epoch = 0;
    for (uint16_t y = 1970; y < year; y++) days_since_epoch += leap_year(y) ? 366 : 365;
    for (uint8_t m = 1; m < month; m++) days_since_epoch += month_days[m - 1] + (m == 2 && leap_year(year));
    days_since_epoch += day - 1;
    return true;
}

}  // namespace

NMEAParser::SentenceType NMEAParser::IngestByte(char c) {
    if (c == '$') {
        // Start of a new sentence; reset accumulation state.
        in_sentence_ = true;
        line_len_ = 0;
        running_checksum_ = 0;
        reading_checksum_ = false;
        checksum_char_count_ = 0;
        return kSentenceNone;
    }

    if (!in_sentence_) {
        return kSentenceNone;  // Junk before a '$'; ignore.
    }

    if (c == '\r' || c == '\n') {
        // End of sentence. Validate checksum (if one was provided) then parse.
        bool complete = in_sentence_ && line_len_ > 0;
        in_sentence_ = false;
        if (!complete) return kSentenceNone;

        if (reading_checksum_) {
            if (checksum_char_count_ != 2) return kSentenceChecksumFail;
            int8_t hi = HexCharToNibble(checksum_chars_[0]);
            int8_t lo = HexCharToNibble(checksum_chars_[1]);
            if (hi < 0 || lo < 0) return kSentenceChecksumFail;
            uint8_t expected = static_cast<uint8_t>((hi << 4) | lo);
            if (expected != running_checksum_) return kSentenceChecksumFail;
        }
        // No '*' / checksum present: accept (tolerant of receivers that omit it).
        line_buf_[line_len_] = '\0';
        return ParseLine();
    }

    if (c == '*') {
        // Remainder of the line is the two-character checksum; stop XOR accumulation.
        reading_checksum_ = true;
        return kSentenceNone;
    }

    if (reading_checksum_) {
        if (checksum_char_count_ < 2) {
            checksum_chars_[checksum_char_count_++] = c;
        }
        return kSentenceNone;
    }

    // Body character: accumulate into the line buffer and update the running checksum.
    running_checksum_ ^= static_cast<uint8_t>(c);
    if (line_len_ < kLineBufMaxLen - 1) {
        line_buf_[line_len_++] = c;
    } else {
        // Overflow: drop the sentence to avoid a buffer overrun.
        in_sentence_ = false;
        return kSentenceMalformed;
    }
    return kSentenceNone;
}

NMEAParser::SentenceType NMEAParser::ParseLine() {
    // Split line_buf_ on ',' into field pointers (in place). Field 0 is the address field
    // (talker + sentence type), e.g. "GPGGA".
    char* fields[kMaxNumFields];
    uint8_t num_fields = 0;
    fields[num_fields++] = line_buf_;
    for (uint16_t i = 0; i < line_len_ && num_fields < kMaxNumFields; i++) {
        if (line_buf_[i] == ',') {
            line_buf_[i] = '\0';
            fields[num_fields++] = &line_buf_[i + 1];
        }
    }

    const char* addr = fields[0];
    // Address field is the 2-char talker + 3-char sentence type. Match on the sentence type only
    // (last 3 chars) to be talker-agnostic.
    uint16_t addr_len = static_cast<uint16_t>(strlen(addr));
    if (addr_len < 5) return kSentenceUnknown;
    const char* type = addr + (addr_len - 3);

    if (strncmp(type, "GGA", 3) == 0) {
        return ParseGGA(fields, num_fields);
    }
    if (strncmp(type, "RMC", 3) == 0) {
        return ParseRMC(fields, num_fields);
    }
    if (strncmp(type, "GST", 3) == 0) {
        return ParseGST(fields, num_fields);
    }
    return kSentenceUnknown;
}

// GGA field layout (talker-agnostic):
//   0: address  1: UTC time  2: lat  3: N/S  4: lon  5: E/W  6: fix quality
//   7: num satellites  8: HDOP  9: altitude (MSL)  10: altitude units (M)  11: geoid separation
//   12: geoid separation units (M)  ...
NMEAParser::SentenceType NMEAParser::ParseGGA(const char* const* fields, uint8_t num_fields) {
    if (num_fields < 11) return kSentenceMalformed;

    uint8_t fix_quality = static_cast<uint8_t>(atoi(fields[6]));
    fix_.fix_quality = fix_quality;
    fix_.num_satellites = static_cast<uint8_t>(atoi(fields[7]));
    ParseUTCTime(fields[1], fix_);

    if (fields[9][0] != '\0') {
        float altitude_m = strtof(fields[9], nullptr);
        fix_.altitude_ft = static_cast<int32_t>(altitude_m * kMetersToFeet);
    }
    // An empty field means the receiver does not know the geoid; older receivers end the sentence before it.
    fix_.geoid_separation_valid = num_fields > 11 && ParseHundredths(fields[11], fix_.geoid_separation_cm);

    float lat, lon;
    if (fix_quality > 0 && ParseLatLon(fields[2], fields[3], lat) && ParseLatLon(fields[4], fields[5], lon)) {
        fix_.latitude_deg = lat;
        fix_.longitude_deg = lon;
        fix_.valid = true;
        fix_.last_update_timestamp_ms = timestamp_ms_;
    } else if (fix_quality == 0) {
        fix_.valid = false;
    }
    return kSentenceGGA;
}

// RMC field layout (talker-agnostic):
//   0: address  1: UTC time  2: status (A=valid, V=void)  3: lat  4: N/S  5: lon  6: E/W
//   7: speed over ground (knots)  8: course over ground (deg)  9: date  ... (trailing fields vary)
NMEAParser::SentenceType NMEAParser::ParseRMC(const char* const* fields, uint8_t num_fields) {
    rmc_time_valid_ = false;
    rmc_leap_second_ = false;
    if (num_fields < 10) return kSentenceMalformed;

    bool status_valid = (fields[2][0] == 'A');
    bool time_valid = ParseUTCTime(fields[1], fix_) && status_valid;
    uint32_t days_since_epoch = 0;
    rmc_leap_second_ = time_valid && fix_.utc_second == 60;
    rmc_time_valid_ = time_valid && fix_.utc_second < 60 && fix_.utc_millisecond == 0 &&
                      ParseUTCDate(fields[9], days_since_epoch);
    if (rmc_time_valid_) {
        rmc_unix_seconds_ = static_cast<uint64_t>(days_since_epoch) * 86400 + fix_.utc_hour * 3600 +
                            fix_.utc_minute * 60 + fix_.utc_second;
    }

    if (fields[7][0] != '\0') {
        fix_.speed_kts = static_cast<int32_t>(strtof(fields[7], nullptr));
    }
    if (fields[8][0] != '\0') {
        fix_.heading_deg = strtof(fields[8], nullptr);
    }

    float lat, lon;
    if (status_valid && ParseLatLon(fields[3], fields[4], lat) && ParseLatLon(fields[5], fields[6], lon)) {
        fix_.latitude_deg = lat;
        fix_.longitude_deg = lon;
        fix_.valid = true;
        fix_.last_update_timestamp_ms = timestamp_ms_;
    } else if (!status_valid) {
        fix_.valid = false;
    }
    return kSentenceRMC;
}

// GST field layout (talker-agnostic):
//   0: address  1: UTC time  2: RMS of pseudorange residuals  3: error ellipse semi-major (m)
//   4: semi-minor (m)  5: orientation (deg)  6: latitude error (m)  7: longitude error (m)
//   8: altitude error (m)  (all 1 sigma)
NMEAParser::SentenceType NMEAParser::ParseGST(const char* const* fields, uint8_t num_fields) {
    if (num_fields < 9) return kSentenceMalformed;

    // Empty while the receiver has no fix.
    fix_.altitude_std_dev_valid = ParseHundredths(fields[8], fix_.altitude_std_dev_cm);
    if (fix_.altitude_std_dev_valid) {
        fix_.altitude_std_dev_timestamp_ms = timestamp_ms_;
    }
    return kSentenceGST;
}
