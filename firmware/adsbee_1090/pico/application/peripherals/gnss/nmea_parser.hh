#pragma once

#include <cstdint>

/**
 * Generic, receiver-agnostic NMEA 0183 parser.
 *
 * Designed to be tolerant of NMEA protocol version differences (2.x through 4.11) so the same
 * parsing core works across a wide range of GNSS receivers:
 *   - Matches on the 3-character sentence type (e.g. "GGA", "RMC"), ignoring the 2-character
 *     talker ID, so multi-constellation talkers ($GPGGA, $GNGGA, $GLGGA, $GAGGA, ...) all parse.
 *   - Indexes fields positionally from the front and tolerates extra trailing fields appended by
 *     newer versions (e.g. RMC navigational status in v2.3+, system/signal IDs in v4.11).
 *   - Treats empty fields as "not present" rather than faulting.
 *   - Validates the XOR checksum when present.
 *
 * The parser has no platform dependencies (no pico-sdk) so it can be unit tested on the host.
 *
 * Usage: feed received UART bytes one at a time to IngestByte(). When a complete, valid sentence
 * is decoded the internally held GNSSFix is updated; query it via fix().
 */
class NMEAParser {
   public:
    // Maximum NMEA sentence length is 82 chars per spec; allow some slack for non-conforming
    // receivers. Includes the leading '$' but not a null terminator.
    static constexpr uint16_t kLineBufMaxLen = 120;
    static constexpr uint16_t kMaxNumFields = 32;

    // Result of ingesting a byte / completing a sentence.
    enum SentenceType : uint8_t {
        kSentenceNone = 0,      // No complete sentence yet (still accumulating).
        kSentenceGGA,           // GGA parsed and applied.
        kSentenceRMC,           // RMC parsed and applied.
        kSentenceUnknown,       // Complete sentence with a type we don't decode.
        kSentenceChecksumFail,  // Complete sentence but checksum did not match.
        kSentenceMalformed,     // Complete sentence but could not be parsed.
    };

    // Decoded GNSS fix, merged across GGA (altitude / satellites / fix quality) and RMC
    // (position / speed / course / validity).
    struct GNSSFix {
        bool valid = false;             // True if the last position-bearing sentence reported a fix.
        float latitude_deg = 0.0f;      // WGS84 degrees, north positive.
        float longitude_deg = 0.0f;     // WGS84 degrees, east positive.
        int32_t altitude_ft = 0;        // MSL altitude, feet (converted from meters).
        float heading_deg = 0.0f;       // Course over ground, degrees from true north.
        int32_t speed_kts = 0;          // Speed over ground, knots.
        uint8_t num_satellites = 0;     // Satellites used in the GGA fix.
        uint8_t fix_quality = 0;        // GGA fix quality (0 = no fix, 1 = GPS, 2 = DGPS, ...).
        bool utc_time_valid = false;    // True when a valid NMEA UTC time has been parsed.
        uint8_t utc_hour = 0;
        uint8_t utc_minute = 0;
        uint8_t utc_second = 0;
        uint16_t utc_millisecond = 0;
        uint32_t last_update_timestamp_ms = 0;  // Caller-supplied timestamp of last applied fix.
    };

    NMEAParser() = default;

    /**
     * Feed a single received byte into the parser.
     * @param[in] c Received character.
     * @retval SentenceType describing what (if anything) completed on this byte.
     */
    SentenceType IngestByte(char c);

    /**
     * @retval Const reference to the most recently merged fix.
     */
    const GNSSFix& fix() const { return fix_; }

    /**
     * Set the timestamp (ms) recorded on the fix when the next sentence is applied. The parser is
     * platform-agnostic, so the owning driver supplies the clock.
     */
    void SetTimestampMs(uint32_t timestamp_ms) { timestamp_ms_ = timestamp_ms; }

   private:
    // Parse the currently buffered line (line_buf_ / line_len_, '$' stripped, checksum validated)
    // by splitting on ',' and dispatching on the 3-char sentence type.
    SentenceType ParseLine();
    SentenceType ParseGGA(const char* const* fields, uint8_t num_fields);
    SentenceType ParseRMC(const char* const* fields, uint8_t num_fields);

    char line_buf_[kLineBufMaxLen];  // Accumulated sentence body (between '$' and '*').
    uint16_t line_len_ = 0;
    bool in_sentence_ = false;  // True once '$' seen, until terminator.

    // Checksum tracking: XOR of all chars between '$' and '*'.
    uint8_t running_checksum_ = 0;
    bool reading_checksum_ = false;  // True once '*' seen; next two chars are the checksum hex.
    char checksum_chars_[2];
    uint8_t checksum_char_count_ = 0;

    GNSSFix fix_;
    uint32_t timestamp_ms_ = 0;
};
