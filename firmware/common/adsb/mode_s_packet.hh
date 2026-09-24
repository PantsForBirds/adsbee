#ifndef _ADSB_PACKET_HH_
#define _ADSB_PACKET_HH_

#include <cstdint>

#include "buffer_utils.hh"
#include "unit_conversions.hh"

// Useful resource: https://mode-s.org/decode/content/ads-b/1-basics.html

class RawModeSPacket {
   public:
    static const uint16_t kMaxPacketLenWords32 = 4;
    static const uint16_t kSquitterPacketLenBits = 56;
    static const uint16_t kSquitterPacketLenBytes = kSquitterPacketLenBits / kBitsPerByte;
    static const uint16_t kSquitterPacketNumWords32 = 2;  // 56 bits = 1.75 words, round up to 2.
    static const uint16_t kExtendedSquitterPacketLenBits = 112;
    static const uint16_t kExtendedSquitterPacketLenBytes = kExtendedSquitterPacketLenBits / kBitsPerByte;
    static const uint16_t kExtendedSquitterPacketNumWords32 = 4;  // 112 bits = 3.5 words, round up to 4.

    RawModeSPacket(const char* rx_string, int16_t source_in = -1, int16_t sigs_dbm_in = INT16_MIN,
                   int16_t sigq_db_in = INT16_MIN, uint64_t mlat_48mhz_64bit_counts = 0);
    RawModeSPacket(uint32_t rx_buffer[kMaxPacketLenWords32], uint16_t rx_buffer_len_words32, int16_t source_in = -1,
                   int16_t sigs_dbm_in = INT16_MIN, int16_t sigq_db_in = INT16_MIN,
                   uint64_t mlat_48mhz_64bit_counts = 0);
    /**
     * Default constructor.
     */
    RawModeSPacket() {
        for (uint16_t i = 0; i < kMaxPacketLenWords32; i++) {
            buffer[i] = 0;
        }
    }

    /**
     * Rescales the 48MHz timestamp to a 12MHz timestamp.
     * @retval 12MHz timestamp.
     */
    uint64_t GetMLAT12MHzCounter() const { return (mlat_48mhz_64bit_counts >> 2) & 0xFFFFFFFFFFFF; }

    /**
     * Helper function that returns the timestamp in seconds. Used to abstract the MLAT timestamp resolution for
     * functions that don't care about it.
     * @return Timestamp in seconds.
     */
    uint64_t GetTimestampMs() const { return mlat_48mhz_64bit_counts / 48000; }

    /**
     * Print the buffer to a string.
     * @param[in] buf Buffer to print to.
     * @param[in] buf_len_bytes Length of the buffer, in characters.
     * @return Number of characters written to the buffer.
     */
    uint16_t PrintBuffer(char* buf, uint16_t buf_len_bytes) const;

    uint32_t buffer[kMaxPacketLenWords32] = {0};
    uint16_t buffer_len_bytes = 0;
    int8_t source = -1;                    // Source of the ADS-B packet (PIO state machine number).
    int16_t sigs_dbm = INT16_MIN;          // Signal strength, in dBm.
    int16_t sigq_db = INT16_MIN;           // Signal quality (dB above noise floor), in dB.
    uint64_t mlat_48mhz_64bit_counts = 0;  // High resolution MLAT counter.
};

class DecodedModeSPacket {
   public:
    static const uint16_t kMaxPacketLenWords32 = RawModeSPacket::kMaxPacketLenWords32;
    static const uint16_t kDFNumBits = 5;  // [1-5] Downlink Format bitlength.

    // Bits 1-5: Downlink Format (DF)
    enum DownlinkFormat {
        kDownlinkFormatInvalid = -1,
        // DF 0-11 = short messages (56 bits)
        kDownlinkFormatShortRangeAirToAirSurveillance = 0,  // All-call request from ground station.
        kDownlinkFormatAltitudeReply = 4,
        kDownlinkFormatIdentityReply = 5,
        kDownlinkFormatAllCallReply = 11,
        // DF 16-24 = long messages (112 bits)
        kDownlinkFormatLongRangeAirToAirSurveillance = 16,   // ACAS/TCAS message.
        kDownlinkFormatExtendedSquitter = 17,                // Aircraft (taxiing or airborne).
        kDownlinkFormatExtendedSquitterNonTransponder = 18,  // Surface vehicle or aircraft in special ground operation.
        kDownlinkFormatMilitaryExtendedSquitter = 19,        // Used by military aircraft, protocol is not public.
        // We don't currently do anything with CommB, CommC, CommD.
        kDownlinkFormatCommBAltitudeReply = 20,  // Data being sent as reply to request from Comm-B ground station.
        kDownlinkFormatCommBIdentityReply = 21,  // Reply to ground station via Comm-C (higher capacity than Comm-B).
        kDownlinkFormatCommDExtendedLengthMessage = 24
        // DF 1-3, 6-10, 11-15, 22-23 not used
    };

    // Bits 6-8 in DF=11 and DF=17-18: Transponder Capability (CA)
    enum Capability : uint8_t {
        kCALevel1Transponder = 0,
        // CA = 1-3 reserved for backwards compatibility.
        kCALevel2PlusTransponderOnSurfaceCanSetCA7 = 4,
        kCALevel2PlusTransponderAirborneCanSetCA7 = 5,
        kCALevel2PlusTransponderOnSurfaceOrAirborneCanSetCA7 = 6,
        kCADRNot0OrFSEquals2345OnSurfaceOrAirborne = 7  // Indicates aircraft is under enhanced surveillance by ATC.
    };

    // Constructors
    /**
     * DecodedModeSPacket constructor.
     * @param[in] rx_buffer Buffer to read from. Must be packed such that all 32 bits of each word are filled, with each
     * word left (MSb) aligned such that the total number of bits is 112. Words must be big-endian, with the MSb of the
     * first word being the oldest bit.
     * @param[in] rx_buffer_len_words32 Number of 32-bit words to read from the rx_buffer.
     * @param[in] sigs_dbm RSSI of the packet that was received, in dBm. Defaults to INT32_MIN if not set.
     * @param[in] mlat_48mhz_64bit_counts Counts of a 12MHz clock used for the 6-byte multilateration timestamp.
     */
    DecodedModeSPacket(uint32_t rx_buffer[kMaxPacketLenWords32], uint16_t rx_buffer_len, int16_t source = -1,
                       int32_t sigs_dbm = INT32_MIN, int32_t sigq_db = INT32_MIN, uint64_t mlat_48mhz_64bit_counts = 0);

    /**
     * DecodedModeSPacket constructor from string.
     * @param[in] rx_string String of nibbles as hex characters. Big-endian, MSB (oldest byte) first.
     * @param[in] sigs_dbm RSSI of the packet that was received, in dBm. Defaults to INT32_MIN if not set.
     * @param[in] mlat_12mhz_counts Counts of a 12MHz clock used for the 6-byte multilateration timestamp.
     */
    DecodedModeSPacket(const char* rx_string, int16_t source = -1, int32_t sigs_dbm = INT32_MIN, int32_t sigq_db = INT32_MIN,
                       uint64_t mlat_48mhz_64bit_counts = 0);

    /**
     * DecodedModeSPacket constructor from RawModeSPacket. Uses RawModeSPacket's implicit constructor.
     * @param[in] packet_in RawModeSPacket to use when creating the DecodedModeSPacket.
     */
    DecodedModeSPacket(const RawModeSPacket& packet_in);

    /**
     * Default constructor.
     */
    DecodedModeSPacket() : raw("") {};

    DownlinkFormat GetDownlinkFormatEnum();

    /**
     * Dumps the internal packet buffer to a destination and returns the number of bytes written.
     * @param[in] to_buffer Destination buffer, must be of length kMaxPacketLenWords32 or larger.
     * @retval Number of bytes written to destination buffer.
     */
    uint16_t DumpPacketBuffer(uint32_t to_buffer[kMaxPacketLenWords32]) const;
    uint16_t DumpPacketBuffer(uint8_t to_buffer[kMaxPacketLenWords32 * kBytesPerWord]) const;

    // Exposed for testing only.
    uint32_t Get24BitWordFromPacketBuffer(uint16_t first_bit_index) const {
        return GetNBitsFromWordBuffer(24, first_bit_index, raw.buffer);
    };

    /**
     * Calculates the 24-bit CRC checksum of the ADS-B packet and returns the checksum value. The returned
     * value should match the last 24-bits in the 112-bit ADS-B packet if the packet is valid.
     * @retval CRC checksum.
     */
    uint32_t CalculateCRC24(uint16_t packet_len_bits = RawModeSPacket::kExtendedSquitterPacketLenBits) const;

    bool is_valid = false;
    bool is_address_parity = false;
    RawModeSPacket raw;

    uint32_t icao_address = 0;
    uint16_t downlink_format = static_cast<uint16_t>(kDownlinkFormatInvalid);

    uint32_t parity_interrogator_id = 0;
    // Calculated CRC XORed with the received parity field. Zero for a valid packet; for address-parity downlink
    // formats this is the ICAO address (or interrogator ID); for other formats it is the CRC syndrome used for
    // single bit error correction.
    uint32_t crc_syndrome = 0;

   private:
    void ConstructModeSPacket();
};

class ModeSADSBPacket : public DecodedModeSPacket {
   public:
    static const uint16_t kMaxTCStrLen = 50;

    // Bitlengths of each field in the ADS-B frame. See Table 3.1 in The 1090MHz Riddle (Junzi Sun) pg. 35.
    static const uint16_t kCANumBits = 3;     // [6-8] Capability bitlength.
    static const uint16_t kICAONumBits = 24;  // [9-32] ICAO Address bitlength.
    static const uint16_t kMENumBits = 56;    // [33-88] Extended Squitter Message bitlength.
    static const uint16_t kTCNumBits = 5;     // [33-37] Type code bitlength. Not always included.
    static const uint16_t kPINumBits = 24;    // Parity / Interrogator ID bitlength.

    static const uint16_t kMEFirstBitIndex = kDFNumBits + kCANumBits + kICAONumBits;

    /**
     * Constructor. Can only create an ModeSADSBPacket from an existing DecodedModeSPacket, which is is referenced as
     * the parent of the ModeSADSBPacket. Think of this as a way to use the ModeSADSBPacket as a "window" into the
     * contents of the parent DecodedModeSPacket. The ModeSADSBPacket cannot exist without the parent
     * DecodedModeSPacket!
     */
    ModeSADSBPacket(const DecodedModeSPacket& decoded_packet) : DecodedModeSPacket(decoded_packet) {
        ConstructADSBPacket();
    };

    // Bits 6-8 [3]: Capability (CA) / Code Format (CF)
    // Bits 9-32 [24]: ICAO Aircraft Address (ICAO)
    // Bits 33-88 [56]: Message, Extended Squitter (ME)
    // (Bits 33-37 [5]): Type code (TC)
    enum TypeCode : uint8_t {
        kTypeCodeInvalid = 0,
        kTypeCodeAircraftID = 1,                 // 1–4     Aircraft identification
        kTypeCodeSurfacePosition = 5,            // 5–8     Surface position
        kTypeCodeAirbornePositionBaroAlt = 9,    // 9–18	Airborne position (w/Baro Altitude)
        kTypeCodeAirborneVelocities = 19,        // 19      Airborne velocities
        kTypeCodeAirbornePositionGNSSAlt = 20,   // 20–22	Airborne position (w/GNSS Height)
        kTypeCodeReserved = 23,                  // 23–27	Reserved
        kTypeCodeAircraftStatus = 28,            // 28      Aircraft status
        kTypeCodeTargetStateAndStatusInfo = 29,  // 29      Target state and status information
        kTypeCodeAircraftOperationStatus = 31    // 31      Aircraft operation status
    };
    // Bits 89-112 [24]: Parity / Interrogator ID (PI)

    // Subtype enums used for specific packet types (not instantiated as part of the ModeSADSBPacket class).
    // Airborne Velocitites (TC = 19)
    enum AirborneVelocitiesSubtype : uint8_t {
        kAirborneVelocitiesGroundSpeedSubsonic = 1,
        kAirborneVelocitiesGroundSpeedSupersonic = 2,
        kAirborneVelocitiesAirspeedSubsonic = 3,
        kAirborneVelocitiesAirspeedSupersonic = 4
    };

    // Operation Status (TC = 31)
    enum OperationStatusSubtype : uint8_t { kOperationStatusSubtypeAirborne = 0, kOperationStatusSubtypeSurface = 1 };

    TypeCode GetTypeCodeEnum() const;

    // Exposed for testing only.
    inline uint32_t GetNBitWordFromMessage(uint16_t n, uint16_t first_bit_index) const {
        return GetNBitsFromWordBuffer(n, kMEFirstBitIndex + first_bit_index, raw.buffer);
    };

    union {
        // Default to most basic capability.
        Capability capability = kCALevel1Transponder;  // Capability (CA), when DF=17.
        uint8_t code_format;                           // Code Format (CF), when DF=18.
    } ca_cf;

    TypeCode type_code = kTypeCodeInvalid;

    void ConstructADSBPacket();
};

class ModeSAllCallReplyPacket : public DecodedModeSPacket {
   public:
    ModeSAllCallReplyPacket(const DecodedModeSPacket& decoded_packet) : DecodedModeSPacket(decoded_packet) {
        capability = static_cast<Capability>(GetNBitsFromWordBuffer(3, 5, raw.buffer));
    }

    Capability capability = kCALevel1Transponder;  // Default to most basic capability.
};

class ModeSAltitudeReplyPacket : public DecodedModeSPacket {
   public:
    enum DownlinkRequest : uint8_t {
        kDownlinkRequestNone = 0b00000,
        kDownlinkRequestSendCommBMessage = 0b00001,
        kDownlinkRequestCommBBroadcastMessage1Available = 0b00100,
        kDownlinkRequestCommBBroadcastMessage2Available = 0b00101
    };

    enum UtilityMessageType : uint8_t {
        kUtilityMessageNoInformation = 0b00,
        kUtilityMessageCommBInterrogatorIdentifierCode = 0b10,
        kUtilityMessageCommCInterrogatorIdentifierCode = 0b10,
        kUtilityMessageCommDInterrogatorIdentifierCode = 0b11
    };

    ModeSAltitudeReplyPacket(const DecodedModeSPacket& decoded_packet);

    bool is_airborne = false;
    bool airborne_state_known = false;
    bool has_alert = false;
    bool has_ident = false;

    DownlinkRequest downlink_request = kDownlinkRequestNone;
    uint8_t utility_message = 0b0;
    UtilityMessageType utility_message_type = kUtilityMessageNoInformation;

    int32_t altitude_ft = -1;
};

class ModeSIdentityReplyPacket : public DecodedModeSPacket {
   public:
    enum DownlinkRequest : uint8_t {
        kDownlinkRequestNone = 0b00000,
        kDownlinkRequestSendCommBMessage = 0b00001,
        kDownlinkRequestCommBBroadcastMessage1Available = 0b00100,
        kDownlinkRequestCommBBroadcastMessage2Available = 0b00101
    };

    enum UtilityMessageType : uint8_t {
        kUtilityMessageNoInformation = 0b00,
        kUtilityMessageCommBInterrogatorIdentifierCode = 0b10,
        kUtilityMessageCommCInterrogatorIdentifierCode = 0b10,
        kUtilityMessageCommDInterrogatorIdentifierCode = 0b11
    };

    ModeSIdentityReplyPacket(const DecodedModeSPacket& decoded_packet);

    bool is_airborne = false;
    bool airborne_state_known = false;
    bool has_alert = false;
    bool has_ident = false;

    DownlinkRequest downlink_request = kDownlinkRequestNone;
    uint8_t utility_message = 0b0;
    UtilityMessageType utility_message_type = kUtilityMessageNoInformation;

    uint16_t squawk = 0xFFFF;
};

#endif /* _ADSB_PACKET_HH_ */
