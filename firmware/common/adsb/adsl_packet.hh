#pragma once

#include <cstdint>

/**
 * ADS-L packets on the M-band (EASA ADS-L 4 SRD-860 Issue 2): 868.2 / 868.4 MHz, 100 kchip/s 2-GFSK, +/-50 kHz.
 *
 * On air: preamble (not Manchester coded), then Manchester coded ('1' = 01, '0' = 10) sync word 0x724B, length byte,
 * network header byte, ADS-L data (XXTEA scrambled unless key index 3), CRC-24 (Mode S polynomial, MSB first). An
 * optional signature follows without Manchester coding and is not counted in the length byte; receivers may ignore it.
 * Multi-byte fields are little endian with bit fields packed LSB first (spec B.3).
 */
class RawADSLPacket {
   public:
    // Manchester coding of sync word 0x724B, the 32 chips the radio syncs on.
    static constexpr uint32_t kSyncWordChips = 0x95A69A65;
    static constexpr uint16_t kChipBytesPerByte = 2;

    // Length byte + network header + ADS-L data + CRC. Largest is 1 + 31 bytes; a Traffic packet is 25 bytes.
    static constexpr uint16_t kMaxPacketLenBytes = 32;
    static constexpr uint16_t kMaxChipBytes = kMaxPacketLenBytes * kChipBytesPerByte;
    static constexpr uint16_t kCRCNumBytes = 3;
    static constexpr uint16_t kNetworkHeaderNumBytes = 1;
    // Length byte values: network header + ADS-L data (5 byte header + 3 + 4N byte payload) + CRC.
    static constexpr uint8_t kMinLenField = kNetworkHeaderNumBytes + 8 + kCRCNumBytes;  // 12.
    static constexpr uint8_t kMaxLenField = kMaxPacketLenBytes - 1;
    static constexpr uint8_t kTrafficLenField = 24;
    static constexpr uint8_t kStatusLenField = 16;

    // Manchester pairs that read 00 or 11 are erasures. Up to this many are repaired by trying both values against
    // the CRC (2^n CRC checks).
    static constexpr uint16_t kMaxErasures = 6;

    enum Channel : uint8_t { kChannel868_2MHz = 0, kChannel868_4MHz = 1, kChannelUnknown = 0xFF };

    RawADSLPacket() {}

    /**
     * Builds a packet from on-air bytes (length byte first). Used by tests and when reading packets from text.
     */
    RawADSLPacket(const uint8_t *buf, uint16_t buf_len_bytes, int16_t sigs_dbm_in = INT16_MIN,
                  int16_t sigq_bits_in = INT16_MIN, uint64_t mlat_48mhz_64bit_counts_in = 0);

    /**
     * Number of chip bytes to receive after the sync word, from the first two chip bytes (the Manchester coded length
     * byte). Lets the radio cut reception short while the packet is still arriving.
     * @retval Chip bytes for the whole packet, or 0 if the length byte has an erasure or is not a supported length.
     */
    static uint16_t ChipBytesFromLengthChips(uint8_t chips_msb, uint8_t chips_lsb);

    /**
     * Manchester decodes chips received after the sync word, repairs up to kMaxErasures erasures against the CRC and
     * fills this packet.
     * @param[in] chips Chip bytes, MSB first, starting right after the sync word.
     * @param[in] num_chip_bytes Number of chip bytes available.
     * @retval True if the packet passed the CRC.
     */
    bool FromChips(const uint8_t *chips, uint16_t num_chip_bytes);

    /**
     * @retval True if buffer holds a whole packet with a matching CRC.
     */
    bool CRCIsValid() const;

    uint64_t GetMLAT12MHzCounter() const { return (mlat_48mhz_64bit_counts >> 2) & 0xFFFFFFFFFFFF; }
    uint64_t GetTimestampMs() const { return mlat_48mhz_64bit_counts / 48000; }

    uint8_t buffer[kMaxPacketLenBytes] = {0};  // On-air bytes after the sync word, length byte first.
    uint8_t buffer_len_bytes = 0;
    uint8_t channel = kChannelUnknown;      // Channel enum.
    int16_t sigs_dbm = INT16_MIN;           // Signal strength, in dBm.
    int16_t sigq_bits = INT16_MIN;          // Manchester erasures repaired (0 = best).
    uint16_t reserved = 0;                  // Keeps mlat_48mhz_64bit_counts 8-byte aligned.
    uint64_t mlat_48mhz_64bit_counts = 0;  // Receive timestamp.
};
static_assert(sizeof(RawADSLPacket) % 4 == 0, "RawADSLPacket must be word-aligned for CompositeArray access.");

/**
 * Fields of a CRC-checked, descrambled ADS-L packet. Traffic and Status payloads are parsed; other payload types keep
 * only the network and ADS-L headers.
 */
class DecodedADSLPacket {
   public:
    static constexpr uint16_t kADSLHeaderNumBytes = 5;
    static constexpr uint16_t kTrafficPayloadNumBytes = 15;
    static constexpr uint16_t kStatusPayloadNumBytes = 7;
    static constexpr uint16_t kMaxDataNumBytes =
        RawADSLPacket::kMaxPacketLenBytes - 1 - RawADSLPacket::kNetworkHeaderNumBytes - RawADSLPacket::kCRCNumBytes;
    static constexpr uint8_t kMaxSupportedProtocolVersion = 1;  // Issue 1 = 0, Issue 2 = 1.

    static constexpr uint8_t kKeyIndexScrambled = 0;  // XXTEA, all-zero key.
    static constexpr uint8_t kKeyIndexClear = 3;

    enum PayloadType : uint8_t {
        kPayloadTypeTraffic = 2,
        kPayloadTypeStatus = 3,
        kPayloadTypeTrafficUplink = 4,
        kPayloadTypeFISBUplink = 5,
        kPayloadTypeRemoteID = 6,
    };

    // Address Mapping Table (spec F.2.2).
    enum AddressMapping : uint8_t {
        kAddressMappingRandom = 0,
        kAddressMappingICAO = 5,
        kAddressMappingFLARM = 6,
        kAddressMappingOGN = 7,
        kAddressMappingFANET = 8,
        kAddressMappingManufacturerPage0 = 9,
    };

    enum FlightState : uint8_t {
        kFlightStateUndefined = 0,
        kFlightStateOnGround = 1,
        kFlightStateAirborne = 2,
    };

    enum AircraftCategory : uint8_t {
        kCategoryNoInfo = 0,
        kCategoryLightFixedWing = 1,
        kCategorySmallToHeavyFixedWing = 2,
        kCategoryLightRotorcraft = 3,
        kCategoryGlider = 4,
        kCategoryLighterThanAir = 5,
        kCategoryUltralight = 6,
        kCategoryParaglider = 7,
        kCategoryParachutist = 8,
        kCategoryEVTOL = 9,
        kCategoryGyrocopter = 10,
        kCategoryUASOpen = 11,
        kCategoryUASSpecific = 12,
        kCategoryUASCertified = 13,
        kCategoryModelPlane = 14,
        kCategoryHeavyRotorcraft = 15,
        kCategoryHangGlider = 16,
        kCategoryParamotor = 17,
    };

    enum EmergencyStatus : uint8_t {
        kEmergencyUndefined = 0,
        kEmergencyNone = 1,
        kEmergencyGeneral = 2,
        kEmergencyMedical = 3,
        kEmergencyNoCommunications = 4,
        kEmergencyUnlawfulInterference = 5,
        kEmergencyDownedAircraft = 6,
    };

    // Raw field values that mean "not available".
    static constexpr uint32_t kLatLonInvalid = 0x800000;
    static constexpr uint16_t kGroundSpeedInvalid = 0xFF;
    static constexpr uint16_t kAltitudeInvalid = 0x3FFF;
    static constexpr uint16_t kVerticalRateInvalid = 0x1FF;

    static constexpr int32_t kLatitudeLSBPerDeg = 93206;
    static constexpr int32_t kLongitudeLSBPerDeg = 46603;
    static constexpr int32_t kAltitudeOffsetM = 320;

    struct Traffic {
        uint8_t timestamp_quarter_s = 0;  // Quarter seconds since the full hour, modulo 60.
        FlightState flight_state = kFlightStateUndefined;
        AircraftCategory category = kCategoryNoInfo;
        EmergencyStatus emergency = kEmergencyUndefined;
        bool position_valid = false;
        float latitude_deg = 0.0f;
        float longitude_deg = 0.0f;
        bool ground_speed_valid = false;
        float ground_speed_mps = 0.0f;
        bool altitude_valid = false;
        int32_t altitude_wgs84_m = 0;  // Above the WGS-84 ellipsoid.
        bool vertical_rate_valid = false;
        float vertical_rate_mps = 0.0f;  // Positive up.
        bool track_valid = false;        // Invalid when ground speed is zero or invalid.
        float track_deg = 0.0f;          // True, clockwise from north.
        uint8_t source_integrity_level = 0;
        uint8_t design_assurance = 0;
        uint8_t navigation_integrity = 0;
        uint8_t horizontal_accuracy = 0;  // NACp equivalent.
        uint8_t vertical_accuracy = 0;    // GVA equivalent.
        uint8_t velocity_accuracy = 0;    // NACv equivalent.
    };

    struct Status {
        uint8_t max_protocol_version = 0;
        uint8_t m_band_rx = 0;
        uint8_t o_band_ldr_rx = 0;
        uint8_t o_band_hdr_rx = 0;
        uint8_t ads_l_mobile = 0;
        uint8_t e_conspicuity_rx = 0;  // Bitmap: FLARM, PilotAware, FANET, OGN, 1090, UAT, Mode S.
        uint8_t e_conspicuity_tx = 0;  // Bitmap: FLARM, PilotAware, FANET, OGN, 1090, UAT.
        uint8_t xpdr = 0;
        bool traffic_uplink_client = false;
        bool fisb_uplink_client = false;
    };

    DecodedADSLPacket() {}
    explicit DecodedADSLPacket(const RawADSLPacket &packet_in);

    /**
     * Exponential field decode (spec G.1.6). Unsigned: <exponent:2><base:n>. Signed: <sign:1><exponent:2><base:n>.
     */
    static inline uint32_t ExpDecodeUnsigned(uint32_t code, uint16_t n) {
        uint32_t exponent = code >> n;
        uint32_t base = code & ((1u << n) - 1);
        return (((1u << n) + base) << exponent) - (1u << n);
    }
    static inline int32_t ExpDecodeSigned(uint32_t code, uint16_t n) {
        int32_t value = static_cast<int32_t>(ExpDecodeUnsigned(code & ((1u << (n + 2)) - 1), n));
        return (code >> (n + 2)) & 1 ? -value : value;
    }

    /**
     * Reads a little endian, LSB first bit field (spec B.3).
     */
    static uint32_t GetBits(const uint8_t *buf, uint16_t offset_bits, uint16_t width_bits);

    /**
     * XXTEA with 6 rounds (spec E.2). num_words is the ADS-L data length in 32-bit words; data is little endian.
     */
    static void XXTEAEncrypt(uint32_t *data, uint16_t num_words, const uint32_t key[4]);
    static void XXTEADecrypt(uint32_t *data, uint16_t num_words, const uint32_t key[4]);

    /**
     * @retval 30 bit sender address: <AMT:6><address:24>, as a 32 bit integer with the AMT in bits 24-29.
     */
    inline uint32_t GetAMTAndAddress() const { return (static_cast<uint32_t>(address_mapping) << 24) | address; }

    RawADSLPacket raw;
    bool is_valid = false;  // CRC passed, supported version and ADS-L data length, payload parsed.

    // Network header.
    uint8_t protocol_version = 0;
    bool has_signature = false;
    uint8_t key_index = 0;
    bool uses_fec = false;

    // ADS-L header.
    uint8_t payload_type = 0;
    uint8_t address_mapping = 0;  // AddressMapping.
    uint32_t address = 0;         // 24 bits.
    bool relayed = false;

    uint8_t data[kMaxDataNumBytes] = {0};  // Descrambled ADS-L data.
    uint8_t data_len_bytes = 0;

    Traffic traffic;  // Valid if payload_type == kPayloadTypeTraffic.
    Status status;    // Valid if payload_type == kPayloadTypeStatus.

   private:
    void ParseTraffic();
    void ParseStatus();
};
