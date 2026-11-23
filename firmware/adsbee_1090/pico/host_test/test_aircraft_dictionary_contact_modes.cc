#include "aircraft_dictionary.hh"
#include "aircraftjson_utils.hh"
#include "asterix_utils.hh"
#include "awb_test_utils.hh"
#include "gtest/gtest.h"
#include "hal_god_powers.hh"
#include "settings.hh"

// Frames heard from 1090 MHz TIS-B and ADS-R ground stations, and DF18 CF=0/1 frames re-addressed from the
// "1090 MHz Riddle" DF17 pair (52.2572 N, 3.9194 E) with a recomputed CRC.
namespace {
// DF18 CF=2 fine TIS-B, ICAO address A46E6A.
const char* kTISBICAOEven = "92A46E6A50074111A20EBC944469";
const char* kTISBICAOOdd = "92A46E6A500744A6ACBCB80F49A0";
const char* kTISBICAOVelocity = "92A46E6A9954710778280BB3B57B";
// DF17 for the same address.
const char* kDirectEven = "8DA46E6A58C382D690C8AC054280";
const char* kDirectOdd = "8DA46E6A58C386435CC412440BF1";
const char* kDirectVelocity = "8DA46E6A9910650CB004008574A3";
// DF18 CF=5 fine TIS-B with a non-ICAO address.
const char* kTISBOtherEven = "952B229A680B00E4382EA32ECF61";
const char* kTISBOtherOdd = "952B229A680B047A04DBF16B3247";
// DF18 CF=6 ADS-R, ICAO address A0964A.
const char* kADSRICAOEven = "96A0964A6013C0F52C0816715592";
const char* kADSRICAOOdd = "96A0964A6013C48AAEB6364DA5F3";
// DF18 CF=6 ADS-R with the ICAO/Mode A flag set (non-ICAO address 6A0EC9), and its identification.
const char* kADSROtherEven = "966A0EC9590740E3E230E9E05C33";
const char* kADSROtherOdd = "966A0EC959074479AEDE2B9DD0AA";
const char* kADSROtherVelocity = "966A0EC999843708E05800A55C23";
const char* kADSROtherID = "966A0EC9215864A0820820583840";
// DF18 CF=0 (ADF010) and CF=1 (ADF011), and DF17 for ADF011.
const char* kNonTransponderEven = "90ADF01058C382D690C8ACAFB0B7";
const char* kNonTransponderOdd = "90ADF01058C386435CC412EEF9C6";
const char* kAnonymousEven = "91ADF01158C382D690C8AC891BED";
const char* kAnonymousOdd = "91ADF01158C386435CC412C8529C";
const char* kADF011Even = "8DADF01158C382D690C8ACAC6660";
const char* kADF011Odd = "8DADF01158C386435CC412ED2F11";
// DF18 CF=4 TIS-B/ADS-R management message.
const char* kManagement = "940EB1743400000000000026C862";

constexpr float kRiddleLatDeg = 52.2572f;
constexpr float kRiddleLonDeg = 3.9194f;

class ContactModes : public ::testing::Test {
   protected:
    void SetUp() override {
        set_time_since_boot_ms(100e3);
        dictionary.SetCPRPositionFilterEnabled(false);  // Some cases move a target between distant positions.
    }

    // Ingests a frame at the current time, then moves time on by step_ms.
    void Ingest(const char* hex, uint32_t step_ms = 500) {
        DecodedModeSPacket packet((char*)hex);
        ASSERT_TRUE(packet.is_valid) << hex;
        packet.raw.mlat_48mhz_64bit_counts = static_cast<uint64_t>(get_time_since_boot_ms()) * 48000;
        EXPECT_TRUE(dictionary.IngestDecodedModeSPacket(packet)) << hex;
        inc_time_since_boot_ms(step_ms);
    }

    ModeSAircraft* Get(uint32_t address, ModeSAircraft::AddressType type) {
        return dictionary.GetAircraftPtr<ModeSAircraft>(ModeSAircraft::AddressToUID(address, type), false);
    }

    std::string JSONType(const ModeSAircraft& aircraft) {
        char buf[kAircraftJSONMessageStrMaxLen];
        int16_t len = WriteAircraftJSONModeSAircraftStr(buf, aircraft);
        std::string json(buf, len > 0 ? len : 0);
        size_t start = json.find("\"type\":\"") + 8;
        return json.substr(start, json.find('"', start) - start);
    }

    AircraftDictionary dictionary;
};
}  // namespace

TEST_F(ContactModes, AddressTypeOfEachControlField) {
    const struct {
        const char* hex;
        ModeSAircraft::AddressType type;
    } kCases[] = {
        {kDirectEven, ModeSAircraft::kAddressTypeADSBICAO},
        {kNonTransponderEven, ModeSAircraft::kAddressTypeADSBICAONonTransponder},
        {kAnonymousEven, ModeSAircraft::kAddressTypeADSBOther},
        {kTISBICAOEven, ModeSAircraft::kAddressTypeTISBICAO},
        {kTISBOtherEven, ModeSAircraft::kAddressTypeTISBOther},
        {kADSRICAOEven, ModeSAircraft::kAddressTypeADSRICAO},
        {kADSROtherEven, ModeSAircraft::kAddressTypeADSROther},
        {kADSROtherVelocity, ModeSAircraft::kAddressTypeADSROther},
        {kADSROtherID, ModeSAircraft::kAddressTypeADSRICAO},  // Identification has no ICAO/Mode A flag.
        {kManagement, ModeSAircraft::kAddressTypeInvalid},
    };
    for (const auto& c : kCases) {
        DecodedModeSPacket packet((char*)c.hex);
        ASSERT_TRUE(packet.is_valid) << c.hex;
        EXPECT_EQ(ModeSAircraft::GetAddressType(ModeSADSBPacket(packet)), c.type) << c.hex;
    }
}

TEST_F(ContactModes, NonTransponderIsAnICAOTarget) {
    Ingest(kNonTransponderEven);
    Ingest(kNonTransponderOdd);
    ModeSAircraft* aircraft = Get(0xADF010, ModeSAircraft::kAddressTypeADSBICAONonTransponder);
    ASSERT_TRUE(aircraft);
    EXPECT_EQ(aircraft->GetUID(), Aircraft::ICAOToUID(0xADF010, Aircraft::kAircraftTypeModeS));
    ASSERT_TRUE(aircraft->HasBitFlag(ModeSAircraft::kBitFlagPositionValid));
    EXPECT_NEAR(AWBToDeg(aircraft->latitude_awb), kRiddleLatDeg, 0.03);
    EXPECT_NEAR(AWBToDeg(aircraft->longitude_awb), kRiddleLonDeg, 0.03);  // Depends on which frame is newest.
    EXPECT_EQ(aircraft->GetAddressType(), ModeSAircraft::kAddressTypeADSBICAONonTransponder);
    EXPECT_TRUE(aircraft->HasBitFlag(ModeSAircraft::kBitFlagIsNonTransponder));
    EXPECT_EQ(JSONType(*aircraft), "adsb_icao_nt");
}

TEST_F(ContactModes, NonICAOAddressesDontCollideWithICAO) {
    Ingest(kADF011Even);
    Ingest(kAnonymousEven);
    Ingest(kADF011Odd);
    Ingest(kAnonymousOdd);
    EXPECT_EQ(dictionary.GetNumAircraft(), 2);
    ModeSAircraft* icao = Get(0xADF011, ModeSAircraft::kAddressTypeADSBICAO);
    ModeSAircraft* anonymous = Get(0xADF011, ModeSAircraft::kAddressTypeADSBOther);
    ASSERT_TRUE(icao && anonymous);
    EXPECT_NE(icao->GetUID(), anonymous->GetUID());
    EXPECT_EQ(anonymous->icao_address, 0xADF011u);
    EXPECT_EQ(icao->GetAddressType(), ModeSAircraft::kAddressTypeADSBICAO);
    EXPECT_EQ(anonymous->GetAddressType(), ModeSAircraft::kAddressTypeADSBOther);
    EXPECT_TRUE(anonymous->HasBitFlag(ModeSAircraft::kBitFlagPositionValid));
    EXPECT_EQ(JSONType(*anonymous), "adsb_other");
    // Non-ICAO entries are reported alongside the ICAO one.
    EXPECT_TRUE(dictionary.IsPreferredReportForAddress(icao->GetUID(), get_time_since_boot_ms()));
    EXPECT_TRUE(dictionary.IsPreferredReportForAddress(anonymous->GetUID(), get_time_since_boot_ms()));
}

TEST_F(ContactModes, TISBICAOTarget) {
    Ingest(kTISBICAOEven, 0);
    Ingest(kTISBICAOOdd);
    Ingest(kTISBICAOVelocity);
    ModeSAircraft* aircraft = Get(0xA46E6A, ModeSAircraft::kAddressTypeTISBICAO);
    ASSERT_TRUE(aircraft);
    ASSERT_TRUE(aircraft->HasBitFlag(ModeSAircraft::kBitFlagPositionValid));
    EXPECT_EQ(aircraft->GetAddressType(), ModeSAircraft::kAddressTypeTISBICAO);
    EXPECT_EQ(aircraft->velocity_address_type, ModeSAircraft::kAddressTypeTISBICAO);
    EXPECT_TRUE(aircraft->HasBitFlag(ModeSAircraft::kBitFlagHorizontalSpeedValid));
    EXPECT_FALSE(aircraft->NICBitIsValid(ADSBTypes::kNICBitB));  // That bit is the ICAO/Mode A flag in TIS-B.
    EXPECT_EQ(JSONType(*aircraft), "tisb_icao");
}

TEST_F(ContactModes, TISBNonICAOTarget) {
    Ingest(kTISBOtherEven, 0);
    Ingest(kTISBOtherOdd);
    EXPECT_EQ(dictionary.GetNumAircraft(), 1);
    EXPECT_FALSE(Get(0x2B229A, ModeSAircraft::kAddressTypeTISBICAO));
    ModeSAircraft* aircraft = Get(0x2B229A, ModeSAircraft::kAddressTypeTISBOther);
    ASSERT_TRUE(aircraft);
    EXPECT_TRUE(aircraft->HasBitFlag(ModeSAircraft::kBitFlagPositionValid));
    EXPECT_EQ(JSONType(*aircraft), "tisb_other");
}

TEST_F(ContactModes, ADSRICAOTarget) {
    Ingest(kADSRICAOEven, 0);
    Ingest(kADSRICAOOdd);
    ModeSAircraft* aircraft = Get(0xA0964A, ModeSAircraft::kAddressTypeADSRICAO);
    ASSERT_TRUE(aircraft);
    EXPECT_TRUE(aircraft->HasBitFlag(ModeSAircraft::kBitFlagPositionValid));
    EXPECT_EQ(JSONType(*aircraft), "adsr_icao");
}

TEST_F(ContactModes, ADSRNonICAOTargetKeepsItsIdentification) {
    Ingest(kADSROtherEven, 0);
    Ingest(kADSROtherOdd);
    Ingest(kADSROtherVelocity);
    Ingest(kADSROtherID);
    // The identification message joins the non-ICAO entry instead of starting an ICAO one.
    EXPECT_EQ(dictionary.GetNumAircraft(), 1);
    ModeSAircraft* aircraft = Get(0x6A0EC9, ModeSAircraft::kAddressTypeADSROther);
    ASSERT_TRUE(aircraft);
    EXPECT_TRUE(aircraft->HasBitFlag(ModeSAircraft::kBitFlagPositionValid));
    EXPECT_TRUE(aircraft->HasBitFlag(ModeSAircraft::kBitFlagHorizontalSpeedValid));
    EXPECT_STRNE(aircraft->callsign, "?");
    EXPECT_EQ(JSONType(*aircraft), "adsr_other");
}

TEST_F(ContactModes, DirectADSBHoldsOffTISB) {
    Ingest(kDirectEven, 0);
    Ingest(kDirectOdd);
    Ingest(kDirectVelocity);
    ModeSAircraft* aircraft = Get(0xA46E6A, ModeSAircraft::kAddressTypeADSBICAO);
    ASSERT_TRUE(aircraft);
    ASSERT_TRUE(aircraft->HasBitFlag(ModeSAircraft::kBitFlagPositionValid));
    int32_t direct_speed_kts = aircraft->speed_kts;

    // TIS-B for the same aircraft is ignored while the direct position and velocity are fresh.
    Ingest(kTISBICAOEven, 0);
    Ingest(kTISBICAOOdd);
    Ingest(kTISBICAOVelocity);
    EXPECT_EQ(dictionary.GetNumAircraft(), 1);
    EXPECT_NEAR(AWBToDeg(aircraft->latitude_awb), kRiddleLatDeg, 0.03);
    EXPECT_EQ(aircraft->speed_kts, direct_speed_kts);
    EXPECT_EQ(aircraft->GetAddressType(), ModeSAircraft::kAddressTypeADSBICAO);
    EXPECT_EQ(JSONType(*aircraft), "adsb_icao");

    // Once the direct data is stale, TIS-B takes over.
    inc_time_since_boot_ms(ModeSAircraft::kAddressTypeHoldMs);
    Ingest(kTISBICAOEven, 0);
    Ingest(kTISBICAOOdd);
    Ingest(kTISBICAOVelocity);
    EXPECT_GT(AWBToDeg(aircraft->latitude_awb), 20.0f);
    EXPECT_LT(AWBToDeg(aircraft->longitude_awb), -60.0f);
    EXPECT_NE(aircraft->speed_kts, direct_speed_kts);
    EXPECT_EQ(aircraft->GetAddressType(), ModeSAircraft::kAddressTypeTISBICAO);
    EXPECT_TRUE(aircraft->HasBitFlag(ModeSAircraft::kBitFlagIsNonTransponder));

    // Direct ADS-B takes the target back at once.
    Ingest(kDirectEven, 0);
    Ingest(kDirectOdd);
    EXPECT_NEAR(AWBToDeg(aircraft->latitude_awb), kRiddleLatDeg, 0.03);
    EXPECT_EQ(aircraft->GetAddressType(), ModeSAircraft::kAddressTypeADSBICAO);
    EXPECT_FALSE(aircraft->HasBitFlag(ModeSAircraft::kBitFlagIsNonTransponder));
}

TEST_F(ContactModes, CPRPacketsFromDifferentSourcesArentPaired) {
    Ingest(kDirectEven);
    Ingest(kTISBICAOOdd);
    ModeSAircraft* aircraft = Get(0xA46E6A, ModeSAircraft::kAddressTypeADSBICAO);
    ASSERT_TRUE(aircraft);
    EXPECT_FALSE(aircraft->HasBitFlag(ModeSAircraft::kBitFlagPositionValid));
    Ingest(kTISBICAOEven);
    EXPECT_TRUE(aircraft->HasBitFlag(ModeSAircraft::kBitFlagPositionValid));
    EXPECT_EQ(aircraft->GetAddressType(), ModeSAircraft::kAddressTypeTISBICAO);
}

TEST_F(ContactModes, ManagementMessagesAddNoTarget) {
    Ingest(kManagement);
    EXPECT_EQ(dictionary.GetNumAircraft(), 0);
}

TEST_F(ContactModes, ASTERIXMarksRebroadcasts) {
    Ingest(kTISBOtherEven, 0);
    Ingest(kTISBOtherOdd);
    Ingest(kADSRICAOEven, 0);
    Ingest(kADSRICAOOdd);
    Ingest(kNonTransponderEven, 0);
    Ingest(kNonTransponderOdd);
    ASTERIXReporter::ReportContext ctx;
    ctx.timestamp_ms = get_time_since_boot_ms();
    ASTERIXReporter::CAT021TargetReport r;

    ASSERT_TRUE(ASTERIXReporter::BuildCAT021Report(r, *Get(0x2B229A, ModeSAircraft::kAddressTypeTISBOther), ctx));
    EXPECT_EQ(r.address_type, ASTERIXReporter::kAddressTypeAnonymous);
    EXPECT_EQ(r.service_id, ASTERIXReporter::kServiceID1090ESRebroadcast);
    EXPECT_FALSE(r.has_mops_version);

    ASSERT_TRUE(ASTERIXReporter::BuildCAT021Report(r, *Get(0xA0964A, ModeSAircraft::kAddressTypeADSRICAO), ctx));
    EXPECT_EQ(r.address_type, ASTERIXReporter::kAddressTypeICAO);
    EXPECT_EQ(r.service_id, ASTERIXReporter::kServiceID1090ESRebroadcast);

    ASSERT_TRUE(
        ASTERIXReporter::BuildCAT021Report(r, *Get(0xADF010, ModeSAircraft::kAddressTypeADSBICAONonTransponder), ctx));
    EXPECT_EQ(r.address_type, ASTERIXReporter::kAddressTypeICAO);
    EXPECT_EQ(r.service_id, ASTERIXReporter::kServiceID1090ES);
}

// UAT frames (with FEC) heard from UAT ground stations: TIS-B/ADS-R with an ICAO address and TIS-B track files.
TEST_F(ContactModes, UATRebroadcastsKeyOnAddressQualifier) {
    const struct {
        const char* hex;
        uint32_t address;
        UATAircraft::AddressQualifier qualifier;
        const char* type_str;
    } kCases[] = {
        {"0A4245D8343609535DFA39880018B9C31F17B14F4EE6C40B1A952280003C2000000089B09B4752D90562777609038245", 0x4245D8,
         UATAircraft::kTISBTargetWithICAO24BitAddress, "tisb_icao"},
        {"0B2B0D4A36E4235198E006D610C428E03105EDED2DED2D0A9A620300000000000000AA16A949E2D39779A302AE435953", 0x2B0D4A,
         UATAircraft::kTISBTargetWithTrackFileIdentifier, "tisb_trackfile"},
    };
    for (const auto& c : kCases) {
        DecodedUATADSBPacket packet(c.hex);
        ASSERT_TRUE(packet.is_valid) << c.hex;
        EXPECT_TRUE(dictionary.IngestDecodedUATADSBPacket(packet));
        UATAircraft* aircraft = dictionary.GetAircraftPtr<UATAircraft>(
            Aircraft::ICAOToUID(c.address | (c.qualifier << Aircraft::kAddressQualifierBitShift),
                                Aircraft::kAircraftTypeUAT),
            false);
        ASSERT_TRUE(aircraft) << c.hex;
        EXPECT_EQ(aircraft->GetAddressQualifier(), c.qualifier);
        char buf[kAircraftJSONMessageStrMaxLen];
        WriteAircraftJSONUATAircraftStr(buf, *aircraft);
        EXPECT_NE(std::string(buf).find(std::string("\"type\":\"") + c.type_str + "\""), std::string::npos) << buf;
    }
}

TEST(Ownship, OnlyEntriesWithOwnshipsICAOAddress) {
    const uint32_t own = 0xADF011;
    EXPECT_TRUE(AircraftDictionary::IsOwnship(Aircraft::ICAOToUID(own, Aircraft::kAircraftTypeModeS), own));
    EXPECT_TRUE(
        AircraftDictionary::IsOwnship(ModeSAircraft::AddressToUID(own, ModeSAircraft::kAddressTypeTISBICAO), own));
    EXPECT_TRUE(
        AircraftDictionary::IsOwnship(ModeSAircraft::AddressToUID(own, ModeSAircraft::kAddressTypeADSRICAO), own));
    // Non-ICAO addresses with the same 24 bits are other aircraft.
    for (ModeSAircraft::AddressType type :
         {ModeSAircraft::kAddressTypeADSBOther, ModeSAircraft::kAddressTypeADSROther,
          ModeSAircraft::kAddressTypeTISBTrackFile, ModeSAircraft::kAddressTypeTISBOther}) {
        EXPECT_FALSE(AircraftDictionary::IsOwnship(ModeSAircraft::AddressToUID(own, type), own)) << int(type);
    }
    auto uat_uid = [](uint32_t address, UATAircraft::AddressQualifier q) {
        return Aircraft::ICAOToUID(address | (uint32_t(q) << Aircraft::kAddressQualifierBitShift),
                                   Aircraft::kAircraftTypeUAT);
    };
    EXPECT_TRUE(AircraftDictionary::IsOwnship(uat_uid(own, UATAircraft::kADSBTargetWithICAO24BitAddress), own));
    EXPECT_TRUE(AircraftDictionary::IsOwnship(uat_uid(own, UATAircraft::kTISBTargetWithICAO24BitAddress), own));
    EXPECT_FALSE(
        AircraftDictionary::IsOwnship(uat_uid(own, UATAircraft::kADSBTargetWithSelfAssignedTemporaryAddress), own));
    EXPECT_FALSE(AircraftDictionary::IsOwnship(uat_uid(own, UATAircraft::kTISBTargetWithTrackFileIdentifier), own));
    // Remote ID UIDs never carry an ICAO address.
    EXPECT_FALSE(AircraftDictionary::IsOwnship(Aircraft::ICAOToUID(own, Aircraft::kAircraftTypeRemoteID), own));
    EXPECT_FALSE(AircraftDictionary::IsOwnship(Aircraft::ICAOToUID(own + 1, Aircraft::kAircraftTypeModeS), own));
    // Without ownship nothing matches, not even address 000000 (a GNSS ownship's GDL90 participant address).
    EXPECT_FALSE(AircraftDictionary::IsOwnship(Aircraft::ICAOToUID(0, Aircraft::kAircraftTypeModeS),
                                               AircraftDictionary::kNoOwnshipICAOAddress));
    EXPECT_FALSE(AircraftDictionary::IsOwnship(Aircraft::ICAOToUID(0xFFFFFF, Aircraft::kAircraftTypeModeS),
                                               AircraftDictionary::kNoOwnshipICAOAddress));
}

TEST(Ownship, ICAOAddressOnlyFromATrackedAircraftPosition) {
    SettingsManager::RxPosition rx = {};
    rx.icao_address = 0xADF011;
    rx.source = SettingsManager::RxPosition::kPositionSourceAircraftMatchingICAO;
    EXPECT_EQ(AircraftDictionary::OwnshipICAOAddress(rx, true), 0xADF011u);
    EXPECT_EQ(AircraftDictionary::OwnshipICAOAddress(rx, false), AircraftDictionary::kNoOwnshipICAOAddress);
    for (auto source :
         {SettingsManager::RxPosition::kPositionSourceGNSS, SettingsManager::RxPosition::kPositionSourceFixed,
          SettingsManager::RxPosition::kPositionSourceLowestAircraft}) {
        rx.source = source;
        EXPECT_EQ(AircraftDictionary::OwnshipICAOAddress(rx, true), AircraftDictionary::kNoOwnshipICAOAddress)
            << int(source);
    }
}

TEST_F(ContactModes, ReportedTrafficLeavesOutOwnship) {
    Ingest(kADF011Even);
    Ingest(kAnonymousEven);
    Ingest(kADF011Odd);
    Ingest(kAnonymousOdd);
    ModeSAircraft* icao = Get(0xADF011, ModeSAircraft::kAddressTypeADSBICAO);
    ModeSAircraft* anonymous = Get(0xADF011, ModeSAircraft::kAddressTypeADSBOther);
    ASSERT_TRUE(icao && anonymous);
    const uint32_t now_ms = get_time_since_boot_ms();
    const AircraftReportFilter filter;
    auto reported = [&](ModeSAircraft* a, uint32_t own) {
        auto itr = dictionary.dict.find(a->GetUID());
        return dictionary.IsReportedTraffic(itr->first, itr->second, now_ms, filter, own);
    };
    EXPECT_TRUE(reported(icao, AircraftDictionary::kNoOwnshipICAOAddress));
    EXPECT_TRUE(reported(anonymous, AircraftDictionary::kNoOwnshipICAOAddress));
    EXPECT_FALSE(reported(icao, 0xADF011));
    EXPECT_TRUE(reported(anonymous, 0xADF011));  // Another aircraft with the same 24 bits.
}
