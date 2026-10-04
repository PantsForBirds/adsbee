#include "settings.hh"

// These strings are initialized here since they can't be initialized in settings.hh because they are static.
// The ESP32 and RP2040 have separate settings.cpp files, but want to share these static string definitions.
const char SettingsManager::kConsoleLogLevelStrs[SettingsManager::LogLevel::kNumLogLevels]
                                                [SettingsManager::kConsoleLogLevelStrMaxLen] = {"SILENT", "ERRORS",
                                                                                                "WARNINGS", "INFO"};
const char SettingsManager::kSerialInterfaceStrs[SettingsManager::SerialInterface::kNumSerialInterfaces]
                                                [SettingsManager::kSerialInterfaceStrMaxLen] = {"CONSOLE"};
const char
    SettingsManager::kReportingProtocolStrs[SettingsManager::ReportingProtocol::kNumProtocols]
                                           [SettingsManager::kReportingProtocolStrMaxLen] = {
                                               "NONE",  "RAW",      "BEAST",    "BEAST_NO_UAT", "BEAST_NO_UAT_UPLINK",
                                               "CSBEE", "MAVLINK1", "MAVLINK2", "GDL90", "AIRCRAFT_JSON",
                                               "GDL90_NO_UAT_UPLINK"};

const char SettingsManager::kSubGHzModeStrs[SettingsManager::kNumSubGHzRadioModes]
                                           [SettingsManager::kSubGHzModeStrMaxLen] = {
                                               "UAT_RX",            // UAT ADS-B + ground uplink.
                                               "UAT_RX_NO_UPLINK",  // UAT ADS-B only.
};

const char SettingsManager::kR1090PreambleModeStrs[SettingsManager::kNumR1090PreambleModes]
                                                  [SettingsManager::kR1090PreambleModeStrMaxLen] = {
                                                      "DF17",           // Preamble chips 8-15 + DF17 header bits.
                                                      "MODE_S",         // Standard preamble, raised AGC trigger.
                                                      "MODE_S_STRONG",  // Standard preamble, fixed low gain.
                                                      "MODE_S_SMART",   // Time slices of MODE_S and MODE_S_STRONG.
};

const char SettingsManager::RxPosition::kPositionSourceStrs[SettingsManager::RxPosition::kNumPositionSources]
                                                           [SettingsManager::RxPosition::kPositionSourceStrMaxLen] = {
                                                               "NONE", "FIXED", "GNSS", "LOWEST", "ICAO",
};