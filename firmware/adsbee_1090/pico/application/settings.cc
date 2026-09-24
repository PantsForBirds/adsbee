#include "settings.hh"

#include "adsbee.hh"
#include "comms.hh"
#include "core1.hh"
#include "eeprom.hh"
#include "firmware_update.hh"
#include "flash_utils.hh"
#include "hardware/flash.h"
#include "settings_migration.hh"
#include "spi_coprocessor.hh"

/* Original flash length: 16384k Bytes.
   FLASH MAP
       0x10000000	(176k)	FLASH_BL	Bootloader
       0x1002C000	(4k)	FLASH_HDR0	    Application 0 Header
       0x1002D000	(8096k)	FLASH_APP0	    Application 0 Data
       0x10815000	(4k)	FLASH_HDR1	    Application 1 Header
       0x10816000	(8096k)	FLASH_APP1	    Application 1 Data
       0x10FFE000	(8k)	FLASH_SETTINGS	Settings
*/
// Use half of FLASH_SETTINGS for settings, half for device info. We can only erase flash a full sector (4096 Bytes) at
// a time, so it's best to not touch the sector with device info in order to avoid corrupting it.
static_assert(sizeof(SettingsManager::Settings) < FlashUtils::kFlashSectorSizeBytes);
static_assert(sizeof(SettingsManager::DeviceInfo) < FlashUtils::kFlashSectorSizeBytes);
const uint32_t kDeviceInfoMaxSizeBytes = sizeof(SettingsManager::DeviceInfo);

// Device info is stored slightly offset from the end of flash to avoid some addresses that can't be written to.
const uint32_t kFlashSettingsStartAddr = 0x10FFE000;
const uint32_t kFlashEndAddr = 0x10FFFD00;          // There's some hardcoded stuff beyond here that causes problems.
static_assert(kFlashEndAddr % kBytesPerWord == 0);  // Must be word aligned.
const uint32_t kFlashDeviceInfoStartAddr = kFlashSettingsStartAddr + FlashUtils::kFlashSectorSizeBytes;
// Must be word aligned to enable direct pointer casting without memcpy.
static_assert(kFlashDeviceInfoStartAddr % kBytesPerWord == 0);
// Make sure that setting struct doesn't run into device info.
static_assert(sizeof(SettingsManager::Settings) < kFlashDeviceInfoStartAddr - kFlashSettingsStartAddr);

// Device info is stored at the very end of EEPROM for backwards compatibility.
const uint32_t kEEPROMSizeBytes = 8e3;  // 8000 Bytes for backwards compatibility.
const uint32_t kEEPROMDeviceInfoOffset = kEEPROMSizeBytes - kDeviceInfoMaxSizeBytes;

bool SettingsManager::GetDeviceInfo(DeviceInfo& device_info) {
    if (bsp.has_eeprom) {
        // Device Info is stored on external EEPROM.
        if (eeprom.RequiresInit()) return false;
        return eeprom.Load(device_info, kEEPROMDeviceInfoOffset);
    } else {
        // Device Info is stored in flash.
        device_info = *(DeviceInfo*)(kFlashDeviceInfoStartAddr);
        return true;
    }
}

bool SettingsManager::Load() {
    if (bsp.has_eeprom) {
        // Load settings from external EEPROM.
        if (!eeprom.Load(settings)) {
            CONSOLE_ERROR("settings.cc::Load", "Failed load settings from EEPROM.");
            return false;
        };
    } else {
        // Load settings from flash.
        memcpy(&settings, (const void*)kFlashSettingsStartAddr, sizeof(Settings));
    }

    // Handle a stored settings version that doesn't match the current firmware.
    if (settings.settings_version != kSettingsVersion) {
        // Snapshot the raw stored bytes before we overwrite `settings` (the blob we just read IS the old struct; its
        // first field, settings_version, is at offset 0 in every version).
        uint8_t raw[sizeof(Settings)];
        memcpy(raw, &settings, sizeof(settings));
        uint32_t old_version = settings.settings_version;

        // First, try to migrate the old settings forward, preserving all user values.
        if (SettingsMigrator::Migrate(raw, sizeof(raw), old_version, settings)) {
            CONSOLE_INFO("SettingsManager::Settings::Load",
                         "Migrated settings from version %lu to %lu, preserving user settings.", old_version,
                         (uint32_t)kSettingsVersion);
        } else {
            // Unmigratable (older than the oldest supported version, unknown, or a corrupt/short read). Fall back to
            // factory defaults, preserving CoreNetworkSettings if its CRC still validates.
            CONSOLE_ERROR("settingsManager::Settings::Load",
                          "Settings version mismatch (expected %d, got %lu) and no migration path. Resetting to "
                          "defaults.",
                          kSettingsVersion, old_version);
            bool found_valid_cns = false;
            Settings::CoreNetworkSettings cns_backup;
            if (settings.core_network_settings.IsValid()) {
                found_valid_cns = true;
                cns_backup = settings.core_network_settings;
            }

            ResetToDefaults();  // Reset to defaults with part number specific overrides.
            // Restore the core network settings if they were valid.
            if (found_valid_cns) {
                CONSOLE_INFO("SettingsManager::Settings::Load",
                             "Restoring core network settings from backup with checksum 0x%x.", cns_backup.crc32);
                settings.core_network_settings = cns_backup;
            }
        }

        // Persist the migrated (or reset) settings so subsequent boots match the current version.
        if (bsp.has_eeprom && !eeprom.Save(settings)) {
            CONSOLE_ERROR("settings.cc::Load", "Failed to save settings after version change.");
            return false;
        } else if (!bsp.has_eeprom) {
            FlashUtils::FlashSafe();
            flash_range_erase(FirmwareUpdateManager::FlashAddrToOffset(kFlashSettingsStartAddr),
                              FlashUtils::kFlashSectorSizeBytes);
            flash_range_program(FirmwareUpdateManager::FlashAddrToOffset(kFlashSettingsStartAddr), (uint8_t*)&settings,
                                sizeof(settings));
            FlashUtils::FlashUnsafe();
        }
    }

    return true;
}

bool SettingsManager::Save() {
    settings.core_network_settings.UpdateCRC32();

    settings.r1090_rx_enabled = adsbee.Receiver1090IsEnabled();
    settings.tl_offset_mv = adsbee.GetTLOffsetMilliVolts();
    settings.r1090_bias_tee_enabled = adsbee.BiasTeeIsEnabled();
    settings.watchdog_timeout_sec = adsbee.GetWatchdogTimeoutSec();
    settings.rx_position = adsbee.rx_position;

    // Save reporting protocols.
    comms_manager.GetReportingProtocol(SerialInterface::kCommsUART,
                                       settings.reporting_protocols[SerialInterface::kCommsUART]);
    comms_manager.GetReportingProtocol(SerialInterface::kConsole,
                                       settings.reporting_protocols[SerialInterface::kConsole]);

    // Save baud rates.
    comms_manager.GetBaudRate(SerialInterface::kCommsUART,
                              settings.baud_rates[SettingsManager::SerialInterface::kCommsUART]);
    comms_manager.GetBaudRate(SerialInterface::kGNSSUART,
                              settings.baud_rates[SettingsManager::SerialInterface::kGNSSUART]);

    settings.core_network_settings.esp32_enabled = esp32.IsEnabled();

    settings.subg_enabled = adsbee.subg_radio_ll.IsEnabledState();

    // Sync settings from RP2040 -> ESP32.
    SyncToCoprocessors();

    if (bsp.has_eeprom) {
        return eeprom.Save(settings);
    } else {
        FlashUtils::FlashSafe();
        flash_range_erase(FirmwareUpdateManager::FlashAddrToOffset(kFlashSettingsStartAddr),
                          FlashUtils::kFlashSectorSizeBytes);
        flash_range_program(FirmwareUpdateManager::FlashAddrToOffset(kFlashSettingsStartAddr), (uint8_t*)&settings,
                            sizeof(settings));
        FlashUtils::FlashUnsafe();
        return true;
    }
}

void SettingsManager::ResetToDefaults() {
    Settings default_settings;
    settings = default_settings;

    // Override default settings with board-specific defaults.
    // NOTE: This section is not currently used.
    DeviceInfo device_info;
    if (GetDeviceInfo(device_info)) {
        switch (device_info.GetPartNumber()) {
            case DeviceInfo::kPNADSBee1090:
            case DeviceInfo::kPNGS3MPoE:  // Nothing special needed for GS3M PoE since it's all taken care of by core
                                          // network settings.
            case DeviceInfo::kPNADSBee1090U:
            case DeviceInfo::kPNADSBeem1090:
            case DeviceInfo::kPNADSBeem1090EvalBoard:
            default:
                // No changes needed, these use the default settings.
                break;
        }
    }
}

bool SettingsManager::SetDeviceInfo(const DeviceInfo& device_info) {
    if (bsp.has_eeprom) {
        // Device Info is stored on external EEPROM.
        if (eeprom.RequiresInit()) return false;
        return eeprom.Save(device_info, kEEPROMDeviceInfoOffset);
    } else {
        // Device Info is stored in flash.
        FlashUtils::FlashSafe();
        flash_range_erase(FirmwareUpdateManager::FlashAddrToOffset(kFlashDeviceInfoStartAddr),
                          FlashUtils::kFlashSectorSizeBytes);
        flash_range_program(FirmwareUpdateManager::FlashAddrToOffset(kFlashDeviceInfoStartAddr), (uint8_t*)&device_info,
                            sizeof(device_info));
        FlashUtils::FlashUnsafe();
        return true;
    }
}

bool SettingsManager::SyncToCoprocessors() {
    bool ret = true;
    if (esp32.IsEnabled()) {
        ret &= esp32.Write(ObjectDictionary::kAddrSettingsData, settings, true);  // Require ACK.
    }
    // Sync settings from RP2040 -> CC1312.
    if (adsbee.subg_radio.IsEnabled()) {
        ret &= adsbee.subg_radio.Write(ObjectDictionary::kAddrSettingsData, settings, true);  // Require ACK.
    }
    return ret;
}

bool SettingsManager::Apply() {
    bool success = true;

    adsbee.SetReceiver1090Enable(settings.r1090_rx_enabled);
    adsbee.SetTLOffsetMilliVolts(settings.tl_offset_mv);
    adsbee.SetBiasTeeEnable(settings.r1090_bias_tee_enabled);
    adsbee.SetWatchdogTimeoutSec(settings.watchdog_timeout_sec);
    adsbee.SetSubGRadioEnable(settings.subg_enabled);
    adsbee.rx_position = settings.rx_position;

    // Apply reporting protocols.
    comms_manager.SetReportingProtocol(SerialInterface::kCommsUART,
                                       settings.reporting_protocols[SerialInterface::kCommsUART]);
    comms_manager.SetReportingProtocol(SerialInterface::kConsole,
                                       settings.reporting_protocols[SerialInterface::kConsole]);

    // Apply baud rates.
    comms_manager.SetBaudRate(SerialInterface::kCommsUART,
                              settings.baud_rates[SettingsManager::SerialInterface::kCommsUART]);
    comms_manager.SetBaudRate(SerialInterface::kGNSSUART,
                              settings.baud_rates[SettingsManager::SerialInterface::kGNSSUART]);

    if (settings.core_network_settings.esp32_enabled) {
        if (!esp32.IsEnabled()) {
            CONSOLE_INFO("SettingsManager::Apply", "Enabling ESP32.");
            success &= esp32.Init();
        }
    } else {
        if (esp32.IsEnabled()) {
            CONSOLE_INFO("SettingsManager::Apply", "Disabling ESP32.");
            success &= esp32.DeInit();
        }
    }

    // All other parameters are stored directly in the global setting struct and don't need to be applied.

    return success;  // Not currently doing any error checking here, relying on AT commands to limit parameters to
                     // allowable ranges. Could be a problem if loading from corrupted EEPROM.
}