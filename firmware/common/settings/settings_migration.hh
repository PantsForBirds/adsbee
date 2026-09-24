#pragma once

#include <cstdint>

#include "settings.hh"           // SettingsManager::Settings + kSettingsVersion.
#include "settings_versions.hh"  // Frozen historical layouts (settings_v12::Settings, ...).

/**
 * Forward-migration utility for the nonvolatile Settings blob.
 *
 * The RP2040 (settings master) stores Settings as a single raw byte blob. When it boots on new firmware and the stored
 * `settings_version` is older than kSettingsVersion, SettingsManager::Load() calls Migrate() to field-copy the old blob
 * forward into the current live struct, preserving user settings instead of wiping to defaults. Migration supports any
 * stored version >= 12. Older/unknown/short blobs are rejected (Load() then falls back to reset + CoreNetworkSettings
 * preservation).
 *
 * The migration is a pure function of bytes (no flash/EEPROM/globals) so it is fully host-testable.
 */
class SettingsMigrator {
   public:
    // The oldest stored version this utility can migrate from.
    static constexpr uint32_t kOldestMigratableVersion = 12;

    // NOTE on v14 -> v15: firmware between commits e1f28fe7 and this fix silently inserted gnss_enabled/
    // gnss_receiver_type/gnss_notify into the live struct WITHOUT bumping kSettingsVersion (see settings_v14::Settings
    // in settings_versions.hh for the full story). Devices that ran that firmware and saved settings now have an
    // on-flash blob tagged version 14 whose true byte layout no longer matches settings_v14 -- it already matches the
    // *current* (v15) layout, just with garbage/misaligned values in gnss_enabled and every field after it (subg_mode
    // onward), because they were read through the wrong-sized struct at least once. Since the version tag can't
    // distinguish a genuine pre-drift v14 blob from an already-drift-corrupted one, MigrateV14ToV15 makes the
    // textbook assumption that a "v14" blob is byte-true settings_v14 -- this is correct for any device untouched by
    // the drifted firmware, and is what makes all *future* upgrades safe now that kSettingsVersion actually changes.
    // It cannot losslessly recover a blob that was already corrupted by the drifted firmware: those devices will
    // need someone to re-check the 1090/Sub-GHz/GNSS enable flags once after applying this fix, but will not corrupt
    // further.

    /**
     * Migrates a stored settings blob at `from_version` forward to the current kSettingsVersion layout.
     * @param[in] blob Raw stored settings bytes (starts with the uint32_t settings_version at offset 0).
     * @param[in] blob_len Number of valid bytes available in `blob`.
     * @param[in] from_version The stored settings_version (blob[0..3]).
     * @param[out] out Receives the migrated, current-version Settings on success. Left unspecified on failure.
     * @retval True if the blob was migrated to the current version; false if `from_version` is unmigratable
     *         (older than kOldestMigratableVersion, unknown, equal to the current version, or `blob` too short).
     */
    static bool Migrate(const uint8_t* blob, uint16_t blob_len, uint32_t from_version, SettingsManager::Settings& out);

   private:
    // Per-version upgrade steps, chained: each takes the frozen layout of version N and produces version N+1. Steps that
    // land on an intermediate frozen version write that frozen struct; only the final step (the one that reaches the
    // current version) writes the live SettingsManager::Settings. To add v16: freeze v15 in settings_versions.hh, change
    // MigrateV14ToV15 to emit settings_v15::Settings, add MigrateV15ToV16 emitting the live struct, and extend
    // Migrate()'s dispatch with a `case 15:`.
    static void MigrateV12ToV13(const settings_v12::Settings& in, settings_v13::Settings& out);
    static void MigrateV13ToV14(const settings_v13::Settings& in, settings_v14::Settings& out);
    static void MigrateV14ToV15(const settings_v14::Settings& in, SettingsManager::Settings& out);
};
