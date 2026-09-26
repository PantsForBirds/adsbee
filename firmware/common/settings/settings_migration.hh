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

    // NOTE on v14 -> v15: both e1f28fe7 (GNSS fields) and #224 (feeds_enabled) inserted fields into the live struct
    // without bumping kSettingsVersion. settings_v14 is the layout every v14 release shipped (with GNSS, without
    // feeds_enabled). Unreleased dev builds from between #224 and v15 stored a feeds_enabled layout under the v14 tag;
    // those blobs migrate with log_level onward misread, and SettingsManager::Sanitize() keeps the result safe.

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
