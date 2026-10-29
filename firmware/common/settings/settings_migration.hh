#pragma once

#include <cstdint>

#include "settings.hh"           // SettingsManager::Settings + kSettingsVersion.
#include "settings_versions.hh"  // Frozen historical layouts (settings_v12::Settings, ...).

/**
 * Forward-migration utility for the nonvolatile Settings blob.
 *
 * The RP2040 (settings master) stores Settings as a single raw byte blob. When it boots on new firmware and the stored
 * `settings_version` is older than kSettingsVersion, SettingsManager::Load() calls Migrate() to field-copy the old blob
 * forward into the current live struct, preserving user settings instead of wiping to defaults. Migration supports each
 * version a tagged release or RC shipped, from 12 on (12, 14, 15, 17, 18). Older/unshipped/unknown/short blobs are
 * rejected (Load() then falls back to reset + CoreNetworkSettings preservation).
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
     *         (older than kOldestMigratableVersion, never shipped, unknown, equal to the current version, or `blob`
     *         too short).
     */
    static bool Migrate(const uint8_t* blob, uint16_t blob_len, uint32_t from_version, SettingsManager::Settings& out);

   private:
    // Upgrade steps between shipped versions, chained: each takes the frozen layout of one shipped version and produces
    // the next shipped one, spanning versions no tag shipped (v13, v16). Steps that land on a frozen version write that
    // frozen struct; only the final step (the one that reaches the current version) writes the live
    // SettingsManager::Settings. Bump kSettingsVersion only when the current version shipped in a tag (see
    // firmware/AGENTS.md). To add v21: change MigrateV19ToV20 to emit a frozen settings_v20::Settings, add
    // MigrateV20ToV21 emitting the live struct, and extend Migrate()'s dispatch with a `case 20:`. While the current
    // version is unreleased, change its step in place instead.
    //
    // v20 only added ble_output_mode in v19's tail padding, so v18 -> v19 builds the live struct (ble_output_mode
    // defaulted off) and v19 -> v20 runs in place on it: the RP2040 stack has no room for another snapshot.
    static void MigrateV12ToV14(const settings_v12::Settings& in, settings_v14::Settings& out);
    static void MigrateV14ToV15(const settings_v14::Settings& in, settings_v15::Settings& out);
    static void MigrateV15ToV17(const settings_v15::Settings& in, settings_v17::Settings& out);
    static void MigrateV17ToV18(const settings_v17::Settings& in, settings_v18::Settings& out);
    static void MigrateV18ToV19(const settings_v18::Settings& in, SettingsManager::Settings& out);
    // `settings` holds the v19 bytes on entry.
    static void MigrateV19ToV20(SettingsManager::Settings& settings);
};
