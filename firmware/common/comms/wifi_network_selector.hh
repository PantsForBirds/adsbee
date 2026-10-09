#pragma once

#include <cstdint>
#include <cstring>

// Chooses which stored WiFi station network to join.
class WiFiNetworkSelector {
   public:
    static constexpr int kNone = -1;

    struct ScanResult {
        const char* ssid;
        int8_t rssi_dbm;
    };

    /**
     * @param[in] stored_ssids Stored network SSIDs; empty strings are unused slots.
     * @param[in] skip_mask Bit i set skips stored network i.
     * @retval Index of the stored network seen with the strongest RSSI (ties go to the lower index), or kNone.
     */
    static int Strongest(const char* const* stored_ssids, uint16_t num_stored, const ScanResult* results,
                         uint16_t num_results, uint32_t skip_mask = 0) {
        int best = kNone;
        int8_t best_rssi_dbm = INT8_MIN;
        for (uint16_t i = 0; i < num_stored; i++) {
            if (stored_ssids[i][0] == '\0' || (skip_mask >> i) & 1) continue;
            for (uint16_t j = 0; j < num_results; j++) {
                if (strcmp(stored_ssids[i], results[j].ssid) != 0) continue;
                if (best == kNone || results[j].rssi_dbm > best_rssi_dbm) {
                    best = i;
                    best_rssi_dbm = results[j].rssi_dbm;
                }
            }
        }
        return best;
    }

    /**
     * Round robin over the stored networks, for when a scan finds none of them (e.g. hidden SSIDs).
     * @retval Index of the next stored network after `prev`, wrapping, or kNone if none are stored.
     */
    static int Next(const char* const* stored_ssids, uint16_t num_stored, int prev, uint32_t skip_mask = 0) {
        for (uint16_t k = 1; k <= num_stored; k++) {
            int i = (prev + k) % num_stored;
            if (i < 0) i += num_stored;
            if (stored_ssids[i][0] != '\0' && !((skip_mask >> i) & 1)) return i;
        }
        return kNone;
    }

    /**
     * Picks the network to join after a scan. Networks whose last attempt failed (bit set in failed_mask, e.g. a wrong
     * password) are skipped until every candidate has failed; then failed_mask is cleared and the cycle restarts.
     * @param[in,out] failed_mask Stored networks whose last connection attempt failed.
     * @param[in] prev Network tried last, for the round robin over networks no scan sees (e.g. hidden SSIDs).
     * @retval Stored network index, or kNone if none are stored.
     */
    static int Pick(const char* const* stored_ssids, uint16_t num_stored, const ScanResult* results,
                    uint16_t num_results, uint32_t& failed_mask, int prev) {
        int index = Strongest(stored_ssids, num_stored, results, num_results, failed_mask);
        if (index == kNone) index = Next(stored_ssids, num_stored, prev, failed_mask);
        if (index == kNone) {
            failed_mask = 0;
            index = Strongest(stored_ssids, num_stored, results, num_results);
            if (index == kNone) index = Next(stored_ssids, num_stored, prev);
        }
        return index;
    }
};
