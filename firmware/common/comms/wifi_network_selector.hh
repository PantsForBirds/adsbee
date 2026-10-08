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
     * @retval Index of the stored network seen with the strongest RSSI (ties go to the lower index), or kNone.
     */
    static int Strongest(const char* const* stored_ssids, uint16_t num_stored, const ScanResult* results,
                         uint16_t num_results) {
        int best = kNone;
        int8_t best_rssi_dbm = INT8_MIN;
        for (uint16_t i = 0; i < num_stored; i++) {
            if (stored_ssids[i][0] == '\0') continue;
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
    static int Next(const char* const* stored_ssids, uint16_t num_stored, int prev) {
        for (uint16_t k = 1; k <= num_stored; k++) {
            int i = (prev + k) % num_stored;
            if (i < 0) i += num_stored;
            if (stored_ssids[i][0] != '\0') return i;
        }
        return kNone;
    }
};
