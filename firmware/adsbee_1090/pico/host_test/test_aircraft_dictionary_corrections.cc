// Measures what corrected and misattributed frames do to the aircraft dictionary. Each frame set runs through the
// decoder and a dictionary, next to a reference dictionary that only gets the clean frames (and, for a frame with
// bit errors from a tracked aircraft, its clean original). A field that differs between the two was changed by a bad
// frame, at that frame or later (a bad CPR half pairs with the next clean one).
//
// CORRECTION_VECTORS=<file> runs a file made by correction_vectors/gen.py and prints the counts.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "aircraft_dictionary.hh"
#include "geo_utils.hh"
#include "gtest/gtest.h"
#include "hal_god_powers.hh"
#include "mode_s_packet.hh"
#include "mode_s_packet_decoder.hh"

namespace {

enum Field {
    kFieldPosition,
    kFieldBaroAltitude,
    kFieldGNSSAltitude,
    kFieldSquawk,
    kFieldCallsign,
    kFieldVelocity,
    kFieldAlertIdent,
    kFieldAirGround,
    kNumFields
};
const char* kFieldNames[kNumFields] = {"position", "baro_alt", "gnss_alt", "squawk",
                                       "callsign", "velocity", "alert/ident", "air/ground"};

struct Frame {
    uint32_t ms;
    char role;  // 'P' clean, 'B' must not change anything, 'G' bit errors from a tracked aircraft.
    std::string hex;
    std::string original;
};

enum Path { kPathNone, kPathClean, kPathAddressParity1Bit, kPathAddressParity2Bit, kPathSquitter1Bit,
            kPathSquitter2Bit, kNumPaths };
const char* kPathNames[kNumPaths] = {"-", "uncorr", "ap1", "ap2", "es1", "es2"};

Path PathOf(const DecodedModeSPacket& p) {
    if (!p.is_valid) return kPathNone;
    if (p.num_corrected_bits == 0) return kPathClean;
    if (p.is_address_parity) return p.num_corrected_bits == 1 ? kPathAddressParity1Bit : kPathAddressParity2Bit;
    return p.num_corrected_bits == 1 ? kPathSquitter1Bit : kPathSquitter2Bit;
}

struct SetResult {
    std::string name;
    uint32_t frames[3] = {0};     // P, B, G
    uint32_t forwarded[3] = {0};  // Valid after the dictionary (sent to Raw, Beast, feeds and the ESP32).
    uint32_t b_credited = 0;      // Bad frames counted as a frame of a tracked aircraft.
    uint32_t b_credited_by_path[kNumPaths] = {0};
    uint32_t b_wrong_by_path[kNumPaths] = {0};  // Bad frames that made a field wrong, by how they got in.
    uint32_t g_applied_by_path[kNumPaths] = {0};
    uint32_t wrong_events[kNumFields] = {0};  // A field became wrong.
    uint32_t wrong_events_at_clean[kNumFields] = {0};  // ...when a clean frame came in (delayed effect).
    double max_position_error_m = 0;
    int32_t max_altitude_error_ft = 0;
    uint32_t created = 0;  // Aircraft only the dictionary under test has.
    uint32_t g_recovered = 0;  // Fixed back to the original.
    uint32_t g_applied = 0;    // ...and applied to the aircraft.
    uint32_t g_position_updates = 0, ref_g_position_updates = 0;
};

int RoleIndex(char role) { return role == 'P' ? 0 : role == 'B' ? 1 : 2; }

double DistanceM(float lat1, float lon1, float lat2, float lon2) {
    double dlat = (lat2 - lat1) * 111320.0;
    double dlon = (lon2 - lon1) * 111320.0 * cos(lat1 * M_PI / 180.0);
    return sqrt(dlat * dlat + dlon * dlon);
}

// Which fields of a differ from the reference r.
uint32_t WrongFields(const ModeSAircraft& a, const ModeSAircraft& r, double* position_error_m,
                     int32_t* altitude_error_ft) {
    uint32_t wrong = 0;
    auto flag = [&](ModeSAircraft::BitFlag f) { return a.HasBitFlag(f) != r.HasBitFlag(f); };
    bool a_pos = a.HasBitFlag(ModeSAircraft::kBitFlagPositionValid);
    if (a_pos) {
        // A position only the dictionary under test has, or one away from the reference.
        double d = r.HasBitFlag(ModeSAircraft::kBitFlagPositionValid)
                       ? DistanceM(a.latitude_deg, a.longitude_deg, r.latitude_deg, r.longitude_deg)
                       : 1e9;
        if (d > 100.0) {
            wrong |= 1 << kFieldPosition;
            *position_error_m = d;
        }
    }
    if (a.HasBitFlag(ModeSAircraft::kBitFlagBaroAltitudeValid) &&
        (flag(ModeSAircraft::kBitFlagBaroAltitudeValid) || a.baro_altitude_ft != r.baro_altitude_ft)) {
        wrong |= 1 << kFieldBaroAltitude;
        *altitude_error_ft = r.HasBitFlag(ModeSAircraft::kBitFlagBaroAltitudeValid)
                                 ? abs(a.baro_altitude_ft - r.baro_altitude_ft)
                                 : 99999;
    }
    if (a.HasBitFlag(ModeSAircraft::kBitFlagGNSSAltitudeValid) &&
        (flag(ModeSAircraft::kBitFlagGNSSAltitudeValid) || a.gnss_altitude_ft != r.gnss_altitude_ft)) {
        wrong |= 1 << kFieldGNSSAltitude;
    }
    if (a.squawk != r.squawk) wrong |= 1 << kFieldSquawk;
    if (strcmp(a.callsign, r.callsign) != 0) wrong |= 1 << kFieldCallsign;
    if ((a.HasBitFlag(ModeSAircraft::kBitFlagHorizontalSpeedValid) &&
         (flag(ModeSAircraft::kBitFlagHorizontalSpeedValid) || a.speed_kts != r.speed_kts ||
          fabsf(a.direction_deg - r.direction_deg) > 0.5f)) ||
        (a.HasBitFlag(ModeSAircraft::kBitFlagBaroVerticalRateValid) &&
         a.baro_vertical_rate_fpm != r.baro_vertical_rate_fpm) ||
        (a.HasBitFlag(ModeSAircraft::kBitFlagGNSSVerticalRateValid) &&
         a.gnss_vertical_rate_fpm != r.gnss_vertical_rate_fpm)) {
        wrong |= 1 << kFieldVelocity;
    }
    if (flag(ModeSAircraft::kBitFlagAlert) || flag(ModeSAircraft::kBitFlagIdent)) wrong |= 1 << kFieldAlertIdent;
    if (a.IsOnGround() != r.IsOnGround()) wrong |= 1 << kFieldAirGround;
    return wrong;
}

class Runner {
   public:
    explicit Runner(const std::string& name) {
        result.name = name;
        for (AircraftDictionary* d : {&dut_, &ref_}) {
            d->SetAddressParityCorrection(true);
        }
    }

    void Run(const Frame& f) {
        if (f.ms / 1000 != last_update_s_) {
            last_update_s_ = f.ms / 1000;
            set_time_since_boot_ms(last_update_s_ * 1000);
            dut_.Update(get_time_since_boot_ms());
            ref_.Update(get_time_since_boot_ms());
        }
        set_time_since_boot_ms(f.ms);
        int role = RoleIndex(f.role);
        result.frames[role]++;

        // Reference: clean frames only.
        const std::string& clean = f.role == 'P' ? f.hex : f.original;
        uint32_t clean_address = 0;
        if (!clean.empty()) {
            DecodedModeSPacket p(RawModeSPacket(clean.c_str()));
            clean_address = p.icao_address;
            p.raw.mlat_48mhz_64bit_counts = static_cast<uint64_t>(f.ms) * 48000;
            ref_.IngestDecodedModeSPacket(p);
        }

        // Dictionary under test, through the decoder.
        ModeSAircraft* dut_target = f.role == 'G' ? Find(dut_, clean_address) : nullptr;
        uint32_t dut_pos_before = dut_target ? dut_target->last_position_update_ms : 0;
        std::map<uint32_t, uint32_t> frames_before;
        for (auto& [uid, entry] : dut_.dict) {
            if (auto* a = std::get_if<ModeSAircraft>(&entry)) frames_before[uid] = a->num_frames_received;
        }

        RawModeSPacket raw(f.hex.c_str());
        raw.mlat_48mhz_64bit_counts = static_cast<uint64_t>(f.ms) * 48000;
        decoder_.raw_mode_s_packet_in_queue.Enqueue(raw);
        decoder_.UpdateDecoderLoop();
        ModeSPacketDecoder::DebugMessage dm;
        while (decoder_.debug_message_out_queue.Dequeue(dm)) {
        }
        uint16_t loc;
        while (decoder_.decoded_mode_s_packet_bit_flip_locations_out_queue.Dequeue(loc)) {
        }
        DecodedModeSPacket packet;
        bool forwarded = false, recovered = false;
        Path path = kPathNone;
        while (decoder_.decoded_mode_s_packet_out_queue.Dequeue(packet)) {
            dut_.IngestDecodedModeSPacket(packet);
            forwarded |= packet.is_valid;
            if (packet.is_valid) path = PathOf(packet);
            if (f.role == 'G' && packet.is_valid) {
                char buf[32];
                packet.raw.PrintBuffer(buf, sizeof(buf));
                recovered |= f.original == buf;
            }
        }
        if (forwarded) result.forwarded[role]++;

        // Credit: a tracked aircraft counted the frame.
        bool credited = false;
        for (auto& [uid, entry] : dut_.dict) {
            auto* a = std::get_if<ModeSAircraft>(&entry);
            if (!a) continue;
            auto it = frames_before.find(uid);
            if (it == frames_before.end() || a->num_frames_received != it->second) credited = true;
        }
        if (f.role == 'B' && credited) {
            result.b_credited++;
            result.b_credited_by_path[path]++;
        }
        if (f.role == 'G' && recovered) {
            result.g_recovered++;
            if (credited) {
                result.g_applied++;
                result.g_applied_by_path[path]++;
            }
            ModeSAircraft* a = Find(dut_, clean_address);
            if (a && a->last_position_update_ms != dut_pos_before && a->last_position_update_ms == f.ms) {
                result.g_position_updates++;
            }
        }
        if (f.role == 'G') {
            ModeSAircraft* ra = Find(ref_, clean_address);
            if (ra && ra->last_position_update_ms == f.ms) result.ref_g_position_updates++;
        }

        // Compare every aircraft with the reference.
        bool made_wrong = false;
        for (auto& [uid, entry] : dut_.dict) {
            auto* a = std::get_if<ModeSAircraft>(&entry);
            if (!a) continue;
            ModeSAircraft* r = ref_.GetAircraftPtr<ModeSAircraft>(uid, false);
            uint32_t wrong;
            double pos_err = 0;
            int32_t alt_err = 0;
            if (r == nullptr) {
                if (!created_.count(uid)) {
                    created_[uid] = true;
                    result.created++;
                }
                continue;
            }
            wrong = WrongFields(*a, *r, &pos_err, &alt_err);
            uint32_t& was = wrong_[uid];
            uint32_t new_wrong = wrong & ~was;
            for (int i = 0; i < kNumFields; i++) {
                if (new_wrong & (1 << i)) {
                    result.wrong_events[i]++;
                    made_wrong = true;
                    if (f.role == 'P') result.wrong_events_at_clean[i]++;
                }
            }
            if (wrong & (1 << kFieldPosition) && pos_err > result.max_position_error_m) {
                result.max_position_error_m = pos_err;
            }
            if (wrong & (1 << kFieldBaroAltitude) && alt_err > result.max_altitude_error_ft) {
                result.max_altitude_error_ft = alt_err;
            }
            was = wrong;
        }
        if (made_wrong && f.role == 'B') result.b_wrong_by_path[path]++;
    }

    SetResult result;

   private:
    static ModeSAircraft* Find(AircraftDictionary& d, uint32_t icao) {
        return d.GetAircraftPtr<ModeSAircraft>(Aircraft::ICAOToUID(icao, Aircraft::kAircraftTypeModeS), false);
    }

    AircraftDictionary dut_, ref_;
    ModeSPacketDecoder decoder_{{.enable_1090_error_correction = true, .max_1090_error_correction_num_bits = 2}};
    uint32_t last_update_s_ = UINT32_MAX;
    std::map<uint32_t, uint32_t> wrong_;
    std::map<uint32_t, bool> created_;
};

std::vector<SetResult> RunFile(const char* path) {
    std::vector<SetResult> results;
    std::ifstream in(path);
    std::string line;
    Runner* runner = nullptr;
    while (std::getline(in, line)) {
        if (line.rfind("# set ", 0) == 0) {
            if (runner) {
                results.push_back(runner->result);
                delete runner;
            }
            runner = new Runner(line.substr(6));
            continue;
        }
        if (line.empty() || line[0] == '#' || runner == nullptr) continue;
        std::istringstream ss(line);
        Frame f;
        std::string role;
        ss >> f.ms >> role >> f.hex >> f.original;
        f.role = role[0];
        runner->Run(f);
    }
    if (runner) {
        results.push_back(runner->result);
        delete runner;
    }
    return results;
}

void Print(const std::vector<SetResult>& results) {
    printf("\n%-22s %7s %7s %7s | %8s %8s %8s |", "set", "clean", "bad", "biterr", "bad_fwd", "bad_cred", "created");
    for (int i = 0; i < kNumFields; i++) printf(" %10s", kFieldNames[i]);
    printf(" | %9s %8s | %9s %9s %9s %9s\n", "maxpos_m", "maxalt", "g_fwd", "g_recov", "g_applied", "g_pos/ref");
    for (const SetResult& r : results) {
        printf("%-22s %7u %7u %7u | %8u %8u %8u |", r.name.c_str(), r.frames[0], r.frames[1], r.frames[2],
               r.forwarded[1], r.b_credited, r.created);
        for (int i = 0; i < kNumFields; i++) {
            char buf[32];
            snprintf(buf, sizeof(buf), "%u(%u)", r.wrong_events[i], r.wrong_events_at_clean[i]);
            printf(" %10s", buf);
        }
        char gp[32];
        snprintf(gp, sizeof(gp), "%u/%u", r.g_position_updates, r.ref_g_position_updates);
        printf(" | %9.0f %8d | %9u %9u %9u %9s\n", r.max_position_error_m, r.max_altitude_error_ft, r.forwarded[2],
               r.g_recovered, r.g_applied, gp);
    }
    printf("Field columns: times a field became wrong (of those, when a clean frame came in).\n");
    printf("\n%-22s |", "by path");
    for (int i = 1; i < kNumPaths; i++) printf(" %16s", kPathNames[i]);
    printf("   (bad credited/bad made a field wrong; good applied)\n");
    for (const SetResult& r : results) {
        printf("%-22s |", r.name.c_str());
        for (int i = 1; i < kNumPaths; i++) {
            char buf[40];
            if (r.frames[2]) {
                snprintf(buf, sizeof(buf), "%u", r.g_applied_by_path[i]);
            } else {
                snprintf(buf, sizeof(buf), "%u/%u", r.b_credited_by_path[i], r.b_wrong_by_path[i]);
            }
            printf(" %16s", buf);
        }
        printf("\n");
    }
}

}  // namespace

TEST(AircraftDictionaryCorrections, VectorFile) {
    const char* path = getenv("CORRECTION_VECTORS");
    if (path == nullptr) {
        GTEST_SKIP() << "Set CORRECTION_VECTORS to a file from correction_vectors/gen.py.";
    }
    Print(RunFile(path));
}
