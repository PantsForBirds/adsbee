// t-1336: PPS labeling (PPSClock::OnRMC) against recorded receiver output, replayed with UART timing.
//
// The NMEA is from gpsd's regression logs (https://gitlab.com/gpsd/gpsd, test/daemon, BSD-2-Clause), one vector per
// epoch in the order each receiver sent it. The logs have no timing, so it is modeled: the PPS edge of second T is at
// T; the receiver starts sending epoch T latency_ms later, at the baud rate (8N1), through a transmit buffer that drops
// whole messages that don't fit (as u-blox, MAX-M10S integration manual UBX-20053088 §3.2.1); the RMC is labeled when
// its last byte is in. The u-blox MAX-M10 case (no log available) is built from the sentence formats and our
// configuration (GGA + RMC + GST, UBX-MON-RF and UBX-NAV-SAT every epoch, 9600 baud).
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nmea_parser.hh"
#include "pps_clock.hh"

namespace {

// gpsd test/daemon/neo-m8n.log: BN-880 (u-blox NEO-M8N), 2015-03-18, submitted by Gary E. Miller. One vector per epoch, in the order the receiver sent them.
const std::vector<std::vector<std::string>> kUbloxNeoM8n = {
    {
        "$GNRMC,171927.00,A,4404.14058,N,12118.85475,W,0.084,,180315,,,A*79",
        "$GNVTG,,T,,M,0.084,N,0.156,K,A*33",
        "$GNGGA,171927.00,4404.14058,N,12118.85475,W,1,12,0.91,1147.0,M,-21.3,M,,*42",
        "$GNGSA,A,3,01,11,31,14,22,32,04,25,03,,,,1.56,0.91,1.27*15",
        "$GNGSA,A,3,67,83,76,82,68,,,,,,,,1.56,0.91,1.27*1D",
        "$GPGSV,4,1,13,01,53,280,36,03,29,301,23,04,55,223,37,08,38,166,15*78",
        "$GPGSV,4,2,13,11,40,245,29,12,05,034,,14,43,055,29,17,06,324,*7C",
        "$GPGSV,4,3,13,22,24,112,24,23,08,249,,25,12,069,21,31,58,133,23*7F",
        "$GPGSV,4,4,13,32,76,331,30*49",
        "$GLGSV,3,1,10,66,37,078,26,67,62,349,26,68,20,296,23,75,04,018,*69",
        "$GLGSV,3,2,10,76,43,057,24,77,43,139,25,78,04,176,18,82,25,245,25*67",
        "$GLGSV,3,3,10,83,27,309,30,84,01,353,*6B",
        "$GNGLL,4404.14058,N,12118.85475,W,171927.00,A,A*62",
    },
    {
        "$GNRMC,171928.00,A,4404.14055,N,12118.85469,W,0.116,,180315,,,A*7C",
        "$GNVTG,,T,,M,0.116,N,0.215,K,A*3D",
        "$GNGGA,171928.00,4404.14055,N,12118.85469,W,1,12,0.91,1146.7,M,-21.3,M,,*4B",
        "$GNGSA,A,3,01,11,31,14,22,32,04,25,03,,,,1.56,0.91,1.27*15",
        "$GNGSA,A,3,67,83,76,82,68,,,,,,,,1.56,0.91,1.27*1D",
        "$GPGSV,4,1,13,01,53,279,36,03,29,301,23,04,55,223,37,08,38,166,15*7E",
        "$GPGSV,4,2,13,11,40,245,29,12,05,034,,14,43,055,29,17,06,324,*7C",
        "$GPGSV,4,3,13,22,24,112,24,23,08,249,,25,12,069,19,31,58,133,23*74",
        "$GPGSV,4,4,13,32,76,331,30*49",
        "$GLGSV,3,1,10,66,37,078,26,67,62,349,24,68,20,296,23,75,04,018,*6B",
        "$GLGSV,3,2,10,76,43,057,24,77,43,139,25,78,04,176,17,82,25,245,25*68",
        "$GLGSV,3,3,10,83,27,309,30,84,01,353,*6B",
        "$GNGLL,4404.14055,N,12118.85469,W,171928.00,A,A*6D",
    },
    {
        "$GNRMC,171929.00,A,4404.14054,N,12118.85469,W,0.145,,180315,,,A*7A",
        "$GNVTG,,T,,M,0.145,N,0.268,K,A*31",
        "$GNGGA,171929.00,4404.14054,N,12118.85469,W,1,12,0.91,1146.8,M,-21.3,M,,*44",
        "$GNGSA,A,3,01,11,31,14,22,32,04,25,03,,,,1.56,0.91,1.27*15",
        "$GNGSA,A,3,67,83,76,82,68,,,,,,,,1.56,0.91,1.27*1D",
        "$GPGSV,4,1,13,01,53,279,36,03,29,301,22,04,55,223,37,08,38,166,15*7F",
        "$GPGSV,4,2,13,11,40,245,29,12,05,034,,14,43,055,29,17,06,324,*7C",
        "$GPGSV,4,3,13,22,24,112,24,23,08,249,17,25,12,069,19,31,58,133,21*70",
        "$GPGSV,4,4,13,32,76,331,32*4B",
        "$GLGSV,3,1,10,66,37,078,24,67,62,349,22,68,20,296,21,75,04,018,*6D",
        "$GLGSV,3,2,10,76,43,057,22,77,43,139,24,78,04,176,15,82,25,245,23*6B",
        "$GLGSV,3,3,10,83,27,309,29,84,01,353,*63",
        "$GNGLL,4404.14054,N,12118.85469,W,171929.00,A,A*6D",
    },
    {
        "$GNRMC,171930.00,A,4404.14047,N,12118.85466,W,0.086,,180315,,,A*71",
        "$GNVTG,,T,,M,0.086,N,0.160,K,A*34",
        "$GNGGA,171930.00,4404.14047,N,12118.85466,W,1,12,1.02,1146.5,M,-21.3,M,,*47",
        "$GNGSA,A,3,01,11,31,14,22,32,04,25,03,,,,1.69,1.02,1.35*11",
        "$GNGSA,A,3,67,83,76,68,,,,,,,,,1.69,1.02,1.35*13",
        "$GPGSV,4,1,13,01,53,279,35,03,30,301,22,04,55,223,37,08,38,166,17*76",
        "$GPGSV,4,2,13,11,40,245,28,12,05,034,,14,43,055,29,17,06,324,*7D",
        "$GPGSV,4,3,13,22,24,112,24,23,08,249,17,25,12,069,19,31,58,133,22*73",
        "$GPGSV,4,4,13,32,76,331,32*4B",
        "$GLGSV,3,1,10,66,37,078,22,67,62,349,21,68,20,296,19,75,04,018,*63",
        "$GLGSV,3,2,10,76,43,057,21,77,43,139,22,78,04,176,11,82,25,245,20*69",
        "$GLGSV,3,3,10,83,27,309,28,84,01,353,*62",
        "$GNGLL,4404.14047,N,12118.85466,W,171930.00,A,A*68",
    },
    {
        "$GNRMC,171931.00,A,4404.14038,N,12118.85466,W,0.097,,180315,,,A*78",
        "$GNVTG,,T,,M,0.097,N,0.180,K,A*3A",
        "$GNGGA,171931.00,4404.14038,N,12118.85466,W,1,12,1.01,1146.2,M,-21.3,M,,*4A",
        "$GNGSA,A,3,01,11,31,14,22,32,04,25,03,,,,1.69,1.01,1.35*12",
        "$GNGSA,A,3,67,83,76,68,,,,,,,,,1.69,1.01,1.35*10",
        "$GPGSV,4,1,13,01,53,279,36,03,30,301,23,04,55,222,37,08,38,166,18*7A",
        "$GPGSV,4,2,13,11,40,245,28,12,05,034,,14,43,055,29,17,06,324,*7D",
        "$GPGSV,4,3,13,22,24,112,24,23,08,249,18,25,12,069,19,31,58,133,23*7D",
        "$GPGSV,4,4,13,32,76,331,32*4B",
        "$GLGSV,3,1,10,66,37,078,21,67,62,349,20,68,20,296,16,75,04,018,*6E",
        "$GLGSV,3,2,10,76,43,057,20,77,43,139,21,78,04,176,,82,25,245,*69",
        "$GLGSV,3,3,10,83,27,309,28,84,01,353,*62",
        "$GNGLL,4404.14038,N,12118.85466,W,171931.00,A,A*61",
    },
};

// gpsd test/daemon/mt3339.log: Adafruit Ultimate GPS HAT (MediaTek MT3339), 2015-04-13, Gary E. Miller. One vector per epoch, in the order the receiver sent them.
const std::vector<std::vector<std::string>> kMediaTekMT3339 = {
    {
        "$GPGGA,202641.000,4404.1305,N,12118.8515,W,2,11,0.83,1127.7,M,-19.6,M,0000,0000*65",
        "$GPGSA,A,3,20,16,09,03,07,10,23,19,27,30,05,,1.33,0.83,1.03*09",
        "$GPRMC,202641.000,A,4404.1305,N,12118.8515,W,0.01,201.71,130415,,,D*7D",
        "$GPZDA,202641.000,13,04,2015,,*55",
    },
    {
        "$GPGGA,202642.000,4404.1305,N,12118.8515,W,2,11,0.83,1127.7,M,-19.6,M,0000,0000*66",
        "$GPGSA,A,3,20,16,09,03,07,10,23,19,27,30,05,,1.33,0.83,1.03*09",
        "$GPRMC,202642.000,A,4404.1305,N,12118.8515,W,0.00,0.00,130415,,,D*7A",
        "$GPZDA,202642.000,13,04,2015,,*56",
    },
    {
        "$GPGGA,202643.000,4404.1305,N,12118.8515,W,2,11,0.84,1127.7,M,-19.6,M,0000,0000*60",
        "$GPGSA,A,3,20,16,09,03,07,10,23,19,27,30,05,,1.33,0.84,1.03*0E",
        "$GPGSV,3,1,12,09,88,089,31,07,60,281,44,23,55,123,33,10,44,274,32*78",
        "$GPGSV,3,2,12,16,39,051,29,30,30,262,20,20,23,308,30,27,23,093,34*7F",
        "$GPGSV,3,3,12,19,13,123,27,03,07,176,19,05,05,328,13,47,,,*41",
        "$GPRMC,202643.000,A,4404.1305,N,12118.8515,W,0.02,171.47,130415,,,D*7D",
        "$GPZDA,202643.000,13,04,2015,,*57",
    },
    {
        "$GPGGA,202644.000,4404.1305,N,12118.8515,W,2,11,0.84,1127.7,M,-19.6,M,0000,0000*67",
        "$GPGSA,A,3,20,16,09,03,07,10,23,19,27,30,05,,1.33,0.84,1.03*0E",
        "$GPRMC,202644.000,A,4404.1305,N,12118.8515,W,0.02,193.13,130415,,,D*77",
        "$GPZDA,202644.000,13,04,2015,,*50",
    },
    {
        "$GPGGA,202645.000,4404.1305,N,12118.8515,W,2,10,0.98,1127.7,M,-19.6,M,0000,0000*6A",
        "$GPGSA,A,3,20,16,09,03,07,10,23,19,27,30,,,1.64,0.98,1.31*05",
        "$GPRMC,202645.000,A,4404.1305,N,12118.8515,W,0.03,173.97,130415,,,D*75",
        "$GPZDA,202645.000,13,04,2015,,*51",
    },
};

// gpsd test/daemon/quectel-L70.log: Quectel L70 (MediaTek), 2022-03-09, submitted by Jared Hagel. One vector per epoch, in the order the receiver sent them.
const std::vector<std::vector<std::string>> kQuectelL70 = {
    {
        "$GPRMC,150120.000,A,5104.0859,N,11405.7388,W,0.13,194.17,090322,,,D*7D",
        "$GPGGA,150120.000,5104.0859,N,11405.7388,W,2,8,0.94,1102.3,M,-17.5,M,,*51",
        "$GPGSA,A,3,04,09,16,26,07,03,27,02,,,,,1.21,0.94,0.76*01",
        "$GPGRS,150120.000,1,-1.62,-3.72,1.47,0.47,-2.96,7.83,-13.2,17.4,,,,*78",
        "$GPGST,150120.000,004.0,008.6,004.7,132.0,006.7,007.1,00013*72",
        "$PMTK705,AXN_2.51_3339_19071200,0002,QUECTEL-L70,1.0*32",
        "$PQVERNO,R,L70BGR03A05S,2019/10/09,21:12*23",
        "$PMTK001,320,2*30",
        "$PMTK001,300,3*33",
        "$PMTK001,314,3*36",
        "$PMTK001,301,3*32",
        "$PMTK001,313,3*31",
        "$PMTK001,424,1*30",
    },
    {
        "$GPZDA,150121.000,09,03,2022,,*58",
        "$GPRMC,150121.000,A,5104.0859,N,11405.7388,W,0.06,194.17,090322,,,D*78",
        "$GPGGA,150121.000,5104.0859,N,11405.7388,W,2,8,0.94,1102.3,M,-17.5,M,,*50",
        "$GPGSA,A,3,04,09,16,26,07,03,27,02,,,,,1.49,0.94,1.15*0B",
        "$GPGSV,4,1,13,04,77,125,31,09,63,293,26,16,56,098,23,26,34,055,33*79",
        "$GPGSV,4,2,13,46,29,199,28,07,27,252,25,03,24,186,33,02,08,319,17*72",
        "$GPGSV,4,3,13,27,07,123,16,06,07,274,,29,04,012,15,20,03,332,*72",
        "$GPGSV,4,4,13,30,03,251,*4D",
        "$GPGRS,150121.000,1,-1.92,-2.89,0.07,-0.04,-2.54,7.31,-13.7,7.39,,,,*51",
        "$GPGST,150121.000,004.0,008.5,004.7,132.0,006.6,007.0,00013*70",
    },
    {
        "$GPZDA,150122.000,09,03,2022,,*5B",
        "$GPRMC,150122.000,A,5104.0860,N,11405.7388,W,0.03,194.17,090322,,,D*74",
        "$GPGGA,150122.000,5104.0860,N,11405.7388,W,2,8,0.94,1102.3,M,-17.5,M,,*59",
        "$GPGSA,A,3,04,09,16,26,07,03,27,02,,,,,1.21,0.94,0.76*01",
        "$GPGRS,150122.000,1,-0.59,-2.63,-0.82,1.10,-3.18,7.97,-11.7,7.04,,,,*50",
        "$GPGST,150122.000,004.0,008.5,004.6,131.7,006.6,007.0,00013*76",
    },
    {
        "$GPZDA,150123.000,09,03,2022,,*5A",
        "$GPRMC,150123.000,A,5104.0859,N,11405.7388,W,0.08,194.17,090322,,,D*74",
        "$GPGGA,150123.000,5104.0859,N,11405.7388,W,2,8,0.94,1102.3,M,-17.5,M,,*52",
        "$GPGSA,A,3,04,09,16,26,07,03,27,02,,,,,1.21,0.94,0.75*02",
        "$GPGRS,150123.000,1,-0.44,-3.67,-0.85,1.10,-2.40,7.55,-11.4,7.43,,,,*5D",
        "$GPGST,150123.000,004.0,008.5,004.6,131.7,006.6,007.1,00013*76",
    },
    {
        "$GPZDA,150124.000,09,03,2022,,*5D",
        "$GPRMC,150124.000,A,5104.0859,N,11405.7388,W,0.04,194.17,090322,,,D*7F",
        "$GPGGA,150124.000,5104.0859,N,11405.7388,W,2,8,0.94,1102.3,M,-17.5,M,,*55",
        "$GPGSA,A,3,04,09,16,26,07,03,27,02,,,,,1.21,0.94,0.76*01",
        "$GPGRS,150124.000,1,-1.71,-3.52,-0.59,0.11,-1.63,7.45,-11.4,5.05,,,,*59",
        "$GPGST,150124.000,004.0,008.5,004.6,131.0,006.5,007.1,00013*75",
    },
};

// An epoch as bytes on the wire: NMEA sentences get CR LF.
std::vector<std::string> WireMessages(const std::vector<std::string>& sentences) {
    std::vector<std::string> out;
    for (const std::string& s : sentences) out.push_back(s[0] == '$' ? s + "\r\n" : s);
    return out;
}

std::string WithChecksum(const std::string& body) {  // body without '$' and '*'.
    uint8_t c = 0;
    for (char ch : body) c ^= static_cast<uint8_t>(ch);
    char tail[8];
    snprintf(tail, sizeof(tail), "*%02X", c);
    return "$" + body + tail;
}

// A UBX message of payload_len bytes (framing 8). The payload holds '$' and '*' bytes, as real ones can.
std::string Ubx(uint8_t cls, uint8_t id, uint16_t payload_len) {
    std::string m = {'\xB5', 'b', static_cast<char>(cls), static_cast<char>(id), static_cast<char>(payload_len & 0xFF),
                     static_cast<char>(payload_len >> 8)};
    for (uint16_t i = 0; i < payload_len; i++) m += static_cast<char>("$GP,*\x01\x7f"[i % 7]);
    m += "\x12\x34";
    return m;
}

// The MAX-M10 epoch of UTC second unix_seconds as our firmware configures it: GGA, RMC, GST (NMEA order assumed) and
// UBX-MON-RF (36 B) + UBX-NAV-SAT (16 + 12 n B), the UBX either before or after the NMEA (u-blox doesn't document it).
std::vector<std::string> MaxM10Epoch(uint64_t unix_seconds, uint16_t num_svs, bool ubx_first) {
    uint32_t sod = unix_seconds % 86400;  // On 2026-11-25 (the date field), unix_seconds from 1795568400.
    char t[16];
    snprintf(t, sizeof(t), "%02u%02u%02u.00", sod / 3600, sod / 60 % 60, sod % 60);
    std::vector<std::string> nmea = {
        WithChecksum(std::string("GNGGA,") + t + ",4404.14063,N,12118.85478,W,1,12,0.91,1147.2,M,-21.3,M,,"),
        WithChecksum(std::string("GNRMC,") + t + ",A,4404.14063,N,12118.85478,W,0.117,,251126,,,A,V"),
        WithChecksum(std::string("GNGST,") + t + ",8.6,,,,1.4,1.1,2.9"),
    };
    std::vector<std::string> ubx = {Ubx(0x0A, 0x38, 4 + 24), Ubx(0x01, 0x35, 8 + 12 * num_svs)};
    std::vector<std::string> out = ubx_first ? ubx : WireMessages(nmea);
    for (const std::string& m : ubx_first ? WireMessages(nmea) : ubx) out.push_back(m);
    return out;
}

constexpr uint64_t kMlatStart = 0x123456789ULL;
constexpr double kRatePPM = 23.456;
uint64_t Mlat(double t) { return kMlatStart + static_cast<uint64_t>(std::llround(t * 48e6 * (1 + kRatePPM * 1e-6))); }

struct Replay {
    uint32_t labeled = 0, restarted = 0, late = 0, outliers = 0, other = 0, dropped_messages = 0;
    double max_rmc_delay_s = 0;  // RMC complete, after the PPS edge of its second.
    PPSClock clock;
};

/**
 * Epoch k (k = 0, 1, ...) describes the PPS edge at GNSS time k + 1. unix_first is the RMC second of epoch 0.
 */
Replay ReplayEpochs(const std::vector<std::vector<std::string>>& epochs, uint64_t unix_first, double latency_s,
           uint32_t baud = 9600, uint32_t tx_buffer_bytes = 4096) {
    Replay r;
    NMEAParser parser;
    const double bytes_per_s = baud / 10.0;
    double line_free_at = 0;
    uint32_t num_edges = 0;
    PPSClock::Edge latest = {}, previous = {};
    auto edge_at = [](double t) {
        return PPSClock::Edge{.mlat_48mhz_counts = Mlat(t),
                              .time_us = static_cast<uint32_t>(0xFFF00000u + static_cast<uint64_t>(t * 1e6))};
    };
    // Messages with their end time on the wire, then edges and messages in time order.
    struct Message {
        double end_s;
        const std::string* bytes;
        uint64_t epoch;
    };
    std::vector<Message> messages;
    std::vector<std::vector<std::string>> wire;
    for (const auto& e : epochs) wire.push_back(WireMessages(e));
    for (uint64_t k = 0; k < wire.size(); k++) {
        double queued_at = k + 1 + latency_s;
        for (const std::string& m : wire[k]) {
            double backlog_bytes = std::max(0.0, (line_free_at - queued_at) * bytes_per_s);
            if (backlog_bytes + m.size() > tx_buffer_bytes) {
                r.dropped_messages++;
                continue;
            }
            line_free_at = std::max(line_free_at, queued_at) + m.size() / bytes_per_s;
            messages.push_back({line_free_at, &m, k});
        }
    }
    double next_edge = 1;
    for (const Message& m : messages) {
        while (next_edge <= m.end_s) {
            previous = latest;
            latest = edge_at(next_edge);
            num_edges++;
            next_edge += 1;
        }
        for (char c : *m.bytes) {
            if (parser.IngestByte(c) != NMEAParser::kSentenceRMC) continue;
            uint64_t unix_seconds = 0;
            bool valid = parser.GetRMCUnixSeconds(unix_seconds);
            if (valid) r.max_rmc_delay_s = std::max(r.max_rmc_delay_s, m.end_s - (unix_seconds - unix_first + 1));
            uint32_t now_us = static_cast<uint32_t>(0xFFF00000u + static_cast<uint64_t>(m.end_s * 1e6));
            switch (r.clock.OnRMC(valid, parser.RMCIsLeapSecond(), unix_seconds, num_edges, latest, previous, now_us)) {
                case PPSClock::kLabeled:
                    r.labeled++;
                    break;
                case PPSClock::kRestarted:
                    r.restarted++;
                    break;
                case PPSClock::kLate:
                    r.late++;
                    break;
                case PPSClock::kOutlier:
                    r.outliers++;
                    break;
                default:
                    r.other++;
                    break;
            }
        }
    }
    return r;
}

// UTC error (s) of the converted MLAT count of GNSS time t, for a fit whose epoch 0 is UTC second unix_first.
double ErrorS(const PPSClock& clock, uint64_t unix_first, double t) {
    uint64_t unix_seconds = 0;
    uint32_t ns = 0;
    EXPECT_TRUE(clock.ToUTC(Mlat(t), unix_seconds, ns));
    return static_cast<double>(unix_seconds) - static_cast<double>(unix_first) + 1 + ns * 1e-9 - t;
}

uint64_t FirstRMCSeconds(const std::vector<std::vector<std::string>>& epochs) {
    NMEAParser parser;
    for (const std::string& s : epochs[0]) {
        for (char c : s + "\r\n") {
            uint64_t unix_seconds = 0;
            if (parser.IngestByte(c) == NMEAParser::kSentenceRMC && parser.GetRMCUnixSeconds(unix_seconds)) {
                return unix_seconds;
            }
        }
    }
    return 0;
}

}  // namespace

// Recorded output of three receivers at 9600 baud: each RMC labels the edge of its own second, at output latencies
// from 20 ms (u-blox) to 250 ms (MediaTek with PMTK255 on, Quectel forum).
TEST(PPSLabelingReplay, RecordedReceiversLabelTheirOwnSecond) {
    const std::pair<const char*, const std::vector<std::vector<std::string>>*> logs[] = {
        {"NEO-M8N", &kUbloxNeoM8n}, {"MT3339", &kMediaTekMT3339}, {"L70", &kQuectelL70}};
    for (const auto& [name, log] : logs) {
        SCOPED_TRACE(name);
        uint64_t unix_first = FirstRMCSeconds(*log);
        ASSERT_GT(unix_first, 1'400'000'000u);
        for (double latency_s : {0.02, 0.1, 0.25}) {
            Replay r = ReplayEpochs(*log, unix_first, latency_s);
            EXPECT_EQ(r.restarted, 1u) << latency_s;  // The first edge.
            EXPECT_EQ(r.labeled, log->size() - 1) << latency_s;
            EXPECT_EQ(r.late + r.outliers + r.other + r.dropped_messages, 0u) << latency_s;
            EXPECT_LT(r.max_rmc_delay_s, 0.8) << latency_s;  // kMaxLabelDelayUs is 0.9 s.
            // Labeled with the right second: the conversion is right to the ns, not off by whole seconds.
            EXPECT_NEAR(ErrorS(r.clock, unix_first, log->size() + 0.5), 0, 50e-9) << latency_s;
        }
    }
}

std::vector<std::vector<std::string>> MaxM10Run(uint64_t unix_first, uint16_t num_svs, bool ubx_first, int n) {
    std::vector<std::vector<std::string>> epochs;
    for (int k = 0; k < n; k++) epochs.push_back(MaxM10Epoch(unix_first + k, num_svs, ubx_first));
    return epochs;
}

// Our MAX-M10 configuration at 9600 baud (960 B/s): 235 B of GGA/RMC/GST/MON-RF + 16 + 12 n B of NAV-SAT.
TEST(PPSLabelingReplay, MaxM10At9600Baud) {
    const uint64_t unix_first = 1795568400;  // 2026-11-25 01:00:00 UTC.
    // 40 satellites (731 B a second): fine in either order.
    for (bool ubx_first : {false, true}) {
        Replay r = ReplayEpochs(MaxM10Run(unix_first, 40, ubx_first, 60), unix_first, 0.02);
        EXPECT_EQ(r.labeled, 59u) << ubx_first;
        EXPECT_EQ(r.late + r.dropped_messages, 0u) << ubx_first;
        EXPECT_NEAR(ErrorS(r.clock, unix_first, 60.5), 0, 50e-9);
    }
    // 56 satellites (914 B): with the UBX first the RMC is done 0.93 s after its edge, too late to label (kLate):
    // no time, but nothing wrong either.
    Replay r = ReplayEpochs(MaxM10Run(unix_first, 56, true, 60), unix_first, 0.02);
    EXPECT_EQ(r.labeled, 0u);
    EXPECT_EQ(r.late, 60u);
    EXPECT_GT(r.max_rmc_delay_s, 0.9);
}

// KNOWN LIMIT: more output than the baud rate carries (here 62 satellites, 995 B a second) backs the stream up by
// whole seconds until the transmit buffer is full and drops messages. Each RMC then arrives within 900 ms of a later
// edge and labels it, consistently, so the fit is self-consistent but whole seconds off: PPSClock can't see it. The
// MAX-M10's TEMPORARY UBX-NAV-SAT debug output at 9600 baud is the only configuration we ship that can get there.
TEST(PPSLabelingReplay, KnownLimitBackloggedStreamIsWholeSecondsOff) {
    const uint64_t unix_first = 1795568400;  // 2026-11-25 01:00:00 UTC.
    const int n = 600;
    Replay r = ReplayEpochs(MaxM10Run(unix_first, 62, false, n), unix_first, 0.02);
    EXPECT_GT(r.dropped_messages, 0u);
    EXPECT_GT(r.max_rmc_delay_s, 2.0);
    ASSERT_TRUE(r.clock.IsSynced());
    double error_s = ErrorS(r.clock, unix_first, n + 0.5);
    EXPECT_LE(error_s, -1.0 + 1e-6);  // UTC reads whole seconds early.
    EXPECT_NEAR(error_s, std::round(error_s), 1e-6);
}
