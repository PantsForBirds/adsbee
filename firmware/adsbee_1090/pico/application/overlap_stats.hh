#pragma once
// Lab-only capture overlap statistics (cmake -DADSBEE_OVERLAP_STATS=ON). Off by default, and idle until AT+OVERLAP=ON.
// Core 1 (receiver ISRs + demodulation) writes one record per capture into a ring; core 0 matches each record to the
// decoded packet it produced (records and packets keep their order) and keeps the histograms.
#ifdef ADSBEE_OVERLAP_STATS

#include <stdint.h>
#include <string.h>

#include "hardware/sync.h"
#include "hardware/timer.h"
#include "mode_s_soft_demod.hh"

namespace overlap_stats {

static constexpr uint8_t kNone = 0xFF;  // No overlapping capture.
static constexpr uint16_t kNumBins = 8;  // 16 us bins of the start offset between two overlapping captures.
static constexpr uint16_t kRingLen = 128;
static constexpr uint16_t kDumpSlots = 4;

enum Role : uint8_t { kIsolated = 0, kFirst = 1, kSecond = 2, kBoth = 3, kNumRoles = 4 };
enum Outcome : uint8_t { kNoMessage = 0, kClean = 1, kFixed = 2, kFailed = 3, kNumOutcomes = 4 };
// Demodulator view of a message: CRC checked here for DF11/17/18 only; address/parity formats need the dictionary.
enum DemodClass : uint8_t { kDemodNone = 0, kDemodClean = 1, kDemodFixed = 2, kDemodCRCFail = 3, kDemodAP = 4 };

// Written by the receiver interrupts per state machine, copied into the capture when it completes.
struct CaptureTag {
    uint8_t later_offset_us = kNone;    // Another capture began this long after this one (first one only).
    uint8_t earlier_offset_us = kNone;  // This capture began this long after another one.
    uint8_t num_later = 0;              // Captures that began during this one (kept or dropped).
    uint8_t duration_us = 0;            // Begin interrupt to capture complete.
    uint8_t earlier_sm = kNone;
};

struct Record {
    uint32_t packet_mlat_lo = 0;   // Low 32 bits of the packet MLAT counts (0 if no message).
    uint32_t begin_48mhz_lo = 0;   // Demod begin, 48 MHz counts.
    uint16_t t_ms = 0;             // Demodulation time, for flushing records whose packet the decoder dropped.
    uint8_t sm = 0;
    uint8_t demod_class = kDemodNone;
    uint8_t df = 0;
    uint8_t len = 0;  // 0 none, 1 = 56 bit, 2 = 112 bit.
    int16_t rssi_mv = 0;
    CaptureTag tag;
};

struct Dump {
    volatile bool full = false;
    uint32_t samples[mode_s_soft_demod::kCaptureWords];
    uint32_t sampler_start_48mhz_lo;
    uint32_t packet_mlat_lo;
    uint16_t seq;
    uint8_t num_words;
    uint8_t sm;
    uint8_t demod_class;
    int16_t rssi_mv;
    CaptureTag tag;
};

// Counts since the last interval line (core 0 copies and clears them; small races only blur one interval).
struct IsrCounts {
    volatile uint32_t begins = 0;               // Accepted demod begins.
    volatile uint32_t overlap_begins = 0;       // Begins while another capture was active.
    volatile uint32_t overlap_aborts = 0;       // ... dropped by the data pulse guard.
    volatile uint32_t mid_noise_aborts = 0;     // Dropped mid-message for too few HI samples.
    volatile uint32_t mid_noise_aborts_ov = 0;  // ... of which overlapped another capture.
    volatile uint32_t no_begin_aborts = 0;      // Mid interrupt without a handled begin.
    volatile uint32_t queue_drops = 0;
    volatile uint32_t queue_hwm = 0;
    volatile uint32_t demod_us = 0;
    volatile uint32_t demods = 0;
    volatile uint32_t ring_full = 0;
    volatile uint32_t dump_full = 0;
};

class Stats {
   public:
    // Settings (core 0 writes, core 1 reads).
    volatile bool enabled = false;
    volatile bool shadow = false;           // Keep overlapping captures instead of dropping them (lab only).
    volatile uint16_t dump_isolated_every = 0;  // Also dump every Nth isolated capture (0 = none).
    volatile bool dump_overlapped = false;
    volatile bool records = false;  // Print one #OVR line per capture.
    volatile uint32_t interval_ms = 10000;

    IsrCounts isr;
    // Aborted second captures by start offset after the first, and by the first being short (56 bit) at the time.
    volatile uint32_t abort_hist[kNumBins][2] = {};

    static inline uint8_t Bin(uint8_t offset_us) { return offset_us >= 112 ? kNumBins - 1 : offset_us / 16; }

    /** Core 1: queue a finished record. */
    inline void Push(const Record& r) {
        uint16_t w = write_;
        if (static_cast<uint16_t>(w - read_) >= kRingLen) {
            isr.ring_full = isr.ring_full + 1;
            return;
        }
        ring_[w % kRingLen] = r;
        __dmb();
        write_ = w + 1;
    }

    /** Core 1: copy a capture into a free dump slot. */
    inline void PushDump(const uint32_t* samples, uint8_t num_words, uint32_t sampler_start, uint32_t packet_mlat,
                         uint8_t sm, uint8_t demod_class, int16_t rssi_mv, const CaptureTag& tag) {
        for (uint16_t i = 0; i < kDumpSlots; i++) {
            Dump& d = dumps_[i];
            if (d.full) continue;
            memcpy(d.samples, samples, num_words * sizeof(uint32_t));
            d.num_words = num_words;
            d.sampler_start_48mhz_lo = sampler_start;
            d.packet_mlat_lo = packet_mlat;
            d.sm = sm;
            d.demod_class = demod_class;
            d.rssi_mv = rssi_mv;
            d.tag = tag;
            d.seq = dump_seq_++;
            __dmb();
            d.full = true;
            return;
        }
        isr.dump_full = isr.dump_full + 1;
    }

    /** Core 0: a decoded packet came out of ingestion. */
    void OnPacket(uint32_t mlat_lo, bool valid, uint8_t corrected_bits, uint8_t df, uint32_t now_ms);

    /** Core 0: flush stale records, print interval lines and dumps. Call from the main loop. */
    void Update(uint32_t now_ms);

    void Reset();
    void PrintHistograms();

   private:
    void Finalize(const Record& r, Outcome outcome);

    Record ring_[kRingLen];
    volatile uint16_t write_ = 0;
    volatile uint16_t read_ = 0;
    Dump dumps_[kDumpSlots];
    uint16_t dump_seq_ = 0;

    // Core 0 histograms since the last reset.
    uint32_t hist_[kNumRoles][kNumBins][kNumOutcomes] = {};
    uint32_t hist_len_[kNumRoles][3][kNumOutcomes] = {};
    // Interval counts kept on core 0.
    uint32_t iv_outcome_[kNumOutcomes] = {};
    uint32_t iv_role_[kNumRoles] = {};
    uint32_t iv_df17_ok_ = 0;
    uint32_t iv_unmatched_ = 0;
    uint32_t last_interval_ms_ = 0;
    uint32_t interval_seq_ = 0;
};

extern Stats stats;

}  // namespace overlap_stats

#endif  // ADSBEE_OVERLAP_STATS
