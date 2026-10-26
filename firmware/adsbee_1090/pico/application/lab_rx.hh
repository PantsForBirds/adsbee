#pragma once
// Lab-only receive path instrumentation and trigger experiments (cmake -DADSBEE_LAB_RX=ON, t-0875). Off by default,
// and idle until AT+LABRX turns records on. Never in a release build.
//
// Records: one per capture event on the receiver core (ISR aborts, demodulation outcomes), handed to core 0 through a
// ring and printed as "#LC" lines. Core 0 prints one "#LP" line per decoded packet it ingests (the downstream outcome),
// "#LS" interval lines (counts, demod time, queue high water mark, noise floor, TL) and "#LD" sample dumps.
//
// Trigger (AT+LABTRIG): the preamble matcher's chip pattern, the number of chips it checks before it starts the
// sampler, and its mismatch budget. Checking fewer chips starts the sampler earlier, so the capture holds the rest of
// the preamble; software can then score it (sw_mode 1) or skip any preamble check (sw_mode 2, for upper bounds).
#ifdef ADSBEE_LAB_RX

#include <stdint.h>
#include <string.h>

#include "hardware/sync.h"
#include "mode_s_soft_demod.hh"

namespace lab_rx {

static constexpr uint16_t kMaxExtraWords = 3;  // Up to 14 chips (84 samples) of preamble before the data.
static constexpr uint16_t kLabCaptureWords = mode_s_soft_demod::kCaptureWords + kMaxExtraWords;
static constexpr uint16_t kRingLen = 128;
static constexpr uint16_t kDumpSlots = 6;
static constexpr uint32_t kDefaultPattern = 0b00110011001010001100110010101010;  // Same as capture.pio.

// Record kinds (printed as letters).
enum Kind : uint8_t {
    kGuardAbort = 'G',    // Began within the guard time of another capture: dropped in OnDemodBegin().
    kMidAbort = 'M',      // Too few HI samples at the mid-message check.
    kNoBeginAbort = 'B',  // Mid interrupt without a handled begin.
    kQueueDrop = 'O',     // Capture queue full.
    kPreambleReject = 'P',  // Software preamble check failed.
    kNoMessage = 'N',     // Demodulator found no message (ties).
    kNotValidated = 'C',  // Demodulated, CRC failed (DF11/17/18) or a format without parity.
    kClean = 'K',         // CRC passed without changes.
    kFlipped = 'F',       // CRC passed after the demodulator's bit flips.
    kRetry = 'R',         // Validated at a retried offset.
    kAddressParity = 'A',  // Address/parity format, passed on for the dictionary to check.
};

struct Settings {
    volatile bool records = false;     // Print #LC/#LP lines.
    volatile uint32_t dump_mask = 0;   // Bit (kind - 'A') set: dump the samples of captures of that kind.
    volatile uint32_t record_mask = 0x3FFFFFF;  // Bit (kind - 'A') set: print #LC records of that kind.
    volatile uint32_t interval_ms = 1000;
    // Trigger.
    uint32_t pattern = kDefaultPattern;
    uint8_t num_chips = 15;  // Chips the matcher steps through before it raises the demod pin (1-15).
    uint8_t budget = 4;      // Matcher mismatch budget (0-31).
    volatile uint8_t sw_mode = 0;    // 0 none, 1 software preamble check, 2 no check, 3 score only (no reject).
    volatile uint8_t sw_budget = 4;  // Mismatched samples allowed by the software check.
    volatile bool guard = true;      // Data pulse guard in OnDemodBegin().
    volatile bool mid_check = true;  // HI sample check at the mid-message interrupt.
    volatile bool retry = true;      // #398 offset retry.

    uint16_t ExtraSamples() const { return (15 - num_chips) * mode_s_soft_demod::kSamplesPerChip; }
    uint16_t ExtraWords() const { return num_chips >= 15 ? 0 : (ExtraSamples() + 4 + 31) / 32; }
};

struct Record {
    uint32_t t48 = 0;          // Sampler start (or demod begin for ISR aborts), low 32 bits of the 48 MHz counter.
    uint32_t pkt_mlat_lo = 0;  // Packet MLAT counts (0 if no message).
    uint32_t buf[4] = {0};     // Demodulated bits (also for failures).
    int16_t rssi_mv = 0;
    uint16_t hi_samples = 0;   // Mid check HI count (of 192).
    uint16_t demod_us = 0;
    uint8_t kind = 0;
    uint8_t sm = 0;
    uint8_t offset = 0;        // Start offset used (samples, relative to the shifted buffer).
    uint8_t flips = 0;
    uint8_t retries = 0;
    uint8_t len = 0;           // Bits / 8.
    uint8_t pre_mismatch = 0xFF;  // Software preamble mismatches at the best alignment (0xFF = not scored).
    int8_t pre_align = 0;
    uint8_t queue_len = 0;
};

struct Dump {
    volatile bool full = false;
    uint32_t samples[kLabCaptureWords];
    uint32_t t48;
    uint8_t num_words;
    uint8_t kind;
    uint8_t sm;
    uint16_t seq;
};

struct IntervalCounts {
    volatile uint32_t begins = 0;  // Demod pin rising edges handled.
    volatile uint32_t kind[26] = {};
    volatile uint32_t demod_us = 0;
    volatile uint32_t demod_us_max = 0;
    volatile uint32_t demods = 0;
    volatile uint32_t queue_hwm = 0;
    volatile uint32_t ring_full = 0;
    volatile uint32_t dump_full = 0;
};

class Lab {
   public:
    Settings settings;
    IntervalCounts iv;

    /** Receiver core, any context. */
    inline void Push(const Record& r) {
        if (r.kind >= 'A' && r.kind <= 'Z') iv.kind[r.kind - 'A'] = iv.kind[r.kind - 'A'] + 1;
        if (!settings.records || r.kind < 'A' || r.kind > 'Z' || !(settings.record_mask & (1u << (r.kind - 'A'))))
            return;
        uint32_t irq = save_and_disable_interrupts();
        uint16_t w = write_;
        if (static_cast<uint16_t>(w - read_) >= kRingLen) {
            iv.ring_full = iv.ring_full + 1;
        } else {
            ring_[w % kRingLen] = r;
            __dmb();
            write_ = w + 1;
        }
        restore_interrupts(irq);
    }
    inline void Abort(uint8_t kind, uint32_t t48, uint8_t sm, uint16_t hi_samples = 0) {
        Record r;
        r.kind = kind;
        r.t48 = t48;
        r.sm = sm;
        r.hi_samples = hi_samples;
        Push(r);
    }
    /** Receiver core, any context. */
    inline void PushDump(uint8_t kind, const uint32_t* samples, uint8_t num_words, uint32_t t48, uint8_t sm) {
        if (!settings.records || kind < 'A' || kind > 'Z' || !(settings.dump_mask & (1u << (kind - 'A')))) return;
        uint32_t irq = save_and_disable_interrupts();
        for (uint16_t i = 0; i < kDumpSlots; i++) {
            Dump& d = dumps_[i];
            if (d.full) continue;
            memcpy(d.samples, samples, num_words * sizeof(uint32_t));
            d.num_words = num_words;
            d.t48 = t48;
            d.kind = kind;
            d.sm = sm;
            d.seq = dump_seq_++;
            __dmb();
            d.full = true;
            restore_interrupts(irq);
            return;
        }
        iv.dump_full = iv.dump_full + 1;
        restore_interrupts(irq);
    }

    /** Core 0: a packet came out of the decoder into the dictionary. */
    void OnPacket(uint32_t mlat_lo, bool valid, uint8_t corrected_bits, uint8_t df, const uint32_t* buf,
                  uint8_t len_bytes);
    /** Core 0 main loop: print records, dumps and interval lines. */
    void Update(uint32_t now_ms, int noise_floor_mv, int tl_offset_mv, int tl_mv, bool learning, int learn_temp_mv);

   private:
    Record ring_[kRingLen];
    volatile uint16_t write_ = 0;
    volatile uint16_t read_ = 0;
    Dump dumps_[kDumpSlots];
    uint16_t dump_seq_ = 0;
    uint32_t last_interval_ms_ = 0;
    uint32_t interval_seq_ = 0;
    uint32_t packets_ = 0;
    uint32_t packets_valid_ = 0;
};

extern Lab lab;

/**
 * Software preamble score. The sampler started at chip num_chips of the preamble (chip c starts about 6 * (c -
 * num_chips) - 1 samples into the capture). Counts the samples that disagree with the preamble pattern in the 4 inner
 * samples of each checked chip (the matcher's chips: pulses 0, 2, 7, 9; quiet 4, 5, 11-14) at alignments -2..+3
 * samples, and returns the smallest count; align gets the alignment (minus 6 per chip when the trigger was the
 * second pulse, tried for a one chip trigger).
 */
uint8_t ScorePreamble(const uint32_t* samples, uint8_t num_chips, int8_t& align);

/**
 * Copies a capture shifted left by shift samples (bits) into out (out_words words).
 */
void ShiftSamples(const uint32_t* in, uint16_t in_words, uint16_t shift, uint32_t* out, uint16_t out_words);

}  // namespace lab_rx

#endif  // ADSBEE_LAB_RX
