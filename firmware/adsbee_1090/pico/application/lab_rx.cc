#include "lab_rx.hh"

#ifdef ADSBEE_LAB_RX

#include <stdio.h>

#include "comms.hh"

namespace lab_rx {

Lab lab;

static constexpr uint16_t kMaxRecordsPerUpdate = 24;
static constexpr uint16_t kMaxDumpsPerUpdate = 2;

void Lab::OnPacket(uint32_t mlat_lo, bool valid, uint8_t corrected_bits, uint8_t df, const uint32_t* buf,
                   uint8_t len_bytes) {
    packets_++;
    if (valid) packets_valid_++;
    if (!settings.records) return;
    if (len_bytes > 7) {
        CONSOLE_PRINTF("#LP,%lu,%d,%u,%u,%08lx%08lx%08lx%04lx\r\n", (unsigned long)mlat_lo, valid, corrected_bits, df,
                       (unsigned long)buf[0], (unsigned long)buf[1], (unsigned long)buf[2],
                       (unsigned long)(buf[3] >> 16));
    } else {
        CONSOLE_PRINTF("#LP,%lu,%d,%u,%u,%08lx%06lx\r\n", (unsigned long)mlat_lo, valid, corrected_bits, df,
                       (unsigned long)buf[0], (unsigned long)(buf[1] >> 8));
    }
}

void Lab::Update(uint32_t now_ms, int noise_floor_mv, int tl_offset_mv, int tl_mv, bool learning, int learn_temp_mv) {
    for (uint16_t n = 0; n < kMaxRecordsPerUpdate && read_ != write_; n++) {
        __dmb();
        Record r = ring_[read_ % kRingLen];
        __dmb();
        read_ = read_ + 1;
        // #LC,t48,kind,sm,len,offset,flips,retries,rssi_mv,hi,pre_mismatch,pre_align,queue_len,demod_us,bits,pkt_mlat
        CONSOLE_PRINTF("#LC,%lu,%c,%u,%u,%u,%u,%u,%d,%u,%u,%d,%u,%u,%08lx%08lx%08lx%04lx,%lu\r\n", (unsigned long)r.t48,
                       r.kind, r.sm, r.len, r.offset, r.flips, r.retries, r.rssi_mv, r.hi_samples, r.pre_mismatch,
                       r.pre_align, r.queue_len, r.demod_us, (unsigned long)r.buf[0], (unsigned long)r.buf[1],
                       (unsigned long)r.buf[2], (unsigned long)(r.buf[3] >> 16), (unsigned long)r.pkt_mlat_lo);
    }
    for (uint16_t n = 0, i = 0; i < kDumpSlots && n < kMaxDumpsPerUpdate; i++) {
        Dump& d = dumps_[i];
        if (!d.full) continue;
        __dmb();
        char line[16 + 8 * kLabCaptureWords + 3];
        int pos = 0;
        for (uint16_t w = 0; w < d.num_words; w++) {
            pos += snprintf(line + pos, sizeof(line) - pos, "%08lx", (unsigned long)d.samples[w]);
        }
        CONSOLE_PRINTF("#LD,%u,%lu,%c,%u,%u,%s\r\n", d.seq, (unsigned long)d.t48, d.kind, d.sm, d.num_words, line);
        __dmb();
        d.full = false;
        n++;
    }
    if (settings.interval_ms == 0 || now_ms - last_interval_ms_ < settings.interval_ms) return;
    last_interval_ms_ = now_ms;
    // Snapshot and clear (small races only blur one interval).
    uint32_t k[26];
    for (uint16_t i = 0; i < 26; i++) {
        k[i] = iv.kind[i];
        iv.kind[i] = 0;
    }
    uint32_t begins = iv.begins, demods = iv.demods, demod_us = iv.demod_us, demod_us_max = iv.demod_us_max,
             qhwm = iv.queue_hwm, ring_full = iv.ring_full, dump_full = iv.dump_full;
    iv.begins = 0;
    iv.demods = 0;
    iv.demod_us = 0;
    iv.demod_us_max = 0;
    iv.queue_hwm = 0;
    iv.ring_full = 0;
    iv.dump_full = 0;
    uint32_t packets = packets_, valid = packets_valid_;
    packets_ = 0;
    packets_valid_ = 0;
#define K(c) (unsigned long)k[(c) - 'A']
    // #LS,seq,ms,begins,G,M,B,O,P,N,C,K,F,R,A,demods,demod_us,demod_us_max,qhwm,ring_full,dump_full,packets,valid,
    //     nf_mv,tl_offset_mv,tl_mv,learning,learn_temp_mv
    CONSOLE_PRINTF(
        "#LS,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%d,%d,%d,%d,%d\r\n",
        (unsigned long)interval_seq_++, (unsigned long)now_ms, (unsigned long)begins, K('G'), K('M'), K('B'), K('O'),
        K('P'), K('N'), K('C'), K('K'), K('F'), K('R'), K('A'), (unsigned long)demods, (unsigned long)demod_us,
        (unsigned long)demod_us_max, (unsigned long)qhwm, (unsigned long)ring_full, (unsigned long)dump_full,
        (unsigned long)packets, (unsigned long)valid, noise_floor_mv, tl_offset_mv, tl_mv, learning, learn_temp_mv);
#undef K
}

static inline uint32_t Sample(const uint32_t* samples, int i) { return (samples[i >> 5] >> (31 - (i & 31))) & 1; }

uint8_t ScorePreamble(const uint32_t* samples, uint8_t num_chips, int8_t& align) {
    static constexpr uint8_t kPulseChips[] = {0, 2, 7, 9};
    static constexpr uint8_t kQuietChips[] = {4, 5, 11, 12, 13, 14};
    uint8_t best = 0xFF;
    align = 0;
    // Trigger on chip 0 (the first pulse), or, for a one chip trigger, on chip 2 when the first pulse was missed.
    for (int h = 0; h <= (num_chips == 1 ? 2 : 0); h += 2) {
        for (int a = -8; a <= 9; a++) {
            uint8_t mismatches = 0;
            for (uint8_t c : kPulseChips) {
                if (c < num_chips + h) continue;
                int start = 6 * (c - num_chips - h) - 1 + a;
                for (int s = 1; s <= 4; s++) {
                    if (start + s < 0 || !Sample(samples, start + s)) mismatches++;  // Before the capture counts as a miss.
                }
            }
            for (uint8_t c : kQuietChips) {
                if (c < num_chips + h) continue;
                int start = 6 * (c - num_chips - h) - 1 + a;
                for (int s = 1; s <= 4; s++) {
                    if (start + s < 0 || Sample(samples, start + s)) mismatches++;
                }
            }
            if (mismatches < best) {
                best = mismatches;
                align = static_cast<int8_t>(a - 6 * h);
            }
        }
    }
    return best;
}

void ShiftSamples(const uint32_t* in, uint16_t in_words, uint16_t shift, uint32_t* out, uint16_t out_words) {
    uint16_t q = shift >> 5, r = shift & 31;
    for (uint16_t i = 0; i < out_words; i++) {
        uint32_t a = i + q < in_words ? in[i + q] : 0;
        uint32_t b = i + q + 1 < in_words ? in[i + q + 1] : 0;
        out[i] = r == 0 ? a : (a << r) | (b >> (32 - r));
    }
}

}  // namespace lab_rx

#endif  // ADSBEE_LAB_RX
