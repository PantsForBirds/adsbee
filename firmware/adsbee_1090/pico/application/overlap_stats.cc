#include "overlap_stats.hh"

#ifdef ADSBEE_OVERLAP_STATS

#include "adsbee.hh"
#include "comms.hh"
#include "cpu_utils.hh"

extern CPUMonitor core_0_monitor;
extern CPUMonitor core_1_monitor;

namespace overlap_stats {

Stats stats;

static constexpr uint32_t kStaleRecordMs = 500;  // A record whose packet hasn't come out by now was dropped.
static constexpr uint16_t kMaxDumpsPerUpdate = 2;

static Role RoleOf(const CaptureTag& t) {
    bool first = t.later_offset_us != kNone, second = t.earlier_offset_us != kNone;
    return second ? (first ? kBoth : kSecond) : (first ? kFirst : kIsolated);
}

void Stats::Finalize(const Record& r, Outcome outcome) {
    Role role = RoleOf(r.tag);
    uint8_t offset = role == kFirst ? r.tag.later_offset_us : r.tag.earlier_offset_us;
    hist_[role][role == kIsolated ? 0 : Bin(offset)][outcome]++;
    hist_len_[role][r.len < 3 ? r.len : 0][outcome]++;
    iv_outcome_[outcome]++;
    iv_role_[role]++;
    if ((outcome == kClean || outcome == kFixed) && r.df == 17) iv_df17_ok_++;
    if (records) {
        // #OVR,begin_48mhz,sm,len,df,demod_class,outcome,later_us,earlier_us,num_later,duration_us,rssi_mv
        CONSOLE_PRINTF("#OVR,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d\r\n", (unsigned long)r.begin_48mhz_lo, r.sm, r.len, r.df,
                       r.demod_class, outcome, r.tag.later_offset_us, r.tag.earlier_offset_us, r.tag.num_later,
                       r.tag.duration_us, r.rssi_mv);
    }
}

void Stats::OnPacket(uint32_t mlat_lo, bool valid, uint8_t corrected_bits, uint8_t df, uint32_t now_ms) {
    if (!enabled) return;
    // Packets come out in capture order. Records before the match had no packet (no message, or dropped by the
    // decoder as invalid or a duplicate).
    while (read_ != write_) {
        __dmb();
        const Record& r = ring_[read_ % kRingLen];
        if (r.demod_class != kDemodNone && r.packet_mlat_lo == mlat_lo) {
            Outcome o = !valid ? kFailed
                               : ((corrected_bits > 0 || r.demod_class == kDemodFixed) ? kFixed : kClean);
            Finalize(r, o);
            read_ = read_ + 1;
            return;
        }
        Finalize(r, r.demod_class == kDemodNone ? kNoMessage : kFailed);
        read_ = read_ + 1;
    }
    iv_unmatched_++;  // No record (ring was full or stats were just turned on).
}

void Stats::Update(uint32_t now_ms) {
    if (!enabled) return;
    // Flush the head of the ring: records without a message at once, others when stale.
    while (read_ != write_) {
        __dmb();
        const Record& r = ring_[read_ % kRingLen];
        if (r.demod_class != kDemodNone && static_cast<uint16_t>(now_ms - r.t_ms) < kStaleRecordMs) break;
        Finalize(r, r.demod_class == kDemodNone ? kNoMessage : kFailed);
        read_ = read_ + 1;
    }

    for (uint16_t n = 0, i = 0; i < kDumpSlots && n < kMaxDumpsPerUpdate; i++) {
        Dump& d = dumps_[i];
        if (!d.full) continue;
        __dmb();
        CONSOLE_PRINTF("#OVD,%u,%u,%lu,%lu,%u,%u,%u,%u,%u,%u,%d,%u,", d.seq, d.sm, (unsigned long)d.sampler_start_48mhz_lo,
                       (unsigned long)d.packet_mlat_lo, d.demod_class, d.tag.later_offset_us, d.tag.earlier_offset_us,
                       d.tag.num_later, d.tag.duration_us, d.tag.earlier_sm, d.rssi_mv, d.num_words);
        char hex[mode_s_soft_demod::kCaptureWords * 8 + 3];
        for (uint16_t w = 0; w < d.num_words; w++) snprintf(hex + 8 * w, 9, "%08lx", (unsigned long)d.samples[w]);
        strcpy(hex + 8 * d.num_words, "\r\n");
        CONSOLE_PRINTF("%s", hex);
        __dmb();
        d.full = false;
        n++;
    }

    if (now_ms - last_interval_ms_ < interval_ms) return;
    uint32_t dt_ms = now_ms - last_interval_ms_;
    last_interval_ms_ = now_ms;
    IsrCounts c;
    memcpy((void*)&c, (const void*)&isr, sizeof(c));
    // Clear (the queue high water mark included: it restarts every interval).
    isr.begins = 0; isr.overlap_begins = 0; isr.overlap_aborts = 0; isr.mid_noise_aborts = 0;
    isr.mid_noise_aborts_ov = 0; isr.no_begin_aborts = 0; isr.queue_drops = 0; isr.queue_hwm = 0; isr.demod_us = 0;
    isr.demods = 0; isr.ring_full = 0; isr.dump_full = 0;
    CONSOLE_PRINTF(
        "#OV,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u,%u,%u,%d\r\n",
        (unsigned long)interval_seq_++, (unsigned long)dt_ms, (unsigned long)c.begins, (unsigned long)c.overlap_begins,
        (unsigned long)c.overlap_aborts, (unsigned long)c.mid_noise_aborts, (unsigned long)c.mid_noise_aborts_ov,
        (unsigned long)c.no_begin_aborts, (unsigned long)c.queue_drops, (unsigned long)c.queue_hwm,
        (unsigned long)c.demods, (unsigned long)c.demod_us, (unsigned long)c.ring_full, (unsigned long)c.dump_full,
        (unsigned long)iv_outcome_[kNoMessage], (unsigned long)iv_outcome_[kClean], (unsigned long)iv_outcome_[kFixed],
        (unsigned long)iv_outcome_[kFailed], (unsigned long)iv_role_[kIsolated], (unsigned long)iv_role_[kFirst],
        (unsigned long)iv_role_[kSecond], (unsigned long)iv_role_[kBoth], (unsigned long)iv_df17_ok_,
        (unsigned long)iv_unmatched_, core_0_monitor.GetUsagePercent(), core_1_monitor.GetUsagePercent(),
        adsbee.aircraft_dictionary.GetNumAircraft(), adsbee.GetNoiseFloordBm());
    memset(iv_outcome_, 0, sizeof(iv_outcome_));
    memset(iv_role_, 0, sizeof(iv_role_));
    iv_df17_ok_ = 0;
    iv_unmatched_ = 0;
}

void Stats::Reset() {
    memset(hist_, 0, sizeof(hist_));
    memset(hist_len_, 0, sizeof(hist_len_));
    memset((void*)abort_hist, 0, sizeof(abort_hist));
}

void Stats::PrintHistograms() {
    // #OVH,role,bin,nomsg,clean,fixed,failed (bins of 16 us start offset; isolated in bin 0).
    for (uint16_t role = 0; role < kNumRoles; role++) {
        for (uint16_t bin = 0; bin < kNumBins; bin++) {
            const uint32_t* h = hist_[role][bin];
            if (h[0] + h[1] + h[2] + h[3] == 0) continue;
            CONSOLE_PRINTF("#OVH,%u,%u,%lu,%lu,%lu,%lu\r\n", role, bin, (unsigned long)h[0], (unsigned long)h[1],
                           (unsigned long)h[2], (unsigned long)h[3]);
        }
        for (uint16_t len = 0; len < 3; len++) {
            const uint32_t* h = hist_len_[role][len];
            if (h[0] + h[1] + h[2] + h[3] == 0) continue;
            CONSOLE_PRINTF("#OVL,%u,%u,%lu,%lu,%lu,%lu\r\n", role, len, (unsigned long)h[0], (unsigned long)h[1],
                           (unsigned long)h[2], (unsigned long)h[3]);
        }
    }
    // #OVA,bin,first_long,first_short: dropped second captures by start offset.
    for (uint16_t bin = 0; bin < kNumBins; bin++) {
        CONSOLE_PRINTF("#OVA,%u,%lu,%lu\r\n", bin, (unsigned long)abort_hist[bin][0], (unsigned long)abort_hist[bin][1]);
    }
}

}  // namespace overlap_stats

#endif  // ADSBEE_OVERLAP_STATS
