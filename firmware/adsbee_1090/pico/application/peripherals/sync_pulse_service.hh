#pragma once

#include <atomic>
#include <cstdint>

#include "comms.hh"
#include "mlat_time_sync.hh"
#include "object_dictionary.hh"

/**
 * Sync pulses put coprocessor timestamps on the RP2040's 48 MHz MLAT clock. The core that owns the MLAT counter raises
 * the shared SYNC line (CC1312 DIO5, ESP32-S3 GPIO38) for ~1 us about every kIntervalMs, and as soon as the pulses
 * are allowed again, and stores the MLAT count of the edge. Core 0 hands the latest count to each coprocessor
 * (ObjectDictionary::SyncPulse) on traffic it already has: the ESP32 gets it in every RP2040 status write
 * (GetLastPulse()), the CC1312 in one small un-acked write after its next status read (TakeNewPulse()). Each
 * coprocessor pairs a count with the captured edge whose time its fit predicts closest to it (SyncPulsePairing), so
 * a count may arrive late, twice or not at all.
 *
 * Each interval is longer by 0 to kStaggerUs (from the last count's low bits), so the pulses are never exactly
 * periodic. A fit whose first two points both took the next pulse's edge (each count arriving just after it, see
 * SyncPulsePairing) then misses its later points by hundreds of microseconds, and MLATTimeSync's outlier rule restarts
 * it. With exact 1 s intervals it would stay a pulse off.
 *
 * SYNC is also the CC1312 bootloader backdoor (high at reset = bootloader), so the CC1312 driver gates the pulses:
 * Allow() once its application has answered a status read or while it holds the CC1312 in reset, Block() before any
 * reset release, bootloader session or flash, after a failed status read, and before core 1 stops.
 *
 * Threads: Service() runs on the core that owns the MLAT counter; everything else on core 0.
 */
class SyncPulseService {
   public:
    enum Subscriber : uint8_t { kSubscriberCC1312 = 0, kSubscriberESP32, kNumSubscribers };

    static constexpr uint32_t kIntervalMs = ObjectDictionary::kSyncPulseIntervalMs;
    static constexpr uint32_t kIntervalUs = kIntervalMs * 1000;
    // Well above MLATTimeSync::kMaxResidualUs, well below SyncPulsePairing::kToleranceUs.
    static constexpr uint32_t kStaggerUs = 500;
    static_assert(kStaggerUs > 10 * MLATTimeSync::kMaxResidualUs && kStaggerUs < SyncPulsePairing::kToleranceUs);
    static_assert(ObjectDictionary::kSyncPointMaxAgeMs * 1000ull <= MLATTimeSync::kMaxExtrapolationUs);
    static_assert(ObjectDictionary::kSyncPointMaxAgeMs * 1000ull <= MLATTimeSync::kMaxSyncIntervalUs);
    static_assert(ObjectDictionary::kSyncPointMaxAgeMs > 2 * kIntervalMs);
    static_assert(SyncPulsePairing::kMaxFirstEdgeAgeUs < kIntervalUs);

    // Raises SYNC, returns the MLAT count of the rising edge, lowers SYNC. Runs with the pulse allowed.
    using Emitter = uint64_t (*)();

    explicit SyncPulseService(Emitter emitter = nullptr) : emitter_(emitter) {}

    // No emitter (no SYNC pin): no pulses.
    void SetEmitter(Emitter emitter) { emitter_ = emitter; }

    void Allow() { allowed_ = true; }

    // Stops pulses and returns once none is in flight. The next Service() after Allow() sends one.
    void Block() {
        allowed_ = false;
        Fence();
        while (busy_) {
        }
        pulse_now_ = true;
        // The CC1312 may restart: a count from before would reach it just after the next pulse's edge, which it would
        // take for that count's (its first point needs only a young edge).
        delivered_mlat_48mhz_counts_ = GetLastPulse().mlat_48mhz_counts;
    }

    bool IsAllowed() const { return allowed_; }

    // Sends a pulse when one is due. Call often from the core that owns the MLAT counter (timestamp_us: time_us_32()).
    void Service(uint32_t timestamp_us) {
        if (!allowed_ || emitter_ == nullptr ||
            (!pulse_now_ && timestamp_us - last_pulse_timestamp_us_ < interval_us_)) {
            return;
        }
        // Checked against Block(): either this sees the pulse blocked, or Block() sees it busy and waits.
        busy_ = true;
        Fence();
        if (allowed_) {
            uint64_t mlat_48mhz_counts = emitter_();
            // Seqlock: core 0 reads the 64-bit count while seq_ is even and unchanged.
            seq_ = seq_ + 1;
            Fence();
            mlat_48mhz_counts_ = mlat_48mhz_counts;
            Fence();
            seq_ = seq_ + 1;
            last_pulse_timestamp_us_ = timestamp_us;
            interval_us_ =
                kIntervalUs + static_cast<uint32_t>(mlat_48mhz_counts / MLATTimeSync::kMLATTicksPerUs) % kStaggerUs;
            pulse_now_ = false;
        }
        Fence();
        busy_ = false;
    }

    // The latest pulse (mlat_48mhz_counts 0: none yet), for the ESP32 status write.
    ObjectDictionary::SyncPulse GetLastPulse() const {
        ObjectDictionary::SyncPulse pulse;
        uint32_t seq;
        do {
            seq = seq_;
            Fence();
            pulse.mlat_48mhz_counts = mlat_48mhz_counts_;
            Fence();
        } while ((seq & 1) || seq != seq_);
        return pulse;
    }

    // Gets a pulse not given yet, for the CC1312 write.
    bool TakeNewPulse(ObjectDictionary::SyncPulse& pulse) {
        pulse = GetLastPulse();
        if (pulse.mlat_48mhz_counts == 0 || pulse.mlat_48mhz_counts == delivered_mlat_48mhz_counts_) return false;
        delivered_mlat_48mhz_counts_ = pulse.mlat_48mhz_counts;
        return true;
    }

    // ObjectDictionary::MLATSyncFlag bits from the subscriber's last status read.
    void ReportFlags(Subscriber subscriber, uint8_t flags) {
        LogSyncChange(subscriber, flags);
        flags_[subscriber] = flags;
    }

    // The subscriber's status can't be read (link down, coprocessor disabled or resetting): its sync doesn't count.
    void ReportAbsent(Subscriber subscriber) { ReportFlags(subscriber, 0); }

   private:
    static constexpr const char* kSubscriberNames[kNumSubscribers] = {"Sub-GHz", "Remote ID"};

    static void Fence() { std::atomic_thread_fence(std::memory_order_seq_cst); }

    void LogSyncChange(Subscriber subscriber, uint8_t flags) {
        bool synced = flags & ObjectDictionary::kMLATSyncFlagSynced;
        if (synced != static_cast<bool>(flags_[subscriber] & ObjectDictionary::kMLATSyncFlagSynced)) {
            CONSOLE_INFO("SyncPulseService::ReportFlags", "%s timestamps %s.", kSubscriberNames[subscriber],
                         synced ? "on the MLAT clock" : "stopped until the next sync pulses");
        }
    }

    Emitter emitter_;

    // Shared between the cores.
    volatile bool allowed_ = false;
    volatile bool busy_ = false;               // Service() is past its allowed_ check.
    volatile bool pulse_now_ = true;           // Send the next pulse at once (the pulses were blocked).
    volatile uint32_t seq_ = 0;                // Odd while mlat_48mhz_counts_ is written.
    volatile uint64_t mlat_48mhz_counts_ = 0;  // Of the last pulse.

    // Pulse core only.
    uint32_t last_pulse_timestamp_us_ = 0;
    uint32_t interval_us_ = kIntervalUs;  // To the next pulse.

    // Core 0 only.
    uint64_t delivered_mlat_48mhz_counts_ = 0;  // Last count given to the CC1312.
    uint8_t flags_[kNumSubscribers] = {};
};
