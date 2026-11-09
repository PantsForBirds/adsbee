#pragma once

#include <atomic>
#include <cstdint>

#include "comms.hh"
#include "mlat_time_sync.hh"
#include "object_dictionary.hh"

/**
 * Sync pulses put coprocessor timestamps on the RP2040's 48 MHz MLAT clock. The RP2040 raises the shared SYNC line
 * (CC1312 DIO5, ESP32-S3 GPIO38) for ~1 us, then gives each coprocessor the MLAT count of that edge
 * (ObjectDictionary::SyncPulse) on traffic it already has: the ESP32 gets it in the RP2040 status written every ~100 ms
 * (GetStatusPulse()), the CC1312 in one small un-acked write after its next status read (TakeNewPulse()). Each
 * coprocessor captures the edge on its own clock, fits offset and rate, and sends packets on the MLAT time base.
 *
 * Cadence: a pulse every kIntervalMs (1 s), and one as soon as allowed (kSyncPulseMinIntervalMs after the last) when a
 * coprocessor asks: boot, coprocessor or sub-GHz receiver restart, a skipped point, a dropped fit.
 *
 * A coprocessor pairs a count with the last edge it captured, so a count must reach it before the next pulse's edge.
 * Update() never requests a pulse in the call that records one (the drivers deliver it in that main loop pass), and
 * a request drops a count that hasn't been delivered.
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
    static constexpr uint32_t kRequestTimeoutMs = 500;  // A request not served by then is dropped.
    static_assert(ObjectDictionary::kSyncPulseMinIntervalMs > ObjectDictionary::kSyncPulseMaxAgeMs);
    static_assert(ObjectDictionary::kSyncPointMaxAgeMs * 1000ull <= MLATTimeSync::kMaxExtrapolationUs);
    static_assert(ObjectDictionary::kSyncPointMaxAgeMs * 1000ull <= MLATTimeSync::kMaxSyncIntervalUs);
    static_assert(ObjectDictionary::kSyncPointMaxAgeMs > 2 * kIntervalMs);

    // Raises SYNC, returns the MLAT count of the rising edge, lowers SYNC. Runs with the pulse allowed.
    using Emitter = uint64_t (*)();

    explicit SyncPulseService(Emitter emitter = nullptr) : emitter_(emitter) {}

    // No emitter (no SYNC pin): no pulses.
    void SetEmitter(Emitter emitter) { emitter_ = emitter; }

    void Allow() { allowed_ = true; }

    // Stops pulses and returns once none is in flight.
    void Block() {
        allowed_ = false;
        requested_ = false;
        Fence();
        while (busy_) {
        }
    }

    bool IsAllowed() const { return allowed_; }

    // Sends a requested pulse. Call from the core that owns the MLAT counter.
    void Service() {
        if (!requested_) {
            return;
        }
        // Checked against Block(): either this sees the pulse blocked, or Block() sees it busy and waits.
        busy_ = true;
        Fence();
        if (allowed_ && emitter_ != nullptr) {
            uint64_t mlat_48mhz_counts = emitter_();
            last_mlat_48mhz_counts_ = mlat_48mhz_counts;
            Fence();
            num_pulses_sent_ = num_pulses_sent_ + 1;
        }
        requested_ = false;
        Fence();
        busy_ = false;
    }

    // Schedules pulses and records sent ones. Call every main loop pass, before the subscriber drivers' updates.
    void Update(uint32_t timestamp_ms) {
        uint32_t num_pulses_sent = num_pulses_sent_;
        if (num_pulses_sent != last_pulse_.pulse_num) {
            Fence();
            last_pulse_ = {.mlat_48mhz_counts = last_mlat_48mhz_counts_, .pulse_num = num_pulses_sent};
            last_pulse_current_ = true;
            // No new pulse in this pass: the subscribers take this one first. A main loop pass can be longer than
            // kIntervalMs (output overload): a request here would send the next edge before this count.
            return;
        }
        if (!allowed_ || emitter_ == nullptr) {
            return;
        }
        uint32_t ms_since_request = timestamp_ms - request_timestamp_ms_;
        if (requested_) {
            if (ms_since_request > kRequestTimeoutMs) {
                requested_ = false;
            }
            return;
        }
        if (ms_since_request >= kIntervalMs ||
            (AnyPulseWanted() && ms_since_request >= ObjectDictionary::kSyncPulseMinIntervalMs)) {
            // A count not delivered by now never is: it would arrive after the next edge. The coprocessor then sees
            // two edges for two pulses at the next count, which still pairs.
            last_pulse_current_ = false;
            request_timestamp_ms_ = timestamp_ms;
            Fence();
            requested_ = true;
        }
    }

    // True if a pulse went out that this subscriber hasn't been given yet.
    bool HasNewPulse(Subscriber subscriber) const {
        return last_pulse_current_ && last_pulse_.pulse_num != delivered_pulse_num_[subscriber];
    }

    // Gets the last pulse for a subscriber to write to its coprocessor, once.
    bool TakeNewPulse(Subscriber subscriber, ObjectDictionary::SyncPulse& pulse) {
        if (!HasNewPulse(subscriber)) {
            return false;
        }
        pulse = last_pulse_;
        delivered_pulse_num_[subscriber] = pulse.pulse_num;
        return true;
    }

    // The pulse for a status message sent every main loop pass or so (RP2040DeviceStatus::sync_pulse): the last pulse
    // until the next is requested, then none (pulse_num 0). The receiver takes each pulse_num once, so a lost status
    // write costs nothing.
    ObjectDictionary::SyncPulse GetStatusPulse() const {
        return last_pulse_current_ ? last_pulse_ : ObjectDictionary::SyncPulse{};
    }

    const ObjectDictionary::SyncPulse& GetLastPulse() const { return last_pulse_; }

    // ObjectDictionary::MLATSyncFlag bits from the subscriber's last status read.
    void ReportFlags(Subscriber subscriber, uint8_t flags) {
        LogSyncChange(subscriber, flags);
        flags_[subscriber] = flags;
    }

    // The subscriber's status can't be read (link down, coprocessor disabled or resetting): its sync doesn't count.
    void ReportAbsent(Subscriber subscriber) {
        LogSyncChange(subscriber, 0);
        flags_[subscriber] = 0;
    }

    uint8_t GetFlags(Subscriber subscriber) const { return flags_[subscriber]; }

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

    bool AnyPulseWanted() const {
        for (uint8_t flags : flags_) {
            if (flags & ObjectDictionary::kMLATSyncFlagPulseWanted) return true;
        }
        return false;
    }

    Emitter emitter_;

    // Shared with the core that sends pulses.
    volatile bool allowed_ = false;
    volatile bool requested_ = false;
    volatile bool busy_ = false;
    volatile uint32_t num_pulses_sent_ = 0;
    volatile uint64_t last_mlat_48mhz_counts_ = 0;  // Of the last pulse sent; valid once its count is seen.

    // Core 0 only.
    uint32_t request_timestamp_ms_ = 0;
    ObjectDictionary::SyncPulse last_pulse_;
    bool last_pulse_current_ = false;  // No pulse requested since last_pulse_: its count may still be delivered.
    uint32_t delivered_pulse_num_[kNumSubscribers] = {};
    uint8_t flags_[kNumSubscribers] = {};
};
