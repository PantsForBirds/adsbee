#pragma once

#include <cstdint>
#include <cstring>
#include <new>

/**
 * Byte queue between the GDL90 report builder and Bluetooth notifications. Holds framed GDL90 messages (each starts
 * and ends with the 0x7E flag byte) and hands out notification payloads made of whole messages, splitting a message
 * only when it alone is longer than a payload. Clients that parse one notification at a time see whole messages
 * whenever the negotiated payload size allows.
 *
 * Not thread safe: use from one task, or under a lock.
 */
class GDL90BLEStream {
   public:
    static constexpr uint8_t kFlagByte = 0x7E;

    // The buffer is allocated here; check Valid().
    explicit GDL90BLEStream(uint16_t capacity) : capacity_(capacity), buf_(new (std::nothrow) uint8_t[capacity]) {}
    ~GDL90BLEStream() { delete[] buf_; }
    GDL90BLEStream(const GDL90BLEStream&) = delete;
    GDL90BLEStream& operator=(const GDL90BLEStream&) = delete;

    bool Valid() const { return buf_ != nullptr; }

    /**
     * Queues framed GDL90 messages.
     * @param[in] droppable True for data that may be refused to keep room for traffic (e.g. FIS-B uplinks): accepted
     * only while the queue stays at most half full.
     * @retval True if queued; false if it didn't fit (nothing is queued then).
     */
    bool Push(const uint8_t* buf, uint16_t len, bool droppable = false) {
        const uint16_t limit = droppable ? capacity_ / 2 : capacity_;
        if (len == 0) return true;
        if (len > limit || count_ > limit - len) {
            num_dropped_bytes_ += len;
            return false;
        }
        for (uint16_t i = 0; i < len; i++) {
            buf_[(head_ + count_ + i) % capacity_] = buf[i];
        }
        count_ += len;
        return true;
    }

    /**
     * Copies the next notification payload: as many whole messages as fit in max_len, else the first max_len bytes of
     * a message that is longer than max_len. Leaves the bytes queued; call Consume() once they were sent.
     * @retval Number of bytes copied.
     */
    uint16_t Peek(uint8_t* out, uint16_t max_len) const {
        uint16_t n = count_ < max_len ? count_ : max_len;
        // Cut after the last message end flag within n, unless that would leave nothing.
        uint16_t cut = 0;
        bool in_message = head_in_message_;
        for (uint16_t i = 0; i < n; i++) {
            if (At(i) == kFlagByte) {
                if (in_message) {
                    cut = i + 1;
                    in_message = false;
                } else {
                    in_message = true;
                }
            }
        }
        if (cut > 0) n = cut;
        for (uint16_t i = 0; i < n; i++) out[i] = At(i);
        return n;
    }

    void Consume(uint16_t len) {
        if (len > count_) len = count_;
        for (uint16_t i = 0; i < len; i++) {
            if (At(i) == kFlagByte) head_in_message_ = !head_in_message_;
        }
        head_ = (head_ + len) % capacity_;
        count_ -= len;
    }

    /**
     * Drops every queued message that hasn't started going out. The rest of a message that is partly sent stays, so
     * the client never sees a cut message.
     */
    void DropUnsent() {
        uint16_t keep = 0;
        if (head_in_message_) {
            while (keep < count_ && At(keep) != kFlagByte) keep++;
            if (keep < count_) keep++;  // Its end flag.
        }
        num_dropped_bytes_ += count_ - keep;
        count_ = keep;
    }

    void Clear() {
        head_ = 0;
        count_ = 0;
        head_in_message_ = false;
    }

    uint16_t Size() const { return count_; }
    bool Empty() const { return count_ == 0; }
    uint32_t NumDroppedBytes() const { return num_dropped_bytes_; }

   private:
    uint8_t At(uint16_t i) const { return buf_[(head_ + i) % capacity_]; }

    const uint16_t capacity_;
    uint8_t* const buf_;
    uint16_t head_ = 0;
    uint16_t count_ = 0;
    bool head_in_message_ = false;  // A split message's start flag was consumed already.
    uint32_t num_dropped_bytes_ = 0;
};

/**
 * Where a 1 Hz report starts its traffic list on Bluetooth. When the queue fills, the next report starts at the first
 * aircraft that didn't fit and wraps around, so a slow link leaves out different aircraft each second instead of the
 * same ones.
 */
class ReportRotation {
   public:
    // Index of the aircraft to send first; send Start()..n-1, then 0..Start()-1.
    uint16_t Start() const { return start_; }

    // The aircraft at index didn't fit.
    void Refused(uint16_t index) {
        if (refused_) return;
        refused_ = true;
        next_start_ = index;
    }

    // Ends a report of num_aircraft aircraft.
    void End(uint16_t num_aircraft) {
        start_ = refused_ && next_start_ < num_aircraft ? next_start_ : 0;
        refused_ = false;
    }

   private:
    uint16_t start_ = 0;
    uint16_t next_start_ = 0;
    bool refused_ = false;
};
