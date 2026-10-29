#pragma once

#include <cstdint>
#include <cstring>

/**
 * Byte queue between the GDL90 report builder and Bluetooth notifications. Holds framed GDL90 messages (each starts
 * and ends with the 0x7E flag byte) and hands out notification payloads made of whole messages, splitting a message
 * only when it alone is longer than a payload. Clients that parse one notification at a time see whole messages
 * whenever the negotiated payload size allows.
 *
 * Not thread safe: use from one task, or under a lock.
 */
template <uint16_t kCapacity>
class GDL90BLEStream {
   public:
    static constexpr uint8_t kFlagByte = 0x7E;

    /**
     * Queues framed GDL90 messages.
     * @param[in] droppable True for data that may be refused to keep room for traffic (e.g. FIS-B uplinks): accepted
     * only while the queue stays at most half full.
     * @retval True if queued; false if it didn't fit (nothing is queued then).
     */
    bool Push(const uint8_t* buf, uint16_t len, bool droppable = false) {
        const uint16_t limit = droppable ? kCapacity / 2 : kCapacity;
        if (len == 0) return true;
        if (len > limit || count_ > limit - len) {
            num_dropped_bytes_ += len;
            return false;
        }
        for (uint16_t i = 0; i < len; i++) {
            buf_[(head_ + count_ + i) % kCapacity] = buf[i];
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
        head_ = (head_ + len) % kCapacity;
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
    uint8_t At(uint16_t i) const { return buf_[(head_ + i) % kCapacity]; }

    uint8_t buf_[kCapacity] = {};
    uint16_t head_ = 0;
    uint16_t count_ = 0;
    bool head_in_message_ = false;  // A split message's start flag was consumed already.
    uint32_t num_dropped_bytes_ = 0;
};
