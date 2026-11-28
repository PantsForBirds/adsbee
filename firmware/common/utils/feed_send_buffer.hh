#pragma once

#include <cstdint>
#include <cstdlib>
#include <cstring>

/**
 * Sends whole batches of frames to one non-blocking stream socket without ever waiting for it.
 *
 * A batch goes straight to the socket. If the socket takes only part of it, the unsent tail is kept in a backlog so
 * that the stream never has a cut frame in it; later batches join the backlog while it has room and are dropped when
 * it is full. The caller flushes the backlog on every pass and closes a socket that makes no progress for too long.
 * One slow feed therefore costs its own frames only. Before a batch goes to a socket with no backlog, room for its
 * whole length is reserved (alloc_tail): if the socket takes only part and no backlog can be allocated, the tail goes
 * there. Without that room the batch is dropped whole, so low memory never cuts a frame or closes the connection.
 */
class FeedSendBuffer {
   public:
    enum Result : uint8_t {
        kSent = 0,  // The socket took the whole batch.
        kQueued,    // All or part of the batch waits in the backlog.
        kDropped,   // No room: the batch was discarded, the stream is intact.
        kError      // Socket error: close the socket.
    };

    // Returns the number of bytes the socket accepted (0 if it would block), or a negative value on a socket error.
    typedef int (*WriteFn)(void* ctx, const uint8_t* buf, uint16_t buf_len);

    struct Config {
        WriteFn write = nullptr;
        void* ctx = nullptr;
        uint16_t backlog_capacity_bytes = 0;
        // The backlog is allocated when a feed falls behind and freed when it has caught up: a feed that keeps up
        // never holds one.
        void* (*alloc)(size_t num_bytes) = malloc;
        // Reserves a batch's length before it is sent, to hold its tail if the socket takes only part and alloc() has
        // no backlog. It may use memory that alloc() keeps for others. nullptr: alloc() is used.
        void* (*alloc_tail)(size_t num_bytes) = nullptr;
        void (*dealloc)(void* ptr) = free;
    };

    FeedSendBuffer() {}
    ~FeedSendBuffer() { Close(); }
    FeedSendBuffer(const FeedSendBuffer&) = delete;
    FeedSendBuffer& operator=(const FeedSendBuffer&) = delete;

    /**
     * Starts a new connection: empties and frees the backlog, restarts the stall timer.
     */
    void Open(const Config& config, uint32_t timestamp_ms) {
        Close();
        config_ = config;
        last_progress_timestamp_ms_ = timestamp_ms;
    }

    void Close() {
        if (backlog_ != nullptr) {
            config_.dealloc(backlog_);
            backlog_ = nullptr;
        }
        backlog_len_bytes_ = 0;
        backlog_alloc_bytes_ = 0;
        waiting_ = false;
    }

    /**
     * Sends one batch of whole frames, or queues or drops it. Never blocks.
     */
    Result Write(const uint8_t* buf, uint16_t buf_len, uint32_t timestamp_ms) {
        if (config_.write == nullptr) {
            return kError;
        }
        if (backlog_len_bytes_ > 0) {
            // Older bytes go first.
            if (!Flush(timestamp_ms)) {
                return kError;
            }
            if (backlog_len_bytes_ > 0) {
                return Append(buf, buf_len) ? kQueued : kDropped;
            }
        }
        // Room for any tail of this batch, before the socket sees it.
        void* (*reserve_alloc)(size_t) = config_.alloc_tail != nullptr ? config_.alloc_tail : config_.alloc;
        uint8_t* reserve = static_cast<uint8_t*>(reserve_alloc(buf_len));
        if (reserve == nullptr) {
            return kDropped;
        }
        int num_sent = config_.write(config_.ctx, buf, buf_len);
        if (num_sent < 0) {
            config_.dealloc(reserve);
            return kError;
        }
        if (num_sent > 0 || !waiting_) {
            last_progress_timestamp_ms_ = timestamp_ms;  // Progress, or the start of a wait.
        }
        waiting_ = num_sent < buf_len;
        if (!waiting_ || Append(buf + num_sent, buf_len - num_sent)) {
            config_.dealloc(reserve);
            return waiting_ ? kQueued : kSent;
        }
        if (num_sent == 0) {
            config_.dealloc(reserve);
            return kDropped;  // Nothing went out: the batch is dropped whole.
        }
        // No backlog for a cut batch: the reserve keeps its tail, and later batches are dropped whole until it is sent.
        backlog_ = reserve;
        backlog_len_bytes_ = buf_len - num_sent;
        backlog_alloc_bytes_ = backlog_len_bytes_;  // Full: nothing joins the tail.
        memcpy(backlog_, buf + num_sent, backlog_len_bytes_);
        return kQueued;
    }

    /**
     * Sends as much of the backlog as the socket takes.
     * @retval False on a socket error.
     */
    bool Flush(uint32_t timestamp_ms) {
        if (backlog_len_bytes_ == 0) {
            return true;
        }
        int num_sent = config_.write(config_.ctx, backlog_, backlog_len_bytes_);
        if (num_sent < 0) {
            return false;
        }
        if (num_sent > 0) {
            last_progress_timestamp_ms_ = timestamp_ms;
            backlog_len_bytes_ -= num_sent;
            memmove(backlog_, backlog_ + num_sent, backlog_len_bytes_);
            waiting_ = backlog_len_bytes_ > 0;
            if (!waiting_) {
                config_.dealloc(backlog_);
                backlog_ = nullptr;
                backlog_alloc_bytes_ = 0;
            }
        }
        return true;
    }

    uint16_t BacklogLenBytes() const { return backlog_len_bytes_; }

    /**
     * Whether Write() can accept a batch of this size now without dropping it for lack of backlog room.
     */
    bool HasRoomFor(uint16_t buf_len) const {
        return backlog_len_bytes_ == 0 || static_cast<uint32_t>(backlog_len_bytes_) + buf_len <= BacklogCapacityBytes();
    }

    /**
     * How long the socket has refused bytes that were offered to it. 0 while it keeps up.
     */
    uint32_t StalledMs(uint32_t timestamp_ms) const {
        return waiting_ ? timestamp_ms - last_progress_timestamp_ms_ : 0;
    }

   private:
    // Room of the backlog in use, or of the one alloc() would give.
    uint16_t BacklogCapacityBytes() const {
        return backlog_ != nullptr ? backlog_alloc_bytes_ : config_.backlog_capacity_bytes;
    }

    bool Append(const uint8_t* buf, uint16_t buf_len) {
        if (static_cast<uint32_t>(backlog_len_bytes_) + buf_len > BacklogCapacityBytes()) {
            return false;
        }
        if (backlog_ == nullptr) {
            backlog_ = static_cast<uint8_t*>(config_.alloc(config_.backlog_capacity_bytes));
            backlog_alloc_bytes_ = config_.backlog_capacity_bytes;
            if (backlog_ == nullptr) {
                backlog_alloc_bytes_ = 0;
                return false;
            }
        }
        memcpy(backlog_ + backlog_len_bytes_, buf, buf_len);
        backlog_len_bytes_ += buf_len;
        return true;
    }

    Config config_;
    uint8_t* backlog_ = nullptr;
    uint16_t backlog_len_bytes_ = 0;
    uint16_t backlog_alloc_bytes_ = 0;  // Room in backlog_: the configured capacity, or a cut tail's length.
    bool waiting_ = false;  // The socket refused bytes and has not caught up since.
    uint32_t last_progress_timestamp_ms_ = 0;
};
