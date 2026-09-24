#ifndef DATA_STRUCTURES_HH_
#define DATA_STRUCTURES_HH_

#include <stdint.h>

#include <type_traits>  // For std::is_trivially_copyable.

#include "pfb_mutex.hh"

template <class T>
class PFBQueue {
   public:
    struct PFBQueueConfig {
        uint16_t buf_len_num_elements = 0;
        T* buffer = nullptr;
        bool overwrite_when_full = false;
        // WARNING: Do not use is_thread_safe if the queue is being accessed from an ISR; will cause deadlocks.
        bool is_thread_safe = false;  // Set to true for queues accessed from multiple cores.
    };

    /**
     * Constructor.
     * NOTE: Copy and move constructors are not implemented! Pass by reference only to avoid creating a "double free"
     * error, which is caused by two PFBQueues sharing the same buffer, and both trying to free it when they are
     * destroyed.
     * @param[in] config_in Defines length of the buffer, and points to the buffer of size buf_len_num_elements if
     * PFBQueue should work with a pre-allocated buffer (which must hold buf_len_num_elements constructed T objects).
     * If config_in.buffer is left as nullptr, an array of buf_len_num_elements default-constructed T objects will be
     * dynamically allocated.
     * Elements are stored with T's copy assignment (not memcpy), so non-trivially-copyable element types such as
     * structs holding a std::function are supported.
     * @retval PFBQueue object.
     */
    PFBQueue(PFBQueueConfig config_in) : config_(config_in), buffer_length_(config_in.buf_len_num_elements) {
        if (config_.buffer == nullptr) {
            config_.buffer = new T[buffer_length_];
            buffer_was_dynamically_allocated_ = true;
        }
        if (config_.is_thread_safe) {
            PFB_MUTEX_INIT(mutex_);
        }
    }

    /**
     * Destructor. Frees the buffer buffer if it was dynamically allocated.
     */
    ~PFBQueue() {
        if (buffer_was_dynamically_allocated_ && config_.buffer != nullptr) {
            delete[] config_.buffer;
            config_.buffer = nullptr;  // Prevent double free in case of shallow copy.
        }
    }

    inline bool IsFull() {
        if (config_.is_thread_safe) PFB_MUTEX_LOCK(mutex_);
        bool result = is_full_;
        if (config_.is_thread_safe) PFB_MUTEX_UNLOCK(mutex_);
        return result;
    }
    inline bool IsEmpty() {
        if (config_.is_thread_safe) PFB_MUTEX_LOCK(mutex_);
        bool result = (!is_full_ && (head_ == tail_));
        if (config_.is_thread_safe) PFB_MUTEX_UNLOCK(mutex_);
        return result;
    }

    /**
     * Enqueues an element to the back of the buffer.
     * @param[in] element Object to push onto the back of the buffer.
     * @retval True if succeeded, false if the buffer is full.
     */
    bool Enqueue(const T& element) {
        if (config_.is_thread_safe) PFB_MUTEX_LOCK(mutex_);
        if (is_full_) {
            if (!config_.overwrite_when_full) {
                // Buffer is full and not overwriting; cannot push.
                if (config_.is_thread_safe) PFB_MUTEX_UNLOCK(mutex_);
                return false;
            } else {
                head_ = IncrementIndex(head_);  // Move head forward to overwrite the oldest element.
            }
        }

        config_.buffer[tail_] = element;
        tail_ = IncrementIndex(tail_);

        if (tail_ == head_) {
            is_full_ = true;
        }

        if (config_.is_thread_safe) PFB_MUTEX_UNLOCK(mutex_);
        return true;
    }

    /**
     * Dequeues an element from the front of the buffer.
     * @param[out] element Reference to an object that will be overwritten by the contents of the popped element.
     * @retval True if successful, false if the buffer is empty.
     */
    bool Dequeue(T& element) {
        if (config_.is_thread_safe) PFB_MUTEX_LOCK(mutex_);
        if (head_ == tail_ && !is_full_) {
            if (config_.is_thread_safe) PFB_MUTEX_UNLOCK(mutex_);
            return false;
        }
        element = config_.buffer[head_];
        if constexpr (!std::is_trivially_copyable<T>::value) {
            // Release whatever the vacated slot owned (e.g. a std::function target) instead of holding it until the
            // slot is overwritten.
            config_.buffer[head_] = T();
        }
        head_ = IncrementIndex(head_);
        is_full_ = false;
        if (config_.is_thread_safe) PFB_MUTEX_UNLOCK(mutex_);
        return true;
    }

    /**
     * Returns the contents of an element in the buffer without removing it from the buffer.
     * @param[out] element Reference to an object that will be overwritten by the contents of the peeked element.
     * @param[in] index Position in the buffer to peek. Defaults to 0 (the front of the buffer).
     * @retval True if successful, false if the buffer is empty or index is out of bounds.
     */
    bool Peek(T& element, uint16_t index = 0) {
        if (config_.is_thread_safe) PFB_MUTEX_LOCK(mutex_);
        if (index >= LengthUnlocked()) {
            if (config_.is_thread_safe) PFB_MUTEX_UNLOCK(mutex_);
            return false;
        }
        element = config_.buffer[IncrementIndex(head_, index)];
        if (config_.is_thread_safe) PFB_MUTEX_UNLOCK(mutex_);
        return true;
    }

    /**
     * Discards a specified number of elements from the front of the buffer.
     * @param[in] num_elements Number of elements to discard from the front of the buffer.
     * @retval True if successful, false if there are not enough elements to discard.
     */
    inline bool Discard(uint16_t num_elements) {
        if (config_.is_thread_safe) PFB_MUTEX_LOCK(mutex_);
        if (num_elements > LengthUnlocked()) {
            if (config_.is_thread_safe) PFB_MUTEX_UNLOCK(mutex_);
            return false;  // Not enough elements to discard.
        }
        head_ = IncrementIndex(head_, num_elements);
        if (num_elements > 0) {
            is_full_ = false;
        }
        if (config_.is_thread_safe) PFB_MUTEX_UNLOCK(mutex_);
        return true;
    }

    /**
     * Returns the number of elements currently in the buffer.
     * @retval Number of elements in the buffer.
     */
    inline uint16_t Length() {
        if (config_.is_thread_safe) PFB_MUTEX_LOCK(mutex_);
        uint16_t len = LengthUnlocked();
        if (config_.is_thread_safe) PFB_MUTEX_UNLOCK(mutex_);
        return len;
    }

    /**
     * Return the maximum number of elements that can be stored in the queue.
     * @retval Number of elements that can be stored in the queue.
     */
    inline uint16_t MaxNumElements() { return config_.buf_len_num_elements; }

    /**
     * Empty out the buffer by setting the head equal to the tail.
     */
    void Clear() {
        if (config_.is_thread_safe) PFB_MUTEX_LOCK(mutex_);
        head_ = tail_ = 0;
        is_full_ = false;
        if (config_.is_thread_safe) PFB_MUTEX_UNLOCK(mutex_);
    }

   private:
    /**
     * Returns the number of elements currently in the buffer without acquiring the lock.
     * Must only be called when the mutex is already held.
     * @retval Number of elements in the buffer.
     */
    inline uint16_t LengthUnlocked() {
        if (is_full_) {
            return buffer_length_;
        } else if (head_ > tail_) {
            return buffer_length_ - (head_ - tail_);  // Wrapped.
        } else {
            return tail_ - head_;  // Not wrapped.
        }
    }

    /**
     * Increments and wraps a buffer index.
     * @param[in] index Value to increment and wrap.
     * @param[in] increment Value to increment the index by. Defaults to 1.
     * @retval Incremented and wrapped value.
     */
    uint16_t IncrementIndex(uint16_t index, uint16_t increment = 1) {
        index += increment;
        return index >= buffer_length_ ? index - buffer_length_ : index;
    }

    PFBQueueConfig config_;
    bool buffer_was_dynamically_allocated_ = false;
    bool is_full_ = false;  // New flag to distinguish full from empty when head == tail
    uint16_t buffer_length_;
    uint16_t head_ = 0;
    uint16_t tail_ = 0;
    mutable PFB_MUTEX_TYPE mutex_;
};

#endif