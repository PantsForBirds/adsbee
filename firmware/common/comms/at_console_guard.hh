#pragma once

#include <cstdint>
#include <cstring>
#include <string_view>

/**
 * Keeps binary console data (e.g. an AT+OTA=WRITE payload) away from the AT parser. The parser runs "AT+CMD=args"
 * found anywhere in a line, and firmware images contain text like "AT+BOOT_USB_UF2=1DEADBEE".
 *   1. While a payload is being read, the line assembler must not consume input (PayloadInProgress()), so a nested
 *      UpdateAT() leaves the bytes to ATReadConsole().
 *   2. If a payload read ends short, the missing bytes are dropped as they arrive, until the count is met or the
 *      console is idle for kDiscardIdleTimeoutMs.
 *   3. Lines with NUL or control characters other than \t, \r, \n are dropped without being parsed or echoed.
 */
class ATConsoleGuard {
   public:
    static constexpr uint32_t kDiscardIdleTimeoutMs = 1000;

    /** Call after printing READY for a binary payload of len_bytes. */
    void BeginBinaryPayload(uint32_t len_bytes) {
        payload_in_progress_ = true;
        payload_len_bytes_ = len_bytes;
    }

    /** Call when the payload read returns. received_bytes < 0 treats the whole payload as unread. */
    void EndBinaryPayload(int32_t received_bytes, uint32_t timestamp_ms) {
        payload_in_progress_ = false;
        uint32_t received = received_bytes < 0 ? 0 : static_cast<uint32_t>(received_bytes);
        discard_remaining_bytes_ = received >= payload_len_bytes_ ? 0 : payload_len_bytes_ - received;
        last_discard_timestamp_ms_ = timestamp_ms;
        payload_len_bytes_ = 0;
    }

    /** True while a payload is being read. The line assembler must not dequeue console input meanwhile. */
    bool PayloadInProgress() const { return payload_in_progress_; }

    uint32_t DiscardRemainingBytes() const { return discard_remaining_bytes_; }

    /** Call for every console character. Returns true if it belongs to an unfinished payload and must be dropped. */
    bool DiscardChar(uint32_t timestamp_ms) {
        if (discard_remaining_bytes_ == 0) {
            return false;
        }
        if (timestamp_ms - last_discard_timestamp_ms_ > kDiscardIdleTimeoutMs) {
            // The sender stopped sending the payload; what arrives now is new input.
            discard_remaining_bytes_ = 0;
            return false;
        }
        discard_remaining_bytes_--;
        last_discard_timestamp_ms_ = timestamp_ms;
        return true;
    }

    /** True if a completed line (terminator included) is text. Bytes >= 0x80 are allowed for UTF-8 SSIDs/passwords. */
    static bool LineIsText(const char *buf, uint16_t len) {
        for (uint16_t i = 0; i < len; i++) {
            uint8_t c = static_cast<uint8_t>(buf[i]);
            if ((c < 0x20 && c != '\t' && c != '\r' && c != '\n') || c == 0x7F) {
                return false;
            }
        }
        return true;
    }

   private:
    bool payload_in_progress_ = false;
    uint32_t payload_len_bytes_ = 0;
    uint32_t discard_remaining_bytes_ = 0;
    uint32_t last_discard_timestamp_ms_ = 0;
};

/**
 * Builds AT parser lines from console characters, applying ATConsoleGuard. One per input source; they share one guard
 * because ATReadConsole() reads payloads from both.
 */
template <uint16_t kBufMaxLen>
class ATLineAssembler {
   public:
    enum Result : uint8_t {
        kNone = 0,       // Character stored; no complete line yet.
        kLine,           // A text line is ready in line().
        kDiscarded,      // Character was the rest of an unfinished binary payload.
        kBinaryLine,     // A completed line held binary data and was dropped.
        kOverflow,       // Line exceeded kBufMaxLen and was dropped.
    };

    explicit ATLineAssembler(ATConsoleGuard &guard) : guard_(guard) {}

    Result Push(char c, uint32_t timestamp_ms) {
        if (guard_.DiscardChar(timestamp_ms)) {
            return kDiscarded;
        }
        if (len_ >= kBufMaxLen) {
            len_ = 0;
            return kOverflow;
        }
        buf_[len_++] = c;
        buf_[len_] = '\0';
        if (c != '\n') {
            return kNone;
        }
        uint16_t line_len = len_;
        len_ = 0;
        if (!ATConsoleGuard::LineIsText(buf_, line_len)) {
            return kBinaryLine;
        }
        line_len_ = line_len;
        return kLine;
    }

    /** The last complete text line, valid until the next Push(). */
    std::string_view line() const { return std::string_view(buf_, line_len_); }

    void Clear() { len_ = 0; }

   private:
    ATConsoleGuard &guard_;
    char buf_[kBufMaxLen + 1] = {0};
    uint16_t len_ = 0;
    uint16_t line_len_ = 0;
};
