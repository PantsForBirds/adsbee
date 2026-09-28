#pragma once

#include <cstdint>
#include <cstring>
#include <string_view>

/**
 * Keeps binary data on an AT console (e.g. an AT+OTA=WRITE payload) away from the AT parser.
 *
 * AT+OTA=WRITE prints READY and then reads len raw bytes with ATReadConsole(). Before this guard, any of those bytes
 * that ATReadConsole() didn't consume went through the normal line assembler and CppAT::ParseMessage(), which runs an
 * "AT+CMD=args" found anywhere in a line. That happened when:
 *   - network_console_putc() found the outgoing console queue full and called UpdateAT() from inside the WRITE
 *     callback (e.g. to print an SPI error while waiting for payload bytes). The nested UpdateAT() drained the incoming
 *     queue, payload included, into the parser.
 *   - The WRITE timed out (or returned early) before the whole payload arrived; the rest of it then reached the parser.
 * A firmware image contains text such as the help line "AT+BOOT_USB_UF2=1DEADBEE", so a misparsed chunk could reboot
 * the RP2040 into its USB bootloader, which a network-only unit can't leave without a power cycle.
 *
 * The guard:
 *   1. While a binary payload is being read (BeginBinaryPayload() .. EndBinaryPayload()), the line assembler must not
 *      consume anything (PayloadInProgress()), so a nested UpdateAT() leaves the payload to ATReadConsole().
 *   2. After a payload read that ended short, the missing bytes are discarded when they arrive, until the count is met
 *      or the console has been idle for kDiscardIdleTimeoutMs (in case the sender gave up).
 *   3. A completed line that contains NUL or ASCII control characters other than \t, \r, \n is treated as binary
 *      data and is dropped without being parsed or echoed.
 */
class ATConsoleGuard {
   public:
    static constexpr uint32_t kDiscardIdleTimeoutMs = 1000;

    /** Call after printing READY for a binary payload of len_bytes. */
    void BeginBinaryPayload(uint32_t len_bytes) {
        payload_in_progress_ = true;
        payload_len_bytes_ = len_bytes;
    }

    /**
     * Call when the payload read returns. received_bytes < 0 means nothing (or an unknown amount) was read; the whole
     * payload is then treated as outstanding.
     */
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

    /**
     * Call for every console character before it goes to the line assembler. Returns true if the character is the rest
     * of an unfinished binary payload and must be dropped.
     */
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

    /**
     * Returns true if a completed console line (including its terminator, and any bytes after an embedded NUL) is
     * text that may be parsed as an AT command. Bytes >= 0x80 are allowed (UTF-8 SSIDs, passwords).
     */
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
 * Assembles console characters into lines for the AT parser, with the ATConsoleGuard rules applied. One per input
 * source (USB stdio, network console); the guard is shared because ATReadConsole() reads payload bytes from both.
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
