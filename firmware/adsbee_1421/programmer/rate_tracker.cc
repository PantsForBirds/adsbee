#include "rate_tracker.hh"

#include <string.h>

#include "console_baud.hh"

void RateTracker::Idle() {
    state_ = State::kIdle;
    target_baud_ = 0;
    ok_matched_ = 0;
    error_matched_ = 0;
    error_count_ = 0;
}

void RateTracker::Start(uint32_t console_baud, uint32_t now_ms) {
    Idle();
    console_baud_ = console_baud;
    since_ms_ = now_ms;
    last_lock_ms_ = now_ms;
    line_len_ = 0;
    line_overflow_ = false;
}

void RateTracker::OnReset(bool sync_high, uint32_t console_baud, uint32_t now_ms) {
    Idle();
    line_len_ = 0;
    line_overflow_ = false;
    since_ms_ = now_ms;
    last_lock_ms_ = now_ms;
    if (sync_high) {
        state_ = State::kRomBootloader;
        console_baud_ = 0;
        return;
    }
    console_baud_ = console_baud;
}

void RateTracker::OnLocked(uint32_t console_baud, uint32_t now_ms) {
    Idle();
    since_ms_ = now_ms;
    last_lock_ms_ = now_ms;
    // Not found: the UART stays where the step said, and the error check retries later.
    console_baud_ = console_baud;
}

void RateTracker::OnRetuned(uint32_t now_ms) {
    console_baud_ = target_baud_;
    Idle();
    since_ms_ = now_ms;
    last_lock_ms_ = now_ms;
}

size_t RateTracker::OnHostBytes(const uint8_t* data, size_t len, uint32_t now_ms) {
    if (state_ == State::kRomBootloader) return len;
    if (HoldHostData()) return 0;
    for (size_t i = 0; i < len; i++) {
        char c = (char)data[i];
        if (c == '\n') {
            // The module runs the line when its '\n' arrives (CommsManager::UpdateAT()).
            if (!line_overflow_) {
                line_[line_len_] = '\0';
                OnHostLine(now_ms);
            }
            line_len_ = 0;
            line_overflow_ = false;
            if (HoldHostData()) return i + 1;  // A switch starts: the rest waits for it.
        } else if (line_len_ < kLineMax) {
            line_[line_len_++] = c == '\0' ? '?' : c;  // Garbage NULs must not end the string early.
        } else {
            line_overflow_ = true;
        }
    }
    return len;
}

void RateTracker::OnHostLine(uint32_t now_ms) {
    // Like cppAT, the command starts at the first "AT" in the line; the module is case sensitive, so is this.
    const char* command = strstr(line_, "AT");
    if (command == nullptr) return;
    size_t command_len = strlen(command);
    while (command_len > 0 && command[command_len - 1] == '\r') command_len--;

    uint32_t target = 0;
    static const char kBaudPrefix[] = "AT+BAUD_RATE=CONSOLE,";
    static const char kSettingsReset[] = "AT+SETTINGS=RESET";
    static const char kReboot[] = "AT+REBOOT";
    if (command_len > sizeof(kBaudPrefix) - 1 && strncmp(command, kBaudPrefix, sizeof(kBaudPrefix) - 1) == 0) {
        uint32_t baud = 0;
        for (size_t i = sizeof(kBaudPrefix) - 1; i < command_len; i++) {
            char c = command[i];
            if (c < '0' || c > '9' || baud > (UINT32_MAX - 9) / 10) return;  // Not a rate the console accepts.
            baud = baud * 10 + (uint32_t)(c - '0');
        }
        // The console answers ERROR to anything else and stays where it is.
        if (!ConsoleBaud::IsSupported(baud)) return;
        target = baud;
    } else if (command_len == sizeof(kSettingsReset) - 1 && strncmp(command, kSettingsReset, command_len) == 0) {
        // Factory defaults, applied live after the OK.
        target = kFactoryConsoleBaud;
    } else if (command_len == sizeof(kReboot) - 1 && strncmp(command, kReboot, command_len) == 0) {
        state_ = State::kRebootPending;
        since_ms_ = now_ms;
        return;
    } else {
        return;
    }
    state_ = State::kAwaitReply;
    target_baud_ = target;
    since_ms_ = now_ms;
    ok_matched_ = 0;
    error_matched_ = 0;
}

size_t RateTracker::OnConsoleBytes(const uint8_t* data, size_t len, uint32_t now_ms) {
    if (state_ == State::kRetunePending) return 0;  // Anything after the OK is at the new rate.
    if (state_ != State::kAwaitReply) return len;
    static const char kOk[] = "OK\r\n";
    static const char kError[] = "ERROR";
    for (size_t i = 0; i < len; i++) {
        char c = (char)data[i];
        ok_matched_ = c == kOk[ok_matched_] ? ok_matched_ + 1 : (c == kOk[0] ? 1 : 0);
        error_matched_ = c == kError[error_matched_] ? error_matched_ + 1 : (c == kError[0] ? 1 : 0);
        if (ok_matched_ == sizeof(kOk) - 1) {
            state_ = State::kRetunePending;
            since_ms_ = now_ms;
            return i + 1;
        }
        if (error_matched_ == sizeof(kError) - 1) {
            // Refused: the console stays at its rate. The rest of the error line passes as usual.
            Idle();
            since_ms_ = now_ms;
            return len;
        }
    }
    return len;
}

void RateTracker::OnRxErrors(uint32_t count, uint32_t now_ms) {
    if (count == 0 || state_ != State::kIdle) return;
    if (error_count_ == 0 || now_ms - error_window_ms_ > kErrorWindowMs) {
        error_window_ms_ = now_ms;
        error_count_ = 0;
    }
    error_count_ += count;
    // Rate-limited, so a module that keeps sending garbage (or none attached) doesn't keep the receiver in wake locks.
    if (error_count_ >= kRelockErrors && now_ms - last_lock_ms_ >= kRelockIntervalMs) {
        state_ = State::kLost;
        since_ms_ = now_ms;
        error_count_ = 0;
    }
}

uint32_t RateTracker::ReplyTimeoutMs() const {
    if (console_baud_ == 0) return kReplyMaxMs;
    // Twice the time for the console's 8 kB TX ring at the old rate (10 bits per byte).
    uint64_t ms = kReplyBaseMs + 2ull * 8192u * 10u * 1000u / console_baud_;
    return ms < kReplyMaxMs ? (uint32_t)ms : kReplyMaxMs;
}

RateTracker::Step RateTracker::Poll(uint32_t now_ms, bool sync_high) {
    Step step;
    uint32_t elapsed = now_ms - since_ms_;  // Unsigned: wrap-safe.
    switch (state_) {
        case State::kIdle:
        case State::kRomBootloader:
            break;
        case State::kRetunePending:
            step = {Step::kRetune, target_baud_};
            break;
        case State::kAwaitReply:
            if (elapsed >= ReplyTimeoutMs()) {
                // Lost the answer: the console is at the old rate or the new one. Find it.
                state_ = State::kLost;
                since_ms_ = now_ms;
            }
            break;
        case State::kRebootPending:
            // Not while SYNC is high, which would start the ROM bootloader.
            if (elapsed >= kRebootSettleMs && !sync_high) step = {Step::kResetAndLock, console_baud_};
            break;
        case State::kLost:
            if (!sync_high) step = {Step::kWakeLock, target_baud_ != 0 ? target_baud_ : console_baud_};
            break;
    }
    return step;
}
