#include "rate_tracker.hh"

#include <string.h>

#include "console_baud.hh"

void RateTracker::Idle() {
    state_ = State::kIdle;
    target_baud_ = 0;
    ok_matched_ = 0;
    error_matched_ = 0;
    error_count_ = 0;
    awaiting_reply_ = false;
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
    state_ = State::kSettling;
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
            // The module parses its line buffer as a C string, so it never sees what follows a NUL (garbage from a
            // held-low line or a wrong rate); the NUL ends this copy the same way.
            line_[line_len_++] = c;
        } else {
            line_overflow_ = true;
        }
    }
    return len;
}

// The module's AT parser (cppAT, firmware/modules/cppAT/src/cpp_at.cc) as far as it decides which commands run with
// which arguments, so the tracker reads a line the way the module does.
namespace {

struct Span {
    const char* p;
    size_t n;
    bool Is(const char* text) const { return strlen(text) == n && strncmp(p, text, n) == 0; }
};

bool IsAlnum(char c) { return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'); }
bool IsSpace(char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\v' || c == '\f' || c == '\r'; }

// CppAT::ArgToNum() for an unsigned value: strtoul() with surrounding whitespace. Returns false for anything else.
bool ArgToUint(Span arg, uint32_t& out) {
    size_t i = 0;
    while (i < arg.n && IsSpace(arg.p[i])) i++;
    if (i < arg.n && arg.p[i] == '+') i++;
    size_t digits = i;
    uint64_t value = 0;
    while (i < arg.n && arg.p[i] >= '0' && arg.p[i] <= '9') {
        value = value * 10 + (uint64_t)(arg.p[i] - '0');
        if (value > UINT32_MAX) value = UINT32_MAX;  // strtoul saturates (32-bit unsigned long on the CC1314).
        i++;
    }
    if (i == digits) return false;
    while (i < arg.n && IsSpace(arg.p[i])) i++;
    if (i != arg.n) return false;
    out = (uint32_t)value;
    return true;
}

}  // namespace

void RateTracker::OnHostLine(uint32_t now_ms) {
    // CppAT::ParseMessage(): every "AT+" command in the line, in order, until one fails.
    const char* message = line_;
    const char* start = strstr(message, "AT+");
    if (start != nullptr && !awaiting_reply_) {
        awaiting_reply_ = true;
        awaiting_reply_ms_ = now_ms;
    }
    while (start != nullptr) {
        start += 3;
        Span command = {start, strcspn(start, "? =\r\n")};
        if (command.n == 0) return;
        start += command.n;
        char op = '\0';
        if (*start != '\0') {
            if (*start != '\r' && *start != '\n') op = *start;
            while (*start != '\0' && !IsAlnum(*start) && *start != ',' && *start != '-') start++;
        }
        // Arguments: up to the line end, split at commas.
        Span args_text = {start, strcspn(start, "\r\n")};
        Span args[3];
        size_t num_args = 0;
        bool too_many = false;
        for (size_t pos = 0; args_text.n > 0;) {
            const char* comma = (const char*)memchr(args_text.p + pos, ',', args_text.n - pos);
            size_t end = comma != nullptr ? (size_t)(comma - args_text.p) : args_text.n;
            if (num_args == 3) {
                too_many = true;
                break;
            }
            args[num_args++] = {args_text.p + pos, end - pos};
            if (comma == nullptr) break;
            pos = end + 1;
            if (pos == args_text.n) {  // Trailing comma: a blank last argument.
                if (num_args == 3) {
                    too_many = true;
                } else {
                    args[num_args++] = {args_text.p + pos, 0};
                }
                break;
            }
        }

        if (command.Is("BAUD_RATE") && op == '=') {
            uint32_t baud;
            // The console answers ERROR to anything else and stays where it is, which ends the line too.
            if (too_many || num_args != 2 || !args[0].Is("CONSOLE") || !ArgToUint(args[1], baud) ||
                !ConsoleBaud::IsSupported(baud)) {
                return;
            }
            StartSwitch(baud, now_ms);
            return;
        }
        if (command.Is("SETTINGS") && op == '=' && !too_many && num_args > 0 && args[0].Is("RESET")) {
            StartSwitch(kFactoryConsoleBaud, now_ms);  // Factory defaults, applied live after the OK.
            return;
        }
        if (command.Is("REBOOT") && num_args == 0 && !too_many) {
            state_ = State::kRebootPending;
            since_ms_ = now_ms;
            return;
        }
        // cppAT looks for the next command from the start of these arguments. If they contain one (a query's empty
        // arguments run on past a CR to the next command), whether it runs depends on how many arguments this
        // command takes, which only the module knows. Leave that line to the safety net.
        const char* next = strstr(start, "AT+");
        if (next != nullptr && next < args_text.p + args_text.n) return;
        start = next;
    }
}

void RateTracker::StartSwitch(uint32_t target, uint32_t now_ms) {
    state_ = State::kAwaitReply;
    target_baud_ = target;
    since_ms_ = now_ms;
    ok_matched_ = 0;
    error_matched_ = 0;
}

size_t RateTracker::OnConsoleBytes(const uint8_t* data, size_t len, uint32_t now_ms) {
    if (len > 0) awaiting_reply_ = false;
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
            if (sync_high) {
                awaiting_reply_ = false;  // A sleeping module doesn't answer.
            } else if (awaiting_reply_ && now_ms - awaiting_reply_ms_ >= kNoReplyMs &&
                       now_ms - last_lock_ms_ >= kRelockIntervalMs) {
                state_ = State::kLost;
                since_ms_ = now_ms;
                awaiting_reply_ = false;
            }
            break;
        case State::kRomBootloader:
            break;
        case State::kRetunePending:
            step = {Step::kRetune, target_baud_};
            break;
        case State::kSettling:
            if (elapsed >= kSwitchSettleMs) {
                state_ = State::kIdle;
                since_ms_ = now_ms;
            }
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
