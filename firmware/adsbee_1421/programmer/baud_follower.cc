#include "baud_follower.hh"

#include <string.h>

#include "host_line_coding.hh"

void BaudFollower::Start(uint32_t console_baud, uint32_t host_baud, uint32_t now_ms) {
    host_baud_ = host_baud;
    console_baud_ = console_baud;
    hinted_baud_ = 0;
    rom_bootloader_ = false;
    reset_wanted_ = false;
    pending_ = false;
    host_changed_ms_ = now_ms - kSettleMs;  // A rate set before the session counts as settled.
    line_len_ = 0;
    line_overflow_ = false;
    // A rate the host set while the Programmer was busy (checking, flashing, negotiating) still has to be applied.
    uint32_t desired = DesiredBaud();
    if (desired != 0 && desired != console_baud_) Pend();
}

void BaudFollower::OnHostBaud(uint32_t baud, uint32_t now_ms) {
    host_baud_ = baud;
    host_changed_ms_ = now_ms;
    Pend();
}

void BaudFollower::OnReset(bool sync_high, uint32_t console_baud) {
    hinted_baud_ = 0;
    reset_wanted_ = false;
    rom_bootloader_ = sync_high;
    if (sync_high) {
        // ROM bootloader: it auto-bauds to the host, so the host's rate goes straight to the UART.
        console_baud_ = 0;
        pending_ = host_baud_ != 0;
        return;
    }
    console_baud_ = console_baud;
    Pend();
}

void BaudFollower::OnHostBytes(const uint8_t* data, size_t len, uint32_t now_ms) {
    for (size_t i = 0; i < len; i++) {
        char c = (char)data[i];
        if (c == '\r' || c == '\n') {
            if (line_len_ > 0 && !line_overflow_) {
                while (line_len_ > 0 && line_[line_len_ - 1] == ' ') line_len_--;
                line_[line_len_] = '\0';
                OnHostLine(now_ms);
            }
            line_len_ = 0;
            line_overflow_ = false;
        } else if (line_len_ < kLineMax) {
            line_[line_len_++] = (c >= 'a' && c <= 'z') ? (char)(c - 'a' + 'A') : c;
        } else {
            line_overflow_ = true;
        }
    }
}

void BaudFollower::OnHostLine(uint32_t now_ms) {
    static const char kBaudPrefix[] = "AT+BAUD_RATE=CONSOLE,";
    if (strncmp(line_, kBaudPrefix, sizeof(kBaudPrefix) - 1) == 0) {
        const char* digits = line_ + sizeof(kBaudPrefix) - 1;
        uint32_t baud = 0;
        for (; *digits >= '0' && *digits <= '9'; digits++) {
            if (baud > (UINT32_MAX - 9) / 10) return;  // Overlong number: not a rate the console accepts.
            baud = baud * 10 + (uint32_t)(*digits - '0');
        }
        // A rejected rate leaves the console where it was; the hint is only tried first, so a wrong guess costs one
        // AT exchange.
        if (*digits == '\0' && ConsoleBaud::IsSupported(baud)) hinted_baud_ = baud;
    } else if (strcmp(line_, "AT+SETTINGS=RESET") == 0) {
        // Factory defaults, applied live after the OK.
        hinted_baud_ = kFactoryConsoleBaud;
        host_changed_ms_ = now_ms;
        Pend();
    } else if (strcmp(line_, "AT+REBOOT") == 0) {
        reset_wanted_ = true;
        host_changed_ms_ = now_ms;
        Pend();
    }
}

uint32_t BaudFollower::DesiredBaud() const { return IsRenegotiableBaud(host_baud_) ? host_baud_ : 0; }

BaudFollower::Step BaudFollower::Poll(uint32_t now_ms, bool sync_high) {
    Step step;
    if (!pending_) return step;
    if (rom_bootloader_) {
        pending_ = false;
        if (host_baud_ != 0) step = {Step::kApplyDirect, host_baud_};
        return step;
    }
    if (now_ms - host_changed_ms_ < kSettleMs) return step;  // Unsigned: wrap-safe.
    if (reset_wanted_) {
        // The module is rebooting into its saved rate; a reset of our own makes it answer the trigger. Not while SYNC
        // is high, which would start the ROM bootloader.
        if (sync_high) return step;
        pending_ = false;
        step = {Step::kReset, 0};
        return step;
    }

    uint32_t desired = DesiredBaud();
    if (desired == 0 && host_baud_ != 0) {
        // A rate the console doesn't accept: straight to the UART.
        pending_ = false;
        step = {Step::kApplyDirect, host_baud_};
        return step;
    }
    // desired == 0 from here on means the host never set a rate: the UART follows the console.
    uint32_t target = desired != 0 ? desired : console_baud_;
    if (target != 0 && target == console_baud_ && hinted_baud_ == 0) {
        pending_ = false;
        step = {Step::kApplyDirect, target};
        return step;
    }
    // Needs the console to answer: not while it may be asleep.
    if (sync_high) return step;
    pending_ = false;
    step = {Step::kRenegotiate, desired};
    return step;
}

void BaudFollower::OnRenegotiated(uint32_t console_baud) {
    console_baud_ = console_baud;
    hinted_baud_ = 0;
}

bool BaudFollower::HoldHostData(bool sync_high) const {
    if (!pending_ || rom_bootloader_ || sync_high) return false;
    if (reset_wanted_) return true;
    uint32_t desired = DesiredBaud();
    if (desired == 0) return host_baud_ == 0 && console_baud_ == 0;
    return desired != console_baud_ || hinted_baud_ != 0;
}
