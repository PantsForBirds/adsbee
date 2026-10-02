#include "baud_follower.hh"

#include <string.h>

#include "host_line_coding.hh"

void BaudFollower::Start(uint32_t console_baud, uint32_t boot_baud, uint32_t host_baud, uint32_t now_ms) {
    host_baud_ = host_baud;
    console_baud_ = console_baud;
    boot_baud_ = boot_baud;
    hinted_baud_ = 0;
    rom_bootloader_ = false;
    pending_ = false;
    waiting_for_boot_ = false;
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

void BaudFollower::OnReset(bool sync_high, uint32_t now_ms, uint32_t boot_wait_ms) {
    hinted_baud_ = 0;
    if (sync_high) {
        // ROM bootloader: it auto-bauds to the host, so the host's rate goes straight to the UART.
        rom_bootloader_ = true;
        console_baud_ = 0;
        waiting_for_boot_ = false;
        pending_ = host_baud_ != 0;
        return;
    }
    rom_bootloader_ = false;
    console_baud_ = boot_baud_;
    WaitForConsole(now_ms, boot_wait_ms);
    Pend();
}

void BaudFollower::WaitForConsole(uint32_t now_ms, uint32_t wait_ms) {
    waiting_for_boot_ = true;
    boot_done_ms_ = now_ms + wait_ms;
}

void BaudFollower::OnHostBytes(const uint8_t* data, size_t len, uint32_t now_ms, uint32_t boot_wait_ms) {
    for (size_t i = 0; i < len; i++) {
        char c = (char)data[i];
        if (c == '\r' || c == '\n') {
            if (line_len_ > 0 && !line_overflow_) {
                while (line_len_ > 0 && line_[line_len_ - 1] == ' ') line_len_--;
                line_[line_len_] = '\0';
                OnHostLine(now_ms, boot_wait_ms);
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

void BaudFollower::OnHostLine(uint32_t now_ms, uint32_t boot_wait_ms) {
    static const char kBaudPrefix[] = "AT+BAUD_RATE=CONSOLE,";
    if (strncmp(line_, kBaudPrefix, sizeof(kBaudPrefix) - 1) == 0) {
        const char* digits = line_ + sizeof(kBaudPrefix) - 1;
        uint32_t baud = 0;
        for (; *digits >= '0' && *digits <= '9'; digits++) {
            if (baud > (UINT32_MAX - 9) / 10) return;  // Overlong number: not a rate the console accepts.
            baud = baud * 10 + (uint32_t)(*digits - '0');
        }
        // A rejected rate leaves the console where it was; the hint is only probed first, so a wrong guess costs one
        // probe.
        if (*digits == '\0' && ConsoleBaud::IsSupported(baud)) hinted_baud_ = baud;
    } else if (strcmp(line_, "AT+SETTINGS=SAVE") == 0) {
        // The console saves its live rate. Unknown if an AT+BAUD_RATE is still unconfirmed.
        uint32_t saved = hinted_baud_ != 0 ? 0 : console_baud_;
        if (saved != boot_baud_) {
            boot_baud_ = saved;
            boot_baud_changed_ = saved != 0;
        }
    } else if (strcmp(line_, "AT+SETTINGS=RESET") == 0) {
        // Factory defaults, applied live after the OK and saved.
        console_baud_ = 0;
        hinted_baud_ = kFactoryConsoleBaud;
        if (boot_baud_ != kFactoryConsoleBaud) {
            boot_baud_ = kFactoryConsoleBaud;
            boot_baud_changed_ = true;
        }
        WaitForConsole(now_ms, kSettleMs);  // It switches after the OK, then saves.
        Pend();
    } else if (strcmp(line_, "AT+REBOOT") == 0) {
        console_baud_ = boot_baud_;
        hinted_baud_ = 0;
        WaitForConsole(now_ms, boot_wait_ms);
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
    // Needs the console to answer: not while it may be asleep, or still booting.
    if (sync_high) return step;
    if (waiting_for_boot_) {
        if ((int32_t)(now_ms - boot_done_ms_) < 0) return step;
        waiting_for_boot_ = false;
    }
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
    uint32_t desired = DesiredBaud();
    if (desired == 0) return host_baud_ == 0 && console_baud_ == 0;
    return desired != console_baud_ || hinted_baud_ != 0;
}

size_t BuildConsoleBaudCandidates(const uint32_t* preferred, size_t num_preferred, uint32_t* out, size_t max_out) {
    size_t count = 0;
    auto add = [&](uint32_t baud) {
        if (count >= max_out || !IsRenegotiableBaud(baud)) return;
        for (size_t i = 0; i < count; i++) {
            if (out[i] == baud) return;
        }
        out[count++] = baud;
    };
    for (size_t i = 0; i < num_preferred; i++) add(preferred[i]);
    for (uint32_t baud : kCommonConsoleBauds) add(baud);
    return count;
}
