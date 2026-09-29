#include "modem_lines.hh"

void ModemLines::Start() {
    following_ = false;
    reset_pending_ = false;
    holding_ = false;
}

void ModemLines::Track(bool dtr, bool rts) {
    dtr_ = dtr;
    rts_ = rts;
}

void ModemLines::OnLineState(bool dtr, bool rts) {
    if (dtr && !dtr_) {
        reset_sync_high_ = !rts;  // RTS asserted = SYNC low = normal app boot after the reset.
        reset_pending_ = true;
    }
    dtr_ = dtr;
    rts_ = rts;
    following_ = true;
}

void ModemLines::OnResetDone(uint32_t now_ms) {
    reset_pending_ = false;
    holding_ = reset_sync_high_;
    hold_start_ms_ = now_ms;
}

bool ModemLines::SyncHigh(uint32_t now_ms) {
    if (reset_pending_) return reset_sync_high_;
    if (!following_ || rts_) return false;
    if (holding_) {
        if (now_ms - hold_start_ms_ < kBackdoorHoldMs) return true;  // Unsigned: wrap-safe.
        holding_ = false;  // Cleared on expiry so a counter wrap can't revive it.
    }
    return dtr_;
}
