#include "rate_watch.hh"

#include "autobaud.hh"
#include "console_baud.hh"

void RateWatch::Start(uint32_t baud, uint64_t now_us) {
    baud_ = baud;
    last_count_ = edges_.Count();
    last_edge_us_ = now_us;
    next_ask_us_ = now_us;
    EndLock(baud != 0 ? Phase::kLocked : Phase::kUnknown);
}

void RateWatch::OnAppReset(uint64_t now_us) {
    phase_ = Phase::kAwaitingBoot;
    watch_phase_ = Phase::kUnknown;
    last_count_ = edges_.Count();
    mark_ = last_count_;
    analyzed_count_ = mark_;
    lock_start_us_ = now_us;
    deadline_us_ = now_us + kBootAnswerMs * 1000ull;
    asks_ = 0;
    pending_errors_ = 0;
}

void RateWatch::OnBootloaderReset(uint32_t bootloader_baud, uint64_t now_us) {
    (void)now_us;
    baud_ = bootloader_baud;
    last_count_ = edges_.Count();
    EndLock(Phase::kBootloader);
}

void RateWatch::Ask(uint64_t now_us) {
    phase_ = Phase::kUnknown;
    watch_phase_ = Phase::kUnknown;
    next_ask_us_ = now_us;
}

void RateWatch::OnRxErrors(uint32_t count) { pending_errors_ += count; }

void RateWatch::EndLock(Phase phase) {
    phase_ = phase;
    watch_phase_ = phase;
    checked_ = last_count_;
    check_from_ = last_count_;
    pending_errors_ = 0;
}

void RateWatch::StartResolve(uint64_t now_us) {
    stats_.hints++;
    last_resolve_us_ = now_us;
    phase_ = Phase::kResolving;
    mark_ = last_count_ > kHintLookback ? last_count_ - kHintLookback : 0;
    analyzed_count_ = 0;  // Analyze at once: the "UU" may be complete already.
    lock_start_us_ = now_us;
    deadline_us_ = now_us + kResolveMs * 1000ull;
    asks_ = 0;
}

RateWatch::Action RateWatch::StartAsk(uint64_t now_us) {
    if (!Locking()) {
        lock_start_us_ = now_us;
        asks_ = 0;
    }
    phase_ = Phase::kAsking;
    asks_++;
    stats_.asks++;
    // Send the NUL now; the break follows after kPreBreakIdleUs.
    break_pending_ = true;
    break_active_ = false;
    break_start_us_ = now_us + (baud_ != 0 ? 10000000ull / baud_ + 1 : 0) + kPreBreakIdleUs;
    deadline_us_ = break_start_us_ + kAskAnswerMs * 1000ull;
    mark_ = last_count_;
    analyzed_count_ = mark_;
    answer_us_ = 0;
    Action action;
    action.send_nul = true;
    return action;
}

bool RateWatch::Analyze(uint64_t now_us, Action* action) {
    uint64_t count = last_count_;
    if (count == analyzed_count_) return false;
    if (now_us - last_edge_us_ < kQuietUs && count - analyzed_count_ < kAnalyzeEvery) return false;
    analyzed_count_ = count;

    uint64_t first = 0;
    size_t n = edges_.Read(mark_, count, cycles_, kMaxAnalyzedEdges, &first);
    bool first_falling = ((first & 1) == 0) == edges_.FirstFalling();
    Autobaud::Measurement m = Autobaud::MeasureNewest(cycles_, n, first_falling, clock_hz_);
    if (m.baud == 0) return false;
    // The "UU" may not be over: wait for two idle bits so the retune lands between frames.
    if (m.last_edge + 1 >= n && now_us - last_edge_us_ < 2000000ull / m.baud + 1) {
        analyzed_count_ = count - 1;  // Look again at the next Poll().
        return false;
    }

    bool retune = baud_ == 0 || !Autobaud::MatchesRate(m.baud, m.span_cycles, baud_);
    *action = Action();
    uint32_t cycles_per_us = clock_hz_ / 1000000;
    uint32_t uu_us = (cycles_[n - 1] - cycles_[m.first_edge]) / cycles_per_us;
    uint64_t uu_start_us = last_edge_us_ > uu_us ? last_edge_us_ - uu_us : 0;
    if (retune) {
        // Console bytes that arrived before the "UU" started were sent at the old rate: keep them.
        uint64_t seen_us = EdgeSeenAt(first + m.first_edge, now_us);
        action->keep = ReceivedBy(seen_us > kKeepMarginUs ? seen_us - kKeepMarginUs : 0);
        if ((int32_t)(action->keep - forwardable_) > 0) forwardable_ = action->keep;
        baud_ = Autobaud::SnapToConsoleRate(m.baud);
        stats_.retunes++;
        action->kind = Action::kRetune;
        action->baud = baud_;
    }
    action->send_nul = true;
    stats_.locks++;
    last_lock_ = LockInfo();
    last_lock_.measured_baud = m.baud;
    last_lock_.baud = baud_;
    last_lock_.elapsed_us = (uint32_t)(now_us - lock_start_us_);
    last_lock_.asks = asks_;
    last_lock_.answer_us = asks_ != 0 && answer_us_ != 0 ? (uint32_t)(answer_us_ - ask_start_us_) : 0;
    last_lock_.uu_to_lock_us = (uint32_t)(now_us - uu_start_us);
    uint32_t quiet = m.first_edge > 0 ? cycles_[m.first_edge] - cycles_[m.first_edge - 1] : UINT32_MAX;
    last_lock_.quiet_before_uu_us = quiet / cycles_per_us < 1000000 ? quiet / cycles_per_us : 1000000;
    last_lock_.retuned = retune;
    EndLock(Phase::kLocked);
    return true;
}

bool RateWatch::EdgesHint(uint64_t to) {
    if (to - check_from_ < 3 || baud_ == 0) return false;
    uint64_t first = 0;
    size_t n = edges_.Read(check_from_, to, cycles_, kMaxCheckedEdges, &first);
    bool first_falling = ((first & 1) == 0) == edges_.FirstFalling();
    return Autobaud::OtherRateHint(cycles_, n, first_falling, baud_, clock_hz_);
}

bool RateWatch::EdgesFitRate(uint32_t baud) {
    uint64_t first = 0;
    size_t n = edges_.Read(mark_, last_count_, cycles_, kMaxAnalyzedEdges, &first);
    bool first_falling = ((first & 1) == 0) == edges_.FirstFalling();
    return Autobaud::FitsRate(cycles_, n, first_falling, baud, clock_hz_);
}

RateWatch::Action RateWatch::Poll(uint64_t now_us, bool sync_high, uint32_t rx_received) {
    uint64_t count = edges_.Count();
    SampleArrivals(rx_received, count, now_us);
    sync_high_ = sync_high;
    if (count != last_count_) {
        last_count_ = count;
        last_edge_us_ = now_us;
    }
    Action action;
    if (break_pending_ && now_us >= break_start_us_) {
        break_pending_ = false;
        break_active_ = true;
        break_end_us_ = now_us + kBreakUs;
        ask_start_us_ = now_us;
        mark_ = count;  // The answer comes after the break starts.
        analyzed_count_ = mark_;
        action.kind = Action::kBreakStart;
        return action;
    }
    if (break_active_ && now_us >= break_end_us_) {
        break_active_ = false;
        action.kind = Action::kBreakEnd;
        return action;
    }

    switch (phase_) {
        case Phase::kLocked:
        case Phase::kBootloader:
        case Phase::kUnknown: {
            // SYNC high: the module may be asleep with a floating line, so ignore UART errors but still check edges
            // (it may have booted the application anyway).
            bool hint = !sync_high && pending_errors_ > 0;
            if (!hint && count != checked_ &&
                (now_us - last_check_us_ >= kCheckIntervalUs || now_us - last_edge_us_ >= kQuietUs)) {
                hint = EdgesHint(count);
                checked_ = count;
                last_check_us_ = now_us;
            }
            pending_errors_ = 0;
            if (hint && phase_ == Phase::kUnknown && now_us - last_resolve_us_ < kUnknownHintIntervalMs * 1000ull) {
                hint = false;
            }
            if (hint) {
                StartResolve(now_us);
                if (Analyze(now_us, &action)) return action;
            } else if (phase_ == Phase::kUnknown && !sync_high && now_us >= next_ask_us_) {
                return StartAsk(now_us);
            }
            return action;
        }
        case Phase::kResolving:
            pending_errors_ = 0;
            if (Analyze(now_us, &action)) return action;
            if (now_us >= deadline_us_) {
                // No "UU". Ignore the hint if the edges fit the current rate. Don't ask the ROM bootloader, a module
                // that may be asleep, or one whose rate is already unknown (asked on a schedule).
                if (watch_phase_ == Phase::kBootloader || watch_phase_ == Phase::kUnknown || sync_high ||
                    EdgesFitRate(baud_)) {
                    EndLock(watch_phase_);
                    return action;
                }
                if (baud_ != kFactoryBaud && EdgesFitRate(kFactoryBaud)) {
                    // Boot output: the module is rebooting.
                    phase_ = Phase::kAwaitingBoot;
                    deadline_us_ = now_us + kBootAnswerMs * 1000ull;
                    return action;
                }
                return StartAsk(now_us);
            }
            return action;
        case Phase::kAsking:
        case Phase::kAwaitingBoot:
            pending_errors_ = 0;  // The break and boot output at the wrong rate leave errors behind.
            if (phase_ == Phase::kAsking && answer_us_ == 0 && !break_pending_ && count != mark_) {
                answer_us_ = last_edge_us_;
            }
            if (Analyze(now_us, &action)) return action;
            if (now_us >= deadline_us_) {
                if (sync_high || break_pending_ || break_active_) {
                    // An asleep module can't answer: ask once SYNC is low. (A break still running ends first.)
                    deadline_us_ = now_us + kBootAnswerMs * 1000ull;
                    return action;
                }
                if (phase_ == Phase::kAwaitingBoot || asks_ < kMaxAsks) return StartAsk(now_us);
                stats_.failed_locks++;
                next_ask_us_ = now_us + kUnknownAskIntervalMs * 1000ull;
                EndLock(Phase::kUnknown);
            }
            return action;
    }
    return action;
}

void RateWatch::SampleArrivals(uint32_t received, uint64_t edges, uint64_t now_us) {
    received_ = received;
    if (!forward_started_) {
        forward_started_ = true;
        forwardable_ = received;
        for (ForwardSample& sample : forward_samples_) sample = {received, edges, now_us};
        return;
    }
    if (now_us - forward_samples_[forward_newest_].us >= kForwardSampleUs) {
        forward_newest_ = (forward_newest_ + 1) % kForwardSamples;
        forward_samples_[forward_newest_] = {received, edges, now_us};
    }
    if (now_us < kConsoleDelayUs) return;
    uint32_t old_enough = ReceivedBy(now_us - kConsoleDelayUs);
    if ((int32_t)(old_enough - forwardable_) > 0) forwardable_ = old_enough;
}

uint64_t RateWatch::EdgeSeenAt(uint64_t index, uint64_t now_us) const {
    uint64_t seen = now_us;
    for (size_t i = 0; i < kForwardSamples; i++) {
        const ForwardSample& sample = forward_samples_[(forward_newest_ + kForwardSamples - i) % kForwardSamples];
        if (sample.edges <= index) break;  // Older samples hadn't seen it either.
        seen = sample.us;
    }
    return seen;
}

uint32_t RateWatch::ReceivedBy(uint64_t us) const {
    for (size_t i = 0; i < kForwardSamples; i++) {
        const ForwardSample& sample = forward_samples_[(forward_newest_ + kForwardSamples - i) % kForwardSamples];
        if (sample.us <= us) return sample.received;
    }
    return forwardable_;
}
