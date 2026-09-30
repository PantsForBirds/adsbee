#pragma once

#include "aircraft_dictionary.hh"
#include "bsp.hh"
#include "lr2021.hh"
#include "settings.hh"

class ADSBee {
   public:
    static constexpr uint32_t kDefaultWatchdogTimeoutSec = 10;

    struct ADSBeeConfig {
        LR2021::LR2021Config lr2021_config = {
            .spi_index = bsp.kCoProSPIIndex,
            .gpio_nss = bsp.kLR2021CSPin,
            .gpio_busy = bsp.kLR2021BusyPin,
            .gpio_enable = bsp.kLR2021ResetPin,
            .gpio_sclk = bsp.kCoProSPICLKPin,
            .gpio_pico = bsp.kCoProSPIMOSIPin,
            .gpio_irq = bsp.kLR2021IrqPin,
        };
        uint32_t aircraft_dictionary_update_interval_ms = 1000;
        uint32_t rx_position_update_interval_ms = 1000;
    };

    ADSBee(ADSBeeConfig config);

    bool Init();
    bool Update();

    // Enables/disables 1090 MHz reception. Disable genuinely powers the LR2021 down (DeInit: async
    // cancelled, SPI closed, chip held in reset, IRQ line disarmed) and discards any staged IRQ-drain
    // slots; enable re-runs the full receiver bring-up. Returns false if the bring-up failed.
    bool SetRx1090Enabled(bool enabled);
    bool Rx1090IsEnabled() const { return rx_1090_enabled_; }
    // Enables/disables the LR2021 interface itself (AT+LR_ENABLE). Disable holds the chip in reset and
    // parks every CC1314-side bus pin hi-Z (NSS pulled up; RESET/SCLK/PICO pulled down) so an external
    // host can drive the shared bus without the MCU entering sync sleep. 1090 MHz reception stops;
    // RX_ENABLE/R1090_* settings are remembered and re-applied on enable.
    bool SetLR2021Enabled(bool enabled);
    bool LR2021IsEnabled() const { return lr2021_enabled_; }
    // Sub-GHz receiver enable is owned by SubGHzRadio (it must open/close the RF core); these forward to it.
    // SetRxSubGHzEnabled() returns false if the RF client failed to open/close.
    bool SetRxSubGHzEnabled(bool enabled);
    bool RxSubGHzIsEnabled() const;

    bool SetWatchdogTimeoutSec(uint32_t timeout_sec);
    inline uint32_t GetWatchdogTimeoutSec() const { return watchdog_timeout_sec_; }
    // Pets the watchdog if it is running. Lets long-running blocking operations (e.g. the CW test) avoid a reset.
    void FeedWatchdog();

    // Mode S receiver sync mode (how the LR2021 triggers reception). Re-applies the receiver config.
    void SetR1090PreambleMode(SettingsManager::R1090PreambleMode mode);
    SettingsManager::R1090PreambleMode GetR1090PreambleMode() const { return r1090_preamble_mode_; }
    // LF-frontend AGC gain (0 = auto, 1..15 manual; 13 = max). Re-applies the receiver config.
    void SetR1090Gain(uint8_t gain_step);
    uint8_t GetR1090Gain() const { return r1090_gain_; }
    // LF RX path boost (0 = off .. 7 = max). Re-applies the receiver config.
    void SetR1090RxBoost(uint8_t rx_boost);
    uint8_t GetR1090RxBoost() const { return r1090_rx_boost_; }

    void Reboot();
    void EnterUARTBootloader();

    // True when an external host has asserted SYNC to request the LR2021 bus + MCU sleep. Latched by the
    // SYNC rising-edge ISR (which also hands the bus off immediately: tri-state + command abort) with a
    // live pin-level fallback for the no-edge cases (SYNC already high at boot, interrupt disarmed).
    // The main loop polls this and calls EnterSyncSleep() for the graceful half.
    bool SyncSleepRequested();

    // Enters MCU STANDBY (deep sleep, SRAM retained) while an external host holds the SYNC line HIGH,
    // and blocks until the host drives SYNC LOW. The SYNC ISR has usually already tri-stated the shared
    // LR2021 bus within microseconds of the edge; this finishes the job (SPI teardown, wake-interrupt
    // swap, STANDBY loop) and re-runs the full receiver re-init on wake. The caller must quiesce the
    // SubGHz RF core (SubGHzRadio::Suspend()) before calling this, otherwise its power constraint
    // prevents the MCU from reaching STANDBY.
    void EnterSyncSleep();

    LR2021 lr2021;
    AircraftDictionary aircraft_dictionary;

    // Health / diagnostics counter, reported and reset via AT+RX_STATS. Incremented when the LR2021
    // RX FIFO reads back completely full (the poll loop fell behind; frames were likely dropped).
    uint32_t lr2021_fifo_full_count = 0;

    // Recovery counters, reported and reset via AT+RX_STATS. In a healthy install fifo_resync tracks
    // lr2021.fifo_overflow_count (every observed overflow gets a flush/realign) and the rest stay 0.
    uint32_t lr2021_fifo_resync_count = 0;   // Thread-level FIFO flush/realign after an observed overflow.
    uint32_t lr2021_rx_rearm_count = 0;      // Health ladder: minimal SetRxAdv re-arm (chip left RX).
    uint32_t lr2021_rx_reconfig_count = 0;   // Health ladder: full ApplyReceiverConfig escalation.
    uint32_t lr2021_config_fail_count = 0;   // ApplyReceiverConfig attempts that failed (retried on backoff).
    // Receiver configs the LR2021 rejected (CMD_PERR), replaced by the factory config (DF17, auto gain, no
    // boost) instead of being retried.
    uint32_t lr2021_config_fallback_count = 0;
    uint32_t lr2021_validity_reconfig_count = 0;  // Validity watchdog: reconfigs after N frames with 0 CRC passes.
    // DF17 mode: captures that started off the nominal bit and were realigned (LR2021OokAdsb::RecoverDF17Frame).
    uint32_t lr2021_df17_realigned_count = 0;
    // DF17 mode: captures passed on without realignment because the raw packet queue was backing up (at least
    // kDF17RealignMaxQueueDepth packets waiting for the decoder). They get the nominal reconstruction, which is
    // what a capture that starts on the nominal bit needs anyway.
    uint32_t lr2021_df17_realign_skipped_count = 0;
    // DF17 mode: FIFO framing slips found (LR2021OokAdsb::FindDF17ByteSlip) and flushed away. Reported via
    // AT+RX_STATS as fifo_slips; each one also counts as a fifo_resync.
    uint32_t lr2021_fifo_slip_count = 0;
    // Slip check: after this many consecutive DF17 captures without a realignment match, look at most once per
    // kSlipCheckIntervalMs whether the last two captures are a real frame split across a slipped read window.
    static constexpr uint16_t kSlipCheckAfterFailures = 4;
    static constexpr uint32_t kSlipCheckIntervalMs = 100;
    static constexpr uint16_t kDF17RealignMaxQueueDepth = 50;

    // CPU cost, in CPU cycles (48 per microsecond; utils/cycle_counter.hh). Reported and reset via AT+RX_STATS.
    uint32_t df17_realign_max_cycles = 0;     // Longest RecoverDF17Frame call.
    uint64_t df17_realign_total_cycles = 0;   // Sum and count of RecoverDF17Frame calls, for the average.
    uint32_t df17_realign_calls = 0;
    uint32_t parse_capture_max_cycles = 0;    // Longest per-capture pass of ParseLR2021RxFifo (realignment included).
    uint32_t max_loop_cycles = 0;             // Longest main loop iteration (max_loop_us, at cycle resolution).
    uint64_t loop_total_cycles = 0;           // Sum and count of main loop iterations, for the average.
    uint32_t loop_count = 0;
    // Share of main loop time spent in the 1090 path (adsbee.Update(): LR2021 drain and parse, plus
    // packet_decoder.Update()); 1000 minus this is what is left for UAT, reporting and the console.
    uint64_t rx1090_total_cycles = 0;

    // Longest single super-loop iteration observed, in microseconds. Reported and reset via AT+RX_STATS. Every
    // millisecond spent in one iteration is a millisecond the LR2021 FIFO isn't drained and AT commands aren't
    // serviced, so this is the primary "is the receiver bogging down" gauge. Updated by main().
    uint32_t max_loop_us = 0;

    // Longest single UpdateLR2021() call, in microseconds (100 us clock granularity). With the async RX
    // drain this should stay at one phase-advance worth of work; a regression toward the old blocking
    // drain cost (hundreds of us) means the LR2021 path is stalling the loop again. Reported and reset
    // via AT+RX_STATS.
    uint32_t max_lr2021_us = 0;
    // Longest wall time of one complete async RX drain (StartRxDrain -> data ready), in microseconds.
    // Must stay well under the ~2.2 ms the 256-byte LR2021 FIFO buys at worst-case 1090 traffic.
    // Reported and reset via AT+RX_STATS.
    uint32_t lr2021_drain_max_us = 0;

    SettingsManager::RxPosition rx_position;
    bool rx_position_available = false;

   private:
#ifdef HARDWARE_UNIT_TESTS
    friend class ADSBeeTestAccessor;  // target_test/test_df17_cpu.cpp: feeds synthetic captures to the parser.
#endif
    void IngestAndForwardPackets();
    void PruneAircraftDictionary();
    // Periodically refreshes rx_position / rx_position_available from the configured source (e.g. the
    // lowest airborne aircraft) and pushes the result into the aircraft dictionary's reference position,
    // which gates Mode S surface position decoding. Ported from the ADSBee 1090.
    void UpdateRxPosition();
    bool UpdateLR2021();
    // Splits a drained LR2021 RX FIFO payload into per-packet Mode S frames and enqueues them for
    // decoding. rx_buf points at the drain payload (valid until FinishRxDrain() / ReleaseSlot()).
    // mlat_timestamp_us stamps every packet in the batch (IRQ-edge time for chain slots, parse time
    // for loop-drain payloads).
    void ParseLR2021RxFifo(const uint8_t* rx_buf, uint16_t rx_len_bytes, uint64_t mlat_timestamp_us);
    // DF17 mode: remembers the capture and, after a run of captures without a realignment match, checks whether
    // the FIFO framing has slipped; if so, schedules the FIFO flush/resync.
    void CheckDF17FramingSlip(const uint8_t* capture, bool realign_failed);
    // (Re)applies the current receiver configuration (sync mode, gain, CRC filter) to the LR2021.
    bool ApplyReceiverConfig();

    ADSBeeConfig config_;

    // Matches the settings default (r1090_rx_enabled = true) and the actual boot behavior: Init() ->
    // ApplyReceiverConfig() arms RX before SettingsManager::Apply() runs, which then disables if the
    // persisted setting says so (same brief boot-RX window the SubGHz receiver has).
    bool rx_1090_enabled_ = true;

    // AT+LR_ENABLE: false = LR2021 held in reset with the CC1314-side bus pins parked hi-Z for an
    // external host. Unlike rx_1090_enabled_, this is seeded from the persisted setting inside Init()
    // (settings are loaded before adsbee.Init() in main()) so a device saved with the bus released
    // never drives it at boot.
    bool lr2021_enabled_ = true;

    SettingsManager::R1090PreambleMode r1090_preamble_mode_ = SettingsManager::kR1090PreambleModeModeS;
    uint8_t r1090_gain_ = 10;      // 0 = auto, 1..15 manual (13 = max). Default: manual step 10.
    uint8_t r1090_rx_boost_ = 0;  // LF RX path boost, 0 (off) .. 7 (max).

    uint32_t last_aircraft_dictionary_update_timestamp_ms_ = 0;
    uint32_t last_rx_position_update_timestamp_ms_ = 0;

    uint32_t watchdog_timeout_sec_ = kDefaultWatchdogTimeoutSec;

    // Timestamp of the current async RX drain's StartRxDrain() call, for lr2021_drain_max_us.
    uint32_t lr2021_drain_start_us_ = 0;

    // RX health ladder (see UpdateLR2021): the LR2021 is armed for continuous RX only when the config
    // is applied, so if it ever leaves RX (or a config attempt failed) nothing else would notice -- the
    // drain would just poll an empty FIFO forever. Every completed drain sweep refreshes last_stat();
    // going kRxHealthTimeoutMs without a confirmed-kRx observation triggers recovery, paced by
    // kRxRecoveryBackoffMs: a minimal in-place SetRxAdv re-arm first, escalating to a full
    // ApplyReceiverConfig after kMaxRearmAttempts (or immediately if the last config attempt failed).
    static constexpr uint32_t kRxHealthTimeoutMs = 1000;
    static constexpr uint32_t kRxRecoveryBackoffMs = 2000;
    static constexpr uint8_t kMaxRearmAttempts = 2;
    // Validity watchdog threshold: this many parsed frames with zero CRC-valid decodes means the
    // FIFO byte stream is mis-framed (statistically impossible with real traffic), and the full
    // receiver bring-up is the recovery. Rate-independent: fires within ~2 s at 500 pkt/s.
    static constexpr uint32_t kMaxFramesWithoutValid = 1000;
    // Full ApplyReceiverConfig body; the public-path wrapper adds failure counting and health-ladder
    // clock resets around it.
    bool ApplyReceiverConfigInner();
    uint32_t lr2021_last_rx_ok_ms_ = 0;     // Last drain sweep that confirmed chip_mode == kRx.
    uint32_t lr2021_last_recovery_ms_ = 0;  // Last recovery attempt (re-arm or reconfig), for backoff.
    uint8_t lr2021_rearm_attempts_ = 0;     // Consecutive minimal re-arms without a confirmed kRx.
    bool receiver_config_ok_ = false;       // Last ApplyReceiverConfig attempt succeeded end-to-end.
    uint32_t last_drain_error_log_ms_ = 0;  // Rate-limits the drain-failure CONSOLE_ERROR (1/s).
    uint32_t lr2021_frames_since_valid_ = 0;
    // Slip check state (ParseLR2021RxFifo, DF17 mode): the previous capture, the run of captures without a
    // realignment match, and when the check last ran.
    uint8_t df17_prev_capture_[LR2021OokAdsb::kModeSFrameLenBytes] = {0};
    bool df17_prev_capture_valid_ = false;
    uint16_t df17_fail_run_ = 0;
    uint32_t df17_last_slip_check_ms_ = 0;  // Parsed frames since the last CRC-valid decode (watchdog input).
};

extern ADSBee adsbee;