#include "lr2021.hh"

#include <cstring>

#include "comms.hh"  // For debug logging.
#include "hal.hh"

// Construction

LR2021::LR2021(LR2021Config config) : config_(config), mode_(ChipMode::kStartup), last_stat_{} {
    SPI_Params_init(&spi_params_);
    // Callback (DMA) mode for every transfer. transferMode is latched at SPI_open, so a single mode must
    // serve both the async RX drain (lr2021_async.cpp) and the ~110 synchronous command call sites; the
    // latter go through the post-and-spin shim in SPITransfer() (lr2021_ll.cpp) and keep their blocking
    // semantics unchanged.
    spi_params_.transferMode = SPI_MODE_CALLBACK;
    spi_params_.transferCallbackFxn = &LR2021::SPICallback;
    spi_params_.mode = SPI_CONTROLLER;
    spi_params_.bitRate = 12'000'000;  // Use max clock rate for CC1314 (12MHz).
    spi_params_.dataSize = 8;
    spi_params_.frameFormat = SPI_POL0_PHA0;
}

// Init

bool LR2021::Init() {
    abort_requested_ = false;
    // Reset the async RX drain: no transfer can be in flight here (DeInit cancels before SPI_close, and
    // a first-ever Init starts from the zero-initialized members).
    drain_state_ = DrainState::kIdle;
    async_done_ = false;
    async_ok_ = false;
    async_in_flight_ = false;
    sync_done_ = false;
    fifo_overflow_pending = false;  // Init reboots the chip below, which clears the FIFO anyway.
    memset(async_tx_buf_, 0, sizeof(async_tx_buf_));  // [2..] must clock zeros during FIFO reads.
    // IRQ chain TX buffers: static contents, pre-packed once so the ISR-side chain never packs bytes.
    // Filled staging slots are deliberately preserved across re-inits -- the thread parses them on its
    // own schedule (e.g. after a sync-sleep wake).
    memset(chain_fifo_tx_buf_, 0, sizeof(chain_fifo_tx_buf_));  // [2..] clocks zeros during the read.
    PackU16(chain_fifo_tx_buf_, kOpcodeReadRxFifo);
    PackU16(chain_clear_tx_buf_, kOpcodeGetAndClearFifoIrqFlags);
    PackU16(chain_irq_tx_buf_, kOpcodeGetAndClearIrq);
    CONSOLE_INFO("LR2021::Init", "Initializing.");
    // Do a proper reboot.
    SetEnable(false);
    DelayUs(100);
    SetEnable(true);

    // A SYNC assertion may have tri-stated the interface (and set the abort flag) while we were getting
    // here. Bail before SPI_open re-muxes SCLK/PICO onto the bus the host now owns. A SYNC ISR landing
    // after this check still self-heals: WaitUntilReady below aborts, we SPI_close (pins return to their
    // hi-Z SysConfig defaults), and the main loop re-enters sync sleep which re-tristates.
    if (abort_requested_) {
        return false;
    }

    spi_handle_ = SPI_open(config_.spi_index, &spi_params_);
    if (spi_handle_ == nullptr) {
        CONSOLE_ERROR("LR2021::Init", "SPI_open failed.");
        return false;
    }

    mode_ = ChipMode::kStartup;
    SetEnable(true);
    if (!WaitUntilReady(kBootupTimeoutMs)) {
        CONSOLE_ERROR("LR2021::Init", "Chip failed to become ready after boot.");
        SPI_close(spi_handle_);
        spi_handle_ = nullptr;
        return false;
    }
    mode_ = ChipMode::kStdbyRC;

    // TODO: Set up NTC compensation.

    CONSOLE_INFO("LR2021::Init", "Initialization complete.");
    return true;
}

bool LR2021::DeInit() {
    CONSOLE_INFO("LR2021::DeInit", "De-initializing.");
    SetEnable(false);
    if (spi_handle_ != nullptr) {
        // Never close the handle with a DMA in flight: cancel any async drain transfer first (also
        // covers EnterSyncSleep, which reaches here with a drain possibly mid-sequence).
        CancelAsync();
        SPI_close(spi_handle_);
        spi_handle_ = nullptr;
    }
    mode_ = ChipMode::kStartup;
    return true;
}

void LR2021::TristateInterface() {
    // Reconfigure the CC1314-driven pins to high-impedance inputs so an external MCU can drive the
    // shared LR2021 bus during SYNC sleep. Drive ENABLE low first (crisp reset edge instead of an RC
    // decay through the pull-down) so the host takes over a cleanly reset chip; harmless no-op when the
    // pin is already tri-stated. Idle-state internal pulls keep the lines defined during the host
    // handoff: NSS pulled up (CS deasserted), ENABLE/SCLK/PICO pulled down. BUSY/POCI are already inputs
    // and are left untouched. Runs from the SYNC rising-edge ISR (GPIO register writes only), possibly
    // while SPI is still open: GPIO_setConfig un-muxes SCLK/PICO from the SPI peripheral, and the later
    // SPI_close leaves them hi-Z (their SysConfig not-in-use configs are inputs).
    SetEnable(false);
    GPIO_setConfig(config_.gpio_nss, GPIO_CFG_INPUT_INTERNAL | GPIO_CFG_IN_INT_NONE | GPIO_CFG_PULL_UP_INTERNAL);
    GPIO_setConfig(config_.gpio_enable, GPIO_CFG_INPUT_INTERNAL | GPIO_CFG_IN_INT_NONE | GPIO_CFG_PULL_DOWN_INTERNAL);
    GPIO_setConfig(config_.gpio_sclk, GPIO_CFG_INPUT_INTERNAL | GPIO_CFG_IN_INT_NONE | GPIO_CFG_PULL_DOWN_INTERNAL);
    GPIO_setConfig(config_.gpio_pico, GPIO_CFG_INPUT_INTERNAL | GPIO_CFG_IN_INT_NONE | GPIO_CFG_PULL_DOWN_INTERNAL);
}

void LR2021::RestoreInterface() {
    // Restore the GPIO-driven pins (NSS, ENABLE) to their driven SysConfig defaults (outputs). The SPI
    // pins (SCLK/PICO/POCI) are re-muxed to the SPI peripheral by SPI_open() in the following Init(), so
    // they need no manual restore here.
    GPIO_resetConfig(config_.gpio_nss);
    GPIO_resetConfig(config_.gpio_enable);
}

// Private helpers

bool LR2021::WaitUntilReady(uint32_t timeout_ms) {
    // Silent bail-out when the SYNC ISR has handed the bus to an external host: BUSY now reflects the
    // host's traffic, and every command sequence must unwind without issuing further transactions. No
    // CONSOLE_ERROR here -- the abort is expected and this runs once per remaining command in the chain.
    if (abort_requested_) {
        return false;
    }
    uint32_t wait_start_time_ms = get_time_since_boot_ms();
    while (IsBusy()) {
        if (abort_requested_) {
            return false;
        }
        // Wait for the chip to finish its internal power-up and calibration sequence.
        if (get_time_since_boot_ms() - wait_start_time_ms > timeout_ms) {
            CONSOLE_ERROR("LR2021::WaitUntilReady", "Timed out after %u ms.", timeout_ms);
            return false;
        }
    }
    return true;
}

bool LR2021::BeginTransaction() {
    // Bus-ownership guard: a synchronous command may be issued (AT handlers, config paths) while the
    // async RX drain is mid-sequence, since a drain spans main-loop iterations. Pump the drain to a
    // quiescent state (idle / data-ready / error: no DMA in flight, NSS high) so the two can't
    // interleave NSS frames. Bounded: in-flight DMA completes in microseconds and every BUSY wait has
    // its own kBusyTimeoutMs, so the worst case is the tail of one drain sequence (~sub-millisecond).
    while (drain_state_ != DrainState::kIdle && drain_state_ != DrainState::kDataReady &&
           drain_state_ != DrainState::kError) {
        if (abort_requested_) {
            return false;
        }
        ServiceRxDrain();
    }
    // Always verify the chip is idle before asserting NSS.  Per §5.4.1.1 the
    // LR2021 will immediately raise BUSY once it sees the NSS falling edge, so
    // checking BUSY *after* asserting NSS would race with the hardware.
    if (!WaitUntilReady()) {
        return false;
    }
    SetNSS(false);
    return true;
}

void LR2021::EndTransaction() { SetNSS(true); }

void LR2021::ParseStat(uint16_t raw) {
    // Table 6-38:
    //   [15:12] always 0
    //   [11:9]  CommandStatus
    //   [8]     InterruptStatus
    //   [7:4]   ResetSource
    //   [3]     rfu
    //   [2:0]   ChipMode
    last_stat_.command_status = static_cast<CommandStatus>((raw >> 9) & 0x7);
    last_stat_.interrupt_active = (raw >> 8) & 0x1;
    last_stat_.reset_source = static_cast<uint8_t>((raw >> 4) & 0xF);
    last_stat_.chip_mode = static_cast<ChipMode>(raw & 0x7);
}

void LR2021::DelayUs(uint32_t us) {
    uint64_t start_time = get_time_since_boot_us();
    while (get_time_since_boot_us() - start_time < us) {
        // Busy wait.
    }
}

// Mode S CRC-24 generator polynomial (0x1FFF409 is 25 bits; bit 24 is the implicit top bit).
static constexpr uint32_t kModeSCrc24Poly = 0x1FFF409;

bool LR2021::SetOokADSB(SettingsManager::R1090PreambleMode preamble_mode, uint8_t agc_gain, uint8_t rx_boost) {
    const bool df17_mode = IsOokDF17PreambleMode(preamble_mode);
    // NOTE: Stat for each command is for the previous instruction, so error messages reflect an error with the previous
    // instruction.
    // The caller (ADSBee::ApplyReceiverConfig) guarantees the chip is in a clean kStdbyRC state via
    // a fresh Init() before this runs, so the config commands below execute from standby.
    if (!SetRfFrequency(1090e6)) {
        CONSOLE_ERROR("LR2021::SetOokADSB", "Error during SetRfFrequency.");
        return false;
    }
    if (rx_boost > RxBoost::kBoostMax) {
        rx_boost = RxBoost::kBoostMax;
    }
    if (!SetRxPathAdv(RxPath::kLfPath, static_cast<RxBoost>(rx_boost))) {
        CONSOLE_ERROR("LR2021::SetOokADSB", "Error during SetRxPathAdv.");
        return false;
    }
    // Calibrate the RF frontend on the LF path at 1090MHz (currently selected RF frequency).
    if (!CalibFe()) {
        CONSOLE_ERROR("LR2021::SetOokADSB", "Error during CalibFe.");
        return false;
    }
    if (!SetPacketType(PacketType::kPktOok)) {
        CONSOLE_ERROR("LR2021::SetOokADSB", "Error during SetPacketType.");
        return false;
    }
    if (!SetOokModulationParams(2e6,                                // 2Mbps bitrate
                                OokPulseShape::kOokPulseShapeNone,  // No pulse shaping
                                OokRxBw::kOokRxBw3076kHz            // 3.076MHz Rx bandwidth

                                )) {
        CONSOLE_ERROR("LR2021::SetOokADSB", "Error during SetOokModulationParams.");
        return false;
    }
    // DF17 mode keys on the DF data bits and captures the message remainder mid-byte, so it runs
    // with the hardware CRC OFF (software validates). MODE_S_PREAMBLE uses the 3-byte hardware CRC
    // (11-byte payload + appended parity). MODE_S_SW_CRC uses the standard preamble detector but
    // runs CRC-off with the parity bytes captured as payload, so software validates -- the FIFO
    // packet is 14 bytes of raw air bits either way (GetOokRxPacketLenBytes).
    uint16_t payload_len_bytes;
    OokCrc crc_mode;
    if (df17_mode) {
        payload_len_bytes = kOokDF17PacketRxLenBytes;
        crc_mode = kOokCrcOff;
    } else if (preamble_mode == SettingsManager::kR1090PreambleModeModeSSwCrc) {
        payload_len_bytes = kOokADSBPacketRxLenBytes + kOokADSBPacketCrcLenBytes;
        crc_mode = kOokCrcOff;
    } else {
        payload_len_bytes = kOokADSBPacketRxLenBytes;
        crc_mode = kOokCrc3Byte;
    }
    if (!SetOokPacketParams(8,                         // Tx preamble length
                            kOokAddrCompOff,           // No address filtering
                            kOokPktFormatFixedLength,  // Fixed length packets
                            payload_len_bytes,         // Payload length (mode dependent)
                            crc_mode,                  // 3-byte hardware CRC or off (mode dependent)
                            kOokEncodingManchesterInv  // Inverted Manchester encoding
                            )) {
        CONSOLE_ERROR("LR2021::SetOokADSB", "Error during SetOokPacketParams.");
        return false;
    }
    // CRC handling. Bit 0x01000000 of 0xF30844 must be CLEARED for Mode S reception. Empirically,
    // setting it breaks reception entirely (it was the old AT+R1090_LR_CRC=1 path): the OOK engine
    // stops delivering discrete fixed-length packets and instead dumps a mangled variable-length
    // stream to the FIFO. With the bit cleared, the engine delivers clean fixed-length packets
    // (11-byte payload + the appended 24-bit Mode S parity = 14 bytes; GetOokRxPacketLenBytes) that
    // the software CRC validates. The 24-bit CRC is already present in the FIFO in this state, so no
    // "append CRC" toggle is needed. We always clear it.
    // https://github.com/TheClams/lr2021/blob/f3a26eed0aebc10f068d4d03a3c133bc97ecdc29/src/radio.rs#L254
    // NOTE(hardware-validate): register 0xF30844 is undocumented; its exact meaning is unconfirmed,
    // but bench testing shows clear = working reception, set = no valid packets.
    if (!WriteRegMemMask32(0xF30844, 0x01000000, 0)) {
        CONSOLE_ERROR("LR2021::SetOokADSB", "Error during WriteRegMemMask32.");
        return false;
    }

    // No sync word in either mode: preamble mode uses the preamble detector; DF17 mode puts the
    // DF=17 data bits directly in the detector pattern.
    if (!SetOokSyncWord(0, kOokBitOrderLsbFirst, 0)) {
        CONSOLE_ERROR("LR2021::SetOokADSB", "Error during SetOokSyncWord.");
        return false;
    }
    if (df17_mode) {
        // Detect on the preamble tail + leading DF=17 data bits rather than the full preamble. The
        // real preamble still lets the (manual) AGC settle before the pattern completes.
        if (!SetOokDetector(kOokDF17Detector.pattern,            // Preamble tail + DF17 header chips
                            kOokDF17Detector.len_chips - 1,      // Pattern length (field is N-1)
                            0,                                   // No pattern repetition
                            false,                               // (no sync word used)
                            OokSfdKind::kOokSfdKindFallingEdge,  // SFD on falling edge
                            0                                    // SFD length
                            )) {
            CONSOLE_ERROR("LR2021::SetOokADSB", "Error during SetOokDetector.");
            return false;
        }
    } else {
        if (!SetOokDetector(0b1010000101,                        // Preamble pattern
                            15,                                  // Pattern length: 16 chips (field N-1)
                            0,                                   // No pattern repetition
                            false,                               // Sync word is not raw
                            OokSfdKind::kOokSfdKindFallingEdge,  // Start frame delimiter on falling edge
                            0                                    // Start frame delimiter length
                            )) {
            CONSOLE_ERROR("LR2021::SetOokADSB", "Error during SetOokDetector.");
            return false;
        }
    }
    // CRC polynomial/init only matter when the hardware CRC is enabled (preamble mode). DF17 mode
    // has CRC off and validates in software, so this is a harmless no-op there.
    if (!SetOokCrcParams(kModeSCrc24Poly, 0)) {
        CONSOLE_ERROR("LR2021::SetOokADSB", "Error during SetOokCrcParams.");
        return false;
    }

    // Set up the FIFO. The high threshold doubles as the IRQ-paced drain valve: kIrqRxFifo latches
    // (and the DIO6 IRQ line rises) only once 9 whole packets have accumulated, i.e. only when the
    // main loop's routine level-read sweep is falling behind. The loop drain does NOT gate on this
    // flag (it reads the level unconditionally), so sub-threshold packets still flow with loop
    // latency.
    static_assert(GetOokRxPacketLenBytes(SettingsManager::kR1090PreambleModeModeS) == kOokFifoPacketLenBytes &&
                      GetOokRxPacketLenBytes(SettingsManager::kR1090PreambleModeDF17) == kOokFifoPacketLenBytes &&
                      GetOokRxPacketLenBytes(SettingsManager::kR1090PreambleModeModeSSwCrc) == kOokFifoPacketLenBytes,
                  "IRQ drain threshold math assumes 14-byte FIFO packets in every preamble mode.");
    uint8_t rx_fifo_flags = kFifoIrqFlagFifoHigh | kFifoIrqFlagFifoOverflow;
    uint8_t tx_fifo_flags = 0x0;
    uint16_t rx_fifo_low_threshold = 0;  // Not actually used.
    uint16_t rx_fifo_high_threshold = kIrqDrainThresholdBytes;
    if (!ConfigFifoIrqAdv(rx_fifo_flags, tx_fifo_flags, rx_fifo_high_threshold, 0, rx_fifo_low_threshold, 0)) {
        CONSOLE_ERROR("LR2021::SetOokADSB", "Error during ConfigFifoIrqAdv.");
        return false;
    }
    // Route the RX FIFO IRQ to DIO6, which is wired to the CC1314's LR_IRQ pin (rising-edge
    // interrupt; see lr2021_irq_drain.cpp). The IRQ register bit is latched: the drain paths clear it
    // over SPI (ClearFifoIrqFlags then GetAndClearIrq) to drop the line.
    if (!SetDioFunction(DioNum::kDio6, DioFunc::kDioFuncIrq, PullDrive::kPullNone)) {
        CONSOLE_ERROR("LR2021::SetOokADSB", "Error during SetDioFunction for the IRQ line.");
        return false;
    }
    if (!SetDioIrqConfig(DioNum::kDio6, HostIrqs::kIrqRxFifo)) {
        CONSOLE_ERROR("LR2021::SetOokADSB", "Error during SetDioIrqConfig for the IRQ line.");
        return false;
    }

    if (!SetAgcGainManual(agc_gain)) {  // 0 = auto, 1..15 manual (13 = max).
        CONSOLE_ERROR("LR2021::SetOokADSB", "Error during SetAgcGainManual.");
        return false;
    }

    uint32_t rx_timeout = 0xFFFFFF;  // Continuous Rx mode (no timeout).
    if (!SetRxAdv(rx_timeout)) {
        CONSOLE_ERROR("LR2021::SetOokADSB", "Error during SetRxAdv.");
        return false;
    }

    // One more status command to mop up errors from previous command.
    LR2021::StatusRsp stat;
    if (!GetStatus(&stat)) {
        CONSOLE_ERROR("LR2021::SetOokADSB", "Error during GetStatus.");
        return false;
    }

    return true;
}

#ifdef HARDWARE_UNIT_TESTS
bool LR2021::StartCwTone(bool use_hf_path, uint32_t freq_hz, int8_t tx_power_dbm) {
    // Begin from a clean standby state.
    if (!SetStandby(SysStandbyMode::kSysStandbyXosc)) {
        CONSOLE_ERROR("LR2021::StartCwTone", "Error during SetStandby.");
        return false;
    }
    // Select a continuous-carrier packet type. The chip otherwise keeps the OOK packet type left over
    // from the ADS-B receiver config (SetOokADSB), and in OOK the modulator gates the carrier off, so
    // the TONE test only leaks the LO (~-40 dBm) and the PA never keys. Generic FSK transmits an
    // unmodulated continuous carrier in TONE test mode.
    if (!SetPacketType(PacketType::kPktFskGeneric)) {
        CONSOLE_ERROR("LR2021::StartCwTone", "Error during SetPacketType.");
        return false;
    }
    if (!SetRfFrequency(freq_hz)) {
        CONSOLE_ERROR("LR2021::StartCwTone", "Error during SetRfFrequency.");
        return false;
    }

    // Select and configure the requested PA. pa_lf_duty_cycle and pa_lf_slices control the PA's duty
    // cycle and maximum output power for BOTH the LF and HF PAs; their standard values are 6 and 7
    // respectively (DS.LR2021 / Semtech reference set_pa_hf()). The basic SetPaConfig leaves
    // pa_hf_duty_cycle at its firmware default (16).
    const PaSel pa_sel = use_hf_path ? PaSel::kHfPa : PaSel::kLfPa;
    if (!SetPaConfig(pa_sel, PaLfMode::kPaLfFsm, /*pa_lf_duty_cycle=*/6, /*pa_lf_slices=*/7)) {
        CONSOLE_ERROR("LR2021::StartCwTone", "Error during SetPaConfig.");
        return false;
    }
    if (!SelPa(pa_sel)) {
        CONSOLE_ERROR("LR2021::StartCwTone", "Error during SelPa.");
        return false;
    }
    // tx_power is in 0.5 dB steps. Convert the requested dBm and clamp to the active path's range
    // (LF: -19..44 = -9.5..22 dBm, HF: -39..24 = -19.5..12 dBm).
    int16_t tx_power_half_db = (int16_t)tx_power_dbm * 2;
    const int16_t tx_power_min = use_hf_path ? -39 : -19;
    const int16_t tx_power_max = use_hf_path ? 24 : 44;
    if (tx_power_half_db < tx_power_min) tx_power_half_db = tx_power_min;
    if (tx_power_half_db > tx_power_max) tx_power_half_db = tx_power_max;
    if (!SetTxParams((int8_t)tx_power_half_db, RampTime::kRamp16u)) {
        CONSOLE_ERROR("LR2021::StartCwTone", "Error during SetTxParams.");
        return false;
    }

    // Calibrate the frontend on the selected path. CalibFe takes the frequency in 4 MHz steps with the
    // MSb selecting the path (0 = LF, 1 = HF); without it CalibFe() would calibrate the LF path.
    uint16_t calib_word = (uint16_t)(freq_hz / 4000000u);
    if (use_hf_path) {
        calib_word |= 0x8000;
    }
    if (!CalibFe(calib_word)) {
        CONSOLE_ERROR("LR2021::StartCwTone", "Error during CalibFe.");
        return false;
    }

    // Key up the continuous tone (unmodulated carrier).
    if (!SetTxTestMode(TxTestMode::kTestTone)) {
        CONSOLE_ERROR("LR2021::StartCwTone", "Error during SetTxTestMode.");
        return false;
    }

    // Read back the chip status to confirm the tone actually put the chip into TX mode. The status word
    // reflects the previous command, so issue a fresh GetStatus transaction (which updates last_stat_)
    // and then inspect last_stat_.chip_mode. Expect kTx (0x5); anything else means the tone did not key.
    LR2021::StatusRsp irq_rsp;
    if (!GetStatus(&irq_rsp)) {
        CONSOLE_ERROR("LR2021::StartCwTone", "Error during GetStatus after keying tone.");
        return false;
    }
    CONSOLE_INFO("LR2021::StartCwTone", "post-tone chip_mode=0x%x (kTx=0x5) cmd_status=%s",
                 (unsigned)last_stat_.chip_mode, CommandStatusToString(last_stat_.command_status));
    // The carrier is only actually keyed if the chip entered TX. Report failure otherwise so callers
    // don't treat a silent radio as a live carrier.
    if (last_stat_.chip_mode != ChipMode::kTx) {
        CONSOLE_ERROR("LR2021::StartCwTone", "Tone did not key TX: chip_mode=0x%x (expected kTx=0x5).",
                      (unsigned)last_stat_.chip_mode);
        return false;
    }
    return true;
}

bool LR2021::StopCwTone() {
    // Returning to standby drops the carrier.
    if (!SetStandby(SysStandbyMode::kSysStandbyRc)) {
        CONSOLE_ERROR("LR2021::StopCwTone", "Error during SetStandby.");
        return false;
    }
    return true;
}

bool LR2021::StartRssiScan(bool use_hf_path, uint32_t freq_hz) {
    // Reconfigure from a clean hardware reset (-> kStdbyRC), mirroring ADSBee::ApplyReceiverConfig():
    // the LR2021's RF/AGC calibration must be set up from kStdbyRC, and reconfiguring out of
    // continuous RX via SetStandby leaves CalibFe failing with CMD_FAIL (observed on hardware).
    DeInit();  // Safe even if never inited (SPI_close is guarded).
    if (!Init()) {
        CONSOLE_ERROR("LR2021::StartRssiScan", "Error during Init.");
        return false;
    }
    if (!SetRfFrequency(freq_hz)) {
        CONSOLE_ERROR("LR2021::StartRssiScan", "Error during SetRfFrequency.");
        return false;
    }
    // Boost off so the reading isn't skewed by the extra front-end gain.
    if (!SetRxPathAdv(use_hf_path ? RxPath::kHfPath : RxPath::kLfPath, RxBoost::kBoostOff)) {
        CONSOLE_ERROR("LR2021::StartRssiScan", "Error during SetRxPathAdv.");
        return false;
    }
    // Calibrate the frontend on the currently selected path at the currently set RF frequency, exactly
    // as SetOokADSB does. (Explicit calib words, as used in the StartCwTone TX flow, fail with
    // CMD_FAIL in this RX bring-up.)
    if (!CalibFe()) {
        CONSOLE_ERROR("LR2021::StartRssiScan", "Error during CalibFe.");
        return false;
    }
    // The RX chain needs a packet type and modulation config even for a bare RSSI measurement; reuse
    // the known-good OOK config from SetOokADSB, which fixes the RX/RSSI bandwidth at ~3.076 MHz.
    if (!SetPacketType(PacketType::kPktOok)) {
        CONSOLE_ERROR("LR2021::StartRssiScan", "Error during SetPacketType.");
        return false;
    }
    if (!SetOokModulationParams(2e6,                                // 2Mbps bitrate
                                OokPulseShape::kOokPulseShapeNone,  // No pulse shaping
                                OokRxBw::kOokRxBw3076kHz            // 3.076MHz Rx bandwidth
                                )) {
        CONSOLE_ERROR("LR2021::StartRssiScan", "Error during SetOokModulationParams.");
        return false;
    }
    // Auto AGC so GetRssiInst tracks the input power across its full range.
    if (!SetAgcGainManual(0)) {
        CONSOLE_ERROR("LR2021::StartRssiScan", "Error during SetAgcGainManual.");
        return false;
    }
    uint32_t rx_timeout = 0xFFFFFF;  // Continuous Rx mode (no timeout).
    if (!SetRxAdv(rx_timeout)) {
        CONSOLE_ERROR("LR2021::StartRssiScan", "Error during SetRxAdv.");
        return false;
    }
    // One more status command to mop up errors from the previous command, then verify the chip
    // actually entered RX so callers don't poll RSSI from a chip sitting in standby.
    LR2021::StatusRsp stat;
    if (!GetStatus(&stat)) {
        CONSOLE_ERROR("LR2021::StartRssiScan", "Error during GetStatus.");
        return false;
    }
    if (last_stat_.chip_mode != ChipMode::kRx) {
        CONSOLE_ERROR("LR2021::StartRssiScan", "Chip did not enter RX: chip_mode=0x%x (expected kRx=0x%x).",
                      (unsigned)last_stat_.chip_mode, (unsigned)ChipMode::kRx);
        return false;
    }
    return true;
}
#endif