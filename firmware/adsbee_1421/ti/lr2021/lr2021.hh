// lr2021.h
//
// Driver for the Semtech LR2021 multi-protocol sub-GHz / 2.4 GHz transceiver.
//
// Datasheet reference: DS.LR2021 Rev. 1.1, 10/14/25.
// All section and table numbers cited in comments refer to that document.
//
// Scope of this revision
//   Only register/memory access (§5.6.2) is implemented:
//     • WriteRegMem32      (opcode 0x0104) – block write, up to 32 words
//     • WriteRegMemMask32  (opcode 0x0105) – single read-modify-write
//     • ReadRegMem32       (opcode 0x0106) – block read,  up to 32 words
//
// SPI framing overview (§5.4.1)
//   Write command (single frame):
//     MOSI: Op[15:8]  Op[7:0]  Arg0  Arg1 ...
//     MISO: Stat[15:8] Stat[7:0] IrqStat[31:0] 0x00 ...
//
//   Read command (two frames):
//     Frame 1 – send opcode + arguments (BUSY asserts then deasserts)
//     Frame 2 – send 0x00 bytes; device clocks out Stat + data
//
// Typical usage
//   MyHal hal;
//   LR2021 radio(&hal);
//   uint32_t words[2];
//   if (!radio.ReadRegMem32(0x080004, words, 2)) { /* handle error */ }

#pragma once

// TI drivers
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>

#include <cstddef>
#include <cstdint>

#include "settings.hh"  // For SettingsManager::R1090PreambleMode.

class LR2021 {
   public:
    static const uint32_t kBootupTimeoutMs = 10;
    static const uint32_t kBusyTimeoutMs = 100;
    static constexpr uint16_t kRxFifoMaxDepthBytes = 256;

    static constexpr uint16_t kOokADSBPacketRxLenBytes = 11;  // 88-bit payload (DF .. end of data).
    static constexpr uint16_t kOokADSBPacketCrcLenBytes = 3;  // 24-bit CRC (parity).

    // DF17 sync mode: instead of correlating on the full preamble, the detector keys on the tail of
    // the preamble plus the leading DF=17 data bits. The detector consumes the trailing DF bits of
    // its pattern, so the captured payload starts mid-byte -- the byte-granular hardware CRC can't
    // align to that, so DF17 mode runs with the hardware CRC OFF and validates in software over the
    // reconstructed full frame.
    //
    // SetOokDetector preamble_pattern is LSB-first (LSB = first chip received) and its 2 LSBs MUST
    // be 01 or 10 (datasheet §16.3.8, p.197). ADS-B chips are inverse-Manchester (1->10, 0->01).
    //
    // The header fields describe the DF bits the detector consumes, which the RX path prepends back
    // onto the captured remainder during frame reconstruction.
    struct OokDetectorConfig {
        uint16_t pattern;          // Detection pattern, LSB-first chips.
        uint8_t len_chips;         // Pattern length in chips (SetOokDetector length field is N-1).
        uint16_t header_bits;      // DF bits consumed by the detector (MSB-first bit values).
        uint16_t header_len_bits;  // Number of DF bits consumed.
    };
    // 2nd half of the preamble (chips 8-15 = "01000000", one pulse + quiet tail) followed by the
    // first 4 DF bits "1000" (8 chips "10010101"). 16 chips total -- the hardware max -- for
    // stronger correlation (datasheet §11.1.3 recommends >=20 bits of detection at osr=4; longer is
    // better). Capture starts at message bit 4.
    static constexpr OokDetectorConfig kOokDF17Detector = {0xA902, 16, 0b1000, 4};
    // Payload bytes captured after the detector pattern in DF17 mode (CRC off, so no CRC bytes
    // appended). 14 bytes = 112 bits covers the message remainder; the last few captured bits
    // (header_len_bits worth) are slop the reconstruction ignores.
    static constexpr uint16_t kOokDF17PacketRxLenBytes = 14;

    // True for the preamble mode that detects on DF17 header bits (hardware CRC off, mid-byte capture).
    static constexpr bool IsOokDF17PreambleMode(SettingsManager::R1090PreambleMode preamble_mode) {
        return preamble_mode == SettingsManager::kR1090PreambleModeDF17;
    }

    // Total FIFO bytes per received packet for the given preamble mode. Mode S preamble mode appends
    // a 3-byte hardware CRC (11 + 3); MODE_S_SW_CRC captures the parity bytes as payload (14); DF17
    // mode runs CRC-off so its payload bytes are the whole packet (14). All modes come out to 14.
    static constexpr uint16_t GetOokRxPacketLenBytes(SettingsManager::R1090PreambleMode preamble_mode) {
        return IsOokDF17PreambleMode(preamble_mode) ? kOokDF17PacketRxLenBytes
                                                    : (kOokADSBPacketRxLenBytes + kOokADSBPacketCrcLenBytes);
    }

    struct LR2021Config {
        // Add any configuration parameters you need here, e.g. GPIO indices or
        // SPI handle.  The LR2021HALCC131X implementation (lr2021_hal_cc1314.hh)
        // can serve as a reference.
        uint_least8_t spi_index;  // Board SPI peripheral index passed to SPI_open().
        uint_least8_t gpio_nss;
        uint_least8_t gpio_busy;
        uint_least8_t gpio_enable;
        // SPI data/clock pins, needed to tri-state the interface for SYNC sleep (TristateInterface()).
        uint_least8_t gpio_sclk;
        uint_least8_t gpio_pico;
        // CC1314 pin wired to LR2021 DIO6, configured chip-side as the kIrqRxFifo IRQ output. Read
        // (level) by the drain paths and used as a rising-edge interrupt by the IRQ-paced drain chain.
        uint_least8_t gpio_irq;
    };

    // CommandStatus field from the Stat word (Table 6-38, bits 11:9).
    enum CommandStatus : uint8_t {
        kFail = 0x0,  // CMD_FAIL – command could not be executed
        kPErr = 0x1,  // CMD_PERR – wrong opcode or bad arguments
        kOk = 0x2,    // CMD_OK   – command processed successfully
        kDat = 0x3,   // CMD_DAT  – read command processed; data follows
    };

    static char* CommandStatusToString(CommandStatus cs) {
        switch (cs) {
            case CommandStatus::kFail:
                return (char*)"CMD_FAIL";
            case CommandStatus::kPErr:
                return (char*)"CMD_PERR";
            case CommandStatus::kOk:
                return (char*)"CMD_OK";
            case CommandStatus::kDat:
                return (char*)"CMD_DAT";
            default:
                return (char*)"UNKNOWN";
        }
    }

    enum ChipMode : uint8_t {
        kSleep = 0x0,
        kStdbyRC = 0x1,    // Standby with internal RC oscillator
        kStdbyXOSC = 0x2,  // Standby with crystal oscillator
        kFS = 0x3,         // Frequency synthesizer
        kRx = 0x4,
        kTx = 0x5,
        kStartup = 0xF0  // Not an actual mode, added for convenience.
    };

    // These are the bit flags used to configure IRQs for a single FIFO.
    enum FifoIrqFlags : uint8_t {
        kFifoIrqFlagFifoEmpty = 0x01,
        kFifoIrqFlagFifoLow = 0x02,
        kFifoIrqFlagFifoHigh = 0x04,
        kFifoIrqFlagFifoFull = 0x08,
        kFifoIrqFlagFifoOverflow = 0x10,
        kFifoIrqFlagFifoUnderflow = 0x20,
    };
    // Sub-flags that imply the RX FIFO byte stream may be torn / packet-misaligned, so observing any
    // of them must schedule the thread-level flush/realign (fifo_overflow_pending): Full and Overflow
    // mean writes raced a hard ceiling (tear on the write side); Underflow means a read popped past
    // the write pointer (tear on the read side).
    static constexpr uint8_t kFifoIrqFlagsResyncMask =
        kFifoIrqFlagFifoFull | kFifoIrqFlagFifoOverflow | kFifoIrqFlagFifoUnderflow;

    enum HostIrqs : uint32_t {
        kIrqRxFifo = (1u << 0),              // Rx FIFO threshold reached
        kIrqTxFifo = (1u << 1),              // Tx FIFO threshold reached
        kIrqRngReqVld = (1u << 2),           // Valid ranging request received (Subordinate)
        kIrqTxTimestamp = (1u << 3),         // IRQ for time-stamping end of packet Tx
        kIrqRxTimestamp = (1u << 4),         // IRQ for time-stamping end of packet Rx
        kIrqPreambleDetected = (1u << 5),    // Preamble detected
        kIrqLoRaHeaderValid = (1u << 6),     // LoRa header detected / Valid sync word
        kIrqCadDetected = (1u << 7),         // Channel activity detected
        kIrqLoRaHdrTimestamp = (1u << 8),    // LoRa header precise time-stamp
        kIrqLoRaHeaderErr = (1u << 9),       // LoRa header CRC error
        kIrqLowBattery = (1u << 10),         // Low Battery detector
        kIrqPaOcpOvp = (1u << 11),           // PA OCP/OVP has triggered
        kIrqLoRaTxRxHop = (1u << 12),        // IRQ for LoRa intra-packet hopping
        kIrqSyncFail = (1u << 13),           // Syncword match failed after detection
        kIrqLoRaSymbolEnd = (1u << 14),      // End of LoRa symbol
        kIrqLoRaTimestampStat = (1u << 15),  // New statistics available in time-stamp register
        kIrqError = (1u << 16),              // An error other than a command error occurred
        kIrqCmdError = (1u << 17),           // There was a host command fail/error
        kIrqRxDone = (1u << 18),             // Packet reception completed
        kIrqTxDone = (1u << 19),             // Packet transmission completed
        kIrqCadDone = (1u << 20),            // Channel activity detection finished
        kIrqTimeout = (1u << 21),            // Rx or Tx timeout
        kIrqCrcError = (1u << 22),           // Packet received with wrong CRC
        kIrqLenError = (1u << 23),           // Packet received with a length error
        kIrqAddrError = (1u << 24),          // Packet received with wrong address match
        kIrqFHSS = (1u << 25),               // Ramp-up for intra-packet hopping occurred
        kIrqInterPacket1 = (1u << 26),       // Host can load new frequencies table
        kIrqInterPacket2 = (1u << 27),       // Host can load new payload
        kIrqRngRespDone = (1u << 28),        // Subordinate sent its ranging response
        kIrqRngReqDis = (1u << 29),          // Ranging request discarded (no address match)
        kIrqRngExchVld = (1u << 30),         // Manager received valid ranging response from Subordinate
        kIrqRngTimeout = (1u << 31),         // Manager did not receive a response from the Subordinate
    };

    // Parsed representation of the 16-bit Stat word (Table 6-38).
    //
    //  Bits 15:12 – always 0
    //  Bits 11:9  – CommandStatus
    //  Bit  8     – InterruptStatus (1 = at least one IRQ pending)
    //  Bits 7:4   – ResetSource (0=cleared, 1=POR/BRN, 2=NRESET pin)
    //  Bit  3     – reserved
    //  Bits 2:0   – ChipMode
    struct Stat {
        CommandStatus command_status;
        bool interrupt_active;
        uint8_t reset_source;
        ChipMode chip_mode;
    };

    // Constants

    // Maximum words transferable in one WriteRegMem32 / ReadRegMem32 call.
    // The LR2021 firmware caps block transfers at 32 words (§6.2.1, §6.2.3).
    static constexpr size_t kMaxBlockWords = 32;

    // Number of IsBusy() polls before WaitUntilReady() declares a timeout.
    // At 80 MHz with ~5 cycles per iteration this is roughly 6 ms; tune as
    // needed for your application's worst-case command latency.
    static constexpr uint32_t kBusyTimeoutIterations = 100'000;

    // Construction

    /**
     * @param[in] config  Configuration parameters; must remain valid for the lifetime of this object.
     */
    LR2021(LR2021Config config);
    ~LR2021() = default;

    LR2021(const LR2021&) = delete;
    LR2021& operator=(const LR2021&) = delete;

    // Init

    bool Init();
    bool DeInit();

    // Requests that any in-progress or future command sequence bail out at the next inter-transaction
    // boundary (checked in WaitUntilReady(), which every command passes through via BeginTransaction()).
    // ISR-safe (single volatile store); called from the SYNC rising-edge interrupt so the main-line code
    // stops touching the handed-off bus. Cleared by the next Init().
    void RequestAbort() { abort_requested_ = true; }

    // Tri-states the CC1314-driven LR2021 interface pins (NSS, ENABLE, SCLK, PICO) to high-impedance
    // inputs so an external MCU can take control of the shared bus during SYNC sleep. Drives ENABLE low
    // first so the host receives a cleanly reset chip. Each pin gets an idle-state internal pull (NSS up;
    // ENABLE/SCLK/PICO down). BUSY/POCI are already inputs and are left untouched. ISR-safe (GPIO
    // register writes only), and safe to call while SPI is still open: GPIO_setConfig un-muxes SCLK/PICO
    // from the SPI peripheral, and an in-flight DMA transfer completes internally against the un-muxed
    // pins. SPI_close afterwards leaves the pins hi-Z (SysConfig not-in-use configs are inputs).
    void TristateInterface();
    // Restores the driven pins tri-stated by TristateInterface(). NSS/ENABLE are reset to their driven
    // SysConfig defaults (outputs); the SPI pins (SCLK/PICO/POCI) are re-muxed by SPI_open() in Init().
    // Call BEFORE the next Init()/ApplyReceiverConfig(), which drives NSS/ENABLE via GPIO_write.
    void RestoreInterface();

    // Async RX drain
    //
    // Non-blocking, DMA-driven version of the steady-state RX poll sequence (GetAndClearIrq ->
    // GetRxFifoLevel -> ReadRxFifo). The drain is a state machine advanced from thread level via
    // ServiceRxDrain(); the SPI completion callback (SWI context) only raises NSS and sets a flag, so it
    // cannot encroach on the UAT RF SWI's ~15 us CMD_PROP_SET_LEN deadline (see sub_ghz_radio.cpp).
    // Implementation in lr2021_async.cpp. Usage (see ADSBee::UpdateLR2021):
    //   if (lr2021.DrainIdle()) lr2021.StartRxDrain();
    //   switch (lr2021.ServiceRxDrain()) {
    //       case LR2021::DrainResult::kDataReady: /* parse drain_payload() */ lr2021.FinishRxDrain(); ...
    //   }
    enum class DrainResult : uint8_t {
        kIdle = 0,    // No drain in progress.
        kInProgress,  // DMA in flight or waiting on BUSY; call again next loop iteration.
        kNoData,      // Drain completed with nothing to read (no RX FIFO IRQ, or FIFO empty). Back to idle.
        kDataReady,   // Payload available via drain_payload()/drain_level(); call FinishRxDrain() when done.
        kError,       // SPI / stat / BUSY-timeout error; reason via drain_error_str(). Call FinishRxDrain().
    };

    // Kicks off a drain sequence. Returns false (and stays idle) if a drain is already in progress, the
    // SPI handle is closed, or an abort is pending.
    bool StartRxDrain();
    // Advances the drain as far as currently possible and returns its status. Never blocks beyond GPIO
    // reads; call once per main-loop iteration. Also backstops the IRQ-paced chain: re-posts a chain
    // frame the BUSY edge missed, and cancels a chain wedged longer than kIrqChainTimeoutMs.
    DrainResult ServiceRxDrain();
    // Releases a terminal kDataReady / kError drain back to idle. drain_payload() is valid until this.
    void FinishRxDrain();
    bool DrainIdle() const { return drain_state_ == DrainState::kIdle; }

    // IRQ-paced RX drain chain (lr2021_irq_drain.cpp)
    //
    // The LR2021 raises its DIO6 IRQ line (-> CC1314 LR_IRQ pin, rising edge) when the RX FIFO crosses
    // kIrqDrainThresholdBytes. The GPIO ISR claims the drain state machine and runs a fully ISR/SWI-paced
    // chain -- ReadRxFifo(threshold) DMA'd into a staging slot, then GetAndClearFifoIrqFlags (overflow
    // check) + GetAndClearIrq to drop the latched IRQ line -- with zero busy-waiting and zero main-loop
    // involvement. The main
    // loop later parses filled slots via NextFilledSlot()/ReleaseSlot(). This is a latency/backpressure
    // valve: with the threshold at 9 packets it stays silent while the routine loop drain keeps up.

    // One 14-byte-packet-aligned FIFO packet (all preamble modes; see static_assert in lr2021.cpp).
    static constexpr uint16_t kOokFifoPacketLenBytes = 14;
    // RX FIFO high threshold and chain read size: 9 whole packets. Leaves 256-126 = 130 B (~1.1 ms of
    // saturated 1090 traffic) of overflow margin while a ~300 us chain runs.
    static constexpr uint16_t kIrqDrainThresholdBytes = 9 * kOokFifoPacketLenBytes;  // 126
    // A healthy chain finishes in ~300 us; a chain stuck longer than this is cancelled by
    // ServiceRxDrain() at thread level.
    static constexpr uint32_t kIrqChainTimeoutMs = 10;
    static constexpr uint8_t kNumIrqDrainSlots = 2;

    // Staging slot: filled by the chain (ISR/SWI context), parsed and released by the main loop.
    // timestamp_us is captured at the IRQ edge -- closer to reception than parse-time stamping.
    struct IrqDrainSlot {
        uint8_t buf[2 + kIrqDrainThresholdBytes];  // [0:1] = Stat, [2..] = FIFO payload (DMA target).
        volatile uint16_t len = 0;                 // Payload bytes valid at buf + 2.
        volatile uint64_t timestamp_us = 0;
        volatile bool filled = false;  // Written last by the producer; cleared by ReleaseSlot().
    };

    // GPIO callback bodies (HWI context, <= ~5 us each -- they preempt the UAT RF SWI's ~15 us
    // CMD_PROP_SET_LEN deadline). Wired up by ADSBee to the LR_IRQ rising edge / LR_BUSY falling edge.
    void HandleIrqLine();
    void HandleBusyFall();
    // Oldest filled staging slot, or nullptr. Thread level; pair each non-null return with ReleaseSlot().
    IrqDrainSlot* NextFilledSlot();
    void ReleaseSlot();

    // IRQ chain diagnostics, reported/reset via AT+RX_STATS.
    volatile uint32_t irq_drain_count = 0;         // Chains started from the LR_IRQ edge.
    volatile uint32_t irq_drain_skip_busy = 0;     // Edge ignored: drain not claimable (in flight/terminal).
    volatile uint32_t irq_drain_skip_slots = 0;    // Edge ignored: no free staging slot.
    volatile uint32_t irq_drain_restarts = 0;      // Chain looped straight into another read (line still high).
    volatile uint32_t irq_chain_timeouts = 0;      // Wedged chains cancelled by the thread backstop.
    volatile uint32_t irq_chain_errors = 0;        // Chain aborted on SPI post failure or bad Stat.
    volatile uint32_t irq_thread_assist_posts = 0; // Chain frames posted by the thread backstop (missed BUSY edge).
    volatile uint32_t fifo_overflow_count = 0;     // FIFO overflow sub-flag observations (either drain path).
    volatile uint32_t drain_dma_timeouts = 0;      // Loop-drain DMA completions that never arrived (cancelled).
    volatile uint32_t irq_stale_edges = 0;         // Latched LR_IRQ edges ignored because the line had dropped.
    // Latched when either drain tail observes the FIFO overflow sub-flag: the FIFO byte stream may be
    // torn mid-packet, and 14-byte-aligned reads would stay misaligned (every slice failing CRC) until
    // the FIFO is flushed. Set from the chain SWI or the loop-drain thread parse; cleared only by the
    // thread-level resync in ADSBee::UpdateLR2021 after it has quiesced the chain, so no lock is needed
    // (a set landing after the clear just schedules one redundant resync). Reset by Init().
    volatile bool fifo_overflow_pending = false;
    // Cancels any in-flight async transfer and resets the drain to idle. Thread level only (never from an
    // ISR); DeInit() calls this so the SPI handle is never closed with a DMA in flight.
    void CancelAsync();
    // Valid in kDataReady until FinishRxDrain(): FIFO payload pointer and its length in bytes.
    const uint8_t* drain_payload() const { return async_rx_buf_ + 2; }
    uint16_t drain_level() const { return drain_fifo_level_; }
    // True if the level readback reported the FIFO completely full (poll fell behind; frames were likely
    // dropped by the radio on top of what we read).
    bool drain_fifo_was_full() const { return drain_fifo_full_; }
    uint32_t drain_irq_flags() const { return drain_irq_flags_; }
    const char* drain_error_str() const { return drain_error_str_ ? drain_error_str_ : "none"; }

    // System firmware command enums (§5.6.1, opcodes 0x01xx)

    enum DioNum : uint8_t {
        kDio5 = 5,
        kDio6 = 6,
        kDio7 = 7,
        kDio8 = 8,
        kDio9 = 9,
        kDio10 = 10,
        kDio11 = 11,
    };

    enum DioFunc : uint8_t {
        kDioFuncNone = 0,
        kDioFuncIrq = 1,
        kDioFuncRfSwitch = 2,
        kDioFuncGpioOutputLow = 5,
        kDioFuncGpioOutputHigh = 6,
        kDioFuncHfClkOut = 7,
        kDioFuncLfClkOut = 8,
        kDioFuncTxTrigger = 9,
        kDioFuncRxTrigger = 10,
    };

    /// Pull-up/down configuration for sleep mode.  PullAuto means the pin
    /// retains the value it had in Standby.
    enum PullDrive : uint8_t {
        kPullNone = 0,
        kPullDown = 1,
        kPullUp = 2,
        kPullAuto = 3,
    };

    enum LfClock : uint8_t {
        kLfClockRc = 0,
        kLfClockXtal = 1,
        kLfClockDio11 = 2,
    };

    enum ClkScaling : uint8_t {
        kClkDiv1 = 0,
        kClkDiv2 = 1,
        kClkDiv4 = 2,
        kClkDiv8 = 3,
        kClkDiv16 = 4,
        kClkDiv32 = 5,
        kClkDiv64 = 6,
        kClkDiv128 = 7,
        kClkDiv256 = 8,
        kClkDiv512 = 9,
        kClkDiv1024 = 10,
        kClkDiv2048 = 11,
        kClkDiv4096 = 12,
        kClkDiv8192 = 13,
        kClkDiv16384 = 14,
        kClkDiv32768 = 15,
    };

    enum SimoUsage : uint8_t {
        kSimoOff = 0,
        kSimoAll = 1,
        kSimoAuto = 2,
        kSimoVdcc = 3,
    };

    /// Resolution unit shared by all four ramp-time fields in SetRegModeAdv.
    enum RampTimeUnit : uint8_t {
        kRampUnitRes2u = 0,
        kRampUnitRes4u = 1,
        kRampUnitRes8u = 2,
        kRampUnitRes16u = 3,
    };

    enum VbatFormat : uint8_t {
        kVbatRaw = 0,         // 13-bit ADC code
        kVbatMillivolts = 1,  // millivolts
    };

    enum AdcRes : uint8_t {
        kAdcRes8bit = 0,
        kAdcRes9bit = 1,
        kAdcRes10bit = 2,
        kAdcRes11bit = 3,
        kAdcRes12bit = 4,
        kAdcRes13bit = 5,
    };

    enum TempSrc : uint8_t {
        kTempSrcVbe = 0,
        kTempSrcXosc = 1,
        kTempSrcNtc = 2,
    };

    /// Oscillator used while in Standby mode (argument to SetStandby).
    /// Distinct from FallbackMode which governs post-RX/TX transitions.
    enum SysStandbyMode : uint8_t {
        kSysStandbyRc = 0,
        kSysStandbyXosc = 1,
    };

    enum EolTrim : uint8_t {
        kEol1p60 = 0,
        kEol1p67 = 1,
        kEol1p74 = 2,
        kEol1p80 = 3,
        kEol1p88 = 4,
        kEol1p95 = 5,
        kEol2p00 = 6,
        kEol2p10 = 7,
    };

    enum TcxoVoltage : uint8_t {
        kTcxo1v6 = 0,
        kTcxo1v7 = 1,
        kTcxo1v8 = 2,
        kTcxo2v2 = 3,
        kTcxo2v4 = 4,
        kTcxo2v7 = 5,
        kTcxo3v0 = 6,
        kTcxo3v3 = 7,
    };

    enum CompMode : uint8_t {
        kCompDisabled = 0,
        kCompRelative = 1,
        kCompAbsolute = 2,
    };

    // System firmware response structs

    /// Response from GetStatus.  last_stat() is also updated.
    struct StatusRsp {
        uint32_t irq_status;  ///< Pending IRQ flags (32-bit register)
    };

    struct VersionRsp {
        uint8_t major;
        uint8_t minor;
    };

    /// Raw 16-bit error register.  Bit positions follow Table 6-xx of DS.LR2021.
    struct ErrorsRsp {
        uint16_t error_flags;
    };

    struct GetAndClearIrqRsp {
        uint32_t irq_flags;
    };

    struct FifoIrqFlagsRsp {
        uint8_t rx_flags;
        uint8_t tx_flags;
    };

    struct FifoLevelRsp {
        uint16_t level;  ///< FIFO level in bytes
    };

    /// Raw 16-bit word from GetVBat.
    /// If format == kVbatRaw      → mask to 0x1FFF to obtain 13-bit ADC code.
    /// If format == kVbatMillivolts → use directly as millivolts.
    struct VBatRsp {
        uint16_t value;
    };

    /// Temperature in degrees Celsius (13-bit two's-complement, unit = 1 °C).
    struct TempRsp {
        int16_t temp_celsius;
    };

    struct RandomNumberRsp {
        uint32_t value;
    };

    // System Firmware Commands (§5.6.1, opcodes 0x0100–0x0133)

    /** Returns the device status word and current IRQ register. */
    bool GetStatus(StatusRsp* rsp_out);

    /** Returns the firmware version. */
    bool GetVersion(VersionRsp* rsp_out);

    /** Returns pending error flags accumulated since the last ClearErrors(). */
    bool GetErrors(ErrorsRsp* rsp_out);

    /** Clears all error flags.  Does NOT clear the Error IRQ — use ClearIrq() for that. */
    bool ClearErrors();

    /** Configures the function and sleep-mode pull of a freely configurable DIO. */
    bool SetDioFunction(DioNum dio_num, DioFunc dio_func, PullDrive pull_drive);

    /** Configures the DIO output value for each operating mode when used as an RF switch. */
    bool SetDioRfSwitchConfig(DioNum dio_num, bool tx_hf, bool rx_hf, bool tx_lf, bool rx_lf, bool standby);

    /** Clears specific FIFO IRQ flags. */
    bool ClearFifoIrqFlags(uint8_t rx_flags, uint8_t tx_flags);

    /** Configures which IRQs are routed to the specified DIO pin. */
    bool SetDioIrqConfig(DioNum dio_num, uint32_t irqs);

    /** Clears the specified pending IRQ flags. */
    bool ClearIrq(uint32_t irqs);

    /** Returns and clears all pending IRQ flags atomically. */
    bool GetAndClearIrq(GetAndClearIrqRsp* rsp_out);

    /** Selects the low-frequency clock source. */
    bool ConfigLfClock(LfClock lf_clock);

    /** Configures the HF/LF clock division for DIO clock-output mode. */
    bool ConfigClkOutputs(ClkScaling clk_scaling);

    /** Enables FIFO status IRQs with default threshold levels. */
    bool ConfigFifoIrq(uint8_t rx_irq_enable, uint8_t tx_irq_enable);

    /** Enables FIFO status IRQs with explicit threshold levels. */
    bool ConfigFifoIrqAdv(uint8_t rx_irq_enable, uint8_t tx_irq_enable, uint16_t rx_high_threshold,
                          uint16_t tx_low_threshold, uint16_t rx_low_threshold, uint16_t tx_high_threshold);

    /** Returns all FIFO flags that triggered since the last clear. */
    bool GetFifoIrqFlags(FifoIrqFlagsRsp* rsp_out);

    /** Returns the current RX FIFO fill level in bytes. */
    bool GetRxFifoLevel(FifoLevelRsp* rsp_out);

    /** Returns the current TX FIFO fill level in bytes. */
    bool GetTxFifoLevel(FifoLevelRsp* rsp_out);

    /** Flushes the RX FIFO. */
    bool ClearRxFifo();

    /** Flushes the TX FIFO. */
    bool ClearTxFifo();

    /** Writes len bytes from buffer to the TX FIFO. Check available space with GetTxFifoLevel(). */
    bool WriteTxFifo(const uint8_t* buffer, size_t len);

    /** Reads len bytes from the RX FIFO into buffer. Check available data with GetRxFifoLevel(). */
    bool ReadRxFifo(uint8_t* buffer, size_t len);

    /** Configures the SIMO regulator mode. */
    bool SetRegMode(SimoUsage simo_usage);

    /** Configures the SIMO regulator mode with explicit ramp times. */
    bool SetRegModeAdv(SimoUsage simo_usage, RampTimeUnit rc2ru_unit, uint8_t rc2ru, RampTimeUnit tx2ru_unit,
                       uint8_t tx2ru, RampTimeUnit ru2rc_unit, uint8_t ru2rc, RampTimeUnit ramp_down_unit,
                       uint8_t ramp_down);

    /** Runs on-chip calibration for the selected blocks.  Chip will be in Standby RC on exit. */
    bool Calibrate(bool pa_offset, bool meas_unit, bool aaf, bool pll, bool hf_rc, bool lf_rc);

    /**
     * Runs front-end calibration (ADC offset, PPF, image) at up to three frequencies.
     * If no frequency is given, frontend is calibrated for the current RF frequency, truncated to a 4MHz multiple
     * (always lower), saving calibration values in the first slot (freq1).
     * @param[in] freq1 First frequency in increments of 4MHz. MSb sets path; 0 = LF, 1 = HF.
     * @param[in] freq2 Second frequency in increments of 4MHz. MSb sets path; 0 = LF, 1 = HF. If not used, set to 0.
     * @param[in] freq3 Third frequency in increments of 4MHz. MSb sets path; 0 = LF, 1 = HF. If not used, set to 0.
     * @retval true if Device ackowledged with CMD_OK.
     */
    bool CalibFe(uint16_t freq1 = 0, uint16_t freq2 = 0, uint16_t freq3 = 0);

    /** Measures and returns the VBAT voltage. */
    bool GetVBat(VbatFormat vbat_format, AdcRes adc_res, VBatRsp* rsp_out);

    /** Measures and returns the die temperature in degrees Celsius. */
    bool GetTemp(TempSrc temp_src, AdcRes adc_res, TempRsp* rsp_out);

    /** Returns a 32-bit hardware random number. */
    bool GetRandomNumber(RandomNumberRsp* rsp_out);

    /** Returns a 32-bit hardware random number from the specified source (0–3). */
    bool GetRandomNumberAdv(uint8_t source, RandomNumberRsp* rsp_out);

    /** Puts the device into sleep mode. */
    bool SetSleep(bool clk_32k_en, uint8_t ret_en);

    /** Puts the device into sleep mode with a timed wake-up. */
    bool SetSleepAdv(bool clk_32k_en, uint8_t ret_en, uint32_t sleep_time);

    /** Puts the device into Standby mode using the selected oscillator. */
    bool SetStandby(SysStandbyMode standby_mode);

    /** Puts the device into Frequency Synthesis mode. */
    bool SetFs();

    /** Specifies an additional register to save/restore across sleep-with-retention. */
    bool SetAdditionalRegToRetain(uint8_t slot, uint32_t addr);

    /** Returns and clears the FIFO IRQ flags that triggered a FIFO IRQ. */
    bool GetAndClearFifoIrqFlags(FifoIrqFlagsRsp* rsp_out);

    /** Configures End-Of-Life voltage detection. */
    bool SetEolConfig(EolTrim eol_trim, bool enable);

    /** Configures the chip to use a TCXO and sets its supply voltage and start time. */
    bool SetTcxoMode(TcxoVoltage tcxo_voltage, uint32_t start_time);

    /** Trims the XOSC load capacitors. */
    bool SetXoscCpTrim(uint8_t xta, uint8_t xtb);

    /** Trims the XOSC load capacitors with an explicit settling delay. */
    bool SetXoscCpTrimAdv(uint8_t xta, uint8_t xtb, uint8_t delay_us);

    /** Configures temperature compensation mode for TX and RX. */
    bool SetTempCompCfg(bool ntc_en, CompMode comp_mode);

    /** Configures the NTC thermistor parameters used for temperature compensation. */
    bool SetNtcParams(uint16_t ntc_r_ratio, uint16_t ntc_beta, uint8_t delay);

    // Common firmware command enums (§5.6.3, opcodes 0x02xx)

    enum RxPath : uint8_t {
        kLfPath = 0,
        kHfPath = 1,
    };

    enum RxBoost : uint8_t {
        kBoostOff = 0,
        kBoostB1 = 1,
        kBoostB2 = 2,
        kBoostB3 = 3,
        kBoostB4 = 4,
        kBoostB5 = 5,
        kBoostB6 = 6,
        kBoostMax = 7,
    };

    enum PaSel : uint8_t {
        kLfPa = 0,
        kHfPa = 1,
    };

    enum PaLfMode : uint8_t {
        kPaLfFsm = 0,
        kPaLfFdm = 1,
        kPaLfHsmRfo1 = 2,
        kPaLfHsmRfo2 = 3,
    };

    enum RampTime : uint8_t {
        kRamp2u = 0,
        kRamp4u = 1,
        kRamp8u = 2,
        kRamp16u = 3,
        kRamp32u = 4,
        kRamp48u = 5,
        kRamp64u = 6,
        kRamp80u = 7,
        kRamp96u = 8,
        kRamp112u = 9,
        kRamp128u = 10,
        kRamp144u = 11,
        kRamp160u = 12,
        kRamp176u = 13,
        kRamp192u = 14,
        kRamp208u = 15,
    };

    enum FallbackMode : uint8_t {
        kStandbyRc = 1,
        kStandbyXosc = 2,
        kFsMode = 3,
    };

    enum PacketType : uint8_t {
        kPktLora = 0,
        kPktFskGeneric = 1,
        kPktFskLegacy = 2,
        kPktBle = 3,
        kPktRanging = 4,
        kPktFlrc = 5,
        kPktBpsk = 6,
        kPktLrFhss = 7,
        kPktWmbus = 8,
        kPktWisun = 9,
        kPktOok = 10,
        kPktRaw = 11,
        kPktZwave = 12,
        kPktZigbee = 13,
    };

    enum TxTestMode : uint8_t {
        kTestPacket = 0,
        kTestPreamble = 1,
        kTestTone = 2,
        kTestPrbs9 = 3,
    };

    enum AutoTxrxMode : uint8_t {
        kAutoTxrxDisable = 0,
        kAutoTxrxAlways = 1,
        kAutoTxrxRxOk = 2,
    };

    enum TimestampIndex : uint8_t {
        kTs0 = 0,
        kTs1 = 1,
        kTs2 = 2,
    };

    enum TimestampSource : uint8_t {
        kTsNone = 0,
        kTsTxDone = 1,
        kTsRxDone = 2,
        kTsSync = 3,
        kTsHeader = 4,
    };

    enum ExitMode : uint8_t {
        kExitFallback = 0,
        kExitTx = 1,
        kExitRx = 2,
    };

    // Common firmware response structs

    struct PacketTypeRsp {
        uint8_t packet_type;
    };

    // rssi is a 9-bit value; actual signal power = -rssi/2 (dBm).
    struct RssiInstRsp {
        uint16_t rssi;
    };

    struct RxPktLengthRsp {
        uint16_t pkt_length;
    };

    struct TimestampValueRsp {
        uint32_t timestamp;  // HF clock ticks
    };

    // All three RSSI values are 9-bit; actual power = -rssi/2 (dBm).
    struct CcaResultRsp {
        uint16_t rssi_min;
        uint16_t rssi_max;
        uint16_t rssi_avg;
    };

    // Shared modulation enums (used by Ook and FSK)

    enum OokPulseShape : uint8_t {
        kOokPulseShapeNone = 0x0,  // No pulse shaping
        kOokPulseShapeReserved1 = 0x1,
        kOokPulseShapeGaussianBT2p0 = 0x2,  // Gaussian with Bandwidth-Time bit period (BT) = 2.0
        kOokPulseShapeReserved2 = 0x3,
        kOokPulseShapeGaussianBT0p3 = 0x4,  // Gaussian with BT = 0.3
        kOokPulseShapeGaussianBT0p5 = 0x5,  // Gaussian with BT = 0.5
        kOokPulseShapeGaussianBT0p7 = 0x6,  // Gaussian with BT = 0.7
        kOokPulseShapeGaussianBT1p0 = 0x7,  // Gaussian with BT = 1.0
        // All other values reserved for future use.
    };

    enum OokRxBw : uint8_t {
        kOokRxBw3076kHz = 0,    // BW_3076:  3076.9 kHz, IF 2666.7 kHz
        kOokRxBw2857kHz = 64,   // BW_2857:  2857.1 kHz, IF 2666.7 kHz
        kOokRxBw2666kHz = 128,  // BW_2666:  2666.7 kHz, IF 2666.7 kHz
        kOokRxBw2222kHz = 192,  // BW_2222:  2222.2 kHz, IF 2666.7 kHz
        kOokRxBw1333kHz = 136,  // BW_1333:  1333.3 kHz, IF 1333.3 kHz
        kOokRxBw1111kHz = 200,  // BW_1111:  1111.1 kHz, IF 1333.3 kHz
        kOokRxBw888kHz = 144,   // BW_888:    888.9 kHz, IF  888.9 kHz
        kOokRxBw769kHz = 24,    // BW_769:    769.2 kHz, IF  666.7 kHz
        kOokRxBw740kHz = 208,   // BW_740:    740.7 kHz, IF  888.9 kHz
        kOokRxBw714kHz = 88,    // BW_714:    714.3 kHz, IF  666.7 kHz
        kOokRxBw666kHz = 152,   // BW_666:    666.7 kHz, IF  666.7 kHz
        kOokRxBw615kHz = 32,    // BW_615:    615.4 kHz, IF  533.3 kHz
        kOokRxBw571kHz = 96,    // BW_571:    571.4 kHz, IF  533.3 kHz
        kOokRxBw555kHz = 216,   // BW_555:    555.6 kHz, IF  666.7 kHz
        kOokRxBw533kHz = 160,   // BW_533:    533.3 kHz, IF  533.3 kHz
        kOokRxBw512kHz = 17,    // BW_512:    512.8 kHz, IF  444.4 kHz
        kOokRxBw476kHz = 81,    // BW_476:    476.2 kHz, IF  444.4 kHz
        kOokRxBw444kHz = 224,   // BW_444:    444.4 kHz, IF  533.3 kHz
        kOokRxBw384kHz = 25,    // BW_384:    384.6 kHz, IF  333.3 kHz
        kOokRxBw370kHz = 209,   // BW_370:    370.4 kHz, IF  444.4 kHz
        kOokRxBw357kHz = 89,    // BW_357:    357.1 kHz, IF  333.3 kHz
        kOokRxBw333kHz = 153,   // BW_333:    333.3 kHz, IF  333.3 kHz
        kOokRxBw307kHz = 33,    // BW_307:    307.7 kHz, IF  266.7 kHz
        kOokRxBw285kHz = 97,    // BW_285:    285.7 kHz, IF  266.7 kHz
        kOokRxBw277kHz = 217,   // BW_277:    277.8 kHz, IF  333.3 kHz
        kOokRxBw266kHz = 161,   // BW_266:    266.7 kHz, IF  266.7 kHz
        kOokRxBw256kHz = 18,    // BW_256:    256.4 kHz, IF  444.4 kHz
        kOokRxBw238kHz = 82,    // BW_238:    238.1 kHz, IF  444.4 kHz
        kOokRxBw222kHz = 225,   // BW_222:    222.2 kHz, IF  266.7 kHz
        kOokRxBw192kHz = 26,    // BW_192:    192.3 kHz, IF  333.3 kHz
        kOokRxBw185kHz = 210,   // BW_185:    185.2 kHz, IF  444.4 kHz
        kOokRxBw178kHz = 90,    // BW_178:    178.6 kHz, IF  333.3 kHz
        kOokRxBw166kHz = 154,   // BW_166:    166.7 kHz, IF  333.3 kHz
        kOokRxBw153kHz = 34,    // BW_153:    153.8 kHz, IF  266.7 kHz
        kOokRxBw142kHz = 98,    // BW_142:    142.9 kHz, IF  266.7 kHz
        kOokRxBw138kHz = 218,   // BW_138:    138.9 kHz, IF  333.3 kHz
        kOokRxBw133kHz = 162,   // BW_133:    133.3 kHz, IF  266.7 kHz
        kOokRxBw128kHz = 19,    // BW_128:    128.2 kHz, IF  444.4 kHz
        kOokRxBw119kHz = 83,    // BW_119:    119.0 kHz, IF  444.4 kHz
        kOokRxBw111kHz = 226,   // BW_111:    111.1 kHz, IF  266.7 kHz
        kOokRxBw96kHz = 27,     // BW_96:      96.2 kHz, IF  333.3 kHz
        kOokRxBw92kHz = 211,    // BW_92:      92.6 kHz, IF  444.4 kHz
        kOokRxBw89kHz = 91,     // BW_89:      89.3 kHz, IF  333.3 kHz
        kOokRxBw83kHz = 155,    // BW_83:      83.3 kHz, IF  333.3 kHz
        kOokRxBw76kHz = 35,     // BW_76:      76.9 kHz, IF  266.7 kHz
        kOokRxBw71kHz = 99,     // BW_71:      71.4 kHz, IF  266.7 kHz
        kOokRxBw69kHz = 219,    // BW_69:      69.4 kHz, IF  333.3 kHz
        kOokRxBw66kHz = 163,    // BW_66:      66.7 kHz, IF  266.7 kHz
        kOokRxBw64kHz = 20,     // BW_64:      64.1 kHz, IF  444.4 kHz
        kOokRxBw59kHz = 84,     // BW_59:      59.5 kHz, IF  444.4 kHz
        kOokRxBw55kHz = 227,    // BW_55:      55.6 kHz, IF  266.7 kHz
        kOokRxBw48kHz = 28,     // BW_48:      48.1 kHz, IF  333.3 kHz
        kOokRxBw46kHz = 212,    // BW_46:      46.3 kHz, IF  444.4 kHz
        kOokRxBw44kHz = 92,     // BW_44:      44.6 kHz, IF  333.3 kHz
        kOokRxBw41kHz = 156,    // BW_41:      41.7 kHz, IF  333.3 kHz
        kOokRxBw38kHz = 36,     // BW_38:      38.5 kHz, IF  266.7 kHz
        kOokRxBw35kHz = 100,    // BW_35:      35.7 kHz, IF  266.7 kHz
        kOokRxBw34kHz = 220,    // BW_34:      34.7 kHz, IF  333.3 kHz
        kOokRxBw33kHz = 164,    // BW_33:      33.3 kHz, IF  266.7 kHz
        kOokRxBw32kHz = 21,     // BW_32:      32.1 kHz, IF  444.4 kHz
        kOokRxBw29kHz = 85,     // BW_29:      29.8 kHz, IF  444.4 kHz
        kOokRxBw27kHz = 228,    // BW_27:      27.8 kHz, IF  266.7 kHz
        kOokRxBw24kHz = 29,     // BW_24:      24.0 kHz, IF  333.3 kHz
        kOokRxBw23kHz = 213,    // BW_23:      23.1 kHz, IF  444.4 kHz
        kOokRxBw22kHz = 93,     // BW_22:      22.3 kHz, IF  333.3 kHz
        kOokRxBw20kHz = 157,    // BW_20:      20.8 kHz, IF  333.3 kHz
        kOokRxBw19kHz = 37,     // BW_19:      19.2 kHz, IF  266.7 kHz
        kOokRxBw17kHz = 101,    // BW_17:      17.9 kHz, IF  266.7 kHz
        kOokRxBw16kHz = 165,    // BW_16:      16.7 kHz, IF  266.7 kHz
        kOokRxBw14kHz = 86,     // BW_14:      14.9 kHz, IF  444.4 kHz
        kOokRxBw13kHz = 229,    // BW_13:      13.9 kHz, IF  266.7 kHz
        kOokRxBw12kHz = 30,     // BW_12:      12.0 kHz, IF  333.3 kHz
        kOokRxBw11kHz = 94,     // BW_11:      11.2 kHz, IF  333.3 kHz
        kOokRxBw10kHz = 158,    // BW_10:      10.4 kHz, IF  333.3 kHz
        kOokRxBw9p6kHz = 38,    // BW_9_6:      9.6 kHz, IF  266.7 kHz
        kOokRxBw8p9kHz = 102,   // BW_8_9:      8.9 kHz, IF  266.7 kHz
        kOokRxBw8p7kHz = 222,   // BW_8_7:      8.7 kHz, IF  333.3 kHz
        kOokRxBw8p3kHz = 166,   // BW_8_3:      8.3 kHz, IF  266.7 kHz
        kOokRxBw8kHz = 23,      // BW_8:        8.0 kHz, IF  444.4 kHz
        kOokRxBw7p4kHz = 87,    // BW_7_4:      7.4 kHz, IF  444.4 kHz
        kOokRxBw6p9kHz = 230,   // BW_6_9:      6.9 kHz, IF  266.7 kHz
        kOokRxBw6kHz = 31,      // BW_6:        6.0 kHz, IF  333.3 kHz
        kOokRxBw5p8kHz = 215,   // BW_5_8:      5.8 kHz, IF  444.4 kHz
        kOokRxBw5p6kHz = 95,    // BW_5_6:      5.6 kHz, IF  333.3 kHz
        kOokRxBw5p2kHz = 159,   // BW_5_2:      5.2 kHz, IF  333.3 kHz
        kOokRxBw4p8kHz = 39,    // BW_4_8:      4.8 kHz, IF  266.7 kHz
        kOokRxBw4p5kHz = 103,   // BW_4_5:      4.5 kHz, IF  266.7 kHz
        kOokRxBw4p3kHz = 223,   // BW_4_3:      4.3 kHz, IF  333.3 kHz
        kOokRxBw4p2kHz = 167,   // BW_4_2:      4.2 kHz, IF  266.7 kHz
        kOokRxBw3p5kHz = 231,   // BW_3_5:      3.5 kHz, IF  266.7 kHz
    };

    // Ook-specific enums

    enum OokDepth : uint8_t {
        kOokDepthFull = 0,
        kOokDepthMax20Db = 1,
    };

    enum OokAddrComp : uint8_t {
        kOokAddrCompOff = 0,
        kOokAddrCompNode = 1,
        kOokAddrCompNodeBcast = 2,
    };

    enum OokPktFormat : uint8_t {
        kOokPktFormatFixedLength = 0,
        kOokPktFormatVariable8bit = 1,
    };

    enum OokCrc : uint8_t {
        kOokCrcOff = 0,
        kOokCrc1Byte = 1,
        kOokCrc2Byte = 2,
        kOokCrc3Byte = 3,
        kOokCrc4Byte = 4,
        kOokCrc1ByteInv = 9,
        kOokCrc2ByteInv = 10,
        kOokCrc3ByteInv = 11,
        kOokCrc4ByteInv = 12,
    };

    enum OokEncoding : uint8_t {
        kOokEncodingNone = 0,
        kOokEncodingManchester = 1,
        kOokEncodingManchesterInv = 9,
        kOokEncodingBiphaseMark = 2,
        kOokEncodingBiphaseMarkInv = 10,
    };

    enum OokBitOrder : uint8_t {
        kOokBitOrderLsbFirst = 0,
        kOokBitOrderMsbFirst = 1,
    };

    enum OokSfdKind : uint8_t {
        kOokSfdKindFallingEdge = 0,
        kOokSfdKindRisingEdge = 1,
    };

    // Ook response structs

    // Response from GetOokRxStats: [0:1]=stat, [2:3]=pkt_rx, [4:5]=crc_error, [6:7]=len_error
    struct OokRxStats {
        uint16_t pkt_rx;
        uint16_t crc_error;
        uint16_t len_error;
    };

    // Response from GetOokRxStatsAdv (16-byte frame 2): extends OokRxStats
    struct OokRxStatsAdv {
        uint16_t pkt_rx;
        uint16_t crc_error;
        uint16_t len_error;
        uint16_t pbl_det;
        uint16_t sync_ok;
        uint16_t sync_fail;
        uint16_t timeout;
    };

    // Response from GetOokPacketStatus.
    // rssi_avg and rssi_high are 9-bit values reconstructed from the raw frame.
    struct OokPacketStatus {
        uint16_t pkt_len;
        uint16_t rssi_avg;   // actual power = -rssi_avg / 2 (dBm)
        uint16_t rssi_high;  // actual power = -rssi_high / 2 (dBm)
        bool addr_match_bcast;
        bool addr_match_node;
        uint8_t lqi;
    };

    // Register / Memory Access (§5.6.2)

    /**
     * Writes consecutive 32-bit words to the device register/memory space.
     *
     * Data is sent big-endian (MSByte first) per the LR2021 SPI protocol.
     *
     * @param[in] addr       Word-aligned destination address (bits 23:0 used).
     * @param[in] data       Pointer to the words to write.
     * @param[in] num_words  Number of words to write; must be in [1, kMaxBlockWords].
     * @retval true   Device acknowledged with CMD_OK.
     * @retval false  Transfer failed or device returned an error status.
     */
    bool WriteRegMem32(uint32_t addr, const uint32_t* data, size_t num_words);

    /**
     * Reads consecutive 32-bit words from the device register/memory space.
     *
     * Executes a two-frame SPI sequence: the first frame sends the opcode and
     * address; the second (after BUSY deasserts) clocks out the data.
     *
     * @param[in]  addr       Word-aligned source address (bits 23:0 used).
     * @param[out] data       Buffer to receive the read words; must hold at least num_words elements.
     * @param[in]  num_words  Number of words to read; must be in [1, kMaxBlockWords].
     * @retval true   Device responded with CMD_DAT (or CMD_OK) and data is populated.
     * @retval false  Transfer failed or device returned an error status.
     */
    bool ReadRegMem32(uint32_t addr, uint32_t* data, size_t num_words);

    /**
     * Performs an atomic read-modify-write on the single 32-bit word at addr (§6.2.2).
     *
     * Only the bits set in mask are updated with the corresponding bits from data;
     * all other bits are left unchanged by the device firmware.
     *
     * @param[in] addr  Word-aligned target address (bits 23:0 used).
     * @param[in] mask  Bitmask selecting which bits to update.
     * @param[in] data  New values for the bits selected by mask.
     * @retval true   Device acknowledged with CMD_OK.
     * @retval false  Transfer failed or device returned an error status.
     */
    bool WriteRegMemMask32(uint32_t addr, uint32_t mask, uint32_t data);

    // Common Firmware Commands (§5.6.3, opcodes 0x0200–0x021C)

    /** Sets the RF frequency for subsequent radio operations. */
    bool SetRfFrequency(uint32_t rf_freq);

    /** Sets the RX path (LF or HF). */
    bool SetRxPath(RxPath rx_path);

    /** Sets the RX path and boost configuration. Re-runs ADC offset calibration when boost changes. */
    bool SetRxPathAdv(RxPath rx_path, RxBoost rx_boost);

    /** Selects which PA to use and sets its LF parameters. */
    bool SetPaConfig(PaSel pa_sel, PaLfMode pa_lf_mode, uint8_t pa_lf_duty_cycle, uint8_t pa_lf_slices);

    /** As SetPaConfig with an additional HF duty-cycle parameter. */
    bool SetPaConfigAdv(PaSel pa_sel, PaLfMode pa_lf_mode, uint8_t pa_lf_duty_cycle, uint8_t pa_lf_slices,
                        uint8_t pa_hf_duty_cycle);

    /** Sets the TX output power and PA ramp time. */
    bool SetTxParams(int8_t tx_power, RampTime ramp_time);

    /** Configures the fallback mode after a RX or TX operation. */
    bool SetRxTxFallbackMode(FallbackMode fallback_mode);

    /** Sets the current packet type. Must be called first when configuring the transceiver. */
    bool SetPacketType(PacketType packet_type);

    /**
     * Returns the current packet type.
     * @param[out] rsp_out  Must not be null.
     */
    bool GetPacketType(PacketTypeRsp* rsp_out);

    /** Defines whether the RX timeout stops on Syncword/Header or on Preamble detection. */
    bool SetStopTimeout(bool stop_on_preamble);

    /** Resets RX statistics counters. */
    bool ResetRxStats();

    /**
     * Returns the instantaneous RSSI value during packet reception.
     * @param[out] rsp_out  Must not be null.
     */
    bool GetRssiInst(RssiInstRsp* rsp_out);

    /** Sets the device into RX mode using the default timeout. */
    bool SetRx();

    /** Sets the device into RX mode with a timeout (1/32.768 kHz steps, max 512 s). */
    bool SetRxAdv(uint32_t rx_timeout);

    /** Sets the device into TX mode using the default timeout. */
    bool SetTx();

    /** Sets the device into TX mode with a timeout (1/32.768 kHz steps, max 512 s). */
    bool SetTxAdv(uint32_t tx_timeout);

    /** Sets the device into a TX test mode. */
    bool SetTxTestMode(TxTestMode test_mode);

    /** Selects which PA to use. Configuration must have been set with SetPaConfig beforehand. */
    bool SelPa(PaSel pa_sel);

#ifdef HARDWARE_UNIT_TESTS
    /**
     * Starts an unmodulated continuous-wave (CW) tone for bench testing. Configures the PA for the
     * requested front end, tunes the synthesizer, and keys up a continuous carrier that stays on until
     * StopCwTone() is called. Debug builds only.
     * @param use_hf_path True to use the HF (2.4 GHz) path/PA, false to use the LF (sub-GHz) path/PA.
     * @param freq_hz Carrier frequency in Hz.
     * @param tx_power_dbm Requested TX power in dBm (interpreted relative to the PA configuration).
     * @retval True if the carrier was started successfully, false otherwise.
     */
    bool StartCwTone(bool use_hf_path, uint32_t freq_hz, int8_t tx_power_dbm);

    /** Stops a CW tone started with StartCwTone() by returning the chip to standby. Debug builds only. */
    bool StopCwTone();

    /**
     * Puts the chip into continuous RX at an arbitrary frequency so instantaneous RSSI can be polled
     * with GetRssiInst(). Replaces the normal receiver configuration; restore it afterwards by
     * re-running the full receiver bring-up (ADSBee::Init()). Debug builds only.
     * @param use_hf_path True to use the HF (2.4 GHz) RX path, false to use the LF (sub-GHz) path.
     * @param freq_hz RX frequency in Hz.
     * @retval True if the chip entered RX successfully, false otherwise.
     */
    bool StartRssiScan(bool use_hf_path, uint32_t freq_hz);
#endif

    /** Starts periodic duty-cycled RX. */
    bool SetRxDutyCycle(uint32_t rx_max_time, uint32_t cycle_time, bool use_lora_cad, uint8_t dram_ret);

    /** Activates or deactivates the auto TX/RX mode. */
    bool SetAutoRxTx(bool clear, AutoTxrxMode auto_txrx_mode, uint32_t timeout, uint32_t delay);

    /**
     * Returns the length of the last received packet.
     * @param[out] rsp_out  Must not be null.
     */
    bool GetRxPktLength(RxPktLengthRsp* rsp_out);

    /** Sets the global power offset value. */
    bool SetPowerOffset(uint8_t power_offset);

    /** Sets the default RX and TX timeouts used for DIO triggers. */
    bool SetDefaultRxTxTimeout(uint32_t rx_timeout, uint32_t tx_timeout);

    /** Configures the event source for one of the three hardware timestamps. */
    bool SetTimestampSource(TimestampIndex timestamp_index, TimestampSource timestamp_source);

    /**
     * Returns the delay between a timestamped event and the NSS falling edge.
     * @param[out] rsp_out  Must not be null.
     */
    bool GetTimestampValue(TimestampIndex timestamp_index, TimestampValueRsp* rsp_out);

    /** Sets the device into CCA (Clear Channel Assessment) mode for the given duration. */
    bool SetCca(uint32_t duration);

    /** As SetCca with an explicit AGC gain override. */
    bool SetCcaAdv(uint32_t duration, uint8_t gain);

    /**
     * Returns RSSI statistics from the last CCA measurement.
     * @param[out] rsp_out  Must not be null.
     */
    bool GetCcaResult(CcaResultRsp* rsp_out);

    /** Sets the AGC gain manually (0 = auto). */
    bool SetAgcGainManual(uint8_t gain_step);

    /** Sets the CAD parameters for RSSI-based Channel Activity Detection. */
    bool SetCadParams(uint32_t cad_timeout, uint8_t threshold, ExitMode exit_mode, uint32_t trx_timeout);

    /** Sets the device into CAD mode. Parameters must be configured first with SetCadParams. */
    bool SetCad();

    // Ook Firmware Commands (§5.6.3, opcodes 0x02xx)

    /**
     * Sets the LR2021 to ADS-B (Mode S) reception mode.
     * @param[in] preamble_mode  How reception is triggered (standard preamble vs. DF17 header).
     * @param[in] agc_gain       Manual LF-frontend AGC gain step (0 = auto, 1..15; 13 = max).
     * @param[in] rx_boost       LF RX path boost level (0 = off .. 7 = max), clamped to kBoostMax.
     */
    bool SetOokADSB(SettingsManager::R1090PreambleMode preamble_mode, uint8_t agc_gain, uint8_t rx_boost = 0);

    /**
     * Sets the Ook modulation parameters.
     */
    bool SetOokModulationParams(uint32_t bitrate, OokPulseShape pulse_shape, OokRxBw rx_bw);

    /**
     * Sets the Ook modulation parameters with advanced ook_depth control.
     */
    bool SetOokModulationParamsAdv(uint32_t bitrate, OokPulseShape pulse_shape, OokRxBw rx_bw, OokDepth ook_depth);

    /**
     * Sets the Ook packet parameters.
     */
    bool SetOokPacketParams(uint16_t pre_len_tx, OokAddrComp addr_comp, OokPktFormat pkt_format, uint16_t pld_len,
                            OokCrc crc, OokEncoding encoding);

    /**
     * Sets the Ook CRC polynomial and initial value.
     * @param[in] polynom    CRC polynomial used to generate the CRC info.
     * @param[in] init       Initial value loaded into the CRC engine at the start of each packet.
     * @retval true   Device acknowledged with CMD_OK.
     * @retval false  Transfer failed or device returned an error status.
     */
    bool SetOokCrcParams(uint32_t polynom, uint32_t init);

    /**
     * Sets the Ook syncword (up to 32 bits).
     * @param[in] syncword    Syncword value, aligned to the LSB.
     * @param[in] bit_order   Bit order of the syncword on the air.
     * @param[in] nb_bits     Number of bits in the syncword.
     * @retval true   Device acknowledged with CMD_OK.
     * @retval false  Transfer failed or device returned an error status.
     */
    bool SetOokSyncWord(uint32_t syncword, OokBitOrder bit_order, uint8_t nb_bits);

    /**
     * Sets the Ook node and broadcast addresses.
     */
    bool SetOokAddress(uint8_t addr_node, uint8_t addr_bcast);

    /**
     * Reads and returns accumulated Ook RX statistics.
     *
     * @param[out] stats_out  Must not be null.
     */
    bool GetOokRxStats(OokRxStats* stats_out);

    /**
     * Reads and returns the advanced accumulated Ook RX statistics (adds detector fires, sync
     * ok/fail, and timeout counters to the basic stats).
     *
     * @param[out] stats_out  Must not be null.
     */
    bool GetOokRxStatsAdv(OokRxStatsAdv* stats_out);

    /**
     * Reads and returns the status of the last received Ook packet.
     *
     * @param[out] status_out  Must not be null.
     */
    bool GetOokPacketStatus(OokPacketStatus* status_out);

    /**
     * Configures the Ook preamble/SFD detector (RX only).
     * @param[in] preamble_pattern       Bit pattern to detect in the preamble, aligned to the LSB.
     * @param[in] pattern_length         Number of bits in the preamble pattern (must be ≤16).
     * @param[in] pattern_num_repeats    Number of times the preamble pattern is repeated. Controls timeouts for sync
     * word search.
     * @param[in] sw_is_raw              0: Syncword is encoded same way as the payload. 1: Syncword is not encoded.
     * @param[in] sfd_kind               Whether to trigger SFD on a falling or rising edge after the preamble.
     * @param[in] sfd_length             Number of bits in the SFD.
     * @retval true   Device acknowledged with CMD_OK.
     * @retval false  Transfer failed or device returned an error status.
     */
    bool SetOokDetector(uint16_t preamble_pattern, uint8_t pattern_length, uint8_t pattern_num_repeats, bool sw_is_raw,
                        OokSfdKind sfd_kind, uint8_t sfd_length);

    /**
     * Configures Ook whitening. A polynom of 0 disables whitening.
     */
    bool SetOokWhiteningParams(uint8_t bit_idx, uint16_t polynom, uint16_t init);

    // Status inspection

    /**
     * Returns the Stat word captured during the most recent SPI transaction.
     * Updated after every frame (including the second frame of a read command).
     *
     * @retval Stat  Reference to the last decoded status word.
     */
    const Stat& last_stat() const { return last_stat_; }

   private:
    friend class LR2021TestAccessor;

    // System firmware command opcodes (§5.6.1, 0x01xx group).
    static constexpr uint16_t kOpcodeGetStatus = 0x0100;
    static constexpr uint16_t kOpcodeGetVersion = 0x0101;
    static constexpr uint16_t kOpcodeGetErrors = 0x0110;
    static constexpr uint16_t kOpcodeClearErrors = 0x0111;
    static constexpr uint16_t kOpcodeSetDioFunction = 0x0112;
    static constexpr uint16_t kOpcodeSetDioRfSwitchConfig = 0x0113;
    static constexpr uint16_t kOpcodeReadRxFifo = 0x0001;
    static constexpr uint16_t kOpcodeWriteTxFifo = 0x0002;
    static constexpr uint16_t kOpcodeClearFifoIrqFlags = 0x0114;
    static constexpr uint16_t kOpcodeSetDioIrqConfig = 0x0115;
    static constexpr uint16_t kOpcodeClearIrq = 0x0116;
    static constexpr uint16_t kOpcodeGetAndClearIrq = 0x0117;
    static constexpr uint16_t kOpcodeConfigLfClock = 0x0118;
    static constexpr uint16_t kOpcodeConfigClkOutputs = 0x0119;
    static constexpr uint16_t kOpcodeConfigFifoIrq = 0x011A;
    static constexpr uint16_t kOpcodeGetFifoIrqFlags = 0x011B;
    static constexpr uint16_t kOpcodeGetRxFifoLevel = 0x011C;
    static constexpr uint16_t kOpcodeGetTxFifoLevel = 0x011D;
    static constexpr uint16_t kOpcodeClearRxFifo = 0x011E;
    static constexpr uint16_t kOpcodeClearTxFifo = 0x011F;
    static constexpr uint16_t kOpcodeSetTcxoMode = 0x0120;
    static constexpr uint16_t kOpcodeSetRegMode = 0x0121;
    static constexpr uint16_t kOpcodeCalibrate = 0x0122;
    static constexpr uint16_t kOpcodeCalibFe = 0x0123;
    static constexpr uint16_t kOpcodeGetVBat = 0x0124;
    static constexpr uint16_t kOpcodeGetTemp = 0x0125;
    static constexpr uint16_t kOpcodeGetRandomNumber = 0x0126;
    static constexpr uint16_t kOpcodeSetSleep = 0x0127;
    static constexpr uint16_t kOpcodeSetStandby = 0x0128;
    static constexpr uint16_t kOpcodeSetFs = 0x0129;
    static constexpr uint16_t kOpcodeSetAdditionalRegToRetain = 0x012A;
    static constexpr uint16_t kOpcodeGetAndClearFifoIrqFlags = 0x012E;
    static constexpr uint16_t kOpcodeSetEolConfig = 0x0130;
    static constexpr uint16_t kOpcodeSetXoscCpTrim = 0x0131;
    static constexpr uint16_t kOpcodeSetTempCompCfg = 0x0132;
    static constexpr uint16_t kOpcodeSetNtcParams = 0x0133;

    // SPI opcodes (16-bit, §5.6.2 / Table 5-2).
    static constexpr uint16_t kOpcodeWriteRegMem32 = 0x0104;
    static constexpr uint16_t kOpcodeWriteRegMemMask32 = 0x0105;
    static constexpr uint16_t kOpcodeReadRegMem32 = 0x0106;

    // Common firmware command opcodes (§5.6.3, 0x02xx group).
    static constexpr uint16_t kOpcodeSetRfFrequency = 0x0200;
    static constexpr uint16_t kOpcodeSetRxPath = 0x0201;
    static constexpr uint16_t kOpcodeSetPaConfig = 0x0202;
    static constexpr uint16_t kOpcodeSetTxParams = 0x0203;
    static constexpr uint16_t kOpcodeSetRxTxFallbackMode = 0x0206;
    static constexpr uint16_t kOpcodeSetPacketType = 0x0207;
    static constexpr uint16_t kOpcodeGetPacketType = 0x0208;
    static constexpr uint16_t kOpcodeSetStopTimeout = 0x0209;
    static constexpr uint16_t kOpcodeResetRxStats = 0x020A;
    static constexpr uint16_t kOpcodeGetRssiInst = 0x020B;
    static constexpr uint16_t kOpcodeSetRx = 0x020C;
    static constexpr uint16_t kOpcodeSetTx = 0x020D;
    static constexpr uint16_t kOpcodeSetTxTestMode = 0x020E;
    static constexpr uint16_t kOpcodeSelPa = 0x020F;
    static constexpr uint16_t kOpcodeSetRxDutyCycle = 0x0210;
    static constexpr uint16_t kOpcodeSetAutoRxTx = 0x0211;
    static constexpr uint16_t kOpcodeGetRxPktLength = 0x0212;
    static constexpr uint16_t kOpcodeSetPowerOffset = 0x0214;
    static constexpr uint16_t kOpcodeSetDefaultRxTxTimeout = 0x0215;
    static constexpr uint16_t kOpcodeSetTimestampSource = 0x0216;
    static constexpr uint16_t kOpcodeGetTimestampValue = 0x0217;
    static constexpr uint16_t kOpcodeSetCca = 0x0218;
    static constexpr uint16_t kOpcodeGetCcaResult = 0x0219;
    static constexpr uint16_t kOpcodeSetAgcGainManual = 0x021A;
    static constexpr uint16_t kOpcodeSetCadParams = 0x021B;
    static constexpr uint16_t kOpcodeSetCad = 0x021C;

    // Ook firmware command opcodes (§5.6.3, 0x02xx group).
    static constexpr uint16_t kOpcodeSetOokModulationParams = 0x0281;  // basic (8 B) and adv (9 B) share this opcode
    static constexpr uint16_t kOpcodeSetOokPacketParams = 0x0282;
    static constexpr uint16_t kOpcodeSetOokCrcParams = 0x0283;
    static constexpr uint16_t kOpcodeSetOokSyncWord = 0x0284;
    static constexpr uint16_t kOpcodeSetOokAddress = 0x0285;
    static constexpr uint16_t kOpcodeGetOokRxStats = 0x0286;
    static constexpr uint16_t kOpcodeGetOokPacketStatus = 0x0287;
    static constexpr uint16_t kOpcodeSetOokDetector = 0x0288;
    static constexpr uint16_t kOpcodeSetOokWhiteningParams = 0x0289;

    /**
     * Polls BUSY until it deasserts.
     *
     * @param[in] timeout_ms  Maximum time to wait in milliseconds (default: kBusyTimeoutMs).
     * @retval true   BUSY deasserted within the timeout.
     * @retval false  Timeout expired while BUSY remained asserted.
     */
    bool WaitUntilReady(uint32_t timeout_ms = kBusyTimeoutMs);

    /**
     * Waits until ready, then drives NSS low to begin a SPI frame.
     *
     * @retval true   NSS asserted; device is ready.
     * @retval false  Timeout waiting for BUSY to deassert.
     */
    bool BeginTransaction();

    /** Drives NSS high, ending the current SPI frame. */
    void EndTransaction();

    /**
     * Decodes a raw 16-bit Stat word into last_stat_.
     *
     * @param[in] raw  Raw 16-bit status word as received from the device.
     */
    void ParseStat(uint16_t raw);

    // SPI Commands
    bool SPITransfer(const uint8_t* tx_buf, uint8_t* rx_buf, size_t length);
    void SetNSS(bool high);
    bool IsBusy();
    void DelayUs(uint32_t us);

    void SetEnable(bool enabled);

    // Big-endian bit-packing helpers used by all command implementations.
    static inline void PackU16(uint8_t* buf, uint16_t value) {
        buf[0] = static_cast<uint8_t>(value >> 8);
        buf[1] = static_cast<uint8_t>(value & 0xFF);
    }
    static inline void PackU24(uint8_t* buf, uint32_t value) {
        buf[0] = static_cast<uint8_t>((value >> 16) & 0xFF);
        buf[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
        buf[2] = static_cast<uint8_t>(value & 0xFF);
    }
    static inline void PackU32(uint8_t* buf, uint32_t value) {
        buf[0] = static_cast<uint8_t>(value >> 24);
        buf[1] = static_cast<uint8_t>(value >> 16);
        buf[2] = static_cast<uint8_t>(value >> 8);
        buf[3] = static_cast<uint8_t>(value);
    }
    static inline uint16_t UnpackU16(const uint8_t* buf) {
        return static_cast<uint16_t>((static_cast<uint16_t>(buf[0]) << 8) | buf[1]);
    }
    static inline uint32_t UnpackU32(const uint8_t* buf) {
        return (static_cast<uint32_t>(buf[0]) << 24) | (static_cast<uint32_t>(buf[1]) << 16) |
               (static_cast<uint32_t>(buf[2]) << 8) | static_cast<uint32_t>(buf[3]);
    }

    // Async RX drain internals (lr2021_async.cpp thread-paced loop drain; lr2021_irq_drain.cpp
    // IRQ-paced chain).

    enum class DrainState : uint8_t {
        kIdle = 0,
        // Thread-paced loop drain (advanced by ServiceRxDrain): unconditional level-read sweep, then a
        // conditional IRQ-latch clear tail whenever the LR_IRQ line reads high.
        kLevelCmdWait,   // Waiting for BUSY low to post the GetRxFifoLevel opcode frame.
        kLevelCmd,       // GetRxFifoLevel frame 1 (2 B) DMA in flight.
        kLevelRspWait,   // Waiting for BUSY low to post the GetRxFifoLevel response frame.
        kLevelRsp,       // GetRxFifoLevel frame 2 (4 B) DMA in flight.
        kFifoWait,       // Waiting for BUSY low to post the ReadRxFifo frame.
        kFifoRead,       // ReadRxFifo single frame (2 + level B) DMA in flight.
        kClearFifoWait,     // (Tail) waiting for BUSY low to post the GetAndClearFifoIrqFlags opcode frame.
        kClearFifoCmd,      // (Tail) GetAndClearFifoIrqFlags frame 1 (2 B) DMA in flight.
        kClearFifoRspWait,  // (Tail) waiting for BUSY low to post the GetAndClearFifoIrqFlags response frame.
        kClearFifoRsp,      // (Tail) GetAndClearFifoIrqFlags frame 2 (4 B) DMA in flight; overflow check.
        kIrqCmdWait,        // (Tail) waiting for BUSY low to post the GetAndClearIrq opcode frame.
        kIrqCmd,            // (Tail) GetAndClearIrq frame 1 (2 B) DMA in flight.
        kIrqRspWait,        // (Tail) waiting for BUSY low to post the GetAndClearIrq response frame.
        kIrqRsp,            // (Tail) GetAndClearIrq frame 2 (6 B) DMA in flight.
        kDataReady,         // Terminal until FinishRxDrain().
        kError,             // Terminal until FinishRxDrain().
        // IRQ-paced chain (advanced from the GPIO HWIs and the SPI completion SWI; the thread only
        // backstops). Must stay the LAST block: IsChainState() tests >= kChainFifoWait.
        kChainFifoWait,          // Waiting (BUSY edge / thread assist) to post ReadRxFifo(threshold).
        kChainFifoRead,          // ReadRxFifo (2 + 126 B) DMA into a staging slot in flight.
        kChainClearFifoWait,     // Waiting to post the GetAndClearFifoIrqFlags opcode frame.
        kChainClearFifo,         // GetAndClearFifoIrqFlags frame 1 (2 B) in flight.
        kChainClearFifoRspWait,  // Waiting to post the GetAndClearFifoIrqFlags response frame.
        kChainClearFifoRsp,      // GetAndClearFifoIrqFlags frame 2 (4 B) in flight; overflow check.
        kChainIrqCmdWait,        // Waiting to post GetAndClearIrq opcode frame.
        kChainIrqCmd,            // GetAndClearIrq frame 1 (2 B) in flight.
        kChainIrqRspWait,        // Waiting to post GetAndClearIrq response frame.
        kChainIrqRsp,            // GetAndClearIrq frame 2 (6 B) in flight; completion decides restart/finish.
    };
    static constexpr bool IsChainState(DrainState s) { return s >= DrainState::kChainFifoWait; }

    // SPI completion callback, registered on the (callback-mode) SPI handle for ALL transfers. Runs in
    // SWI context; NoRTOS SWIs are non-preemptive alongside the UAT RF SWI's ~15 us deadline, so this
    // must stay to a few flag/GPIO writes plus (for chain frames) posting the next DMA frame. No
    // logging, no busy-waiting, no protocol parsing beyond single-byte inspections. Finds its LR2021
    // instance via SPI_Transaction::arg.
    static void SPICallback(SPI_Handle handle, SPI_Transaction* transaction);
    // Posts one NSS frame of a drain as a single DMA transaction. tx_buf may be nullptr (clocks the
    // driver's 0x00 default); rx_buf nullptr targets async_rx_buf_. Asserts NSS; SPICallback deasserts
    // it on completion. HWI/SWI/thread-safe (SPI_transfer in callback mode is documented HWI-safe).
    bool PostAsyncFrame(const uint8_t* tx_buf, size_t len, uint8_t* rx_buf = nullptr);
    // Enters a BUSY-wait drain state and arms its timeout.
    void EnterBusyWait(DrainState state);
    // Enters kError with a reason string (logged by the consumer at thread level).
    DrainResult EnterDrainError(const char* reason);
    // Called from a DMA-in-flight state while async_done_ is still false. Normally kInProgress; if the
    // completion is overdue (lost SPI callback) it cancels the transfer and enters kError so the drain
    // can restart instead of wedging forever.
    DrainResult AsyncStillInFlight();
    // True when the LR2021's IRQ output (kIrqRxFifo routed to DIO6 -> LR_IRQ pin) reads asserted.
    bool IrqLineHigh();

    // IRQ chain helpers (lr2021_irq_drain.cpp).
    // Posts the frame for the current kChain*Wait state and advances to its in-flight state. Caller
    // guarantees atomicity (GPIO HWI, or HwiP_disable from SWI/thread) and that the chip is ready.
    bool PostChainFrame();
    // Chain-frame completion handling, called from SPICallback for chain in-flight states (SWI).
    void HandleChainFrameComplete();
    // Starts a chain segment into the current fill slot: posts immediately if BUSY is low, else arms
    // the BUSY falling-edge interrupt. Caller has claimed drain_state_ and stamped the slot.
    void StartChainSegment();
    // Aborts the chain: disarms the BUSY interrupt, returns the drain to kIdle. ISR/SWI-safe.
    void AbortChainFromIsr();
    // Arms (clearInt + enableInt) / disarms the LR_BUSY falling-edge interrupt.
    void ArmBusyInterrupt();
    void DisarmBusyInterrupt();

    LR2021Config config_;
    ChipMode mode_ = ChipMode::kStartup;
    SPI_Params spi_params_;
    SPI_Handle spi_handle_ = nullptr;
    // Set from the SYNC ISR via RequestAbort(); checked in WaitUntilReady(); cleared in Init().
    volatile bool abort_requested_ = false;
    Stat last_stat_;

    // Async RX drain state. The tx/rx buffers and transaction must be persistent: DMA reads/writes them
    // after ServiceRxDrain() returns. One extra 2-byte slot ahead of the payload holds the ReadRxFifo
    // opcode (TX) / Stat word (RX). volatile: mutated from GPIO HWI / SPI SWI (chain) and thread.
    volatile DrainState drain_state_ = DrainState::kIdle;
    SPI_Transaction async_txn_ = {};
    uint8_t async_tx_buf_[2 + kRxFifoMaxDepthBytes] = {};  // Opcode at [0:1]; [2..] stays zero (FIFO clocking).
    uint8_t async_rx_buf_[2 + kRxFifoMaxDepthBytes] = {};  // Stat at [0:1]; response/payload at [2..].
    // Loop-drain tail command buffers: the tail runs AFTER the FIFO payload has landed in
    // async_rx_buf_, so its frames must not touch the async buffers (would clobber the unparsed
    // payload / break the async_tx_buf_ "[2..] zeros" invariant).
    uint8_t tail_tx_buf_[4] = {};
    uint8_t tail_rx_buf_[8] = {};
    volatile bool async_ok_ = false;         // Transfer status; written by SPICallback before async_done_.
    volatile bool async_done_ = false;       // Set last by SPICallback; cleared when posting a frame.
    volatile bool async_in_flight_ = false;  // True from post until SPICallback (or cancel).
    uint32_t busy_wait_start_ms_ = 0;
    uint32_t drain_irq_flags_ = 0;
    uint16_t drain_fifo_level_ = 0;
    bool drain_fifo_full_ = false;
    const char* drain_error_str_ = nullptr;

    // Synchronous shim state (see SPITransfer() in lr2021_ll.cpp).
    volatile bool sync_done_ = false;   // Completion flag for the in-progress synchronous transfer.
    volatile int32_t sync_status_ = 0;  // SPI_Status of the completed synchronous transfer.

    // IRQ-paced chain state (lr2021_irq_drain.cpp). Chain frames use dedicated buffers -- never the
    // async_* pair (see the tail buffer comment above). TX contents are static, pre-packed in Init().
    IrqDrainSlot irq_slots_[kNumIrqDrainSlots];
    volatile uint8_t irq_slot_fill_idx_ = 0;   // Producer (ISR/SWI) side of the slot ring.
    uint8_t irq_slot_parse_idx_ = 0;           // Consumer (thread) side of the slot ring.
    uint8_t chain_fifo_tx_buf_[2 + kIrqDrainThresholdBytes] = {};  // ReadRxFifo opcode + zeros.
    uint8_t chain_clear_tx_buf_[4] = {};                           // GetAndClearFifoIrqFlags opcode.
    uint8_t chain_irq_tx_buf_[2] = {};                             // GetAndClearIrq opcode.
    uint8_t chain_rx_scratch_[8] = {};                             // RX for the non-payload chain frames.
    volatile uint32_t chain_start_ms_ = 0;  // Chain segment start, for the wedge-timeout backstop.
};

extern LR2021 lr2021;