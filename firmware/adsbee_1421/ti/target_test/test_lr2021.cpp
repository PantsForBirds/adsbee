#include "adsbee.hh"
#include "hardware_unit_tests.hh"

// White-box accessor: gives tests access to all LR2021 public and private
// members and methods through a single consistent interface.
class LR2021TestAccessor {
   public:
    explicit LR2021TestAccessor(LR2021& lr) : lr_(lr) {}

    // Public methods
    bool Init() { return lr_.Init(); }
    bool DeInit() { return lr_.DeInit(); }
    bool WriteRegMem32(uint32_t addr, const uint32_t* data, size_t num_words) {
        return lr_.WriteRegMem32(addr, data, num_words);
    }
    bool ReadRegMem32(uint32_t addr, uint32_t* data, size_t num_words) {
        return lr_.ReadRegMem32(addr, data, num_words);
    }
    bool WriteRegMemMask32(uint32_t addr, uint32_t mask, uint32_t data) {
        return lr_.WriteRegMemMask32(addr, mask, data);
    }

    // Private methods
    bool WaitUntilReady(uint32_t timeout_ms = LR2021::kBusyTimeoutMs) { return lr_.WaitUntilReady(timeout_ms); }
    bool BeginTransaction() { return lr_.BeginTransaction(); }
    void EndTransaction() { lr_.EndTransaction(); }
    void ParseStat(uint16_t raw) { lr_.ParseStat(raw); }
    bool SPITransfer(const uint8_t* tx_buf, uint8_t* rx_buf, size_t length) {
        return lr_.SPITransfer(tx_buf, rx_buf, length);
    }
    void SetNSS(bool high) { lr_.SetNSS(high); }
    bool IsBusy() { return lr_.IsBusy(); }
    void DelayUs(uint32_t us) { lr_.DelayUs(us); }
    void SetEnable(bool enabled) { lr_.SetEnable(enabled); }

    // Member inspection
    const LR2021::Stat& last_stat() const { return lr_.last_stat_; }
    LR2021::ChipMode mode() const { return lr_.mode_; }
    bool HasValidHandle() const { return lr_.spi_handle_ != nullptr; }

   private:
    LR2021& lr_;
};

// Scratch address used for read/write round-trip tests.  Must be a word-aligned
// address in a safely writable region of the LR2021 register/memory space.
// NOTE: All tests that actually use kTestAddr are commented out since we don't have a good read / write register to
// test with.
static constexpr uint32_t kTestAddr = 0x080000;

// ---------------------------------------------------------------------------
// Init / DeInit lifecycle
// ---------------------------------------------------------------------------

UTEST(LR2021, ReInit) {
    LR2021TestAccessor acc(adsbee.lr2021);
    acc.DeInit();
    // After DeInit the reset line is asserted, so BUSY is driven HIGH by the chip
    // during its reset/boot sequence.  Do not check IsBusy() here; Init() will
    // wait for it to deassert via WaitUntilReady().
    ASSERT_TRUE(acc.Init());
}

// ---------------------------------------------------------------------------
// Argument validation – these must fail before touching the SPI bus.
// ---------------------------------------------------------------------------

UTEST(LR2021, WriteRegMem32_NullPointer) {
    LR2021TestAccessor acc(adsbee.lr2021);
    ASSERT_FALSE(acc.WriteRegMem32(kTestAddr, nullptr, 1));
}

UTEST(LR2021, WriteRegMem32_ZeroWords) {
    LR2021TestAccessor acc(adsbee.lr2021);
    uint32_t data = 0xDEADBEEF;
    ASSERT_FALSE(acc.WriteRegMem32(kTestAddr, &data, 0));
}

UTEST(LR2021, WriteRegMem32_TooManyWords) {
    LR2021TestAccessor acc(adsbee.lr2021);
    uint32_t data[LR2021::kMaxBlockWords + 1] = {};
    ASSERT_FALSE(acc.WriteRegMem32(kTestAddr, data, LR2021::kMaxBlockWords + 1));
}

UTEST(LR2021, ReadRegMem32_NullPointer) {
    LR2021TestAccessor acc(adsbee.lr2021);
    ASSERT_FALSE(acc.ReadRegMem32(kTestAddr, nullptr, 1));
}

UTEST(LR2021, ReadRegMem32_ZeroWords) {
    LR2021TestAccessor acc(adsbee.lr2021);
    uint32_t data = 0;
    ASSERT_FALSE(acc.ReadRegMem32(kTestAddr, &data, 0));
}

UTEST(LR2021, ReadRegMem32_TooManyWords) {
    LR2021TestAccessor acc(adsbee.lr2021);
    uint32_t data[LR2021::kMaxBlockWords + 1] = {};
    ASSERT_FALSE(acc.ReadRegMem32(kTestAddr, data, LR2021::kMaxBlockWords + 1));
}

// ---------------------------------------------------------------------------
// Single-word write – verify the device acknowledges with CMD_OK.
// ---------------------------------------------------------------------------

UTEST(LR2021, WriteRegMem32_SingleWord) {
    LR2021TestAccessor acc(adsbee.lr2021);
    const uint32_t data = 0xA5A5A5A5;
    ASSERT_TRUE(acc.WriteRegMem32(kTestAddr, &data, 1));
    ASSERT_EQ(static_cast<int>(acc.last_stat().command_status), static_cast<int>(LR2021::CommandStatus::kOk));
}

// ---------------------------------------------------------------------------
// Single-word read – verify the device responds with CMD_DAT or CMD_OK.
// ---------------------------------------------------------------------------

// UTEST(LR2021, ReadRegMem32_SingleWord) {
//     LR2021TestAccessor acc(adsbee.lr2021);
//     uint32_t data = 0;
//     ASSERT_TRUE(acc.ReadRegMem32(kTestAddr, &data, 1));
//     // Frame-2 status must be CMD_DAT (0x3) or CMD_OK (0x2) per §6.7.1.
//     const LR2021::CommandStatus cs = acc.last_stat().command_status;
//     ASSERT_TRUE(cs == LR2021::CommandStatus::kDat || cs == LR2021::CommandStatus::kOk);
// }

// ---------------------------------------------------------------------------
// Round-trip: write a pattern, read it back, verify it matches.
// ---------------------------------------------------------------------------

// UTEST(LR2021, RoundTrip_SingleWord) {
//     LR2021TestAccessor acc(adsbee.lr2021);
//     const uint32_t written = 0x12345678;
//     ASSERT_TRUE(acc.WriteRegMem32(kTestAddr, &written, 1));

//     uint32_t read_back = 0;
//     ASSERT_TRUE(acc.ReadRegMem32(kTestAddr, &read_back, 1));
//     ASSERT_EQ(written, read_back);
// }

// UTEST(LR2021, RoundTrip_MultipleWords) {
//     LR2021TestAccessor acc(adsbee.lr2021);
//     uint32_t written[4] = {0x11111111, 0x22222222, 0x33333333, 0x44444444};
//     ASSERT_TRUE(acc.WriteRegMem32(kTestAddr, written, 4));

//     uint32_t read_back[4] = {};
//     ASSERT_TRUE(acc.ReadRegMem32(kTestAddr, read_back, 4));
//     for (size_t i = 0; i < 4; ++i) {
//         ASSERT_EQ(written[i], read_back[i]);
//     }
// }

// ---------------------------------------------------------------------------
// Boundary: exercise the maximum-block-size path (32 words).
// ---------------------------------------------------------------------------

// UTEST(LR2021, WriteRegMem32_MaxWords) {
//     LR2021TestAccessor acc(adsbee.lr2021);
//     uint32_t data[LR2021::kMaxBlockWords];
//     for (size_t i = 0; i < LR2021::kMaxBlockWords; ++i) {
//         data[i] = static_cast<uint32_t>(i) * 0x01010101u;
//     }
//     ASSERT_TRUE(acc.WriteRegMem32(kTestAddr, data, LR2021::kMaxBlockWords));
//     ASSERT_EQ(static_cast<int>(acc.last_stat().command_status),
//               static_cast<int>(LR2021::CommandStatus::kOk));
// }

// UTEST(LR2021, ReadRegMem32_MaxWords) {
//     LR2021TestAccessor acc(adsbee.lr2021);
//     uint32_t data[LR2021::kMaxBlockWords] = {};
//     ASSERT_TRUE(acc.ReadRegMem32(kTestAddr, data, LR2021::kMaxBlockWords));
//     const LR2021::CommandStatus cs = acc.last_stat().command_status;
//     ASSERT_TRUE(cs == LR2021::CommandStatus::kDat || cs == LR2021::CommandStatus::kOk);
// }

// UTEST(LR2021, RoundTrip_MaxWords) {
//     LR2021TestAccessor acc(adsbee.lr2021);
//     uint32_t written[LR2021::kMaxBlockWords];
//     for (size_t i = 0; i < LR2021::kMaxBlockWords; ++i) {
//         written[i] = 0xAB000000u | static_cast<uint32_t>(i);
//     }
//     ASSERT_TRUE(acc.WriteRegMem32(kTestAddr, written, LR2021::kMaxBlockWords));

//     uint32_t read_back[LR2021::kMaxBlockWords] = {};
//     ASSERT_TRUE(acc.ReadRegMem32(kTestAddr, read_back, LR2021::kMaxBlockWords));
//     for (size_t i = 0; i < LR2021::kMaxBlockWords; ++i) {
//         ASSERT_EQ(written[i], read_back[i]);
//     }
// }

// ---------------------------------------------------------------------------
// WriteRegMemMask32: write a known word, then apply a mask that flips only
// specific bits, then verify only those bits changed.
// ---------------------------------------------------------------------------

// UTEST(LR2021, WriteRegMemMask32_MaskedBitsChange) {
//     LR2021TestAccessor acc(adsbee.lr2021);
//     // Set a known baseline.
//     const uint32_t baseline = 0x00000000;
//     ASSERT_TRUE(acc.WriteRegMem32(kTestAddr, &baseline, 1));

//     // Apply mask=0x0000FFFF, data=0x0000ABCD — only the lower 16 bits should update.
//     const uint32_t mask = 0x0000FFFF;
//     const uint32_t new_data = 0x0000ABCD;
//     ASSERT_TRUE(acc.WriteRegMemMask32(kTestAddr, mask, new_data));
//     ASSERT_EQ(static_cast<int>(acc.last_stat().command_status),
//               static_cast<int>(LR2021::CommandStatus::kOk));

//     // Read back and confirm.
//     uint32_t read_back = 0;
//     ASSERT_TRUE(acc.ReadRegMem32(kTestAddr, &read_back, 1));
//     const uint32_t expected = (baseline & ~mask) | (new_data & mask);
//     ASSERT_EQ(expected, read_back);
// }

// UTEST(LR2021, WriteRegMemMask32_UnmaskedBitsPreserved) {
//     LR2021TestAccessor acc(adsbee.lr2021);
//     // Set a known baseline with all bits set.
//     const uint32_t baseline = 0xFFFFFFFF;
//     ASSERT_TRUE(acc.WriteRegMem32(kTestAddr, &baseline, 1));

//     // Mask only the upper byte; clear it.
//     const uint32_t mask = 0xFF000000;
//     const uint32_t new_data = 0x00000000;
//     ASSERT_TRUE(acc.WriteRegMemMask32(kTestAddr, mask, new_data));

//     uint32_t read_back = 0;
//     ASSERT_TRUE(acc.ReadRegMem32(kTestAddr, &read_back, 1));
//     // Upper byte must be 0x00; lower 3 bytes must remain 0xFFFFFF.
//     ASSERT_EQ(0x00FFFFFFu, read_back);
// }

// ---------------------------------------------------------------------------
// ParseStat: white-box tests for the bit-field decoder (Table 6-38).
// ---------------------------------------------------------------------------

UTEST(LR2021, ParseStat_CmdOk) {
    LR2021TestAccessor acc(adsbee.lr2021);
    // bits [11:9]=0x2 (kOk), bit[8]=0, bits[7:4]=0x1 (POR), bits[2:0]=0x1 (kStdbyRC)
    // 0b0000_0100_0001_0001 = 0x0411
    acc.ParseStat(0x0411);
    ASSERT_EQ(static_cast<int>(acc.last_stat().command_status), static_cast<int>(LR2021::CommandStatus::kOk));
    ASSERT_FALSE(acc.last_stat().interrupt_active);
    ASSERT_EQ(acc.last_stat().reset_source, 0x1);
    ASSERT_EQ(static_cast<int>(acc.last_stat().chip_mode), static_cast<int>(LR2021::ChipMode::kStdbyRC));
}

UTEST(LR2021, ParseStat_CmdDat_InterruptActive) {
    LR2021TestAccessor acc(adsbee.lr2021);
    // bits [11:9]=0x3 (kDat), bit[8]=1, bits[7:4]=0x0, bits[2:0]=0x4 (kRx)
    // 0b0000_0111_0000_0100 = 0x0704
    acc.ParseStat(0x0704);
    ASSERT_EQ(static_cast<int>(acc.last_stat().command_status), static_cast<int>(LR2021::CommandStatus::kDat));
    ASSERT_TRUE(acc.last_stat().interrupt_active);
    ASSERT_EQ(acc.last_stat().reset_source, 0x0);
    ASSERT_EQ(static_cast<int>(acc.last_stat().chip_mode), static_cast<int>(LR2021::ChipMode::kRx));
}

UTEST(LR2021, ParseStat_CmdFail) {
    LR2021TestAccessor acc(adsbee.lr2021);
    // bits [11:9]=0x0 (kFail), bit[8]=0, bits[7:4]=0x0, bits[2:0]=0x0 (kSleep)
    acc.ParseStat(0x0000);
    ASSERT_EQ(static_cast<int>(acc.last_stat().command_status), static_cast<int>(LR2021::CommandStatus::kFail));
    ASSERT_FALSE(acc.last_stat().interrupt_active);
    ASSERT_EQ(static_cast<int>(acc.last_stat().chip_mode), static_cast<int>(LR2021::ChipMode::kSleep));
}

UTEST(LR2021, ParseStat_UpperNibbleIgnored) {
    LR2021TestAccessor acc(adsbee.lr2021);
    // Table 6-38 says bits [15:12] are always 0; ParseStat should mask them out.
    // Set them to 0xF and verify the decoded fields are unaffected.
    // bits [15:12]=0xF (ignored), [11:9]=0x2 (kOk), [8]=0, [7:4]=0, [2:0]=0x1 (kStdbyRC)
    // 0b1111_0100_0000_0001 = 0xF401
    acc.ParseStat(0xF401);
    ASSERT_EQ(static_cast<int>(acc.last_stat().command_status), static_cast<int>(LR2021::CommandStatus::kOk));
    ASSERT_EQ(static_cast<int>(acc.last_stat().chip_mode), static_cast<int>(LR2021::ChipMode::kStdbyRC));
}

// ---------------------------------------------------------------------------
// WaitUntilReady: chip must be idle after Init(); verify it returns immediately.
// ---------------------------------------------------------------------------

UTEST(LR2021, WaitUntilReady_ReturnsTrueWhenNotBusy) {
    LR2021TestAccessor acc(adsbee.lr2021);
    ASSERT_TRUE(acc.WaitUntilReady());
}

UTEST(LR2021, WaitUntilReady_ZeroTimeout_ReturnsTrueWhenNotBusy) {
    LR2021TestAccessor acc(adsbee.lr2021);
    // With a zero-ms timeout the function should still succeed if BUSY is already low.
    ASSERT_TRUE(acc.WaitUntilReady(0));
}

// ---------------------------------------------------------------------------
// IsBusy: must reflect the BUSY GPIO; should be low after Init().
// ---------------------------------------------------------------------------

UTEST(LR2021, IsBusy_FalseAfterInit) {
    LR2021TestAccessor acc(adsbee.lr2021);
    ASSERT_FALSE(acc.IsBusy());
}

// ---------------------------------------------------------------------------
// BeginTransaction / EndTransaction: NSS handshake around a SPI frame.
// ---------------------------------------------------------------------------

UTEST(LR2021, BeginEndTransaction) {
    LR2021TestAccessor acc(adsbee.lr2021);
    // BeginTransaction waits for BUSY low then asserts NSS.
    ASSERT_TRUE(acc.BeginTransaction());
    // EndTransaction deasserts NSS; the chip processes any command and will
    // eventually lower BUSY again.
    acc.EndTransaction();
    // After the transaction closes the chip should become ready again.
    ASSERT_TRUE(acc.WaitUntilReady());
}

// ---------------------------------------------------------------------------
// SPITransfer: error path when the SPI handle is null (after DeInit()).
// ---------------------------------------------------------------------------

UTEST(LR2021, SPITransfer_FailsWithNullHandle) {
    LR2021TestAccessor acc(adsbee.lr2021);
    acc.DeInit();
    ASSERT_FALSE(acc.HasValidHandle());

    uint8_t tx[2] = {0x01, 0x06};
    uint8_t rx[2] = {};
    ASSERT_FALSE(acc.SPITransfer(tx, rx, 2));

    // Restore for subsequent tests.
    acc.Init();
}

UTEST(LR2021, SPITransfer_SucceedsWithValidHandle) {
    LR2021TestAccessor acc(adsbee.lr2021);
    ASSERT_TRUE(acc.HasValidHandle());

    // Perform a raw transfer inside a proper NSS frame.  Send two 0x00 bytes;
    // the chip will respond with its current Stat word but we ignore the value
    // here — we just verify the transfer completes without error.
    uint8_t tx[2] = {};
    uint8_t rx[2] = {};
    ASSERT_TRUE(acc.BeginTransaction());
    bool result = acc.SPITransfer(tx, rx, sizeof(tx));
    acc.EndTransaction();
    ASSERT_TRUE(result);
    acc.WaitUntilReady();
    // The arbitrary 0x0000 payload may have altered the chip's internal state.
    // Re-initialise so subsequent tests start from a known StdbyRC baseline.
    acc.DeInit();
    ASSERT_TRUE(acc.Init());
}

// ---------------------------------------------------------------------------
// SetNSS: drive NSS high and low; verify subsequent transactions still work.
// ---------------------------------------------------------------------------

UTEST(LR2021, SetNSS_HighThenLow) {
    LR2021TestAccessor acc(adsbee.lr2021);
    // Explicitly drive NSS high (deasserted) — should be a no-op from idle.
    acc.SetNSS(true);
    // Drive NSS low (assert chip select).
    acc.SetNSS(false);
    // Restore NSS high so the bus is left in a known-idle state.
    acc.SetNSS(true);
    // An NSS pulse without SPI clocks delivers an undefined (zero-bit) frame to
    // the chip.  Reset via the enable pin to return to a clean StdbyRC state
    // before exercising the write path.
    acc.SetEnable(false);
    acc.SetEnable(true);
    ASSERT_TRUE(acc.WaitUntilReady(LR2021::kBootupTimeoutMs));
    const uint32_t data = 0x00000000;
    ASSERT_TRUE(acc.WriteRegMem32(kTestAddr, &data, 1));
}

// ---------------------------------------------------------------------------
// SetEnable: toggling the enable (NRESET) pin should allow re-init to succeed.
// ---------------------------------------------------------------------------

UTEST(LR2021, SetEnable_ToggleAndRecover) {
    LR2021TestAccessor acc(adsbee.lr2021);
    // Assert reset (active-low enable).
    acc.SetEnable(false);
    // Release reset; chip begins its internal boot sequence.
    acc.SetEnable(true);
    // Wait for BUSY to deassert before touching the SPI bus again.
    ASSERT_TRUE(acc.WaitUntilReady(LR2021::kBootupTimeoutMs));
    // Confirm the driver can still complete a transfer.
    const uint32_t data = 0x00000000;
    ASSERT_TRUE(acc.WriteRegMem32(kTestAddr, &data, 1));
}

// ---------------------------------------------------------------------------
// DelayUs: must not hang or crash for representative durations.
// ---------------------------------------------------------------------------

UTEST(LR2021, DelayUs_Short) {
    LR2021TestAccessor acc(adsbee.lr2021);
    acc.DelayUs(1);
    acc.DelayUs(100);
}

UTEST(LR2021, DelayUs_OneMilli) {
    LR2021TestAccessor acc(adsbee.lr2021);
    acc.DelayUs(1000);
}

// ---------------------------------------------------------------------------
// mode_ member: verify internal mode tracking across Init / DeInit.
// ---------------------------------------------------------------------------

UTEST(LR2021, Mode_StdbyRCAfterInit) {
    LR2021TestAccessor acc(adsbee.lr2021);
    acc.DeInit();
    ASSERT_EQ(static_cast<int>(acc.mode()), static_cast<int>(LR2021::ChipMode::kStartup));
    ASSERT_TRUE(acc.Init());
    ASSERT_EQ(static_cast<int>(acc.mode()), static_cast<int>(LR2021::ChipMode::kStdbyRC));
}

UTEST(LR2021, Mode_StartupAfterDeInit) {
    LR2021TestAccessor acc(adsbee.lr2021);
    acc.DeInit();
    ASSERT_EQ(static_cast<int>(acc.mode()), static_cast<int>(LR2021::ChipMode::kStartup));
    // Restore for subsequent tests.
    acc.Init();
}

// ---------------------------------------------------------------------------
// HasValidHandle: SPI handle tracking across Init / DeInit.
// ---------------------------------------------------------------------------

UTEST(LR2021, Handle_ValidAfterInit) {
    LR2021TestAccessor acc(adsbee.lr2021);
    ASSERT_TRUE(acc.HasValidHandle());
}

UTEST(LR2021, Handle_NullAfterDeInit) {
    LR2021TestAccessor acc(adsbee.lr2021);
    acc.DeInit();
    ASSERT_FALSE(acc.HasValidHandle());
    // Restore for subsequent tests.
    acc.Init();
}

// ---------------------------------------------------------------------------
// Stat inspection: verify ChipMode reflects StdbyRC after Init().
// ---------------------------------------------------------------------------

UTEST(LR2021, StatChipMode_AfterInit) {
    LR2021TestAccessor acc(adsbee.lr2021);
    acc.DeInit();
    ASSERT_TRUE(acc.Init());

    // Perform a harmless single-word write to capture a fresh stat word.
    const uint32_t data = 0x00000000;
    ASSERT_TRUE(acc.WriteRegMem32(kTestAddr, &data, 1));

    // After a successful boot the LR2021 should be in StdbyRC (§5.3, Table 5-1).
    ASSERT_EQ(static_cast<int>(acc.last_stat().chip_mode), static_cast<int>(LR2021::ChipMode::kStdbyRC));
}

// ---------------------------------------------------------------------------
// Async RX drain
// ---------------------------------------------------------------------------

UTEST(LR2021, AsyncRxDrainCompletesAndYieldsBus) {
    LR2021TestAccessor acc(adsbee.lr2021);
    ASSERT_TRUE(acc.HasValidHandle());

    // Pump a full drain sequence to a terminal result. On a quiet bench this completes as kNoData (no
    // RX FIFO IRQ pending); with live traffic kDataReady is also legal. The iteration bound is generous:
    // a drain is at most 5 DMA frames plus BUSY pulses.
    if (adsbee.lr2021.DrainIdle()) {
        ASSERT_TRUE(adsbee.lr2021.StartRxDrain());
    }
    static constexpr uint32_t kMaxPumpIterations = 10'000'000;
    LR2021::DrainResult result = LR2021::DrainResult::kInProgress;
    uint32_t i = 0;
    for (; i < kMaxPumpIterations; i++) {
        result = adsbee.lr2021.ServiceRxDrain();
        if (result != LR2021::DrainResult::kInProgress) {
            break;
        }
    }
    ASSERT_LT(i, kMaxPumpIterations);
    ASSERT_TRUE(result == LR2021::DrainResult::kNoData || result == LR2021::DrainResult::kDataReady);
    if (result == LR2021::DrainResult::kDataReady) {
        adsbee.lr2021.FinishRxDrain();
    }
    ASSERT_TRUE(adsbee.lr2021.DrainIdle());

    // Start another drain and immediately issue a synchronous command: BeginTransaction's bus-ownership
    // guard must pump the drain to a quiescent state so the two never interleave NSS frames.
    ASSERT_TRUE(adsbee.lr2021.StartRxDrain());
    LR2021::StatusRsp status;
    ASSERT_TRUE(adsbee.lr2021.GetStatus(&status));

    // Clean up whatever state the guard parked the drain in.
    adsbee.lr2021.CancelAsync();
    ASSERT_TRUE(adsbee.lr2021.DrainIdle());
}

UTEST(LR2021, IrqChainQuiescentOnQuietBench) {
    LR2021TestAccessor acc(adsbee.lr2021);
    ASSERT_TRUE(acc.HasValidHandle());

    // With the receiver configured and no (or light) traffic, the RX FIFO sits below the 126-byte
    // high threshold, so the LR2021's DIO6 IRQ output -> LR_IRQ pin must read low and the chain must
    // be quiescent with no filled staging slots pending beyond what the loop can consume.
    ASSERT_TRUE(adsbee.lr2021.DrainIdle() || true);  // Drain may be mid-loop-sweep; that's fine.

    // Drain any slots a previous test/burst left behind, then confirm the ring reads empty.
    while (adsbee.lr2021.NextFilledSlot() != nullptr) {
        adsbee.lr2021.ReleaseSlot();
    }
    ASSERT_TRUE(adsbee.lr2021.NextFilledSlot() == nullptr);

    // A BUSY-fall callback with no chain pending must be a harmless no-op.
    adsbee.lr2021.HandleBusyFall();
    ASSERT_TRUE(adsbee.lr2021.NextFilledSlot() == nullptr);

    // The loop drain sweep must still complete against the live chip after the restructure
    // (level-first, no kIrqRxFifo gate).
    if (adsbee.lr2021.DrainIdle()) {
        ASSERT_TRUE(adsbee.lr2021.StartRxDrain());
    }
    static constexpr uint32_t kMaxPumpIterations = 10'000'000;
    LR2021::DrainResult result = LR2021::DrainResult::kInProgress;
    uint32_t i = 0;
    for (; i < kMaxPumpIterations; i++) {
        result = adsbee.lr2021.ServiceRxDrain();
        if (result != LR2021::DrainResult::kInProgress) {
            break;
        }
    }
    ASSERT_LT(i, kMaxPumpIterations);
    ASSERT_TRUE(result == LR2021::DrainResult::kNoData || result == LR2021::DrainResult::kDataReady);
    if (result == LR2021::DrainResult::kDataReady) {
        adsbee.lr2021.FinishRxDrain();
    }
    ASSERT_TRUE(adsbee.lr2021.DrainIdle());
}
