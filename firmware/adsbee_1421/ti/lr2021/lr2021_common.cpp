// lr2021_common.cpp
//
// Semtech LR2021 driver – common firmware command implementation.
// See lr2021.hh for API documentation and datasheet references.
//
// All commands in this file use the 0x02xx opcode group (firmware commands).
// SPI framing follows the same rules as lr2021_ook.cpp:
//
//   Write command (single frame):
//     MOSI: Op[15:8] Op[7:0] Arg0 Arg1 ...
//     MISO: Stat[15:8] Stat[7:0] (remaining bytes ignored)
//
//   Read command (two frames):
//     Frame 1 – send opcode + optional arguments (BUSY asserts then deasserts)
//     Frame 2 – send 0x00 bytes; device clocks out Stat + data
//               Note: 0x02xx reads do NOT append trailing IrqStatus bytes.

#include <cstring>

#include "comms.hh"
#include "lr2021.hh"

// ---------------------------------------------------------------------------
// Write commands (single SPI frame)
// ---------------------------------------------------------------------------

/**
 * SetRfFrequency (opcode 0x0200, 6 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x00
 *   2   | rf_freq[31:24]
 *   3   | rf_freq[23:16]
 *   4   | rf_freq[15:8]
 *   5   | rf_freq[7:0]
 */
bool LR2021::SetRfFrequency(uint32_t rf_freq) {
    constexpr size_t kFrameLen = 6;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetRfFrequency);
    PackU32(tx_buf + 2, rf_freq);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetRfFrequency", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetRfFrequency", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetRxPath (opcode 0x0201, 3 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x01
 *   2   | rx_path & 0x1
 */
bool LR2021::SetRxPath(RxPath rx_path) {
    constexpr size_t kFrameLen = 3;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetRxPath);
    tx_buf[2] = static_cast<uint8_t>(rx_path) & 0x1;

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetRxPath", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetRxPath", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetRxPathAdv (opcode 0x0201, 4 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x01
 *   2   | rx_path & 0x1
 *   3   | rx_boost & 0x7
 */
bool LR2021::SetRxPathAdv(RxPath rx_path, RxBoost rx_boost) {
    constexpr size_t kFrameLen = 4;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetRxPath);
    tx_buf[2] = static_cast<uint8_t>(rx_path) & 0x1;
    tx_buf[3] = static_cast<uint8_t>(rx_boost) & 0x7;

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetRxPathAdv", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetRxPathAdv", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetPaConfig (opcode 0x0202, 4 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x02
 *   2   | (pa_sel & 0x1) << 7 | (pa_lf_duty_cycle & 0xF) << 4 | pa_lf_mode & 0x3
 *   3   | pa_lf_slices & 0xF
 */
bool LR2021::SetPaConfig(PaSel pa_sel, PaLfMode pa_lf_mode, uint8_t pa_lf_duty_cycle, uint8_t pa_lf_slices) {
    constexpr size_t kFrameLen = 4;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetPaConfig);
    tx_buf[2] = static_cast<uint8_t>((static_cast<uint8_t>(pa_sel) & 0x1) << 7) |
                static_cast<uint8_t>((pa_lf_duty_cycle & 0xF) << 4) |
                static_cast<uint8_t>(static_cast<uint8_t>(pa_lf_mode) & 0x3);
    tx_buf[3] = pa_lf_slices & 0xF;

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetPaConfig", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetPaConfig", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetPaConfigAdv (opcode 0x0202, 5 bytes)
 *
 * Identical to SetPaConfig with one extra byte appended:
 *   4   | pa_hf_duty_cycle & 0x1F
 */
bool LR2021::SetPaConfigAdv(PaSel pa_sel, PaLfMode pa_lf_mode, uint8_t pa_lf_duty_cycle, uint8_t pa_lf_slices,
                            uint8_t pa_hf_duty_cycle) {
    constexpr size_t kFrameLen = 5;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetPaConfig);
    tx_buf[2] = static_cast<uint8_t>((static_cast<uint8_t>(pa_sel) & 0x1) << 7) |
                static_cast<uint8_t>((pa_lf_duty_cycle & 0xF) << 4) |
                static_cast<uint8_t>(static_cast<uint8_t>(pa_lf_mode) & 0x3);
    tx_buf[3] = pa_lf_slices & 0xF;
    tx_buf[4] = pa_hf_duty_cycle & 0x1F;

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetPaConfigAdv", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetPaConfigAdv", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetTxParams (opcode 0x0203, 4 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x03
 *   2   | tx_power (signed, cast to uint8_t)
 *   3   | ramp_time
 */
bool LR2021::SetTxParams(int8_t tx_power, RampTime ramp_time) {
    constexpr size_t kFrameLen = 4;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetTxParams);
    tx_buf[2] = static_cast<uint8_t>(tx_power);
    tx_buf[3] = static_cast<uint8_t>(ramp_time);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetTxParams", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetTxParams", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetRxTxFallbackMode (opcode 0x0206, 3 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x06
 *   2   | fallback_mode & 0x3
 */
bool LR2021::SetRxTxFallbackMode(FallbackMode fallback_mode) {
    constexpr size_t kFrameLen = 3;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetRxTxFallbackMode);
    tx_buf[2] = static_cast<uint8_t>(fallback_mode) & 0x3;

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetRxTxFallbackMode", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetRxTxFallbackMode", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetPacketType (opcode 0x0207, 3 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x07
 *   2   | packet_type
 */
bool LR2021::SetPacketType(PacketType packet_type) {
    constexpr size_t kFrameLen = 3;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetPacketType);
    tx_buf[2] = static_cast<uint8_t>(packet_type);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetPacketType", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetPacketType", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetStopTimeout (opcode 0x0209, 3 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x09
 *   2   | stop_on_preamble ? 1 : 0
 */
bool LR2021::SetStopTimeout(bool stop_on_preamble) {
    constexpr size_t kFrameLen = 3;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetStopTimeout);
    tx_buf[2] = stop_on_preamble ? 0x1 : 0x0;

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetStopTimeout", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetStopTimeout", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * ResetRxStats (opcode 0x020A, 2 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x0A
 */
bool LR2021::ResetRxStats() {
    constexpr size_t kFrameLen = 2;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    PackU16(tx_buf, kOpcodeResetRxStats);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::ResetRxStats", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::ResetRxStats", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetRx (opcode 0x020C, 2 bytes) – uses default timeout
 */
bool LR2021::SetRx() {
    constexpr size_t kFrameLen = 2;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    PackU16(tx_buf, kOpcodeSetRx);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetRx", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetRx", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetRxAdv (opcode 0x020C, 5 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x0C
 *   2   | rx_timeout[23:16]
 *   3   | rx_timeout[15:8]
 *   4   | rx_timeout[7:0]
 */
bool LR2021::SetRxAdv(uint32_t rx_timeout) {
    constexpr size_t kFrameLen = 5;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetRx);
    PackU24(tx_buf + 2, rx_timeout);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetRxAdv", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetRxAdv", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetTx (opcode 0x020D, 2 bytes) – uses default timeout
 */
bool LR2021::SetTx() {
    constexpr size_t kFrameLen = 2;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    PackU16(tx_buf, kOpcodeSetTx);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetTx", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetTx", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetTxAdv (opcode 0x020D, 5 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x0D
 *   2   | tx_timeout[23:16]
 *   3   | tx_timeout[15:8]
 *   4   | tx_timeout[7:0]
 */
bool LR2021::SetTxAdv(uint32_t tx_timeout) {
    constexpr size_t kFrameLen = 5;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetTx);
    PackU24(tx_buf + 2, tx_timeout);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetTxAdv", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetTxAdv", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetTxTestMode (opcode 0x020E, 3 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x0E
 *   2   | test_mode
 */
bool LR2021::SetTxTestMode(TxTestMode test_mode) {
    constexpr size_t kFrameLen = 3;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetTxTestMode);
    tx_buf[2] = static_cast<uint8_t>(test_mode);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetTxTestMode", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetTxTestMode", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SelPa (opcode 0x020F, 3 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x0F
 *   2   | pa_sel & 0x1
 */
bool LR2021::SelPa(PaSel pa_sel) {
    constexpr size_t kFrameLen = 3;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSelPa);
    tx_buf[2] = static_cast<uint8_t>(pa_sel) & 0x1;

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SelPa", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SelPa", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetRxDutyCycle (opcode 0x0210, 9 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x10
 *   2   | rx_max_time[23:16]
 *   3   | rx_max_time[15:8]
 *   4   | rx_max_time[7:0]
 *   5   | cycle_time[23:16]
 *   6   | cycle_time[15:8]
 *   7   | cycle_time[7:0]
 *   8   | (use_lora_cad ? 16 : 0) | dram_ret & 0x7
 */
bool LR2021::SetRxDutyCycle(uint32_t rx_max_time, uint32_t cycle_time, bool use_lora_cad, uint8_t dram_ret) {
    constexpr size_t kFrameLen = 9;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetRxDutyCycle);
    PackU24(tx_buf + 2, rx_max_time);
    PackU24(tx_buf + 5, cycle_time);
    tx_buf[8] = static_cast<uint8_t>(use_lora_cad ? 0x10u : 0x00u) | (dram_ret & 0x7);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetRxDutyCycle", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetRxDutyCycle", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetAutoRxTx (opcode 0x0211, 10 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x11
 *   2   | (clear ? 128 : 0) | auto_txrx_mode & 0x3
 *   3   | timeout[23:16]
 *   4   | timeout[15:8]
 *   5   | timeout[7:0]
 *   6   | delay[31:24]
 *   7   | delay[23:16]
 *   8   | delay[15:8]
 *   9   | delay[7:0]
 */
bool LR2021::SetAutoRxTx(bool clear, AutoTxrxMode auto_txrx_mode, uint32_t timeout, uint32_t delay) {
    constexpr size_t kFrameLen = 10;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetAutoRxTx);
    tx_buf[2] = static_cast<uint8_t>(clear ? 0x80u : 0x00u) |
                (static_cast<uint8_t>(auto_txrx_mode) & 0x3);
    PackU24(tx_buf + 3, timeout);
    PackU32(tx_buf + 6, delay);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetAutoRxTx", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetAutoRxTx", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetPowerOffset (opcode 0x0214, 3 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x14
 *   2   | power_offset & 0x3F
 */
bool LR2021::SetPowerOffset(uint8_t power_offset) {
    constexpr size_t kFrameLen = 3;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetPowerOffset);
    tx_buf[2] = power_offset & 0x3F;

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetPowerOffset", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetPowerOffset", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetDefaultRxTxTimeout (opcode 0x0215, 8 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x15
 *   2   | rx_timeout[23:16]
 *   3   | rx_timeout[15:8]
 *   4   | rx_timeout[7:0]
 *   5   | tx_timeout[23:16]
 *   6   | tx_timeout[15:8]
 *   7   | tx_timeout[7:0]
 */
bool LR2021::SetDefaultRxTxTimeout(uint32_t rx_timeout, uint32_t tx_timeout) {
    constexpr size_t kFrameLen = 8;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetDefaultRxTxTimeout);
    PackU24(tx_buf + 2, rx_timeout);
    PackU24(tx_buf + 5, tx_timeout);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetDefaultRxTxTimeout", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetDefaultRxTxTimeout", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetTimestampSource (opcode 0x0216, 3 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x16
 *   2   | (timestamp_index & 0x3) << 4 | timestamp_source & 0xF
 */
bool LR2021::SetTimestampSource(TimestampIndex timestamp_index, TimestampSource timestamp_source) {
    constexpr size_t kFrameLen = 3;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetTimestampSource);
    tx_buf[2] = static_cast<uint8_t>((static_cast<uint8_t>(timestamp_index) & 0x3) << 4) |
                (static_cast<uint8_t>(timestamp_source) & 0xF);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetTimestampSource", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetTimestampSource", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetCca (opcode 0x0218, 5 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x18
 *   2   | duration[23:16]
 *   3   | duration[15:8]
 *   4   | duration[7:0]
 */
bool LR2021::SetCca(uint32_t duration) {
    constexpr size_t kFrameLen = 5;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetCca);
    PackU24(tx_buf + 2, duration);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetCca", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetCca", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetCcaAdv (opcode 0x0218, 6 bytes)
 *
 * Identical to SetCca with one extra byte appended:
 *   5   | gain
 */
bool LR2021::SetCcaAdv(uint32_t duration, uint8_t gain) {
    constexpr size_t kFrameLen = 6;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetCca);
    PackU24(tx_buf + 2, duration);
    tx_buf[5] = gain;

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetCcaAdv", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetCcaAdv", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetAgcGainManual (opcode 0x021A, 3 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x1A
 *   2   | gain_step & 0xF
 */
bool LR2021::SetAgcGainManual(uint8_t gain_step) {
    constexpr size_t kFrameLen = 3;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetAgcGainManual);
    tx_buf[2] = gain_step & 0xF;

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetAgcGainManual", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetAgcGainManual", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetCadParams (opcode 0x021B, 10 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x1B
 *   2   | cad_timeout[23:16]
 *   3   | cad_timeout[15:8]
 *   4   | cad_timeout[7:0]
 *   5   | threshold
 *   6   | exit_mode & 0x3
 *   7   | trx_timeout[23:16]
 *   8   | trx_timeout[15:8]
 *   9   | trx_timeout[7:0]
 */
bool LR2021::SetCadParams(uint32_t cad_timeout, uint8_t threshold, ExitMode exit_mode, uint32_t trx_timeout) {
    constexpr size_t kFrameLen = 10;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetCadParams);
    PackU24(tx_buf + 2, cad_timeout);
    tx_buf[5] = threshold;
    tx_buf[6] = static_cast<uint8_t>(exit_mode) & 0x3;
    PackU24(tx_buf + 7, trx_timeout);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetCadParams", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetCadParams", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetCad (opcode 0x021C, 2 bytes)
 */
bool LR2021::SetCad() {
    constexpr size_t kFrameLen = 2;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    PackU16(tx_buf, kOpcodeSetCad);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetCad", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetCad", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

// ---------------------------------------------------------------------------
// Read commands (two-frame SPI sequence)
//
// Frame 1: send opcode (+ optional args); chip raises then lowers BUSY.
// Frame 2: send zeros; chip clocks out Stat (2 bytes) + data bytes.
//          0x02xx reads do NOT append trailing IrqStatus bytes.
// ---------------------------------------------------------------------------

/**
 * GetPacketType (opcode 0x0208)
 *
 * Frame 2 layout (3 bytes):
 *   [0:1]  Stat
 *   [2]    packet_type
 */
bool LR2021::GetPacketType(PacketTypeRsp* rsp_out) {
    if (rsp_out == nullptr) {
        CONSOLE_ERROR("LR2021::GetPacketType", "rsp_out is null.");
        return false;
    }

    // Frame 1
    {
        constexpr size_t kFrame1Len = 2;
        uint8_t tx_buf[kFrame1Len];
        uint8_t rx_buf[kFrame1Len];
        PackU16(tx_buf, kOpcodeGetPacketType);

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetPacketType", "Failed to begin SPI transaction.");
            return false;
        }
        SPITransfer(tx_buf, rx_buf, kFrame1Len);
        EndTransaction();
        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    }

    // Frame 2
    {
        constexpr size_t kFrame2Len = 3;  // 2 (stat) + 1 (data)
        uint8_t rx_buf[kFrame2Len];

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetPacketType", "Failed to begin SPI transaction (frame 2).");
            return false;
        }
        SPITransfer(nullptr, rx_buf, kFrame2Len);
        EndTransaction();

        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
        if (last_stat_.command_status != CommandStatus::kDat && last_stat_.command_status != CommandStatus::kOk) {
            CONSOLE_ERROR("LR2021::GetPacketType", "Unexpected command status %s in frame 2.",
                          LR2021::CommandStatusToString(last_stat_.command_status));
            return false;
        }

        rsp_out->packet_type = rx_buf[2];
    }

    return true;
}

/**
 * GetRssiInst (opcode 0x020B)
 *
 * Frame 2 layout (4 bytes):
 *   [0:1]  Stat
 *   [2]    rssi[8:1]  (MSBs)
 *   [3]    rssi[0]    (LSB, bit 0 only)
 *
 * rssi = (buf[2] << 1) | (buf[3] & 1)
 * Actual signal power = -rssi/2 (dBm).
 */
bool LR2021::GetRssiInst(RssiInstRsp* rsp_out) {
    if (rsp_out == nullptr) {
        CONSOLE_ERROR("LR2021::GetRssiInst", "rsp_out is null.");
        return false;
    }

    // Frame 1
    {
        constexpr size_t kFrame1Len = 2;
        uint8_t tx_buf[kFrame1Len];
        uint8_t rx_buf[kFrame1Len];
        PackU16(tx_buf, kOpcodeGetRssiInst);

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetRssiInst", "Failed to begin SPI transaction.");
            return false;
        }
        SPITransfer(tx_buf, rx_buf, kFrame1Len);
        EndTransaction();
        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    }

    // Frame 2
    {
        constexpr size_t kFrame2Len = 4;  // 2 (stat) + 2 (data)
        uint8_t rx_buf[kFrame2Len];

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetRssiInst", "Failed to begin SPI transaction (frame 2).");
            return false;
        }
        SPITransfer(nullptr, rx_buf, kFrame2Len);
        EndTransaction();

        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
        if (last_stat_.command_status != CommandStatus::kDat && last_stat_.command_status != CommandStatus::kOk) {
            CONSOLE_ERROR("LR2021::GetRssiInst", "Unexpected command status %s in frame 2.",
                          LR2021::CommandStatusToString(last_stat_.command_status));
            return false;
        }

        rsp_out->rssi = static_cast<uint16_t>((static_cast<uint16_t>(rx_buf[2]) << 1) | (rx_buf[3] & 0x1));
    }

    return true;
}

/**
 * GetRxPktLength (opcode 0x0212)
 *
 * Frame 2 layout (4 bytes):
 *   [0:1]  Stat
 *   [2:3]  pkt_length (big-endian)
 */
bool LR2021::GetRxPktLength(RxPktLengthRsp* rsp_out) {
    if (rsp_out == nullptr) {
        CONSOLE_ERROR("LR2021::GetRxPktLength", "rsp_out is null.");
        return false;
    }

    // Frame 1
    {
        constexpr size_t kFrame1Len = 2;
        uint8_t tx_buf[kFrame1Len];
        uint8_t rx_buf[kFrame1Len];
        PackU16(tx_buf, kOpcodeGetRxPktLength);

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetRxPktLength", "Failed to begin SPI transaction.");
            return false;
        }
        SPITransfer(tx_buf, rx_buf, kFrame1Len);
        EndTransaction();
        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    }

    // Frame 2
    {
        constexpr size_t kFrame2Len = 4;  // 2 (stat) + 2 (data)
        uint8_t rx_buf[kFrame2Len];

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetRxPktLength", "Failed to begin SPI transaction (frame 2).");
            return false;
        }
        SPITransfer(nullptr, rx_buf, kFrame2Len);
        EndTransaction();

        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
        if (last_stat_.command_status != CommandStatus::kDat && last_stat_.command_status != CommandStatus::kOk) {
            CONSOLE_ERROR("LR2021::GetRxPktLength", "Unexpected command status %s in frame 2.",
                          LR2021::CommandStatusToString(last_stat_.command_status));
            return false;
        }

        rsp_out->pkt_length = UnpackU16(rx_buf + 2);
    }

    return true;
}

/**
 * GetTimestampValue (opcode 0x0217)
 *
 * Frame 1 layout (3 bytes): opcode + timestamp_index
 *
 * Frame 2 layout (6 bytes):
 *   [0:1]  Stat
 *   [2:5]  timestamp (big-endian, HF clock ticks)
 */
bool LR2021::GetTimestampValue(TimestampIndex timestamp_index, TimestampValueRsp* rsp_out) {
    if (rsp_out == nullptr) {
        CONSOLE_ERROR("LR2021::GetTimestampValue", "rsp_out is null.");
        return false;
    }

    // Frame 1
    {
        constexpr size_t kFrame1Len = 3;
        uint8_t tx_buf[kFrame1Len];
        uint8_t rx_buf[kFrame1Len];
        memset(tx_buf, 0, kFrame1Len);
        PackU16(tx_buf, kOpcodeGetTimestampValue);
        tx_buf[2] = static_cast<uint8_t>(timestamp_index) & 0x3;

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetTimestampValue", "Failed to begin SPI transaction.");
            return false;
        }
        SPITransfer(tx_buf, rx_buf, kFrame1Len);
        EndTransaction();
        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    }

    // Frame 2
    {
        constexpr size_t kFrame2Len = 6;  // 2 (stat) + 4 (data)
        uint8_t rx_buf[kFrame2Len];

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetTimestampValue", "Failed to begin SPI transaction (frame 2).");
            return false;
        }
        SPITransfer(nullptr, rx_buf, kFrame2Len);
        EndTransaction();

        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
        if (last_stat_.command_status != CommandStatus::kDat && last_stat_.command_status != CommandStatus::kOk) {
            CONSOLE_ERROR("LR2021::GetTimestampValue", "Unexpected command status %s in frame 2.",
                          LR2021::CommandStatusToString(last_stat_.command_status));
            return false;
        }

        rsp_out->timestamp = UnpackU32(rx_buf + 2);
    }

    return true;
}

/**
 * GetCcaResult (opcode 0x0219)
 *
 * Frame 2 layout (6 bytes):
 *   [0:1]  Stat
 *   [2]    rssi_min MSBs
 *   [3]    rssi_max MSBs
 *   [4]    rssi_avg MSBs
 *   [5]    packed LSBs: bit2=rssi_min LSB, bit1=rssi_max LSB, bit0=rssi_avg LSB
 *
 * rssi_min = (buf[2] << 1) | ((buf[5] >> 2) & 1)
 * rssi_max = (buf[3] << 1) | ((buf[5] >> 1) & 1)
 * rssi_avg = (buf[4] << 1) | (buf[5] & 1)
 * Actual power = -rssi/2 (dBm) for each value.
 */
bool LR2021::GetCcaResult(CcaResultRsp* rsp_out) {
    if (rsp_out == nullptr) {
        CONSOLE_ERROR("LR2021::GetCcaResult", "rsp_out is null.");
        return false;
    }

    // Frame 1
    {
        constexpr size_t kFrame1Len = 2;
        uint8_t tx_buf[kFrame1Len];
        uint8_t rx_buf[kFrame1Len];
        PackU16(tx_buf, kOpcodeGetCcaResult);

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetCcaResult", "Failed to begin SPI transaction.");
            return false;
        }
        SPITransfer(tx_buf, rx_buf, kFrame1Len);
        EndTransaction();
        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    }

    // Frame 2
    {
        constexpr size_t kFrame2Len = 6;  // 2 (stat) + 4 (data)
        uint8_t rx_buf[kFrame2Len];

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetCcaResult", "Failed to begin SPI transaction (frame 2).");
            return false;
        }
        SPITransfer(nullptr, rx_buf, kFrame2Len);
        EndTransaction();

        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
        if (last_stat_.command_status != CommandStatus::kDat && last_stat_.command_status != CommandStatus::kOk) {
            CONSOLE_ERROR("LR2021::GetCcaResult", "Unexpected command status %s in frame 2.",
                          LR2021::CommandStatusToString(last_stat_.command_status));
            return false;
        }

        const uint8_t lsbs = rx_buf[5];
        rsp_out->rssi_min = static_cast<uint16_t>((static_cast<uint16_t>(rx_buf[2]) << 1) | ((lsbs >> 2) & 0x1));
        rsp_out->rssi_max = static_cast<uint16_t>((static_cast<uint16_t>(rx_buf[3]) << 1) | ((lsbs >> 1) & 0x1));
        rsp_out->rssi_avg = static_cast<uint16_t>((static_cast<uint16_t>(rx_buf[4]) << 1) | (lsbs & 0x1));
    }

    return true;
}
