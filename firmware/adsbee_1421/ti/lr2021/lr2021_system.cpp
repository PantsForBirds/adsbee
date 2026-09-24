// lr2021_system.cpp
//
// Semtech LR2021 driver – system firmware command implementation.
// See lr2021.hh for API documentation and datasheet references.
//
// All commands in this file use the 0x01xx opcode group (system commands).
// SPI framing follows the same rules as lr2021_common.cpp:
//
//   Write command (single frame):
//     MOSI: Op[15:8] Op[7:0] Arg0 Arg1 ...
//     MISO: Stat[15:8] Stat[7:0] IrqStat[31:0] 0x00 ...
//
//   Read command (two frames):
//     Frame 1 – send opcode + optional arguments (BUSY asserts then deasserts)
//     Frame 2 – send 0x00 bytes; device clocks out Stat + data
//
// Note on GetStatus: its frame 2 payload is Stat[2] + IrqStat[4] (6 bytes).
// All other system read commands return Stat[2] + data only (no trailing IrqStat).

#include <cstring>

#include "comms.hh"
#include "lr2021.hh"

// ---------------------------------------------------------------------------
// Read commands (two-frame SPI sequence)
// ---------------------------------------------------------------------------

/**
 * GetStatus (opcode 0x0100)
 *
 * Frame 1: 2 bytes (opcode only)
 *
 * Frame 2 layout (6 bytes):
 *   [0:1]  Stat
 *   [2:5]  IrqStatus (big-endian)
 */
bool LR2021::GetStatus(StatusRsp* rsp_out) {
    if (rsp_out == nullptr) return false;

    // Frame 1
    {
        constexpr size_t kFrame1Len = 2;
        uint8_t tx_buf[kFrame1Len];
        uint8_t rx_buf[kFrame1Len];
        memset(tx_buf, 0, kFrame1Len);

        PackU16(tx_buf + 0, kOpcodeGetStatus);

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetStatus", "Failed to begin SPI transaction.");
            return false;
        }
        SPITransfer(tx_buf, rx_buf, kFrame1Len);
        EndTransaction();
        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    }

    // Frame 2
    {
        constexpr size_t kFrame2Len = 6;  // 2 (stat) + 4 (irq status)
        uint8_t rx_buf[kFrame2Len];

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetStatus", "Failed to begin SPI transaction (frame 2).");
            return false;
        }
        SPITransfer(nullptr, rx_buf, kFrame2Len);
        EndTransaction();

        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
        if (last_stat_.command_status != CommandStatus::kDat && last_stat_.command_status != CommandStatus::kOk) {
            CONSOLE_ERROR("LR2021::GetStatus", "Unexpected command status %s in frame 2.",
                          LR2021::CommandStatusToString(last_stat_.command_status));
            return false;
        }

        rsp_out->irq_status = UnpackU32(rx_buf + 2);
    }

    return true;
}

/**
 * GetVersion (opcode 0x0101)
 *
 * Frame 2 layout (4 bytes):
 *   [0:1]  Stat
 *   [2]    major version
 *   [3]    minor version
 */
bool LR2021::GetVersion(VersionRsp* rsp_out) {
    if (rsp_out == nullptr) return false;

    // Frame 1
    {
        constexpr size_t kFrame1Len = 2;
        uint8_t tx_buf[kFrame1Len];
        uint8_t rx_buf[kFrame1Len];
        memset(tx_buf, 0, kFrame1Len);

        PackU16(tx_buf + 0, kOpcodeGetVersion);

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetVersion", "Failed to begin SPI transaction.");
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
            CONSOLE_ERROR("LR2021::GetVersion", "Failed to begin SPI transaction (frame 2).");
            return false;
        }
        SPITransfer(nullptr, rx_buf, kFrame2Len);
        EndTransaction();

        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
        if (last_stat_.command_status != CommandStatus::kDat && last_stat_.command_status != CommandStatus::kOk) {
            CONSOLE_ERROR("LR2021::GetVersion", "Unexpected command status %s in frame 2.",
                          LR2021::CommandStatusToString(last_stat_.command_status));
            return false;
        }

        rsp_out->major = rx_buf[2];
        rsp_out->minor = rx_buf[3];
    }

    return true;
}

/**
 * GetErrors (opcode 0x0110)
 *
 * Frame 2 layout (4 bytes):
 *   [0:1]  Stat
 *   [2:3]  error_flags (big-endian, 16-bit)
 */
bool LR2021::GetErrors(ErrorsRsp* rsp_out) {
    if (rsp_out == nullptr) return false;

    // Frame 1
    {
        constexpr size_t kFrame1Len = 2;
        uint8_t tx_buf[kFrame1Len];
        uint8_t rx_buf[kFrame1Len];
        memset(tx_buf, 0, kFrame1Len);

        PackU16(tx_buf + 0, kOpcodeGetErrors);

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetErrors", "Failed to begin SPI transaction.");
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
            CONSOLE_ERROR("LR2021::GetErrors", "Failed to begin SPI transaction (frame 2).");
            return false;
        }
        SPITransfer(nullptr, rx_buf, kFrame2Len);
        EndTransaction();

        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
        if (last_stat_.command_status != CommandStatus::kDat && last_stat_.command_status != CommandStatus::kOk) {
            CONSOLE_ERROR("LR2021::GetErrors", "Unexpected command status %s in frame 2.",
                          LR2021::CommandStatusToString(last_stat_.command_status));
            return false;
        }

        rsp_out->error_flags = UnpackU16(rx_buf + 2);
    }

    return true;
}

/**
 * GetAndClearIrq (opcode 0x0117)
 *
 * Frame 2 layout (6 bytes):
 *   [0:1]  Stat
 *   [2:5]  irq_flags (big-endian, 32-bit)
 */
bool LR2021::GetAndClearIrq(GetAndClearIrqRsp* rsp_out) {
    if (rsp_out == nullptr) return false;

    // Frame 1
    {
        constexpr size_t kFrame1Len = 2;
        uint8_t tx_buf[kFrame1Len];
        uint8_t rx_buf[kFrame1Len];
        memset(tx_buf, 0, kFrame1Len);

        PackU16(tx_buf + 0, kOpcodeGetAndClearIrq);

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetAndClearIrq", "Failed to begin SPI transaction.");
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
            CONSOLE_ERROR("LR2021::GetAndClearIrq", "Failed to begin SPI transaction (frame 2).");
            return false;
        }
        SPITransfer(nullptr, rx_buf, kFrame2Len);
        EndTransaction();

        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
        if (last_stat_.command_status != CommandStatus::kDat && last_stat_.command_status != CommandStatus::kOk) {
            CONSOLE_ERROR("LR2021::GetAndClearIrq", "Unexpected command status %s in frame 2.",
                          LR2021::CommandStatusToString(last_stat_.command_status));
            return false;
        }

        rsp_out->irq_flags = UnpackU32(rx_buf + 2);
    }

    return true;
}

/**
 * GetFifoIrqFlags (opcode 0x011B)
 *
 * Frame 2 layout (4 bytes):
 *   [0:1]  Stat
 *   [2]    rx_flags
 *   [3]    tx_flags
 */
bool LR2021::GetFifoIrqFlags(FifoIrqFlagsRsp* rsp_out) {
    if (rsp_out == nullptr) return false;

    // Frame 1
    {
        constexpr size_t kFrame1Len = 2;
        uint8_t tx_buf[kFrame1Len];
        uint8_t rx_buf[kFrame1Len];
        memset(tx_buf, 0, kFrame1Len);

        PackU16(tx_buf + 0, kOpcodeGetFifoIrqFlags);

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetFifoIrqFlags", "Failed to begin SPI transaction.");
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
            CONSOLE_ERROR("LR2021::GetFifoIrqFlags", "Failed to begin SPI transaction (frame 2).");
            return false;
        }
        SPITransfer(nullptr, rx_buf, kFrame2Len);
        EndTransaction();

        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
        if (last_stat_.command_status != CommandStatus::kDat && last_stat_.command_status != CommandStatus::kOk) {
            CONSOLE_ERROR("LR2021::GetFifoIrqFlags", "Unexpected command status %s in frame 2.",
                          LR2021::CommandStatusToString(last_stat_.command_status));
            return false;
        }

        rsp_out->rx_flags = rx_buf[2];
        rsp_out->tx_flags = rx_buf[3];
    }

    return true;
}

/**
 * GetRxFifoLevel (opcode 0x011C)
 *
 * Frame 2 layout (4 bytes):
 *   [0:1]  Stat
 *   [2:3]  level in bytes (big-endian)
 */
bool LR2021::GetRxFifoLevel(FifoLevelRsp* rsp_out) {
    if (rsp_out == nullptr) return false;

    // Frame 1
    {
        constexpr size_t kFrame1Len = 2;
        uint8_t tx_buf[kFrame1Len];
        uint8_t rx_buf[kFrame1Len];
        memset(tx_buf, 0, kFrame1Len);

        PackU16(tx_buf + 0, kOpcodeGetRxFifoLevel);

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetRxFifoLevel", "Failed to begin SPI transaction.");
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
            CONSOLE_ERROR("LR2021::GetRxFifoLevel", "Failed to begin SPI transaction (frame 2).");
            return false;
        }
        SPITransfer(nullptr, rx_buf, kFrame2Len);
        EndTransaction();

        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
        if (last_stat_.command_status != CommandStatus::kDat && last_stat_.command_status != CommandStatus::kOk) {
            CONSOLE_ERROR("LR2021::GetRxFifoLevel", "Unexpected command status %s in frame 2.",
                          LR2021::CommandStatusToString(last_stat_.command_status));
            return false;
        }

        rsp_out->level = UnpackU16(rx_buf + 2);
    }

    return true;
}

/**
 * GetTxFifoLevel (opcode 0x011D)
 *
 * Frame 2 layout (4 bytes):
 *   [0:1]  Stat
 *   [2:3]  level in bytes (big-endian)
 */
bool LR2021::GetTxFifoLevel(FifoLevelRsp* rsp_out) {
    if (rsp_out == nullptr) return false;

    // Frame 1
    {
        constexpr size_t kFrame1Len = 2;
        uint8_t tx_buf[kFrame1Len];
        uint8_t rx_buf[kFrame1Len];
        memset(tx_buf, 0, kFrame1Len);

        PackU16(tx_buf + 0, kOpcodeGetTxFifoLevel);

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetTxFifoLevel", "Failed to begin SPI transaction.");
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
            CONSOLE_ERROR("LR2021::GetTxFifoLevel", "Failed to begin SPI transaction (frame 2).");
            return false;
        }
        SPITransfer(nullptr, rx_buf, kFrame2Len);
        EndTransaction();

        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
        if (last_stat_.command_status != CommandStatus::kDat && last_stat_.command_status != CommandStatus::kOk) {
            CONSOLE_ERROR("LR2021::GetTxFifoLevel", "Unexpected command status %s in frame 2.",
                          LR2021::CommandStatusToString(last_stat_.command_status));
            return false;
        }

        rsp_out->level = UnpackU16(rx_buf + 2);
    }

    return true;
}

/**
 * GetVBat (opcode 0x0124, 3 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------------------------------
 *   0   | 0x01
 *   1   | 0x24
 *   2   | (vbat_format & 0x1) << 3 | (adc_res & 0x7)
 *
 * Frame 2 layout (4 bytes):
 *   [0:1]  Stat
 *   [2:3]  value (big-endian; interpretation depends on vbat_format)
 */
bool LR2021::GetVBat(VbatFormat vbat_format, AdcRes adc_res, VBatRsp* rsp_out) {
    if (rsp_out == nullptr) return false;

    // Frame 1
    {
        constexpr size_t kFrame1Len = 3;
        uint8_t tx_buf[kFrame1Len];
        uint8_t rx_buf[kFrame1Len];
        memset(tx_buf, 0, kFrame1Len);

        PackU16(tx_buf + 0, kOpcodeGetVBat);
        tx_buf[2] = static_cast<uint8_t>((static_cast<uint8_t>(vbat_format) & 0x1) << 3) |
                    static_cast<uint8_t>(static_cast<uint8_t>(adc_res) & 0x7);

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetVBat", "Failed to begin SPI transaction.");
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
            CONSOLE_ERROR("LR2021::GetVBat", "Failed to begin SPI transaction (frame 2).");
            return false;
        }
        SPITransfer(nullptr, rx_buf, kFrame2Len);
        EndTransaction();

        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
        if (last_stat_.command_status != CommandStatus::kDat && last_stat_.command_status != CommandStatus::kOk) {
            CONSOLE_ERROR("LR2021::GetVBat", "Unexpected command status %s in frame 2.",
                          LR2021::CommandStatusToString(last_stat_.command_status));
            return false;
        }

        rsp_out->value = UnpackU16(rx_buf + 2);
    }

    return true;
}

/**
 * GetTemp (opcode 0x0125, 3 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------------------------------
 *   0   | 0x01
 *   1   | 0x25
 *   2   | (temp_src & 0x3) << 4 | 0x8 (force Celsius) | (adc_res & 0x7)
 *
 * Frame 2 layout (4 bytes):
 *   [0:1]  Stat
 *   [2]    temp[12:5]  – full byte
 *   [3]    temp[4:0]   – top 5 bits (bits[7:3])
 *
 * The 13-bit two's-complement temperature is reconstructed as:
 *   raw = (buf[3] >> 3) | (buf[2] << 5)
 *   if buf[2] bit 7 set: temp = raw - 8192
 */
bool LR2021::GetTemp(TempSrc temp_src, AdcRes adc_res, TempRsp* rsp_out) {
    if (rsp_out == nullptr) return false;

    // Frame 1
    {
        constexpr size_t kFrame1Len = 3;
        uint8_t tx_buf[kFrame1Len];
        uint8_t rx_buf[kFrame1Len];
        memset(tx_buf, 0, kFrame1Len);

        PackU16(tx_buf + 0, kOpcodeGetTemp);
        tx_buf[2] = static_cast<uint8_t>((static_cast<uint8_t>(temp_src) & 0x3) << 4) | 0x8 |  // force Celsius format
                    static_cast<uint8_t>(static_cast<uint8_t>(adc_res) & 0x7);

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetTemp", "Failed to begin SPI transaction.");
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
            CONSOLE_ERROR("LR2021::GetTemp", "Failed to begin SPI transaction (frame 2).");
            return false;
        }
        SPITransfer(nullptr, rx_buf, kFrame2Len);
        EndTransaction();

        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
        if (last_stat_.command_status != CommandStatus::kDat && last_stat_.command_status != CommandStatus::kOk) {
            CONSOLE_ERROR("LR2021::GetTemp", "Unexpected command status %s in frame 2.",
                          LR2021::CommandStatusToString(last_stat_.command_status));
            return false;
        }

        // Reconstruct 13-bit two's-complement value.
        const uint16_t raw = static_cast<uint16_t>(rx_buf[3] >> 3) | (static_cast<uint16_t>(rx_buf[2]) << 5);
        rsp_out->temp_celsius = static_cast<int16_t>(raw);
        if (rx_buf[2] & 0x80) {
            rsp_out->temp_celsius = static_cast<int16_t>(raw) - (1 << 13);
        }
    }

    return true;
}

/**
 * GetRandomNumber (opcode 0x0126, 2 bytes)
 *
 * Frame 2 layout (6 bytes):
 *   [0:1]  Stat
 *   [2:5]  random number (big-endian, 32-bit)
 */
bool LR2021::GetRandomNumber(RandomNumberRsp* rsp_out) {
    if (rsp_out == nullptr) return false;

    // Frame 1
    {
        constexpr size_t kFrame1Len = 2;
        uint8_t tx_buf[kFrame1Len];
        uint8_t rx_buf[kFrame1Len];
        memset(tx_buf, 0, kFrame1Len);

        PackU16(tx_buf + 0, kOpcodeGetRandomNumber);

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetRandomNumber", "Failed to begin SPI transaction.");
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
            CONSOLE_ERROR("LR2021::GetRandomNumber", "Failed to begin SPI transaction (frame 2).");
            return false;
        }
        SPITransfer(nullptr, rx_buf, kFrame2Len);
        EndTransaction();

        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
        if (last_stat_.command_status != CommandStatus::kDat && last_stat_.command_status != CommandStatus::kOk) {
            CONSOLE_ERROR("LR2021::GetRandomNumber", "Unexpected command status %s in frame 2.",
                          LR2021::CommandStatusToString(last_stat_.command_status));
            return false;
        }

        rsp_out->value = UnpackU32(rx_buf + 2);
    }

    return true;
}

/**
 * GetRandomNumberAdv (opcode 0x0126, 3 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x01
 *   1   | 0x26
 *   2   | source & 0x3
 *
 * Frame 2 layout: same as GetRandomNumber.
 */
bool LR2021::GetRandomNumberAdv(uint8_t source, RandomNumberRsp* rsp_out) {
    if (rsp_out == nullptr) return false;

    // Frame 1
    {
        constexpr size_t kFrame1Len = 3;
        uint8_t tx_buf[kFrame1Len];
        uint8_t rx_buf[kFrame1Len];
        memset(tx_buf, 0, kFrame1Len);

        PackU16(tx_buf + 0, kOpcodeGetRandomNumber);
        tx_buf[2] = source & 0x3;

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetRandomNumberAdv", "Failed to begin SPI transaction.");
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
            CONSOLE_ERROR("LR2021::GetRandomNumberAdv", "Failed to begin SPI transaction (frame 2).");
            return false;
        }
        SPITransfer(nullptr, rx_buf, kFrame2Len);
        EndTransaction();

        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
        if (last_stat_.command_status != CommandStatus::kDat && last_stat_.command_status != CommandStatus::kOk) {
            CONSOLE_ERROR("LR2021::GetRandomNumberAdv", "Unexpected command status %s in frame 2.",
                          LR2021::CommandStatusToString(last_stat_.command_status));
            return false;
        }

        rsp_out->value = UnpackU32(rx_buf + 2);
    }

    return true;
}

/**
 * GetAndClearFifoIrqFlags (opcode 0x012E)
 *
 * Frame 2 layout (4 bytes):
 *   [0:1]  Stat
 *   [2]    rx_flags
 *   [3]    tx_flags
 */
bool LR2021::GetAndClearFifoIrqFlags(FifoIrqFlagsRsp* rsp_out) {
    if (rsp_out == nullptr) return false;

    // Frame 1
    {
        constexpr size_t kFrame1Len = 2;
        uint8_t tx_buf[kFrame1Len];
        uint8_t rx_buf[kFrame1Len];
        memset(tx_buf, 0, kFrame1Len);

        PackU16(tx_buf + 0, kOpcodeGetAndClearFifoIrqFlags);

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetAndClearFifoIrqFlags", "Failed to begin SPI transaction.");
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
            CONSOLE_ERROR("LR2021::GetAndClearFifoIrqFlags", "Failed to begin SPI transaction (frame 2).");
            return false;
        }
        SPITransfer(nullptr, rx_buf, kFrame2Len);
        EndTransaction();

        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
        if (last_stat_.command_status != CommandStatus::kDat && last_stat_.command_status != CommandStatus::kOk) {
            CONSOLE_ERROR("LR2021::GetAndClearFifoIrqFlags", "Unexpected command status %s in frame 2.",
                          LR2021::CommandStatusToString(last_stat_.command_status));
            return false;
        }

        rsp_out->rx_flags = rx_buf[2];
        rsp_out->tx_flags = rx_buf[3];
    }

    return true;
}

// ---------------------------------------------------------------------------
// Write commands (single SPI frame)
// ---------------------------------------------------------------------------

/**
 * ClearErrors (opcode 0x0111, 2 bytes)
 */
bool LR2021::ClearErrors() {
    constexpr size_t kFrameLen = 2;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeClearErrors);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::ClearErrors", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::ClearErrors", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetDioFunction (opcode 0x0112, 4 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------------------------------
 *   0   | 0x01
 *   1   | 0x12
 *   2   | dio_num & 0xF
 *   3   | (dio_func & 0xF) << 4 | (pull_drive & 0xF)
 */
bool LR2021::SetDioFunction(DioNum dio_num, DioFunc dio_func, PullDrive pull_drive) {
    constexpr size_t kFrameLen = 4;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetDioFunction);
    tx_buf[2] = static_cast<uint8_t>(dio_num) & 0xF;
    tx_buf[3] = static_cast<uint8_t>((static_cast<uint8_t>(dio_func) & 0xF) << 4) |
                static_cast<uint8_t>(static_cast<uint8_t>(pull_drive) & 0xF);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetDioFunction", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetDioFunction", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetDioRfSwitchConfig (opcode 0x0113, 4 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------------------------------
 *   0   | 0x01
 *   1   | 0x13
 *   2   | dio_num & 0xF
 *   3   | tx_hf<<4 | rx_hf<<3 | tx_lf<<2 | rx_lf<<1 | standby
 */
bool LR2021::SetDioRfSwitchConfig(DioNum dio_num, bool tx_hf, bool rx_hf, bool tx_lf, bool rx_lf, bool standby) {
    constexpr size_t kFrameLen = 4;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetDioRfSwitchConfig);
    tx_buf[2] = static_cast<uint8_t>(dio_num) & 0xF;
    tx_buf[3] = (tx_hf ? 0x10u : 0u) | (rx_hf ? 0x08u : 0u) | (tx_lf ? 0x04u : 0u) | (rx_lf ? 0x02u : 0u) |
                (standby ? 0x01u : 0u);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetDioRfSwitchConfig", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetDioRfSwitchConfig", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * ClearFifoIrqFlags (opcode 0x0114, 4 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x01
 *   1   | 0x14
 *   2   | rx_flags
 *   3   | tx_flags
 */
bool LR2021::ClearFifoIrqFlags(uint8_t rx_flags, uint8_t tx_flags) {
    constexpr size_t kFrameLen = 4;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeClearFifoIrqFlags);
    tx_buf[2] = rx_flags;
    tx_buf[3] = tx_flags;

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::ClearFifoIrqFlags", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::ClearFifoIrqFlags", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetDioIrqConfig (opcode 0x0115, 7 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x01
 *   1   | 0x15
 *   2   | dio_num & 0xF
 *   3   | irqs[31:24]
 *   4   | irqs[23:16]
 *   5   | irqs[15:8]
 *   6   | irqs[7:0]
 */
bool LR2021::SetDioIrqConfig(DioNum dio_num, uint32_t irqs) {
    constexpr size_t kFrameLen = 7;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetDioIrqConfig);
    tx_buf[2] = static_cast<uint8_t>(dio_num) & 0xF;
    PackU32(tx_buf + 3, irqs);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetDioIrqConfig", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetDioIrqConfig", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * ClearIrq (opcode 0x0116, 6 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x01
 *   1   | 0x16
 *   2   | irqs[31:24]
 *   3   | irqs[23:16]
 *   4   | irqs[15:8]
 *   5   | irqs[7:0]
 */
bool LR2021::ClearIrq(uint32_t irqs) {
    constexpr size_t kFrameLen = 6;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeClearIrq);
    PackU32(tx_buf + 2, irqs);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::ClearIrq", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::ClearIrq", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * ConfigLfClock (opcode 0x0118, 3 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x01
 *   1   | 0x18
 *   2   | lf_clock & 0x3
 */
bool LR2021::ConfigLfClock(LfClock lf_clock) {
    constexpr size_t kFrameLen = 3;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeConfigLfClock);
    tx_buf[2] = static_cast<uint8_t>(lf_clock) & 0x3;

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::ConfigLfClock", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::ConfigLfClock", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * ConfigClkOutputs (opcode 0x0119, 3 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x01
 *   1   | 0x19
 *   2   | clk_scaling
 */
bool LR2021::ConfigClkOutputs(ClkScaling clk_scaling) {
    constexpr size_t kFrameLen = 3;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeConfigClkOutputs);
    tx_buf[2] = static_cast<uint8_t>(clk_scaling);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::ConfigClkOutputs", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::ConfigClkOutputs", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * ConfigFifoIrq (opcode 0x011A, 4 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x01
 *   1   | 0x1A
 *   2   | rx_irq_enable
 *   3   | tx_irq_enable
 */
bool LR2021::ConfigFifoIrq(uint8_t rx_irq_enable, uint8_t tx_irq_enable) {
    constexpr size_t kFrameLen = 4;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeConfigFifoIrq);
    tx_buf[2] = rx_irq_enable;
    tx_buf[3] = tx_irq_enable;

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::ConfigFifoIrq", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::ConfigFifoIrq", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * ConfigFifoIrqAdv (opcode 0x011A, 12 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x01
 *   1   | 0x1A
 *   2   | rx_irq_enable
 *   3   | tx_irq_enable
 *   4   | rx_high_threshold[15:8]
 *   5   | rx_high_threshold[7:0]
 *   6   | tx_low_threshold[15:8]
 *   7   | tx_low_threshold[7:0]
 *   8   | rx_low_threshold[15:8]
 *   9   | rx_low_threshold[7:0]
 *  10   | tx_high_threshold[15:8]
 *  11   | tx_high_threshold[7:0]
 */
bool LR2021::ConfigFifoIrqAdv(uint8_t rx_irq_enable, uint8_t tx_irq_enable, uint16_t rx_high_threshold,
                              uint16_t tx_low_threshold, uint16_t rx_low_threshold, uint16_t tx_high_threshold) {
    constexpr size_t kFrameLen = 12;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeConfigFifoIrq);
    tx_buf[2] = rx_irq_enable;
    tx_buf[3] = tx_irq_enable;
    PackU16(tx_buf + 4, rx_high_threshold);
    PackU16(tx_buf + 6, tx_low_threshold);
    PackU16(tx_buf + 8, rx_low_threshold);
    PackU16(tx_buf + 10, tx_high_threshold);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::ConfigFifoIrqAdv", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::ConfigFifoIrqAdv", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * ClearRxFifo (opcode 0x011E, 2 bytes)
 */
bool LR2021::ClearRxFifo() {
    constexpr size_t kFrameLen = 2;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeClearRxFifo);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::ClearRxFifo", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::ClearRxFifo", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * ClearTxFifo (opcode 0x011F, 2 bytes)
 */
bool LR2021::ClearTxFifo() {
    constexpr size_t kFrameLen = 2;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeClearTxFifo);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::ClearTxFifo", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::ClearTxFifo", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetRegMode (opcode 0x0121, 3 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x01
 *   1   | 0x21
 *   2   | simo_usage & 0x3
 */
bool LR2021::SetRegMode(SimoUsage simo_usage) {
    constexpr size_t kFrameLen = 3;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetRegMode);
    tx_buf[2] = static_cast<uint8_t>(simo_usage) & 0x3;

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetRegMode", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetRegMode", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetRegModeAdv (opcode 0x0121, 7 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------------------------------
 *   0   | 0x01
 *   1   | 0x21
 *   2   | simo_usage & 0x3
 *   3   | (rc2ru_unit & 0x3) << 5 | (rc2ru & 0x1F)
 *   4   | (tx2ru_unit & 0x3) << 5 | (tx2ru & 0x1F)
 *   5   | (ru2rc_unit & 0x3) << 5 | (ru2rc & 0x1F)
 *   6   | (ramp_down_unit & 0x3) << 5 | (ramp_down & 0x1F)
 */
bool LR2021::SetRegModeAdv(SimoUsage simo_usage, RampTimeUnit rc2ru_unit, uint8_t rc2ru, RampTimeUnit tx2ru_unit,
                           uint8_t tx2ru, RampTimeUnit ru2rc_unit, uint8_t ru2rc, RampTimeUnit ramp_down_unit,
                           uint8_t ramp_down) {
    constexpr size_t kFrameLen = 7;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetRegMode);
    tx_buf[2] = static_cast<uint8_t>(simo_usage) & 0x3;
    tx_buf[3] = static_cast<uint8_t>((static_cast<uint8_t>(rc2ru_unit) & 0x3) << 5) | (rc2ru & 0x1F);
    tx_buf[4] = static_cast<uint8_t>((static_cast<uint8_t>(tx2ru_unit) & 0x3) << 5) | (tx2ru & 0x1F);
    tx_buf[5] = static_cast<uint8_t>((static_cast<uint8_t>(ru2rc_unit) & 0x3) << 5) | (ru2rc & 0x1F);
    tx_buf[6] = static_cast<uint8_t>((static_cast<uint8_t>(ramp_down_unit) & 0x3) << 5) | (ramp_down & 0x1F);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetRegModeAdv", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetRegModeAdv", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * Calibrate (opcode 0x0122, 3 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------------------------------
 *   0   | 0x01
 *   1   | 0x22
 *   2   | pa_offset<<6 | meas_unit<<4 | aaf<<3 | pll<<2 | hf_rc<<1 | lf_rc
 */
bool LR2021::Calibrate(bool pa_offset, bool meas_unit, bool aaf, bool pll, bool hf_rc, bool lf_rc) {
    constexpr size_t kFrameLen = 3;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeCalibrate);
    tx_buf[2] = (pa_offset ? 0x40u : 0u) | (meas_unit ? 0x10u : 0u) | (aaf ? 0x08u : 0u) | (pll ? 0x04u : 0u) |
                (hf_rc ? 0x02u : 0u) | (lf_rc ? 0x01u : 0u);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::Calibrate", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::Calibrate", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * CalibFe (opcode 0x0123, 8 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x01
 *   1   | 0x23
 *   2   | freq1[15:8]
 *   3   | freq1[7:0]
 *   4   | freq2[15:8]
 *   5   | freq2[7:0]
 *   6   | freq3[15:8]
 *   7   | freq3[7:0]
 */
bool LR2021::CalibFe(uint16_t freq1, uint16_t freq2, uint16_t freq3) {
    constexpr size_t kFrameLen = 8;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeCalibFe);
    PackU16(tx_buf + 2, freq1);
    PackU16(tx_buf + 4, freq2);
    PackU16(tx_buf + 6, freq3);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::CalibFe", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::CalibFe", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetSleep (opcode 0x0127, 3 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------------------------------
 *   0   | 0x01
 *   1   | 0x27
 *   2   | (ret_en & 0xF) << 1 | clk_32k_en
 */
bool LR2021::SetSleep(bool clk_32k_en, uint8_t ret_en) {
    constexpr size_t kFrameLen = 3;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetSleep);
    tx_buf[2] = static_cast<uint8_t>((ret_en & 0xF) << 1) | static_cast<uint8_t>(clk_32k_en ? 0x1u : 0u);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetSleep", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetSleep", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetSleepAdv (opcode 0x0127, 7 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------------------------------
 *   0   | 0x01
 *   1   | 0x27
 *   2   | (ret_en & 0xF) << 1 | clk_32k_en
 *   3   | sleep_time[31:24]
 *   4   | sleep_time[23:16]
 *   5   | sleep_time[15:8]
 *   6   | sleep_time[7:0]
 */
bool LR2021::SetSleepAdv(bool clk_32k_en, uint8_t ret_en, uint32_t sleep_time) {
    constexpr size_t kFrameLen = 7;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetSleep);
    tx_buf[2] = static_cast<uint8_t>((ret_en & 0xF) << 1) | static_cast<uint8_t>(clk_32k_en ? 0x1u : 0u);
    PackU32(tx_buf + 3, sleep_time);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetSleepAdv", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetSleepAdv", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetStandby (opcode 0x0128, 3 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x01
 *   1   | 0x28
 *   2   | standby_mode & 0x1
 */
bool LR2021::SetStandby(SysStandbyMode standby_mode) {
    constexpr size_t kFrameLen = 3;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetStandby);
    tx_buf[2] = static_cast<uint8_t>(standby_mode) & 0x1;

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetStandby", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetStandby", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetFs (opcode 0x0129, 2 bytes)
 */
bool LR2021::SetFs() {
    constexpr size_t kFrameLen = 2;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetFs);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetFs", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetFs", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetAdditionalRegToRetain (opcode 0x012A, 6 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x01
 *   1   | 0x2A
 *   2   | slot & 0x1F
 *   3   | addr[23:16]
 *   4   | addr[15:8]
 *   5   | addr[7:0]
 */
bool LR2021::SetAdditionalRegToRetain(uint8_t slot, uint32_t addr) {
    constexpr size_t kFrameLen = 6;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetAdditionalRegToRetain);
    tx_buf[2] = slot & 0x1F;
    PackU24(tx_buf + 3, addr);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetAdditionalRegToRetain", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetAdditionalRegToRetain", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetEolConfig (opcode 0x0130, 3 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------------------------------
 *   0   | 0x01
 *   1   | 0x30
 *   2   | enable<<3 | (eol_trim & 0x7)
 */
bool LR2021::SetEolConfig(EolTrim eol_trim, bool enable) {
    constexpr size_t kFrameLen = 3;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetEolConfig);
    tx_buf[2] = static_cast<uint8_t>(static_cast<uint8_t>(eol_trim) & 0x7) | static_cast<uint8_t>(enable ? 0x8u : 0u);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetEolConfig", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetEolConfig", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetTcxoMode (opcode 0x0120, 7 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x01
 *   1   | 0x20
 *   2   | tcxo_voltage & 0x7
 *   3   | start_time[31:24]
 *   4   | start_time[23:16]
 *   5   | start_time[15:8]
 *   6   | start_time[7:0]
 */
bool LR2021::SetTcxoMode(TcxoVoltage tcxo_voltage, uint32_t start_time) {
    constexpr size_t kFrameLen = 7;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetTcxoMode);
    tx_buf[2] = static_cast<uint8_t>(tcxo_voltage) & 0x7;
    PackU32(tx_buf + 3, start_time);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetTcxoMode", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetTcxoMode", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetXoscCpTrim (opcode 0x0131, 4 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x01
 *   1   | 0x31
 *   2   | xta & 0x3F
 *   3   | xtb & 0x3F
 */
bool LR2021::SetXoscCpTrim(uint8_t xta, uint8_t xtb) {
    constexpr size_t kFrameLen = 4;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetXoscCpTrim);
    tx_buf[2] = xta & 0x3F;
    tx_buf[3] = xtb & 0x3F;

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetXoscCpTrim", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetXoscCpTrim", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetXoscCpTrimAdv (opcode 0x0131, 5 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x01
 *   1   | 0x31
 *   2   | xta & 0x3F
 *   3   | xtb & 0x3F
 *   4   | delay_us
 */
bool LR2021::SetXoscCpTrimAdv(uint8_t xta, uint8_t xtb, uint8_t delay_us) {
    constexpr size_t kFrameLen = 5;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetXoscCpTrim);
    tx_buf[2] = xta & 0x3F;
    tx_buf[3] = xtb & 0x3F;
    tx_buf[4] = delay_us;

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetXoscCpTrimAdv", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetXoscCpTrimAdv", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetTempCompCfg (opcode 0x0132, 3 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------------------------------
 *   0   | 0x01
 *   1   | 0x32
 *   2   | ntc_en<<2 | (comp_mode & 0x3)
 */
bool LR2021::SetTempCompCfg(bool ntc_en, CompMode comp_mode) {
    constexpr size_t kFrameLen = 3;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetTempCompCfg);
    tx_buf[2] = static_cast<uint8_t>(ntc_en ? 0x4u : 0u) | static_cast<uint8_t>(static_cast<uint8_t>(comp_mode) & 0x3);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetTempCompCfg", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetTempCompCfg", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetNtcParams (opcode 0x0133, 7 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x01
 *   1   | 0x33
 *   2   | ntc_r_ratio[15:8]
 *   3   | ntc_r_ratio[7:0]
 *   4   | ntc_beta[15:8]
 *   5   | ntc_beta[7:0]
 *   6   | delay
 */
bool LR2021::SetNtcParams(uint16_t ntc_r_ratio, uint16_t ntc_beta, uint8_t delay) {
    constexpr size_t kFrameLen = 7;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetNtcParams);
    PackU16(tx_buf + 2, ntc_r_ratio);
    PackU16(tx_buf + 4, ntc_beta);
    tx_buf[6] = delay;

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetNtcParams", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetNtcParams", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}
