#include <cstring>

#include "comms.hh"  // For debug logging.
#include "hal.hh"
#include "lr2021.hh"

/**
 * WriteRegMem32 (opcode 0x0104, §6.2.1, Table 6-3)
 *
 * SPI frame layout (single frame, write command):
 *
 *  Byte  | From host           | To host (MISO)
 *  ------+---------------------+---------------------------
 *   0    | Op[15:8]  = 0x01    | Stat[15:8]
 *   1    | Op[7:0]   = 0x04    | Stat[7:0]
 *   2    | Addr[23:16]         | IrqStatus[31:24]
 *   3    | Addr[15:8]          | IrqStatus[23:16]
 *   4    | Addr[7:0]           | IrqStatus[15:8]
 *   5    | Data1[31:24]        | IrqStatus[7:0]
 *   6    | Data1[23:16]        | 0x00
 *   7    | Data1[15:8]         | 0x00
 *   8    | Data1[7:0]          | 0x00
 *   ...  | ...                 | 0x00
 *   N*4+4| DataN[7:0]          | 0x00
 */
bool LR2021::WriteRegMem32(uint32_t addr, const uint32_t* data, size_t num_words) {
    if (data == nullptr || num_words == 0 || num_words > kMaxBlockWords) {
        CONSOLE_ERROR("LR2021::WriteRegMem32", "Invalid arguments: num_words=%u.", static_cast<unsigned>(num_words));
        return false;
    }

    // 2 (opcode) + 3 (addr) + num_words * 4 (data)
    constexpr size_t kHeaderLen = 5;
    const size_t tx_len = kHeaderLen + num_words * 4;

    // Stack-allocate worst-case buffers.  Maximum is 5 + 32*4 = 133 bytes.
    uint8_t tx_buf[kHeaderLen + kMaxBlockWords * 4];
    uint8_t rx_buf[kHeaderLen + kMaxBlockWords * 4];
    memset(tx_buf, 0, tx_len);

    PackU16(tx_buf + 0, kOpcodeWriteRegMem32);
    PackU24(tx_buf + 2, addr);

    for (size_t i = 0; i < num_words; ++i) {
        PackU32(tx_buf + kHeaderLen + i * 4, data[i]);
    }

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::WriteRegMem32", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, tx_len);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::WriteRegMem32", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * WriteRegMemMask32 (opcode 0x0105, §6.2.2, Table 6-4)
 *
 * SPI frame layout (single frame, masked write command):
 *
 *  Byte | From host           | To host (MISO)
 *  -----+---------------------+---------------------------
 *   0   | Op[15:8]  = 0x01    | Stat[15:8]
 *   1   | Op[7:0]   = 0x05    | Stat[7:0]
 *   2   | Addr[23:16]         | IrqStatus[31:24]
 *   3   | Addr[15:8]          | IrqStatus[23:16]
 *   4   | Addr[7:0]           | IrqStatus[15:8]
 *   5   | Mask[31:24]         | IrqStatus[7:0]
 *   6   | Mask[23:16]         | 0x00
 *   7   | Mask[15:8]          | 0x00
 *   8   | Mask[7:0]           | 0x00
 *   9   | Data[31:24]         | 0x00
 *  10   | Data[23:16]         | 0x00
 *  11   | Data[15:8]          | 0x00
 *  12   | Data[7:0]           | 0x00
 */
bool LR2021::WriteRegMemMask32(uint32_t addr, uint32_t mask, uint32_t data) {
    constexpr size_t kFrameLen = 13;  // 2 + 3 + 4 + 4
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeWriteRegMemMask32);
    PackU24(tx_buf + 2, addr);
    PackU32(tx_buf + 5, mask);
    PackU32(tx_buf + 9, data);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::WriteRegMemMask32", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::WriteRegMemMask32", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * ReadRegMem32 (opcode 0x0106, §6.2.3, Tables 6-5 / 6-6)
 *
 * Two-frame read sequence (§5.4.1.2):
 *
 * Frame 1 – command:
 *  Byte | From host           | To host
 *  -----+---------------------+---------------------------
 *   0   | Op[15:8]  = 0x01    | Stat[15:8]
 *   1   | Op[7:0]   = 0x06    | Stat[7:0]
 *   2   | Addr[23:16]         | IrqStatus[31:24]
 *   3   | Addr[15:8]          | IrqStatus[23:16]
 *   4   | Addr[7:0]           | IrqStatus[15:8]
 *   5   | Len[7:0]            | IrqStatus[7:0]
 *                  (NSS deasserted; device raises then lowers BUSY)
 *
 * Frame 2 – data readback (after BUSY deasserts):
 *  Byte    | From host           | To host
 *  --------+---------------------+---------------------------
 *   0      | 0x00                | Stat[15:8]   (CMD_DAT expected)
 *   1      | 0x00                | Stat[7:0]
 *   2      | 0x00                | Data1[31:24]
 *   3      | 0x00                | Data1[23:16]
 *   4      | 0x00                | Data1[15:8]
 *   5      | 0x00                | Data1[7:0]
 *   ...    | 0x00                | ...
 *   N*4+1  | 0x00                | DataN[7:0]
 *   N*4+2  | 0x00                | IrqStatus[31:24]  (trailing; must be clocked)
 *   N*4+3  | 0x00                | IrqStatus[23:16]
 *   N*4+4  | 0x00                | IrqStatus[15:8]
 *   N*4+5  | 0x00                | IrqStatus[7:0]
 */
bool LR2021::ReadRegMem32(uint32_t addr, uint32_t* data, size_t num_words) {
    if (data == nullptr || num_words == 0 || num_words > kMaxBlockWords) {
        CONSOLE_ERROR("LR2021::ReadRegMem32", "Invalid arguments: num_words=%u.", static_cast<unsigned>(num_words));
        return false;
    }

    // Frame 1: send the read command
    {
        constexpr size_t kFrame1Len = 6;  // 2 (opcode) + 3 (addr) + 1 (len)
        uint8_t tx_buf[kFrame1Len];
        uint8_t rx_buf[kFrame1Len];
        memset(tx_buf, 0, kFrame1Len);

        PackU16(tx_buf + 0, kOpcodeReadRegMem32);
        PackU24(tx_buf + 2, addr);
        tx_buf[5] = static_cast<uint8_t>(num_words);

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::ReadRegMem32", "Failed to begin SPI transaction.");
            return false;
        }
        SPITransfer(tx_buf, rx_buf, kFrame1Len);
        EndTransaction();

        // Parse the interim Stat; a CMD_OK here means the read was accepted.
        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    }

    // Frame 2: clock out data after BUSY deasserts
    //
    // The device raises BUSY after seeing NSS fall in frame 1 and lowers it
    // once the data is prepared (§5.4.1.2).  We poll before asserting NSS for
    // the second frame rather than relying on a fixed delay.
    {
        // 2 (stat) + num_words * 4 (data) + 4 (trailing IrqStatus)
        const size_t rx_len = 6 + num_words * 4;

        // Worst-case: 6 + 32*4 = 134 bytes.
        uint8_t rx_buf[6 + kMaxBlockWords * 4];

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::ReadRegMem32", "Failed to begin SPI transaction.");
            return false;
        }
        // tx_buf is nullptr: the HAL sends 0x00 for every clock cycle.
        SPITransfer(nullptr, rx_buf, rx_len);
        EndTransaction();

        // After a successful read the device reports CMD_DAT (0x3) to signal
        // that data follows the Stat bytes (§6.7.1).  The 4 trailing IrqStatus
        // bytes at the end of the frame must be clocked out (included in rx_len)
        // to prevent a premature NSS rise that would assert BUSY.
        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);

        if (last_stat_.command_status != CommandStatus::kDat && last_stat_.command_status != CommandStatus::kOk) {
            CONSOLE_ERROR("LR2021::ReadRegMem32", "Unexpected command status %s in frame 2.",
                          LR2021::CommandStatusToString(last_stat_.command_status));
            return false;
        }

        for (size_t i = 0; i < num_words; ++i) {
            data[i] = UnpackU32(rx_buf + 2 + i * 4);
        }
    }

    return true;
}