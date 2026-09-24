// lr2021_ook.cpp
//
// Semtech LR2021 driver – Ook firmware command implementation.
// See lr2021.hh for API documentation and datasheet references.
//
// All commands in this file use the 0x02xx opcode group (firmware commands).
// SPI framing follows the same rules as lr2021.cpp:
//
//   Write command (single frame):
//     MOSI: Op[15:8] Op[7:0] Arg0 Arg1 ...
//     MISO: Stat[15:8] Stat[7:0] (remaining bytes ignored)
//
//   Read command (two frames):
//     Frame 1 – send opcode (BUSY asserts then deasserts)
//     Frame 2 – send 0x00 bytes; device clocks out Stat + data
//               Note: 0x02xx reads do NOT append trailing IrqStatus bytes
//               (unlike the 0x01xx register-access reads in lr2021.cpp).

#include <cstring>

#include "comms.hh"
#include "lr2021.hh"

// ---------------------------------------------------------------------------
// Write commands (single SPI frame)
// ---------------------------------------------------------------------------

/**
 * SetOokModulationParams (opcode 0x0281, 8 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x81
 *   2   | bitrate[31:24]
 *   3   | bitrate[23:16]
 *   4   | bitrate[15:8]
 *   5   | bitrate[7:0]
 *   6   | pulse_shape & 0xF
 *   7   | rx_bw
 */
bool LR2021::SetOokModulationParams(uint32_t bitrate, OokPulseShape pulse_shape, OokRxBw rx_bw) {
    constexpr size_t kFrameLen = 8;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetOokModulationParams);
    PackU32(tx_buf + 2, bitrate);
    tx_buf[6] = static_cast<uint8_t>(pulse_shape) & 0xF;
    tx_buf[7] = static_cast<uint8_t>(rx_bw);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetOokModulationParams", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetOokModulationParams", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetOokModulationParamsAdv (opcode 0x0281, 9 bytes)
 *
 * Identical to SetOokModulationParams with one extra byte appended:
 *   8   | ook_depth & 0x1
 */
bool LR2021::SetOokModulationParamsAdv(uint32_t bitrate, OokPulseShape pulse_shape, OokRxBw rx_bw, OokDepth ook_depth) {
    constexpr size_t kFrameLen = 9;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetOokModulationParams);  // same opcode as basic variant
    PackU32(tx_buf + 2, bitrate);
    tx_buf[6] = static_cast<uint8_t>(pulse_shape) & 0xF;
    tx_buf[7] = static_cast<uint8_t>(rx_bw);
    tx_buf[8] = static_cast<uint8_t>(ook_depth) & 0x1;

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetOokModulationParamsAdv", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetOokModulationParamsAdv", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetOokPacketParams (opcode 0x0282, 8 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x82
 *   2   | pre_len_tx[15:8]
 *   3   | pre_len_tx[7:0]
 *   4   | (addr_comp & 0x3) << 2 | pkt_format & 0x3
 *   5   | pld_len[15:8]
 *   6   | pld_len[7:0]
 *   7   | (crc & 0xF) << 4 | encoding & 0xF
 */
bool LR2021::SetOokPacketParams(uint16_t pre_len_tx, OokAddrComp addr_comp, OokPktFormat pkt_format, uint16_t pld_len,
                                OokCrc crc, OokEncoding encoding) {
    constexpr size_t kFrameLen = 8;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetOokPacketParams);
    PackU16(tx_buf + 2, pre_len_tx);
    tx_buf[4] = static_cast<uint8_t>((static_cast<uint8_t>(addr_comp) & 0x3) << 2) | static_cast<uint8_t>(pkt_format);
    PackU16(tx_buf + 5, pld_len);
    tx_buf[7] = static_cast<uint8_t>((static_cast<uint8_t>(crc) & 0xF) << 4) | static_cast<uint8_t>(encoding);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetOokPacketParams", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetOokPacketParams", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetOokCrcParams (opcode 0x0283, 10 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x83
 *   2   | polynom[31:24]
 *   3   | polynom[23:16]
 *   4   | polynom[15:8]
 *   5   | polynom[7:0]
 *   6   | init[31:24]
 *   7   | init[23:16]
 *   8   | init[15:8]
 *   9   | init[7:0]
 */
bool LR2021::SetOokCrcParams(uint32_t polynom, uint32_t init) {
    constexpr size_t kFrameLen = 10;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetOokCrcParams);
    PackU32(tx_buf + 2, polynom);
    PackU32(tx_buf + 6, init);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetOokCrcParams", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetOokCrcParams", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetOokSyncWord (opcode 0x0284, 7 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x84
 *   2   | syncword[31:24]
 *   3   | syncword[23:16]
 *   4   | syncword[15:8]
 *   5   | syncword[7:0]
 *   6   | (bit_order & 0x1) << 7 | nb_bits & 0x7F
 */
bool LR2021::SetOokSyncWord(uint32_t syncword, OokBitOrder bit_order, uint8_t nb_bits) {
    constexpr size_t kFrameLen = 7;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetOokSyncWord);
    PackU32(tx_buf + 2, syncword);
    tx_buf[6] = static_cast<uint8_t>((static_cast<uint8_t>(bit_order) & 0x1) << 7) | (nb_bits & 0x7F);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetOokSyncWord", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetOokSyncWord", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetOokAddress (opcode 0x0285, 4 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x85
 *   2   | addr_node
 *   3   | addr_bcast
 */
bool LR2021::SetOokAddress(uint8_t addr_node, uint8_t addr_bcast) {
    constexpr size_t kFrameLen = 4;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetOokAddress);
    tx_buf[2] = addr_node;
    tx_buf[3] = addr_bcast;

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetOokAddress", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetOokAddress", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetOokDetector (opcode 0x0288, 7 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x88
 *   2   | preamble_pattern[15:8]
 *   3   | preamble_pattern[7:0]
 *   4   | pattern_length & 0xF
 *   5   | pattern_num_repeats & 0x1F
 *   6   | (sw_is_raw ? 1 : 0) << 5 | (sfd_kind & 0x1) << 4 | sfd_length & 0xF
 */
bool LR2021::SetOokDetector(uint16_t preamble_pattern, uint8_t pattern_length, uint8_t pattern_num_repeats,
                            bool sw_is_raw, OokSfdKind sfd_kind, uint8_t sfd_length) {
    constexpr size_t kFrameLen = 7;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetOokDetector);
    PackU16(tx_buf + 2, preamble_pattern);
    tx_buf[4] = pattern_length & 0xF;
    tx_buf[5] = pattern_num_repeats & 0x1F;
    tx_buf[6] = static_cast<uint8_t>((sw_is_raw ? 0x1u : 0x0u) << 5) |
                static_cast<uint8_t>((static_cast<uint8_t>(sfd_kind) & 0x1) << 4) | (sfd_length & 0xF);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetOokDetector", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetOokDetector", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * SetOokWhiteningParams (opcode 0x0289, 6 bytes)
 *
 *  Byte | MOSI
 *  -----+---------------------
 *   0   | 0x02
 *   1   | 0x89
 *   2   | (bit_idx & 0xF) << 4 | (polynom >> 8) & 0xF
 *   3   | polynom & 0xFF
 *   4   | init[15:8]
 *   5   | init[7:0]
 */
bool LR2021::SetOokWhiteningParams(uint8_t bit_idx, uint16_t polynom, uint16_t init) {
    constexpr size_t kFrameLen = 6;
    uint8_t tx_buf[kFrameLen];
    uint8_t rx_buf[kFrameLen];
    memset(tx_buf, 0, kFrameLen);

    PackU16(tx_buf + 0, kOpcodeSetOokWhiteningParams);
    tx_buf[2] = static_cast<uint8_t>((bit_idx & 0xF) << 4) | static_cast<uint8_t>((polynom >> 8) & 0xF);
    tx_buf[3] = static_cast<uint8_t>(polynom & 0xFF);
    PackU16(tx_buf + 4, init);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::SetOokWhiteningParams", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(tx_buf, rx_buf, kFrameLen);
    EndTransaction();

    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::SetOokWhiteningParams", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

// ---------------------------------------------------------------------------
// Read commands (two-frame SPI sequence)
//
// Frame 1: send 2-byte opcode; chip raises then lowers BUSY.
// Frame 2: send zeros; chip clocks out Stat (2 bytes) + data bytes.
//          0x02xx reads do NOT append trailing IrqStatus bytes.
// ---------------------------------------------------------------------------

/**
 * GetOokRxStats (opcode 0x0286)
 *
 * Frame 2 layout (8 bytes):
 *   [0:1]  Stat
 *   [2:3]  pkt_rx
 *   [4:5]  crc_error
 *   [6:7]  len_error
 */
bool LR2021::GetOokRxStats(OokRxStats* stats_out) {
    if (stats_out == nullptr) {
        CONSOLE_ERROR("LR2021::GetOokRxStats", "stats_out is null.");
        return false;
    }

    // Frame 1: issue the read request.
    {
        constexpr size_t kFrame1Len = 2;
        uint8_t tx_buf[kFrame1Len];
        uint8_t rx_buf[kFrame1Len];
        PackU16(tx_buf, kOpcodeGetOokRxStats);

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetOokRxStats", "Failed to begin SPI transaction.");
            return false;
        }
        SPITransfer(tx_buf, rx_buf, kFrame1Len);
        EndTransaction();
        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    }

    // Frame 2: clock out data after BUSY deasserts.
    {
        constexpr size_t kFrame2Len = 8;  // 2 (stat) + 6 (data)
        uint8_t rx_buf[kFrame2Len];

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetOokRxStats", "Failed to begin SPI transaction (frame 2).");
            return false;
        }
        SPITransfer(nullptr, rx_buf, kFrame2Len);
        EndTransaction();

        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
        if (last_stat_.command_status != CommandStatus::kDat && last_stat_.command_status != CommandStatus::kOk) {
            CONSOLE_ERROR("LR2021::GetOokRxStats", "Unexpected command status %s in frame 2.",
                          LR2021::CommandStatusToString(last_stat_.command_status));
            return false;
        }

        stats_out->pkt_rx = UnpackU16(rx_buf + 2);
        stats_out->crc_error = UnpackU16(rx_buf + 4);
        stats_out->len_error = UnpackU16(rx_buf + 6);
    }

    return true;
}

/**
 * GetOokRxStatsAdv (opcode 0x0286, extended 16-byte read)
 *
 * Frame 2 layout (16 bytes):
 *   [0:1]   Stat
 *   [2:3]   pkt_rx
 *   [4:5]   crc_error
 *   [6:7]   len_error
 *   [8:9]   pbl_det
 *   [10:11] sync_ok
 *   [12:13] sync_fail
 *   [14:15] timeout
 */
bool LR2021::GetOokRxStatsAdv(OokRxStatsAdv* stats_out) {
    if (stats_out == nullptr) {
        CONSOLE_ERROR("LR2021::GetOokRxStatsAdv", "stats_out is null.");
        return false;
    }

    // Frame 1: issue the read request.
    {
        constexpr size_t kFrame1Len = 2;
        uint8_t tx_buf[kFrame1Len];
        uint8_t rx_buf[kFrame1Len];
        PackU16(tx_buf, kOpcodeGetOokRxStats);

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetOokRxStatsAdv", "Failed to begin SPI transaction.");
            return false;
        }
        SPITransfer(tx_buf, rx_buf, kFrame1Len);
        EndTransaction();
        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    }

    // Frame 2: clock out data after BUSY deasserts.
    {
        constexpr size_t kFrame2Len = 16;  // 2 (stat) + 14 (data)
        uint8_t rx_buf[kFrame2Len];

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetOokRxStatsAdv", "Failed to begin SPI transaction (frame 2).");
            return false;
        }
        SPITransfer(nullptr, rx_buf, kFrame2Len);
        EndTransaction();

        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
        if (last_stat_.command_status != CommandStatus::kDat && last_stat_.command_status != CommandStatus::kOk) {
            CONSOLE_ERROR("LR2021::GetOokRxStatsAdv", "Unexpected command status %s in frame 2.",
                          LR2021::CommandStatusToString(last_stat_.command_status));
            return false;
        }

        stats_out->pkt_rx = UnpackU16(rx_buf + 2);
        stats_out->crc_error = UnpackU16(rx_buf + 4);
        stats_out->len_error = UnpackU16(rx_buf + 6);
        stats_out->pbl_det = UnpackU16(rx_buf + 8);
        stats_out->sync_ok = UnpackU16(rx_buf + 10);
        stats_out->sync_fail = UnpackU16(rx_buf + 12);
        stats_out->timeout = UnpackU16(rx_buf + 14);
    }

    return true;
}

/**
 * GetOokPacketStatus (opcode 0x0287)
 *
 * Frame 2 layout (8 bytes):
 *   [0:1]  Stat
 *   [2:3]  pkt_len
 *   [4]    rssi_avg MSBs  → rssi_avg  = (buf[4] << 1) | ((buf[6] >> 2) & 1)
 *   [5]    rssi_high MSBs → rssi_high = (buf[5] << 1) | (buf[6] & 1)
 *   [6]    packed flags:
 *            bit5 = addr_match_bcast
 *            bit4 = addr_match_node
 *            bit2 = rssi_avg LSB
 *            bit0 = rssi_high LSB
 *   [7]    lqi
 */
bool LR2021::GetOokPacketStatus(OokPacketStatus* status_out) {
    if (status_out == nullptr) {
        CONSOLE_ERROR("LR2021::GetOokPacketStatus", "status_out is null.");
        return false;
    }

    // Frame 1
    {
        constexpr size_t kFrame1Len = 2;
        uint8_t tx_buf[kFrame1Len];
        uint8_t rx_buf[kFrame1Len];
        PackU16(tx_buf, kOpcodeGetOokPacketStatus);

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetOokPacketStatus", "Failed to begin SPI transaction.");
            return false;
        }
        SPITransfer(tx_buf, rx_buf, kFrame1Len);
        EndTransaction();
        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    }

    // Frame 2
    {
        constexpr size_t kFrame2Len = 8;  // 2 (stat) + 6 (data)
        uint8_t rx_buf[kFrame2Len];

        if (!BeginTransaction()) {
            CONSOLE_ERROR("LR2021::GetOokPacketStatus", "Failed to begin SPI transaction (frame 2).");
            return false;
        }
        SPITransfer(nullptr, rx_buf, kFrame2Len);
        EndTransaction();

        ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
        if (last_stat_.command_status != CommandStatus::kDat && last_stat_.command_status != CommandStatus::kOk) {
            CONSOLE_ERROR("LR2021::GetOokPacketStatus", "Unexpected command status %s in frame 2.",
                          LR2021::CommandStatusToString(last_stat_.command_status));
            return false;
        }

        const uint8_t flags = rx_buf[6];
        status_out->pkt_len = UnpackU16(rx_buf + 2);
        status_out->rssi_avg = static_cast<uint16_t>((static_cast<uint16_t>(rx_buf[4]) << 1) | ((flags >> 2) & 0x1));
        status_out->rssi_high = static_cast<uint16_t>((static_cast<uint16_t>(rx_buf[5]) << 1) | (flags & 0x1));
        status_out->addr_match_bcast = (flags >> 5) & 0x1;
        status_out->addr_match_node = (flags >> 4) & 0x1;
        status_out->lqi = rx_buf[7];
    }

    return true;
}
