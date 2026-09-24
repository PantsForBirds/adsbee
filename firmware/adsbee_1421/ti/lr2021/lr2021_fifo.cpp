// lr2021_fifo.cpp
//
// Semtech LR2021 driver – FIFO data transfer implementation.
// See lr2021.hh for API documentation and datasheet references.
//
// Unlike the command functions in lr2021_system.cpp, WriteTxFifo and
// ReadRxFifo use streaming single-frame SPI transfers (§5.4.1.1):
//
//   WriteTxFifo (opcode 0x0002, write command – single frame):
//     MOSI: Op[15:8]  Op[7:0]  data[0]  data[1] ...  data[N-1]
//     MISO: Stat[15:8] Stat[7:0] IrqStat[31:0]  0x00 ...
//
//   ReadRxFifo (opcode 0x0001, streaming read – single frame):
//     MOSI: Op[15:8]  Op[7:0]  0x00  0x00 ...  0x00
//     MISO: Stat[15:8] Stat[7:0] data[0]  data[1] ...  data[N-1]
//
// NSS remains asserted throughout the opcode and payload phases, so two
// SPITransfer calls are made within a single BeginTransaction/EndTransaction
// pair – first for the 2-byte opcode (which returns the Stat word), then for
// the variable-length payload.

#include <cstring>

#include "comms.hh"
#include "lr2021.hh"

/**
 * WriteTxFifo (opcode 0x0002)
 *
 * Streams len bytes from buffer into the TX FIFO in a single NSS frame.
 * The Stat word is captured while the opcode is being clocked; the payload
 * follows immediately without releasing NSS.
 *
 * @param[in] buffer  Source byte array.
 * @param[in] len     Number of bytes to write (must be > 0 and ≤ kRxFifoMaxDepthBytes).
 * @retval true   Stat indicated CMD_OK.
 * @retval false  Invalid arguments, SPI error, or device reported non-OK status.
 */
bool LR2021::WriteTxFifo(const uint8_t* buffer, size_t len) {
    if (buffer == nullptr || len == 0 || len > kRxFifoMaxDepthBytes) {
        CONSOLE_ERROR("LR2021::WriteTxFifo", "Invalid arguments: len=%u.", static_cast<unsigned>(len));
        return false;
    }

    // Frame phase 1: send 2-byte opcode, capture Stat in MISO.
    uint8_t opcode_tx[2];
    uint8_t opcode_rx[2];
    memset(opcode_tx, 0, sizeof(opcode_tx));
    PackU16(opcode_tx, kOpcodeWriteTxFifo);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::WriteTxFifo", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(opcode_tx, opcode_rx, sizeof(opcode_tx));
    ParseStat(static_cast<uint16_t>(opcode_rx[0] << 8) | opcode_rx[1]);

    // Frame phase 2: stream payload bytes within the same NSS frame.
    // MISO carries IrqStat and trailing zeros which are not needed here.
    SPITransfer(buffer, nullptr, len);
    EndTransaction();

    if (last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::WriteTxFifo", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
    }
    return last_stat_.command_status == CommandStatus::kOk;
}

/**
 * ReadRxFifo (opcode 0x0001)
 *
 * Streams len bytes out of the RX FIFO into buffer in a single NSS frame.
 * The Stat word is captured while the opcode is being clocked; the FIFO
 * payload follows immediately without releasing NSS.
 *
 * @param[out] buffer  Destination byte array; must be at least len bytes.
 * @param[in]  len     Number of bytes to read (must be > 0 and ≤ kRxFifoMaxDepthBytes).
 * @retval true   Stat indicated CMD_OK or CMD_DAT and data was received.
 * @retval false  Invalid arguments, SPI error, or device reported an error status.
 */
bool LR2021::ReadRxFifo(uint8_t* buffer, size_t len) {
    if (buffer == nullptr || len == 0 || len > kRxFifoMaxDepthBytes) {
        CONSOLE_ERROR("LR2021::ReadRxFifo", "Invalid arguments: len=%u.", static_cast<unsigned>(len));
        return false;
    }

    // Frame phase 1: send 2-byte opcode, capture Stat in MISO.
    uint8_t opcode_tx[2];
    uint8_t opcode_rx[2];
    memset(opcode_tx, 0, sizeof(opcode_tx));
    PackU16(opcode_tx, kOpcodeReadRxFifo);

    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021::ReadRxFifo", "Failed to begin SPI transaction.");
        return false;
    }
    SPITransfer(opcode_tx, opcode_rx, sizeof(opcode_tx));
    ParseStat(static_cast<uint16_t>(opcode_rx[0] << 8) | opcode_rx[1]);

    // Frame phase 2: clock out zeros on MOSI; device returns FIFO bytes on MISO.
    SPITransfer(nullptr, buffer, len);
    EndTransaction();

    if (last_stat_.command_status != CommandStatus::kDat && last_stat_.command_status != CommandStatus::kOk) {
        CONSOLE_ERROR("LR2021::ReadRxFifo", "Unexpected command status %s.",
                      LR2021::CommandStatusToString(last_stat_.command_status));
        return false;
    }
    return true;
}
