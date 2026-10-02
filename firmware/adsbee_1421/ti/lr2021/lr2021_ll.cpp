// Low level functions for LR2021 driver.

#include <cstring>

#include "comms.hh"
#include "hal.hh"
#include "lr2021.hh"

bool LR2021::SPITransfer(const uint8_t* tx_buf, uint8_t* rx_buf, size_t length) {
    if (spi_handle_ == nullptr) {
        CONSOLE_ERROR("LR2021::SPITransfer", "SPI handle is null; did you forget to call Init()?");
        return false;
    }
    // Synchronous shim over the callback-mode (DMA) SPI handle: post the transfer, then spin until the
    // completion callback (LR2021::SPICallback) fires. The spin runs at thread level and is preempted by
    // the RF core's Hwi/SWI exactly like the old blocking-mode clocking, so it costs the UAT deadline
    // nothing. The stack-allocated transaction is safe because we do not return until the callback has
    // consumed it. arg = this lets the shared callback find the instance; SPICallback distinguishes shim
    // transfers from async drain frames by transaction pointer.
    SPI_Transaction txn = {
        .count = length,
        .txBuf = const_cast<uint8_t*>(tx_buf),
        .rxBuf = rx_buf,
        .arg = this,
        .status = SPI_TRANSFER_QUEUED,
    };

    // The LPF2 SPI driver accepts nullptr for txBuf (sends 0x00) and for
    // rxBuf (discards incoming bytes), matching the LR2021HAL contract.
    // Which command the Stat word coming back reports on, for error logs (status_command_opcode()).
    status_tracker_.OnFrame(FrameOpcode(tx_buf, length));

    sync_done_ = false;
    if (!SPI_transfer(spi_handle_, &txn)) {
        CONSOLE_ERROR("LR2021::SPITransfer", "SPI_transfer failed to post %u-byte transfer.",
                      static_cast<unsigned>(length));
        return false;
    }
    // The longest legal frame (258 B at 12 MHz) clocks in ~0.2 ms; a generous bound means a lost
    // callback degrades to an error return instead of a watchdog reset.
    static constexpr uint32_t kSyncTransferTimeoutMs = 5;
    uint32_t start_ms = get_time_since_boot_ms();
    while (!sync_done_) {
        if (get_time_since_boot_ms() - start_ms > kSyncTransferTimeoutMs) {
            SPI_transferCancel(spi_handle_);  // Invokes the callback with a canceled status.
            CONSOLE_ERROR("LR2021::SPITransfer", "Timed out waiting for %u-byte transfer completion.",
                          static_cast<unsigned>(length));
            return false;
        }
    }
    return sync_status_ == SPI_TRANSFER_COMPLETED;
}

// "LR2021::SetOokADSB" -> "SetOokADSB": the logs below already carry the "LR2021" tag.
static const char* WithoutClassPrefix(const char* name) {
    return strncmp(name, "LR2021::", 8) == 0 ? name + 8 : name;
}

uint16_t LR2021::FrameOpcode(const uint8_t* tx_buf, size_t length) {
    return (tx_buf != nullptr && length >= 2) ? static_cast<uint16_t>((tx_buf[0] << 8) | tx_buf[1]) : 0;
}

const char* LR2021::OpcodeName(uint16_t opcode) {
    switch (opcode) {
        case 0:
            return "none";
        case kOpcodeReadRxFifo:
            return "ReadRxFifo";
        case kOpcodeWriteTxFifo:
            return "WriteTxFifo";
        case kOpcodeGetStatus:
            return "GetStatus";
        case kOpcodeGetVersion:
            return "GetVersion";
        case kOpcodeWriteRegMem32:
            return "WriteRegMem32";
        case kOpcodeWriteRegMemMask32:
            return "WriteRegMemMask32";
        case kOpcodeReadRegMem32:
            return "ReadRegMem32";
        case kOpcodeGetErrors:
            return "GetErrors";
        case kOpcodeClearErrors:
            return "ClearErrors";
        case kOpcodeSetDioFunction:
            return "SetDioFunction";
        case kOpcodeSetDioRfSwitchConfig:
            return "SetDioRfSwitchConfig";
        case kOpcodeClearFifoIrqFlags:
            return "ClearFifoIrqFlags";
        case kOpcodeSetDioIrqConfig:
            return "SetDioIrqConfig";
        case kOpcodeClearIrq:
            return "ClearIrq";
        case kOpcodeGetAndClearIrq:
            return "GetAndClearIrq";
        case kOpcodeConfigLfClock:
            return "ConfigLfClock";
        case kOpcodeConfigClkOutputs:
            return "ConfigClkOutputs";
        case kOpcodeConfigFifoIrq:
            return "ConfigFifoIrq";
        case kOpcodeGetFifoIrqFlags:
            return "GetFifoIrqFlags";
        case kOpcodeGetRxFifoLevel:
            return "GetRxFifoLevel";
        case kOpcodeGetTxFifoLevel:
            return "GetTxFifoLevel";
        case kOpcodeClearRxFifo:
            return "ClearRxFifo";
        case kOpcodeClearTxFifo:
            return "ClearTxFifo";
        case kOpcodeSetTcxoMode:
            return "SetTcxoMode";
        case kOpcodeSetRegMode:
            return "SetRegMode";
        case kOpcodeCalibrate:
            return "Calibrate";
        case kOpcodeCalibFe:
            return "CalibFe";
        case kOpcodeGetVBat:
            return "GetVBat";
        case kOpcodeGetTemp:
            return "GetTemp";
        case kOpcodeGetRandomNumber:
            return "GetRandomNumber";
        case kOpcodeSetSleep:
            return "SetSleep";
        case kOpcodeSetStandby:
            return "SetStandby";
        case kOpcodeSetFs:
            return "SetFs";
        case kOpcodeSetAdditionalRegToRetain:
            return "SetAdditionalRegToRetain";
        case kOpcodeGetAndClearFifoIrqFlags:
            return "GetAndClearFifoIrqFlags";
        case kOpcodeSetEolConfig:
            return "SetEolConfig";
        case kOpcodeSetXoscCpTrim:
            return "SetXoscCpTrim";
        case kOpcodeSetTempCompCfg:
            return "SetTempCompCfg";
        case kOpcodeSetNtcParams:
            return "SetNtcParams";
        case kOpcodeSetRfFrequency:
            return "SetRfFrequency";
        case kOpcodeSetRxPath:
            return "SetRxPath";
        case kOpcodeSetPaConfig:
            return "SetPaConfig";
        case kOpcodeSetTxParams:
            return "SetTxParams";
        case kOpcodeSetRxTxFallbackMode:
            return "SetRxTxFallbackMode";
        case kOpcodeSetPacketType:
            return "SetPacketType";
        case kOpcodeGetPacketType:
            return "GetPacketType";
        case kOpcodeSetStopTimeout:
            return "SetStopTimeout";
        case kOpcodeResetRxStats:
            return "ResetRxStats";
        case kOpcodeGetRssiInst:
            return "GetRssiInst";
        case kOpcodeSetRx:
            return "SetRx";
        case kOpcodeSetTx:
            return "SetTx";
        case kOpcodeSetTxTestMode:
            return "SetTxTestMode";
        case kOpcodeSelPa:
            return "SelPa";
        case kOpcodeSetRxDutyCycle:
            return "SetRxDutyCycle";
        case kOpcodeSetAutoRxTx:
            return "SetAutoRxTx";
        case kOpcodeGetRxPktLength:
            return "GetRxPktLength";
        case kOpcodeSetPowerOffset:
            return "SetPowerOffset";
        case kOpcodeSetDefaultRxTxTimeout:
            return "SetDefaultRxTxTimeout";
        case kOpcodeSetTimestampSource:
            return "SetTimestampSource";
        case kOpcodeGetTimestampValue:
            return "GetTimestampValue";
        case kOpcodeSetCca:
            return "SetCca";
        case kOpcodeGetCcaResult:
            return "GetCcaResult";
        case kOpcodeSetAgcGainManual:
            return "SetAgcGainManual";
        case kOpcodeSetCadParams:
            return "SetCadParams";
        case kOpcodeSetCad:
            return "SetCad";
        case kOpcodeSetOokModulationParams:
            return "SetOokModulationParams";
        case kOpcodeSetOokPacketParams:
            return "SetOokPacketParams";
        case kOpcodeSetOokCrcParams:
            return "SetOokCrcParams";
        case kOpcodeSetOokSyncWord:
            return "SetOokSyncWord";
        case kOpcodeSetOokAddress:
            return "SetOokAddress";
        case kOpcodeGetOokRxStats:
            return "GetOokRxStats";
        case kOpcodeGetOokPacketStatus:
            return "GetOokPacketStatus";
        case kOpcodeSetOokDetector:
            return "SetOokDetector";
        case kOpcodeSetOokWhiteningParams:
            return "SetOokWhiteningParams";
        default:
            return "unknown";
    }
}

void LR2021::LogCommandStatus(const char* observer) {
    if (last_stat_.command_status == CommandStatus::kOk || last_stat_.command_status == CommandStatus::kDat) {
        return;
    }
    const uint16_t opcode = status_command_opcode();
    CONSOLE_ERROR("LR2021", "%s: %s (0x%04x) returned %s.", WithoutClassPrefix(observer), OpcodeName(opcode), opcode,
                  CommandStatusToString(last_stat_.command_status));
}

bool LR2021::SequenceStepFailed(const char* sequence, const char* step) {
    if (last_stat_.command_status != CommandStatus::kOk && last_stat_.command_status != CommandStatus::kDat) {
        // The status read while sending `step` belongs to the command before it (or to the read itself): name that.
        const uint16_t opcode = status_command_opcode();
        CONSOLE_ERROR("LR2021", "%s stopped: %s (0x%04x) returned %s (reported during %s).", WithoutClassPrefix(sequence),
                      OpcodeName(opcode), opcode, CommandStatusToString(last_stat_.command_status), step);
    } else {
        CONSOLE_ERROR("LR2021", "%s stopped during %s: no valid answer from the chip.", WithoutClassPrefix(sequence), step);
    }
    return false;
}

bool LR2021::CheckLastCommandStatus(const char* observer) {
    uint8_t tx_buf[2];
    uint8_t rx_buf[2] = {0, 0};
    PackU16(tx_buf, kOpcodeGetStatus);
    if (!BeginTransaction()) {
        CONSOLE_ERROR("LR2021", "%s: failed to begin SPI transaction for the status check.", WithoutClassPrefix(observer));
        return false;
    }
    const bool sent = SPITransfer(tx_buf, rx_buf, sizeof(tx_buf));
    EndTransaction();
    if (!sent) {
        return false;
    }
    ParseStat(static_cast<uint16_t>(rx_buf[0] << 8) | rx_buf[1]);
    LogCommandStatus(observer);
    return last_stat_.command_status == CommandStatus::kOk || last_stat_.command_status == CommandStatus::kDat;
}

void LR2021::SetNSS(bool high) { GPIO_write(config_.gpio_nss, high ? 1 : 0); }

bool LR2021::IsBusy() { return GPIO_read(config_.gpio_busy) != 0; }

void LR2021::SetEnable(bool enabled) { GPIO_write(config_.gpio_enable, enabled ? 1 : 0); }