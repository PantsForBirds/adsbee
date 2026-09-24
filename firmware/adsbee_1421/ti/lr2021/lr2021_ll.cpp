// Low level functions for LR2021 driver.

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

void LR2021::SetNSS(bool high) { GPIO_write(config_.gpio_nss, high ? 1 : 0); }

bool LR2021::IsBusy() { return GPIO_read(config_.gpio_busy) != 0; }

void LR2021::SetEnable(bool enabled) { GPIO_write(config_.gpio_enable, enabled ? 1 : 0); }