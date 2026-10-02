#include "baud_store.hh"

#include <string.h>

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "host_line_coding.hh"
#include "pico/stdlib.h"

static constexpr uint32_t kStoreOffset = PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE;
static constexpr uint32_t kMagic = 0x42415544;  // "BAUD"

struct StoredBaud {
    uint32_t magic;
    uint32_t baud;
    uint32_t baud_inverted;  // ~baud: catches a torn or foreign sector.
};

extern "C" char __flash_binary_end;

uint32_t BaudStoreLoad() {
    StoredBaud stored;
    memcpy(&stored, (const void*)(XIP_BASE + kStoreOffset), sizeof(stored));
    if (stored.magic != kMagic || stored.baud != ~stored.baud_inverted) return 0;
    return IsRenegotiableBaud(stored.baud) ? stored.baud : 0;
}

void BaudStoreSave(uint32_t baud) {
    // Never erase into the image itself (the baked module firmware makes it large).
    if ((uintptr_t)&__flash_binary_end > XIP_BASE + kStoreOffset) return;
    uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));
    StoredBaud stored = {kMagic, baud, ~baud};
    memcpy(page, &stored, sizeof(stored));
    // Flash is unreadable (no XIP) while it is erased and programmed, so nothing may run from it: interrupts off.
    // ~50 ms; USB NAKs meanwhile and the UART RX FIFO can overflow, so callers pick a quiet moment.
    uint32_t irq_state = save_and_disable_interrupts();
    flash_range_erase(kStoreOffset, FLASH_SECTOR_SIZE);
    flash_range_program(kStoreOffset, page, FLASH_PAGE_SIZE);
    restore_interrupts(irq_state);
}
