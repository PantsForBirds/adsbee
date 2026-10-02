#pragma once

#include <stdint.h>

// The console baud rate the module last booted at (its saved rate), kept in the last 4 kB flash sector of the
// Programmer's RP2040 so a module saved at an uncommon rate is found with the first probe after a power cycle. Written
// only when the rate changes. A blank or corrupt sector, or a rate the console doesn't accept, reads as 0.
uint32_t BaudStoreLoad();
void BaudStoreSave(uint32_t baud);
