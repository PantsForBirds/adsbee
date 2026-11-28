#pragma once
// Code and tables that core 1 runs for every Mode S frame (decode, CRC, duplicate filter). On the RP2040 they live in
// RAM: both cores share the 16 kB flash cache, and core 0's reporting code evicts them under load.
#ifdef ON_PICO
#include "pico.h"
#define CORE1_RAM_FUNC(name) __not_in_flash_func(name)
#define CORE1_RAM_DATA __not_in_flash("core1_tables")
#else
#define CORE1_RAM_FUNC(name) name
#define CORE1_RAM_DATA
#endif
