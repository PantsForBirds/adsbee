#pragma once

#include <ti/drivers/GPIO.h>

#include "stdint.h"
#include "ti_drivers_config.h"

class BSP {
   public:
    // Pins are retained for reference only, all initialization is done in the syscfg code.
    // static const uint_least8_t kSyncPin = SYNC_PIN;
    // static const uint_least8_t kSubGIRQPin = SUBG_IRQ_PIN;
    // static const uint_least8_t kSubGBiasTeeEnablePin = SUBG_BIAS_TEE_ENABLE_PIN;
    static const uint_least8_t kCoProSPIMISOPin = COPRO_SPI_POCI;
    static const uint_least8_t kCoProSPIMOSIPin = COPRO_SPI_PICO;
    static const uint_least8_t kCoProSPICLKPin = COPRO_SPI_SCLK;

    // Coprocessor SPI index is used to select which SPI peripheral to use.
    static const uint_least8_t kCoProSPIIndex = COPRO_SPI;

    static const uint_least8_t kLR2021CSPin = LR_CS;
    static const uint_least8_t kLR2021BusyPin = LR_BUSY;
    static const uint_least8_t kLR2021ResetPin = LR_RESET;
    // LR2021 DIO6 -> CC1314 DIO_12, chip-side configured as IRQ output for kIrqRxFifo. (LR2021 DIO5 ->
    // CC1314 DIO_13 is wired but unclaimed: spare.)
    static const uint_least8_t kLR2021IrqPin = LR_IRQ;

    static const uint_least8_t kSubGUARTTXPin = SUBG_UART_TX;
    static const uint_least8_t kSubGUARTRXPin = SUBG_UART_RX;
    static const uint_least8_t kSubGUARTIndex = SUBG_UART;

    static const uint_least8_t k1090LEDPin = SUBG_DIO26;
    static const uint_least8_t kSubGLEDPin = SUBG_DIO27;
    static const uint_least8_t kSyncPin = SYNC;
};

extern BSP bsp;