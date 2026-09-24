/*
 *  ======== ti_drivers_config.h ========
 *  Configured TI-Drivers module declarations
 *
 *  The macros defines herein are intended for use by applications which
 *  directly include this header. These macros should NOT be hard coded or
 *  copied into library source code.
 *
 *  Symbols declared as const are intended for use with libraries.
 *  Library source code must extern the correct symbol--which is resolved
 *  when the application is linked.
 *
 *  DO NOT EDIT - This file is generated for the CC1314R10RSK
 *  by the SysConfig tool.
 */
#ifndef ti_drivers_config_h
#define ti_drivers_config_h

#define CONFIG_SYSCONFIG_PREVIEW

#define CONFIG_CC1314R10RSK
#ifndef DeviceFamily_CC13X4
#define DeviceFamily_CC13X4
#endif

#include <ti/devices/DeviceFamily.h>

#include <stdint.h>

/* support C++ sources */
#ifdef __cplusplus
extern "C" {
#endif


/*
 *  ======== CCFG ========
 */


/*
 *  ======== GPIO ========
 */
extern const uint_least8_t SUBG_DIO26_CONST;
#define SUBG_DIO26 26

extern const uint_least8_t LR_CS_CONST;
#define LR_CS 11

extern const uint_least8_t LR_BUSY_CONST;
#define LR_BUSY 14

extern const uint_least8_t LR_RESET_CONST;
#define LR_RESET 15

extern const uint_least8_t RSUBG_LED_CONST;
#define RSUBG_LED 1

extern const uint_least8_t SUBG_DIO27_CONST;
#define SUBG_DIO27 27

extern const uint_least8_t SYNC_CONST;
#define SYNC 5

extern const uint_least8_t LR_IRQ_CONST;
#define LR_IRQ 12

/* Owned by COPRO_SPI as  */
extern const uint_least8_t COPRO_SPI_SCLK_CONST;
#define COPRO_SPI_SCLK 10

/* Owned by COPRO_SPI as  */
extern const uint_least8_t COPRO_SPI_POCI_CONST;
#define COPRO_SPI_POCI 8

/* Owned by COPRO_SPI as  */
extern const uint_least8_t COPRO_SPI_PICO_CONST;
#define COPRO_SPI_PICO 9

/* Owned by SUBG_UART as  */
extern const uint_least8_t SUBG_UART_TX_CONST;
#define SUBG_UART_TX 3

/* Owned by SUBG_UART as  */
extern const uint_least8_t SUBG_UART_RX_CONST;
#define SUBG_UART_RX 2

/* The range of pins available on this device */
extern const uint_least8_t GPIO_pinLowerBound;
extern const uint_least8_t GPIO_pinUpperBound;

/* LEDs are active high */
#define CONFIG_GPIO_LED_ON  (1)
#define CONFIG_GPIO_LED_OFF (0)

#define CONFIG_LED_ON  (CONFIG_GPIO_LED_ON)
#define CONFIG_LED_OFF (CONFIG_GPIO_LED_OFF)




/*
 *  ======== SPI ========
 */

/*
 *  PICO: DIO9
 *  POCI: DIO8
 *  SCLK: DIO10
 */
extern const uint_least8_t              COPRO_SPI_CONST;
#define COPRO_SPI                       0
#define CONFIG_TI_DRIVERS_SPI_COUNT     1


/*
 *  ======== UART2 ========
 */

/*
 *  TX: DIO3
 *  RX: DIO2
 */
extern const uint_least8_t                  SUBG_UART_CONST;
#define SUBG_UART                           0
#define CONFIG_TI_DRIVERS_UART2_COUNT       1


/*
 *  ======== Board_init ========
 *  Perform all required TI-Drivers initialization
 *
 *  This function should be called once at a point before any use of
 *  TI-Drivers.
 */
extern void Board_init(void);

/*
 *  ======== Board_initGeneral ========
 *  (deprecated)
 *
 *  Board_initGeneral() is defined purely for backward compatibility.
 *
 *  All new code should use Board_init() to do any required TI-Drivers
 *  initialization _and_ use <Driver>_init() for only where specific drivers
 *  are explicitly referenced by the application.  <Driver>_init() functions
 *  are idempotent.
 */
#define Board_initGeneral Board_init

#ifdef __cplusplus
}
#endif

#endif /* include guard */
