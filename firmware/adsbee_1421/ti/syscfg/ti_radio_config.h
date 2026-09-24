/*
 *  ======== ti_radio_config.h ========
 *  Configured RadioConfig module definitions
 *
 *  DO NOT EDIT - This file is generated for the CC1314R10RSK
 *  by the SysConfig tool.
 *
 *  Radio Config module version : 1.20.0
 *  SmartRF Studio data version : 2.32.0
 */
#ifndef _TI_RADIO_CONFIG_H_
#define _TI_RADIO_CONFIG_H_

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/rf_mailbox.h)
#include DeviceFamily_constructPath(driverlib/rf_common_cmd.h)
#include DeviceFamily_constructPath(driverlib/rf_prop_cmd.h)
#include <ti/drivers/rf/RF.h>

/* SmartRF Studio version that the RF data is fetched from */
#define SMARTRF_STUDIO_VERSION "2.32.0"

// *********************************************************************************
//   RF Frontend configuration
// *********************************************************************************
// RF design based on: LP_EM_CC1314R10
#define LP_EM_CC1314R10

// RF frontend configuration
#define FRONTEND_SUB1G_DIFF_RF
#define FRONTEND_SUB1G_EXT_BIAS

// Supported frequency bands
#define SUPPORT_FREQBAND_868

// TX power table size definitions
#define TXPOWERTABLE_868_PA13_SIZE 35 // 868 MHz, 13 dBm

// TX power tables
extern RF_TxPowerTable_Entry txPowerTable_868_pa13[]; // 868 MHz, 13 dBm



//*********************************************************************************
//  RF Setting:   Custom (50 kbps, 25 kHz Deviation, 2-GFSK, 100 kHz RX Bandwidth)
//
//  PHY:          custom868
//  Setting file: setting_tc106_custom.json
//*********************************************************************************

// PA table usage
#define TX_POWER_TABLE_SIZE_custom868_0 TXPOWERTABLE_868_PA13_SIZE

#define txPowerTable_custom868_0 txPowerTable_868_pa13

// TI-RTOS RF Mode object
extern RF_Mode RF_prop_custom868_0;

// RF Core API commands
extern rfc_CMD_PROP_RADIO_DIV_SETUP_t RF_cmdPropRadioDivSetup_custom868_0;
extern rfc_CMD_FS_t RF_cmdFs_custom868_0;
extern rfc_CMD_PROP_TX_t RF_cmdPropTx_custom868_0;
extern rfc_CMD_PROP_RX_t RF_cmdPropRx_custom868_0;

// RF Core API overrides
extern uint32_t pOverrides_custom868_0[];

#endif // _TI_RADIO_CONFIG_H_
