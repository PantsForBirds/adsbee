/*
 * hw_internal.h
 *
 *  Created on: Aug 31, 2017
 *      Author: a0132647
 */

#ifndef HW_INTERNAL_H_
#define HW_INTERNAL_H_

/* memmap */
#define RFC_MDM_BASE            0x40045000 // RFC_MDM
#define RFC_RFE_BASE            0x40046000 // RFC_RFE
#define RFC_S2R_BASE            0x40048000 // RFC_S2R
#define RFC_PWM_BASE            0x40040000 // RFC_PWM


/* registers */
#define RFC_MDM_O_MDMENABLE                                         0x00000000
#define RFC_MDM_O_MDMINIT                                           0x00000004
#define RFC_MDM_O_MDMPDREQ                      8
#define RFC_MDM_O_MCEPROGRAMSRC                 84
#define RFC_MDM_O_MDMSTATUS                     108
#define RFC_MDM_O_MDMAPI                        88
#define RFC_MDM_O_DEMDEBUG                      484 // Changed from 464 on CC1312

#define RFC_PWR_O_PWMCLKENABLE                  0

#define RFC_RFE_O_RFEENABLE                                         0x00000000
#define RFC_RFE_O_RFEINIT                                           0x00000004
#define RFC_RFE_O_RFEEVENT0                                         0x00000010
#define RFC_RFE_O_RFEEVENTCLR0                                      0x00000018
#define RFC_RFE_O_MAGNCTRL0                                         0x00000074
#define RFC_RFE_O_MAGNACC0                                          0x0000007C
#define RFC_RFE_O_MAGNACC1                                          0x00000080
#define RFC_RFE_O_RFEPROGRAMSRC                 28

#define RFC_RFE_O_RFEMSGBOX                     44
#define RFC_RFE_O_RFEAPI                        32
#define RFC_RFE_O_RFEPDREQ                      8
#define RFC_RFE_O_SPARE0                                            0x00000098
#define RFC_RFE_O_SPARE1                                            0x0000009C
#define RFC_RFE_O_SPARE2                                            0x000000A0
#define RFC_RFE_O_SPARE4                                            0x000000A8

#define RFC_RFE_O_RFGAIN                                            0x00000094

#define RFC_RAT_O_RATCFG                                            0x00000000

#define RFC_S2R_O_S2RCFG                                            0x00000000
#define RFC_S2R_O_S2RSTART                                          0x00000004
#define RFC_S2R_O_S2RSTOP                                           0x00000008
#define RFC_S2R_O_S2RON                                             0x0000000C


/* bits */
#define RFC_RAT_RATCFG_TIMEN_M                                      0x00000001
#define RFC_S2R_S2RON_ADDRCNT_S                                             16


#define RFC_RFE_O_RSSIVAL                       108
#define RFC_RFE_O_RSSIMAXVAL                    112
#define RFC_RFE_O_MATHACCELIN                   132

#endif /* HW_INTERNAL_H_ */
