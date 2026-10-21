/**
 *******************************************************************************
 * @file  ev_hc32f4a0_lqfp176_rtl8201.h
 * @brief This file contains all the functions prototypes for rtl8201 of the
 *        board EV_HC32F4A0_LQFP176.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2023-09-30       CDT             First version
 @endverbatim
 *******************************************************************************
 * Copyright (C) 2022-2025, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */
#ifndef __BOARD_RTL8201_H__
#define __BOARD_RTL8201_H__

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "rtl8201.h"
#include "hc32_ll_eth.h"
#include "hc32_ll_utility.h"

/**
 * @addtogroup BSP
 * @{
 */

/**
 * @addtogroup EV_HC32F4A0_LQFP176
 * @{
 */

/**
 * @addtogroup EV_HC32F4A0_LQFP176_RTL8201
 * @{
 */


/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/
/**
 * @defgroup EV_HC32F4A0_LQFP176_RTL8201_Global_Macros  EV_HC32F4A0_LQFP176 RTL8201 Global Macros
 * @{
 */
/* RTL8201 device address */
#define BSP_RTL8201_DEV_ADDR            (0x01U)

/**
 * @defgroup BSP_RTL8201_SMI_Port_configure BSP RTL8201 ETH SMI Port Configure
 * @{
 */

// YT8522C
// 21
#define ETH_RESET_PORT					GPIO_PORT_E
#define ETH_RESET_PIN					GPIO_PIN_04


// Media Independent Interface
// 8
#define ETH_MII_RX_DV_PORT				GPIO_PORT_A
#define ETH_MII_RX_DV_PIN				GPIO_PIN_07
// 9
#define ETH_MII_RXD0_PORT				GPIO_PORT_C
#define ETH_MII_RXD0_PIN				GPIO_PIN_04
// 10
#define ETH_MII_RXD1_PORT				GPIO_PORT_C
#define ETH_MII_RXD1_PIN				GPIO_PIN_05
// 11
#define ETH_MII_RXD2_PORT				GPIO_PORT_B
#define ETH_MII_RXD2_PIN				GPIO_PIN_00
// 12
#define ETH_MII_RXD3_PORT				GPIO_PORT_B
#define ETH_MII_RXD3_PIN				GPIO_PIN_01
// 13
#define ETH_MII_RX_CLK_PORT				GPIO_PORT_A
#define ETH_MII_RX_CLK_PIN				GPIO_PIN_01


// 15
#define ETH_MII_TX_CLK_PORT				GPIO_PORT_C
#define ETH_MII_TX_CLK_PIN				GPIO_PIN_03
// 16
#define ETH_MII_TXD0_PORT				GPIO_PORT_B
#define ETH_MII_TXD0_PIN				GPIO_PIN_12
// 17
#define ETH_MII_TXD1_PORT				GPIO_PORT_B
#define ETH_MII_TXD1_PIN				GPIO_PIN_13
// 18
#define ETH_MII_TXD2_PORT				GPIO_PORT_C
#define ETH_MII_TXD2_PIN				GPIO_PIN_02
// 19
#define ETH_MII_TXD3_PORT				GPIO_PORT_E
#define ETH_MII_TXD3_PIN				GPIO_PIN_02
// 20
#define ETH_MII_TX_EN_PORT				GPIO_PORT_E
#define ETH_MII_TX_EN_PIN				GPIO_PIN_03

// 26
#define ETH_MII_CRS_PORT				GPIO_PORT_A
#define ETH_MII_CRS_PIN					GPIO_PIN_00
// 2
#define ETH_MII_COL_PORT				GPIO_PORT_A
#define ETH_MII_COL_PIN					GPIO_PIN_03
// 28
#define ETH_MII_RX_ER_PORT				GPIO_PORT_B
#define ETH_MII_RX_ER_PIN				GPIO_PIN_10


// Status Management Interface
// 22
#define ETH_SMI_MDC_PORT				GPIO_PORT_C
#define ETH_SMI_MDC_PIN					GPIO_PIN_01
// 23
#define ETH_SMI_MDIO_PORT				GPIO_PORT_A
#define ETH_SMI_MDIO_PIN				GPIO_PIN_02
/**
 * @}
 */

/**
 * @defgroup BSP_RTL8201_Reset_Mode BSP RTL8201 Reset Mode
 * @{
 */
#define BSP_RTL8201_RST_MD_SW           (0x01U)
#define BSP_RTL8201_RST_MD_HW           (0x02U)
/**
 * @}
 */

/**
 * @}
 */

/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/

/*******************************************************************************
  Global function prototypes (definition in C source)
 ******************************************************************************/
/**
 * @addtogroup EV_HC32F4A0_LQFP176_RTL8201_Global_Functions
 * @{
 */

int32_t BSP_RTL8201_Init(void);
int32_t BSP_RTL8201_Reset(uint16_t u16Mode);
int32_t BSP_RTL8201_AutoNegoCmd(en_functional_state_t enNewState);
int32_t BSP_RTL8201_SetLinkMode(uint32_t u32Mode);
int32_t BSP_RTL8201_SetLedMode(uint32_t u32Mode);
int32_t BSP_RTL8201_PowerSaveModeCmd(en_functional_state_t enNewState);
int32_t BSP_RTL8201_IntCmd(uint16_t u16IntType, en_functional_state_t enNewState);
int32_t BSP_RTL8201_LoopBackCmd(en_functional_state_t enNewState);
int32_t BSP_RTL8201_GetLinkStatus(uint16_t *pu16LinkStatus);
int32_t BSP_RTL8201_GetIntStatus(uint16_t u16IntType, en_flag_status_t *penStatus);

void BSP_RTL8201_IrqCallback(void);

/**
 * @}
 */


/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __EV_HC32F4A0_LQFP176_RTL8201_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
