/**
 *******************************************************************************
 * @file  eth/eth_loopback/source/ethernetif.h
 * @brief Ethernet interface header file.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
   2024-11-08       CDT             Strip PHY operation code into BSP file
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
#ifndef __ETHERNETIF_H__
#define __ETHERNETIF_H__

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32f4xx.h"

/**
 * @addtogroup HC32F4A0_DDL_Examples
 * @{
 */

/**
 * @addtogroup ETH_Loopback
 * @{
 */

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/
/**
 * @defgroup ETH_IF_Global_Types Ethernet Interface Global Types
 * @{
 */

#include "lwip/netif.h"
#include "lwip/pbuf.h"


#if	0
/**
 * @brief Generic network interfaces Structure Definition
 */
struct netif {
    struct netif *next;                 /* pointer to next in linked list                */
    uint16_t      mtu;                  /* maximum transfer unit (in bytes)              */
    uint8_t       hwaddr_len;           /* number of bytes used in hwaddr                */
    uint8_t       hwaddr[6];            /* link level hardware address of this interface */
    char          name[2];              /* descriptive abbreviation                      */
};

/**
 * @brief Main packet buffer Structure Definition
 */
struct pbuf {
    struct pbuf *next;                  /* next pbuf in singly linked pbuf chain    */
    void        *payload;               /* pointer to the actual data in the buffer */
    uint32_t    len;                    /* length of this buffer                    */
};
#endif

/**
 * @}
 */

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/
/**
 * @defgroup ETH_IF_Global_Macros Ethernet Interface Global Macros
 * @{
 */

/* Ethernet PHY interface */
// #define ETH_INTERFACE_RMII

/* Number of milliseconds when to check for link status from PHY */
#ifndef LINK_TIMER_INTERVAL
#define LINK_TIMER_INTERVAL                     (100U)
#endif

/* ETH PHY link status */
#define ETH_LINK_DOWN                           (0U)
#define ETH_LINK_UP                             (1U)

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
 * @addtogroup ETH_IF_Global_Functions
 * @{
 */
int32_t ethernetif_init(struct netif *netif);
void    ethernetif_input(struct netif *netif);
int32_t low_level_output(struct netif *netif, struct pbuf *p);


void    EthernetIF_CheckLink(struct netif *netif);
void    EthernetIF_PeriodicHandle(struct netif *netif);
void    EthernetIF_LinkCallback(struct netif *netif);
int32_t EthernetIF_IsLinkUp(struct netif *netif);

void    EthernetIF_NotifyLinkChange(struct netif *netif);
void    EthernetIF_InputCallback(struct netif *netif, struct pbuf *p);



#include <stdbool.h>

extern err_t rtl8201_netif_init(struct netif *netif);
extern struct pbuf * rtl8201_netif_input(struct netif *netif);

extern bool link_status_changed(void);
extern bool link_is_up(void);



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

#endif /* __ETHERNETIF_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
