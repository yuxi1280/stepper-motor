/**
 *******************************************************************************
 * @file  eth/eth_loopback/source/ethernetif.c
 * @brief This file implements Ethernet network interface drivers.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
   2023-01-15       CDT             Delect RX_ER Pin in RMII mode
   2023-09-30       CDT             Modify the process of obtaining PHY status
                                    Add MAC interface selection before ETH_DeInit function
                                    Disable auto negotiation in low_level_init function
   2024-11-08       CDT             Strip PHY operation code into BSP file
                                    Add high driver for ETH output pin
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

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ll.h"
#include "board.h"
#include "board_rtl8201.h"

#include "ethernetif.h"
#include <string.h>

#include "lwip/opt.h"
#include "lwip/etharp.h"


/**
 * @addtogroup HC32F4A0_DDL_Examples
 * @{
 */

/**
 * @addtogroup ETH_Loopback
 * @{
 */

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/* Define those to better describe your network interface. */
#define IFNAME0                         'q'
#define IFNAME1                         'w'

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
// 发送接收数据多时需增加缓冲区数量
#include "new_memory_pool.h"

#define QW_ETH_TX_BUF_NUM			8
#define QW_ETH_RX_BUF_NUM			24		// 64*1524

/* Global Ethernet handle*/
static stc_eth_handle_t EthHandle;
/* Ethernet Tx DMA Descriptor */
__ALIGN_BEGIN static stc_eth_dma_desc_t EthDmaTxDscrTab[QW_ETH_TX_BUF_NUM];
/* Ethernet Rx DMA Descriptor */
__ALIGN_BEGIN static stc_eth_dma_desc_t EthDmaRxDscrTab[QW_ETH_RX_BUF_NUM];
/* Ethernet Transmit Buffer */
//__ALIGN_BEGIN static uint8_t EthTxBuff[QW_ETH_TX_BUF_NUM][ETH_TX_BUF_SIZE];
//__ALIGN_BEGIN static uint8_t *EthTxBuff[QW_ETH_TX_BUF_NUM];
__ALIGN_BEGIN static uint8_t *EthTxBuff;


/* Ethernet Receive Buffer */
//__ALIGN_BEGIN static uint8_t EthRxBuff[QW_ETH_RX_BUF_NUM][ETH_RX_BUF_SIZE];
//__ALIGN_BEGIN static uint8_t *EthRxBuff[QW_ETH_RX_BUF_NUM];
__ALIGN_BEGIN static uint8_t *EthRxBuff;

/* Ethernet link status */
static __IO uint8_t u8PhyLinkStatus = 0U;




/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 * @defgroup ETH_IF_Global_Functions Ethernet Interface Global Functions
 * @{
 */

uint32_t hdsc_lwip_rand(void)
{
	uint32_t r;
	TRNG_GenerateRandom(&r, 1);
	return r;
}

uint32_t sys_now(void)
{
	return SysTick_GetTick();
}

void lwip_example_app_platform_assert(const char *msg, int line, const char *file)
{
	log_printf("%s:%d:%s\n", file, line, msg);
}

/**
 * @brief  Initializes the Ethernet GPIO.
 * @param  None
 * @retval None
 */
static void Ethernet_GpioInit(void)
{
#if	0
    stc_gpio_init_t stcGpioInit;

    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDrv = PIN_HIGH_DRV;
    /* Configure MII/RMII selection IO for ETH */
#ifdef ETH_INTERFACE_RMII
    /* Ethernet RMII pins configuration */
    /**
        ETH_RMII_TX_EN --------------> PG11
        ETH_RMII_TXD0 ---------------> PG13
        ETH_RMII_TXD1 ---------------> PG14
        ETH_RMII_REF_CLK ------------> PA1
        ETH_RMII_CRS_DV -------------> PA7
        ETH_RMII_RXD0 ---------------> PC4
        ETH_RMII_RXD1 ---------------> PC5
    */
    GPIO_Init(GPIO_PORT_G, (GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14), &stcGpioInit);
    /* Configure PA1 and PA7 */
    GPIO_SetFunc(GPIO_PORT_A, (GPIO_PIN_01 | GPIO_PIN_07), GPIO_FUNC_11);
    /* Configure PC4 and PC5 */
    GPIO_SetFunc(GPIO_PORT_C, (GPIO_PIN_04 | GPIO_PIN_05), GPIO_FUNC_11);
    /* Configure PG11, PG13 and PG14 */
    GPIO_SetFunc(GPIO_PORT_G, (GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14), GPIO_FUNC_11);
#else
    /* Ethernet MII pins configuration */
    /**
        ETH_MII_TX_CLK --------------> PB6
        ETH_MII_TX_EN ---------------> PG11
        ETH_MII_TXD0 ----------------> PG13
        ETH_MII_TXD1 ----------------> PG14
        ETH_MII_TXD2 ----------------> PB9
        ETH_MII_TXD3 ----------------> PB8
        ETH_MII_RX_CLK --------------> PA1
        ETH_MII_RX_DV ---------------> PA7
        ETH_MII_RXD0 ----------------> PC4
        ETH_MII_RXD1 ----------------> PC5
        ETH_MII_RXD2 ----------------> PB0
        ETH_MII_RXD3 ----------------> PB1
        ETH_MII_RX_ER ---------------> PI10
        ETH_MII_CRS -----------------> PH2
        ETH_MII_COL -----------------> PH3
    */
    GPIO_Init(GPIO_PORT_B, (GPIO_PIN_08 | GPIO_PIN_09), &stcGpioInit);
    GPIO_Init(GPIO_PORT_G, (GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14), &stcGpioInit);
    /* Configure PA1 and PA7 */
    GPIO_SetFunc(GPIO_PORT_A, (GPIO_PIN_01 | GPIO_PIN_07), GPIO_FUNC_11);
    /* Configure PB0, PB1, PB6, PB8 and PB9 */
    GPIO_SetFunc(GPIO_PORT_B, (GPIO_PIN_00 | GPIO_PIN_01 | GPIO_PIN_06 | GPIO_PIN_08 | GPIO_PIN_09), GPIO_FUNC_11);
    /* Configure PC4 and PC5 */
    GPIO_SetFunc(GPIO_PORT_C, (GPIO_PIN_04 | GPIO_PIN_05), GPIO_FUNC_11);
    /* Configure PG11, PG13 and PG14 */
    GPIO_SetFunc(GPIO_PORT_G, (GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14), GPIO_FUNC_11);
    /* Configure PH2, PH3 */
    GPIO_SetFunc(GPIO_PORT_H, (GPIO_PIN_02 | GPIO_PIN_03), GPIO_FUNC_11);
    /* Configure PI10 */
    GPIO_SetFunc(GPIO_PORT_I, GPIO_PIN_10, GPIO_FUNC_11);
#endif
#endif

	stc_gpio_init_t gpio_init;
    (void)GPIO_StructInit(&gpio_init);
    gpio_init.u16PinDrv = PIN_HIGH_DRV;

	// MII
	GPIO_Init(ETH_MII_TX_EN_PORT, ETH_MII_TX_EN_PIN, &gpio_init);
	GPIO_Init(ETH_MII_TXD0_PORT, ETH_MII_TXD0_PIN, &gpio_init);
	GPIO_Init(ETH_MII_TXD1_PORT, ETH_MII_TXD1_PIN, &gpio_init);
	GPIO_Init(ETH_MII_TXD2_PORT, ETH_MII_TXD2_PIN, &gpio_init);
	GPIO_Init(ETH_MII_TXD3_PORT, ETH_MII_TXD3_PIN, &gpio_init);

	GPIO_SetFunc(ETH_MII_RX_DV_PORT, ETH_MII_RX_DV_PIN, GPIO_FUNC_11);
	GPIO_SetFunc(ETH_MII_RXD0_PORT, ETH_MII_RXD0_PIN, GPIO_FUNC_11);
	GPIO_SetFunc(ETH_MII_RXD1_PORT, ETH_MII_RXD1_PIN, GPIO_FUNC_11);
	GPIO_SetFunc(ETH_MII_RXD2_PORT, ETH_MII_RXD2_PIN, GPIO_FUNC_11);
	GPIO_SetFunc(ETH_MII_RXD3_PORT, ETH_MII_RXD3_PIN, GPIO_FUNC_11);
	GPIO_SetFunc(ETH_MII_RX_CLK_PORT, ETH_MII_RX_CLK_PIN, GPIO_FUNC_11);

	GPIO_SetFunc(ETH_MII_TX_CLK_PORT, ETH_MII_TX_CLK_PIN, GPIO_FUNC_11);
	GPIO_SetFunc(ETH_MII_TXD0_PORT, ETH_MII_TXD0_PIN, GPIO_FUNC_11);
	GPIO_SetFunc(ETH_MII_TXD1_PORT, ETH_MII_TXD1_PIN, GPIO_FUNC_11);
	GPIO_SetFunc(ETH_MII_TXD2_PORT, ETH_MII_TXD2_PIN, GPIO_FUNC_11);
	GPIO_SetFunc(ETH_MII_TXD3_PORT, ETH_MII_TXD3_PIN, GPIO_FUNC_11);
	GPIO_SetFunc(ETH_MII_TX_EN_PORT, ETH_MII_TX_EN_PIN, GPIO_FUNC_11);

	GPIO_SetFunc(ETH_MII_CRS_PORT, ETH_MII_CRS_PIN, GPIO_FUNC_11);
	GPIO_SetFunc(ETH_MII_COL_PORT, ETH_MII_COL_PIN, GPIO_FUNC_11);
	GPIO_SetFunc(ETH_MII_RX_ER_PORT, ETH_MII_RX_ER_PIN, GPIO_FUNC_11);

}

#include "shelf.h"

/**
 * @brief  In this function, the hardware should be initialized.
 * @param  netif                         The already initialized network interface structure for this ethernetif.
 * @retval int32_t:
 *           - LL_OK: Initialize success
 *           - LL_ERR: Initialize failed
 */
static int32_t low_level_init(struct netif *netif)
{
    int32_t i32Ret = LL_OK;
    stc_eth_init_t stcEthInit;

    /* Enable ETH clock */
    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_ETHMAC, ENABLE);
    /* Init Ethernet GPIO */
    Ethernet_GpioInit();
    /* PHY initialization */
    BSP_RTL8201_Init();
    /* Configure structure initialization */
    (void)ETH_CommStructInit(&EthHandle.stcCommInit);

	EthHandle.stcCommInit.au8MacAddr[0] = g_config._mac[0];
	EthHandle.stcCommInit.au8MacAddr[1] = g_config._mac[1];
	EthHandle.stcCommInit.au8MacAddr[2] = g_config._mac[2];
	EthHandle.stcCommInit.au8MacAddr[3] = g_config._mac[3];
	EthHandle.stcCommInit.au8MacAddr[4] = g_config._mac[4];
	EthHandle.stcCommInit.au8MacAddr[5] = g_config._mac[5];

    (void)ETH_StructInit(&stcEthInit);
#ifdef ETH_INTERFACE_RMII
    EthHandle.stcCommInit.u32Interface  = ETH_MAC_IF_RMII;
#else
    EthHandle.stcCommInit.u32Interface  = ETH_MAC_IF_MII;
#endif
    stcEthInit.stcMacInit.u32ReceiveAll = ETH_MAC_RX_ALL_ENABLE;

    /* Configure ethernet peripheral */
    ETH_Init(&EthHandle, &stcEthInit);
    /* Enable PHY loopback */
    (void)BSP_RTL8201_LoopBackCmd(ENABLE);
    /* Initialize Tx Descriptors list: Chain Mode */
//    (void)ETH_DMA_TxDescListInit(&EthHandle, EthDmaTxDscrTab, &EthTxBuff[0][0], QW_ETH_TX_BUF_NUM);
    (void)ETH_DMA_TxDescListInit(&EthHandle, EthDmaTxDscrTab, EthTxBuff, QW_ETH_TX_BUF_NUM);
    /* Initialize Rx Descriptors list: Chain Mode  */
    (void)ETH_DMA_RxDescListInit(&EthHandle, EthDmaRxDscrTab, EthRxBuff, QW_ETH_RX_BUF_NUM);

    /* set MAC hardware address length */
    netif->hwaddr_len = 6U;
    /* set MAC hardware address */
    netif->hwaddr[0] = (EthHandle.stcCommInit).au8MacAddr[0];
    netif->hwaddr[1] = (EthHandle.stcCommInit).au8MacAddr[1];
    netif->hwaddr[2] = (EthHandle.stcCommInit).au8MacAddr[2];
    netif->hwaddr[3] = (EthHandle.stcCommInit).au8MacAddr[3];
    netif->hwaddr[4] = (EthHandle.stcCommInit).au8MacAddr[4];
    netif->hwaddr[5] = (EthHandle.stcCommInit).au8MacAddr[5];

    /* maximum transfer unit */
    netif->mtu = 1500U;

#if	0
	// 注意：不能是局部变量
	static char buffer[32];
	int n;
	n = snprintf(buffer, sizeof(buffer), "%s%05d\n",
			ETH_HOST_NAME, ((uint16_t)g_config._mac[4]<<8)|g_config._mac[5]);
	(void)n;
#endif
	netif_set_hostname(netif, ETH_HOST_NAME);

    /* Configure PHY */
    BSP_RTL8201_AutoNegoCmd(DISABLE);
    BSP_RTL8201_SetLinkMode(RTL8201_LINK_100M_FULL_DUPLEX);
    BSP_RTL8201_SetLedMode(RTL8201_LED0_LINK10_ACTALL_LED1_LINK100);
#ifdef ETH_INTERFACE_RMII
    /* Disable Power Saving Mode */
    BSP_RTL8201_PowerSaveModeCmd(DISABLE);
#endif

    return i32Ret;
}

/**
 * @brief  This function should do the actual transmission of the packet.
 * @param  netif                        The network interface structure for this ethernetif.
 * @param  p                            The MAC packet to send.
 * @retval int32_t:
 *           - LL_OK: The packet could be sent
 *           - LL_ERR: The packet couldn't be sent
 */
int32_t low_level_output(struct netif *netif, struct pbuf *p)
{
    int32_t i32Ret;
    struct pbuf *q;
    uint8_t *txBuffer;
    __IO stc_eth_dma_desc_t *DmaTxDesc;
    uint32_t byteCnt;
    uint32_t frameLength = 0UL;
    uint32_t bufferOffset;
    uint32_t payloadOffset;

    DmaTxDesc = EthHandle.stcTxDesc;
    txBuffer = (uint8_t *)((EthHandle.stcTxDesc)->u32Buf1Addr);
    bufferOffset = 0UL;
    /* Copy frame from pbufs to driver buffers */
    for (q = p; q != NULL; q = q->next) {
        /* If this buffer isn't available, goto error */
        if (0UL != (DmaTxDesc->u32ControlStatus & ETH_DMA_TXDESC_OWN)) {
            i32Ret = LL_ERR;
            goto error;
        }

        /* Get bytes in current buffer */
        byteCnt = q->len;
        payloadOffset = 0UL;
        /* Check if the length of data to copy is bigger than Tx buffer size */
        while ((byteCnt + bufferOffset) > ETH_TX_BUF_SIZE) {
            /* Copy data to Tx buffer*/
            (void)memcpy((uint8_t *) & (txBuffer[bufferOffset]),
					(uint8_t *) & (((uint8_t *)q->payload)[payloadOffset]),
					(ETH_TX_BUF_SIZE - bufferOffset));
            /* Point to next descriptor */
            DmaTxDesc = (stc_eth_dma_desc_t *)(DmaTxDesc->u32Buf2NextDescAddr);
            /* Check if the buffer is available */
            if (0UL != (DmaTxDesc->u32ControlStatus & ETH_DMA_TXDESC_OWN)) {
                i32Ret = LL_ERR;
                goto error;
            }

            txBuffer = (uint8_t *)(DmaTxDesc->u32Buf1Addr);
            byteCnt = byteCnt - (ETH_TX_BUF_SIZE - bufferOffset);
            payloadOffset = payloadOffset + (ETH_TX_BUF_SIZE - bufferOffset);
            frameLength = frameLength + (ETH_TX_BUF_SIZE - bufferOffset);
            bufferOffset = 0UL;
        }
        /* Copy the remaining bytes */
        (void)memcpy((uint8_t *) & (txBuffer[bufferOffset]),
				(uint8_t *) & (((uint8_t *)q->payload)[payloadOffset]),
				byteCnt);
        bufferOffset = bufferOffset + byteCnt;
        frameLength = frameLength + byteCnt;
    }
    /* Prepare transmit descriptors to give to DMA */
    (void)ETH_DMA_SetTransFrame(&EthHandle, frameLength);
    i32Ret = LL_OK;

error:
    /* When Transmit Underflow flag is set, clear it and issue a Transmit Poll Demand to resume transmission */
    if (RESET != ETH_DMA_GetStatus(ETH_DMA_FLAG_UNS)) {
        /* Clear DMA UNS flag */
        ETH_DMA_ClearStatus(ETH_DMA_FLAG_UNS);
        /* Resume DMA transmission */
        WRITE_REG32(CM_ETH->DMA_TXPOLLR, 0UL);
    }

    return i32Ret;
}

err_t rtl8201_netif_output(struct netif *netif, struct pbuf *p)
{
#if	0
	log_printf("netif_output:%d\n", p->len);
	uint8_t *payload = (uint8_t *)p->payload;
	for (int i = 0; i < p->len; ++i) {
		log_printf("%02x ", payload[i]);
	}
	log_printf("\n");
#endif

	LINK_STATS_INC(link.xmit);

	// SNMP?

	return low_level_output(netif, p) == LL_OK ? ERR_OK : ERR_IF;
}

/**
 * @brief  Should allocate a pbuf and transfer the bytes of the incoming packet from the interface into the pbuf.
 * @param  netif                        The network interface structure for this ethernetif.
 * @retval A pbuf filled with the received packet (including MAC header) or NULL on memory error.
 */
static struct pbuf *low_level_input(struct netif *netif)
{
    struct pbuf *p = NULL;
    struct pbuf *q;
    uint32_t len;
    uint8_t *rxBuffer;
    __IO stc_eth_dma_desc_t *DmaRxDesc;
    uint32_t byteCnt;
    uint32_t bufferOffset;
    uint32_t payloadOffset;
    uint32_t i;

    /* Get received frame */
    if (ETH_DMA_GetReceiveFrame(&EthHandle) != LL_OK) {
        return NULL;
    }

    /* Obtain the size of the packet */
    len = (EthHandle.stcRxFrame).u32Len;
    rxBuffer = (uint8_t *)(EthHandle.stcRxFrame).u32Buf;
    if (len > 0UL) {
        /* Allocate a pbuf chain of pbufs from the buffer */
#if	0
        p = (struct pbuf *)malloc(sizeof(struct pbuf) + len);
        if (NULL != p) {
            p->next = NULL;
            p->payload = &((uint8_t *)p)[sizeof(struct pbuf)];
            p->len = len;
            (void)memset(p->payload, 0, p->len);
        }
#endif
		p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
    }
    if (p != NULL) {
        DmaRxDesc = (EthHandle.stcRxFrame).pstcFSDesc;
        bufferOffset = 0UL;
        for (q = p; q != NULL; q = q->next) {
            byteCnt = q->len;
            payloadOffset = 0UL;

            /* Check if the length of bytes to copy in current pbuf is bigger than Rx buffer size */
            while ((byteCnt + bufferOffset) > ETH_RX_BUF_SIZE) {
                /* Copy data to pbuf */
                (void)memcpy((uint8_t *) & (((uint8_t *)q->payload)[payloadOffset]),
						(uint8_t *) & (rxBuffer[bufferOffset]),
						(ETH_RX_BUF_SIZE - bufferOffset));
                /* Point to next descriptor */
                DmaRxDesc = (stc_eth_dma_desc_t *)(DmaRxDesc->u32Buf2NextDescAddr);
                rxBuffer = (uint8_t *)(DmaRxDesc->u32Buf1Addr);
                byteCnt = byteCnt - (ETH_RX_BUF_SIZE - bufferOffset);
                payloadOffset = payloadOffset + (ETH_RX_BUF_SIZE - bufferOffset);
                bufferOffset = 0UL;
            }

            /* Copy remaining data in pbuf */
            (void)memcpy((uint8_t *) & (((uint8_t *)q->payload)[payloadOffset]),
					(uint8_t *) & (rxBuffer[bufferOffset]),
					byteCnt);
            bufferOffset = bufferOffset + byteCnt;
        }
    }
    /* Release descriptors to DMA */
    DmaRxDesc = (EthHandle.stcRxFrame).pstcFSDesc;
    for (i = 0UL; i < (EthHandle.stcRxFrame).u32SegCount; i++) {
        DmaRxDesc->u32ControlStatus |= ETH_DMA_RXDESC_OWN;
        DmaRxDesc = (stc_eth_dma_desc_t *)(DmaRxDesc->u32Buf2NextDescAddr);
    }
    /* Clear Segment_Count */
    (EthHandle.stcRxFrame).u32SegCount = 0UL;

    /* When Rx Buffer unavailable flag is set, clear it and resume reception */
    if (RESET != ETH_DMA_GetStatus(ETH_DMA_FLAG_RUS)) {
        /* Clear DMA RUS flag */
        ETH_DMA_ClearStatus(ETH_DMA_FLAG_RUS);
        /* Resume DMA reception */
        WRITE_REG32(CM_ETH->DMA_RXPOLLR, 0UL);
    }

    return p;
}

/**
 * @brief  Should be called at the beginning of the program to set up the network interface.
 * @param  netif                        The network interface structure for this ethernetif.
 * @retval int32_t:
 *           - LL_OK: The IF is initialized
 *           - LL_ERR: The IF is uninitialized
 */
int32_t ethernetif_init(struct netif *netif)
{

	netif->name[0] = IFNAME0;
	netif->name[1] = IFNAME1;
	/* initialize the hardware */
	return low_level_init(netif);
}


err_t rtl8201_netif_init(struct netif *netif)
{
	printf("netif_init\n");
	EthTxBuff = new_mp_malloc(QW_ETH_TX_BUF_NUM*ETH_TX_BUF_SIZE);
	EthRxBuff = new_mp_malloc(QW_ETH_RX_BUF_NUM*ETH_RX_BUF_SIZE);
	printf("EthTxBuff:%p,EthRxBuff:%p\n", EthTxBuff, EthRxBuff);

    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;

	netif->linkoutput = rtl8201_netif_output;
	netif->output = etharp_output;
//	netif->output_ip6 = ethip6_output;

	netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET | NETIF_FLAG_IGMP;

	// SNMP?

    return low_level_init(netif) == LL_OK ? ERR_OK : ERR_IF;
}


/**
 * @brief  This function should be called when a packet is ready to be read from the interface.
 * @param  netif                        The network interface structure for this ethernetif.
 * @retval None
 */
void ethernetif_input(struct netif *netif)
{
    struct pbuf *p;

    /* Move received packet into a new pbuf */
    p = low_level_input(netif);
    /* No packet could be read, silently ignore this */
    if (p != NULL) {
        EthernetIF_InputCallback(netif, p);
        pbuf_free(p);
    } else {
		log_printf("p == NULL\n");
	}
}

struct pbuf *rtl8201_netif_input(struct netif *netif)
{
	struct pbuf *rx_p = low_level_input(netif);

#if	0
	if (rx_p != NULL) {
		struct pbuf *p = rx_p;
		uint8_t *payload = NULL;
		log_printf("netif_input:%d\n", rx_p->len);
		while (p != NULL) {
			payload = (uint8_t *)p->payload;
			for (int i = 0; i < p->len; ++i) {
				log_printf("%02x ", payload[i]);
			}
			log_printf("\n");

			p = p->next;
		}
	}
#endif

	return rx_p;
}

/**
 * @brief  Check the netif link status.
 * @param  netif the network interface
 * @retval None
 */
void EthernetIF_CheckLink(struct netif *netif)
{
    uint16_t u16LinkStatus;
    static uint8_t u8PreStatus = 0U;

    if (LL_OK == BSP_RTL8201_GetLinkStatus(&u16LinkStatus)) {
//		log_printf("link_status:%u\n", u16LinkStatus);
        /* Check whether the link is up or down*/
        if ((0U == u8PreStatus) && (u16LinkStatus > RTL8201_LINK_AUTO_NEGO_NOT_CPLT)) {
            u8PhyLinkStatus = ETH_LINK_UP;
            u8PreStatus = 1U;
            EthernetIF_LinkCallback(netif);
        } else if ((1U == u8PreStatus) && (u16LinkStatus <= RTL8201_LINK_AUTO_NEGO_NOT_CPLT)) {
            u8PhyLinkStatus = ETH_LINK_DOWN;
            u8PreStatus = 0U;
            EthernetIF_LinkCallback(netif);
        }
    } else {
		log_printf("BSP_RTL8201_GetLinkStatus failed\n");
	}
}

/**
 * @brief  Ethernet interface periodic handle
 * @param  netif                        The network interface
 * @retval None
 */
void EthernetIF_PeriodicHandle(struct netif *netif)
{
    uint32_t curTick;
    static uint32_t u32LinkTimer = 0UL;

    curTick = SysTick_GetTick();
    /* Check link status periodically */
    if ((curTick - u32LinkTimer) >= LINK_TIMER_INTERVAL) {
        u32LinkTimer = curTick;
        EthernetIF_CheckLink(netif);
    }
}

/**
 * @brief  Link callback function
 * @note   This function is called on change of link status to update low level driver configuration.
 * @param  netif                        The network interface
 * @retval None
 */
void EthernetIF_LinkCallback(struct netif *netif)
{
    uint16_t u16LinkStatus;
    uint32_t u32DuplexMode = ETH_MAC_DUPLEX_MD_FULL;
    uint32_t u32Speed = ETH_MAC_SPEED_100M;

    if (ETH_LINK_UP == u8PhyLinkStatus) {
        if (LL_OK == BSP_RTL8201_GetLinkStatus(&u16LinkStatus)) {
            /* Check whether the link is up or down*/
            if (u16LinkStatus > RTL8201_LINK_AUTO_NEGO_NOT_CPLT) {
                switch (u16LinkStatus) {
                    case RTL8201_LINK_100M_FULL_DUPLEX:
                        u32DuplexMode   = ETH_MAC_DUPLEX_MD_FULL;
                        u32Speed        = ETH_MAC_SPEED_100M;
                        break;
                    case RTL8201_LINK_100M_HALF_DUPLEX:
                        u32DuplexMode   = ETH_MAC_DUPLEX_MD_HALF;
                        u32Speed        = ETH_MAC_SPEED_100M;
                        break;
                    case RTL8201_LINK_10M_FULL_DUPLEX:
                        u32DuplexMode   = ETH_MAC_DUPLEX_MD_FULL;
                        u32Speed        = ETH_MAC_SPEED_10M;
                        break;
                    case RTL8201_LINK_10M_HALF_DUPLEX:
                        u32DuplexMode   = ETH_MAC_DUPLEX_MD_HALF;
                        u32Speed        = ETH_MAC_SPEED_10M;
                        break;
                }
                /* ETH MAC Re-Configuration */
                ETH_MAC_SetDuplexSpeed(u32DuplexMode, u32Speed);
                /* Restart MAC interface */
                (void)ETH_Start();
            }
        }
    } else {
        /* Stop MAC interface */
        (void)ETH_Stop();
    }
    /* Notify link status change */
    EthernetIF_NotifyLinkChange(netif);
}

/**
 * @brief  Ethernet interface periodic handle
 * @param  netif                        The network interface
 * @retval int32_t:
 *           - LL_OK: The IF is link up
 *           - LL_ERR: The IF is link down
 */
int32_t EthernetIF_IsLinkUp(struct netif *netif)
{
    return (0U != u8PhyLinkStatus) ? LL_OK : LL_ERR;
}

/**
 * @brief  Notify link status change.
 * @param  netif                        The network interface
 * @retval None
 */
__WEAKDEF void EthernetIF_NotifyLinkChange(struct netif *netif)
{
    /* This is function could be implemented in user file when the callback is needed */
}

/**
 * @brief  Input data handle callback.
 * @param  netif                        The network interface structure for this ethernetif
 * @param  p                            The MAC packet to receive
 * @retval None
 */
__WEAKDEF void EthernetIF_InputCallback(struct netif *netif, struct pbuf *p)
{
    /* This is function could be implemented in user file when the callback is needed */
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

static uint16_t s_link_status = 0;

bool link_status_changed(void)
{
	uint16_t link_status;
	if (BSP_RTL8201_GetLinkStatus(&link_status) == LL_OK) {
		if (link_status != s_link_status) {
			log_printf("link_state_changed:%u->%u\n", s_link_status, link_status);
			s_link_status = link_status;
			return true;
		}
	}

	return false;
}

bool link_is_up(void)
{
	return s_link_status == RTL8201_LINK_100M_FULL_DUPLEX
		|| s_link_status == RTL8201_LINK_100M_HALF_DUPLEX 
		|| s_link_status == RTL8201_LINK_10M_FULL_DUPLEX 
		|| s_link_status == RTL8201_LINK_10M_HALF_DUPLEX;
}

/******************************************************************************
 * EOF (not truncated)
 *****************************************************************************/
