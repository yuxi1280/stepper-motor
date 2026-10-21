/**
 *******************************************************************************
 * @file  usb/usb_dev_cdc/source/usb_bsp.c
 * @brief BSP function for USB example
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
   2022-10-31       CDT             Add USB core ID select function
 @endverbatim
 *******************************************************************************
 * Copyright (C) 2022-2023, Xiaohua Semiconductor Co., Ltd. All rights reserved.
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
#include "usb_bsp.h"
#include "hc32_ll.h"
#include "usb_dev_int.h"

#include "board.h"

/**
 * @addtogroup HC32F4A0_DDL_Applications
 * @{
 */

/**
 * @addtogroup USB_Dev_Cdc
 * @{
 */

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/* unlock/lock peripheral */
#define EXAMPLE_PERIPH_WE               (LL_PERIPH_GPIO | LL_PERIPH_EFM | LL_PERIPH_FCG | \
                                         LL_PERIPH_PWC_CLK_RMU | LL_PERIPH_SRAM)
#define EXAMPLE_PERIPH_WP               (LL_PERIPH_EFM | LL_PERIPH_FCG | LL_PERIPH_SRAM)


/* USBHS Core, embedded PHY */
#if	0
#define USBH_DP_PORT            (GPIO_PORT_B)
#define USBH_DP_PIN             (GPIO_PIN_15)
#define USBH_DM_PORT            (GPIO_PORT_B)
#define USBH_DM_PIN             (GPIO_PIN_14)
#define USBH_VBUS_PORT          (GPIO_PORT_B)
#define USBH_VBUS_PIN           (GPIO_PIN_13)
#endif
//#define USBH_SOF_PORT           (GPIO_PORT_A)
//#define USBH_SOF_PIN            (GPIO_PIN_04)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
extern usb_core_instance g_usb_dev;

/**
 * @brief  handle the USB interrupt
 * @param  None
 * @retval None
 */
static void usb_irq_handler(void)
{
	usb_isr_handler(&g_usb_dev);
}

/**
 * @brief  initialize configurations for the BSP
 * @param  [in] pdev                device instance
 * @param  [in] pstcPortIdentify    usb core and phy select
 * @retval None
 */
/*
 * 芯片内部集成全速 PHY 仅支持全速（FS，12Mb/s）和低速（LS，1.5Mb/s）传输，
 * 通过 ULPI 接口外接片外高速 PHY 则支持高速（HS, 480Mb/s）、全速（FS，12Mb/s）和低速（LS，1.5Mb/s）传输。
*/
void usb_bsp_init(usb_core_instance *pdev, stc_usb_port_identify *pstcPortIdentify)
{
	stc_gpio_init_t gpio_init;
	log_printf("usb_bsp_init\n");

	/* Unlock peripherals or registers */
	LL_PERIPH_WE(EXAMPLE_PERIPH_WE);

	/* USB clock source configurate */
	// 240MHz/5=48MHz
	CLK_SetUSBClockSrc(CLK_USBCLK_SYSCLK_DIV5);

	(void)GPIO_StructInit(&gpio_init);

	if (USBFS_CORE_ID == pstcPortIdentify->u8CoreID) {
		log_printf("usbfs\n");
		// USB FS
		gpio_init.u16PinAttr = PIN_ATTR_ANALOG;
		GPIO_Init(USB_DEVICE_DM_PORT, USB_DEVICE_DM_PIN, &gpio_init);
		GPIO_Init(USB_DEVICE_DP_PORT, USB_DEVICE_DP_PIN, &gpio_init);
		GPIO_SetFunc(USB_DEVICE_VBUS_PORT, USB_DEVICE_VBUS_PIN, GPIO_FUNC_10);

		FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USBFS, ENABLE);
	} else {
		log_printf("usbhs\n");
		if (USBHS_PHY_EMBED == pstcPortIdentify->u8PhyType) {
			// 内部PHY模块
			gpio_init.u16PinAttr = PIN_ATTR_ANALOG;
			GPIO_Init(USB_DEVICE_DM_PORT, USB_DEVICE_DM_PIN, &gpio_init);
			GPIO_Init(USB_DEVICE_DP_PORT, USB_DEVICE_DP_PIN, &gpio_init);
			GPIO_SetFunc(USB_DEVICE_VBUS_PORT, USB_DEVICE_VBUS_PIN, GPIO_FUNC_12);
		} else {
			// 外部PHY芯片

		}

		FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USBHS, ENABLE);
	}
}

/**
 * @brief  configure the NVIC of USB
 * @param  [in] pdev                    device instance
 * @retval None
 */
void usb_bsp_nvicconfig(usb_core_instance *pdev)
{
	stc_irq_signin_config_t stcIrqRegiConf;
	/* Register INT_SRC_USBFS_GLB Int to Vect.No.030 */
	stcIrqRegiConf.enIRQn = USB_DEVICE_IRQn;

	/* Select interrupt function */
	stcIrqRegiConf.enIntSrc = USB_DEVICE_INT_SRC;

	/* Callback function */
	stcIrqRegiConf.pfnCallback = &usb_irq_handler;
	/* Registration IRQ */
	INTC_IrqSignIn(&stcIrqRegiConf);
	/* Clear Pending */
	NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
	/* Set priority */
	NVIC_SetPriority(stcIrqRegiConf.enIRQn, USB_DEVICE_IRQ_PRIO);
	/* Enable NVIC */
	NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
}

/**
 * @brief  This function provides delay time in micro sec
 * @param  [in] usec        Value of delay required in micro sec
 * @retval None
 */
void usb_udelay(const uint32_t usec)
{
	__IO uint32_t i;
	uint32_t j;
	j = (HCLK_VALUE + 1000000UL - 1UL) / 1000000UL * usec;
	for (i = 0UL; i < j; i++) {
	}
}

/**
 * @brief  This function provides delay time in milli sec
 * @param  [in] msec        Value of delay required in milli sec
 * @retval None
 */
void usb_mdelay(const uint32_t msec)
{
	usb_udelay(msec * 1000UL);
}

/**
 * @}
 */

/**
 * @}
 */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
