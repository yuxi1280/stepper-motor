/**
 *******************************************************************************
 * @file  usb_dev_cdc_class.c
 * @brief The CDC VCP core functions.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
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
#include "usb_dev_cdc_class.h"
#include "usb_app_conf.h"
#include "usb_dev_driver.h"
#include "usb_dev_ctrleptrans.h"
#include "usb_dev_stdreq.h"
#include "usb_dev_desc.h"

#include "board.h"
#include "ring_buffer.h"

/**
 * @addtogroup LL_USB_LIB
 * @{
 */

/**
 * @addtogroup LL_USB_DEV_CLASS
 * @{
 */

/**
 * @addtogroup LL_USB_SINGLE_CDC USB Device CDC
 * @{
 */

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
uint8_t *usb_dev_cdc_getcfgdesc(uint16_t *length);

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
usb_dev_class_func  class_cdc_cbk = {
    &usb_dev_cdc_init,
    &usb_dev_cdc_deinit,
    &usb_dev_cdc_setup,
    NULL,
    &usb_dev_cdc_ctrlep_rxready,
    &usb_dev_cdc_getcfgdesc,
    &usb_dev_cdc_sof,
    &usb_dev_cdc_datain,
    &usb_dev_cdc_dataout,
    NULL,
    NULL,
};

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
__USB_ALIGN_BEGIN static uint8_t _usb_rx_buffer[MAX_CDC_PACKET_SIZE];

usb_packet_buffer_t _usb_rx_rb;
usb_packet_buffer_t _usb_tx_rb;
bool _usb_tx_flag;

__USB_ALIGN_BEGIN static uint8_t usb_dev_cdc_cfgdesc[USB_CDC_CONFIG_DESC_SIZ]  = {
	0x09,
	USB_CFG_DESCRIPTOR_TYPE,		// 0x02
	USB_CDC_CONFIG_DESC_SIZ, 0x00,	// wTotalLength
	0x01,							// bNumInterfaces
	0x01,							// bConfigurationValue
	0x00,							// iConfiguration: _configuration_name
	0xC0,							// bmAttributes: Bus_Powered Self_Powered
	0x32,							// MaxPower: 500mA

	/*
    0x08,
    0x0B,
    0x00,
    0x02,
    0x02,
    0x02,
    0x01,
    0x04,

    0x09,
    USB_INTERFACE_DESCRIPTOR_TYPE,
    0x00,
    0x00,
    0x01,
    0x02,
    0x02,
    0x01,
    0x00,

    0x05,
    0x24,
    0x00,
    0x10,
    0x01,

    0x05,
    0x24,
    0x01,
    0x00,
    0x01,

    0x04,
    0x24,
    0x02,
    0x02,

    0x05,
    0x24,
    0x06,
    0x00,
    0x01,

    0x07,
    USB_ENDPOINT_DESCRIPTOR_TYPE,
    CDC_CMD_EP,
    0x03,
    LOBYTE(CDC_CMD_PACKET_SIZE),
    HIBYTE(CDC_CMD_PACKET_SIZE),
    0xFF,
	*/

	0x09,
	USB_INTERFACE_DESCRIPTOR_TYPE,
	0x01,									// bInterfaceNumber
	0x00,									// bAlternateSetting
	0x02,									// bNumEndPoints
	0x0A,									// bInterfaceClass:CDC-Data
	0x00,									// bInterfaceSubClass
	0x00,									// bInterfaceProtocol
	0x00,									// iInterface: _interface2_name



	0x07,
	USB_ENDPOINT_DESCRIPTOR_TYPE,
	CDC_IN_EP,
	0x02,
	LOBYTE(MAX_CDC_PACKET_SIZE), HIBYTE(MAX_CDC_PACKET_SIZE),
	0x00,

	0x07,
	USB_ENDPOINT_DESCRIPTOR_TYPE,
	CDC_OUT_EP,
	0x02,
	LOBYTE(MAX_CDC_PACKET_SIZE), HIBYTE(MAX_CDC_PACKET_SIZE),
	0x00,
} ;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 * @brief  Initilaize the CDC application
 * @param  [in] pdev        Device instance
 * @retval None
 */
void usb_dev_cdc_init(void *pdev)
{
    usb_opendevep(pdev, CDC_IN_EP, MAX_CDC_IN_PACKET_SIZE, EP_TYPE_BULK);
    usb_opendevep(pdev, CDC_OUT_EP, MAX_CDC_OUT_PACKET_SIZE, EP_TYPE_BULK);

    usb_readytorx(pdev, CDC_OUT_EP, (uint8_t *)(_usb_rx_buffer), MAX_CDC_OUT_PACKET_SIZE);
}

/**
 * @brief  Deinitialize the CDC application
 * @param  [in] pdev        Device instance
 * @retval None
 */
void usb_dev_cdc_deinit(void *pdev)
{
    usb_shutdevep(pdev, CDC_IN_EP);
    usb_shutdevep(pdev, CDC_OUT_EP);
}

/**
 * @brief  Handle the setup requests
 * @param  [in] pdev        Device instance
 * @param  [in] req         usb requests
 * @retval status
 */
uint8_t usb_dev_cdc_setup(void *pdev, USB_SETUP_REQ *req)
{
	log_printf("usb_dev_cdc_setup\n");

	uint8_t  u8Res = USB_DEV_OK;

	return u8Res;
}

/**
 * @brief  Data received on control endpoint
 * @param  [in] pdev        device device instance
 * @retval None
 */
void usb_dev_cdc_ctrlep_rxready(void *pdev)
{
	log_printf("usb_dev_cdc_ctrlep_rxready\n");

}

/**
 * @brief  Data sent on non-control IN endpoint
 * @param  [in] pdev        Device instance
 * @param  [in] epnum       endpoint index
 * @retval None
 */

void usb_dev_cdc_datain(void *pdev, uint8_t epnum)
{
	//	log_printf("usb_dev_cdc_datain\n");
	usb_packet_buffer_remove_head(&_usb_tx_rb);

	if (usb_packet_buffer_not_empty(_usb_tx_rb)) {
		usb_packet_t *packet = usb_packet_buffer_head(&_usb_tx_rb);
		usb_deveptx(pdev, CDC_IN_EP, packet->data, packet->size);
	} else {
		_usb_tx_flag = false;
	}
}



/**
 * @brief  Data received on non-control Out endpoint
 * @param  [in] pdev        device instance
 * @param  [in] epnum       endpoint index
 * @retval None
 */
void usb_dev_cdc_dataout(void *pdev, uint8_t epnum)
{
	uint16_t n;
	(void)n;

	n = (uint16_t)((usb_core_instance *)pdev)->dev.out_ep[epnum].xfer_count;

	if (usb_packet_buffer_not_full(_usb_rx_rb)) {
		usb_packet_buffer_put(&_usb_rx_rb, _usb_rx_buffer, n);
	}

	usb_readytorx(pdev, CDC_OUT_EP, (uint8_t *)(_usb_rx_buffer), MAX_CDC_OUT_PACKET_SIZE);
}

void usb_dev_cdc_poll(void)
{
	while (usb_packet_buffer_not_empty(_usb_rx_rb)) {
		usb_packet_t *packet = usb_packet_buffer_head(&_usb_rx_rb);
		usb_dev_cdc_on_recv(packet->data, packet->size);
		usb_packet_buffer_remove_head(&_usb_rx_rb);
	}
}

/**
 * @brief  Start Of Frame event management
 * @param  [in] pdev        Device instance
 * @retval status
 */
uint8_t usb_dev_cdc_sof(void *pdev)
{
	return USB_DEV_OK;
}

bool usb_dev_cdc_send_ex(void *pdev, const uint8_t *p, uint32_t n)
{
	uint32_t i;
	uint32_t delta;

	for (i = 0; i < n; i += delta) {
		if (n - i > MAX_CDC_PACKET_SIZE) {
			delta = MAX_CDC_PACKET_SIZE;
		} else {
			delta = n - i;
		}

		if (!usb_dev_cdc_send(pdev, p+i, delta)) {
			return false;
		}
	}

	return true;
}

bool usb_dev_cdc_send(void *pdev, const uint8_t *p, uint32_t n)
{
	if (n > MAX_CDC_PACKET_SIZE) {
		return false;
	}

	if (usb_packet_buffer_not_full(_usb_tx_rb)) {
		usb_packet_t *packet = usb_packet_buffer_put(&_usb_tx_rb, p, n);
		if (_usb_tx_flag == false) {
			_usb_tx_flag = true;
			usb_deveptx(pdev, CDC_IN_EP, packet->data, packet->size);
		}
		return true;
	} else {
		return false;
	}
}



/**
 * @brief  get the configuration descriptor
 * @param  [in] length          length of configuration descriptor in bytes
 * @retval the pointer to configuration descriptor buffer
 */
uint8_t *usb_dev_cdc_getcfgdesc(uint16_t *length)
{
	log_printf("usb_dev_cdc_getcfgdesc\n");

	*length = (uint16_t)sizeof(usb_dev_cdc_cfgdesc);
	return usb_dev_cdc_cfgdesc;
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

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
