#ifndef SHELF_USB_H
#define SHELF_USB_H


#include <stdint.h>


#define USB_ROUTER_VERSION		0x01

// (model+sn)+mac+(ip+mask+gw)+(server_ip+server_port)
#define USB_CONFIG_DATA_SIZE		(24+6+12+6+2+64)
// (chip_id+unique_id)+(ip+mask+gw)
#define USB_INFO_DATA_SIZE			(16+12+8)

#define USB_TIME_DATA_SIZE			(8)

enum {
	USB_ROUTER_TEST				= 0x00,
	USB_ROUTER_TEST_ACK			= 0x80,
	USB_ROUTER_GET_CONFIG			= 0x01,
	USB_ROUTER_GET_CONFIG_ACK		= 0x81,
	USB_ROUTER_SET_CONFIG			= 0x02,
	USB_ROUTER_SET_CONFIG_ACK		= 0x82,
	USB_ROUTER_GET_INFO			= 0x03,
	USB_ROUTER_GET_INFO_ACK		= 0x83,

	USB_ROUTER_SET_TIME			= 0x04,
	USB_ROUTER_SET_TIME_ACK		= 0x84,

	USB_ROUTER_RESET			= 0x05,

};

#define USB_RX_BUFFER_SIZE			128



#endif	// SHELF_USB_H
