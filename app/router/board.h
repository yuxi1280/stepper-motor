#ifndef BOARD_H
#define BOARD_H


#include "hc32_ll.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// #define HARDWARE_VERSION		1
#define HARDWARE_VERSION		2


#define DEBUG

#if	defined(DEBUG)
#define log_printf(fmt, ...)	printf((fmt), ##__VA_ARGS__)
#else
#define log_printf(fmt, ...)
#endif


// 240MHz,2MiB FLASH,516KiB SRAM(128KiB SRAMH)

extern void peripheral_we(void);
extern void peripheral_wp(void);


/** 时钟 */
#define XTAL_PORT						(GPIO_PORT_H)
#define XTAL_PINS						(GPIO_PIN_00 | GPIO_PIN_01)

#define XTAL32_PORT						(GPIO_PORT_C)
#define XTAL32_PINS						(GPIO_PIN_14|GPIO_PIN_15)


extern void clk_init(void);
extern int xtal32_init(void);



/** 调试LED */
#if	HARDWARE_VERSION == 1
#define DEBUG_LED_PORT				GPIO_PORT_D
#define DEBUG_LED_PIN				GPIO_PIN_00

extern void debug_led_io_init(void);

#define debug_led_off()\
	GPIO_ResetPins(DEBUG_LED_PORT, DEBUG_LED_PIN)
#define debug_led_on()\
	GPIO_SetPins(DEBUG_LED_PORT, DEBUG_LED_PIN)

#endif


/** 状态LED */
#define STATE1_LED_PORT			GPIO_PORT_D
#define STATE1_LED_PIN			GPIO_PIN_01

#define STATE2_LED_PORT			GPIO_PORT_D
#define STATE2_LED_PIN			GPIO_PIN_02


/** 通信LED */
#define STATE3_LED_PORT			GPIO_PORT_D
#define STATE3_LED_PIN			GPIO_PIN_03

#define STATE4_LED_PORT			GPIO_PORT_D
#define STATE4_LED_PIN			GPIO_PIN_04

#define STATE5_LED_PORT			GPIO_PORT_D
#define STATE5_LED_PIN			GPIO_PIN_05

#define STATE6_LED_PORT			GPIO_PORT_D
#define STATE6_LED_PIN			GPIO_PIN_06

extern void led_io_init(void);

#define state1_led_off()\
	GPIO_ResetPins(STATE1_LED_PORT, STATE1_LED_PIN)
#define state1_led_on()\
	GPIO_SetPins(STATE1_LED_PORT, STATE1_LED_PIN)

#define state2_led_off()\
	GPIO_ResetPins(STATE2_LED_PORT, STATE2_LED_PIN)
#define state2_led_on()\
	GPIO_SetPins(STATE2_LED_PORT, STATE2_LED_PIN)

#define state3_led_off()\
	GPIO_ResetPins(STATE3_LED_PORT, STATE3_LED_PIN)
#define state3_led_on()\
	GPIO_SetPins(STATE3_LED_PORT, STATE3_LED_PIN)

#define state4_led_off()\
	GPIO_ResetPins(STATE4_LED_PORT, STATE4_LED_PIN)
#define state4_led_on()\
	GPIO_SetPins(STATE4_LED_PORT, STATE4_LED_PIN)

#define state5_led_off()\
	GPIO_ResetPins(STATE5_LED_PORT, STATE5_LED_PIN)
#define state5_led_on()\
	GPIO_SetPins(STATE5_LED_PORT, STATE5_LED_PIN)

#define state6_led_off()\
	GPIO_ResetPins(STATE6_LED_PORT, STATE6_LED_PIN)
#define state6_led_on()\
	GPIO_SetPins(STATE6_LED_PORT, STATE6_LED_PIN)



#if	HARDWARE_VERSION == 2
/** 按键 */
#define	KEY1_PORT						GPIO_PORT_D
#define KEY1_PIN						GPIO_PIN_14
#define KEY1_EXTINT_CH					EXTINT_CH14
#define KEY1_INT_SRC					INT_SRC_PORT_EIRQ14
#define KEY1_INT_IRQn					INT011_IRQn
#define KEY1_IRQ_PRIO					DDL_IRQ_PRIO_DEFAULT

#define KEY2_PORT						GPIO_PORT_D
#define KEY2_PIN						GPIO_PIN_15
#define KEY2_EXTINT_CH					EXTINT_CH15
#define KEY2_INT_SRC					INT_SRC_PORT_EIRQ15
#define KEY2_INT_IRQn					INT012_IRQn
#define KEY2_IRQ_PRIO					DDL_IRQ_PRIO_DEFAULT


extern void key_io_init(void);


#endif



/** 调试串口 */
/** RS232串口 */
// FG1,Func38 -> USART4_TX
#define DEBUG_USART_TX_PORT				GPIO_PORT_A
#define DEBUG_USART_TX_PIN				GPIO_PIN_06
#define DEBUG_USART_TX_FUNC				GPIO_FUNC_38

// FG1,Func39 -> USART4_RX
#define DEBUG_USART_RX_PORT				GPIO_PORT_A
#define DEBUG_USART_RX_PIN				GPIO_PIN_05
#define DEBUG_USART_RX_FUNC				GPIO_FUNC_39

#define DEBUG_USART_UNIT				CM_USART4
//#define DEBUG_USART_BAUDRATE			115200UL
//#define DEBUG_USART_BAUDRATE			1152000UL
#define DEBUG_USART_BAUDRATE			3000000UL

#define DEBUG_USART_FCG_ENABLE()		FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_USART4, ENABLE)


#define DEBUG_USART_RX_ERR_IRQn			INT000_IRQn
#define DEBUG_USART_RX_ERR_INT_SRC		INT_SRC_USART4_EI

#define DEBUG_USART_RX_FULL_IRQn		INT001_IRQn
#define DEBUG_USART_RX_FULL_INT_SRC		INT_SRC_USART4_RI


extern bool debug_usart_init(void);
extern void debug_usart_tx_byte(int c);

/** RS485串口1 */
// FG3,Func32 -> USART3_TX
#define RS485_1_USART_TX_PORT			GPIO_PORT_E
#define RS485_1_USART_TX_PIN			GPIO_PIN_11
#define RS485_1_USART_TX_FUNC			GPIO_FUNC_32

// FG3,Func33 -> USART3_RX
#define RS485_1_USART_RX_PORT			GPIO_PORT_E
#define RS485_1_USART_RX_PIN			GPIO_PIN_10
#define RS485_1_USART_RX_FUNC			GPIO_FUNC_33

#define RS485_1_CTRL_PORT				GPIO_PORT_E
#define RS485_1_CTRL_PIN				GPIO_PIN_12


#define RS485_1_USART_UNIT				CM_USART3


/** RS485串口2 */
// FG2,Func34 -> USART5_TX
#define RS485_2_USART_TX_PORT			GPIO_PORT_E
#define RS485_2_USART_TX_PIN			GPIO_PIN_08
#define RS485_2_USART_TX_FUNC			GPIO_FUNC_32

// FG2,Func35 -> USART5_RX
#define RS485_2_USART_RX_PORT			GPIO_PORT_E
#define RS485_2_USART_RX_PIN			GPIO_PIN_07
#define RS485_2_USART_RX_FUNC			GPIO_FUNC_33

#define RS485_2_CTRL_PORT				GPIO_PORT_E
#define RS485_2_CTRL_PIN				GPIO_PIN_09


#define RS485_2_USART_UNIT				CM_USART5


/** CAN1 */
// FG3,Func60 -> CAN1_TX
#define CAN1_TX_PORT					GPIO_PORT_B
#define CAN1_TX_PIN						GPIO_PIN_11
#define CAN1_TX_FUNC					GPIO_FUNC_60

// FG3,Func61 -> CAN1_RX
#define CAN1_RX_PORT					GPIO_PORT_E
#define CAN1_RX_PIN						GPIO_PIN_15
#define CAN1_RX_FUNC					GPIO_FUNC_61

#define CAN1_IRQn						INT002_IRQn
#define CAN1_INT_SRC					INT_SRC_CAN1_HOST

extern void can1_io_init(void);
extern void can1_interrupt_init(void);
extern void can1_poll(void);

/** CAN2 */
// FG3,Func62 -> CAN2_TX
#define CAN2_TX_PORT					GPIO_PORT_E
#define CAN2_TX_PIN						GPIO_PIN_14
#define CAN2_TX_FUNC					GPIO_FUNC_62

// FG3,Func63 -> CAN2_RX
#define CAN2_RX_PORT					GPIO_PORT_E
#define CAN2_RX_PIN						GPIO_PIN_13
#define CAN2_RX_FUNC					GPIO_FUNC_63

#define CAN2_IRQn						INT003_IRQn
#define CAN2_INT_SRC					INT_SRC_CAN2_HOST

extern void can2_io_init(void);
extern void can2_interrupt_init(void);
extern void can2_poll(void);



/** USB_DEVICE */
#define USB_DEVICE_VBUS_PORT			GPIO_PORT_A
#define USB_DEVICE_VBUS_PIN				GPIO_PIN_09

#define USB_DEVICE_DM_PORT				GPIO_PORT_A
#define USB_DEVICE_DM_PIN				GPIO_PIN_11
#define USB_DEVICE_DP_PORT				GPIO_PORT_A
#define USB_DEVICE_DP_PIN				GPIO_PIN_12


#define USB_DEVICE_IRQn					INT030_IRQn
//#define USB_DEVICE_INT_SRC				INT_SRC_USBHS_GLB
#define USB_DEVICE_INT_SRC				INT_SRC_USBFS_GLB

#define USB_DEVICE_IRQ_PRIO				DDL_IRQ_PRIO_15



/** USB_HOST */
#define USB_HOST_VBUS_EN_PORT			GPIO_PORT_C
#define USB_HOST_VBUS_EN_PIN			GPIO_PIN_06

#define USB_HOST_OC_PORT				GPIO_PORT_C
#define USB_HOST_OC_PIN					GPIO_PIN_07

#define USB_HOST_DM_PORT				GPIO_PORT_B
#define USB_HOST_DM_PIN					GPIO_PIN_14
#define USB_HOST_DP_PORT				GPIO_PORT_B
#define USB_HOST_DP_PIN					GPIO_PIN_15


#define USB_HOST_IRQn					INT031_IRQn
#define USB_HOST_INT_SRC				INT_SRC_USBHS_GLB

#define USB_HOST_IRQ_PRIO				DDL_IRQ_PRIO_15


/** WIFI */
#if HARDWARE_VERSION == 2

#define WIFI_EN_PORT					GPIO_PORT_D
#define WIFI_EN_PIN						GPIO_PIN_07

#define wifi_enable()\
	GPIO_SetPins(WIFI_EN_PORT, WIFI_EN_PIN)
#define wifi_disable()\
	GPIO_ResetPins(WIFI_EN_PORT, WIFI_EN_PIN)

#define WIFI_INT_PORT					GPIO_PORT_B
#define WIFI_INT_PIN					GPIO_PIN_02

#define WIFI_USART_TX_PORT				GPIO_PORT_D
#define WIFI_USART_TX_PIN				GPIO_PIN_11
#define WIFI_USART_TX_FUNC				GPIO_FUNC_32	// USART1_TX

#define WIFI_USART_RX_PORT				GPIO_PORT_D
#define WIFI_USART_RX_PIN				GPIO_PIN_10
#define WIFI_USART_RX_FUNC				GPIO_FUNC_33	// USART1_RX

#if	0
#define WIFI_USART_RTS_PORT				GPIO_PORT_B
#define WIFI_USART_RTS_PIN				GPIO_PIN_14
#define WIFI_USART_CTS_PORT				GPIO_PORT_B
#define WIFI_USART_CTS_PIN				GPIO_PIN_15
#endif

#define WIFI_USART_UNIT					CM_USART1
//#define WIFI_USART_BAUDRATE				115200UL
#define WIFI_USART_BAUDRATE				3000000UL
#define WIFI_USART_FCG_ENABLE()			FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_USART1, ENABLE)


#define WIFI_USART_RX_ERR_IRQn			INT004_IRQn
#define WIFI_USART_RX_ERR_INT_SRC		INT_SRC_USART1_EI

#define WIFI_USART_RX_FULL_IRQn			INT005_IRQn
#define WIFI_USART_RX_FULL_INT_SRC		INT_SRC_USART1_RI



extern bool wifi_io_init(void);


#endif


#endif	// BOARD_h
