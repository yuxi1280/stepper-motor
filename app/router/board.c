#include "board.h"
#include "can.h"
#include "hc32_ll_dma.h"
#include "hc32_ll_gpio.h"
#include "hc32_ll_interrupts.h"
#include "hc32_ll_utility.h"

#include <inttypes.h>


static void install_irq_handler(const stc_irq_signin_config_t *cfg, uint32_t priority, bool enable)
{
	if (cfg == NULL) {
		return;
	}

	INTC_IrqSignIn(cfg);
	NVIC_ClearPendingIRQ(cfg->enIRQn);
	NVIC_SetPriority(cfg->enIRQn, priority);
	if (enable) {
		NVIC_EnableIRQ(cfg->enIRQn);
	}
}

#define ROUTER_PERIPH_WE	(LL_PERIPH_GPIO | LL_PERIPH_FCG | LL_PERIPH_PWC_CLK_RMU | LL_PERIPH_EFM | LL_PERIPH_SRAM)
//#define ROUTER_PERIPH_WP	(LL_PERIPH_GPIO | LL_PERIPH_FCG | LL_PERIPH_PWC_CLK_RMU | LL_PERIPH_EFM | LL_PERIPH_SRAM)
#define ROUTER_PERIPH_WP	(LL_PERIPH_FCG | LL_PERIPH_EFM | LL_PERIPH_SRAM)

void peripheral_we(void)
{
	LL_PERIPH_WE(ROUTER_PERIPH_WE);
}

void peripheral_wp(void)
{
	LL_PERIPH_WP(ROUTER_PERIPH_WP);
}

void clk_init(void)
{
	stc_clock_xtal_init_t xtal_init;
	stc_clock_pll_init_t pllh_init;

	CLK_SetClockDiv(CLK_BUS_CLK_ALL,
			(CLK_PCLK0_DIV1 | CLK_PCLK1_DIV2 | CLK_PCLK2_DIV4 |
			 CLK_PCLK3_DIV4 | CLK_PCLK4_DIV2 | CLK_EXCLK_DIV2 |
			 CLK_HCLK_DIV1));

	CLK_XtalStructInit(&xtal_init);
	xtal_init.u8Mode		= CLK_XTAL_MD_OSC;
	xtal_init.u8Drv			= CLK_XTAL_DRV_ULOW;
	xtal_init.u8State		= CLK_XTAL_ON;
	xtal_init.u8StableTime	= CLK_XTAL_STB_2MS;

	GPIO_AnalogCmd(XTAL_PORT, XTAL_PINS, ENABLE);
	CLK_XtalInit(&xtal_init);

	/* 8MHz/M*N = 8/1*120/4 =240MHz */
	CLK_PLLStructInit(&pllh_init);
	pllh_init.u8PLLState		= CLK_PLL_ON;
	pllh_init.PLLCFGR			= 0UL;
	pllh_init.PLLCFGR_f.PLLM	= 1UL - 1UL;
	pllh_init.PLLCFGR_f.PLLN	= 120UL - 1UL;
	pllh_init.PLLCFGR_f.PLLP	= 4UL - 1UL;
	pllh_init.PLLCFGR_f.PLLQ	= 4UL - 1UL;
	pllh_init.PLLCFGR_f.PLLR	= 4UL - 1UL;
	pllh_init.PLLCFGR_f.PLLSRC	= CLK_PLL_SRC_XTAL;
	CLK_PLLInit(&pllh_init);

	/* Highspeed SRAM set to 1 Read/Write wait cycle */
	SRAM_SetWaitCycle(SRAM_SRAMH, SRAM_WAIT_CYCLE1, SRAM_WAIT_CYCLE1);
	/* SRAM1_2_3_4_backup set to 2 Read/Write wait cycle */
	SRAM_SetWaitCycle((SRAM_SRAM123 | SRAM_SRAM4 | SRAM_SRAMB), SRAM_WAIT_CYCLE2, SRAM_WAIT_CYCLE2);
	/* 0-wait @ 40MHz */
	EFM_SetWaitCycle(EFM_WAIT_CYCLE5);
	/* 4 cycles for 200 ~ 250MHz */
	GPIO_SetReadWaitCycle(GPIO_RD_WAIT4);
	CLK_SetSysClockSrc(CLK_SYSCLK_SRC_PLL);
}

int xtal32_init(void)
{
	stc_clock_xtal32_init_t stcXtal32Init;
	stc_fcm_init_t stcFcmInit;
	uint32_t u32TimeOut = 0UL;
	uint32_t u32Time = HCLK_VALUE / 5UL;

	if (CLK_XTAL32_ON == READ_REG8(CM_CMU->XTAL32CR)) {
		/* Disable xtal32 */
		(void)CLK_Xtal32Cmd(DISABLE);
	}

	/* Xtal32 config */
	(void)CLK_Xtal32StructInit(&stcXtal32Init);
	stcXtal32Init.u8State  = CLK_XTAL32_ON;
	stcXtal32Init.u8Drv    = CLK_XTAL32_DRV_MID;
	stcXtal32Init.u8Filter = CLK_XTAL32_FILTER_ALL_MD;
	GPIO_AnalogCmd(XTAL32_PORT, XTAL32_PINS, ENABLE);
	(void)CLK_Xtal32Init(&stcXtal32Init);

	/* FCM config */
	FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_FCM, ENABLE);
	(void)FCM_StructInit(&stcFcmInit);
	stcFcmInit.u32RefClock       = FCM_REF_CLK_MRC;
	stcFcmInit.u32RefClockDiv    = FCM_REF_CLK_DIV8192;
	stcFcmInit.u32RefClockEdge   = FCM_REF_CLK_RISING;
	stcFcmInit.u32TargetClock    = FCM_TARGET_CLK_XTAL32;
	stcFcmInit.u32TargetClockDiv = FCM_TARGET_CLK_DIV1;
	stcFcmInit.u16LowerLimit     = (uint16_t)((XTAL32_VALUE / (MRC_VALUE / 8192U)) * 96UL / 100UL);
	stcFcmInit.u16UpperLimit     = (uint16_t)((XTAL32_VALUE / (MRC_VALUE / 8192U)) * 104UL / 100UL);
	(void)FCM_Init(CM_FCM, &stcFcmInit);
	/* Enable FCM, to ensure xtal32 stable */
	FCM_Cmd(CM_FCM, ENABLE);
	for (;;) {
		if (SET == FCM_GetStatus(CM_FCM, FCM_FLAG_END)) {
			FCM_ClearStatus(CM_FCM, FCM_FLAG_END);
			if (SET == FCM_GetStatus(CM_FCM, FCM_FLAG_ERR | FCM_FLAG_OVF)) {
				FCM_ClearStatus(CM_FCM, FCM_FLAG_ERR | FCM_FLAG_OVF);
			} else {
				(void)FCM_DeInit(CM_FCM);
				FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_FCM, DISABLE);
				return LL_OK;
			}
		}
		u32TimeOut++;
		if (u32TimeOut > u32Time) {
			(void)FCM_DeInit(CM_FCM);
			FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_FCM, DISABLE);
			return LL_ERR_TIMEOUT;
		}
	}
}


#if	HARDWARE_VERSION == 1
void debug_led_io_init(void)
{
	stc_gpio_init_t gpio_init;
	GPIO_StructInit(&gpio_init);
	gpio_init.u16PinState = PIN_STAT_RST;
	gpio_init.u16PinDir = PIN_DIR_OUT;

	GPIO_Init(DEBUG_LED_PORT, DEBUG_LED_PIN, &gpio_init);
}
#endif

void led_io_init(void)
{
	stc_gpio_init_t gpio_init;
	GPIO_StructInit(&gpio_init);
	gpio_init.u16PinState = PIN_STAT_RST;
	gpio_init.u16PinDir = PIN_DIR_OUT;

	GPIO_Init(STATE1_LED_PORT, STATE1_LED_PIN, &gpio_init);
	GPIO_Init(STATE2_LED_PORT, STATE2_LED_PIN, &gpio_init);
	GPIO_Init(STATE3_LED_PORT, STATE3_LED_PIN, &gpio_init);
	GPIO_Init(STATE4_LED_PORT, STATE4_LED_PIN, &gpio_init);
	GPIO_Init(STATE5_LED_PORT, STATE5_LED_PIN, &gpio_init);
	GPIO_Init(STATE6_LED_PORT, STATE6_LED_PIN, &gpio_init);
}


// 功能
void key1_irq_callback(void)
{
	if (EXTINT_GetExtIntStatus(KEY1_EXTINT_CH) == SET) {
		EXTINT_ClearExtIntStatus(KEY1_EXTINT_CH);
	//	printf("key1_irq_callback\n");



#if	0
		static int state5 = 0;
		state5 = state5 == 0 ? 1 : 0;

		if (state5 == 0) {
			state5_led_off();
		} else {
			state5_led_on();
		}
#endif

	}
}

// 复位
void key2_irq_callback(void)
{
	if (EXTINT_GetExtIntStatus(KEY2_EXTINT_CH) == SET) {
		EXTINT_ClearExtIntStatus(KEY2_EXTINT_CH);
		//printf("key2_irq_callback\n");

		int i;
		state6_led_on();
#define RESET_DELAY		500
		for (i = 0; i < RESET_DELAY; ++i) {
			if (GPIO_ReadInputPins(KEY2_PORT, KEY2_PIN)) {
				state6_led_off();
				break;
			}
			DDL_DelayMS(10);
		}
		if (i == RESET_DELAY) {
			for (;;) {}
		}


#if	0
	static int state6 = 0;
		state6 = state6 == 0 ? 1 : 0;

		if (state6 == 0) {
			state6_led_off();
		} else {
			state6_led_on();
		}
#endif
	}
}

void key_io_init(void)
{
	stc_extint_init_t extint_init;
	stc_irq_signin_config_t irq_signin_cfg;
	stc_gpio_init_t gpio_init;

	(void)GPIO_StructInit(&gpio_init);
	gpio_init.u16ExtInt = PIN_EXTINT_ON;
	gpio_init.u16PullUp = PIN_PU_ON;
	(void)GPIO_Init(KEY1_PORT, KEY1_PIN, &gpio_init);

	(void)EXTINT_StructInit(&extint_init);
	extint_init.u32Filter		= EXTINT_FILTER_ON;
	extint_init.u32FilterClock	= EXTINT_FCLK_DIV8;
	extint_init.u32Edge = EXTINT_TRIG_FALLING;
	(void)EXTINT_Init(KEY1_EXTINT_CH, &extint_init);

	irq_signin_cfg.enIntSrc	= KEY1_INT_SRC;
	irq_signin_cfg.enIRQn	= KEY1_INT_IRQn;
	irq_signin_cfg.pfnCallback = &key1_irq_callback;
	(void)INTC_IrqSignIn(&irq_signin_cfg);

	NVIC_ClearPendingIRQ(irq_signin_cfg.enIRQn);
	NVIC_SetPriority(irq_signin_cfg.enIRQn, KEY1_IRQ_PRIO);
	NVIC_EnableIRQ(irq_signin_cfg.enIRQn);

	(void)GPIO_StructInit(&gpio_init);
	gpio_init.u16ExtInt = PIN_EXTINT_ON;
	gpio_init.u16PullUp = PIN_PU_ON;
	(void)GPIO_Init(KEY2_PORT, KEY2_PIN, &gpio_init);

	(void)EXTINT_StructInit(&extint_init);
	extint_init.u32Filter		= EXTINT_FILTER_ON;
	extint_init.u32FilterClock	= EXTINT_FCLK_DIV8;
	extint_init.u32Edge = EXTINT_TRIG_FALLING;
	(void)EXTINT_Init(KEY2_EXTINT_CH, &extint_init);

	irq_signin_cfg.enIntSrc	= KEY2_INT_SRC;
	irq_signin_cfg.enIRQn	= KEY2_INT_IRQn;
	irq_signin_cfg.pfnCallback = &key2_irq_callback;
	(void)INTC_IrqSignIn(&irq_signin_cfg);

	NVIC_ClearPendingIRQ(irq_signin_cfg.enIRQn);
	NVIC_SetPriority(irq_signin_cfg.enIRQn, KEY2_IRQ_PRIO);
	NVIC_EnableIRQ(irq_signin_cfg.enIRQn);

}

void debug_usart_rx_err_irq_callback(void)
{
	USART_ReadData(DEBUG_USART_UNIT);
	USART_ClearStatus(DEBUG_USART_UNIT, (USART_FLAG_PARITY_ERR|USART_FLAG_FRAME_ERR|USART_FLAG_OVERRUN));
}

extern void debug_usart_rx_full_irq_callback(void);

bool debug_usart_init(void)
{
    stc_usart_uart_init_t uart_init;

	GPIO_SetFunc(DEBUG_USART_TX_PORT, DEBUG_USART_TX_PIN, DEBUG_USART_TX_FUNC);
	GPIO_SetFunc(DEBUG_USART_RX_PORT, DEBUG_USART_RX_PIN, DEBUG_USART_RX_FUNC);

	DEBUG_USART_FCG_ENABLE();


	USART_UART_StructInit(&uart_init);
	uart_init.u32ClockDiv = USART_CLK_DIV4;
	uart_init.u32Baudrate = DEBUG_USART_BAUDRATE;
	uart_init.u32OverSampleBit = USART_OVER_SAMPLE_8BIT;

	if (USART_UART_Init(DEBUG_USART_UNIT, &uart_init, NULL) != LL_OK) {
		//state6_led_on();
		return false;
	}

	int32_t result;
	float error;
	result = USART_SetBaudrate(DEBUG_USART_UNIT, DEBUG_USART_BAUDRATE, &error);


	stc_irq_signin_config_t irq_cfg;

	irq_cfg.enIRQn = DEBUG_USART_RX_ERR_IRQn;
	irq_cfg.enIntSrc = DEBUG_USART_RX_ERR_INT_SRC;
	irq_cfg.pfnCallback = &debug_usart_rx_err_irq_callback;
	install_irq_handler(&irq_cfg, DDL_IRQ_PRIO_DEFAULT, true);

	irq_cfg.enIRQn = DEBUG_USART_RX_FULL_IRQn;
	irq_cfg.enIntSrc = DEBUG_USART_RX_FULL_INT_SRC;
	irq_cfg.pfnCallback = &debug_usart_rx_full_irq_callback;
	install_irq_handler(&irq_cfg, DDL_IRQ_PRIO_DEFAULT, true);


    USART_FuncCmd(DEBUG_USART_UNIT, (USART_RX | USART_INT_RX | USART_TX), ENABLE);


	log_printf("result:%"PRIi32",error:%d\n", result, (int)error*1000);


	return true;
}

void debug_usart_tx_byte(int c)
{
	while (USART_GetStatus(DEBUG_USART_UNIT, USART_FLAG_TX_EMPTY) == RESET) {}
	USART_WriteData(DEBUG_USART_UNIT, c);
}

void wifi_usart_tx_byte(int c)
{
	while (USART_GetStatus(WIFI_USART_UNIT, USART_FLAG_TX_EMPTY) == RESET) {}
	USART_WriteData(WIFI_USART_UNIT, c);
}

void wifi_usart_tx_bytes(const uint8_t *p, int n)
{
	for (int i = 0; i < n; ++i) {
		while (USART_GetStatus(WIFI_USART_UNIT, USART_FLAG_TX_EMPTY) == RESET) {}
		USART_WriteData(WIFI_USART_UNIT, p[i]);
	}
}


extern void can1_irq_callback(void);

void can1_io_init(void)
{
	// 时钟
	CLK_SetCANClockSrc(CLK_CAN1, CLK_CANCLK_SYSCLK_DIV6);

	// 管脚
	GPIO_SetFunc(CAN1_TX_PORT, CAN1_TX_PIN, CAN1_TX_FUNC);
	GPIO_SetFunc(CAN1_RX_PORT, CAN1_RX_PIN, CAN1_RX_FUNC);
}

void can1_interrupt_init(void)
{
	// 中断
	stc_irq_signin_config_t irq_cfg;

	irq_cfg.enIntSrc	= CAN1_INT_SRC;
	irq_cfg.enIRQn		= CAN1_IRQn;
	irq_cfg.pfnCallback	= &can1_irq_callback;

	install_irq_handler(&irq_cfg, DDL_IRQ_PRIO_DEFAULT, false);
}

extern void can2_irq_callback(void);

void can2_io_init(void)
{
	// 时钟
	CLK_SetCANClockSrc(CLK_CAN2, CLK_CANCLK_SYSCLK_DIV6);

	// 管脚
	GPIO_SetFunc(CAN2_TX_PORT, CAN2_TX_PIN, CAN2_TX_FUNC);
	GPIO_SetFunc(CAN2_RX_PORT, CAN2_RX_PIN, CAN2_RX_FUNC);
}

void can2_interrupt_init(void)
{
	// 中断
	stc_irq_signin_config_t irq_cfg;

	irq_cfg.enIntSrc	= CAN2_INT_SRC;
	irq_cfg.enIRQn		= CAN2_IRQn;
	irq_cfg.pfnCallback	= &can2_irq_callback;

	install_irq_handler(&irq_cfg, DDL_IRQ_PRIO_DEFAULT, false);
}

#if	0
void yt8522_hardware_reset(void)
{
/*
YT8522 have a hardware reset pin(RESET_N) which is low active. RESET_N should be active for at
least 10ms to make sure all internal logic is reset to a known state. Hardware reset should be applied
after power up.
*/

	stc_gpio_init_t gpio_init;
	GPIO_StructInit(&gpio_init);
	gpio_init.u16PinState = PIN_STAT_RST;
	gpio_init.u16PinDir = PIN_DIR_OUT;

	GPIO_Init(ETH_RESET_PORT, ETH_RESET_PIN, &gpio_init);

	DDL_DelayMS(10);

	GPIO_SetPins(ETH_RESET_PORT, ETH_RESET_PIN);

	DDL_DelayMS(10);
}


void eth_mii_io_init(void)
{
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

void eth_smi_io_init(void)
{
	stc_gpio_init_t gpio_init;

    (void)GPIO_StructInit(&gpio_init);
    gpio_init.u16PinDrv = PIN_HIGH_DRV;

	// SMI
	GPIO_Init(ETH_SMI_MDC_PORT, ETH_SMI_MDC_PIN, &gpio_init);
	GPIO_Init(ETH_SMI_MDIO_PORT, ETH_SMI_MDIO_PIN, &gpio_init);
	GPIO_SetFunc(ETH_SMI_MDC_PORT, ETH_SMI_MDC_PIN, GPIO_FUNC_11);
	GPIO_SetFunc(ETH_SMI_MDIO_PORT, ETH_SMI_MDIO_PIN, GPIO_FUNC_11);


    ETH_MAC_SetMdcClock();
}
#endif

#if	HARDWARE_VERSION == 2

void wifi_usart_rx_err_irq_callback(void)
{
	USART_ReadData(WIFI_USART_UNIT);
	USART_ClearStatus(WIFI_USART_UNIT, (USART_FLAG_PARITY_ERR|USART_FLAG_FRAME_ERR|USART_FLAG_OVERRUN));
}



extern void wifi_usart_rx_full_irq_callback(void);


bool wifi_io_init(void)
{
#if	1
	stc_gpio_init_t gpio_init;
	GPIO_StructInit(&gpio_init);

	gpio_init.u16PinState = PIN_STAT_RST;
	gpio_init.u16PinDir = PIN_DIR_OUT;
	GPIO_Init(WIFI_EN_PORT, WIFI_EN_PIN, &gpio_init);


	GPIO_StructInit(&gpio_init);
	gpio_init.u16PullUp = PIN_PU_ON;
	gpio_init.u16PinDir = PIN_DIR_IN;
	GPIO_Init(WIFI_INT_PORT, WIFI_INT_PIN, &gpio_init);


#endif


	stc_usart_uart_init_t uart_init;

	GPIO_SetFunc(WIFI_USART_TX_PORT, WIFI_USART_TX_PIN, WIFI_USART_TX_FUNC);
	GPIO_SetFunc(WIFI_USART_RX_PORT, WIFI_USART_RX_PIN, WIFI_USART_RX_FUNC);

	WIFI_USART_FCG_ENABLE();

	USART_UART_StructInit(&uart_init);
	uart_init.u32ClockDiv = USART_CLK_DIV4;
	uart_init.u32Baudrate = WIFI_USART_BAUDRATE;
	uart_init.u32OverSampleBit = USART_OVER_SAMPLE_8BIT;

	if (USART_UART_Init(WIFI_USART_UNIT, &uart_init, NULL) != LL_OK) {
		return false;
	}

	int32_t result;
	float error;
	result = USART_SetBaudrate(WIFI_USART_UNIT, WIFI_USART_BAUDRATE, &error);


	stc_irq_signin_config_t irq_cfg;

	irq_cfg.enIRQn = WIFI_USART_RX_ERR_IRQn;
	irq_cfg.enIntSrc = WIFI_USART_RX_ERR_INT_SRC;
	irq_cfg.pfnCallback = &wifi_usart_rx_err_irq_callback;
	install_irq_handler(&irq_cfg, DDL_IRQ_PRIO_DEFAULT, true);

	irq_cfg.enIRQn = WIFI_USART_RX_FULL_IRQn;
	irq_cfg.enIntSrc = WIFI_USART_RX_FULL_INT_SRC;
	irq_cfg.pfnCallback = &wifi_usart_rx_full_irq_callback;
	install_irq_handler(&irq_cfg, DDL_IRQ_PRIO_DEFAULT, true);


    USART_FuncCmd(WIFI_USART_UNIT, (USART_RX | USART_INT_RX | USART_TX), ENABLE);


	log_printf("result:%"PRIi32",error:%d\n", result, (int)error*1000);


	return true;

}

#endif
