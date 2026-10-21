#include "app.h"
#include "board.h"
#include "can_context.h"

#include <string.h>

#include "usb_dev_user.h"
#include "usb_dev_desc.h"
#include "usb_dev_cdc_class.h"

//#define TEST_DEVICE

// newlib_sys_calls.c
int _write(int fd, char *p, int n)
{
#if	defined(DEBUG)
	for (int i = 0; i < n; ++i) {
		debug_usart_tx_byte(*(uint8_t *)(p+i));
	}

	return n;
#else

	return 0;
#endif
}

extern uint8_t __StackTop[];
extern uint8_t __StackLimit[];

extern uint8_t __HeapBase[];
extern uint8_t __HeapLimit[];


static int64_t _infinite_ticks = 0;

int64_t sys_tick_get_infinite_ticks(void)
{
	int64_t t;
	__disable_irq();
	t = _infinite_ticks;
	__enable_irq();

	return t;
}

void SysTick_Handler(void)
{
	SysTick_IncTick();
	__DSB();		// Arm Errata 838869
	++_infinite_ticks;
}

void usb_dev_cdc_on_recv(const uint8_t *p, uint32_t n) {}
usb_core_instance g_usb_dev;



can_context_t _can1_ctx, _can2_ctx;

can_config_t _can1_cfg = {
	10, 24, 8, 8,	// 125kbps
//	200, 16, 4, 4,	// 10kbps
	2,
	{
		{0, CAN_ACF_FT_EXT, 0x00000000, 0x1FFFFFFF},
		{1, CAN_ACF_FT_STD, 0x00000000, 0x000007FF},
	},
	7,
	128,
};
can_config_t _can2_cfg = {
	10, 24, 8, 8,	// 125kbps
	1,
	{
		{0, CAN_ACF_FT_STD, 0x00000000, 0x000007FF},
	},
	7,
	128,
};

void can1_irq_callback(void)
{
	can_irq_callback(&_can1_ctx);
}

void can2_irq_callback(void)
{
	can_irq_callback(&_can2_ctx);
}


int _rx_count;

void can_on_error(can_context_t *ctx, uint8_t error)
{

}

void can_on_recv(can_context_t *ctx, can_frame_t *f)
{
	uint8_t dlc = ((f->control&CAN_RBUF_CONTROL_DLC_Msk)>>CAN_RBUF_CONTROL_DLC_Pos);
	_rx_count += dlc;

#if	1
	if (ctx->_can == CM_CAN1) {
		log_printf("[#1]");
	} else {
		log_printf("[#2]");
	}
	uint8_t ide = (f->control&(1<<CAN_RBUF_CONTROL_IDE_Pos));
	uint8_t rtr = (f->control&(1<<CAN_RBUF_CONTROL_RTR_Pos));
	if (ide) {
		log_printf("[%08lx]", f->ext_id);
	} else {
		log_printf("[%03lx]", f->std_id);
	}
	log_printf("ide:%u,rtr:%u,dlc:%u\n", ide ? 1 : 0, rtr ? 1 : 0, dlc);
#endif


#if	1
	if (!(f->control&(1<<CAN_RBUF_CONTROL_RTR_Pos))) {
		for (uint8_t i = 0; i < dlc; ++i) {
			log_printf("%02x ", f->data[i]);
		}
		log_printf("\n");
	}
#endif
}



int main(void)
{
	peripheral_we();

	clk_init();

	setvbuf(stdout, NULL, _IONBF, 0);
	debug_led_io_init();
	led_io_init();
	debug_usart_init();

	can1_io_init();
	can1_interrupt_init();
	can2_io_init();
	can2_interrupt_init();

	peripheral_wp();

	debug_led_on();



#if	defined(DEBUG)

#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	log_printf("__FPU_USED\n");
#endif

	log_printf("stack:%p,%p(%#04x),heap:%p,%p(%#04x)\n",
			__StackTop, __StackLimit, __StackTop-__StackLimit,
			__HeapBase, __HeapLimit, __HeapLimit-__HeapBase);


	stc_clock_freq_t clk_freq;
	CLK_GetClockFreq(&clk_freq);

	log_printf("sysclk:%10lu\n", clk_freq.u32SysclkFreq);
	log_printf("  hclk:%10lu\n", clk_freq.u32HclkFreq);
	log_printf(" exclk:%10lu\n", clk_freq.u32ExclkFreq);
	log_printf(" pclk0:%10lu\n", clk_freq.u32Pclk0Freq);
	log_printf(" pclk1:%10lu\n", clk_freq.u32Pclk1Freq);
	log_printf(" pclk2:%10lu\n", clk_freq.u32Pclk2Freq);
	log_printf(" pclk3:%10lu\n", clk_freq.u32Pclk3Freq);
	log_printf(" pclk4:%10lu\n", clk_freq.u32Pclk4Freq);
#endif


	SysTick_Init(1000);


	_can1_ctx._can = CM_CAN1;
	_can1_ctx._transmit_ptb_ready = true;

	_can2_ctx._can = CM_CAN2;
	_can2_ctx._transmit_ptb_ready = true;

#if	defined(TEST_DEVICE)
	can_open(&_can1_ctx);
	can_config(&_can1_ctx, &_can1_cfg);
//	can_enable_listen_only_mode(_can1_ctx._can);

//	can_enable_self_acknowledge(_can1_ctx._can);
//	can_enable_loopback_mode_external(_can1_ctx._can);
//	can_enable_loopback_mode_internal(_can1_ctx._can);
//	can_enable_transmit_ptb_single_mode(_can1_ctx._can);
#else
	can_open(&_can1_ctx);
	can_config(&_can1_ctx, &_can1_cfg);

#endif

	can_open(&_can2_ctx);
	can_config(&_can2_ctx, &_can2_cfg);


	uint8_t p[] = {
		0x19, 0x86, 0x11, 0x02, 0x19, 0x90, 0x10, 0x24,
		0x20, 0x16, 0x02, 0x22, 0x20, 0x16, 0x02, 0x22,
		0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
		0x08, 0x09, 0x0A, 0x0B, 0xCC, 0xDD, 0xEE, 0xFF,
		0x19, 0x86, 0x11, 0x02, 0x19, 0x90, 0x10, 0x24,
		0x20, 0x16, 0x02, 0x22, 0x20, 0x16, 0x02, 0x22,
		0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
		0x08, 0x09, 0x0A, 0x0B, 0xCC, 0xDD, 0xEE, 0xFF,
	};
	(void)p;

	int64_t now = sys_tick_get_infinite_ticks();
	int64_t timer = now;
	(void)now;
	(void)timer;
	bool flag = true;
	(void)flag;

	for (;;) {
		can_poll(&_can1_ctx);
		can_poll(&_can2_ctx);

		now = sys_tick_get_infinite_ticks();
		if (now - timer >= 5000) {
			timer = now;
			/*log_printf("%d BytesPerSecond\n", _rx_count/5);
			_rx_count = 0;

			if (flag) {
				flag = false;
				can_close(&_can1_ctx);
			} else {
				flag = true;
				can_open(&_can1_ctx);
				can_config(&_can1_ctx, &_can1_cfg);
			}
			*/
		
#if	defined(TEST_DEVICE)
			can_tx_data_frames(&_can1_ctx, 0, 0x08, p, 19);
			can_tx_remote_frame(&_can1_ctx, 1, 0x19861102);
#endif
		}
	}

	return 0;
}
