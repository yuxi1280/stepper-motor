#include "app.h"
#include "board.h"

#include <string.h>

#include "can_context.h"

#include "usb_dev_user.h"
#include "usb_dev_desc.h"
#include "usb_dev_cdc_class.h"

#define USB_DEV_SPEED_TEST

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

void can_on_error(can_context_t *ctx, uint8_t error) {}
void can_on_recv(can_context_t *ctx, can_frame_t *f) {}
void can1_irq_callback(void) {}
void can2_irq_callback(void) {}


usb_core_instance g_usb_dev;
uint8_t _usb_rx_data[USB_RX_BUFFER_SIZE];

uint8_t _usb_rx_state;
uint8_t _usb_rx_index;
uint16_t _usb_rx_size;

uint8_t _usb_tx_buffer[8+USB_RX_BUFFER_SIZE];


void usb_dev_cdc_on_recv(const uint8_t *p, uint32_t n)
{
#if	defined(USB_DEV_SPEED_TEST)
	const uint8_t q[64] = {
		0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
		0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF,
		0x19, 0x86, 0x11, 0x02, 0x19, 0x90, 0x10, 0x24,
		0x20, 0x16, 0x02, 0x22, 0x20, 0x16, 0x02, 0x22,
		0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
		0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF,
		0x19, 0x86, 0x11, 0x02, 0x19, 0x90, 0x10, 0x24,
		0x20, 0x16, 0xAA, 0x55, 0x20, 0x16, 0x55, 0xAA,
	};

	// 测试10240*64需要传输多长时间
	for (int i = 0; i < 10240;) {
		if (usb_dev_cdc_send(&g_usb_dev, (const uint8_t *)q, sizeof(q))) {
			//log_printf("usb_dev_cdc_send ok\n");
			++i;
		} else {
			//log_printf("usb_dev_cdc_send failed\n");
		}
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


	stc_usb_port_identify upi;
//	upi.u8CoreID = USBHS_CORE_ID;
//	upi.u8PhyType = USBHS_PHY_EMBED;
	upi.u8CoreID = USBFS_CORE_ID;

	usb_dev_init(&g_usb_dev, &upi, &user_desc, &class_cdc_cbk, &user_cb);


	for (;;) {
		usb_dev_cdc_poll();
	}

	return 0;
}
