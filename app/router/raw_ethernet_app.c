#include "app.h"
#include "board.h"
#include <string.h>
#include "rtc.h"
#include "can_context.h"
#include "usb_dev_user.h"
#include "usb_dev_desc.h"
#include "usb_dev_cdc_class.h"

#include "ethernetif.h"

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
void usb_dev_cdc_on_recv(const uint8_t *p, uint32_t n) {}
usb_core_instance g_usb_dev;



struct netif g_netif;
struct pbuf g_rx_pbuf;
struct pbuf g_tx_pbuf;


void EthernetIF_NotifyLinkChange(struct netif *netif)
{
	log_printf("notify_link_change\n");

}

void EthernetIF_InputCallback(struct netif *netif, struct pbuf *head)
{
	struct pbuf *p = head;
	unsigned char *payload;
	while (p != NULL) {
		log_printf("[%5d]", p->len);
		payload = (uint8_t *)p->payload;
		for (int i = 0; i < p->len; ++i) {
			log_printf("%02x ", payload[i]);
		}
		p = p->next;
	}
	log_printf("\n");

}

void trng_init(void)
{
	FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_TRNG, ENABLE);

	TRNG_Init(TRNG_SHIFT_CNT64, TRNG_RELOAD_INIT_VAL_ENABLE);
	TRNG_Cmd(ENABLE);
}

int main(void)
{
	peripheral_we();

	clk_init();

	setvbuf(stdout, NULL, _IONBF, 0);
	debug_led_io_init();
	led_io_init();
	debug_usart_init();

	rtc_init();
	trng_init();

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

#if	0
	struct pbuf *test_p = pbuf_alloc(PBUF_RAW, 4096, PBUF_POOL);
	struct pbuf *temp_p = test_p;
	while (temp_p != NULL) {
		log_printf("%p->%d\n", temp_p, temp_p->len);
		temp_p = temp_p->next;
	}
#endif


	ethernetif_init(&g_netif);

	static const uint8_t msg[128] = {
		0x19, 0x86, 0x11, 0x02, 0x19, 0x90, 0x10, 0x24,
		0x20, 0x16, 0x02, 0x22, 0x20, 0x16, 0x02, 0x22,
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
		0x19, 0x86, 0x11, 0x02, 0x19, 0x90, 0x10, 0x24,
		0x20, 0x16, 0x02, 0x22, 0x20, 0x16, 0x02, 0x22,
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
		0x19, 0x86, 0x11, 0x02, 0x19, 0x90, 0x10, 0x24,
		0x20, 0x16, 0x02, 0x22, 0x20, 0x16, 0x02, 0x22,
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
		0x19, 0x86, 0x11, 0x02, 0x19, 0x90, 0x10, 0x24,
		0x20, 0x16, 0x02, 0x22, 0x20, 0x16, 0x02, 0x22,
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0a, 0x0b, 0x20, 0x26, 0x03, 0x17,
	};
	g_tx_pbuf.next = NULL;
	g_tx_pbuf.payload = (void *)msg;
	g_tx_pbuf.len = sizeof(msg);


	for (;;) {
		EthernetIF_PeriodicHandle(&g_netif);

		if (low_level_output(&g_netif, &g_tx_pbuf) != LL_OK) {
			log_printf("low_level_output failed\n");
		} else {
			log_printf("low_level_output ok\n");
		}

		ethernetif_input(&g_netif);
	}

	return 0;
}
