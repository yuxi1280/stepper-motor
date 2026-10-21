#include "app.h"
#include "board.h"
#include "can_context.h"

#include <string.h>

#include "esp_at_parser.h"
#include "hc32_ll_def.h"
#include "hc32_ll_usart.h"
#include "lwip/pbuf.h"
#include "usb_dev_user.h"
#include "usb_dev_desc.h"
#include "usb_dev_cdc_class.h"

#include "rtc.h"

#include "lwip/sys.h"
#include "lwip/timeouts.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/init.h"
#include "lwip/tcpip.h"
#include "lwip/netif.h"
#include "lwip/api.h"

#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "lwip/dns.h"
#include "lwip/dhcp.h"
#include "lwip/autoip.h"

#include "ethernetif.h"

#include "shelf.h"
#include "ring_buffer.h"
#include "esp_at_parser.h"
#include "new_memory_pool.h"

struct udp_pcb *g_udp_pcb;
struct at_context_t g_at_ctx;
struct qw_udp_context_t g_udp_ctx;

const char __date__[] = {
	// year
	__DATE__[7], __DATE__[8], __DATE__[9], __DATE__[10],

	'-',

	// month
	// Oct,Nov,Dec->1,->0
	(__DATE__[0] == 'O' || __DATE__[0] == 'N' || __DATE__[0] == 'D') ? '1' : '0',

	(__DATE__[0] == 'J')
		? ((__DATE__[1] == 'a')
				? '1'								  // Jan
				: ((__DATE__[2] == 'n') ? '6' : '7'))  // Jun,Jul
		: ((__DATE__[0] == 'F')
				? '2'  // Feb
				: ((__DATE__[0] == 'M')
					? ((__DATE__[2] == 'r') ? '3' : '5')  // Mar,May
					: ((__DATE__[0] == 'A')
						? ((__DATE__[1] == 'p') ? '4' : '8')  // Apr,Aug
						: ((__DATE__[0] == 'S')
							? '9'  // Sep
							: ((__DATE__[0] == 'O')
								? '0'  // Oct
								: ((__DATE__[0] == 'N')
									? '1'										// Nov
									: ((__DATE__[0] == 'D') ? '2' : '?'))))))),  // Dec,???

	'-',

	// day
	__DATE__[4] == ' ' ? '0' : __DATE__[4],
	__DATE__[5],

	'\0',
};

struct new_mp_span_t s_memory_spans[] = {
	{0, 0},
};

// newlib_sys_calls.c
int _write(int fd, char *p, int n)
{
#if	defined(DEBUG)
	for (int i = 0; i < n; ++i) {
		debug_usart_tx_byte(*(uint8_t *)(p+i));
	}

#if	0
	if (g_udp_pcb != NULL) {
		qw_udp_log(&g_udp_ctx, p, n);
	}
#endif

	return n;
#else

	return 0;
#endif
}

extern uint8_t __StackTop[];
extern uint8_t __StackLimit[];

extern uint8_t __HeapBase[];
extern uint8_t __HeapLimit[];

extern uint8_t __PoolHeapBase[];
extern uint8_t __PoolHeapLimit[];


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

#define WDT_COUNT_PERIOD		(16384u)

// pclk3=60MHz
// wdt_clk=60MHz/clock_div

// WDT_CNT_PERIOD16384*(1000ms/(60000000Hz/WDT_CLK_DIV2048))
// ≈560ms
// WDT_CNT_PERIOD16384*(1000ms/(60000000Hz/WDT_CLK_DIV8192))
// ≈2237ms
void wdt_init(void)
{
	stc_wdt_init_t wdt_init;

	wdt_init.u32CountPeriod		= WDT_CNT_PERIOD16384;
	wdt_init.u32ClockDiv		= WDT_CLK_DIV8192;
//	wdt_init.u32RefreshRange	= WDT_RANGE_0TO25PCT;
	wdt_init.u32RefreshRange	= WDT_RANGE_0TO100PCT;
	wdt_init.u32LPMCount		= WDT_LPM_CNT_STOP;
	wdt_init.u32ExceptionType	= WDT_EXP_TYPE_RST;
	(void)WDT_Init(&wdt_init);
}

struct qw_router_config_t g_config = {
	._magic = {QW_CONFIG_MAGIC0, QW_CONFIG_MAGIC1, QW_CONFIG_MAGIC2, QW_CONFIG_MAGIC3},
	._model = "ECR01",
	._sn = "0000000001",
	._mac = {ETH_MAC_ADDR0, ETH_MAC_ADDR1, ETH_MAC_ADDR2, 0x00, 0x00, 0x01},
	._ip = {0, 0, 0, 0},
	._mask = {255, 255, 255, 0},
	._gw = {0, 0, 0, 0},
	._server_ip = {192, 168, 10, 19},
	._server_port = 8888,
	._use_dhcp = 1,
	._use_wifi = 0,
	._wifi_ssid = "",
	._wifi_password = "",
};

struct qw_router_info_t g_info = {
	._ping_interval = QW_SHELF_REPORT_INTERVAL,
	._online = 0,
	._reset = 0,
	._reset_timestamp = 0,
};

can_context_t _can1_ctx, _can2_ctx;

can_config_t _can1_cfg = {
	10, 24, 8, 8,	// 125kbps
//	200, 16, 4, 4,	// 10kbps
	0,	// no filter
	{
		{0, CAN_ACF_FT_EXT, 0x00000000, 0x1FFFFFFF},
		{1, CAN_ACF_FT_STD, 0x00000000, 0x000007FF},
	},
	7,
	128,
};
can_config_t _can2_cfg = {
	10, 24, 8, 8,	// 125kbps
	0,	// no filter
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

usb_core_instance g_usb_dev;


void can_on_error(can_context_t *ctx, uint8_t error)
{
#if	1
	CM_CAN_TypeDef *can = ctx->_can;
	switch (error) {
	case CAN_ERROR_EP:
		{
			if (can_get_error_passive(can)) {
				// 主动->被动
				log_printf("error passive\n");
			} else {
				// 被动->主动
				log_printf("error active\n");
				can_enable_bus_error_interrupt(can);
			}
		}
		break;

	case CAN_ERROR_E:
		{

		}
		break;

	case CAN_ERROR_BE:
		{
			can_disable_bus_error_interrupt(can);
		}
		break;

	default:
		break;
	}
#endif
#if	0
	++ctx->_error_count;

	if (ctx->_error_count >= 300) {
		can_close(ctx);
	}
#endif
}


void can_on_recv(can_context_t *ctx, can_frame_t *f)
{
#if	0
	uint8_t dlc = ((f->control&CAN_RBUF_CONTROL_DLC_Msk)>>CAN_RBUF_CONTROL_DLC_Pos);

	if (ctx->_can == CM_CAN1) {
		log_printf("->[1](%d)", can_get_error_passive(ctx->_can));
	} else {
		log_printf("->[2](%d)", can_get_error_passive(ctx->_can));
	}
	uint8_t ide = (f->control&(1<<CAN_RBUF_CONTROL_IDE_Pos));
	uint8_t rtr = (f->control&(1<<CAN_RBUF_CONTROL_RTR_Pos));
	if (ide) {
		log_printf("[%08"PRIx32"]", f->ext_id);
	} else {
		log_printf("[%03"PRIx32"]", f->std_id);
	}
	log_printf("ide:%u,rtr:%u,dlc:%u\n", ide ? 1 : 0, rtr ? 1 : 0, dlc);

	if (!(f->control&(1<<CAN_RBUF_CONTROL_RTR_Pos))) {
		for (uint8_t i = 0; i < dlc; ++i) {
			log_printf("%02x ", f->data[i]);
		}
		log_printf("\n");
	}
#endif

	qw_on_can_frame(ctx, f, &g_udp_ctx);
}


struct netif g_netif;


void EthernetIF_NotifyLinkChange(struct netif *netif)
{
	log_printf("notify_link_change\n");

}





void esp_at_udp_recv_callback(const uint8_t *addr, uint16_t port, const uint8_t *p, int n)
{
	qw_on_udp_packet(&g_udp_ctx, addr, port, p, n);
}

static void netif_status_callback(struct netif *netif)
{
	if (g_config._use_wifi) {
		return;
	}
	if (!link_is_up()) {
		return;
	}

	state1_led_off();
	state2_led_on();

	printf("netif_status_callback:%s,", ip4addr_ntoa(netif_ip4_addr(netif)));
	printf("%s,", ip4addr_ntoa(netif_ip4_netmask(netif)));
	printf("%s\n", ip4addr_ntoa(netif_ip4_gw(netif)));

	if (g_udp_pcb == NULL) {
		g_udp_pcb = udp_new();
		g_udp_ctx._ll_ctx = g_udp_pcb;


		if (g_udp_pcb == NULL) {
			log_printf("udp_new failed\n");
		} else {
			log_printf("udp_new ok:%p\n", g_udp_pcb);
		}
	}

	if (g_udp_pcb != NULL) {
		const ip_addr_t *addr = netif_ip4_addr(netif);
		memcpy(g_udp_ctx._addr, addr, sizeof(g_udp_ctx._addr));

		g_udp_ctx.open(&g_udp_ctx);
	}
}

void trng_init(void)
{
	FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_TRNG, ENABLE);

	TRNG_Init(TRNG_SHIFT_CNT64, TRNG_RELOAD_INIT_VAL_ENABLE);
	TRNG_Cmd(ENABLE);
}


void delay(int n)
{
	for (volatile int i = 0; i < n; ++i) {
		for (volatile int j = 0; j < 1000; ++j) {

		}
	}
}

extern void wifi_usart_tx_byte(int c);

byte_buffer_t g_wifi_rb;

void wifi_usart_reset(void)
{
	printf("wifi_usart_reset\n");
    USART_FuncCmd(WIFI_USART_UNIT, (USART_RX | USART_INT_RX | USART_TX), DISABLE);
	USART_ReadData(WIFI_USART_UNIT);

	g_wifi_rb.head = 0;
	g_wifi_rb.tail = 0;

    USART_FuncCmd(WIFI_USART_UNIT, (USART_RX | USART_INT_RX | USART_TX), ENABLE);
}

static uint8_t s_rx_buffer[256];
static int s_rx_index = 0;
static int s_rx_state = 0;

void debug_usart_rx_full_irq_callback(void)
{
	uint8_t b;
	b = (uint8_t)USART_ReadData(DEBUG_USART_UNIT);
	(void)b;
	//wifi_usart_tx_byte(b);
	switch (s_rx_state) {
	case 0:
		if (b == 'A') {
			s_rx_buffer[s_rx_index++] = b;
			s_rx_state = 1;
		}
		break;

	case 1:
		if (s_rx_index >= sizeof(s_rx_buffer)-1) {
			s_rx_state = 0;
			s_rx_index = 0;
		} else {
			s_rx_buffer[s_rx_index++] = b;
			if (b == '\r') {
				s_rx_state = 2;
			}
		}
		break;

	case 2:
		{
			if (b == '\n') {
				s_rx_buffer[s_rx_index++] = b;
				esp_at_try_tx_command(&g_at_ctx, (const char *)s_rx_buffer, s_rx_index, NULL, NULL, NULL, NULL, 0);
			}

			s_rx_index = 0;
			s_rx_state = 0;
		}
		break;

	default:
		break;
	}
#if	1
//	log_printf("debug:%01x\n", b);
	switch (b) {
	case 'a':
		{
			uint32_t used_memory;
			uint32_t free_memory;
			new_mp_dump(s_memory_spans, sizeof(s_memory_spans)/sizeof(s_memory_spans[0]), &used_memory, &free_memory);
		}
		break;

	case 'b':
		{
			if (g_config._use_wifi) {
				const char *s = "AT+GMR\r\n";
				esp_at_try_tx_command(&g_at_ctx, s, strlen(s), NULL, NULL, NULL, NULL, 0);
				s = "AT+UART_CUR?\r\n";
				esp_at_try_tx_command(&g_at_ctx, s, strlen(s), NULL, NULL, NULL, NULL, 0);
				s = "AT+CIPSTAMAC?\r\n";
				esp_at_try_tx_command(&g_at_ctx, s, strlen(s), NULL, NULL, NULL, NULL, 0);

				uint32_t used_memory;
				uint32_t free_memory;
				new_mp_dump(s_memory_spans, sizeof(s_memory_spans)/sizeof(s_memory_spans[0]), &used_memory, &free_memory);
			}
		}
		break;

	default:
		break;
	}
#endif
}


void wifi_usart_rx_full_irq_callback(void)
{
	uint8_t b;

	b = (uint8_t)USART_ReadData(WIFI_USART_UNIT);
//	log_printf("%c", b);
	if (byte_buffer_not_full(g_wifi_rb)) {
		byte_buffer_put(g_wifi_rb, b);
	}
}


void at_on_wifi_got_ip(struct at_context_t *ctx)
{
	printf("wifi_got_ip\n");

	if (g_config._use_wifi) {
		state1_led_off();
		state2_led_on();

		const char *cmd = "AT+CIPDINFO=1\r\n";
		esp_at_try_tx_command(&g_at_ctx, cmd, strlen(cmd), NULL, NULL, NULL, NULL, 0);

		g_udp_ctx.open(&g_udp_ctx);
	}
}

void at_on_wifi_disconnect(struct at_context_t *ctx)
{
	if (g_config._use_wifi) {
		state1_led_on();
		state2_led_off();
	}
}

void mp_lock(void)
{}

void mp_unlock(const char *tag)
{}

extern char *_heap_ptr;

int main(void)
{
	peripheral_we();

	clk_init();


	setvbuf(stdout, NULL, _IONBF, 0);
//	setvbuf(stdout, NULL, _IOLBF, 0);
#if	HARDWARE_VERSION == 1
	debug_led_io_init();
#endif
	led_io_init();
	debug_usart_init();

	uint8_t status;
	status = RMU_GetStatus(RMU_FLAG_ALL);
	RMU_ClearStatus();

	state5_led_off();
	state6_led_off();
#if	defined(DEBUG)
	log_printf("rmu_flag:%08x\n", status);

#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	log_printf("__FPU_USED\n");
#endif

	log_printf("stack:%p,%p(%#08x)\nheap:%p,%p(%#08x)\npool_heap:%p,%p(%#08x)\n_heap_ptr:%p\n",
			__StackTop, __StackLimit, __StackTop-__StackLimit,
			__HeapBase, __HeapLimit, __HeapLimit-__HeapBase,
			__PoolHeapBase, __PoolHeapLimit, __PoolHeapLimit-__PoolHeapBase,
			_heap_ptr
			);

	//s_memory_spans[0]._addr = (uint32_t)__HeapBase;
	//s_memory_spans[0]._size = __HeapLimit - __HeapBase;

	s_memory_spans[0]._addr = (uint32_t)__PoolHeapBase;
	s_memory_spans[0]._size = __PoolHeapLimit - __PoolHeapBase;

	new_mp_init(s_memory_spans, sizeof(s_memory_spans)/sizeof(s_memory_spans[0]));



	stc_clock_freq_t clk_freq;
	CLK_GetClockFreq(&clk_freq);

	log_printf("sysclk:%10"PRIu32"\n", clk_freq.u32SysclkFreq);
	log_printf("  hclk:%10"PRIu32"\n", clk_freq.u32HclkFreq);
	log_printf(" exclk:%10"PRIu32"\n", clk_freq.u32ExclkFreq);
	log_printf(" pclk0:%10"PRIu32"\n", clk_freq.u32Pclk0Freq);
	log_printf(" pclk1:%10"PRIu32"\n", clk_freq.u32Pclk1Freq);
	log_printf(" pclk2:%10"PRIu32"\n", clk_freq.u32Pclk2Freq);
	log_printf(" pclk3:%10"PRIu32"\n", clk_freq.u32Pclk3Freq);
	log_printf(" pclk4:%10"PRIu32"\n", clk_freq.u32Pclk4Freq);
#endif


	can1_io_init();
	can1_interrupt_init();
	can2_io_init();
	can2_interrupt_init();



	rtc_init();
	qw_init_config();
	qw_load_config();
	//qw_dump_config();
	trng_init();

	if (g_config._use_wifi) {
		byte_buffer_init(g_wifi_rb, 32767);
	} else {
		byte_buffer_init(g_wifi_rb, 1023);
	}

#if HARDWARE_VERSION == 2
	if (g_config._use_wifi) {
		wifi_io_init();
		wifi_disable();
		DDL_DelayMS(200);
		wifi_enable();
		state5_led_on();
	} else {
		state5_led_off();
	}

	key_io_init();
#endif

#if	HARDWARE_VERSION == 1
	debug_led_on();
#endif


	state1_led_on();
	state2_led_off();

	peripheral_wp();

	SysTick_Init(1000);


	_can1_ctx._can = CM_CAN1;
	_can1_ctx._transmit_ptb_ready = true;

	_can2_ctx._can = CM_CAN2;
	_can2_ctx._transmit_ptb_ready = true;

	can_open(&_can1_ctx);
	can_config(&_can1_ctx, &_can1_cfg);
	can_open(&_can2_ctx);
	can_config(&_can2_ctx, &_can2_cfg);
#if	0
	can_enable_self_acknowledge(_can1_ctx._can);
	can_enable_loopback_mode_external(_can1_ctx._can);
	can_enable_self_acknowledge(_can2_ctx._can);
	can_enable_loopback_mode_external(_can2_ctx._can);
#endif

	at_report_callback_map_init();
	at_report_callback_map_register("WIFI GOT IP", at_on_wifi_got_ip);
	at_report_callback_map_register("WIFI DISCONNECT", at_on_wifi_disconnect);
	esp_at_init(&g_at_ctx);

	(void)netif_status_callback;

	g_udp_ctx._port = 8888;

			uint32_t used_memory;
			uint32_t free_memory;
			new_mp_dump(s_memory_spans, sizeof(s_memory_spans)/sizeof(s_memory_spans[0]), &used_memory, &free_memory);

	if (g_config._use_wifi == 0) {
		lwip_init();

		netif_add(&g_netif, IP4_ADDR_ANY, IP4_ADDR_ANY, IP4_ADDR_ANY, NULL,
				rtl8201_netif_init, netif_input);

		//	netif_create_ip6_linklocal_address(&g_netif, 1);
		netif_set_status_callback(&g_netif, netif_status_callback);
		netif_set_default(&g_netif);
		netif_set_up(&g_netif);

		//g_udp_ctx._ll_ctx = g_udp_pcb;
		g_udp_ctx.open = lwip_udp_open;
		g_udp_ctx.sendto = lwip_udp_sendto;
		g_udp_ctx.close = lwip_udp_close;
	} else {

		g_udp_ctx._ll_ctx = &g_at_ctx;
		g_udp_ctx.open = esp_at_udp_open;
		g_udp_ctx.sendto = esp_at_udp_sendto;
		g_udp_ctx.close = esp_at_udp_close;
	}


	stc_usb_port_identify upi;
//	upi.u8CoreID = USBHS_CORE_ID;
//	upi.u8PhyType = USBHS_PHY_EMBED;
	upi.u8CoreID = USBFS_CORE_ID;
	usb_dev_init(&g_usb_dev, &upi, &user_desc, &class_cdc_cbk, &user_cb);

	int64_t discover_interval_timer = sys_tick_get_infinite_ticks();
	int64_t udp_ping_timer = sys_tick_get_infinite_ticks();
	(void)discover_interval_timer;
	(void)udp_ping_timer;
	int64_t can_ping_timer = sys_tick_get_infinite_ticks();
	int discover_count = 0;
	(void)discover_count;
	int64_t now;

	wdt_init();
	WDT_FeedDog();
	// 0%~25%
//	uint16_t cmp_value = WDT_COUNT_PERIOD/4u;
	__enable_irq();


	for (;;) {
		now = sys_tick_get_infinite_ticks();


		if (byte_buffer_not_empty(g_wifi_rb)) {
			//printf("@");
			uint8_t b;
			uint8_t buffer[ESP_AT_LINE_SIZE];
			int n = 0;
			while (byte_buffer_not_empty(g_wifi_rb)) {
				if (n < ESP_AT_LINE_SIZE) {
					byte_buffer_get(g_wifi_rb, b);
					buffer[n++] = b;
				} else {
					break;
				}
				//printf("%02x ", b);
			}
			//printf("@\n");
			esp_at_on_rx(&g_at_ctx, buffer, n);
		}

		//if (WDT_GetCountValue() < cmp_value) {
		WDT_FeedDog();
		//}


		if (g_udp_ctx._state == 1) {
			// 开机只发一次
			if (discover_count < QW_CAN_DISCOVER_COUNT_MAX) {
				if (now - discover_interval_timer >= QW_SHELF_DISCOVER_INTERVAL) {
					discover_interval_timer = now;
					qw_can_discover();
					//const uint8_t msg[] = {0x00, 0x00};
					//can_tx_data_frames(&_can1_ctx, 0, 0x7ff, msg, sizeof(msg));
					++discover_count;
				}
			}
		}

		// 定时PING
		if (now - udp_ping_timer >= g_info._ping_interval) {
			// FIXME 连接到网络后且发现货架过程已结束
			if (g_config._server_port != 0 && discover_count >= QW_CAN_DISCOVER_COUNT_MAX) {
				//				ip_addr_t server_ip;
				//				IP4_ADDR(&server_ip, g_config._server_ip[0], g_config._server_ip[1], g_config._server_ip[2], g_config._server_ip[3]);
				if (g_udp_ctx._state == 1) {
					qw_udp_ping(&g_udp_ctx, g_config._server_ip, g_config._server_port);
				}
			}
			udp_ping_timer = now;
		}

		if (now - can_ping_timer >= QW_SHELF_CAN_PING_INTERVAL) {
#if	0
			if (!_can1_ctx._is_opened) {
				can_open(&_can1_ctx);
				can_config(&_can1_ctx, &_can2_cfg);
			}
			if (!_can2_ctx._is_opened) {
				can_open(&_can2_ctx);
				can_config(&_can2_ctx, &_can2_cfg);
			}
#endif
			qw_can_ping(now);
			can_ping_timer = now;
		}



		// CAN
		can_poll(&_can1_ctx);
		can_poll(&_can2_ctx);

		if (g_config._use_wifi == 0) {
			// LWIP
			EthernetIF_PeriodicHandle(&g_netif);

			if (link_status_changed()) {
				if (link_is_up()) {
					netif_set_link_up(&g_netif);

					if (g_config._use_dhcp == 1) {
						// 自动获取IP
						printf("dhcp_start...\n");
						dhcp_start(&g_netif);
					} else {
						// 设置固定IP
						ip4_addr_t ip, mask, gw;
						IP4_ADDR(&ip, g_config._ip[0], g_config._ip[1], g_config._ip[2], g_config._ip[3]);
						IP4_ADDR(&mask, g_config._mask[0], g_config._mask[1], g_config._mask[2], g_config._mask[3]);
						IP4_ADDR(&gw, g_config._gw[0], g_config._gw[1], g_config._gw[2], g_config._gw[3]);
						netif_set_ipaddr(&g_netif, &ip);
						netif_set_netmask(&g_netif, &mask);
						netif_set_gw(&g_netif, &gw);

					}
				} else {
					netif_set_link_down(&g_netif);
				}
			}

			__disable_irq();
			struct pbuf *rx_p = rtl8201_netif_input(&g_netif);
			__enable_irq();

			if (rx_p != NULL) {
				LINK_STATS_INC(link.recv);

				// SNMP?

				//log_printf("netif_input:%d\n", rx_p->tot_len);
				if (g_netif.input(rx_p, &g_netif) != ERR_OK) {
					pbuf_free(rx_p);
				} else {
				}
			} else {
				//log_printf("netif_input:NULL\n");
			}

			sys_check_timeouts();
		}

		// USB
		usb_dev_cdc_poll();

		if (g_config._use_wifi) {
			esp_at_poll(&g_at_ctx, now);
		}

		if (g_info._reset) {
			if (now - g_info._reset_timestamp >= 3000) {
				for (;;) {}
			}
		}
	}

	return 0;
}
