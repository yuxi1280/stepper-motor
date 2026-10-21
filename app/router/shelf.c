#include "shelf.h"
#include "can_context.h"

#include "board.h"
#include "app.h"

#include "esp_at_parser.h"
#include "lwip/ip_addr.h"
#include "lwip/udp.h"

extern struct qw_udp_context_t g_udp_ctx;

struct qw_shelf_t g_shelves[QW_SHELF_COUNT_MAX];

static uint8_t s_udp_tx_packet[QW_UDP_PACKET_SIZE];
static uint8_t s_udp_rx_packet[QW_UDP_PACKET_SIZE];

static uint8_t s_can_tx_packet[QW_UDP_PACKET_SIZE];
extern can_context_t _can1_ctx;
extern can_context_t _can2_ctx;




static void qw_shelf_on_can_frame(struct qw_shelf_t *shelf, can_frame_t *f, struct qw_udp_context_t *udp_ctx);
static void qw_shelf_on_can_packet(struct qw_shelf_t *shelf, struct qw_udp_context_t *udp_ctx);


void esp_at_on_cipstart_rc(struct at_context_t *ctx)
{
	if (strcmp((const char *)ctx->_line, "OK") == 0) {
		g_udp_ctx._state = 1;

		if (g_config._server_port != 0) {
			IP4_ADDR(&g_info._server_addr, g_config._server_ip[0], g_config._server_ip[1], g_config._server_ip[2], g_config._server_ip[3]);
			qw_udp_on_query_shelves(&g_udp_ctx, (const uint8_t *)&g_info._server_addr, g_config._server_port, 0);
		}
	}
}

void esp_at_on_cipclose_rc(struct at_context_t *ctx)
{
	g_udp_ctx._state = 0;
}

int esp_at_udp_open(struct qw_udp_context_t *ctx)
{
	struct at_context_t *at_ctx = (struct at_context_t *)ctx->_ll_ctx;
	char buffer[256];
	int n;
	n = snprintf(buffer, sizeof(buffer), "AT+CIPSTART=\"UDP\",\"%u.%u.%u.%u\",%u,%u,0\r\n",
			g_config._server_ip[0], g_config._server_ip[1], g_config._server_ip[2], g_config._server_ip[3],
			g_config._server_port,
			ctx->_port);
	esp_at_try_tx_command(at_ctx, buffer, n, esp_at_on_cipstart_rc, NULL, NULL, NULL, 0);

	return 0;
}

int esp_at_udp_close(struct qw_udp_context_t *ctx)
{
	struct at_context_t *at_ctx = (struct at_context_t *)ctx->_ll_ctx;

	const char *cmd = "AT+CIPCLOSE\r\n";
	esp_at_try_tx_command(at_ctx, cmd, strlen(cmd), esp_at_on_cipclose_rc, NULL, NULL, NULL, 0);

	return 0;
}

int esp_at_udp_sendto(struct qw_udp_context_t *ctx, const uint8_t *addr, uint16_t port, const uint8_t *p, int n)
{
	if (ctx == NULL || addr == NULL || p == NULL || n <= 0) {
		return -1;
	}
	if (ctx->_state == 0) {
		return -1;
	}
	struct at_context_t *at_ctx = (struct at_context_t *)ctx->_ll_ctx;
	char buffer[64];
	int m;
	m = snprintf(buffer, sizeof(buffer), "AT+CIPSEND=%d\r\n", n);
	uint8_t *extra_data = (uint8_t *)ESP_AT_MALLOC(n);
	memcpy(extra_data, p, n);
	esp_at_try_tx_command(at_ctx, buffer, m, NULL, NULL, NULL, extra_data, n);

	return 0;
}


void udp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *rx_p,
		const ip_addr_t *addr, u16_t port)
{
#if	0
	log_printf("%s:%u->[%d]@%.*s@\n", ip4addr_ntoa(addr), port, p->len,
			p->len, (const char *)p->payload);
#endif
#if	0
	log_printf("%s:%u->[%d]%p\n", ip4addr_ntoa(addr), port, p->len, p);
#endif
	
	if (rx_p->tot_len > sizeof(s_udp_rx_packet)) {
		log_printf("%s:%d:invalid udp packet size:%d\n", __func__, __LINE__, rx_p->tot_len);
		return;
	}
	pbuf_copy_partial(rx_p, s_udp_rx_packet, rx_p->tot_len, 0);
	
	struct qw_udp_context_t *udp_ctx = (struct qw_udp_context_t *)arg;
	qw_on_udp_packet(udp_ctx, (const uint8_t *)addr, port, s_udp_rx_packet, rx_p->tot_len);

	// 注意：需在回调内释放
	pbuf_free(rx_p);
}



int lwip_udp_open(struct qw_udp_context_t *ctx)
{
	struct udp_pcb *pcb = (struct udp_pcb *)ctx->_ll_ctx;
	if (pcb == NULL) {
		return -1;
	}
	err_t e = 0;

	if ((e = udp_bind(pcb, (const ip_addr_t *)ctx->_addr, ctx->_port)) != ERR_OK) {
		log_printf("udp_bind failed:%d\n", e);
	} else {
		log_printf("udp_bind ok\n");

		ctx->_state = 1;
		// udp_remove(g_udp_pcb);
		// g_udp_pcb = NULL;
	}

	udp_recv(pcb, udp_recv_callback, ctx);

	// 如果存在中心服务器，则连接并发送配置信息
	// 注意：不能使用connect，否则将失去UDP服务器功能！
	if (g_config._server_port != 0) {
		IP4_ADDR(&g_info._server_addr, g_config._server_ip[0], g_config._server_ip[1], g_config._server_ip[2], g_config._server_ip[3]);

#if	0
		// 网关作为UDP服务器，只使用bind，不使用connect
		if ((e = udp_connect(g_udp_pcb, &server_ip, g_config._server_port)) != ERR_OK) {
			log_printf("udp_connect failed\n");
		} else {
			log_printf("udp_connect ok\n");

			// 发送货架信息
			qw_udp_on_query_shelves_ex(ctx, (const uint8_t *)&g_info._server_addr, g_config._server_port, 0);
		}
#endif
		// 发送货架信息
		qw_udp_on_query_shelves(ctx, (const uint8_t *)&g_info._server_addr, g_config._server_port, 0);


#if	0
		ip_addr_t test_ip;
		IP4_ADDR(&test_ip, 192, 168, 10, 101);
		if ((e = udp_connect(g_udp_pcb, &test_ip, 6666)) != ERR_OK) {
			log_printf("udp_connect failed\n");
		} else {
			udp_recv(g_udp_pcb, udp_recv_callback, NULL);

			log_printf("udp_connect ok\n");
			const char *msg = "hello udp via lwip";
			struct pbuf *tx_p = pbuf_alloc(PBUF_TRANSPORT, 32, PBUF_POOL);
			memset(tx_p->payload, 0, tx_p->len);
			memcpy(tx_p->payload, msg, strlen(msg));

			if ((e = udp_send(g_udp_pcb, tx_p)) != ERR_OK) {
				log_printf("udp_send failed:%d", e);
			} else {
				log_printf("udp_send ok\n");
			}
			pbuf_free(tx_p);


			//udp_disconnect(g_udp_pcb);
		}
#endif
	}

	return 0;
}

int lwip_udp_close(struct qw_udp_context_t *ctx)
{
	ctx->_state = 0;

	return 0;
}

int lwip_udp_sendto(struct qw_udp_context_t *ctx, const uint8_t *addr, uint16_t port, const uint8_t *p, int n)
{
	struct udp_pcb *pcb = (struct udp_pcb *)ctx->_ll_ctx;
	printf("lwip_udp_sendto:%u.%u.%u.%u:%u,%p,%d\n",
			addr[0], addr[1], addr[2], addr[3], port,
			p, n);
	if (ctx == NULL || addr == NULL || p == NULL || n <= 0) {
		log_printf("%s:%d:invalid parameter\n", __func__, __LINE__);
		return -1;
	}
	struct pbuf *tx_p = pbuf_alloc(PBUF_TRANSPORT, n, PBUF_POOL);
	if (tx_p == NULL) {
		log_printf("%s:%d:pbuf_alloc failed\n", __func__, __LINE__);
		return -1;
	}

	int result = -1;
	
	if (pbuf_take(tx_p, p, n) != ERR_OK) {
		log_printf("%s:%d:pbuf_take failed\n", __func__, __LINE__);
		goto failed;
	}

	if (udp_sendto(pcb, tx_p, (const ip_addr_t *)addr, port) != ERR_OK) {
		log_printf("%s:%d:udp_sendto failed\n", __func__, __LINE__);
		goto failed;
	} else {
		result = 0;
	}

failed:
	pbuf_free(tx_p);
	return result;
}



struct qw_shelf_t *qw_shelf_alloc(void)
{
	struct qw_shelf_t *shelf = NULL;
	for (int i = 0; i < QW_SHELF_COUNT_MAX; ++i) {
		shelf = g_shelves+i;
		if (shelf->_id == 0) {
			return shelf;
		}
	}
	return NULL;
}

void qw_shelf_free(struct qw_shelf_t *shelf)
{
	shelf->_id = 0;
}


struct qw_shelf_t *qw_get_shelf_by_shelf_id(uint32_t id)
{
	struct qw_shelf_t *shelf = NULL;
	for (int i = 0; i < QW_SHELF_COUNT_MAX; ++i) {
		shelf = g_shelves+i;
		if (shelf->_id == id) {
			return shelf;
		}
	}

	return NULL;
}


/*
■ 容量高达2Mbytes，由两块1Mbytes的FLASH构成，共256个扇区，每个扇区为8Kbytes。
块0中扇区0~扇区15为可配置为OTP区域。
■ OTP(One Time Program)区域共134KBytes，其中128Kbytes配置在块0地址0x0000_0000~0x0001_FFFF，
6Kbytes配置在地址0x0300_0000~0x0300_17FF。地址0x0300_1800~0x0300_1AD7为OTP数据锁存区。
■ 128位宽数据读取，读缓存128位宽缓冲，加速代码执行。
■ 编程单位为4bytes,擦除单位为8Kbytes。
*/

#if	defined(USE_EFM_CONFIG)
static const uint32_t k_config_sector_num = 120;
static uint32_t s_config_address;
#endif


void qw_init_config(void)
{
	en_int_status_t flag;
	do {
		flag = EFM_GetStatus(EFM_FLAG_RDY);
	} while (flag != SET);

#if	defined(USE_EFM_CONFIG)
	s_config_address = EFM_SECTOR_ADDR(k_config_sector_num);
#endif
}


bool qw_save_config(void)
{
	struct qw_router_config_t config;
	bool result = false;

#if	defined(USE_RTC_CONFIG)
	rtc_write_backup_register((const uint8_t *)&g_config, sizeof(g_config));
	rtc_read_backup_register((uint8_t *)&config, sizeof(config));
#elif defined(USE_EFM_CONFIG)
	EFM_FWMC_Cmd(ENABLE);
	EFM_SingleSectorOperateCmd(k_config_sector_num, ENABLE);

	if (EFM_SectorErase(s_config_address) != LL_OK) {
		log_printf("EFM_SectorErase failed\n");
		goto failed;
	} else {
		log_printf("EFM_SectorErase ok\n");
	}

	if (EFM_Program(s_config_address, (uint8_t *)&g_config, sizeof(g_config)) != LL_OK) {
		log_printf("EFM_Program failed\n");
		goto failed;
	} else {
		log_printf("EFM_Program ok\n");
	}

	if (EFM_ReadByte(s_config_address, (uint8_t *)&config, sizeof(config)) != LL_OK) {
		log_printf("EFM_ReadByte failed\n");
		goto failed;
	} else {
		log_printf("EFM_ReadByte ok\n");
	}


#endif

	if (memcmp(&g_config, &config, sizeof(g_config)) != 0) {
		log_printf("qw_save_config failed\n");
		goto failed;
	} else {
		log_printf("qw_save_config ok\n");
		result = true;
	}

failed:
	EFM_SingleSectorOperateCmd(k_config_sector_num, DISABLE);
	EFM_FWMC_Cmd(DISABLE);

	return result;
}





bool qw_load_config(void)
{
	struct qw_router_config_t config;


#if	defined(USE_RTC_CONFIG)
	rtc_read_backup_register((uint8_t *)&config, sizeof(config));

#elif defined(USE_EFM_CONFIG)
	if (EFM_ReadByte(s_config_address, (uint8_t *)&config, sizeof(config)) != LL_OK) {
		log_printf("EFM_ReadByte failed\n");
		return false;
	} else {

	}

#endif


	bool result = false;

	if (config._magic[0] != QW_CONFIG_MAGIC0
			|| config._magic[1] != QW_CONFIG_MAGIC1
			|| config._magic[2] != QW_CONFIG_MAGIC2
			|| config._magic[3] != QW_CONFIG_MAGIC3) {
		result = (qw_save_config());
	} else {
		memcpy(&g_config, &config, sizeof(g_config));
		result = true;
	}


	qw_dump_config();

	return result;
}

void qw_dump_config(void)
{
	log_printf("magic:%02x%02x%02x%02x\n", g_config._magic[0],
			g_config._magic[1],
			g_config._magic[2],
			g_config._magic[3]);
	log_printf("%s,%s\n", g_config._model, g_config._sn);

	log_printf("mac:%02x:%02x:%02x:%02x:%02x:%02x\n",
			g_config._mac[0], g_config._mac[1], g_config._mac[2],
			g_config._mac[3], g_config._mac[4], g_config._mac[5]);

	log_printf("use_dhcp:%u\n", g_config._use_dhcp);
	log_printf("ip:%u.%u.%u.%u\n", g_config._ip[0], g_config._ip[1], g_config._ip[2], g_config._ip[3]);
	log_printf("mask:%u.%u.%u.%u\n", g_config._mask[0], g_config._mask[1], g_config._mask[2], g_config._mask[3]);
	log_printf("gw:%u.%u.%u.%u\n", g_config._gw[0], g_config._gw[1], g_config._gw[2], g_config._gw[3]);

	log_printf("server:%u.%u.%u.%u:%u\n", g_config._server_ip[0],
			g_config._server_ip[1],
			g_config._server_ip[2],
			g_config._server_ip[3],
			g_config._server_port);

	log_printf("use_wifi:%u\n", g_config._use_wifi);
	log_printf("wifi_ssid:%s,wifi_password:%s\n", g_config._wifi_ssid, g_config._wifi_password);
}



void qw_udp_on_detect(struct qw_udp_context_t *ctx, const uint8_t *addr, uint16_t port)
{
	if (ctx == NULL || addr == NULL) {
		log_printf("%s:%d:invalid parameter\n", __func__, __LINE__);
		return;
	}
	const uint16_t on_detect_data_size = sizeof(g_config._model)+sizeof(g_config._sn);
	uint8_t packet[QW_UDP_SHELF_HEADER_SIZE+on_detect_data_size];
	int index = 0;
	__pack_u8(packet, index, QW_UDP_DETECT_ACK);
	__pack_u16(packet, index, on_detect_data_size);

	__pack_bytes(packet, index, g_config._model, sizeof(g_config._model));
	__pack_bytes(packet, index, g_config._sn, sizeof(g_config._sn));

	log_printf("udp_on_detect:addr:%u.%u.%u.%u,port:%u\n",
			addr[0], addr[1], addr[2], addr[3], port);

	if (ctx->sendto(ctx, addr, port, packet, index) < 0) {
		log_printf("%s:%d:udp_sendto failed\n", __func__, __LINE__);
	} else {
		//log_printf("ctx->sendto ok\n");
	}

	// 收到来自服务器的探测
	g_info._online = 1;
	g_info._ping_interval = QW_SHELF_PING_INTERVAL;

	// FIXME 更新服务器地址端口
#if	0
	g_config._server_ip[0] = addr[0];
	g_config._server_ip[1] = addr[1];
	g_config._server_ip[2] = addr[2];
	g_config._server_ip[3] = addr[3];
	g_config._server_port = port;

//	qw_save_config();
	qw_dump_config();
#endif

}


void qw_udp_log(struct qw_udp_context_t *ctx, const char *s, int n)
{
	if (ctx == NULL || s == NULL || n == 0) {
		return;
	}
	int udp_tx_index = 0;
	__pack_u8(s_udp_tx_packet, udp_tx_index, QW_UDP_LOG);
	__pack_u16(s_udp_tx_packet, udp_tx_index, n);
	__pack_bytes(s_udp_tx_packet, udp_tx_index, s, n);

	if (ctx->sendto(ctx, (const uint8_t *)&g_info._server_addr, g_config._server_port, s_udp_tx_packet, udp_tx_index) < 0) {
		//log_printf("%s:%d:qw_udp_sendto failed\n", __func__, __LINE__);
	} else {
		//log_printf("qw_udp_sendto ok\n");
	}

}


void qw_udp_ping(struct qw_udp_context_t *ctx, const uint8_t *addr, uint16_t port)
{
	if (ctx == NULL || addr == NULL) {
		log_printf("%s:%d:invalid parameter\n", __func__, __LINE__);
		return;
	}
#if	1
	log_printf("qw_udp_ping:%u.%u.%u.%u:%u\n", addr[0], addr[1], addr[2], addr[3], port);
#endif

	int udp_tx_index = 0;
	__pack_u8(s_udp_tx_packet, udp_tx_index, QW_UDP_PING);
	__pack_u16(s_udp_tx_packet, udp_tx_index, sizeof(g_config._sn));
	__pack_bytes(s_udp_tx_packet, udp_tx_index, g_config._sn, sizeof(g_config._sn))

//	__pack_u8(s_udp_tx_packet, udp_tx_index, QW_SHELF_PING_MAGIC_BYTE0);
//	__pack_u8(s_udp_tx_packet, udp_tx_index, QW_SHELF_PING_MAGIC_BYTE1);


	if (ctx->sendto(ctx, addr, port, s_udp_tx_packet, udp_tx_index) < 0) {
		log_printf("%s:%d:qw_udp_sendto failed\n", __func__, __LINE__);
	} else {
		//log_printf("qw_udp_sendto ok\n");
	}


}

// 将货架CAN_PING通过UDP转发至服务器
// 或者通知服务器货架已在线
void qw_udp_forward_shelf_ping(struct qw_udp_context_t *ctx, const uint8_t *addr, uint16_t port, struct qw_shelf_t *shelf)
{
	if (ctx == NULL || addr == NULL || shelf == NULL) {
		log_printf("%s:%d:invalid parameter\n", __func__, __LINE__);
		return;
	}
	if (shelf->_online == 0) {
		// 离线->在线
		shelf->_online = 1;
	}
	shelf->_ping_timestamp = sys_tick_get_infinite_ticks();



	int udp_tx_index = 0;
	__pack_u8(s_udp_tx_packet, udp_tx_index, QW_UDP_SHELF_PING);
	__pack_u16(s_udp_tx_packet, udp_tx_index, 4);
	__pack_u16(s_udp_tx_packet, udp_tx_index, shelf->_id);
	__pack_u8(s_udp_tx_packet, udp_tx_index, QW_SHELF_PING_MAGIC_BYTE0);
	__pack_u8(s_udp_tx_packet, udp_tx_index, QW_SHELF_PING_MAGIC_BYTE1);


	if (ctx->sendto(ctx, addr, port, s_udp_tx_packet, udp_tx_index) < 0) {
		log_printf("%s:%d:qw_udp_sendto failed\n", __func__, __LINE__);
	} else {
		log_printf("ctx->sendto ok:%d\n", udp_tx_index);
	}
}




void qw_udp_on_query_shelves(struct qw_udp_context_t *ctx, const uint8_t *addr, uint16_t port, uint8_t msg_id)
{
	if (ctx == NULL || addr == NULL) {
		log_printf("%s:%d:invalid parameter\n", __func__, __LINE__);
		return;
	}

	// 路由最多支持80个货架
	int udp_tx_index = 0;

	__pack_u8(s_udp_tx_packet, udp_tx_index, QW_UDP_QUERY_SHELVES_ACK);
	__pack_u16(s_udp_tx_packet, udp_tx_index, 0);		// 数据包长度
	// FIXME 兼容老协议
	__pack_u16(s_udp_tx_packet, udp_tx_index, msg_id);
	__pack_bytes(s_udp_tx_packet, udp_tx_index, g_config._sn, sizeof(g_config._sn));
	__pack_u16(s_udp_tx_packet, udp_tx_index, 0);		// 货架个数

	uint16_t shelf_count = 0;

	struct qw_shelf_t *shelf = NULL;
	for (int i = 0; i < QW_SHELF_COUNT_MAX; ++i) {
		shelf = g_shelves+i;
		if (shelf->_id != 0) {
			// 货架编号
			__pack_u16(s_udp_tx_packet, udp_tx_index, shelf->_id);
			// 货架型号
			__pack_bytes(s_udp_tx_packet, udp_tx_index, shelf->_model, sizeof(shelf->_model));
			// 货架特性
			__pack_u8(s_udp_tx_packet, udp_tx_index, shelf->_attribs);
			// 面数
			__pack_u8(s_udp_tx_packet, udp_tx_index, shelf->_plane_count);
			// 每面灯数
			// FIXME 目前协议固定两面
			__pack_u16(s_udp_tx_packet, udp_tx_index, shelf->_light_count[0]);
			__pack_u16(s_udp_tx_packet, udp_tx_index, shelf->_light_count[1]);

			__pack_u16(s_udp_tx_packet, udp_tx_index, (can_get_channel(shelf->_can_ctx)<<12)|(shelf->_online<<11));

			++shelf_count;
		}
	}

	__set_u16(s_udp_tx_packet, 1, udp_tx_index-4);
	__set_u16(s_udp_tx_packet, 5+16, shelf_count);

	if (ctx->sendto(ctx, addr, port, s_udp_tx_packet, udp_tx_index) < 0) {
		log_printf("%s:%d:qw_udp_sendto failed\n", __func__, __LINE__);
	} else {
		log_printf("ctx->sendto ok:%d\n", udp_tx_index);
	}


}




void qw_udp_on_set_slot_lights(struct qw_udp_context_t *ctx, const uint8_t *addr, uint16_t port, const uint8_t *p, int n)
{
	// 注意！
	// 单个UDP数据包最多支持设置200个槽位灯
	// 单个CAN包最多支持50个槽位灯
	// 修改为统一最多支持50个槽位灯
	if (n < 8) {
		return;
	}

	int udp_rx_index = 0;
	uint8_t version, command;
	uint8_t msg_id;
	(void)version;
	(void)command;
	uint16_t length;
	(void)length;
	uint16_t shelf_id;
	__unpack_u8(command, p, udp_rx_index);
	__unpack_u16(length, p, udp_rx_index);

	__unpack_u16(shelf_id, p, udp_rx_index);
	__unpack_u8(msg_id, p, udp_rx_index);

	uint16_t light_count;
	__unpack_u16(light_count, p, udp_rx_index);

	log_printf("%02x,%u,%u,%u,%u\n", command, length, shelf_id, msg_id, light_count);

	if (light_count > QW_UDP_PACKET_SLOT_MAX) {
		log_printf("%s:%d:invalid light_count:%u\n", __func__, __LINE__, light_count);
		return;
	}

	uint8_t udp_tx_packet[] = {
		QW_UDP_SET_SLOT_LIGHT_ACK,
		0x00, 0x04,
		(shelf_id>>8)&0xff, shelf_id&0xff,
		msg_id,
		0,
	};


	struct qw_shelf_t *shelf = NULL;
	shelf = qw_get_shelf_by_shelf_id(shelf_id);
	if (shelf == NULL) {
		log_printf("%s:%d:shelf not found\n", __func__, __LINE__);
		udp_tx_packet[6] = QW_ERROR_SHELF_NOT_FOUND;
		if (ctx->sendto(ctx, addr, port, udp_tx_packet, sizeof(udp_tx_packet)) < 0) {
			log_printf("%s:%d:qw_udp_sendto failed\n", __func__, __LINE__);
		}
		return;
	} else {
		if (!shelf->_online) {
			log_printf("%s:%d:shelf offline\n", __func__, __LINE__);
			udp_tx_packet[6] = QW_ERROR_SHELF_OFFLINE;
			if (ctx->sendto(ctx, addr, port, udp_tx_packet, sizeof(udp_tx_packet)) < 0) {
				log_printf("%s:%d:qw_udp_sendto failed\n", __func__, __LINE__);
			}
			return;
		}

	}

#if	0
	uint16_t slot_id;
	uint8_t slot_state;
#endif

	uint16_t slot_id_state;


	uint8_t slot_r, slot_g, slot_b;
	uint16_t can_light_index = 0;
	int can_tx_index = 0;

	can_tx_index = 0;
	__pack_u8(s_can_tx_packet, can_tx_index, QW_UDP_SET_SLOT_LIGHT);
	__pack_u16(s_can_tx_packet, can_tx_index, 0);
	__pack_u8(s_can_tx_packet, can_tx_index, msg_id);

	for (int i = 0; i < light_count; ++i) {
		if (udp_rx_index+5 <= n) {
#if	0
			__unpack_u16(slot_id, p, udp_rx_index);
			__unpack_u8(slot_state, p, udp_rx_index);
#endif
			__unpack_u16(slot_id_state, p, udp_rx_index);
			__unpack_u8(slot_r, p, udp_rx_index);
			__unpack_u8(slot_g, p, udp_rx_index);
			__unpack_u8(slot_b, p, udp_rx_index);

//			__pack_u16(can_tx_packet, can_tx_index, (slot_id<<4)|slot_state);
			__pack_u16(s_can_tx_packet, can_tx_index, slot_id_state);
			__pack_u8(s_can_tx_packet, can_tx_index, slot_r);
			__pack_u8(s_can_tx_packet, can_tx_index, slot_g);
			__pack_u8(s_can_tx_packet, can_tx_index, slot_b);

			++can_light_index;
		} else {
			break;
		}
	}

	uint16_t tx_n = 1+can_light_index*5;
	s_can_tx_packet[1] = (tx_n>>8)&0xff;
	s_can_tx_packet[2] = (tx_n)&0xff;

	if (!can_tx_data_frames(shelf->_can_ctx, 1,
				QW_ROUTER_CAN_ID_MASK|shelf->_id, s_can_tx_packet, can_tx_index)) {
		log_printf("%s:%d:can_tx_data_frames failed\n", __func__, __LINE__);
		udp_tx_packet[6] = QW_ERROR_CAN_BUS_ERROR;
		if (ctx->sendto(ctx, addr, port, udp_tx_packet, sizeof(udp_tx_packet)) < 0) {
			log_printf("%s:%d:qw_udp_sendto failed\n", __func__, __LINE__);
		}
	} else {
		// 等待货架回应，超时交给上层服务器处理

#if	0
		// 立刻响应，不管货架有没有真正响应
		// 如果需要保证可靠性，则需网关进行重发处理
		udp_tx_packet[7] = QW_ERROR_OK;
		if (!qw_udp_sendto(pcb, addr, port, udp_tx_packet, sizeof(udp_tx_packet))) {
			log_printf("%s:%d:qw_udp_sendto failed\n", __func__, __LINE__);
		}
#endif
	}
}

void qw_udp_on_set_alarm_light(struct qw_udp_context_t *ctx, const uint8_t *addr, uint16_t port, const uint8_t *p, int n)
{
//	log_printf("qw_udp_on_set_alarm_light\n");
//	return;
	// 协议版本，命令，长度，货架号，指示灯号，指示灯状态
	if (n < 7) {
		return;
	}
	int rx_index = 0;
	uint8_t version, command;
	uint8_t msg_id;
	(void)version;
	(void)command;
	uint16_t length;
	(void)length;
	uint16_t shelf_id;
	__unpack_u8(command, p, rx_index);
	__unpack_u16(length, p, rx_index);

	__unpack_u16(shelf_id, p, rx_index);
	__unpack_u8(msg_id, p, rx_index);

	uint8_t udp_tx_packet[] = {
		QW_UDP_SET_ALARM_LIGHT_ACK,
		0x00, 0x04,
		(shelf_id>>8)&0xff, shelf_id&0xff,
		msg_id,
		0,
	};


	struct qw_shelf_t *shelf = NULL;
	shelf = qw_get_shelf_by_shelf_id(shelf_id);
	if (shelf == NULL) {
		log_printf("%s:%d:shelf not found\n", __func__, __LINE__);

		udp_tx_packet[6] = QW_ERROR_SHELF_NOT_FOUND;
		if (ctx->sendto(ctx, addr, port, udp_tx_packet, sizeof(udp_tx_packet)) < 0) {
			log_printf("%s:%d:qw_udp_sendto failed\n", __func__, __LINE__);
		}
		return;
	} else {
		if (!shelf->_online) {
			log_printf("%s:%d:shelf offline\n", __func__, __LINE__);
			udp_tx_packet[6] = QW_ERROR_SHELF_OFFLINE;
			if (ctx->sendto(ctx, addr, port, udp_tx_packet, sizeof(udp_tx_packet)) < 0) {
				log_printf("%s:%d:qw_udp_sendto failed\n", __func__, __LINE__);
			}
			return;
		}
	}


#if	0
	uint8_t light_id;
	uint8_t light_state;

	__unpack_u8(light_id, p, rx_index);
	__unpack_u8(light_state, p, rx_index);
#endif

	uint8_t light_id_state;
	__unpack_u8(light_id_state, p, rx_index);

	uint8_t can_tx_packet[8];
	int can_tx_index = 0;

	__pack_u8(can_tx_packet, can_tx_index, QW_UDP_SET_ALARM_LIGHT);
	__pack_u16(can_tx_packet, can_tx_index, 2);
	__pack_u8(can_tx_packet, can_tx_index, msg_id);
	__pack_u8(can_tx_packet, can_tx_index, light_id_state);

	if (!can_tx_data_frames(shelf->_can_ctx, 1, QW_ROUTER_CAN_ID_MASK|shelf->_id, can_tx_packet, can_tx_index)) {
		log_printf("%s:%d:can_tx_data_frames failed\n", __func__, __LINE__);
		udp_tx_packet[6] = QW_ERROR_CAN_BUS_ERROR;
		if (ctx->sendto(ctx, addr, port, udp_tx_packet, sizeof(udp_tx_packet)) < 0) {
			log_printf("%s:%d:qw_udp_sendto failed\n", __func__, __LINE__);
		}
	} else {
		// 等待货架回应，超时交给上层服务器处理

#if	0
		// 立刻响应，不管货架有没有真正响应
		// 如果需要保证可靠性，则需网关进行重发处理
		udp_tx_packet[7] = QW_ERROR_OK;
		if (!qw_udp_sendto(pcb, addr, port, udp_tx_packet, sizeof(udp_tx_packet))) {
			log_printf("%s:%d:qw_udp_sendto failed\n", __func__, __LINE__);
		}
#endif
	}
}

void qw_on_udp_packet(struct qw_udp_context_t *ctx, const uint8_t *addr, uint16_t port, const uint8_t *p, int n)
{
	uint8_t *payload = NULL;
#if	1
	if (p != NULL) {
		log_printf("[%u.%u.%u.%u]:%u->[%d]%p ", addr[0], addr[1], addr[2], addr[3],
				port, n, p);
	}

#if	0
	for (int i = 0; i < n; ++i) {
		log_printf("%02x ", p[i]);
	}
#endif

	printf("\n");
#endif

	if (ctx == NULL || addr == NULL || p == NULL || n < 3) {
		log_printf("%s:%d:invalid parameter\n", __func__, __LINE__);
		return;
	}

	int udp_rx_index = 0;
	payload = (uint8_t *)p;
	uint8_t version;
	uint8_t command;
	uint8_t msg_id;
	uint16_t size;
	(void)version;
	(void)msg_id;
	(void)size;
	__unpack_u8(command, payload, udp_rx_index);
	__unpack_u16(size, payload, udp_rx_index);

	switch (command) {
	case QW_UDP_SET_SLOT_LIGHT:
		{
			qw_udp_on_set_slot_lights(ctx, addr, port, p, n);
		}
		break;

	case QW_UDP_SET_ALARM_LIGHT:
		{
			qw_udp_on_set_alarm_light(ctx, addr, port, p, n);
		}
		break;

	case QW_UDP_DETECT:
		{
			qw_udp_on_detect(ctx, addr, port);
		}
		break;

	case QW_UDP_QUERY_SHELVES:
		{
			if (udp_rx_index+2 <= n) {
				// FIXME 兼容老协议
				__unpack_u16(msg_id, payload, udp_rx_index);
				qw_udp_on_query_shelves(ctx, addr, port, msg_id);
			}
		}
		break;

	case QW_UDP_PING_ACK:
		{
			// 已与服务器建立连接
			char sn[16];
			__unpack_bytes(sn, sizeof(sn), payload, udp_rx_index);
			if (memcmp(sn, g_config._sn, sizeof(g_config._sn)) == 0) {
				g_info._online = 1;
				g_info._ping_interval = QW_SHELF_PING_INTERVAL;
			} else {

			}
		}
		break;

	case QW_UDP_OPERATE:
		{
			if (udp_rx_index+2 <= n) {
				uint8_t operate_type;
				__unpack_u8(msg_id, payload, udp_rx_index);
				__unpack_u8(operate_type, payload, udp_rx_index);

				printf("udp_operate:%u,%u\n", msg_id, operate_type);
				switch (operate_type) {
				case QW_UDP_OPERATE_TEST:
					{
						uint8_t packet[QW_UDP_SHELF_HEADER_SIZE+3];
						int index = 0;
						__pack_u8(packet, index, QW_UDP_OPERATE_ACK);
						__pack_u16(packet, index, 3);

						__pack_u8(packet, index, msg_id);
						__pack_u8(packet, index, QW_UDP_OPERATE_TEST);
						__pack_u8(packet, index, 0);

						if (ctx->sendto(ctx, addr, port, packet, index) < 0) {
							log_printf("%s:%d:udp_sendto failed\n", __func__, __LINE__);
						} else {
							//log_printf("ctx->sendto ok\n");
						}
					}
					break;

				case QW_UDP_OPERATE_RESET:
					{
						uint8_t packet[QW_UDP_SHELF_HEADER_SIZE+2];
						int index = 0;
						__pack_u8(packet, index, QW_UDP_OPERATE_ACK);
						__pack_u16(packet, index, 3);

						__pack_u8(packet, index, msg_id);
						__pack_u8(packet, index, QW_UDP_OPERATE_RESET);
						__pack_u8(packet, index, 0);

						if (ctx->sendto(ctx, addr, port, packet, index) < 0) {
							log_printf("%s:%d:udp_sendto failed\n", __func__, __LINE__);
						} else {
							//log_printf("ctx->sendto ok\n");
						}

						g_info._reset = 1;
						g_info._reset_timestamp = sys_tick_get_infinite_ticks();
					}
					break;

				default:
					break;
				}
			}
		}
		break;


	default:
		break;
	}

}


void qw_can_discover(void)
{
	// 注意：CAN口出现总线错误会影响另外总线的发送
	// 而在发送函数之前延迟一段时间则可以发送成功
	//
	//
	// 可能原因是CAN中断调试输出导致中断未被响应，
	// 去掉调试输出功能正常
	log_printf("qw_can_discover\n");
	const uint8_t data[] = {QW_CAN_CMD_INFO, 0x00, 0x00};
	can_tx_data_frames(&_can1_ctx, 1, QW_ROUTER_CAN_ID_MASK, data, sizeof(data));
	can_tx_data_frames(&_can2_ctx, 1, QW_ROUTER_CAN_ID_MASK, data, sizeof(data));
}

void qw_can_detect(can_context_t *ctx, uint16_t shelf_id)
{
	log_printf("qw_can_detect\n");
	const uint8_t data[] = {QW_CAN_CMD_INFO, 0x00, 0x00};
	can_tx_data_frames(ctx, 1, QW_ROUTER_CAN_ID_MASK|shelf_id, data, sizeof(data));
}

void qw_can_ping(int64_t now)
{
	log_printf("qw_can_ping:%d,%d\n",
			can_get_bus_off(_can1_ctx._can),
			can_get_bus_off(_can2_ctx._can)
			);

	// 判断设备是否离线
	struct qw_shelf_t *shelf = NULL;
	for (int i = 0; i < QW_SHELF_COUNT_MAX; ++i) {
		shelf = g_shelves + i;
		if (shelf->_id != 0) {
			if (shelf->_online == 1) {
				if (now - shelf->_ping_timestamp >= QW_SHELF_CAN_PING_INTERVAL*3) {
					// 在线->离线
					shelf->_online = 0;
				}
			}
		}
	}

	const uint8_t data[] = {QW_CAN_CMD_PING, 0x00, 0x00};
	can_tx_data_frames(&_can1_ctx, 1, QW_ROUTER_CAN_ID_MASK, data, sizeof(data));
	can_tx_data_frames(&_can2_ctx, 1, QW_ROUTER_CAN_ID_MASK, data, sizeof(data));
}




void qw_on_can_frame(can_context_t *ctx, can_frame_t *f, struct qw_udp_context_t *udp_ctx)
{
	if (!(f->control&(1<<CAN_TBUF_CONTROL_IDE_Pos)) || f->ext_id > 0xffff) {
		return;
	}

	uint16_t shelf_id = f->ext_id&0xffff;
	struct qw_shelf_t *shelf = qw_get_shelf_by_shelf_id(shelf_id);
	if (shelf == NULL) {
		// 货架不存在？
		shelf = qw_shelf_alloc();
		if (shelf == NULL) {
			return;
		}
		shelf->_id = shelf_id;
		shelf->_can_ctx = ctx;
	}

	if (shelf != NULL) {
		qw_shelf_on_can_frame(shelf, f, udp_ctx);
	}
}




void qw_shelf_on_can_frame(struct qw_shelf_t *shelf, can_frame_t *f, struct qw_udp_context_t *udp_ctx)
{
	uint8_t dlc = ((f->control&CAN_RBUF_CONTROL_DLC_Msk)>>CAN_RBUF_CONTROL_DLC_Pos);
//	log_printf("can:%u,%u,%u\n", shelf->_id, shelf->_can_id, dlc);
	if (shelf->_rx_state == 0) {
		memcpy(shelf->_rx_packet, f->data, dlc);
		shelf->_rx_index = dlc;
		shelf->_rx_size = 3+((uint16_t)shelf->_rx_packet[1]<<8)+shelf->_rx_packet[2];

	//	log_printf("$$$%d\n", shelf->_rx_size);

		// 超过一个数据帧
		if (shelf->_rx_size > 8) {
			// 继续接收
			shelf->_rx_state = 1;
		} else {
			qw_shelf_on_can_packet(shelf, udp_ctx);
		}
	} else if (shelf->_rx_state == 1) {
		memcpy(shelf->_rx_packet+shelf->_rx_index, f->data, dlc);
		shelf->_rx_index += dlc;
		if (shelf->_rx_index >= shelf->_rx_size) {
			qw_shelf_on_can_packet(shelf, udp_ctx);
			shelf->_rx_state = 0;
		}

	} else {
		shelf->_rx_state = 0;
	}
}

void qw_shelf_on_can_packet(struct qw_shelf_t *shelf, struct qw_udp_context_t *udp_ctx)
{
	if (shelf->_rx_size < 3) {
		log_printf("%s:%d:invalid can packet size:%"PRIu32"\n", __func__, __LINE__, shelf->_rx_size);
		return;
	}
//	log_printf("qw_shelf_on_can_packet\n");
	uint8_t *rx_packet = shelf->_rx_packet;
	uint8_t can_rx_index = 0;

	uint8_t command;
	uint16_t size;
	(void)size;
	__unpack_u8(command, rx_packet, can_rx_index);
	__unpack_u16(size, rx_packet, can_rx_index);


	switch (command) {
	case QW_CAN_CMD_SLOT_LIGHT_ACK:
		{
			if (shelf->_rx_size < 5) {
				log_printf("%s:%d:invalid can packet size:%"PRIu32"\n", __func__, __LINE__, shelf->_rx_size);
				return;
			}
			// 设置储位灯状态完成
			uint8_t msg_id;
			__unpack_u8(msg_id, rx_packet, can_rx_index);
			__unpack_u8(shelf->_error_code, rx_packet, can_rx_index);

			log_printf("msg_id:%u,error_code:%u\n", msg_id, shelf->_error_code);

			uint8_t udp_tx_packet[] = {
				QW_UDP_SET_SLOT_LIGHT_ACK,
				0x00, 0x04,
				(shelf->_id>>8)&0xff, (shelf->_id)&0xff,
				msg_id,
				shelf->_error_code,
			};
			if (udp_ctx->sendto(udp_ctx, (const uint8_t *)&g_info._server_addr, g_config._server_port, udp_tx_packet, sizeof(udp_tx_packet)) < 0) {
				log_printf("%s:%d:qw_udp_sendto failed\n", __func__, __LINE__);
			}
		}
		break;


	case QW_CAN_CMD_PING_ACK:
		{
			if (shelf->_model[0] == '\0') {
				// 无货架信息，则请求
				qw_can_detect(shelf->_can_ctx, shelf->_id);
			} else {
				// 向服务器发送SHELF_PING
				qw_udp_forward_shelf_ping(udp_ctx, (const uint8_t *)&g_info._server_addr, g_config._server_port, shelf);
			}
		}
		break;


	case QW_CAN_CMD_ALARM_LIGHT_ACK:
		{
			if (shelf->_rx_size < 5) {
				log_printf("%s:%d:invalid can packet size:%"PRIu32"\n", __func__, __LINE__, shelf->_rx_size);
				return;
			}

			// 设置报警灯状态完成
			uint8_t msg_id;
			__unpack_u8(msg_id, rx_packet, can_rx_index);
			__unpack_u8(shelf->_error_code, rx_packet, can_rx_index);

			log_printf("msg_id:%u,error_code:%u\n", msg_id, shelf->_error_code);

			uint8_t udp_tx_packet[] = {
				QW_UDP_SET_ALARM_LIGHT_ACK,
				0x00, 0x04,
				(shelf->_id>>8)&0xff, (shelf->_id)&0xff,
				msg_id,
				shelf->_error_code,
			};
			if (udp_ctx->sendto(udp_ctx, (const uint8_t *)&g_info._server_addr, g_config._server_port, udp_tx_packet, sizeof(udp_tx_packet)) < 0) {
				log_printf("%s:%d:qw_udp_sendto failed\n", __func__, __LINE__);
			}
		}
		break;


	case QW_CAN_CMD_INFO_ACK:
		{
			const uint8_t k_info_ack_data_size = 16;
			if (shelf->_rx_size < 3+k_info_ack_data_size) {
				log_printf("%s:%d:invalid can packet size:%"PRIu32"\n", __func__, __LINE__, shelf->_rx_size);
				return;
			}

			// 设置货架信息
			uint16_t shelf_id;
			__unpack_u16(shelf_id, rx_packet, can_rx_index);

			if (shelf->_id == shelf_id) {
				// 向服务器发送SHELF_PING
				qw_udp_forward_shelf_ping(udp_ctx, (const uint8_t *)&g_info._server_addr, g_config._server_port, shelf);

				__unpack_bytes(shelf->_model, sizeof(shelf->_model), rx_packet, can_rx_index);
				__unpack_u8(shelf->_attribs, rx_packet, can_rx_index);
				__unpack_u8(shelf->_plane_count, rx_packet, can_rx_index);

				// FIXME 目前协议固定两面
				//for (int i = 0; i < shelf->_plane_count; ++i) {
				for (int i = 0; i < 2; ++i) {
					__unpack_u16(shelf->_light_count[i], rx_packet, can_rx_index);
				}

				log_printf("[%d]%d,%s,%u,%u,(%u,%u)\n", shelf->_id, shelf_id, shelf->_model, shelf->_attribs,
						shelf->_plane_count, shelf->_light_count[0], shelf->_light_count[1]);
			} else {
				log_printf("%s:%d:invalid shelf id:should be %d<->but %d\n", __func__, __LINE__, shelf->_id, shelf_id);
			}
		}
		break;

	default:
		break;
	}
}
