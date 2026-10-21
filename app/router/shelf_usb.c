#include "shelf_usb.h"
#include "shelf.h"
#include "board.h"
#include "app.h"
#include "rtc.h"

#include "usb_dev_user.h"
#include "usb_dev_desc.h"
#include "usb_dev_cdc_class.h"
#include "inttypes.h"

#include "esp_at_parser.h"
extern struct at_context_t g_at_ctx;

uint8_t s_usb_rx_data[USB_RX_BUFFER_SIZE];

uint8_t s_usb_rx_state;
uint8_t s_usb_rx_index;


uint16_t s_usb_rx_size;
uint8_t s_usb_command;


uint8_t s_usb_tx_buffer[8+USB_RX_BUFFER_SIZE];


extern usb_core_instance g_usb_dev;
static int64_t s_last_packet_ticks;
extern struct netif g_netif;

uint32_t efm_get_chip_id(void)
{
	return READ_REG32(CM_EFM->CHIPID);
}

void usbcan_handle_packet(void)
{
	switch (s_usb_command) {
	case USB_ROUTER_TEST:
		{
			log_printf("USB_ROUTER_TEST\n");
			int index = 0;
			__pack_u8(s_usb_tx_buffer, index, USB_ROUTER_VERSION);
			__pack_u8(s_usb_tx_buffer, index, USB_ROUTER_TEST_ACK);
			__pack_u16(s_usb_tx_buffer, index, s_usb_rx_size);
			int j;
			for (j = 0; j < s_usb_rx_size; ++j) {
				s_usb_tx_buffer[4+j] = ~s_usb_rx_data[j];
			}
			usb_dev_cdc_send(&g_usb_dev, s_usb_tx_buffer, 4+j);
		}
		break;

	case USB_ROUTER_GET_CONFIG:
		{
			log_printf("USB_ROUTER_GET_CONFIG\n");
			int index = 0;
			__pack_u8(s_usb_tx_buffer, index, USB_ROUTER_VERSION);
			__pack_u8(s_usb_tx_buffer, index, USB_ROUTER_GET_CONFIG_ACK);
			uint16_t n = USB_CONFIG_DATA_SIZE;
			__pack_u16(s_usb_tx_buffer, index, n);

			__pack_bytes(s_usb_tx_buffer, index, g_config._model, sizeof(g_config._model));
			__pack_bytes(s_usb_tx_buffer, index, g_config._sn, sizeof(g_config._sn));
			__pack_bytes(s_usb_tx_buffer, index, g_config._mac, sizeof(g_config._mac));
			__pack_bytes(s_usb_tx_buffer, index, g_config._ip, sizeof(g_config._ip));
			__pack_bytes(s_usb_tx_buffer, index, g_config._mask, sizeof(g_config._mask));
			__pack_bytes(s_usb_tx_buffer, index, g_config._gw, sizeof(g_config._gw));
			__pack_bytes(s_usb_tx_buffer, index, g_config._server_ip, sizeof(g_config._server_ip));
			__pack_u16(s_usb_tx_buffer, index, g_config._server_port);

			__pack_u8(s_usb_tx_buffer, index, g_config._use_dhcp);
			__pack_u8(s_usb_tx_buffer, index, g_config._use_wifi);
			__pack_bytes(s_usb_tx_buffer, index, g_config._wifi_ssid, sizeof(g_config._wifi_ssid));
			__pack_bytes(s_usb_tx_buffer, index, g_config._wifi_password, sizeof(g_config._wifi_password));

			log_printf("index:%d\n", index);

			usb_dev_cdc_send_ex(&g_usb_dev, s_usb_tx_buffer, index);
		}
		break;

	case USB_ROUTER_SET_CONFIG:
		{
			log_printf("USB_ROUTER_SET_INFO\n");
			struct qw_router_config_t config;
			memcpy(&config, &g_config, sizeof(g_config));

			if (s_usb_rx_size != USB_CONFIG_DATA_SIZE) {
				log_printf("s_usb_rx_data:%u\n", s_usb_rx_size);
				return;
			}

			int rx_index = 0;
			__unpack_bytes(config._model, sizeof(config._model), s_usb_rx_data, rx_index);
			__unpack_bytes(config._sn, sizeof(config._sn), s_usb_rx_data, rx_index);
			__unpack_bytes(config._mac, sizeof(config._mac), s_usb_rx_data, rx_index);
			__unpack_bytes(config._ip, sizeof(config._ip), s_usb_rx_data, rx_index);
			__unpack_bytes(config._mask, sizeof(config._mask), s_usb_rx_data, rx_index);
			__unpack_bytes(config._gw, sizeof(config._gw), s_usb_rx_data, rx_index);
			__unpack_bytes(config._server_ip, sizeof(config._server_ip), s_usb_rx_data, rx_index);
			__unpack_u16(config._server_port, s_usb_rx_data, rx_index);

			__unpack_u8(config._use_dhcp, s_usb_rx_data, rx_index);
			__unpack_u8(config._use_wifi, s_usb_rx_data, rx_index);
			__unpack_bytes(config._wifi_ssid, sizeof(config._wifi_ssid), s_usb_rx_data, rx_index);
			__unpack_bytes(config._wifi_password, sizeof(config._wifi_password), s_usb_rx_data, rx_index);

			memcpy(&g_config, &config, sizeof(g_config));
			bool result = qw_save_config();
			qw_dump_config();

			int index = 0;
			__pack_u8(s_usb_tx_buffer, index, USB_ROUTER_VERSION);
			__pack_u8(s_usb_tx_buffer, index, USB_ROUTER_SET_CONFIG_ACK);
			__pack_u16(s_usb_tx_buffer, index, 1);

			__pack_u8(s_usb_tx_buffer, index, result ? 0 : 1);

			usb_dev_cdc_send(&g_usb_dev, s_usb_tx_buffer, index);


			// 设置WiFi
			if (g_config._use_wifi) {
				// station
				const char *cmd = "AT+CWMODE=1\r\n";
				esp_at_try_tx_command(&g_at_ctx, cmd, strlen(cmd), NULL, NULL, NULL, NULL, 0);
				// ssid,password
				char buffer[128];
				int n;
				n = snprintf(buffer, sizeof(buffer), "AT+CWJAP=\"%s\",\"%s\"\r\n",
						g_config._wifi_ssid, g_config._wifi_password);

				esp_at_try_tx_command(&g_at_ctx, buffer, n, NULL, NULL, NULL, NULL, 0);
			} else {
				// softap
				const char *cmd = "AT+CWMODE=2\r\n";
				esp_at_try_tx_command(&g_at_ctx, cmd, strlen(cmd), NULL, NULL, NULL, NULL, 0);
			}
		}
		break;

	case USB_ROUTER_GET_INFO:
		{
			log_printf("USB_ROUTER_GET_INFO\n");
			int index = 0;
			__pack_u8(s_usb_tx_buffer, index, USB_ROUTER_VERSION);
			__pack_u8(s_usb_tx_buffer, index, USB_ROUTER_GET_INFO_ACK);
			uint16_t n = USB_INFO_DATA_SIZE;
			__pack_u16(s_usb_tx_buffer, index, n);

			stc_efm_unique_id_t unique_id;
			EFM_GetUID(&unique_id);
			uint32_t chip_id = efm_get_chip_id();

			log_printf("chip_id:%08"PRIx32"\n", chip_id);
			/*
			   unique_id_0
			   50323357	芯片LOT number(批次号)
			   unique_id_1
			   00		固定全为0
			   11		Wafer(晶圆) number
			   0000	芯片LOT number
			   unique_id_2
			   0000	固定全为0
			   13		芯片在Wafer上Y坐标	19
			   1f		芯片在Wafer上X坐标	31
			   */
			log_printf("unique_id:%08"PRIx32" %08"PRIx32" %08"PRIx32"\n", unique_id.u32UniqueID0, unique_id.u32UniqueID1, unique_id.u32UniqueID2);



			__pack_u32(s_usb_tx_buffer, index, chip_id);
			__pack_u32(s_usb_tx_buffer, index, unique_id.u32UniqueID0);
			__pack_u32(s_usb_tx_buffer, index, unique_id.u32UniqueID1);
			__pack_u32(s_usb_tx_buffer, index, unique_id.u32UniqueID2);

			const ip4_addr_t *ip = netif_ip4_addr(&g_netif);
			const ip4_addr_t *mask = netif_ip4_netmask(&g_netif);
			const ip4_addr_t *gw = netif_ip4_gw(&g_netif);

			__pack_bytes(s_usb_tx_buffer, index, ip, sizeof(ip4_addr_t));
			__pack_bytes(s_usb_tx_buffer, index, mask, sizeof(ip4_addr_t));
			__pack_bytes(s_usb_tx_buffer, index, gw, sizeof(ip4_addr_t));

			stc_rtc_date_t rtc_date = {0, 0, 0, 0};
			stc_rtc_time_t rtc_time = {0, 0, 0, 0};


			if (LL_OK == RTC_GetDate(RTC_DATA_FMT_DEC, &rtc_date)) {
				if (LL_OK == RTC_GetTime(RTC_DATA_FMT_DEC, &rtc_time)) {
					log_printf("20%02d/%02d/%02d %02d:%02d:%02d %u\n", rtc_date.u8Year, rtc_date.u8Month,
							rtc_date.u8Day, rtc_time.u8Hour,
							rtc_time.u8Minute, rtc_time.u8Second, rtc_date.u8Weekday);
				} else {
					log_printf("RTC_GetTime failed\n");
				}
			} else {
				log_printf("RTC_GetDate failed\n");
			}

			__pack_u16(s_usb_tx_buffer, index, 2000+rtc_date.u8Year);
			__pack_u8(s_usb_tx_buffer, index, rtc_date.u8Month);
			__pack_u8(s_usb_tx_buffer, index, rtc_date.u8Day);
			__pack_u8(s_usb_tx_buffer, index, rtc_date.u8Weekday);
			__pack_u8(s_usb_tx_buffer, index, rtc_time.u8Hour);
			__pack_u8(s_usb_tx_buffer, index, rtc_time.u8Minute);
			__pack_u8(s_usb_tx_buffer, index, rtc_time.u8Second);



			usb_dev_cdc_send(&g_usb_dev, s_usb_tx_buffer, index);
		}
		break;

	case USB_ROUTER_SET_TIME:
		{
			log_printf("USB_ROUTER_SET_TIME\n");

			if (s_usb_rx_size != USB_TIME_DATA_SIZE) {
				log_printf("s_usb_rx_data:%u\n", s_usb_rx_size);
				return;
			}

			uint16_t year;
			uint8_t month, day, wday, hour, minute, second;
			(void)wday;
			int rx_index = 0;
			__unpack_u16(year, s_usb_rx_data, rx_index);
			__unpack_u8(month, s_usb_rx_data, rx_index);
			__unpack_u8(day, s_usb_rx_data, rx_index);
			__unpack_u8(wday, s_usb_rx_data, rx_index);
			__unpack_u8(hour, s_usb_rx_data, rx_index);
			__unpack_u8(minute, s_usb_rx_data, rx_index);
			__unpack_u8(second, s_usb_rx_data, rx_index);


			RTC_Cmd(DISABLE);
			rtc_set_date_time(year, month, day, wday, hour, minute, second);
			RTC_Cmd(ENABLE);


			int index = 0;
			__pack_u8(s_usb_tx_buffer, index, USB_ROUTER_VERSION);
			__pack_u8(s_usb_tx_buffer, index, USB_ROUTER_SET_TIME_ACK);
			__pack_u16(s_usb_tx_buffer, index, 1);

			__pack_u8(s_usb_tx_buffer, index, 0);

			usb_dev_cdc_send(&g_usb_dev, s_usb_tx_buffer, index);
		}
		break;



	case USB_ROUTER_RESET:
		{
			for (;;) {}
		}
		break;

	default:
		break;
	}
}

void usb_dev_cdc_on_recv(const uint8_t *p, uint32_t n)
{
	(void)s_last_packet_ticks;

	// 同一逻辑包数据间隔不能超过50ms
	volatile int delta = sys_tick_get_infinite_ticks() - s_last_packet_ticks;
//	log_printf("delta:%d\n", delta);
	if (s_last_packet_ticks > 0 && delta >= 50) {
		s_usb_rx_state = 0;
	}
	s_last_packet_ticks = sys_tick_get_infinite_ticks();

	log_printf("usb_dev_cdc_on_recv[%"PRIu32"]\n", n);
	uint8_t b;
	for (uint16_t i = 0; i < n; ++i) {
		log_printf("%02x ", p[i]);

		b = p[i];
		switch (s_usb_rx_state) {
		case 0:
			{
				if (b == USB_ROUTER_VERSION) {
					s_usb_rx_state = 1;
				}
			}
			break;

		case 1:
			{
				s_usb_command = b;
				s_usb_rx_state = 2;
			}
			break;

		case 2:
			{
				s_usb_rx_size = b;
				s_usb_rx_size = (s_usb_rx_size<<8);
				s_usb_rx_state = 3;
			}
			break;
		
		case 3:
			{
				s_usb_rx_size |= b;
				if (b <= USB_RX_BUFFER_SIZE) {
					if (s_usb_rx_size == 0) {
						usbcan_handle_packet();
						s_usb_rx_state = 0;
					} else {
						s_usb_rx_state = 4;
						s_usb_rx_index = 0;
					}
				} else {
					s_usb_rx_state = 0;
				}
			}
			break;

		case 4:
			{
				s_usb_rx_data[s_usb_rx_index++] = b;
				if (s_usb_rx_index == s_usb_rx_size) {
					usbcan_handle_packet();
					s_usb_rx_state = 0;
				}
			}
			break;

		default:
			break;
		}
	}
	log_printf("\n");
}


