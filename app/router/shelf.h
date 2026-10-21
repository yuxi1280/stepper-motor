#ifndef SHELF_H
#define SHELF_H

#include <stdint.h>

#include "can_context.h"

#include "lwip/pbuf.h"
#include "lwip/udp.h"

struct qw_udp_context_t;
typedef int (*QW_UDP_OPEN)(struct qw_udp_context_t *ctx);
typedef int (*QW_UDP_SENDTO)(struct qw_udp_context_t *ctx, const uint8_t *addr, uint16_t port, const uint8_t *p, int n);
typedef int (*QW_UDP_CLOSE)(struct qw_udp_context_t *ctx);

struct qw_udp_context_t {
	QW_UDP_OPEN open;
	QW_UDP_SENDTO sendto;
	QW_UDP_CLOSE close;

	int _state;
	uint8_t _addr[4];
	uint16_t _port;
	void *_ll_ctx;
};


#define __pack_u8(p, i, u)\
{\
	(p)[(i)] = (u);\
	++(i);\
}

#define __unpack_u8(u, p, i)\
{\
	(u) = (p)[(i)];\
	++(i);\
}

#define __pack_u16(p, i, u)\
{\
	(p)[(i)+0] = ((u)>>8)&0xff;\
	(p)[(i)+1] = (u)&0xff;\
	(i) += 2;\
}

#define __pack_u32(p, i, u)\
{\
	(p)[(i)+0] = ((u)>>24)&0xff;\
	(p)[(i)+1] = ((u)>>16)&0xff;\
	(p)[(i)+2] = ((u)>>8)&0xff;\
	(p)[(i)+3] = (u)&0xff;\
	(i) += 4;\
}


#define __unpack_u16(u, p, i)\
{\
	(u) = (((uint16_t)(p)[(i)])<<8) | (p)[(i)+1];\
	(i) += 2;\
}


#define __set_u16(p, i, u)\
{\
	(p)[(i)] = ((u)>>8)&0xff;\
	(p)[(i)+1] = (u)&0xff;\
}


#define __pack_bytes(p, i, data, n)\
{\
	memcpy((p)+(i), (data), (n));\
	(i) += (n);\
}

#define __unpack_bytes(data, n, p, i)\
{\
	memcpy((data), (p)+(i), (n));\
	(i) += (n);\
}




#define QW_UDP_PROTOCOL_VERSION				0x01

// UDP包最大尺寸
// 1472=ETHERNET_MTU(1500)-IP(20)-UDP(8)
//#define QW_UDP_PACKET_SIZE					1286
#define QW_UDP_PACKET_SIZE					1472

#define QW_UDP_SHELF_HEADER_SIZE			4

#define QW_UDP_PACKET_SLOT_MAX				250

enum {
	QW_UDP_DETECT				= 0x00,
	QW_UDP_DETECT_ACK			= 0x80,

	QW_UDP_QUERY_SHELVES		= 0x01,
	QW_UDP_QUERY_SHELVES_ACK	= 0x81,

	QW_UDP_DATA					= 0x02,
	QW_UDP_ALARM				= 0x03,
	QW_UDP_FAULT				= 0x04,

	QW_UDP_OPERATE				= 0x05,
	QW_UDP_OPERATE_ACK			= 0x85,

	QW_UDP_SET_SLOT_LIGHT		= 0x06,
	QW_UDP_SET_SLOT_LIGHT_ACK	= 0x86,

	QW_UDP_SET_ALARM_LIGHT		= 0x07,
	QW_UDP_SET_ALARM_LIGHT_ACK	= 0x87,

	QW_UDP_PING					= 0x08,
	QW_UDP_PING_ACK				= 0x88,

	QW_UDP_SHELF_PING			= 0x09,
	QW_UDP_SHELF_PING_ACK		= 0x89,

	QW_UDP_LOG					= 0x0a,
	QW_UDP_LOG_ACK				= 0x8a,

	QW_UDP_CALIB				= 0x0b,
	QW_UDP_CONFIG				= 0x0c,

};

enum {
	QW_UDP_OPERATE_TEST			= 0,
	QW_UDP_OPERATE_RESET		= 1,
};


#define QW_ROUTER_CAN_ID_MASK		0x10000000

enum {
	QW_CAN_CMD_INFO				= 0x01,
	QW_CAN_CMD_INFO_ACK			= 0x81,

	QW_CAN_CMD_DATA				= 0x02,
	QW_CAN_CMD_DATA_ACK			= 0x82,
	QW_CAN_CMD_FAULT			= 0x03,
	QW_CAN_CMD_FAULT_ACK		= 0x83,
	QW_CAN_CMD_ALARM			= 0x04,
	QW_CAN_CMD_ALARM_ACK		= 0x84,
	QW_CAN_CMD_CONFIG			= 0x06,
	QW_CAN_CMD_CONFIG_ACK		= 0x85,

	QW_CAN_CMD_SLOT_LIGHT			= QW_UDP_SET_SLOT_LIGHT,
	QW_CAN_CMD_SLOT_LIGHT_ACK		= QW_UDP_SET_SLOT_LIGHT_ACK,

	QW_CAN_CMD_ALARM_LIGHT			= QW_UDP_SET_ALARM_LIGHT,
	QW_CAN_CMD_ALARM_LIGHT_ACK		= QW_UDP_SET_ALARM_LIGHT_ACK,

	QW_CAN_CMD_PING					= 0x08,
	QW_CAN_CMD_PING_ACK				= 0x88,
};


enum {
	QW_ERROR_OK							= 0x00,

	QW_ERROR_SHELF_NOT_FOUND			= 0x81,
	QW_ERROR_SHELF_OFFLINE				= 0x82,
	QW_ERROR_SHELF_INVALID_PARAMETER	= 0x83,
	QW_ERROR_CAN_BUS_ERROR				= 0x84,
};




#define QW_SHELF_COUNT_MAX					80

#define QW_SHELF_PLANE_COUNT_MAX			2
#define QW_SHELF_PACKET_SIZE				256


// 在发现时间内，只处理信息包和广播包
#define QW_CAN_DISCOVER_COUNT_MAX			1
#define QW_SHELF_DISCOVER_INTERVAL			3000

// 在未与服务器连接的情况下，PING得更频繁
#define QW_SHELF_REPORT_INTERVAL			10000
// 已与服务器连接，正常PING
#define QW_SHELF_PING_INTERVAL				60000
#define QW_SHELF_PING_TIMEOUT				180000

#define QW_SHELF_PING_MAGIC_BYTE0			0x03
#define QW_SHELF_PING_MAGIC_BYTE1			0x20


#define QW_SHELF_CAN_PING_INTERVAL			60000

struct qw_shelf_t {
	uint16_t _id;
	char _model[8];
	uint8_t _attribs;
	uint8_t _plane_count;
	uint16_t _light_count[QW_SHELF_PLANE_COUNT_MAX];

	can_context_t *_can_ctx;

	uint8_t _rx_state;
	uint32_t _rx_index;
	uint32_t _rx_size;
	uint8_t _error_code;
	uint8_t _rx_packet[QW_SHELF_PACKET_SIZE];

	int64_t _ping_timestamp;
	uint8_t _online;
};

extern void qw_can_discover(void);
extern void qw_can_ping(int64_t now);

extern void qw_on_can_frame(can_context_t *ctx, can_frame_t *f, struct qw_udp_context_t *udp_ctx);


extern void qw_on_udp_packet(struct qw_udp_context_t *ctx, const uint8_t *addr, uint16_t port, const uint8_t *p, int n);
extern void qw_udp_ping(struct qw_udp_context_t *ctx, const uint8_t *addr, uint16_t port);

extern void qw_udp_log(struct qw_udp_context_t *ctx, const char *s, int n);
extern void qw_udp_on_query_shelves(struct qw_udp_context_t *ctx, const uint8_t *addr, uint16_t port, uint8_t msg_id);


// 后两位作为版本号
#define QW_CONFIG_MAGIC0		0x20
#define QW_CONFIG_MAGIC1		0x26
#define QW_CONFIG_MAGIC2		0x00
#define QW_CONFIG_MAGIC3		0x05

// 保证以4字节对齐，尺寸4字节倍数
struct qw_router_config_t {
	uint8_t _magic[4];
	char _model[8];
	char _sn[16];
	uint8_t _mac[6];
	uint8_t _ip[4];
	uint8_t _mask[4];
	uint8_t _gw[4];
	uint8_t _server_ip[4];
	uint16_t _server_port;

	uint8_t _use_dhcp;
	uint8_t _use_wifi;
	char _wifi_ssid[32];
	char _wifi_password[32];
};

extern struct qw_router_config_t g_config __attribute__((aligned(4)));

//#define USE_RTC_CONFIG
#define USE_EFM_CONFIG

extern void qw_init_config(void);
extern bool qw_save_config(void);
extern bool qw_load_config(void);
extern void qw_dump_config(void);

struct qw_router_info_t {
	int _ping_interval;
	uint8_t _online;
	uint8_t _reset;
	ip_addr_t _server_addr;
	int64_t _reset_timestamp;
};

extern struct qw_router_info_t g_info;





extern int esp_at_udp_open(struct qw_udp_context_t *ctx);
extern int esp_at_udp_close(struct qw_udp_context_t *ctx);
extern int esp_at_udp_sendto(struct qw_udp_context_t *ctx, const uint8_t *addr, uint16_t port, const uint8_t *p, int n);

extern int lwip_udp_open(struct qw_udp_context_t *ctx);
extern int lwip_udp_close(struct qw_udp_context_t *ctx);
extern int lwip_udp_sendto(struct qw_udp_context_t *ctx, const uint8_t *addr, uint16_t port, const uint8_t *p, int n);




#endif	// SHELF_H
