#ifndef __CAN_H__
#define __CAN_H__

#include "main.h"
/* ================= 设备配置 ================= */
#define SLAVE_ADDR             (8U)    /* 1~15 */

/* ================= 节点 / 报文 ID 配置 ================= */
#define CAN_BROADCAST_ID     (0x10000000U)  /* 广播扩展帧 ID */
#define CAN_NODE_ID          (0x0100U | SLAVE_ADDR)  /* 本机应答 ID(扩展帧): 地址1->0x0101 ... 地址15->0x010F */

/* 自检命令字 */
#define CAN_CMD_INTERIOR_INFO      (0x0AU)   /* 主机广播: 查询舵机控制板信息 */
#define CAN_CMD_INTERIOR_INFO_ACK  (0x8AU)   /* 从机应答: 返回自身信息 */

/* 过流上报 ID(与本机地址相关) */
#define CAN_OVER_CURRENT_ID    (0x0100U | SLAVE_ADDR)

/* 收发环形缓冲容量 */
#define CAN_TX_RING_SIZE     (64U)
#define CAN_RX_QUEUE_SIZE    (8U)

/* ================= 对外 API ================= */
void can_init(void);
void can_poll(void);                        /* 主循环: 处理接收队列 */
void can_tx_bytes(uint32_t ext_id, const uint8_t *p, uint8_t n);    /* 扩展帧发送(>8B 自动分包) */
void can_tx_bytes_std(uint32_t std_id, const uint8_t *p, uint8_t n);/* 标准帧发送 */
void can_report_overcurrent(void);          /* 过流上报(供 check.c 调用) */

#endif /* __CAN_H__ */
