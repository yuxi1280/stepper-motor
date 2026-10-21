#ifndef __SERVO_H__
#define __SERVO_H__

#include "main.h"

/* ============================================================================
 * 众灵科技 ZP 系列总线舵机(单总线 UART) 收发驱动 —— 单舵机精简版
 * ----------------------------------------------------------------------------
 * 硬件:  USART3
 *          PB14 = USART3_TX  -> 接舵机信号线
 *          PB13 = USART3_RX  -> 回读(短接 TX)
 * 协议:  '#' + ID(3位补零) + 'P' + 命令 + '!'
 *         位置控制: #000P1500T1000!   PWM:500~2500  TIME:0~9999(ms)
 *         回读:     #OK!           通用成功
 *                   #000P1500!      位置回读
 * 参数:  115200 / 8 数据位 / 1 停止位 / 无校验
 * 说明:  仅单个舵机, ID 固定, 波特率固定 115200
 * ========================================================================== */

#define SERVO_BAUDRATE          (115200U)    /* 舵机总线波特率(固定) */
#define SERVO_ID                (0U)         /* 舵机 ID(固定, 出厂默认 0) */

#define SERVO_PWM_MIN           (500U)       /* PWM 下限 */
#define SERVO_PWM_MAX           (2500U)      /* PWM 上限 */
#define SERVO_PWM_MID           (1500U)      /* PWM 中值 */
#define SERVO_TIME_MAX          (9999U)      /* 运动时间上限 ms */

/* 缓冲/帧长(缓冲大小必须是 2 的幂) */
#define SERVO_TX_BUF_SIZE       (64U)
#define SERVO_RX_BUF_SIZE       (128U)
#define SERVO_LINE_MAX          (32U)

/* 回读结果类型 */
enum {
    SERVO_REPLY_NONE = 0,
    SERVO_REPLY_OK,          /* #OK!         通用成功 */
    SERVO_REPLY_POS          /* #000P1500!   位置回读 */
};

/* 最近一次回读解析结果 */
typedef struct {
    uint8_t  last_reply;     /* 见 SERVO_REPLY_* */
    uint16_t position;       /* 位置 500~2500 */
} servo_status_t;

/* ---------------- 初始化 ---------------- */
void servo_init(void);                             /* 初始化 USART3 + 中断 + 缓冲 */

/* ---------------- 位置控制 ---------------- */
void servo_move(uint16_t pwm, uint16_t time_ms);   /* #000PYYYYTZZZZ! */

/* ---------------- 控制指令(均回 #OK!) ---------------- */
void servo_release(void);        /* PULK 释放扭力 */
void servo_restore(void);        /* PULR 恢复扭力 */
void servo_pause(void);          /* PDPT 暂停 */
void servo_resume(void);         /* PDCT 继续 */
void servo_stop(void);           /* PDST 停止在当前位置 */
void servo_set_mid_here(void);   /* PSCK 当前位置标定为 1500 中值 */

/* ---------------- 读取位置(回 #000P1500!) ---------------- */
void servo_read_pos(void);       /* PRAD */

/* ---------------- 主循环任务 & 辅助 ---------------- */
void    servo_rx_task(void);                                /* 消费接收缓冲, 解析应答帧 */
void    servo_run(void);                                    /* 演示: 扫动一次(阻塞, 可选) */
void    servo_flush_rx(void);                               /* 清空接收缓冲 */
uint8_t servo_wait_reply(uint8_t reply, uint32_t timeout_ms);/* 等某类应答, 1=收到 */

/* ---------------- 回读状态 ---------------- */
extern volatile servo_status_t g_servo_st;      /* 最近一次解析结果 */
extern volatile uint32_t       g_servo_ok_cnt;  /* 收到 #OK! 次数 */

/* ---------------- 回中(过流保护用) ---------------- */
#define SERVO_CENTER_TIME_MS    (200U)   /* 回中运动时间 ms */
#define SERVO_CMD_GAP_MS        (50U)    /* 指令间隔, 等舵机收帧/应答 */

void servo_center(void);   /* 暂停 -> 释放扭力 -> 回1500中位 -> 恢复扭力 */



#endif /* __SERVO_H__ */
