#ifndef __SERVO_H__
#define __SERVO_H__

#include "main.h"

/* ============================================================================
 * 众灵科技 ZP 系列总线舵机(单总线 UART) 驱动
 * ----------------------------------------------------------------------------
 * 硬件:  USART3
 *          PB14 = USART3_TX  -> 接舵机信号线
 *          PB13 = USART3_RX  -> 预留回读(短接 TX), 当前不接收
 * 协议:  '#' + ID(3位补零) + 'P' + 命令 + '!'
 *         位置控制: #000P1500T1000!   PWM:500~2500  TIME:0~9999(ms)
 * 参数:  115200 / 8 数据位 / 1 停止位 / 无校验
 * 说明:  当前只发送指令, 不解析返回; 供电直接上电(无开关)
 * ========================================================================== */

#define SERVO_BAUDRATE          (115200U)       /* 舵机总线波特率 */

#define SERVO_ID_DEFAULT        (0U)            /* 出厂默认 ID */
#define SERVO_ID_BROADCAST      (255U)          /* 广播地址 */

#define SERVO_PWM_MIN           (500U)
#define SERVO_PWM_MID           (1500U)
#define SERVO_PWM_MAX           (2500U)
#define SERVO_TIME_MAX          (9999U)

/* 工作模式(PMODn) */
#define SERVO_MODE_270_CW       (1U)    /* 舵机 270° 顺时针 */
#define SERVO_MODE_270_CCW      (2U)    /* 舵机 270° 逆时针 */
#define SERVO_MODE_180_CW       (3U)    /* 舵机 180° 顺时针 */
#define SERVO_MODE_180_CCW      (4U)    /* 舵机 180° 逆时针 */
#define SERVO_MODE_360_TURN_CW  (5U)    /* 马达 360° 定圈 顺时针 */
#define SERVO_MODE_360_TURN_CCW (6U)    /* 马达 360° 定圈 逆时针 */
#define SERVO_MODE_360_TIME_CW  (7U)    /* 马达 360° 定时 顺时针 */
#define SERVO_MODE_360_TIME_CCW (8U)    /* 马达 360° 定时 逆时针 */

/* 多舵机同步控制参数 */
typedef struct {
    uint8_t  id;        /* 舵机 ID (0~254) */
    uint16_t pwm;       /* 位置 500~2500 */
    uint16_t time_ms;   /* 运动时间 0~9999 ms */
} servo_cmd_t;

/* ---------------- 底层 ---------------- */
void servo_init(void);                                  /* USART3 初始化(只发) */
void servo_send(const uint8_t *data, uint16_t len);     /* 阻塞发送原始数据 */
void servo_send_str(const char *str);                   /* 阻塞发送字符串 */

/* ---------------- 位置控制 ---------------- */
void servo_move(uint8_t id, uint16_t pwm, uint16_t time_ms);         /* 指定ID, PWM */
void servo_move_angle(uint8_t id, uint16_t angle, uint16_t time_ms); /* 角度0~270° */
void servo_move_broadcast(uint16_t pwm, uint16_t time_ms);           /* 广播(255) */
void servo_move_multi(const servo_cmd_t *cmds, uint8_t n);           /* 多舵机 {} */

/* ---------------- 配置/控制命令(只发) ---------------- */
void servo_unlock_torque(uint8_t id);             /* PULK 释放扭力 */
void servo_lock_torque(uint8_t id);               /* PULR 恢复扭力 */
void servo_set_mode(uint8_t id, uint8_t mode);    /* PMODn 1~8 */
void servo_set_id(uint8_t old_id, uint8_t new_id);/* PIDxxx 改 ID */
void servo_stop(uint8_t id);                      /* PDST 停止当前位置 */
void servo_pause(uint8_t id);                     /* PDPT 暂停 */
void servo_resume(uint8_t id);                    /* PDCT 继续 */
void servo_set_baud(uint8_t id, uint8_t code);    /* PBDn 1~8(5=115200) */
void servo_calib_mid(uint8_t id);                 /* PSCK 当前位置设为1500 */
void servo_factory_reset(uint8_t id);             /* PCLE 恢复出厂 */

/* ---------------- 上电流程 ---------------- */
void servo_boot(void);   /* 等待上电稳定 -> 回中位 -> 恢复扭力 */
void servo_calib_mid(uint8_t id);                 /* PSCK 当前位置设为1500 */
void servo_set_mid_here(uint8_t id);              /* 释放扭力 -> 当前位置标定为中位 -> 恢复扭力 */
void servo_run(void);   /*舵机动作流程*/




#endif /* __SERVO_H__ */
