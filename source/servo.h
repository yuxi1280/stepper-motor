#ifndef __SERVO_H__
#define __SERVO_H__

#include "main.h"

/* ================= 众灵科技 ZP 系列总线舵机(单总线 UART) =================
 * 帧格式: '#' + ID(3位补零) + 'P' + 命令 + '!'
 * 位置控制: #000P1500T1000!   1500=PWM(500~2500)  1000=时间ms(0~9999)
 * 多舵机:   {#000P1602T1000!#001P2500T0000!}   (2条以上需加 {})
 * 波特率:   默认 115200
 * 本驱动运行在 USART3: PB13 = RX(FUNC_33) / PB14 = TX(FUNC_32)
 * 发送: 阻塞; 接收: IRQ137 中断 -> 环形缓冲
 * 注意: 单总线会有自发自收回显, 需要时用 servo_rx_flush() 丢弃
 */

#define SERVO_BUS_BAUDRATE      (115200U)   /* ZP 舵机默认波特率 */

#define SERVO_ID_BROADCAST      (255U)      /* 广播地址, 对所有舵机生效 */
#define SERVO_PWM_MIN           (500U)
#define SERVO_PWM_MAX           (2500U)
#define SERVO_PWM_MID           (1500U)

/* 多舵机同步控制参数 */
typedef struct {
    uint8_t  id;        /* 舵机ID 0~254 */
    uint16_t pwm;       /* 位置 500~2500 */
    uint16_t time_ms;   /* 运动时间 0~9999 */
} servo_cmd_t;

/* ---- 底层 ---- */
void servo_init(void);                                  /* USART3 初始化(TX阻塞 + RX中断) */
void servo_send_raw(const uint8_t *data, uint16_t len); /* 阻塞发送任意数据 */

/* ---- 接收 ---- */
int16_t  servo_recv_byte(void);                         /* 取一字节, 空返回 -1 */
uint16_t servo_recv(uint8_t *dst, uint16_t max, uint32_t timeout_ms);
void     servo_rx_flush(void);                          /* 丢弃接收缓冲(清回显) */

/* ---- 位置控制 ---- */
void servo_move(uint8_t id, uint16_t pwm, uint16_t time_ms);          /* 单舵机 */
void servo_move_angle(uint8_t id, uint16_t angle, uint16_t time_ms);  /* 单舵机(0~270°) */
void servo_move_broadcast(uint16_t pwm, uint16_t time_ms);            /* 广播 255 */
void servo_move_multi(const servo_cmd_t *cmds, uint8_t n);            /* 多舵机叠加 */

/* ---- 常用配置/控制命令(只发不收) ---- */
void servo_unlock_torque(uint8_t id);                  /* PULK 释放扭力 */
void servo_lock_torque(uint8_t id);                    /* PULR 恢复扭力 */
void servo_set_mode(uint8_t id, uint8_t mode);         /* PMODn 工作模式 1~8 */
void servo_set_id(uint8_t old_id, uint8_t new_id);     /* PIDxxx 修改ID */
void servo_stop(uint8_t id);                           /* PDST 停止 */
void servo_pause(uint8_t id);                          /* PDPT 暂停 */
void servo_resume(uint8_t id);                         /* PDCT 继续 */
void servo_set_baud(uint8_t id, uint8_t code);         /* PBDn 1~8(5=115200) */
void servo_factory_reset(uint8_t id);                  /* PCLE 恢复出厂 */
void servo_calib_mid(uint8_t id);                      /* PSCK 当前位置标定1500 */
void servo_start(void);
void servo_initial(void);   /* 释放扭力 -> 标定中位 -> 恢复扭力 */

#endif /* __SERVO_H__ */
