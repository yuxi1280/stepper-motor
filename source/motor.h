#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

#define MOTOR_ADDR_DEFAULT      (1U)    /* 出厂默认地址 */
#define MOTOR_ADDR_BROADCAST    (0U)    /* 广播地址 */
#define MOTOR_CHECK_BYTE        (0x6BU) /* 出厂默认校验 */

#define MOTOR_PULSES_PER_REV    (3200U) /* Emm: 3200脉冲/圈 */
#define MOTOR_MAX_RPM           (3000U)

/* FD 位置模式 运动模式 */
#define MOTOR_MODE_REL_LAST     (0U)    /* 相对上一输入目标位置 */
#define MOTOR_MODE_ABS_ZERO     (1U)    /* 相对坐标零点(绝对) */
#define MOTOR_MODE_REL_CUR      (2U)    /* 相对当前实时位置 */

/* 9A 回零模式(常用) */
#define MOTOR_HOME_NEAREST      (0U)    /* 单圈就近回零 */
#define MOTOR_HOME_DIR          (1U)    /* 单圈方向回零 */
#define MOTOR_HOME_COLLIDE      (2U)    /* 无限位碰撞回零 */

/* ---- 底层(可用来发任意命令/广播) ---- */
void motor_tx(uint8_t addr, uint8_t code, const uint8_t *data, uint8_t len);

/* ---- 地址/校验配置 ---- */
void    motor_set_addr(uint8_t addr);
uint8_t motor_get_addr(void);
void    motor_set_check(uint8_t chk);

/* ---- 通用控制(发往当前地址) ---- */
void motor_enable(uint8_t en);            /* F3: 1=使能锁轴 0=失能松轴 */
void motor_stop(void);                    /* FE: 立即停止 */
void motor_home(uint8_t mode);            /* 9A: 触发回零 */
void motor_clear_alarm(void);             /* 0E: 解除堵转/过流/过热 */
void motor_zero_pos(void);                /* 0A: 当前位置清零 */
void motor_factory_reset(void);           /* 0F: 恢复出厂 */

/* ---- Emm 位置模式 FD(带符号脉冲, 自动拆方向) ---- */
void motor_move_abs(int32_t pulse, uint16_t rpm, uint8_t acc);  /* 绝对位置 */
void motor_move_rel(int32_t pulse, uint16_t rpm, uint8_t acc);  /* 相对位置 */
void motor_move(int8_t dir, uint32_t mag_pulse, uint16_t rpm,
                uint8_t acc, uint8_t mode, uint8_t sync);       /* 原始6参数 */

/* ---- Emm 快速位置 F1/FC (需固件>=V2.0.0) ---- */
void motor_set_profile(uint16_t rpm, uint8_t acc, uint8_t mode, uint8_t sync); /* F1 */
void motor_go(int32_t pulse);             /* FC: 有符号脉冲运动 */

/* ---- Emm 速度模式 F6 ---- */
void motor_speed(int8_t dir, uint16_t rpm, uint8_t acc);  /* dir: 0=正 1=反 */

/* ---- 读取(仅发送查询, 返回帧用 uart_recv_byte(UART_IDX_0) 取) ---- */
void motor_req_position(void);            /* 36 */

/* ---- 角度/脉冲换算(1.8°电机/16细分) ---- */
int32_t motor_deg_to_pulse(int32_t deg_x100);  /* deg_x100: 角度*100, 返回脉冲 */

/* ================= IO: 限位开关(PB4) =================
 * PB4 默认是 NJTRST, main.c 已调用 GPIO_SetDebugPort(GPIO_PIN_TRST, DISABLE)
 * 输入上拉, 低电平 = 触发
 */
#define MOTOR_LIMIT_SPEED       (200U)   /* 回零逼近速度 RPM(慢速, 防撞坏) */
#define MOTOR_LIMIT_ACC         (200U)   /* 加速度档 */
#define MOTOR_LIMIT_TIMEOUT_MS  (10000U) /* 超时10s, 防限位失效一直转 */

void    motor_io_init(void);              /* PB4 限位输入初始化(上拉) */
uint8_t motor_limit_home(void);           /* 返回1=回零成功 0=超时失败 */
void    motor_limit_task(void);           /* 限位沿检测: 主循环每轮调用 */

/* ================= IO: 轨道凸起计数(PB12, EXTINT12 -> IRQ128) =================
 * 用于位置校准: 轨道上固定位置放凸起, 传感器每经过一个凸起产生一个脉冲
 * 方向由最近一次 motor_move*/motor_speed 的 dir 参数决定(开环加减)
 */
#define MOTOR_BUMP_SPACING_MM   (20U)    /* TODO: 实测凸起间距(mm), 必须按实际填写 */
#define MOTOR_BUMP_ACTIVE_EDGE  (EXTINT_TRIG_FALLING) /* TODO: 按传感器极性改 RISING */

void    motor_bump_init(void);            /* PB12 + EXTINT12 + IRQ128 初始化 */
void    motor_bump_reset(void);           /* 计数清零(校准起点) */
int32_t motor_bump_pos(void);             /* 有符号凸起计数(方向感知) */
int32_t motor_bump_to_pulse(uint32_t bumps); /* 凸起数 -> 电机脉冲 */
void    motor_bump_set_dir(int8_t dir);   /* 记录运动方向(供凸起计数开环加减) */


/* ===== 机械参数: 16齿 S3M 同步轮 =====
 * S3M 节距 3.000mm -> 每圈皮带行程 = 16 * 3 = 48mm
 * 电机 3200 脉冲/圈 -> 10 圈 = 32000 脉冲 = 480mm = 48cm
 */
#define PULLEY_MM_PER_REV       (48U)     /* 皮带每圈行程 mm */

#define POS_RPM                 (600U)    /* 运行速度 */
#define POS_ACC                 (100U)    /* 加速度档 */

int32_t motor_mm_to_pulse(uint32_t mm);   /* 皮带行程 mm -> 电机脉冲 */

#endif /* __MOTOR_H__ */
