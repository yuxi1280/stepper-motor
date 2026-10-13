#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

/* ============================================================================
 * Emm 闭环步进/伺服电机 总线驱动 (基于 source/uart.c)
 * ----------------------------------------------------------------------------
 * 总线:  USART1 (PA04=TX, PA03=RX), 由 uart_driver_init() 初始化
 * 帧:    addr + code + data... + check
 * 默认:  地址 1, 校验 0x6B, 3200 脉冲/圈
 * 限位:  PB4 (低电平=触发)
 * ========================================================================== */

 /* ===== 工位定义(归零后) =====
 * 第 1 个点距零点 10mm, 之后每点比前一个远 17mm:
 *   n=1 -> 10mm, n=2 -> 27mm, n=3 -> 44mm, ...
 */
#define POS_FIRST_MM         (10U)   /* 第一个工位距零点 mm */
#define POS_STEP_MM          (17U)   /* 工位间距 mm */


#define MOTOR_ADDR_DEFAULT      (1U)    /* 出厂默认地址 */
#define MOTOR_ADDR_BROADCAST    (0U)    /* 广播地址 */
#define MOTOR_CHECK_BYTE        (0x6B)  /* 出厂默认校验 */

#define MOTOR_PULSES_PER_REV    (3200U) /* Emm: 3200 脉冲/圈 */
#define MOTOR_MAX_RPM           (3000U)

/* FD 位置模式 运动模式 */
#define MOTOR_MODE_REL_LAST     (0U)    /* 相对上一输入目标位置 */
#define MOTOR_MODE_ABS_ZERO     (1U)    /* 相对坐标零点(绝对) */
#define MOTOR_MODE_REL_CUR      (2U)    /* 相对当前实时位置 */

/* 9A 回零模式(常用) */
#define MOTOR_HOME_NEAREST      (0U)    /* 单圈就近回零 */
#define MOTOR_HOME_DIR          (1U)    /* 单圈方向回零 */
#define MOTOR_HOME_COLLIDE      (2U)    /* 无限位碰撞回零 */

/* ---- 初始化(配置限位输入 GPIO, 用限位回零时调用) ---- */
void motor_init(void);

/* ---- 底层(可发任意命令/广播) ---- */
void motor_tx(uint8_t addr, uint8_t code, const uint8_t *data, uint8_t len);

/* ---- 地址/校验配置 ---- */
void motor_set_addr(uint8_t addr);
uint8_t motor_get_addr(void);
void motor_set_check(uint8_t chk);

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

/* ---- 读取(仅发送查询) ---- */
void motor_req_position(void);            /* 36 */

/* ---- 角度/脉冲换算(1.8°电机/16细分) ---- */
int32_t motor_deg_to_pulse(int32_t deg_x100);  /* deg_x100: 角度*100, 返回脉冲 */

/* ---- 限位开关回零 ----
 * 新板限位输入: PB4 (低电平=触发)
 * 注意: PB4 默认是 TRST(JTAG), 用作普通 IO 需先关闭调试口
 */
#define MOTOR_LIMIT_PORT        (GPIO_PORT_B)   /* 限位输入端口 */
#define MOTOR_LIMIT_PIN         (GPIO_PIN_04)   /* 限位输入引脚, 低电平=触发 */
#define MOTOR_LIMIT_SPEED       (200U)          /* 逼近速度 RPM(慢速防撞) */
#define MOTOR_LIMIT_ACC         (200U)          /* 加速度档 */
#define MOTOR_LIMIT_TIMEOUT_MS  (10000U)        /* 超时 10s 防限位失效 */

uint8_t motor_limit_home(void);          /* 返回1=回零成功 0=超时失败 */
void motor_limit_task(void);             /* 主循环每轮调用: 限位沿检测 */

/* ===== 机械参数(示例: 16齿 S3M 同步轮, 按实际修改) =====
 * S3M 节距 3.000mm -> 每圈皮带行程 = 16 * 3 = 48mm
 * 电机 3200 脉冲/圈 -> 10 圈 = 32000 脉冲 = 480mm = 48cm
 */
#define PULLEY_MM_PER_REV       (48U)     /* 皮带每圈行程 mm */
#define POS_48CM_MM             (480U)
#define POS_48CM_PULSE          ((int32_t)(((uint32_t)POS_48CM_MM * MOTOR_PULSES_PER_REV) \
                                           / PULLEY_MM_PER_REV))
#define POS_RPM                 (600U)
#define POS_ACC                 (200U)
#define POS_WAIT_MS             (1500U)

int32_t motor_mm_to_pulse(uint32_t mm);   /* 皮带行程 mm -> 电机脉冲 */

#define POS_863MM_MM            (863U)
#define POS_863MM_PULSE         ((int32_t)(((uint32_t)POS_863MM_MM * MOTOR_PULSES_PER_REV) \
                                           / PULLEY_MM_PER_REV))
#define HOME_SETTLE_MS          (500U)    /* 回零后机械稳定 */

void motor_goto_point(uint8_t n);   /* 去第 n 个工位(n 从 1 开始), 内部换算脉冲 */
/* ================= 回读(自由协议: 地址 + 功能码 + 数据 + 校验) ================= */
#define MOTOR_RX_TIMEOUT_MS   (50U)   /* 单次读取默认超时(ms) */
#define MOTOR_RX_FRAME_MAX    (8U)    /* 手册里最长返回帧 8 字节 */
#define MOTOR_ERR_TIMEOUT     (INT32_MIN)  /* 读取失败的返回值 */

/* 电机状态标志(0x3A) */
#define MOTOR_ST_OAC_TF       (0x80U) /* 掉电标志 */
#define MOTOR_ST_ESI_RF       (0x20U) /* 右限位 */
#define MOTOR_ST_ESI_LF       (0x10U) /* 左限位 */
#define MOTOR_ST_CGP_TF       (0x08U) /* 堵转保护 */
#define MOTOR_ST_CGI_TF       (0x04U) /* 堵转标志 */
#define MOTOR_ST_PRF_TF       (0x02U) /* 位置到达 */
#define MOTOR_ST_ENS_TF       (0x01U) /* 使能状态 */

/* 回零状态标志(0x3B/0x3C) */
#define MOTOR_ORG_ENC_RDY     (0x01U) /* 编码器正常 */
#define MOTOR_ORG_CAL_RDY     (0x02U) /* 已校准 */
#define MOTOR_ORG_SF          (0x04U) /* 正在回零 */
#define MOTOR_ORG_CF          (0x08U) /* 回零失败 */
#define MOTOR_ORG_OTP_TF      (0x10U) /* 过热保护 */
#define MOTOR_ORG_OCP_TF      (0x20U) /* 过流保护 */

/* 0x36 实时位置返回值语义(按实测二选一) */
#define MOTOR_POS_MODE_PULSE    (0U)  /* 返回脉冲数, 3200/圈, 可多圈 */
#define MOTOR_POS_MODE_ENCODER  (1U)  /* 返回单圈编码器值, 65536=360°(Emm固件手册写法) */
#define MOTOR_POS_MODE          (MOTOR_POS_MODE_ENCODER)

typedef struct {
    uint16_t fw_ver;      /* 例 200 -> V2.0.0 */
    uint8_t  hw_series;   /* 0=X 系列 1=Y 系列 */
    uint8_t  hw_type;     /* 3=35 / 4=42 / 5=57 */
    uint8_t  hw_ver;      /* 例 14 -> V2.0 */
} motor_version_t;

/* ---- 接收底层 ---- */
void     motor_flush_rx(void);                      /* 丢弃接收缓冲里的残留 */
uint16_t motor_rx_available(void);                  /* 缓冲里还有多少字节 */
void     motor_rx_task(void);                       /* 主循环调用: 消费主动返回帧 */

/* ---- 读取接口(失败返回负数/0xFF) ---- */
int32_t  motor_read_input_pulses(uint32_t timeout_ms);    /* 0x32 输入脉冲数(有符号) */
int32_t  motor_read_position_raw(uint32_t timeout_ms);    /* 0x36 实时位置原始值 */
int32_t  motor_read_position_mdeg(uint32_t timeout_ms);   /* 0x36 换算成 0.001° */
int32_t  motor_read_target_raw(uint32_t timeout_ms);      /* 0x33 目标位置 */
int32_t  motor_read_pos_error_raw(uint32_t timeout_ms);   /* 0x37 位置误差 */
int16_t  motor_read_speed_rpm(uint32_t timeout_ms);       /* 0x35 实时转速(失败 INT16_MIN) */
uint16_t motor_read_enc_raw(uint32_t timeout_ms);         /* 0x31 编码器值(失败 0xFFFF) */
uint8_t  motor_read_status(uint32_t timeout_ms);          /* 0x3A 状态标志(失败 0xFF) */
uint8_t  motor_read_home_status(uint32_t timeout_ms);     /* 0x3B 回零状态 */
uint8_t  motor_read_temp_c(uint32_t timeout_ms);          /* 0x39 驱动温度 */
uint16_t motor_read_bus_volt_mv(uint32_t timeout_ms);     /* 0x24 总线电压 mV */
uint16_t motor_read_bus_current_ma(uint32_t timeout_ms);  /* 0x26 总线电流 mA */
uint16_t motor_read_phase_current_ma(uint32_t timeout_ms);/* 0x27 相电流 mA */
uint8_t  motor_read_version(motor_version_t *ver, uint32_t timeout_ms); /* 0x1F */

/* ---- 到位等待 & 换算 ---- */
uint8_t  motor_wait_arrived(uint32_t timeout_ms);   /* 1=已到位(轮询 Prf_TF) */
int32_t  motor_pulse_to_mm_x100(int32_t pulse);     /* 脉冲 -> 皮带行程 x100 mm */

/* ---- 诊断计数 ---- */
extern volatile uint16_t g_motor_done_cnt;      /* 收到 9F 到位/回零完成次数 */
extern volatile uint32_t g_motor_err_cnt;       /* 收到 E2/EE 次数 */
extern volatile uint8_t  g_motor_last_reply;    /* 最近一次返回码 */


#endif /* __MOTOR_H__ */
