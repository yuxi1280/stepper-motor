#include "main.h"
#include "uart.h"
#include "motor.h"
#include "servo.h"
#include "lc.h"
//#include "can.h"
//#include "check.h"


/* 电机所在串口(UART_IDX_0 = USART1), 换串口只改这里 */
#define MOTOR_UART_IDX          (UART_IDX_0)

static uint8_t s_motor_addr = MOTOR_ADDR_DEFAULT;
static uint8_t s_motor_chk  = MOTOR_CHECK_BYTE;
static uint8_t s_rxFailCnt  = 0U;        /* 连续收不到应答的次数(用于串口自愈) */

/* ================= 初始化: 限位输入 GPIO (PB4) ================= */
void motor_init(void)
{
    stc_gpio_init_t stcGpioInit;

    /* PB4 默认是 TRST(JTAG), 用作普通 IO 需先关闭调试口 */
    GPIO_SetDebugPort(GPIO_PIN_TRST, DISABLE);

    /* 限位输入: 数字输入 + 内部上拉(低电平=触发) */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDir  = PIN_DIR_IN;
    stcGpioInit.u16PullUp = PIN_PU_ON;
    stcGpioInit.u16PinAttr = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(MOTOR_LIMIT_PORT, MOTOR_LIMIT_PIN, &stcGpioInit);
}

/* ================= 底层组帧发送 ================= */
void motor_tx(uint8_t addr, uint8_t code, const uint8_t *data, uint8_t len)
{
    uint8_t frame[20];
    uint8_t i = 0U;

    frame[i++] = addr;
    frame[i++] = code;
    while (len--) {
        frame[i++] = *data++;
    }
    frame[i++] = s_motor_chk;

    uart_send_data(MOTOR_UART_IDX, frame, i);
}

/* ================= 地址/校验配置 ================= */
void motor_set_addr(uint8_t addr) { s_motor_addr = addr; }
uint8_t motor_get_addr(void)      { return s_motor_addr; }
void motor_set_check(uint8_t chk) { s_motor_chk = chk; }

/* ================= 通用控制 ================= */
void motor_enable(uint8_t en)
{
    uint8_t d[3] = { 0xABU, (uint8_t)(en & 0x01U), 0x00U };
    motor_tx(s_motor_addr, 0xF3U, d, 3U);       /* F3 AB en sync */
}

void motor_stop(void)
{
    uint8_t d[2] = { 0x98U, 0x00U };
    motor_tx(s_motor_addr, 0xFEU, d, 2U);       /* FE 98 sync */
}

void motor_home(uint8_t mode)
{
    uint8_t d[2] = { mode, 0x00U };
    motor_tx(s_motor_addr, 0x9AU, d, 2U);       /* 9A mode sync */
}

void motor_clear_alarm(void)
{
    uint8_t d[1] = { 0x52U };
    motor_tx(s_motor_addr, 0x0EU, d, 1U);
}

void motor_zero_pos(void)
{
    uint8_t d[1] = { 0x6DU };
    motor_tx(s_motor_addr, 0x0AU, d, 1U);
}

void motor_factory_reset(void)
{
    uint8_t d[1] = { 0x5FU };
    motor_tx(s_motor_addr, 0x0FU, d, 1U);
}

/* ================= Emm 位置模式 FD =================
 * 帧: FD dir spd2 acc pulse4 mode sync
 */
void motor_move(int8_t dir, uint32_t mag_pulse, uint16_t rpm,
                uint8_t acc, uint8_t mode, uint8_t sync)
{
    uint8_t d[10];

    d[0] = (uint8_t)(dir ? 0x01U : 0x00U);
    d[1] = (uint8_t)(rpm >> 8);
    d[2] = (uint8_t)(rpm);
    d[3] = acc;
    d[4] = (uint8_t)(mag_pulse >> 24);
    d[5] = (uint8_t)(mag_pulse >> 16);
    d[6] = (uint8_t)(mag_pulse >> 8);
    d[7] = (uint8_t)(mag_pulse);
    d[8] = mode;
    d[9] = sync;

    motor_tx(s_motor_addr, 0xFDU, d, 10U);
}

void motor_move_abs(int32_t pulse, uint16_t rpm, uint8_t acc)
{
    if (pulse < 0) {
        motor_move(1, (uint32_t)(-(int64_t)pulse), rpm, acc, MOTOR_MODE_ABS_ZERO, 0U);
    } else {
        motor_move(0, (uint32_t)pulse, rpm, acc, MOTOR_MODE_ABS_ZERO, 0U);
    }
}

void motor_move_rel(int32_t pulse, uint16_t rpm, uint8_t acc)
{
    if (pulse < 0) {
        motor_move(1, (uint32_t)(-(int64_t)pulse), rpm, acc, MOTOR_MODE_REL_LAST, 0U);
    } else {
        motor_move(0, (uint32_t)pulse, rpm, acc, MOTOR_MODE_REL_LAST, 0U);
    }
}

/* ================= Emm 快速位置 F1/FC ================= */
void motor_set_profile(uint16_t rpm, uint8_t acc, uint8_t mode, uint8_t sync)
{
    uint8_t d[5];
    d[0] = (uint8_t)(rpm >> 8);
    d[1] = (uint8_t)(rpm);
    d[2] = acc;
    d[3] = mode;
    d[4] = sync;
    motor_tx(s_motor_addr, 0xF1U, d, 5U);
}

void motor_go(int32_t pulse)
{
    uint32_t v = (uint32_t)pulse;
    uint8_t d[4];
    d[0] = (uint8_t)(v >> 24);
    d[1] = (uint8_t)(v >> 16);
    d[2] = (uint8_t)(v >> 8);
    d[3] = (uint8_t)(v);
    motor_tx(s_motor_addr, 0xFCU, d, 4U);
}

/* ================= Emm 速度模式 F6 ================= */
void motor_speed(int8_t dir, uint16_t rpm, uint8_t acc)
{
    uint8_t d[5];
    d[0] = (uint8_t)(dir ? 0x01U : 0x00U);
    d[1] = (uint8_t)(rpm >> 8);
    d[2] = (uint8_t)(rpm);
    d[3] = acc;
    d[4] = 0x00U;                       /* 同步: 立即执行 */
    motor_tx(s_motor_addr, 0xF6U, d, 5U);
}

/* ================= 读取位置(仅发查询) ================= */
// void motor_req_position(void)
// {
//     motor_tx(s_motor_addr, 0x36U, NULL, 0U);
// }
void motor_req_position(void)
{
    (void)motor_read_position_raw(MOTOR_RX_TIMEOUT_MS);
}

/* ================= 换算 =================
 * deg_x100 为角度*100(避免浮点), 返回脉冲数(四舍五入)
 * 3200 脉冲 / 360.00° = 8.8889 脉冲每度
 * pulse = deg_x100 * 3200 / 36000
 */
int32_t motor_deg_to_pulse(int32_t deg_x100)
{
    return (int32_t)(((int64_t)deg_x100 * (int64_t)MOTOR_PULSES_PER_REV + 18000) / 36000);
}

/* ================= 限位开关回零 (限位: PB4, 低电平=触发) =================
 * 流程: 向左(负方向)匀速逼近限位 -> 触发后停止 -> 当前点清零为0
 */
static uint8_t motor_limit_active(void)
{
    return (GPIO_ReadInputPins(MOTOR_LIMIT_PORT, MOTOR_LIMIT_PIN) == PIN_RESET);
}

/* 单次归零: 逼近限位 -> 清零, 返回 1=成功 0=超时 */
static uint8_t motor_limit_home_once(void)
{
    uint32_t tick = 0U;

    /* 开机先解除上一次留下的堵转/过流保护, 再使能;
     * 否则驱动锁在保护态, 速度模式也不会转, 表现为"只有信息、电机不动",
     * 然后 30s 回零超时。
     */
    motor_clear_alarm();
    DDL_DelayMS(20);
    motor_enable(1);
    DDL_DelayMS(100);

    /* 开机已停在限位上: 直接归零, 不必再转 */
    if (motor_limit_active()) {
        motor_stop();
        DDL_DelayMS(20);
        motor_clear_alarm();
        DDL_DelayMS(20);
        motor_enable(1);              /* 解除报警后必须重新使能 */
        DDL_DelayMS(50);
        motor_zero_pos();             /* 当前位置赋为0点 */
        return 1U;
    }

    /* 速度模式向左(负方向)匀速逼近限位 */
    motor_speed(1, MOTOR_LIMIT_SPEED, MOTOR_LIMIT_ACC);

    while (motor_limit_active() == 0U) {
        app_service();                 /* 回零最长 30 秒, 期间照样应答上位机 */
        DDL_DelayMS(1);
        if (++tick >= MOTOR_LIMIT_TIMEOUT_MS) {
            motor_stop();
            return 0U;
        }
    }

    /* 撞到限位: 停止 -> 解除堵转保护 -> 重新使能 -> 当前位置设为0点 */
    motor_stop();
    DDL_DelayMS(20);
    motor_clear_alarm();
    DDL_DelayMS(20);
    motor_enable(1);              /* 0x0E 之后驱动会失能, 必须重新使能,
                                   * 否则后面的 FD 运动指令只回 ACK 但电机不动 */
    DDL_DelayMS(50);
    motor_zero_pos();
    return 1U;
}

/* 归零后状态校验: 读 0x3B 回零状态 + 0x3A 电机状态
 * 返回 1=状态正常, 0=检测到掉电(OAC_TF 置位)
 */
static uint8_t motor_home_state_ok(void)
{
    uint8_t home_st;   /* 0x3B 回零状态 */
    uint8_t motor_st;  /* 0x3A 电机状态 */

    home_st  = motor_read_home_status(MOTOR_RX_TIMEOUT_MS);
    motor_st = motor_read_status(MOTOR_RX_TIMEOUT_MS);

    (void)home_st;     /* 回零状态本版仅读取, 可按需判 ORG_CF(回零失败)等 */

    /* 掉电标志 OAC_TF 置位 -> 判定掉电 */
    if ((motor_st != 0xFFU) && ((motor_st & MOTOR_ST_OAC_TF) != 0U)) {
        return 0U;
    }
    return 1U;
}

/* 归零 + 掉电校验: 每次归零后读 0x3B/0x3A, 掉电则重新上电并重归零一次 */
uint8_t motor_limit_home(void)
{
    uint8_t try;

    for (try = 0U; try <= MOTOR_HOME_PWR_RETRY; try++) {
        if (motor_limit_home_once() == 0U) {
            return 0U;                       /* 归零超时失败 */
        }
        if (motor_home_state_ok() != 0U) {
            return 1U;                       /* 状态正常(未掉电) */
        }
        /* 掉电: 重新上电(解除报警 + 重新使能), 然后重试归零 */
        motor_clear_alarm();
        DDL_DelayMS(20);
        motor_enable(1);
        DDL_DelayMS(100);
    }
    return 0U;                               /* 重试后仍掉电: 报失败 */
}


/* 限位任务: 主循环每轮调用, 触发边沿只停一次 */
static uint8_t s_limit_last = 1U;    /* 上次电平: 1=未触发(高) 0=触发(低) */

void motor_limit_task(void)
{
    uint8_t tmp1, tmp2, cur;

    tmp1 = (GPIO_ReadInputPins(MOTOR_LIMIT_PORT, MOTOR_LIMIT_PIN) == PIN_RESET)
           ? 0U : 1U;
    if (tmp1 == s_limit_last) {
        return;                        /* 状态没变: 什么都不做 */
    }

    DDL_DelayMS(10);                   /* 消抖 */
    tmp2 = (GPIO_ReadInputPins(MOTOR_LIMIT_PORT, MOTOR_LIMIT_PIN) == PIN_RESET)
           ? 0U : 1U;
    if (tmp1 != tmp2) {
        return;
    }
    cur = tmp2;

    if (cur == 0U) {
        motor_stop();
        DDL_DelayMS(20);
        motor_clear_alarm();              /* 解除堵转保护 */
        DDL_DelayMS(20);
        motor_enable(1);                  /* 0x0E 之后驱动可能失能, 必须重新使能 */
        DDL_DelayMS(20);
        /* 这里不再 motor_zero_pos(): 运行中清零会把 0x32 坐标系整体挪走,
         * 后续所有绝对目标都会算错(表现为"走到一半停住/超时")。 */
    }

    s_limit_last = cur;
}

int32_t motor_mm_to_pulse(uint32_t mm)
{
    return (int32_t)(((uint64_t)mm * MOTOR_PULSES_PER_REV) / PULLEY_MM_PER_REV);
}


/* 去第 n 个工位(n 从 1 开始): 位置 = POS_FIRST_MM + (n-1)*POS_STEP_MM (mm) */
void motor_goto_point(uint8_t n)
{
    uint32_t mm;
    int32_t  pulse;

    if (n == 0U) {
        return;                          /* 0 无效 */
    }
    mm    = (uint32_t)POS_FIRST_MM + ((uint32_t)(n - 1U) * (uint32_t)POS_STEP_MM);
    pulse = motor_mm_to_pulse(mm);
    motor_move_abs(pulse, POS_RPM, POS_ACC);   /* 绝对位置(相对回零的0点) */
}



/* ================= 接收/读取 ================= */

/* 按功能码给出返回帧总长度(手册 5.5 节), 不靠找 0x6B 断帧 */
static uint8_t motor_reply_len(uint8_t code)
{
    switch (code) {
        case 0x24U: case 0x26U: case 0x27U:
        case 0x31U: case 0x39U: case 0x3CU:  return 5U;   /* 单参数 + 校验 */
        case 0x3AU: case 0x3BU: case 0x3DU:  return 4U;   /* 状态标志 */
        case 0x35U:                          return 6U;   /* 符号 + 2 字节转速 */
        case 0x1FU: case 0x20U:              return 7U;   /* 版本 / 相电阻电感 */
        case 0x32U: case 0x33U: case 0x34U:
        case 0x36U: case 0x37U:              return 8U;   /* 符号 + 4 字节值 */
        default:                             return 4U;   /* 02/12/E2/EE/9F */
    }
}

static uint8_t s_motor_reading = 0U;    /* 1=正在等应答, 阻止 rx_task 抢字节 */

void motor_flush_rx(void)
{
    while (uart_recv_byte(MOTOR_UART_IDX) >= 0) {
    }
}

uint16_t motor_rx_available(void)
{
    return uart_rx_count(MOTOR_UART_IDX);
}

/* 定长收帧(滑动重同步 + 字节预算):
 * 期望 buf[0]=地址, buf[1]=功能码, buf[len-1]=校验。
 *
 * 原来的写法是"收满 len 字节, 校验不对就整帧丢弃", 残留字节会污染下一次读取;
 * 一旦错位, 后面几十次读位置全部失败(表现为 0x0A 且 PARAM=0x7FFF 连成一片)。
 * 现在改成滑动窗口: 不在帧头就丢 1 字节继续找, 校验不过也只退 1 字节。
 *
 * 同时必须保留"字节预算": 线路上一直有杂波时不能无限循环, 否则主循环会卡死。
 */
static uint8_t motor_read_frame(uint8_t code, uint8_t *buf, uint8_t len, uint32_t timeout_ms)
{
    uint32_t idle   = 0U;
    uint32_t budget = ((uint32_t)len * 4U) + 32U;   /* 硬预算: len=8 时最多处理 64 字节 */
    uint8_t  n = 0U;
    uint8_t  i;
    int16_t  c;

    while (budget != 0U) {
        c = uart_recv_byte(MOTOR_UART_IDX);
        if (c < 0) {
            if (++idle > timeout_ms) {      /* 字节间断太久: 认为没有应答 */
                return 0U;
            }
            DDL_DelayMS(1);
            continue;
        }
        idle = 0U;
        budget--;

        if ((n == 0U) && ((uint8_t)c != s_motor_addr)) {
            continue;                        /* 不在帧头: 丢掉这个字节 */
        }
        if (n >= len) {                      /* 缓冲满还没对上: 左移 1 字节 */
            for (i = 1U; i < len; i++) {
                buf[i - 1U] = buf[i];
            }
            n = (uint8_t)(len - 1U);
        }
        buf[n++] = (uint8_t)c;
        if (n < len) {
            continue;
        }
        if ((buf[0] == s_motor_addr) && (buf[1] == code) && (buf[len - 1U] == s_motor_chk)) {
            return 1U;                       /* 完整且校验通过 */
        }
        /* 校验不过: 只退 1 字节, 继续在同一窗口里找真正的帧头 */
        for (i = 1U; i < len; i++) {
            buf[i - 1U] = buf[i];
        }
        n = (uint8_t)(len - 1U);
    }
    return 0U;                               /* 预算用完还没对上: 放弃本次, 绝不在这里死等 */
}

/* 发查询 + 收应答; 成功返回帧长(数据在 buf[2..len-2]), 失败返回 0
 *
 * 自愈: 连续 MOTOR_RX_FAIL_REINIT 次收不到应答时, 判定 MCU 的 USART1 接收端锁死
 * (现象: 电机能收指令能动作, 但上位机一个字节都收不到, 只有断电重启才恢复),
 * 这里主动重新初始化一次串口, 然后立刻重试本次查询。
 */
static uint8_t motor_query(uint8_t code, uint8_t *buf, uint32_t timeout_ms)
{
    uint8_t len = motor_reply_len(code);
    uint8_t retry = 0U;

    if (len > MOTOR_RX_FRAME_MAX) {
        len = MOTOR_RX_FRAME_MAX;
    }

    for (retry = 0U; retry < 2U; retry++) {
        s_motor_reading = 1U;
        motor_flush_rx();                   /* 清残留, 保证从帧头开始收 */
        motor_tx(s_motor_addr, code, NULL, 0U);
        if (motor_read_frame(code, buf, len, timeout_ms) != 0U) {
            s_motor_reading = 0U;
            s_rxFailCnt    = 0U;
            return len;                     /* 收到, 正常返回 */
        }
        s_motor_reading = 0U;

        if (++s_rxFailCnt < MOTOR_RX_FAIL_REINIT) {
            return 0U;                      /* 偶发失败: 交给上层重试 */
        }

        /* 连续多次收不到: 重新初始化 USART1, 然后重试一次 */
        s_rxFailCnt = 0U;
        uart_driver_init();
        DDL_DelayMS(10U);
    }
    return 0U;
}

/* 符号 + 4 字节大端值 -> int32 (0x32/0x33/0x34/0x36/0x37 通用) */
static int32_t motor_parse_sign_u32(const uint8_t *b)
{
    uint32_t v = ((uint32_t)b[3] << 24) | ((uint32_t)b[4] << 16) |
                 ((uint32_t)b[5] << 8)  |  (uint32_t)b[6];

    return (b[2] != 0U) ? -(int32_t)v : (int32_t)v;
}

/* 0x32 输入脉冲数: 3200 脉冲/圈, 与 motor_mm_to_pulse() 同一坐标系 */
int32_t motor_read_input_pulses(uint32_t timeout_ms)
{
    uint8_t b[MOTOR_RX_FRAME_MAX];

    if (motor_query(0x32U, b, timeout_ms) == 0U) { return MOTOR_ERR_TIMEOUT; }
    return motor_parse_sign_u32(b);
}

/* 0x36 实时位置 */
int32_t motor_read_position_raw(uint32_t timeout_ms)
{
    uint8_t b[MOTOR_RX_FRAME_MAX];

    if (motor_query(0x36U, b, timeout_ms) == 0U) { return MOTOR_ERR_TIMEOUT; }
    return motor_parse_sign_u32(b);
}

/* 0x33 电机目标位置 */
int32_t motor_read_target_raw(uint32_t timeout_ms)
{
    uint8_t b[MOTOR_RX_FRAME_MAX];

    if (motor_query(0x33U, b, timeout_ms) == 0U) { return MOTOR_ERR_TIMEOUT; }
    return motor_parse_sign_u32(b);
}

/* 0x37 位置误差 */
int32_t motor_read_pos_error_raw(uint32_t timeout_ms)
{
    uint8_t b[MOTOR_RX_FRAME_MAX];

    if (motor_query(0x37U, b, timeout_ms) == 0U) { return MOTOR_ERR_TIMEOUT; }
    return motor_parse_sign_u32(b);
}

/* 0x35 实时转速: Addr 35 符号 转速(2B) 6B (Emm 固件单位 RPM) */
int16_t motor_read_speed_rpm(uint32_t timeout_ms)
{
    uint8_t  b[MOTOR_RX_FRAME_MAX];
    uint16_t v;

    if (motor_query(0x35U, b, timeout_ms) == 0U) { return INT16_MIN; }
    v = (uint16_t)(((uint16_t)b[3] << 8) | (uint16_t)b[4]);
    return (int16_t)((b[2] != 0U) ? -(int16_t)v : (int16_t)v);
}

/* 0x31 线性化编码器值: Addr 31 值(2B) 6B, 0-65535 = 0-360° */
uint16_t motor_read_enc_raw(uint32_t timeout_ms)
{
    uint8_t b[MOTOR_RX_FRAME_MAX];

    if (motor_query(0x31U, b, timeout_ms) == 0U) { return 0xFFFFU; }
    return (uint16_t)(((uint16_t)b[2] << 8) | (uint16_t)b[3]);
}

/* 0x3A 电机状态标志 */
uint8_t motor_read_status(uint32_t timeout_ms)
{
    uint8_t b[MOTOR_RX_FRAME_MAX];

    if (motor_query(0x3AU, b, timeout_ms) == 0U) { return 0xFFU; }
    return b[2];
}

/* 0x3B 回零状态标志 */
uint8_t motor_read_home_status(uint32_t timeout_ms)
{
    uint8_t b[MOTOR_RX_FRAME_MAX];

    if (motor_query(0x3BU, b, timeout_ms) == 0U) { return 0xFFU; }
    return b[2];
}

/* 0x39 驱动温度: Addr 39 符号 温度 6B */
uint8_t motor_read_temp_c(uint32_t timeout_ms)
{
    uint8_t b[MOTOR_RX_FRAME_MAX];

    if (motor_query(0x39U, b, timeout_ms) == 0U) { return 0xFFU; }
    return b[3];
}

/* 0x24 总线电压(mV) */
uint16_t motor_read_bus_volt_mv(uint32_t timeout_ms)
{
    uint8_t b[MOTOR_RX_FRAME_MAX];

    if (motor_query(0x24U, b, timeout_ms) == 0U) { return 0xFFFFU; }
    return (uint16_t)(((uint16_t)b[2] << 8) | (uint16_t)b[3]);
}

/* 0x26 总线电流(mA) */
uint16_t motor_read_bus_current_ma(uint32_t timeout_ms)
{
    uint8_t b[MOTOR_RX_FRAME_MAX];

    if (motor_query(0x26U, b, timeout_ms) == 0U) { return 0xFFFFU; }
    return (uint16_t)(((uint16_t)b[2] << 8) | (uint16_t)b[3]);
}

/* 0x27 相电流(mA) */
uint16_t motor_read_phase_current_ma(uint32_t timeout_ms)
{
    uint8_t b[MOTOR_RX_FRAME_MAX];

    if (motor_query(0x27U, b, timeout_ms) == 0U) { return 0xFFFFU; }
    return (uint16_t)(((uint16_t)b[2] << 8) | (uint16_t)b[3]);
}

/* 0x1F 固件/硬件版本: Addr 1F FW_Ver(2B) HW(2B) 6B */
uint8_t motor_read_version(motor_version_t *ver, uint32_t timeout_ms)
{
    uint8_t b[MOTOR_RX_FRAME_MAX];

    if ((ver == NULL) || (motor_query(0x1FU, b, timeout_ms) == 0U)) { return 0U; }
    ver->fw_ver    = (uint16_t)(((uint16_t)b[2] << 8) | (uint16_t)b[3]);
    ver->hw_series = (uint8_t)(b[4] >> 4);
    ver->hw_type   = (uint8_t)(b[4] & 0x0FU);
    ver->hw_ver    = b[5];
    return 1U;
}

/* 0x36 换算成 0.001°(按 MOTOR_POS_MODE 选择语义) */
int32_t motor_read_position_mdeg(uint32_t timeout_ms)
{
    int32_t raw = motor_read_position_raw(timeout_ms);

    if (raw == MOTOR_ERR_TIMEOUT) { return raw; }
#if (MOTOR_POS_MODE == MOTOR_POS_MODE_ENCODER)
    return (int32_t)(((int64_t)raw * 360000LL) / 65536LL);
#else
    return (int32_t)(((int64_t)raw * 360000LL) / (int64_t)MOTOR_PULSES_PER_REV);
#endif
}

/* 脉冲 -> 皮带行程(0.01mm): mm = pulse * 48 / 3200 */
int32_t motor_pulse_to_mm_x100(int32_t pulse)
{
    return (int32_t)(((int64_t)pulse * (int64_t)PULLEY_MM_PER_REV * 100LL) /
                     (int64_t)MOTOR_PULSES_PER_REV);
}

/* 轮询 Prf_TF 等待到位(可替代 motor_goto_point 里的固定延时) */
uint8_t motor_wait_arrived(uint32_t timeout_ms)
{
    uint32_t t = 0U;
    uint8_t  st;

    for (;;) {
        st = motor_read_status(MOTOR_RX_TIMEOUT_MS);
        if ((st != 0xFFU) && ((st & MOTOR_ST_PRF_TF) != 0U)) {
            return 1U;
        }
        if (t >= timeout_ms) {
            return 0U;
        }
        t += 5U;
        DDL_DelayMS(5);
    }
}


volatile uint16_t g_motor_done_cnt   = 0U;
volatile uint32_t g_motor_err_cnt    = 0U;
volatile uint8_t  g_motor_last_reply = 0U;

void motor_rx_task(void)
{
    uint8_t b[4];

    if ((s_motor_reading != 0U) || (motor_rx_available() < 4U)) {
        return;
    }
    b[0] = (uint8_t)uart_recv_byte(MOTOR_UART_IDX);
    b[1] = (uint8_t)uart_recv_byte(MOTOR_UART_IDX);
    b[2] = (uint8_t)uart_recv_byte(MOTOR_UART_IDX);
    b[3] = (uint8_t)uart_recv_byte(MOTOR_UART_IDX);
    if ((b[0] != s_motor_addr) || (b[3] != s_motor_chk)) {
        return;                                  /* 帧对不上, 丢弃 */
    }

    /* 返回码白名单: 防止 8 字节读取应答的残留被误判成主动帧。
     * 注意返回码在 b[1](帧格式: addr code status chk), 原来查 b[2] 是错的,
     * 导致 9F 到位帧永远认不出来。
     */
    if ((b[1] != 0x02U) && (b[1] != 0x12U) && (b[1] != 0x9FU) &&
        (b[1] != 0xE2U) && (b[1] != 0xEEU)) {
        return;
    }
    g_motor_last_reply = b[1];
    if (b[1] == 0x9FU) {                         /* 到位 / 回零完成 */
        g_motor_done_cnt++;
    } else if ((b[1] == 0xE2U) || (b[1] == 0xEEU)) {
        g_motor_err_cnt++;                       /* 参数错 / 格式错 / 保护触发 */
    }
}
