#include "main.h"
#include "uart.h"
#include "motor.h"
#include "servo.h"
//#include "can.h"
//#include "check.h"


/* 电机所在串口(UART_IDX_0 = USART1), 换串口只改这里 */
#define MOTOR_UART_IDX          (UART_IDX_0)

static uint8_t s_motor_addr = MOTOR_ADDR_DEFAULT;
static uint8_t s_motor_chk  = MOTOR_CHECK_BYTE;

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

uint8_t motor_limit_home(void)
{
    uint32_t tick = 0U;

    motor_enable(1);
    DDL_DelayMS(100);

    /* 开机已停在限位上: 直接归零, 不必再转 */
    if (motor_limit_active()) {
        motor_stop();
        DDL_DelayMS(20);
        motor_clear_alarm();
        DDL_DelayMS(20);
        motor_zero_pos();             /* 当前位置赋为0点 */
        return 1U;
    }

    /* 速度模式向左(负方向)匀速逼近限位 */
    motor_speed(1, MOTOR_LIMIT_SPEED, MOTOR_LIMIT_ACC);

    while (motor_limit_active() == 0U) {
        DDL_DelayMS(1);
        if (++tick >= MOTOR_LIMIT_TIMEOUT_MS) {
            motor_stop();
            return 0U;
        }
    }

    /* 撞到限位: 停止 -> 解除堵转保护 -> 当前位置设为0点(归零校准) */
    motor_stop();
    DDL_DelayMS(20);
    motor_clear_alarm();
    DDL_DelayMS(20);
    motor_zero_pos();
    return 1U;
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
        motor_clear_alarm();
        DDL_DelayMS(20);
        motor_zero_pos();
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

/* 定长收帧: 期望 buf[0]=地址, buf[1]=功能码, buf[len-1]=校验 */
static uint8_t motor_read_frame(uint8_t code, uint8_t *buf, uint8_t len, uint32_t timeout_ms)
{
    uint32_t idle = 0U;
    uint8_t  n = 0U;
    int16_t  c;

    while (n < len) {
        c = uart_recv_byte(MOTOR_UART_IDX);
        if (c >= 0) {
            buf[n++] = (uint8_t)c;
            idle = 0U;
            continue;
        }
        if (++idle > timeout_ms) {          /* 字节间断太久: 认为没有应答 */
            return 0U;
        }
        DDL_DelayMS(1);
    }
    if ((buf[0] != s_motor_addr) || (buf[1] != code) || (buf[len - 1U] != s_motor_chk)) {
        return 0U;                          /* 地址/功能码/校验任一不符: 丢弃 */
    }
    return 1U;
}

/* 发查询 + 收应答; 成功返回帧长(数据在 buf[2..len-2]), 失败返回 0 */
static uint8_t motor_query(uint8_t code, uint8_t *buf, uint32_t timeout_ms)
{
    uint8_t len = motor_reply_len(code);

    if (len > MOTOR_RX_FRAME_MAX) {
        len = MOTOR_RX_FRAME_MAX;
    }
    s_motor_reading = 1U;
    motor_flush_rx();                       /* 清残留, 保证从帧头开始收 */
    motor_tx(s_motor_addr, code, NULL, 0U);
    if (motor_read_frame(code, buf, len, timeout_ms) == 0U) {
        s_motor_reading = 0U;
        return 0U;
    }
    s_motor_reading = 0U;
    return len;
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

/* 主循环调用: 消费电机主动返回(02 确认 / 9F 到位完成 / E2/EE 错误) */
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

    /* 返回码白名单: 防止 8 字节读取应答的残留被误判成主动帧 */
    if ((b[2] != 0x02U) && (b[2] != 0x12U) && (b[2] != 0x9FU) &&
        (b[2] != 0xE2U) && (b[2] != 0xEEU)) {
        return;
    }
    g_motor_last_reply = b[2];
    if (b[2] == 0x9FU) {                         /* 到位 / 回零完成 */
        g_motor_done_cnt++;
    } else if ((b[2] == 0xE2U) || (b[2] == 0xEEU)) {
        g_motor_err_cnt++;                       /* 参数错 / 格式错 / 保护触发 */
    }
}
