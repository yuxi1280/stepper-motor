#include "motor.h"
#include "uart.h"

static uint8_t s_motor_addr = MOTOR_ADDR_DEFAULT;
static uint8_t s_motor_chk  = MOTOR_CHECK_BYTE;

/* ================= 底层组帧发送(USART1 = 电机) ================= */
void motor_tx(uint8_t addr, uint8_t code, const uint8_t *data, uint8_t len)
{
    uint8_t frame[24];
    uint8_t i = 0U;

    frame[i++] = addr;
    frame[i++] = code;
    while (len--) {
        frame[i++] = *data++;
    }
    frame[i++] = s_motor_chk;

    uart_send_data(UART_IDX_0, frame, i);
}

/* ================= 地址/校验配置 ================= */
void    motor_set_addr(uint8_t addr) { s_motor_addr = addr; }
uint8_t motor_get_addr(void)         { return s_motor_addr; }
void    motor_set_check(uint8_t chk) { s_motor_chk = chk; }

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

    motor_bump_set_dir(dir);        /* 记录方向, 供凸起计数开环加减 */
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
    uint8_t  d[4];

    d[0] = (uint8_t)(v >> 24);
    d[1] = (uint8_t)(v >> 16);
    d[2] = (uint8_t)(v >> 8);
    d[3] = (uint8_t)(v);

    if (pulse < 0) {
        motor_bump_set_dir(1);
    } else if (pulse > 0) {
        motor_bump_set_dir(0);
    }
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

    motor_bump_set_dir(dir);
    motor_tx(s_motor_addr, 0xF6U, d, 5U);
}

/* ================= 读取位置(仅发查询) ================= */
void motor_req_position(void)
{
    motor_tx(s_motor_addr, 0x36U, NULL, 0U);
}

/* ================= 换算 =================
 * deg_x100 为角度*100(避免浮点), 返回脉冲数(四舍五入)
 * 3200 脉冲 / 360.00° -> pulse = deg_x100 * 3200 / 36000
 */
int32_t motor_deg_to_pulse(int32_t deg_x100)
{
    return (int32_t)(((int64_t)deg_x100 * (int64_t)MOTOR_PULSES_PER_REV + 18000) / 36000);
}

int32_t motor_mm_to_pulse(uint32_t mm)
{
    return (int32_t)(((uint64_t)mm * MOTOR_PULSES_PER_REV) / PULLEY_MM_PER_REV);
}

/* ==================================================================
 * 限位开关: PB4 (低电平 = 触发)
 * ================================================================== */
#define MOTOR_LIMIT_GPIO_PORT   (GPIO_PORT_B)
#define MOTOR_LIMIT_GPIO_PIN    (GPIO_PIN_04)

void motor_io_init(void)
{
    stc_gpio_init_t stcGpioInit;

    LL_PERIPH_WE(LL_PERIPH_GPIO);          /* GPIO_Init 内部要求寄存器解锁 */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDir   = PIN_DIR_IN;
    stcGpioInit.u16PinState = PIN_STAT_SET;   /* 配合 PUU 形成上拉 */
    stcGpioInit.u16PullUp   = PIN_PU_ON;
    stcGpioInit.u16PinAttr  = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(MOTOR_LIMIT_GPIO_PORT, MOTOR_LIMIT_GPIO_PIN, &stcGpioInit);
    LL_PERIPH_WP(LL_PERIPH_GPIO);
}

static uint8_t motor_limit_active(void)
{
    return (GPIO_ReadInputPins(MOTOR_LIMIT_GPIO_PORT, MOTOR_LIMIT_GPIO_PIN) == PIN_RESET);
}

/* ================= 限位开关回零 =================
 * 流程: 向左(负方向)匀速逼近限位 -> 触发后停止 -> 当前点清零为0
 */
uint8_t motor_limit_home(void)
{
    uint32_t tick = 0U;

    motor_enable(1U);
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

static uint8_t s_limit_last = 1U;    /* 上次电平: 1=未触发 0=触发 */

/* 限位任务: 主循环每轮调用, 触发边沿只处理一次 */
void motor_limit_task(void)
{
    uint8_t tmp1, tmp2, cur;

    tmp1 = motor_limit_active() ? 0U : 1U;
    if (tmp1 == s_limit_last) {
        return;                        /* 状态没变: 什么都不做 */
    }

    DDL_DelayMS(10);                   /* 消抖 */
    tmp2 = motor_limit_active() ? 0U : 1U;
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

/* ==================================================================
 * 轨道凸起计数: PB12 (EXTINT12 -> 共享中断 IRQ128)
 * ================================================================== */
#define MOTOR_BUMP_GPIO_PORT    (GPIO_PORT_B)
#define MOTOR_BUMP_GPIO_PIN     (GPIO_PIN_12)

static volatile int32_t s_bump_pos = 0;     /* 有符号凸起计数(方向感知) */
static volatile int8_t  s_bump_dir = 0;     /* 0 = 正方向, 1 = 负方向  */

/* 由 motor_move/motor_go/motor_speed 调用, 记录本次运动方向 */
void motor_bump_set_dir(int8_t dir)
{
    s_bump_dir = (dir == 0) ? 0 : 1;
}

/* IRQ128 分派进来(DDL 弱符号覆盖) */
void EXTINT12_IrqHandler(void)
{
    EXTINT_ClearExtIntStatus(EXTINT_CH12);
    s_bump_pos += (s_bump_dir == 0) ? 1 : -1;
}

void motor_bump_init(void)
{
    stc_gpio_init_t   stcGpioInit;
    stc_extint_init_t stcExtIntInit;

    LL_PERIPH_WE(LL_PERIPH_GPIO);
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDir   = PIN_DIR_IN;
    stcGpioInit.u16PinState = PIN_STAT_SET;   /* 配合 PUU 形成上拉 */
    stcGpioInit.u16PullUp   = PIN_PU_ON;
    stcGpioInit.u16PinAttr  = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(MOTOR_BUMP_GPIO_PORT, MOTOR_BUMP_GPIO_PIN, &stcGpioInit);

    /* 外部中断: 数字滤波 A 抗抖, 采样时钟 8 分频 */
    (void)EXTINT_StructInit(&stcExtIntInit);
    stcExtIntInit.u32Edge        = MOTOR_BUMP_ACTIVE_EDGE;
    stcExtIntInit.u32Filter      = EXTINT_FILTER_ON;
    stcExtIntInit.u32FilterClock = EXTINT_FCLK_DIV8;
    (void)EXTINT_Init(EXTINT_CH12, &stcExtIntInit);
    GPIO_ExtIntCmd(MOTOR_BUMP_GPIO_PORT, MOTOR_BUMP_GPIO_PIN, ENABLE);
    LL_PERIPH_WP(LL_PERIPH_GPIO);

    EXTINT_ClearExtIntStatus(EXTINT_CH12);
    (void)INTC_ShareIrqCmd(INT_SRC_PORT_EIRQ12, ENABLE);
    NVIC_ClearPendingIRQ(INT128_IRQn);
    NVIC_SetPriority(INT128_IRQn, DDL_IRQ_PRIO_02);
    NVIC_EnableIRQ(INT128_IRQn);
}

void motor_bump_reset(void)
{
    s_bump_pos = 0;
}

int32_t motor_bump_pos(void)
{
    return s_bump_pos;
}

/* 凸起数 -> 电机脉冲: bumps * BUMP_SPACING_MM(皮带mm) -> 脉冲 */
int32_t motor_bump_to_pulse(uint32_t bumps)
{
    return (int32_t)(((uint64_t)bumps * (uint64_t)MOTOR_BUMP_SPACING_MM *
                      (uint64_t)MOTOR_PULSES_PER_REV) / (uint64_t)PULLEY_MM_PER_REV);
}
