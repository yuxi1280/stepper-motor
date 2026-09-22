#include "motor.h"
#include "uart.h"

static uint8_t s_motor_addr = MOTOR_ADDR_DEFAULT;
static uint8_t s_motor_chk  = MOTOR_CHECK_BYTE;

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

    uart_send_data(UART_IDX_0, frame, i);
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
void motor_req_position(void)
{
    motor_tx(s_motor_addr, 0x36U, NULL, 0U);
}

/* ================= 换算 =================
 * deg_x100 为角度*100(避免浮点), 返回脉冲数(四舍五入)
 */
int32_t motor_deg_to_pulse(int32_t deg_x100)
{
    /* 3200 脉冲 / 360.00° = 8.8889 脉冲每度
     * pulse = deg_x100 * 3200 / 36000 */
    return (int32_t)(((int64_t)deg_x100 * (int64_t)MOTOR_PULSES_PER_REV + 18000) / 36000);
}

/* ================= 限位开关回零 =================
 * 依赖: PA7 为限位输入, 低电平=触发
 * 流程: 向左(负方向)匀速逼近限位 -> 触发后停止 -> 当前点清零为0
 */
#define MOTOR_LIMIT_GPIO_PORT   (GPIO_PORT_A)
#define MOTOR_LIMIT_GPIO_PIN    (GPIO_PIN_07)

static uint8_t motor_limit_active(void)
{
    return (GPIO_ReadInputPins(MOTOR_LIMIT_GPIO_PORT, MOTOR_LIMIT_GPIO_PIN) == PIN_RESET);
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


static uint8_t s_limit_last = 1U;    /* 上次电平: 1=未触发(高) 0=触发(低) */


/* 限位任务 */
void motor_limit_task(void)
{
    uint8_t tmp1, tmp2, cur;

    tmp1 = (GPIO_ReadInputPins(MOTOR_LIMIT_GPIO_PORT, MOTOR_LIMIT_GPIO_PIN) == PIN_RESET)
           ? 0U : 1U;
    if (tmp1 == s_limit_last) {
        return;                        /* 状态没变: 什么都不做 */
    }

    DDL_DelayMS(10);                   /* 消抖 */
    tmp2 = (GPIO_ReadInputPins(MOTOR_LIMIT_GPIO_PORT, MOTOR_LIMIT_GPIO_PIN) == PIN_RESET)
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

