#include "servo.h"

/* ============================================================================
 * 硬件映射: USART3 --- PB14(TX) --> 舵机信号线, PB13(RX) 预留回读
 * ========================================================================== */

#define SERVO_USART             (CM_USART3)

/* ------------------------- 底层: 阻塞发送 ------------------------- */

static void servo_putc(uint8_t c)
{
    /* 等待发送数据寄存器空 */
    while (USART_GetStatus(SERVO_USART, USART_FLAG_TX_EMPTY) == RESET) {
    }
    USART_WriteData(SERVO_USART, c);
}

void servo_send(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    for (i = 0U; i < len; i++) {
        servo_putc(data[i]);
    }
    /* 等待最后一字节移位发送完成 */
    while (USART_GetStatus(SERVO_USART, USART_FLAG_TX_CPLT) == RESET) {
    }
}

void servo_send_str(const char *str)
{
    while (*str != '\0') {
        servo_putc((uint8_t)(*str));
        str++;
    }
    while (USART_GetStatus(SERVO_USART, USART_FLAG_TX_CPLT) == RESET) {
    }
}

/* ------------------------- 协议组帧 ------------------------- */

/* 把 val 按 digits 位十进制补零写入 dst, 返回写后的指针 */
static char *servo_fmt_digits(char *dst, uint32_t val, uint8_t digits)
{
    uint8_t i;
    for (i = 0U; i < digits; i++) {
        dst[digits - 1U - i] = (char)('0' + (val % 10U));
        val /= 10U;
    }
    return &dst[digits];
}

/* 发送 '#XXXP<cmd>!' 形式指令 */
static void servo_send_cmd(uint8_t id, const char *cmd)
{
    char buf[16];
    char *p = buf;
    const char *q;

    *p++ = '#';
    p = servo_fmt_digits(p, (uint32_t)id, 3);
    *p++ = 'P';
    for (q = cmd; *q != '\0'; q++) {
        *p++ = *q;
    }
    *p++ = '!';
    servo_send((const uint8_t *)buf, (uint16_t)(p - buf));
}

/* 位置控制帧 '#XXXPYYYYTZZZZ!' */
static void servo_move_frame(uint8_t id, uint16_t pwm, uint16_t time_ms)
{
    char buf[16];
    char *p = buf;

    *p++ = '#';
    p = servo_fmt_digits(p, (uint32_t)id, 3);
    *p++ = 'P';
    p = servo_fmt_digits(p, (uint32_t)pwm, 4);
    *p++ = 'T';
    p = servo_fmt_digits(p, (uint32_t)time_ms, 4);
    *p++ = '!';
    servo_send((const uint8_t *)buf, (uint16_t)(p - buf));
}

/* ------------------------- 硬件初始化 ------------------------- */

void servo_init(void)
{
    stc_usart_uart_init_t stcUartInit;
    stc_gpio_init_t stcGpioInit;
    float32_t f32Err;

    /* 1. 使能 USART3 外设时钟 */
    LL_PERIPH_WE(LL_PERIPH_FCG);
    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USART3, ENABLE);
    LL_PERIPH_WP(LL_PERIPH_FCG);

    /* 2. PB13=USART3_RX(FUNC33), PB14=USART3_TX(FUNC32) */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(GPIO_PORT_B, (GPIO_PIN_13 | GPIO_PIN_14), &stcGpioInit);
    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_13, GPIO_FUNC_33);   /* RX 预留 */
    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_14, GPIO_FUNC_32);   /* TX 接舵机 */

    /* 3. USART3: 115200 / 8bit / 1停止位 / 无校验 */
    USART_DeInit(SERVO_USART);
    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32Baudrate = SERVO_BAUDRATE;
    (void)USART_UART_Init(SERVO_USART, &stcUartInit, &f32Err);

    /* 4. 只使能发送(当前不接收回读) */
    USART_FuncCmd(SERVO_USART, USART_TX, ENABLE);
}

/* ------------------------- 对外接口 ------------------------- */

void servo_move(uint8_t id, uint16_t pwm, uint16_t time_ms)
{
    if (pwm < SERVO_PWM_MIN)      { pwm = SERVO_PWM_MIN; }
    if (pwm > SERVO_PWM_MAX)      { pwm = SERVO_PWM_MAX; }
    if (time_ms > SERVO_TIME_MAX) { time_ms = SERVO_TIME_MAX; }
    servo_move_frame(id, pwm, time_ms);
}

/* 角度(0~270°) -> PWM(500~2500) 线性映射, 135°≈1500 (270°模式) */
static uint16_t servo_angle_to_pwm(uint16_t angle)
{
    if (angle > 270U) {
        angle = 270U;
    }
    return (uint16_t)(SERVO_PWM_MIN +
           ((uint32_t)angle * (SERVO_PWM_MAX - SERVO_PWM_MIN)) / 270U);
}

void servo_move_angle(uint8_t id, uint16_t angle, uint16_t time_ms)
{
    servo_move(id, servo_angle_to_pwm(angle), time_ms);
}

void servo_move_broadcast(uint16_t pwm, uint16_t time_ms)
{
    servo_move(SERVO_ID_BROADCAST, pwm, time_ms);
}

void servo_move_multi(const servo_cmd_t *cmds, uint8_t n)
{
    uint8_t i;
    uint8_t brace;

    if ((cmds == 0) || (n == 0U)) {
        return;
    }
    if (n >= 2U) {                     /* 2 条及以上需 {} 包裹 */
        brace = '{';
        servo_send(&brace, 1U);
    }
    for (i = 0U; i < n; i++) {
        uint16_t pwm = cmds[i].pwm;
        uint16_t t   = cmds[i].time_ms;
        if (pwm < SERVO_PWM_MIN)      { pwm = SERVO_PWM_MIN; }
        if (pwm > SERVO_PWM_MAX)      { pwm = SERVO_PWM_MAX; }
        if (t > SERVO_TIME_MAX)       { t = SERVO_TIME_MAX; }
        servo_move_frame(cmds[i].id, pwm, t);
    }
    if (n >= 2U) {
        brace = '}';
        servo_send(&brace, 1U);
    }
}

void servo_unlock_torque(uint8_t id)  { servo_send_cmd(id, "ULK"); }
void servo_lock_torque(uint8_t id)    { servo_send_cmd(id, "ULR"); }
void servo_stop(uint8_t id)           { servo_send_cmd(id, "DST"); }
void servo_pause(uint8_t id)          { servo_send_cmd(id, "DPT"); }
void servo_resume(uint8_t id)         { servo_send_cmd(id, "DCT"); }
void servo_calib_mid(uint8_t id)      { servo_send_cmd(id, "SCK"); }
void servo_factory_reset(uint8_t id)  { servo_send_cmd(id, "CLE"); }

void servo_set_mode(uint8_t id, uint8_t mode)
{
    char cmd[5];
    if (mode < SERVO_MODE_270_CW)       { mode = SERVO_MODE_270_CW; }
    if (mode > SERVO_MODE_360_TIME_CCW) { mode = SERVO_MODE_360_TIME_CCW; }
    cmd[0] = 'M'; cmd[1] = 'O'; cmd[2] = 'D';
    cmd[3] = (char)('0' + mode);
    cmd[4] = '\0';
    servo_send_cmd(id, cmd);
}

void servo_set_id(uint8_t old_id, uint8_t new_id)
{
    char cmd[8];
    char *p = cmd;
    *p++ = 'I'; *p++ = 'D';
    p = servo_fmt_digits(p, (uint32_t)new_id, 3);
    *p = '\0';
    servo_send_cmd(old_id, cmd);   /* 注意: 改 ID 时舵机不要串联 */
}

void servo_set_baud(uint8_t id, uint8_t code)
{
    char cmd[4];
    /* 1-9600 2-19200 3-38400 4-57600 5-115200 6-128000 7-256000 8-1000000 */
    if (code < 1U) { code = 1U; }
    if (code > 8U) { code = 8U; }
    cmd[0] = 'B'; cmd[1] = 'D';
    cmd[2] = (char)('0' + code);
    cmd[3] = '\0';
    servo_send_cmd(id, cmd);
}

void servo_boot(void)
{
    /* 供电直接上电, 舵机内部自检需要时间 */
    DDL_DelayMS(100);

    /* 释放扭力 -> 柔和回中位 -> 恢复扭力锁定 */
    servo_unlock_torque(SERVO_ID_BROADCAST);
    DDL_DelayMS(200);
    servo_move_broadcast(SERVO_PWM_MID, 500);
    DDL_DelayMS(600);
    servo_lock_torque(SERVO_ID_BROADCAST);
}

void servo_set_mid_here(uint8_t id)
{
    servo_unlock_torque(id);        /* PULK: 释放扭力, 此时可手动摆正 */
    DDL_DelayMS(200);
    servo_calib_mid(id);            /* PSCK: 把当前位置标定为 1500 中值 */
    DDL_DelayMS(200);
    servo_lock_torque(id);          /* PULR: 恢复扭力, 锁在当前位置 */
}

void servo_run(void)
{
    servo_move(SERVO_ID_DEFAULT, 1500, 200);
    DDL_DelayMS(300);
    servo_move(SERVO_ID_DEFAULT, 2200, 200);
    DDL_DelayMS(300);
}
