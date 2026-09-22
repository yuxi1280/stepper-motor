#include "one.h"

/* ================= 底层: USART2 阻塞发送 ================= */

/* 等待 TXE 后写入一个字节 */
static void usart2_putc(uint8_t c)
{
    while (USART_GetStatus(CM_USART2, USART_FLAG_TX_EMPTY) == RESET) {
    }
    USART_WriteData(CM_USART2, c);
}

/* 发送 len 字节, 并等待最后一字节移位发送完成(TC 由写 TDR 自动清零) */
void one_send_data(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    for (i = 0U; i < len; i++) {
        usart2_putc(data[i]);
    }
    while (USART_GetStatus(CM_USART2, USART_FLAG_TX_CPLT) == RESET) {
    }
}

/* ================= 协议组帧 ================= */

/* 将 val 按 digits 位十进制补零写入 dst, 返回写后的指针 */
static char *servo_fmt_digits(char *dst, uint32_t val, uint8_t digits)
{
    uint8_t i;
    for (i = 0U; i < digits; i++) {
        dst[digits - 1U - i] = (char)('0' + (val % 10U));
        val /= 10U;
    }
    return &dst[digits];
}

/* 发送 '#ID P 命令 !' 形式的指令 */
static void servo_send_cmd(uint8_t id, const char *cmd)
{
    char buf[12];
    char *p = buf;
    const char *q;

    *p++ = '#';
    p = servo_fmt_digits(p, id, 3);
    *p++ = 'P';
    for (q = cmd; *q != '\0'; q++) {
        *p++ = *q;
    }
    *p++ = '!';
    one_send_data((const uint8_t *)buf, (uint16_t)(p - buf));
}

/* 位置控制帧 '#XXXPYYYYTZZZZ!' */
static void servo_move_frame(uint8_t id, uint16_t pwm, uint16_t time_ms)
{
    char buf[15];
    char *p = buf;

    *p++ = '#';
    p = servo_fmt_digits(p, id, 3);
    *p++ = 'P';
    p = servo_fmt_digits(p, pwm, 4);
    *p++ = 'T';
    p = servo_fmt_digits(p, time_ms, 4);
    *p++ = '!';
    one_send_data((const uint8_t *)buf, (uint16_t)(p - buf));
}

/* ================= 对外接口 ================= */

void one_init(void)
{
    stc_usart_uart_init_t stcUartInit;
    float32_t f32Err;

    /* 1. 使能 USART2 时钟 */
    LL_PERIPH_WE(LL_PERIPH_FCG);
    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USART2, ENABLE);
    LL_PERIPH_WP(LL_PERIPH_FCG);

    /* 2. GPIO 复用 PA10=USART2_TX 已在 main.c App_PortCfg() 配置 */

    /* 3. 配置 USART2: 115200 / 8bit / 1停止位 / 无校验 */
    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32Baudrate = SERVO_BUS_BAUDRATE;
    (void)USART_UART_Init(CM_USART2, &stcUartInit, &f32Err);

    /* 4. 只使能发送, 暂不接收 */
    USART_FuncCmd(CM_USART2, USART_TX, ENABLE);
    
}

void servo_move(uint8_t id, uint16_t pwm, uint16_t time_ms)
{
    /* PWM 限幅 */
    if (pwm < SERVO_PWM_MIN) pwm = SERVO_PWM_MIN;
    if (pwm > SERVO_PWM_MAX) pwm = SERVO_PWM_MAX;
    servo_move_frame(id, pwm, time_ms);
}

/* 角度(0~270°)→PWM(500~2500) 线性映射, 适用于 270° 模式, 135°≈1500 */
static uint16_t servo_angle_to_pwm(uint16_t angle)
{
    if (angle > 270U) angle = 270U;
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
    uint8_t ch;

    if (n == 0U) {
        return;
    }
    /* 2条以上叠加需要 {} 包裹 */
    if (n >= 2U) {
        ch = '{';
        one_send_data(&ch, 1U);
    }
    for (i = 0U; i < n; i++) {
        servo_move_frame(cmds[i].id, cmds[i].pwm, cmds[i].time_ms);
    }
    if (n >= 2U) {
        ch = '}';
        one_send_data(&ch, 1U);
    }
}

void servo_unlock_torque(uint8_t id)   { servo_send_cmd(id, "ULK"); }
void servo_lock_torque(uint8_t id)     { servo_send_cmd(id, "ULR"); }
void servo_stop(uint8_t id)            { servo_send_cmd(id, "DST"); }
void servo_pause(uint8_t id)           { servo_send_cmd(id, "DPT"); }
void servo_resume(uint8_t id)          { servo_send_cmd(id, "DCT"); }
void servo_factory_reset(uint8_t id)   { servo_send_cmd(id, "CLE"); }

void servo_set_mode(uint8_t id, uint8_t mode)
{
    char cmd[4];
    if (mode < 1U) mode = 1U;
    if (mode > 8U) mode = 8U;
    cmd[0] = 'M'; cmd[1] = 'O'; cmd[2] = 'D';
    cmd[3] = (char)('0' + mode);
    servo_send_cmd(id, cmd);
}

void servo_set_id(uint8_t old_id, uint8_t new_id)
{
    char cmd[8];
    char *p = cmd;
    *p++ = 'I'; *p++ = 'D';
    p = servo_fmt_digits(p, new_id, 3);
    *p = '\0';
    servo_send_cmd(old_id, cmd);
}

void servo_set_baud(uint8_t id, uint8_t code)
{
    char cmd[4];
    /* 1-9600 2-19200 3-38400 4-57600 5-115200 6-128000 7-256000 8-1000000 */
    if (code < 1U) code = 1U;
    if (code > 8U) code = 8U;
    cmd[0] = 'B'; cmd[1] = 'D';
    cmd[2] = (char)('0' + code);
    servo_send_cmd(id, cmd);
}
 

void servo_calib_mid(uint8_t id) { servo_send_cmd(id, "SCK"); }

void servo_start(void)
{
    servo_move_broadcast(1500, 300);
    DDL_DelayMS(500);
    servo_move_broadcast(2200, 300);   
    DDL_DelayMS(500);
    servo_move_broadcast(1500, 300);
    DDL_DelayMS(500);

}
    

void servo_initial(void)
{
    servo_unlock_torque(255);
    DDL_DelayMS(300);
    servo_calib_mid(255);
    DDL_DelayMS(300);  
    servo_lock_torque(255); 
    DDL_DelayMS(300);
}

