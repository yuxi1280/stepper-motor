#include "servo.h"

/* ============================================================================
 * 众灵科技 ZP 系列总线舵机 收发驱动 (USART3) —— 单舵机精简版
 * ----------------------------------------------------------------------------
 * 发送: 中断驱动 + 环形缓冲, 组 ASCII 帧后入队, 不阻塞主循环
 * 接收: RX 中断只搬字节进缓冲, servo_rx_task() 在主循环拼帧并解析
 * 帧:   '#' + ID(3位补零) + 'P' + [body] + '!'
 * 回读: 仅解析 #OK! 与位置 #000Pnnnn!
 * ==========================================================================*/

/* ---------------- 环形缓冲(大小 2 的幂, 用掩码取模) ---------------- */
#define SERVO_TX_MASK       (SERVO_TX_BUF_SIZE - 1U)
#define SERVO_RX_MASK       (SERVO_RX_BUF_SIZE - 1U)
#define SERVO_TX_NEXT(p)    ((uint16_t)(((p) + 1U) & SERVO_TX_MASK))
#define SERVO_RX_NEXT(p)    ((uint16_t)(((p) + 1U) & SERVO_RX_MASK))

static volatile char     s_tx[SERVO_TX_BUF_SIZE];
static volatile uint16_t s_tx_head;
static volatile uint16_t s_tx_tail;

static volatile char     s_rx[SERVO_RX_BUF_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;

/* 拼帧缓冲(仅在 servo_rx_task 里使用) */
static char     s_line[SERVO_LINE_MAX];
static uint16_t s_line_len;

/* ---------------- 回读状态 ---------------- */
volatile servo_status_t g_servo_st;
volatile uint32_t       g_servo_ok_cnt;

/* ==================== 硬件层: USART3 中断 ====================
 * 必须与驱动弱符号同名, 不能改名:
 *   USART3_RxFull / USART3_TxEmpty / USART3_RxError
 * 这三个走共享中断 IRQ137 -> INT137_IRQn
 * ========================================================== */

/* 接收完成中断: 读 RDR 存入接收缓冲 */
void USART3_RxFull_IrqHandler(void)
{
    uint8_t  c;
    uint16_t tail;

    (void)CM_USART3->SR;                 /* 读 SR 清 ORE/FE/PE 等错误标志 */

    if (USART_GetStatus(CM_USART3, USART_FLAG_RX_FULL)) {
        c    = (uint8_t)USART_ReadData(CM_USART3);
        tail = SERVO_RX_NEXT(s_rx_tail);
        if (tail != s_rx_head) {
            s_rx[s_rx_tail] = (char)c;
            s_rx_tail = tail;
        }
    }
}

/* 发送空中断: 从发送缓冲取数据写 TDR; 缓冲空则关 TXE 中断 */
void USART3_TxEmpty_IrqHandler(void)
{
    if (s_tx_head == s_tx_tail) {
        USART_FuncCmd(CM_USART3, USART_INT_TX_EMPTY, DISABLE);
    } else {
        USART_WriteData(CM_USART3, (uint8_t)s_tx[s_tx_head]);
        s_tx_head = SERVO_TX_NEXT(s_tx_head);
    }
}

/* 接收错误中断: 读 SR + RDR 清标志, 丢弃错误数据 */
void USART3_RxError_IrqHandler(void)
{
    (void)CM_USART3->SR;
    (void)USART_ReadData(CM_USART3);
}

/* 启动发送: 若缓冲非空, 写首字节并开 TXE 中断 */
static void servo_start_tx(void)
{
    if (s_tx_head == s_tx_tail) {
        return;
    }
    USART_WriteData(CM_USART3, (uint8_t)s_tx[s_tx_head]);
    s_tx_head = SERVO_TX_NEXT(s_tx_head);
    USART_FuncCmd(CM_USART3, USART_INT_TX_EMPTY, ENABLE);
}

/* ==================== 底层发送(内部) ==================== */

/* 入队 1 字节并启动发送(缓冲满则等待中断腾空间) */
static void servo_send_byte(uint8_t c)
{
    uint16_t next = SERVO_TX_NEXT(s_tx_tail);

    while (next == s_tx_head) {
        /* 缓冲满: 等待发送中断推进 head */
    }
    s_tx[s_tx_tail] = (char)c;
    s_tx_tail = next;
    servo_start_tx();
}

/* ==================== 组帧 ==================== */

/* 把 val 按 digits 位十进制补零写入 dst, 返回写后的指针 */
static char *servo_fmt_u(char *dst, uint32_t val, uint8_t digits)
{
    uint8_t i;

    for (i = 0U; i < digits; i++) {
        dst[digits - 1U - i] = (char)('0' + (val % 10U));
        val /= 10U;
    }
    return &dst[digits];
}

/* 发送 '#ID(3位)P + body + !' */
static void servo_send_cmd(const char *body)
{
    char buf[16];
    char *p = buf;
    const char *q;

    *p++ = '#';
    p = servo_fmt_u(p, SERVO_ID, 3U);
    *p++ = 'P';
    for (q = body; *q != '\0'; q++) {
        *p++ = *q;
    }
    *p++ = '!';

    q = buf;                             /* 逐字节入队 */
    while (p != (char *)q) {
        servo_send_byte((uint8_t)*q++);
    }
}

/* ==================== 位置控制 ==================== */

void servo_move(uint16_t pwm, uint16_t time_ms)
{
    char body[10];
    char *p = body;

    if (pwm < SERVO_PWM_MIN)      { pwm = SERVO_PWM_MIN; }
    if (pwm > SERVO_PWM_MAX)      { pwm = SERVO_PWM_MAX; }
    if (time_ms > SERVO_TIME_MAX) { time_ms = SERVO_TIME_MAX; }

    p = servo_fmt_u(p, pwm, 4U);
    *p++ = 'T';
    p = servo_fmt_u(p, time_ms, 4U);
    *p = '\0';

    servo_send_cmd(body);
}

/* ==================== 控制指令(均回 #OK!) ==================== */

void servo_release(void)      { servo_send_cmd("ULK"); }  /* PULK 释放扭力 */
void servo_restore(void)      { servo_send_cmd("ULR"); }  /* PULR 恢复扭力 */
void servo_pause(void)        { servo_send_cmd("DPT"); }  /* PDPT 暂停 */
void servo_resume(void)       { servo_send_cmd("DCT"); }  /* PDCT 继续 */
void servo_stop(void)         { servo_send_cmd("DST"); }  /* PDST 停止在当前位置 */
void servo_set_mid_here(void) { servo_send_cmd("SCK"); }  /* PSCK 中值标定 */

/* ==================== 读取位置(回 #000P1500!) ==================== */

void servo_read_pos(void)     { servo_send_cmd("RAD"); }  /* PRAD */

/* ==================== 接收解析(仅核心) ==================== */

/* 从接收缓冲取 1 字节; 空返回 -1 */
static int16_t servo_rx_pop(void)
{
    uint16_t head;

    if (s_rx_head == s_rx_tail) {
        return -1;
    }
    head = s_rx_head;
    s_rx_head = SERVO_RX_NEXT(head);
    return (int16_t)(uint8_t)s_rx[head];
}

/* 解析无符号十进制整数, 指针前移 */
static uint32_t servo_parse_u(const char **pp)
{
    uint32_t v = 0U;

    while ((**pp >= '0') && (**pp <= '9')) {
        v = (v * 10U) + (uint32_t)(**pp - '0');
        (*pp)++;
    }
    return v;
}

/* 解析一帧完整应答(以 '#' 开头, 以 '!' 结尾) */
static void servo_parse_frame(const char *s, uint16_t len)
{
    const char *p;

    /* #OK! */
    if ((len == 4U) && (s[0] == '#') && (s[1] == 'O') && (s[2] == 'K') && (s[3] == '!')) {
        g_servo_st.last_reply = SERVO_REPLY_OK;
        g_servo_ok_cnt++;
        return;
    }

    /* 位置回读: #000P1500!  (至少 6 字节, 第 5 个字符为 'P') */
    if ((len >= 6U) && (s[0] == '#') && (s[4] == 'P')) {
        p = &s[5];
        if ((p[0] >= '0') && (p[0] <= '9')) {
            g_servo_st.position   = (uint16_t)servo_parse_u(&p);
            g_servo_st.last_reply = SERVO_REPLY_POS;
        }
    }
    /* 其它回帧本版不解析, 直接忽略 */
}

/* 主循环任务: 消费接收缓冲, 按 '#'...'!' 拼帧后解析 */
void servo_rx_task(void)
{
    int16_t c;

    while ((c = servo_rx_pop()) >= 0) {
        char ch = (char)c;

        if (ch == '#') {
            s_line_len = 0U;
            s_line[s_line_len++] = ch;
        } else if (s_line_len != 0U) {
            if (s_line_len >= (SERVO_LINE_MAX - 1U)) {
                s_line_len = 0U;             /* 超长帧: 丢弃 */
            } else {
                s_line[s_line_len++] = ch;
                if (ch == '!') {
                    s_line[s_line_len] = '\0';
                    servo_parse_frame(s_line, s_line_len);
                    s_line_len = 0U;
                }
            }
        }
        /* 帧头 '#' 之前的杂散字节直接忽略 */
    }
}

/* ==================== 初始化 ==================== */

void servo_init(void)
{
    stc_usart_uart_init_t stcUartInit;
    float32_t f32Err;

    /* 1. 复位收发缓冲与状态 */
    s_tx_head  = 0U;
    s_tx_tail  = 0U;
    s_rx_head  = 0U;
    s_rx_tail  = 0U;
    s_line_len = 0U;
    g_servo_st.last_reply = SERVO_REPLY_NONE;

    /* 2. 使能 USART3 时钟 */
    LL_PERIPH_WE(LL_PERIPH_FCG);
    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USART3, ENABLE);
    LL_PERIPH_WP(LL_PERIPH_FCG);

    /* 3. GPIO 复用(PB13/PB14)已在 main.c App_PortCfg 配置 */

    /* 4. 配置 USART3: 115200 / 8bit / 1停止位 / 无校验 */
    USART_DeInit(CM_USART3);
    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32Baudrate = SERVO_BAUDRATE;
    (void)USART_UART_Init(CM_USART3, &stcUartInit, &f32Err);

    /* 5. 使能 TX / RX 功能 */
    USART_FuncCmd(CM_USART3, USART_TX | USART_RX, ENABLE);

    /* 6. 共享中断(USART3 RI/TI/EI -> IRQ137)并启用 NVIC */
    (void)INTC_ShareIrqCmd(INT_SRC_USART3_RI, ENABLE);
    (void)INTC_ShareIrqCmd(INT_SRC_USART3_TI, ENABLE);
    (void)INTC_ShareIrqCmd(INT_SRC_USART3_EI, ENABLE);
    NVIC_ClearPendingIRQ(INT137_IRQn);
    NVIC_SetPriority(INT137_IRQn, DDL_IRQ_PRIO_02);
    NVIC_EnableIRQ(INT137_IRQn);

    /* 7. 使能接收中断(RIE 同时覆盖接收完成与接收错误) */
    USART_FuncCmd(CM_USART3, USART_INT_RX, ENABLE);
}

/* ==================== 辅助接口 ==================== */

void servo_flush_rx(void)
{
    s_rx_head  = s_rx_tail;     /* 丢弃残留字节 */
    s_line_len = 0U;
}

uint8_t servo_wait_reply(uint8_t reply, uint32_t timeout_ms)
{
    g_servo_st.last_reply = SERVO_REPLY_NONE;

    while (timeout_ms-- > 0U) {
        servo_rx_task();
        if (g_servo_st.last_reply == reply) {
            return 1U;
        }
        DDL_DelayMS(1U);
    }
    return 0U;
}


void servo_run(void)
{
    servo_move(SERVO_PWM_MID, 500U);
    DDL_DelayMS(800U);
    servo_move(2200U, 500U);
    DDL_DelayMS(800U);
    servo_move(SERVO_PWM_MID, 500U);
    DDL_DelayMS(800U);
}


