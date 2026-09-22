#include "uart.h"

/* ==================== 实例(USART1 = 步进电机) ==================== */
static uart_tx_buf_t g_uart0_tx_buf = {.head = 0, .tail = 0};
static uart_rx_buf_t g_uart0_rx_buf = {.head = 0, .tail = 0};
static uart_t      g_uart0;

/* ==================== 通用框架 ==================== */

/* 只入发送缓冲, 不立即启动发送 */
static bool buf_write_1B_no_tx(const uart_t *u, char c)
{
    uint32_t tail = BUF_NEXT(u->t_buf->tail, u->t_buf->dat);
    while (u->t_buf->head == tail) {
        u->uart_suspend_handle();
    }
    u->t_buf->dat[u->t_buf->tail] = c;
    u->t_buf->tail = tail;
    return true;
}

/* 入缓冲并启动发送 */
static bool buf_write_char(const uart_t *u, char c)
{
    uint32_t tail = BUF_NEXT(u->t_buf->tail, u->t_buf->dat);
    while (u->t_buf->head == tail) {
        u->uart_suspend_handle();
    }
    u->t_buf->dat[u->t_buf->tail] = c;
    u->t_buf->tail = tail;
    u->start_tx();
    return true;
}

/* 发送字符串 */
static void buf_write_s(const uart_t *u, const char *str)
{
    char *p, c;
    p = (char *)str;
    while ((c = *p++) != '\0') {
        u->buf_write_1B_no_tx(u, c);
    }
    u->start_tx();
}

/* 发送指定长度数据 */
static void buf_write_n(const uart_t *u, const uint8_t *str, uint16_t n)
{
    uint8_t *p = (uint8_t *)str;
    while (n--) {
        u->buf_write_1B_no_tx(u, (char)*p++);
    }
    u->start_tx();
}

/* 从接收缓冲读取一字节; 空则返回 (uint16_t)-1 */
static uint16_t buf_read_c(const uart_t *u)
{
    uint8_t c;
    if (u->r_buf->head == u->r_buf->tail) {
        return (uint16_t)-1;
    }
    c = (uint8_t)u->r_buf->dat[u->r_buf->head];
    u->r_buf->head = BUF_NEXT(u->r_buf->head, u->r_buf->dat);
    return c;
}

static bool t_buf_empty_check(const uart_t *u)
{
    return (u->t_buf->head == u->t_buf->tail) ? true : false;
}

static bool r_buf_empty_check(const uart_t *u)
{
    return (u->r_buf->head == u->r_buf->tail) ? true : false;
}

/* ==================== 硬件层(LL 库) ====================
 * USART1: PA03 = RX(FUNC_33) / PA04 = TX(FUNC_32), 已在 main.c App_PortCfg 配置
 * RI/TI 走共享中断 -> INT136_IRQn
 * 注意: 下面三个中断函数名必须与 DDL 弱符号一致, 不能改名!
 */

/* 接收完成中断: 读 RDR 存入接收缓冲 */
void USART1_RxFull_IrqHandler(void)
{
    uint8_t  c;
    uint32_t tail;

    (void)CM_USART1->SR;                 /* 读 SR 清 ORE/FE/PE 等错误标志 */

    if (USART_GetStatus(CM_USART1, USART_FLAG_RX_FULL)) {
        c    = (uint8_t)USART_ReadData(CM_USART1);
        tail = BUF_NEXT(g_uart0.r_buf->tail, g_uart0.r_buf->dat);
        if (tail != g_uart0.r_buf->head) {
            g_uart0.r_buf->dat[g_uart0.r_buf->tail] = (char)c;
            g_uart0.r_buf->tail = tail;
        }
    }
}

/* 发送空中断: 从发送缓冲取数据写 TDR; 缓冲空则关 TXE 中断 */
void USART1_TxEmpty_IrqHandler(void)
{
    uint32_t head;

    if (g_uart0.t_buf->head == g_uart0.t_buf->tail) {
        USART_FuncCmd(CM_USART1, USART_INT_TX_EMPTY, DISABLE);
    } else {
        head = g_uart0.t_buf->head;
        USART_WriteData(CM_USART1, (uint8_t)g_uart0.t_buf->dat[head]);
        g_uart0.t_buf->head = BUF_NEXT(head, g_uart0.t_buf->dat);
    }
}

/* 接收错误中断: 读 SR + RDR 清标志, 丢弃错误数据 */
void USART1_RxError_IrqHandler(void)
{
    (void)CM_USART1->SR;
    (void)USART_ReadData(CM_USART1);
}

/* 发送缓冲满时的挂起处理(按需实现) */
static void uart_suspend_handle(void)
{
}

/* 启动发送: 若缓冲非空, 写首字节并开 TXE 中断 */
static void start_tx(void)
{
    uint32_t head;

    if (g_uart0.t_buf->head == g_uart0.t_buf->tail) {
        return;
    }
    head = g_uart0.t_buf->head;
    USART_WriteData(CM_USART1, (uint8_t)g_uart0.t_buf->dat[head]);
    g_uart0.t_buf->head = BUF_NEXT(head, g_uart0.t_buf->dat);
    USART_FuncCmd(CM_USART1, USART_INT_TX_EMPTY, ENABLE);
}

/* 硬件初始化 */
static void hw_init(void)
{
    stc_usart_uart_init_t stcUartInit;
    float32_t f32Err;

    /* 1. 使能 USART1 时钟 */
    LL_PERIPH_WE(LL_PERIPH_FCG);
    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USART1, ENABLE);
    LL_PERIPH_WP(LL_PERIPH_FCG);

    /* 2. GPIO 复用已在 main.c App_PortCfg 配置 */

    /* 3. 配置 USART1: 8bit / 1停止位 / 无校验 / 115200(无硬件流控) */
    USART_DeInit(CM_USART1);
    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32Baudrate = g_uart0.baud;
    (void)USART_UART_Init(CM_USART1, &stcUartInit, &f32Err);

    /* 4. 使能 TX / RX 功能 */
    USART_FuncCmd(CM_USART1, USART_TX | USART_RX, ENABLE);

    /* 5. 配置共享中断(USART1 接收/发送 -> INT136)并启用 NVIC */
    (void)INTC_ShareIrqCmd(INT_SRC_USART1_RI, ENABLE);
    (void)INTC_ShareIrqCmd(INT_SRC_USART1_TI, ENABLE);
    NVIC_ClearPendingIRQ(INT136_IRQn);
    NVIC_SetPriority(INT136_IRQn, DDL_IRQ_PRIO_01);
    NVIC_EnableIRQ(INT136_IRQn);

    /* 6. 使能接收中断(RIE 同时覆盖接收完成与接收错误) */
    USART_FuncCmd(CM_USART1, USART_INT_RX, ENABLE);
}

/* ==================== 初始化接口 ==================== */
void uart_init(uart_t *u)
{
    u->buf_write_1B_no_tx = buf_write_1B_no_tx;
    u->buf_write_char     = buf_write_char;
    u->buf_write_s        = buf_write_s;
    u->buf_write_n        = buf_write_n;
    u->buf_read_c         = buf_read_c;
    u->t_buf_empty_check  = t_buf_empty_check;
    u->r_buf_empty_check  = r_buf_empty_check;
}

/* ==================== 应用层便捷接口 ==================== */
void uart_send_str(uart_idx_t idx, const char *str)
{
    if (idx == UART_IDX_0) {
        g_uart0.buf_write_s(&g_uart0, str);
    }
}

void uart_send_data(uart_idx_t idx, const uint8_t *data, uint16_t len)
{
    if (idx == UART_IDX_0) {
        g_uart0.buf_write_n(&g_uart0, data, len);
    }
}

int16_t uart_recv_byte(uart_idx_t idx)
{
    uint16_t c;
    if (idx == UART_IDX_0) {
        c = g_uart0.buf_read_c(&g_uart0);
        return (c == (uint16_t)-1) ? -1 : (int16_t)c;
    }
    return -1;
}

uint16_t uart_recv(uart_idx_t idx, uint8_t *dst, uint16_t len)
{
    uint16_t n = 0U;
    int16_t  c;

    while (n < len) {
        c = uart_recv_byte(idx);
        if (c < 0) {
            break;
        }
        dst[n++] = (uint8_t)c;
    }
    return n;
}

void uart_driver_init(void)
{
    g_uart0.h_usart = CM_USART1;
    g_uart0.baud     = 115200U;
    g_uart0.bit      = 8U;
    g_uart0.stop     = 1U;
    g_uart0.t_buf    = &g_uart0_tx_buf;
    g_uart0.r_buf    = &g_uart0_rx_buf;
    g_uart0.start_tx = start_tx;
    g_uart0.uart_suspend_handle = uart_suspend_handle;

    uart_init(&g_uart0);
    hw_init();
}
