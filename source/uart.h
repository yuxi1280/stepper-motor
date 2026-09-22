#ifndef __UART_H_
#define __UART_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "main.h"

/* 缓冲大小必须是 2 的幂(BUF_NEXT 用掩码取模) */
#define UART_BUF_RX_MAX     512
#define UART_BUF_TX_MAX     256

#define BUF_NEXT(ptr, buf)  ((ptr + 1) & (sizeof(buf) - 1))

/* 串口实例索引 */
typedef enum {
    UART_IDX_0,          /* USART1: 步进电机(Emm 总线协议) */
    UART_IDX_MAX
} uart_idx_t;

typedef struct {
    volatile char     dat[UART_BUF_TX_MAX];
    volatile uint32_t head;
    volatile uint32_t tail;
} uart_tx_buf_t;

typedef struct {
    volatile char     dat[UART_BUF_RX_MAX];
    volatile uint32_t head;
    volatile uint32_t tail;
} uart_rx_buf_t;

typedef struct uart_st {
    CM_USART_TypeDef *h_usart;       /* USART 外设指针 */
    uint32_t baud;
    uint8_t  bit;                    /* 8/9 位 */
    uint8_t  stop;                   /* 停止位 */

    uart_tx_buf_t *t_buf;
    uart_rx_buf_t *r_buf;

    /* ---- 通用框架回调(uart.c 实现) ---- */
    bool     (*buf_write_1B_no_tx)(const struct uart_st *u, char c);
    bool     (*buf_write_char)(const struct uart_st *u, char c);
    void     (*buf_write_s)(const struct uart_st *u, const char *str);
    void     (*buf_write_n)(const struct uart_st *u, const uint8_t *str, uint16_t n);
    uint16_t (*buf_read_c)(const struct uart_st *u);
    bool     (*t_buf_empty_check)(const struct uart_st *u);
    bool     (*r_buf_empty_check)(const struct uart_st *u);

    /* ---- 硬件层回调 ---- */
    void     (*start_tx)(void);
    void     (*uart_suspend_handle)(void);
} uart_t;

void uart_init(uart_t *u);

/* ---- 应用层便捷接口 ---- */
void     uart_send_str(uart_idx_t idx, const char *str);
void     uart_send_data(uart_idx_t idx, const uint8_t *data, uint16_t len);
int16_t  uart_recv_byte(uart_idx_t idx);                        /* 空缓冲返回 -1  */
uint16_t uart_recv(uart_idx_t idx, uint8_t *dst, uint16_t len); /* 取最多 len 字节 */
void     uart_driver_init(void);

#endif /* __UART_H_ */
