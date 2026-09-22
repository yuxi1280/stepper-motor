#include "can.h"
#include <string.h>

/* ================= 内部帧结构 ================= */
typedef struct {
    uint32_t u32Id;          /* 低29位ID; bit31=1 表示扩展帧 */
    uint8_t  u8Dlc;
    uint8_t  au8Data[8];
} can_frame_t;

/* 发送环形缓冲 */
typedef struct {
    can_frame_t buf[CAN_TX_RING_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} can_tx_ring_t;

static can_tx_ring_t     s_txRing;
static volatile uint8_t  s_txBusy = 0U;      /* PTB 正在发送 */

/* 接收队列(中断填充, can_poll 消费) */
typedef struct {
    can_frame_t buf[CAN_RX_QUEUE_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} can_rx_queue_t;
static can_rx_queue_t s_rxQueue;

static uint32_t s_can_rx_count = 0U;

/* ================= 环形缓冲辅助 ================= */
static inline uint16_t can_ring_next(uint16_t i) { return (uint16_t)((i + 1U) % CAN_TX_RING_SIZE); }
static inline uint8_t  can_ring_full(const can_tx_ring_t *r) { return (uint8_t)(can_ring_next(r->head) == r->tail); }
static inline uint8_t  can_ring_empty(const can_tx_ring_t *r) { return (uint8_t)(r->head == r->tail); }
static inline uint8_t  can_is_ext(uint32_t id) { return (uint8_t)((id >> 31U) & 1U); }

/* 从环形缓冲取出一帧送入硬件 PTB 并启动 */
static void can_send_next(void)
{
    stc_can_tx_frame_t tx;
    can_frame_t *f;

    if (s_txBusy || can_ring_empty(&s_txRing)) {
        return;
    }

    f = &s_txRing.buf[s_txRing.head];
    memset(&tx, 0, sizeof(tx));
    tx.u32ID  = f->u32Id & CAN_EXT_ID_MASK;
    tx.IDE    = can_is_ext(f->u32Id) ? 1U : 0U;
    tx.DLC    = (f->u8Dlc > 8U) ? 8U : f->u8Dlc;
    memcpy(tx.au8Data, f->au8Data, tx.DLC);

    if (CAN_FillTxFrame(CM_CAN, CAN_TX_BUF_PTB, &tx) != LL_OK) {
        return;                              /* PTB 忙, 等发送完成中断再取 */
    }
    CAN_StartTx(CM_CAN, CAN_TX_REQ_PTB);
    s_txBusy = 1U;
    s_txRing.head = can_ring_next(s_txRing.head);
}

/* ================= 发送接口 ================= */
void can_tx_bytes(uint32_t ext_id, const uint8_t *p, uint8_t n)
{
    uint8_t off, len;

    if (p == NULL || n == 0U) {
        return;
    }
    for (off = 0U; off < n; off += 8U) {
        can_frame_t f;
        len = (uint8_t)((n - off > 8U) ? 8U : (n - off));
        f.u32Id = (ext_id & CAN_EXT_ID_MASK) | (1UL << 31U); /* 标记扩展帧 */
        f.u8Dlc = len;
        memcpy(f.au8Data, &p[off], len);

        if (can_ring_full(&s_txRing)) {
            return;                          /* 缓冲满, 丢弃 */
        }
        s_txRing.buf[s_txRing.tail] = f;
        s_txRing.tail = can_ring_next(s_txRing.tail);
    }
    can_send_next();
}

void can_tx_bytes_std(uint32_t std_id, const uint8_t *p, uint8_t n)
{
    can_frame_t f;

    if (p == NULL || n == 0U) {
        return;
    }
    f.u32Id = std_id & CAN_STD_ID_MASK;      /* bit31=0 表示标准帧 */
    f.u8Dlc = (n > 8U) ? 8U : n;
    memcpy(f.au8Data, p, f.u8Dlc);

    if (can_ring_full(&s_txRing)) {
        return;
    }
    s_txRing.buf[s_txRing.tail] = f;
    s_txRing.tail = can_ring_next(s_txRing.tail);
    can_send_next();
}

/* 过流上报: 发送一帧扩展帧 */
void can_report_overcurrent(void)
{
    uint8_t msg[4] = { 0x00, 0x00, 0x00, 0x01 };
    can_tx_bytes(CAN_OVER_CURRENT_ID, msg, sizeof(msg));
}

/* ================= 接收 ================= */
static void can_rx_put(const can_frame_t *f)
{
    uint16_t next = (uint16_t)((s_rxQueue.head + 1U) % CAN_RX_QUEUE_SIZE);
    if (next == s_rxQueue.tail) {
        return;                              /* 队列满, 丢弃 */
    }
    s_rxQueue.buf[s_rxQueue.head] = *f;
    s_rxQueue.head = next;
}

/* ================= 中断处理(由 INT008 -> m_apfnIrqHandler 回调) ================= */
static void can_irq_handler(void)
{
    /* 接收 */
    if (CAN_GetStatus(CM_CAN, CAN_FLAG_RX) == SET) {
        stc_can_rx_frame_t rx;
        while (CAN_GetRxFrame(CM_CAN, &rx) == LL_OK) {
            can_frame_t f;
            f.u32Id = rx.u32ID | ((rx.IDE) ? (1UL << 31U) : 0UL);
            f.u8Dlc = rx.DLC;
            memcpy(f.au8Data, rx.au8Data, (rx.DLC > 8U) ? 8U : rx.DLC);
            can_rx_put(&f);
            s_can_rx_count++;
        }
        CAN_ClearStatus(CM_CAN, CAN_FLAG_RX);
    }

    /* PTB 发送完成 */
    if (CAN_GetStatus(CM_CAN, CAN_FLAG_PTB_TX) == SET) {
        CAN_ClearStatus(CM_CAN, CAN_FLAG_PTB_TX);
        s_txBusy = 0U;
        can_send_next();
    }

    /* 溢出 / 错误标志清理 */
    CAN_ClearStatus(CM_CAN, CAN_FLAG_RX_BUF_OVF | CAN_FLAG_BUS_ERR |
                            CAN_FLAG_ARBITR_LOST | CAN_FLAG_ERR_INT |
                            CAN_FLAG_RX_BUF_FULL | CAN_FLAG_RX_BUF_WARN);
}

/* ================= 广播自检应答 =================
 * 收到主机广播查询 0A 00 00 后, 用本机地址 ID 应答 8A 00 00
 */
static void can_handle_interior_query(const can_frame_t *f)
{
    uint8_t ack[3];

    if (f->u8Dlc < 3U) {
        return;
    }
    if (f->au8Data[0] != CAN_CMD_INTERIOR_INFO) {
        return;
    }

    ack[0] = CAN_CMD_INTERIOR_INFO_ACK;   /* 0x8A */
    ack[1] = 0x00;
    ack[2] = 0x00;

    can_tx_bytes(CAN_NODE_ID, ack, sizeof(ack));
}

/* ================= 主循环处理 ================= */
void can_poll(void)
{
    while (s_rxQueue.tail != s_rxQueue.head) {
        can_frame_t f = s_rxQueue.buf[s_rxQueue.tail];
        s_rxQueue.tail = (uint16_t)((s_rxQueue.tail + 1U) % CAN_RX_QUEUE_SIZE);
        /* TODO: 根据 f.u32Id / f.au8Data 解析协议, 再调用 motor_/servo_ 接口 */
        can_handle_interior_query(&f);
    }
}

/* ================= 初始化 ================= */
void can_init(void)
{
    stc_can_init_t          stcCanInit;
    stc_can_filter_config_t stcFilter[2];
    stc_irq_signin_config_t irqCfg;

    /* 1. 使能 CAN 时钟 */
    LL_PERIPH_WE(LL_PERIPH_FCG);
    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_CAN, ENABLE);
    LL_PERIPH_WP(LL_PERIPH_FCG);

    /* 2. 初始化结构(先取默认值再覆盖) */
    (void)CAN_StructInit(&stcCanInit);

    /* 波特率: CAN 时钟 8MHz, 目标 1Mbps
     *  1TQ = Prescaler / fCAN = 1 / 8M = 125ns
     *  Bit = (TimeSeg1 + TimeSeg2) = (6+2) = 8TQ = 1us -> 1Mbps
     *  采样点 = 1 - TimeSeg2/(TimeSeg1+TimeSeg2) = 75%
     *  500k 时: TimeSeg1=12, TimeSeg2=4 */
    stcCanInit.stcBitCfg.u32Prescaler = 1U;
    stcCanInit.stcBitCfg.u32TimeSeg1  = 6U;//500k时改为12
    stcCanInit.stcBitCfg.u32TimeSeg2  = 2U;//500k时改为4
    stcCanInit.stcBitCfg.u32SJW       = 2U;

    stcCanInit.u8WorkMode       = CAN_WORK_MD_NORMAL;
    stcCanInit.u8SelfAck        = CAN_SELF_ACK_DISABLE;
    stcCanInit.u8RxWarnLimit    = 8U;
    stcCanInit.u8ErrorWarnLimit = 15U;

    /* 3. 验收过滤器: 精确接收广播帧与本机帧(扩展帧) */
    stcFilter[0].u32ID     = CAN_BROADCAST_ID;
    stcFilter[0].u32IDMask = 0U;
    stcFilter[0].u32IDType = CAN_ID_EXT;
    stcFilter[1].u32ID     = CAN_NODE_ID;
    stcFilter[1].u32IDMask = 0U;
    stcFilter[1].u32IDType = CAN_ID_EXT;
    stcCanInit.pstcFilter      = stcFilter;
    stcCanInit.u16FilterSelect = CAN_FILTER1 | CAN_FILTER2;

    (void)CAN_Init(CM_CAN, &stcCanInit);

    /* 4. 使能中断 */
    CAN_IntCmd(CM_CAN, CAN_INT_RX | CAN_INT_PTB_TX | CAN_INT_BUS_ERR, ENABLE);

    /* 5. 注册中断回调(走 LL 库回调分发, 不覆盖 IRQ008_Handler) */
    irqCfg.enIntSrc    = INT_SRC_CAN_INT;
    irqCfg.enIRQn      = INT008_IRQn;
    irqCfg.pfnCallback = can_irq_handler;
    (void)INTC_IrqSignIn(&irqCfg);

    NVIC_ClearPendingIRQ(INT008_IRQn);
    NVIC_SetPriority(INT008_IRQn, DDL_IRQ_PRIO_01);
    NVIC_EnableIRQ(INT008_IRQn);

    /* 6. 状态复位 */
    s_txRing.head  = s_txRing.tail  = 0U;
    s_rxQueue.head = s_rxQueue.tail = 0U;
    s_txBusy = 0U;
}
