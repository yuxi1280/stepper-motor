/* ============================================================================
 * EPS 执行设备 CAN 通信 (协议 v3, 见 Eclipse/app/CANv3.txt)
 * ----------------------------------------------------------------------------
 * 需求对应:
 *  1) 开机自动自检: 回零 -> 读已存凸点/补偿 -> 无数据才标定, 失败重试 1 次
 *  2) 一包 1~50 个乱序工位 -> 收到即回 0x83, 内部就近排序 -> 每个工位到位后
 *     自动执行固定舵机动作 -> 整包只回 1 条 0x84
 *  3) 运行中新任务回复已收到并排队(最多 3 个), 失败立即中断并上报
 *  4) PING 心跳, 连续丢失(3s) -> 0x89 上报 + 停机
 *  5) 补偿量每次更新自动上报 0x8A, 超上限只报不补偿
 *  6) 故障分级: L3 急停(步进失能 + 舵机释放), 0x05 清除后软件重新使能并自检
 * ========================================================================== */
#include "can.h"
#include "motor.h"
#include "lc.h"
#include "servo.h"
#include "check.h"
#include "uart.h"
#include <string.h>

/* ================= 内部帧结构 ================= */
typedef struct {
    uint32_t u32Id;
    uint8_t  u8Dlc;
    uint8_t  au8Data[8];
} can_frame_t;

/* 发送环形缓冲 */
static can_frame_t       s_txRing[CAN_TX_RING_SIZE];
static volatile uint16_t s_txHead;
static volatile uint16_t s_txTail;
static volatile uint8_t  s_txBusy;

/* 接收队列(中断填充, can_task 消费) */
static can_frame_t       s_rxQueue[CAN_RX_QUEUE_SIZE];
static volatile uint16_t s_rxHead;
static volatile uint16_t s_rxTail;

/* 重组缓冲 */
static uint8_t  s_rxMsg[CANP_MSG_MAX];
static uint8_t  s_rxCmd;
static uint8_t  s_rxSeq;
static uint8_t  s_rxExpect;
static uint8_t  s_rxGot;
static uint8_t  s_rxActive;

static volatile uint32_t s_lastRxMs;      /* 最近一次收到上位机帧的时刻 */
static volatile uint8_t  s_busOff;

/* 调试计数(0x8F 上报) */
static volatile uint8_t  s_dbgIsr;
static volatile uint8_t  s_dbgRx;
static volatile uint8_t  s_dbgAcc;
static volatile uint8_t  s_dbgDis;
static volatile uint8_t  s_dbgDlc;
static volatile uint32_t s_dbgLastId;
static uint32_t          s_dbgT0;

static void can_hw_config(void);
static void can_rx_process(void);
static void can_servo_delay(uint32_t ms);
static void can_report_home_fail(uint8_t fatal);

/* ================= 设备状态 ================= */
static uint8_t s_txSeq;                   /* 设备主动帧的序号 */
static uint8_t s_compSeq;                 /* 补偿量更新序号 */
static uint8_t s_state = CANP_ST_IDLE;
static uint8_t s_fault;
static uint8_t s_faultLv;
static uint8_t s_commLost;
static uint8_t s_bootDone;
static uint8_t s_lastFaultCode;
/* 零点有效性: 1=已回零且驱动状态/坐标校验通过。
 * 驱动过流/堵转保护会松轴并丢位置, 这时 zero_pos 可能无效, 必须置 0,
 * 拒绝任务直到重新回零成功(0x09/0x05 自检/0x0A 校准)。 */
static uint8_t s_zeroValid;

/* 主动校准状态 */
static uint8_t  s_calibRun;               /* 1=主动校准进行中 */
static uint32_t s_calibT0;                /* 校准周期计时起点(SysTick tick) */

/* ================= 任务队列 ================= */
typedef struct {
    uint8_t seq;                          /* 上位机本次任务的 SEQ, 0x84 回显 */
    uint8_t n;
    uint8_t slot[CANP_TASK_N_MAX];
} can_task_item_t;

static can_task_item_t s_queue[CANP_TASK_QUEUE_MAX];
static uint8_t s_qHead;
static uint8_t s_qTail;
static uint8_t s_qCnt;

/* ================= 执行状态机 ================= */
#define EXEC_IDLE       (0U)
#define EXEC_NEXT       (1U)
#define EXEC_WAIT       (2U)
#define EXEC_SETTLE     (3U)
#define EXEC_SERVO      (4U)
#define EXEC_DONE       (5U)

/* 回 0 位: s_parkMode=1 时 EXEC_WAIT 走"回0"分支 */
static uint8_t  s_parkMode;
static uint8_t  s_parkAfter;              /* 1=任务结束后自动回0(报0x84) 0=0x09命令(报0x8B) */
static uint8_t  s_parkSeq;                /* 0x09 的 SEQ, 0x8B 回显 */

static uint8_t  s_exec = EXEC_IDLE;
static uint8_t  s_curSlot[CANP_TASK_N_MAX];
static uint8_t  s_curN;
static uint8_t  s_curIdx;
static uint8_t  s_curSeq;
static uint8_t  s_curNo;                  /* 当前工位号 */
static uint8_t  s_doneCnt;
static uint8_t  s_result;
static uint8_t  s_lastSlot;               /* 上一次到达的工位, 就近排序起点 */
static uint32_t s_moveT0;
static uint32_t s_moveMinMs;              /* 本次运动理论最短时间 */
static uint32_t s_settleT0;
static int32_t  s_targetPulse;            /* 本次目标脉冲 */
static int32_t  s_lastPulse;              /* 上一次到达的脉冲(算行程用) */
static int32_t  s_zeroRef;                /* 回零位在 0x32 坐标系里的读数(真正的 0 点) */
static int32_t  s_moveStart;              /* 下发运动时的位置 */
static int32_t  s_moveWant;               /* 本次该走的脉冲数(带符号) */
static uint16_t s_doneAtMove;             /* 下发运动时的 9F 到位计数 */
static uint32_t s_posChkMs;               /* 上次位置核对时刻 */
static uint8_t  s_moveRetry;              /* 本次运动已重发次数(最多 2 次) */
static uint8_t  s_readRetry;              /* 读位置失败后的重试轮数 */
static uint32_t s_moveTmo;                /* 本次运动超时时间(重发后自动延长) */
static uint32_t s_moveLastAct;            /* 上次"下发/重发"指令的时刻 */
static uint32_t s_progMs;                 /* 执行器最近一次有实质进展的时刻(看门狗用) */

/* ================= 小工具 ================= */
static uint8_t can_next_seq(void)
{
    s_txSeq = (uint8_t)((s_txSeq + 1U) & CANP_SEQ_MASK);
    return s_txSeq;
}

static void can_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

static uint8_t can_id_is_mine(uint32_t id)
{
    id &= ~(1UL << 31U);            /* 去掉中断里打的"扩展帧"标记位, 只比 29 位 ID */
    if (id == CANP_ID_HOST_BCAST) {
        return 1U;
    }
    if (id == CANP_ID_HOST_TO(CANP_NODE_ADDR)) {
        return 1U;
    }
    return 0U;
}

/* 工位号 -> 皮带行程 mm */
static uint32_t can_slot_mm(uint8_t slot)
{
    return (uint32_t)POS_FIRST_MM + ((uint32_t)(slot - 1U) * (uint32_t)POS_STEP_MM);
}

/* 理论最短运动时间(ms): |位移| / 最高速 * 1.4 + 启动余量 */
static uint32_t can_move_min_ms(int32_t dist)
{
    uint32_t pulses = (uint32_t)((dist < 0) ? -dist : dist);
    uint32_t v      = ((uint32_t)POS_RPM * (uint32_t)MOTOR_PULSES_PER_REV) / 60U;
    uint32_t ms;

    if (v == 0U) {
        v = 32000U;
    }
    ms  = (pulses * 1000U) / v;
    ms  = (ms * CAN_MOVE_EXTRA_PCT) / 100U;
    ms += CAN_MOVE_START_MS;
    if (ms < CAN_MOVE_MIN_MS) {
        ms = CAN_MOVE_MIN_MS;
    }
    if (ms > CAN_MOVE_TIMEOUT_MS) {
        ms = CAN_MOVE_TIMEOUT_MS;
    }
    return ms;
}

/* 读位置(带重试 + 清缓冲)。
 * 电机上一动作结束会主动发 0x9F 到位帧, 舵机动作那 2 秒里没人收,
 * 直接读 0x32 会碰上残留字节 -> 解析错位 -> 读失败 ->
 * 不发电磁指令 -> 表现成"走到一半停住"。这里先等字节收完再清, 失败重试 3 次。
 */
static int32_t can_read_pos(void)
{
    int32_t cur;
    uint8_t i;

    for (i = 0U; i < 3U; i++) {
        DDL_DelayMS(10U);
        motor_flush_rx();
        cur = motor_read_input_pulses(MOTOR_RX_TIMEOUT_MS);
        if (cur != MOTOR_ERR_TIMEOUT) {
            return cur;
        }
    }
    return MOTOR_ERR_TIMEOUT;
}

/* 相对运动到"绝对目标位置"。
 * 不能用 motor_move_abs(): 它按"目标值的正负"决定方向, 从 1400mm 回 0 时
 * 目标 0 是正号 -> 方向反了。这里读一次当前位置, 用"目标-当前"决定方向。
 * 返回位移; 位置读不到时用上次已知位置兜底, 保证指令一定会发出去。
 */
/* 运动前确认驱动没锁在保护态:
 * 堵转/过流/掉电保护一旦触发, 后续 FD 指令会被静默丢弃(电机不动作),
 * 表现就是"前面几个工位都好的, 突然某一个工位不动 / 回 0 不动"。
 */
static void can_driver_ready(void)
{
    uint8_t st = motor_read_status(MOTOR_RX_TIMEOUT_MS);

    if ((st != 0xFFU) &&
        ((st & (MOTOR_ST_CGP_TF | MOTOR_ST_CGI_TF | MOTOR_ST_OAC_TF)) != 0U)) {
        motor_stop();
        DDL_DelayMS(20U);
        motor_clear_alarm();
        DDL_DelayMS(20U);
        motor_enable(1U);
        DDL_DelayMS(50U);
    }
}

/* 串口 + 驱动的"软复位"。
 * 标定/扫描过程中会高频查询位置, 驱动或 MCU 的应答通道可能卡住:
 * 现象是之后所有读位置都超时, 任务全部 PARAM=1 失败, 只有断电重启才恢复。
 * 这里重新初始化 USART1 并重新使能驱动, 不用断电就能恢复。
 */
uint8_t can_serial_recover(void)
{
    uint8_t i;

    /* 最多试 4 轮: 每轮重置串口 + 解锁驱动, 然后**实际读一次位置来验证**。
     * 标定这种连续长时间动作之后, 应答通道可能卡住,
     * 不验证的话会出现"自检说恢复好了, 实际读位置还是全失败"。
     * 返回 1=通道已恢复, 0=4 轮都读不到(驱动侧彻底失效, 需断电重启)。
     */
    for (i = 0U; i < 4U; i++) {
        uart_driver_init();                /* 重置 USART1(DeInit + 重新初始化) */
        DDL_DelayMS(50U);
        motor_flush_rx();
        motor_clear_alarm();               /* 解除可能锁存的保护 */
        DDL_DelayMS(20U);
        motor_enable(1U);                  /* 0x0E 之后必须重新使能 */
        DDL_DelayMS(100U);

        if (motor_read_input_pulses(60U) != MOTOR_ERR_TIMEOUT) {
            motor_flush_rx();
            return 1U;                     /* 读到了: 通道已恢复 */
        }
        DDL_DelayMS(100U);
    }
    motor_flush_rx();
    return 0U;
}

static int32_t can_move_to(int32_t target, uint16_t rpm, uint8_t acc)
{
    int32_t cur;
    int32_t d;

    can_driver_ready();                    /* 有保护先解除, 否则指令发下去也不动 */

    cur = can_read_pos();
    if (cur == MOTOR_ERR_TIMEOUT) {
        return INT32_MIN;
    }
    d = target - cur;
    s_moveStart = cur;                     /* 记下出发位置 */
    s_moveWant  = d;                       /* 记下"该走多少脉冲" */
    s_doneAtMove = g_motor_done_cnt;       /* 记下出发时的到位计数 */
    s_posChkMs   = SysTick_GetTick();
    s_moveRetry  = 0U;                     /* 本次运动还没重发过 */
    s_moveTmo    = CAN_MOVE_TIMEOUT_MS;
    s_moveLastAct = SysTick_GetTick();     /* 记下发指令的时刻 */
    s_progMs     = s_moveLastAct;          /* 有新动作下发 = 执行器有进展 */
    g_motor_last_reply = 0U; 
    if (d > 0) {
        motor_move(0, (uint32_t)d, rpm, acc, MOTOR_MODE_REL_CUR, 0U);
    } else if (d < 0) {
        motor_move(1, (uint32_t)(-d), rpm, acc, MOTOR_MODE_REL_CUR, 0U);
    } else {
        /* 已经在目标位置 */
    }
    return d;
}

/* 到位判定: 用"相对出发位置实际走了多少"和"该走多少"比。
 */
static uint8_t can_moved_ok(void)
{
    int32_t cur;
    int32_t moved;
    int32_t want;

    cur = can_read_pos();                  /* 带清缓冲+重试, 避免残留字节导致错位 */
    if (cur == MOTOR_ERR_TIMEOUT) {
        return 0U;                         /* 本次没读到, 下一轮再试 */
    }
    moved = cur - s_moveStart;
    if (moved < 0) {
        moved = -moved;
    }
    want = (s_moveWant < 0) ? -s_moveWant : s_moveWant;
    if ((moved > (want - CAN_POS_TOL_PULSE)) && (moved < (want + CAN_POS_TOL_PULSE))) {
        return 1U;
    }
    return 0U;
}

/* 舵机等待: 阻塞, 但同时解析舵机应答和电机应答。
 * 电机每完成一个动作都会主动发 4 字节 0x9F 到位帧, 这 2 秒里如果没人收,
 * 下一动作前的 motor_flush_rx() 就会把它清掉 -> 9F 计数永远是 0 ->
 * 到位判定只能靠不稳的读位置 -> 频繁 0x0A。
 */
/* 阻塞期间的"服务": 收 CAN 并派发(查询类照样应答) + 跑两个串口任务。
 * 带重入保护: 在 can_idle 内部又触发阻塞时不重复递归。
 */
void can_idle(void)
{
    static uint8_t s_busy = 0U;

    if (s_busy != 0U) {
        return;
    }
    s_busy = 1U;
    can_rx_process();                      /* 上位机的 PING/状态/调试 都能回 */
    motor_rx_task();                       /* 收驱动的 9F 到位帧 / ACK */
    servo_rx_task();                       /* 收舵机应答 */
    /* 注意: 这里不调 check_task() —— 回零/扫描的长阻塞循环会高频调 can_idle,
     * ADC 采样(尤其 ADC 超时时)会把 30s 超时拖成几分钟。电流检测只在
     * "推"的延时(can_servo_delay)和主循环空闲里做。 */
    s_busy = 0U;
}

static void can_servo_delay(uint32_t ms)
{
    while (ms-- > 0U) {
        can_idle();                        /* 舵机动作 2 秒里也保持通讯 */
        check_task();                      /* 推/回中是电流最大时刻, 这里保持过流检测 */
        DDL_DelayMS(1U);
    }
}

/* 一个完整舵机动作: 推出 -> 保持 -> 回中 -> 稳定(全程阻塞) */
static void can_servo_cycle(void)
{
    servo_move((uint16_t)CAN_SERVO_PUSH_PWM, (uint16_t)CAN_SERVO_PUSH_MS);
    can_servo_delay(CAN_SERVO_PUSH_MS + CAN_SERVO_GAP_MS);   /* 等推到位 */
    can_servo_delay(CAN_SERVO_HOLD_MS);                      /* 推到位保持 */

    servo_move((uint16_t)CAN_SERVO_MID_PWM, (uint16_t)CAN_SERVO_BACK_MS);
    can_servo_delay(CAN_SERVO_BACK_MS + CAN_SERVO_GAP_MS);   /* 等回中到位 */
    can_servo_delay(CAN_SERVO_SETTLE_MS);                    /* 回中后稳定 */
}

/* 阻塞走到指定脉冲并确认到位(1=到位), 用于自检/标定结束后回 0 位 */
static uint8_t can_move_wait(int32_t pulse, uint16_t rpm, uint8_t acc)
{
    uint32_t t0;
    uint32_t el;
    uint32_t chk = 0U;

    if (can_move_to(pulse, rpm, acc) == INT32_MIN) {
        return 0U;                            /* 位置读不到: 本次不动, 直接判失败 */
    }
    t0 = SysTick_GetTick();
    for (;;) {
        el = SysTick_GetTick() - t0;
        if (el >= CAN_MOVE_TIMEOUT_MS) {
            break;
        }
        can_idle();                           /* 收 9F + 服务 CAN(最长 15 秒不失联) */
        if (g_motor_done_cnt != s_doneAtMove) {
            s_lastPulse = pulse;
            return 1U;
        }
        if ((el - chk) >= 300U) {             /* 位置核对降频, 减少串口冲突 */
            chk = el;
            if (can_moved_ok() != 0U) {
                s_lastPulse = pulse;
                return 1U;
            }
        }
        DDL_DelayMS(5U);
    }
    motor_stop();
    return 0U;
}

/* 标定/校准结束回 0: 此时零点已知, 先用位置模式回坐标 0, 不再全行程速度模式撞限位。
 * 到位后确认限位开关仍闭合; 不闭合(零点有偏移)再从当前位置短距离慢速找限位。
 * 正常情况不再以速度模式全行程顶限位, 从根上减少"校准最后回零顶死"的次数。
 */
static uint8_t can_return_to_zero_safe(void)
{
    if (can_move_wait(s_zeroRef, POS_RPM, POS_ACC) != 0U) {
        if (motor_limit_is_active() != 0U) {
            return 1U;                        /* 位置到位且限位闭合: 零点确认 */
        }
        /* 位置到位但限位没闭合: 零点有偏移, 从当前位置慢速找限位(距离很短) */
    }
    return motor_limit_home();
}

/* 开始回 0 点位运动(不是回零, 只是走到坐标 0)
 * after_task = 1: 任务做完了自动回 0, 到位后报 0x84
 * after_task = 0: 0x09 命令触发, 到位后报 0x8B
 */
static void can_start_park(uint8_t after_task)
{
    int32_t d = INT32_MIN;
    uint8_t i;

    s_targetPulse = s_zeroRef;             /* 0 点 = 回零时记下的读数, 不假设它是 0 */
    for (i = 0U; i <= CAN_READ_RETRY_MAX; i++) {
        d = can_move_to(s_zeroRef, POS_RPM, POS_ACC);   /* 相对运动, 方向按差值算 */
        if (d != INT32_MIN) {
            break;
        }
        if (i == 0U) {
            can_serial_recover();
        } else {
            DDL_DelayMS(30U);
        }
    }

    if (d == INT32_MIN) {
        /* 多轮都读不到: 判回 0 失败, 不要傻等 15 秒 */
        s_parkMode = 0U;
        s_state    = CANP_ST_IDLE;
        if (after_task != 0U) {
            /* 位置读不到但没有运动指令发出: 用慢速撞限位重新定点,
             * 成功则视为已回 0(同时修复零点), 失败才报 PARK_FAIL。 */
            uint8_t bOk;

            s_state = CANP_ST_HOMING;
            if (motor_limit_home() != 0U) {
                DDL_DelayMS(150U);
                motor_flush_rx();
                bOk = lc_set_base();
                if (bOk == 0U) {
                    DDL_DelayMS(150U);
                    motor_flush_rx();
                    bOk = lc_set_base();
                }
                s_zeroRef   = (bOk != 0U) ? lc_get_base() : 0;   /* 刚清零, 读不到按 0 兜底 */
                s_lastPulse = s_zeroRef;
                s_lastSlot  = 0U;
                s_zeroValid = 1U;
                s_state     = CANP_ST_IDLE;
                s_result    = CANP_DONE_OK;
                s_exec      = EXEC_DONE;
            } else {
                s_zeroValid = 0U;
                can_report_home_fail(0U);
                s_state     = (s_faultLv >= CANP_LV_FATAL) ? CANP_ST_FAULT : CANP_ST_IDLE;
                s_result    = CANP_DONE_PARK_FAIL;
                s_exec      = EXEC_DONE;
            }
        } else {
            uint8_t r[3];
            r[0] = CANP_GZ_FAIL;
            r[1] = 0U;
            r[2] = 0U;
            can_tx_msg(CANP_R_GOZERO, s_parkSeq, r, 3U);
            s_exec = EXEC_IDLE;
        }
        return;
    }
    s_moveMinMs = can_move_min_ms(d);
    s_parkMode  = 1U;
    s_parkAfter = after_task;
    s_moveT0    = SysTick_GetTick();
    s_state     = CANP_ST_MOVING;
    s_exec      = EXEC_WAIT;
}

/* 工位间距离(单位: 工位), a=0 视为 1 号位 */
static uint8_t can_slot_dist(uint8_t a, uint8_t b)
{
    if (a == 0U) {
        a = 1U;
    }
    return (a > b) ? (uint8_t)(a - b) : (uint8_t)(b - a);
}

/* 就近排序: 从 from 出发, 每次挑剩余里最近的(贪心, N<=50) */
static void can_sort_slots(uint8_t *s, uint8_t n, uint8_t from)
{
    uint8_t i, j, best, tmp;

    for (i = 0U; i < n; i++) {
        best = i;
        for (j = (uint8_t)(i + 1U); j < n; j++) {
            if (can_slot_dist(from, s[j]) < can_slot_dist(from, s[best])) {
                best = j;
            }
        }
        if (best != i) {
            tmp = s[i];
            s[i] = s[best];
            s[best] = tmp;
        }
        from = s[i];
    }
}

/* ================= 发送 ================= */
static void can_tx_kick(void)
{
    stc_can_tx_frame_t tx;
    can_frame_t *f;

    if ((s_txBusy != 0U) || (s_txHead == s_txTail)) {
        return;
    }

    f = &s_txRing[s_txHead];
    memset(&tx, 0, sizeof(tx));
    tx.u32ID = f->u32Id & CAN_EXT_ID_MASK;
    tx.IDE   = 1U;
    tx.DLC   = f->u8Dlc;
    memcpy(tx.au8Data, f->au8Data, f->u8Dlc);

    s_txBusy = 1U;
    if (CAN_FillTxFrame(CM_CAN, CAN_TX_BUF_PTB, &tx) != LL_OK) {
        s_txBusy = 0U;
        return;                            /* PTB 忙, 等发送完成中断再取 */
    }
    CAN_StartTx(CM_CAN, CAN_TX_REQ_PTB);
    s_txHead = (uint16_t)((s_txHead + 1U) % CAN_TX_RING_SIZE);
}

static void can_tx_push(uint32_t u32Id, const uint8_t *p, uint8_t n)
{
    uint16_t next = (uint16_t)((s_txTail + 1U) % CAN_TX_RING_SIZE);

    if (next == s_txHead) {
        return;                            /* 缓冲满, 丢弃 */
    }
    s_txRing[s_txTail].u32Id = u32Id;
    s_txRing[s_txTail].u8Dlc = (n > 8U) ? 8U : n;
    memset(s_txRing[s_txTail].au8Data, 0, 8U);
    memcpy(s_txRing[s_txTail].au8Data, p, s_txRing[s_txTail].u8Dlc);
    s_txTail = next;
    can_tx_kick();
}

/* 发送一条消息: 自动按 5 字节/帧拆分 */
void can_tx_msg(uint8_t cmd, uint8_t seq, const uint8_t *p, uint8_t len)
{
    uint8_t frame[8];
    uint8_t off = 0U;
    uint8_t chunk;

    if (len > CANP_MSG_MAX) {
        return;
    }
    do {
        chunk = (uint8_t)(((uint8_t)(len - off) > CANP_FRAME_DATA) ?
                          CANP_FRAME_DATA : (uint8_t)(len - off));
        frame[0] = cmd;
        frame[1] = (uint8_t)((seq & CANP_SEQ_MASK) |
                             (((off + chunk) < len) ? CANP_SEQ_MORE : 0U));
        frame[2] = len;
        if (chunk != 0U && p != NULL) {
            memcpy(&frame[3], &p[off], chunk);
        }
        can_tx_push(CANP_ID_DEV(CANP_NODE_ADDR), frame, (uint8_t)(CANP_HDR_SIZE + chunk));
        off = (uint8_t)(off + chunk);
    } while (off < len);
}

/* ================= 上报 ================= */
void can_report_comp(void)
{
    const lc_check_t *c = lc_get_check();
    uint8_t d[8];
    int32_t a100;
    int32_t r100;
    uint8_t dec;

    a100 = (c->a * 3) / 2;                 /* 脉冲 -> 0.01mm */
    r100 = (c->max_r * 3) / 2;
    if (a100 > 32767)  { a100 = 32767; }
    if (a100 < -32768) { a100 = -32768; }
    if (r100 > 255)    { r100 = 255; }
    if (r100 < 0)      { r100 = 0; }

    dec = c->decision;
    if (dec > CANP_CMP_MEAS_BAD) {
        dec = CANP_CMP_MEAS_BAD;           /* 0xFE 未保存参考位 -> 测量无效 */
    }
    if (c->valid_cnt == 0U) {
        dec = CANP_CMP_MEAS_BAD;           /* 本次没做扫描, 不能声称"正常" */
    }

    d[0] = s_compSeq++;
    d[1] = dec;
    can_put_u16(&d[2], (uint16_t)(int16_t)a100);
    can_put_u16(&d[4], (uint16_t)(int16_t)c->b_ppm);
    d[6] = (uint8_t)r100;
    d[7] = c->valid_cnt;
    can_tx_msg(CANP_R_COMP, can_next_seq(), d, 8U);

    /* 超上限: 只报不补偿(3=零点异常 / 4=皮带明显异常) */
    if ((dec == CANP_CMP_ZERO_ALARM) || (dec == CANP_CMP_BELT_ALARM)) {
        int32_t b = (c->b_ppm < 0) ? -c->b_ppm : c->b_ppm;
        can_report_fault(CANP_F_BELT_DEV, CANP_LV_WARN, CANP_SRC_BELT,
                         (uint16_t)((b > 65535) ? 65535 : b));
    }
}

/* 0x8C SUB=1: 凸点位置记录完之后(标定完成 / 开机读 Flash)上报全部点位.
 * 格式: [SUB][N][pos1_H][pos1_L]...[posN_H][posN_L]
 *       pos = int16, 单位 0.1mm, 相对本次回零位, 负数补码
 */
void can_report_points(void)
{
    uint8_t d[2U + (LC_POINT_MAX * 2U)];
    uint8_t n = lc_point_count();
    uint8_t i;
    int32_t mm10;

    if (n > LC_POINT_MAX) {
        n = LC_POINT_MAX;
    }
    d[0] = CANP_PT_CAL;
    d[1] = n;
    for (i = 0U; i < n; i++) {
        if (lc_point_get((uint8_t)(i + 1U), &mm10) == 0U) {
            mm10 = 0;
        }
        if (mm10 > 32767)  { mm10 = 32767; }
        if (mm10 < -32768) { mm10 = -32768; }
        can_put_u16(&d[2U + (i * 2U)], (uint16_t)(int16_t)mm10);
    }
    can_tx_msg(CANP_R_POINT, can_next_seq(), d, (uint8_t)(2U + (n * 2U)));
}

void can_report_event(uint8_t ev, uint8_t res, uint8_t detail)
{
    uint8_t d[3];

    d[0] = ev;
    d[1] = res;
    d[2] = detail;
    can_tx_msg(CANP_R_EVENT, can_next_seq(), d, 3U);
}

void can_report_fault(uint8_t code, uint8_t lv, uint8_t src, uint16_t param)
{
    uint8_t d[5];

    if ((code == s_lastFaultCode) && (code != 0U)) {
        return;                            /* 同一故障不重复刷屏 */
    }
    s_lastFaultCode = code;

    d[0] = code;
    d[1] = lv;
    d[2] = src;
    can_put_u16(&d[3], param);
    can_tx_msg(CANP_R_FAULT, can_next_seq(), d, 5U);
}

/* 调试信息: [isr][rx][acc][dis][RTIE][RTIF][RCTRL][ACFEN][CFG_STAT][TECNT][RECNT][lastID x4] */
void can_report_debug(void)
{
    uint8_t d[32];
    uint8_t mst;
    uint8_t mhs;

    mst = motor_read_status(10U);       /* 0x3A 电机状态(短超时, 运动中查询不长时间阻塞) */
    mhs = motor_read_home_status(10U);  /* 0x3B 回零状态 */

    d[0]  = s_dbgIsr;
    d[1]  = s_dbgRx;
    d[2]  = s_dbgAcc;
    d[3]  = s_dbgDis;
    d[4]  = CM_CAN->RTIE;
    d[5]  = CM_CAN->RTIF;
    d[6]  = CM_CAN->RCTRL;
    d[7]  = CM_CAN->ACFEN;
    d[8]  = CM_CAN->CFG_STAT;
    d[9]  = CM_CAN->TECNT;
    d[10] = CM_CAN->RECNT;
    d[11] = (uint8_t)(s_dbgLastId >> 24);
    d[12] = (uint8_t)(s_dbgLastId >> 16);
    d[13] = (uint8_t)(s_dbgLastId >> 8);
    d[14] = (uint8_t)(s_dbgLastId);
    d[15] = s_dbgDlc;
    d[16] = g_servo_st.last_reply;          /* 0=无应答 1=#OK! 2=位置回读 */
    d[17] = (uint8_t)(g_servo_st.position >> 8);
    d[18] = (uint8_t)(g_servo_st.position);
    d[19] = (uint8_t)g_servo_ok_cnt;        /* 收到的 #OK! 次数(低 8 位) */

    /* 步进驱动诊断 + 上次复位原因 */
    d[20] = g_motor_last_reply;             /* 最近返回码: 02=ACK 9F=到位 E2/EE=失步/参数错 */
    d[21] = (uint8_t)g_motor_err_cnt;       /* 收到 E2/EE 次数 */
    d[22] = (uint8_t)g_motor_done_cnt;      /* 收到 9F 到位次数 */
    d[23] = (uint8_t)((uint32_t)s_moveWant >> 24);
    d[24] = (uint8_t)((uint32_t)s_moveWant >> 16);
    d[25] = (uint8_t)((uint32_t)s_moveWant >> 8);
    d[26] = (uint8_t)((uint32_t)s_moveWant);/* 最近一次下发的位移(脉冲) */
    d[27] = (uint8_t)(CM_RMU->RSTF0);       /* 上次复位原因(低字节) */
    d[28] = (uint8_t)(CM_RMU->RSTF0 >> 8);
    d[29] = mst;                            /* 0x3A 电机状态: bit3堵转保护 bit2堵转 bit0使能 */
    d[30] = mhs;                            /* 0x3B 回零状态: bit5过流 bit4过热 bit3回零失败 */
    d[31] = s_zeroValid;                    /* 1=零点有效 0=需重新回零 */

    can_tx_msg(CANP_R_DEBUG, can_next_seq(), d, 32U);
}

/* ================= 急停 / 故障 ================= */
void can_estop(void)
{
    motor_stop();
    motor_enable(0);                       /* 步进下电(失能松轴) */
    servo_release();                       /* 舵机释放扭力 */
    app_abort_calib();                     /* 若在校准: 中止校准动作 */
    s_zeroValid = 0U;                      /* 步进失能后零点不再可信 */

    if (s_exec != EXEC_IDLE) {
        uint8_t d[3];
        d[0] = CANP_DONE_FAULT;
        d[1] = s_doneCnt;
        d[2] = s_curNo;
        can_tx_msg(CANP_R_TASK_DONE, s_curSeq, d, 3U);
    }
    s_exec = EXEC_IDLE;
    s_parkMode = 0U;
    s_qCnt = 0U;
    s_qHead = 0U;
    s_qTail = 0U;
    s_lastSlot = 0U;
}

void can_raise_fault(uint8_t code, uint8_t lv, uint8_t src, uint16_t param)
{
    s_fault   = code;
    s_faultLv = lv;

    /* 驱动保护/回零/位置类故障 -> 零点不可信, 必须重新回零 */
    if ((lv >= CANP_LV_FATAL) ||
        (code == CANP_F_MOTOR_PWR) || (code == CANP_F_MOTOR_STALL) ||
        (code == CANP_F_HOME_TIMEOUT) || (code == CANP_F_POSITION)) {
        s_zeroValid = 0U;
    }

    can_report_fault(code, lv, src, param);

    if (lv >= CANP_LV_FATAL) {
        can_estop();
        s_state = CANP_ST_FAULT;
    } else if (lv >= CANP_LV_ERR) {
        /* L2: 停当前任务, 设备仍可接单 */
        if (s_exec != EXEC_IDLE) {
            s_result = CANP_DONE_FAULT;
            s_exec   = EXEC_DONE;
            s_state  = CANP_ST_FAULT;
        }
    } else {
        /* L1: 只报 */
    }
}

uint8_t can_fault_code(void) { return s_fault; }
uint8_t can_is_fault(void)   { return (s_fault != 0U) ? 1U : 0U; }

/* 回零失败上报: 把驱动保护位/失败原因映射成 0x89。
 * PARAM = [失败原因(高字节)][驱动状态位(低字节)], 便于上位机区分
 * 限位超时 / 驱动过流堵转松轴 / 未停稳 / 坐标校验失败。
 */
static void can_report_home_fail(uint8_t fatal)
{
    uint8_t  e = motor_home_last_error();
    uint16_t f = motor_home_last_faults();
    uint16_t param = (uint16_t)(f | ((uint16_t)e << 8));

    switch (e) {
        case MOTOR_HOME_ERR_ABORT:
            return;                        /* 被 0x04/急停中止, 不报故障 */
        case MOTOR_HOME_ERR_TIMEOUT:
            can_raise_fault(CANP_F_HOME_TIMEOUT, (fatal != 0U) ? CANP_LV_FATAL : CANP_LV_ERR,
                            CANP_SRC_MOTOR, param);
            return;
        case MOTOR_HOME_ERR_DRIVER:
            if ((f & (MOTOR_FLT_OVERCURR | MOTOR_FLT_OVERTEMP | MOTOR_FLT_STALL)) != 0U) {
                can_raise_fault(CANP_F_MOTOR_STALL, (fatal != 0U) ? CANP_LV_FATAL : CANP_LV_ERR,
                                CANP_SRC_MOTOR, param);
            } else {
                can_raise_fault(CANP_F_MOTOR_PWR, (fatal != 0U) ? CANP_LV_FATAL : CANP_LV_ERR,
                                CANP_SRC_MOTOR, param);
            }
            return;
        case MOTOR_HOME_ERR_NOSTOP:
        case MOTOR_HOME_ERR_POS:
        default:
            can_raise_fault(CANP_F_POSITION, (fatal != 0U) ? CANP_LV_FATAL : CANP_LV_ERR,
                            CANP_SRC_MOTOR, param);
            return;
    }
}

/* 回零失败原因 -> 协议故障码(0x87/0x8D 的 RESULT 字段用) */
static uint8_t can_home_fault_code(void)
{
    uint8_t  e = motor_home_last_error();
    uint16_t f = motor_home_last_faults();

    if (e == MOTOR_HOME_ERR_DRIVER) {
        if ((f & (MOTOR_FLT_OVERCURR | MOTOR_FLT_OVERTEMP | MOTOR_FLT_STALL)) != 0U) {
            return CANP_F_MOTOR_STALL;
        }
        return CANP_F_MOTOR_PWR;
    }
    if ((e == MOTOR_HOME_ERR_NOSTOP) || (e == MOTOR_HOME_ERR_POS)) {
        return CANP_F_POSITION;
    }
    return CANP_F_HOME_TIMEOUT;
}

/* ================= 接收 ================= */
static void can_dispatch(uint8_t cmd, uint8_t seq, const uint8_t *d, uint8_t len);

static void can_msg_input(const can_frame_t *f)
{
    uint8_t cmd;
    uint8_t seq;
    uint8_t len;
    uint8_t n;
    uint8_t more;

    if (f->u8Dlc < CANP_HDR_SIZE) {
        return;
    }
    cmd  = f->au8Data[0];
    seq  = f->au8Data[1];
    len  = f->au8Data[2];
    more = (uint8_t)(seq & CANP_SEQ_MORE);
    seq  = (uint8_t)(seq & CANP_SEQ_MASK);

    if (len > CANP_MSG_MAX) {
        s_rxActive = 0U;
        return;
    }
    n = (uint8_t)(f->u8Dlc - CANP_HDR_SIZE);
    if (n > CANP_FRAME_DATA) {
        n = CANP_FRAME_DATA;
    }

    /* 不是同一条消息的续帧 -> 重新开始重组(丢帧保护) */
    if ((s_rxActive == 0U) || (cmd != s_rxCmd) || (seq != s_rxSeq)) {
        s_rxActive = 1U;
        s_rxCmd    = cmd;
        s_rxSeq    = seq;
        s_rxExpect = len;
        s_rxGot    = 0U;
    }

    /* 以 LEN 为准, 不越界也不多收 */
    if (s_rxGot < s_rxExpect) {
        uint8_t remain = (uint8_t)(s_rxExpect - s_rxGot);
        if (n > remain) {
            n = remain;
        }
    } else {
        n = 0U;
    }

    if (((uint16_t)s_rxGot + (uint16_t)n) > CANP_MSG_MAX) {
        s_rxActive = 0U;
        return;
    }
    if (n != 0U) {
        memcpy(&s_rxMsg[s_rxGot], &f->au8Data[CANP_HDR_SIZE], n);
        s_rxGot = (uint8_t)(s_rxGot + n);
    }

    if (more == 0U) {
        s_rxActive = 0U;
        if (s_rxGot >= s_rxExpect) {
            can_dispatch(s_rxCmd, s_rxSeq, s_rxMsg, s_rxExpect);
        }
    }
}

static void can_rx_process(void)
{
    can_frame_t f;

    while (s_rxTail != s_rxHead) {
        f = s_rxQueue[s_rxTail];
        s_rxTail = (uint16_t)((s_rxTail + 1U) % CAN_RX_QUEUE_SIZE);
        if (can_id_is_mine(f.u32Id) == 0U) {
            continue;
        }
        s_dbgAcc++;
        s_lastRxMs = SysTick_GetTick();
        can_msg_input(&f);
    }
}

/* ================= 中断 ================= */
static void can_irq_handler(void)
{
    s_dbgIsr++;

    if (CAN_GetStatus(CM_CAN, CAN_FLAG_RX) == SET) {
        stc_can_rx_frame_t rx;
        while (CAN_GetRxFrame(CM_CAN, &rx) == LL_OK) {
            uint32_t id = rx.u32ID | ((rx.IDE != 0U) ? (1UL << 31U) : 0UL);
            uint16_t next = (uint16_t)((s_rxHead + 1U) % CAN_RX_QUEUE_SIZE);

            s_dbgRx++;
            s_dbgLastId = rx.u32ID;
            s_dbgDlc    = rx.DLC;

            /* 心跳计时放在中断里, 且不受队列满影响: 标定/长动作阻塞时也准 */
            if (can_id_is_mine(id) != 0U) {
                s_lastRxMs = SysTick_GetTick();
            }
            if (next != s_rxTail) {
                s_rxQueue[s_rxHead].u32Id = id;
                s_rxQueue[s_rxHead].u8Dlc = (rx.DLC > 8U) ? 8U : rx.DLC;
                memcpy(s_rxQueue[s_rxHead].au8Data, rx.au8Data, s_rxQueue[s_rxHead].u8Dlc);
                s_rxHead = next;
            }
        }
        CAN_ClearStatus(CM_CAN, CAN_FLAG_RX);
    }

    if (CAN_GetStatus(CM_CAN, CAN_FLAG_PTB_TX) == SET) {
        CAN_ClearStatus(CM_CAN, CAN_FLAG_PTB_TX);
        s_txBusy = 0U;
        can_tx_kick();
    }

    if (CAN_GetStatus(CM_CAN, CAN_FLAG_BUS_ERR) == SET) {
        s_busOff = 1U;
    }

    CAN_ClearStatus(CM_CAN, CAN_FLAG_RX_BUF_OVF | CAN_FLAG_BUS_ERR |
                            CAN_FLAG_ARBITR_LOST | CAN_FLAG_ERR_INT |
                            CAN_FLAG_RX_BUF_FULL | CAN_FLAG_RX_BUF_WARN);
}

/* ================= 命令处理 ================= */
static void can_reply_info(uint8_t seq)
{
    uint8_t d[8];

    d[0] = CANP_VER;
    d[1] = CANP_FW_MAJOR;
    d[2] = CANP_FW_MINOR;
    d[3] = CANP_SLOT_TOTAL;
    d[4] = s_state;
    d[5] = s_fault;
    d[6] = CANP_NODE_ADDR;
    d[7] = CANP_CAP_ALL;
    can_tx_msg(CANP_R_INFO, seq, d, 8U);
}

static void can_reply_status(uint8_t seq)
{
    uint8_t  d[8];
    int32_t  pulse = motor_read_input_pulses(MOTOR_RX_TIMEOUT_MS);
    int32_t  mm100 = 0;

    if (pulse != MOTOR_ERR_TIMEOUT) {
        mm100 = motor_pulse_to_mm_x100(pulse - s_zeroRef);   /* 0.01mm, 相对 0 点 */
    }
    if (mm100 < 0) {
        mm100 = 0;
    }
    if (mm100 > 65535) {
        mm100 = 65535;
    }

    d[0] = s_state;
    d[1] = s_fault;
    d[2] = CANP_SLOT_TOTAL;
    d[3] = s_curNo;
    can_put_u16(&d[4], (uint16_t)(mm100 / 10));   /* 0.1mm */
    d[6] = s_qCnt;
    d[7] = s_doneCnt;
    can_tx_msg(CANP_R_STATUS, seq, d, 8U);
}

static void can_handle_task(uint8_t seq, const uint8_t *d, uint8_t len)
{
    uint8_t ack[3];
    uint8_t n;
    uint8_t i;

    ack[0] = CANP_ACK_OK;
    ack[1] = 0U;
    ack[2] = 0U;

    if (len < 1U) {
        ack[0] = CANP_ACK_PARAM;
        can_tx_msg(CANP_R_TASK_ACK, seq, ack, 3U);
        return;
    }
    n = d[0];
    if ((n == 0U) || (n > CANP_TASK_N_MAX) || (len < (uint8_t)(n + 1U))) {
        ack[0] = CANP_ACK_PARAM;
        can_tx_msg(CANP_R_TASK_ACK, seq, ack, 3U);
        return;
    }
    for (i = 0U; i < n; i++) {
        if ((d[1U + i] < CANP_SLOT_MIN) || (d[1U + i] > CANP_SLOT_MAX)) {
            ack[0] = CANP_ACK_PARAM;
            can_tx_msg(CANP_R_TASK_ACK, seq, ack, 3U);
            return;
        }
    }
    if (s_faultLv >= CANP_LV_FATAL) {
        ack[0] = CANP_ACK_PARAM;                   /* 故障未清除, 拒收 */
        can_tx_msg(CANP_R_TASK_ACK, seq, ack, 3U);
        return;
    }
    if (s_zeroValid == 0U) {
        /* 零点不可信(驱动保护/回零失败): 拒收, 要求先 0x09 重新回零 */
        ack[0] = CANP_ACK_PARAM;
        ack[1] = n;
        ack[2] = 0U;
        can_report_fault(CANP_F_HOME_TIMEOUT, CANP_LV_ERR, CANP_SRC_MOTOR, 0xFFFFU);
        can_tx_msg(CANP_R_TASK_ACK, seq, ack, 3U);
        return;
    }
    if (s_qCnt >= CANP_TASK_QUEUE_MAX) {
        ack[0] = CANP_ACK_QUEUE_FULL;
        ack[1] = n;
        can_tx_msg(CANP_R_TASK_ACK, seq, ack, 3U);
        return;
    }

    s_queue[s_qTail].seq = seq;
    s_queue[s_qTail].n   = n;
    memcpy(s_queue[s_qTail].slot, &d[1], n);
    s_qTail = (uint8_t)((s_qTail + 1U) % CANP_TASK_QUEUE_MAX);
    s_qCnt++;

    ack[1] = n;
    ack[2] = (s_exec != EXEC_IDLE) ? s_qCnt : 0U;  /* 0=立即执行 */
    can_tx_msg(CANP_R_TASK_ACK, seq, ack, 3U);

    /* 任务已被接受, 清掉 L1/L2 旧故障记录 */
    if (s_faultLv <= CANP_LV_ERR) {
        s_fault      = 0U;
        s_faultLv    = 0U;
        s_lastFaultCode = 0U;
        if (s_state == CANP_ST_FAULT) {
            s_state = CANP_ST_IDLE;
        }
    }
}

static void can_handle_stop(uint8_t seq)
{
    uint8_t d[3];
    uint8_t rseq;

    motor_stop();
    servo_stop();
    app_abort_calib();                     /* 若在校准: 中止校准动作 */

    d[0] = CANP_DONE_STOPPED;
    d[1] = s_doneCnt;
    d[2] = s_curNo;
    rseq = (s_exec != EXEC_IDLE) ? s_curSeq : seq;   /* 有任务跑就回显任务序号 */
    can_tx_msg(CANP_R_TASK_DONE, rseq, d, 3U);

    s_exec    = EXEC_IDLE;
    s_parkMode = 0U;
    s_qCnt    = 0U;
    s_qHead   = 0U;
    s_qTail   = 0U;
    s_state   = (s_faultLv >= CANP_LV_FATAL) ? CANP_ST_FAULT : CANP_ST_IDLE;
}

/* 0x09 去 0 点位: 采用开机自检同款"慢速撞限位回零", 成功后重新标记零点。
 * 立即应答"已接受", 回零完成后回"已到达"/"失败"(最长约 30s, 期间照常应答 CAN)。
 */
static void can_handle_goto_zero(uint8_t seq)
{
    uint8_t r[3];
    uint8_t ok;
    uint8_t baseOk;

    r[0] = CANP_GZ_ACCEPT;
    r[1] = 0U;
    r[2] = 0U;

    if (s_faultLv >= CANP_LV_FATAL) {
        r[0] = CANP_GZ_FAIL;                    /* 故障态, 先发 0x05 清除 */
        can_tx_msg(CANP_R_GOZERO, seq, r, 3U);
        return;
    }
    if (s_exec != EXEC_IDLE) {
        r[0] = CANP_GZ_BUSY;                    /* 正在跑任务, 拒绝 */
        can_tx_msg(CANP_R_GOZERO, seq, r, 3U);
        return;
    }

    can_tx_msg(CANP_R_GOZERO, seq, r, 3U);      /* 已接受 */

    /* 慢速回零(与开机自检相同): 撞限位 -> 0x0A 坐标清零 -> 重记 0 点 */
    app_abort_clear();                          /* 清掉上一次急停残留的中止标志 */
    s_state = CANP_ST_HOMING;
    ok = motor_limit_home();
    s_state = (s_faultLv >= CANP_LV_FATAL) ? CANP_ST_FAULT : CANP_ST_IDLE;

    if (ok == 0U) {
        if (motor_home_last_error() != MOTOR_HOME_ERR_ABORT) {
            s_zeroValid = 0U;                   /* 回零没完成: 零点不可信 */
        }                                       /* 被 0x04 中止: 旧零点坐标仍有效 */
        can_report_home_fail(0U);
        r[0] = CANP_GZ_FAIL;                    /* 限位未触发/超时/驱动保护/被 0x04 中止 */
        can_tx_msg(CANP_R_GOZERO, seq, r, 3U);
        return;
    }

    /* 0x0A 已把驱动坐标清零, 重新读一次作为新的 0 点 */
    DDL_DelayMS(200U);                          /* 等驱动稳定, 位置读数才可靠 */
    motor_flush_rx();
    baseOk = lc_set_base();
    if (baseOk == 0U) {
        DDL_DelayMS(200U);
        motor_flush_rx();
        baseOk = lc_set_base();
    }
    if (baseOk == 0U) {
        (void)can_serial_recover();
        motor_flush_rx();
        baseOk = lc_set_base();
    }
    s_zeroRef   = (baseOk != 0U) ? lc_get_base() : 0;  /* 刚清零, 读不到按 0 兜底 */
    s_lastPulse = s_zeroRef;
    s_lastSlot  = 0U;
    s_zeroValid = 1U;                                  /* 回零+状态+坐标校验都通过 */

    r[0] = CANP_GZ_ARRIVED;
    can_tx_msg(CANP_R_GOZERO, seq, r, 3U);
}

static void can_boot_run(void);

static void can_handle_fault_clear(uint8_t seq)
{
    uint8_t r = CANP_CLR_OK;
    uint8_t oldFault = s_fault;

    check_clear_overcurrent();
    motor_clear_alarm();
    motor_enable(1);                        /* 软件重新使能, 本机无电源开关 */
    servo_restore();

    /* 强制复位执行状态机: 万一卡在 EXEC_WAIT/SETTLE/SERVO, 先回到 IDLE,
     * 否则清完故障后状态仍然"忙", 后面发什么都进不来。
     */
    app_abort_calib();                     /* 若在校准: 中止校准动作 */
    motor_stop();
    DDL_DelayMS(20U);
    s_exec     = EXEC_IDLE;
    s_parkMode = 0U;
    s_qCnt     = 0U;
    s_qHead    = 0U;
    s_qTail    = 0U;

    s_fault         = 0U;
    s_faultLv       = 0U;
    s_commLost      = 0U;
    s_lastFaultCode = 0U;
    s_state         = CANP_ST_IDLE;
    s_lastRxMs      = SysTick_GetTick();
    s_progMs        = s_lastRxMs;

    can_tx_msg(CANP_R_CLR_ACK, seq, &r, 1U);
    can_serial_recover();

    if ((lc_ref_valid() == 0U) || (oldFault == CANP_F_SELFTEST) ||
        (oldFault == CANP_F_HOME_TIMEOUT) || (oldFault == CANP_F_MOTOR_PWR) ||
        (oldFault == CANP_F_MOTOR_STALL) || (oldFault == CANP_F_POSITION) ||
        (s_zeroValid == 0U)) {             /* 零点无效也走一次完整自检(含回零) */
        s_bootDone  = 0U;
        s_zeroValid = 0U;                  /* 重新自检会重新回零, 成功后才恢复 */
    }
}

static void can_handle_recal(uint8_t seq)
{
    uint8_t d[2];
    lc_result_t r;

    if (s_exec != EXEC_IDLE) {
        d[0] = CANP_RECAL_FAIL;
        d[1] = CANP_F_PARAM;
        can_tx_msg(CANP_R_RECAL, seq, d, 2U);
        return;
    }

    d[0] = CANP_RECAL_ACCEPT;
    d[1] = 0U;
    can_tx_msg(CANP_R_RECAL, seq, d, 2U);

    /* 换皮带: 清参考位/补偿 -> 回零 -> 重新记录凸点 -> 存 Flash */
    s_state = CANP_ST_CALIB;
    app_abort_clear();                     /* 清除上一次急停残留的中止标志 */
    (void)lc_ref_clear();
    r = lc_scan_run(1U);
    if (r != LC_RESULT_ALL) {
        uint16_t mf = motor_read_faults();

        (void)can_move_wait(s_zeroRef, LC_RETURN_RPM, LC_SCAN_ACC);   /* 失败也先回 0 */
        s_state = CANP_ST_IDLE;
        if ((r == LC_RESULT_HOME_FAIL) || (r == LC_RESULT_POS_ERR)) {
            s_zeroValid = 0U;              /* 回零/坐标失效, 需要重新回零 */
        }
        if ((mf & MOTOR_FLT_ZERO_BAD) != 0U) {
            s_zeroValid = 0U;
            can_report_fault(((mf & (MOTOR_FLT_STALL | MOTOR_FLT_OVERCURR |
                                     MOTOR_FLT_OVERTEMP)) != 0U)
                             ? CANP_F_MOTOR_STALL : CANP_F_MOTOR_PWR,
                             CANP_LV_ERR, CANP_SRC_MOTOR, mf);
        }
        d[0] = CANP_RECAL_FAIL;
        d[1] = CANP_F_SELFTEST;
        can_tx_msg(CANP_R_RECAL, seq, d, 2U);
        can_raise_fault(CANP_F_SELFTEST, CANP_LV_ERR, CANP_SRC_BELT, (uint16_t)r);
        return;
    }
    (void)lc_ref_save();
    s_zeroRef = lc_get_base();             /* 扫描前已重新回零, 刷新 0 点坐标 */
    /* 标定完回 0 位: 零点已知, 先用位置模式回 0, 不再全行程撞限位 */
    s_state = CANP_ST_HOMING;
    if (can_return_to_zero_safe() != 0U) {
        (void)lc_set_base();
        s_zeroRef = lc_get_base();         /* 重新回零过, 刷新 0 点坐标 */
    } else {
        /* 回零失败: 零点基准失效, 上报驱动状态并停机(0x05 重新自检) */
        can_report_event(CANP_EV_SELF_ABORT, CANP_F_HOME_TIMEOUT, CANP_PH_HOME);
        can_report_home_fail(1U);
        d[0] = CANP_RECAL_FAIL;
        d[1] = can_home_fault_code();
        can_tx_msg(CANP_R_RECAL, seq, d, 2U);
        return;
    }
    s_lastPulse = s_zeroRef;
    s_lastSlot = 0U;
    s_zeroValid = 1U;
    s_state    = CANP_ST_IDLE;

    can_serial_recover();                  /* 标定后软复位串口/驱动, 否则读位置会全失败 */

    d[0] = CANP_RECAL_DONE;
    d[1] = 0U;
    can_tx_msg(CANP_R_RECAL, seq, d, 2U);
    can_report_comp();
    can_report_points();                   /* 重标定完: 上报新的凸点位置 */
}

/* ================= 主动校准 =================
 * 与 0x07 重标定(换皮带, 覆盖参考位)不同: 校准只"慢速扫描对比参考位",
 * 算出来的补偿量写 Flash 持久化, 参考位本身不覆盖, 扫描到的凸点只用于对比上报。
 * 流程: 慢速回零 -> 慢速扫描对比 -> 补偿落盘 -> 慢速回零 -> 上报 0x8C/0x8A。
 * 校准期间操控类指令回"校准中"; 急停/停止置 app_abort, 阻塞循环立刻退出,
 * 退出后恢复 Flash 里的旧补偿继续运行。
 */
static void can_calib_run(uint8_t seq, uint8_t from_cmd)
{
    lc_result_t r;
    uint8_t d[2];

    /* 1) 声明校准中 + 应答 */
    s_calibRun = 1U;
    s_state    = CANP_ST_CALIB;
    app_abort_clear();
    can_report_event(CANP_EV_CAL_START, 0U, 0U);
    if (from_cmd != 0U) {
        d[0] = CANP_CAL_ACCEPT;
        d[1] = 0U;
        can_tx_msg(CANP_R_CALIB, seq, d, 2U);
    }

    /* 2) 慢速回零(零点也一起重新校准) */
    s_state = CANP_ST_HOMING;
    if (motor_limit_home() == 0U) {
        uint8_t aborted = app_abort_active();
        (void)lc_ref_reload();             /* 恢复旧补偿 */
        s_state    = CANP_ST_IDLE;
        s_calibRun = 0U;
        if (aborted == 0U) {               /* 非急停: 真回零失败才上报 */
            can_report_event(CANP_EV_SELF_ABORT, CANP_F_HOME_TIMEOUT, CANP_PH_HOME);
            can_report_home_fail(0U);
        }
        if (from_cmd != 0U) {
            d[0] = CANP_CAL_FAIL;
            d[1] = (aborted != 0U) ? 0U : can_home_fault_code();
            can_tx_msg(CANP_R_CALIB, seq, d, 2U);
        }
        s_calibT0 = SysTick_GetTick();
        return;
    }
    DDL_DelayMS(200U);
    motor_flush_rx();
    if (lc_set_base() != 0U) {
        s_zeroRef = lc_get_base();         /* 刷新 0 点坐标 */
    }
    s_zeroValid = 1U;                      /* 回零+状态+坐标校验通过 */

    /* 3) 慢速扫描 + 与参考位对比(内部 lc_check_ref + lc_update_comp) */
    s_state = CANP_ST_CALIB;
    r = lc_scan_run(0U);
    if (r != LC_RESULT_ALL) {
        uint8_t  aborted = app_abort_active();
        uint16_t mf      = motor_read_faults();

        (void)lc_ref_reload();             /* 扫描失败/被中止: 恢复旧补偿 */
        s_state    = CANP_ST_IDLE;
        s_calibRun = 0U;
        if (aborted == 0U) {               /* 非急停: 真扫描失败才上报 */
            can_report_event(CANP_EV_SELF_ABORT, CANP_F_SELFTEST, (uint8_t)r);
            if ((mf & MOTOR_FLT_ZERO_BAD) != 0U) {
                s_zeroValid = 0U;          /* 驱动保护/松轴: 零点可能已失效 */
                can_report_fault(((mf & (MOTOR_FLT_STALL | MOTOR_FLT_OVERCURR |
                                         MOTOR_FLT_OVERTEMP)) != 0U)
                                 ? CANP_F_MOTOR_STALL : CANP_F_MOTOR_PWR,
                                 CANP_LV_ERR, CANP_SRC_MOTOR, mf);
            }
        }
        if (from_cmd != 0U) {
            d[0] = CANP_CAL_FAIL;
            d[1] = (aborted != 0U) ? 0U : CANP_F_SELFTEST;
            can_tx_msg(CANP_R_CALIB, seq, d, 2U);
        }
        s_calibT0 = SysTick_GetTick();
        return;
    }

    /* 4) 补偿量落盘(参考位不变) */
    if (lc_comp_persist() == 0U) {
        can_report_fault(CANP_F_SELFTEST, CANP_LV_WARN, CANP_SRC_BELT, 1U);
    }

    /* 5) 回 0(校准结束): 零点已知, 用位置模式回 0, 避免全行程撞限位 */
    s_state = CANP_ST_HOMING;
    if (can_return_to_zero_safe() != 0U) {
        (void)lc_set_base();
        s_zeroRef = lc_get_base();
    } else {
        /* 回零失败: 零点基准失效。不锁 L3(避免卡死), 用 s_zeroValid=0 拒绝任务,
         * 上位机发 0x09 重新回零即可恢复。 */
        s_calibRun  = 0U;
        s_zeroValid = 0U;
        can_report_event(CANP_EV_SELF_ABORT, CANP_F_HOME_TIMEOUT, CANP_PH_HOME);
        can_report_home_fail(0U);
        /* 关键: 必须退出 HOMING 状态, 否则 0x09/0x0A/0x03 全被当成"忙"拒绝 */
        s_state = (s_faultLv >= CANP_LV_FATAL) ? CANP_ST_FAULT : CANP_ST_IDLE;
        if (from_cmd != 0U) {
            d[0] = CANP_CAL_FAIL;
            d[1] = can_home_fault_code();
            can_tx_msg(CANP_R_CALIB, seq, d, 2U);
        }
        s_calibT0 = SysTick_GetTick();
        return;
    }
    s_lastPulse = s_zeroRef;
    s_lastSlot  = 0U;
    s_zeroValid = 1U;

    /* 6) 串口/驱动恢复验证: 失败说明驱动侧彻底失效, 明确上报并判校准失败 */
    if (can_serial_recover() == 0U) {
        s_state    = CANP_ST_IDLE;
        s_calibRun = 0U;
        can_report_event(CANP_EV_SELF_ABORT, CANP_F_POSITION, (uint8_t)LC_RESULT_POS_ERR);
        can_raise_fault(CANP_F_POSITION, CANP_LV_ERR, CANP_SRC_MOTOR, 0x7FFFU);
        if (from_cmd != 0U) {
            d[0] = CANP_CAL_FAIL;
            d[1] = CANP_F_POSITION;
            can_tx_msg(CANP_R_CALIB, seq, d, 2U);
        }
        s_calibT0 = SysTick_GetTick();
        return;
    }

    /* 7) 上报: 扫描到的凸点只作对比, 补偿量用新值 */
    s_state    = CANP_ST_IDLE;
    s_calibRun = 0U;
    can_report_event(CANP_EV_CAL_DONE, 0U, lc_get_check()->valid_cnt);
    can_report_points();
    can_report_comp();
    if (from_cmd != 0U) {
        d[0] = CANP_CAL_DONE;
        d[1] = 0U;
        can_tx_msg(CANP_R_CALIB, seq, d, 2U);
    }

    s_calibT0 = SysTick_GetTick();         /* 重置周期计时 */
}

/* 0x0A 主动校准指令处理 */
static void can_handle_calib(uint8_t seq)
{
    uint8_t d[2];

    if (s_faultLv >= CANP_LV_FATAL) {
        d[0] = CANP_CAL_FAIL;
        d[1] = CANP_F_PARAM;
        can_tx_msg(CANP_R_CALIB, seq, d, 2U);
        return;
    }
    if (s_calibRun != 0U) {
        d[0] = CANP_CAL_FAIL;              /* 已在校准中 */
        d[1] = CANP_F_PARAM;
        can_tx_msg(CANP_R_CALIB, seq, d, 2U);
        return;
    }
    if (s_exec != EXEC_IDLE) {
        d[0] = CANP_CAL_FAIL;              /* 有任务在执行, 拒绝 */
        d[1] = CANP_F_PARAM;
        can_tx_msg(CANP_R_CALIB, seq, d, 2U);
        return;
    }

    can_calib_run(seq, 1U);
}

static void can_dispatch(uint8_t cmd, uint8_t seq, const uint8_t *d, uint8_t len)
{
    s_dbgDis++;

    /* 主动校准期间: 操控类指令回"校准中", 心跳/查询/调试正常回,
     * 04(停止)/05(清故障) 永远放行 —— 急停是保护的底线。
     */
    if (s_calibRun != 0U) {
        if (cmd == CANP_C_TASK) {
            uint8_t r[3];
            r[0] = CANP_ACK_CALIBRATING;       /* 校准中: 任务拒收 */
            r[1] = 0U;
            r[2] = 0U;
            can_tx_msg(CANP_R_TASK_ACK, seq, r, 3U);
            return;
        }
        if (cmd == CANP_C_GOZERO) {
            uint8_t r[3];
            r[0] = CANP_GZ_CALIBRATING;        /* 校准中: 拒绝去 0 */
            r[1] = 0U;
            r[2] = 0U;
            can_tx_msg(CANP_R_GOZERO, seq, r, 3U);
            return;
        }
        if (cmd == CANP_C_RECAL) {
            uint8_t r[2];
            r[0] = CANP_RECAL_CALIBRATING;     /* 校准中: 拒绝重标定 */
            r[1] = CANP_F_PARAM;
            can_tx_msg(CANP_R_RECAL, seq, r, 2U);
            return;
        }
        if (cmd == CANP_C_CALIB) {
            uint8_t r[2];
            r[0] = CANP_CAL_FAIL;              /* 已在校准中 */
            r[1] = CANP_F_PARAM;
            can_tx_msg(CANP_R_CALIB, seq, r, 2U);
            return;
        }
        if ((cmd != CANP_C_PING) && (cmd != CANP_C_INFO) && (cmd != CANP_C_STATUS) &&
            (cmd != CANP_C_DEBUG) && (cmd != CANP_C_COMP_GET) &&
            (cmd != CANP_C_STOP) && (cmd != CANP_C_FAULT_CLR)) {
            return;
        }
    }

    /* 开机自检/回零/标定(非主动校准)这三种独占电机的长动作期间做指令隔离,
     * 避免 03/07/09 和正在跑的动作抢电机。
     * 关键: 被拒绝的指令一律回一个"拒绝"应答, 绝不静默丢弃。
     */
    if ((s_calibRun == 0U) &&
        ((s_state == CANP_ST_SELFCHECK) || (s_state == CANP_ST_HOMING) ||
         (s_state == CANP_ST_CALIB))) {
        if (cmd == CANP_C_TASK) {
            uint8_t r[3];
            r[0] = CANP_ACK_PARAM;             /* 自检中: 任务拒收 */
            r[1] = 0U;
            r[2] = 0U;
            can_tx_msg(CANP_R_TASK_ACK, seq, r, 3U);
            return;
        }
        if (cmd == CANP_C_GOZERO) {
            uint8_t r[3];
            r[0] = CANP_GZ_BUSY;               /* 忙: 拒绝去 0 */
            r[1] = 0U;
            r[2] = 0U;
            can_tx_msg(CANP_R_GOZERO, seq, r, 3U);
            return;
        }
        if (cmd == CANP_C_RECAL) {
            uint8_t r[2];
            r[0] = CANP_RECAL_FAIL;
            r[1] = CANP_F_PARAM;
            can_tx_msg(CANP_R_RECAL, seq, r, 2U);
            return;
        }
        if (cmd == CANP_C_CALIB) {
            uint8_t r[2];
            r[0] = CANP_CAL_FAIL;              /* 自检/标定中: 拒绝校准 */
            r[1] = CANP_F_PARAM;
            can_tx_msg(CANP_R_CALIB, seq, r, 2U);
            return;
        }
        if ((cmd != CANP_C_PING) && (cmd != CANP_C_INFO) && (cmd != CANP_C_STATUS) &&
            (cmd != CANP_C_DEBUG) && (cmd != CANP_C_COMP_GET) &&
            (cmd != CANP_C_STOP) && (cmd != CANP_C_FAULT_CLR)) {
            return;
        }
    }

    switch (cmd) {
        case CANP_C_DEBUG:    can_report_debug();                 break;
        case CANP_C_INFO:     can_reply_info(seq);              break;
        case CANP_C_PING: {
            uint8_t a[2];
            a[0] = s_state;
            a[1] = s_fault;
            can_tx_msg(CANP_R_PING, seq, a, 2U);
            break;
        }
        case CANP_C_TASK:     can_handle_task(seq, d, len);      break;
        case CANP_C_STOP:     can_handle_stop(seq);              break;
        case CANP_C_FAULT_CLR:can_handle_fault_clear(seq);       break;
        case CANP_C_STATUS:   can_reply_status(seq);             break;
        case CANP_C_RECAL:    can_handle_recal(seq);             break;
        case CANP_C_GOZERO:   can_handle_goto_zero(seq);         break;
        case CANP_C_CALIB:    can_handle_calib(seq);             break;
        case CANP_C_COMP_GET: can_report_comp();                 break;
        default:                                                 break;
    }
}

/* ================= 开机自检 ================= */
static void can_boot_run(void)
{
    lc_result_t r;
    uint8_t     phase;
    uint8_t     baseOk = 1U;               /* 0=回零后读不到位置, 已用 0 兜底 */

    app_abort_clear();                     /* 清除上一次急停残留的中止标志 */
    can_report_event(CANP_EV_SELF_START, 0U, 0U);

    /* 1) 回零 */
    s_state = CANP_ST_HOMING;
    motor_enable(1);
    if (motor_limit_home() == 0U) {
        can_report_event(CANP_EV_SELF_ABORT, CANP_F_HOME_TIMEOUT, CANP_PH_HOME);
        can_report_home_fail(1U);
        return;
    }
    DDL_DelayMS(200U);                     /* 等驱动稳定, 位置读数才可靠 */
    motor_flush_rx();
    if (lc_set_base() == 0U) {             /* 记录回零位读数, 扫描起点判据 */
        DDL_DelayMS(200U);
        if (lc_set_base() == 0U) {
            can_serial_recover();          /* 读不到: 先恢复串口/驱动, 再试一次 */
            if (lc_set_base() == 0U) {
                /* 恢复后仍然读不到: 回零时刚做过 0x0A 坐标清零,
                 * 当前位置就是 0, 用 0 兜底继续, 不判 L3 锁死。
                 */
                motor_zero_pos();          /* 再确认一次坐标零点 */
                DDL_DelayMS(30U);
                baseOk = 0U;
                can_report_fault(CANP_F_SELFTEST, CANP_LV_WARN, CANP_SRC_MOTOR, 3U);
            }
        }
    }
    s_lastSlot    = 0U;
    /* 回零位在 0x32 坐标系里的读数 = 真正的 0 点; 读不到时按 0 处理 */
    s_zeroRef     = (baseOk != 0U) ? lc_get_base() : 0;
    s_lastPulse   = s_zeroRef;
    s_targetPulse = s_zeroRef;
    s_zeroValid   = 1U;                    /* 回零+状态+坐标校验通过, 允许接任务 */

    /* 2) 有没有已存的凸点/补偿 */
    if (lc_ref_valid() == 0U) {
        can_report_event(CANP_EV_CAL_START, 0U, 0U);
        s_state = CANP_ST_CALIB;
        phase   = CANP_PH_CALIB;

        r = lc_scan_run(0U);                   /* 上面已经回过零, 直接扫 */
        if (r != LC_RESULT_ALL) {
            /* 重试前必须重新回零: 上一轮扫完停在 1400, 不回零再扫会一路冲到 2800 */
            r = lc_scan_run(1U);
        }
        if (r != LC_RESULT_ALL) {
            /* 失败时把机构所在位置(mm)带在 0x89 参数里, LC 结果码放在 0x88 的 DETAIL */
            int32_t  mm = (lc_get_base() * (int32_t)LC_MM_PER_REV) /
                          (int32_t)MOTOR_PULSES_PER_REV;
            uint16_t mf = motor_read_faults();

            (void)can_move_wait(s_zeroRef, LC_RETURN_RPM, LC_SCAN_ACC);   /* 失败也先回 0 */
            s_state = CANP_ST_FAULT;
            can_report_event(CANP_EV_SELF_ABORT, CANP_F_SELFTEST, (uint8_t)r);
            if ((mf & MOTOR_FLT_ZERO_BAD) != 0U) {
                s_zeroValid = 0U;
                can_report_fault(((mf & (MOTOR_FLT_STALL | MOTOR_FLT_OVERCURR |
                                         MOTOR_FLT_OVERTEMP)) != 0U)
                                 ? CANP_F_MOTOR_STALL : CANP_F_MOTOR_PWR,
                                 CANP_LV_ERR, CANP_SRC_MOTOR, mf);
            }
            can_raise_fault(CANP_F_SELFTEST, CANP_LV_FATAL, CANP_SRC_BELT, (uint16_t)mm);
            return;
        }
        if (lc_ref_save() == 0U) {
            /* 参考位没写进 Flash: 下次上电还会重新标定, 这里先报出来(参数=1) */
            can_report_fault(CANP_F_SELFTEST, CANP_LV_WARN, CANP_SRC_BELT, 1U);
        }
        can_report_event(CANP_EV_CAL_DONE, 0U, lc_get_check()->valid_cnt);

        /* 标定完回 0 位: 直接用限位回零(1400mm @80RPM=64mm/s 约 22 秒)。
         * 注意: 速度不能低于驱动最低速度(实测 40RPM 驱动不动作, 电机根本不动)。 */

        s_state = CANP_ST_HOMING;
        if (can_return_to_zero_safe() != 0U) {
            (void)lc_set_base();
            s_zeroRef   = lc_get_base();
            s_lastPulse = s_zeroRef;
            s_zeroValid = 1U;
        } else {
            /* 标定后回零失败: 零点基准失效, 报驱动状态并锁 L3, 等 0x05 重新自检 */
            s_zeroValid = 0U;
            can_report_event(CANP_EV_SELF_ABORT, CANP_F_HOME_TIMEOUT, CANP_PH_HOME);
            can_report_home_fail(1U);
            return;
        }
        /* ★ 标定结束后把串口和驱动软复位一次:
         * 标定期间高频查询位置, 之后读位置会全部失败(只有断电重启才恢复),
         * 这里主动恢复, 不用断电。 */
        can_serial_recover();
    } else {
        phase = CANP_PH_READ_DATA;
    }

    /* 3) 完成 */
    s_state    = CANP_ST_IDLE;
    s_lastSlot = 0U;
    can_report_event(CANP_EV_SELF_DONE, 0U, phase);
    can_report_comp();
    can_report_points();                   /* 凸点位置记录完: 上报全部点位 */
}

/* ================= 任务执行 ================= */
static void can_exec_task(void)
{
    uint32_t now = SysTick_GetTick();

    /* 执行器看门狗: 万一状态机长时间不推进(实测有卡在 EXEC_WAIT 里两小时、
     * 既不动作也不报错、队列被上位机灌满只能回"队列满"的情况),
     * 这里强制收尾: 停电机 -> 报位置异常 -> 回 0x84, 让上位机能继续往下走。
     */
    if ((s_exec != EXEC_IDLE) && ((now - s_progMs) > CAN_EXEC_STALL_MS)) {
        uint8_t wasPark = s_parkMode;

        motor_stop();
        s_parkMode = 0U;
        s_state    = CANP_ST_IDLE;
        s_progMs   = now;
        s_zeroValid = 0U;                  /* 卡死 40s 未推进, 零点不再可信 */
        s_fault    = CANP_F_POSITION;
        s_faultLv  = CANP_LV_ERR;
        can_report_fault(CANP_F_POSITION, CANP_LV_ERR, CANP_SRC_MOTOR, 2U);

        if ((wasPark != 0U) && (s_parkAfter == 0U)) {
            uint8_t r[3];
            r[0] = CANP_GZ_FAIL;
            r[1] = 0U;
            r[2] = 0U;
            can_tx_msg(CANP_R_GOZERO, s_parkSeq, r, 3U);
            s_exec = EXEC_IDLE;
        } else {
            s_result = (wasPark != 0U) ? CANP_DONE_PARK_FAIL : CANP_DONE_MOVE_FAIL;
            s_exec   = EXEC_DONE;
        }
        return;
    }

    if (s_exec == EXEC_IDLE) {
        if ((s_qCnt == 0U) || (s_faultLv >= CANP_LV_FATAL)) {
            return;
        }
        s_curN   = s_queue[s_qHead].n;
        s_curSeq = s_queue[s_qHead].seq;
        memcpy(s_curSlot, s_queue[s_qHead].slot, s_curN);
        s_qHead  = (uint8_t)((s_qHead + 1U) % CANP_TASK_QUEUE_MAX);
        s_qCnt--;

        motor_enable(1);                   /* 每个任务开始前确保驱动使能 */
        can_sort_slots(s_curSlot, s_curN, s_lastSlot);
        s_curIdx  = 0U;
        s_doneCnt = 0U;
        s_result  = CANP_DONE_OK;
        s_parkMode = 0U;
        s_exec    = EXEC_NEXT;
        s_progMs  = now;
    }

    switch (s_exec) {
        case EXEC_NEXT: {
            uint32_t mm;
            int32_t  pulse;
            int32_t  d;

            if (s_curIdx >= s_curN) {
                can_start_park(1U);         /* 全部工位做完 -> 自动回 0 位 */
                break;
            }

            /* 运动前确保舵机已回中(1500): 防止舵机没收回来就运动、撞物料 */
            if (servo_ensure_mid() == 0U) {
                can_report_fault(CANP_F_SERVO_NOACK, CANP_LV_ERR, CANP_SRC_SERVO, 0U);
                s_result  = CANP_DONE_SERVO_FAIL;
                s_exec    = EXEC_DONE;
                break;
            }

            s_curNo = s_curSlot[s_curIdx];
            mm      = can_slot_mm(s_curNo);
            pulse   = motor_mm_to_pulse(mm);
            pulse  += lc_comp_at(pulse);        /* 皮带脉冲补偿 */
            pulse  += s_zeroRef;                /* 叠上回零位的坐标偏移 */

            s_targetPulse = pulse;
            d = can_move_to(pulse, POS_RPM, POS_ACC);   /* 相对运动, 方向按差值算 */
            if (d == INT32_MIN) {
                if (s_readRetry < CAN_READ_RETRY_MAX) {
                    s_readRetry++;
                    DDL_DelayMS(30U);
                    break;                          /* 留在 EXEC_NEXT, 下一轮再试 */
                }
                s_readRetry = 0U;
                /* 多轮都读不到: 不瞎走, 报位置异常并中止本包(参数 1 = 位置读取失败) */
                {
                    uint16_t mf = motor_read_faults();
                    if ((mf & MOTOR_FLT_ZERO_BAD) != 0U) {
                        s_zeroValid = 0U;          /* 驱动保护/松轴 -> 零点失效 */
                        can_report_fault(((mf & (MOTOR_FLT_STALL | MOTOR_FLT_OVERCURR |
                                                 MOTOR_FLT_OVERTEMP)) != 0U)
                                         ? CANP_F_MOTOR_STALL : CANP_F_MOTOR_PWR,
                                         CANP_LV_ERR, CANP_SRC_MOTOR, mf);
                    }
                }
                s_fault   = CANP_F_POSITION;
                s_faultLv = CANP_LV_ERR;
                can_report_fault(CANP_F_POSITION, CANP_LV_ERR, CANP_SRC_MOTOR, 1U);
                s_result  = CANP_DONE_MOVE_FAIL;
                s_exec    = EXEC_DONE;
                break;
            }
            s_readRetry = 0U;
            s_moveMinMs = can_move_min_ms(d);
            s_moveT0 = now;
            s_state  = CANP_ST_MOVING;
            s_exec   = EXEC_WAIT;
            break;
        }

        case EXEC_WAIT: {
            uint8_t arrived = 0U;

            if ((now - s_moveT0) < s_moveMinMs) {
                break;                          /* 理论时间没到, 不采信到位 */
            }
            /* 优先用驱动主动发来的 9F 到位帧(最可信, 且不用频繁问串口);
             * 兜底: 每 200ms 才核对一次位置, 避免把串口读乱 */
            if (g_motor_done_cnt != s_doneAtMove) {
                arrived = 1U;
            } else if ((now - s_posChkMs) >= 200U) {
                s_posChkMs = now;
                if (can_moved_ok() != 0U) {
                    arrived = 1U;
                }
            }

            if (arrived != 0U) {
                s_lastPulse = s_targetPulse;
                s_progMs    = now;              /* 有进展, 喂看门狗 */

                if (s_parkMode != 0U) {
                    /* 回到 0 位 */
                    s_parkMode = 0U;
                    s_lastSlot = 0U;
                    s_state    = CANP_ST_IDLE;
                    if (s_parkAfter != 0U) {
                        s_exec = EXEC_DONE;      /* 回0 完成, 再报任务完成 */
                    } else {
                        uint8_t r[3];
                        r[0] = CANP_GZ_ARRIVED;
                        r[1] = 0U;
                        r[2] = 0U;
                        can_tx_msg(CANP_R_GOZERO, s_parkSeq, r, 3U);
                        s_exec = EXEC_IDLE;
                    }
                } else {
                    s_settleT0 = now;
                    s_exec     = EXEC_SETTLE;    /* 再稳定一下才动舵机 */
                }
            } else if ((s_moveRetry < 2U) &&
                       ((now - s_moveT0) <= s_moveTmo) &&   /* ★ 硬超时必须优先 */
                       ((now - s_moveLastAct) > (s_moveMinMs + 800U))) {
                int32_t cur = can_read_pos();

                if ((cur != MOTOR_ERR_TIMEOUT) &&
                    (cur > (s_moveStart - CAN_POS_TOL_PULSE)) &&
                    (cur < (s_moveStart + CAN_POS_TOL_PULSE))) {
                    s_moveRetry++;
                    can_driver_ready();
                    if (s_moveWant > 0) {
                        motor_move(0, (uint32_t)s_moveWant, POS_RPM, POS_ACC,
                                   MOTOR_MODE_REL_CUR, 0U);
                    } else if (s_moveWant < 0) {
                        motor_move(1, (uint32_t)(-s_moveWant), POS_RPM, POS_ACC,
                                   MOTOR_MODE_REL_CUR, 0U);
                    }
                    s_doneAtMove  = g_motor_done_cnt;  /* 旧计数别误触发 */
                    s_posChkMs    = now;
                    s_moveLastAct = now;               /* 下次重发至少隔 minMs+0.8s */
                    s_moveTmo    += 6000U;             /* 超时相应延长 */
                    s_progMs      = now;               /* 喂看门狗 */
                }
            } else if ((now - s_moveT0) > s_moveTmo) {
                int32_t act = motor_read_input_pulses(MOTOR_RX_TIMEOUT_MS);
                int32_t amm = (act == MOTOR_ERR_TIMEOUT)
                              ? 0x7FFF
                              : ((act - s_zeroRef) * (int32_t)LC_MM_PER_REV
                                 / (int32_t)MOTOR_PULSES_PER_REV);
                uint16_t mf = motor_read_faults();

                s_progMs = now;
                motor_stop();

                /* 驱动自己的过流/堵转/松轴: 明确上报并作废零点, 不再继续用旧坐标 */
                if ((mf & MOTOR_FLT_ZERO_BAD) != 0U) {
                    s_zeroValid = 0U;
                    can_report_fault(((mf & (MOTOR_FLT_STALL | MOTOR_FLT_OVERCURR |
                                             MOTOR_FLT_OVERTEMP)) != 0U)
                                     ? CANP_F_MOTOR_STALL : CANP_F_MOTOR_PWR,
                                     CANP_LV_ERR, CANP_SRC_MOTOR, mf);
                }
                s_fault     = CANP_F_POSITION;     /* 同时发 0x89 (L2) */
                s_faultLv   = CANP_LV_ERR;
                can_report_fault(CANP_F_POSITION, CANP_LV_ERR, CANP_SRC_MOTOR,
                                 (uint16_t)amm);   /* 参数 = 失败时实际位置 mm */
                if (s_parkMode != 0U) {
                    s_parkMode = 0U;
                    s_state    = CANP_ST_IDLE;
                    if (s_parkAfter != 0U) {
                        /* 自动回 0 失败: 先用慢速撞限位重新定点(修复零点),
                         * 成功则按正常完成收尾, 失败才报 PARK_FAIL。 */
                        uint8_t bOk;

                        s_state = CANP_ST_HOMING;
                        if (motor_limit_home() != 0U) {
                            DDL_DelayMS(150U);
                            motor_flush_rx();
                            bOk = lc_set_base();
                            if (bOk == 0U) {
                                DDL_DelayMS(150U);
                                motor_flush_rx();
                                bOk = lc_set_base();
                            }
                            s_zeroRef   = (bOk != 0U) ? lc_get_base() : 0;
                            s_lastPulse = s_zeroRef;
                            s_lastSlot  = 0U;
                            s_zeroValid = 1U;
                            s_result    = CANP_DONE_OK;
                        } else {
                            s_zeroValid = 0U;
                            can_report_home_fail(0U);
                            s_result    = CANP_DONE_PARK_FAIL;   /* 工位做完了, 回0失败 */
                        }
                        s_state = (s_faultLv >= CANP_LV_FATAL) ? CANP_ST_FAULT : CANP_ST_IDLE;
                        s_exec  = EXEC_DONE;
                    } else {
                        uint8_t r[3];
                        r[0] = CANP_GZ_FAIL;
                        r[1] = 0U;
                        r[2] = 0U;
                        can_tx_msg(CANP_R_GOZERO, s_parkSeq, r, 3U);
                        s_exec = EXEC_IDLE;
                    }
                } else {
                    s_result = CANP_DONE_MOVE_FAIL;   /* 0x84 报运动失败 */
                    s_exec   = EXEC_DONE;
                }
            }
            break;
        }

        case EXEC_SETTLE:
            if ((now - s_settleT0) >= CAN_ARRIVE_SETTLE_MS) {
                s_state = CANP_ST_SERVO;
                s_exec  = EXEC_SERVO;
            }
            break;

        case EXEC_SERVO:
            can_servo_cycle();                  /* 推出 -> 保持 -> 回中 -> 稳定 */
            s_doneCnt++;
            s_curIdx++;
            s_lastSlot = s_curNo;
            s_exec     = EXEC_NEXT;
            break;

        case EXEC_DONE:
        default: {
            uint8_t d[3];

            d[0] = s_result;
            d[1] = s_doneCnt;
            d[2] = s_curNo;
            can_tx_msg(CANP_R_TASK_DONE, s_curSeq, d, 3U);

            /* 本包失败 -> 后面排队的指令已无意义, 全部作废并逐条回"故障中断",
             * 免得上位机一直等这些任务的完成帧 */
            if (s_result != CANP_DONE_OK) {
                while (s_qCnt != 0U) {
                    uint8_t q[3];
                    q[0] = CANP_DONE_FAULT;
                    q[1] = 0U;
                    q[2] = 0U;
                    can_tx_msg(CANP_R_TASK_DONE, s_queue[s_qHead].seq, q, 3U);
                    s_qHead = (uint8_t)((s_qHead + 1U) % CANP_TASK_QUEUE_MAX);
                    s_qCnt--;
                }
            }

            s_exec  = EXEC_IDLE;
            s_state = (s_faultLv >= CANP_LV_FATAL) ? CANP_ST_FAULT : CANP_ST_IDLE;
            break;
        }
    }
}

/* ================= 心跳 ================= */
static void can_check_heartbeat(void)
{
#if (CAN_HEARTBEAT_ENABLE == 0U)
    /* 调试模式: 心跳超时监控关闭, 只保留 0x02 -> 0x82 应答 */
    return;
#else
    if ((s_commLost != 0U) || (s_bootDone == 0U)) {
        return;
    }
    if ((SysTick_GetTick() - s_lastRxMs) > CANP_PING_TIMEOUT_MS) {
        s_commLost = 1U;
        can_raise_fault(CANP_F_COMM_LOST, CANP_LV_FATAL, CANP_SRC_COMM, 0U);
    }
#endif
}

/* ================= 主循环 ================= */
void can_task(void)
{
    can_rx_process();
    can_tx_kick();

    if (s_bootDone == 0U) {
        s_bootDone = 1U;
        s_rxTail   = s_rxHead;                 /* 丢掉自检前堆积的帧 */
        can_boot_run();
        s_rxTail   = s_rxHead;
        s_lastRxMs = SysTick_GetTick();        /* 自检期间不计心跳 */
        return;
    }

    can_check_heartbeat();

    /* 定时主动校准: 到点且完全空闲(无任务/无排队/无故障/非校准)才触发。
     * 若到点时正在忙, 则等忙完的下一轮自然触发(条件里要求 IDLE)。
     */
    if ((s_calibRun == 0U) && (s_exec == EXEC_IDLE) && (s_qCnt == 0U) &&
        (s_faultLv < CANP_LV_FATAL) && (s_state == CANP_ST_IDLE) &&
        ((SysTick_GetTick() - s_calibT0) >= CAN_CALIB_PERIOD_MS)) {
        can_calib_run(0U, 0U);             /* 定时触发: 不回 0x8D, 只发 0x88/0x8A/0x8C */
    }

    if (s_busOff != 0U) {
        /* 总线关闭: 丢空发送缓冲并重新初始化 CAN 控制器(总线上发不出东西, 不报故障) */
        s_busOff = 0U;
        s_txHead = 0U;
        s_txTail = 0U;
        s_txBusy = 0U;
        can_hw_config();
        s_commLost = 0U;
        s_lastRxMs = SysTick_GetTick();
    }

    can_exec_task();

#if (CAN_DEBUG_REPORT != 0U)
    if ((SysTick_GetTick() - s_dbgT0) >= 2000U) {
        s_dbgT0 = SysTick_GetTick();
        can_report_debug();
    }
#endif
}

/* ================= 初始化 ================= */
static void can_hw_config(void)
{
    stc_can_init_t          stcCanInit;
    stc_can_filter_config_t stcFilter;

    LL_PERIPH_WE(LL_PERIPH_FCG);
    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_CAN, ENABLE);
    LL_PERIPH_WP(LL_PERIPH_FCG);

    (void)CAN_StructInit(&stcCanInit);

    /* 1Mbps: fCAN=8M, 1TQ=125ns, 8TQ/bit, 采样点 75% */
    stcCanInit.stcBitCfg.u32Prescaler = 1U;
    stcCanInit.stcBitCfg.u32TimeSeg1  = 6U;
    stcCanInit.stcBitCfg.u32TimeSeg2  = 2U;
    stcCanInit.stcBitCfg.u32SJW       = 2U;

    stcCanInit.u8WorkMode       = CAN_WORK_MD_NORMAL;
    stcCanInit.u8SelfAck        = CAN_SELF_ACK_DISABLE;
    stcCanInit.u8RxWarnLimit    = 8U;
    stcCanInit.u8ErrorWarnLimit = 15U;
    stcCanInit.u8RxOvfMode      = CAN_RX_OVF_SAVE_NEW;   /* 溢出时覆盖最旧 */

    /* 全收滤波器(与 app/router 一致):
     *   MASK 位 = 1 表示"该位不参与比较", 所以全收要写 0x1FFFFFFF, 写 0 只能收到 ID=0!
     *   CODE 里带帧类型位 0x3<<29 = 扩展帧。
     */
    stcFilter.u32ID     = (0x3UL << 29U);              /* 扩展帧, ID = 0 */
    stcFilter.u32IDMask = 0x1FFFFFFFUL;                /* 29 位 ID 全部不比较 */
    stcFilter.u32IDType = CAN_ID_EXT;
    stcCanInit.pstcFilter      = &stcFilter;
    stcCanInit.u16FilterSelect = CAN_FILTER1;

    (void)CAN_Init(CM_CAN, &stcCanInit);

    CAN_IntCmd(CM_CAN, CAN_INT_RX | CAN_INT_PTB_TX | CAN_INT_BUS_ERR, ENABLE);
}

void can_init(void)
{
    stc_irq_signin_config_t irqCfg;

    s_txHead = 0U;
    s_txTail = 0U;
    s_txBusy = 0U;
    s_rxHead = 0U;
    s_rxTail = 0U;
    s_rxActive = 0U;
    s_busOff = 0U;
    s_txSeq  = 0U;
    s_compSeq = 0U;
    s_state  = CANP_ST_IDLE;
    s_fault  = 0U;
    s_faultLv = 0U;
    s_commLost = 0U;
    s_bootDone = 0U;
    s_lastFaultCode = 0U;
    s_zeroValid = 0U;
    s_qHead = 0U;
    s_qTail = 0U;
    s_qCnt  = 0U;
    s_exec  = EXEC_IDLE;
    s_lastSlot = 0U;
    s_moveMinMs   = 0U;
    s_targetPulse = 0;
    s_lastPulse   = 0;
    s_zeroRef     = 0;
    s_parkMode    = 0U;
    s_parkAfter   = 0U;
    s_parkSeq     = 0U;
    s_calibRun    = 0U;
    s_calibT0     = 0U;                       /* 校准周期从 0 起算 */

    can_hw_config();

    irqCfg.enIntSrc    = INT_SRC_CAN_INT;
    irqCfg.enIRQn      = INT008_IRQn;
    irqCfg.pfnCallback = can_irq_handler;
    (void)INTC_IrqSignIn(&irqCfg);

    NVIC_ClearPendingIRQ(INT008_IRQn);
    NVIC_SetPriority(INT008_IRQn, DDL_IRQ_PRIO_01);
    NVIC_EnableIRQ(INT008_IRQn);

    s_lastRxMs = SysTick_GetTick();
    s_dbgT0    = SysTick_GetTick();
    s_dbgIsr   = 0U;
    s_dbgRx    = 0U;
    s_dbgAcc   = 0U;
    s_dbgDis   = 0U;
    s_dbgDlc   = 0U;
    s_dbgLastId = 0U;

    /* 注册阻塞期服务: motor.c 回零 / lc.c 扫描 的等待循环里会调 app_service() */
    app_set_service(can_idle);
}
