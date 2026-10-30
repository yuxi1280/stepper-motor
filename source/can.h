#ifndef __CAN_H__
#define __CAN_H__

#include "main.h"
#include "can_proto.h"

/* ============================================================================
 * CAN 协议 v3 应用层 (Eclipse/app/CANv3.txt)
 * ----------------------------------------------------------------------------
 * 物理层 : CAN 2.0B, 29 位扩展帧, 1 Mbps
 * 帧结构 : [CMD][SEQ(bit7=MORE)][LEN][DATA<=5B], 一条消息 1~11 帧
 * 设备ID : 上位机->设备 0x10000000|节点号, 设备->上位机 0x00000100|节点号
 * 功能   : 心跳、工位任务(1~50 个, 自排队+就近排序)、固定舵机动作、
 *          自检/标定、脉冲补偿上报、故障分级上报与清除
 * ========================================================================== */

/* 本机节点地址(1~3)：三块板只需要改这一处 */
#define SLAVE_ADDR              (CANP_NODE_ADDR)

/* 收发缓冲 */
#define CAN_TX_RING_SIZE        (64U)
#define CAN_RX_QUEUE_SIZE       (48U)     /* 50 工位包 11 帧 + 心跳, 必须够深 */

/* ================= 调试开关 =================
 * 心跳超时监控: 1=打开(整机出厂必须为 1), 0=关闭(CAN 盒台架调试用)
 * 关闭后设备不再判断"上位机掉线", 但收到 0x02 PING 仍然正常回 0x82。
 */
#define CAN_HEARTBEAT_ENABLE    (0U)

/* 调试上报: 1=每 2s 主动发一帧 0x8F, 0=关闭(需要时手动发 0x0F 查询一次) */
#define CAN_DEBUG_REPORT        (0U)

/* 单工位运动超时(全行程 1.5m @600rpm 约 3s, 留足余量) */
#define CAN_MOVE_TIMEOUT_MS     (15000U)

/* ---- 步进到位判定 ----
 * PRF(到位)标志在上一动作结束后是锁存的, 新指令下发瞬间它仍为 1,
 * 所以不能只靠 PRF: 先按距离算出"理论最短运行时间", 时间到了再看 PRF,
 * 并用 0x32 实际位置二次确认。
 */
#define CAN_MOVE_START_MS       (250U)    /* 启动/加速余量 */
#define CAN_MOVE_EXTRA_PCT      (140U)    /* 理论时间 * 1.4 (留加速与余量) */
#define CAN_MOVE_MIN_MS         (300U)    /* 每段最短等待 */
#define CAN_POS_TOL_PULSE       (200)     /* 到位位置容差(约 3mm) */
#define CAN_ARRIVE_SETTLE_MS    (150U)    /* 到位后稳定时间再动舵机 */

/* ---- 舵机动作时序(每个工位一轮) ----
 * 推出 -> 保持 -> 回中 -> 稳定，全部用时间硬等，确保真的推完并回中。
 */
#define CAN_SERVO_PUSH_PWM      (2200U)   /* 推出位置 */
#define CAN_SERVO_MID_PWM       (1500U)   /* 中位 */
#define CAN_SERVO_PUSH_MS       (100U)    /* 推出行程时间(T 参数) */
#define CAN_SERVO_HOLD_MS       (100U)    /* 推到位后保持 */
#define CAN_SERVO_BACK_MS       (100U)    /* 回中行程时间(T 参数) */
#define CAN_SERVO_GAP_MS        (50U)    /* 行程结束后额外等待 */
#define CAN_SERVO_SETTLE_MS     (100U)    /* 回中后稳定 */

/* ---- 初始化 / 主循环 ---- */
void can_init(void);                       /* 时钟、位时序、过滤器、中断 */
void can_task(void);                       /* 主循环每轮调用: 收包/调度/心跳/执行 */

/* ---- 发送 ---- */
void can_tx_msg(uint8_t cmd, uint8_t seq, const uint8_t *p, uint8_t len);

/* ---- 主动上报 ---- */
void can_report_comp(void);                                    /* 0x8A 补偿量 */
void can_report_event(uint8_t ev, uint8_t res, uint8_t detail);/* 0x88 自检事件 */
void can_report_fault(uint8_t code, uint8_t lv, uint8_t src, uint16_t param); /* 0x89 */

/* ---- 故障 ---- */
void    can_raise_fault(uint8_t code, uint8_t lv, uint8_t src, uint16_t param);
void    can_estop(void);                   /* 急停: 步进失能 + 舵机释放 + 清队列 */
uint8_t can_fault_code(void);              /* 当前故障码, 0=无 */
uint8_t can_is_fault(void);                /* 1=当前处于故障态 */
void    can_report_debug(void);            /* 0x8F 调试信息(寄存器/计数快照) */

#endif /* __CAN_H__ */
