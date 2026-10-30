#ifndef __CAN_PROTO_H__
#define __CAN_PROTO_H__

/* ============================================================================
 * EPS 执行设备 CAN 协议 v3（与 Eclipse/app/CANv3.txt 一一对应）
 * ----------------------------------------------------------------------------
 * 物理层: CAN 2.0B / 29 位扩展帧 / 1 Mbps
 * 帧结构: [CMD][SEQ(bit7=MORE,bit6..0=序号)][LEN][DATA 最多 5 字节]
 * 消息长度: 1~11 帧，LEN = 数据总长度(0~63)，>5 字节自动续帧
 * ========================================================================== */

#include "main.h"

#define CANP_VER                (3U)
#define CANP_FW_MAJOR           (1U)
#define CANP_FW_MINOR           (0U)

/* ================= 节点 / ID ================= */
#define CANP_NODE_ADDR          (1U)          /* 本机节点号 1~3, 与 SLAVE_ADDR 同源 */
#define CANP_NODE_BCAST         (0x000U)      /* 广播节点号 */

#define CANP_ID_HOST_BASE       (0x10000000UL)      /* 上位机 -> 设备 */
#define CANP_ID_DEV_BASE        (0x00000100UL)      /* 设备 -> 上位机 */

#define CANP_ID_HOST_BCAST      (CANP_ID_HOST_BASE | CANP_NODE_BCAST)
#define CANP_ID_HOST_TO(n)      (CANP_ID_HOST_BASE | ((uint32_t)(n) & 0x3FFUL))
#define CANP_ID_DEV( n)         (CANP_ID_DEV_BASE  | ((uint32_t)(n) & 0xFFUL))

#define CANP_ID_MASK            (0x1FFC0000UL)      /* 只区分 bit28..10, 低 10 位是节点号 */

/* ================= 帧头 ================= */
#define CANP_HDR_SIZE           (3U)          /* CMD + SEQ + LEN */
#define CANP_FRAME_DATA         (5U)          /* 每帧有效数据字节数 */
#define CANP_MSG_MAX            (64U)         /* 单条消息最大数据长度(含头 3 字节后) */
#define CANP_SEQ_MORE           (0x80U)       /* SEQ bit7: 还有后续帧 */
#define CANP_SEQ_MASK           (0x7FU)

/* ================= 命令字: 上位机 -> 设备 ================= */
#define CANP_C_INFO             (0x01U)       /* 查询设备信息 */
#define CANP_C_PING             (0x02U)       /* 心跳 */
#define CANP_C_TASK             (0x03U)       /* 工位任务包 1~50 个 */
#define CANP_C_STOP             (0x04U)       /* 停止当前任务 */
#define CANP_C_FAULT_CLR        (0x05U)       /* 清除故障 */
#define CANP_C_STATUS           (0x06U)       /* 查询状态 */
#define CANP_C_RECAL            (0x07U)       /* 重新标定凸点(换皮带) */
#define CANP_C_COMP_GET         (0x08U)       /* 读取补偿量 */
#define CANP_C_GOZERO           (0x09U)       /* 去 0 点位(不重新回零) */
#define CANP_C_DEBUG            (0x0FU)       /* 调试信息(台架用) */

/* ================= 命令字: 设备 -> 上位机 ================= */
#define CANP_R_INFO             (0x81U)       /* 信息应答 */
#define CANP_R_PING             (0x82U)       /* 心跳应答 */
#define CANP_R_TASK_ACK         (0x83U)       /* 任务收到确认 */
#define CANP_R_TASK_DONE        (0x84U)       /* 任务总完成 */
#define CANP_R_CLR_ACK          (0x85U)       /* 故障清除应答 */
#define CANP_R_STATUS           (0x86U)       /* 状态应答 */
#define CANP_R_RECAL            (0x87U)       /* 重新标定进度/结果 */
#define CANP_R_EVENT            (0x88U)       /* 自检/标定事件(主动) */
#define CANP_R_FAULT            (0x89U)       /* 故障上报(主动) */
#define CANP_R_COMP             (0x8AU)       /* 补偿量上报(主动) */
#define CANP_R_GOZERO           (0x8BU)       /* 去 0 点位应答 */
#define CANP_R_DEBUG            (0x8FU)       /* 调试信息应答 */

/* ================= 状态 ================= */
#define CANP_ST_IDLE            (0x00U)
#define CANP_ST_SELFCHECK       (0x01U)
#define CANP_ST_HOMING          (0x02U)
#define CANP_ST_CALIB           (0x03U)
#define CANP_ST_MOVING          (0x04U)
#define CANP_ST_SERVO           (0x05U)
#define CANP_ST_FAULT           (0x06U)

/* ================= 故障等级 ================= */
#define CANP_LV_WARN            (0x01U)       /* 只报, 继续 */
#define CANP_LV_ERR             (0x02U)       /* 停当前任务 */
#define CANP_LV_FATAL           (0x03U)       /* 急停: 舵机释放 + 步进失能 */

/* ================= 故障码 ================= */
#define CANP_F_OVERCURRENT      (0x01U)       /* L3 总轨过流 > 2500mA */
#define CANP_F_MOTOR_PWR        (0x02U)       /* L3 步进掉电/位置丢失 */
#define CANP_F_MOTOR_STALL      (0x03U)       /* L2 堵转 */
#define CANP_F_HOME_TIMEOUT     (0x04U)       /* L3 回零超时/限位失效 */
#define CANP_F_SERVO_NOACK      (0x05U)       /* L2 舵机无应答 */
#define CANP_F_COMM_LOST        (0x06U)       /* L3 上位机心跳丢失 */
#define CANP_F_SELFTEST         (0x07U)       /* L3 自检/标定失败 */
#define CANP_F_BELT_DEV         (0x08U)       /* L1 皮带偏差超限, 只报不补偿 */
#define CANP_F_PARAM            (0x09U)       /* L2 指令/参数非法 */
#define CANP_F_POSITION         (0x0AU)       /* L2 位置异常/未到位 */
#define CANP_F_OVERTEMP         (0x0BU)       /* L2 过温 */

/* ================= 故障来源 ================= */
#define CANP_SRC_MOTOR          (0x01U)
#define CANP_SRC_SERVO          (0x02U)
#define CANP_SRC_POWER          (0x03U)
#define CANP_SRC_COMM           (0x04U)
#define CANP_SRC_BELT           (0x05U)
#define CANP_SRC_SYSTEM         (0x06U)

/* ================= 任务相关结果码 ================= */
/* 0x83 TASK_ACK [RESULT][N][QUEUE_IDX] */
#define CANP_ACK_OK             (0x00U)       /* 已接受/已排队 */
#define CANP_ACK_QUEUE_FULL     (0x01U)       /* 队列已满 */
#define CANP_ACK_PARAM          (0x02U)       /* 参数非法(也用于故障态拒收) */

/* 0x84 TASK_DONE [RESULT][DONE][SLOT] */
#define CANP_DONE_OK            (0x00U)
#define CANP_DONE_HOME_FAIL     (0x01U)
#define CANP_DONE_MOVE_FAIL     (0x02U)
#define CANP_DONE_SERVO_FAIL    (0x03U)
#define CANP_DONE_SLOT_ERR      (0x04U)
#define CANP_DONE_STOPPED       (0x05U)
#define CANP_DONE_FAULT         (0x06U)
#define CANP_DONE_RESET         (0x07U)
#define CANP_DONE_PARK_FAIL     (0x08U)       /* 工位都做完了, 但回 0 位失败 */

/* 0x09 去 0 点位 / 0x8B 应答 [RESULT][0][0] */
#define CANP_GZ_ACCEPT          (0x00U)       /* 已接受, 开始运动 */
#define CANP_GZ_ARRIVED         (0x01U)       /* 已到达 0 点 */
#define CANP_GZ_FAIL            (0x02U)       /* 未到位/超时 */
#define CANP_GZ_BUSY            (0x03U)       /* 正在执行任务, 拒绝 */

/* 0x85 清除应答 */
#define CANP_CLR_OK             (0x00U)
#define CANP_CLR_FAIL           (0x01U)

/* 0x87 重新标定 [STAGE][RESULT] */
#define CANP_RECAL_ACCEPT       (0x00U)
#define CANP_RECAL_DONE         (0x01U)
#define CANP_RECAL_FAIL         (0x02U)

/* 0x88 事件 [EVENT][RESULT][DETAIL] */
#define CANP_EV_SELF_START      (0x01U)
#define CANP_EV_SELF_DONE       (0x02U)
#define CANP_EV_CAL_START       (0x03U)
#define CANP_EV_CAL_DONE        (0x04U)
#define CANP_EV_SELF_ABORT      (0x05U)
#define CANP_EV_PASS_START      (0x06U)       /* 标定第 N 轮开始, DETAIL=N */
#define CANP_EV_PASS_DONE       (0x07U)       /* 本轮结束, RESULT=确认点数(0xFF=本轮失败) */
#define CANP_EV_RETURN0         (0x08U)       /* 回0位, RESULT: 0开始 1位置到位 2限位触发 3超时 */

/* 0x88 自检阶段(DETAIL) */
#define CANP_PH_HOME            (1U)
#define CANP_PH_READ_DATA       (2U)
#define CANP_PH_CALIB           (3U)

/* ================= 补偿判定(DECISION) ================= */
#define CANP_CMP_OK             (0x00U)
#define CANP_CMP_ZERO_FIXED     (0x01U)
#define CANP_CMP_BELT_FIXED     (0x02U)
#define CANP_CMP_ZERO_ALARM     (0x03U)       /* 超上限, 不补偿 */
#define CANP_CMP_BELT_ALARM     (0x04U)       /* 超上限, 不补偿 */
#define CANP_CMP_MEAS_BAD       (0x05U)       /* 有效点不足, 不补偿 */

/* ================= 能力位(CAP) ================= */
#define CANP_CAP_SERVO          (1U << 0)
#define CANP_CAP_COMP           (1U << 1)
#define CANP_CAP_RECAL          (1U << 2)
#define CANP_CAP_QUEUE          (1U << 3)
#define CANP_CAP_ALL            (CANP_CAP_SERVO | CANP_CAP_COMP | CANP_CAP_RECAL | CANP_CAP_QUEUE)

/* ================= 规模/时序 ================= */
#define CANP_SLOT_TOTAL         (100U)        /* 工位总数 */
#define CANP_SLOT_MIN           (1U)          /* 工位号从 1 开始 */
#define CANP_SLOT_MAX           (100U)
#define CANP_TASK_N_MAX         (50U)         /* 单包最多 50 个工位 */
#define CANP_TASK_QUEUE_MAX     (3U)          /* 待执行任务队列深度 */

#define CANP_PING_PERIOD_MS     (500U)        /* 上位机 PING 周期 */
#define CANP_PING_TIMEOUT_MS    (3000U)       /* 心跳超时 -> 0x89 0x06 L3 */
#define CANP_TASK_ACK_MS        (100U)        /* 收到任务后应答时限 */
#define CANP_CAL_RETRY          (1U)          /* 标定失败重试次数 */
#define CANP_FAULT_BUF          (8U)          /* 故障去抖缓存条数 */

#endif /* __CAN_PROTO_H__ */
