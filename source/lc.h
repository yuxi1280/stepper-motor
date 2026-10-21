#ifndef __LC_H__
#define __LC_H__

#include "main.h"
#include "motor.h"

/* ============================================================================
 * 距离标注尺(凸点)扫描 —— PM-K45 光电传感器
 * ----------------------------------------------------------------------------
 * 硬件:  PM-K45 输出 -> PB12, 有物体(凸点)时低电平
 *        电机: 42 闭环步进(Emm), 16齿 S3M 轮 -> 3mm × 16 = 48mm/圈
 * 功能:  从 0 点走到 LC_TARGET_MM(1500mm), 途中每个凸点停车记录,
 *        测满 LC_POINT_MAX(10) 个点 或 走到 1500mm 即结束
 * 注意:  全程用"相对当前实时位置"(FD mode=2)走剩余距离,
 *        因为 0x0A 不会复位驱动的绝对坐标, motor_move_abs() 不可用
 * ========================================================================== */

/* ---- 传感器输入 ---- */
#define LC_SENSOR_PORT          (GPIO_PORT_B)
#define LC_SENSOR_PIN           (GPIO_PIN_12)
#define LC_SENSOR_ACTIVE_LOW    (1U)      /* 1=有物体时低电平 */

/* ---- 标尺/运动 ---- */
#define LC_POINT_MAX            (10U)
#define LC_TARGET_MM            (1500U)
#define LC_SCAN_RPM             (80U)    /* 扫描速度, 凸点窄就再降 */
#define LC_SCAN_ACC             (100U)

/* 16齿 S3M 同步轮: 16 × 3mm = 48mm/圈 */
#define LC_MM_PER_REV           (PULLEY_MM_PER_REV)

#define LC_TARGET_PULSE         ((int32_t)(((uint32_t)LC_TARGET_MM * MOTOR_PULSES_PER_REV) / LC_MM_PER_REV))
#define LC_ARRIVE_TOL_PULSE     ((int32_t)((2U * MOTOR_PULSES_PER_REV) / LC_MM_PER_REV))  /* 2mm */

/* ---- 时序 ---- */
#define LC_POLL_MS              (1U)
#define LC_POS_TIMEOUT_MS       (50U)     /* 停车后读数: 驱动应答偶尔 >5ms */
#define LC_POS_TIMEOUT_FAST_MS  (5U)      /* 运行中兜底读数 */
#define LC_READ_RETRY           (3U)
#define LC_BACKUP_CHECK_MS      (500U)
#define LC_STOP_SETTLE_MS       (80U)
#define LC_STALL_CNT            (6U)      /* 6×500ms 不动 -> 判卡住 */
#define LC_TOTAL_TIMEOUT_MS     (120000U)

/* ---- 结果码 ---- */
typedef enum {
    LC_RESULT_NONE      = 0U,
    LC_RESULT_ALL       = 1U,   /* 记录满 10 个点 */
    LC_RESULT_ARRIVED   = 2U,   /* 未满 10 个点但已到 1500mm */
    LC_RESULT_TIMEOUT   = 3U,
    LC_RESULT_STALL     = 4U,
    LC_RESULT_POS_ERR   = 5U,
    LC_RESULT_HOME_FAIL = 6U,
    LC_RESULT_DRV_ERR   = 7U,
    LC_RESULT_BUSY      = 8U
} lc_result_t;

typedef struct {
    int32_t raw_pulse;     /* 0x32 原始值 */
    int32_t pulse;         /* 相对扫描起点的脉冲 */
    int32_t pos_mm_x100;   /* 距起点 0.01mm */
    int32_t gap_mm_x100;   /* 与上一点间距 0.01mm(第1点=距起点) */
} lc_point_t;

void              lc_init(void);
uint8_t           lc_sensor_active(void);
lc_result_t       lc_scan_run(uint8_t do_home);
uint8_t           lc_get_count(void);
const lc_point_t *lc_get_points(void);
lc_result_t       lc_get_result(void);
int32_t           lc_get_base_pulse(void);
void              lc_report_summary(void);
void              lc_report_can(void);

#endif /* __LC_H__ */
