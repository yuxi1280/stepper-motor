#ifndef __LC_H__
#define __LC_H__

#include "main.h"
#include "motor.h"

/* ============================================================================
 * 距离标注尺(凸点)多轮扫描 —— PM-K45, PB12, 有物体时低电平
 * ----------------------------------------------------------------------------
 * 每轮从 0 点一次走满 LC_TARGET_MM, 中途不停车;
 * 检测到的位置按 ±LC_MATCH_TOL_PULSE 归到同一个凸点;
 * 每个凸点必须有 >=2 次、相互差 <=LC_AGREE_TOL_PULSE 的数据才算确认;
 * 有未确认的就回 0 点重扫, 最多 LC_PASS_MAX 轮, 最后取平均值上报.
 * ========================================================================== */

/* ---- 传感器 ---- */
#define LC_SENSOR_PORT          (GPIO_PORT_B)
#define LC_SENSOR_PIN           (GPIO_PIN_12)
#define LC_SENSOR_ACTIVE_LOW    (1U)

/* ---- 标尺/运动 ---- */
#define LC_POINT_MAX            (10U)
#define LC_TARGET_MM            (1400U)
#define LC_SCAN_RPM             (100U)
#define LC_SCAN_ACC             (100U)
#define LC_MM_PER_REV           (PULLEY_MM_PER_REV)

#define LC_TARGET_PULSE         ((int32_t)(((uint32_t)LC_TARGET_MM * MOTOR_PULSES_PER_REV) / LC_MM_PER_REV))
#define LC_ARRIVE_TOL_PULSE     ((int32_t)((2U * MOTOR_PULSES_PER_REV) / LC_MM_PER_REV))


/*电机: 3200 脉冲/圈
同步轮: 48mm/圈
1 脉冲 = 48 / 3200 = 0.015 mm
1 mm   = 3200 / 48 = 66.6667 脉冲*/
/* ---- 多轮确认 ---- */
#define LC_PASS_MAX             (4U)      /* 最多扫几轮 */
#define LC_SAMPLE_MAX           (4U)      /* 每个凸点最多存几个样本 */
#define LC_SLOT_MAX             (12U)     /* 槽位数: 10 个点 + 余量 */
#define LC_AGREE_TOL_PULSE      (66)     /* 0.5mm: 两次误差在内才算一致 */
#define LC_MATCH_TOL_PULSE      (1000)    /* 15mm误差内的点: 归到同一个凸点的容差 */
#define LC_ZERO_TOL_PULSE       (133)     /* 2mm: 回到 0 点的判定 */
#define LC_RETURN_RPM           (300U)    /* 回 0 点速度 */
#define LC_RETURN_TIMEOUT_MS    (30000U)  /* 回 0 点超时 */
#define LC_PASS_TIMEOUT_MS      (60000U)  /* 单轮超时 */

/* ---- 时序 ---- */
#define LC_POLL_MS              (1U)
#define LC_EDGE_DEBOUNCE_MS     (3U)
#define LC_POS_CHECK_MS         (300U)
#define LC_POS_TIMEOUT_MS       (50U)
#define LC_POS_TIMEOUT_FAST_MS  (5U)
#define LC_READ_RETRY           (3U)

/* ---- 结果码 ---- */
typedef enum {
    LC_RESULT_NONE       = 0U,
    LC_RESULT_ALL        = 1U,   /* 10 个点全部确认 */
    LC_RESULT_TIMEOUT    = 3U,   /* 单轮/回 0 超时 */
    LC_RESULT_POS_ERR    = 5U,   /* 读位置失败 */
    LC_RESULT_HOME_FAIL  = 6U,   /* 回零失败 */
    LC_RESULT_INCOMPLETE = 9U    /* 跑满轮数仍有未确认点 */
} lc_result_t;

void        lc_init(void);
lc_result_t lc_scan_run(uint8_t do_home);
void        lc_report_can(void);

#endif /* __LC_H__ */
