#ifndef __LC_H__
#define __LC_H__

#include "main.h"
#include "motor.h"

/* ============================================================================
 * 距离标注尺(凸点)扫描 + 参考位 + 三层判断
 * ----------------------------------------------------------------------------
 * 第一层: 每个点是否有 >=2 个一致样本, 且不是离群点
 * 第二层: 误差分解 e[i] = a + b*(R[i]-R[1]) + r[i]
 *          a=整体偏移(零点/回零), b=斜率(皮带拉伸), r=单点残差(凸点/传感器)
 * 第三层: 正常 / 补偿 / 报警 / 剔除
 * ========================================================================== */

/* ---- 传感器 ---- */
#define LC_SENSOR_PORT          (GPIO_PORT_B)
#define LC_SENSOR_PIN           (GPIO_PIN_12)
#define LC_SENSOR_ACTIVE_LOW    (1U)

/* ---- 标尺/运动 ---- */
#define LC_POINT_MAX            (10U)
#define LC_TARGET_MM            (1400U)
#define LC_SCAN_RPM             (80U)
#define LC_SCAN_ACC             (100U)
#define LC_MM_PER_REV           (PULLEY_MM_PER_REV)

#define LC_TARGET_PULSE         ((int32_t)(((uint32_t)LC_TARGET_MM * MOTOR_PULSES_PER_REV) / LC_MM_PER_REV))
#define LC_ARRIVE_TOL_PULSE     ((int32_t)((2U * MOTOR_PULSES_PER_REV) / LC_MM_PER_REV))

/* ---- 多轮确认 ---- */
#define LC_PASS_MAX             (5U)      /* 最多 5 轮, 漏检多时多给一轮 */
#define LC_SAMPLE_MAX           (4U)
#define LC_SLOT_MAX             (12U)
#define LC_AGREE_TOL_PULSE      (100)     /* 1.5mm: 两次误差在内才算一致(原 0.5mm 过严, 10 个点只能确认 7 个) */
#define LC_MATCH_TOL_PULSE      (1000)    /* 15mm: 同一轮内归到同一凸点 */
#define LC_ZERO_TOL_PULSE       (133)     /* 2mm: 回到 0 点判定 */
#define LC_SCAN_START_TOL_PULSE (6667)    /* 100mm: 起点偏离回零位超过这个值就不许扫描 */
#define LC_RETURN_RPM           (300U)
#define LC_RETURN_TIMEOUT_MS    (30000U)
#define LC_PASS_TIMEOUT_MS      (60000U)

/* ---- 时序 ---- */
#define LC_POLL_MS              (1U)
#define LC_EDGE_DEBOUNCE_MS     (3U)
#define LC_POS_CHECK_MS         (300U)
#define LC_POS_TIMEOUT_MS       (50U)
#define LC_POS_TIMEOUT_FAST_MS  (20U)     /* 原来 5ms 太紧, 凸点边沿经常读不到位置 */
#define LC_READ_RETRY           (3U)

/* ---- 参考位 / 三层判断 ---- */
#define LC_REF_ADDR             (0x0007E000UL)   /* 最后一个 8KB 扇区, 确认程序 < 0x7E000 */
#define LC_REF_MAGIC            (0x4C435231UL)   /* "LCR1" */
#define LC_REF_VERSION          (1U)
#define LC_REF_MATCH_PULSE      (3000)           /* 45mm: 跨次匹配容差 */
#define LC_MIN_VALID            (7U)             /* 有效点少于 7 个不补偿 */
#define LC_A_WARN_PULSE         (33)             /* 0.5mm */
#define LC_A_ALARM_PULSE        (133)            /* 2mm */
#define LC_B_WARN_PPM           (500)            /* 0.05% */
#define LC_B_ALARM_PPM          (1000)           /* 0.10% */
#define LC_R_TOL_PULSE          (47)             /* 0.7mm: 残差超过就剔除 */
#define LC_E_MAX_PULSE          (133)            /* 2mm: 单点误差上限 */
#define LC_COMP_MAX_PULSE       (67)             /* 1mm: 补偿限幅 */

/* ---- 结果码 ---- */
typedef enum {
    LC_RESULT_NONE       = 0U,
    LC_RESULT_ALL        = 1U,   /* 10 个点全部确认 */
    LC_RESULT_TIMEOUT    = 3U,
    LC_RESULT_POS_ERR    = 5U,
    LC_RESULT_HOME_FAIL  = 6U,
    LC_RESULT_INCOMPLETE = 9U
} lc_result_t;

/* ---- 检查结论 ---- */
#define LC_CHK_OK           (0U)   /* 正常 */
#define LC_CHK_WARN_ZERO    (1U)   /* 零点偏差, 已补偿 */
#define LC_CHK_WARN_BELT    (2U)   /* 皮带轻微拉伸, 已补偿 */
#define LC_CHK_ALARM_ZERO   (3U)   /* 零点异常, 不补偿 */
#define LC_CHK_ALARM_BELT   (4U)   /* 皮带明显松/跳齿, 不补偿 */
#define LC_CHK_MEAS_BAD     (5U)   /* 有效点不足/测量异常, 不补偿 */
#define LC_CHK_NO_REF       (0xFEU)/* 还没保存参考位 */

typedef struct {
    int32_t a;          /* 整体偏移(脉冲) */
    int32_t b_ppm;      /* 斜率(ppm, 1ppm=0.0001%) */
    int32_t max_r;      /* 最大残差(脉冲) */
    uint8_t valid_cnt;  /* 有效点数 */
    uint8_t decision;   /* LC_CHK_xxx */
} lc_check_t;

/* ---- 对外接口 ---- */
void        lc_init(void);
lc_result_t lc_scan_run(uint8_t do_home);   /* 扫描 + 自动对比参考位 */
void        lc_report_can(void);            /* 重发汇总/点位/检查结果 */
void        lc_report_check_can(void);      /* 只重发 AD 检查帧 */

uint8_t     lc_ref_save(void);              /* 把当前 10 点存为参考位(写 Flash) */
uint8_t     lc_ref_clear(void);             /* 清除参考位 */
uint8_t     lc_ref_valid(void);             /* 1=已有参考位 */
const lc_check_t *lc_get_check(void);       /* 上次检查结果 */
int32_t     lc_get_target(uint8_t idx);     /* 第 idx 点目标脉冲 = R + C */
int32_t     lc_comp_at(int32_t ref_pulse);  /* 任意参考位置的补偿量(线性插值) */

#define LC_GOTO_TIMEOUT_MS      (30000U)

uint8_t lc_set_base(void);                  /* 回零后调用: 记录当前 0 点 */
uint8_t lc_goto_mark(uint8_t idx);          /* 去第 idx 个标注点(自动带补偿) */
int32_t lc_get_base(void);

#endif /* __LC_H__ */
