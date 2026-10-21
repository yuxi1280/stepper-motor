#include "lc.h"
#include "can.h"

static lc_point_t  s_points[LC_POINT_MAX];
static uint8_t     s_count      = 0U;
static uint8_t     s_busy       = 0U;
static lc_result_t s_result     = LC_RESULT_NONE;
static int32_t     s_base_pulse = 0;

/* ================= PB12 ================= */
void lc_init(void)
{
    stc_gpio_init_t stcGpioInit;

    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDir  = PIN_DIR_IN;
    stcGpioInit.u16PullUp  = PIN_PU_ON;
    stcGpioInit.u16PinAttr = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(LC_SENSOR_PORT, LC_SENSOR_PIN, &stcGpioInit);
}

uint8_t lc_sensor_active(void)
{
#if (LC_SENSOR_ACTIVE_LOW != 0U)
    return (GPIO_ReadInputPins(LC_SENSOR_PORT, LC_SENSOR_PIN) == PIN_RESET) ? 1U : 0U;
#else
    return (GPIO_ReadInputPins(LC_SENSOR_PORT, LC_SENSOR_PIN) == PIN_SET) ? 1U : 0U;
#endif
}

uint8_t           lc_get_count(void)      { return s_count; }
const lc_point_t *lc_get_points(void)     { return s_points; }
lc_result_t       lc_get_result(void)     { return s_result; }
int32_t           lc_get_base_pulse(void) { return s_base_pulse; }

/* ================= 换算 ================= */
static int32_t lc_pulse_to_mm_x100(int32_t pulse)
{
    return (int32_t)(((int64_t)pulse * (int64_t)LC_MM_PER_REV * 100LL) /
                     (int64_t)MOTOR_PULSES_PER_REV);
}

/* ================= 读 0x32 ================= */
static uint8_t lc_read_pulse(int32_t *pulse, uint32_t timeout_ms, uint8_t retry)
{
    uint8_t i;
    int32_t v;

    for (i = 0U; i < retry; i++) {
        v = motor_read_input_pulses(timeout_ms);
        if (v != MOTOR_ERR_TIMEOUT) {
            *pulse = v;
            return 1U;
        }
        DDL_DelayMS(5U);
    }
    return 0U;
}

/* ================= CAN 上报 ================= */
static void lc_report_point(uint8_t idx)
{
    const lc_point_t *p;
    uint8_t  f[8];
    uint32_t v;

    if ((idx == 0U) || (idx > s_count)) {
        return;
    }
    p = &s_points[idx - 1U];

    /* A4 idx pos_mm_x100(4B) gap_mm_x100(2B) */
    v = (uint32_t)p->pos_mm_x100;
    f[0] = CAN_LC_RPT_POINT;
    f[1] = idx;
    f[2] = (uint8_t)(v >> 24);
    f[3] = (uint8_t)(v >> 16);
    f[4] = (uint8_t)(v >> 8);
    f[5] = (uint8_t)v;
    v = (uint32_t)p->gap_mm_x100;
    f[6] = (uint8_t)(v >> 8);
    f[7] = (uint8_t)v;
    can_tx_bytes(CAN_NODE_ID, f, 8U);
    DDL_DelayMS(2U);

    /* A5 idx rel_pulse(4B) */
    v = (uint32_t)p->pulse;
    f[0] = CAN_LC_RPT_PULSE;
    f[1] = idx;
    f[2] = (uint8_t)(v >> 24);
    f[3] = (uint8_t)(v >> 16);
    f[4] = (uint8_t)(v >> 8);
    f[5] = (uint8_t)v;
    can_tx_bytes(CAN_NODE_ID, f, 6U);
    DDL_DelayMS(2U);
}

void lc_report_summary(void)
{
    uint8_t  f[8];
    uint32_t v;

    /* A3 00 result count targetH targetL */
    f[0] = CAN_LC_RPT_SUM;
    f[1] = 0x00U;
    f[2] = (uint8_t)s_result;
    f[3] = s_count;
    f[4] = (uint8_t)(LC_TARGET_MM >> 8);
    f[5] = (uint8_t)(LC_TARGET_MM);
    can_tx_bytes(CAN_NODE_ID, f, 6U);
    DDL_DelayMS(2U);

    /* A6 base_pulse(4B) pulses_per_rev(2B) mm_per_rev(1B) */
    v = (uint32_t)s_base_pulse;
    f[0] = CAN_LC_RPT_INFO;
    f[1] = (uint8_t)(v >> 24);
    f[2] = (uint8_t)(v >> 16);
    f[3] = (uint8_t)(v >> 8);
    f[4] = (uint8_t)v;
    f[5] = (uint8_t)(MOTOR_PULSES_PER_REV >> 8);
    f[6] = (uint8_t)(MOTOR_PULSES_PER_REV);
    f[7] = (uint8_t)(LC_MM_PER_REV);
    can_tx_bytes(CAN_NODE_ID, f, 8U);
    DDL_DelayMS(2U);
}

void lc_report_can(void)
{
    uint8_t i;

    lc_report_summary();
    for (i = 0U; i < s_count; i++) {
        lc_report_point((uint8_t)(i + 1U));
    }
}

/* ================= 记录一个点 ================= */
static void lc_store_point(int32_t raw)
{
    lc_point_t *p;
    uint8_t idx;

    if (s_count >= LC_POINT_MAX) {
        return;
    }
    idx = s_count;
    p   = &s_points[idx];

    p->raw_pulse   = raw;
    p->pulse       = raw - s_base_pulse;
    p->pos_mm_x100 = lc_pulse_to_mm_x100(p->pulse);
    p->gap_mm_x100 = (idx == 0U) ? p->pos_mm_x100
                                 : (p->pos_mm_x100 - s_points[idx - 1U].pos_mm_x100);
    s_count = (uint8_t)(idx + 1U);

    lc_report_point((uint8_t)(idx + 1U));
}

/* ================= 运动辅助 =================*/
static void lc_move_forward(int32_t pulse)
{
    if (pulse <= 0) {
        return;
    }
    motor_move(0, (uint32_t)pulse, LC_SCAN_RPM, LC_SCAN_ACC, MOTOR_MODE_REL_CUR, 0U);
}

/* 急停后软恢复: 停 -> 清报警 -> 重新使能(不松轴) */
static void lc_recover(void)
{
    motor_stop();
    DDL_DelayMS(20U);
    motor_clear_alarm();
    DDL_DelayMS(20U);
    motor_enable(1U);
    DDL_DelayMS(30U);
}

/* ================= 扫描主体 ================= */
static lc_result_t lc_scan_body(uint8_t do_home)
{
    lc_result_t res = LC_RESULT_NONE;
    uint32_t t_ms    = 0U;
    uint32_t chk_ms  = 0U;
    int32_t  raw     = 0;
    int32_t  rel     = 0;
    int32_t  chk_pos;
    uint8_t  still   = 0U;
    uint8_t  retry   = 0U;
    uint8_t  last_act;
    uint8_t  act;
    uint16_t done_cnt;
    uint32_t err_cnt;

    /* 1. 回零 */
    if (do_home != 0U) {
        if (motor_limit_home() == 0U) {
            return LC_RESULT_HOME_FAIL;
        }
    }

    /* 2. 使能 + 记录起点脉冲 */
    motor_enable(1U);
    DDL_DelayMS(100U);
    motor_flush_rx();
    if (lc_read_pulse(&s_base_pulse, LC_POS_TIMEOUT_MS, LC_READ_RETRY) == 0U) {
        return LC_RESULT_POS_ERR;
    }

    /* 3. 相对当前点走 1500mm */
    last_act = lc_sensor_active();
    chk_pos  = s_base_pulse;
    done_cnt = g_motor_done_cnt;
    err_cnt  = g_motor_err_cnt;
    motor_flush_rx();
    lc_move_forward(LC_TARGET_PULSE);

    /* 4. 边走边测 */
    for (;;) {
        DDL_DelayMS(LC_POLL_MS);
        t_ms += LC_POLL_MS;

        /* 4.1 凸点进入(无 -> 有) */
        act = lc_sensor_active();
        if ((last_act == 0U) && (act != 0U)) {
            DDL_DelayMS(1U);
            if (lc_sensor_active() != 0U) {
                motor_stop();
                DDL_DelayMS(LC_STOP_SETTLE_MS);
                t_ms += LC_STOP_SETTLE_MS;

                if (lc_read_pulse(&raw, LC_POS_TIMEOUT_MS, LC_READ_RETRY) == 0U) {
                    res = LC_RESULT_POS_ERR;
                    break;
                }
                lc_store_point(raw);

                last_act = 1U;
                rel      = raw - s_base_pulse;
                chk_pos  = raw;
                still    = 0U;
                retry    = 0U;

                if (s_count >= LC_POINT_MAX) {
                    res = LC_RESULT_ALL;
                    break;
                }
                if (rel >= (LC_TARGET_PULSE - LC_ARRIVE_TOL_PULSE)) {
                    res = LC_RESULT_ARRIVED;
                    break;
                }

                lc_recover();

                done_cnt = g_motor_done_cnt;
                err_cnt  = g_motor_err_cnt;
                motor_flush_rx();
                lc_move_forward(LC_TARGET_PULSE - rel);   /* 只走剩余距离 */
                continue;
            }
            act = 0U;
        }
        last_act = act;

        /* 4.2 主动上报: 9F=到位, E2/EE=报错 */
        motor_rx_task();
        if (g_motor_done_cnt != done_cnt) { res = LC_RESULT_ARRIVED; break; }
        if (g_motor_err_cnt != err_cnt)   { res = LC_RESULT_DRV_ERR;  break; }

        /* 4.3 总超时 */
        if (t_ms >= LC_TOTAL_TIMEOUT_MS) { res = LC_RESULT_TIMEOUT; break; }

        /* 4.4 兜底: 每 500ms 读位置, 判到位/卡住, 卡住自动重发两次 */
        chk_ms += LC_POLL_MS;
        if (chk_ms >= LC_BACKUP_CHECK_MS) {
            chk_ms = 0U;
            if (lc_read_pulse(&raw, LC_POS_TIMEOUT_FAST_MS, 1U) != 0U) {
                rel = raw - s_base_pulse;
                if (rel >= (LC_TARGET_PULSE - LC_ARRIVE_TOL_PULSE)) {
                    res = LC_RESULT_ARRIVED;
                    break;
                }
                if (raw > chk_pos) {
                    chk_pos = raw;
                    still   = 0U;
                } else {
                    still++;
                    if ((still >= 3U) && (retry < 2U)) {
                        retry++;
                        still = 0U;
                        lc_recover();
                        motor_flush_rx();
                        lc_move_forward(LC_TARGET_PULSE - rel);
                    } else if (still >= LC_STALL_CNT) {
                        res = LC_RESULT_STALL;
                        break;
                    }
                }
            }
        }
    }

    motor_stop();
    return res;
}

/* ================= 对外入口 ================= */
lc_result_t lc_scan_run(uint8_t do_home)
{
    if (s_busy != 0U) {
        return LC_RESULT_BUSY;
    }
    s_busy       = 1U;
    s_count      = 0U;
    s_base_pulse = 0;
    s_result     = LC_RESULT_NONE;

    s_result = lc_scan_body(do_home);

    s_busy = 0U;
    lc_report_summary();      /* 点已实时上报, 这里只补汇总 */
    return s_result;
}
