#include "lc.h"
#include "can.h"

/* ================= 数据 ================= */
typedef struct {
    int32_t s[LC_SAMPLE_MAX];   /* 同一凸点的各次测量(相对 0 点脉冲) */
    uint8_t n;                  /* 已有样本数 */
} lc_slot_t;

typedef struct {
    int32_t pulse;              /* 平均值(相对 0 点) */
    int32_t pos_mm_x100;
    int32_t gap_mm_x100;
} lc_point_t;

static lc_slot_t   s_slots[LC_SLOT_MAX];
static uint8_t     s_slot_cnt = 0U;
static lc_point_t  s_points[LC_POINT_MAX];
static uint8_t     s_count    = 0U;
static lc_result_t s_result   = LC_RESULT_NONE;
static int32_t     s_base_pulse = 0;

/* ================= 小工具 ================= */
static void lc_put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

static int32_t lc_pulse_to_mm_x100(int32_t pulse)
{
    return (int32_t)(((int64_t)pulse * (int64_t)LC_MM_PER_REV * 100LL) /
                     (int64_t)MOTOR_PULSES_PER_REV);
}

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

/* ================= 传感器 ================= */
void lc_init(void)
{
    stc_gpio_init_t stcGpioInit;

    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDir  = PIN_DIR_IN;
    stcGpioInit.u16PullUp  = PIN_PU_ON;
    stcGpioInit.u16PinAttr = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(LC_SENSOR_PORT, LC_SENSOR_PIN, &stcGpioInit);
}

static uint8_t lc_sensor_active(void)
{
#if (LC_SENSOR_ACTIVE_LOW != 0U)
    return (GPIO_ReadInputPins(LC_SENSOR_PORT, LC_SENSOR_PIN) == PIN_RESET) ? 1U : 0U;
#else
    return (GPIO_ReadInputPins(LC_SENSOR_PORT, LC_SENSOR_PIN) == PIN_SET) ? 1U : 0U;
#endif
}

/* ================= 上报 ================= */
static void lc_report_sample(uint8_t pass, uint8_t slot, uint8_t n, int32_t pulse)
{
    uint8_t f[8];

    f[0] = CAN_LC_RPT_SAMPLE;
    f[1] = pass;
    f[2] = slot;
    f[3] = n;
    lc_put_u32(&f[4], (uint32_t)pulse);
    can_tx_bytes(CAN_NODE_ID, f, 8U);
}

/* ================= 槽位 ================= */
/* 找最大一致簇: 簇内 >=2 个样本且相互差 <= LC_AGREE_TOL_PULSE */
static uint8_t lc_slot_eval(uint8_t i, int32_t *avg)
{
    const lc_slot_t *p = &s_slots[i];
    uint8_t a, b, cnt, best_cnt = 0U;
    int32_t sum, best_sum = 0;

    for (a = 0U; a < p->n; a++) {
        cnt = 0U;
        sum = 0;
        for (b = 0U; b < p->n; b++) {
            int32_t d = p->s[b] - p->s[a];
            if (d < 0) {
                d = -d;
            }
            if (d <= LC_AGREE_TOL_PULSE) {
                cnt++;
                sum += p->s[b];
            }
        }
        if (cnt > best_cnt) {
            best_cnt = cnt;
            best_sum = sum;
        }
    }
    if (best_cnt >= 2U) {
        *avg = best_sum / (int32_t)best_cnt;
        return 1U;
    }
    return 0U;
}

/* 槽位代表值: 已确认取簇平均, 否则取全部样本平均 */
static int32_t lc_slot_value(uint8_t i)
{
    const lc_slot_t *p = &s_slots[i];
    int32_t v = 0;
    uint8_t k;

    if (lc_slot_eval(i, &v) != 0U) {
        return v;
    }
    if (p->n == 0U) {
        return 0;
    }
    for (k = 0U; k < p->n; k++) {
        v += p->s[k];
    }
    return v / (int32_t)p->n;
}

/* 把一次检测并入最近的槽位, 没有就新建 */
static void lc_slot_add(int32_t pulse, uint8_t pass)
{
    uint8_t i, best = 0xFFU;
    int32_t d, best_d = LC_MATCH_TOL_PULSE;

    for (i = 0U; i < s_slot_cnt; i++) {
        d = pulse - lc_slot_value(i);
        if (d < 0) {
            d = -d;
        }
        if (d < best_d) {
            best_d = d;
            best   = i;
        }
    }
    if (best == 0xFFU) {
        if (s_slot_cnt >= LC_SLOT_MAX) {
            return;                          /* 槽位满, 丢弃 */
        }
        best = s_slot_cnt++;
        s_slots[best].n = 0U;
    }

    if (s_slots[best].n < LC_SAMPLE_MAX) {
        s_slots[best].s[s_slots[best].n++] = pulse;
    } else {
        for (i = 1U; i < LC_SAMPLE_MAX; i++) {
            s_slots[best].s[i - 1U] = s_slots[best].s[i];
        }
        s_slots[best].s[LC_SAMPLE_MAX - 1U] = pulse;
    }

    lc_report_sample(pass, best, s_slots[best].n, pulse);
}

static uint8_t lc_confirmed_count(void)
{
    uint8_t i, n = 0U;
    int32_t v;

    for (i = 0U; i < s_slot_cnt; i++) {
        if (lc_slot_eval(i, &v) != 0U) {
            n++;
        }
    }
    return n;
}

/* ================= 运动 ================= */
static void lc_move_forward(int32_t pulse)
{
    if (pulse <= 0) {
        return;
    }
    if (pulse > LC_TARGET_PULSE) {
        pulse = LC_TARGET_PULSE;
    }
    motor_move(0, (uint32_t)pulse, LC_SCAN_RPM, LC_SCAN_ACC, MOTOR_MODE_REL_CUR, 0U);
}

/* 相对运动回到 0 点, 有界 */
static uint8_t lc_return_zero(void)
{
    int32_t cur, delta;
    uint32_t t = 0U;

    if (lc_read_pulse(&cur, LC_POS_TIMEOUT_MS, LC_READ_RETRY) == 0U) {
        return 0U;
    }
    delta = cur - s_base_pulse;
    if ((delta > (LC_TARGET_PULSE * 2)) || (delta < -(LC_TARGET_PULSE * 2))) {
        return 0U;                           /* 位置异常, 拒绝移动 */
    }
    if (delta > LC_ZERO_TOL_PULSE) {
        motor_move(1, (uint32_t)delta, LC_RETURN_RPM, LC_SCAN_ACC, MOTOR_MODE_REL_CUR, 0U);
    } else if (delta < -LC_ZERO_TOL_PULSE) {
        motor_move(0, (uint32_t)(-delta), LC_RETURN_RPM, LC_SCAN_ACC, MOTOR_MODE_REL_CUR, 0U);
    } else {
        return 1U;
    }

    while (t < LC_RETURN_TIMEOUT_MS) {
        DDL_DelayMS(20U);
        t += 20U;
        if (lc_read_pulse(&cur, LC_POS_TIMEOUT_FAST_MS, 1U) != 0U) {
            delta = cur - s_base_pulse;
            if ((delta <= LC_ZERO_TOL_PULSE) && (delta >= -LC_ZERO_TOL_PULSE)) {
                return 1U;
            }
        }
    }

    motor_stop();
    motor_stop();
    motor_stop();
    return 0U;
}

/* ================= 单轮扫描 ================= */
static uint8_t lc_scan_pass(uint8_t pass)
{
    uint32_t t_ms   = 0U;
    uint32_t chk_ms = 0U;
    int32_t  raw    = 0;
    uint8_t  last_act;
    uint8_t  act;

    last_act = lc_sensor_active();
    motor_flush_rx();
    lc_move_forward(LC_TARGET_PULSE);

    for (;;) {
        DDL_DelayMS(LC_POLL_MS);
        t_ms += LC_POLL_MS;

        /* 凸点进入: 不停车, 直接读位置 */
        act = lc_sensor_active();
        if ((last_act == 0U) && (act != 0U)) {
            DDL_DelayMS(LC_EDGE_DEBOUNCE_MS);
            if (lc_sensor_active() != 0U) {
                if (lc_read_pulse(&raw, LC_POS_TIMEOUT_FAST_MS, 2U) != 0U) {
                    lc_slot_add(raw - s_base_pulse, pass);
                }
                last_act = 1U;
                continue;
            }
            act = 0U;
        }
        last_act = act;

        /* 单轮超时 */
        if (t_ms >= LC_PASS_TIMEOUT_MS) {
            motor_stop();
            motor_stop();
            motor_stop();
            return 0U;
        }

        /* 每 300ms 看一次是否到位 */
        chk_ms += LC_POLL_MS;
        if (chk_ms >= LC_POS_CHECK_MS) {
            chk_ms = 0U;
            if (lc_read_pulse(&raw, LC_POS_TIMEOUT_FAST_MS, 1U) != 0U) {
                if ((raw - s_base_pulse) >= (LC_TARGET_PULSE - LC_ARRIVE_TOL_PULSE)) {
                    break;
                }
            }
        }
    }

    motor_stop();
    motor_stop();
    motor_stop();
    return 1U;
}

/* ================= 汇总 ================= */
static void lc_build_points(void)
{
    uint8_t order[LC_SLOT_MAX];
    uint8_t n = 0U, i, j;
    int32_t v;

    for (i = 0U; i < s_slot_cnt; i++) {
        order[n++] = i;
    }
    /* 按代表位置插入排序 */
    for (i = 1U; i < n; i++) {
        uint8_t cur = order[i];
        int32_t kv  = lc_slot_value(cur);

        j = i;
        while ((j > 0U) && (lc_slot_value(order[j - 1U]) > kv)) {
            order[j] = order[j - 1U];
            j--;
        }
        order[j] = cur;
    }

    s_count = 0U;
    for (i = 0U; (i < n) && (s_count < LC_POINT_MAX); i++) {
        v = lc_slot_value(order[i]);
        s_points[s_count].pulse       = v;
        s_points[s_count].pos_mm_x100 = lc_pulse_to_mm_x100(v);
        if (s_count == 0U) {
            s_points[0].gap_mm_x100 = s_points[0].pos_mm_x100;
        } else {
            s_points[s_count].gap_mm_x100 =
                s_points[s_count].pos_mm_x100 - s_points[s_count - 1U].pos_mm_x100;
        }
        s_count++;
    }
}

static void lc_report_final_point(uint8_t idx)
{
    const lc_point_t *p;
    uint8_t f[8];

    if ((idx == 0U) || (idx > s_count)) {
        return;
    }
    p = &s_points[idx - 1U];

    f[0] = CAN_LC_RPT_POINT;
    f[1] = idx;
    lc_put_u32(&f[2], (uint32_t)p->pos_mm_x100);
    f[6] = (uint8_t)((uint32_t)p->gap_mm_x100 >> 8);
    f[7] = (uint8_t)p->gap_mm_x100;
    can_tx_bytes(CAN_NODE_ID, f, 8U);
    DDL_DelayMS(2U);

    f[0] = CAN_LC_RPT_PULSE;
    f[1] = idx;
    lc_put_u32(&f[2], (uint32_t)p->pulse);
    can_tx_bytes(CAN_NODE_ID, f, 6U);
    DDL_DelayMS(2U);
}

static void lc_report_summary(void)
{
    uint8_t f[8];

    /* A3 00 result count targetH targetL (f[1] = 已确认点数) */
    f[0] = CAN_LC_RPT_SUM;
    f[1] = lc_confirmed_count();
    f[2] = (uint8_t)s_result;
    f[3] = s_count;
    f[4] = (uint8_t)(LC_TARGET_MM >> 8);
    f[5] = (uint8_t)(LC_TARGET_MM);
    can_tx_bytes(CAN_NODE_ID, f, 6U);
    DDL_DelayMS(2U);

    /* A6 base_pulse(4B) ppr(2B) mm_per_rev(1B) */
    f[0] = CAN_LC_RPT_INFO;
    lc_put_u32(&f[1], (uint32_t)s_base_pulse);
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
        lc_report_final_point((uint8_t)(i + 1U));
    }
}

/* ================= 多轮主流程 ================= */
static lc_result_t lc_scan_all(uint8_t do_home)
{
    lc_result_t res = LC_RESULT_NONE;
    uint8_t pass;

    if (do_home != 0U) {
        if (motor_limit_home() == 0U) {
            return LC_RESULT_HOME_FAIL;
        }
    }

    motor_enable(1U);
    DDL_DelayMS(100U);
    motor_flush_rx();
    if (lc_read_pulse(&s_base_pulse, LC_POS_TIMEOUT_MS, LC_READ_RETRY) == 0U) {
        return LC_RESULT_POS_ERR;
    }

    for (pass = 0U; pass < LC_PASS_MAX; pass++) {
        if (pass > 0U) {
            if (lc_return_zero() == 0U) {
                res = LC_RESULT_TIMEOUT;
                break;
            }
        }
        if (lc_scan_pass(pass) == 0U) {
            res = LC_RESULT_TIMEOUT;
            break;
        }
        if (lc_confirmed_count() >= LC_POINT_MAX) {
            res = LC_RESULT_ALL;
            break;
        }
    }

    return (res == LC_RESULT_NONE) ? LC_RESULT_INCOMPLETE : res;
}

lc_result_t lc_scan_run(uint8_t do_home)
{
    uint8_t i;

    s_slot_cnt   = 0U;
    s_count      = 0U;
    s_base_pulse = 0;
    s_result     = LC_RESULT_NONE;
    for (i = 0U; i < LC_SLOT_MAX; i++) {
        s_slots[i].n = 0U;
    }

    s_result = lc_scan_all(do_home);

    lc_build_points();
    lc_report_summary();
    for (i = 0U; i < s_count; i++) {
        lc_report_final_point((uint8_t)(i + 1U));
    }
    return s_result;
}
