#include "lc.h"
#include "can.h"
#include <string.h>
#include <stddef.h>

/* ================= 类型 ================= */
typedef struct {
    int32_t s[LC_SAMPLE_MAX];
    uint8_t n;
} lc_slot_t;

typedef struct {
    int32_t pulse;
    int32_t pos_mm_x100;
    int32_t gap_mm_x100;
    uint8_t ok;                 /* 1=已确认(>=2个一致样本) */
} lc_point_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t count;
    int32_t  ref[LC_POINT_MAX];
    uint32_t crc32;
} lc_ref_t;

/* ================= 状态 ================= */
static lc_slot_t   s_slots[LC_SLOT_MAX];
static uint8_t     s_slot_cnt = 0U;
static lc_point_t  s_points[LC_POINT_MAX];
static uint8_t     s_count    = 0U;
static lc_result_t s_result   = LC_RESULT_NONE;
static int32_t     s_base_pulse = 0;
static int32_t     s_zero_pulse = 0;   /* 回零后读到的原点读数(扫描起点的判据) */
static uint8_t     s_pass_edges = 0U;  /* 本轮检出凸点的上升沿个数(诊断用) */

static lc_ref_t    s_ref;
static uint8_t     s_ref_valid = 0U;
static uint8_t     s_ref_tried = 0U;
static int32_t     s_comp[LC_POINT_MAX];
static lc_check_t  s_check;

/* 前置声明: lc_confirmed_count() 在槽位函数之前使用 */
static uint8_t lc_slot_eval(uint8_t i, int32_t *avg);
static int32_t lc_slot_value(uint8_t i);
static uint8_t lc_check_ref(const lc_ref_t *ref, lc_check_t *out);


/* ================= 小工具 ================= */
/* 已确认(>=2 个一致样本)的凸点数 */
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

static int32_t lc_abs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

static int32_t lc_pulse_to_mm_x100(int32_t pulse)
{
    return (int32_t)(((int64_t)pulse * (int64_t)LC_MM_PER_REV * 100LL) /
                     (int64_t)MOTOR_PULSES_PER_REV);
}

static uint32_t lc_crc32(const uint8_t *p, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i, j;

    for (i = 0U; i < len; i++) {
        crc ^= p[i];
        for (j = 0U; j < 8U; j++) {
            crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1UL)));
        }
    }
    return ~crc;
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

/* ================= 上报 =================
 * 补偿量统一由 can_report_comp() 按协议 0x8A 上报(见 can.c / CANv3.txt)。
 * 旧的 A3~AF 私有帧与逐点/采样数据流已废弃, 凸点数据由设备本地保存。
 */
void lc_report_check_can(void)
{
    can_report_comp();
}

void lc_report_can(void)
{
    can_report_comp();
}

/* ================= 槽位 ================= */
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

static void lc_slot_add(int32_t pulse, uint8_t pass)
{
    uint8_t i, best = 0xFFU;
    int32_t d, best_d = LC_MATCH_TOL_PULSE;

    for (i = 0U; i < s_slot_cnt; i++) {
        d = lc_abs32(pulse - lc_slot_value(i));
        if (d < best_d) {
            best_d = d;
            best   = i;
        }
    }
    if (best == 0xFFU) {
        if (s_slot_cnt >= LC_SLOT_MAX) {
            return;
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

    (void)pass;             /* 采样明细不再上传, 设备本地汇总 */
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

/* 限位开关是否触发(低电平=触发) */
static uint8_t lc_limit_active(void)
{
    return (GPIO_ReadInputPins(MOTOR_LIMIT_PORT, MOTOR_LIMIT_PIN) == PIN_RESET) ? 1U : 0U;
}

/* 回 0 位: 相对退回 s_base_pulse, 并以"撞到限位开关"作为最终的到 0 判据 */
static uint8_t lc_return_zero(void)
{
    int32_t cur, delta;
    uint32_t t = 0U;

    can_report_event(CANP_EV_RETURN0, 0U, 0U);

    /* 回 0 前先"解锁"驱动: 扫描末期如果顶到机械末端, 驱动会锁存堵转保护,
     * 之后的运动指令全部被忽略 -> 回 0 一直不动 -> 30 秒超时。
     * 这是"扫描到最远端之后卡很久才回零 / 干脆回不去"的直接原因。
     */
    motor_clear_alarm();
    DDL_DelayMS(20U);
    motor_enable(1);
    DDL_DelayMS(50U);

    motor_flush_rx();                      /* 丢掉上一轮 motor_stop 的残留应答 */
    DDL_DelayMS(50U);

    if (lc_read_pulse(&cur, LC_POS_TIMEOUT_MS, LC_READ_RETRY) == 0U) {
        /* 读不到位置: 直接反向走满一个行程, 靠限位开关停下(和回零一样可靠) */
        motor_move(1, (uint32_t)LC_TARGET_PULSE, LC_RETURN_RPM, LC_SCAN_ACC,
                   MOTOR_MODE_REL_CUR, 0U);
        can_report_event(CANP_EV_RETURN0, 4U, 0U);
    } else {
        delta = cur - s_base_pulse;
        if ((delta <= LC_ZERO_TOL_PULSE) && (delta >= -LC_ZERO_TOL_PULSE)) {
            can_report_event(CANP_EV_RETURN0, 1U, 0U);
            return 1U;
        }
        if ((delta < (LC_TARGET_PULSE * 2)) && (delta > -(LC_TARGET_PULSE * 2))) {
            if (delta > 0) {
                motor_move(1, (uint32_t)delta, LC_RETURN_RPM, LC_SCAN_ACC, MOTOR_MODE_REL_CUR, 0U);
            } else {
                motor_move(0, (uint32_t)(-delta), LC_RETURN_RPM, LC_SCAN_ACC, MOTOR_MODE_REL_CUR, 0U);
            }
        } else {
            /* 位移离谱: 同样按整行程反向退回 */
            motor_move(1, (uint32_t)LC_TARGET_PULSE, LC_RETURN_RPM, LC_SCAN_ACC,
                       MOTOR_MODE_REL_CUR, 0U);
            can_report_event(CANP_EV_RETURN0, 5U, 0U);
        }
    }

    while (t < LC_RETURN_TIMEOUT_MS) {
        app_service();                      /* 回零最长 30 秒, 期间照样应答上位机 */
        DDL_DelayMS(20U);
        t += 20U;

        /* 撞到限位开关 = 物理 0 点, 直接以此为准(最可靠) */
        if (lc_limit_active() != 0U) {
            motor_stop();
            DDL_DelayMS(30U);
            motor_clear_alarm();          /* 0x0E 之后驱动可能被失能, 必须重新使能 */
            DDL_DelayMS(30U);
            motor_enable(1);
            DDL_DelayMS(50U);
            motor_zero_pos();
            DDL_DelayMS(30U);
            s_base_pulse = 0;
            s_zero_pulse = 0;
            can_report_event(CANP_EV_RETURN0, 2U, 0U);
            return 1U;
        }
        /* 位置读数: 用 50ms 超时(原来 5ms 太紧, 经常读不到导致一直等) */
        if (lc_read_pulse(&cur, LC_POS_TIMEOUT_MS, 1U) != 0U) {
            delta = cur - s_base_pulse;
            if ((delta <= LC_ZERO_TOL_PULSE) && (delta >= -LC_ZERO_TOL_PULSE)) {
                can_report_event(CANP_EV_RETURN0, 1U, 0U);
                return 1U;
            }
        }
    }

    motor_stop();
    motor_stop();
    motor_stop();
    can_report_event(CANP_EV_RETURN0, 3U, 0U);
    return 0U;
}

static uint8_t lc_scan_pass(uint8_t pass)
{
    uint32_t t_ms   = 0U;
    uint32_t chk_ms = 0U;
    int32_t  raw    = 0;
    uint8_t  last_act;
    uint8_t  act;

    s_pass_edges = 0U;
    last_act = lc_sensor_active();
    /* RESULT = 本轮开始时传感器电平(诊断用), DETAIL = 轮次 */
    can_report_event(CANP_EV_PASS_START, last_act, pass);

    /* 每轮开始前先把驱动"解锁": 只发 enable 是不够的,
     * 堵转/过流保护一旦锁存, enable 会被忽略 -> 指令发下去电机不动
     * (表现就是"标定只接收不动作")。
     */
    motor_clear_alarm();
    DDL_DelayMS(20U);
    motor_enable(1);
    DDL_DelayMS(50U);
    motor_flush_rx();
    lc_move_forward(LC_TARGET_PULSE);

    for (;;) {
        app_service();                      /* 扫描 20 多秒, 期间照样应答上位机 */
        DDL_DelayMS(LC_POLL_MS);
        t_ms += LC_POLL_MS;

        act = lc_sensor_active();
        if ((last_act == 0U) && (act != 0U)) {
            DDL_DelayMS(LC_EDGE_DEBOUNCE_MS);
            if (lc_sensor_active() != 0U) {
                s_pass_edges++;
                if (lc_read_pulse(&raw, LC_POS_TIMEOUT_FAST_MS, 2U) != 0U) {
                    lc_slot_add(raw - s_base_pulse, pass);
                }
                last_act = 1U;
                continue;
            }
            act = 0U;
        }
        last_act = act;

        if (t_ms >= LC_PASS_TIMEOUT_MS) {
            motor_stop();
            motor_stop();
            motor_stop();
            can_report_event(CANP_EV_PASS_DONE, 0xFFU, s_pass_edges);
            return 0U;
        }

        chk_ms += LC_POLL_MS;
        if (chk_ms >= LC_POS_CHECK_MS) {
            chk_ms = 0U;
            if (lc_read_pulse(&raw, LC_POS_TIMEOUT_FAST_MS, 1U) != 0U) {
                if ((raw - s_base_pulse) >= (LC_TARGET_PULSE - LC_ARRIVE_TOL_PULSE)) {
                    break;
                }
                /* 超程保护: 冲过目标 20mm 立刻停, 防止位置读失败时一路冲到底 */
                if ((raw - s_base_pulse) > (LC_TARGET_PULSE + (LC_ARRIVE_TOL_PULSE * 10))) {
                    motor_stop();
                    motor_stop();
                    can_report_event(CANP_EV_PASS_DONE, 0xFEU, s_pass_edges);
                    return 0U;
                }
            }
        }
    }

    motor_stop();
    motor_stop();
    motor_stop();
    can_report_event(CANP_EV_PASS_DONE, (uint8_t)lc_confirmed_count(), s_pass_edges);
    return 1U;
}

/* ================= 汇总点位 ================= */
/* 按位置升序排列下标数组(插入排序, 最多 12 个) */
static void lc_sort_idx(uint8_t *idx, uint8_t n)
{
    uint8_t i, j, cur;
    int32_t kv;

    for (i = 1U; i < n; i++) {
        cur = idx[i];
        kv  = lc_slot_value(cur);
        j   = i;
        while ((j > 0U) && (lc_slot_value(idx[j - 1U]) > kv)) {
            idx[j] = idx[j - 1U];
            j--;
        }
        idx[j] = cur;
    }
}

/* 追加一个点(带与上一点的间距) */
static void lc_point_store(int32_t pulse, uint8_t ok)
{
    if (s_count >= LC_POINT_MAX) {
        return;
    }
    s_points[s_count].pulse       = pulse;
    s_points[s_count].pos_mm_x100 = lc_pulse_to_mm_x100(pulse);
    s_points[s_count].ok          = ok;
    if (s_count == 0U) {
        s_points[0].gap_mm_x100 = s_points[0].pos_mm_x100;
    } else {
        s_points[s_count].gap_mm_x100 =
            s_points[s_count].pos_mm_x100 - s_points[s_count - 1U].pos_mm_x100;
    }
    s_count++;
}

/* 已确认(>=2 个一致样本)的凸点优先占满 10 个参考位, 不足才用未确认的补。
 * 原实现是"按位置取前 10 个", 一旦多测出几个虚假凸点, 前 10 个里混进
 * 未确认点 -> lc_ref_save() 直接返回 0 -> 每次开机器都重新标定。
 */
static void lc_build_points(void)
{
    uint8_t conf[LC_SLOT_MAX];
    uint8_t rest[LC_SLOT_MAX];
    uint8_t nc = 0U, nr = 0U, i;
    int32_t v;

    for (i = 0U; i < s_slot_cnt; i++) {
        if (lc_slot_eval(i, &v) != 0U) {
            conf[nc++] = i;
        } else {
            rest[nr++] = i;
        }
    }
    lc_sort_idx(conf, nc);
    lc_sort_idx(rest, nr);

    s_count = 0U;
    for (i = 0U; (i < nc) && (s_count < LC_POINT_MAX); i++) {
        (void)lc_slot_eval(conf[i], &v);
        lc_point_store(v, 1U);
    }
    for (i = 0U; (i < nr) && (s_count < LC_POINT_MAX); i++) {
        lc_point_store(lc_slot_value(rest[i]), 0U);
    }
}

/* ================= 参考位(Flash) ================= */
static uint8_t lc_ref_load(void)
{
    memcpy(&s_ref, (const void *)LC_REF_ADDR, sizeof(s_ref));
    s_ref_tried = 1U;
    s_ref_valid = 0U;

    if (s_ref.magic != LC_REF_MAGIC) {
        return 0U;
    }
    if (s_ref.version != LC_REF_VERSION) {
        return 0U;
    }
    if ((s_ref.count < LC_MIN_VALID) || (s_ref.count > LC_POINT_MAX)) {
        return 0U;
    }
    if (lc_crc32((const uint8_t *)&s_ref, (uint32_t)offsetof(lc_ref_t, crc32)) != s_ref.crc32) {
        return 0U;
    }
    s_ref_valid = 1U;
    return 1U;
}

static uint8_t lc_ref_ensure(void)
{
    if (s_ref_tried == 0U) {
        (void)lc_ref_load();
    }
    return s_ref_valid;
}

uint8_t lc_ref_save(void)
{
    lc_ref_t r;
    uint8_t buf[sizeof(lc_ref_t)];
    uint32_t primask;
    int32_t ret;
    uint8_t i;
    uint8_t n = 0U;

    if ((s_count < LC_MIN_VALID) || (s_result != LC_RESULT_ALL)) {
        return 0U;
    }
    /* lc_build_points() 已把"已确认"的点排在前面, 只存这些 */
    for (i = 0U; (i < s_count) && (i < LC_POINT_MAX); i++) {
        if (s_points[i].ok == 0U) {
            break;
        }
        r.ref[n] = s_points[i].pulse;
        n++;
    }
    if (n < LC_MIN_VALID) {
        return 0U;
    }

    r.magic   = LC_REF_MAGIC;
    r.version = LC_REF_VERSION;
    r.count   = n;
    r.crc32   = lc_crc32((const uint8_t *)&r, (uint32_t)offsetof(lc_ref_t, crc32));
    memcpy(buf, &r, sizeof(r));

    primask = __get_PRIMASK();
    __disable_irq();
    EFM_REG_Unlock();
    EFM_FWMC_Cmd(ENABLE);                 /* ← 关键：打开 FWMC 写模式 */
    ret = EFM_SectorErase(LC_REF_ADDR);
    if (ret == LL_OK) {
        ret = EFM_Program(LC_REF_ADDR, buf, (uint32_t)sizeof(buf));
    }
    EFM_FWMC_Cmd(DISABLE);                /* 关闭 */
    EFM_REG_Lock();
    __set_PRIMASK(primask);


    if (ret != LL_OK) {
        return 0U;
    }

    s_ref_valid = 0U;
    if (lc_ref_load() == 0U) {
        return 0U;
    }
    for (i = 0U; i < LC_POINT_MAX; i++) {
        s_comp[i] = 0;
    }
    /* 用刚存的参考位算一次判定, 让上位机看到有效的补偿帧(而不是"测量无效") */
    (void)lc_check_ref(&s_ref, &s_check);
    return 1U;
}

uint8_t lc_ref_clear(void)
{
    uint32_t primask;
    int32_t ret;
    uint8_t i;

    primask = __get_PRIMASK();
    __disable_irq();
    EFM_REG_Unlock();
    EFM_FWMC_Cmd(ENABLE);
    ret = EFM_SectorErase(LC_REF_ADDR);
    EFM_FWMC_Cmd(DISABLE);
    EFM_REG_Lock();
    __set_PRIMASK(primask);


    s_ref_valid = 0U;
    s_ref_tried = 1U;
    for (i = 0U; i < LC_POINT_MAX; i++) {
        s_comp[i] = 0;
    }
    return (ret == LL_OK) ? 1U : 0U;
}

uint8_t lc_ref_valid(void)
{
    return lc_ref_ensure();
}

const lc_check_t *lc_get_check(void)
{
    return &s_check;
}

/* ================= 三层判断 ================= */
static uint8_t lc_fit(const lc_ref_t *ref, const int32_t *e, const uint8_t *ok,
                      uint8_t n, int32_t *a, int32_t *b_ppm, int32_t *max_r)
{
    int64_t sx = 0, se = 0, sxx = 0, sxe = 0;
    int32_t mx, me;
    uint8_t i;

    if (n < 3U) {
        return 0U;
    }
    for (i = 0U; i < ref->count; i++) {
        if (ok[i] == 0U) {
            continue;
        }
        sx += ref->ref[i];
        se += e[i];
    }
    mx = (int32_t)(sx / n);
    me = (int32_t)(se / n);

    for (i = 0U; i < ref->count; i++) {
        int64_t dx, de;
        if (ok[i] == 0U) {
            continue;
        }
        dx = (int64_t)ref->ref[i] - mx;
        de = (int64_t)e[i] - me;
        sxx += dx * dx;
        sxe += dx * de;
    }
    if (sxx == 0) {
        return 0U;
    }

    *b_ppm = (int32_t)(sxe * 1000000LL / sxx);
    *a     = (int32_t)((int64_t)me - (int64_t)(*b_ppm) * mx / 1000000LL);

    *max_r = 0;
    for (i = 0U; i < ref->count; i++) {
        int32_t pred, r;
        if (ok[i] == 0U) {
            continue;
        }
        pred = *a + (int32_t)((int64_t)(*b_ppm) * ref->ref[i] / 1000000LL);
        r = lc_abs32(e[i] - pred);
        if (r > *max_r) {
            *max_r = r;
        }
    }
    return 1U;
}

static uint8_t lc_check_ref(const lc_ref_t *ref, lc_check_t *out)
{
    int32_t e[LC_POINT_MAX];
    uint8_t ok[LC_POINT_MAX];
    uint8_t i, j, n = 0U, outliers = 0U;
    int32_t a = 0, b_ppm = 0, max_r = 0;

    out->a = 0;
    out->b_ppm = 0;
    out->max_r = 0;
    out->valid_cnt = 0U;
    out->decision = LC_CHK_MEAS_BAD;

    for (i = 0U; i < LC_POINT_MAX; i++) {
        ok[i] = 0U;
        e[i] = 0;
    }

    /* 第一层: 每个参考点找最近的实测点 */
    for (i = 0U; i < ref->count; i++) {
        int32_t best = LC_REF_MATCH_PULSE;
        uint8_t best_j = 0xFFU;

        for (j = 0U; j < s_count; j++) {
            int32_t d = lc_abs32(s_points[j].pulse - ref->ref[i]);
            if (d < best) {
                best = d;
                best_j = j;
            }
        }
        if ((best_j == 0xFFU) || (s_points[best_j].ok == 0U)) {
            continue;
        }
        e[i] = s_points[best_j].pulse - ref->ref[i];
        ok[i] = 1U;
        n++;
    }
    out->valid_cnt = n;
    if (n < LC_MIN_VALID) {
        return out->decision;
    }

    /* 第二层: 初拟合 */
    if (lc_fit(ref, e, ok, n, &a, &b_ppm, &max_r) == 0U) {
        return out->decision;
    }

    /* 剔除残差大的点, 重拟合 */
    for (i = 0U; i < ref->count; i++) {
        int32_t pred, r;
        if (ok[i] == 0U) {
            continue;
        }
        pred = a + (int32_t)((int64_t)b_ppm * ref->ref[i] / 1000000LL);
        r = lc_abs32(e[i] - pred);
        if (r > LC_R_TOL_PULSE) {
            ok[i] = 0U;
            outliers++;
        }
    }
    if (outliers > 0U) {
        n -= outliers;
        out->valid_cnt = n;
        if (n < LC_MIN_VALID) {
            return out->decision;
        }
        if (lc_fit(ref, e, ok, n, &a, &b_ppm, &max_r) == 0U) {
            return out->decision;
        }
    }

    out->a     = a;
    out->b_ppm = b_ppm;
    out->max_r = max_r;

    /* 第三层: 判定 */
    if (lc_abs32(b_ppm) >= LC_B_ALARM_PPM) {
        out->decision = LC_CHK_ALARM_BELT;
    } else if (lc_abs32(a) >= LC_A_ALARM_PULSE) {
        out->decision = LC_CHK_ALARM_ZERO;
    } else if (lc_abs32(b_ppm) >= LC_B_WARN_PPM) {
        out->decision = LC_CHK_WARN_BELT;
    } else if (lc_abs32(a) >= LC_A_WARN_PULSE) {
        out->decision = LC_CHK_WARN_ZERO;
    } else {
        out->decision = LC_CHK_OK;
    }
    return out->decision;
}

static void lc_update_comp(const lc_ref_t *ref, const lc_check_t *chk)
{
    uint8_t i;

    if (chk->decision >= LC_CHK_ALARM_ZERO) {
        return;                          /* 报警/测量差: 不更新 */
    }
    for (i = 0U; i < ref->count; i++) {
        int32_t c = chk->a + (int32_t)((int64_t)chk->b_ppm * ref->ref[i] / 1000000LL);

        if (c > LC_COMP_MAX_PULSE)  { c = LC_COMP_MAX_PULSE; }
        if (c < -LC_COMP_MAX_PULSE) { c = -LC_COMP_MAX_PULSE; }
        s_comp[i] = (3 * s_comp[i] + c) / 4;
    }
}

/* ================= 补偿查询 ================= */
int32_t lc_get_target(uint8_t idx)
{
    if ((idx == 0U) || (idx > LC_POINT_MAX) || (s_ref_valid == 0U)) {
        return 0;
    }
    return s_ref.ref[idx - 1U] + s_comp[idx - 1U];
}

int32_t lc_comp_at(int32_t ref_pulse)
{
    uint8_t i;

    if (s_ref_valid == 0U) {
        return 0;
    }
    if (ref_pulse <= s_ref.ref[0]) {
        return s_comp[0];
    }
    for (i = 1U; i < s_ref.count; i++) {
        if (ref_pulse <= s_ref.ref[i]) {
            int32_t d = s_ref.ref[i] - s_ref.ref[i - 1U];
            int64_t n = (int64_t)(ref_pulse - s_ref.ref[i - 1U]) *
                        (s_comp[i] - s_comp[i - 1U]);
            return s_comp[i - 1U] + (int32_t)(n / d);
        }
    }
    return s_comp[s_ref.count - 1U];
}

int32_t lc_get_base(void)
{
    return s_base_pulse;
}

uint8_t lc_set_base(void)
{
    if (lc_read_pulse(&s_base_pulse, LC_POS_TIMEOUT_MS, LC_READ_RETRY) == 0U) {
        return 0U;
    }
    s_zero_pulse = s_base_pulse;           /* 记下回零位的读数 */
    return 1U;
}

/* 去第 idx 个标注点: 目标 = 参考位 R + 补偿 C, 自动换算成相对运动 */
uint8_t lc_goto_mark(uint8_t idx)
{
    int32_t target, cur, delta;
    uint32_t t = 0U;

    if ((idx == 0U) || (idx > LC_POINT_MAX) || (s_ref_valid == 0U)) {
        return 0U;                          /* 没有参考位, 拒绝 */
    }

    target = s_ref.ref[idx - 1U] + s_comp[idx - 1U];

    if (lc_read_pulse(&cur, LC_POS_TIMEOUT_MS, LC_READ_RETRY) == 0U) {
        return 0U;
    }
    delta = target - (cur - s_base_pulse);  /* 相对本次 0 点的剩余距离 */
    if ((delta > (LC_TARGET_PULSE * 2)) || (delta < -(LC_TARGET_PULSE * 2))) {
        return 0U;                          /* 位置异常, 拒绝移动 */
    }

    if (delta > 0) {
        motor_move(0, (uint32_t)delta, LC_SCAN_RPM, LC_SCAN_ACC, MOTOR_MODE_REL_CUR, 0U);
    } else if (delta < 0) {
        motor_move(1, (uint32_t)(-delta), LC_SCAN_RPM, LC_SCAN_ACC, MOTOR_MODE_REL_CUR, 0U);
    } else {
        return 1U;                          /* 已经在了 */
    }

    while (t < LC_GOTO_TIMEOUT_MS) {
        DDL_DelayMS(20U);
        t += 20U;
        if (lc_read_pulse(&cur, LC_POS_TIMEOUT_FAST_MS, 1U) != 0U) {
            if (lc_abs32((cur - s_base_pulse) - target) <= LC_ARRIVE_TOL_PULSE) {
                return 1U;                  /* 到位 */
            }
        }
    }

    motor_stop();
    motor_stop();
    motor_stop();
    return 0U;
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
        /* 回零后重新记一次原点读数, 作为"是否从原点出发"的判据 */
        if (lc_read_pulse(&s_zero_pulse, LC_POS_TIMEOUT_MS, LC_READ_RETRY) == 0U) {
            return LC_RESULT_POS_ERR;
        }
    }

    motor_enable(1U);
    DDL_DelayMS(100U);
    motor_flush_rx();
    if (lc_read_pulse(&s_base_pulse, LC_POS_TIMEOUT_MS, LC_READ_RETRY) == 0U) {
        return LC_RESULT_POS_ERR;
    }
    /* 必须从回零位附近开始扫描!
     * lc_scan_pass() 下发的是"相对当前位置 +1400mm", 如果上一轮扫完停在 1400,
     * 再扫一次就会从 1400 一路冲到 2800, 撞坏机械。这里直接拦掉。
     * 判据用"与刚回零时的读数之差", 而不是和绝对 0 比:
     * 0x32 的读数在回零后不一定正好为 0。
     */
    {
        int32_t d = s_base_pulse - s_zero_pulse;
        if (d < 0) {
            d = -d;
        }
        if (d > LC_SCAN_START_TOL_PULSE) {
            return LC_RESULT_POS_ERR;
        }
    }

    for (pass = 0U; pass < LC_PASS_MAX; pass++) {
        if (pass > 0U) {
            if (lc_return_zero() == 0U) {
                /* 快速回 0 失败(位置读不到 + 限位没触发): 改用慢速撞限位。
                 * 它自带"解除保护 + 使能 + 重试", 1400mm @64mm/s 约 22 秒,
                 * 比直接放弃可靠得多, 否则这台设备上标定永远过不去。
                 */
                if (motor_limit_home() == 0U) {
                    res = LC_RESULT_TIMEOUT;
                    break;
                }
                /* 回零已完成 0x0A 清零, 坐标就是 0 */
                s_base_pulse = 0;
                s_zero_pulse = 0;
                can_report_event(CANP_EV_RETURN0, 2U, 0U);
            }
        }
        if (lc_scan_pass(pass) == 0U) {
            res = LC_RESULT_TIMEOUT;
            break;
        }
        /* 确认点数达到 LC_MIN_VALID(7) 即可收工。
         * 原要求 10 个全中, 只要漏检一个凸点就永远标定不成功, 也就永远存不进 Flash。
         */
        if (lc_confirmed_count() >= LC_MIN_VALID) {
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
    s_check.decision  = LC_CHK_NO_REF;
    s_check.valid_cnt = 0U;
    s_check.a         = 0;
    s_check.b_ppm     = 0;
    s_check.max_r     = 0;
    for (i = 0U; i < LC_SLOT_MAX; i++) {
        s_slots[i].n = 0U;
    }

    s_result = lc_scan_all(do_home);
    lc_build_points();

    if (lc_ref_ensure() != 0U) {
        (void)lc_check_ref(&s_ref, &s_check);
        lc_update_comp(&s_ref, &s_check);
    } else {
        /* 首次标定(还没有参考位): 把本轮确认到的凸点数报上去, 方便判断漏检情况 */
        s_check.valid_cnt = lc_confirmed_count();
        s_check.decision  = LC_CHK_NO_REF;
    }

    lc_report_check_can();
    return s_result;
}
