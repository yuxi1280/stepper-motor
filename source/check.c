#include "check.h"
#include "can.h"

/* ------------------------- 全局状态 ------------------------- */
uint16_t g_check_adc_raw  = 0U;
uint16_t g_check_current  = 0U;
uint8_t  g_check_oc       = 0U;

static uint8_t  s_tick = 0U;


#if (CHECK_ZERO_CALIB != 0U)
static int16_t  s_zero_raw = 0;          /* 零电流时的 ADC 原始值 */
#endif

/* ------------------------- 单次 ADC 采样 -------------------------
 * 超时返回 0xFFFF
 */
static uint16_t check_adc_read(void)
{
    uint32_t timeout = 65535UL;
    uint16_t v;

    (void)ADC_Start(CM_ADC1);
    while ((ADC_GetStatus(CM_ADC1, ADC_FLAG_EOCA) == RESET) && (timeout-- > 0UL)) {
    }
    if (timeout == 0UL) {
        (void)ADC_Stop(CM_ADC1);
        ADC_ClearStatus(CM_ADC1, ADC_FLAG_EOCA);
        return 0xFFFFU;
    }
    v = ADC_GetValue(CM_ADC1, CHECK_ADC_CH);
    ADC_ClearStatus(CM_ADC1, ADC_FLAG_EOCA);
    return v;
}

/* 平均采样(抑制 PWM/电源干扰) */
static uint16_t check_adc_read_avg(void)
{
    uint32_t sum = 0U;
    uint16_t n   = 0U;
    uint16_t v;
    uint8_t  i;

    for (i = 0U; i < CHECK_AVG_CNT; i++) {
        v = check_adc_read();
        if (v != 0xFFFFU) {
            sum += v;
            n++;
        }
    }
    return (n != 0U) ? (uint16_t)(sum / n) : 0xFFFFU;
}

/* ADC 原始值 -> 电流(mA) */
static uint16_t check_adc_to_ma(uint16_t raw)
{
    int32_t v = (int32_t)raw;

#if (CHECK_ZERO_CALIB != 0U)
    v -= s_zero_raw;
    if (v < 0) {
        v = 0;
    }
#endif
    return (uint16_t)(((uint32_t)v * CHECK_ADC_VREF_MV) / CHECK_ADC_FULL_SCALE);
}




/* ------------------------- 初始化 ------------------------- */
void check_init(void)
{
    stc_gpio_init_t stcGpioInit;
    stc_adc_init_t  stcAdcInit;

    /* 1. ADC1 时钟 */
    LL_PERIPH_WE(LL_PERIPH_FCG);
    FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_ADC1, ENABLE);
    LL_PERIPH_WP(LL_PERIPH_FCG);

    /* 2. 检测引脚设为模拟输入 */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    (void)GPIO_Init(CHECK_ADC_PORT, CHECK_ADC_PIN, &stcGpioInit);

    /* 3. ADC1: 序列A单次转换, 12bit, 右对齐 */
    (void)ADC_StructInit(&stcAdcInit);
    stcAdcInit.u16ScanMode   = ADC_MD_SEQA_SINGLESHOT;
    stcAdcInit.u16Resolution = ADC_RESOLUTION_12BIT;
    stcAdcInit.u16DataAlign  = ADC_DATAALIGN_RIGHT;
    (void)ADC_Init(CM_ADC1, &stcAdcInit);

    /* 4. 使能检测通道 + 采样时间 */
    ADC_ChCmd(CM_ADC1, ADC_SEQ_A, CHECK_ADC_CH, ENABLE);
    ADC_SetSampleTime(CM_ADC1, CHECK_ADC_CH, 8U);

    /* 5. 纯轮询, 关掉 EOCA 中断(若用中断, ISR 里必须清 EOCA 标志, 否则中断风暴) */
    ADC_IntCmd(CM_ADC1, ADC_INT_EOCA, DISABLE);

#if (CHECK_ZERO_CALIB != 0U)
    check_calib_zero();                  /* 上电按零电流标定一次 */
#endif
}

/* ------------------------- 零点标定 ------------------------- */
void check_calib_zero(void)
{
#if (CHECK_ZERO_CALIB != 0U)
    uint32_t sum = 0U;
    uint16_t n   = 0U;
    uint16_t v;
    uint8_t  i;

    for (i = 0U; i < 16U; i++) {
        v = check_adc_read_avg();
        if (v != 0xFFFFU) {
            sum += v;
            n++;
        }
    }
    if (n != 0U) {
        s_zero_raw = (int16_t)(sum / n);
    }
#endif
}


void check_task(void)
{
    uint16_t raw;
    uint16_t cur;

    if (++s_tick < CHECK_TICK_DIV) {
        return;
    }
    s_tick = 0U;

    raw = check_adc_read_avg();
    if (raw == 0xFFFFU) {
        return;                          /* 采样失败, 保留上次值 */
    }
    g_check_adc_raw = raw;
    cur = check_adc_to_ma(raw);
    g_check_current = cur;

    if (cur > CHECK_CURRENT_THRESH_MA) {
        if (g_check_oc == 0U) {
            g_check_oc = 1U;
            check_overcurrent_action();
        }
    }
}



/* ------------------------- 查询 / 清除 ------------------------- */
uint16_t check_current_ma(void)          { return g_check_current; }
uint16_t check_adc_raw(void)             { return g_check_adc_raw; }
uint8_t  check_is_overcurrent(void)      { return g_check_oc; }
void     check_clear_overcurrent(void)   { g_check_oc = 0U; }

/* ------------------------- 过流动作(默认空, 可在 main.c 重写) ------------------------- */
__WEAKDEF void check_overcurrent_action(void)
{
    /* 默认什么都不做; 在 main.c 里重写它即可, 例如:
     *   servo_stop(SERVO_ID_DEFAULT);
     *   motor_stop();
     */
}
