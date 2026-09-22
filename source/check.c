#include "check.h"
#include "can.h"

uint16_t AdcValue = 0U;

/* 电流监测计数(避免每次循环都采ADC) */
#define CURRENT_CHECK_TICK   (10U)
static uint8_t s_curTick = 0U;

/* 舵机电源控制: PH2, 高=供电, 低=断电 */
#define SERVO_PWR_GPIO_PORT   (GPIO_PORT_H)
#define SERVO_PWR_GPIO_PIN    (GPIO_PIN_02)

/* ADC 原始值 -> 电流(mA): I(mA) = Adc * 3300 / 4095 */
static uint16_t adc_to_current_ma(uint16_t adc)
{
    return (uint16_t)(((uint32_t)adc * 3300UL) / 4095UL);
}

void check_init(void)
{
    stc_adc_init_t  stcAdcInit;
    stc_gpio_init_t stcGpioInit;

    /* ---- 1. ADC1 时钟 ---- */
    LL_PERIPH_WE(LL_PERIPH_FCG);
    FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_ADC1, ENABLE);
    LL_PERIPH_WP(LL_PERIPH_FCG);

    /* ---- 2. ADC1 配置 ---- */
    (void)ADC_StructInit(&stcAdcInit);
    stcAdcInit.u16ScanMode   = ADC_MD_SEQA_SINGLESHOT;
    stcAdcInit.u16Resolution = ADC_RESOLUTION_12BIT;
    stcAdcInit.u16DataAlign  = ADC_DATAALIGN_RIGHT;
    (void)ADC_Init(CM_ADC1, &stcAdcInit);

    /* 通道 9 = PB1 (analog 属性已由 main.c App_PortCfg 配置) */
    ADC_ChCmd(CM_ADC1, ADC_SEQ_A, ADC_CURRENT_CHANNEL, ENABLE);
    ADC_SetSampleTime(CM_ADC1, ADC_CURRENT_CHANNEL, 8U);

    /* 注意: 本模块用轮询读取, 不要开 ADC_INT_EOCA, 否则空回调会造成中断风暴 */

    /* ---- 3. 舵机电源 PH2(输出, 默认供电) ---- */
    LL_PERIPH_WE(LL_PERIPH_GPIO);
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDir   = PIN_DIR_OUT;
    stcGpioInit.u16PinState = PIN_STAT_SET;
    stcGpioInit.u16PinAttr  = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(SERVO_PWR_GPIO_PORT, SERVO_PWR_GPIO_PIN, &stcGpioInit);
    LL_PERIPH_WP(LL_PERIPH_GPIO);
}

/* 电流采样(阻塞, 带超时) */
static uint16_t adc_read_current(void)
{
    uint16_t v;
    uint32_t timeout = 65535UL;

    (void)ADC_Start(CM_ADC1);
    while ((ADC_GetStatus(CM_ADC1, ADC_FLAG_EOCA) == RESET) && (timeout-- > 0UL)) {
    }
    if (timeout > 0UL) {
        v = ADC_GetValue(CM_ADC1, ADC_CURRENT_CHANNEL);
    } else {
        v = 0xFFFFU;
    }
    ADC_ClearStatus(CM_ADC1, ADC_FLAG_EOCA);
    return v;
}

void check_task(void)
{
    uint16_t cur_ma;

    s_curTick++;
    if (s_curTick < CURRENT_CHECK_TICK) {
        return;
    }
    s_curTick = 0U;

    AdcValue = adc_read_current();
    cur_ma   = adc_to_current_ma(AdcValue);

    if (cur_ma > ADC_CURRENT_THRESHOLD_MA) {
        /* 过流: 切断舵机电源(PH2) + 上报主机 */
        GPIO_ResetPins(SERVO_PWR_GPIO_PORT, SERVO_PWR_GPIO_PIN);
        can_report_overcurrent();
    }
}
