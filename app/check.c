#include "check.h"
#include "can.h"

uint16_t AdcValue = 0U;
uint32_t storage = 0U;
static uint16_t s_hc165_buf[3];

/* 电流监测计数(避免每次循环都采ADC) */
#define CURRENT_CHECK_TICK   (10U)
static uint8_t s_curTick = 0U;

/* ADC 原始值 -> 电流(mA)
 * INA180A2: 增益50, Rsense=20mΩ -> Vout = I * 1.0V
 * 12位ADC  3.3V: I(mA) = AdcValue * 3300 / 4095 */
static uint16_t adc_to_current_ma(uint16_t adc)
{
    return (uint16_t)(((uint32_t)adc * 3300UL) / 4095UL);
}

void check_init(void)
{
    LL_PERIPH_WE(LL_PERIPH_FCG);
    FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_ADC1, ENABLE);
    LL_PERIPH_WP(LL_PERIPH_FCG);

    stc_adc_init_t stcAdcInit;
    (void)ADC_StructInit(&stcAdcInit);
    stcAdcInit.u16ScanMode   = ADC_MD_SEQA_SINGLESHOT;
    stcAdcInit.u16Resolution = ADC_RESOLUTION_12BIT;
    stcAdcInit.u16DataAlign  = ADC_DATAALIGN_RIGHT;
    (void)ADC_Init(CM_ADC1, &stcAdcInit);

    ADC_ChCmd(CM_ADC1, ADC_SEQ_A, ADC_CH3, ENABLE);
    ADC_SetSampleTime(CM_ADC1, ADC_CH3, 8U);

    /*165引脚初始化 */
    GPIO_ResetPins(GPIO_PORT_A, GPIO_PIN_04);
    GPIO_ResetPins(GPIO_PORT_A, GPIO_PIN_06);
    GPIO_SetPins(GPIO_PORT_A, GPIO_PIN_07);
}

/* 电流采样 */
static uint16_t adc_read_current(void)
{
    (void)ADC_Start(CM_ADC1);
    uint32_t timeout = 65535UL;
    while ((ADC_GetStatus(CM_ADC1, ADC_FLAG_EOCA) == RESET) && (timeout-- > 0UL));
    uint16_t v;
    if (timeout > 0UL) v = ADC_GetValue(CM_ADC1, ADC_CH3);
    else               v = 0xFFFFU;
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
        GPIO_ResetPins(GPIO_PORT_H, GPIO_PIN_02);   /* PH2=0 断电 */
        can_report_overcurrent();
    }
}


/* HC165 读取(与原始逻辑一致) */
void hc165_Run(void)
{
    /* 1. 给 HC165 供电 */
    GPIO_SetPins(GPIO_PORT_A, GPIO_PIN_04);

    /* 2. 并行装载: PL 低脉冲, 把3片HC165的24路输入锁存到移位寄存器 */
    GPIO_ResetPins(GPIO_PORT_A, GPIO_PIN_05);   /* PL = 0: 装载并行数据 */
    DDL_DelayUS(100);
    GPIO_SetPins(GPIO_PORT_A, GPIO_PIN_05);     /* PL = 1: 允许移位 */

    /* 3. 移位读取 24 位: 每片8位, 共3片 */
    for (uint8_t i = 0U; i < 24U; i++) {
        if ((i % 8U) == 0U) s_hc165_buf[i/8U] = 0U;
        s_hc165_buf[i/8U] <<= 1U;
        GPIO_SetPins(GPIO_PORT_A, GPIO_PIN_06);   /* CP 上升沿移出一位 */
        if (GPIO_ReadInputPins(GPIO_PORT_A, GPIO_PIN_07) == PIN_SET) {  /* 读 DS */
            s_hc165_buf[i/8U] |= 0x01U;
        }
        GPIO_ResetPins(GPIO_PORT_A, GPIO_PIN_06); /* CP = 0 */
    }

    /* 4. 数据拼接: 3片, 每片8位 = 24位 */
    storage = ((uint32_t)s_hc165_buf[0] << 16U) |
              ((uint32_t)s_hc165_buf[1] << 8U)  |
              ((uint32_t)s_hc165_buf[2]);

    /* 5. 可选: 关HC165供电省电(按需) */
    // GPIO_ResetPins(GPIO_PORT_A, GPIO_PIN_04);
}



