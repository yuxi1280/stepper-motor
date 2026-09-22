#ifndef __CHECK_H__
#define __CHECK_H__

#include "main.h"

extern uint16_t AdcValue;

/* 电流阈值: 超此值判定过流卡死 (单位 mA)
 * INA180A2(增益50) + 20mΩ 采样电阻:
 *   Vout = I * 0.02 * 50 = I * 1.0 (V)  ->  1A 对应 1V
 * 12位ADC @ 3.3V:  I(mA) = AdcValue * 3300 / 4095
 * 2500mA 对应原始ADC ≈ 2500*4095/3300 ≈ 3102
 * 采样引脚: PB1 = ADC12_IN9 = ADC_CH9
 */
#define ADC_CURRENT_THRESHOLD_MA  (2500U)
#define ADC_CURRENT_CHANNEL       (ADC_CH9)   /* PB1 */

void check_init(void);       /* ADC + 舵机电源(PH2) 初始化 */
void check_task(void);       /* 主循环: 电流监测(周期调用) */

#endif /* __CHECK_H__ */
