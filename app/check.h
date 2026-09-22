#ifndef __CHECK_H__
#define __CHECK_H__

#include "main.h"

extern uint16_t AdcValue;
extern uint32_t storage;

/* 电流阈值: 超此值判定过流卡死 (单位 mA)
 * INA180A2(增益50) + 20mΩ 采样电阻:
 *   Vout = I * 0.02 * 50 = I * 1.0 (V)  ->  1A 对应 1V
 * 12位ADC @ 3.3V:  I(mA) = AdcValue * 3300 / 4095
 * 2500mA 过流阈值对应原始ADC ≈ 2500*4095/3300 ≈ 3102 */
#define ADC_CURRENT_THRESHOLD_MA  (2500U)  /* 电流阈值: 超此值判定过流卡死 */

void check_init(void);       /* ADC + HC165 初始化 */
void check_task(void);       /* 主循环: 电流监测(周期调用) */
void hc165_Run(void);


#endif
