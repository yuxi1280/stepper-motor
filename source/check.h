#ifndef __CHECK_H__
#define __CHECK_H__

#include "main.h"

/* ============================================================================
 * 电流检测: INA180A2IDBVR + 20mΩ, 检测 7.4V 供电轨
 * ----------------------------------------------------------------------------
 * INA180A2 增益 50 V/V
 *   Vout = I × 0.02 × 50 = I × 1.0 (V/A)   =>  1A = 1.0V
 * ADC: HC32F460 ADC1, 12bit, Vref = 3.3V
 *   I(mA) = ADC_raw × 3300 / 4095    量程 0~3300mA
 * ========================================================================== */
#define CHECK_ADC_PORT        (GPIO_PORT_B)
#define CHECK_ADC_PIN         (GPIO_PIN_01)
#define CHECK_ADC_CH          (ADC_CH9)

/* ---- 采样/换算参数 ---- */
#define CHECK_ADC_VREF_MV     (3300U)     /* ADC 参考电压(mV)*/
#define CHECK_ADC_FULL_SCALE  (4095U)     /* 12bit 满量程 */
#define CHECK_AVG_CNT         (8U)        /* 软件平均次数 */
#define CHECK_TICK_DIV        (10U)       /* 主循环每 N 次调用采一次 */

/* ---- 过流阈值(mA) ---- */
#define CHECK_CURRENT_THRESH_MA (2500U)

/* ---- 零点校准: 1=启用(减掉零电流时的 ADC 值, 抵消 INA180/ADC 失调) ---- */
#define CHECK_ZERO_CALIB      (1U)

/* ---- 对外接口 ---- */
void     check_init(void);              /* ADC1 + 引脚初始化 */
void     check_task(void);              /* 主循环周期调用: 采样 + 过流判定 */
void     check_calib_zero(void);        /* 把当前值标定为零点(确认无负载电流时调用) */

uint16_t check_current_ma(void);        /* 最近一次电流(mA) */
uint16_t check_adc_raw(void);           /* 最近一次 ADC 原始值 */
uint8_t  check_is_overcurrent(void);    /* 1=已发生过流(锁存, 不自动清) */
void     check_clear_overcurrent(void); /* 手动清除过流标志 */
/* 过流动作 */
void check_overcurrent_action(void);



/* --- */


#endif /* __CHECK_H__ */
