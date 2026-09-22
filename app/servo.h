#ifndef __SERVO_H__
#define __SERVO_H__

#include "main.h"

#define SERVO_ANGLE_LEFT (62500*9/200)//36°
#define SERVO_ANGLE_RESET (62500*15/200)//90°
#define SERVO_ANGLE_RIGHT (62500*21/200)//144°

void servo_init(void);   /* 舵机初始化 */
void servo_Run(void);    /* 舵机执行测试 */
void servo_Reset(void);  /* 舵机复位 */


#endif

/*舵机顺序
PWM1	TIMA_1_PWM3
PWM2	TIMA_1_PWM5
PWM3	TIMA_1_PWM6
PWM4	TIMA_1_PWM7
PWM5	TIMA_1_PWM1
PWM6	TIMA_1_PWM2
PWM7	TIMA_5_PWM8
PWM8	TIMA_1_PWM8
PWM9	TIMA_3_PWM4
PWM10	TIMA_3_PWM3
*/
