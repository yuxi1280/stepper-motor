#include "servo.h"


void servo_init(void)
{
    LL_PERIPH_WE(LL_PERIPH_FCG);
    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMRA_1, ENABLE);
    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMRA_3, ENABLE);
    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMRA_5, ENABLE);
    LL_PERIPH_WP(LL_PERIPH_FCG);
    stc_tmra_init_t stcTmraInit;
    (void)TMRA_StructInit(&stcTmraInit);
    stcTmraInit.u8CountSrc           = TMRA_CNT_SRC_SW;
    stcTmraInit.sw_count.u8ClockDiv  = TMRA_CLK_DIV32;     // 100MHz/32 = 3.125MHz
    stcTmraInit.sw_count.u8CountMode = TMRA_MD_SAWTOOTH;   // 锯齿波
    stcTmraInit.sw_count.u8CountDir  = TMRA_DIR_UP;        // 上升计数
    stcTmraInit.u32PeriodValue       = 62500U;             // 周期 → 约50Hz
    (void)TMRA_Init(CM_TMRA_1, &stcTmraInit);
    (void)TMRA_Init(CM_TMRA_3, &stcTmraInit);
    (void)TMRA_Init(CM_TMRA_5, &stcTmraInit);


    stc_tmra_pwm_init_t stcPwmInit;
    (void)TMRA_PWM_StructInit(&stcPwmInit);
    stcPwmInit.u32CompareValue         = SERVO_ANGLE_RESET;  // 初始占空比
    stcPwmInit.u16StartPolarity       = TMRA_PWM_HOLD;
    stcPwmInit.u16StopPolarity        = TMRA_PWM_LOW;
    stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_LOW;      // 比较匹配→拉低
    stcPwmInit.u16PeriodMatchPolarity  = TMRA_PWM_HIGH;    
    
    (void)TMRA_PWM_Init(CM_TMRA_1, TMRA_CH1, &stcPwmInit);
    (void)TMRA_PWM_Init(CM_TMRA_1, TMRA_CH2, &stcPwmInit);
    (void)TMRA_PWM_Init(CM_TMRA_1, TMRA_CH3, &stcPwmInit);
    (void)TMRA_PWM_Init(CM_TMRA_1, TMRA_CH5, &stcPwmInit);
    (void)TMRA_PWM_Init(CM_TMRA_1, TMRA_CH6, &stcPwmInit);
    (void)TMRA_PWM_Init(CM_TMRA_1, TMRA_CH7, &stcPwmInit);
    (void)TMRA_PWM_Init(CM_TMRA_1, TMRA_CH8, &stcPwmInit);

    (void)TMRA_PWM_Init(CM_TMRA_3, TMRA_CH3, &stcPwmInit);
    (void)TMRA_PWM_Init(CM_TMRA_3, TMRA_CH4, &stcPwmInit);

    (void)TMRA_PWM_Init(CM_TMRA_5, TMRA_CH8, &stcPwmInit);
    

    
    TMRA_PWM_OutputCmd(CM_TMRA_1, TMRA_CH1, ENABLE);
    TMRA_PWM_OutputCmd(CM_TMRA_1, TMRA_CH2, ENABLE);
    TMRA_PWM_OutputCmd(CM_TMRA_1, TMRA_CH3, ENABLE);
    TMRA_PWM_OutputCmd(CM_TMRA_1, TMRA_CH5, ENABLE);
    TMRA_PWM_OutputCmd(CM_TMRA_1, TMRA_CH6, ENABLE);
    TMRA_PWM_OutputCmd(CM_TMRA_1, TMRA_CH7, ENABLE);
    TMRA_PWM_OutputCmd(CM_TMRA_1, TMRA_CH8, ENABLE);

    TMRA_PWM_OutputCmd(CM_TMRA_3, TMRA_CH3, ENABLE);
    TMRA_PWM_OutputCmd(CM_TMRA_3, TMRA_CH4, ENABLE);
    
    TMRA_PWM_OutputCmd(CM_TMRA_5, TMRA_CH8, ENABLE);
    

    
    GPIO_SetPins(GPIO_PORT_H, GPIO_PIN_02);
    TMRA_Start(CM_TMRA_1);
    TMRA_Start(CM_TMRA_5);
    TMRA_Start(CM_TMRA_3);  
    //GPIO_ResetPins(GPIO_PORT_H, GPIO_PIN_02); 
    
}


void servo_Reset(void)
{
    TMRA_SetCompareValue(CM_TMRA_1, TMRA_CH3, SERVO_ANGLE_RESET);  // PWM1  TIMA_1_PWM3
    TMRA_SetCompareValue(CM_TMRA_1, TMRA_CH5, SERVO_ANGLE_RESET);  // PWM2  TIMA_1_PWM5
    TMRA_SetCompareValue(CM_TMRA_1, TMRA_CH6, SERVO_ANGLE_RESET);  // PWM3  TIMA_1_PWM6
    DDL_DelayMS(300U);
    TMRA_SetCompareValue(CM_TMRA_1, TMRA_CH7, SERVO_ANGLE_RESET);  // PWM4  TIMA_1_PWM7
    TMRA_SetCompareValue(CM_TMRA_1, TMRA_CH1, SERVO_ANGLE_RESET);  // PWM5  TIMA_1_PWM1
    TMRA_SetCompareValue(CM_TMRA_1, TMRA_CH2, SERVO_ANGLE_RESET);  // PWM6  TIMA_1_PWM2
    DDL_DelayMS(300U);
    TMRA_SetCompareValue(CM_TMRA_5, TMRA_CH8, SERVO_ANGLE_RESET);  // PWM7  TIMA_5_PWM8
    TMRA_SetCompareValue(CM_TMRA_1, TMRA_CH8, SERVO_ANGLE_RESET);  // PWM8  TIMA_1_PWM8
    TMRA_SetCompareValue(CM_TMRA_3, TMRA_CH4, SERVO_ANGLE_RESET);  // PWM9  TIMA_3_PWM4
    TMRA_SetCompareValue(CM_TMRA_3, TMRA_CH3, SERVO_ANGLE_RESET);  // PWM10 TIMA_3_PWM3

}

void servo_Run(void)//测试
{
    //GPIO_SetPins(GPIO_PORT_H, GPIO_PIN_02);
    
    // TMRA_SetCompareValue(CM_TMRA_3, TMRA_CH4, SERVO_ANGLE_RESET);
    // TMRA_SetCompareValue(CM_TMRA_1, TMRA_CH6, SERVO_ANGLE_RESET);
    TMRA_SetCompareValue(CM_TMRA_1, TMRA_CH3, SERVO_ANGLE_RESET);
    DDL_DelayMS(500U);
    
    // TMRA_SetCompareValue(CM_TMRA_3, TMRA_CH4, SERVO_ANGLE_RIGHT);
    // TMRA_SetCompareValue(CM_TMRA_1, TMRA_CH6, SERVO_ANGLE_RIGHT);
    TMRA_SetCompareValue(CM_TMRA_1, TMRA_CH3, SERVO_ANGLE_RIGHT);
    DDL_DelayMS(500U);

    // TMRA_SetCompareValue(CM_TMRA_3, TMRA_CH4, SERVO_ANGLE_RESET);
    // TMRA_SetCompareValue(CM_TMRA_1, TMRA_CH6, SERVO_ANGLE_RESET);
    TMRA_SetCompareValue(CM_TMRA_1, TMRA_CH3, SERVO_ANGLE_RESET);
    DDL_DelayMS(500U);

    TMRA_SetCompareValue(CM_TMRA_1, TMRA_CH3, SERVO_ANGLE_LEFT);
    DDL_DelayMS(500U);
    
}
