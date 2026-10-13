/**
 *******************************************************************************
 * @file  main.c
 * @brief Main program.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2026-09-20       CDT             First version
 @endverbatim
 *******************************************************************************
 * Copyright (C) 2022-2025, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "main.h"
#include "uart.h"
#include "motor.h"
#include "servo.h"
#include "can.h"
#include "check.h"

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
/* INT_SRC_CAN_INT Callback. */
static void INT_SRC_CAN_INT_IrqCallback(void);
/* INT_SRC_ADC1_EOCA Callback. */
static void INT_SRC_ADC1_EOCA_IrqCallback(void);
/* Configures ADC. */
//static void App_ADCCfg(void);
/* Configures USARTx. */
static void App_USARTxCfg(void);
/* Configures CAN. */
static void App_CANCfg(void);
/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
//Clock Config
static void App_ClkCfg(void)
{
    /* Set bus clock div. */
    CLK_SetClockDiv(CLK_BUS_CLK_ALL, (CLK_HCLK_DIV1 | CLK_EXCLK_DIV2 | CLK_PCLK0_DIV1 | CLK_PCLK1_DIV2 | \
                                   CLK_PCLK2_DIV4 | CLK_PCLK3_DIV4 | CLK_PCLK4_DIV2));
    /* sram init include read/write wait cycle setting */
    SRAM_SetWaitCycle(SRAM_SRAM_ALL, SRAM_WAIT_CYCLE1, SRAM_WAIT_CYCLE1);
    SRAM_SetWaitCycle(SRAM_SRAMH, SRAM_WAIT_CYCLE0, SRAM_WAIT_CYCLE0);
    /* flash read wait cycle setting */
    EFM_SetWaitCycle(EFM_WAIT_CYCLE5);
    /* XTAL config */
    stc_clock_xtal_init_t stcXtalInit;
    (void)CLK_XtalStructInit(&stcXtalInit);
    stcXtalInit.u8State = CLK_XTAL_ON;
    stcXtalInit.u8Drv = CLK_XTAL_DRV_HIGH;
    stcXtalInit.u8Mode = CLK_XTAL_MD_OSC;
    stcXtalInit.u8StableTime = CLK_XTAL_STB_2MS;
    (void)CLK_XtalInit(&stcXtalInit);
    /* MPLL config */
    stc_clock_pll_init_t stcMPLLInit;
    (void)CLK_PLLStructInit(&stcMPLLInit);
    stcMPLLInit.PLLCFGR = 0UL;
    stcMPLLInit.PLLCFGR_f.PLLM = (1UL - 1UL);
    stcMPLLInit.PLLCFGR_f.PLLN = (50UL - 1UL);
    stcMPLLInit.PLLCFGR_f.PLLP = (2UL - 1UL);
    stcMPLLInit.PLLCFGR_f.PLLQ = (2UL - 1UL);
    stcMPLLInit.PLLCFGR_f.PLLR = (2UL - 1UL);
    stcMPLLInit.u8PLLState = CLK_PLL_ON;
    stcMPLLInit.PLLCFGR_f.PLLSRC = CLK_PLL_SRC_XTAL;
    (void)CLK_PLLInit(&stcMPLLInit);
    /* 3 cycles for 126MHz ~ 200MHz */
    GPIO_SetReadWaitCycle(GPIO_RD_WAIT3);
    /* Switch driver ability */
    PWC_HighSpeedToHighPerformance();
    /* Set the system clock source */
    CLK_SetSysClockSrc(CLK_SYSCLK_SRC_PLL);
}

//Port Config
static void App_PortCfg(void)
{
    /* GPIO initialize */
    stc_gpio_init_t stcGpioInit;
    /* PH0 set to XTAL-EXT/XTAL-OUT */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    (void)GPIO_Init(GPIO_PORT_H, GPIO_PIN_00, &stcGpioInit);

    /* PH1 set to XTAL-IN */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    (void)GPIO_Init(GPIO_PORT_H, GPIO_PIN_01, &stcGpioInit);

    /* PB1 set to ADC12-IN9 */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    (void)GPIO_Init(GPIO_PORT_B, GPIO_PIN_01, &stcGpioInit);

    /* PB12 set to GPIO-Input */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(GPIO_PORT_B, GPIO_PIN_12, &stcGpioInit);

    /* PB4 set to GPIO-Input */
    GPIO_SetDebugPort(GPIO_PIN_TRST, DISABLE);
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(GPIO_PORT_B, GPIO_PIN_04, &stcGpioInit);

    GPIO_SetFunc(GPIO_PORT_A,GPIO_PIN_03,GPIO_FUNC_33);//USART1-RX
    
    GPIO_SetFunc(GPIO_PORT_A,GPIO_PIN_04,GPIO_FUNC_32);//USART1-TX
    
    GPIO_SetFunc(GPIO_PORT_B,GPIO_PIN_13,GPIO_FUNC_33);//USART3-RX
    
    GPIO_SetFunc(GPIO_PORT_B,GPIO_PIN_14,GPIO_FUNC_32);//USART3-TX
    
    GPIO_SetFunc(GPIO_PORT_B,GPIO_PIN_08,GPIO_FUNC_51);//CAN-RXD
    
    GPIO_SetFunc(GPIO_PORT_B,GPIO_PIN_09,GPIO_FUNC_50);//CAN-TXD
    
}

//Int Config
static void App_IntCfg(void)
{
    stc_irq_signin_config_t stcIrq;

    /* IRQ sign-in */
    // stcIrq.enIntSrc = INT_SRC_CAN_INT;
    // stcIrq.enIRQn = INT122_IRQn;
    // stcIrq.pfnCallback = &INT_SRC_CAN_INT_IrqCallback;
    // (void)INTC_IrqSignIn(&stcIrq);
    // /* NVIC config */
    // NVIC_ClearPendingIRQ(INT122_IRQn);
    // NVIC_SetPriority(INT122_IRQn, DDL_IRQ_PRIO_15);
    // NVIC_EnableIRQ(INT122_IRQn);

    /* IRQ sign-in */
    stcIrq.enIntSrc = INT_SRC_ADC1_EOCA;
    stcIrq.enIRQn = INT116_IRQn;
    stcIrq.pfnCallback = &INT_SRC_ADC1_EOCA_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrq);
    /* NVIC config */
    NVIC_ClearPendingIRQ(INT116_IRQn);
    NVIC_SetPriority(INT116_IRQn, DDL_IRQ_PRIO_15);
    NVIC_EnableIRQ(INT116_IRQn);

}




/**
 * @brief  Main function of the project
 * @param  None
 * @retval int32_t return value, if needed
 */
int main(void)
{
    /* Register write unprotected for some required peripherals. */
    LL_PERIPH_WE(LL_PERIPH_ALL);
    //Clock Config
    App_ClkCfg();
    //Port Config
    App_PortCfg();
    //Int Config
    App_IntCfg();
    //ADC Config
    //App_ADCCfg();
    //USARTx Config
    App_USARTxCfg();
    //CAN Config
    //App_CANCfg();
    can_init(); 
    

    uart_driver_init();
    check_init();         

    motor_init();
    motor_enable(1);
    DDL_DelayMS(200);
    //motor_limit_home();

    servo_init();
    
    //servo_set_mid_here(SERVO_ID_BROADCAST); 
    //DDL_DelayMS(100);


    /* Register write protected for some required peripherals. */
    LL_PERIPH_WP(LL_PERIPH_ALL);
    for (;;) {
        motor_limit_task();     
        motor_rx_task(); 
        //servo_run();
        // servo_move(SERVO_ID_DEFAULT, 1500, 500);    /* 另一端 */
        // DDL_DelayMS(1000);   
        //check_task();   
        can_poll();       
        
                 



        // static const uint8_t frame[] = { 0x01U, 0xF3U, 0xABU, 0x01U, 0x00U, 0x6BU };
        // uart_send_data(UART_IDX_0, frame, (uint16_t)sizeof(frame));
        // DDL_DelayMS(100);         
        

    }
}

/* INT_SRC_CAN_INT Callback. */
static void INT_SRC_CAN_INT_IrqCallback(void)
{
    //add your codes here
}
/* INT_SRC_ADC1_EOCA Callback. */
static void INT_SRC_ADC1_EOCA_IrqCallback(void)
{
    //add your codes here
}

//ADC Config
static void App_ADCCfg(void)
{
    //independent mode
    //ADC1 config 
    stc_adc_init_t stcAdcInit;

    /* 1. Enable ADC1 peripheral clock. */
    FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_ADC1, ENABLE);

    /* 2. Modify the default value depends on the application. */
    (void)ADC_StructInit(&stcAdcInit);
    stcAdcInit.u16ScanMode = ADC_MD_SEQA_SINGLESHOT;
    stcAdcInit.u16Resolution = ADC_RESOLUTION_12BIT;
    stcAdcInit.u16DataAlign = ADC_DATAALIGN_RIGHT;

    /* 3. Initializes ADC. */
    (void)ADC_Init(CM_ADC1, &stcAdcInit);

    /* 4. ADC sequence configuration. */
    /* ADC sequence A configuration. */
    ADC_ChCmd(CM_ADC1, ADC_SEQ_A, ADC_CH9, ENABLE);

    /* ADC Int configuration */
    ADC_IntCmd(CM_ADC1, ADC_INT_EOCA, ENABLE);
}

//USARTx Config
static void App_USARTxCfg(void)
{
    stc_usart_uart_init_t stcUartInit;

    /* Enable USART1 clock */
    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USART1, ENABLE);
    /************************* Configure USART1***************************/
    USART_DeInit(CM_USART1);
    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32ClockSrc = USART_CLK_SRC_INTERNCLK;
    stcUartInit.u32ClockDiv = USART_CLK_DIV1;
    stcUartInit.u32CKOutput = USART_CK_OUTPUT_DISABLE;
    stcUartInit.u32Baudrate = 115200UL;
    stcUartInit.u32DataWidth = USART_DATA_WIDTH_8BIT;
    stcUartInit.u32StopBit = USART_STOPBIT_1BIT;
    stcUartInit.u32Parity = USART_PARITY_NONE;
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_16BIT;
    stcUartInit.u32FirstBit = USART_FIRST_BIT_LSB;
    stcUartInit.u32StartBitPolarity = USART_START_BIT_FALLING;
    stcUartInit.u32HWFlowControl = USART_HW_FLOWCTRL_RTS;
    USART_UART_Init(CM_USART1, &stcUartInit, NULL);
    /* Enable USART_TX | USART_RX function */
    USART_FuncCmd(CM_USART1, (USART_TX | USART_RX), ENABLE);

    /* Enable USART3 clock */
    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USART3, ENABLE);
    /************************* Configure USART3***************************/
    USART_DeInit(CM_USART3);
    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32ClockSrc = USART_CLK_SRC_INTERNCLK;
    stcUartInit.u32ClockDiv = USART_CLK_DIV1;
    stcUartInit.u32CKOutput = USART_CK_OUTPUT_DISABLE;
    stcUartInit.u32Baudrate = 115200UL;
    stcUartInit.u32DataWidth = USART_DATA_WIDTH_8BIT;
    stcUartInit.u32StopBit = USART_STOPBIT_1BIT;
    stcUartInit.u32Parity = USART_PARITY_NONE;
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_16BIT;
    stcUartInit.u32FirstBit = USART_FIRST_BIT_LSB;
    stcUartInit.u32StartBitPolarity = USART_START_BIT_FALLING;
    stcUartInit.u32HWFlowControl = USART_HW_FLOWCTRL_RTS;
    USART_UART_Init(CM_USART3, &stcUartInit, NULL);
    /* Enable USART_TX | USART_RX function */
    USART_FuncCmd(CM_USART3, (USART_TX | USART_RX), ENABLE);
}

//CAN Config
static void App_CANCfg(void)
{
    stc_can_init_t stcCanInit;

    /************************* Configure CAN1***************************/

    /* Initializes CAN. */
    (void)CAN_StructInit(&stcCanInit);
    stcCanInit.stcBitCfg.u32Prescaler = 4U;
    stcCanInit.stcBitCfg.u32TimeSeg1  = 16U;
    stcCanInit.stcBitCfg.u32TimeSeg2  = 4U;
    stcCanInit.stcBitCfg.u32SJW       = 2U;
    stcCanInit.u8WorkMode             = CAN_WORK_MD_NORMAL;
    stcCanInit.u8PTBSingleShotTx      = CAN_PTB_SINGLESHOT_TX_DISABLE;
    stcCanInit.u8STBSingleShotTx      = CAN_STB_SINGLESHOT_TX_DISABLE;
    stcCanInit.u8STBPrioMode          = CAN_STB_PRIO_MD_DISABLE;
    stcCanInit.u8RxWarnLimit          = 1U;
    stcCanInit.u8ErrorWarnLimit       = 11U;
    stcCanInit.u8RxAllFrame           = CAN_RX_ALL_FRAME_DISABLE;
    stcCanInit.u8RxOvfMode            = CAN_RX_OVF_SAVE_NEW;

    /* Enable peripheral clock of CAN. */
    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_CAN, ENABLE);
    (void)CAN_Init(CM_CAN, &stcCanInit);
    /* Enable the interrupts, the status flags can be read. */
    CAN_IntCmd(CM_CAN, CAN_INT_ALL, DISABLE);
    CAN_IntCmd(CM_CAN, CAN_INT_RX | CAN_INT_RX_OVERRUN | CAN_INT_RX_BUF_FULL | CAN_INT_RX_BUF_WARN | CAN_INT_PTB_TX | CAN_INT_STB_TX | CAN_INT_ERR_INT, ENABLE);
}

/**
 * @}
 */

/**
 * @}
 */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
