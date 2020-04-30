/**************************************************************************************************
  Filename:       OnBoard.c
  Revised:        $Date: 2012-11-15 11:48:16 -0800 (Thu, 15 Nov 2012) $
  Revision:       $Revision: 32196 $

  Description:    This file contains the UI and control for the
                  peripherals on the EVAL development board
  Notes:          This file targets the Chipcon CC2530/31


  Copyright 2005-2011 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

//#include "ZComDef.h"
//#include "ZGlobals.h"
//#include "OSAL.h"
#include "OnBoard.h"
//
//#include "MT.h"
//#include "MT_SYS.h"
//#include "DebugTrace.h"

/* Hal */
//#include "hal_lcd.h"
//#include "hal_mcu.h"
//#include "hal_timer.h"
//#include "hal_key.h"
//#include "hal_led.h"

/* Allow access macRandomByte() */
//#include "mac_radio_defs.h"

// #include "hal_nrf.h"
// #include "radio_esb.h"


#include "stm32f10x.h"
// #include "nRF24L01.h"

/*********************************************************************
 * CONSTANTS
 */

// Task ID not initialized
#define NO_TASK_ID 0xFF

// Minimum length RAM "pattern" for Stack check
#define MIN_RAM_INIT 12

/*********************************************************************
 * GLOBAL VARIABLES
 */

uint16_t        logo_displayTime;



extern void dmx_init(void);





// 64-bit Extended Address of this device


/*********************************************************************
 * LOCAL VARIABLES
 */

// Registered keys task ID, initialized to NOT USED.
static uint8_t registeredKeysTaskID = NO_TASK_ID;

/*********************************************************************
 * 内部函数声明
 */

static void clock_Init ( void );
static void gpioInit ( void );
static void timerInit ( void );
static void timer3Init ( void );


/*********************************************************************
 * @fn      InitBoard()
 * @brief   Initialize the STM8 Board Peripherals
 * @param   level: COLD,WARM,READY
 * @return  None
 */
void InitBoard ( uint8_t level )
{
    if ( level == OB_COLD )
    {
        // Interrupts off
        // osal_int_disable ( INTS_ALL );
        clock_Init();
        gpioInit();
        timerInit();
        timer3Init();
//      ADCInit(void);
//      ExitInit();
//      BSP_Time1Init();
//      BSP_Time2Init();
//      BSP_Time3Init();
//      BSP_Time4Init();
//      SPI_Config();


    }
    else     // !OB_COLD
    {
        /* Initialize Key stuff */
#if defined (ISR_KEYINTERRUPT)
//      HalKeyConfig ( HAL_KEY_INTERRUPT_ENABLE, OnBoard_KeyCallback );
#else
//      HalKeyConfig ( HAL_KEY_INTERRUPT_DISABLE, OnBoard_KeyCallback );
#endif
//        menuInit();
        dmx_init();

    }
}


/*********************************************************************
 * @fn      OnBoard_KeyCallback
 *
 * @brief   Callback service for keys
 *
 * @param   keys  - keys that were pressed
 *          state - shifted
 *
 * @return  void
 *********************************************************************/
void OnBoard_KeyCallback ( uint8_t keys, uint8_t state )
{
//    uint8_t shift;
    ( void ) state;

//  shift = (keys & HAL_KEY_SW_6) ? true : false;

//  if ( OnBoard_SendKeys( keys, shift ) != ZSuccess )
//  {
//    // Process SW1 here
//    if ( keys & HAL_KEY_SW_1 )  // Switch 1
//    {
//    }
//    // Process SW2 here
//    if ( keys & HAL_KEY_SW_2 )  // Switch 2
//    {
//    }
//    // Process SW3 here
//    if ( keys & HAL_KEY_SW_3 )  // Switch 3
//    {
//    }
//    // Process SW4 here
//    if ( keys & HAL_KEY_SW_4 )  // Switch 4
//    {
//    }
//    // Process SW5 here
//    if ( keys & HAL_KEY_SW_5 )  // Switch 5
//    {
//    }
//    // Process SW6 here
//    if ( keys & HAL_KEY_SW_6 )  // Switch 6
//    {
//    }
//  }
}


void NVIC_Config ( void )
{
    // NVIC_InitTypeDef  NVIC_InitStructure;

    // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);     //设置中断分组

    // NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;   //制定专断通道
    // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     //使能中断
    // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   //抢占优先级
    // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;    //子优先级
    // NVIC_Init(&NVIC_InitStructure);   //初始化

    // // 初始化 串口中断优先级
    // NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    // NVIC_Init(&NVIC_InitStructure);

    // // 初始化 定时器3中断优先级
    // NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    // NVIC_Init(&NVIC_InitStructure);
}

/***************************************************************************
 * @ fn
 * @ brief
 * @ param
 * @ param
 * @ retval
 **/
void clock_Init ( void )
{
    RCC_DeInit();

    RCC_HSEConfig ( RCC_HSE_ON );
//      RCC_HSICmd ( ENABLE );

    RCC_PLLConfig ( RCC_PLLSource_HSE_Div1, RCC_PLLMul_9 );
    /* 开启PLL */
    RCC_PLLCmd ( ENABLE );
    /* Wait till PLL is ready */
    while ( RCC_GetFlagStatus ( RCC_FLAG_PLLRDY ) == 0 )
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig ( RCC_SYSCLKSource_PLLCLK );

    RCC_PCLK2Config(RCC_HCLK_Div1);

    /* 配置操作系统节拍 */
    SysTick_CLKSourceConfig ( SysTick_CLKSource_HCLK );
    SysTick_Config ( 72000000 / 1000 );

    RCC_APB2PeriphClockCmd ( RCC_APB2Periph_GPIOB, ENABLE );
    RCC_APB2PeriphClockCmd ( RCC_APB2Periph_GPIOA, ENABLE );
    RCC_APB2PeriphClockCmd ( RCC_APB2Periph_ADC1, ENABLE );
    RCC_APB1PeriphClockCmd (RCC_APB1Periph_USART2, ENABLE);

}


/***************************************************************************
 * @ fn
 * @ brief      管脚初始化配置
 * @ param
 * @ param
 * @ retval
 **/
static void gpioInit ( void )
{
    GPIO_InitTypeDef    GPIO_InitStructure;

    /* DMX512接口，串口管脚初始化 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;//TX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init ( GPIOB, &GPIO_InitStructure );
    GPIO_ResetBits(GPIOB, GPIO_Pin_1);   /* DMX512发送接收控制端口，低电平接收 */

    /* 24C04存储，IIC接口 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_Init ( GPIOB, &GPIO_InitStructure );

    GPIO_SetBits ( GPIOB, GPIO_Pin_10 );
    GPIO_SetBits ( GPIOB, GPIO_Pin_11 );

    /* LCD管脚初始化，IIC接口 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_Init ( GPIOB, &GPIO_InitStructure );

    /* 灯珠PWM控制脚 */

    GPIO_ResetBits ( GPIOA, GPIO_Pin_9 );       //红
    GPIO_ResetBits ( GPIOA, GPIO_Pin_10 );      //绿
    GPIO_ResetBits ( GPIOA, GPIO_Pin_11 );      //蓝色
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;   //配置为PWM输出，选择GPIO_Mode_AF_PP
    GPIO_Init ( GPIOA, &GPIO_InitStructure );

    /* ADC管脚配置 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init ( GPIOB, &GPIO_InitStructure );

    /* 四个按键 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); //必须执行这句
    /* Disable JLink, enable SW */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;    //上拉输入
    GPIO_Init ( GPIOB, &GPIO_InitStructure );
    GPIO_SetBits ( GPIOB, GPIO_Pin_3 );      //up
    GPIO_SetBits ( GPIOB, GPIO_Pin_4 );     //key
    GPIO_SetBits ( GPIOB, GPIO_Pin_8 );     //down
    GPIO_SetBits ( GPIOB, GPIO_Pin_9 );     //down

    /* wireless mode 2.4G */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;   //CE
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init ( GPIOA, &GPIO_InitStructure );
    GPIO_ResetBits ( GPIOA, GPIO_Pin_1 );     //

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12;   //CSN
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init ( GPIOB, &GPIO_InitStructure );
    GPIO_SetBits ( GPIOB, GPIO_Pin_12 );     //down

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;   //SCK
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init ( GPIOB, &GPIO_InitStructure );
    GPIO_ResetBits ( GPIOB, GPIO_Pin_13 );     //

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;   //MOSI
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init ( GPIOB, &GPIO_InitStructure );
    GPIO_ResetBits ( GPIOB, GPIO_Pin_15 );     //

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_14;   //MISO
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init ( GPIOB, &GPIO_InitStructure );
    GPIO_SetBits ( GPIOB, GPIO_Pin_14 );     //

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8;    //IRQ
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init ( GPIOA, &GPIO_InitStructure );
    GPIO_SetBits ( GPIOA, GPIO_Pin_8 );     //
    /* 指示灯 */
//      RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); //必须执行这句
//      /* Disable JLink, enable SW */
//      GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;   //
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init ( GPIOA, &GPIO_InitStructure );
    GPIO_SetBits(GPIOA, GPIO_Pin_0);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_15;   //
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init ( GPIOA, &GPIO_InitStructure );





}


/***************************************************************************
 * @ fn
 * @ brief   定时器初始化
 * @ param
 * @ param
 * @ retval
 **/
static void timerInit ( void )
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef        TIM_OCInitStructure;

    TIM_DeInit ( TIM1 );

//      TIM_DeInit ( TIM2 );
    TIM_DeInit ( TIM3 );

    RCC_APB2PeriphClockCmd ( RCC_APB2Periph_TIM1, ENABLE );

//      RCC_APB1PeriphClockCmd ( RCC_APB1Periph_TIM2, ENABLE );

//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
//  GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE);

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 3000;
    TIM_TimeBaseStructure.TIM_Prescaler = 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit ( TIM1, &TIM_TimeBaseStructure ); //

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 1000;
    TIM_TimeBaseStructure.TIM_Prescaler = 22;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit ( TIM2, &TIM_TimeBaseStructure ); //

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;        //占空比值是500，所以占空比是：500/1000=50%
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC1Init ( TIM1, &TIM_OCInitStructure );

    TIM_OC1PreloadConfig ( TIM1, TIM_OCPreload_Enable );

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;        //占空比值是500，所以占空比是：500/1000=50%
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC2Init ( TIM1, &TIM_OCInitStructure );

    TIM_OC2PreloadConfig ( TIM1, TIM_OCPreload_Enable );

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;        //占空比值是500，所以占空比是：500/1000=50%
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC3Init ( TIM1, &TIM_OCInitStructure );

    TIM_OC3PreloadConfig ( TIM1, TIM_OCPreload_Enable );

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;        //占空比值是500，所以占空比是：500/1000=50%
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC4Init ( TIM1, &TIM_OCInitStructure );
    TIM_OC4PreloadConfig ( TIM1, TIM_OCPreload_Enable );

//      TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
//      TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//      TIM_OCInitStructure.TIM_Pulse = 0;        //占空比值是500，所以占空比是：500/1000=50%
//      TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
//      TIM_OC2Init ( TIM2, &TIM_OCInitStructure );

//      TIM_OC2PreloadConfig ( TIM2, TIM_OCPreload_Enable );


    TIM_Cmd ( TIM1, ENABLE );
//      TIM_Cmd ( TIM2, ENABLE );

    TIM_CtrlPWMOutputs ( TIM1, ENABLE );
    TIM_CtrlPWMOutputs ( TIM2, ENABLE );

    TIM_SetCompare1 ( TIM1,  0 );       //白
    TIM_SetCompare2 ( TIM1,  0 );       //
    TIM_SetCompare3 ( TIM1,  0 );       //黄
    TIM_SetCompare4 ( TIM1,  0 );       //绿色

//      TIM_SetCompare2 ( TIM2,  500 );    //蓝

}


/***************************************************************************
 * @ fn
 * @ brief   定时器初始化
 * @ param
 * @ param
 * @ retval
 **/
static void timer3Init ( void )
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    TIM_DeInit ( TIM3 );

    RCC_APB1PeriphClockCmd ( RCC_APB1Periph_TIM3, ENABLE );

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 1000;
    TIM_TimeBaseStructure.TIM_Prescaler = 48;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit ( TIM3, &TIM_TimeBaseStructure ); //
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
//      TIM_Cmd ( TIM3, ENABLE );

}

void ExitInit(void)
{
    EXTI_InitTypeDef   EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /*中断的初始化*/
    EXTI_InitStructure.EXTI_Line = EXTI_Line8;
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);     //初始化外设EXTI寄存器

    // NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;                  //使能按键外部中断通道
    // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;  //抢占优先级2，
    // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x07;    //响应优先级2
    // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;      //使能外部中断通道
    // NVIC_Init(&NVIC_InitStructure);

}

/* 初始化系统参数 */
// void sys_parameterInit(sys_data* pdec)
// {
//     pdec->header   =  0x55;
//     pdec->language = SYS_LANG_TYPE_CN;
//     pdec->dev_mode = SYS_DEV_SLAVE;
//     pdec->bools.wl_state = DISABLE;
//     pdec->bools.dmx_state = DISABLE;  //dmx是否有数据接收
//     pdec->bools.flag_reset = DISABLE;
//     pdec->dmx_addr = 1;
//     pdec->rf_addr = 1;
//     pdec->profile = 1;
//     pdec->dim = 100;
//     pdec->cct = 3200;
//     pdec->gel = 3200;
//     pdec->wk_mode = WK_MODE_CCT;
//     pdec->eff_mode = EFF_MODE_STROBE;
// }



void DMX_dataDirectionSet(DMXDataDir_t dir)
{
    if(dir == DMX_SEND_OUT)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_1);
    }
    else
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_1);
    }
}

void SYS_INSLedSet(sysINSLed_t sta)
{
    if(sta == INS_LED_MASTER)
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_15);
    }
    else if(sta == INS_LED_SLAVE)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_15);
    }
    else if(sta == INS_LED_RF_OPEN)
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_0);
    }
    else if(sta == INS_LED_RF_CLOSE)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_0);
    }
    else
    {
    }
}




