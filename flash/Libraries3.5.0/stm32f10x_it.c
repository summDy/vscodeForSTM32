/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "BSP.h"


//unsigned int i =0;
extern uint8_t T1msFlag;

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}
 __asm void wait()
{
      BX lr
}
/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
    wait();
//  while (1)
//  {
//
//  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */

  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
//	OSIntEnter(); 
//    OSTimeTick(); 
//    OSIntExit(); 
}

void USART1_IRQHandler(void)
{ 
  	char RX_dat;                                                        //�����ַ�����
  	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)               //�жϷ��������ж�
    {
	  USART_ClearITPendingBit(USART1,  USART_IT_RXNE);                 //����жϱ�־
      //GPIO_WriteBit(GPIOB, GPIO_Pin_10, (BitAction)0x01);              //��ʼ����
      RX_dat=USART_ReceiveData(USART1) & 0x7F;                         //�������ݣ������ȥǰ��λ
      USART_SendData(USART1, RX_dat);                                  //��������
      while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){}    //�ȴ����ͽ���
  	}

}

void USART3_IRQHandler(void)
{

//  char RX_dat;                                                         //�����ַ�����
//  OSIntEnter(); 
//  if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)               //�жϷ��������ж�
//    {
//	  USART_ClearITPendingBit(USART3,  USART_IT_RXNE);                 //����жϱ�־
//      //GPIO_WriteBit(GPIOB, GPIO_Pin_10, (BitAction)0x01);              //��ʼ����
//      RX_dat=USART_ReceiveData(USART3) & 0x7F;                         //�������ݣ������ȥǰ��λ
//      USART_SendData(USART3, RX_dat);                                  //��������
//      while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET){}    //�ȴ����ͽ���
//  }
//  OSIntExit();
}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ
	
			CH0_Pro();
			CH1_Pro();
			CH2_Pro();
			CH3_Pro();
	}
}

void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ 

		T1msFlag = 1 ;		//1msʱ��Ƭʹ��
//		PWM_DelayPro();
//		PWM_control();
	}
}

void EXTI4_IRQHandler (void)
{ 
   if(EXTI_GetITStatus(EXTI_Line4) != RESET)
    {		
		if(PLUSEENCODER_A_GET==1)
		{
			if(PLUSEENCODER_B_GET)	pluseEncoder_West_signal =1;
			else pluseEncoder_East_signal =1;			
		}
		else 
		{
			if(PLUSEENCODER_B_GET)	pluseEncoder_East_signal =1;
			else pluseEncoder_West_signal =1;			
		}
		
     	EXTI_ClearFlag(EXTI_Line4);          //����жϱ�־�����룩
     	EXTI_ClearITPendingBit(EXTI_Line4);
     }
}
void EXTI15_10_IRQHandler(void)
{    
    if(EXTI_GetITStatus(EXTI_IMR_MR12) != RESET)
    {
		while(1);
 	}
}











/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
