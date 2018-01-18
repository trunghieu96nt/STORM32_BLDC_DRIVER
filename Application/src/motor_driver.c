/**
  ******************************************************************************
  * @file    motor_driver.c
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    16-January-2018
  * @brief   This file contains functions for control pwm
  *
 @verbatim
 ===============================================================================
                      ##### How to use this driver #####
 ===============================================================================
 @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "motor_driver.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void v_PWM_Init(void);
//static void v_Motor_Timer_Init(void);

/* Private functions ---------------------------------------------------------*/

/** @defgroup Motor Initialization
 *  @brief   ...
 *
 @verbatim
 ===============================================================================
                        ##### Motor Initialization #####
 ===============================================================================  

 @endverbatim
  * @{
  */
/**
  * @brief  Motor Driver Init Function
  * @note   Call function first if using this driver
  * @param  none
  * @retval none
  */
void v_Motor_Init(void)
{
  v_PWM_Init();
  //v_Motor_Timer_Init();
}
/**
  * @}
  */

/** @defgroup PWM Driver
 *  @brief   ...
 *
 @verbatim
 ===============================================================================
                            ##### PWM Driver #####
 ===============================================================================  

 @endverbatim
  * @{
  */

/**
  * @brief  PWM Driver Init Function
  * @note   ...
  * @param  none
  * @retval none
  */
void v_PWM_Init(void)
{
  GPIO_InitTypeDef            GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
  TIM_OCInitTypeDef          TIM_OCInitStructure;
  
  /* TIM clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
  
  /* GPIO clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);
  
  
  /* Pulse pin configuration */
  /* M0(PB1->TIM3_CH4, PB0->TIM3_CH3) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* M0(PA7->TIM3_CH2) && M1(PA6->TIM3_CH1 , PA3->TIM2_CH4 , PA2->TIM2_CH3) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_3 | GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  
  /* Config TIM3 (CH4,CH3,CH2) for M0 */
  TIM_TimeBaseStructure.TIM_Prescaler = 0; //36Hz
  TIM_TimeBaseStructure.TIM_Period = 1800 - 1;   //20KHz
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
 
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_Pulse = 0;
  
 
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
  
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
 
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
 
  TIM_ARRPreloadConfig(TIM3, ENABLE);
 
 /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
  TIM_CtrlPWMOutputs(TIM3, ENABLE);
  
  /* Config TIM3 (CH1) && TIM2 (CH4,CH3) for M1 */
  
  TIM_TimeBaseStructure.TIM_Prescaler = 0; //36MHz
  TIM_TimeBaseStructure.TIM_Period = 1800 - 1;   //20KHz
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_Pulse = 0;
  
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
  
  TIM_OC3Init(TIM2, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
 
  TIM_OC4Init(TIM2, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
  
  /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);
  TIM_CtrlPWMOutputs(TIM2, ENABLE);
}

void v_PWM_Set(ENUM_MOTOR_T enum_motor,uint16_t u16_duty_A, uint32_t u16_duty_B, uint32_t u16_duty_C)
{
  switch (enum_motor)
  {
    case MOTOR_0: 
          TIM3->CCR4 = u16_duty_A ;  
          TIM3->CCR3 = u16_duty_B; 
          TIM3->CCR2 = u16_duty_C;
          break;
    case MOTOR_1:
          TIM3->CCR1 = u16_duty_A ;  
          TIM2->CCR4 = u16_duty_B; 
          TIM2->CCR3 = u16_duty_C;
          break;
    default:
      break;
  }
  
}
/**
  * @}
  */

/** @defgroup Motor Timer
 *  @brief   ...
 *
 @verbatim
 ===============================================================================
                               ##### Motor Timer #####
 ===============================================================================  

 @endverbatim
  * @{
  */
void v_Motor_Timer_Init(void)
{
  TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
  NVIC_InitTypeDef          NVIC_InitStructure;
  
  /* Enable Clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
   
 /* Time base configuration */
 TIM_TimeBaseStructure.TIM_Prescaler = 9 - 1; //4MHz
 TIM_TimeBaseStructure.TIM_Period = 800 - 1; //5KHz - 0.2ms
 TIM_TimeBaseStructure.TIM_ClockDivision = 0;
 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
 TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
 TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
 TIM_Cmd(TIM4, ENABLE);
 
 NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 NVIC_Init(&NVIC_InitStructure);   
}
/**
  * @}
  */

/*********************************END OF FILE**********************************/
