/**
  ******************************************************************************
  * @file    gpio_driver.c
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    15-January-2018
  * @brief   This file contains functions for control GPIO
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
#include "gpio_driver.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void v_LED_Init(void);

/* Private functions ---------------------------------------------------------*/

/** @defgroup GPIO Driver Init Function
 *  @brief   ...
 *
 @verbatim
 ===============================================================================
                      ##### GPIO Driver Init Function #####
 ===============================================================================  

 @endverbatim
  * @{
  */

/**
  * @brief  GPIO Driver Init Function
  * @note   Call function first if using this driver
  * @param  none
  * @retval none
  */
void v_GPIO_Init(void)
{
  v_LED_Init();
  v_LED_Reset(LED0_PIN);
  v_LED_Reset(LED1_PIN);

}

/**
  * @}
  */

/** @defgroup LED DRIVER
 *  @brief    ...
 *
 @verbatim
 ===============================================================================
                             ##### LED DRIVER #####
 ===============================================================================  

 @endverbatim
  * @{
  */

/**
  * @brief  Init 3 ouputs for 3 led RGB
  * @note   ...
  * @param  none
  * @retval none
  */
static void v_LED_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  /* GPIOD Peripheral clock enable */
  RCC_APB2PeriphClockCmd(LED_PERIPH_GPIO, ENABLE);
  
  /* Configure LED1, LED2, LED3 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin   = LED0_PIN | LED1_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(LED_GPIO, &GPIO_InitStructure);
}

/**
  * @brief  LED Set, Reset, Toggle
  * @note   ...
  * @param  LEDx_Pin: specifies the LED pin to write
  * @retval none
  */
void v_LED_Set(uint16_t LEDx_Pin)
{
  GPIO_SetBits(LED_GPIO, LEDx_Pin);
}

void v_LED_Reset(uint16_t LEDx_Pin)
{
  GPIO_ResetBits(LED_GPIO, LEDx_Pin);
}

void v_LED_Toggle(uint16_t LEDx_Pin)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(LED_GPIO));

  LED_GPIO->ODR ^= LEDx_Pin;
}
/**
  * @}
  */

/*********************************END OF FILE**********************************/
