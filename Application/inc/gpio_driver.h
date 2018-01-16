/**
  ******************************************************************************
  * @file    gpio_driver.h
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    15-January-2018
  * @brief   This file contains all the functions prototypes for the gpio_driver
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_DRIVER_H
#define __GPIO_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup LED Peripheral
  * @{
  */
#define LED_PERIPH_GPIO                   RCC_APB2Periph_GPIOB
#define LED_GPIO                          GPIOB
#define LED0_PIN                          GPIO_Pin_12
#define LED1_PIN                          GPIO_Pin_13

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup LED Peripheral
  * @{
  */
#define v_Green_On()                      v_LED_Set(LED0_PIN)
#define v_Green_Off()                     v_LED_Reset(LED0_PIN)
#define v_Green_Toggle()                  v_LED_Toggle(LED0_PIN)

#define v_Red_On()                        v_LED_Set(LED1_PIN)
#define v_Red_Off()                       v_LED_Reset(LED1_PIN)
#define v_Red_Toggle()                    v_LED_Toggle(LED1_PIN)
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/* GPIO Initial ***************************************************************/
void v_GPIO_Init(void);

/* LED functions **************************************************************/
void v_LED_Set(uint16_t LEDx_Pin);
void v_LED_Reset(uint16_t LEDx_Pin);
void v_LED_Toggle(uint16_t LEDx_Pin);

#ifdef __cplusplus
}
#endif

#endif /* __GPIO_DRIVER_H */

/*********************************END OF FILE**********************************/
