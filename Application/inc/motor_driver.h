/**
  ******************************************************************************
  * @file    motor_driver.h
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    16-January-2018
  * @brief   This file contains all the functions prototypes for the motor_driver
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PWM_DRIVER_H
#define __PWM_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  MOTOR_INVALID = -1,
  MOTOR_0 = 0,
  MOTOR_1,
  MOTOR_2,
} ENUM_MOTOR_T;
/* Exported constants --------------------------------------------------------*/
/** @defgroup Motor Timer Define
  * @{
  */
#define MINIMUM_TS                5000
#define MOTOR_TIMER_IRQ_Handler   TIM5_IRQHandler
#define FREQ_MOTOR_TIMER          1100
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* Initialization and Configuration functions *********************************/
void v_Motor_Init(void);
void v_PWM_Set(ENUM_MOTOR_T enum_motor,uint16_t u16_duty_A, uint32_t u16_duty_B, uint32_t u16_duty_C);



#ifdef __cplusplus
}
#endif

#endif /* __PWM_DRIVER_H */

/*********************************END OF FILE**********************************/
