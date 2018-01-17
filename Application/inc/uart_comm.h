/**
  ******************************************************************************
  * @file    uart_comm.h
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    17-January-2018
  * @brief   This file contains all the functions prototypes for the uart_comm
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_COMM_H
#define __UART_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdbool.h"
#include "stdint.h"
  
/* Exported types ------------------------------------------------------------*/
//typedef here

/* Exported constants --------------------------------------------------------*/
/** @defgroup BLDC UART (Receiver)
  * @{
  */
#define BLDC_TXBUFF_SIZE           64
#define BLDC_RXBUFF_SIZE           512
//#define BLDC_FRAME_LEN_MAX         64

#define BLDC_UART                  UART4
#define BLDC_UART_CLK              RCC_APB1Periph_UART4
#define BLDC_PORT                  GPIOC
#define BLDC_PORT_CLK              RCC_APB2Periph_GPIOC
#define BLDC_TX                    GPIO_Pin_10
#define BLDC_RX                    GPIO_Pin_11
#define BLDC_BAUDRATE              (uint32_t)115200 //921600 //115200

#define BLDC_AHB_PERIPH_DMA        RCC_AHBPeriph_DMA2
#define BLDC_DATA_REG              (uint32_t)BLDC_UART + 0x04
#define BLDC_TX_DMA_CHANNEL        DMA2_Channel5
#define BLDC_TX_DMA_FLAG           DMA2_FLAG_TC5
#define BLDC_RX_DMA_CHANNEL        DMA2_Channel3
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/* Initialization and Configuration functions *********************************/
void v_UART_Comm_Init(void);

/* BLDC function **************************************************************/
//bool bool_BLDC_Send(const uint8_t *pu8_message, uint32_t u32_message_size);

#ifdef __cplusplus
}
#endif

#endif /* __UART_COMM_H */

/*********************************END OF FILE**********************************/
