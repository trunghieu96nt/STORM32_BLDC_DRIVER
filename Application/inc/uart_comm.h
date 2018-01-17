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
typedef enum{
  MSG_NONE                = 0x00,
  MSG_HOME                = 0x01,
  MSG_STOP                = 0x02,
  MSG_EMERGENCY_STOP      = 0x03,
  MSG_STABILIZING_MODE    = 0x04,
  MSG_GET_MODE            = 0x05,
  MSG_SET_POS             = 0x06,
  MSG_SET_VEL             = 0x07,
  MSG_SET_POS_VEL         = 0x08,
  MSG_GET_POS             = 0x09,
  MSG_SET_KP              = 0x0A,
  MSG_SET_KI              = 0x0B,
  MSG_SET_KD              = 0x0C,
  MSG_SET_KFF1            = 0x0D,
  MSG_SET_KFF2            = 0x0E,
  MSG_GET_PARAMS          = 0x0F,
  MSG_SET_ACTIVE_AXIS     = 0x10,
  MSG_GET_ACTIVE_AXIS     = 0x11,
  MSG_SEND_IMAGE_DATA     = 0x12,
  MSG_SET_BLDC_SPEED      = 0x13,
} ENUM_MSG_ID_T;

typedef bool (*BLDC_HANDLER_FUNC)(uint8_t, uint8_t *, uint32_t);

typedef struct{
  ENUM_MSG_ID_T enum_msg_id;
  uint32_t u32_data_num_bytes;
  BLDC_HANDLER_FUNC bool_msg_handler;
} STRU_BLDC_HANDLER_T;

/* Exported constants --------------------------------------------------------*/
/** @defgroup BLDC UART (Receiver)
  * @{
  */
#define DLDC_RX_FRAME_TIMEOUT      5000 //x10us
#define BLDC_NUM_MSG_ID_MAX        20

#define BLDC_TXBUFF_SIZE           64
#define BLDC_RXBUFF_SIZE           512
#define BLDC_FRAME_LEN_MAX         64

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
void v_BLDC_Receive(void);

#ifdef __cplusplus
}
#endif

#endif /* __UART_COMM_H */

/*********************************END OF FILE**********************************/
