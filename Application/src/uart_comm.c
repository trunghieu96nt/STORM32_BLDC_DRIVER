/**
  ******************************************************************************
  * @file    uart_comm.c
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    17-January-2018
  * @brief   This file contains functions for uart communication
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
#include "uart_comm.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t au8_BLDC_rx[BLDC_RXBUFF_SIZE]= {0};
uint8_t au8_BLDC_tx[BLDC_TXBUFF_SIZE]= {0};

/* Private function prototypes -----------------------------------------------*/
static void v_BLDC_UART_Init(void);

/* Private functions ---------------------------------------------------------*/

/** @defgroup Initialization and Configuration
 *  @brief   ...
 *
 @verbatim
 ===============================================================================
                 ##### Initialization and Configuration #####
 ===============================================================================  

 @endverbatim
  * @{
  */

/**
  * @brief  Communication Initialize
  * @note   Including UART (CMD, DATA, RESV), SPI
  * @param  none
  * @retval none
  */
void v_UART_Comm_Init(void)
{
  v_BLDC_UART_Init();
}

/**
  * @}
  */
  
/** @defgroup BCLD - UART
 *  @brief   Receive CMD from Master Board
 *
 @verbatim
 ===============================================================================
                            ##### BCLD - UART #####
 ===============================================================================  

 @endverbatim
  * @{
  */

/**
  * @brief  CMD Initialize (Receiver)
  * @note   ...
  * @param  none
  * @retval none
  */
static void v_BLDC_UART_Init(void)
{
  USART_InitTypeDef     USART_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  
  /* GPIO Init */
  RCC_APB2PeriphClockCmd(BLDC_PORT_CLK, ENABLE);
  /* RX */
  GPIO_InitStructure.GPIO_Pin   = BLDC_RX;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(BLDC_PORT, &GPIO_InitStructure);
  /* TX */
//  GPIO_InitStructure.GPIO_Pin   = BLDC_TX;
//  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(BLDC_PORT, &GPIO_InitStructure);

  /* UART Init */
  RCC_APB1PeriphClockCmd(BLDC_UART_CLK, ENABLE);
  USART_InitStructure.USART_BaudRate            = BLDC_BAUDRATE;
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx; // | USART_Mode_Tx;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(BLDC_UART, &USART_InitStructure);
  USART_Cmd(BLDC_UART, ENABLE); 
  USART_ClearFlag(BLDC_UART, USART_FLAG_TC);

  /* DMA RX configuration */
  RCC_AHBPeriphClockCmd(BLDC_AHB_PERIPH_DMA, ENABLE);
  DMA_DeInit(BLDC_RX_DMA_CHANNEL);
  DMA_InitStructure.DMA_PeripheralBaseAddr = BLDC_DATA_REG;
  DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)&au8_BLDC_rx[0];
  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize         = BLDC_RXBUFF_SIZE;
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular; // avoid buffer overload error
  DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
  DMA_Init(BLDC_RX_DMA_CHANNEL, &DMA_InitStructure);
  
  /* Enable  request */
  USART_DMACmd(BLDC_UART, USART_DMAReq_Rx, ENABLE);  
  /* Enable DMA RX Channel */
  DMA_Cmd(BLDC_RX_DMA_CHANNEL, ENABLE);
  
  /* DMA TX configuration */
//  DMA_DeInit(BLDC_TX_DMA_CHANNEL);
//  DMA_InitStructure.DMA_PeripheralBaseAddr = BLDC_DATA_REG;
//  DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)&au8_BLDC_tx[0];
//  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralDST;
//  DMA_InitStructure.DMA_BufferSize         = BLDC_TXBUFF_SIZE;
//  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
//  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
//  DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
//  DMA_InitStructure.DMA_Priority           = DMA_Priority_Low;
//  DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
//  DMA_Init(BLDC_TX_DMA_CHANNEL, &DMA_InitStructure);
//  
//  // Enable DMA Stream Transfer Complete interrupt
//  //DMA_ITConfig(BLDC_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
//  
//  // Enable USART DMA TX request
//  USART_DMACmd(BLDC_UART, USART_DMAReq_Tx, ENABLE);
}

/**
  * @brief  Send message through DMA - UART
  * @note   BLDC
  * @param  pu8_message: pointer message to send (should be static)
  * @param  u32_message_size: number of char to send
  * @retval true if success and vice versa
  */
bool bool_BLDC_Send(const uint8_t *pu8_message, uint32_t u32_message_size)
{
  uint32_t u32_idx;
  
  if (u32_message_size > BLDC_TXBUFF_SIZE)
  {
    return false;
  }
  else
  {
    //copy buff
    for (u32_idx = 0; u32_idx < u32_message_size; u32_idx++)
    {
      au8_BLDC_tx[u32_idx] = *(pu8_message + u32_idx);
    }
    
    //clear flag
    DMA_ClearFlag(BLDC_TX_DMA_FLAG);
    //DMA_MemoryTargetConfig(DATA_TX_DMA_STREAM, (uint32_t)au8_DATA_tx, DMA_Memory_0);
    DMA_SetCurrDataCounter(BLDC_TX_DMA_CHANNEL, u32_message_size);
    //DATA_TX_DMA_STREAM->NDTR = BUFF_SIZE;
    DMA_Cmd(BLDC_TX_DMA_CHANNEL, ENABLE);
    return true;
  }
}

/**
  * @}
  */

/*********************************END OF FILE**********************************/
