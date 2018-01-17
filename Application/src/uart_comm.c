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
#include "stm32f10x_it.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t au8_BLDC_rx[BLDC_RXBUFF_SIZE]= {0};
uint8_t au8_BLDC_tx[BLDC_TXBUFF_SIZE]= {0};

extern bool bool_Set_BLDC_Speed_Handler  (uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt);

const STRU_BLDC_HANDLER_T astru_BLDC_handler[BLDC_NUM_MSG_ID_MAX] =
{
  {MSG_NONE,                0,    0}, //bool_None_Handler},
  {MSG_HOME,                1,    0}, //bool_Home_Handler},
  {MSG_STOP,                1,    0}, //bool_Stop_Handler},
  {MSG_EMERGENCY_STOP,      1,    0}, //bool_Emergency_Stop_Handler},
  {MSG_STABILIZING_MODE,    2,    0}, //bool_Stabilizing_Mode_Handler},
  {MSG_GET_MODE,            1,    0}, //bool_Get_Mode_Handler},
  {MSG_SET_POS,             5,    0}, //bool_Set_Pos_Handler},
  {MSG_SET_VEL,             5,    0}, //bool_Set_Vel_Handler},
  {MSG_SET_POS_VEL,         9,    0}, //bool_Set_Pos_Vel_Handler},
  {MSG_GET_POS,             1,    0}, //bool_Get_Pos_Handler},
  {MSG_SET_KP,              6,    0}, //bool_Set_Kp_Handler},
  {MSG_SET_KI,              6,    0}, //bool_Set_Ki_Handler},
  {MSG_SET_KD,              6,    0}, //bool_Set_Kd_Handler},
  {MSG_SET_KFF1,            6,    0}, //bool_Set_Kff1_Handler},
  {MSG_SET_KFF2,            6,    0}, //bool_Set_Kff2_Handler},
  {MSG_GET_PARAMS,          2,    0}, //bool_Get_Params_Handler},
  {MSG_SET_ACTIVE_AXIS,     2,    0}, //bool_Set_Active_Axis_Handler},
  {MSG_GET_ACTIVE_AXIS,     1,    0}, //bool_Get_Active_Axis_Handler},
  {MSG_SEND_IMAGE_DATA,     5,    0}, //bool_Send_Image_Data_Handler},
  {MSG_SET_BLDC_SPEED,      5,    bool_Set_BLDC_Speed_Handler}
};

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
  * @brief  Receive and Parse message through DMA - UART
  * @note   CMD, Call in while loop (main.c) if used
  * @param  None
  * @retval None
  */
void v_BLDC_Receive(void)
{
  static uint32_t u32_idx_pre = 0, u32_idx = 0, u32_time_tick = 0;
  static bool bool_header_detected = false, bool_length_detected = false;
  static uint8_t au8_BLDC_frame[BLDC_FRAME_LEN_MAX] = {'G', 'B', 0x03, 0x02};
  uint32_t u32_length, u32_idx_cur, u32_cnt;
  uint16_t u16_crc_check;
  
  u32_idx_cur = BLDC_RXBUFF_SIZE - BLDC_RX_DMA_CHANNEL->CNDTR;
  
  if (u32_idx_cur == u32_idx_pre)
  {
    if (bool_header_detected == true)
    {
      if (SysTick_IsTimeout(u32_time_tick, DLDC_RX_FRAME_TIMEOUT))
      {
        u32_idx = 0;
        bool_length_detected = false;
        bool_header_detected = false;
      }
    }
    return;
  }
  
  u32_time_tick = SysTick_GetTick();
  
  /* Search Header "GB" */
  if (bool_header_detected == false)
  {
    while (true)
    {
      if (u32_idx_cur >= u32_idx_pre) u32_length = u32_idx_cur - u32_idx_pre;
      else u32_length = BLDC_RXBUFF_SIZE - (u32_idx_pre - u32_idx_cur);
      
      if (u32_length < 2) return;
      
      if (*(au8_BLDC_rx + u32_idx_pre) == 'G')
      {
        if (++u32_idx_pre >= BLDC_RXBUFF_SIZE) u32_idx_pre = 0;
        if (*(au8_BLDC_rx + u32_idx_pre) == 'B')
        {
          if (++u32_idx_pre >= BLDC_RXBUFF_SIZE) u32_idx_pre = 0;
          bool_header_detected = true;
          break;
        }
      }
      else
      {
        if (++u32_idx_pre >= BLDC_RXBUFF_SIZE) u32_idx_pre = 0;
      }
    }
  }
  
  /* Search Length of Frame*/
  if (bool_length_detected == false)
  {
    if (u32_idx_cur >= u32_idx_pre) u32_length = u32_idx_cur - u32_idx_pre;
    else u32_length = BLDC_RXBUFF_SIZE - (u32_idx_pre - u32_idx_cur);
    
    if (u32_length < 5) return;
    
    /* Check DEST_ID - ID_BLDC_DRIVER */
    if (au8_BLDC_rx[u32_idx_pre] != 0x03) 
    {
      bool_header_detected = false;
      return;
    }
    if (++u32_idx_pre == BLDC_RXBUFF_SIZE) u32_idx_pre = 0;
    
    /* Check SRC_ID - ID_GIMBAL_CONTROLER */
    if (au8_BLDC_rx[u32_idx_pre] != 0x02) 
    {
      bool_header_detected = false;
      return;
    }
    if (++u32_idx_pre == BLDC_RXBUFF_SIZE) u32_idx_pre = 0;
    
    /* Get Seq */
    au8_BLDC_frame[4] = au8_BLDC_rx[u32_idx_pre];
    if (++u32_idx_pre == BLDC_RXBUFF_SIZE) u32_idx_pre = 0;
    
    /* Get Length */
    bool_length_detected = true;
    au8_BLDC_frame[5] = au8_BLDC_rx[u32_idx_pre];
    if (++u32_idx_pre == BLDC_RXBUFF_SIZE) u32_idx_pre = 0;
  }
  
  /* Getting Frame */
  while (true)
  {
    au8_BLDC_frame[u32_idx + 6] = *(au8_BLDC_rx + u32_idx_pre);
    if (++u32_idx_pre >= BLDC_RXBUFF_SIZE) u32_idx_pre = 0;
    if (++u32_idx == au8_BLDC_frame[5])
    {
      u32_idx = 0;
      bool_length_detected = false;
      bool_header_detected = false;
      break;
    }
    if (u32_idx_pre == u32_idx_cur) return;
  }
  
  /* Check CRC */
  u16_crc_check = 0;
  u32_length = au8_BLDC_frame[5] + 6 - 2; //Total Length except 2 byte CRC
  for (u32_cnt = 0; u32_cnt < u32_length; u32_cnt++)
  {
    u16_crc_check += au8_BLDC_frame[u32_cnt];
  }
  u16_crc_check = ~u16_crc_check;
  if (((u16_crc_check >> 8) & 0x0FF) != au8_BLDC_frame[u32_length]) return;
  if ((u16_crc_check & 0x0FF) != au8_BLDC_frame[u32_length + 1]) return;
  
  /* Check MsgID */
  if (au8_BLDC_frame[6] != MSG_SET_BLDC_SPEED) return;
  
  /* Check enough length */
  u32_length = au8_BLDC_frame[5] - 3; //Length Payload
  if (astru_BLDC_handler[au8_BLDC_frame[6]].u32_data_num_bytes > u32_length) return;
  
  /* Handle Data */
  astru_BLDC_handler[au8_BLDC_frame[6]].bool_msg_handler(astru_BLDC_handler[au8_BLDC_frame[6]].enum_msg_id, 
                                                       &au8_BLDC_frame[7], u32_length);
}

/**
  * @}
  */

/*********************************END OF FILE**********************************/
