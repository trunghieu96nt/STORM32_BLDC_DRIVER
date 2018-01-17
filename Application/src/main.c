/**
  ******************************************************************************
  * @file    main.c 
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    13-January-2018
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "include.h"

/** @addtogroup GIMBAL_BLDC_NCKH
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SVPWM_TABLE_SIZE 720
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern volatile uint32_t tick_count;
extern volatile uint32_t u32_system_tick_count;

static const uint16_t u16_svpwm_table[SVPWM_TABLE_SIZE] = 
{121,117,113,109,105,102,98,95,91,88,84,81,78,75,72,68,65,63,60,57,54,52,49,46,44,42,39,37,35,33,31,29,27,25,23,21,20,18,17,15,14,12,11,10,9,8,7,6,5,4,3,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,3,4,5,6,7,8,9,10,11,12,14,15,17,18,20,21,23,25,27,29,31,33,35,37,39,42,44,46,49,52,54,57,60,63,65,68,72,75,78,81,84,88,91,95,98,102,105,109,113,117,121,132,144,156,168,180,192,204,217,229,241,253,266,278,291,303,316,328,341,354,367,379,392,405,418,431,444,457,470,483,496,509,523,536,549,562,576,589,602,616,629,642,656,669,683,696,710,723,737,750,764,777,791,804,818,832,845,859,872,886,899,913,927,940,954,967,981,995,1008,1022,1035,1049,1062,1076,1089,1103,1116,1130,1143,1157,1170,1183,1197,1210,1223,1237,1250,1263,1276,1290,1303,1316,1329,1342,1355,1368,1381,1394,1407,1420,1432,1445,1458,1471,1483,1496,1508,1521,1533,1546,1558,1570,1582,1595,1607,1619,1631,1643,1655,1667,1678,1682,1686,1690,1694,1697,1701,1704,1708,1711,1715,1718,1721,1724,1727,1731,1734,1736,1739,1742,1745,1747,1750,1753,1755,1757,1760,1762,1764,1766,1768,1770,1772,1774,1776,1778,1779,1781,1782,1784,1785,1787,1788,1789,1790,1791,1792,1793,1794,1795,1796,1796,1797,1797,1798,1798,1798,1799,1799,1799,1799,1799,1799,1799,1798,1798,1798,1797,1797,1796,1796,1795,1794,1793,1792,1791,1790,1789,1788,1787,1785,1784,1782,1781,1779,1778,1776,1774,1772,1770,1768,1766,1764,1762,1760,1757,1755,1753,1750,1747,1745,1742,1739,1736,1734,1731,1727,1724,1721,1718,1715,1711,1708,1704,1701,1697,1694,1690,1686,1682,1678,1682,1686,1690,1694,1697,1701,1704,1708,1711,1715,1718,1721,1724,1727,1731,1734,1736,1739,1742,1745,1747,1750,1753,1755,1757,1760,1762,1764,1766,1768,1770,1772,1774,1776,1778,1779,1781,1782,1784,1785,1787,1788,1789,1790,1791,1792,1793,1794,1795,1796,1796,1797,1797,1798,1798,1798,1799,1799,1799,1799,1799,1799,1799,1798,1798,1798,1797,1797,1796,1796,1795,1794,1793,1792,1791,1790,1789,1788,1787,1785,1784,1782,1781,1779,1778,1776,1774,1772,1770,1768,1766,1764,1762,1760,1757,1755,1753,1750,1747,1745,1742,1739,1736,1734,1731,1727,1724,1721,1718,1715,1711,1708,1704,1701,1697,1694,1690,1686,1682,1678,1667,1655,1643,1631,1619,1607,1595,1582,1570,1558,1546,1533,1521,1508,1496,1483,1471,1458,1445,1432,1420,1407,1394,1381,1368,1355,1342,1329,1316,1303,1290,1276,1263,1250,1237,1223,1210,1197,1183,1170,1157,1143,1130,1116,1103,1089,1076,1062,1049,1035,1022,1008,995,981,967,954,940,927,913,899,886,872,859,845,832,818,804,791,777,764,750,737,723,710,696,683,669,656,642,629,616,602,589,576,562,549,536,523,509,496,483,470,457,444,431,418,405,392,379,367,354,341,328,316,303,291,278,266,253,241,229,217,204,192,180,168,156,144,132,121,117,113,109,105,102,98,95,91,88,84,81,78,75,72,68,65,63,60,57,54,52,49,46,44,42,39,37,35,33,31,29,27,25,23,21,20,18,17,15,14,12,11,10,9,8,7,6,5,4,3,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,3,4,5,6,7,8,9,10,11,12,14,15,17,18,20,21,23,25,27,29,31,33,35,37,39,42,44,46,49,52,54,57,60,63,65,68,72,75,78,81,84,88,91,95,98,102,105,109,113,117};
static uint32_t u32_svpwm_cnt = 0;
static uint32_t u32_svpwm_idx_A, u32_svpwm_idx_B, u32_svpwm_idx_C;
uint16_t u16_svpwm_A, u16_svpwm_B, u16_svpwm_C;
float flt_motor_hz = 0.1;
uint32_t u32_ts_us = 0;

/* Private function prototypes -----------------------------------------------*/
void v_Board_Init(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  v_Board_Init();
  
  /* Infinite loop */
  while (1)
  {
//    if (u32_system_tick_count >= 100000)
//    {
//      u32_system_tick_count = 0;
//      v_Red_Toggle();
//      v_Green_Toggle();
//    }
    
    /* Test 1Hz */
    //if (u32_svpwm_cnt < 1000)
    {
      u32_ts_us = ((100000.0f/((float)SVPWM_TABLE_SIZE*flt_motor_hz))/6.0f);
      if (u32_system_tick_count >= u32_ts_us) // unit: 10us
      {
        u32_system_tick_count = 0;
        u32_svpwm_idx_A = u32_svpwm_cnt;
        u32_svpwm_idx_B = (u32_svpwm_cnt + 240)%SVPWM_TABLE_SIZE; // SVPWM_TABLE_SIZE/3
        u32_svpwm_idx_C = (u32_svpwm_cnt + 480)%SVPWM_TABLE_SIZE; // 2*SVPWM_TABLE_SIZE/3
        u16_svpwm_A = u16_svpwm_table[u32_svpwm_idx_A];
        u16_svpwm_B = u16_svpwm_table[u32_svpwm_idx_B];
        u16_svpwm_C = u16_svpwm_table[u32_svpwm_idx_C];
        v_PWM_Set(MOTOR_0, u16_svpwm_table[u32_svpwm_idx_A], u16_svpwm_table[u32_svpwm_idx_B], u16_svpwm_table[u32_svpwm_idx_C]);
        if (u32_svpwm_cnt == 0)
        {
          u32_svpwm_cnt = SVPWM_TABLE_SIZE - 1;
          v_Red_Toggle();
        }
        else 
          u32_svpwm_cnt--;
//        if (u32_svpwm_cnt >= SVPWM_TABLE_SIZE)
//        {
//          u32_svpwm_cnt = 0;
//          v_Red_Toggle();
//        }
      }
    }
//    else
//    {
//      v_Red_On();
//    }
  }
}

/**
  * @brief  Init board
  * @param  None
  * @retval None
  */
void v_Board_Init(void)
{
  /* Check System Clock*/
  RCC_ClocksTypeDef RCC_ClocksStructure;
  uint8_t u8_clock_source;
  u8_clock_source = RCC_GetSYSCLKSource();
  if (u8_clock_source != 0x08) // 0x08: PLL used as system clock
  {
    //while (true);
  }
  RCC_GetClocksFreq(&RCC_ClocksStructure);
  if (RCC_ClocksStructure.SYSCLK_Frequency != 72000000)
  {
    //while (true);
  }
  
  /* Enable SysTick at 1ms interrupt */
  SysTick_Config(SystemCoreClock / F_CTRL);
  
  /* Driver Initialization */
  v_GPIO_Init();
  v_Green_On();
  
  v_Motor_Init();
  //v_PWM_Set(MOTOR_0, 0, 900, 1799);
  
  v_UART_Comm_Init();
}

/**
  * @brief  Motor Timer - 0.2ms.
  * @param  None
  * @retval None
  */
void MOTOR_TIMER_IRQ_Handler(void)
{
  // Not use
}
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
    
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
